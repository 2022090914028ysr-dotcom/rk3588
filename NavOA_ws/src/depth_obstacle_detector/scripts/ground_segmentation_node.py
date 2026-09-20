#!/usr/bin/env python3
"""Measured clearing rays and filtered hits, with one sensor frame/stamp.

Ground classification is in base_footprint; temporal matching is in odom.
Invalid/NaN pixels are unknown, never fabricated free-space measurements.
"""
import copy
import threading
import time
from collections import deque

import numpy as np
import rospy
import tf2_ros
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


def read_xyz(msg, stride=1):
    """Read XYZ respecting endian, field offsets and organized row padding.

    The HP60C driver publishes an unorganized cloud (height=1, roughly one
    point per depth pixel).  In that representation a 2-D ``stride`` cannot
    be applied as ``[::stride]``: that would only reduce a 640x480 frame by
    four instead of the intended 4x4.  Use the squared stride for the
    unorganized case, while retaining true row/column sampling for organized
    clouds.
    """
    fields = {f.name: f for f in msg.fields}
    endian = '>' if msg.is_bigendian else '<'
    formats = []
    for name in ('x', 'y', 'z'):
        f = fields[name]
        if f.count != 1 or f.datatype not in (PointField.FLOAT32, PointField.FLOAT64):
            raise ValueError('XYZ fields must be scalar float32/float64')
        formats.append(endian + ('f4' if f.datatype == PointField.FLOAT32 else 'f8'))
    dtype = np.dtype({'names': ['x', 'y', 'z'], 'formats': formats,
                      'offsets': [fields[n].offset for n in ('x', 'y', 'z')],
                      'itemsize': msg.point_step})
    if msg.width == 0 or msg.height == 0:
        return np.empty((0, 3), dtype=np.float32)
    cloud = np.ndarray((msg.height, msg.width), dtype=dtype, buffer=msg.data,
                       strides=(msg.row_step, msg.point_step))
    stride = max(1, int(stride))
    cloud = cloud[::stride, ::stride] if msg.height > 1 else cloud[:, :: (stride * stride)]
    xyz = np.stack([cloud[n].reshape(-1) for n in ('x', 'y', 'z')], axis=1)
    return xyz[np.isfinite(xyz).all(axis=1)].astype(np.float32, copy=False)


def make_cloud(header, points):
    points = np.ascontiguousarray(points, dtype='<f4').reshape(-1, 3)
    return PointCloud2(header=copy.deepcopy(header), height=1, width=len(points),
                       fields=[PointField(n, 4*i, PointField.FLOAT32, 1)
                               for i, n in enumerate(('x', 'y', 'z'))],
                       is_bigendian=False, point_step=12, row_step=12*len(points),
                       data=points.tobytes(), is_dense=True)


def transform_xyz(points, transform):
    t, q = transform.transform.translation, transform.transform.rotation
    quat = np.array([q.x, q.y, q.z, q.w], dtype=np.float64)
    norm = np.linalg.norm(quat)
    if norm < 1e-9:
        raise ValueError('Invalid zero TF quaternion')
    x, y, z, w = quat/norm
    rotation = np.array([[1-2*y*y-2*z*z, 2*x*y-2*z*w, 2*x*z+2*y*w],
                         [2*x*y+2*z*w, 1-2*x*x-2*z*z, 2*y*z-2*x*w],
                         [2*x*z-2*y*w, 2*y*z+2*x*w, 1-2*x*x-2*y*y]])
    return points @ rotation.T + np.array([t.x, t.y, t.z])


def ground_mask(points, height_min=0.05, search_height=0.10,
                distance=0.025, max_tilt_deg=15.0, max_offset=0.04,
                max_fit_points=5000):
    """Constrained robust plane fit: tables/walls cannot become the floor."""
    z = points[:, 2]
    fallback = np.abs(z) <= distance
    candidates = points[(z > -search_height) & (z < search_height)]
    # SVD is only used to estimate the ground plane.  Fitting on every point
    # is needlessly expensive on dense clouds; classify all points against the
    # resulting plane below.  A deterministic subsample preserves repeatable
    # behavior and keeps the callback bounded on RK3588.
    if max_fit_points > 0 and len(candidates) > max_fit_points:
        fit_stride = int(np.ceil(float(len(candidates)) / max_fit_points))
        candidates = candidates[::fit_stride]
    if len(candidates) < 30:
        return fallback
    seed_z = np.percentile(candidates[:, 2], 20)
    candidates = candidates[np.abs(candidates[:, 2]-seed_z) < 2*distance]
    for _ in range(3):
        if len(candidates) < 20:
            return fallback
        centroid = candidates.mean(axis=0)
        try:
            _, singular, vt = np.linalg.svd(candidates-centroid, full_matrices=False)
        except np.linalg.LinAlgError:
            return fallback
        if singular[1] < 0.05:
            return fallback
        normal = vt[-1]
        if normal[2] < 0:
            normal = -normal
        offset = -float(centroid @ normal)
        if normal[2] < np.cos(np.deg2rad(max_tilt_deg)) or abs(offset) > max_offset:
            return fallback
        candidates = candidates[np.abs(candidates @ normal + offset) < distance]
    return np.abs(points @ normal + offset) < distance


def spatial_mask(points, voxel_size, min_points):
    if not len(points):
        return np.zeros(0, dtype=bool)
    cells = np.floor(points[:, :2]/voxel_size).astype(np.int64)
    _, inverse, counts = np.unique(cells, axis=0, return_inverse=True, return_counts=True)
    return counts[inverse] >= min_points


class TemporalFilter:
    def __init__(self, hits, cell_size):
        self.hits, self.cell_size = hits, cell_size
        self.history = deque(maxlen=max(1, hits-1))

    def mask(self, points):
        cells = np.floor(points[:, :2]/self.cell_size).astype(np.int64)
        current = set(map(tuple, cells))
        keep = np.ones(len(points), dtype=bool)
        if self.hits > 1:
            if len(self.history) < self.hits-1:
                keep[:] = False
            else:
                allowed = current.copy()
                for previous in self.history:
                    neighbors = {(x+dx, y+dy) for x, y in previous
                                 for dx in (-1, 0, 1) for dy in (-1, 0, 1)}
                    allowed &= neighbors
                keep = np.array([tuple(cell) in allowed for cell in cells], dtype=bool)
        self.history.append(current)
        return keep


class GroundSegmentationNode:
    def __init__(self):
        self.input_topic = rospy.get_param('~input_pointcloud_topic', '/ascamera_hp60c/depth0/points')
        self.ground_frame = rospy.get_param('~target_frame', 'base_footprint')
        self.fixed_frame = rospy.get_param('~temporal_frame', 'odom')
        self.height_min = float(rospy.get_param('~height_min', 0.05))
        self.height_max = float(rospy.get_param('~height_max', 1.9))
        self.max_range = float(rospy.get_param('~max_range', 2.5))
        self.clearing_range = float(rospy.get_param('~clearing_range', 5.0))
        self.min_range = float(rospy.get_param('~min_range', 0.15))
        self.stride = int(rospy.get_param('~pixel_stride', 4))
        self.voxel_size = float(rospy.get_param('~voxel_size', 0.05))
        self.min_points = int(rospy.get_param('~min_points_per_voxel', 2))
        self.rate = float(rospy.get_param('~max_rate', 10.0))
        self.tf_timeout = float(rospy.get_param('~transform_timeout', 0.3))
        self.max_age = float(rospy.get_param('~max_cloud_age', 1.5))
        self.publish_clearing = bool(rospy.get_param('~publish_clearing_points', False))
        self.clearing_stride = max(1, int(rospy.get_param('~clearing_pixel_stride', 8)))
        self.stamp_mode = rospy.get_param('~stamp_mode', 'sensor')
        self.ground_distance = float(rospy.get_param('~distance_threshold', 0.025))
        self.plane_fit_max_points = int(rospy.get_param('~plane_fit_max_points', 5000))
        self.ground_search = float(rospy.get_param('~ground_search_height', 0.10))
        # One frame is sufficient after voxel density filtering; set 2+ only
        # when a stable camera stream has excessive transient false positives.
        self.temporal = TemporalFilter(int(rospy.get_param('~temporal_min_hits', 1)),
                                       float(rospy.get_param('~temporal_cell_size', 0.10)))
        if not (self.rate > 0 and self.stride >= 1 and self.voxel_size > 0
                and self.min_points >= 1 and self.temporal.hits >= 1
                and self.temporal.cell_size > 0 and self.height_max > self.height_min
                and self.clearing_range >= self.max_range > self.min_range):
            raise ValueError('Invalid depth obstacle filter parameters')
        if self.stamp_mode not in ('sensor', 'receive'):
            raise ValueError('stamp_mode must be sensor or receive')
        self.last_stamp = None
        self.last_success = time.monotonic()
        self.latest_lock = threading.Lock()
        self.latest_cloud = None
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.pub = rospy.Publisher('/obstacle_points', PointCloud2, queue_size=1)
        self.clear_pub = (rospy.Publisher('/depth/clearing_points', PointCloud2, queue_size=1)
                          if self.publish_clearing else None)
        # Keep ROS transport callbacks cheap. Dense clouds arrive faster than
        # Python can segment them; a worker always consumes the newest frame so
        # old clouds cannot build a seconds-long queue while the robot moves.
        self.sub = rospy.Subscriber(self.input_topic, PointCloud2, self.input_callback,
                                    queue_size=1, buff_size=16*1024*1024)
        self.worker = rospy.Timer(rospy.Duration(1.0/self.rate), self.process_latest)
        self.watchdog = rospy.Timer(rospy.Duration(2.0), self.check_input)
        rospy.loginfo('Depth fusion: ground=%s temporal=%s output=sensor frame; stamp=%s',
                      self.ground_frame, self.fixed_frame, self.stamp_mode)

    def check_input(self, _event):
        if time.monotonic()-self.last_success > 2.0:
            rospy.logwarn_throttle(5.0, 'No valid depth frame for >2s: check camera, timestamps and TF')

    def input_callback(self, msg):
        with self.latest_lock:
            self.latest_cloud = msg

    def process_latest(self, _event):
        with self.latest_lock:
            msg = self.latest_cloud
            self.latest_cloud = None
        if msg is None:
            return
        arrival = rospy.Time.now()
        stamp = msg.header.stamp
        if self.stamp_mode == 'receive':
            # Explicit fallback for unsynchronized hardware; choose once before TF.
            stamp = arrival
            rospy.logwarn_throttle(30.0, 'Using receive timestamps; fix camera clock for motion accuracy')
        age = (arrival-stamp).to_sec()
        if not msg.header.frame_id or stamp == rospy.Time(0) or age > self.max_age or age < -0.1:
            self.temporal.history.clear()
            rospy.logwarn_throttle(5.0, 'Dropped depth frame: invalid frame/stamp, age=%.3fs', age)
            return
        if self.last_stamp is not None:
            dt = (stamp-self.last_stamp).to_sec()
            if dt <= 0 or dt > self.max_age:
                self.temporal.history.clear()
            if dt <= 0:
                self.last_stamp = stamp
                return
        self.last_stamp = stamp
        started = time.monotonic()
        try:
            xyz = read_xyz(msg, self.stride)
            ranges = np.einsum('ij,ij->i', xyz, xyz)
            xyz = xyz[(ranges >= self.min_range**2) & (ranges <= self.clearing_range**2)]
            if len(xyz) == 0:
                self.temporal.history.clear()
                rospy.logwarn_throttle(5.0, 'Depth contains no valid measured endpoints; not declaring free space')
                return
            ground_tf = self.tf_buffer.lookup_transform(self.ground_frame, msg.header.frame_id,
                                                        stamp, rospy.Duration(self.tf_timeout))
            # A one-hit filter has no inter-frame comparison, so a second TF lookup
            # is needless and can create boundary warnings at the newest timestamp.
            fixed_tf = ground_tf
            if self.temporal.hits > 1:
                try:
                    fixed_tf = self.tf_buffer.lookup_transform(
                        self.fixed_frame, msg.header.frame_id, stamp,
                        rospy.Duration(self.tf_timeout))
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                        tf2_ros.ExtrapolationException):
                    # Startup can precede localization. Matching in base coordinates
                    # is safe for the stationary/short-gap case.
                    rospy.logwarn_throttle(
                        10.0, 'Temporal TF %s unavailable; matching in %s',
                        self.fixed_frame, self.ground_frame)
            base = transform_xyz(xyz, ground_tf)
            is_ground = ground_mask(
                base, self.height_min, self.ground_search, self.ground_distance,
                max_fit_points=self.plane_fit_max_points)
            distance_sq = np.einsum('ij,ij->i', xyz, xyz)
            hits = ((~is_ground) & (base[:, 2] >= self.height_min)
                    & (base[:, 2] <= self.height_max) & (distance_sq <= self.max_range**2))
            indices = np.flatnonzero(hits)
            n_candidates = len(indices)
            indices = indices[spatial_mask(base[indices], self.voxel_size, self.min_points)]
            n_voxel = len(indices)
            if len(indices):
                _, selected = np.unique(np.floor(base[indices]/self.voxel_size).astype(np.int64),
                                        axis=0, return_index=True)
                indices = indices[selected]
            # Match voxel representatives instead of tens of thousands of raw pixels.
            fixed = transform_xyz(xyz[indices], fixed_tf)
            indices = indices[self.temporal.mask(fixed)]
            header = Header(stamp=stamp, frame_id=msg.header.frame_id)
            # Keep all measured endpoints including floor and background for 3-D rays.
            # Empty clouds replace observations; they do NOT erase stored voxels.
            if self.clear_pub is not None:
                # Visibility only needs sparse measured endpoints.  The
                # fusion layer angularly subsamples them again before raytrace.
                self.clear_pub.publish(make_cloud(header, xyz[::self.clearing_stride]))
            self.pub.publish(make_cloud(header, xyz[indices]))
            self.last_success = time.monotonic()
            rospy.loginfo_throttle(5.0, 'Depth valid=%d ground=%d candidates=%d voxel=%d marked=%d age=%.3fs process=%.1fms',
                                   len(xyz), int(is_ground.sum()), n_candidates, n_voxel, len(indices), age,
                                   1000*(time.monotonic()-started))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as exc:
            self.temporal.history.clear()
            rospy.logwarn_throttle(5.0, 'Dropped depth frame: TF at acquisition time unavailable: %s', exc)
        except (ValueError, KeyError, TypeError) as exc:
            self.temporal.history.clear()
            rospy.logwarn_throttle(5.0, 'Dropped malformed depth cloud: %s', exc)


if __name__ == '__main__':
    rospy.init_node('ground_segmentation')
    GroundSegmentationNode()
    rospy.spin()
