"""Timestamp-correct point fusion with finite-FoV depth obstacle memory.

LiDAR and depth observations are transformed at their acquisition times into
the continuous ``odom`` frame. Depth hits are retained in a bounded 2-D grid
so an obstacle does not disappear merely because the camera turned away.
Only measured depth rays can clear that memory; absence from the camera image
is never treated as free space.
"""
import threading

import numpy as np
import rospy
import sensor_msgs.point_cloud2 as pc2
import tf
from sensor_msgs.msg import LaserScan, PointCloud2
from std_msgs.msg import Header


def _transform_xyz(points, trans, quat):
    points = np.asarray(points, dtype=np.float32).reshape(-1, 3)
    rotation = tf.transformations.quaternion_matrix(quat)[:3, :3]
    return points @ rotation.T + np.asarray(trans, dtype=np.float32)


def _fresh_observation(observation, now, timeout):
    if observation is None:
        return None, False, float('inf')
    stamp, points = observation[:2]
    age = (now - stamp).to_sec()
    fresh = -0.1 <= age <= timeout
    return (points if fresh else None), fresh, age


def _deduplicate_nearest(points, resolution, origin):
    """Keep the point nearest the robot in each 2-D hash-grid cell."""
    if points is None or points.shape[1] == 0:
        return points
    xy = points.T
    cells = np.floor(xy / resolution).astype(np.int64)
    distance_sq = np.einsum('ij,ij->i', xy - origin, xy - origin)
    by_distance = np.argsort(distance_sq, kind='stable')
    _, first = np.unique(cells[by_distance], axis=0, return_index=True)
    return xy[by_distance[first]].T


def _limit_nearest(points, origin, max_points):
    """Keep the closest hazards when the planner input has a point cap.

    DUNE otherwise decimates by point index.  That can discard a small
    low obstacle among dense wall/background points, while nearest-point
    selection is conservative for collision avoidance.
    """
    if points is None or max_points <= 0 or points.shape[1] <= max_points:
        return points
    xy = points.T
    distance_sq = np.einsum('ij,ij->i', xy - origin, xy - origin)
    keep = np.argsort(distance_sq, kind='stable')[:int(max_points)]
    return xy[keep].T


def _limit_directional(points, origin, yaw, max_points,
                       front_share=0.5, side_share=0.2, rear_share=0.1,
                       front_half_angle=np.deg2rad(60.0),
                       side_max_angle=np.deg2rad(135.0)):
    """Directional quota: spend the point budget where it matters.

    Pure nearest-point selection lets a wall hugging the robot's flank
    or rear consume the whole DUNE budget and starves the mid-range
    forward hazards the planner must react to.  Points are bucketed by
    bearing around the robot; each bucket keeps its nearest members up
    to its quota, leftover quota flows to the front bucket.

    points: (2, N) in the fusion frame; origin: (2,) robot position in
    the same frame; yaw: robot heading in the same frame.
    """
    if points is None or max_points <= 0 or points.shape[1] <= max_points:
        return points

    rel = points.T - np.asarray(origin, dtype=np.float32).reshape(1, 2)
    c, s = np.cos(yaw), np.sin(yaw)
    bx = c * rel[:, 0] + s * rel[:, 1]
    by = -s * rel[:, 0] + c * rel[:, 1]
    bearing = np.abs(np.arctan2(by, bx))  # 0 = dead ahead, pi = behind

    front_q = max(1, int(round(max_points * front_share)))
    side_q = max(0, int(round(max_points * side_share)))
    rear_q = max(0, int(round(max_points * rear_share)))

    dist = np.hypot(bx, by)
    buckets = [
        (bearing < front_half_angle, front_q),
        ((bearing >= front_half_angle) & (bearing <= side_max_angle)
         & (by > 0), side_q),
        ((bearing >= front_half_angle) & (bearing <= side_max_angle)
         & (by < 0), side_q),
        (bearing > side_max_angle, rear_q),
    ]

    keep = []
    for mask, quota in buckets:
        idx = np.flatnonzero(mask)
        if idx.size and quota:
            idx = idx[np.argsort(dist[idx], kind='stable')[:quota]]
            keep.extend(idx.tolist())

    if len(keep) > max_points:
        keep = keep[:max_points]
    elif len(keep) < max_points:
        # Unused quota (mostly empty side/rear buckets) flows to the
        # nearest remaining forward points first, then anywhere.
        chosen = set(keep)
        order = np.argsort(dist, kind='stable')
        front_rest = [i for i in order
                      if i not in chosen and bearing[i] < front_half_angle]
        other_rest = [i for i in order
                      if i not in chosen and bearing[i] >= front_half_angle]
        keep.extend((front_rest + other_rest)[:max_points - len(keep)])

    if not keep:
        return _limit_nearest(points, origin, max_points)
    return points[:, np.array(keep, dtype=np.int64)]


class DepthObstacleMemory:
    """Bounded odom-frame memory for depth-only obstacles."""
    def __init__(self, resolution=0.05, ttl=4.0, radius=3.0,
                 clear_angle_resolution=0.02):
        if resolution <= 0 or ttl <= 0 or radius <= 0 or clear_angle_resolution <= 0:
            raise ValueError('Depth memory parameters must be positive')
        self.resolution = float(resolution)
        self.ttl = float(ttl)
        self.radius = float(radius)
        self.clear_angle_resolution = float(clear_angle_resolution)
        # key -> (representative_xy, last_seen_ros_time, hit_count)
        self.cells = {}

    def add(self, points, stamp):
        if points is None or points.shape[1] == 0:
            return
        xy = np.asarray(points, dtype=np.float32).T
        cells = np.floor(xy / self.resolution).astype(np.int64)
        unique, inverse = np.unique(cells, axis=0, return_inverse=True)
        for index, cell in enumerate(unique):
            candidates = xy[inverse == index]
            representative = candidates[np.argmin(
                np.einsum('ij,ij->i', candidates, candidates))]
            key = (int(cell[0]), int(cell[1]))
            old = self.cells.get(key)
            hits = 1 if old is None else old[2] + 1
            self.cells[key] = (representative.copy(), stamp, hits)

    def clear_rays(self, origin, endpoints):
        """Clear cells traversed by measured camera rays, excluding endpoints."""
        if not self.cells or endpoints is None or endpoints.shape[1] == 0:
            return
        origin = np.asarray(origin, dtype=np.float32).reshape(2)
        xy = np.asarray(endpoints, dtype=np.float32).T
        delta = xy - origin
        ranges = np.linalg.norm(delta, axis=1)
        valid = np.isfinite(ranges) & (ranges > self.resolution)
        if not np.any(valid):
            return
        xy, delta, ranges = xy[valid], delta[valid], ranges[valid]

        # One farthest endpoint per angular bin bounds work for dense clouds.
        angles = np.arctan2(delta[:, 1], delta[:, 0])
        bins = np.floor(angles / self.clear_angle_resolution).astype(np.int64)
        selected = []
        for bin_id in np.unique(bins):
            candidates = np.flatnonzero(bins == bin_id)
            selected.append(candidates[np.argmax(ranges[candidates])])
        xy, delta, ranges = xy[selected], delta[selected], ranges[selected]

        to_remove = set()
        for vector, distance in zip(delta, ranges):
            steps = max(1, int(np.ceil(distance / self.resolution)))
            fractions = np.arange(steps, dtype=np.float32) / float(steps)
            ray = origin[None, :] + fractions[:, None] * vector[None, :]
            ray_cells = np.floor(ray / self.resolution).astype(np.int64)
            to_remove.update((int(c[0]), int(c[1])) for c in ray_cells)
        for key in to_remove:
            self.cells.pop(key, None)

    def points(self, now, origin):
        origin = np.asarray(origin, dtype=np.float32).reshape(2)
        output, expired = [], []
        for key, (point, stamp, _hits) in self.cells.items():
            age = (now - stamp).to_sec()
            distance = float(np.linalg.norm(point - origin))
            if age > self.ttl or age < -1.0 or distance > self.radius:
                expired.append(key)
            else:
                output.append(point)
        for key in expired:
            self.cells.pop(key, None)
        return np.asarray(output, dtype=np.float32).T if output else None


class ObstacleFusion:
    def __init__(self, listener, planner_frame, base_frame='base_link',
                 fusion_frame='odom', lidar_topic='/scan',
                 depth_topic='/obstacle_points', use_depth=True,
                 lidar_timeout=0.25, depth_timeout=0.80,
                 grid_resolution=0.05, scan_range=(0.1, 5.0),
                 scan_angle_range=(-3.14, 3.14), scan_downsample=1,
                 flip_angle=False, output_topic='/navoa/fused_obstacle_points',
                 depth_memory_enabled=True, depth_memory_ttl=4.0,
                 depth_memory_radius=3.0, depth_clear_topic='/depth/clearing_points',
                 depth_memory_clear_enabled=True,
                 depth_memory_clear_angle_resolution=0.02,
                 max_obstacle_points=100):
        if lidar_timeout <= 0 or depth_timeout <= 0 or grid_resolution <= 0:
            raise ValueError('Fusion timeouts and grid resolution must be positive')
        self.listener = listener
        self.planner_frame = planner_frame
        self.base_frame = base_frame
        self.fusion_frame = fusion_frame
        self.lidar_timeout = float(lidar_timeout)
        self.depth_timeout = float(depth_timeout)
        self.grid_resolution = float(grid_resolution)
        self.max_obstacle_points = int(max_obstacle_points)
        self.scan_range = tuple(map(float, scan_range))
        self.scan_angle_range = tuple(map(float, scan_angle_range))
        self.scan_downsample = max(1, int(scan_downsample))
        self.flip_angle = bool(flip_angle)
        self.use_depth = bool(use_depth)
        self.depth_memory_enabled = bool(depth_memory_enabled and self.use_depth)
        self.depth_memory_clear_enabled = bool(
            depth_memory_clear_enabled and self.depth_memory_enabled)
        self.lock = threading.Lock()
        self.lidar_observation = None
        self.depth_observation = None
        self.clear_observation = None
        self.memory = (DepthObstacleMemory(
            resolution=grid_resolution, ttl=depth_memory_ttl,
            radius=depth_memory_radius,
            clear_angle_resolution=depth_memory_clear_angle_resolution)
                       if self.depth_memory_enabled else None)
        self.memory_depth_stamp = rospy.Time(0)
        self.memory_clear_stamp = rospy.Time(0)
        self.last_publish = rospy.Time(0)
        self.pub = rospy.Publisher(output_topic, PointCloud2, queue_size=1)
        self.lidar_sub = rospy.Subscriber(
            lidar_topic, LaserScan, self.lidar_callback, queue_size=1)
        self.depth_sub = None
        self.clear_sub = None
        if self.use_depth:
            self.depth_sub = rospy.Subscriber(
                depth_topic, PointCloud2, self.depth_callback, queue_size=1,
                buff_size=4 * 1024 * 1024)
        if self.depth_memory_clear_enabled:
            self.clear_sub = rospy.Subscriber(
                depth_clear_topic, PointCloud2, self.clear_callback, queue_size=1,
                buff_size=4 * 1024 * 1024)

    def _lookup(self, target, source, stamp):
        self.listener.waitForTransform(target, source, stamp, rospy.Duration(0.2))
        return self.listener.lookupTransform(target, source, stamp)

    @staticmethod
    def _read_cloud(msg):
        xyz = np.asarray(list(pc2.read_points(
            msg, field_names=('x', 'y', 'z'), skip_nans=True)),
            dtype=np.float32).reshape(-1, 3)
        return xyz[np.isfinite(xyz).all(axis=1)]

    def lidar_callback(self, msg):
        stamp = msg.header.stamp
        age = (rospy.Time.now() - stamp).to_sec()
        if not msg.header.frame_id or stamp == rospy.Time(0) or not -0.1 <= age <= self.lidar_timeout:
            rospy.logwarn_throttle(5.0, '[NavOA] Dropped stale/invalid lidar scan (age=%.3fs)', age)
            return
        ranges = np.asarray(msg.ranges, dtype=np.float32)
        angles = msg.angle_min + np.arange(len(ranges), dtype=np.float32) * msg.angle_increment
        if self.flip_angle:
            angles = angles[::-1]
        indices = np.arange(len(ranges))
        sensor_min = msg.range_min if msg.range_min > 0 else -float('inf')
        sensor_max = msg.range_max if msg.range_max > 0 else float('inf')
        with np.errstate(invalid='ignore'):
            valid = (np.isfinite(ranges)
                     & (ranges >= max(self.scan_range[0], sensor_min))
                     & (ranges <= min(self.scan_range[1], sensor_max))
                     & (angles > self.scan_angle_range[0])
                     & (angles < self.scan_angle_range[1])
                     & (indices % self.scan_downsample == 0))
        local = np.column_stack((ranges[valid] * np.cos(angles[valid]),
                                 ranges[valid] * np.sin(angles[valid]),
                                 np.zeros(np.count_nonzero(valid), dtype=np.float32)))
        try:
            trans, quat = self._lookup(self.fusion_frame, msg.header.frame_id, stamp)
            points = _transform_xyz(local, trans, quat)[:, :2].T
            with self.lock:
                self.lidar_observation = (stamp, points)
        except (tf.Exception, ValueError) as exc:
            rospy.logwarn_throttle(5.0, '[NavOA] Lidar transform at acquisition time failed: %s', exc)

    def depth_callback(self, msg):
        stamp = msg.header.stamp
        age = (rospy.Time.now() - stamp).to_sec()
        if not msg.header.frame_id or stamp == rospy.Time(0) or not -0.1 <= age <= self.depth_timeout:
            rospy.logwarn_throttle(5.0, '[NavOA] Dropped stale/invalid depth obstacles (age=%.3fs)', age)
            return
        try:
            xyz = self._read_cloud(msg)
            trans, quat = self._lookup(self.fusion_frame, msg.header.frame_id, stamp)
            points = _transform_xyz(xyz, trans, quat)[:, :2].T
            with self.lock:
                self.depth_observation = (stamp, points)
        except (tf.Exception, ValueError) as exc:
            rospy.logwarn_throttle(5.0, '[NavOA] Depth transform at acquisition time failed: %s', exc)

    def clear_callback(self, msg):
        stamp = msg.header.stamp
        age = (rospy.Time.now() - stamp).to_sec()
        if not msg.header.frame_id or stamp == rospy.Time(0) or not -0.1 <= age <= self.depth_timeout:
            rospy.logwarn_throttle(5.0, '[NavOA] Dropped stale/invalid depth clearing rays (age=%.3fs)', age)
            return
        try:
            xyz = self._read_cloud(msg)
            trans, quat = self._lookup(self.fusion_frame, msg.header.frame_id, stamp)
            points = _transform_xyz(xyz, trans, quat)[:, :2].T
            with self.lock:
                self.clear_observation = (
                    stamp, points, np.asarray(trans[:2], dtype=np.float32))
        except (tf.Exception, ValueError) as exc:
            rospy.logwarn_throttle(5.0, '[NavOA] Depth clearing transform at acquisition time failed: %s', exc)

    def _apply_memory_events(self, depth_observation, clear_observation):
        if self.memory is None:
            return
        events = []
        if depth_observation is not None and depth_observation[0] != self.memory_depth_stamp:
            events.append((depth_observation[0], 1, depth_observation[1]))
        if clear_observation is not None and clear_observation[0] != self.memory_clear_stamp:
            events.append((clear_observation[0], 0, clear_observation[1:]))
        # Clearing precedes marking on equal stamps so a current obstacle is
        # not erased by its own measured ray.
        for stamp, kind, payload in sorted(events, key=lambda event: (event[0], event[1])):
            if kind == 0:
                endpoints, origin = payload
                self.memory.clear_rays(origin, endpoints)
                self.memory_clear_stamp = stamp
            else:
                self.memory.add(payload, stamp)
                self.memory_depth_stamp = stamp

    def snapshot(self, now):
        with self.lock:
            lidar_observation = self.lidar_observation
            depth_observation = self.depth_observation
            clear_observation = self.clear_observation
        self._apply_memory_events(depth_observation, clear_observation)
        lidar, lidar_fresh, lidar_age = _fresh_observation(
            lidar_observation, now, self.lidar_timeout)
        depth, depth_fresh, depth_age = _fresh_observation(
            depth_observation, now, self.depth_timeout)
        try:
            robot_trans, robot_rot = self.listener.lookupTransform(
                self.fusion_frame, self.base_frame, rospy.Time(0))
            robot_origin = np.asarray(robot_trans[:2], dtype=np.float32)
        except (tf.Exception, ValueError) as exc:
            rospy.logwarn_throttle(5.0, '[NavOA] Robot/fusion transform failed: %s', exc)
            return None, {'lidar_fresh': lidar_fresh,
                          'depth_fresh': depth_fresh if self.use_depth else True,
                          'lidar_age': lidar_age, 'depth_age': depth_age,
                          'memory_count': 0, 'transform_valid': False}

        def _quat_yaw(quat):
            qx, qy, qz, qw = quat
            return np.arctan2(2.0 * (qw * qz + qx * qy),
                              1.0 - 2.0 * (qy * qy + qz * qz))

        memory_points = (self.memory.points(now, robot_origin)
                         if self.memory is not None else None)
        parts = [p for p in (lidar, depth if self.use_depth else None, memory_points)
                 if p is not None and p.shape[1] > 0]
        fused = np.hstack(parts) if parts else None
        transform_valid = True
        if fused is not None:
            try:
                fused = _deduplicate_nearest(fused, self.grid_resolution, robot_origin)
                fused = _limit_directional(
                    fused, robot_origin, _quat_yaw(robot_rot),
                    getattr(self, 'max_obstacle_points', 0))
                if self.planner_frame != self.fusion_frame:
                    trans, quat = self.listener.lookupTransform(
                        self.planner_frame, self.fusion_frame, rospy.Time(0))
                    xyz = np.column_stack((fused.T, np.zeros(fused.shape[1], dtype=np.float32)))
                    fused = _transform_xyz(xyz, trans, quat)[:, :2].T
            except (tf.Exception, ValueError) as exc:
                rospy.logwarn_throttle(5.0, '[NavOA] Fused obstacle transform failed: %s', exc)
                fused = None
                transform_valid = False
        status = {'lidar_fresh': lidar_fresh,
                  'depth_fresh': depth_fresh if self.use_depth else True,
                  'lidar_age': lidar_age, 'depth_age': depth_age,
                  'memory_count': 0 if memory_points is None else memory_points.shape[1],
                  'transform_valid': transform_valid}
        self._publish_debug(fused, now)
        return fused, status

    def _publish_debug(self, points, stamp):
        if (stamp - self.last_publish).to_sec() < 0.1:
            return
        self.last_publish = stamp
        xyz = [] if points is None else np.column_stack(
            (points.T, np.zeros(points.shape[1], dtype=np.float32)))
        self.pub.publish(pc2.create_cloud_xyz32(
            Header(stamp=stamp, frame_id=self.planner_frame), xyz))
