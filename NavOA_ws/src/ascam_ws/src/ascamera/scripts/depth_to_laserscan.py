#!/usr/bin/env python
"""Depth Image to 2D LaserScan (numpy-only, no cv_bridge)."""

import rospy
import math
from sensor_msgs.msg import Image, LaserScan
import numpy as np


def decode_depth(msg):
    """Decode depth image without cv_bridge."""
    if msg.encoding in ('16UC1', 'mono16'):
        dtype = np.uint16
    elif msg.encoding == '32FC1':
        dtype = np.float32
    else:
        rospy.logwarn("Unknown encoding: %s, trying 16UC1", msg.encoding)
        dtype = np.uint16
    data = np.frombuffer(msg.data, dtype=dtype)
    return data.reshape((msg.height, msg.width))


class DepthToLaserScan:
    def __init__(self):
        rospy.init_node('depth_to_laserscan', anonymous=True)

        self.scan_rows    = rospy.get_param('~scan_rows', 0)       # 0=disabled
        self.row_start    = rospy.get_param('~scan_row_start', 400)
        self.row_end      = rospy.get_param('~scan_row_end', 480)
        self.range_min    = rospy.get_param('~range_min',   0.1)
        self.range_max    = rospy.get_param('~range_max',   10.0)
        self.scan_time    = rospy.get_param('~scan_time',   1.0/30.0)
        self.output_frame = rospy.get_param('~output_frame', 'laser_frame')
        self.fy = rospy.get_param('~fy', 210.43)            # focal length y (px)
        self.cy = rospy.get_param('~cy', 120.35)            # principal point y (px)
        self.camera_h = rospy.get_param('~camera_height', 0.3)  # height above ground (m)
        self.ground_tol = rospy.get_param('~ground_tolerance', 0.05)  # tolerance (m)
        self.range_offset = rospy.get_param('~range_offset', 0.0)   # shift all ranges (m)
        self.angle_offset = rospy.get_param('~angle_offset', 0.0) # horizontal shift (rad)

        depth_topic = rospy.get_param('~depth_topic', '/camera/depth/image_raw')
        self.sub = rospy.Subscriber(depth_topic, Image, self.callback)
        self.pub = rospy.Publisher('/camscan', LaserScan, queue_size=10)
        self.pub_debug = rospy.Publisher('/camscan/debug_image', Image, queue_size=5)

        self.latest_ranges = None
        self.latest_header = None
        self.latest_w = None
        self.pub_rate = rospy.get_param('~pub_rate', 50.0)  # publish Hz
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.pub_rate), self._timer_pub)

        self.width = None
        self.fov_rad = None
        self._frame_cnt = 0
        rospy.loginfo("DepthToLaserScan started, listening on %s", depth_topic)

    def callback(self, msg):
        self._frame_cnt += 1

        try:
            depth_img = decode_depth(msg)
        except Exception as e:
            rospy.logerr("decode error: %s", e)
            return

        h, w = depth_img.shape[:2]

        if self.width is None:
            self.width = w
            self.fov_rad = rospy.get_param('~fov_horizontal', math.radians(87.0))
            rospy.loginfo("Depth: %dx%d enc=%s FOV=%.1f deg range=[%.2f, %.2f]",
                          w, h, msg.encoding, math.degrees(self.fov_rad),
                          self.range_min, self.range_max)

        # take rows: explicit range or bottom N
        if self.scan_rows > 0:
            n_rows = int(min(self.scan_rows, h))
            r_start = h - n_rows
            r_end = h
        else:
            r_start = max(0, int(min(self.row_start, h - 1)))
            r_end = max(r_start + 1, int(min(self.row_end, h)))
            n_rows = r_end - r_start

        bottom_rows = depth_img[r_start:r_end, ::-1].astype(np.float32)

        # 16UC1 values are mm -> m
        if msg.encoding in ('16UC1', 'mono16'):
            bottom_rows /= 1000.0

        # ground filtering: mask pixels at/below ground level
        # Y = (v - cy) * Z / fy  (vertical distance from camera center)
        vv = np.arange(r_start, r_end, dtype=np.float32).reshape(-1, 1)
        Y_world = (vv - self.cy) * bottom_rows / self.fy  # height below camera (m)
        ground_mask = (bottom_rows > 0) & (Y_world >= (self.camera_h - self.ground_tol))
        bottom_rows[ground_mask] = 0.0

        # per-column min ignoring zeros
        masked = np.ma.masked_equal(bottom_rows, 0)
        ranges_raw = np.ma.min(masked, axis=0).filled(0)

        # debug every 30 frames
        if self._frame_cnt % 30 == 0:
            nz = ranges_raw[ranges_raw > 0.01]
            if len(nz) > 0:
                rospy.loginfo("frame=%d rows=%d enc=%s min=%.2fm max=%.2fm nz=%d",
                              self._frame_cnt, n_rows, msg.encoding,
                              float(np.min(nz)), float(np.max(nz)), len(nz))
            else:
                rospy.logwarn("frame=%d rows=%d ALL ZERO", self._frame_cnt, n_rows)

        # filter
        ranges_raw = np.where(np.isfinite(ranges_raw), ranges_raw, 0.0)
        ranges_raw = np.clip(ranges_raw, 0, self.range_max)
        ranges_raw[ranges_raw < self.range_min] = 0.0

        # apply range offset (positive = push obstacles farther)
        if self.range_offset != 0:
            valid = ranges_raw > 0
            ranges_raw[valid] = np.clip(ranges_raw[valid] + self.range_offset, self.range_min, self.range_max)

        has_data = np.any(ranges_raw > 0)

        # if not has_data:
        #     ranges_raw[:] = 0.13  # treat too-close as min-distance obstacle

        # apply horizontal angle offset (roll ranges)
        if self.angle_offset != 0:
            shift = int(round(self.angle_offset / (self.fov_rad / float(w))))
            ranges_raw = np.roll(ranges_raw, shift)

        # store latest data for timer-based publishing
        self.latest_ranges = ranges_raw
        self.latest_header = msg.header
        self.latest_w = w

        # debug image: draw scan row lines
        self._publish_debug(depth_img, r_start, r_end, msg)

    def _timer_pub(self, event):
        """Publish at fixed rate using latest processed data."""
        if self.latest_ranges is None:
            return
        scan = LaserScan()
        scan.header.frame_id = self.output_frame
        scan.header.stamp = rospy.Time.now()
        scan.angle_min = -self.fov_rad / 2.0 + self.angle_offset
        scan.angle_max =  self.fov_rad / 2.0 + self.angle_offset
        scan.angle_increment = self.fov_rad / float(self.latest_w)
        scan.time_increment = 0.0
        scan.scan_time = self.scan_time
        scan.range_min = self.range_min
        scan.range_max = self.range_max
        scan.ranges = self.latest_ranges.tolist()
        self.pub.publish(scan)


    def _publish_debug(self, depth_img, r_start, r_end, ref_msg):
        """Publish debug image with scan row lines overlaid."""
        # normalize depth to 0-255 grayscale
        d = depth_img.astype(np.float32)
        valid = d[d > 0]
        vmax = float(np.percentile(valid, 95)) if len(valid) > 0 else 65535
        vmax = max(vmax, 1)
        gray = np.clip(d / vmax * 255, 0, 255).astype(np.uint8)

        # draw white scan lines
        gray[r_start, :] = 255
        gray[r_end - 1, :] = 255

        # build Image msg (MONO8, no cv_bridge)
        img_msg = Image()
        img_msg.header = ref_msg.header
        img_msg.height = gray.shape[0]
        img_msg.width = gray.shape[1]
        img_msg.encoding = 'mono8'
        img_msg.step = gray.shape[1]
        img_msg.data = gray.tobytes()
        self.pub_debug.publish(img_msg)


if __name__ == '__main__':
    try:
        DepthToLaserScan()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
        pass
