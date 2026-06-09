#!/usr/bin/env python
"""Accumulate narrow-FOV camscan into 360 via robot pose in map frame."""
import rospy
import tf2_ros
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped
import numpy as np
import math
from collections import deque


class CamscanAccumulator:
    def __init__(self):
        rospy.init_node('camscan_accumulator')

        self.topic_in = rospy.get_param('~topic_in', '/camscan')
        self.topic_out = rospy.get_param('~topic_out', '/camscan_wide')
        self.target_frame = rospy.get_param('~target_frame', 'base_footprint')
        self.map_frame = rospy.get_param('~map_frame', 'map')
        self.cache_duration = rospy.Duration(rospy.get_param('~cache_sec', 5.0))
        self.pub_rate = rospy.get_param('~pub_rate', 10.0)
        self.resolution = rospy.get_param('~output_resolution', 0.01745)  # 1 deg
        self.max_range = rospy.get_param('~max_range', 10.0)

        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)

        self.sub = rospy.Subscriber(self.topic_in, LaserScan, self.callback)
        self.pub = rospy.Publisher(self.topic_out, LaserScan, queue_size=10)

        self.cache = deque()  # (stamp, x_map, y_map)
        rospy.loginfo("CamscanAccumulator: %s -> %s (cache=%.1fs)",
                      self.topic_in, self.topic_out, self.cache_duration.to_sec())

    def _get_robot_pose(self, stamp):
        """Get robot pose (x, y, yaw) in map frame. Returns None on failure."""
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, self.target_frame, stamp, rospy.Duration(0.2))
        except Exception as e:
            rospy.logwarn_throttle(10, "TF lookup map->%s failed: %s",
                                   self.target_frame, e)
            return None
        x = tf.transform.translation.x
        y = tf.transform.translation.y
        q = tf.transform.rotation
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
        return (x, y, yaw)

    def callback(self, scan):
        """Transform scan points to map frame using robot pose, cache them."""
        pose = self._get_robot_pose(rospy.Time(0))
        if pose is None:
            return
        rx, ry, ryaw = pose

        angles = np.linspace(scan.angle_min, scan.angle_max, len(scan.ranges))
        ranges = np.array(scan.ranges, dtype=np.float32)

        now = rospy.Time.now()
        for r, a in zip(ranges, angles):
            if r <= scan.range_min or r >= scan.range_max or r >= self.max_range:
                continue
            # point in base_footprint frame
            bx = r * math.cos(a)
            by = r * math.sin(a)
            # rotate + translate to map frame
            mx = rx + bx * math.cos(ryaw) - by * math.sin(ryaw)
            my = ry + bx * math.sin(ryaw) + by * math.cos(ryaw)
            self.cache.append((now, mx, my))

    def expire_cache(self):
        now = rospy.Time.now()
        while self.cache and (now - self.cache[0][0]) > self.cache_duration:
            self.cache.popleft()

    def build_and_publish(self):
        self.expire_cache()
        if not self.cache:
            return

        pose = self._get_robot_pose(rospy.Time(0))
        if pose is None:
            return
        rx, ry, ryaw = pose

        n = int(2 * math.pi / self.resolution)
        ranges = np.full(n, 0.0, dtype=np.float32)

        for _, mx, my in list(self.cache):
            dx = mx - rx
            dy = my - ry
            bx = dx * math.cos(-ryaw) - dy * math.sin(-ryaw)
            by = dx * math.sin(-ryaw) + dy * math.cos(-ryaw)
            dist = math.hypot(bx, by)
            if dist <= 0.05:
                continue
            ang = math.atan2(by, bx)
            idx = int((ang + math.pi) / self.resolution) % n
            if ranges[idx] == 0 or dist < ranges[idx]:
                ranges[idx] = dist

        out = LaserScan()
        out.header.stamp = rospy.Time.now()
        out.header.frame_id = self.target_frame
        out.angle_min = -math.pi
        out.angle_max = math.pi
        out.angle_increment = self.resolution
        out.range_min = 0.1
        out.range_max = self.max_range
        out.ranges = ranges.tolist()
        self.pub.publish(out)

    def run(self):
        r = rospy.Rate(self.pub_rate)
        while not rospy.is_shutdown():
            self.build_and_publish()
            r.sleep()


if __name__ == '__main__':
    try:
        CamscanAccumulator().run()
    except rospy.ROSInterruptException:
        pass
