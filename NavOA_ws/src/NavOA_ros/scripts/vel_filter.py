#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import copy
import math

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class VelocityFilter:
    def __init__(self):
        rospy.init_node('cmd_vel_modifier_node', anonymous=True)

        self.scan_topic = rospy.get_param('~scan_topic', '/scan')
        self.cmd_in_topic = rospy.get_param('~cmd_in_topic', '/cmd_vel_tem')
        self.cmd_out_topic = rospy.get_param('~cmd_out_topic', '/cmd_vel')

        self.cmd_timeout = float(rospy.get_param('~cmd_timeout', 0.5))
        self.publish_rate = float(rospy.get_param('~publish_rate', 20.0))

        self.deadband_linear_x = float(rospy.get_param('~deadband_linear_x', 0.0))
        self.deadband_angular_z = float(rospy.get_param('~deadband_angular_z', 0.0))

        self.allow_reverse = bool(rospy.get_param('~allow_reverse', True))
        self.reverse_requires_obstacle = bool(rospy.get_param('~reverse_requires_obstacle', True))
        self.reverse_obstacle_distance = float(rospy.get_param('~reverse_obstacle_distance', 0.35))
        self.max_reverse_speed = float(rospy.get_param('~max_reverse_speed', 0.25))

        self.enable_forward_safety = bool(rospy.get_param('~enable_forward_safety', True))
        self.forward_stop_distance = float(rospy.get_param('~forward_stop_distance', 0.30))
        self.forward_slow_distance = float(rospy.get_param('~forward_slow_distance', 0.60))
        self.min_forward_speed = float(rospy.get_param('~min_forward_speed', 0.05))
        self.front_half_angle = float(rospy.get_param('~front_half_angle', 0.52))

        # Output clamps (safety / easy tuning)
        # Use <=0 to disable.
        self.max_forward_speed = float(rospy.get_param('~max_forward_speed', 0.0))
        self.max_angular_speed = float(rospy.get_param('~max_angular_speed', 0.0))

        # If enabled: when an obstacle is closer than the stop distance and an incoming
        # command requests forward motion, override to a small reverse speed.
        # This is useful when the upstream planner keeps commanding forward.
        self.enable_auto_reverse = bool(rospy.get_param('~enable_auto_reverse', False))
        self.auto_reverse_speed = float(rospy.get_param('~auto_reverse_speed', -0.1))

        self.stop_turn_when_forward_blocked = bool(
            rospy.get_param('~stop_turn_when_forward_blocked', True)
        )

        # Ignore scan ranges below this distance when estimating obstacle distance.
        # Keep 0.0 by default for safety.
        self.min_scan_range = float(rospy.get_param('~min_scan_range', 0.0))

        self.front_min_range = float('inf')
        self.obstacle_close = False

        self._last_cmd_time = rospy.Time(0)
        self._latest_output = Twist()
        self._has_valid_cmd = False
        self._timeout_published = False

        rospy.Subscriber(self.cmd_in_topic, Twist, self.callback)
        rospy.Subscriber(self.scan_topic, LaserScan, self.scan_callback)
        self.pub = rospy.Publisher(self.cmd_out_topic, Twist, queue_size=10)
        self._timer = rospy.Timer(
            rospy.Duration(1.0 / max(self.publish_rate, 1.0)), self._publish_timer
        )

        rospy.loginfo(
            "vel_filter running: %s -> %s (timeout=%.2fs, rate=%.1fHz)",
            self.cmd_in_topic,
            self.cmd_out_topic,
            self.cmd_timeout,
            self.publish_rate,
        )

    def _apply_deadband(self, twist: Twist) -> Twist:
        if self.deadband_linear_x > 0.0 and abs(twist.linear.x) < self.deadband_linear_x:
            twist.linear.x = 0.0
        if self.deadband_angular_z > 0.0 and abs(twist.angular.z) < self.deadband_angular_z:
            twist.angular.z = 0.0
        return twist

    def _publish_timer(self, _evt):
        now = rospy.Time.now()
        if self._last_cmd_time != rospy.Time(0) and (now - self._last_cmd_time).to_sec() <= self.cmd_timeout:
            out = copy.deepcopy(self._latest_output)
            self.pub.publish(out)
            self._timeout_published = False
        elif self._has_valid_cmd and not self._timeout_published:
            # Publish a single zero command when the last valid command expires,
            # then stop publishing until a new command arrives.
            rospy.loginfo_once("[vel_filter] cmd_vel timeout expired; publishing single zero command.")
            self.pub.publish(Twist())
            self._timeout_published = True
            self._has_valid_cmd = False
        else:
            pass

    def scan_callback(self, msg: LaserScan):
        if not msg.ranges or msg.angle_increment == 0.0:
            self.front_min_range = float('inf')
            self.obstacle_close = False
            return

        min_valid = max(float(getattr(msg, 'range_min', 0.0) or 0.0), self.min_scan_range)
        min_r = float('inf')

        angle = msg.angle_min
        for r in msg.ranges:
            if -self.front_half_angle <= angle <= self.front_half_angle:
                if r is not None and math.isfinite(r) and r >= min_valid:
                    if r < min_r:
                        min_r = r
            angle += msg.angle_increment

        self.front_min_range = min_r
        self.obstacle_close = math.isfinite(min_r) and (min_r < self.reverse_obstacle_distance)

    def callback(self, msg: Twist):
        self._last_cmd_time = rospy.Time.now()

        modified = copy.deepcopy(msg)



        # Forward safety (stop/slow when obstacle is in front)
        if self.enable_forward_safety and modified.linear.x > 0.0:
            original_x = modified.linear.x
            d = self.front_min_range


            if math.isfinite(d) and d < self.forward_stop_distance:
                if self.enable_auto_reverse and self.allow_reverse:
                    modified.linear.x = -abs(float(self.auto_reverse_speed))
                    modified.angular.z = 0.0
                    rospy.logwarn_throttle(
                        1.0,
                        "前方障碍过近(最小距离=%.3fm < stop=%.2fm)，触发自动倒车: 原始前进=%.3f 倒车=%.3f",
                        d,
                        self.forward_stop_distance,
                        original_x,
                        modified.linear.x,
                    )
                else:
                    modified.linear.x = 0.0
                    if self.stop_turn_when_forward_blocked:
                        modified.angular.z = 0.0
                    rospy.logwarn_throttle(
                        1.0,
                        "前方障碍过近(最小距离=%.3fm < stop=%.2fm)，前进指令已钳制为 0: 原始=%.3f",
                        d,
                        self.forward_stop_distance,
                        original_x,
                    )
            elif (
                math.isfinite(d)
                and d < self.forward_slow_distance
                and self.forward_slow_distance > self.forward_stop_distance
            ):
                scale = (d - self.forward_stop_distance) / (
                    self.forward_slow_distance - self.forward_stop_distance
                )
                scale = max(0.0, min(1.0, scale))
                limited = original_x * scale
                limited = max(0.0, min(original_x, limited))
                if self.min_forward_speed > 0.0:
                    limited = max(min(limited, original_x), min(self.min_forward_speed, original_x))
                modified.linear.x = limited
                rospy.loginfo_throttle(
                    1.0,
                    "前方接近障碍(最小距离=%.3fm)，前进限速: 原始=%.3f 修改后=%.3f",
                    d,
                    original_x,
                    modified.linear.x,
                )

        # Reverse gating (never amplify reverse speed)
        if modified.linear.x < 0.0:
            original_x = modified.linear.x

            if not self.allow_reverse:
                modified.linear.x = 0.0
                rospy.loginfo_throttle(2.0, "倒车已禁用，已钳制为 0: 原始值=%.3f", original_x)
            elif self.reverse_requires_obstacle and (not self.obstacle_close):
                modified.linear.x = 0.0
                rospy.loginfo_throttle(
                    2.0,
                    "未检测到前方近障碍物(最小距离=%.3fm)，倒车指令已钳制为 0: 原始值=%.3f",
                    self.front_min_range,
                    original_x,
                )
            else:
                modified.linear.x = max(original_x, -abs(self.max_reverse_speed))
                rospy.loginfo_throttle(
                    2.0,
                    "检测到前方近障碍物(最小距离=%.3fm)，允许倒车并限幅: 原始=%.3f 修改后=%.3f",
                    self.front_min_range,
                    original_x,
                    modified.linear.x,
                )

        # Clamp outputs (after all safety logic)
        if self.max_forward_speed > 0.0 and modified.linear.x > self.max_forward_speed:
            modified.linear.x = self.max_forward_speed
        if self.max_angular_speed > 0.0:
            az = modified.angular.z
            if abs(az) > self.max_angular_speed:
                modified.angular.z = math.copysign(self.max_angular_speed, az)

        modified = self._apply_deadband(modified)
        self._latest_output = modified


if __name__ == '__main__':
    try:
        VelocityFilter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass