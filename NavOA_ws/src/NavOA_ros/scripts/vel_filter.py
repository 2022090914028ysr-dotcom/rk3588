#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math

class VelocityFilter:
    def __init__(self):
        # 1. 初始化节点
        rospy.init_node('cmd_vel_modifier_node', anonymous=True)

        # Params
        self.scan_topic = rospy.get_param('~scan_topic', '/scan')
        self.allow_reverse = rospy.get_param('~allow_reverse', True)
        # If True: only allow reverse when an obstacle is close in front
        self.reverse_requires_obstacle = rospy.get_param('~reverse_requires_obstacle', True)
        self.reverse_obstacle_distance = float(rospy.get_param('~reverse_obstacle_distance', 0.35))

        # Forward safety (stop before hitting walls)
        self.enable_forward_safety = rospy.get_param('~enable_forward_safety', True)
        # If front obstacle is closer than this, block forward motion
        self.forward_stop_distance = float(rospy.get_param('~forward_stop_distance', 0.30))
        # If closer than this, linearly scale down forward speed (must be > stop_distance)
        self.forward_slow_distance = float(rospy.get_param('~forward_slow_distance', 0.60))
        # Minimum forward speed allowed when slowing (set 0 to allow crawl)
        self.min_forward_speed = float(rospy.get_param('~min_forward_speed', 0.05))
        # Front sector half-angle in radians (e.g. 30deg)
        self.front_half_angle = float(rospy.get_param('~front_half_angle', 0.52))
        # Limit reverse speed magnitude (m/s)
        self.max_reverse_speed = float(rospy.get_param('~max_reverse_speed', 0.25))

        self.front_min_range = float('inf')
        self.obstacle_close = False

        # 2. 设置订阅者：监听 /cmd_vel_tem
        self.sub = rospy.Subscriber("/cmd_vel_tem", Twist, self.callback)

        # Subscribe to scan for simple obstacle proximity gating
        self.scan_sub = rospy.Subscriber(self.scan_topic, LaserScan, self.scan_callback)

        # 3. 设置发布者：将处理后的数据发往 /cmd_vel (或者是你的机器人实际接收的话题)
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        rospy.loginfo("节点已启动：正在监听 /cmd_vel_tem 并处理数据...")

    def scan_callback(self, msg: LaserScan):
        # Compute min range in a front angular window around 0 rad
        if not msg.ranges or msg.angle_increment == 0.0:
            self.front_min_range = float('inf')
            self.obstacle_close = False
            return

        min_r = float('inf')
        angle = msg.angle_min
        for r in msg.ranges:
            if -self.front_half_angle <= angle <= self.front_half_angle:
                if r is not None and math.isfinite(r) and r > 0.0:
                    if r < min_r:
                        min_r = r
            angle += msg.angle_increment

        self.front_min_range = min_r
        self.obstacle_close = (min_r < self.reverse_obstacle_distance)

    def callback(self, msg):
        """
        回调函数：每次接收到 /cmd_vel_tem 数据时触发
        """
        # 复制一份消息，避免直接修改原引用
        modified_twist = Twist()
        modified_twist.linear.x = msg.linear.x
        modified_twist.linear.y = msg.linear.y
        modified_twist.linear.z = msg.linear.z
        modified_twist.angular.x = msg.angular.x
        modified_twist.angular.y = msg.angular.y
        modified_twist.angular.z = msg.angular.z

        # 核心逻辑：
        # - 永远不要“放大”倒车速度（之前的 -0.3 会导致意外快速后退）
        # - 可配置：只有前方检测到近距离障碍物时才允许倒车
        if modified_twist.linear.x < 0.0:
            original_x = modified_twist.linear.x

            if not self.allow_reverse:
                modified_twist.linear.x = 0.0
                rospy.loginfo_throttle(0.5, f"倒车已禁用，已钳制为 0: 原始值={original_x}")
            elif self.reverse_requires_obstacle and (not self.obstacle_close):
                modified_twist.linear.x = 0.0
                rospy.loginfo_throttle(
                    0.5,
                    f"未检测到前方近障碍物(最小距离={self.front_min_range:.3f}m)，倒车指令已钳制为 0: 原始值={original_x}",
                )
            else:
                # Obstacle close: allow reverse but cap magnitude
                modified_twist.linear.x = max(original_x, -abs(self.max_reverse_speed))
                rospy.loginfo_throttle(
                    0.5,
                    f"检测到前方近障碍物(最小距离={self.front_min_range:.3f}m)，允许倒车并限幅: 原始值={original_x}, 修改后={modified_twist.linear.x}",
                )

        # Forward safety: keep distance from walls/obstacles
        if self.enable_forward_safety and modified_twist.linear.x > 0.0:
            original_x = modified_twist.linear.x
            d = self.front_min_range

            # Hard stop inside stop distance
            if math.isfinite(d) and d < self.forward_stop_distance:
                modified_twist.linear.x = 0.0
                rospy.logwarn_throttle(
                    0.2,
                    f"前方障碍过近(最小距离={d:.3f}m < stop={self.forward_stop_distance:.2f}m)，前进指令已钳制为 0: 原始值={original_x}",
                )
            # Soft slow-down in the buffer zone
            elif math.isfinite(d) and d < self.forward_slow_distance and self.forward_slow_distance > self.forward_stop_distance:
                scale = (d - self.forward_stop_distance) / (self.forward_slow_distance - self.forward_stop_distance)
                scale = max(0.0, min(1.0, scale))
                limited = original_x * scale
                limited = max(0.0, min(original_x, limited))
                if self.min_forward_speed > 0.0:
                    limited = max(min(limited, original_x), min(self.min_forward_speed, original_x))
                modified_twist.linear.x = limited
                rospy.loginfo_throttle(
                    0.5,
                    f"前方接近障碍(最小距离={d:.3f}m)，前进限速: 原始={original_x:.3f}, 修改后={modified_twist.linear.x:.3f}",
                )
        
        # 6. 将（修改后或未修改的）数据发布出去
        self.pub.publish(modified_twist)

if __name__ == '__main__':
    try:
        node = VelocityFilter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass