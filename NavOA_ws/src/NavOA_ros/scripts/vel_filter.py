#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist

class VelocityFilter:
    def __init__(self):
        # 1. 初始化节点
        rospy.init_node('cmd_vel_modifier_node', anonymous=True)

        # 2. 设置订阅者：监听 /cmd_vel_tem
        self.sub = rospy.Subscriber("/cmd_vel_tem", Twist, self.callback)

        # 3. 设置发布者：将处理后的数据发往 /cmd_vel (或者是你的机器人实际接收的话题)
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        rospy.loginfo("节点已启动：正在监听 /cmd_vel_tem 并处理数据...")

    def callback(self, msg):
        """
        回调函数：每次接收到 /cmd_vel_tem 数据时触发
        """
        # 复制一份消息，避免直接修改原引用（虽然在Python中赋值对于简单类型是安全的，但这是好习惯）
        modified_twist = msg

        # 4. 核心逻辑：检测 x 方向线速度是否小于 0
        if modified_twist.linear.x < 0:
            original_x = modified_twist.linear.x
            
            # 5. 在此基础上减去 0.3
            # 注意：负数减去 0.3 会变得更小（即绝对值变大，倒退速度变快）
            # 例如：-0.1 变为 -0.4
            modified_twist.linear.x = original_x - 0.3
            
            # 打印调试信息（可选）
            rospy.loginfo(f"检测到后退: 原始值={original_x}, 修改后={modified_twist.linear.x}")
        
        # 6. 将（修改后或未修改的）数据发布出去
        self.pub.publish(modified_twist)

if __name__ == '__main__':
    try:
        node = VelocityFilter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass