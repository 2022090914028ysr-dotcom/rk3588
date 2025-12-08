#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import Twist, Quaternion
from nav_msgs.msg import Odometry
import tf
import math

class CmdVelOdom:
    def __init__(self):
        # 参数
        self.frame_id = rospy.get_param("~odom_frame", "odom")
        self.child_frame_id = rospy.get_param("~base_frame", "base_footprint")
        self.pub_tf = rospy.get_param("~pub_odom_tf", True)

        # 里程计状态
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.v = 0.0
        self.omega = 0.0

        self.last_time = rospy.Time.now()

        # 订阅 /cmd_vel
        self.cmd_sub = rospy.Subscriber("/cmd_vel", Twist, self.cmd_cb, queue_size=10)
        # 发布 /odom
        self.odom_pub = rospy.Publisher("/odom", Odometry, queue_size=10)
        # 发布 TF
        self.br = tf.TransformBroadcaster()

    def cmd_cb(self, msg):
        self.v = msg.linear.x
        self.omega = msg.angular.z

    def spin(self):
        rate = rospy.Rate(50)  # 积分频率，50Hz
        while not rospy.is_shutdown():
            now = rospy.Time.now()
            dt = (now - self.last_time).to_sec()
            self.last_time = now

            # 差速模型积分
            delta_x = self.v * math.cos(self.theta) * dt
            delta_y = self.v * math.sin(self.theta) * dt
            delta_theta = self.omega * dt

            self.x += delta_x
            self.y += delta_y
            self.theta += delta_theta

            # 归一化角度
            self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

            # 四元数
            quat = tf.transformations.quaternion_from_euler(0, 0, self.theta)

            # 发布 TF
            if self.pub_tf:
                self.br.sendTransform(
                    (self.x, self.y, 0.0),
                    quat,
                    now,
                    self.child_frame_id,
                    self.frame_id
                )

            # 发布 Odometry
            odom = Odometry()
            odom.header.stamp = now
            odom.header.frame_id = self.frame_id
            odom.child_frame_id = self.child_frame_id

            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y
            odom.pose.pose.position.z = 0.0
            odom.pose.pose.orientation = Quaternion(*quat)

            odom.twist.twist.linear.x = self.v
            odom.twist.twist.angular.z = self.omega

            self.odom_pub.publish(odom)

            rate.sleep()

if __name__ == "__main__":
    rospy.init_node("cmd_vel_odom_node")
    node = CmdVelOdom()
    node.spin()