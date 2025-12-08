#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

import rospy
import tf2_ros
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
from imu_controller import IMUController  # 请确保路径正确
from robot_control import RobotController  # 你自己的 RobotController 所在模块

class IMUOdomPublisher:
    def __init__(self):
        # 初始化 ROS 节点
        rospy.init_node('imu_odom_publisher', anonymous=True)
        
        # 创建 odom 发布者
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
        
        # 创建 TF 广播器
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        
        # 初始化 RobotController（用于提供线速度）
        self.robot = RobotController()
        if not self.robot.robot_init():
            rospy.logerr("RobotController 初始化失败！")
            raise RuntimeError("电机控制器初始化失败")

        # 初始化 IMUController，并绑定机器人控制器
        self.imu = IMUController(robot_controller=self.robot, serial_port="/dev/ttyS4")
        if not self.imu.initialize():
            rospy.logerr("IMU 初始化失败！")
            raise RuntimeError("IMU 初始化失败")
        
        # 启动 IMU 后台读取线程
        if not self.imu.start_reading():
            rospy.logerr("无法启动 IMU 读取线程")
            raise RuntimeError("IMU 启动失败")

        # 发布频率（建议 30~100 Hz）
        self.rate = rospy.Rate(50)  # 50 Hz

        rospy.loginfo("IMU 里程计发布节点已启动，发布到 /odom")

    def publish_odom(self):
        """主循环：发布 odom 和 TF"""
        try:
            while not rospy.is_shutdown():
                # 检查 IMU 数据是否有效
                if not self.imu.is_data_valid():
                    rospy.logwarn_throttle(1, "IMU 数据无效或超时，跳过本次发布")
                    self.rate.sleep()
                    continue

                # 获取最新的里程计数据（由 IMUController 自动更新）
                odom_data = self.imu.odometry_data

                # 创建 Odometry 消息
                odom_msg = Odometry()
                odom_msg.header.stamp = rospy.Time.now()
                odom_msg.header.frame_id = "odom"
                odom_msg.child_frame_id = "base_footprint"

                # 位置
                odom_msg.pose.pose.position.x = odom_data.x
                odom_msg.pose.pose.position.y = odom_data.y
                odom_msg.pose.pose.position.z = odom_data.z

                # 姿态（四元数）
                odom_msg.pose.pose.orientation.x = odom_data.qx
                odom_msg.pose.pose.orientation.y = odom_data.qy
                odom_msg.pose.pose.orientation.z = odom_data.qz
                odom_msg.pose.pose.orientation.w = odom_data.qw

                # 速度（线速度 + 角速度）
                odom_msg.twist.twist.linear.x = odom_data.vx
                odom_msg.twist.twist.linear.y = odom_data.vy
                odom_msg.twist.twist.linear.z = odom_data.vz

                odom_msg.twist.twist.angular.x = odom_data.wx
                odom_msg.twist.twist.angular.y = odom_data.wy
                odom_msg.twist.twist.angular.z = odom_data.wz

                # 发布 /odom
                self.odom_pub.publish(odom_msg)

                # 发布 TF: odom -> base_link
                t = TransformStamped()
                t.header.stamp = odom_msg.header.stamp
                t.header.frame_id = "odom"
                t.child_frame_id = "base_footprint"
                t.transform.translation.x = odom_data.x
                t.transform.translation.y = odom_data.y
                t.transform.translation.z = odom_data.z
                t.transform.rotation = odom_msg.pose.pose.orientation

                self.tf_broadcaster.sendTransform(t)

                self.rate.sleep()

        except rospy.ROSInterruptException:
            pass
        finally:
            # 安全关闭
            rospy.loginfo("正在关闭 IMU 和机器人控制器...")
            self.imu.stop_reading()
            self.imu.close()
            self.robot.robot_close()

if __name__ == '__main__':
    try:
        publisher = IMUOdomPublisher()
        publisher.publish_odom()
    except Exception as e:
        rospy.logerr(f"IMU 里程计节点异常退出: {e}")
        rospy.signal_shutdown("Error occurred")