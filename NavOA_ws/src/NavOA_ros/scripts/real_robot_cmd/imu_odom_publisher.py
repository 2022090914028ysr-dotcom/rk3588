#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

import rospy
import tf2_ros
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
from robot_control import RobotController

# 假设 IMUController 在同一目录或已加入路径
from imu_controller import IMUController

class IMUOdomPublisher:
    def __init__(self):
        rospy.init_node('imu_odom_publisher', anonymous=True)
        
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        
        self.robot = RobotController.get_instance()
        
        # 如果尚未初始化电机，则初始化（安全幂等）
        if not self.robot.robot_init():
            rospy.logerr("RobotController 初始化失败！")
            raise RuntimeError("电机控制器初始化失败")

        self.imu = IMUController(robot_controller=self.robot, serial_port="/dev/ttyS4")
        if not self.imu.initialize():
            rospy.logerr("IMU 初始化失败！")
            raise RuntimeError("IMU 初始化失败")
        
        if not self.imu.start_reading():
            rospy.logerr("无法启动 IMU 读取线程")
            raise RuntimeError("IMU 启动失败")

        self.rate = rospy.Rate(10)  # 10 Hz
        rospy.loginfo("IMU 里程计发布节点已启动，发布到 /odom")

    def publish_odom(self):
        try:
            while not rospy.is_shutdown():
                if not self.imu.is_data_valid():
                    rospy.logwarn_throttle(1, "IMU 数据无效或超时，跳过本次发布")
                    self.rate.sleep()
                    continue

                odom_data = self.imu.get_odometry_data()

                odom_msg = Odometry()
                odom_msg.header.stamp = rospy.Time.now()
                odom_msg.header.frame_id = "odom"
                odom_msg.child_frame_id = "base_footprint"

                odom_msg.pose.pose.position.x = odom_data.x
                odom_msg.pose.pose.position.y = odom_data.y
                odom_msg.pose.pose.position.z = odom_data.z

                odom_msg.pose.pose.orientation = Quaternion(
                    x=odom_data.qx,
                    y=odom_data.qy,
                    z=odom_data.qz,
                    w=odom_data.qw
                )

                # 通常差分驱动只使用 x 方向线速度和 z 方向角速度
                odom_msg.twist.twist.linear.x = odom_data.vx
                odom_msg.twist.twist.linear.y = 0.0
                odom_msg.twist.twist.linear.z = 0.0
                odom_msg.twist.twist.angular.x = 0.0
                odom_msg.twist.twist.angular.y = 0.0
                odom_msg.twist.twist.angular.z = odom_data.wz

                self.odom_pub.publish(odom_msg)

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

        except (rospy.ROSInterruptException, KeyboardInterrupt):
            rospy.loginfo("接收到中断信号，正在关闭...")
        except Exception as e:
            rospy.logerr(f"发布里程计时发生异常: {e}")
        finally:
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