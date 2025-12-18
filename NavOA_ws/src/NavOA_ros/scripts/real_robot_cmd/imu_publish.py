#!/usr/bin/python3
# -*- coding: UTF-8 -*-

import rospy
import math
import sys
import os

# 导入 ROS 消息类型
from sensor_msgs.msg import Imu

# 尝试导入你提供的 imu_controller 模块
# 假设 imu_controller.py 位于同一目录。
# 已移除对 robot_config 的导入和依赖
import imu_controller
from imu_controller import imu_controller # 导入全局实例

# --- 移除对 robot_config 的导入 ---
# import robot_config
# from robot_config import config
# -----------------------------------


def euler_to_quaternion(roll, pitch, yaw):
    """
    将欧拉角 (Roll, Pitch, Yaw) 转换为四元数 (x, y, z, w)
    """
    # 
    
    # 公式： q = [sin(r/2)cos(p/2)cos(y/2) - cos(r/2)sin(p/2)sin(y/2),
    #             cos(r/2)sin(p/2)cos(y/2) + sin(r/2)cos(p/2)sin(y/2),
    #             cos(r/2)cos(p/2)sin(y/2) - sin(r/2)sin(p/2)cos(y/2),
    #             cos(r/2)cos(p/2)cos(y/2) + sin(r/2)sin(p/2)sin(y/2)]
            
    qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
    qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
    qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    return [qx, qy, qz, qw]

class IMUPublisherNode:
    def __init__(self):
        # 1. 初始化 ROS 节点
        rospy.init_node('wit_imu_driver', anonymous=False)
        
        # 2. 获取参数 (可选，支持从 launch 文件修改)
        self.imu_topic = rospy.get_param('~imu_topic', '/imu')
        self.frame_id = rospy.get_param('~frame_id', 'imu_link')
        self.publish_rate = rospy.get_param('~rate', 200) # 目标频率 200Hz

        # 3. 创建发布者
        self.pub = rospy.Publisher(self.imu_topic, Imu, queue_size=10)
        
        rospy.loginfo(f"正在初始化 IMU 驱动，发布话题: {self.imu_topic}，Frame ID: {self.frame_id}")

        # 4. 初始化底层控制器
        if not imu_controller.initialize():
            rospy.logerr("IMU 控制器初始化失败！")
            sys.exit(1)
        
        # 5. 启动读取线程
        imu_controller.start_reading()
        
        # 等待数据稳定
        rospy.sleep(1)

    def run(self):
        rate = rospy.Rate(self.publish_rate)
        
        while not rospy.is_shutdown():
            # 检查数据是否有效
            if imu_controller.is_data_valid():
                # 获取当前数据 (IMUData 对象)
                # 注意：get_current_data() 返回的是已校准和处理过的数据
                data = imu_controller.get_current_data() 
                
                # 构建 ROS 消息
                imu_msg = Imu()
                imu_msg.header.stamp = rospy.Time.now()
                imu_msg.header.frame_id = self.frame_id
                
                # --- 1. 线加速度 (m/s^2) ---
                imu_msg.linear_acceleration.x = data.accel_x
                imu_msg.linear_acceleration.y = data.accel_y
                imu_msg.linear_acceleration.z = data.accel_z
                
                # --- 2. 角速度 (rad/s) ---
                imu_msg.angular_velocity.x = data.gyro_x
                imu_msg.angular_velocity.y = data.gyro_y
                imu_msg.angular_velocity.z = data.gyro_z
                
                # --- 3. 姿态 (四元数) ---
                q = euler_to_quaternion(data.roll, data.pitch, data.yaw)
                imu_msg.orientation.x = q[0]
                imu_msg.orientation.y = q[1]
                imu_msg.orientation.z = q[2]
                imu_msg.orientation.w = q[3]
                
                # --- 4. 协方差矩阵 (Covariance) ---
                imu_msg.angular_velocity_covariance = [1e-4, 0, 0, 0, 1e-4, 0, 0, 0, 1e-4]
                imu_msg.linear_acceleration_covariance = [1e-3, 0, 0, 0, 1e-3, 0, 0, 0, 1e-3]
                imu_msg.orientation_covariance = [1e-4, 0, 0, 0, 1e-4, 0, 0, 0, 1e-4]

                # 发布消息
                self.pub.publish(imu_msg)
            
            else:
                rospy.logwarn_throttle(2.0, "等待有效的 IMU 数据...")
            
            rate.sleep()

    def cleanup(self):
        rospy.loginfo("正在关闭 IMU 驱动...")
        # 调用 IMU 控制器的 close 方法，停止读取线程并关闭串口
        imu_controller.close()

if __name__ == '__main__':
    node = IMUPublisherNode()
    try:
        node.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        node.cleanup()