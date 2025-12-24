#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler

def vins_odom_callback(msg):
    # 1. 获取 VINS 的高频时间戳
    vins_time = msg.header.stamp

    # 2. 提取并拍扁位姿 (Pose)
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    # z = msg.pose.pose.position.z # 丢弃

    quat = msg.pose.pose.orientation
    roll, pitch, yaw = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])

    # 强制 2D 化
    flat_x = x
    flat_y = y
    flat_z = 0.0
    flat_roll = 0.0
    flat_pitch = 0.0
    flat_yaw = yaw 

    flat_quat = quaternion_from_euler(flat_roll, flat_pitch, flat_yaw)

    # 3. 构建新的 Odom 消息
    new_odom = Odometry()
    new_odom.header.stamp = vins_time
    new_odom.header.frame_id = "odom"
    new_odom.child_frame_id = "base_footprint"

    # 填充位姿
    new_odom.pose.pose.position.x = flat_x
    new_odom.pose.pose.position.y = flat_y
    new_odom.pose.pose.position.z = flat_z
    new_odom.pose.pose.orientation.x = flat_quat[0]
    new_odom.pose.pose.orientation.y = flat_quat[1]
    new_odom.pose.pose.orientation.z = flat_quat[2]
    new_odom.pose.pose.orientation.w = flat_quat[3]
    new_odom.pose.covariance = msg.pose.covariance # 协方差直接透传即可

    # ---------------------------------------------------------
    # 【优化】拍扁速度 (Twist) —— 防止导航包报错
    # ---------------------------------------------------------
    new_odom.twist.twist.linear.x = msg.twist.twist.linear.x
    new_odom.twist.twist.linear.y = msg.twist.twist.linear.y
    new_odom.twist.twist.linear.z = 0.0  # 强制 Z 轴速度为 0

    new_odom.twist.twist.angular.x = 0.0 # 强制 Roll 角速度为 0
    new_odom.twist.twist.angular.y = 0.0 # 强制 Pitch 角速度为 0
    new_odom.twist.twist.angular.z = msg.twist.twist.angular.z # 保留 Yaw 角速度
    new_odom.twist.covariance = msg.twist.covariance
    # ---------------------------------------------------------

    # 4. 发布 /odom
    odom_pub.publish(new_odom)

    # 5. 发布 TF
    t = TransformStamped()
    t.header.stamp = vins_time
    t.header.frame_id = "odom"
    t.child_frame_id = "base_footprint"
    t.transform.translation.x = flat_x
    t.transform.translation.y = flat_y
    t.transform.translation.z = flat_z
    t.transform.rotation.x = flat_quat[0]
    t.transform.rotation.y = flat_quat[1]
    t.transform.rotation.z = flat_quat[2]
    t.transform.rotation.w = flat_quat[3]
    tf_broadcaster.sendTransform(t)

if __name__ == '__main__':
    rospy.init_node('vins_to_odom')
    
    odom_pub = rospy.Publisher('/odom', Odometry, queue_size=100)
    tf_broadcaster = TransformBroadcaster()
    
    # 使用 imu_propagate 是对的，它是 VINS 输出的最高频、最低延时的话题
    rospy.Subscriber('/vins_estimator/imu_propagate', Odometry, vins_odom_callback)
    
    rospy.loginfo("[vins_to_odom] 2D flattener running...")
    rospy.spin()