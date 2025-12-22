#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import tf.transformations as tftrans
import numpy as np

# 全局变量（或使用类封装更好）
last_time = None
last_yaw = None

def quaternion_to_yaw(q):
    _, _, yaw = tftrans.euler_from_quaternion([q.x, q.y, q.z, q.w])
    return yaw

def vins_odom_callback(msg):
    global last_time, last_yaw

    # ✅ 获取当前 ROS 时间（关键！）
    current_ros_time = rospy.Time.now()

    # 创建新消息
    new_odom = Odometry()
    new_odom.header.stamp = current_ros_time  # ✅ 修正：使用当前时间
    new_odom.header.frame_id = "odom"
    new_odom.child_frame_id = "base_footprint"
    new_odom.pose.pose = msg.pose.pose

    # ... [协方差设置部分保持不变] ...

    # 估算 angular.z（可选，不影响时间戳）
    current_yaw = quaternion_to_yaw(msg.pose.pose.orientation)
    # 注意：这里仍用 msg.header.stamp 计算 dt（因为需要 VINS 内部时间差）
    current_vins_time = msg.header.stamp.to_sec()
    if last_time is not None and last_yaw is not None and current_vins_time > last_time:
        dt = current_vins_time - last_time
        if dt > 1e-6:
            dyaw = current_yaw - last_yaw
            dyaw = np.arctan2(np.sin(dyaw), np.cos(dyaw))
            new_odom.twist.twist.angular.z = dyaw / dt
        else:
            new_odom.twist.twist.angular.z = 0.0
    else:
        new_odom.twist.twist.angular.z = 0.0

    # ... [twist 协方差保持不变] ...

    # 发布
    odom_pub.publish(new_odom)

    # 发布 TF（关键修正！）
    t = TransformStamped()
    t.header.stamp = current_ros_time  # ✅ 修正：使用当前时间
    t.header.frame_id = "odom"
    t.child_frame_id = "base_footprint"
    t.transform.translation = msg.pose.pose.position
    t.transform.rotation = msg.pose.pose.orientation
    tf_broadcaster.sendTransform(t)

    # 更新历史（仍用 VINS 时间计算角速度）
    last_yaw = current_yaw
    last_time = current_vins_time  # 注意：这里保持 VINS 时间

if __name__ == '__main__':
    rospy.init_node('vins_to_odom')
    odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
    tf_broadcaster = TransformBroadcaster()
    rospy.Subscriber('/vins_estimator/odometry', Odometry, vins_odom_callback)
    rospy.spin()
