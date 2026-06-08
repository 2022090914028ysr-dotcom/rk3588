#!/usr/bin/env python3

import rospy
import time
from geometry_msgs.msg import PoseStamped
import tf

def publish_2d_goal(x, y, theta, frame_id="map"):
    """发布2D Nav Goal位姿（模拟RViz 2D Nav Goal）"""
    pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
    rospy.init_node('cyclic_2d_goal_publisher', anonymous=True)

    goal_msg = PoseStamped()
    goal_msg.header.frame_id = frame_id
    goal_msg.header.stamp = rospy.Time.now()

    # 设置位置
    goal_msg.pose.position.x = x
    goal_msg.pose.position.y = y
    goal_msg.pose.position.z = 0.0  # 2D导航z固定为0

    # 将角度转换为四元数
    quaternion = tf.transformations.quaternion_from_euler(0, 0, theta)
    goal_msg.pose.orientation.x = quaternion[0]
    goal_msg.pose.orientation.y = quaternion[1]
    goal_msg.pose.orientation.z = quaternion[2]
    goal_msg.pose.orientation.w = quaternion[3]

    pub.publish(goal_msg)
    rospy.loginfo("Published goal: (x={}, y={}, theta={} rad)".format(x, y, theta))

if __name__ == '__main__':
    try:
        # 定义两个位姿（x, y, theta，theta单位为弧度）
        pose1 = (10.377, 1.770, 41.2 * 3.14159 / 180)  # 角度转换为弧度
        pose2 = (2.277, 3.720, 135.0 * 3.14159 / 180)
        
        while not rospy.is_shutdown():
            publish_2d_goal(*pose1)
            time.sleep(120)
            publish_2d_goal(*pose2)
            time.sleep(120)
    except rospy.ROSInterruptException:
        pass