#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
测试程序：订阅 /ultrasonic/distance_mm
消息类型：std_msgs/Int32
"""

import rospy
from std_msgs.msg import Int32


def distance_callback(msg):
    """
    回调函数：每收到一次消息就会被调用
    """
    distance_mm = msg.data
    rospy.loginfo(f"收到超声波距离: {distance_mm} mm")


def main():

    rospy.init_node("ultrasonic_subscriber_test", anonymous=True)

    rospy.Subscriber(
        "/ultrasonic/distance_mm",
        Int32,
        distance_callback
    )

    rospy.loginfo("Ultrasonic subscriber test started, waiting for data...")

    rospy.spin()


if __name__ == "__main__":
    main()
