#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from collections import deque
import threading

class SyncChecker:
    def __init__(self, window_size=10):
        self.odom_buffer = deque(maxlen=window_size)
        self.scan_buffer = deque(maxlen=window_size)
        self.lock = threading.Lock()

        rospy.Subscriber('/odom', Odometry, self.odom_cb)
        rospy.Subscriber('/scan', LaserScan, self.scan_cb)

    def odom_cb(self, msg):
        with self.lock:
            self.odom_buffer.append(msg.header.stamp.to_sec())

    def scan_cb(self, msg):
        with self.lock:
            self.scan_buffer.append(msg.header.stamp.to_sec())

    def print_latest_diff(self):
        with self.lock:
            if self.odom_buffer and self.scan_buffer:
                latest_odom = self.odom_buffer[-1]
                latest_scan = self.scan_buffer[-1]
                diff = latest_scan - latest_odom
                print(f"[SYNC] scan - odom = {diff:+.3f} s")
            else:
                print("[SYNC] Waiting for data...")

if __name__ == '__main__':
    rospy.init_node('sync_checker', anonymous=True)
    checker = SyncChecker()

    rate = rospy.Rate(2)  # 每秒打印 2 次（避免刷屏）
    try:
        while not rospy.is_shutdown():
            checker.print_latest_diff()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass