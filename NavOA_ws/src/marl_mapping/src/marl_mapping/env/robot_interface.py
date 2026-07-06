#!/usr/bin/python3
"""
Single Robot Interface for Real-World Autonomous Mapping
Subscribes to global topics: /map, /odom_rf2o, publishes /cmd_vel.
No RGB camera, no namespace prefix, no EKF required.
"""

import rospy
import numpy as np
import threading

from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry


class SingleRobot:
    """Single robot interface for cartographer-based robot with rf2o odometry."""

    def __init__(self, rb_name="car1", check_topic=False,
                 map_topic="/map", odom_topic="/odom_rf2o",
                 cmd_vel_topic="/cmd_vel", check_timeout=30.0):
        self.rb_name = rb_name
        self.map_data = None
        self.map_info = None
        self.odom_pos = [0.0, 0.0, 0.0]
        self.odom_orientation = [0.0, 0.0, 0.0, 1.0]

        self.map_init = False
        self.odom_init = False

        self.lock = threading.Lock()

        self.map_sub = rospy.Subscriber(
            map_topic, OccupancyGrid, self.map_callback, queue_size=1)
        self.odom_sub = rospy.Subscriber(
            odom_topic, Odometry, self.odom_callback, queue_size=1)
        self.vel_pub = rospy.Publisher(cmd_vel_topic, Twist, queue_size=1)

        rospy.loginfo("[%s] Topics: map=%s, odom=%s, cmd=%s",
                      rb_name, map_topic, odom_topic, cmd_vel_topic)

        if check_topic:
            rospy.loginfo("[%s] Waiting for topics (timeout=%.0fs)...",
                         rb_name, check_timeout)
            start = rospy.Time.now()
            to = rospy.Duration(check_timeout)
            while not self.map_init:
                if rospy.Time.now() - start > to:
                    rospy.logwarn("[%s] Map timeout", rb_name)
                    break
                rospy.sleep(0.2)
            while not self.odom_init:
                if rospy.Time.now() - start > to:
                    rospy.logwarn("[%s] Odom timeout", rb_name)
                    break
                rospy.sleep(0.2)
            rospy.loginfo("[%s] Ready: map=%s odom=%s",
                         rb_name, self.map_init, self.odom_init)

    # ---- Callbacks ----
    def map_callback(self, msg):
        try:
            with self.lock:
                self.map_info = msg.info
                self.map_data = np.array(msg.data).reshape(
                    (msg.info.height, msg.info.width))
                if not self.map_init:
                    self.map_init = True
                    rospy.loginfo("[%s] Map: %dx%d res=%.3f",
                                 self.rb_name, msg.info.width,
                                 msg.info.height, msg.info.resolution)
        except Exception as e:
            rospy.logerr_throttle(10, "[%s] Map err: %s", self.rb_name, e)

    def odom_callback(self, msg):
        with self.lock:
            p = msg.pose.pose.position
            q = msg.pose.pose.orientation
            self.odom_pos = [p.x, p.y, p.z]
            self.odom_orientation = [q.x, q.y, q.z, q.w]
            if not self.odom_init:
                self.odom_init = True
                rospy.loginfo("[%s] Odom ready", self.rb_name)

    # ---- Data access ----
    def get_map(self):
        with self.lock:
            return self.map_data.copy() if self.map_data is not None else None

    def get_map_info(self):
        with self.lock:
            return self.map_info

    def get_ekf_pos(self):
        return list(self.odom_pos)

    def get_ekf_orientation(self):
        return list(self.odom_orientation)

    # ---- Motion ----
    def send_vel(self, linear, angular):
        t = Twist()
        t.linear.x = linear
        t.angular.z = angular
        self.vel_pub.publish(t)

    def move(self, n=1, move_type=8, linear=0.2, angular=0.6):
        """8=Forward 2=Back 6=Right 4=Left 5=Stop"""
        if move_type == 8:
            self.send_vel(linear, 0)
        elif move_type == 2:
            self.send_vel(-linear, 0)
        elif move_type == 6:
            self.send_vel(0, -angular)
        elif move_type == 4:
            self.send_vel(0, angular)
        else:
            self.send_vel(0, 0)

    def stop(self):
        self.send_vel(0, 0)
