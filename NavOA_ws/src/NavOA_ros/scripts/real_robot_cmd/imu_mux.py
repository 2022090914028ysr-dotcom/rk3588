#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""IMU mux node: prefer serial IMU, fall back to ROS IMU.

Subscribes to two Imu topics:
  - primary_topic: e.g. /imu_serial (from serial bridge)
  - fallback_topic: e.g. /imu (original ROS IMU from other PC)

Publishes a single Imu stream on output_topic (e.g. /imu_mux), choosing:
  - primary IMU whenever messages are arriving within timeout seconds
  - otherwise, uses fallback IMU.

This lets the system "prefer serial, else ROS" cleanly.
"""

import rospy
from sensor_msgs.msg import Imu


class ImuMux(object):
    def __init__(self):
        rospy.init_node("imu_mux", anonymous=False)

        self.primary_topic = rospy.get_param("~primary_topic", "/imu_serial")
        self.fallback_topic = rospy.get_param("~fallback_topic", "/imu")
        self.output_topic = rospy.get_param("~output_topic", "/imu_mux")
        self.timeout = rospy.get_param("~timeout", 0.2)

        self.last_primary_time = None
        self.last_fallback_time = None

        self.pub = rospy.Publisher(self.output_topic, Imu, queue_size=50)

        self.primary_sub = rospy.Subscriber(
            self.primary_topic, Imu, self._primary_cb, queue_size=50
        )
        self.fallback_sub = rospy.Subscriber(
            self.fallback_topic, Imu, self._fallback_cb, queue_size=50
        )

        rospy.loginfo(
            "imu_mux running: primary=%s, fallback=%s, output=%s, timeout=%.3fs",
            self.primary_topic,
            self.fallback_topic,
            self.output_topic,
            self.timeout,
        )

    def _use_primary(self):
        """Return True if primary IMU is considered active."""
        if self.last_primary_time is None:
            return False
        now = rospy.Time.now()
        dt = (now - self.last_primary_time).to_sec()
        return dt <= self.timeout

    def _primary_cb(self, msg: Imu):
        self.last_primary_time = rospy.Time.now()
        # Always forward primary IMU when available
        self.pub.publish(msg)

    def _fallback_cb(self, msg: Imu):
        self.last_fallback_time = rospy.Time.now()
        # Only forward fallback if primary is not active
        if not self._use_primary():
            self.pub.publish(msg)


def main():
    mux = ImuMux()
    rospy.spin()


if __name__ == "__main__":
    main()
