#!/usr/bin/env python3
"""
Leg-odometry ground/flattening bridge.

The remote leg odometry (fusion_estimator_node, topic SMX/Odom) publishes x/y/yaw
correctly, but its body Z is INVERTED (-0.45 instead of +0.45) because its
body-orientation quaternion carries a ~180deg pitch/roll flip. This node consumes
that topic and emits two corrected outputs, both with the pitch/roll dropped (yaw
only) and z zeroed:

  1. TF    odom -> base_footprint   (z = 0, ground level)
  2. topic /SMX/Odom_flat (Odometry) (z = 0, child = base_footprint) for Cartographer

A static base_footprint -> base_link (+0.45) lifts the body back up.

Why the flattened odometry matters: Cartographer subscribes to the odometry and
uses its pose (including z) to build the published map -> odom transform. If it
reads the raw SMX/Odom, the inverted z leaks into map -> odom and the whole tree
bobs below ground. Point Cartographer's "odom" remap at /SMX/Odom_flat instead.

IMPORTANT: the remote leg node must have publish_tf = false (it must only publish
the SMX/Odom topic, never a TF), otherwise base_link gets two parents.
"""

import math

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry


def quat_to_yaw(q):
    """Return yaw (rad) from a geometry_msgs/Quaternion (x, y, z, w)."""
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def yaw_to_quat(yaw):
    """Return a pure-yaw quaternion as [x, y, z, w]."""
    return [0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)]


class LegOdomGroundBridge:
    def __init__(self):
        self.odom_topic = rospy.get_param("~odom_topic", "SMX/Odom")
        self.odom_frame = rospy.get_param("~odom_frame", "odom")
        self.footprint_frame = rospy.get_param("~footprint_frame", "base_footprint")
        self.flat_odom_topic = rospy.get_param("~flat_odom_topic", "/SMX/Odom_flat")
        self.flat_child_frame = rospy.get_param("~flat_child_frame", "base_footprint")

        self.br = tf2_ros.TransformBroadcaster()
        self.flat_pub = rospy.Publisher(self.flat_odom_topic, Odometry, queue_size=10)
        self.sub = rospy.Subscriber(self.odom_topic, Odometry, self._cb)

        rospy.loginfo("leg_odom_footprint_broadcaster: %s -> %s -> %s TF (z=0) + %s (z=0)",
                      self.odom_topic, self.odom_frame, self.footprint_frame,
                      self.flat_odom_topic)

    def _cb(self, msg):
        yaw = quat_to_yaw(msg.pose.pose.orientation)
        q = yaw_to_quat(yaw)

        # 1) dynamic TF: odom -> base_footprint (ground, yaw-only)
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.footprint_frame
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = 0.0  # discard inverted leg z
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.br.sendTransform(t)

        # 2) flattened odometry for Cartographer (z=0, yaw-only)
        flat = Odometry()
        flat.header = msg.header
        flat.header.frame_id = self.odom_frame
        flat.child_frame_id = self.flat_child_frame
        flat.pose.pose.position.x = msg.pose.pose.position.x
        flat.pose.pose.position.y = msg.pose.pose.position.y
        flat.pose.pose.position.z = 0.0
        flat.pose.pose.orientation.x = q[0]
        flat.pose.pose.orientation.y = q[1]
        flat.pose.pose.orientation.z = q[2]
        flat.pose.pose.orientation.w = q[3]
        flat.pose.covariance = msg.pose.covariance
        flat.twist = msg.twist
        flat.twist.twist.linear.z = 0.0
        self.flat_pub.publish(flat)


def main():
    rospy.init_node("leg_odom_footprint_broadcaster")
    LegOdomGroundBridge()
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
