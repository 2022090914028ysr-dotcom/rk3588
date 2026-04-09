#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from tf2_ros import TransformBroadcaster


class OdomTopicToTf(object):
    def __init__(self):
        rospy.init_node("odom_topic_to_tf", anonymous=False)

        self.odom_topic = rospy.get_param("~odom_topic", "/odom")
        self.parent_frame = rospy.get_param("~parent_frame", "odom")
        self.child_frame = rospy.get_param("~child_frame", "base_footprint")
        self.flatten_2d = rospy.get_param("~flatten_2d", True)

        self._broadcaster = TransformBroadcaster()
        self._sub = rospy.Subscriber(self.odom_topic, Odometry, self._on_odom, queue_size=20)

        rospy.loginfo(
            "odom_topic_to_tf bridging %s to TF %s -> %s (flatten_2d=%s)",
            self.odom_topic,
            self.parent_frame,
            self.child_frame,
            self.flatten_2d,
        )

    def _on_odom(self, msg: Odometry):
        transform = TransformStamped()
        transform.header.stamp = msg.header.stamp
        transform.header.frame_id = self.parent_frame
        transform.child_frame_id = self.child_frame

        transform.transform.translation.x = msg.pose.pose.position.x
        transform.transform.translation.y = msg.pose.pose.position.y
        transform.transform.translation.z = 0.0 if self.flatten_2d else msg.pose.pose.position.z

        orientation = msg.pose.pose.orientation
        if self.flatten_2d:
            roll, pitch, yaw = euler_from_quaternion(
                [orientation.x, orientation.y, orientation.z, orientation.w]
            )
            qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, yaw)
            transform.transform.rotation.x = qx
            transform.transform.rotation.y = qy
            transform.transform.rotation.z = qz
            transform.transform.rotation.w = qw
        else:
            transform.transform.rotation = orientation

        self._broadcaster.sendTransform(transform)


def main():
    OdomTopicToTf()
    rospy.spin()


if __name__ == "__main__":
    main()