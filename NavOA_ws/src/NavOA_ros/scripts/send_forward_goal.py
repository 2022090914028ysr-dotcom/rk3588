#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped, PointStamped
import math
from tf.transformations import quaternion_from_euler


def publish_forward_goal():
    rospy.init_node('send_forward_goal')

    target_frame = rospy.get_param('~target_frame', 'map')
    base_frame = rospy.get_param('~base_frame', 'base_footprint')
    distance = float(rospy.get_param('~distance', 10.0))
    z = float(rospy.get_param('~z', 0.0))
    yaw_offset = float(rospy.get_param('~yaw_offset', 0.0))

    pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1, latch=True)

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    try:
        tf = tf_buffer.lookup_transform(target_frame, base_frame, rospy.Time(0), rospy.Duration(5.0))
    except Exception as e:
        rospy.logerr('TF lookup failed %s -> %s: %s', target_frame, base_frame, e)
        return

    ps = PointStamped()
    ps.header.stamp = rospy.Time.now()
    ps.header.frame_id = base_frame
    ps.point.x = distance
    ps.point.y = 0.0
    ps.point.z = z

    try:
        p_target = tf2_geometry_msgs.do_transform_point(ps, tf)
    except Exception as e:
        rospy.logerr('Point transform failed: %s', e)
        return

    goal = PoseStamped()
    goal.header.stamp = rospy.Time.now()
    goal.header.frame_id = target_frame
    goal.pose.position = p_target.point

    # Compute yaw to face the target point from current base position in target_frame
    base_tx = tf.transform.translation.x
    base_ty = tf.transform.translation.y
    dx = p_target.point.x - base_tx
    dy = p_target.point.y - base_ty
    yaw = math.atan2(dy, dx)
    q = quaternion_from_euler(0.0, 0.0, yaw + yaw_offset)
    goal.pose.orientation.x = q[0]
    goal.pose.orientation.y = q[1]
    goal.pose.orientation.z = q[2]
    goal.pose.orientation.w = q[3]

    rospy.loginfo('Publishing 2D Nav Goal: distance %.2fm, frame=%s', distance, target_frame)
    pub.publish(goal)
    rospy.sleep(0.5)


if __name__ == '__main__':
    try:
        publish_forward_goal()
    except rospy.ROSInterruptException:
        pass