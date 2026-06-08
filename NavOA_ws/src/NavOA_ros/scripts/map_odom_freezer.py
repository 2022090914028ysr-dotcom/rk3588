#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import time
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from std_srvs.srv import Empty
import subprocess


def lookup_transform(buffer, parent, child, timeout=5.0):
    deadline = time.time() + timeout
    last_err = None
    while not rospy.is_shutdown() and time.time() < deadline:
        try:
            return buffer.lookup_transform(parent, child, rospy.Time(0), rospy.Duration(0.5))
        except Exception as e:
            last_err = e
            rospy.sleep(0.1)
    if last_err:
        raise last_err
    raise RuntimeError("Timeout waiting for transform {} -> {}".format(parent, child))


def transform_to_static(ts):
    out = TransformStamped()
    out.header.stamp = rospy.Time.now()
    out.header.frame_id = ts.header.frame_id
    out.child_frame_id = ts.child_frame_id
    out.transform.translation = ts.transform.translation
    out.transform.rotation = ts.transform.rotation
    return out


def kill_node(node_name):
    try:
        # Prefer rosnode kill for reliability
        subprocess.check_call(["rosnode", "kill", node_name])
        return True
    except Exception as e:
        rospy.logwarn("Failed to kill node {} via rosnode: {}".format(node_name, e))
        return False


def main():
    rospy.init_node('map_odom_freezer')

    parent_frame = rospy.get_param('~parent_frame', 'map')
    child_frame = rospy.get_param('~child_frame', 'odom')
    freeze_after_secs = float(rospy.get_param('~freeze_after_secs', 30.0))
    carto_node = rospy.get_param('~cartographer_node_name', '/cartographer_node')
    wait_for_stability_secs = float(rospy.get_param('~stability_secs', 5.0))

    rospy.loginfo(u"[map_odom_freezer] waiting {:.1f}s for TF {} -> {} then killing {}".format(
        freeze_after_secs, parent_frame, child_frame, carto_node))

    buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
    listener = tf2_ros.TransformListener(buffer)

    # Ensure transform is available first
    try:
        _ = lookup_transform(buffer, parent_frame, child_frame, timeout=freeze_after_secs + 10.0)
    except Exception as e:
        rospy.logerr(u"[map_odom_freezer] initial transform not available {} -> {}: {}".format(parent_frame, child_frame, e))
        return

    # Wait for the desired time window
    start = rospy.Time.now()
    while not rospy.is_shutdown() and (rospy.Time.now() - start).to_sec() < freeze_after_secs:
        rospy.sleep(0.1)

    # Optionally ensure stability: sample transform for stability window and verify small changes
    try:
        first = lookup_transform(buffer, parent_frame, child_frame, timeout=2.0)
        t0 = rospy.Time.now()
        stable = True
        while not rospy.is_shutdown() and (rospy.Time.now() - t0).to_sec() < wait_for_stability_secs:
            cur = lookup_transform(buffer, parent_frame, child_frame, timeout=1.0)
            # Simple stability check: rotation quaternion and translation difference thresholds
            dx = abs(cur.transform.translation.x - first.transform.translation.x)
            dy = abs(cur.transform.translation.y - first.transform.translation.y)
            dz = abs(cur.transform.translation.z - first.transform.translation.z)
            dq = (
                abs(cur.transform.rotation.x - first.transform.rotation.x) +
                abs(cur.transform.rotation.y - first.transform.rotation.y) +
                abs(cur.transform.rotation.z - first.transform.rotation.z) +
                abs(cur.transform.rotation.w - first.transform.rotation.w)
            )
            if dx > 0.02 or dy > 0.02 or dz > 0.02 or dq > 0.02:
                first = cur
                t0 = rospy.Time.now()
            rospy.sleep(0.05)
        chosen = first
    except Exception as e:
        rospy.logwarn(u"[map_odom_freezer] stability check failed, using current TF: {}".format(e))
        chosen = buffer.lookup_transform(parent_frame, child_frame, rospy.Time(0), rospy.Duration(0.5))

    # Publish static transform
    static_broadcaster = tf2_ros.StaticTransformBroadcaster()
    static_ts = transform_to_static(chosen)
    static_broadcaster.sendTransform(static_ts)
    rospy.loginfo(u"[map_odom_freezer] published static TF {} -> {}".format(parent_frame, child_frame))

    # Give time for /tf_static to latch
    rospy.sleep(0.5)

    # Kill cartographer node to free resources
    if kill_node(carto_node):
        rospy.loginfo(u"[map_odom_freezer] successfully killed {}".format(carto_node))
    else:
        rospy.logwarn(u"[map_odom_freezer] failed to kill {} automatically".format(carto_node))

    rospy.loginfo(u"[map_odom_freezer] exiting; /tf_static should remain published")
    # Keep node alive so late subscribers also receive /tf_static
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
