#!/usr/bin/env python3

import math
import rospy
from geometry_msgs.msg import PoseStamped


def _quat_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    # yaw (z-axis rotation)
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def main():
    rospy.init_node("waypoint_capture_from_goal", anonymous=True)

    topic = rospy.get_param("~goal_topic", "/move_base_simple/goal")
    name = rospy.get_param("~name", "place")

    msg: PoseStamped = rospy.wait_for_message(topic, PoseStamped, timeout=None)

    x = msg.pose.position.x
    y = msg.pose.position.y
    q = msg.pose.orientation
    yaw = _quat_to_yaw(q.x, q.y, q.z, q.w)

    # Print YAML snippet you can paste under places:
    out = f"  {name}: {{ x: {x:.3f}, y: {y:.3f}, yaw: {yaw:.3f} }}"
    rospy.loginfo("Captured goal in frame '%s'", msg.header.frame_id)
    print(out)


if __name__ == "__main__":
    main()
