#!/usr/bin/python3
"""
Helper: add region points for autonomous exploration from the command line.

Usage:
  rosrun marl_mapping add_region_point.py 5.0 10.0
  rosrun marl_mapping add_region_point.py 5.0 10.0 --frame map

Publishes a PointStamped to /clicked_point so the exploration node
picks it up and adds it to the region polygon.
"""

import sys
import rospy
from geometry_msgs.msg import PointStamped


def main():
    rospy.init_node('add_region_point', anonymous=True)

    frame = 'map'
    args = [a for a in sys.argv[1:] if not a.startswith('--')]
    if '--frame' in sys.argv:
        idx = sys.argv.index('--frame')
        if idx + 1 < len(sys.argv):
            frame = sys.argv[idx + 1]

    if len(args) < 2:
        print("Usage: add_region_point.py <x> <y> [--frame <frame_id>]")
        print("Example: add_region_point.py 5.0 10.0 --frame map")
        sys.exit(1)

    try:
        x = float(args[0])
        y = float(args[1])
    except ValueError:
        print("Error: x and y must be numbers")
        sys.exit(1)

    pub = rospy.Publisher('/clicked_point', PointStamped, queue_size=1, latch=True)
    rospy.sleep(0.5)  # wait for publisher to register

    ps = PointStamped()
    ps.header.frame_id = frame
    ps.header.stamp = rospy.Time.now()
    ps.point.x = x
    ps.point.y = y
    ps.point.z = 0.0

    pub.publish(ps)
    rospy.loginfo("Region point sent: frame=%s, (%.2f, %.2f)", frame, x, y)
    rospy.sleep(0.3)  # give ROS time to deliver


if __name__ == '__main__':
    main()
