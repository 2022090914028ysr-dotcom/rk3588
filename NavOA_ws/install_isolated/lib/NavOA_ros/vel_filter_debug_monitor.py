#!/usr/bin/env python3

import rospy
from std_msgs.msg import String


class VelFilterDebugMonitor:
    def __init__(self):
        rospy.init_node('vel_filter_debug_monitor', anonymous=True)
        debug_topic = rospy.get_param('~debug_topic', '/vel_filter/debug')
        rospy.Subscriber(debug_topic, String, self._callback, queue_size=10)
        rospy.loginfo('vel_filter_debug_monitor listening on %s', debug_topic)

    def _callback(self, msg: String):
        rospy.loginfo_throttle(0.5, '[vel_filter_debug] %s', msg.data)


if __name__ == '__main__':
    try:
        VelFilterDebugMonitor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass