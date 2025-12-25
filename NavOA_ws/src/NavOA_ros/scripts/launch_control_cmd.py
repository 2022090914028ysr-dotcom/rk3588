#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


def main():
    rospy.init_node('navoa_launch_control', anonymous=True)

    cmd_pub = rospy.Publisher('/external_cmd_vel', Twist, queue_size=10)
    enable_pub = rospy.Publisher('/external_control_enable', Bool, queue_size=10)

    forward = rospy.get_param('~forward', False)
    left = rospy.get_param('~left', False)
    right = rospy.get_param('~right', False)
    backward = rospy.get_param('~backward', False)

    linear_speed = rospy.get_param('~linear_speed', 0.5)
    angular_speed = rospy.get_param('~angular_speed', 0.5)
    duration = rospy.get_param('~duration', 1.0)
    enable_external = rospy.get_param('~enable_external_control', True)

    true_count = sum([1 if forward else 0,
                      1 if left else 0,
                      1 if right else 0,
                      1 if backward else 0])
    if true_count != 1:
        rospy.logerr('Exactly one of forward/left/right/backward must be true; got %d', true_count)
        return

    if enable_external:
        enable_pub.publish(Bool(data=True))

    twist = Twist()
    action = ''
    if forward:
        twist.linear.x = linear_speed
        action = 'forward'
    elif backward:
        twist.linear.x = -linear_speed
        action = 'backward'
    elif left:
        twist.angular.z = angular_speed
        action = 'left'
    elif right:
        twist.angular.z = -angular_speed
        action = 'right'

    rate = rospy.Rate(10)
    end_time = rospy.Time.now() + rospy.Duration(duration)
    rospy.loginfo('Sending %s for %.2fs (v=%.2f, w=%.2f)', action, duration, twist.linear.x, twist.angular.z)
    while not rospy.is_shutdown() and rospy.Time.now() < end_time:
        cmd_pub.publish(twist)
        rate.sleep()

    stop_twist = Twist()
    cmd_pub.publish(stop_twist)

    if enable_external:
        enable_pub.publish(Bool(data=False))

    rospy.loginfo('Command %s completed; robot stopped', action)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
