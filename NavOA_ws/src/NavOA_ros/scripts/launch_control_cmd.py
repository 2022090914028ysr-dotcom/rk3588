#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


def main():
    rospy.init_node('navoa_launch_control', anonymous=True)

    # Publish to /cmd_vel for the bridge to detect linear.z signal
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    # Keep external cmd publisher for movement commands
    external_cmd_pub = rospy.Publisher('/external_cmd_vel', Twist, queue_size=10)
    enable_pub = rospy.Publisher('/external_control_enable', Bool, queue_size=10)

    forward = rospy.get_param('~forward', False)
    left = rospy.get_param('~left', False)
    right = rospy.get_param('~right', False)
    backward = rospy.get_param('~backward', False)
    stand = rospy.get_param('~stand', False)
    walk = rospy.get_param('~walk', False)  # New parameter for walking

    linear_speed = rospy.get_param('~linear_speed', 0.5)
    angular_speed = rospy.get_param('~angular_speed', 0.5)
    duration = rospy.get_param('~duration', 1.0)
    enable_external = rospy.get_param('~enable_external_control', True)

    # Check which command is active (including stand and walk)
    true_count = sum([1 if forward else 0,
                      1 if left else 0,
                      1 if right else 0,
                      1 if backward else 0,
                      1 if stand else 0,
                      1 if walk else 0])
    
    if true_count != 1:
        rospy.logerr('Exactly one of forward/left/right/backward/stand/walk must be true; got %d', true_count)
        return

    # Handle STAND command separately
    if stand:
        action = 'stand'
        # Create a twist message with linear.z = 1.0 to trigger button 6
        stand_twist = Twist()
        stand_twist.linear.z = 1.0  # This will trigger your bridge code
        
        # Publish the stand signal for the specified duration
        rospy.loginfo('Sending stand command (linear.z = 1.0) for %.2f seconds', duration)
        
        rate = rospy.Rate(10)  # 10 Hz
        end_time = rospy.Time.now() + rospy.Duration(duration)
        
        while not rospy.is_shutdown() and rospy.Time.now() < end_time:
            cmd_vel_pub.publish(stand_twist)
            rate.sleep()
        
        rospy.loginfo('Stand command completed after %.2f seconds', duration)
        return  # Exit after sending stand command
    
    # if walk:
    #     action = 'walk'
    #     # Create a twist message with linear.z = 0.0 to trigger walking
    #     walk_twist = Twist()
    #     walk_twist.linear.z = 0.0  # This should trigger walking mode
        
    #     # Publish the walk signal for the specified duration
    #     rospy.loginfo('Sending walk command (linear.z = 0.0) for %.2f seconds', duration)
        
    #     rate = rospy.Rate(10)  # 10 Hz
    #     end_time = rospy.Time.now() + rospy.Duration(duration)
        
    #     while not rospy.is_shutdown() and rospy.Time.now() < end_time:
    #         cmd_vel_pub.publish(walk_twist)
    #         rate.sleep()
        
    #     rospy.loginfo('Walk command completed after %.2f seconds', duration)
    #     return
    
    # Handle movement commands (forward/backward/left/right)
    if enable_external:
        enable_pub.publish(Bool(data=True))
        rospy.sleep(0.1)

    twist = Twist()
    
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

    # Publish movement commands
    rate = rospy.Rate(10)
    end_time = rospy.Time.now() + rospy.Duration(duration)
    rospy.loginfo('Sending %s for %.2fs (v=%.2f, w=%.2f)', 
                  action, duration, twist.linear.x, twist.angular.z)
    
    while not rospy.is_shutdown() and rospy.Time.now() < end_time:
        external_cmd_pub.publish(twist)
        rate.sleep()

    # Stop the robot after movement duration
    stop_twist = Twist()
    external_cmd_pub.publish(stop_twist)
    rospy.sleep(0.1)

    if enable_external:
        enable_pub.publish(Bool(data=False))

    rospy.loginfo('Movement command %s completed', action)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass