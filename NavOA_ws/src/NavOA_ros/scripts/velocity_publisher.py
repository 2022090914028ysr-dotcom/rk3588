#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import sys
import time

class VelocityPublisher:
    def __init__(self):
        rospy.init_node('velocity_publisher', anonymous=True)
        
        # Create publisher
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        # Wait for connection
        rospy.sleep(1.0)
        
        rospy.loginfo("Velocity publisher node started")
    
    def send_velocity_command(self, linear_x=0.0, angular_z=0.0, duration=1.0):
        """Send a velocity command for a specified duration"""
        # Create twist message
        twist = Twist()
        twist.linear.x = linear_x
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = angular_z
        
        rospy.loginfo("Sending velocity command: linear_x=%.2f, angular_z=%.2f for %.1f seconds", 
                      linear_x, angular_z, duration)
        
        # Send command for specified duration
        start_time = time.time()
        rate = rospy.Rate(100)  # 100 Hz
        while (time.time() - start_time) < duration and not rospy.is_shutdown():
            self.cmd_vel_pub.publish(twist)
            rate.sleep()
        
        # Stop the robot
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        
        rospy.loginfo("Command sent and robot stopped")
    
    def run_square_pattern(self):
        """Run a square pattern demonstration"""
        rospy.loginfo("Running square pattern demonstration")
        
        # Move forward
        self.send_velocity_command(0.5, 0.0, 2.0)
        rospy.sleep(1.0)
        
        # Turn 90 degrees
        self.send_velocity_command(0.0, 1.0, 1.57)  # Approximately 90 degrees
        rospy.sleep(1.0)
        
        # Move forward
        self.send_velocity_command(0.5, 0.0, 2.0)
        rospy.sleep(1.0)
        
        # Turn 90 degrees
        self.send_velocity_command(0.0, 1.0, 1.57)
        rospy.sleep(1.0)
        
        # Move forward
        self.send_velocity_command(0.5, 0.0, 2.0)
        rospy.sleep(1.0)
        
        # Turn 90 degrees
        self.send_velocity_command(0.0, 1.0, 1.57)
        rospy.sleep(1.0)
        
        # Move forward
        self.send_velocity_command(0.5, 0.0, 2.0)
        rospy.sleep(1.0)
        
        # Turn 90 degrees to complete square
        self.send_velocity_command(0.0, 1.0, 1.57)
        rospy.sleep(1.0)
        
        rospy.loginfo("Square pattern completed")

if __name__ == '__main__':
    try:
        publisher = VelocityPublisher()
        
        if len(sys.argv) > 1 and sys.argv[1] == "square":
            # Run square pattern demo
            publisher.run_square_pattern()
        elif len(sys.argv) >= 3:
            # Send single command from command line arguments
            linear_x = float(sys.argv[1])
            angular_z = float(sys.argv[2])
            duration = float(sys.argv[3]) if len(sys.argv) > 3 else 1.0
            publisher.send_velocity_command(linear_x, angular_z, duration)
        else:
            print("Usage:")
            print("  Single command: rosrun NavOA_ros velocity_publisher.py <linear_x> <angular_z> [duration]")
            print("  Square pattern: rosrun NavOA_ros velocity_publisher.py square")
            print("Examples:")
            print("  rosrun NavOA_ros velocity_publisher.py 0.5 0.0 2.0")
            print("  rosrun NavOA_ros velocity_publisher.py square")
            sys.exit(1)
            
    except rospy.ROSInterruptException:
        pass
    except ValueError:
        print("Error: Please provide valid numeric values")
        sys.exit(1)