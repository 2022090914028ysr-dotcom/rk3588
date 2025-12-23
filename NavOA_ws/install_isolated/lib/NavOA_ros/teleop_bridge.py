#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import sys
import os
import time
import threading
sys.path.append(os.path.join(os.path.dirname(__file__), 'real_robot_cmd'))

from real_robot_cmd.robot_control import RobotController

class TeleopBridge:
    def __init__(self):
        rospy.init_node('teleop_bridge', anonymous=True)
        
        # Create robot controller
        self.robot_controller = RobotController()
        
        # # Initialize robot
        # if not self.robot_controller.robot_init():
        #     rospy.logerr("Failed to initialize robot controller")
        #     return
            
        # Command timeout handling
        self.last_command_time = time.time()
        self.timeout_duration = 0.5  # 500ms timeout
        self.timeout_active = False
        
        # Start timeout monitoring thread
        self.timeout_thread = threading.Thread(target=self.timeout_monitor, daemon=True)
        self.timeout_thread.start()
        
        # Subscribe to cmd_vel topic
        self.cmd_vel_sub = rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        
        rospy.on_shutdown(self.shutdown_hook)
        rospy.loginfo("Teleop bridge initialized")
        
    def cmd_vel_callback(self, msg):
        # Update last command time
        self.last_command_time = time.time()
        
        # Convert Twist message to robot velocity commands
        linear_velocity = msg.linear.x
        angular_velocity = msg.angular.z
        
        # Reset timeout flag when we receive a command
        self.timeout_active = False
        
        # Send to robot
        success = self.robot_controller.update_robot_velocity(linear_velocity, angular_velocity)
        if not success:
            rospy.logwarn("Failed to send velocity command to robot")
            
    def timeout_monitor(self):
        """Monitor for command timeouts and send zero velocities if needed"""
        while not rospy.is_shutdown():
            # Check if we haven't received a command for longer than timeout duration
            if time.time() - self.last_command_time > self.timeout_duration and not self.timeout_active:
                rospy.loginfo("Command timeout detected, sending zero velocities")
                self.timeout_active = True
                success = self.robot_controller.update_robot_velocity(0, 0)
                if not success:
                    rospy.logwarn("Failed to send timeout zero velocity command to robot")
            
            time.sleep(0.1)  # Check every 100ms
            
    def shutdown_hook(self):
        # Stop robot and close connection
        self.robot_controller.update_robot_velocity(0, 0)
        self.robot_controller.robot_close()
        rospy.loginfo("Teleop bridge shutdown")

if __name__ == '__main__':
    try:
        bridge = TeleopBridge()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        print(f"Error: {e}")