#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import sys
import os

# Add the real_robot_cmd directory to the Python path
sys.path.append(os.path.dirname(__file__))

from robot_control import RobotController

class RobotCommandInterface:
    def __init__(self):
        """Initialize the robot command interface"""
        rospy.init_node('robot_command_interface', anonymous=True)
        
        # Create RobotController instance
        self.robot_controller = RobotController()
        
        # Initialize the robot
        try:
            self.robot_controller.robot_init()
            rospy.loginfo("Robot controller initialized successfully")
        except Exception as e:
            rospy.logerr("Failed to initialize robot controller: %s", str(e))
            raise
            
        # Create subscriber for cmd_vel topic
        self.cmd_vel_sub = rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback, queue_size=10)
        
        # Register shutdown handler
        rospy.on_shutdown(self.shutdown_hook)
        
        rospy.loginfo("Robot command interface is ready and listening to /cmd_vel")
    
    def cmd_vel_callback(self, msg):
        """Callback function for cmd_vel messages"""
        # Extract linear and angular velocities
        linear_x = msg.linear.x  # Forward/backward velocity (m/s)
        angular_z = msg.angular.z  # Rotation velocity (rad/s)
        
        rospy.loginfo("Received cmd_vel: linear_x=%.3f, angular_z=%.3f", linear_x, angular_z)
        
        # Send command to robot
        try:
            success = self.robot_controller.update_robot_velocity(linear_x, angular_z)
            if success:
                rospy.loginfo("Successfully sent command to robot")
            else:
                rospy.logwarn("Failed to send command to robot")
        except Exception as e:
            rospy.logerr("Eriiiiiror sending command to robot: %s", str(e))
            
    def shutdown_hook(self):
        """Function called when ROS node is shutting down"""
        rospy.loginfo("Shutting down robot command interface...")
        try:
            # Close connection
            self.robot_controller.robot_close()
            rospy.loginfo("Robot command interface shut down successfully")
        except Exception as e:
            rospy.logerr("Error during shutdown: %s", str(e))

def main():
    """Main function"""
    try:
        # Create controller instance
        controller = RobotCommandInterface()
        
        rospy.loginfo("Robot command interface is running. Waiting for cmd_vel messages...")
        rospy.loginfo("Press Ctrl+C to stop")
        
        # Keep the node running
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr("Error in robot command interface: %s", str(e))
        sys.exit(1)

if __name__ == "__main__":
    main()