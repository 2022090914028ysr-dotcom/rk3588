#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import sys
import os

# Add current directory to path (optional if in ROS package)
sys.path.append(os.path.dirname(__file__))

from robot_control import RobotController

class RobotCommandInterface:
    def __init__(self):
        rospy.init_node('robot_command_interface', anonymous=True)
        
        self.robot_controller = RobotController.get_instance()
        
        if not self.robot_controller.robot_init():
            rospy.logerr("Failed to initialize robot controller")
            raise RuntimeError("Robot controller init failed")
            
        self.cmd_vel_sub = rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback, queue_size=10)
        rospy.on_shutdown(self.shutdown_hook)
        rospy.loginfo("Robot command interface is ready and listening to /cmd_vel")

    def cmd_vel_callback(self, msg):
        linear_x = msg.linear.x
        angular_z = msg.angular.z    
        try:
            success = self.robot_controller.update_robot_velocity(linear_x, angular_z)
            if success:
                rospy.loginfo("Successfully sent command to robot")
            else:
                rospy.logwarn("Failed to send command to robot")
        except Exception as e:
            rospy.logerr("Error sending command to robot: %s", str(e))
            
    def shutdown_hook(self):
        rospy.loginfo("Shutting down robot command interface...")
        try:
            self.robot_controller.robot_close()
            rospy.loginfo("Robot command interface shut down successfully")
        except Exception as e:
            rospy.logerr("Error during shutdown: %s", str(e))

def main():
    try:
        controller = RobotCommandInterface()
        rospy.loginfo("Robot command interface is running. Waiting for cmd_vel messages...")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr("Error in robot command interface: %s", str(e))
        sys.exit(1)

if __name__ == "__main__":
    main()