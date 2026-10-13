#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import rospkg
import sys
import os
import threading

# rosrun/roslaunch executes the generated devel-space wrapper, not this source file.
motor_scripts_path = os.path.join(
    rospkg.RosPack().get_path('NavOA_ros'), 'scripts', 'real_robot_cmd'
)
sys.path.insert(0, motor_scripts_path)

try:
    from robot_control import RobotController
    MOTOR_CONTROL_AVAILABLE = True
except ImportError as e:
    rospy.logerr("Failed to import RobotController: %s", str(e))
    MOTOR_CONTROL_AVAILABLE = False

class VelocitySubscriber:
    def __init__(self):
        if not MOTOR_CONTROL_AVAILABLE:
            rospy.logerr("Motor control module not available. Exiting.")
            raise RuntimeError("Motor control module not available")
            
        rospy.init_node('velocity_subscriber', anonymous=True)
        
        # Initialize robot controller
        self.robot_controller = RobotController()
        
        # Try to initialize the robot
        try:
            self.robot_controller.robot_init()
            rospy.loginfo("Robot controller initialized successfully")
        except Exception as e:
            rospy.logerr("Failed to initialize robot controller: %s", str(e))
            raise
            
        # Subscriber to cmd_vel topic
        self.cmd_vel_sub = rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback, queue_size=10)
        
        # Lock for thread safety
        self.lock = threading.Lock()
        
        rospy.on_shutdown(self.shutdown_hook)
        
        rospy.loginfo("Velocity subscriber node started")
    
    def cmd_vel_callback(self, msg):
        """Callback for velocity commands"""
        with self.lock:
            linear_x = msg.linear.x
            angular_z = msg.angular.z
            
            rospy.loginfo("Received velocity command: linear_x=%.3f, angular_z=%.3f", linear_x, angular_z)
            
            try:
                # Send command to robot controller
                success = self.robot_controller.update_robot_velocity(linear_x, angular_z)
                if success:
                    rospy.loginfo("Successfully sent velocity command to motors")
                else:
                    rospy.logwarn("Failed to send velocity command to motors")
            except Exception as e:
                rospy.logerr("Error sending velocity command to motors: %s", str(e))
    
    def shutdown_hook(self):
        """Clean shutdown"""
        rospy.loginfo("Shutting down velocity subscriber...")
        try:
            # Stop the robot
            if hasattr(self, 'robot_controller'):
                self.robot_controller.update_robot_velocity(0.0, 0.0)
                self.robot_controller.robot_close()
            rospy.loginfo("Robot controller closed successfully")
        except Exception as e:
            rospy.logerr("Error during shutdown: %s", str(e))
    
    def run(self):
        """Main loop"""
        rospy.spin()

if __name__ == '__main__':
    try:
        subscriber = VelocitySubscriber()
        subscriber.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr("Error in velocity subscriber: %s", str(e))
        sys.exit(1)