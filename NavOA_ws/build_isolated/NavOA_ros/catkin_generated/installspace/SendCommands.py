#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import sys
import os
from typing import Optional

def _add_real_robot_cmd_to_syspath() -> Optional[str]:
    """Ensure we can import modules from scripts/real_robot_cmd.

    When launched via roslaunch after installation, __file__ points to
    devel/lib/... and not the source scripts directory.
    """
    # 1) Try to resolve via rospkg (works in a ROS environment)
    try:
        import rospkg  # type: ignore

        pkg_path = rospkg.RosPack().get_path('NavOA_ros')
        scripts_dir = os.path.join(pkg_path, 'scripts')
        if os.path.isdir(scripts_dir) and scripts_dir not in sys.path:
            sys.path.insert(0, scripts_dir)
            return scripts_dir
    except Exception:
        pass

    # 2) Fallback: relative to this file location (works when running from source)
    scripts_dir = os.path.dirname(os.path.abspath(__file__))
    if os.path.basename(scripts_dir) == 'real_robot_cmd':
        scripts_dir = os.path.dirname(scripts_dir)
    if os.path.isdir(scripts_dir) and scripts_dir not in sys.path:
        sys.path.insert(0, scripts_dir)
        return scripts_dir
    return None


_add_real_robot_cmd_to_syspath()

from real_robot_cmd.robot_control import RobotController
from real_robot_cmd.motor_config import config as motor_comm_config

class RobotCommandInterface:
    def __init__(self):
        rospy.init_node('robot_command_interface', anonymous=True)

        # Optional serial overrides (used by MotorController via motor_config.config)
        serial_port = rospy.get_param('~serial_port', None)
        baudrate = rospy.get_param('~baudrate', None)
        timeout = rospy.get_param('~serial_timeout', None)
        if serial_port:
            motor_comm_config.port = str(serial_port)
        if baudrate:
            motor_comm_config.baudrate = int(baudrate)
        if timeout is not None:
            motor_comm_config.timeout = float(timeout)
        
        self.robot_controller = RobotController.get_instance()
        
        if not self.robot_controller.robot_init():
            rospy.logerr("Failed to initialize robot controller")
            raise RuntimeError("Robot controller init failed")
            
        self.cmd_vel_topic = rospy.get_param('~cmd_vel_topic', '/cmd_vel')
        self.cmd_vel_sub = rospy.Subscriber(self.cmd_vel_topic, Twist, self.cmd_vel_callback, queue_size=10)
        rospy.on_shutdown(self.shutdown_hook)
        rospy.loginfo("Robot command interface is ready and listening to %s", self.cmd_vel_topic)

    def cmd_vel_callback(self, msg):
        linear_x = msg.linear.x
        angular_z = msg.angular.z    
        try:
            success = self.robot_controller.update_robot_velocity(linear_x, angular_z)
            if success:
                rospy.loginfo_throttle(1.0, "Forwarded cmd_vel to robot")
            else:
                rospy.logwarn_throttle(1.0, "Failed to forward cmd_vel to robot")
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