#!/usr/bin/env python3

import rospy
import csv
import os
from datetime import datetime
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState

class RobotDataLogger:
    def __init__(self):
        rospy.init_node('robot_data_logger', anonymous=True)
        
        # Create logs directory if it doesn't exist
        self.log_dir = os.path.join(os.path.dirname(__file__), '..', 'logs')
        if not os.path.exists(self.log_dir):
            os.makedirs(self.log_dir)
        
        # Create timestamp for filenames
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        # Open CSV files for writing
        self.odom_file = open(os.path.join(self.log_dir, f'odometry_{timestamp}.csv'), 'w')
        self.cmd_vel_file = open(os.path.join(self.log_dir, f'cmd_velocity_{timestamp}.csv'), 'w')
        self.joint_file = open(os.path.join(self.log_dir, f'joint_states_{timestamp}.csv'), 'w')
        
        # Create CSV writers
        self.odom_writer = csv.writer(self.odom_file)
        self.cmd_vel_writer = csv.writer(self.cmd_vel_file)
        self.joint_writer = csv.writer(self.joint_file)
        
        # Write headers
        self.odom_writer.writerow(['timestamp', 'position_x', 'position_y', 'position_z', 
                                  'orientation_x', 'orientation_y', 'orientation_z', 'orientation_w',
                                  'linear_velocity_x', 'linear_velocity_y', 'angular_velocity_z'])
        
        self.cmd_vel_writer.writerow(['timestamp', 'linear_velocity_x', 'linear_velocity_y', 
                                    'linear_velocity_z', 'angular_velocity_x', 'angular_velocity_y', 
                                    'angular_velocity_z'])
        
        self.joint_writer.writerow(['timestamp', 'joint_name', 'position', 'velocity', 'effort'])
        
        # Subscribe to topics
        rospy.Subscriber('/limo/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        rospy.Subscriber('/joint_states', JointState, self.joint_state_callback)
        
        rospy.on_shutdown(self.shutdown_hook)
        
        rospy.loginfo("Data logger started. Logging to: {}".format(self.log_dir))
    
    def odom_callback(self, msg):
        """Callback for odometry data"""
        timestamp = msg.header.stamp.to_sec()
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        lin_vel = msg.twist.twist.linear
        ang_vel = msg.twist.twist.angular
        
        self.odom_writer.writerow([
            timestamp,
            pos.x, pos.y, pos.z,
            ori.x, ori.y, ori.z, ori.w,
            lin_vel.x, lin_vel.y, ang_vel.z
        ])
        
        # Flush to ensure data is written
        self.odom_file.flush()
    
    def cmd_vel_callback(self, msg):
        """Callback for command velocity data"""
        timestamp = rospy.Time.now().to_sec()
        
        self.cmd_vel_writer.writerow([
            timestamp,
            msg.linear.x, msg.linear.y, msg.linear.z,
            msg.angular.x, msg.angular.y, msg.angular.z
        ])
        
        # Flush to ensure data is written
        self.cmd_vel_file.flush()
    
    def joint_state_callback(self, msg):
        """Callback for joint state data"""
        timestamp = msg.header.stamp.to_sec()
        
        # Write each joint's data
        for i in range(len(msg.name)):
            self.joint_writer.writerow([
                timestamp,
                msg.name[i],
                msg.position[i] if i < len(msg.position) else 0,
                msg.velocity[i] if i < len(msg.velocity) else 0,
                msg.effort[i] if i < len(msg.effort) else 0
            ])
        
        # Flush to ensure data is written
        self.joint_file.flush()
    
    def shutdown_hook(self):
        """Close all files when shutting down"""
        rospy.loginfo("Shutting down data logger...")
        self.odom_file.close()
        self.cmd_vel_file.close()
        self.joint_file.close()
    
    def run(self):
        """Main loop"""
        rospy.spin()

if __name__ == '__main__':
    try:
        logger = RobotDataLogger()
        logger.run()
    except rospy.ROSInterruptException:
        pass