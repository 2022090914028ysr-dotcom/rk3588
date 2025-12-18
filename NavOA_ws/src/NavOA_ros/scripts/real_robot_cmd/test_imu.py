#!/usr/bin/env python3
"""
Pure Real-time IMU Data Display
Shows raw IMU data without any calibration
"""

import rospy
import math
import time
from sensor_msgs.msg import Imu


class PureIMUMonitor:
    def __init__(self):
        rospy.init_node('pure_imu_monitor', anonymous=True)
        
        self.update_interval = 0.5  # seconds
        self.last_update = time.time()
        self.sample_count = 0
        
        print("\033[2J\033[H")  # Clear screen
        self.display_header()
        
        # Subscribe to IMU topic
        rospy.Subscriber('/imu', Imu, self.callback, queue_size=1)
    
    def display_header(self):
        """Display header info"""
        print("=" * 70)
        print("IMU Real-time Data Display (Raw, No Calibration)")
        print("=" * 70)
        print("Topic: /imu")
        print("Move the robot to observe changes")
        print("Press Ctrl+C to stop")
        print("=" * 70)
    
    def callback(self, msg):
        """IMU data callback"""
        current_time = time.time()
        dt = current_time - self.last_update
        if dt < self.update_interval:
            return
        
        self.last_update = current_time
        self.sample_count += 1
        
        # Clear screen and display new data
        print("\033[2J\033[H")
        print("=" * 70)
        print(f"IMU Real-time Data - Sample #{self.sample_count}")
        print("=" * 70)
        
        # Extract quaternion
        qx = msg.orientation.x
        qy = msg.orientation.y
        qz = msg.orientation.z
        qw = msg.orientation.w
        
        # Calculate Euler angles (roll, pitch, yaw) in radians
        # Yaw
        yaw = math.atan2(2.0 * (qw * qz + qx * qy),
                         1.0 - 2.0 * (qy * qy + qz * qz))
        # Pitch
        sinp = 2.0 * (qw * qy - qz * qx)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)
        # Roll
        roll = math.atan2(2.0 * (qw * qx + qy * qz),
                          1.0 - 2.0 * (qx * qx + qy * qy))
        
        # Convert to degrees
        roll_deg = math.degrees(roll)
        pitch_deg = math.degrees(pitch)
        yaw_deg = math.degrees(yaw)
        # Normalize yaw to [0, 360)
        yaw_deg = yaw_deg % 360
        if yaw_deg < 0:
            yaw_deg += 360

        # Display orientation
        print("\n[Orientation (Euler Angles)]")
        print(f"Roll:   {roll_deg:8.2f} deg")
        print(f"Pitch:  {pitch_deg:8.2f} deg")
        print(f"Yaw:    {yaw_deg:8.2f} deg")
        
        # Display linear acceleration (IMU body frame)
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z
        print("\n[Linear Acceleration (IMU frame)]")
        print(f"X: {ax:8.3f} m/s^2")
        print(f"Y: {ay:8.3f} m/s^2")
        print(f"Z: {az:8.3f} m/s^2")

        # Define robot-forward as IMU +X
        a_forward = ax

        # Estimate velocity change in the IMU body frame over this interval
        vx = ax * dt
        vy = ay * dt
        v_forward = vx
        # print("\n[Estimated Velocity Change (since last update)]")
        # print(f"Vx (IMU +X / forward): {vx:8.3f} m/s")
        # print(f"Vy (IMU +Y):            {vy:8.3f} m/s")
        # print(f"V_forward:              {v_forward:8.3f} m/s  (robot forward)")
        # print(f"a_forward:              {a_forward:8.3f} m/s^2 (robot forward)")
        # print(f"dt: {dt:8.3f} s")
        
        # Display angular velocity
        print("\n[Angular Velocity]")
        print(f"X: {msg.angular_velocity.x:8.3f} rad/s")
        print(f"Y: {msg.angular_velocity.y:8.3f} rad/s")
        print(f"Z: {msg.angular_velocity.z:8.3f} rad/s")
        
        # Gravity analysis
        gravity = 9.81
        total_acc = math.sqrt(
            msg.linear_acceleration.x**2 +
            msg.linear_acceleration.y**2 +
            msg.linear_acceleration.z**2
        )
        print(f"\n[Gravity Analysis]")
        print(f"Total Acc: {total_acc:8.3f} m/s^2")
        print(f"Diff from g: {abs(total_acc - gravity):8.3f} m/s^2")
        
        print("=" * 70)
        print("Move the robot to observe changes | Press Ctrl+C to stop")


def main():
    try:
        monitor = PureIMUMonitor()
        rospy.spin()
    except KeyboardInterrupt:
        print("\n\nMonitoring stopped.")
    except Exception as e:
        print(f"Error: {e}")


if __name__ == '__main__':
    main()