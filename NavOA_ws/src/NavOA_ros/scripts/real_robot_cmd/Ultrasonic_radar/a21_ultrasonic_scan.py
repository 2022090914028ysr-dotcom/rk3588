#!/usr/bin/env python3

import rospy
import time
import serial
import struct
import binascii
import numpy as np
from sensor_msgs.msg import LaserScan

class UltrasonicRadar:
    def __init__(self):
        rospy.init_node('ultrasonic_scan', anonymous=True)
        
        # Get parameters
        self.port = rospy.get_param('~port', '/dev/ttyS6')
        self.baud = rospy.get_param('~baud', 115200)
        self.period = rospy.get_param('~period', 0.1)
        self.frame_id = rospy.get_param('~frame_id', 'ultrasonic_frame')
        
        # Scan parameters - 30 degree range
        self.scan_angle = np.radians(30)  # 30 degrees in radians
        self.num_points = 31  # 31 points in 30 degree range
        self.angle_increment = self.scan_angle / (self.num_points - 1)
        
        # Noise parameters
        self.noise_std = 0.02  # 2cm standard deviation noise
        self.min_valid_distance = 0  # minimum valid distance
        self.max_valid_distance = 5.0   # maximum valid distance
        
        # History data for filtering
        self.distance_history = []
        self.history_size = 5  # sliding window size
        
        # Initialize serial
        self.serial_port = None
        self.init_serial()
        
        # Create publisher for LaserScan
        rospy.loginfo("Creating publisher for /Ultrasound_scan with 30-degree scan")
        self.scan_pub = rospy.Publisher('/Ultrasound_scan', LaserScan, queue_size=10)
        
        # Wait for publisher to be ready
        time.sleep(0.5)
        
        rospy.loginfo("Publisher ready. Starting timer...")
        
        # Timer
        self.publish_count = 0
        rospy.Timer(rospy.Duration(self.period), self.publish_callback)
        
        rospy.loginfo("=== Ultrasonic Node Started ===")
        rospy.loginfo("Scan angle: %.1f deg, Points: %d, Rate: %.1f Hz", 
                     np.degrees(self.scan_angle), self.num_points, 1.0/self.period)

    def init_serial(self):
        """Initialize serial connection"""
        try:
            self.serial_port = serial.Serial(
                port=self.port,
                baudrate=self.baud,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.2
            )
            
            if self.serial_port.is_open:
                rospy.loginfo("Serial port %s opened at %d baud", self.port, self.baud)
                # Clear buffers
                self.serial_port.reset_input_buffer()
                self.serial_port.reset_output_buffer()
                
        except Exception as e:
            rospy.logerr("Failed to open serial port: %s", str(e))
            rospy.loginfo("Will continue with simulated data")
            self.serial_port = None

    def read_sensor_data(self):
        """Read raw distance from sensor"""
        if not self.serial_port:
            # Simulated data for testing
            simulated_dist = 1.0 + (self.publish_count % 10) * 0.1
            rospy.logdebug("Using simulated distance: %.3f m", simulated_dist)
            return simulated_dist
        
        try:
            # Send read command
            cmd = b'R'  # Most common command
            self.serial_port.write(cmd)
            time.sleep(0.05)
            
            # Read response
            response = self.serial_port.read(10)
            
            if response:
                rospy.logdebug("Raw response: %s", binascii.hexlify(response))
                
                # Parse based on response length
                if len(response) >= 4:
                    # Try to parse as distance
                    # For response like: ff00fdfc
                    
                    # Method 1: Take first 2 bytes as distance in mm (big endian)
                    try:
                        distance_mm = struct.unpack('>H', response[:2])[0]
                        distance_m = distance_mm / 1000.0
                        
                        # Validate
                        if self.min_valid_distance <= distance_m <= self.max_valid_distance:
                            rospy.logdebug("Sensor distance (BE): %.3f m", distance_m)
                            return distance_m
                    except:
                        pass
                    
                    # Method 2: Try little endian
                    try:
                        distance_mm = struct.unpack('<H', response[:2])[0]
                        distance_m = distance_mm / 1000.0
                        if self.min_valid_distance <= distance_m <= self.max_valid_distance:
                            rospy.logdebug("Sensor distance (LE): %.3f m", distance_m)
                            return distance_m
                    except:
                        pass
            
            rospy.logwarn("No valid sensor data")
            return None
            
        except Exception as e:
            rospy.logwarn("Sensor read error: %s", str(e))
            return None

    def moving_average_filter(self, new_distance):
        """Apply moving average filter to smooth distance readings"""
        if new_distance is None:
            if self.distance_history:
                # Use last valid distance if new reading is invalid
                return sum(self.distance_history) / len(self.distance_history)
            else:
                return self.max_valid_distance  # Default value
        
        self.distance_history.append(new_distance)
        if len(self.distance_history) > self.history_size:
            self.distance_history.pop(0)
        
        return sum(self.distance_history) / len(self.distance_history)

    def generate_scan_points(self, center_distance):
        """Generate 30-degree scan points based on center distance"""
        ranges = []
        intensities = []
        
        # Generate scan points in 30 degree range
        # Angle distribution: -15 degrees to +15 degrees
        half_angle = self.scan_angle / 2.0
        
        for i in range(self.num_points):
            # Calculate current angle (from -15 to +15 degrees)
            angle = -half_angle + i * self.angle_increment
            
            # Adjust distance based on angle (simulate ultrasonic beam characteristics)
            # For center point (0 degrees), use original distance
            # For edge points, distance slightly increases, simulating real ultrasonic attenuation
            angle_factor = 1.0 + 0.1 * abs(angle) / half_angle  # edge increases up to 10%
            adjusted_distance = center_distance * angle_factor
            
            # Add noise
            noisy_distance = adjusted_distance + np.random.normal(0, self.noise_std)
            
            # Clamp to valid range
            noisy_distance = max(self.min_valid_distance, 
                               min(noisy_distance, self.max_valid_distance))
            
            ranges.append(float(noisy_distance))
            intensities.append(1.0)  # intensity set to 1.0
            
        return ranges, intensities

    def publish_callback(self, event):
        """Timer callback to publish data"""
        try:
            # Read raw distance from sensor
            raw_distance = self.read_sensor_data()
            
            # Apply filter
            filtered_distance = self.moving_average_filter(raw_distance)
            
            # Generate 30-degree scan points
            ranges, intensities = self.generate_scan_points(filtered_distance)
            
            # Create LaserScan message
            scan_msg = LaserScan()
            
            # Header
            scan_msg.header.stamp = rospy.Time.now()
            scan_msg.header.frame_id = self.frame_id
            scan_msg.header.seq = self.publish_count
            
            # Scan parameters - 30 degree field of view
            scan_msg.angle_min = -self.scan_angle / 2.0  # -15 degrees
            scan_msg.angle_max = self.scan_angle / 2.0   # +15 degrees
            scan_msg.angle_increment = self.angle_increment
            scan_msg.time_increment = 0.0  # all points collected at same time
            scan_msg.scan_time = self.period
            scan_msg.range_min = self.min_valid_distance
            scan_msg.range_max = self.max_valid_distance
            
            # Data
            scan_msg.ranges = ranges
            scan_msg.intensities = intensities
            
            # Publish
            self.scan_pub.publish(scan_msg)
            
            # Log for debugging
            if self.publish_count % 20 == 0:  # Log every 20 messages
                rospy.loginfo("Published scan #%d: center=%.3f m, points=%d", 
                            self.publish_count, filtered_distance, len(ranges))
                rospy.loginfo("  Angle range: [%.1f deg, %.1f deg], increment: %.2f deg",
                            np.degrees(scan_msg.angle_min),
                            np.degrees(scan_msg.angle_max),
                            np.degrees(scan_msg.angle_increment))
            
            self.publish_count += 1
            
        except Exception as e:
            rospy.logerr("Publish error: %s", str(e))

    def run(self):
        """Main run loop"""
        rospy.loginfo("Node running. Publishing 30-degree scan to /scan")
        rospy.loginfo("Press Ctrl+C to stop.")
        rospy.spin()

def main():
    radar = UltrasonicRadar()
    radar.run()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("Node shutdown")
    except Exception as e:
        rospy.logerr("Fatal error: %s", str(e))

# Save this as a21_ultrasonic_scan.py and run