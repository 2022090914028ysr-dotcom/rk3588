import rospy
from sensor_msgs.msg import LaserScan
import math

def callback(scan):
    print(f"angle_min: {scan.angle_min:.4f}")
    print(f"angle_max: {scan.angle_max:.4f}")
    print(f"angle_increment: {scan.angle_increment:.4f}")
    print(f"range_min: {scan.range_min:.4f}")
    print(f"range_max: {scan.range_max:.4f}")
    print(f"Number of ranges: {len(scan.ranges)}")

    centers = [0, 1.57, -1.57, 3.14, -3.14]
    half_angle = 0.7
    
    print("\nCenter -> Min Range (within ±0.7 rad):")
    for c in centers:
        start_angle = c - half_angle
        end_angle = c + half_angle
        
        # Calculate indices
        idx_start = int((start_angle - scan.angle_min) / scan.angle_increment)
        idx_end = int((end_angle - scan.angle_min) / scan.angle_increment)
        
        # Wrap indices or clamp? Depends on whether it's 360.
        valid_ranges = []
        for i in range(len(scan.ranges)):
            angle = scan.angle_min + i * scan.angle_increment
            # Handle wrap around or simple check
            diff = math.atan2(math.sin(angle - c), math.cos(angle - c))
            if abs(diff) <= half_angle:
                val = scan.ranges[i]
                if scan.range_min < val < scan.range_max:
                    valid_ranges.append(val)
        
        min_r = min(valid_ranges) if valid_ranges else float('inf')
        print(f"{c:6.2f} -> {min_r:.4f}")

    rospy.signal_shutdown("Done")

rospy.init_node('scan_checker')
rospy.Subscriber('/scan', LaserScan, callback)
rospy.spin()
