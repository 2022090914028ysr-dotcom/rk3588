/**
 * @file      depth_to_scancamera_node.cpp
 * @brief     Depth-to-2D-LaserScan node entry point.
 *
 * Usage:
 *   rosrun depth_to_scancamera depth_to_scancamera_node
 *
 * Subscribed topics:
 *   depth0/image_raw   (sensor_msgs/Image)      - depth image
 *   depth0/camera_info (sensor_msgs/CameraInfo) - camera intrinsics
 *
 * Published topics:
 *   scan_camera (sensor_msgs/LaserScan) - simulated 2D laser scan
 *
 * Parameters (private namespace ~):
 *   ~output_frame_id   - frame_id for output LaserScan (default: "camera_link")
 *   ~scan_height        - number of pixel rows for scan extraction (default: 10)
 *   ~scan_row_start     - starting row, -1 for auto-center (default: -1)
 *   ~range_min          - minimum valid range [m] (default: 0.15)
 *   ~range_max          - maximum valid range [m] (default: 10.0)
 *   ~fov_override_deg   - manual FOV override [deg], 0=use camera (default: 0)
 */

#include <ros/ros.h>
#include "depth_to_scancamera/depth_to_scancamera.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "depth_to_scancamera_node");

    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");

    depth_to_scancamera::DepthToScanCamera converter(nh, nh_priv);

    ros::spin();

    return 0;
}
