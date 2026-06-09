/**
 * @file      depth_to_scancamera.h
 * @brief     Depth image to 2D laser scan converter node.
 *
 * Takes a depth image from a depth camera, extracts the minimum depth
 * value per column from a horizontal band of rows, and publishes
 * the result as a sensor_msgs/LaserScan message simulating a 2D lidar.
 *
 * @author    Copilot
 * @date      2026/06/03
 * @version   1.0
 */

#pragma once

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/LaserScan.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>

#include <string>
#include <mutex>

namespace depth_to_scancamera
{

/**
 * @brief Depth-to-LaserScan conversion node.
 *
 * Subscribes to depth images and camera intrinsics, extracts
 * the minimum depth per column over a configurable row band,
 * and publishes a LaserScan message.
 */
class DepthToScanCamera
{
public:
    DepthToScanCamera(ros::NodeHandle& nh, ros::NodeHandle& nh_priv);
    ~DepthToScanCamera() = default;

private:
    // ©¤©¤ Callbacks ©¤©¤
    void depthCallback(const sensor_msgs::ImageConstPtr& msg);
    void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg);

    // ©¤©¤ Core conversion ©¤©¤
    void convertAndPublish(const sensor_msgs::ImageConstPtr& depth_msg);
    void computeScanAngles();

    // ©¤©¤ ROS interface ©¤©¤
    ros::NodeHandle& nh_;
    ros::NodeHandle& nh_priv_;

    image_transport::Subscriber depth_sub_;
    ros::Subscriber           camera_info_sub_;
    ros::Publisher            scan_pub_;

    // ©¤©¤ Camera model ©¤©¤
    image_geometry::PinholeCameraModel cam_model_;
    bool cam_model_ready_;

    // ©¤©¤ Precomputed data ©¤©¤
    std::vector<double> scan_angle_cos_;
    std::vector<double> scan_angle_sin_;
    int scan_width_;

    // ©¤©¤ Configurable parameters ©¤©¤
    std::string output_frame_id_;
    int         scan_height_;
    int         scan_row_start_;
    double      range_min_;
    double      range_max_;
    double      fov_override_;

    // ©¤©¤ Thread safety ©¤©¤
    std::mutex mutex_;
};

} // namespace depth_to_scancamera
