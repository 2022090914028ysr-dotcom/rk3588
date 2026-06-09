/**
 * @file      depth_to_scancamera.cpp
 * @brief     Depth image to 2D laser scan converter implementation.
 */

#include "depth_to_scancamera/depth_to_scancamera.h"
#include <sensor_msgs/image_encodings.h>
#include <algorithm>
#include <cmath>
#include <limits>

namespace depth_to_scancamera
{

DepthToScanCamera::DepthToScanCamera(ros::NodeHandle& nh, ros::NodeHandle& nh_priv)
    : nh_(nh)
    , nh_priv_(nh_priv)
    , cam_model_ready_(false)
    , scan_width_(0)
{
    // ©¤©¤ Read parameters ©¤©¤
    nh_priv_.param<std::string>("output_frame_id", output_frame_id_, "camera_link");
    nh_priv_.param<int>("scan_height", scan_height_, 10);
    nh_priv_.param<double>("range_min", range_min_, 0.15);
    nh_priv_.param<double>("range_max", range_max_, 10.0);
    // fov_override_deg: 0 = use camera intrinsics
    double fov_deg;
    nh_priv_.param<double>("fov_override_deg", fov_deg, 0.0);
    fov_override_ = fov_deg * M_PI / 180.0;

    // scan_row_start: -1 = auto-center
    nh_priv_.param<int>("scan_row_start", scan_row_start_, -1);

    ROS_INFO("[depth_to_scancamera] Parameters:");
    ROS_INFO("  output_frame_id : %s", output_frame_id_.c_str());
    ROS_INFO("  scan_height     : %d", scan_height_);
    ROS_INFO("  range_min       : %.2f m", range_min_);
    ROS_INFO("  range_max       : %.2f m", range_max_);
    ROS_INFO("  fov_override    : %.2f deg (0=use camera)", fov_deg);

    // ©¤©¤ Subscribers & Publishers ©¤©¤
    image_transport::ImageTransport it(nh_);
    depth_sub_ = it.subscribe("depth0/image_raw", 1,
                              &DepthToScanCamera::depthCallback, this);

    camera_info_sub_ = nh_.subscribe("depth0/camera_info", 1,
                                     &DepthToScanCamera::cameraInfoCallback, this);

    scan_pub_ = nh_.advertise<sensor_msgs::LaserScan>("scan_camera", 10);

    ROS_INFO("[depth_to_scancamera] Node started, waiting for depth images...");
}

// ¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T
// CameraInfo callback
// ¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T
void DepthToScanCamera::cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(mutex_);

    if (!cam_model_.fromCameraInfo(msg))
    {
        ROS_ERROR_THROTTLE(5, "[depth_to_scancamera] Failed to initialize camera model");
        return;
    }

    cam_model_ready_ = true;
    computeScanAngles();

    ROS_INFO_ONCE("[depth_to_scancamera] Camera model initialized: %dx%d",
                  msg->width, msg->height);
}

// ¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T
// Precompute ray angles for each image column
// ¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T
void DepthToScanCamera::computeScanAngles()
{
    double cx = cam_model_.cx();
    double fx = cam_model_.fx();
    int width = cam_model_.fullResolution().width;

    scan_width_ = width;
    scan_angle_cos_.resize(width);
    scan_angle_sin_.resize(width);

    ROS_INFO("[depth_to_scancamera] Computing scan angles: width=%d, cx=%.2f, fx=%.2f",
             width, cx, fx);

    for (int col = 0; col < width; ++col)
    {
        double angle = std::atan2(static_cast<double>(col) - cx, fx);
        scan_angle_cos_[col] = std::cos(angle);
        scan_angle_sin_[col] = std::sin(angle);
    }
}

// ¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T
// Depth image callback
// ¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T
void DepthToScanCamera::depthCallback(const sensor_msgs::ImageConstPtr& msg)
{
    if (!cam_model_ready_)
    {
        ROS_WARN_THROTTLE(3, "[depth_to_scancamera] Waiting for camera_info...");
        return;
    }
    convertAndPublish(msg);
}

// ¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T
// Core conversion: depth image -> LaserScan
// ¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T¨T
void DepthToScanCamera::convertAndPublish(const sensor_msgs::ImageConstPtr& depth_msg)
{
    std::lock_guard<std::mutex> lock(mutex_);

    int img_width  = depth_msg->width;
    int img_height = depth_msg->height;

    // ©¤©¤ Determine scan row range ©¤©¤
    int row_start, row_end;
    if (scan_row_start_ >= 0)
    {
        row_start = scan_row_start_;
    }
    else
    {
        // auto-center
        row_start = (img_height - scan_height_) / 2;
    }
    row_start = std::max(0, std::min(row_start, img_height - 1));
    row_end   = std::min(row_start + scan_height_, img_height);

    // ©¤©¤ Prepare LaserScan message ©¤©¤
    sensor_msgs::LaserScan scan_msg;
    scan_msg.header.stamp    = depth_msg->header.stamp;
    scan_msg.header.frame_id = output_frame_id_;

    int effective_width = img_width;

    // Handle resolution mismatch: recalculate angles if needed
    if (static_cast<int>(scan_angle_cos_.size()) != img_width)
    {
        double cx = cam_model_.cx();
        double fx = cam_model_.fx();
        scan_width_ = effective_width;
        scan_angle_cos_.resize(effective_width);
        scan_angle_sin_.resize(effective_width);
        for (int col = 0; col < effective_width; ++col)
        {
            double angle = std::atan2(static_cast<double>(col) - cx, fx);
            scan_angle_cos_[col] = std::cos(angle);
            scan_angle_sin_[col] = std::sin(angle);
        }
    }

    // Determine angle range
    double angle_min, angle_max;
    if (fov_override_ > 0.0)
    {
        angle_min = -fov_override_ / 2.0;
        angle_max = +fov_override_ / 2.0;
    }
    else
    {
        double cx = cam_model_.cx();
        double fx = cam_model_.fx();
        angle_min = std::atan2(0.0 - cx, fx);
        angle_max = std::atan2(static_cast<double>(effective_width - 1) - cx, fx);
    }

    scan_msg.angle_min       = angle_min;
    scan_msg.angle_max       = angle_max;
    scan_msg.angle_increment = (angle_max - angle_min) / (effective_width - 1);
    scan_msg.time_increment  = 0.0;
    scan_msg.scan_time       = 0.0;
    scan_msg.range_min       = range_min_;
    scan_msg.range_max       = range_max_;

    scan_msg.ranges.resize(effective_width, std::numeric_limits<float>::infinity());
    scan_msg.intensities.resize(effective_width, 0.0);

    // ©¤©¤ Process column by column ©¤©¤
    bool is_16bit = (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1 ||
                     depth_msg->encoding == sensor_msgs::image_encodings::MONO16);
    bool is_32bit = (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1);
    bool is_32sc1 = (depth_msg->encoding == "32SC1");

    int bpp = 0;
    double depth_scale = 1.0; // 32FC1 = meters, 16UC1 = millimeters
    if (is_16bit)
    {
        bpp = 2;
        depth_scale = 0.001;
    }
    else if (is_32bit)
    {
        bpp = 4;
        depth_scale = 1.0;
    }
    else if (is_32sc1)
    {
        bpp = 4;
        depth_scale = 0.001;
    }
    else
    {
        ROS_ERROR_THROTTLE(5, "[depth_to_scancamera] Unsupported depth encoding: %s",
                           depth_msg->encoding.c_str());
        return;
    }

    const uint8_t* data = depth_msg->data.data();
    int row_step = depth_msg->step;

    for (int col = 0; col < effective_width; ++col)
    {
        float min_depth_in_col = std::numeric_limits<float>::max();

        for (int row = row_start; row < row_end; ++row)
        {
            double depth_val = 0.0;
            int offset = row * row_step + col * bpp;

            if (is_16bit)
            {
                uint16_t val;
                std::memcpy(&val, data + offset, sizeof(uint16_t));
                if (val == 0) continue;
                depth_val = static_cast<double>(val) * depth_scale;
            }
            else if (is_32bit)
            {
                float val;
                std::memcpy(&val, data + offset, sizeof(float));
                if (val <= 0.0f || !std::isfinite(val)) continue;
                depth_val = static_cast<double>(val);
            }
            else if (is_32sc1)
            {
                int32_t val;
                std::memcpy(&val, data + offset, sizeof(int32_t));
                if (val <= 0) continue;
                depth_val = static_cast<double>(val) * depth_scale;
            }

            if (depth_val < static_cast<double>(min_depth_in_col))
            {
                min_depth_in_col = static_cast<float>(depth_val);
            }
        }

        if (min_depth_in_col >= range_min_ &&
            min_depth_in_col <= range_max_)
        {
            scan_msg.ranges[col] = min_depth_in_col;
        }
        else
        {
            scan_msg.ranges[col] = std::numeric_limits<float>::infinity();
        }
    }

    // ©¤©¤ Publish ©¤©¤
    scan_pub_.publish(scan_msg);
}

} // namespace depth_to_scancamera
