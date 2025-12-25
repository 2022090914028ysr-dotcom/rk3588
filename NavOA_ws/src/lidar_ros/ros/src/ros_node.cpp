/*
 * @Version      : V1.1 (Fixed timestamp)
 * @Date         : 2024-10-18
 * @Description  : Fixed scan timestamp to use actual frame start time, not ros::Time::now()
 */
#include "interface/serial/interface_serial.hpp"
#include "interface/console/interface_console.hpp"
#include "ros/init.h"
#include "ros_convert.hpp"
#include "lidar.hpp"
#include <chrono>
#include <cstddef>
#include <cstdio>
#include <inttypes.h>
#include <thread>
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

// Global pointers
nvistar::InterfaceSerial* _serial;
nvistar::Lidar* _lidar;
nvistar::ROSConvert* _convert;

// Global variables for timestamp alignment
static uint64_t g_frame_start_time_ns = 0;      // 帧起始时间（纳秒）
static bool g_first_point_in_frame = true;      // 标记是否是当前帧的第一个点

// Serial interface functions
int serial_write(const uint8_t* data, int length) {
    return _serial->serial_write(data, length);
}

int serial_read(uint8_t* data, int length) {
    return _serial->serial_read(data, length);
}

void serial_flush() {
    _serial->serial_flush();
}

// Timestamp function: record first point time as frame start time
uint64_t get_stamp() {
    uint64_t now_ns = ros::Time::now().sec * 1000000000ULL + ros::Time::now().nsec;
    
    if (g_first_point_in_frame) {
        g_frame_start_time_ns = now_ns;   // 记录帧起始时间
        g_first_point_in_frame = false;
    }
    return now_ns;
}

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "lidar_node");
    ros::NodeHandle nh;
    ros::Publisher scan_pub = nh.advertise<sensor_msgs::LaserScan>("scan", 1000);
    ros::NodeHandle nh_private("~");

    nvistar::ROSConvert::lidar_ros_config_t config;
    std::string serial_name;
    int serial_baud;
    int timeout_count = 0;

    // Load parameters
    nh_private.param<std::string>("serial_name", serial_name, "/dev/ttyUSB0");
    nh_private.param<int>("serial_baud", serial_baud, 230400);
    nh_private.param<std::string>("frame_id", config.frame_id, "laser_frame");
    nh_private.param<bool>("resolution_fixed", config.resolution_fixed, true);
    nh_private.param<bool>("counterclockwise", config.counterclockwise, false);
    nh_private.param<double>("angle_max", config.angle_max, 180.0);
    nh_private.param<double>("angle_min", config.angle_min, -180.0);
    nh_private.param<double>("range_max", config.range_max, 64.0);
    nh_private.param<double>("range_min", config.range_min, 0.001);
    nh_private.param<std::string>("angle_corp_string", config.angle_corp_string, "");

    // Initialize hardware
    _serial = new nvistar::InterfaceSerial();
    _lidar = new nvistar::Lidar();
    _convert = new nvistar::ROSConvert();

    nvistar::lidar_interface_t _interface = {
        .transmit = {
            .write = serial_write,
            .read = serial_read,
            .flush = serial_flush,
        },
        .get_timestamp = get_stamp
    };

    bool ret = _serial->serial_open(serial_name, serial_baud);
    if (ret) {
        _lidar->lidar_register(&_interface);
        ROS_INFO("Lidar is scanning...\n");
    } else {
        ROS_ERROR("Lidar serial open failed!");
        _serial->serial_close();
        return -1;
    }

    ros::Rate rate(20);

    while (ret && ros::ok()) {
        nvistar::lidar_scan_period_t lidar_raw_scan;
        sensor_msgs::LaserScan lidar_ros_scan;

        nvistar::lidar_scan_status_t status = _lidar->lidar_get_scandata(lidar_raw_scan);
        switch (status) {
            case nvistar::LIDAR_SCAN_OK: {
                timeout_count = 0;
                _convert->lidar_raw_to_ros(lidar_raw_scan, config, lidar_ros_scan);

                // ✅ FIX: Use actual frame start time instead of ros::Time::now()
                ros::Time scan_time;
                scan_time.fromNSec(g_frame_start_time_ns);
                lidar_ros_scan.header.stamp = scan_time;
                lidar_ros_scan.header.frame_id = config.frame_id;

                scan_pub.publish(lidar_ros_scan);

                // Reset for next frame
                g_first_point_in_frame = true;
                break;
            }
            case nvistar::LIDAR_SCAN_ERROR_MOTOR_LOCK: {
                timeout_count = 0;
                ROS_ERROR("Lidar motor lock!");
                break;
            }
            case nvistar::LIDAR_SCAN_ERROR_MOTOR_SHORTCIRCUIT: {
                timeout_count = 0;
                ROS_ERROR("Lidar motor short circuit!");
                break;
            }
            case nvistar::LIDAR_SCAN_ERROR_UP_NO_POINT: {
                timeout_count = 0;
                ROS_ERROR("Lidar upboard no points!");
                break;
            }
            case nvistar::LIDAR_SCAN_TIMEOUT: {
                ROS_ERROR("Lidar data timeout!");
                timeout_count++;
                if (timeout_count >= 10) {
                    timeout_count = 0;
                    _serial->serial_reopen();
                    ROS_ERROR("Lidar serial reopened!\n");
                }
                break;
            }
            default: {
                break;
            }
        }
        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("Lidar is stopping...\n");
    delete _convert;
    delete _lidar;
    delete _serial;
    return 0;
}
