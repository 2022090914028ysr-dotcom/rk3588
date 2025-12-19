/*
 * @Version      : V1.0
 * @Date         : 2024-10-18 15:24:38
 * @Description  : 
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

//define 
nvistar::InterfaceSerial *_serial;
nvistar::Lidar  *_lidar;
nvistar::ROSConvert *_convert;

//comm interface 
int serial_write(const uint8_t* data,int length){
  return _serial->serial_write(data, length);
}
int serial_read(uint8_t* data,int length){
  return _serial->serial_read(data, length);
}
void serial_flush(){
  _serial->serial_flush();
}
//timestamp 
uint64_t get_stamp(){
  uint64_t nanoseconds = 0;
  nanoseconds = ros::Time::now().sec * 1e9 + ros::Time::now().nsec;
	return nanoseconds;
}

int main(int argc, char * argv[]) {
  ros::init(argc, argv, "lidar_node"); 

  ros::NodeHandle nh;
  ros::Publisher scan_pub = nh.advertise<sensor_msgs::LaserScan>("scan", 1000);
  ros::NodeHandle nh_private("~");

  nvistar::ROSConvert::lidar_ros_config_t config;
  std::string serial_name;
  int serial_baud;
  int  timeout_count = 0;
  //sync para form launch 
  nh_private.param<std::string>("serial_name", serial_name, "/dev/ttyUSB0"); 
  nh_private.param<int>("serial_baud", serial_baud, 230400); 
  nh_private.param<std::string>("frame_id", config.frame_id, "laser_frame");
  nh_private.param<bool>("resolution_fixed", config.resolution_fixed, true);
  nh_private.param<bool>("counterclockwise", config.counterclockwise, false);
  nh_private.param<double>("angle_max", config.angle_max , 180.0);
  nh_private.param<double>("angle_min", config.angle_min , -180.0);
  nh_private.param<double>("range_max", config.range_max , 64.0);
  nh_private.param<double>("range_min", config.range_min , 0.001);
  nh_private.param<std::string>("angle_corp_string",  config.angle_corp_string, "");
  //start function 
  _serial = new nvistar::InterfaceSerial();
  _lidar = new nvistar::Lidar();
  _convert = new nvistar::ROSConvert();
  //callback function
  nvistar::lidar_interface_t  _interface = {
    .transmit = {
      .write = serial_write,
      .read = serial_read,
      .flush = serial_flush,
    },
    .get_timestamp = get_stamp
  };
  //serial open 
  bool ret = _serial->serial_open(serial_name, serial_baud);
  //lidar register 
  if(ret){
    _lidar->lidar_register(&_interface);
    ROS_INFO("lidar is scanning...\n");
  }else{
    ROS_ERROR("lidar serial open failed!");
    _serial->serial_close();
  }
  ros::Rate rate(20);
  //loop to get point
  while (ret && (ros::ok())) {
    nvistar::lidar_scan_period_t lidar_raw_scan;
    sensor_msgs::LaserScan lidar_ros_scan;

    nvistar::lidar_scan_status_t status = _lidar->lidar_get_scandata(lidar_raw_scan);
    switch(status){
      case nvistar::LIDAR_SCAN_OK:{
        timeout_count = 0;
        _convert->lidar_raw_to_ros(lidar_raw_scan, config, lidar_ros_scan);
        lidar_ros_scan.header.stamp = ros::Time::now();
        scan_pub.publish(lidar_ros_scan);
        break;
      }
      case nvistar::LIDAR_SCAN_ERROR_MOTOR_LOCK: {
        timeout_count = 0;
        ROS_ERROR("lidar motor lock!");
        break;
      }
      case nvistar::LIDAR_SCAN_ERROR_MOTOR_SHORTCIRCUIT: {
        timeout_count = 0;
        ROS_ERROR("lidar motor short circuit!");
        break;
      }
      case nvistar::LIDAR_SCAN_ERROR_UP_NO_POINT: {
        timeout_count = 0;
        ROS_ERROR("lidar upboard no points!");
        break;
      }
      case nvistar::LIDAR_SCAN_TIMEOUT: {
        ROS_ERROR("lidar data timeout!");
        //reconnect 
        timeout_count++;
        if(timeout_count >= 10){
          timeout_count = 0;
          _serial->serial_reopen();
          ROS_ERROR("lidar serial reopen!\n");
        }
        break;
      }
      default:{
        break;
      }
    }
    ros::spinOnce();
    rate.sleep();
  }
  ROS_INFO("lidar is stoping...\n");
  //ros stop 
  delete _convert;
  delete _lidar;
  delete _serial;
}

