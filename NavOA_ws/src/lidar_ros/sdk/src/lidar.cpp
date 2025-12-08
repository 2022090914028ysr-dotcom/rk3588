/*
 * @Version      : V1.0
 * @Date         : 2024-10-15 17:25:02
 * @Description  : 
 */
#include "lidar.hpp"
#include "lidar/lidar_protocol.hpp"
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cmath>
#include <utility>
#include <vector>

namespace nvistar{
//pre define
class LidarImpl{
public:
  std::atomic<bool> lidar_pointcloud_ready_flag = {false};
  lidar_scan_period_t  scan_period;   //one period points
  bool  lidar_scandata_update_flag = false;  //lidar data update flag 
  std::chrono::steady_clock::time_point   last_point_update_time = std::chrono::steady_clock::now();    //last point    
};

Lidar::Lidar() : _impl(new LidarImpl){
  _protocol = new LidarProtocol();
}

Lidar::~Lidar(){
  _protocol->lidar_protocol_unregister();
  delete _impl;
  delete _protocol;
}

/**
 * @Function: get_sdk_version
 * @Description: get lidar sdk version 
 * @Return: std::string 
 */
std::string Lidar::get_sdk_version(){
  return LIDAR_SDK_VERSION;
}

/**
 * @Function: lidar_register
 * @Description: lidar register comm interface 
 * @Return: void
 * @param {lidar_interface_t*} interface
 */
void Lidar::lidar_register(lidar_interface_t* interface, bool protocol_070c_raw_flag){
  //callback function 
  auto pointcloud_callback = [this](lidar_scan_period_t rawdata_output){
    //read finish, then enable to read next package 
    if(_impl->lidar_pointcloud_ready_flag.load() == false){
      //calc every point stamp value
      if(rawdata_output.points.size() > 1){
        uint64_t  timestamp_differ = rawdata_output.timestamp_stop - rawdata_output.timestamp_start;
        uint64_t  timestamp_gap = (timestamp_differ / rawdata_output.points.size() - 1);
        for(size_t i = 0; i<rawdata_output.points.size(); i++){
          rawdata_output.points[i].timestamp = rawdata_output.timestamp_start + i*timestamp_gap;
        }
      }

      //update points 
      _impl->scan_period = rawdata_output;

      //change flag 
      if(rawdata_output.timestamp_start > 0){
        _impl->lidar_pointcloud_ready_flag.store(true);   //finished
      }
    }
  };
  _protocol->lidar_protocol_register(interface, pointcloud_callback, protocol_070c_raw_flag);
}

/**
 * @Function: lidar_unregister
 * @Description: lidar unregister 
 * @Return: void
 */
void Lidar::lidar_unregister(){
  _protocol->lidar_protocol_unregister();
}

/**
 * @Function: lidar_stop_scan
 * @Description: lidar stop scan 
 * @Return: bool 
 */
bool Lidar::lidar_stop_scan(){
  return _protocol->lidar_protocol_stop_scan();
}

/**
 * @Function: lidar_start_scan
 * @Description: lidar start scan
 * @Return: bool 
 */
bool Lidar::lidar_start_scan(){
  return _protocol->lidar_protocol_start_scan();
}

/**
 * @Function: lidar_reset
 * @Description: lidar reset
 * @Return: bool 
 */
bool Lidar::lidar_reset(){
  return _protocol->lidar_protocol_reset();
}

/**
 * @Function: lidar_get_model
 * @Description: lidar get model 
 * @Return: bool 
 * @param {string} &model
 */
bool Lidar::lidar_get_model(std::string &model){
  return _protocol->lidar_protocol_get_model(model);
}

/**
 * @Function: lidar_get_down_soft_version
 * @Description: lidar get down software version
 * @Return: bool 
 * @param {string} &version
 */
bool Lidar::lidar_get_down_soft_version(std::string &version){
  return _protocol->lidar_protocol_get_down_soft_version(version);
}

/**
 * @Function: lidar_get_up_soft_version
 * @Description: lidar get down software version
 * @Return: bool
 * @param {string} &version
 */
bool Lidar::lidar_get_up_soft_version(std::string &version){
  return _protocol->lidar_protocol_get_up_soft_version(version);
}


/**
 * @Function: angle_to_ros
 * @Description: angle to ros 
 * @Return: double 
 * @param {double} angle
 */
double Lidar::angle_to_ros(bool counterclockwise_flag,double angle){
  //clock wise 
  if(counterclockwise_flag){
    angle = 360.f - angle;
  }
  //angle change 
  if(angle > 180.f){
    angle -= 360.f;
  }
  //to rad 
  angle = angle * M_PI / 180.f;

  return angle;
}

/**
 * @Function: lidar_raw_to_ros_format
 * @Description: lidar rawdata to ros format data 
 * @Return: void 
 * @param {lidar_scan_period_t} lidar_raw
 * @param {lidar_ros_config_t} config
 * @param {LaserScan} &ros_scan
 */
void Lidar::lidar_raw_to_ros_format(lidar_scan_period_t lidar_raw, lidar_scan_ros_format_t &ros_format_scan){
  double angle_min_radian = M_PI * (-1.f);    //angle min 
  double angle_max_radian = M_PI;    //angle min 
  double angle_increment = 0.f;
  size_t points_size = lidar_raw.points.size();
  if(points_size <= 1){
    angle_increment = 0;
  }else{
    angle_increment = 2.f * M_PI / static_cast<int>(points_size);
  }
  ros_format_scan.angle_min = angle_min_radian;
  ros_format_scan.angle_max = angle_max_radian;
  ros_format_scan.range_min = 0.001;
  ros_format_scan.range_max = 15.f;
  ros_format_scan.speed = lidar_raw.speed;
  ros_format_scan.intensity_flag = lidar_raw.intensity_flag;
  ros_format_scan.timestamp_start = lidar_raw.timestamp_start;
  ros_format_scan.timestamp_stop = lidar_raw.timestamp_stop;
  //points and intensity
  ros_format_scan.points.resize(points_size);
  for(size_t index = 0; index < points_size; index++){
    double range = 0.f;
    double intensity = 0.f;
    range = lidar_raw.points[index].distance / 1000.f;
    intensity = lidar_raw.points[index].intensity;
    //angle to ros 
    double angle = angle_to_ros(true, lidar_raw.points[index].angle); //to [-PI ,PI] 
    ros_format_scan.points[index].distance = range;
    ros_format_scan.points[index].intensity = intensity;
    ros_format_scan.points[index].angle = angle;
    ros_format_scan.points[index].timestamp = lidar_raw.points[index].timestamp;
  }
}

/**
 * @Function: lidar_get_scandata
 * @Description: lidar get scan data 
 * @Return: lidar_scan_status_t --- status 
 * @param {lidar_scan_period_t} &scan
 * @param {uint32_t} timeout
 */
lidar_scan_status_t Lidar::lidar_get_scandata(lidar_scan_period_t &scan, uint32_t timeout){
  //update info 
  if(_impl->lidar_pointcloud_ready_flag.load()){
    //update the last upate time 
    _impl->last_point_update_time = std::chrono::steady_clock::now();
    //update scan data 
    scan = _impl->scan_period;
    //clear the flag 
    _impl->lidar_pointcloud_ready_flag.store(false);
    //error code judge
    if(scan.error_code == LidarProtocol::ERROR_CODE_MOTOR_LOCK){
      return LIDAR_SCAN_ERROR_MOTOR_LOCK;
    }else if(scan.error_code == LidarProtocol::ERROR_CODE_UP_NO_POINT){
      return LIDAR_SCAN_ERROR_UP_NO_POINT;
    }else if(scan.error_code == LidarProtocol::ERROR_CODE_MOTOR_SHORTCIRCUIT) {
      return LIDAR_SCAN_ERROR_MOTOR_SHORTCIRCUIT;
    }else if(scan.error_code == LidarProtocol::ERROR_CODE_RESET){
      return LIDAR_SCAN_ERROR_RESET;
    }
    return LIDAR_SCAN_OK;
  }else{
    //check timeout
    if(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - _impl->last_point_update_time).count() > timeout){
      _impl->last_point_update_time = std::chrono::steady_clock::now();   //update last time 
      return LIDAR_SCAN_TIMEOUT;
    }
  }
  //waiting...
  return LIDAR_SCAN_WAITING;
}
}