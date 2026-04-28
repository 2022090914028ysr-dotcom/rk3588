/*
 * @Version      : V1.0
 * @Date         : 2024-10-17 19:34:42
 * @Description  : 
 */

#include "ros_convert.hpp"
#include <cmath>
#include <cstddef>
#include <vector>

namespace nvistar{

ROSConvert::ROSConvert(){
}

ROSConvert::~ROSConvert(){
}

/**
 * @Function: lidar_raw_to_ros
 * @Description: lidar rawdata to ros format data 
 * @Return: void 
 * @param {lidar_scan_period_t} lidar_raw
 * @param {lidar_ros_config_t} config
 * @param {LaserScan} &ros_scan
 */
void ROSConvert::lidar_raw_to_ros(lidar_scan_period_t lidar_raw, lidar_ros_config_t config, sensor_msgs::LaserScan &ros_scan){
  float angle_min_radian = config.angle_min / 180.f * M_PI;    //angle min 
  float angle_max_radian = config.angle_max / 180.f * M_PI;    //angle min 
  float angle_increment = 0.0;
  size_t points_size = lidar_raw.points.size();
  if(points_size <= 1){
    angle_increment = 0;
  }else{
    angle_increment = 2.f * M_PI / static_cast<int>(points_size);
  }
  uint64_t scan_time_ns_differ = lidar_raw.timestamp_stop - lidar_raw.timestamp_start;
  float scan_time = static_cast<double>(scan_time_ns_differ) / static_cast<double>(1e9);
  float time_increment = 0.0;
  if(points_size <= 1){
    time_increment = 0;
  }else{
    time_increment =  scan_time / static_cast<int>(points_size);
  }
  std::vector<float> angle_corp_list = angle_corp_get(config.angle_corp_string);
  //ros config info
  ros_scan.header.frame_id = config.frame_id;
  ros_scan.header.stamp.sec = lidar_raw.timestamp_start/1000000000ul;
  ros_scan.header.stamp.nsec = lidar_raw.timestamp_start%1000000000ul;
  ros_scan.angle_min = angle_min_radian;
  ros_scan.angle_max = angle_max_radian;
  ros_scan.range_min = config.range_min;
  ros_scan.range_max = config.range_max;
  ros_scan.angle_increment = angle_increment;
  ros_scan.scan_time = scan_time;
  ros_scan.time_increment = time_increment;
  //points and intensity
  ros_scan.ranges.assign(points_size, 0.f);
  ros_scan.intensities.assign(points_size, 0.f);
  for(auto point : lidar_raw.points){
    float range = 0.f;
    float intensity = 0.f;
    //filter angle???
    if(angle_corp_flag(point.angle, angle_corp_list)){
      range = 0.f;
      intensity = 0.f;
    }else{
      range = point.distance / 1000.f;
      intensity = point.intensity;
    }
    //angle to ros 
    float angle = angle_to_ros(config.counterclockwise, point.angle); //to [-PI ,PI] 
    int index = static_cast<int>(std::ceil((angle - angle_min_radian) / angle_increment));  //up to int 
    if((index < points_size) && (index >= 0)){
      ros_scan.ranges[index] = range;
      ros_scan.intensities[index] = intensity;
    }
  }
}

/**
 * @Function: angle_to_ros
 * @Description: angle to ros 
 * @Return: double 
 * @param {double} angle
 */
double ROSConvert::angle_to_ros(bool counterclockwise_flag,double angle){
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
 * @Function: angle_corp_get
 * @Description: get angle corp 
 * @Return: vector
 * @param {string} angle_string
 */
std::vector<float> ROSConvert::angle_corp_get(std::string angle_string){
  std::vector<float> elems;
  std::stringstream ss(angle_string);
  std::string number;
  while (std::getline(ss, number, ',')) {
    elems.push_back(atof(number.c_str()));
  }

  if(elems.size() % 2 != 0){
    elems.clear();
    elems.shrink_to_fit();
  }
  return elems;
}

/**
 * @Function: angle_corp_flag
 * @Description: angle corp flag get 
 * @Return: bool 
 * @param {float} angle
 * @param {vector<float>} angle_corp_list
 */
bool ROSConvert::angle_corp_flag(float angle,std::vector<float> angle_corp_list){
  if(angle_corp_list.size() % 2 != 0){
    return false;
  }
  
  for(int i = 0; i<angle_corp_list.size() / 2; i++){
    if((angle >= angle_corp_list[i*2]) && (angle <= angle_corp_list[i*2 + 1])){
      return true;
    }
  }
  return false;
}

}
