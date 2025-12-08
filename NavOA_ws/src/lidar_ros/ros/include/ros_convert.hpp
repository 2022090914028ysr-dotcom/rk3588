/*
 * @Version      : V1.0
 * @Date         : 2024-10-17 19:34:30
 * @Description  : 
 */
#ifndef __ROS_CONVERT_H__
#define __ROS_CONVERT_H__

#include "lidar.hpp"
#include "ros/ros.h"
#include <string>
#include <sensor_msgs/LaserScan.h>

namespace nvistar{
class ROSConvert{
public:
  #pragma pack(push)
  #pragma pack(1)
    typedef struct{
      std::string frame_id;          //frame id 
      bool        counterclockwise;  //counter clockwise 
      double      angle_min;         //angle min 
      double      angle_max;         //angle max 
      double      range_min;         //range min
      double      range_max;         //range max 
      bool        resolution_fixed;  //fixed resolution
      std::string angle_corp_string; //angle ignore
    }lidar_ros_config_t;;
  #pragma pack(pop)

  ROSConvert();
  ~ROSConvert();

  void lidar_raw_to_ros(lidar_scan_period_t lidar_raw, lidar_ros_config_t config, sensor_msgs::LaserScan &ros_scan);
private:
  double angle_to_ros(bool counterclockwise_flag,double angle);         //angle to ros 
  std::vector<float> angle_corp_get(std::string angle_string);          //angle corp get 
  bool angle_corp_flag(float angle,std::vector<float> angle_corp_list); //angle corp 
};
}

#endif