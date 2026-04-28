/*
 * @Version      : V1.0
 * @Date         : 2024-10-15 17:24:46
 * @Description  : lidar process
 */

#ifndef __LIDAR_H__
#define __LIDAR_H__

#include "lidar/lidar_protocol.hpp"
#include <string>

namespace nvistar{

#ifndef DLL_EXPORT
  #ifdef _MSC_VER
    #define DLL_EXPORT __declspec(dllexport)
  #else
    #define DLL_EXPORT
  #endif 
#endif 

#define LIDAR_SDK_VERSION   "2.0.4"

#ifndef M_PI
  #define M_PI 3.14159265358979323846
#endif

class LidarImpl;     //forward declaration

//lidar return status 
typedef enum{
  LIDAR_SCAN_OK = 0,
  LIDAR_SCAN_WAITING,
  LIDAR_SCAN_TIMEOUT,
  LIDAR_SCAN_ERROR_MOTOR_LOCK,
  LIDAR_SCAN_ERROR_UP_NO_POINT,
  LIDAR_SCAN_ERROR_MOTOR_SHORTCIRCUIT,
  LIDAR_SCAN_ERROR_RESET,
}lidar_scan_status_t;

//ros format 
typedef struct{
  double    angle_min;
  double    angle_max;
  double    range_min;
  double    range_max;
  bool      intensity_flag;               //intensity?
  double    speed;                        //RPM
  int       error_code;                   //error code 
  uint64_t  timestamp_start;              //stamp start 
  uint64_t  timestamp_stop;               //stamp stop 
  std::vector<lidar_scan_point_t> points;
}lidar_scan_ros_format_t;

class DLL_EXPORT Lidar{
  public:
    Lidar();
    ~Lidar();
    void lidar_register(lidar_interface_t* interface, bool protocol_070c_raw_flag = false);
    void lidar_unregister();
    bool lidar_stop_scan();
    bool lidar_start_scan();
    bool lidar_reset();
    bool lidar_get_model(std::string &model);
    bool lidar_get_down_soft_version(std::string &version);
    bool lidar_get_up_soft_version(std::string &version);
    lidar_scan_status_t lidar_get_scandata(lidar_scan_period_t &scan, uint32_t timeout = 2000);
    void lidar_raw_to_ros_format(lidar_scan_period_t lidar_raw, lidar_scan_ros_format_t &ros_format_scan);
    std::string get_sdk_version();  
  private:
    LidarImpl *_impl;
    LidarProtocol *_protocol = nullptr;
    double angle_to_ros(bool counterclockwise_flag,double angle);
};

}

#endif