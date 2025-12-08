/*
 * @Version      : V1.0
 * @Date         : 2024-10-14 11:15:28
 * @Description  : lidar protocol functon, and the protocol support multi type lidar
 */
#ifndef __LIDAR_PROTOCOL_H__
#define __LIDAR_PROTOCOL_H__

#include <stdint.h>
#include <vector>
#include <functional>

namespace nvistar{

#ifndef DLL_EXPORT
  #ifdef _MSC_VER
    #define DLL_EXPORT __declspec(dllexport)
  #else
    #define DLL_EXPORT
  #endif 
#endif 

//callback function,it to be serailport,socket,etc...
typedef struct{
  std::function<int(const uint8_t* data,int length)> write;
  std::function<int(uint8_t *data,int max_length)>  read;
  std::function<void(void)> flush;
}lidar_transmit_interface_t;
//callback function
typedef struct{
  lidar_transmit_interface_t  transmit;
  std::function<uint64_t(void)> get_timestamp;
}lidar_interface_t;
//single point info 
typedef struct{
    double    angle;
    double    distance;
    double    intensity;
    double    distance_raw;
    uint64_t  timestamp;
}lidar_scan_point_t;
//point info for 1 period 
typedef struct{
  int       model_code;                   //lidar model code 
  std::vector<lidar_scan_point_t> points; //one period points 
  bool      intensity_flag;               //intensity?
  double    speed;                        //RPM
  int       error_code;                   //error code 
  uint64_t  timestamp_start;              //stamp start 
  uint64_t  timestamp_stop;               //stamp stop 
}lidar_scan_period_t;

class LidarProtocolImpl;     //forward declaration

class DLL_EXPORT LidarProtocol{
  public:
    //error code define
    typedef enum{
        ERROR_CODE_RESET = 0,
        ERROR_CODE_MOTOR_LOCK = 1,
        ERROR_CODE_UP_NO_POINT = 2,
        ERROR_CODE_MOTOR_SHORTCIRCUIT = 3, 
        ERROR_CODE_NO_DATA = 0xFE,
        ERROR_CODE_NONE = 0xFF,
    }lidar_error_code_t;
    //protocol type
    typedef enum{
        PROTOCOL_MODEL_NORMAL_NO_QUALITY = 0x0208,                 //normal protocol no quality
        PROTOCOL_MODEL_NORMAL_HAS_QUALITY = 0x0308,                //normal protocol has quality
        PROTOCOL_MODEL_YW_HAS_QUALITY = 0x070C,                    //yw protocol has quality
        PROTOCOL_MODEL_LD_HAS_QUAILIY = 0x2C54,                    //ld protocol has quality
        PROTOCOL_MODEL_TM21_HAS_QUAILIY = 0x030C,                  //tm21 protocol has quality
        PROTOCOL_MODEL_ERROR_FAULT = 0x8008                        //protocol error code
    }lidar_protocol_model_t;

    //function callback define 
    typedef std::function<void(lidar_scan_period_t)> protocol_rawdata_output_callback;     //pointcloud callback 

    //function 
    LidarProtocol();
    ~LidarProtocol();
    void lidar_protocol_register(lidar_interface_t* api, protocol_rawdata_output_callback rawdata_output, bool protocol_070c_raw_flag = false); //register communitcation api
    void lidar_protocol_unregister();                               //unregister 
    bool lidar_protocol_stop_scan();                                //stop motor and scan 
    bool lidar_protocol_start_scan();                               //start motor and scan 
    bool lidar_protocol_reset();                                    //reset the lidar 
    bool lidar_protocol_get_model(std::string &model);              //get the lidar model(send at startup, so you neet send reset and get the para)
    bool lidar_protocol_get_down_soft_version(std::string &version);  //get the lidar down board software version(send at startup, so you neet send reset and get the para)
    bool lidar_protocol_get_up_soft_version(std::string &version);   //get the lidar up board software version(send at startup, so you neet send reset and get the para)
  private:
    LidarProtocolImpl* _impl;  //pimpl function
};

}

#endif


