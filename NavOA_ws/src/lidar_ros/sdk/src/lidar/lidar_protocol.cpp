/*
 * @Version      : V1.0
 * @Date         : 2024-10-14 11:15:39
 * @Description  : lidar protocol 
 */
#include "lidar/lidar_protocol.hpp"
#include <atomic>
#include <bits/stdint-uintn.h>
#include <chrono>
#include <thread>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <cstring>
#include <mutex>

#include "lidar.hpp"
#if defined(_WIN32)
    #include <windows.h>
#else 
	#include <unistd.h>
#endif 

namespace nvistar{
//forward define
class LidarProtocolImpl{
public:
    //one pack has points def 
    #define NORMAL_NO_QUALITY_PACK_MAX_POINTS         8            //normal protocol no quality pack points
    #define NORMAL_HAS_QUALITY_PACK_MAX_POINTS        8            //normal protocol has quality pack points
    #define YW_HAS_QUALITY_PACK_MAX_POINTS            12           //yw protocol has quality pack points
    #define YW_HAS_QUALITY_WITH_RAW_PACK_MAX_POINTS   12           //yw protocol has quality and raw distance pack points
    #define LD_HAS_QUALITY_PACK_MAX_POINTS            12           //ld protocol has quality pack points
    #define TM21_HAS_QUALITY_PACK_MAX_POINTS          12           //tm21 protocol has quality pack points
    #define ROBOROCK_HAS_QUALITY_PACK_MAX_POINTS      4            //roborock has quality pack points

    #define LIDAR_TRANSMIT_RECEIVED_BUF               1024         //lidar transmit buffer

    //crc table
    const uint8_t ld_crc_table[256] = {
        0x00, 0x4d, 0x9a, 0xd7, 0x79, 0x34, 0xe3,
        0xae, 0xf2, 0xbf, 0x68, 0x25, 0x8b, 0xc6, 0x11, 0x5c, 0xa9, 0xe4, 0x33,
        0x7e, 0xd0, 0x9d, 0x4a, 0x07, 0x5b, 0x16, 0xc1, 0x8c, 0x22, 0x6f, 0xb8,
        0xf5, 0x1f, 0x52, 0x85, 0xc8, 0x66, 0x2b, 0xfc, 0xb1, 0xed, 0xa0, 0x77,
        0x3a, 0x94, 0xd9, 0x0e, 0x43, 0xb6, 0xfb, 0x2c, 0x61, 0xcf, 0x82, 0x55,
        0x18, 0x44, 0x09, 0xde, 0x93, 0x3d, 0x70, 0xa7, 0xea, 0x3e, 0x73, 0xa4,
        0xe9, 0x47, 0x0a, 0xdd, 0x90, 0xcc, 0x81, 0x56, 0x1b, 0xb5, 0xf8, 0x2f,
        0x62, 0x97, 0xda, 0x0d, 0x40, 0xee, 0xa3, 0x74, 0x39, 0x65, 0x28, 0xff,
        0xb2, 0x1c, 0x51, 0x86, 0xcb, 0x21, 0x6c, 0xbb, 0xf6, 0x58, 0x15, 0xc2,
        0x8f, 0xd3, 0x9e, 0x49, 0x04, 0xaa, 0xe7, 0x30, 0x7d, 0x88, 0xc5, 0x12,
        0x5f, 0xf1, 0xbc, 0x6b, 0x26, 0x7a, 0x37, 0xe0, 0xad, 0x03, 0x4e, 0x99,
        0xd4, 0x7c, 0x31, 0xe6, 0xab, 0x05, 0x48, 0x9f, 0xd2, 0x8e, 0xc3, 0x14,
        0x59, 0xf7, 0xba, 0x6d, 0x20, 0xd5, 0x98, 0x4f, 0x02, 0xac, 0xe1, 0x36,
        0x7b, 0x27, 0x6a, 0xbd, 0xf0, 0x5e, 0x13, 0xc4, 0x89, 0x63, 0x2e, 0xf9,
        0xb4, 0x1a, 0x57, 0x80, 0xcd, 0x91, 0xdc, 0x0b, 0x46, 0xe8, 0xa5, 0x72,
        0x3f, 0xca, 0x87, 0x50, 0x1d, 0xb3, 0xfe, 0x29, 0x64, 0x38, 0x75, 0xa2,
        0xef, 0x41, 0x0c, 0xdb, 0x96, 0x42, 0x0f, 0xd8, 0x95, 0x3b, 0x76, 0xa1,
        0xec, 0xb0, 0xfd, 0x2a, 0x67, 0xc9, 0x84, 0x53, 0x1e, 0xeb, 0xa6, 0x71,
        0x3c, 0x92, 0xdf, 0x08, 0x45, 0x19, 0x54, 0x83, 0xce, 0x60, 0x2d, 0xfa,
        0xb7, 0x5d, 0x10, 0xc7, 0x8a, 0x24, 0x69, 0xbe, 0xf3, 0xaf, 0xe2, 0x35,
        0x78, 0xd6, 0x9b, 0x4c, 0x01, 0xf4, 0xb9, 0x6e, 0x23, 0x8d, 0xc0, 0x17,
        0x5a, 0x06, 0x4b, 0x9c, 0xd1, 0x7f, 0x32, 0xe5, 0xa8
    };

    #pragma pack(push)
    #pragma pack(1)

    //lidar version info 
    typedef struct{
        std::string downBoard_Model;            //lidar down model 
        std::string downBoard_HardVersion;      //lidar down hard version 
        std::string downBoard_SoftVersion;      //lidar down software version 
        std::string downBoard_ID;               //lidar down id
        std::string downBoard_Date;             //lidar down date
        std::string downBoard_UID;              //lidar down uid 
        std::string downBoard_InputVoltage;     //lidar down voltage 
        std::string downBoard_BuildTime;        //lidar down buildtime
        uint16_t downBoard_IRMaxVoltage;
        std::string downBoard_MCUTemperature;   //lidar down MCU temperature
        std::string downBoard_SnNumber;         //lidar down sn number 

        std::string upBoard_Model;              //lidar up model 
        std::string upBoard_HardVersion;        //lidar up hard version 
        std::string upBoard_SoftVersion;        //lidar up soft version 
        std::string upBoard_ID;                 //lidar up id 
        std::string upBoard_Date;               //lidar up date 
        std::string upBoard_UID;                //lidar up uid
        std::string upBoard_BuildTime;          //lidar up build time
        uint8_t upBoard_VBD;                    //lidar up vbd
        uint8_t upBoard_TDC;                    //lidar up tdc
        int8_t  upBoard_Temperature;            //lidar up temperature 
    }lidar_boot_header_info_t;

    //received info 
    typedef struct{
        uint8_t   package_head;                 
        uint8_t   package_cmd;                  
        uint8_t   package_length;               
        uint8_t   package_data[255];            
    }lidar_downboard_receive_buf_t;
    typedef struct{
        uint16_t  package_head;                 
        uint8_t   package_cmd;                  
        uint8_t   package_length;               
        uint8_t   package_data[255];           
    }lidar_upboard_receive_buf_t;
    //error code
    typedef struct{
        uint16_t  package_head;               
        uint8_t   package_information;         
        uint8_t   package_point_count;  
        uint8_t   package_reserved_1;       
        uint8_t   package_errorcode;           
        uint8_t   package_reserved_2;      
        uint8_t   package_checksum;         
    }lidar_errorcode_package_t;

    //===================normal protocol has quality
    typedef struct{
        uint16_t distance;
    }lidar_normal_no_quality_point_t;

    typedef struct{
        uint16_t  package_head;                 
        uint8_t   package_information;          
        uint8_t   package_point_count;                  
        uint16_t  package_speed;                
        uint16_t  package_first_angle;     
        lidar_normal_no_quality_point_t points[NORMAL_NO_QUALITY_PACK_MAX_POINTS];
        uint16_t  package_last_angle;       
        uint16_t  package_checksum;             
    }lidar_normal_no_quality_package_t;

    //===================normal protocol has quality
    typedef struct{
        uint16_t distance;
        uint8_t  quality;
    }lidar_normal_has_quality_point_t;

    typedef struct{
        uint16_t  package_head;                 
        uint8_t   package_information;          
        uint8_t   package_point_count;                  
        uint16_t  package_speed;                
        uint16_t  package_first_angle;     
        lidar_normal_has_quality_point_t points[NORMAL_HAS_QUALITY_PACK_MAX_POINTS];
        uint16_t  package_last_angle;       
        uint16_t  package_checksum;             
    }lidar_normal_has_quality_package_t;

    //===================yw protocol has quality
    typedef struct{
        uint16_t  distance;
        uint16_t  quality;
    }lidar_yw_has_quality_point_t;

    typedef struct{
        uint16_t  package_head;                 
        uint8_t   package_information;          
        uint8_t   package_point_count;                  
        uint16_t  package_speed;                
        uint16_t  package_first_angle;     
        lidar_yw_has_quality_point_t points[YW_HAS_QUALITY_PACK_MAX_POINTS];
        uint16_t  package_last_angle;       
        uint16_t  package_checksum;             
    }lidar_yw_has_quality_package_t;

    //===================yw protocol has quality with raw
    typedef struct{
        uint16_t  distance;
        uint16_t  quality;
        uint16_t  distance_raw;
    }lidar_yw_has_quality_with_raw_point_t;

    typedef struct{
        uint16_t  package_head;
        uint8_t   package_information;
        uint8_t   package_point_count;
        uint16_t  package_speed;
        uint16_t  package_first_angle;
        lidar_yw_has_quality_with_raw_point_t points[YW_HAS_QUALITY_WITH_RAW_PACK_MAX_POINTS];
        uint16_t  package_last_angle;
        uint16_t  package_checksum;
    }lidar_yw_has_quality_with_raw_package_t;

    //===================ld protocol has quality
    typedef struct{
        uint16_t  distance;
        uint8_t   quality;
    }lidar_ld_has_quality_point_t;

    typedef struct{
        uint16_t  package_head;                                 
        uint16_t  package_speed;                
        uint16_t  package_first_angle;     
        lidar_ld_has_quality_point_t points[LD_HAS_QUALITY_PACK_MAX_POINTS];
        uint16_t  package_last_angle; 
        uint16_t  package_timestamp;        
        uint8_t   package_checksum;             
    }lidar_ld_has_quality_package_t;

    //===================tm21 protocol has quality
    typedef struct{
        uint16_t distance;
        uint8_t  quality;
    }lidar_tm21_has_quality_point_t;

    typedef struct{
        uint16_t  package_head;
        uint8_t   package_information;
        uint8_t   package_point_count;
        uint16_t  package_speed;
        uint16_t  package_first_angle;
        lidar_tm21_has_quality_point_t points[TM21_HAS_QUALITY_PACK_MAX_POINTS];
        uint16_t  package_last_angle;
        uint16_t  package_checksum;
    }lidar_tm21_has_quality_package_t;


    //all protocol union 
    typedef union {
        lidar_normal_no_quality_package_t               normal_no_quality;       
        lidar_normal_has_quality_package_t              normal_has_quality;  
        lidar_yw_has_quality_package_t                  yw_has_quality;
        lidar_yw_has_quality_with_raw_package_t         yw_has_quality_with_raw;
        lidar_ld_has_quality_package_t                  ld_has_quality;
        lidar_tm21_has_quality_package_t                tm21_has_quality;
        lidar_upboard_receive_buf_t                     upboard_info;     
        lidar_downboard_receive_buf_t                   downboard_info;   
        lidar_errorcode_package_t                       error_code;  
        uint8_t  buf[255];
    }lidar_receive_package_t;


    #define GET_LIDAR_DATA_SIZE(model, with_raw) \
    (       \
        (model == LidarProtocol::PROTOCOL_MODEL_NORMAL_NO_QUALITY) ? sizeof(lidar_normal_no_quality_package_t) : \
        (model == LidarProtocol::PROTOCOL_MODEL_NORMAL_HAS_QUALITY) ? sizeof(lidar_normal_has_quality_package_t) : \
        (model == LidarProtocol::PROTOCOL_MODEL_YW_HAS_QUALITY) ? ( \
                (with_raw) ? sizeof(lidar_yw_has_quality_with_raw_package_t) : sizeof(lidar_yw_has_quality_package_t) \
                ) : \
        (model == LidarProtocol::PROTOCOL_MODEL_LD_HAS_QUAILIY) ? sizeof(lidar_ld_has_quality_package_t) : \
        (model == LidarProtocol::PROTOCOL_MODEL_TM21_HAS_QUAILIY) ? sizeof(lidar_tm21_has_quality_package_t) : \
        (model == LidarProtocol::PROTOCOL_MODEL_ERROR_FAULT) ? sizeof(lidar_errorcode_package_t) : \
        0   \
    )
    
    //lidar data transmit
    typedef struct{
        uint8_t buf[LIDAR_TRANSMIT_RECEIVED_BUF];
        int length;
    }lidar_transmit_received_data_t;
    #pragma pack(pop)

    //var
    std::atomic<bool> thread_running_flag = {false};                  //thread running flag  
    std::atomic<bool> thread_finished_flag = {true};                  //thread finished flag 
    lidar_transmit_received_data_t lidar_transmit_received_data;      //lidar received data 
    lidar_receive_package_t        lidar_receive_package;             //lidar received package
    std::vector<lidar_scan_point_t> lidar_points_cache; //points cache
    lidar_scan_period_t  lidar_point_raw_period_cache;                     //one period point cache
    std::mutex  lidar_mtx;
    
    int received_pos = 0;   //analysis received position
    int received_package_size = 0;  //received package size 
    int lidar_model_code = 0;       //lidar model code 
    lidar_boot_header_info_t  lidar_boot_header_info;                 //lidar boot info 
    double                    lidar_last_angle = 0.f;                 //lidar last angle 
    bool lidar_boot_head_received_finished_flag = false;              //lidar receive header finished
    bool protocol_070c_with_raw_flag = false;                         //07 0c protocol has raw?

    lidar_interface_t*                                  lidar_interface_function = nullptr;         //lidar interface 
    LidarProtocol::protocol_rawdata_output_callback     lidar_rawdata_output_function = nullptr;    //rawdata output function 

    /**
     * @Function: lidar_send_cmd
     * @Description: send command to lidar
     * @Return: bool 
     * @param {uint8_t} cmd
     * @param {uint8_t*} payload
     * @param {uint8_t} length
     */
    bool lidar_send_cmd(uint8_t cmd, uint8_t* payload, uint8_t length){
        uint8_t buf[255];
        //length overload
        if(length + 5 > 255){
            return false;
        }
        //judge the interface 
        if(lidar_interface_function == nullptr){
            return false;
        }
        if(lidar_interface_function->transmit.write == nullptr){
            return false;
        }
        //send 
        if((payload == nullptr) && (length == 0)){
            buf[0] = 0xA5;
            buf[1] = 0x5A;
            buf[2] = cmd;
            buf[3] = 0x00;
            buf[4] = acc_checksum(buf, 4);

            lidar_interface_function->transmit.write(buf, 5);
        }else{
            buf[0] = 0xA5;
            buf[1] = 0x5A;
            buf[2] = cmd;
            buf[3] = length;
            memcpy(&buf[4], payload, length);
            buf[length + 4] = acc_checksum(buf, length+4);

            lidar_interface_function->transmit.write(buf, length+5);
        }
        return true;
    }

    /**
    * @Function: lidar_pointcloud_data_unpack
    * @Description: data analysis
    * @Return: void
    * @param {uint8_t} *data
    * @param {int} length
    */
    void lidar_pointcloud_data_unpack(uint8_t *data,int length){
        uint8_t cur_byte = 0;
        //judge the size
        if(0 == length){
            return;
        }
        //loop for data 
        for(int i = 0; i<length; i++){
            cur_byte = data[i];
            //every bytes to analysis 
            switch(received_pos) {
                case 0:{
                    if(0x55 == cur_byte){
                        lidar_receive_package.buf[received_pos] = cur_byte;
                        received_pos++;
                    }else if(0xA5 == cur_byte){
                        lidar_receive_package.buf[received_pos] = cur_byte;
                        received_pos++;
                    }else if(0x54 == cur_byte){
                        lidar_receive_package.buf[received_pos] = cur_byte;
                        received_pos++;
                    }else{
                        received_pos = 0;
                    }
                    break;
                }
                case 1:{
                    if(0x55 == lidar_receive_package.buf[0]){          //55 AA
                        if(0xAA == cur_byte){
                            lidar_receive_package.buf[received_pos] = cur_byte;
                            received_pos++;
                        }else if( (0xAB == cur_byte) || (0xAC == cur_byte) || (0xAD == cur_byte) || (0xAE == cur_byte) ||
                                (0xAF == cur_byte) || (0xB0 == cur_byte) || (0xB6 == cur_byte) || (0xB7 == cur_byte) ||
                                (0xBA == cur_byte) || (0xB1 == cur_byte) || (0xB8 == cur_byte) || (0xBB == cur_byte)){
                            lidar_receive_package.buf[received_pos] = cur_byte;
                            received_pos++;
                        }  
                    }else if(0x54 == lidar_receive_package.buf[0]){    //54 2c
                        if(0x2C == cur_byte){          
                            lidar_receive_package.buf[received_pos] = cur_byte;
                            received_pos++;
                        }else{
                            received_pos = 0;
                        }
                    }else if(0xA5 == lidar_receive_package.buf[0]){     //A5 AB
                        if(0xAB == cur_byte){        
                            lidar_receive_package.buf[received_pos] = cur_byte;
                            received_pos++;
                        }else {
                            received_pos = 0;
                        }
                    }else{
                        received_pos = 0;
                    }
                    break;
                }
                case 2:{
                    if((0x55 == lidar_receive_package.buf[0]) && (0xAA != lidar_receive_package.buf[1])){     //0x55 0xXX downboard info
                        if(0 != cur_byte){
                            lidar_receive_package.buf[received_pos] = cur_byte;
                            received_pos++;
                            received_package_size = cur_byte + 4;       //3byte head + 1byte crc
                        }else{
                            received_pos = 0;
                        }
                    }else if((0xA5 == lidar_receive_package.buf[0]) && (0xAB == lidar_receive_package.buf[1])){  //0xA5 0xAB upboard info 
                        if(0 != cur_byte){
                            lidar_receive_package.buf[received_pos] = cur_byte;
                            received_pos++;
                        }else{
                            received_pos = 0;
                        }
                    }else if((0x55 == lidar_receive_package.buf[0]) && (0xAA == lidar_receive_package.buf[1])){   //0x55 0xAA pointcloud 
                        if( (((LidarProtocol::PROTOCOL_MODEL_NORMAL_NO_QUALITY >> 8)& 0xFF) == cur_byte) ||
                            (((LidarProtocol::PROTOCOL_MODEL_NORMAL_HAS_QUALITY >> 8)& 0xFF) == cur_byte) ||
                            (((LidarProtocol::PROTOCOL_MODEL_YW_HAS_QUALITY >> 8)& 0xFF) == cur_byte) ||
                            (((LidarProtocol::PROTOCOL_MODEL_TM21_HAS_QUAILIY >> 8)& 0xFF) == cur_byte) ||
                            (((LidarProtocol::PROTOCOL_MODEL_ERROR_FAULT >> 8)& 0xFF) == cur_byte)   ){
                            lidar_model_code = (int)(cur_byte << 8);
                            lidar_receive_package.buf[received_pos] = cur_byte;
                            received_pos++;
                        }else{
                            received_pos = 0;
                        }
                    }else if((0x54 == lidar_receive_package.buf[0]) && (0x2C == lidar_receive_package.buf[1])){    //0x54 0x2C pointcloud_ld
                        lidar_receive_package.buf[received_pos] = cur_byte;
                        received_pos++;
                    }else{
                        received_pos = 0;
                    }
                    break;
                }
                case 3:{
                    if((0x55 == lidar_receive_package.buf[0]) && (0xAA != lidar_receive_package.buf[1])){       //0x55 0xXX downboard info
                        lidar_receive_package.buf[received_pos] = cur_byte;
                        received_pos++;
                    }else if((0xA5 == lidar_receive_package.buf[0]) && (0xAB == lidar_receive_package.buf[1])){  //0xA5 0xAB upboard info 
                        if(0 != cur_byte){
                            lidar_receive_package.buf[received_pos] = cur_byte;
                            received_pos++;
                            received_package_size = cur_byte + 5;       //4byte head + 1byte crc
                        }else{
                            received_pos = 0;
                        }
                    }else if((0x55 == lidar_receive_package.buf[0]) && (0xAA == lidar_receive_package.buf[1])){
                        if( ((LidarProtocol::PROTOCOL_MODEL_NORMAL_NO_QUALITY & 0xFF) == cur_byte) ||
                            ((LidarProtocol::PROTOCOL_MODEL_NORMAL_HAS_QUALITY & 0xFF) == cur_byte) ||
                            ((LidarProtocol::PROTOCOL_MODEL_YW_HAS_QUALITY & 0xFF) == cur_byte) ||
                            ((LidarProtocol::PROTOCOL_MODEL_TM21_HAS_QUAILIY & 0xFF) == cur_byte) ||
                            ((LidarProtocol::PROTOCOL_MODEL_ERROR_FAULT & 0xFF) == cur_byte)){
                            lidar_model_code |= cur_byte;
                            received_package_size = GET_LIDAR_DATA_SIZE(lidar_model_code, protocol_070c_with_raw_flag);
                            lidar_receive_package.buf[received_pos] = cur_byte;
                            received_pos++;
                        }else{
                            received_pos = 0;
                        }
                    }else if((0x54 == lidar_receive_package.buf[0]) && (0x2C == lidar_receive_package.buf[1])){   //0x54 0x2C pointcloud_ld
                        lidar_model_code = LidarProtocol::PROTOCOL_MODEL_LD_HAS_QUAILIY;
                        received_package_size = GET_LIDAR_DATA_SIZE(lidar_model_code, protocol_070c_with_raw_flag);
                        lidar_receive_package.buf[received_pos] = cur_byte;
                        received_pos++;
                    }else{
                        received_pos = 0;
                    }
                    break;
                }
                default:{   
                    if(0 == received_package_size){                   
                        received_pos = 0;
                        break;
                    }
                    if(received_pos >= received_package_size - 1){   
                        lidar_receive_package.buf[received_pos] = cur_byte;

                        if( ((0x55 == lidar_receive_package.buf[0]) && (0xAA != lidar_receive_package.buf[1])) ||
                            ((0xA5 == lidar_receive_package.buf[0]) && (0xAB == lidar_receive_package.buf[1]))  ){     //upboard and downboard info 
                            lidar_info_unpack(&lidar_receive_package,received_package_size);
                        }else if((0x55 == lidar_receive_package.buf[0]) && (0xAA == lidar_receive_package.buf[1])){     //some points 
                            switch(lidar_model_code){
                                case LidarProtocol::PROTOCOL_MODEL_NORMAL_NO_QUALITY:{
                                    lidar_pointcloud_normal_no_quality_unpack(&lidar_receive_package);
                                    break;
                                }
                                case LidarProtocol::PROTOCOL_MODEL_NORMAL_HAS_QUALITY:{
                                    lidar_pointcloud_normal_has_quality_unpack(&lidar_receive_package);
                                    break;
                                }
                                case LidarProtocol::PROTOCOL_MODEL_YW_HAS_QUALITY:{
                                    if (protocol_070c_with_raw_flag){
                                        lidar_pointcloud_yw_has_quality_with_raw_unpack(&lidar_receive_package);
                                    }else{
                                        lidar_pointcloud_yw_has_quality_unpack(&lidar_receive_package);
                                    }
                                    break;
                                }
                                case LidarProtocol::PROTOCOL_MODEL_TM21_HAS_QUAILIY:{
                                    lidar_pointcloud_tm21_has_quality_unpack(&lidar_receive_package);
                                    break;
                                }
                                case LidarProtocol::PROTOCOL_MODEL_ERROR_FAULT:{
                                    lidar_pointcloud_errorcode_unpack(&lidar_receive_package);
                                    break;
                                }
                                default:{
                                    break;
                                }
                            }
                        }else if((0x54 == lidar_receive_package.buf[0]) && (0x2C == lidar_receive_package.buf[1])){
                            switch(lidar_model_code){
                                case LidarProtocol::PROTOCOL_MODEL_LD_HAS_QUAILIY:{
                                    lidar_pointcloud_ld_has_quality_unpack(&lidar_receive_package);
                                    break;
                                }
                                default:{
                                    break;
                                }
                            }
                        }

                        //clear cache
                        memset((char *)lidar_receive_package.buf,0x00,sizeof(lidar_receive_package_t));
                        received_package_size = 0;
                        received_pos = 0;
                        break;
                    }else{
                        lidar_receive_package.buf[received_pos] = cur_byte;
                        received_pos++;
                    }
                    break;
                }
            }
        }
    }

    /**
    * @Function: lidar_info_unpack
    * @Description: lidar info unpack 
    * @Return: void
    * @param {lidar_receive_package_t} *pack
    * @param {int} pack_size
    */
    void lidar_info_unpack(lidar_receive_package_t *pack, int pack_size){
        if((0x55 == pack->buf[0]) && (0xAA != pack->buf[1])){       //downboard 
            //calc acc 
            uint8_t acc_value = acc_checksum(pack->buf, pack_size - 1);
            if(acc_value != pack->buf[pack_size - 1]){
                return;
            }
            switch (pack->downboard_info.package_cmd) {
                case 0xAB:{
                    std::string str;
                    lidar_boot_header_info.downBoard_Model = str.assign((char *)pack->downboard_info.package_data,
                                                                            pack->downboard_info.package_length);

                    break;
                }
                case 0xAC:{
                    std::string str;
                    lidar_boot_header_info.downBoard_HardVersion = str.assign((char *)pack->downboard_info.package_data,
                                                                                pack->downboard_info.package_length);
                    break;
                }
                case 0xAD:{
                    std::string str;
                    lidar_boot_header_info.downBoard_SoftVersion = str.assign((char *)pack->downboard_info.package_data,
                                                                                pack->downboard_info.package_length);
                    break;
                }
                case 0xAE:{
                    lidar_boot_header_info.downBoard_ID = hex_bytes_to_string((char *)pack->downboard_info.package_data,
                                                                                pack->downboard_info.package_length);
                    break;
                }
                case 0xAF:{
                    std::string str;
                    lidar_boot_header_info.downBoard_Date = str.assign((char *)pack->downboard_info.package_data,
                                                                            pack->downboard_info.package_length);                                                         
                    break;
                }
                case 0xB0:{
                    std::string str;
                    lidar_boot_header_info.downBoard_InputVoltage = str.assign((char *)pack->downboard_info.package_data,
                                                                                    pack->downboard_info.package_length);
                    lidar_boot_header_info.downBoard_InputVoltage.insert(1,1,'.');
                    break;
                }
                case 0xB6:{
                    lidar_boot_header_info.downBoard_UID = hex_bytes_to_string((char *)pack->downboard_info.package_data,
                                                                                    pack->downboard_info.package_length);
                    break;
                }
                case 0xBA:{
                    lidar_boot_header_info.upBoard_VBD = pack->downboard_info.package_data[0];
                    lidar_boot_header_info.upBoard_TDC = pack->downboard_info.package_data[1];
                    lidar_boot_header_info.upBoard_Temperature = pack->downboard_info.package_data[2];
                    break;
                }
                case 0xB7:{
                    std::string str;
                    lidar_boot_header_info.downBoard_BuildTime = str.assign((char *)pack->downboard_info.package_data,
                                                                                pack->downboard_info.package_length);
                    break;
                }
                case 0xB1:{
                    std::string str;
                    lidar_boot_header_info.downBoard_MCUTemperature = str.assign((char *)pack->downboard_info.package_data,pack->downboard_info.package_length);
                    break;
                }
                case 0xBB:{
                    std::string str;
                    lidar_boot_header_info.downBoard_SnNumber = str.assign((char *)pack->downboard_info.package_data,pack->downboard_info.package_length);
                    break;
                }
                case 0xB8:{
                    uint16_t vref_adc = (pack->downboard_info.package_data[0] + pack->downboard_info.package_data[1]*256);
                    uint16_t ir_adc = (pack->downboard_info.package_data[2] + pack->downboard_info.package_data[3]*256);
                    lidar_boot_header_info.downBoard_IRMaxVoltage = 1200*ir_adc/vref_adc;
                    lidar_boot_head_received_finished_flag = true;      //lidar received all head 
                    break;
                }
                default:{
                    break;
                }
            }
        }else if((0xA5 == pack->buf[0]) && (0xAB == pack->buf[1])){     //0xA5 0xAB upboard info 
            //calc acc 
            uint8_t acc_value = acc_checksum(pack->buf, pack_size - 1);
            if(acc_value != pack->buf[pack_size - 1]){
                return;
            }
            switch (pack->upboard_info.package_cmd){
                case 0x13:{
                    lidar_boot_header_info.upBoard_ID = hex_bytes_to_string((char *)pack->upboard_info.package_data,
                                                                                pack->upboard_info.package_length);
                    break;
                }
                case 0x14:{
                    std::string str;
                    lidar_boot_header_info.upBoard_Date = str.assign((char *)pack->upboard_info.package_data,
                                                                                pack->upboard_info.package_length);
                    break;
                }
                case 0x16:{
                    std::string str;
                    lidar_boot_header_info.upBoard_Model = str.assign((char *)pack->upboard_info.package_data,
                                                                                pack->upboard_info.package_length);
                    break;
                }
                case 0x17:{
                    std::string str;
                    lidar_boot_header_info.upBoard_HardVersion = str.assign((char *)pack->upboard_info.package_data,
                                                                                pack->upboard_info.package_length);
                    break;
                }
                case 0x18:{
                    std::string str;
                    lidar_boot_header_info.upBoard_SoftVersion = str.assign((char *)pack->upboard_info.package_data,
                                                                                pack->upboard_info.package_length);
                    break;
                }
                case 0x19:{
                    lidar_boot_header_info.upBoard_UID = hex_bytes_to_string((char *)pack->upboard_info.package_data,
                                                                                pack->upboard_info.package_length);
                    break;
                }
                case 0x1A:{
                    std::string str;
                    lidar_boot_header_info.upBoard_BuildTime = str.assign((char *)pack->upboard_info.package_data,
                                                                                pack->upboard_info.package_length);
                    break;
                }
                default:{
                    break;
                }
            }
        }
    }

    /**
    * @Function: lidar_pointcloud_normal_no_quality_unpack
    * @Description: normal no quality 
    * @Return: void 
    * @param {lidar_receive_package_t} *pack
    */
    void lidar_pointcloud_normal_no_quality_unpack(lidar_receive_package_t *pack){
        //crc
        uint16_t crc_get = pack->normal_no_quality.package_checksum;
        uint16_t crc_calc = crc16_checksum(pack->buf, sizeof(lidar_normal_no_quality_package_t)-2);
        if(crc_calc != crc_get){
            return;
        }
        //calc angle 
        double angle_differ = 0.0;
        uint16_t first_angle = pack->normal_no_quality.package_first_angle - 0xA000;
        uint16_t last_angle =  pack->normal_no_quality.package_last_angle - 0xA000;
        if(last_angle >= first_angle){      //start angle > end angle
            angle_differ = (static_cast<double>(last_angle - first_angle)/static_cast<double>(NORMAL_NO_QUALITY_PACK_MAX_POINTS - 1))/64.0;
        }else {
            angle_differ = (static_cast<double>(last_angle + (360*64) - first_angle)/static_cast<double>(NORMAL_NO_QUALITY_PACK_MAX_POINTS - 1))/64.0;
        }
        double first_angle_true = static_cast<double>(first_angle)/64.0;
        //calc points info 
        for(int j = 0; j<NORMAL_NO_QUALITY_PACK_MAX_POINTS; j++){
            lidar_scan_point_t  point_raw_single;
            //point angle 
            point_raw_single.angle = first_angle_true + angle_differ*j;
            if(point_raw_single.angle >= 360.0){
                point_raw_single.angle -= 360.0;
            }
            //point distance
            uint16_t cur_distance_u16 = pack->normal_no_quality.points[j].distance;
            if((cur_distance_u16 & 0x8000) != 0){
                point_raw_single.distance = 0;
            }else {
                point_raw_single.distance = static_cast<double>(cur_distance_u16);
            }
            //distance raw
            point_raw_single.distance_raw = 0;
            //quality
            point_raw_single.intensity = 0;
            //add cache 
            lidar_points_cache.push_back(point_raw_single);
            //check is period
            if(point_raw_single.angle < lidar_last_angle){
                //mutex
                std::lock_guard<std::mutex> lock(lidar_mtx);
                //update 
                lidar_point_raw_period_cache.intensity_flag = false;
                lidar_point_raw_period_cache.speed = static_cast<double>(pack->normal_no_quality.package_speed)/64.f;
                lidar_point_raw_period_cache.model_code = LidarProtocol::PROTOCOL_MODEL_NORMAL_NO_QUALITY;
                lidar_point_raw_period_cache.error_code = LidarProtocol::ERROR_CODE_NONE;
                lidar_point_raw_period_cache.points = lidar_points_cache;
                if(lidar_interface_function->get_timestamp != nullptr){
                    lidar_point_raw_period_cache.timestamp_start = lidar_point_raw_period_cache.timestamp_stop;
                    lidar_point_raw_period_cache.timestamp_stop = lidar_interface_function->get_timestamp();
                }

                if(lidar_rawdata_output_function != nullptr){
                    lidar_rawdata_output_function(lidar_point_raw_period_cache);
                }    

                lidar_points_cache.clear();
            }
            lidar_last_angle = point_raw_single.angle;
        }
    }

    /**
    * @Function: lidar_pointcloud_normal_has_quality_unpack
    * @Description: normal has quality 
    * @Return: void
    * @param {lidar_receive_package_t} *pack
    */
    void lidar_pointcloud_normal_has_quality_unpack(lidar_receive_package_t *pack){
        //crc
        uint16_t crc_get = pack->normal_has_quality.package_checksum;
        uint16_t crc_calc = crc16_checksum(pack->buf, sizeof(lidar_normal_has_quality_package_t)-2);
        if(crc_calc != crc_get){
            return;
        }
        //calc angle 
        double angle_differ = 0.0;
        uint16_t first_angle = pack->normal_has_quality.package_first_angle - 0xA000;
        uint16_t last_angle =  pack->normal_has_quality.package_last_angle - 0xA000;
        if(last_angle >= first_angle){      //start angle > end angle
            angle_differ = (static_cast<double>(last_angle - first_angle)/static_cast<double>(NORMAL_HAS_QUALITY_PACK_MAX_POINTS - 1))/64.0;
        }else {
            angle_differ = (static_cast<double>(last_angle + (360*64) - first_angle)/static_cast<double>(NORMAL_HAS_QUALITY_PACK_MAX_POINTS - 1))/64.0;
        }
        double first_angle_true = static_cast<double>(first_angle)/64.0;
        //calc points info 
        for(int j = 0; j<NORMAL_HAS_QUALITY_PACK_MAX_POINTS; j++){
            lidar_scan_point_t  point_raw_single;
            //point angle 
            point_raw_single.angle = first_angle_true + angle_differ*j;
            if(point_raw_single.angle >= 360.0){
                point_raw_single.angle -= 360.0;
            }
            //point distance
            uint16_t cur_distance_u16 = pack->normal_has_quality.points[j].distance;
            if((cur_distance_u16 & 0x8000) != 0){
                point_raw_single.distance = 0;
            }else {
                point_raw_single.distance = static_cast<double>(cur_distance_u16);
            }
            //distance raw
            point_raw_single.distance_raw = 0;
            //quality
            point_raw_single.intensity = static_cast<double>(pack->normal_has_quality.points[j].quality);
            //add cache 
            lidar_points_cache.push_back(point_raw_single);
            //check is period
            if(point_raw_single.angle < lidar_last_angle){
                //mutex
                std::lock_guard<std::mutex> lock(lidar_mtx);
                //update 
                lidar_point_raw_period_cache.intensity_flag = false;
                lidar_point_raw_period_cache.speed = static_cast<double>(pack->normal_has_quality.package_speed)/64.f;
                lidar_point_raw_period_cache.model_code = LidarProtocol::PROTOCOL_MODEL_NORMAL_HAS_QUALITY;
                lidar_point_raw_period_cache.error_code = LidarProtocol::ERROR_CODE_NONE;
                lidar_point_raw_period_cache.points = lidar_points_cache;
                if(lidar_interface_function->get_timestamp != nullptr){
                    lidar_point_raw_period_cache.timestamp_start = lidar_point_raw_period_cache.timestamp_stop;
                    lidar_point_raw_period_cache.timestamp_stop = lidar_interface_function->get_timestamp();
                }

                if(lidar_rawdata_output_function != nullptr){
                    lidar_rawdata_output_function(lidar_point_raw_period_cache);
                }  
                lidar_points_cache.clear();
            }
            lidar_last_angle = point_raw_single.angle;
        }
    }

    /**
    * @Function: lidar_pointcloud_yw_has_quality_unpack
    * @Description: yw has quality 
    * @Return: void
    * @param {lidar_receive_package_t} *pack
    */
    void lidar_pointcloud_yw_has_quality_unpack(lidar_receive_package_t *pack){
        //crc
        uint16_t crc_get = pack->yw_has_quality.package_checksum;
        uint16_t crc_calc = crc16_checksum(pack->buf, sizeof(lidar_yw_has_quality_package_t)-2);
        if(crc_calc != crc_get){ 
            return;
        }
        //calc angle 
        double angle_differ = 0.0;
        uint16_t first_angle = pack->yw_has_quality.package_first_angle - 0xA000;
        uint16_t last_angle =  pack->yw_has_quality.package_last_angle - 0xA000;
        if(last_angle >= first_angle){      //start angle > end angle
            angle_differ = (static_cast<double>(last_angle - first_angle)/static_cast<double>(YW_HAS_QUALITY_PACK_MAX_POINTS - 1))/64.0;
        }else {
            angle_differ = (static_cast<double>(last_angle + (360*64) - first_angle)/static_cast<double>(YW_HAS_QUALITY_PACK_MAX_POINTS - 1))/64.0;
        }
        double first_angle_true = static_cast<double>(first_angle)/64.0;
        //calc points info 
        for(int j = 0; j<YW_HAS_QUALITY_PACK_MAX_POINTS; j++){
            lidar_scan_point_t  point_raw_single;
            //point angle 
            point_raw_single.angle = first_angle_true + angle_differ*j;
            if(point_raw_single.angle >= 360.0){
                point_raw_single.angle -= 360.0;
            }
            //point distance
            uint16_t cur_distance_u16 = pack->yw_has_quality.points[j].distance;
            if((cur_distance_u16 & 0x8000) != 0){
                point_raw_single.distance = 0;
            }else {
                point_raw_single.distance = static_cast<double>(cur_distance_u16);
            }
            //distance raw
            point_raw_single.distance_raw = 0;
            //quality
            point_raw_single.intensity = static_cast<double>(pack->yw_has_quality.points[j].quality);
            //add cache 
            lidar_points_cache.push_back(point_raw_single);
            //check is period
            if(point_raw_single.angle < lidar_last_angle){
                //mutex
                std::lock_guard<std::mutex> lock(lidar_mtx);
                //update 
                lidar_point_raw_period_cache.intensity_flag = false;
                lidar_point_raw_period_cache.speed = static_cast<double>(pack->yw_has_quality.package_speed)/64.f;
                lidar_point_raw_period_cache.model_code = LidarProtocol::PROTOCOL_MODEL_YW_HAS_QUALITY;
                lidar_point_raw_period_cache.error_code = LidarProtocol::ERROR_CODE_NONE;
                lidar_point_raw_period_cache.points =  lidar_points_cache;
                if(lidar_interface_function->get_timestamp != nullptr){
                    lidar_point_raw_period_cache.timestamp_start = lidar_point_raw_period_cache.timestamp_stop;
                    lidar_point_raw_period_cache.timestamp_stop = lidar_interface_function->get_timestamp();
                }

                
                
                if(lidar_rawdata_output_function != nullptr){
                    lidar_rawdata_output_function(lidar_point_raw_period_cache);
                } 

                lidar_points_cache.clear();
            }
            lidar_last_angle = point_raw_single.angle;
        }
    }

    /**
    * @Function: lidar_pointcloud_yw_has_quality_with_raw_unpack
    * @Description: yw has quality with raw distance
    * @Return: void
    * @param {lidar_receive_package_t} *pack
    */
    void lidar_pointcloud_yw_has_quality_with_raw_unpack(lidar_receive_package_t *pack){
        //crc
        uint16_t crc_get = pack->yw_has_quality_with_raw.package_checksum;
        uint16_t crc_calc = crc16_checksum(pack->buf, sizeof(lidar_yw_has_quality_with_raw_package_t)-2);
        if(crc_calc != crc_get){
            return;
        }
        //calc angle
        double angle_differ = 0.0;
        uint16_t first_angle = pack->yw_has_quality_with_raw.package_first_angle - 0xA000;
        uint16_t last_angle =  pack->yw_has_quality_with_raw.package_last_angle - 0xA000;
        if(last_angle >= first_angle){      //start angle > end angle
            angle_differ = (static_cast<double>(last_angle - first_angle)/static_cast<double>(YW_HAS_QUALITY_WITH_RAW_PACK_MAX_POINTS - 1))/64.0;
        }else {
            angle_differ = (static_cast<double>(last_angle + (360*64) - first_angle)/static_cast<double>(YW_HAS_QUALITY_WITH_RAW_PACK_MAX_POINTS - 1))/64.0;
        }
        double first_angle_true = static_cast<double>(first_angle)/64.0;
        //calc points info
        for(int j = 0; j<YW_HAS_QUALITY_WITH_RAW_PACK_MAX_POINTS; j++){
            lidar_scan_point_t  point_raw_single;
            //point angle
            point_raw_single.angle = first_angle_true + angle_differ*j;
            if(point_raw_single.angle >= 360.0){
                point_raw_single.angle -= 360.0;
            }
            //point distance
            uint16_t cur_distance_u16 = pack->yw_has_quality_with_raw.points[j].distance;
            if((cur_distance_u16 & 0x8000) != 0){
                point_raw_single.distance = 0;
            }else {
                point_raw_single.distance = static_cast<double>(cur_distance_u16);
            }
            //distance raw
            point_raw_single.distance_raw = pack->yw_has_quality_with_raw.points[j].distance_raw;
            //quality
            point_raw_single.intensity = static_cast<double>(pack->yw_has_quality_with_raw.points[j].quality);
            //add cache
            lidar_points_cache.push_back(point_raw_single);
            //check is period
            if(point_raw_single.angle < lidar_last_angle){
                //mutex
                std::lock_guard<std::mutex> lock(lidar_mtx);
                //update
                lidar_point_raw_period_cache.intensity_flag = false;
                lidar_point_raw_period_cache.speed = static_cast<double>(pack->yw_has_quality_with_raw.package_speed)/64.f;
                lidar_point_raw_period_cache.model_code = LidarProtocol::PROTOCOL_MODEL_YW_HAS_QUALITY;
                lidar_point_raw_period_cache.error_code = LidarProtocol::ERROR_CODE_NONE;
                lidar_point_raw_period_cache.points =  lidar_points_cache;
                if(lidar_interface_function->get_timestamp != nullptr){
                    lidar_point_raw_period_cache.timestamp_start = lidar_point_raw_period_cache.timestamp_stop;
                    lidar_point_raw_period_cache.timestamp_stop = lidar_interface_function->get_timestamp();
                }
                if(lidar_rawdata_output_function != nullptr){
                    lidar_rawdata_output_function(lidar_point_raw_period_cache);
                }

                lidar_points_cache.clear();
            }
            lidar_last_angle = point_raw_single.angle;
        }
    }


    /**
    * @Function: lidar_pointcloud_ld_has_quality_unpack
    * @Description: ld has quality 
    * @Return: void 
    * @param {lidar_receive_package_t} *pack
    */
    void lidar_pointcloud_ld_has_quality_unpack(lidar_receive_package_t *pack){
        //crc
        uint8_t crc_get = pack->ld_has_quality.package_checksum;
        uint8_t crc_calc = crc8_checksum(pack->buf, sizeof(lidar_ld_has_quality_package_t)-1);
        if(crc_calc != crc_get){
            return;
        }
        //calc angle 
        double angle_differ = 0.0;
        uint16_t first_angle = pack->ld_has_quality.package_first_angle;
        uint16_t last_angle =  pack->ld_has_quality.package_last_angle;
        if(last_angle >= first_angle){      //start angle > end angle
            angle_differ = (static_cast<double>(last_angle - first_angle)/static_cast<double>(LD_HAS_QUALITY_PACK_MAX_POINTS - 1))/100.0;
        }else {
            angle_differ = (static_cast<double>(last_angle + (360*100.f) - first_angle)/static_cast<double>(LD_HAS_QUALITY_PACK_MAX_POINTS - 1))/100.0;
        }
        double first_angle_true = static_cast<double>(first_angle)/100.0;
        //calc points info 
        for(int j = 0; j<LD_HAS_QUALITY_PACK_MAX_POINTS; j++){
            lidar_scan_point_t  point_raw_single;
            //point angle 
            point_raw_single.angle = first_angle_true + angle_differ*j;
            if(point_raw_single.angle >= 360.0){
                point_raw_single.angle -= 360.0;
            }
            //distance raw
            point_raw_single.distance_raw = 0;
            //point distance
            uint16_t cur_distance_u16 = pack->ld_has_quality.points[j].distance;
            if((cur_distance_u16 & 0x8000) != 0){
                point_raw_single.distance = 0;
            }else {
                point_raw_single.distance = static_cast<double>(cur_distance_u16);
            }
            //quality
            point_raw_single.intensity = static_cast<double>(pack->ld_has_quality.points[j].quality);
            //add cache 
            lidar_points_cache.push_back(point_raw_single);
            //check is period
            if(point_raw_single.angle < lidar_last_angle){
                //mutex
                std::lock_guard<std::mutex> lock(lidar_mtx);
                //update 
                lidar_point_raw_period_cache.intensity_flag = false;
                lidar_point_raw_period_cache.speed = static_cast<double>(pack->ld_has_quality.package_speed) / 360.f * 60.f;
                lidar_point_raw_period_cache.model_code = LidarProtocol::PROTOCOL_MODEL_LD_HAS_QUAILIY;
                lidar_point_raw_period_cache.error_code = LidarProtocol::ERROR_CODE_NONE;
                lidar_point_raw_period_cache.points =  lidar_points_cache;
                if(lidar_interface_function->get_timestamp != nullptr){
                    lidar_point_raw_period_cache.timestamp_start = lidar_point_raw_period_cache.timestamp_stop;
                    lidar_point_raw_period_cache.timestamp_stop = lidar_interface_function->get_timestamp();
                }

                if(lidar_rawdata_output_function != nullptr){
                    lidar_rawdata_output_function(lidar_point_raw_period_cache);
                }

                lidar_points_cache.clear();
            }
            lidar_last_angle = point_raw_single.angle;
        }
    }

    /**
    * @Function: lidar_pointcloud_tm21_has_quality_unpack
    * @Description: tim21 has quality
    * @Return: void
    * @param {lidar_receive_package_t} *pack
    */
    void lidar_pointcloud_tm21_has_quality_unpack(lidar_receive_package_t *pack){
        //crc
        uint16_t crc_get = pack->tm21_has_quality.package_checksum;
        uint16_t crc_calc = crc16_checksum(pack->buf, sizeof(lidar_tm21_has_quality_package_t)-2);
        if(crc_calc != crc_get){
            return;
        }
        //calc angle
        double angle_differ = 0.0;
        uint16_t first_angle = pack->tm21_has_quality.package_first_angle - 0xA000;
        uint16_t last_angle =  pack->tm21_has_quality.package_last_angle - 0xA000;
        if(last_angle >= first_angle){      //start angle > end angle
            angle_differ = (static_cast<double>(last_angle - first_angle)/static_cast<double>(TM21_HAS_QUALITY_PACK_MAX_POINTS - 1))/64.0;
        }else {
            angle_differ = (static_cast<double>(last_angle + (360*64) - first_angle)/static_cast<double>(TM21_HAS_QUALITY_PACK_MAX_POINTS - 1))/64.0;
        }
        double first_angle_true = static_cast<double>(first_angle)/64.0;
        //calc points info
        for(int j = 0; j<TM21_HAS_QUALITY_PACK_MAX_POINTS; j++){
            lidar_scan_point_t  point_raw_single;
            //point angle
            point_raw_single.angle = first_angle_true + angle_differ*j;
            if(point_raw_single.angle >= 360.0){
                point_raw_single.angle -= 360.0;
            }
            //point distance
            uint16_t cur_distance_u16 = pack->tm21_has_quality.points[j].distance;
            if((cur_distance_u16 & 0x8000) != 0){
                point_raw_single.distance = 0;
            }else {
                point_raw_single.distance = static_cast<double>(cur_distance_u16);
            }
            //distance raw
            point_raw_single.distance_raw = 0;
            //quality
            point_raw_single.intensity = static_cast<double>(pack->tm21_has_quality.points[j].quality);
            //add cache
            lidar_points_cache.push_back(point_raw_single);
            //check is period
            if(point_raw_single.angle < lidar_last_angle){
                //mutex
                std::lock_guard<std::mutex> lock(lidar_mtx);
                //update
                lidar_point_raw_period_cache.intensity_flag = false;
                lidar_point_raw_period_cache.speed = static_cast<double>(pack->tm21_has_quality.package_speed)/64.f;
                lidar_point_raw_period_cache.model_code = LidarProtocol::PROTOCOL_MODEL_TM21_HAS_QUAILIY;
                lidar_point_raw_period_cache.error_code = LidarProtocol::ERROR_CODE_NONE;
                lidar_point_raw_period_cache.points = lidar_points_cache;
                if(lidar_interface_function->get_timestamp != nullptr){
                    lidar_point_raw_period_cache.timestamp_start = lidar_point_raw_period_cache.timestamp_stop;
                    lidar_point_raw_period_cache.timestamp_stop = lidar_interface_function->get_timestamp();
                }

                if(lidar_rawdata_output_function != nullptr){
                    lidar_rawdata_output_function(lidar_point_raw_period_cache);
                }
                lidar_points_cache.clear();
            }
            lidar_last_angle = point_raw_single.angle;
        }
    }

    /**
    * @Function: lidar_pointcloud_errorcode_unpack
    * @Description: error code unpack 
    * @Return: void 
    * @param {lidar_receive_package_t} *pack
    */
    void lidar_pointcloud_errorcode_unpack(lidar_receive_package_t *pack){
        //mutex
        std::lock_guard<std::mutex> lock(lidar_mtx);
        //calc acc value 
        uint8_t add_sum_value = acc_checksum(pack->buf,sizeof(lidar_errorcode_package_t) - 1);
        if(add_sum_value != pack->buf[sizeof(lidar_errorcode_package_t) - 1]){
            return;
        }
        //get errorcode
        lidar_point_raw_period_cache.intensity_flag = false;
        lidar_point_raw_period_cache.speed = 0;
        lidar_point_raw_period_cache.model_code = LidarProtocol::PROTOCOL_MODEL_ERROR_FAULT;
        lidar_point_raw_period_cache.error_code = pack->error_code.package_errorcode;
        lidar_point_raw_period_cache.points.clear();
        if(lidar_interface_function->get_timestamp != nullptr){
            lidar_point_raw_period_cache.timestamp_start = lidar_point_raw_period_cache.timestamp_stop;
            lidar_point_raw_period_cache.timestamp_stop = lidar_interface_function->get_timestamp();
        }

        //callback 
        if(lidar_rawdata_output_function != nullptr){
            lidar_rawdata_output_function(lidar_point_raw_period_cache);
        }
    }

    /**
    * @Function: acc_checksum
    * @Description: calc acc 
    * @Return: uint8_t 
    * @param {uint8_t} *data
    * @param {int} length
    */
    uint8_t acc_checksum(const uint8_t *data, int length){
        uint8_t check_sum_value = 0;

        for (int i = 0; i < length; i++) {
            check_sum_value += data[i];
        }
        return check_sum_value;
    }

    /**
    * @Function: crc16_checksum
    * @Description: crc16 calc 
    * @Return: uint16_t
    * @param {uint8_t} *data
    * @param {int} length
    */
    uint16_t crc16_checksum(const uint8_t *data, int length){
        if (length % 2 != 0) {
            return 0;
        }
        uint32_t temp_crc_value = 0;
        const uint16_t *words = (const uint16_t *)data;
        int word_count = length / 2;

        for (int i = 0; i < word_count; i++) {
            temp_crc_value = (temp_crc_value << 1) + words[i];
        }
        temp_crc_value = (temp_crc_value & 0x7FFF) + (temp_crc_value >> 15);
        return temp_crc_value & 0x7FFF;
    }

    /**
    * @Function: crc8_checksum
    * @Description: crc8 check 
    * @Return: uint8_t
    * @param {uint8_t} *data
    * @param {int} length
    */
    uint8_t crc8_checksum(const uint8_t *data, int length){
        uint8_t crc = 0;
        for (uint16_t i = 0; i < length; i++){
            crc = ld_crc_table[(crc ^ *data++) & 0xff];
        }
        return crc;
    }

    /**
    * @Function: hex_bytes_to_string
    * @Description: hex bytes to string 
    * @Return: std::string 
    * @param {char*} data
    * @param {size_t} length
    */
    std::string hex_bytes_to_string(const char* data, size_t length){
        std::stringstream ss;
        ss << std::hex << std::uppercase << std::setfill('0');
        for (size_t i = 0; i < length; ++i) {
            ss << std::setw(2) << static_cast<unsigned>(static_cast<unsigned char>(data[i]));
        }
        return ss.str();
    }
};

LidarProtocol::LidarProtocol() : _impl(new LidarProtocolImpl){
}

LidarProtocol::~LidarProtocol(){
    lidar_protocol_unregister();    //close the thread 
    delete _impl;                   //delete the point
}

/**
 * @Function: lidar_protocol_register
 * @Description: lidar transmit register 
 * @Return: void
 * @param {lidar_interface_t*} api
 * @param {protocol_rawdata_output_callback} rawdata_output
 */
void LidarProtocol::lidar_protocol_register(lidar_interface_t* api, protocol_rawdata_output_callback rawdata_output, bool protocol_070c_raw_flag){
  //register the io function 
  _impl->lidar_interface_function = api;
  _impl->lidar_rawdata_output_function = rawdata_output;
  _impl->protocol_070c_with_raw_flag = protocol_070c_raw_flag;

  _impl->thread_finished_flag.store(false);
  _impl->thread_running_flag.store(true);

  std::this_thread::sleep_for(std::chrono::milliseconds(10));

  //open thread 
  std::thread readThread([this]() {
      while(_impl->thread_running_flag.load()) {
          if((_impl->lidar_interface_function != nullptr) && (_impl->lidar_interface_function->transmit.read != nullptr)){
              _impl->lidar_transmit_received_data.length = _impl->lidar_interface_function->transmit.read(_impl->lidar_transmit_received_data.buf, LIDAR_TRANSMIT_RECEIVED_BUF);
              if(_impl->lidar_transmit_received_data.length > 0){
                  //pointcloud unpack
                  _impl->lidar_pointcloud_data_unpack(_impl->lidar_transmit_received_data.buf, _impl->lidar_transmit_received_data.length); 
              }
          }
          //delay 
          std::this_thread::sleep_for(std::chrono::milliseconds(2)); 
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      _impl->thread_finished_flag.store(true);
  });
  readThread.detach();
}

/**
 * @Function: lidar_protocol_unregister
 * @Description: lidar transmit unregister 
 * @Return: void 
 */
void LidarProtocol::lidar_protocol_unregister(){
    //close the threand
    _impl->thread_running_flag.store(false);
    while (!_impl->thread_finished_flag.load()) {
       std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

/**
 * @Function: lidar_protocol_stop_scan
 * @Description: lidar stop scan and stop motor
 * @Return: bool 
 */
bool LidarProtocol::lidar_protocol_stop_scan(){
    return _impl->lidar_send_cmd(0x02, nullptr, 0);
}

/**
 * @Function: lidar_protocol_start_scan
 * @Description: lidar start scan and start motor
 * @Return: bool 
 */
bool LidarProtocol::lidar_protocol_start_scan(){
    return _impl->lidar_send_cmd(0x01, nullptr, 0);
}

/**
 * @Function: lidar_protocol_reset
 * @Description: reset lidar 
 * @Return: bool 
 */
bool LidarProtocol::lidar_protocol_reset(){
    return _impl->lidar_send_cmd(0x03, nullptr, 0);
}

/**
 * @Function: lidar_protocol_get_model
 * @Description: lidar get head model
 * @Return: 
 * @param {string} &model
 */
bool LidarProtocol::lidar_protocol_get_model(std::string &model){
    if(false == _impl->lidar_boot_head_received_finished_flag){
        return false;
    }
    model = _impl->lidar_boot_header_info.upBoard_Model;
    return true;
}

/**
 * @Function: lidar_protocol_get_down_soft_version
 * @Description: lidar get down head version`
 * @Return: bool
 * @param {string} &version
 */
bool LidarProtocol::lidar_protocol_get_down_soft_version(std::string &version){
    if(false == _impl->lidar_boot_head_received_finished_flag){
        return false;
    }
    version = _impl->lidar_boot_header_info.downBoard_SoftVersion;
    return true;
}

    /**
 * @Function: lidar_protocol_get_up_soft_version
 * @Description: lidar get up head version`
 * @Return: bool
 * @param {string} &version
 */
bool LidarProtocol::lidar_protocol_get_up_soft_version(std::string &version){
    if(false == _impl->lidar_boot_head_received_finished_flag){
        return false;
    }
    version = _impl->lidar_boot_header_info.upBoard_SoftVersion;
    return true;
}

}
