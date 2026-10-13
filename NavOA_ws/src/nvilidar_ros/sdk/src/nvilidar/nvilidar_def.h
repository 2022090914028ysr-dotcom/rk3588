#ifndef _NVILIDAR_DEF_H_
#define _NVILIDAR_DEF_H_

#include <stdint.h>
#include "nvilidar_protocol.h"
#include <string>


//======================================basic parameter============================================ 

//SDK version 
#define NVILIDAR_SDKVerision     "1.1.4"

//PI def
#ifndef M_PI
#define M_PI        3.14159265358979323846
#endif 

//other 
#define NVILIDAR_DEFAULT_TIMEOUT     2000    //default timeout 
#define NVILIDAR_POINT_TIMEOUT		 2000	 //one circle time  for example, the lidar speed is 10hz ,the timeout must smaller the 100ms


//lidar model  list 
typedef enum
{
	NVILIDAR_Unknow = 0,		//unknow lidar 
   	NVILIDAR_ROC300,			//lidar ROC300
	NVILIDAR_VP350,				//lidar VP350
   	NVILIDAR_Tail,
}LidarModelListEnumTypeDef;


//======================================other parameters============================================ 

//lidar current state 
struct Nvilidar_PackageStateTypeDef
{
	bool m_CommOpen;              	//serialport open flag 
	bool m_Scanning;                //lidar is scanning data 
	uint8_t last_device_byte;       //last byte 
};

//stored para for lidar
struct  Nvilidar_StoreConfigTypeDef
{
	uint8_t     isHasSensitive;         //has sensitive 
	uint16_t    aimSpeed;               //motor aim speed == x100
	uint32_t    samplingRate;           //sampling rate == x1
	int16_t     angleOffset;            //angle offset == x64
	uint8_t     tailingLevel;           //tailling level 0-max 20-min
	uint16_t    apdValue;				//apd value 
	uint16_t    qualityFilterThreshold;	//quality filter threshold
};

//lidar data info  
struct Nvilidar_DeviceInfo
{
	std::string m_SoftVer;				//software version
	std::string m_HardVer;				//hardware version
	std::string m_ProductName;			//product name 
	std::string m_SerialNum;			//serialnumber 
};

//lidar filter para --- lidar tail para 
typedef struct{
	bool   enable;
	int    level;
	bool   distance_limit_flag;
	int    distance_limit_value;
	int    neighbors;
}TailFilterPara;
//lidar filter para --- lidar sliding filter para 
typedef struct{
	bool   enable;          
	int    jump_threshold;  
	int    max_range;       
	bool   max_range_flag;  
	int    window;
}SlidingFilterPara;
//lidar filter para 
typedef struct{
	TailFilterPara  tail_filter;
	SlidingFilterPara  sliding_filter;
}FilterPara;

//lidar configure para
struct  Nvilidar_UserConfigTypeDef
{
	LidarModelListEnumTypeDef  lidar_model_name;	//lidar model name 

	std::string frame_id;				//ID
	std::string serialport_name;		//serialport name 
	int    		serialport_baud;		//serialport baudrate 
	std::string ip_addr;				//ip addr for net convert
	int    		lidar_udp_port;			//ip port for net convert
	int    		config_tcp_port;		//ip port for config net para 
	bool		auto_reconnect;			//auto reconnect 
    bool		reversion;				//add 180.0 
	bool		inverted;				//turn backwards(if it is true)
	double		angle_max;				//angle max value for lidar 
	double		angle_min;				//angle min value for lidar  
	double		range_max;				//measure distance max value for lidar  
	double		range_min;				//measure distance min value for lidar  
	double 		aim_speed;				//lidar aim speed   
	int			sampling_rate;			//sampling rate  
	bool		sensitive;				//is contain sensitive  
	int			tailing_level;			//tailling level  
	bool		apd_change_flag;		//is enable to change apd value  
	int			apd_value;				//default apd value 
	bool 		angle_offset_change_flag;  //is enable to change angle offset 
	double 		angle_offset;			//angle offset 

	std::string ignore_array_string;	//filter angle ,string,like ,
	std::vector<float> ignore_array;	//filter angle to array list 

	bool 		resolution_fixed;		//is good resolution  
	Nvilidar_DeviceInfo			deviceInfo;	//lidar info 
	Nvilidar_StoreConfigTypeDef	storePara;	//lidar needed to store  

	FilterPara	filter_para;			//lidar pointcloud filter para info 

	bool 		quality_threshold_change_flag;	//quality threshold change flag
	int 		quality_threshold;				//quality threshold value(less then this ,distance=0)
};

//lidar receive info typedef 
union Nvilidar_PackageBufTypeDef{
	uint8_t buf[1200];
	Nvilidar_Node_Package_Quality        pack_qua;
	Nvilidar_Node_Package_No_Quality     pack_no_qua;
};

//lidar package info 
typedef struct 
{
	uint16_t packageIndex;         //angle 0 index 
	Nvilidar_PackageBufTypeDef  packageBuffer;    //pacage buffer data 
	bool     packageErrFlag;       //package error flag
	uint16_t packageCheckSumGet;   //checksum get from protocol 
	uint16_t packageCheckSumCalc;  //checksum calc by ros
	uint16_t  packageFreq;         //lidar run speed 
	int16_t  packageTemp;          //lidar temperature 
	uint32_t packagePointTime;     //lidar point time info 
	uint16_t packageFirstAngle;    //lidar start angle 
	uint16_t packageLastAngle;     //lidar stop angle 
	float    packageAngleDiffer;   //lidar angle differ 
	float    packageLastAngleDiffer; //lidar last angle differ 
	uint8_t  packagePointDistSize; //package point distance 
	bool     packageHas0CAngle;    //package is 0 angle 
	bool     packageHasTemp;       //current package has temperature data???
	bool     packageHas0CFirst;    //first byte 
	bool     packageHasTempFirst;  //first byte 
	uint16_t  package0CIndex;      //angle 0 index 
	uint64_t packageStamp;		   //received the data stamp info  
	uint16_t packagePointNum;	   //point num 
}Nvilidar_PointViewerPackageInfoTypeDef;

//lidar received data info 
typedef struct
{
	bool	recvFinishFlag;
	Nvilidar_Protocol_DeviceInfo lidar_device_info;//Data received and returned by the radar
	Nvilidar_Protocol_GetPara    lidar_get_para;	//Getting parameter information 
	uint8_t     isHasSensitive;					//Signal quality information is available
	uint16_t    aimSpeed;						//speed information x100
	uint32_t    samplingRate;					//Sample rate x1
	int16_t     angleOffset;					//Angular offset x64
	uint8_t     tailingLevel;					//dragging class for mcu 
	uint16_t    apdValue;						//apd value information 
	uint16_t 	qualityFilter;					//quality filter    
	uint8_t     saveFlag;						//Did you save it successfully? 
}NvilidarRecvInfoTypeDef;

//one circle data info  
typedef struct
{
	uint64_t  startStamp;			//One Lap Start Timestamp 
	uint64_t  stopStamp;			//One Lap Stop Timestamp 
	std::vector<Nvilidar_Node_Info>  lidarCircleNodePoints;	//lidar point data
}CircleDataInfoTypeDef;



//======================================Output data information============================================ 
/**
 * @brief The Laser Point struct
 * @note angle unit: rad.\n
 * range unit: meter.\n
 */
typedef struct {
	/// lidar angle. unit(rad)
	float angle;
	/// lidar range. unit(m)
	float range;
	/// lidar intensity
	float intensity;
} NviLidarPoint;

/**
 * @brief A struct for returning configuration from the NVILIDAR
 * @note angle unit: rad.\n
 * time unit: second.\n
 * range unit: meter.\n
 */
typedef struct {
	/// Start angle for the laser scan [rad].  0 is forward and angles are measured clockwise when viewing NVILIDAR from the top.
	float min_angle;
	/// Stop angle for the laser scan [rad].   0 is forward and angles are measured clockwise when viewing NVILIDAR from the top.
	float max_angle;
	/// angle resoltuion [rad]
	float angle_increment;
	/// Scan resoltuion [s]
	float time_increment;
	/// Time between scans
	float scan_time;
	/// Minimum range [m]
	float min_range;
	/// Maximum range [m]
	float max_range;
} NviLidarConfig;


typedef struct {
	/// System time when first range was measured in nanoseconds
	uint64_t stamp;
	/// Array of lidar points
	std::vector<NviLidarPoint> points;
	/// Configuration of scan
	NviLidarConfig config;
} LidarScan;



#endif
