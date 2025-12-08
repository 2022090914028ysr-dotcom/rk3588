/*
 * @Version      : V1.0
 * @Date         : 2024-10-17 18:33:18
 * @Description  : 
 */
#include "ros/init.h"
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

#define RAD2DEG(x) ((x)*180./M_PI)

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    int count = scan->scan_time / scan->time_increment;
    printf("[info]: I heard a laser scan %s[%d]:\n", scan->header.frame_id.c_str(), count);
    printf("[info]: angle_range : [%f, %f]\n", RAD2DEG(scan->angle_min), RAD2DEG(scan->angle_max));
  
    for(int i = 0; i < count; i++) {
        float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i);
	if(degree > -5 && degree< 5)
        printf("[info]: angle-distance : [%f, %f, %i]\n", degree, scan->ranges[i], i);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vp100_client");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 1000, scanCallback);
    ros::spin();

    return 0;
}