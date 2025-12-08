# VP100/T10 LIDAR ROS DRIVER

## How to [install ROS](http://wiki.ros.org/cn/ROS/Installation)

[ubuntu](http://wiki.ros.org/cn/Installation/Ubuntu)

[windows](http://wiki.ros.org/Installation/Windows)

## How to Create a ROS workspace

[Create a workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)

you also can with this:

    1)  $mkdir -p ~/lidar_ros_ws/src
        $cd ~/lidar_ros_ws/src
    2)  $cd..
    3)  $catkin_make
    4)  $source devel/setup.bash
    5)  echo $ROS_PACKAGE_PATH
        /home/xiaoxiangyu/project/ros1_new/src:/opt/ros/noetic/share


## How to build Lidar ROS Package
### 1.Get the ros code
    1) Clone this project to your catkin's workspace src folder
    	$ git clone https://gitee.com/nvilidar/vp100_ros.git       
		or
		$ git clone https://github.com/nvilidar/vp100_ros.git

    2) download the sdk code from our webset,  http://www.nvistar.com/?jishuzhichi/xiazaizhongxin
### 2.Copy the ros code
    1) Copy the ros source file to the "~/lidar_ros_ws/src"
    2) Running "catkin_make" to build lidar_node and lidar_client
### 3.Serialport configuration
if you use the lidar device name,you must give the permissions to user.
```shell
whoami
```
get the user name.link ubuntu.
```shell
sudo usermod -a -G dialout ubuntu
```
ubuntu is the user name.
```shell
sudo reboot   
```

## ROS Parameter Configuration
### 1. Lidar Support
    the baudrate can be 115200bsp or 230400bps 

## How to Run LIDAR ROS Package
### 1. Run LIDAR node and view in the rviz
------------------------------------------------------------
	roslaunch lidar_ros lidar_view.launch

### 2. Run node and view using test application
------------------------------------------------------------
	roslaunch lidar_ros lidar.launch

	rosrun lidar_ros lidar_client

## LIDAR ROS Parameter
|  value   |  information  |
|  :----:    | :----:  |
| serial_baud  | if use serialport,the lidar's serialport |
| serial_name  | if use serialport,the lidar's port name |
| frame_id  | it is useful in ros,lidar ros frame id |
| resolution_fixed  | Rotate one circle fixed number of points,it is 'true' in ros,default |
| counterclockwise  | lidar's point counterclockwise|
| angle_max  | lidar angle max value,max:180.0°|
| angle_max  | lidar angle min value,min:-180.0°|
| range_max  | lidar's max measure distance,default:15.0 meters|
| range_min  | lidar's min measure distance,default:0.001 meters|
| angle_corp_string  | if you want to filter some point's you can change it,it is anti-clockwise for the lidar.eg. you can set the value "30,60,90,120",you can remove the 30°-60° and 90°-120° points in the view|