NavOA_ws使用说明

conda activate nav
roslaunch NavOA_ros NavOA_gazebo_limo.launch

依次打开
````
roslaunch lidar_ros lidar.launch

````
conda deactivate
python3 /home/ubuntu/ShenZhenYu-DLUT-Robot/NavOA_ws/src/NavOA_ros/scripts/real_robot_cmd/SendCommands.py
````

roslaunch rf2o_laser_odometry rf2o_laser_odometry.launch
````