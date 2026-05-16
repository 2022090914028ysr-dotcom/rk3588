

# Mapping and Dynamic Navigation & Obstacle Avoidance

## 1. Environment Configuration

### ROS Configuration
```
wget http://fishros.com/install -O fishros && . fishros
```
//You MUST use this to install ROS, so that you can use `rosdepc`. Otherwise, due to the instability of `rosdep`, you will have to install ROS packages one by one later, which is very painful!!!

```
cd NavOA_ws
rosdepc install --from-paths src --ignore-src -r -y
```

---

## Cartographer Environment Configuration
```
cd NavOA_ws/src/cartographer/

./scripts/install_abseil.sh
```
Open the `install_abseil.sh` file and comment out the line `git clone https://github.com/abseil/abseil-cpp.git`.

Contents of `install_abseil.sh`:

```
set -o errexit
set -o verbose

*git clone https://github.com/abseil/abseil-cpp.git*
cd abseil-cpp
git checkout 215105818dfde3174fe799600bb0f3cae233d0bf # 20211102.0
mkdir build
```

```
rm -rf ./abseil-cpp/build
./install_abseil.sh # Compile the abseil-cpp package
```

===== Page 2 =====

**Overall Compilation**

```
cd NavOA_ws
rm -rf build_isolated/ devel_isolated/
source devel_isolated/setup.bash

catkin_make_isolated --use-ninja
```
cd NavOA_ws
source devel_isolated/setup.bash
Note: If compilation still reports errors, it is basically due to missing some ROS packages. Download whichever package is reported missing according to the error message.

**conda Environment Configuration**

Required packages:
- numpy
- cxxpy
- cxxpylayers
- ecos
- matplotlib
- scipy
- torch
- rich
- pyyaml
- scikit-learn
- gctl==1.2

# for simulation
- ir-sim>=2.4.0

Configure the conda environment; the Python version should be 3.10.
Save the above as a `requirements.txt` file.
Run `pip install -e .`

If there any error during install rknn related to onnxoptimizer==0.3.8  
can be solved by installing cmake 
conda install -c conda-forge cmake

===== Page 3 =====

# 2. Function Usage

**Integrated launch file — `NavOA_gazebo_limo_all.launch`**

```
cd NavOA_ws
conda deactivate
deactivate venv
roslaunch NavOA_ros NavOA_gazebo_limo_all.launch
```

This launch file integrates the following functionalities:

1) Starts the LiDAR node
2) Starts the laser odometry node
3) Starts the relocation node
4) Starts the robot chassis control node

---

**Navigation & Obstacle Avoidance Usage — `NavOA_gazebo_limo.launch`**

```
cd NavOA_ws
conda activate nav //Need to change to your own configured environment
roslaunch NavOA_ros NavOA_gazebo_limo.launch

rosnode list | egrep 'cartographer'
rosrun dynamic_reconfigure dynparam set /NavOA_control ref_speed 0.5
```

First, start `NavOA_gazebo_limo_all.launch`.

This launch will start the navigation and obstacle avoidance node. Use the **2D Goal Nav** tool in rviz to select a target point on the map, and the robot will autonomously navigate to the target point.

Note: After starting, you need to wait for the relocation function to complete initialization. You can observe whether the LiDAR scan aligns with the walls.

===== Page 4 =====

# **SLAM Usage — `carto_slam.launch`**

```
cd NavOA_ws
conda deactivate
export DISPLAY=:0

roslaunch NavOA_ros carto_slam.launch

roslaunch NavOA_ros NavOA_slam.launch

# 3. Save the map when satisfied
rosrun map_server map_saver -f ~/map
# While gmapping is running, save the map:
rosrun map_server map_saver -f ~/my_map map:=/dynamic_map
```

First, start `NavOA_gazebo_limo_all.launch`.
This launch will start the SLAM mapping function. First, install the required package with `sudo apt install ros-noetic-teleop-twist-keyboard`, then use `rosrun teleop_twist_keyboard teleop_twist_keyboard.py` to control the robot's movement for mapping.

Important: After finishing mapping and before killing the current SLAM terminal, you must promptly run the following command to save the map in `.pbstream` format. Run the following command:

#Save MAP-----------------------------
```
rosservice call /write_state \
"filename: '/home/ubuntu/Robit_SlamNav/NavOA_ws/src/carto_ws/map/meeting_room/meeting_room.pbstream'
include_unfinished_submaps: true"
```
rosrun map_server map_saver -f /home/ubuntu/Robit_SlamNav/NavOA_ws/src/carto_ws/map/meeting_room/meeting_room

# Must save as `.pbstream` format, and modify the save path to your own address.

cd /home/ubuntu/Robit_SlamNav/NavOA_ws/src/carto_ws
./save_map.sh meeting_room

#####################################################easy navigation launch file#####################################################
cd NavOA_ws

source /opt/ros/noetic/setup.bash

rm -rf build_isolated/ devel_isolated/

source devel_isolated/setup.bash

catkin_make_isolated --install --use-ninja

catkin_make_isolated --use-ninja

conda activate control 

export DISPLAY=:0

tmux attach -t robot_start

taskset -c 1,2 python3  /home/ubuntu/Robit_SlamNav/NavOA_ws/src/NavOA_ros/scripts/pub.py

taskset -c 1,2 python3 /home/ubuntu/Robit_SlamNav/NavOA_ws/src/NavOA_ros/scripts/maplocation.py

taskset -c 4,5 roslaunch NavOA_ros NavOA_gazebo_limo_all.launch

taskset -c 6,7 roslaunch NavOA_ros NavOA_gazebo_limo.launch

taskset -c 5,6,7 roslaunch cartographer_ros livox_2d_localization.launch

taskset -c 5,6,7 roslaunch cartographer_ros livox_2d.launch

roslaunch NavOA_ros navoa_nav_only.launch

roslaunch cartographer_ros cartographer_guard.launch


rosrun tf tf_echo map odom 1
rosrun tf tf_echo map imu_link 1


roslaunch ascamera hp60c.launch

taskset -c 6,7 python Ros_control/cmd_vel_bridge.py 

sudo stty -F /dev/ttyS6 230400 cs8 -cstopb -parenb -ixon -ixoff
sudo cat /dev/ttyS6

did some correction 
**[File Content End]**


#####################################################voice navigation (English place names)#####################################################

This workspace already uses RViz "2D Nav Goal" by publishing a `PoseStamped` to `/move_base_simple/goal`.
The voice integration added here follows the same interface:

- Publish recognized text to: `/voice/text` (`std_msgs/String`)
- The router publishes a goal to: `/move_base_simple/goal` (`geometry_msgs/PoseStamped`)

### 1) Start the router

In a terminal (after your usual `source devel_isolated/setup.bash`):

```
roslaunch NavOA_ros voice_nav.launch
```

It loads named places from:

- `NavOA_ros/example/gazebo_limo/config/voice_places.yaml`

Supported place names (canonical keys):

- `kitchen` (accepts: "kitchen" / "厨房")
- `meeting_room` (accepts: "meeting room" / "会议室")
- `living_room` (accepts: "living room" / "客厅")
- `balcony` (accepts: "balcony" / "balcoy" / "阳台")
- `bedroom` (accepts: "bed room" / "卧室")
- `hall` (accepts: "hall room" / "hallway" / "大厅" / "走廊")

### 2) Test without a real microphone (simulate ASR)

Send a text command:

```
rostopic pub -1 /voice/text std_msgs/String "data: 'go to kitchen'"
```

Other examples:

```
rostopic pub -1 /voice/text std_msgs/String "data: 'navigate to meeting room'"
rostopic pub -1 /voice/text std_msgs/String "data: 'go living room'"
rostopic pub -1 /voice/text std_msgs/String "data: 'go to balcoy'"

Chinese examples:

```
rostopic pub -1 /voice/text std_msgs/String "data: '去厨房'"
rostopic pub -1 /voice/text std_msgs/String "data: '前往会议室'"
rostopic pub -1 /voice/text std_msgs/String "data: '导航到客厅'"
rostopic pub -1 /voice/text std_msgs/String "data: '去阳台'"
```
```

### 3) Fill in real coordinates for each place

By default all places are 0.0. To set a place:

1) In RViz, use **2D Nav Goal** to click the desired location and heading.
2) In another terminal, run the capture helper (it waits for the next `/move_base_simple/goal`):

```
rosrun NavOA_ros waypoint_capture_from_goal.py _name:=kitchen
```

3) Click the goal in RViz again. The terminal prints a YAML line like:

```
	kitchen: { x: 1.234, y: -0.456, yaw: 1.570 }
```

4) Paste it under `places:` in `NavOA_ros/example/gazebo_limo/config/voice_places.yaml`.

