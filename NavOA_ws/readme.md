

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

roslaunch NavOA_ros carto_slam.launch
# 3. Save the map when satisfied
rosrun map_server map_saver -f ~/map
# While gmapping is running, save the map:
rosrun map_server map_saver -f ~/my_map map:=/dynamic_map
```

First, start `NavOA_gazebo_limo_all.launch`.
This launch will start the SLAM mapping function. First, install the required package with `sudo apt install ros-noetic-teleop-twist-keyboard`, then use `rosrun teleop_twist_keyboard teleop_twist_keyboard.py` to control the robot's movement for mapping.

Important: After finishing mapping and before killing the current SLAM terminal, you must promptly run the following command to save the map in `.pbstream` format. Run the following command:

```
rosservice call /write_state "{filename: '/home/ubuntu/Robit_SlamNav/NavOA_ws/src/NavOA_ros/example/gazebo_limo/maps/limo_map.pbstream', include_unfinished_submaps: true}"
```
# Must save as `.pbstream` format, and modify the save path to your own address.

**[File Content End]**