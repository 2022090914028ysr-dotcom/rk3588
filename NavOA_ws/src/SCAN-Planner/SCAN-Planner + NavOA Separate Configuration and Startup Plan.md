# SCAN-Planner + NavOA Separate Configuration and Startup Plan

This plan does not use a combined launch file. SCAN, the path conversion node, and NavOA are started separately.

## 1. Files to Configure

### 1.1 SCAN Real-Robot Interface

File:

```text
/home/ubuntu/Robit_SlamNav/NavOA_ws/src/SCAN-Planner/src/planner/plan_manage/launch/run.launch
```

Configuration:

```xml
<arg name="is_real_world" default="true"/>
<arg name="navi_mode" default="1"/>
<arg name="sensor_type" default="lidar"/>

<arg name="world_frame" default="real-robot localization parent frame"/>
<arg name="body_pose_topic" default="real-robot body odometry topic"/>
<arg name="sensor_pose_topic" default="real-robot sensor pose topic"/>
<arg name="cloud_topic" default="real-robot 3D point cloud topic"/>
<arg name="cloud_is_world" default="false"/>

<arg name="controller_mode" default="none"/>
<arg name="launch_go2_support" default="false"/>
<arg name="launch_odom_visualization" default="false"/>
```

### 1.2 SCAN Planning Parameters

File:

```text
/home/ubuntu/Robit_SlamNav/NavOA_ws/src/SCAN-Planner/src/planner/plan_manage/launch/advanced_param.xml
```

Suggested for first real-robot test:

```xml
<arg name="max_vel" default="0.30"/>
<arg name="max_acc" default="0.30"/>
<arg name="planning_horizon" default="3.5"/>
```

Adjust according to the robot's physical dimensions:

```xml
<param name="grid_map/double_cylinder_radius" value="real-robot bounding circle radius" />
<param name="grid_map/double_cylinder_offset" value="front/rear center offset" />
<param name="grid_map/body_height" value="real-robot body center height" />
```

Confirm the world frame:

```xml
<param name="grid_map/frame_id" value="real-robot localization parent frame" type="str"/>
```

### 1.3 SCAN Path Output

File:

```text
/home/ubuntu/Robit_SlamNav/NavOA_ws/src/SCAN-Planner/src/planner/plan_manage/src/bspline_to_path.cpp
```

Keep the internal interface:

```text
/planning/bspline -> /planning/path
```

The output `frame_id` uses the real-robot localization parent frame. It is recommended to pass it in at startup rather than modifying the source code.

### 1.4 NavOA Path Segmentation

File:

```text
/home/ubuntu/Robit_SlamNav/NavOA_ws/src/NavOA_ros/scripts/path_segment_publisher.py
```

Configure the path input as:

```python
rospy.Subscriber('/planning/path', Path, self.path_callback)
```

Keep the sub-goal output:

```python
self.goal_pub = rospy.Publisher(
    '/split_goal', PoseStamped, queue_size=1)
```

Recommended segmentation parameters:

```text
segment_length = 0.8
reach_tolerance = 0.35
final_tolerance = 0.15
```

### 1.5 NavOA Control Configuration

File:

```text
/home/ubuntu/Robit_SlamNav/NavOA_ws/src/NavOA_ros/example/gazebo_limo/config/NavOA_planner_limo.yaml
```

Suggested for first real-robot test:

```yaml
receding: 5
step_time: 0.3
ref_speed: 0.20
device: 'cpu'

robot:
  kinematics: 'diff'
  max_speed: [0.30, 0.50]
  max_acce: [0.30, 0.80]
  length: 0.60       # change to the real-robot length
  width: 0.30        # change to the real-robot width

ipath:
  interval: 0.09
  arrive_threshold: 0.3
  close_threshold: 0.1
  max_tracking_distance: 1.0
```

### 1.6 NavOA Launch Configuration

File:

```text
/home/ubuntu/Robit_SlamNav/NavOA_ws/src/NavOA_ros/example/gazebo_limo/launch/NavOA_gazebo_limo.launch
```

Configuration:

```xml
<arg name="map_frame" default="real-robot localization parent frame"/>
<arg name="base_frame" default="real-robot base frame"/>
<arg name="lidar_frame" default="real-robot 2D laser frame"/>
<arg name="scan_topic" default="real-robot 2D laser topic"/>
<arg name="ref_speed" default="0.20"/>
<arg name="vel_filter_cmd_out_topic" default="real-robot base velocity topic"/>

<arg name="path_topic" default="/planning/path"/>
<arg name="split_goal_topic" default="/split_goal"/>
<arg name="segment_length" default="0.8"/>
<arg name="reach_tolerance" default="0.35"/>
<arg name="final_tolerance" default="0.15"/>

<arg name="path_progress_timeout" default="0.0"/>
<arg name="blocked_route_memory_enabled" default="false"/>
<arg name="return_to_previous_goal_enabled" default="false"/>
<arg name="goal_timeout_button_enabled" default="false"/>
```

Confirm that the following velocity and goal mappings are retained:

```xml
<remap from="/NavOA_goal" to="/split_goal"/>
<remap from="/NavOA_cmd_vel" to="/cmd_vel_tem"/>
```

`vel_filter` configuration:

```xml
<param name="scan_topic" value="real-robot 2D laser topic"/>
<param name="cmd_in_topic" value="/cmd_vel_tem"/>
<param name="cmd_out_topic" value="real-robot base velocity topic"/>
<param name="max_forward_speed" value="0.20"/>
<param name="max_angular_speed" value="0.50"/>
```

## 2. Build

```bash
cd /home/ubuntu/Robit_SlamNav/NavOA_ws/src/SCAN-Planner
source /opt/ros/noetic/setup.bash
catkin_make -j2
```

## 3. Stop the Old System Before Startup

Stop any previously started:

```text
scan_planner run.launch
move_base
navoa_nav_only.launch
closed_loop_controller
open_loop_controller
other nodes that directly publish to the real-robot velocity topic
```

## 4. Startup Order

### Terminal 1: Real-Robot Base, Localization, and Sensors

Start the real robot's existing driver files, ensuring the following data is already present:

```text
real-robot body odometry: nav_msgs/Odometry
real-robot sensor pose: nav_msgs/Odometry
real-robot 3D point cloud: sensor_msgs/PointCloud2
real-robot 2D laser: sensor_msgs/LaserScan
real-robot localization parent frame -> real-robot base frame TF
real-robot localization parent frame -> real-robot laser frame TF
```

Which file this terminal launches depends on the real-robot driver; do not use SCAN or NavOA simulation drivers.

### Terminal 2: Start the SCAN Planner

Launch file:

```text
/home/ubuntu/Robit_SlamNav/NavOA_ws/src/SCAN-Planner/src/planner/plan_manage/launch/run.launch
```

Command:

```bash
source /home/ubuntu/Robit_SlamNav/NavOA_ws/src/SCAN-Planner/devel/setup.bash

roslaunch scan_planner run.launch \
  is_real_world:=true \
  navi_mode:=1 \
  sensor_type:=lidar \
  controller_mode:=none \
  launch_go2_support:=false \
  launch_odom_visualization:=false \
  world_frame:=real-robot localization parent frame \
  body_pose_topic:=real-robot body odometry topic \
  sensor_pose_topic:=real-robot sensor pose topic \
  cloud_topic:=real-robot 3D point cloud topic \
  cloud_is_world:=false
```

### Terminal 3: Start the B-spline to Path Conversion Node

Executable:

```text
/home/ubuntu/Robit_SlamNav/NavOA_ws/src/SCAN-Planner/devel/lib/scan_planner/bspline_to_path
```

Command:

```bash
source /home/ubuntu/Robit_SlamNav/NavOA_ws/src/SCAN-Planner/devel/setup.bash

rosrun scan_planner bspline_to_path \
  _input_topic:=/planning/bspline \
  _output_topic:=/planning/path \
  _frame_id:=real-robot localization parent frame \
  _sample_dt:=0.05 \
  _min_point_distance:=0.03 \
  _flatten_z:=true
```

### Terminal 4: Start the NavOA Control Part

Only launch this file:

```text
/home/ubuntu/Robit_SlamNav/NavOA_ws/src/NavOA_ros/example/gazebo_limo/launch/NavOA_gazebo_limo.launch
```

Since there is a backup launch file with the same name inside the project, launch using the full path:

```bash
source /home/ubuntu/Robit_SlamNav/NavOA_ws/src/miniconda3/etc/profile.d/conda.sh
conda activate neupan
source /home/ubuntu/Robit_SlamNav/NavOA_ws/src/SCAN-Planner/devel/setup.bash
export ROS_PACKAGE_PATH=/home/Robit_SlamNav:$ROS_PACKAGE_PATH

roslaunch /home/ubuntu/Robit_SlamNav/NavOA_ws/src/NavOA_ros/example/gazebo_limo/launch/NavOA_gazebo_limo.launch \
  map_frame:=real-robot localization parent frame \
  base_frame:=real-robot base frame \
  lidar_frame:=real-robot 2D laser frame \
  scan_topic:=real-robot 2D laser topic \
  vel_filter_cmd_out_topic:=real-robot base velocity topic \
  path_topic:=/planning/path \
  split_goal_topic:=/split_goal \
  segment_length:=0.8 \
  reach_tolerance:=0.35 \
  final_tolerance:=0.15 \
  ref_speed:=0.20 \
  path_progress_timeout:=0.0 \
  blocked_route_memory_enabled:=false \
  return_to_previous_goal_enabled:=false \
  goal_timeout_button_enabled:=false
```

If there is no RKNN runtime environment, add the following to the end of the command:

```bash
use_rknn:=false
```

### Terminal 5: Start RViz

Launch file:

```text
/home/ubuntu/Robit_SlamNav/NavOA_ws/src/SCAN-Planner/src/planner/plan_manage/launch/rviz.launch
```

Command:

```bash
source /home/ubuntu/Robit_SlamNav/NavOA_ws/src/SCAN-Planner/devel/setup.bash
roslaunch scan_planner rviz.launch
```

In RViz, change the Fixed Frame to the real-robot localization parent frame, then use `2D Nav Goal` to publish the destination.

## 5. Files Not to Launch

Do not launch:

```text
/home/ubuntu/Robit_SlamNav/NavOA_ws/src/NavOA_ros/launch/navoa_nav_only.launch
```

It starts the original move_base, GlobalPlanner, and costmap.

Do not launch separately:

```text
scan_planner/closed_loop_controller
scan_planner/open_loop_controller
```

## 6. Topic Order After Successful Startup

Check in order:

```bash
rostopic hz real-robot body odometry topic
rostopic hz real-robot 3D point cloud topic
rostopic hz real-robot 2D laser topic
rostopic hz /planning/bspline
rostopic hz /planning/path
rostopic echo /split_goal
rostopic hz /NavOA_cmd_vel
rostopic hz real-robot base velocity topic
```

Check the final velocity publisher:

```bash
rostopic info real-robot base velocity topic
```

The final velocity topic should be published only by NavOA's `vel_filter`.
