# SCAN-Planner + NavOA 分开配置与启动方案

本方案不使用组合启动文件。SCAN、路径转换节点和 NavOA 分别启动。

## 1. 需要配置的文件

### 1.1 SCAN 实机接口

文件：

```text
/home/ubuntu/Robit_SlamNav/NavOA_ws/src/SCAN-Planner/src/planner/plan_manage/launch/run.launch
```

配置：

```xml
<arg name="is_real_world" default="true"/>
<arg name="navi_mode" default="1"/>
<arg name="sensor_type" default="lidar"/>

<arg name="world_frame" default="实机定位父坐标系"/>
<arg name="body_pose_topic" default="实机车体里程计话题"/>
<arg name="sensor_pose_topic" default="实机传感器位姿话题"/>
<arg name="cloud_topic" default="实机三维点云话题"/>
<arg name="cloud_is_world" default="false"/>

<arg name="controller_mode" default="none"/>
<arg name="launch_go2_support" default="false"/>
<arg name="launch_odom_visualization" default="false"/>
```

### 1.2 SCAN 规划参数

文件：

```text
/home/ubuntu/Robit_SlamNav/NavOA_ws/src/SCAN-Planner/src/planner/plan_manage/launch/advanced_param.xml
```

首次实机测试建议：

```xml
<arg name="max_vel" default="0.30"/>
<arg name="max_acc" default="0.30"/>
<arg name="planning_horizon" default="3.5"/>
```

根据实机尺寸修改：

```xml
<param name="grid_map/double_cylinder_radius" value="实机包围圆半径" />
<param name="grid_map/double_cylinder_offset" value="前后圆心偏移" />
<param name="grid_map/body_height" value="实机机身中心高度" />
```

确认世界坐标系：

```xml
<param name="grid_map/frame_id" value="实机定位父坐标系" type="str"/>
```

### 1.3 SCAN 路径输出

文件：

```text
/home/ubuntu/Robit_SlamNav/NavOA_ws/src/SCAN-Planner/src/planner/plan_manage/src/bspline_to_path.cpp
```

内部接口保持：

```text
/planning/bspline -> /planning/path
```

输出 `frame_id` 使用实机定位父坐标系。推荐启动时传入，不修改源码。

### 1.4 NavOA 路径分割

文件：

```text
/home/ubuntu/Robit_SlamNav/NavOA_ws/src/NavOA_ros/scripts/path_segment_publisher.py
```

路径输入配置为：

```python
rospy.Subscriber('/planning/path', Path, self.path_callback)
```

子目标输出保持：

```python
self.goal_pub = rospy.Publisher(
    '/split_goal', PoseStamped, queue_size=1)
```

推荐分割参数：

```text
segment_length = 0.8
reach_tolerance = 0.35
final_tolerance = 0.15
```

### 1.5 NavOA 控制配置

文件：

```text
/home/ubuntu/Robit_SlamNav/NavOA_ws/src/NavOA_ros/example/gazebo_limo/config/NavOA_planner_limo.yaml
```

首次实机测试建议：

```yaml
receding: 5
step_time: 0.3
ref_speed: 0.20
device: 'cpu'

robot:
  kinematics: 'diff'
  max_speed: [0.30, 0.50]
  max_acce: [0.30, 0.80]
  length: 0.60       # 改成实机长度
  width: 0.30        # 改成实机宽度

ipath:
  interval: 0.09
  arrive_threshold: 0.3
  close_threshold: 0.1
  max_tracking_distance: 1.0
```

### 1.6 NavOA 启动配置

文件：

```text
/home/ubuntu/Robit_SlamNav/NavOA_ws/src/NavOA_ros/example/gazebo_limo/launch/NavOA_gazebo_limo.launch
```

配置：

```xml
<arg name="map_frame" default="实机定位父坐标系"/>
<arg name="base_frame" default="实机底盘坐标系"/>
<arg name="lidar_frame" default="实机二维激光坐标系"/>
<arg name="scan_topic" default="实机二维激光话题"/>
<arg name="ref_speed" default="0.20"/>
<arg name="vel_filter_cmd_out_topic" default="实机底盘速度话题"/>

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

确认保留以下速度和目标映射：

```xml
<remap from="/NavOA_goal" to="/split_goal"/>
<remap from="/NavOA_cmd_vel" to="/cmd_vel_tem"/>
```

`vel_filter` 配置：

```xml
<param name="scan_topic" value="实机二维激光话题"/>
<param name="cmd_in_topic" value="/cmd_vel_tem"/>
<param name="cmd_out_topic" value="实机底盘速度话题"/>
<param name="max_forward_speed" value="0.20"/>
<param name="max_angular_speed" value="0.50"/>
```

## 2. 编译

```bash
cd /home/ubuntu/Robit_SlamNav/NavOA_ws/src/SCAN-Planner
source /opt/ros/noetic/setup.bash
catkin_make -j2
```

## 3. 启动前停止旧系统

停止以前启动的：

```text
scan_planner run.launch
move_base
navoa_nav_only.launch
closed_loop_controller
open_loop_controller
其他直接发布实机速度话题的节点
```

## 4. 启动顺序

### 终端 1：实机底盘、定位和传感器

启动实机已有的驱动文件，保证以下数据已经存在：

```text
实机车体里程计：nav_msgs/Odometry
实机传感器位姿：nav_msgs/Odometry
实机三维点云：sensor_msgs/PointCloud2
实机二维激光：sensor_msgs/LaserScan
实机定位父坐标系 -> 实机底盘坐标系 TF
实机定位父坐标系 -> 实机激光坐标系 TF
```

这一终端启动哪个文件取决于实机驱动，不使用 SCAN 或 NavOA 的仿真驱动。

### 终端 2：启动 SCAN 规划器

启动文件：

```text
/home/ubuntu/Robit_SlamNav/NavOA_ws/src/SCAN-Planner/src/planner/plan_manage/launch/run.launch
```

命令：

```bash
source /home/ubuntu/Robit_SlamNav/NavOA_ws/src/SCAN-Planner/devel/setup.bash

roslaunch scan_planner run.launch \
  is_real_world:=true \
  navi_mode:=1 \
  sensor_type:=lidar \
  controller_mode:=none \
  launch_go2_support:=false \
  launch_odom_visualization:=false \
  world_frame:=实机定位父坐标系 \
  body_pose_topic:=实机车体里程计话题 \
  sensor_pose_topic:=实机传感器位姿话题 \
  cloud_topic:=实机三维点云话题 \
  cloud_is_world:=false
```

### 终端 3：启动 B 样条路径转换节点

启动程序：

```text
/home/ubuntu/Robit_SlamNav/NavOA_ws/src/SCAN-Planner/devel/lib/scan_planner/bspline_to_path
```

命令：

```bash
source /home/ubuntu/Robit_SlamNav/NavOA_ws/src/SCAN-Planner/devel/setup.bash

rosrun scan_planner bspline_to_path \
  _input_topic:=/planning/bspline \
  _output_topic:=/planning/path \
  _frame_id:=实机定位父坐标系 \
  _sample_dt:=0.05 \
  _min_point_distance:=0.03 \
  _flatten_z:=true
```

### 终端 4：启动 NavOA 控制部分

只启动文件：

```text
/home/ubuntu/Robit_SlamNav/NavOA_ws/src/NavOA_ros/example/gazebo_limo/launch/NavOA_gazebo_limo.launch
```

由于工程内存在同名备份 launch，使用完整路径启动：

```bash
source /home/ubuntu/Robit_SlamNav/NavOA_ws/src/miniconda3/etc/profile.d/conda.sh
conda activate neupan
source /home/ubuntu/Robit_SlamNav/NavOA_ws/src/SCAN-Planner/devel/setup.bash
export ROS_PACKAGE_PATH=/home/Robit_SlamNav:$ROS_PACKAGE_PATH

roslaunch /home/ubuntu/Robit_SlamNav/NavOA_ws/src/NavOA_ros/example/gazebo_limo/launch/NavOA_gazebo_limo.launch \
  map_frame:=实机定位父坐标系 \
  base_frame:=实机底盘坐标系 \
  lidar_frame:=实机二维激光坐标系 \
  scan_topic:=实机二维激光话题 \
  vel_filter_cmd_out_topic:=实机底盘速度话题 \
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

没有 RKNN 运行环境时，在命令末尾增加：

```bash
use_rknn:=false
```

### 终端 5：启动 RViz

启动文件：

```text
/home/ubuntu/Robit_SlamNav/NavOA_ws/src/SCAN-Planner/src/planner/plan_manage/launch/rviz.launch
```

命令：

```bash
source /home/ubuntu/Robit_SlamNav/NavOA_ws/src/SCAN-Planner/devel/setup.bash
roslaunch scan_planner rviz.launch
```

在 RViz 中将 Fixed Frame 改为实机定位父坐标系，然后使用 `2D Nav Goal` 发布终点。

## 5. 不启动的文件

不要启动：

```text
/home/ubuntu/Robit_SlamNav/NavOA_ws/src/NavOA_ros/launch/navoa_nav_only.launch
```

它会启动原来的 move_base、GlobalPlanner 和 costmap。

不要单独启动：

```text
scan_planner/closed_loop_controller
scan_planner/open_loop_controller
```

## 6. 启动成功后的话题顺序

依次检查：

```bash
rostopic hz 实机车体里程计话题
rostopic hz 实机三维点云话题
rostopic hz 实机二维激光话题
rostopic hz /planning/bspline
rostopic hz /planning/path
rostopic echo /split_goal
rostopic hz /NavOA_cmd_vel
rostopic hz 实机底盘速度话题
```

检查最终速度发布者：

```bash
rostopic info 实机底盘速度话题
```

最终速度话题应只有 NavOA 的 `vel_filter` 发布。
