# 原始 SCAN-Planner 与原始 NavOA 实机话题修改清单

## 一、SCAN-Planner 中需要修改的文件

### 1. `run.launch`

文件：

```text
/home/ubuntu/Robit_SlamNav/NavOA_ws/src/SCAN-Planner/src/planner/plan_manage/launch/run.launch
```

#### 车体里程计

原始内容：

```xml
<arg name="body_pose_topic"
     value="$(eval '/LIO/odom_vehicle' if arg('is_real_world') else '/quad_0/body_pose')" />
```

改为实机车体里程计话题：

```xml
<arg name="body_pose_topic" default="/实机车体里程计话题" />
```

消息类型必须是：

```text
nav_msgs/Odometry
```

例如：

```xml
<arg name="body_pose_topic" default="/odom" />
```

#### 三维雷达或相机位姿

原始内容：

```xml
<arg name="sensor_pose_topic"
     value="$(eval '/LIO/odom_imu' if arg('is_real_world') else ('/quad_0/camera_pose' if arg('sensor_type') == 'depth' else '/quad_0/lidar_pose'))" />
```

改为实机传感器位姿话题：

```xml
<arg name="sensor_pose_topic" default="/实机传感器位姿话题" />
```

消息类型必须是：

```text
nav_msgs/Odometry
```

#### 三维点云

原始内容：

```xml
<arg name="cloud_topic"
     value="$(eval '/LIO/clouds_lidar' if arg('is_real_world') else '/pcl_render_node/cloud')"/>
```

改为实机三维雷达点云：

```xml
<arg name="cloud_topic" default="/实机三维点云话题"/>
```

消息类型必须是：

```text
sensor_msgs/PointCloud2
```

#### 点云所在坐标系

原始实机配置：

```xml
<arg name="cloud_is_world" value="false"/>
```

按实机点云修改：

```xml
<!-- 点云位于雷达自身坐标系 -->
<arg name="cloud_is_world" default="false"/>

<!-- 点云已经转换到定位世界坐标系 -->
<arg name="cloud_is_world" default="true"/>
```

#### 深度图

只在 `sensor_type=depth` 时修改。

原始内容：

```xml
<arg name="depth_topic"
     value="$(eval '/camera/aligned_depth_to_color/image_raw' if arg('is_real_world') else '/pcl_render_node/depth')"/>
```

改为：

```xml
<arg name="depth_topic" default="/实机深度图话题"/>
```

消息类型必须是：

```text
sensor_msgs/Image
```

### 2. `advanced_param.xml`

文件：

```text
/home/ubuntu/Robit_SlamNav/NavOA_ws/src/SCAN-Planner/src/planner/plan_manage/launch/advanced_param.xml
```

确认 SCAN 世界坐标系与实机里程计 `header.frame_id` 相同：

```xml
<param name="grid_map/frame_id" value="实机定位父坐标系" type="str"/>
```

例如实机里程计是 `odom -> base_link`：

```xml
<param name="grid_map/frame_id" value="odom" type="str"/>
```

例如实机里程计是 `map -> base_link`：

```xml
<param name="grid_map/frame_id" value="map" type="str"/>
```

### 3. `bspline_to_path.cpp`

文件：

```text
/home/ubuntu/Robit_SlamNav/NavOA_ws/src/SCAN-Planner/src/planner/plan_manage/src/bspline_to_path.cpp
```

原始默认输出坐标系：

```cpp
pnh_.param<std::string>("frame_id", frame_id_, "map");
```

将 `map` 改为实机里程计的父坐标系，例如：

```cpp
pnh_.param<std::string>("frame_id", frame_id_, "odom");
```

也可以不改源码，启动节点时传入：

```bash
rosrun scan_planner bspline_to_path _frame_id:=odom
```

以下内部话题保持不变：

```text
输入：/planning/bspline
输出：/planning/path
```

## 二、原始 NavOA 中需要修改的文件

### 1. `path_segment_publisher.py`

文件：

```text
/home/ubuntu/Robit_SlamNav/NavOA_ws/src/NavOA_ros/scripts/path_segment_publisher.py
```

#### 将 move_base 路径改为 SCAN 路径

原始内容：

```python
rospy.Subscriber(
    '/move_base/GlobalPlanner/plan', Path, self.path_callback)
```

改为：

```python
rospy.Subscriber(
    '/planning/path', Path, self.path_callback)
```

以下话题保持不变：

```python
rospy.Subscriber(
    '/move_base_simple/goal', PoseStamped, self.goal_reset_callback)

self.goal_pub = rospy.Publisher(
    '/split_goal', PoseStamped, queue_size=1)
```

### 2. `NavOA_gazebo_limo.launch`

文件：

```text
/home/ubuntu/Robit_SlamNav/NavOA_ws/src/NavOA_ros/example/gazebo_limo/launch/NavOA_gazebo_limo.launch
```

#### 定位坐标系

原始内容：

```xml
<arg name="map_frame" default="map"/>
```

改为实机里程计父坐标系：

```xml
<arg name="map_frame" default="实机定位父坐标系"/>
```

#### 底盘坐标系

原始内容：

```xml
<arg name="base_frame" default="base_link"/>
```

改为实机底盘坐标系：

```xml
<arg name="base_frame" default="实机底盘坐标系"/>
```

#### 激光坐标系

原始内容：

```xml
<arg name="lidar_frame" default="laser_frame"/>
```

改为实机二维激光坐标系：

```xml
<arg name="lidar_frame" default="实机二维激光坐标系"/>
```

#### 二维激光话题

原始内容：

```xml
<remap from="/scan" to="/scan"/>
```

改为：

```xml
<remap from="/scan" to="/实机二维激光话题"/>
```

消息类型必须是：

```text
sensor_msgs/LaserScan
```

同时在 `vel_filter` 节点中加入：

```xml
<param name="scan_topic" value="/实机二维激光话题"/>
```

#### NavOA 子目标

确认保留：

```xml
<remap from="/NavOA_goal" to="/split_goal"/>
```

#### 底盘速度话题

NavOA 原始速度先输出到中间话题：

```xml
<remap from="/NavOA_cmd_vel" to="/cmd_vel_tem"/>
```

在 `vel_filter` 节点中，将：

```xml
<param name="cmd_out_topic" value="/cmd_vel"/>
```

改为实机底盘接收的速度话题：

```xml
<param name="cmd_out_topic" value="/实机底盘速度话题"/>
```

消息类型必须是：

```text
geometry_msgs/Twist
```

### 3. `NavOA_core.py`

文件：

```text
/home/ubuntu/Robit_SlamNav/NavOA_ws/src/NavOA_ros/src/NavOA_core.py
```

如果不使用 launch remap，则将原始二维激光订阅：

```python
rospy.Subscriber('/scan', LaserScan, self.scan_callback)
```

改为：

```python
rospy.Subscriber(
    '/实机二维激光话题', LaserScan, self.scan_callback)
```

推荐使用 launch remap，不直接修改此 Python 文件。

## 三、不要启动的原始文件

不要启动：

```text
/home/ubuntu/Robit_SlamNav/NavOA_ws/src/NavOA_ros/launch/navoa_nav_only.launch
```

该文件会启动原 NavOA 的 `move_base`、GlobalPlanner 和 costmap。

使用 SCAN 提供 `/planning/path` 后，不再启动：

```text
/move_base
/move_base/GlobalPlanner
/move_base/GlobalPlanner/plan
/move_base/global_costmap
/move_base/local_costmap
```

只启动原 NavOA 控制文件：

```bash
roslaunch NavOA_ros NavOA_gazebo_limo.launch
```

## 四、原始 SCAN 控制器需要关闭

文件：

```text
/home/ubuntu/Robit_SlamNav/NavOA_ws/src/SCAN-Planner/src/planner/plan_manage/launch/run.launch
```

将：

```xml
<arg name="controller_mode" default="closed_loop"/>
```

改为：

```xml
<arg name="controller_mode" default="none"/>
```

保证以下节点不启动：

```text
closed_loop_controller
open_loop_controller
```

最终只有 NavOA 的 `vel_filter` 可以向实机底盘速度话题发布。

## 五、需要替换的话题汇总

| 原始话题或坐标系 | 替换为实机上的内容 | 修改文件 |
|---|---|---|
| `/LIO/odom_vehicle` | 实机车体里程计 | SCAN `run.launch` |
| `/LIO/odom_imu` | 实机传感器位姿 | SCAN `run.launch` |
| `/LIO/clouds_lidar` | 实机三维点云 | SCAN `run.launch` |
| `/camera/aligned_depth_to_color/image_raw` | 实机深度图 | SCAN `run.launch` |
| `world` 或 `map` | 实机定位父坐标系 | SCAN `advanced_param.xml`、转换节点、NavOA launch |
| `base_link` | 实机底盘坐标系 | NavOA launch |
| `laser_frame` | 实机二维激光坐标系 | NavOA launch |
| `/move_base/GlobalPlanner/plan` | `/planning/path` | `path_segment_publisher.py` |
| `/scan` | 实机二维 LaserScan | NavOA launch |
| `/cmd_vel` | 实机底盘速度话题 | NavOA launch 的 `vel_filter` |

以下内部话题不要修改：

```text
/move_base_simple/goal
/planning/bspline
/planning/path
/split_goal
/NavOA_cmd_vel
/cmd_vel_tem
```
