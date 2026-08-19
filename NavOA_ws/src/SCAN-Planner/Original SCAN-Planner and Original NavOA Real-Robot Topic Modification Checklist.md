# Original SCAN-Planner and Original NavOA Real-Robot Topic Modification Checklist

## 1. Files to Modify in SCAN-Planner

### 1. `run.launch`

File:

```text
/home/ubuntu/Robit_SlamNav/NavOA_ws/src/SCAN-Planner/src/planner/plan_manage/launch/run.launch
```

#### Body Odometry

Original content:

```xml
<arg name="body_pose_topic"
     value="$(eval '/LIO/odom_vehicle' if arg('is_real_world') else '/quad_0/body_pose')" />
```

Change to the real-robot body odometry topic:

```xml
<arg name="body_pose_topic" default="/real-robot body odometry topic" />
```

The message type must be:

```text
nav_msgs/Odometry
```

For example:

```xml
<arg name="body_pose_topic" default="/odom" />
```

#### 3D Lidar or Camera Pose

Original content:

```xml
<arg name="sensor_pose_topic"
     value="$(eval '/LIO/odom_imu' if arg('is_real_world') else ('/quad_0/camera_pose' if arg('sensor_type') == 'depth' else '/quad_0/lidar_pose'))" />
```

Change to the real-robot sensor pose topic:

```xml
<arg name="sensor_pose_topic" default="/real-robot sensor pose topic" />
```

The message type must be:

```text
nav_msgs/Odometry
```

#### 3D Point Cloud

Original content:

```xml
<arg name="cloud_topic"
     value="$(eval '/LIO/clouds_lidar' if arg('is_real_world') else '/pcl_render_node/cloud')"/>
```

Change to the real-robot 3D lidar point cloud:

```xml
<arg name="cloud_topic" default="/real-robot 3D point cloud topic"/>
```

The message type must be:

```text
sensor_msgs/PointCloud2
```

#### Frame in Which the Point Cloud Resides

Original real-robot configuration:

```xml
<arg name="cloud_is_world" value="false"/>
```

Modify according to the real-robot point cloud:

```xml
<!-- point cloud is in the lidar's own frame -->
<arg name="cloud_is_world" default="false"/>

<!-- point cloud has already been transformed to the localization world frame -->
<arg name="cloud_is_world" default="true"/>
```

#### Depth Image

Modify only when `sensor_type=depth`.

Original content:

```xml
<arg name="depth_topic"
     value="$(eval '/camera/aligned_depth_to_color/image_raw' if arg('is_real_world') else '/pcl_render_node/depth')"/>
```

Change to:

```xml
<arg name="depth_topic" default="/real-robot depth image topic"/>
```

The message type must be:

```text
sensor_msgs/Image
```

### 2. `advanced_param.xml`

File:

```text
/home/ubuntu/Robit_SlamNav/NavOA_ws/src/SCAN-Planner/src/planner/plan_manage/launch/advanced_param.xml
```

Confirm that SCAN's world frame is the same as the real-robot odometry's `header.frame_id`:

```xml
<param name="grid_map/frame_id" value="real-robot localization parent frame" type="str"/>
```

For example, if the real-robot odometry is `odom -> base_link`:

```xml
<param name="grid_map/frame_id" value="odom" type="str"/>
```

For example, if the real-robot odometry is `map -> base_link`:

```xml
<param name="grid_map/frame_id" value="map" type="str"/>
```

### 3. `bspline_to_path.cpp`

File:

```text
/home/ubuntu/Robit_SlamNav/NavOA_ws/src/SCAN-Planner/src/planner/plan_manage/src/bspline_to_path.cpp
```

Original default output frame:

```cpp
pnh_.param<std::string>("frame_id", frame_id_, "map");
```

Change `map` to the real-robot odometry's parent frame, for example:

```cpp
pnh_.param<std::string>("frame_id", frame_id_, "odom");
```

You can also avoid modifying the source code by passing it in at node startup:

```bash
rosrun scan_planner bspline_to_path _frame_id:=odom
```

Keep the following internal topics unchanged:

```text
Input:  /planning/bspline
Output: /planning/path
```

## 2. Files to Modify in the Original NavOA

### 1. `path_segment_publisher.py`

File:

```text
/home/ubuntu/Robit_SlamNav/NavOA_ws/src/NavOA_ros/scripts/path_segment_publisher.py
```

#### Change the move_base Path to the SCAN Path

Original content:

```python
rospy.Subscriber(
    '/move_base/GlobalPlanner/plan', Path, self.path_callback)
```

Change to:

```python
rospy.Subscriber(
    '/planning/path', Path, self.path_callback)
```

Keep the following topics unchanged:

```python
rospy.Subscriber(
    '/move_base_simple/goal', PoseStamped, self.goal_reset_callback)

self.goal_pub = rospy.Publisher(
    '/split_goal', PoseStamped, queue_size=1)
```

### 2. `NavOA_gazebo_limo.launch`

File:

```text
/home/ubuntu/Robit_SlamNav/NavOA_ws/src/NavOA_ros/example/gazebo_limo/launch/NavOA_gazebo_limo.launch
```

#### Localization Frame

Original content:

```xml
<arg name="map_frame" default="map"/>
```

Change to the real-robot odometry parent frame:

```xml
<arg name="map_frame" default="real-robot localization parent frame"/>
```

#### Base Frame

Original content:

```xml
<arg name="base_frame" default="base_link"/>
```

Change to the real-robot base frame:

```xml
<arg name="base_frame" default="real-robot base frame"/>
```

#### Laser Frame

Original content:

```xml
<arg name="lidar_frame" default="laser_frame"/>
```

Change to the real-robot 2D laser frame:

```xml
<arg name="lidar_frame" default="real-robot 2D laser frame"/>
```

#### 2D Laser Topic

Original content:

```xml
<remap from="/scan" to="/scan"/>
```

Change to:

```xml
<remap from="/scan" to="/real-robot 2D laser topic"/>
```

The message type must be:

```text
sensor_msgs/LaserScan
```

Also add to the `vel_filter` node:

```xml
<param name="scan_topic" value="/real-robot 2D laser topic"/>
```

#### NavOA Sub-Goal

Confirm that the following is retained:

```xml
<remap from="/NavOA_goal" to="/split_goal"/>
```

#### Base Velocity Topic

NavOA's raw velocity is first output to an intermediate topic:

```xml
<remap from="/NavOA_cmd_vel" to="/cmd_vel_tem"/>
```

In the `vel_filter` node, change:

```xml
<param name="cmd_out_topic" value="/cmd_vel"/>
```

to the velocity topic received by the real-robot base:

```xml
<param name="cmd_out_topic" value="/real-robot base velocity topic"/>
```

The message type must be:

```text
geometry_msgs/Twist
```

### 3. `NavOA_core.py`

File:

```text
/home/ubuntu/Robit_SlamNav/NavOA_ws/src/NavOA_ros/src/NavOA_core.py
```

If launch remapping is not used, change the original 2D laser subscription:

```python
rospy.Subscriber('/scan', LaserScan, self.scan_callback)
```

to:

```python
rospy.Subscriber(
    '/real-robot 2D laser topic', LaserScan, self.scan_callback)
```

Using launch remapping is recommended; do not modify this Python file directly.

## 3. Original Files Not to Launch

Do not launch:

```text
/home/ubuntu/Robit_SlamNav/NavOA_ws/src/NavOA_ros/launch/navoa_nav_only.launch
```

This file starts the original NavOA's `move_base`, GlobalPlanner, and costmap.

After using SCAN to provide `/planning/path`, no longer start:

```text
/move_base
/move_base/GlobalPlanner
/move_base/GlobalPlanner/plan
/move_base/global_costmap
/move_base/local_costmap
```

Only launch the original NavOA control file:

```bash
roslaunch NavOA_ros NavOA_gazebo_limo.launch
```

## 4. The Original SCAN Controllers Need to Be Disabled

File:

```text
/home/ubuntu/Robit_SlamNav/NavOA_ws/src/SCAN-Planner/src/planner/plan_manage/launch/run.launch
```

Change:

```xml
<arg name="controller_mode" default="closed_loop"/>
```

to:

```xml
<arg name="controller_mode" default="none"/>
```

Ensure the following nodes are not started:

```text
closed_loop_controller
open_loop_controller
```

In the end, only NavOA's `vel_filter` may publish to the real-robot base velocity topic.

## 5. Summary of Topics to Replace

| Original Topic or Frame | Replace With Real-Robot Content | Modified File |
|---|---|---|
| `/LIO/odom_vehicle` | real-robot body odometry | SCAN `run.launch` |
| `/LIO/odom_imu` | real-robot sensor pose | SCAN `run.launch` |
| `/LIO/clouds_lidar` | real-robot 3D point cloud | SCAN `run.launch` |
| `/camera/aligned_depth_to_color/image_raw` | real-robot depth image | SCAN `run.launch` |
| `world` or `map` | real-robot localization parent frame | SCAN `advanced_param.xml`, conversion node, NavOA launch |
| `base_link` | real-robot base frame | NavOA launch |
| `laser_frame` | real-robot 2D laser frame | NavOA launch |
| `/move_base/GlobalPlanner/plan` | `/planning/path` | `path_segment_publisher.py` |
| `/scan` | real-robot 2D LaserScan | NavOA launch |
| `/cmd_vel` | real-robot base velocity topic | `vel_filter` in NavOA launch |

Do not modify the following internal topics:

```text
/move_base_simple/goal
/planning/bspline
/planning/path
/split_goal
/NavOA_cmd_vel
/cmd_vel_tem
```
