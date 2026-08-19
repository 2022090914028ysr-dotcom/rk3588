# SCAN-Planner and Original NavOA Real-Robot Integration Modification Notes

This document explains how to combine the original SCAN-Planner and the original NavOA into the following system:

```text
User goal /move_base_simple/goal
              |
              v
        SCAN-Planner
              |
       /planning/bspline
              |
              v
       bspline_to_path
              |
        /planning/path
              |
              v
  path_segment_publisher
              |
         /split_goal
              |
              v
            NavOA  <----- /scan
              |
       /NavOA_cmd_vel
              |
          vel_filter
              |
           /cmd_vel
              |
            base
```

Final division of responsibilities:

- SCAN-Planner: receives the final goal and continuously generates a local collision-free path based on the 3D point cloud and robot pose.
- `bspline_to_path`: converts SCAN's custom B-spline message into a standard `nav_msgs/Path`.
- `path_segment_publisher`: splits the local path by distance into NavOA sub-goals.
- NavOA: tracks the current sub-goal and performs local dynamic obstacle avoidance based on the 2D laser.
- `vel_filter`: velocity limiting and the last-level safety protection; it is the only node that publishes to the base `/cmd_vel`.

> SCAN-Planner itself is a local planner, not a traditional move_base global planner. This combination uses
> SCAN's continuous local replanning to replace the move_base GlobalPlanner in the original NavOA system.

## 1. Modifications Already Completed in SCAN-Planner

### 1.1 Added a B-spline to Path Conversion Node

New file:

```text
src/planner/plan_manage/src/bspline_to_path.cpp
```

Node interface:

| Type | Topic | Message Type |
|---|---|---|
| Subscribe | `/planning/bspline` | `scan_planner/Bspline` |
| Publish | `/planning/path` | `nav_msgs/Path` |

The conversion node samples the B-spline over time and generates the yaw of each path point from the direction of adjacent points. Main parameters:

| Parameter | Default | Description |
|---|---:|---|
| `~input_topic` | `/planning/bspline` | SCAN trajectory input |
| `~output_topic` | `/planning/path` | standard path output |
| `~frame_id` | `map` | output path frame |
| `~sample_dt` | `0.05` | B-spline sampling time interval, in seconds |
| `~min_point_distance` | `0.03` | minimum 2D distance between adjacent output points |
| `~flatten_z` | `false` | whether to zero out the output path z |

A `bspline_to_path` build target was added to `src/planner/plan_manage/CMakeLists.txt`.

### 1.2 SCAN Real-Robot Topics Changed to Overridable from Launch

In the original `run.launch`, the following topics were fixed with `value=`, which was not convenient to override from an external launch; they have now been changed to `default=` parameters:

- `body_pose_topic`
- `sensor_pose_topic`
- `cloud_topic`
- `cloud_is_world`
- `depth_topic`
- `world_frame`

Therefore, the combined launch can directly pass in the actual robot's localization and sensor topics.

### 1.3 Added Pure-Planning Mode

The combined system passes the following to `run.launch`:

```xml
<arg name="controller_mode" value="none"/>
```

This prevents SCAN's `closed_loop_controller` or `open_loop_controller` from starting. Otherwise SCAN
and NavOA would publish velocities simultaneously, causing a robot control conflict.

### 1.4 Go2 and Simulation Helper Nodes Can Be Disabled

`run.launch` adds:

```text
launch_go2_support
launch_odom_visualization
```

Both are set to `false` during real-robot combined startup, avoiding overwriting the real robot's `robot_description` and avoiding name conflicts with an already-running `robot_state_publisher`, Go2 gait node, or odometry visualization node.

### 1.5 Added a Combined Launch File

The combined launch file is located in the NavOA project:

```text
/home/ubuntu/Robit_SlamNav/NavOA_ws/src/NavOA_ros/launch/scan_navoa.launch
```

This file launches simultaneously:

1. SCAN-Planner in pure-planning mode;
2. `bspline_to_path`;
3. the path segmentation node;
4. the NavOA control node;
5. `vel_filter`.

## 2. How to Modify Starting from the Original NavOA Version

This section explains in "original code -> modified code" form. If a fresh copy of the original NavOA is downloaded again, the following modifications should be made.

### 2.1 Make the Path Segmentation Node's Path Input Configurable

The original `scripts/path_segment_publisher.py` fixed the subscription to the move_base path:

```python
rospy.Subscriber(
    '/move_base/GlobalPlanner/plan', Path, self.path_callback)
```

Should be changed to:

```python
self.path_topic = rospy.get_param(
    '~path_topic', '/move_base/GlobalPlanner/plan')

rospy.Subscriber(self.path_topic, Path, self.path_callback)
```

Keeping the move_base path as the default guarantees that old launch files remain compatible; when combined with SCAN, set:

```xml
<param name="path_topic" value="/planning/path"/>
```

### 2.2 Make the Segmented Goal Topic Configurable

The original code fixed the publication:

```python
self.goal_pub = rospy.Publisher(
    '/split_goal', PoseStamped, queue_size=1)
```

Should be changed to:

```python
self.split_goal_topic = rospy.get_param(
    '~split_goal_topic', '/split_goal')

self.goal_pub = rospy.Publisher(
    self.split_goal_topic, PoseStamped, queue_size=1)
```

It is also recommended to parameterize the following topics as well:

```python
self.goal_input_topic = rospy.get_param(
    '~goal_input_topic', '/move_base_simple/goal')
self.cancel_topic = rospy.get_param(
    '~cancel_topic', '/move_base/cancel')
```

Use these parameters both when subscribing and when republishing goals, and no longer hardcode topic names repeatedly in the code.

### 2.3 Parameterize the Path Segmentation Distance and Arrival Thresholds

Keep at least the following parameters:

```python
self.segment_length = rospy.get_param('~segment_length', 1.5)
self.reach_tolerance = rospy.get_param('~reach_tolerance', 0.5)
self.final_tolerance = rospy.get_param('~final_tolerance', 0.15)
```

SCAN outputs a local path about one planning horizon long, so it is not appropriate to keep using a very large global-path segmentation distance. Suggested initial values for the combined system:

```text
segment_length: 0.8 m
reach_tolerance: 0.35 m
final_tolerance: 0.15 m
```

If the robot sways left/right or changes goals frequently, increase `segment_length` to `1.0 m`; if turning tracking is not timely in narrow environments, decrease it to `0.6 m`.

### 2.4 Keep NavOA Goal Input as `/NavOA_goal`, Receive Sub-Goals via Remap

The original `NavOA_core.py` already supports:

```python
rospy.Subscriber('/NavOA_goal', PoseStamped, self.goal_callback)
```

This part does not need to be rewritten; only add to the launch file:

```xml
<remap from="/NavOA_goal" to="/split_goal"/>
```

It is not recommended to remap `/planning/path` directly to NavOA's `/initial_path`, because this plan explicitly requires the segmentation node to manage sub-goal switching and final stop accuracy first.

### 2.5 Disable the Global Planner move_base Bundled with the Original NavOA

The original system's:

```text
launch/navoa_nav_only.launch
```

starts `move_base`, GlobalPlanner, and the global/local costmaps. When combining with SCAN, do not launch this file, and do not separately launch the following nodes:

```text
/move_base
/move_base/GlobalPlanner
move_base global_costmap
move_base local costmap
```

Reasons:

1. `/planning/path` is already provided by SCAN, so `/move_base/GlobalPlanner/plan` is no longer needed;
2. move_base would receive the same `/move_base_simple/goal`, forming a second planning chain;
3. move_base may publish its own velocity, competing with NavOA for the base;
4. the original path-recovery logic may call `/move_base/clear_costmaps`, but that service does not exist in the combined system.

When combining, only launch the control part of the original NavOA, namely:

```text
NavOA_node.py
path_segment_publisher.py
vel_filter.py
```

The current project directly includes:

```text
example/gazebo_limo/launch/NavOA_gazebo_limo.launch
```

to launch these three parts, without including `navoa_nav_only.launch`.

### 2.6 Disable the Original move_base-Specific Path Recovery Features

The no-progress recovery, blocked-path memory, and costmap-clearing features in the original `path_segment_publisher.py` are written around move_base. SCAN continuously replans based on new point clouds, so set the following in the combined launch:

```xml
<arg name="path_progress_timeout" value="0.0"/>
<arg name="blocked_route_memory_enabled" value="false"/>
```

When migrating from the original NavOA, also ensure the following interfaces are not called periodically:

```text
/move_base/clear_costmaps
/move_base/cancel
/navoa/blocked_route_points -> move_base obstacle layer
```

`/move_base/cancel` can be kept temporarily as a compatibility interface, but without move_base it will not participate in normal combined navigation.

### 2.7 Disable NavOA's "Return to Previous Goal" Behavior

SCAN continuously updates the local path and sub-goals. If NavOA automatically returns to an old goal after a sub-goal is reached, it will cause back-and-forth motion. The combined system must set:

```xml
<param name="return_to_previous_goal_enabled" value="false"/>
<param name="goal_timeout_button_enabled" value="false"/>
<param name="require_explicit_goal" value="true"/>
```

### 2.8 Ensure There Is Only One Final Velocity Publisher

Recommended velocity chain:

```text
NavOA:       /NavOA_cmd_vel -> remap /cmd_vel_tem
vel_filter:  /cmd_vel_tem -> /cmd_vel
```

The original NavOA launch should include:

```xml
<remap from="/NavOA_cmd_vel" to="/cmd_vel_tem"/>
```

`vel_filter` should set:

```xml
<param name="cmd_in_topic" value="/cmd_vel_tem"/>
<param name="cmd_out_topic" value="/cmd_vel"/>
```

The following potential velocity sources must be disabled or remapped:

- SCAN `closed_loop_controller`;
- SCAN `open_loop_controller`;
- move_base controller;
- keyboard teleop node;
- other test nodes that directly publish `/cmd_vel`.

Check command:

```bash
rostopic info /cmd_vel
```

Normally the only Publisher is `vel_filter`.

### 2.9 Change NavOA's 2D Laser Topic to the Real-Robot Topic

Both NavOA and `vel_filter` need `sensor_msgs/LaserScan`. The original NavOA typically uses `/scan`, and the launch should pass the actual topic to both:

```xml
<arg name="scan_topic" default="/scan"/>

<remap from="/scan" to="$(arg scan_topic)"/>
<param name="scan_topic" value="$(arg scan_topic)"/>
```

If the lidar only outputs `PointCloud2`, first use the driver's own conversion node or
`pointcloud_to_laserscan` to generate a LaserScan; do not fill the point cloud topic directly into NavOA.

## 3. Topics the Real Robot Must Confirm or Replace

The following are the default values of the current combined launch. Each must be confirmed against the real robot's `rostopic list` and message types.

| Function | Current Default Topic | Type | Real-Robot Requirement |
|---|---|---|---|
| Body pose | `/LIO/odom_vehicle` | `nav_msgs/Odometry` | change to real-robot localization output |
| Sensor pose | `/LIO/odom_imu` | `nav_msgs/Odometry` | change to the lidar/camera's corresponding pose |
| SCAN point cloud | `/LIO/clouds_lidar` | `sensor_msgs/PointCloud2` | change to the 3D lidar point cloud |
| Depth image | `/camera/aligned_depth_to_color/image_raw` | `sensor_msgs/Image` | only used when `sensor_type:=depth` |
| NavOA laser | `/scan` | `sensor_msgs/LaserScan` | change to the real-robot 2D scan |
| User goal | `/move_base_simple/goal` | `geometry_msgs/PoseStamped` | RViz 2D Nav Goal |
| SCAN B-spline | `/planning/bspline` | `scan_planner/Bspline` | internal topic, usually unchanged |
| Standard path | `/planning/path` | `nav_msgs/Path` | internal topic, usually unchanged |
| NavOA sub-goal | `/split_goal` | `geometry_msgs/PoseStamped` | internal topic, usually unchanged |
| NavOA raw velocity | `/cmd_vel_tem` | `geometry_msgs/Twist` | internal topic |
| Base velocity | `/cmd_vel` | `geometry_msgs/Twist` | change to the base's actual control topic |

Check message types:

```bash
rostopic type /LIO/odom_vehicle
rostopic type /LIO/odom_imu
rostopic type /LIO/clouds_lidar
rostopic type /scan
```

If the base receives another topic such as `/limo_base_controller/cmd_vel`, pass it in at startup:

```bash
cmd_vel_topic:=/limo_base_controller/cmd_vel
```

## 4. Coordinate Frame and TF Requirements

This is the most important item in real-robot adaptation.

`world_frame`, the frame in which the body odometry values live, the SCAN path frame, and NavOA's `map_frame`
must represent the same coordinate frame. In addition, the TF tree must contain:

```text
world_frame -> base_frame
world_frame -> lidar_frame
```

Check the odometry parent frame:

```bash
rostopic echo -n1 /LIO/odom_vehicle/header
```

Check TF:

```bash
rosrun tf tf_echo map base_link
rosrun tf tf_echo map laser_frame
```

If the odometry's `header.frame_id` is `odom`, `world`, or `camera_init`, that name should be passed to:

```bash
world_frame:=actual parent frame
```

Do not only modify the string on the message to fake the coordinate frame; if SCAN and NavOA use different frames, first provide the correct static or dynamic TF and perform a real coordinate transformation on the path/goal.

## 5. Point Cloud Coordinate Parameter

Meaning of `cloud_is_world`:

- `true`: the points in the point cloud are already in `world_frame`;
- `false`: the point cloud is in the lidar's own frame, and SCAN uses the sensor pose to transform it to the world frame.

Most original lidar drivers output point clouds in the lidar frame, so use:

```bash
cloud_is_world:=false
```

Only set it to `true` if the upstream LIO explicitly publishes an undistorted point cloud that has already been transformed to the world frame. Setting it incorrectly will cause obstacles to move with the robot or appear in wrong positions.

## 6. Build and Startup

Build the new SCAN node:

```bash
cd /home/ubuntu/Robit_SlamNav/NavOA_ws/src/SCAN-Planner
source /opt/ros/noetic/setup.bash
catkin_make -j2
```

Before startup, first stop the following original launches:

```text
roslaunch scan_planner run.launch
roslaunch NavOA_ros navoa_nav_only.launch
any separately running move_base
```

Start the combined system:

```bash
source /home/ubuntu/Robit_SlamNav/NavOA_ws/src/miniconda3/etc/profile.d/conda.sh
conda activate neupan
source /home/ubuntu/Robit_SlamNav/NavOA_ws/src/SCAN-Planner/devel/setup.bash
export ROS_PACKAGE_PATH=/home/Robit_SlamNav:$ROS_PACKAGE_PATH

roslaunch NavOA_ros scan_navoa.launch \
  world_frame:=map \
  base_frame:=base_link \
  lidar_frame:=laser_frame \
  body_pose_topic:=/LIO/odom_vehicle \
  sensor_pose_topic:=/LIO/odom_imu \
  cloud_topic:=/LIO/clouds_lidar \
  cloud_is_world:=false \
  scan_topic:=/scan \
  cmd_vel_topic:=/cmd_vel
```

If the current platform has no RKNN runtime environment:

```bash
roslaunch NavOA_ros scan_navoa.launch use_rknn:=false
```

## 7. Safety Check Before First Real-Robot Run

For the first test, jack up the wheels or disconnect the base enable, and only check the topic chain:

```bash
rostopic hz /LIO/odom_vehicle
rostopic hz /LIO/clouds_lidar
rostopic hz /scan
rostopic hz /planning/bspline
rostopic hz /planning/path
rostopic echo /split_goal
rostopic hz /NavOA_cmd_vel
rostopic hz /cmd_vel
```

Judge in order:

1. No `/planning/bspline`: check SCAN pose, point cloud, goal, and coordinate frame.
2. Has B-spline but no `/planning/path`: check whether `bspline_to_path` is running.
3. Has Path but no `/split_goal`: check `path_segment_publisher`'s `path_topic`.
4. Has sub-goal but no velocity: check NavOA TF, `/scan`, model files, and Python environment.
5. Has velocity but the base does not move: check `cmd_vel_topic` and base enable state.
6. Obstacles in the wrong direction: focus on `laser_frame`, the LaserScan zero-angle direction, and TF.

For the first ground-driving run, it is recommended to set:

```text
ref_speed:=0.15
segment_length:=0.6
```

After confirming that direction, coordinate frames, and emergency stop are all normal, gradually increase the speed.
