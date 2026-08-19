# SCAN-Planner 与原始 NavOA 实机集成修改说明

本文说明如何把原始 SCAN-Planner 和原始 NavOA 组合成以下系统：

```text
用户终点 /move_base_simple/goal
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
             底盘
```

最终职责划分：

- SCAN-Planner：接收最终目标，根据三维点云和机器人位姿持续生成局部无碰路径。
- `bspline_to_path`：将 SCAN 自定义 B 样条消息转换成标准 `nav_msgs/Path`。
- `path_segment_publisher`：将局部路径按距离切分成 NavOA 子目标。
- NavOA：跟踪当前子目标，并根据二维激光进行局部动态避障。
- `vel_filter`：速度限幅和最后一级安全保护，是唯一向底盘 `/cmd_vel` 发布的节点。

> SCAN-Planner 本身是局部规划器，不是传统 move_base 全局规划器。本组合使用
> SCAN 的连续局部重规划替代原 NavOA 系统中的 move_base GlobalPlanner。

## 1. SCAN-Planner 已完成的修改

### 1.1 新增 B 样条到 Path 的转换节点

新增文件：

```text
src/planner/plan_manage/src/bspline_to_path.cpp
```

节点接口：

| 类型 | 话题 | 消息类型 |
|---|---|---|
| 订阅 | `/planning/bspline` | `scan_planner/Bspline` |
| 发布 | `/planning/path` | `nav_msgs/Path` |

转换节点对 B 样条按时间采样，并根据相邻点方向生成每个路径点的 yaw。主要参数：

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `~input_topic` | `/planning/bspline` | SCAN 轨迹输入 |
| `~output_topic` | `/planning/path` | 标准路径输出 |
| `~frame_id` | `map` | 输出路径坐标系 |
| `~sample_dt` | `0.05` | B 样条采样时间间隔，单位秒 |
| `~min_point_distance` | `0.03` | 相邻输出点的最小二维距离 |
| `~flatten_z` | `false` | 是否将输出路径 z 置零 |

`src/planner/plan_manage/CMakeLists.txt` 中增加了 `bspline_to_path` 编译目标。

### 1.2 SCAN 实机话题改为可从 launch 覆盖

原始 `run.launch` 中以下话题由 `value=` 固定，不便于外部 launch 覆盖；现在改为
`default=` 参数：

- `body_pose_topic`
- `sensor_pose_topic`
- `cloud_topic`
- `cloud_is_world`
- `depth_topic`
- `world_frame`

因此组合 launch 可以直接传入实际机器人的定位和传感器话题。

### 1.3 增加纯规划模式

组合系统向 `run.launch` 传入：

```xml
<arg name="controller_mode" value="none"/>
```

这样不会启动 SCAN 的 `closed_loop_controller` 或 `open_loop_controller`。否则 SCAN
和 NavOA 会同时发布速度，造成机器人控制冲突。

### 1.4 可关闭 Go2 和仿真辅助节点

`run.launch` 新增：

```text
launch_go2_support
launch_odom_visualization
```

实机组合启动时均设为 `false`，避免覆盖实机的 `robot_description`，也避免与已经运行
的 `robot_state_publisher`、Go2 步态节点或里程计可视化节点重名。

### 1.5 新增组合启动文件

组合启动文件位于 NavOA 工程：

```text
/home/ubuntu/Robit_SlamNav/NavOA_ws/src/NavOA_ros/launch/scan_navoa.launch
```

该文件同时启动：

1. 纯规划模式的 SCAN-Planner；
2. `bspline_to_path`；
3. 路径分割节点；
4. NavOA 控制节点；
5. `vel_filter`。

## 2. 从原始 NavOA 版本开始应该如何修改

本节按“原始代码 -> 修改后代码”的方式说明。若重新下载一份原始 NavOA，应完成
以下修改。

### 2.1 将路径分割节点的路径输入改为可配置

原始 `scripts/path_segment_publisher.py` 固定订阅 move_base 路径：

```python
rospy.Subscriber(
    '/move_base/GlobalPlanner/plan', Path, self.path_callback)
```

应改为：

```python
self.path_topic = rospy.get_param(
    '~path_topic', '/move_base/GlobalPlanner/plan')

rospy.Subscriber(self.path_topic, Path, self.path_callback)
```

保留 move_base 路径作为默认值，可以保证旧启动文件仍兼容；与 SCAN 组合时设置：

```xml
<param name="path_topic" value="/planning/path"/>
```

### 2.2 将分割后的目标话题改为可配置

原始代码固定发布：

```python
self.goal_pub = rospy.Publisher(
    '/split_goal', PoseStamped, queue_size=1)
```

应改为：

```python
self.split_goal_topic = rospy.get_param(
    '~split_goal_topic', '/split_goal')

self.goal_pub = rospy.Publisher(
    self.split_goal_topic, PoseStamped, queue_size=1)
```

同时建议将以下话题一并参数化：

```python
self.goal_input_topic = rospy.get_param(
    '~goal_input_topic', '/move_base_simple/goal')
self.cancel_topic = rospy.get_param(
    '~cancel_topic', '/move_base/cancel')
```

订阅和重新发布目标时均使用这些参数，不再在代码中重复写死话题名。

### 2.3 将路径切分距离和到达阈值参数化

至少保留以下参数：

```python
self.segment_length = rospy.get_param('~segment_length', 1.5)
self.reach_tolerance = rospy.get_param('~reach_tolerance', 0.5)
self.final_tolerance = rospy.get_param('~final_tolerance', 0.15)
```

SCAN 输出的是约一个规划视野长度的局部路径，不宜继续使用很大的全局路径切分距离。
组合系统建议初值：

```text
segment_length: 0.8 m
reach_tolerance: 0.35 m
final_tolerance: 0.15 m
```

若机器人左右摆动或频繁更换目标，可将 `segment_length` 增大到 `1.0 m`；若狭窄环境
转弯跟踪不及时，可减小到 `0.6 m`。

### 2.4 NavOA 目标输入保持 `/NavOA_goal`，通过 remap 接收子目标

原始 `NavOA_core.py` 已支持：

```python
rospy.Subscriber('/NavOA_goal', PoseStamped, self.goal_callback)
```

该部分不需要重写，只需要在 launch 中加入：

```xml
<remap from="/NavOA_goal" to="/split_goal"/>
```

不建议把 `/planning/path` 直接 remap 到 NavOA 的 `/initial_path`，因为本方案明确要求先
由分割节点管理子目标切换和最终停止精度。

### 2.5 关闭原 NavOA 自带的全局规划器 move_base

原系统的：

```text
launch/navoa_nav_only.launch
```

会启动 `move_base`、GlobalPlanner 和全局/局部 costmap。与 SCAN 组合时不要启动该文件，
也不要单独启动以下节点：

```text
/move_base
/move_base/GlobalPlanner
move_base global_costmap
move_base local costmap
```

原因：

1. `/planning/path` 已由 SCAN 提供，不再需要 `/move_base/GlobalPlanner/plan`；
2. move_base 会接收同一个 `/move_base_simple/goal`，形成第二套规划链路；
3. move_base 可能发布自己的速度，和 NavOA 争夺底盘；
4. 原路径恢复逻辑可能调用 `/move_base/clear_costmaps`，但组合系统中该服务不存在。

组合时只启动原 NavOA 中的控制部分，即：

```text
NavOA_node.py
path_segment_publisher.py
vel_filter.py
```

当前工程通过直接包含：

```text
example/gazebo_limo/launch/NavOA_gazebo_limo.launch
```

来启动这三个部分，没有包含 `navoa_nav_only.launch`。

### 2.6 关闭原 move_base 专用的路径恢复功能

原 `path_segment_publisher.py` 中的无进展恢复、阻塞路径记忆和清 costmap 功能是围绕
move_base 编写的。SCAN 会持续根据新点云重规划，因此组合 launch 中设置：

```xml
<arg name="path_progress_timeout" value="0.0"/>
<arg name="blocked_route_memory_enabled" value="false"/>
```

从原始 NavOA 迁移时，也应确保以下接口不会被周期调用：

```text
/move_base/clear_costmaps
/move_base/cancel
/navoa/blocked_route_points -> move_base obstacle layer
```

`/move_base/cancel` 可暂时保留作为兼容接口，但没有 move_base 时不会参与正常组合导航。

### 2.7 关闭 NavOA 的“返回上一个目标”行为

SCAN 会不断更新局部路径和子目标。若 NavOA 在子目标到达后自动返回旧目标，会导致来回
运动。组合系统必须设置：

```xml
<param name="return_to_previous_goal_enabled" value="false"/>
<param name="goal_timeout_button_enabled" value="false"/>
<param name="require_explicit_goal" value="true"/>
```

### 2.8 保证只有一个最终速度发布者

推荐速度链路：

```text
NavOA:       /NavOA_cmd_vel -> remap /cmd_vel_tem
vel_filter:  /cmd_vel_tem -> /cmd_vel
```

原 NavOA launch 应包含：

```xml
<remap from="/NavOA_cmd_vel" to="/cmd_vel_tem"/>
```

`vel_filter` 应设置：

```xml
<param name="cmd_in_topic" value="/cmd_vel_tem"/>
<param name="cmd_out_topic" value="/cmd_vel"/>
```

必须关闭或重映射以下潜在速度源：

- SCAN `closed_loop_controller`；
- SCAN `open_loop_controller`；
- move_base 控制器；
- 键盘遥控节点；
- 其他直接发布 `/cmd_vel` 的测试节点。

检查命令：

```bash
rostopic info /cmd_vel
```

正常情况下 Publisher 只有 `vel_filter`。

### 2.9 NavOA 的二维激光话题改为实机话题

NavOA 和 `vel_filter` 都需要 `sensor_msgs/LaserScan`。原始 NavOA 通常使用 `/scan`，
应在 launch 中将实际话题同时传给两者：

```xml
<arg name="scan_topic" default="/scan"/>

<remap from="/scan" to="$(arg scan_topic)"/>
<param name="scan_topic" value="$(arg scan_topic)"/>
```

如果激光雷达只输出 `PointCloud2`，需要先使用驱动自带转换节点或
`pointcloud_to_laserscan` 生成 LaserScan，不能直接把点云话题填给 NavOA。

## 3. 实际机器人必须确认或更换的话题

下面是当前组合 launch 的默认值。必须根据实机 `rostopic list` 和消息类型逐项确认。

| 功能 | 当前默认话题 | 类型 | 实机要求 |
|---|---|---|---|
| 车体位姿 | `/LIO/odom_vehicle` | `nav_msgs/Odometry` | 改为实机定位输出 |
| 传感器位姿 | `/LIO/odom_imu` | `nav_msgs/Odometry` | 改为雷达/相机对应位姿 |
| SCAN 点云 | `/LIO/clouds_lidar` | `sensor_msgs/PointCloud2` | 改为三维雷达点云 |
| 深度图 | `/camera/aligned_depth_to_color/image_raw` | `sensor_msgs/Image` | 仅 `sensor_type:=depth` 使用 |
| NavOA 激光 | `/scan` | `sensor_msgs/LaserScan` | 改为实机二维扫描 |
| 用户终点 | `/move_base_simple/goal` | `geometry_msgs/PoseStamped` | RViz 2D Nav Goal |
| SCAN B 样条 | `/planning/bspline` | `scan_planner/Bspline` | 内部话题，通常不改 |
| 标准路径 | `/planning/path` | `nav_msgs/Path` | 内部话题，通常不改 |
| NavOA 子目标 | `/split_goal` | `geometry_msgs/PoseStamped` | 内部话题，通常不改 |
| NavOA 原始速度 | `/cmd_vel_tem` | `geometry_msgs/Twist` | 内部话题 |
| 底盘速度 | `/cmd_vel` | `geometry_msgs/Twist` | 改为底盘真实控制话题 |

查看消息类型：

```bash
rostopic type /LIO/odom_vehicle
rostopic type /LIO/odom_imu
rostopic type /LIO/clouds_lidar
rostopic type /scan
```

若底盘接收的是 `/limo_base_controller/cmd_vel` 等其他话题，启动时传入：

```bash
cmd_vel_topic:=/limo_base_controller/cmd_vel
```

## 4. 坐标系和 TF 要求

这是实机适配中最重要的一项。

`world_frame`、车体里程计数值所在坐标系、SCAN 路径坐标系和 NavOA `map_frame`
必须表示同一个坐标系。并且 TF 树中必须存在：

```text
world_frame -> base_frame
world_frame -> lidar_frame
```

检查里程计父坐标系：

```bash
rostopic echo -n1 /LIO/odom_vehicle/header
```

检查 TF：

```bash
rosrun tf tf_echo map base_link
rosrun tf tf_echo map laser_frame
```

如果里程计的 `header.frame_id` 是 `odom`、`world` 或 `camera_init`，应把该名称传给：

```bash
world_frame:=实际父坐标系
```

不能只修改消息上的字符串来伪装坐标系；若 SCAN 和 NavOA 使用不同坐标系，应先提供
正确的静态或动态 TF，并对路径/目标进行真实坐标变换。

## 5. 点云坐标参数

`cloud_is_world` 的含义：

- `true`：点云中的点已经位于 `world_frame`；
- `false`：点云位于雷达自身坐标系，SCAN 使用传感器位姿转换到世界坐标系。

大多数原始雷达驱动输出雷达坐标系点云，应使用：

```bash
cloud_is_world:=false
```

如果上游 LIO 已明确发布去畸变且转换到世界坐标系的点云，才设置为 `true`。设置错误
会导致障碍物随机器人运动或出现在错误位置。

## 6. 编译和启动

编译 SCAN 新增节点：

```bash
cd /home/ubuntu/Robit_SlamNav/NavOA_ws/src/SCAN-Planner
source /opt/ros/noetic/setup.bash
catkin_make -j2
```

启动前，先停止原来的以下 launch：

```text
roslaunch scan_planner run.launch
roslaunch NavOA_ros navoa_nav_only.launch
任何单独运行的 move_base
```

启动组合系统：

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

若当前平台没有 RKNN 运行环境：

```bash
roslaunch NavOA_ros scan_navoa.launch use_rknn:=false
```

## 7. 实机首次运行前的安全检查

首次测试应架空轮子或断开底盘使能，只检查话题链路：

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

依次判断：

1. 没有 `/planning/bspline`：检查 SCAN 位姿、点云、目标和坐标系。
2. 有 B 样条但没有 `/planning/path`：检查 `bspline_to_path` 是否启动。
3. 有 Path 但没有 `/split_goal`：检查 `path_segment_publisher` 的 `path_topic`。
4. 有子目标但没有速度：检查 NavOA TF、`/scan`、模型文件和 Python 环境。
5. 有速度但底盘不动：检查 `cmd_vel_topic` 和底盘使能状态。
6. 障碍方向错误：重点检查 `laser_frame`、LaserScan 零角方向和 TF。

首次落地行驶建议将：

```text
ref_speed:=0.15
segment_length:=0.6
```

确认方向、坐标系和急停均正常后，再逐步提高速度。
