# NavOA_ws 项目文档 v1

> 自主导航与探索系统，集成深度相机虚拟保险杠、超声波雷达、激光SLAM与自主探索。

---

## 一、系统架构概览

```
┌─────────────────────────────────────────────────────────────────┐
│                        传感器层                                  │
│  lidar_node          → /scan         (激光雷达, 360°)            │
│  ascamera_hp60c      → /depth/image  (HP60C 深度相机)            │
│  depth_to_laserscan  → /camscan      (深度→激光, ~74° FOV)       │
│  camscan_accumulator → /camscan_wide (累积拓宽至360°)            │
│  multi_ultrasonic_scan → /voscan, /ultrasonic/* (3路超声波)     │
├─────────────────────────────────────────────────────────────────┤
│                        SLAM / 定位层                             │
│  cartographer_node   → 激光SLAM建图/定位                         │
│  rf2o_laser_odometry → /odom_rf2o   (激光里程计)                 │
│  map_server          → /map          (静态地图服务)              │
├─────────────────────────────────────────────────────────────────┤
│                        规划控制层                                │
│  global_planner      → 全局路径规划                              │
│  exploration_node    → 自主探索 (发布目标点)                     │
│  NavOA_node          → 局部规划 + 速度指令                        │
├─────────────────────────────────────────────────────────────────┤
│                        安全过滤层 (最高优先级)                    │
│  vel_filter          → /cmd_vel_tem → /cmd_vel                   │
│  ├─ 超声波虚拟保险杠 (front/left/right)                          │
│  └─ camscan虚拟保险杠 (前方深度相机检测)                         │
└─────────────────────────────────────────────────────────────────┘
```

---

## 二、文件说明

### 2.1 安全过滤层（最核心）

| 文件 | 路径 | 作用 |
|------|------|------|
| `vel_filter.py` | `NavOA_ros/scripts/vel_filter.py` | 速度过滤器：超声波 + camscan 虚拟保险杠，合并检测前方障碍物，执行自动倒车/侧移/减速/停止 |
| `NavOA_gazebo_limo.launch` | `NavOA_ros/example/gazebo_limo/launch/` | 主导航 launch 文件，配置 vel_filter、NavOA_control、path_segment_publisher |

#### vel_filter.py 全部参数

**基础配置：**

| 参数 | 实际值 | 说明 |
|------|--------|------|
| `scan_topic` | `/scan` | 激光雷达话题（用于自由空间转向） |
| `cmd_in_topic` | `/cmd_vel_tem` | 输入速度指令话题（来自 NavOA） |
| `cmd_out_topic` | `/cmd_vel` | 输出速度指令话题（发给底盘） |
| `cmd_timeout` | `0.5` | 指令超时（秒），超时后发零速 |
| `publish_rate` | `100.0` | 输出发布频率（Hz，launch 覆盖代码默认 20→100） |
| `debug_topic` | `/vel_filter/debug` | 调试信息话题 |

**死区与限幅：**

| 参数 | 实际值 | 说明 |
|------|--------|------|
| `deadband_linear_x` | `0.0` | 线速度死区（0=不禁用） |
| `deadband_angular_z` | `0.0` | 角速度死区（0=不禁用） |
| `max_forward_speed` | `0.3` | 最大前向速度（m/s，launch 覆盖 0→0.3） |
| `max_angular_speed` | `0.5` | 最大角速度（rad/s，launch 覆盖 0→0.5） |
| `wz_scale` | `1.0` | 全局角速度缩放系数 |

**倒车相关：**

| 参数 | 实际值 | 说明 |
|------|--------|------|
| `allow_reverse` | `true` | 是否允许倒车 |
| `reverse_requires_obstacle` | `true` | 倒车是否需要检测到障碍物 |
| `reverse_obstacle_distance` | `0.35` | 障碍物判定距离（m） |
| `max_reverse_speed` | `0.25` | 最大倒车速度（m/s） |
| `enable_auto_reverse` | `true` | 前方被挡时自动倒车（launch 覆盖 false→true） |
| `auto_reverse_speed` | `-0.12` | 自动倒车速度（负值=后退，m/s，launch 覆盖 -0.1→-0.12） |

**激光雷达前方安全：**

| 参数 | 实际值 | 说明 |
|------|--------|------|
| `enable_forward_safety` | `true` | 启用前方安全检测 |
| `front_center_angle` | `0.0` | 前方中心角度（rad，0=正前方） |
| `front_half_angle` | `0.52` | 前方检测半角（rad，≈30°，共60°锥形） |
| `forward_stop_distance` | `0.30` | 前方急停距离（m） |
| `forward_slow_distance` | `0.60` | 前方减速距离（m） |
| `forward_stop_release_distance` | `0.60` | 停止状态释放距离（m，=forward_slow_distance） |
| `min_forward_speed` | `0.05` | 最小前向速度（m/s） |
| `min_scan_range` | `0.0` | 忽略此距离以下的扫描值（m） |
| `stop_turn_when_forward_blocked` | `true` | 前方被挡时停止旋转 |

**自由空间转向（可选）：**

| 参数 | 实际值 | 说明 |
|------|--------|------|
| `enable_free_space_steer` | `false` | ⚠️ 未启用（代码默认关闭） |
| `steer_scan_half_angle` | `1.57` | 转向扫描半角（rad，≈90°） |
| `steer_deadzone` | `0.10` | 转向死区（m，左右差距小于此值不转向） |
| `steer_gain` | `1.2` | 转向增益 |
| `steer_max_angular` | `0.6` | 转向最大角速度（rad/s） |
| `steer_activation_distance` | `0.60` | 转向激活距离（m，=forward_slow_distance） |
| `steer_release_distance` | `0.80` | 转向释放距离（m，=activation+0.20） |
| `steer_forward_speed` | `0.05` | 转向时前向速度（m/s） |
| `enable_turn_search_when_blocked` | `true` | 前方被挡时搜索转向方向 |
| `turn_search_angular` | `0.35` | 搜索转向角速度（rad/s） |

**超声波虚拟保险杠（最高优先级）：**

| 参数 | 实际值 | 说明 |
|------|--------|------|
| `enable_ultrasonic_bumper` | `true` | 启用超声波虚拟保险杠 |
| `ultrasonic_stop_distance` | `0.23` | 超声波前方急停距离（m，launch 覆盖 0.25→0.23） |
| `ultrasonic_slow_distance` | `0.23` | 超声波前方减速距离（m，launch 覆盖 0.45→0.23，与 stop 相同=无渐变直接停） |
| `ultrasonic_side_stop_distance` | `0.17` | 超声波侧面停止距离（m，launch 覆盖 0.20→0.17） |
| `ultrasonic_blend_distance` | `0.14` | 超声波混合距离（m，launch 覆盖 0.13→0.14）：低于此值时纯规避，高于此值时混入算法指令 |
| `ultrasonic_evade_forward_speed` | `0.05` | 超声波侧向规避时的前向速度（m/s） |
| `ultrasonic_evade_lateral_speed` | `1.5` | 超声波侧向规避速度（m/s，launch 覆盖 0.10→1.5） |
| `ultrasonic_linear_evade` | `true` | 线性规避：越近侧移越快 |
| `ultrasonic_timeout` | `1.0` | 超声波数据超时（秒），超时视为无数据 |

**倒车侧移保持：**

| 参数 | 实际值 | 说明 |
|------|--------|------|
| `reverse_side_stop_distance` | `0.25` | 倒车时侧面扩展检测距离（m，launch 覆盖 0.35→0.25） |
| `reverse_side_hold_duration` | `2.0` | 倒车结束后侧移速度保持时长（秒，launch 覆盖 0.5→2.0） |

**camscan 偏置（玻璃墙等激光雷达盲区检测）：**

| 参数 | 实际值 | 说明 |
|------|--------|------|
| `enable_camscan_bias` | `false` | ⚠️ 已关闭，使用 camscan_bumper 替代（launch 覆盖 true→false） |
| `camscan_topic` | `/camscan` | camscan 话题名 |
| `camscan_front_threshold` | `0.50` | camscan 前方触发阈值（m，launch 覆盖 0.4→0.50） |
| `camscan_bias_gain` | `0.50` | camscan 偏置增益 |
| `raw_scan_topic` | `/scan` | 原始激光雷达话题（用于与 camscan 对比） |
| `camscan_timeout` | `1.0` | camscan 数据超时（秒） |

**camscan 虚拟保险杠（与超声波并列前方检测）：**

| 参数 | 实际值 | 说明 |
|------|--------|------|
| `enable_camscan_bumper` | `true` | 启用 camscan 虚拟保险杠（launch 覆盖 false→true） |
| `camscan_stop_distance` | `0.40` | camscan 前方急停距离（m，launch 覆盖 0.30→0.40） |
| `camscan_slow_distance` | `0.40` | camscan 前方减速距离（m，launch 覆盖 0.50→0.40，与 stop 相同=无渐变直接停） |

> **合并逻辑**：`d_front = min(超声波前方距离, camscan前方距离)`，停止阈值取两者中最保守的 `min`，减速阈值取 `max`。

---

### 2.2 深度相机 → 激光扫描

| 文件 | 路径 | 作用 |
|------|------|------|
| `hp60c_depth2laser.launch` | `ascam_ws/src/ascamera/launch/` | HP60C 相机 + depth→laser + 累积器一体化 launch |
| `depth_to_laserscan.py` | `ascam_ws/src/ascamera/scripts/` | 深度图像 → `/camscan` LaserScan |
| `camscan_accumulator.py` | `ascam_ws/src/ascamera/scripts/` | 窄 FOV camscan 通过 odom 累积拓宽至 360° → `/camscan_wide` |

#### depth_to_laserscan.py 参数（来自 `lidar_full.launch` 实际配置）

| 参数 | 实际值 | 说明 |
|------|--------|------|
| `depth_topic` | `/ascamera_hp60c/depth0/image_raw` | 输入深度图像话题 |
| `scan_rows` | `0` | 扫描行数（0=使用 row_start/row_end） |
| `scan_row_start` | `80` | 扫描起始行（垂直方向，像素，launch 覆盖 400→80） |
| `scan_row_end` | `320` | 扫描结束行（垂直方向，像素，launch 覆盖 480→320） |
| `range_min` | `0.0` | 最小有效距离（m，launch 覆盖 0.1→0.0） |
| `range_max` | `5.0` | 最大有效距离（m，launch 覆盖 10.0→5.0） |
| `scan_time` | `1/30` | 单次扫描时间（秒） |
| `output_frame` | `base_footprint` | 输出坐标系（launch 覆盖 laser_frame→base_footprint） |
| `fy` | `420.85` | 相机 y 方向焦距（像素，launch 覆盖 210.43→420.85） |
| `cy` | `240.69` | 相机 y 方向主点（像素，launch 覆盖 120.35→240.69） |
| `camera_height` | `0.3` | 相机离地高度（m） |
| `ground_tolerance` | `0.25` | 地面过滤容差（m，launch 覆盖 0.05→0.25） |
| `range_offset` | `0.13` | 距离偏移（m，正值=推远障碍物，launch 覆盖 0.0→0.13） |
| `angle_offset` | `0.1` | 水平角度偏移（rad，launch 覆盖 0.0→0.1） |
| `pub_rate` | `10.0` | 发布频率（Hz，launch 覆盖 50.0→10.0） |
| `fov_horizontal` | `87°` | 水平视场角（仅首次初始化时通过 rosparam 读取） |

#### camscan_accumulator.py 参数（来自 `hp60c_depth2laser.launch` 实际配置）

| 参数 | 实际值 | 说明 |
|------|--------|------|
| `topic_in` | `/camscan` | 输入窄 FOV scan |
| `topic_out` | `/camscan_wide` | 输出宽 FOV scan |
| `target_frame` | `base_footprint` | 目标坐标系 |
| `map_frame` | `map` | 地图坐标系 |
| `cache_sec` | `2.0` | 缓存时长（秒，launch 覆盖 5.0→2.0，更短=更实时） |
| `pub_rate` | `10.0` | 发布频率（Hz） |
| `output_resolution` | `0.01745` | 输出角分辨率（rad，≈1°） |
| `max_range` | `10.0` | 最大距离（m） |

---

### 2.3 超声波传感器

| 文件 | 路径 | 作用 |
|------|------|------|
| `multi_ultrasonic_scan.py` | `NavOA_ros/scripts/` | 管理3路 DYP-A21 超声波，发布 `/voscan` 和 `/ultrasonic/front|left|right` |
| `a21_ultrasonic_mm.py` | `NavOA_ros/scripts/` | 单路 DYP-A21 超声波驱动 |

#### multi_ultrasonic_scan.py 参数

| 参数 | 实际值 | 说明 |
|------|--------|------|
| `ports` | `[/dev/ttyS3, /dev/ttyS7, /dev/ttyS6]` | 三路串口（front/left/right） |
| `angles_deg` | `[0, -75, 75]` | 三路传感器安装角度（度） |
| `beam_width_deg` | `20.0` | 单路波束宽度（度） |
| `scan_fov_deg` | `180.0` | 合成 /voscan 的 FOV（度） |
| `angular_res_deg` | `1.0` | 合成 /voscan 角分辨率（度） |
| `range_min_m` | `0.02` | 最小距离（m） |
| `range_max_m` | `3.0` | 最大距离（m） |
| `baudrate` | `115200` | 串口波特率 |

---

### 2.4 NavOA 核心规划

| 文件 | 路径 | 作用 |
|------|------|------|
| `NavOA_node.py` | `NavOA_ros/scripts/` | NavOA 入口节点，调用 NavOA_core |
| `NavOA_core.py` | `NavOA_ros/src/` | NavOA 核心 DUNE 规划器 |
| `NavOA_gazebo_limo.launch` | 见 2.1 | 主 launch |

#### NavOA_node 核心参数（launch 中配置）

| 参数 | 实际值 | 说明 |
|------|--------|------|
| `config_file` | `NavOA_planner_limo.yaml` | 规划器 YAML 配置文件路径 |
| `map_frame` | `map` | 地图坐标系 |
| `base_frame` | `base_footprint` | 机器人基坐标系 |
| `lidar_frame` | `laser_frame` | 激光雷达坐标系 |
| `ref_speed` | `0.5` | 参考速度（m/s） |
| `scan_range` | `0.1 5.0` | 激光雷达有效距离范围（m） |
| `scan_angle_range` | `-3.14 3.14` | 激光雷达角度范围（rad） |
| `marker_size` | `0.05` | RViz 可视化标记大小 |
| `marker_z` | `0.3` | RViz 可视化标记 Z 高度（m） |
| `scan_downsample` | `2` | 激光雷达下采样因子 |
| `dune_checkpoint` | `model_5000.pth` | DUNE 模型 PyTorch 权重路径 |
| `use_rknn` | `true` | 是否使用 RKNN 推理加速 |
| `dune_rknn` | `model_5000_quant.rknn` | DUNE 模型 RKNN 权重路径 |
| `rknn_target` | `` | RKNN target 平台（空=自动） |
| `rknn_core` | `auto` | RKNN NPU 核心选择 |
| `refresh_initial_path` | `true` | 是否刷新初始路径 |
| `control_rate` | `50.0` | 控制频率（Hz） |
| `require_explicit_goal` | `true` | 是否需要显式目标 |
| `return_to_previous_goal_enabled` | `true` | 超时后是否返回上一个目标 |

**按钮超时配置：**

| 参数 | 实际值 | 说明 |
|------|--------|------|
| `goal_timeout_button_enabled` | `false` | 启用按钮超时 |
| `goal_timeout_button_topic` | `/app_controller/button` | 按钮话题 |
| `goal_timeout_button_id` | `9` | 按钮 ID |
| `goal_timeout_button_event_type` | `1` | 按钮按下事件类型 |
| `goal_timeout_button_release_event_type` | `0` | 按钮释放事件类型 |
| `goal_timeout_button_release_delay` | `0.10` | 按钮释放延迟（秒） |
| `goal_timeout_button_send_release` | `true` | 是否发送释放事件 |

**vel_filter 相关（在 launch 中配置）：**

| 参数 | 实际值 | 说明 |
|------|--------|------|
| `vel_filter_cmd_out_topic` | `/cmd_vel` | vel_filter 输出话题 |
| `ultrasonic_evade_lateral_speed` | `1.5` | 超声波规避侧移速度 |
| `reverse_side_stop_distance` | `0.25` | 倒车侧面检测距离 |
| `reverse_side_hold_duration` | `2` | 倒车侧移保持时长 |

---

### 2.5 自主探索

| 文件 | 路径 | 作用 |
|------|------|------|
| `exploration.launch` | `marl_mapping/launch/` | 自主探索 launch |
| `exploration_node.py` | `marl_mapping/scripts/` | 探索主节点：检测前沿 → 发送目标点 |
| `robot_interface.py` | `marl_mapping/scripts/` | 机器人接口抽象 |
| `add_region_point.py` | `marl_mapping/scripts/` | 添加探索区域点 |
| `exploration.yaml` | `marl_mapping/config/` | 探索配置 |

#### exploration_node.py 参数（来自 `exploration.launch` 实际配置）

**基础配置：**

| 参数 | 实际值 | 说明 |
|------|--------|------|
| `map_frame` | `map` | 地图坐标系 |
| `base_frame` | `base_footprint` | 机器人基坐标系 |
| `map_topic` | `/map` | 地图话题 |
| `odom_topic` | `/odom_rf2o` | 里程计话题 |
| `goal_topic` | `/move_base_simple/goal` | 目标点发布话题 |
| `max_steps` | `500` | 最大探索步数 |
| `robot_name` | `car1` | 机器人名称 |

**目标与超时：**

| 参数 | 实际值 | 说明 |
|------|--------|------|
| `goal_tolerance` | `1.0` | 目标到达容差（m，launch 覆盖 0.5→1.0） |
| `goal_timeout` | `60.0` | 目标超时（秒） |
| `idle_wait` | `1.0` | 空闲等待时间（秒） |
| `post_reach_wait` | `2.5` | 到达目标后等待时间（秒） |

**前沿检测：**

| 参数 | 实际值 | 说明 |
|------|--------|------|
| `min_frontier_size` | `5` | 最小前沿大小（栅格数） |
| `exploration_method` | `whitest` | 探索方法：`whitest`（代码默认 `nearest`，launch 覆盖为 `whitest`） |
| `inflation_cells` | `6` | 膨胀栅格数（未连接 costmap 时使用） |
| `costmap_topic` | `/move_base/global_costmap/costmap` | 全局代价地图话题（用于可达性判断） |

**区域探索：**

| 参数 | 实际值 | 说明 |
|------|--------|------|
| `regional_explore` | `true` | 启用区域探索 |
| `local_radius` | `60` | 局部探索半径（栅格数，60×0.05=3m） |
| `frontier_update_interval` | `5.0` | 前沿重计算间隔（秒） |
| `goal_check_radius` | `30` | 目标检查半径（栅格数，30×0.05=1.5m） |

**探索区域过滤（Rviz Publish Point 限定区域）：**

| 参数 | 实际值 | 说明 |
|------|--------|------|
| `enable_region_filter` | `true` | 启用区域过滤（launch 覆盖 false→true） |
| `clicked_point_topic` | `/clicked_point` | Rviz 点击点话题 |
| `region_default_radius` | `3.0` | 单点默认半径（m） |
| `region_shape_mode` | `polygon` | 区域形状模式（launch 覆盖 convex_hull→polygon） |
| `region_close_threshold` | `1.5` | 多边形闭合阈值（m，launch 覆盖 2.0→1.5） |
| `map_padding_cells` | `400` | 地图边界扩展（栅格数，launch 覆盖 0→400，400×0.05=20m） |

**可视化与数据保存：**

| 参数 | 实际值 | 说明 |
|------|--------|------|
| `visualization` | `true` | 启用 RViz 可视化 |
| `save_data` | `true` | 保存探索数据 |
| `data_dir` | `~/exploration_data` | 数据保存目录 |

**路径规划监控：**

| 参数 | 实际值 | 说明 |
|------|--------|------|
| `plan_watch_enabled` | `true` | 启用路径监控（检测空 path 及时切换目标） |
| `plan_topic` | `/move_base/GlobalPlanner/plan` | 全局规划路径话题 |

---

### 2.6 激光雷达

| 文件 | 路径 | 作用 |
|------|------|------|
| `lidar_full.launch` | `lidar_ros/launch/` | 完整 launch：激光雷达 + 深度相机 + 超声波 + 里程计 |
| `lidar.launch` | `lidar_ros/launch/` | 仅激光雷达 |
| `lidar_view.launch` | `lidar_ros/launch/` | 激光雷达可视化 |

#### lidar_node 参数（来自 `lidar_full.launch` 实际配置）

| 参数 | 实际值 | 说明 |
|------|--------|------|
| `serial_name` | `/dev/ttyS4` | 串口设备 |
| `serial_baud` | `230400` | 波特率 |
| `frame_id` | `laser_frame` | 坐标系名 |
| `counterclockwise` | `true` | 逆时针旋转 |
| `angle_min` | `-180` | 最小角度（度） |
| `angle_max` | `180` | 最大角度（度） |
| `range_min` | `0.1` | 最小距离（m） |
| `range_max` | `15` | 最大距离（m） |
| `resolution_fixed` | `false` | 固定分辨率 |
| `angle_corp_string` | `` | 角度校正字符串 |

---

### 2.7 Cartographer SLAM

| 文件 | 路径 | 作用 |
|------|------|------|
| `livox_2d.launch` | `carto_ws/.../launch/` | 建图模式：启动 lidar + 超声波 + odom + cartographer |
| `livox_2d_localization.launch` | `carto_ws/.../launch/` | 定位模式：加载已有 map 进行纯定位 |

#### livox_2d_localization.launch 关键参数

| 参数 | 说明 |
|------|------|
| `load_state_filename` | 预建地图 pbstream 路径 |
| `map_file` | map_server 使用的 YAML 地图文件 |
| `launch_rviz` | 是否启动 RViz |

---

### 2.8 其他辅助脚本

| 文件 | 路径 | 作用 |
|------|------|------|
| `path_segment_publisher.py` | `NavOA_ros/scripts/` | 路径分段发布 |
| `status_state.py` | `NavOA_ros/scripts/` | 状态监测与上报 |
| `data_logger.py` | `NavOA_ros/scripts/` | 数据日志记录 |
| `localization_recovery.py` | `NavOA_ros/scripts/` | 定位恢复 |
| `voice_goal_router.py` | `NavOA_ros/scripts/` | 语音目标路由 |
| `serial_cmd_vel_imu_bridge.py` | `NavOA_ros/scripts/` | 串口 cmd_vel / IMU 桥接 |
| `map_odom_freezer.py` | `NavOA_ros/scripts/` | map→odom 变换冻结 |
| `imu_mux.py` | `NavOA_ros/scripts/` | IMU 多路复用 |
| `error_collector.py` | `NavOA_ros/scripts/` | 错误收集 |
| `external_cmd.py` | `NavOA_ros/scripts/` | 外部指令接口 |
| `send_forward_goal.py` | `NavOA_ros/scripts/` | 发送前方目标点 |

---

### 2.9 其他 Launch 文件

| 文件 | 路径 | 作用 |
|------|------|------|
| `rf2o_laser_odometry.launch` | `rf2o_laser_odometry/launch/` | 激光里程计 |
| `ekf_fusion.launch` | `rf2o_laser_odometry/launch/` | EKF 融合（里程计 + IMU） |
| `global_planner.launch` | `NavOA_ros/example/gazebo_limo/launch/` | 全局路径规划器 |
| `goal_publisher_limo.launch` | `NavOA_ros/example/gazebo_limo/launch/` | 目标发布 |
| `goal_sampler_limo.launch` | `NavOA_ros/example/gazebo_limo/launch/` | 目标采样 |
| `ultrasonic_scan.launch` | `NavOA_ros/example/gazebo_limo/launch/` | 超声波独立 launch |
| `external_cmd.launch` | `NavOA_ros/example/gazebo_limo/launch/` | 外部指令 |
| `send_forward_goal.launch` | `NavOA_ros/example/gazebo_limo/launch/` | 发送前方目标 |
| `nav_exploration.launch` | `marl_mapping/launch/` | 导航 + 探索联合 launch |

---

### 2.10 配置文件

| 文件 | 路径 | 作用 |
|------|------|------|
| `NavOA_planner_limo.yaml` | `NavOA_ros/example/gazebo_limo/config/` | NavOA DUNE 规划器参数 |
| `global_costmap_params.yaml` | 同上 | 全局代价地图参数 |
| `local_costmap_params.yaml` | 同上 | 局部代价地图参数 |
| `costmap_common_params.yaml` | 同上 | 代价地图通用参数 |
| `global_planner_params.yaml` | 同上 | 全局规划器参数 |
| `rvo_config_complex.yaml` | 同上 | RVO 避障配置 |
| `goal_list.yaml` | 同上 | 目标列表 |
| `voice_places.yaml` | 同上 | 语音地点映射 |
| `exploration.yaml` | `marl_mapping/config/` | 探索参数 |
| `GlobalPlanner.cfg` | `global_planner/cfg/` | 全局规划器动态参数 |
| `ekf.yaml` | `src/` | EKF 融合参数 |
| `meeting_room.yaml` | `src/map/meeting_room/` | 会议室静态地图 |
| `ekf_params.yaml` | `rf2o_laser_odometry/` | 激光里程计 EKF 参数 |

---

## 三、数据流详解

### 3.1 传感器 → 速度指令流程

```
┌──────────────┐    ┌──────────────┐    ┌──────────┐    ┌──────────┐
│ lidar_node   │    │ ascamera     │    │ ultrasonic│    │ odom     │
│ /scan (360°) │    │ /camscan    │    │ /voscan   │    │ /odom_rf2o│
└──────┬───────┘    └──────┬───────┘    │ /ultrasonic│   └────┬─────┘
       │                   │            │ /front     │        │
       │                   │            │ /left      │        │
       │                   │            │ /right     │        │
       ▼                   ▼            └─────┬──────┘        │
┌──────────────┐    ┌──────────────┐          │               │
│ cartographer │    │ camscan_ac-  │          │               │
│ (定位/SLAM)  │    │ cumulator    │          │               │
└──────┬───────┘    └──────┬───────┘          │               │
       │                   │                  │               │
       ▼                   ▼                  │               ▼
┌──────────────┐    ┌──────────────┐          │        ┌──────────────┐
│ map_server   │    │ /camscan_wide│          │        │ rf2o odom    │
│ /map         │    └──────┬───────┘          │        └──────────────┘
└──────┬───────┘           │                  │
       │                   │                  │
       ▼                   ▼                  ▼
┌──────────────────────────────────────────────────┐
│              NavOA 规划器 (NavOA_node)            │
│  输入: /map, /scan, /odom_rf2o, /split_goal      │
│  输出: /cmd_vel_tem                               │
└──────────────────────┬───────────────────────────┘
                       │
                       ▼
┌──────────────────────────────────────────────────┐
│            vel_filter (安全过滤)                  │
│  输入: /cmd_vel_tem, /ultrasonic/*, /camscan     │
│  ┌─────────────────────────────────────┐         │
│  │ 1. 超声波虚拟保险杠 (最高优先级)     │         │
│  │    front<stop  → 倒车/停止           │         │
│  │    left<side   → 右侧移              │         │
│  │    right<side  → 左侧移              │         │
│  ├─────────────────────────────────────┤         │
│  │ 2. camscan 虚拟保险杠               │         │
│  │    与超声波合并前方检测              │         │
│  │    front<stop → 同上                 │         │
│  ├─────────────────────────────────────┤         │
│  │ 3. camscan 偏置 (玻璃墙等盲区)      │         │
│  ├─────────────────────────────────────┤         │
│  │ 4. 自由空间转向(可选)               │         │
│  └─────────────────────────────────────┘         │
│  输出: /cmd_vel                                  │
└──────────────────────┬───────────────────────────┘
                       │
                       ▼
                底盘执行
```

### 3.2 自主探索流程

```
┌──────────────┐
│  exploration  │
│  _node       │
│              │
│ 1. 接收 /map │
│ 2. 检测前沿  │
│ 3. 选择目标  │────────► /move_base_simple/goal
│ 4. 监控到达  │              │
└──────────────┘              │
                              ▼
                     NavOA 规划器
                         │
                         ▼
                     vel_filter
                         │
                         ▼
                     底盘执行
```

---

## 四、调试话题汇总

| 话题 | 类型 | 说明 |
|------|------|------|
| `/vel_filter/debug` | `String` | vel_filter 实时调试信息（100Hz） |
| `/vel_filter/status` | `String` | vel_filter 状态变化（仅更新时发布） |
| `/camscan/debug_image` | `Image` | depth_to_laserscan 调试图像 |
| `/carto_submap_viz` | `OccupancyGrid` | Cartographer 子图可视化 |

### 常用调试命令

```bash
# 查看 vel_filter 状态
rostopic echo /vel_filter/status

# 查看 vel_filter 调试信息
rostopic echo /vel_filter/debug

# 查看 camscan 数据
rostopic echo /camscan -n 1

# 查看超声波原始数据
rostopic echo /ultrasonic/front
rostopic echo /ultrasonic/left
rostopic echo /ultrasonic/right

# 查看当前 cmd_vel
rostopic echo /cmd_vel

# 查看 NavOA 输出
rostopic echo /cmd_vel_tem
```

---

## 五、当前 Launch 参数配置示例

详见 `NavOA_ros/example/gazebo_limo/launch/NavOA_gazebo_limo.launch` 中的 `vel_filter` 节点配置：

```xml
<!-- 超声波保险杠 -->
<param name="enable_ultrasonic_bumper" value="true" />
<param name="ultrasonic_stop_distance" value="0.23" />
<param name="ultrasonic_slow_distance" value="0.23" />
<param name="ultrasonic_side_stop_distance" value="0.17" />
<param name="ultrasonic_blend_distance" value="0.14" />
<param name="ultrasonic_evade_lateral_speed" value="1.5" />
<param name="ultrasonic_linear_evade" value="true" />

<!-- 倒车侧移保持 -->
<param name="reverse_side_stop_distance" value="0.25" />
<param name="reverse_side_hold_duration" value="2" />

<!-- 自动倒车 -->
<param name="enable_auto_reverse" value="true" />
<param name="auto_reverse_speed" value="-0.12" />

<!-- camscan 虚拟保险杠 -->
<param name="enable_camscan_bumper" value="true" />
<param name="camscan_stop_distance" value="0.40" />
<param name="camscan_slow_distance" value="0.40" />

<!-- camscan 偏置（关闭，使用保险杠替代） -->
<param name="enable_camscan_bias" value="false" />

<!-- 速度限幅 -->
<param name="max_forward_speed" value="0.3" />
<param name="max_angular_speed" value="0.5" />
```
