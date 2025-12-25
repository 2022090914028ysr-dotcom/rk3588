include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_footprint",     -- ✅ 与你的 TF 树一致
  published_frame = "odom",              -- Cartographer 发布 map->odom
  odom_frame = "odom",                   -- 接收 /odom topic
  provide_odom_frame = false,            -- ✅ false：由 vins_to_odom 提供 odom->base_footprint
  publish_frame_projected_to_2d = false,
  use_pose_extrapolator = true,

  -- 传感器配置（关键：使用 VINS 里程计）
  use_odometry = true,                   -- ✅ 启用里程计（来自 vins_to_odom）
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,

  -- 频率与超时（关键调整）
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 0.01,        -- 100Hz，匹配 vins_to_odom 的 publish 频率
  trajectory_publish_period_sec = 0.03,

  -- 采样率：全部设为 1.0 以保留所有数据
  rangefinder_sampling_ratio = 1.0,
  odometry_sampling_ratio = 1.0,         -- ✅ 不丢弃任何 odom 数据
  fixed_frame_pose_sampling_ratio = 1.0,
  imu_sampling_ratio = 1.0,
  landmarks_sampling_ratio = 1.0,
}

-- 启用 2D 模式
MAP_BUILDER.use_trajectory_builder_2d = true

-- 2D 建图参数
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 35
TRAJECTORY_BUILDER_2D.min_range = 0.1
TRAJECTORY_BUILDER_2D.max_range = 8.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 8.5

-- 启用 IMU（你已提供 /imu）
TRAJECTORY_BUILDER_2D.use_imu_data = false

-- 扫描匹配（可选，提升定位）
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 10.
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1e-1

-- 回环优化
POSE_GRAPH.optimization_problem.huber_scale = 1e2
POSE_GRAPH.optimize_every_n_nodes = 35

return options