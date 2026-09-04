include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_footprint",
  published_frame = "odom",      
  odom_frame = "odom",
  provide_odom_frame = false,            -- rf2o 负责发布 odom->base_link
  publish_frame_projected_to_2d = false,
  publish_tracked_pose = true,             -- publish /tracked_pose for all modes
  use_pose_extrapolator = true,
  use_odometry = true,                   -- 订阅 /odom（EKF 融合后）
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

TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1
-- 2D 建图参数
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 60
TRAJECTORY_BUILDER_2D.min_range = 0.3
TRAJECTORY_BUILDER_2D.max_range = 8.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 1.5

-- 启用 IMU（你已提供 /imu）
TRAJECTORY_BUILDER_2D.use_imu_data = false

-- 扫描匹配（可选，提升定位）
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = false
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 10.0
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1e-1

-- 回环优化
POSE_GRAPH.optimize_every_n_nodes = 35

POSE_GRAPH.constraint_builder.min_score = 0.65            -- 提高回环匹配门槛，减少误匹配
POSE_GRAPH.constraint_builder.sampling_ratio = 0.15
POSE_GRAPH.constraint_builder.max_constraint_distance = 6.0
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 4.0
-- whole-map folds during mapping: you have continuous odometry, you don't need this
POSE_GRAPH.global_sampling_ratio = 0.001
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.70

-- one bad constraint should not dominate the solve
POSE_GRAPH.optimization_problem.huber_scale = 5e0

POSE_GRAPH.optimization_problem.odometry_translation_weight = 1e4
POSE_GRAPH.optimization_problem.odometry_rotation_weight    = 1e4

-- Make the grid sticky: hits persist, misses erode slowly.
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.hit_probability  = 0.58
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.miss_probability = 0.49

TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 1.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight    = 10.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight       = 40.0

return options