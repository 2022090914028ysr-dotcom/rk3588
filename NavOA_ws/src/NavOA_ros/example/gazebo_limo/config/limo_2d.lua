include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_footprint",    -- 机器人的基座坐标系
  published_frame = "odom",             -- Cartographer 发布的坐标系
  odom_frame = "odom",                  -- 机器人原本的里程计坐标系
  provide_odom_frame = false,           -- 关键！设为 false，因为 Limo 底盘已经发布了 odom，我们不抢
  publish_frame_projected_to_2d = false,
  use_pose_extrapolator = true,
  
  -- 传感器配置
  use_odometry = true,                  -- 使用 Limo 的轮式里程计
  use_nav_sat = false,                  -- 不用 GPS
  use_landmarks = false,
  num_laser_scans = 1,                  -- 单线雷达数量
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  
  -- 频率与超时
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 20e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true

-- 2D 建图参数微调
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 35
TRAJECTORY_BUILDER_2D.min_range = 0.1
TRAJECTORY_BUILDER_2D.max_range = 8.0      -- 根据你的雷达最大距离调整 (Limo 通常 8-12米)
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 8.5
TRAJECTORY_BUILDER_2D.use_imu_data = true
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 10.
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1e-1

POSE_GRAPH.optimization_problem.huber_scale = 1e2
POSE_GRAPH.optimize_every_n_nodes = 35     -- 每积累35个节点优化一次地图（回环检测）

return options