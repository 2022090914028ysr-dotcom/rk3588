include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  
  -- 关键点 1：开启IMU后，tracking_frame 必须是 IMU 的 TF frame！
  -- 去查你的机器人模型，改成真实的 IMU link 名字！
  tracking_frame = "imu_link",     
  published_frame = "base_link",              
  
  -- 致命关键点 2：夺回发布权
  odom_frame = "odom",                   
  provide_odom_frame = true,             -- 必须是 true！Carto自己推算并发布odom
  
  publish_frame_projected_to_2d = false,
  use_pose_extrapolator = true,          -- 必须开启位姿推测器融合IMU

  -- 致命关键点 3：关闭外部的残羹冷炙
  use_odometry = false,                  -- 关掉！不要任何外部里程计
  use_nav_sat = false,
  use_landmarks = false,
  
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,

  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 0.01,        -- 100Hz，因为你要用它做导航，频率必须高
  trajectory_publish_period_sec = 0.03,

  rangefinder_sampling_ratio = 1.0,
  odometry_sampling_ratio = 1.0,         
  fixed_frame_pose_sampling_ratio = 1.0,
  imu_sampling_ratio = 1.0,
  landmarks_sampling_ratio = 1.0,
}

MAP_BUILDER.use_trajectory_builder_2d = true
MAP_BUILDER.num_background_threads = 4   

-- 子图参数
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 35
TRAJECTORY_BUILDER_2D.min_range = 0.1
TRAJECTORY_BUILDER_2D.max_range = 8.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 8.5

-- 致命关键点 4：强制信任 IMU
TRAJECTORY_BUILDER_2D.use_imu_data = true  

-- 没有外部里程计，必须极度依赖内部的前端扫描匹配
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 10.
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1e-1

-- 后端回环
POSE_GRAPH.optimization_problem.huber_scale = 1e2
POSE_GRAPH.optimize_every_n_nodes = 35

return options
