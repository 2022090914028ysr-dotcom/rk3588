include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_footprint",    -- 【注意】如果有IMU且外参准，强烈建议改为 "imu_link"
  published_frame = "base_footprint",
  odom_frame = "odom",
  provide_odom_frame = true,       -- 【正确】由 Cartographer 发布 odom -> base_footprint
  publish_frame_projected_to_2d = false,
  use_odometry = false,            -- 【正确】不信赖轮式里程计
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 20e-3, -- 【修改1】改为 50Hz。100Hz(10e-3) 太高，容易抢占 CPU 资源。
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

-- === 2D 轨迹构建器参数调优 ===

-- 1. 开启实时相关匹配 (CSM)
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true 

-- 2. 【修改2】扩大搜索范围：防止机器人加速时定位丢失
-- 原来的 0.1 (10cm) 太极限了，如果没有轮速计辅助，稍微一抖动就会超出搜索范围。
-- 改为 0.15 (15cm) 甚至 0.2 更安全。
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.15
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(20.)
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 10.
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1e-1

-- 3. IMU 设置
TRAJECTORY_BUILDER_2D.use_imu_data = false 
TRAJECTORY_BUILDER_2D.min_range = 0.2      -- 【建议】0.3 改为 0.2，防止近距离盲区过大
TRAJECTORY_BUILDER_2D.max_range = 25.0     -- 【建议】改为 25.0，充分利用雷达性能

-- 4. 【新增】性能优化
-- 必须加上这个，否则点云太密，算 CSM 会非常卡
TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.1
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1

-- === 纯定位的关键配置 ===

TRAJECTORY_BUILDER.pure_localization_trimmer = {
  max_submaps_to_keep = 3,
}

-- 5. 【修改3】初始化搜索
-- 原来的 0.003 是建图用的。纯定位开机时，如果这里太小，机器人可能找不到自己在地图的哪里。
POSE_GRAPH.global_sampling_ratio = 0.01 

-- 6. 减少优化频率
POSE_GRAPH.optimize_every_n_nodes = 40 -- 既然是纯定位，可以稍微懒一点，20 改 40
POSE_GRAPH.constraint_builder.sampling_ratio = 0.3

-- 7. 【新增】降低匹配门槛
-- 纯定位时稍微宽容一点，能更快定住
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.55
POSE_GRAPH.constraint_builder.min_score = 0.65

return options