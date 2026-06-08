include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_footprint",
  published_frame = "base_footprint",       -- 不能与 odom_frame 同名！
  odom_frame = "odom",
  provide_odom_frame = false,            -- rf2o 负责发布 odom->base_footprint
  publish_frame_projected_to_2d = false,
  use_pose_extrapolator = true,
  use_odometry = true,                   -- 订阅 /odom（EKF 融合后）
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true

-- =====================================================
-- 子图参数：更多帧 → 更多平均 → 地图更平滑
-- =====================================================
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 60        -- 35→60，更多扫描帧累积平滑抖动
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.05

-- =====================================================
-- 距离范围：缩窄以减少远处噪声
-- =====================================================
TRAJECTORY_BUILDER_2D.min_range = 0.2
TRAJECTORY_BUILDER_2D.max_range = 30.                    -- 100→30，裁掉远处不可靠数据
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 3.       -- 5→3，减少自由空间清除（更保守）

-- =====================================================
-- 传感器
-- =====================================================
TRAJECTORY_BUILDER_2D.use_imu_data = false

-- =====================================================
-- 体素滤波：适当降采样平滑点云抖动
-- =====================================================
TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.05

-- =====================================================
-- 实时相关扫描匹配（前端）
-- =====================================================
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.15
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(10.)
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 20.
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1.0

-- =====================================================
-- Ceres 扫描匹配（更保守的权重 → 不易被抖动带偏）
-- =====================================================
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 1.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 20.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 60.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 20

-- =====================================================
-- 运动滤波器：抖动时跳过冗余帧，防止无效子图生成
-- =====================================================
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 0.5
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.05
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.5)

-- =====================================================
-- 概率插入参数：降低 hit/miss 概率 → 单帧影响变小 → 抗抖动
-- =====================================================
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.hit_probability = 0.50
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.miss_probability = 0.48

-- =====================================================
-- 后端：⚠️ 必须启用全局优化，之前=0意味着完全不优化
-- =====================================================
POSE_GRAPH.optimize_every_n_nodes = 50                   -- 0→90，每90个节点做一次全局优化
POSE_GRAPH.optimization_problem.huber_scale = 1e2         -- 降低Huber尺度，对离群约束更鲁棒
POSE_GRAPH.constraint_builder.min_score = 0.70            -- 提高回环匹配门槛，减少误匹配
POSE_GRAPH.constraint_builder.sampling_ratio = 0.3
POSE_GRAPH.constraint_builder.max_constraint_distance = 15.0

POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 7.0
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(30.)
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.branch_and_bound_depth = 7

return options