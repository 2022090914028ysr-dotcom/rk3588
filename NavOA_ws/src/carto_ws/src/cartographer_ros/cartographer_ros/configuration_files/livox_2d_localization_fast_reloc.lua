include "livox_2d.lua"

options.publish_tracked_pose = true
options.use_odometry = true          -- rf2o is not running in localization mode (if true rf2o is running)
options.provide_odom_frame = false     -- Cartographer provides map→odom→base_footprint TF without external odometry
options.published_frame = "odom" --for rf2o make it odom_frame = "odom"                 --for rf2o make it odom

-- =====================================================
-- [新增] 请改用这行！这是限制算法内部线程的正确位置
MAP_BUILDER.num_background_threads = 4

-- ==================== 以下保持原样 ====================

-- 前端搜索范围
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.3
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(20.)

-- 权重配置
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 1.5
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 4.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 8.0

-- Motion Filter
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.02
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.5)
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 0.1

-- 体素滤波 (保留 0.05 高精度)
TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.05

-- 多帧累积
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 2

-- 后端设置
TRAJECTORY_BUILDER.pure_localization_trimmer = {
  max_submaps_to_keep = 2,
}

-- 优化频率
POSE_GRAPH.optimize_every_n_nodes = 20

-- 采样率
POSE_GRAPH.global_sampling_ratio = 0.01

-- POSE_GRAPH.constraint_builder.sampling_ratio = 0.05

-- 约束构建
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 5.0
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(30.)
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.branch_and_bound_depth = 7

POSE_GRAPH.constraint_builder.global_localization_min_score = 0.45
POSE_GRAPH.constraint_builder.min_score = 0.55

POSE_GRAPH.constraint_builder.sampling_ratio = 0.3
POSE_GRAPH.constraint_builder.max_constraint_distance = 15.0

-- 子图参数
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 20
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.05

return options