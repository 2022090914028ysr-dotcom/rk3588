include "livox_2d.lua"

options.publish_tracked_pose = true
options.use_odometry = true          -- rf2o is not running in localization mode (if true rf2o is running)
options.provide_odom_frame = false     -- Cartographer provides map→odom→base_footprint TF without external odometry
options.published_frame = "odom" --for rf2o make it odom_frame = "odom"                 --for rf2o make it odom

-- =====================================================
-- 纯定位模式配置（保留更少的子图以减少计算）
-- =====================================================
TRAJECTORY_BUILDER.pure_localization_trimmer = {
  max_submaps_to_keep = 2,
}

-- =====================================================
-- 优化频率（v2：回到激进，加快约束应用）
-- =====================================================
POSE_GRAPH.optimize_every_n_nodes = 5  -- ⚠ v2 改：原 15 → 10，更频繁优化

-- =====================================================
-- 前端实时扫描匹配
-- =====================================================
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.3
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(15.)
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 5.0
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 0.05

-- =====================================================
-- Ceres扫描匹配
-- =====================================================
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 1.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 2.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 4.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 10

-- =====================================================
-- 运动滤波器
-- =====================================================
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.03
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(1.0)
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 0.3

-- =====================================================
-- 体素滤波（v1 改动保留，对齐 pbstream 分辨率）
-- =====================================================
TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.05   -- ⚠ v1 改：0.08 → 0.05，保留

-- =====================================================
-- 多帧累积
-- =====================================================
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1

-- =====================================================
-- 子图参数（v2：更激进，第一个 submap 早完成）
-- =====================================================
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 50                   -- ⚠ v2 改：原 30 → 25，加速 submap
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.05    -- ⚠ v1 改：0.10 → 0.05，保留

-- =====================================================
-- 后端全局定位
-- =====================================================
POSE_GRAPH.global_sampling_ratio = 0.02

POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 8.0
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(45.)
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.branch_and_bound_depth = 6

POSE_GRAPH.constraint_builder.global_localization_min_score =0.60
POSE_GRAPH.constraint_builder.min_score = 0.65

POSE_GRAPH.constraint_builder.sampling_ratio = 0.25
POSE_GRAPH.constraint_builder.max_constraint_distance = 20.0

-- =====================================================
-- 距离和角度范围
-- =====================================================
TRAJECTORY_BUILDER_2D.min_range = 0.3
TRAJECTORY_BUILDER_2D.max_range = 30.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 3.0

-- =====================================================
-- 线程优化
-- =====================================================
MAP_BUILDER.num_background_threads = 4 

return options