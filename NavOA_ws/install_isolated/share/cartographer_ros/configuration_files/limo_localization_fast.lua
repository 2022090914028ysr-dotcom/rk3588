include "limo_2d.lua"

-- ==================== 核心修复 ====================
-- [删除] 下面这行会导致崩溃，因为你的 Cartographer 版本不识别它
-- options.num_background_threads = 2 

-- [新增] 请改用这行！这是限制算法内部线程的正确位置
MAP_BUILDER.num_background_threads = 2

-- ==================== 以下保持原样 ====================

-- 前端搜索范围
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.25
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(20.)

-- 权重配置
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 1.5
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 4.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 8.0

-- Motion Filter
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.02
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.5)
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 0.5

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
POSE_GRAPH.global_sampling_ratio = 0.015 

-- 约束构建
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 5.0
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(30.)
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.branch_and_bound_depth = 7

POSE_GRAPH.constraint_builder.global_localization_min_score = 0.45
POSE_GRAPH.constraint_builder.min_score = 0.55

POSE_GRAPH.constraint_builder.sampling_ratio = 0.3
POSE_GRAPH.constraint_builder.max_constraint_distance = 15.0

-- 子图参数
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 40
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.05

return options