-- =====================================================================
-- livox_2d_map_continue.lua
--
-- 流程第 2 阶段：建图延伸。
-- 由 reloc_then_map.py 在定位轨迹 finish 后通过 /start_trajectory 启动，
-- 以保存的 map->base TF 换算出的 relative_pose 为种子，与已加载地图
-- 处于同一 pose graph，无缝扩展地图。无 trimmer，保留全部新子图。
-- =====================================================================

include "livox_2d.lua"

options.publish_tracked_pose = true
options.use_odometry          = true
options.provide_odom_frame    = false
options.published_frame       = "odom"
options.publish_to_tf         = true   -- 继续发布 map->odom 供导航使用

MAP_BUILDER.num_background_threads = 4

-- 前端实时扫描匹配：新轨迹的扫描尽快锁到周围（含冻结）子图上
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window  = 0.3
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(20.)

TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.02
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians   = math.rad(0.5)
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds    = 0.1

-- 后端：适度频繁优化、全局采样，宽窗口以便新节点回环到冻结子图
POSE_GRAPH.optimize_every_n_nodes = 10
POSE_GRAPH.global_sampling_ratio  = 0.01

POSE_GRAPH.constraint_builder.min_score = 0.55
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.55
POSE_GRAPH.constraint_builder.sampling_ratio = 0.3
POSE_GRAPH.constraint_builder.max_constraint_distance = 15.0

POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window  = 8.0
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(120.)
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.branch_and_bound_depth = 7

-- 避免单个坏约束主导优化
POSE_GRAPH.optimization_problem.huber_scale = 5e0
POSE_GRAPH.optimization_problem.odometry_translation_weight = 1e4
POSE_GRAPH.optimization_problem.odometry_rotation_weight    = 1e4

-- 建图质量（与原 livox_2d.lua 一致）
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 1.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight    = 10.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight       = 40.0

TRAJECTORY_BUILDER_2D.voxel_filter_size          = 0.05
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1
TRAJECTORY_BUILDER_2D.submaps.num_range_data     = 60
TRAJECTORY_BUILDER_2D.min_range     = 0.1
TRAJECTORY_BUILDER_2D.max_range     = 8.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 1.5

return options