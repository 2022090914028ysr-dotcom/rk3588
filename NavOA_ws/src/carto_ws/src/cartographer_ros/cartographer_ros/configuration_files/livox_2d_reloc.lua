-- =====================================================================
-- livox_2d_reloc.lua
--
-- 流程第 1 阶段：纯定位。
-- 由 reloc_then_map.py 通过 /start_trajectory 服务启动，使用
-- pure_localization_trimmer 限制定位轨迹只保留最近子图，内存有界，
-- 不会扩展已加载的地图。
-- =====================================================================

include "livox_2d.lua"

options.publish_tracked_pose = true   -- 发布 /tracked_pose（map 系），用于 TF 兜底
options.use_odometry          = true
options.provide_odom_frame    = false -- 外部腿部里程计发布 odom->base_link
options.published_frame       = "odom"
options.publish_to_tf         = true   -- 发布 map->odom，RViz/nav 立即可见定位结果

MAP_BUILDER.num_background_threads = 4

-- 纯定位：只保留最近 max_submaps_to_keep 个子图
TRAJECTORY_BUILDER.pure_localization_trimmer = {
  max_submaps_to_keep = 3,
}

-- 前端实时扫描匹配（开：让扫描快速锁到地图上）
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window  = 0.5
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(30.)

TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.02
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians   = math.rad(0.5)
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds    = 0.1

-- 后端：频繁优化 + 全局搜索，尽快锁定到冻结地图
POSE_GRAPH.optimize_every_n_nodes = 1
POSE_GRAPH.global_sampling_ratio  = 0.3
POSE_GRAPH.global_constraint_search_after_n_seconds = 3

POSE_GRAPH.constraint_builder.min_score = 0.55
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.55
POSE_GRAPH.constraint_builder.sampling_ratio = 0.5
POSE_GRAPH.constraint_builder.max_constraint_distance = 10

POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window  = 10.0
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(180.)
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.branch_and_bound_depth = 12

TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 20.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight    = 1.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight       = 1.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 10

TRAJECTORY_BUILDER_2D.voxel_filter_size          = 0.05
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1
TRAJECTORY_BUILDER_2D.submaps.num_range_data     = 30
TRAJECTORY_BUILDER_2D.min_range     = 0.1
TRAJECTORY_BUILDER_2D.max_range     = 10.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 3.0

return options