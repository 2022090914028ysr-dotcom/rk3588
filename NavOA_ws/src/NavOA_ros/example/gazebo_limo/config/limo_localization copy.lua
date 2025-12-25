include "limo_2d.lua"

-- ================= 核心调整 =================

-- [1] 适度扩大搜索范围（保留）
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.15 
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(10.)

-- [2] 降低雷达匹配权重（关键！）
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 2.0    -- 原 10
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 5.0       -- 原 20
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 10.0         -- 原 40

-- [3] 放宽 motion_filter（关键！）
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.05
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(1.)
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 1.0   -- 原 0.1 → 1.0

-- [4] 保留降延迟设置
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1
TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.1

-- ================= 后端配置 =================
TRAJECTORY_BUILDER.pure_localization_trimmer = {
  max_submaps_to_keep = 3,
}

POSE_GRAPH.global_sampling_ratio = 0.015
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.55
POSE_GRAPH.optimize_every_n_nodes = 40 
POSE_GRAPH.constraint_builder.sampling_ratio = 0.2
POSE_GRAPH.constraint_builder.min_score = 0.65

return options