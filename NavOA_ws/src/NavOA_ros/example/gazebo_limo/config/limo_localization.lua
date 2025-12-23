include "limo_2d.lua"

-- ================= 核心调整（优化重定位） =================

-- [1] 扩大搜索范围以提升重定位能力
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.25  -- 原0.15 → 0.25
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(20.)  -- 原10° → 20°

-- [2] 优化雷达匹配权重（平衡精度与重定位）
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 1.5    -- 原2.0 → 1.5
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 4.0       -- 原5.0 → 4.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 8.0          -- 原10.0 → 8.0

-- [3] 进一步放宽 motion_filter 以保留更多数据用于重定位
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.02          -- 原0.05 → 0.02
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.5)   -- 原1° → 0.5°
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 0.5              -- 原1.0 → 0.5

-- [4] 降低体素滤波粒度以保留更多特征
TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.05                          -- 原0.1 → 0.05

-- [5] 增加累计数据以提供更多上下文信息（重定位时有用）
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 2                    -- 原1 → 2

-- ================= 后端配置（优化重定位） =================

-- 纯定位模式配置（关键！）
TRAJECTORY_BUILDER.pure_localization_trimmer = {
  max_submaps_to_keep = 2,  -- 原3 → 2，减少子图数量以加快匹配
}

-- 全局约束构建优化
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 5.0  -- 扩大全局搜索
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(30.)
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.branch_and_bound_depth = 7

-- 降低匹配阈值以提升重定位成功率
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.45        -- 原0.55 → 0.45
POSE_GRAPH.constraint_builder.min_score = 0.55                            -- 原0.65 → 0.55

-- 优化采样和约束频率
POSE_GRAPH.global_sampling_ratio = 0.03                                   -- 原0.015 → 0.03
POSE_GRAPH.constraint_builder.sampling_ratio = 0.3                        -- 原0.2 → 0.3
POSE_GRAPH.constraint_builder.max_constraint_distance = 15.0              -- 增加约束距离

-- 更频繁的优化以快速收敛
POSE_GRAPH.optimize_every_n_nodes = 20                                    -- 原40 → 20

-- 添加自适应参数（Cartographer自适应版本可用）
-- POSE_GRAPH.adaptive_parameters = {
--   enabled = true,
--   min_score = 0.4,
--   global_localization_min_score = 0.35,
-- }

-- ================= 子图配置优化 =================
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 40                         -- 原60 → 40，更小的子图
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.05           -- 原0.05保持不变

-- ================= 重定位专用模式开关 =================
-- 可在启动时通过ROS参数设置
-- options.fast_relocalization = true  -- 自定义标志

return options