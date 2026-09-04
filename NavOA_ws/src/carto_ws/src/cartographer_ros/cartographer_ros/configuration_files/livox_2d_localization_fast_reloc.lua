include "livox_2d.lua"

options.publish_tracked_pose = true
options.use_odometry = true          -- rf2o is not running in localization mode (if true rf2o is running)
options.provide_odom_frame = false     -- Cartographer provides map→odom→base_link TF without external odometry
options.published_frame = "odom" --for rf2o make it odom_frame = "odom"                 --for rf2o make it odom
options.publish_to_tf = true
-- =====================================================
-- [新增] 请改用这行！这是限制算法内部线程的正确位置
MAP_BUILDER.num_background_threads = 6

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
POSE_GRAPH.global_constraint_search_after_n_seconds = 10   -- force global match every node

POSE_GRAPH.global_sampling_ratio = 0.01


-- =====================================================
-- [新增] 里程计一致性校验：全局定位候选位姿在创建约束前，用可信里程计
-- 校验其与上一节点之间的相对运动是否一致。不一致（如重复结构导致的错误
-- 高分匹配）则直接拒绝，约束不会进入位姿图，避免位姿被拉偏。
-- =====================================================
POSE_GRAPH.constraint_builder.global_localization_odometry_check_enabled = true   -- 开启里程计校验（关闭即为原始行为）
POSE_GRAPH.constraint_builder.global_localization_max_odometry_translation_error = 1.0        -- 相对平移允许的最大误差（米）
POSE_GRAPH.constraint_builder.global_localization_max_odometry_rotation_error = math.rad(30.) -- 相对旋转允许的最大误差（弧度）

-- POSE_GRAPH.constraint_builder.sampling_ratio = 0.05-- 约束构建
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 5.0
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(30.)
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.branch_and_bound_depth = 7

POSE_GRAPH.constraint_builder.global_localization_min_score = 0.55 
POSE_GRAPH.constraint_builder.min_score = 0.55

POSE_GRAPH.constraint_builder.sampling_ratio = 0.3
POSE_GRAPH.constraint_builder.max_constraint_distance = 15.0

-- 子图参数
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 20
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.05

return options