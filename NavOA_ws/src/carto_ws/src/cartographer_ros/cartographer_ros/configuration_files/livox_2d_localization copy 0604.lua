include "livox_2d.lua"

options.publish_tracked_pose = true
options.publish_tracked_pose = true
options.use_odometry = true          -- rf2o is not running in localization mode (if true rf2o is running)
options.provide_odom_frame = false     -- Cartographer provides map→odom→base_link TF without external odometry
options.published_frame = "odom" --for rf2o make it odom_frame = "odom"--for rf2o make it odom

MAP_BUILDER.num_background_threads = 8    -- 增加后台线程数（根据CPU核心数调整）
-- =====================================================
-- 纯定位模式配置（保留更少的子图以减少计算）
-- =====================================================
TRAJECTORY_BUILDER.pure_localization_trimmer = {
  max_submaps_to_keep = 3,  -- 从3减少到2，更快匹配
}

-- =====================================================
-- 优化频率（更频繁的优化帮助快速重定位）
-- =====================================================
POSE_GRAPH.optimize_every_n_nodes = 3  -- 从20减少到15减少到 5，更频繁优化

-- =====================================================
-- 前端实时扫描匹配（扩大搜索范围）
-- =====================================================
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.3   -- 从0.1增加到0.3
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(15.)  -- 新增角度搜索 ±15°
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 5.0   -- 降低权重加快收敛
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 0.05     -- 从0.1降低

-- =====================================================
-- Ceres扫描匹配（降低精度换取速度）
-- =====================================================
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 1.0   -- 从默认1.5降低
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 2.0      -- 降低
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 4.0         -- 降低
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 10  -- 减少迭代次数

-- =====================================================
-- 运动滤波器（减少冗余计算）
-- =====================================================
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.03   -- 增大阈值，减少匹配次数
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(1.0)  -- 增大角度阈值
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 0.3        -- 减少时间阈值，更频繁检查

-- =====================================================
-- 体素滤波（降低分辨率提升速度）
-- =====================================================
TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.08   -- 从默认0.05增大到0.08，点数减少约40%

-- =====================================================
-- 多帧累积（减少累积帧数加快处理）
-- =====================================================
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1  -- 从默认2减少到1，实时处理

-- =====================================================
-- 子图参数（降低子图精度提升速度）
-- =====================================================
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 30     -- 从35减少到30，更快完成子图
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.10  -- 降低子图分辨率（从0.05到0.10）

-- =====================================================
-- 后端全局定位（快速重定位核心）
-- =====================================================
POSE_GRAPH.global_sampling_ratio = 0.05   -- 采样5%的节点（0.02太少导致错误收敛）

-- 快速相关扫描匹配器（用于重定位）
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 8.0   -- 扩大到8m搜索范围
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(45.)  -- 扩大到±45°
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.branch_and_bound_depth = 7   -- 深度7，平衡精度与速度（6太浅导致错误收敛）

-- 重定位分数阈值（降低阈值更容易匹配成功）
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.60   -- 全局定位最低分0.50（0.40太低，接受错误匹配）
POSE_GRAPH.constraint_builder.min_score = 0.62                       -- 约束匹配分0.55（0.50太低）

-- 约束构建加速
POSE_GRAPH.constraint_builder.sampling_ratio = 0.25   -- 只采样25%的约束（从默认1.0降低）
POSE_GRAPH.constraint_builder.max_constraint_distance = 15  -- 增加搜索距离，帮助找到远距离回环

-- =====================================================
-- 距离和角度范围（限制无效计算）
-- =====================================================
TRAJECTORY_BUILDER_2D.min_range = 0.1     -- 从0.2增加到0.3，忽略太近的点
TRAJECTORY_BUILDER_2D.max_range = 10    -- 从100减少到30，减少点数
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 1.0  -- 从5减少到3



return options