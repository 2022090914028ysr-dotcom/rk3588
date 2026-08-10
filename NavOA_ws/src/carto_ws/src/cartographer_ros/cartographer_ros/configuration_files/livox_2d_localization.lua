-- include "livox_2d.lua"

-- options.publish_tracked_pose = true
-- options.use_odometry = true          -- rf2o is not running in localization mode (if true rf2o is running)
-- options.provide_odom_frame = false     -- Cartographer provides map→odom→base_link TF without external odometry
-- options.published_frame = "odom" --for rf2o make it odom_frame = "odom"                 --for rf2o make it odom

-- -- =====================================================
-- -- [新增] 请改用这行！这是限制算法内部线程的正确位置
-- MAP_BUILDER.num_background_threads = 4

-- -- 优化频率
-- POSE_GRAPH.optimize_every_n_nodes = 20 
-- -- ==================== 以下保持原样 ====================

-- -- 前端搜索范围
-- TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.3
-- TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(20.)

-- -- 权重配置
-- TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 1.5
-- TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 4.0
-- TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 8.0

-- -- Motion Filter
-- TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.02
-- TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.5)
-- TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 0.5

-- -- 体素滤波 (保留 0.05 高精度)
-- TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.05

-- -- 多帧累积
-- TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 2

-- -- 后端设置
-- TRAJECTORY_BUILDER.pure_localization_trimmer = {
--   max_submaps_to_keep = 2,
-- }

-- -- 采样率
-- POSE_GRAPH.global_sampling_ratio = 0.015 

-- -- 约束构建
-- POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 5.0
-- POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(30.)
-- POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.branch_and_bound_depth = 7

-- POSE_GRAPH.constraint_builder.global_localization_min_score = 0.45
-- POSE_GRAPH.constraint_builder.min_score = 0.55

-- POSE_GRAPH.constraint_builder.sampling_ratio = 0.3
-- POSE_GRAPH.constraint_builder.max_constraint_distance = 15.0

-- -- 子图参数
-- TRAJECTORY_BUILDER_2D.submaps.num_range_data = 40
-- TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.05

-- return options

include "livox_2d.lua"


options.publish_tracked_pose = true
options.use_odometry = true          -- rf2o is not running in localization mode (if true rf2o is running)
options.provide_odom_frame = false     -- Cartographer provides map→odom→base_link TF without external odometry
options.published_frame = "odom" --for rf2o make it odom_frame = "odom"--for rf2o make it odom
-- | Parameter Name           | Description |
-- | :---                     | :---        |
-- | max_submaps_to_keep      | Maximum number of recent submaps retained for loop closure. |
-- | num_background_threads   | Number of CPU threads allocated for background computation. |
-- | global_sampling_ratio    | Ratio of nodes sampled for global matching/loop closure search. |
-- | optimize_every_n_nodes   | Triggers a global optimization run after every N nodes are inserted. |
-- | num_range_data           | Number of range data frames required to create a new submap. |
-- =====================================================
-- 纯定位模式配置（保留更少的子图以减少计算）
-- =====================================================
TRAJECTORY_BUILDER.pure_localization_trimmer = {
  max_submaps_to_keep = 3,  -- 10 个 node 一个子图，3 个子图相当于保留 30 个 node，每个node 0.2 秒钟，保留 6 秒钟的历史数据，足够进行局部匹配和重定位
}
MAP_BUILDER.num_background_threads = 4    -- 增加后台线程数（根据CPU核心数调整）
POSE_GRAPH.global_sampling_ratio = 0.1   -- 发生全局匹配的概率，0-1之间，值越大，发生全局匹配的概率越高。0.1 表示 10% 的概率进行全局匹配。
POSE_GRAPH.optimize_every_n_nodes = 2  -- 每插入 2 个 node 就进行一次全局优化，增加频率可以更快地修正位姿，但过高可能增加计算负担，适当调整以平衡速度和精度.
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 30     -- 30 个 node 完成一个子图，多一些更稳定。

POSE_GRAPH.constraint_builder.min_score = 0.60      -- 局部内匹配时的最低分，分数低一点儿也可以，可以加快计算，后续由 fast_correlative_scan_matcher和 Ceres来精修位姿，过高的分数可能导致无法快速匹配成功，适当降低以增加成功率
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.60   -- 全局定位最低分，高分数减少误匹配，但过高可能导致无法重定位，适当降低以增加成功率
-- POSE_GRAPH.constraint_builder.global_localization_min_score = 0.45
-- POSE_GRAPH.constraint_builder.min_score = 0.55
-- below: what you dont need to change unless you know what you are doing
-- | max_distance_meters      | Distance threshold (in meters) for new node insertion. |
-- | max_angle_radians        | Angular threshold (in radians) for new node insertion. |
-- | max_time_seconds         | Time threshold (in seconds) for new node insertion. |
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.1   -- 增大阈值，减少匹配次数
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(1.0)  -- 增大角度阈值
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 1        -- 减少时间阈值，更频繁检查

-- 快速相关扫描匹配器（用于重定位）
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 10.0   -- 单个房间的对角线长度，足够覆盖整个房间，帮助快速重定位
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(180.)  -- 扩大到±45°
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.branch_and_bound_depth = 12 -- 深度9，平衡精度与速度

-- global 触发后相关参数 
POSE_GRAPH.global_constraint_search_after_n_seconds = 3 -- 如果 5 秒没有成功匹配过，5秒后强制进行一次全局约束搜索，帮助快速发现丢失并重定位

-- local 触发后相关参数
-- 重定位分数阈值（降低阈值更容易匹配成功）
-- 约束构建加速
POSE_GRAPH.constraint_builder.sampling_ratio = 0.5   -- 地图越大，采样越少，地图小，采样就需要大一些，能够采样到更多子地图
POSE_GRAPH.constraint_builder.max_constraint_distance = 10 -- 增加搜索距离，帮助找到远距离回环，根据地图大小决定，如果地图很大，为了能够找到位姿，需要至少设到面积的对角线长度的一半以上

-- =====================================================
-- 前端实时扫描匹配（扩大搜索范围）
-- =====================================================
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.3   -- 实时相关扫描匹配的线性搜索窗口，单位是米，增大到0.5米，帮助在局部范围内找到匹配，过大可能增加计算量，但对于定位来说更重要的是成功率
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(30.)  -- 新增角度搜索 ±30°
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 0.05   -- 降低权重加快收敛
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 0.05     -- 从0.1降低

-- =====================================================
-- Ceres扫描匹配（降低精度换取速度）
-- =====================================================
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 20.0   -- 从默认1.5降低
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 1.0      -- 降低
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 1.0         -- 降低
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 10  -- 减少迭代次数
 

-- =====================================================
-- 体素滤波（降低分辨率提升速度）
-- =====================================================
TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.08   -- 从默认0.05增大到0.08，点数减少约40%

-- =====================================================
-- 多帧累积（减少累积帧数加快处理）
-- =====================================================
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1  -- 2 帧累积成一个 node，

-- TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.10  -- 降低子图分辨率（从0.05到0.10）

-- =====================================================
-- 距离和角度范围（限制无效计算） 传感器参数
-- =====================================================
TRAJECTORY_BUILDER_2D.min_range = 0.1     -- 从0.2增加到0.3，忽略太近的点
TRAJECTORY_BUILDER_2D.max_range = 10    -- 从100减少到30，减少点数
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 3.0  -- 从5减少到3


return options