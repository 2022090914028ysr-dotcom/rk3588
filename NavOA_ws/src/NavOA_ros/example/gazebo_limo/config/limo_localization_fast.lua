include "limo_2d.lua"

-- ================= 前端（Local SLAM）参数优化：重定位增强 =================

-- [1] 扩大前端搜索范围，辅助重定位
-- 当机器人打滑或初始位置给的不准时，允许在更大的范围内搜索匹配点
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.25  -- 原 0.15 -> 0.25 (米)
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(20.)  -- 原 10度 -> 20度

-- [2] 优化雷达匹配权重，平衡精度与重定位
-- 降低平移和旋转的刚性约束，更依赖环境几何特征（occupied_space），有助于在相似走廊中定位
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 1.5    -- 原 2.0 -> 1.5
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 4.0       -- 原 5.0 -> 4.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 8.0          -- 原 10.0 -> 8.0

-- [3] 放宽 Motion Filter，对微小移动也进行更新
-- 增加更新频率，防止机器人动了但 Cartographer 以为没动，导致定位滞后
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.02          -- 原 0.05 -> 0.02 (米)
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.5)   -- 原 1度 -> 0.5度
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 0.5              -- 原 1.0 -> 0.5 (秒)

-- [4] 细化体素滤波
-- 0.05m 的网格，保留更多细节，提高匹配精度
TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.05                          -- 原 0.1 -> 0.05

-- [5] 多帧雷达数据累积
-- 累积 2 帧数据再进行一次匹配，增加特征点密度，防止空旷区域特征不足
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 2                    -- 原 1 -> 2

-- ================= 后端（Global SLAM）设置：纯定位模式 =================

-- 纯定位模式设置（关键设置）
-- 只保留最新的 2 个子图用于匹配，旧的地图数据来自静态 .pbstream 文件
-- 这样可以防止 Cartographer 试图在已有地图上覆盖新的建图数据
TRAJECTORY_BUILDER.pure_localization_trimmer = {
  max_submaps_to_keep = 2,  -- 原 3 -> 2，保持子图窗口最小化，加快匹配
}

-- 全局约束搜索窗口（Global Localization）
-- 当发生回环检测或全局重定位时，在非常大的范围内搜索（5米，30度）
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 5.0  -- 扩大全局搜索范围
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(30.)
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.branch_and_bound_depth = 7

-- 降低匹配得分阈值，提高重定位成功率
-- 分数越低，越容易认为“匹配上了”，但也越容易出现错误匹配（误识别）
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.45        -- 原 0.55 -> 0.45 (全局定位阈值)
POSE_GRAPH.constraint_builder.min_score = 0.55                            -- 原 0.65 -> 0.55 (约束匹配阈值)

-- 增加后端优化采样频率
-- 让后端看到更多的数据，从而更频繁地尝试构建约束
POSE_GRAPH.global_sampling_ratio = 0.03                                   -- 原 0.015 -> 0.03
POSE_GRAPH.constraint_builder.sampling_ratio = 0.3                        -- 原 0.2 -> 0.3
POSE_GRAPH.constraint_builder.max_constraint_distance = 15.0              -- 最大约束距离

-- 高频进行位姿图优化
POSE_GRAPH.optimize_every_n_nodes = 20                                    -- 原 40 -> 20 (每插入20个节点优化一次)

-- 自适应参数（这是 Cartographer 旧版本的参数，新版通常已弃用或更改逻辑）
-- POSE_GRAPH.adaptive_parameters = {
--   enabled = true,
--   min_score = 0.4,
--   global_localization_min_score = 0.35,
-- }

-- ================= 子图（Submaps）参数优化 =================
-- 减小子图大小，使地图更新更敏捷
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 40                         -- 原 60 -> 40 (每个子图包含的雷达帧数)
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.05           -- 原 0.05 (分辨率)

-- ================= 重定位专用模式 =================
-- 启动时通过 ROS 参数传入
-- options.fast_relocalization = true  -- 快速重定位标志

return options