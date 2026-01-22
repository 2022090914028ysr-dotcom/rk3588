include "limo_2d.lua"

-- 2. [关键] 拒绝延迟 (解决 Rviz 画面滞后)
-- 默认为 2 或 3，改成 1。雷达来一帧算一帧，绝对不等待堆叠。
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1

-- 3. [关键] 稀疏化点云 (极大提升计算速度)
-- 改为 0.1 (10cm)。对于纯定位，10cm 的格子足够了，计算量减少 16 倍。
TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.1

-- 4. [速度] 缩小实时搜索范围
-- 既然有 rf2o 里程计，不要在大海捞针。缩小搜索窗，算得飞快。
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.05
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(10.)

-- 5. [灵敏] 提升刷新率
-- 只要动一点点 (5cm) 就立刻发布 TF，不再傻等。
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.05
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(2.)
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 0.5

-- ========================================================
-- 后端配置：针对 "初始定位要一两分钟" 的问题
-- ========================================================

TRAJECTORY_BUILDER.pure_localization_trimmer = {
  max_submaps_to_keep = 3,
}

-- 6. [初始化] 加大全局搜索力度 (解决开机定位慢)
-- 调大 5 倍 (0.003 -> 0.015)，让它开机时更疯狂地在全图找位置。
POSE_GRAPH.global_sampling_ratio = 0.015

-- 7. [初始化] 降低匹配门槛
-- 0.55 分就认为匹配成功。先定住，再修正，别太挑剔。
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.55

-- 8. [减负] 降低常规维护开销
-- 既然地图是固定的，日常维护可以懒一点，把算力留给上面的全局搜索。
POSE_GRAPH.optimize_every_n_nodes = 40 
POSE_GRAPH.constraint_builder.sampling_ratio = 0.2
POSE_GRAPH.constraint_builder.min_score = 0.65

return options