include "limo_2d.lua"

-- ==================== 策略核心：开启纯定位模式 ====================
TRAJECTORY_BUILDER.pure_localization_trimmer = {
  max_submaps_to_keep = 2,
}

-- ==================== 前后端微调（针对定位优化） ====================
-- 没有了外部odom，如果雷达短暂被遮挡，重定位的搜索范围需要稍微大一点
-- TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.25
-- 3. 稍微放大前端扫描匹配的窗口：防止小幅度搬动直接丢定位
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.3
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(20.)

-- 降低后端全局优化的触发阈值
POSE_GRAPH.optimize_every_n_nodes = 20 

-- 1. 提高全局采样的频率，后台更频繁地拿着当前数据去全图撞大运（会增加CPU负载！）
POSE_GRAPH.global_sampling_ratio = 0.015  -- 如果原本很小（比如0.003），适当提高

-- 全局定位的置信度门槛
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.55
-- POSE_GRAPH.constraint_builder.min_score = 0.55
-- 2. 严格限制局部匹配得分：得分低于这个值，后端就认为局部跟踪失效了
POSE_GRAPH.constraint_builder.min_score = 0.60  -- 原本如果是0.55，稍微调高点，让它眼里容不得沙子

return options



