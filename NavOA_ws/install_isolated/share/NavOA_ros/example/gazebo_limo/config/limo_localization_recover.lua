include "limo_2d.lua"

-- Faster re-localization when the robot is stopped.
-- Trade-offs: higher CPU usage and higher risk of false matches if scores are set too low.

-- Correct place to set background threads for this Cartographer version.
MAP_BUILDER.num_background_threads = 2

-- Process scans even when stationary (important for fast recovery).
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.02
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.5)
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 0.5

-- Lower latency.
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 2

-- Keep submaps small for quicker matching.
TRAJECTORY_BUILDER.pure_localization_trimmer = {
  max_submaps_to_keep = 2,
}

-- More frequent optimization so constraints take effect quickly.
POSE_GRAPH.optimize_every_n_nodes = 20

-- Try global constraints more often (this is what helps recovery).
POSE_GRAPH.global_sampling_ratio = 0.05
POSE_GRAPH.constraint_builder.sampling_ratio = 0.3

-- Global localization acceptance threshold.
-- If you see occasional wrong "teleport" to a different corridor, increase this (e.g. 0.45).
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.40

-- Keep local constraint threshold conservative.
POSE_GRAPH.constraint_builder.min_score = 0.55

-- Search windows: wide enough to recover, but can be expensive.
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 5.0
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(30.)
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.branch_and_bound_depth = 7

POSE_GRAPH.constraint_builder.max_constraint_distance = 15.0

return options
