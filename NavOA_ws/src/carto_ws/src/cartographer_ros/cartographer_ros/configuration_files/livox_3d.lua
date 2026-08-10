-- Cartographer 3D ˫��������ͼ����
-- Livox 2D ���� + HP60C ����������

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "odom",
  odom_frame = "odom",
  provide_odom_frame = false,            -- rf2o ���𷢲�base_linkotprint
  publish_frame_projected_to_2d = false,
  use_pose_extrapolator = true,
  use_odometry = true,                   -- ���� /odom_rf2o
  use_nav_sat = false,
  use_landmarks = false,

  -- Livox 2D ����
  num_laser_scans = 1,
  -- HP60C ����������
  num_point_clouds = 1,

  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,

  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 0.01,
  trajectory_publish_period_sec = 0.03,

  rangefinder_sampling_ratio = 1.0,
  odometry_sampling_ratio = 1.0,
  fixed_frame_pose_sampling_ratio = 1.0,
  imu_sampling_ratio = 1.0,
  landmarks_sampling_ratio = 1.0,
}

-- ���� 3D �켣������
MAP_BUILDER.use_trajectory_builder_3d = true

-- 3D ��ͼ����
TRAJECTORY_BUILDER_3D.num_accumulated_range_data = 30
TRAJECTORY_BUILDER_3D.min_range = 0.1
TRAJECTORY_BUILDER_3D.max_range = 4.0

-- �����˲�����������
TRAJECTORY_BUILDER_3D.voxel_filter_size = 0.05
TRAJECTORY_BUILDER_3D.high_resolution_adaptive_voxel_filter.max_length = 0.5
TRAJECTORY_BUILDER_3D.high_resolution_adaptive_voxel_filter.min_num_points = 100

-- ��ʹ�� IMU
TRAJECTORY_BUILDER_3D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_3D.real_time_correlative_scan_matcher.linear_search_window = 0.15
TRAJECTORY_BUILDER_3D.real_time_correlative_scan_matcher.angular_search_window = math.rad(15.)

-- Ceres ɨ��ƥ��
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.occupied_space_weight_0 = 1.0
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.occupied_space_weight_1 = 6.0
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.translation_weight = 10.0
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.rotation_weight = 40.0

-- Submap
TRAJECTORY_BUILDER_3D.submaps.num_range_data = 90

-- �ػ����
POSE_GRAPH.optimization_problem.huber_scale = 5e2
POSE_GRAPH.optimize_every_n_nodes = 90
POSE_GRAPH.constraint_builder.min_score = 0.65
POSE_GRAPH.constraint_builder.sampling_ratio = 0.3
POSE_GRAPH.constraint_builder.max_constraint_distance = 15.0
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.6

return options
