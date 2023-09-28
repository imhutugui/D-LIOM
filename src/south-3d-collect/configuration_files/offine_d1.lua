include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "imu_frame",
  sensor_type = "velodyne",
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = false,
  publish_frame_projected_to_2d = false,
  use_odometry = false,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 0,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 2,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  full_map_cloud_publish_period_sec = 5.;
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

TRAJECTORY_BUILDER_3D.min_range = 0.5
TRAJECTORY_BUILDER_3D.max_range = 60
TRAJECTORY_BUILDER_3D.voxel_filter_size = 0.1

TRAJECTORY_BUILDER_3D.num_accumulated_range_data = 160
TRAJECTORY_BUILDER_3D.submaps.num_range_data = 160
TRAJECTORY_BUILDER_3D.submaps.high_resolution = 0.05
TRAJECTORY_BUILDER_3D.submaps.low_resolution = 0.4


TRAJECTORY_BUILDER_3D.enable_gravity_factor = true
TRAJECTORY_BUILDER_3D.imu.prior_gravity_noise = 0.1

TRAJECTORY_BUILDER_3D.scan_period = 0.05
TRAJECTORY_BUILDER_3D.eable_mannually_discrew = false
TRAJECTORY_BUILDER_3D.frames_for_static_initialization = 7
TRAJECTORY_BUILDER_3D.frames_for_dynamic_initialization = 7
TRAJECTORY_BUILDER_3D.frames_for_online_gravity_estimate = 3
TRAJECTORY_BUILDER_3D.enable_ndt_initialization = false

TRAJECTORY_BUILDER_3D.imu.acc_noise= 1.98392079e01
TRAJECTORY_BUILDER_3D.imu.gyr_noise= 1.71348431e00
TRAJECTORY_BUILDER_3D.imu.acc_bias_noise= 4.6552e-04
TRAJECTORY_BUILDER_3D.imu.gyr_bias_noise= 6.6696e-06
TRAJECTORY_BUILDER_3D.imu.gravity= 9.80

TRAJECTORY_BUILDER_3D.imu.ceres_pose_noise_t = 0.1
TRAJECTORY_BUILDER_3D.imu.ceres_pose_noise_r = 0.1
TRAJECTORY_BUILDER_3D.imu.ceres_pose_noise_t_drift = 0.01
TRAJECTORY_BUILDER_3D.imu.ceres_pose_noise_r_drift = 0.01

TRAJECTORY_BUILDER_3D.imu.prior_pose_noise = 0.1
TRAJECTORY_BUILDER_3D.imu.prior_vel_noise = 0.1
TRAJECTORY_BUILDER_3D.imu.prior_bias_noise = 1e-03


-- TRAJECTORY_BUILDER_3D.pose_extrapolator.imu_based.gravity_constant = 9.7672
-- TRAJECTORY_BUILDER_3D.ceres_scan_matcher.translation_weight = 1


TRAJECTORY_BUILDER_3D.ceres_scan_matcher.occupied_space_weight_0 = 6 -- 1
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.occupied_space_weight_1 = 6 -- 6
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.rotation_weight = 4e2 -- 4e2
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.translation_weight = 5 -- 5
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 12
-- TRAJECTORY_BUILDER_3D.ceres_scan_matcher.ceres_solver_options.num_threads = 2


MAP_BUILDER.use_trajectory_builder_3d = true
MAP_BUILDER.num_background_threads = 10
POSE_GRAPH.optimize_every_n_nodes = 320
POSE_GRAPH.constraint_builder.sampling_ratio = 0.1
-- POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 200
POSE_GRAPH.constraint_builder.min_score = 0.58
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.66
-- POSE_GRAPH.optimization_problem.use_online_imu_extrinsics_in_3d = true
-- -- 增加全局搜索范围
-- POSE_GRAPH.constraint_builder.max_constraint_distance = 35
-- POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.linear_z_search_window = 3
-- POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.linear_xy_search_window = 10
POSE_GRAPH.optimization_problem.huber_scale = 5e2
POSE_GRAPH.optimization_problem.log_solver_summary = true
POSE_GRAPH.constraint_builder.log_matches = true
POSE_GRAPH.max_num_final_iterations = 600


return options

