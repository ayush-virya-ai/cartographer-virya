
include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "imu_link",
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = true,
  publish_frame_projected_to_2d = false,
  use_odometry = false,
  use_nav_sat = false,
  use_landmarks = false,
  publish_tracked_pose = true,
  num_laser_scans = 0,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 1,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 0.01,
  trajectory_publish_period_sec = 0.03,
  rangefinder_sampling_ratio = 1.0,
  odometry_sampling_ratio = 1.0,
  fixed_frame_pose_sampling_ratio = 1.0,
  imu_sampling_ratio = 1.0,
  landmarks_sampling_ratio = 1.0,
  use_pose_extrapolator = true,
}
  
MAP_BUILDER.use_trajectory_builder_3d = true

-- Modifications to these values are for the Local SLAM (trajectory building)
TRAJECTORY_BUILDER_3D = {
  min_range = 1.0,
  max_range = 250.0,
  num_accumulated_range_data = 1,
  voxel_filter_size = 0.05,
  use_imu_data = true,
 
  --- CHANGE: Increased from 2.0 to 10.0. A low value causes the gravity vector to be unstable
  -- by reacting too quickly to linear accelerations. This is a primary fix for Z-axis tilt.
  imu_gravity_time_constant = 10.0,
  
  rotational_histogram_size = 30,
  high_resolution_adaptive_voxel_filter = {
    max_length = 1.0,
    max_range = 15.0,
    min_num_points = 100,
  },

  submaps = {
    high_resolution = 0.07,
    high_resolution_max_range = 60.,
    low_resolution = 0.45,
    num_range_data = 30,
    range_data_inserter = {
      hit_probability = 0.55,
      miss_probability = 0.49,
    },
  },

  ceres_scan_matcher = {
    occupied_space_weight_0 = 2.5,
    occupied_space_weight_1 = 4.0,
    translation_weight = 15.0,
    rotation_weight = 3.0,
    only_optimize_yaw = false,
    ceres_solver_options = {
      use_nonmonotonic_steps = false,
      max_num_iterations = 100,
      num_threads = 1,
    },
  }
}

-- Modifications to these values are for the Global SLAM (pose graph and loop closure)
POSE_GRAPH = {
  optimize_every_n_nodes = 90,
  global_sampling_ratio = 0.003,
  max_num_final_iterations = 200,
  log_residual_histograms = false,
  global_constraint_search_after_n_seconds = 1.0,

  constraint_builder = {
    sampling_ratio = 0.07,
    max_constraint_distance = 500.0,
    min_score = 0.55,
    global_localization_min_score = 0.4,
    loop_closure_translation_weight = 1e4,
    loop_closure_rotation_weight = 1e3,

    fast_correlative_scan_matcher_3d = {
      branch_and_bound_depth = 7,
      full_resolution_depth = 3,
      min_rotational_score = 0.77,
      min_low_resolution_score = 0.55,
      linear_xy_search_window = 30.0,
      linear_z_search_window = 1.0,
      angular_search_window = math.rad(30.0),
    },

    ceres_scan_matcher_3d = {
      occupied_space_weight_0 = 5.0,
      occupied_space_weight_1 = 30.0,
      translation_weight = 10.0,
      rotation_weight = 1.0,
      only_optimize_yaw = false,
      ceres_solver_options = {
        use_nonmonotonic_steps = false,
        max_num_iterations = 10,
        num_threads = 1,
      },
    },
  },

  optimization_problem = {
    --- CHANGE: Decreased from 1000.0. A very high value can make the optimizer less robust to outliers. 5e1 is the default.
    huber_scale = 5e1,
    
    --- CHANGE: Decreased from 500.0. High weights force the pose graph to trust a potentially drifty IMU too much.
    -- This is a primary fix for global Z-axis drift.
    acceleration_weight = 1e1,
    rotation_weight = 1e2,
    
    log_solver_summary = true,
    
    -- NOTE: If Z-axis issues persist, try setting this to 'false'. It disables online calibration of the
    -- IMU's orientation, which can be unstable if the robot doesn't perform enough rotational motion.
    use_online_imu_extrinsics_in_3d = true,
    
    ceres_solver_options = {
      use_nonmonotonic_steps = false,
      max_num_iterations = 100,
      num_threads = 4,
    },
  }
}

return options