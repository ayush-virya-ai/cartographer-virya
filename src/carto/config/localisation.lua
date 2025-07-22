-- Copyright 2016 The Cartographer Authors
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.

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

TRAJECTORY_BUILDER_3D = {
  min_range = 1.0,
  max_range = 250.0,
  num_accumulated_range_data = 1,
  voxel_filter_size = 0.05,
  use_imu_data = true,
  imu_gravity_time_constant = 10.0,
  rotational_histogram_size = 30,
  
  -- Local scan matching parameters can often remain the same as mapping.
  ceres_scan_matcher = {
    occupied_space_weight_0 = 2.5,
    occupied_space_weight_1 = 4.0,
    translation_weight = 15.0,
    rotation_weight = 3.0,
    only_optimize_yaw = false,
    ceres_solver_options = {
      use_nonmonotonic_steps = false,
      max_num_iterations = 10, -- Can be lower for localization as we only match to existing submaps.
      num_threads = 1,
    },
  },
  
  -- Other sections like high_resolution_adaptive_voxel_filter and submaps are not critical here
  -- as we are not creating new submaps.
  submaps = {
    high_resolution = 0.07,
    high_resolution_max_range = 75.,
    low_resolution = 0.45,
    num_range_data = 15,
    range_data_inserter = {
      hit_probability = 0.55,
      miss_probability = 0.49,
    },
  },
}

-- ########################################################################################
-- ##                  MOST IMPORTANT CHANGES ARE IN THE POSE_GRAPH                      ##
-- ########################################################################################

POSE_GRAPH = {
  --- CHANGE: This is the MASTER switch for localization mode.
  -- Setting to 0 disables the pose graph optimizer, effectively freezing the map.
  optimize_every_n_nodes = 0,

  --- CHANGE: Set to a low value to force continuous global searching.
  -- This makes Cartographer constantly try to find its place in the map.
  global_constraint_search_after_n_seconds = 1.0,

  max_num_final_iterations = 200,
  global_sampling_ratio = 0.003,
  log_residual_histograms = false,

  constraint_builder = {
    sampling_ratio = 0.3, -- Sample more scans to find a match.
    max_constraint_distance = 500.0,
    min_score = 0.55,
    global_localization_min_score = 0.6, -- Require a higher score for confident localization.
    loop_closure_translation_weight = 1e4,
    loop_closure_rotation_weight = 1e3,

    fast_correlative_scan_matcher_3d = {
      branch_and_bound_depth = 8,
      full_resolution_depth = 3,
      min_rotational_score = 0.77,
      min_low_resolution_score = 0.55,
      
      --- CHANGE: Increase search windows to help the robot find its initial pose if it's lost.
      linear_xy_search_window = 7.0,
      linear_z_search_window = 3.0,
      angular_search_window = math.rad(30.0),
    },
    
    -- The ceres_scan_matcher_3d in the constraint_builder is less critical
    -- as it's for refining matches already found by the fast matcher.
    ceres_scan_matcher_3d = {
      occupied_space_weight_0 = 20.0,
      occupied_space_weight_1 = 30.0,
      translation_weight = 10.0,
      rotation_weight = 1.0,
      only_optimize_yaw = false,
      ceres_solver_options = {
        use_nonmonotonic_steps = true,
        max_num_iterations = 10,
        num_threads = 1,
      },
    },
  },

  optimization_problem = {
    huber_scale = 5e1,
    acceleration_weight = 1e1,
    rotation_weight = 1e2,
    log_solver_summary = false,
    use_online_imu_extrinsics_in_3d = false,
  }
}

TRAJECTORY_BUILDER.pure_localization_trimmer = {
  max_submaps_to_keep = 4,
}

return options
