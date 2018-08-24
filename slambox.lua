-- Copyright 2016 The Cartographer Authors
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--      http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
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
  provide_odom_frame = false,
  use_odometry = false,
  use_nav_sat = false,
  num_laser_scans = 0,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 2,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 1e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,

  publish_frame_projected_to_2d = false,
  use_pose_extrapolator = false,
  use_landmarks = false,
  landmarks_sampling_ratio = 1.,
  
}

-- BACKPACK_3D

MAP_BUILDER.use_trajectory_builder_3d = true
MAP_BUILDER.num_background_threads = 7
-- POSE_GRAPH.optimization_problem.huber_scale = 5e2
-- POSE_GRAPH.optimize_every_n_nodes = 320
-- POSE_GRAPH.constraint_builder.sampling_ratio = 0.03
-- POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 10
-- POSE_GRAPH.constraint_builder.min_score = 0.62
-- POSE_GRAPH.constraint_builder.global_localization_min_score = 0.66



-- -- LOCAL
-- MAP_BUILDER.use_trajectory_builder_3d = true
-- TRAJECTORY_BUILDER_3D.num_accumulated_range_data = 2
TRAJECTORY_BUILDER_3D.submaps.num_range_data = 80
TRAJECTORY_BUILDER_3D.num_accumulated_range_data = 2
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.translation_weight = 10
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.rotation_weight = 50
-- TRAJECTORY_BUILDER_3D.min_range = 2.0
-- TRAJECTORY_BUILDER_3D.submaps.range_data_inserter.hit_probability = 0.55
-- TRAJECTORY_BUILDER_3D.submaps.range_data_inserter.miss_probability = 0.49
-- TRAJECTORY_BUILDER_3D.submaps.range_data_inserter.num_free_space_voxels = 10
-- TRAJECTORY_BUILDER_3D.high_resolution_adaptive_voxel_filter.max_length = 5.
-- TRAJECTORY_BUILDER_3D.high_resolution_adaptive_voxel_filter.min_num_points = 300.
-- TRAJECTORY_BUILDER_3D.low_resolution_adaptive_voxel_filter.max_length = 8.
-- TRAJECTORY_BUILDER_3D.low_resolution_adaptive_voxel_filter.min_num_points = 500.




-- -- GLOBAL
POSE_GRAPH.optimization_problem.huber_scale = 5
POSE_GRAPH.optimize_every_n_nodes = 160
POSE_GRAPH.optimization_problem.acceleration_weight = 20.0
POSE_GRAPH.optimization_problem.rotation_weight = 20.0
POSE_GRAPH.constraint_builder.sampling_ratio = 0.03
POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 100
POSE_GRAPH.constraint_builder.min_score = 0.62
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.66
POSE_GRAPH.log_residual_histograms = true

-- POSE_GRAPH.optimization_problem.use_online_imu_extrinsics_in_3d = false

POSE_GRAPH.optimization_problem.log_solver_summary = true
-- POSE_GRAPH.optimize_every_n_nodes = 50
-- -- POSE_GRAPH.constraint_builder.min_score = 0.6
-- POSE_GRAPH.optimization_problem.huber_scale = 5e2

-- POSE_GRAPH.constraint_builder.sampling_ratio = 1.0
-- POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 100
-- POSE_GRAPH.constraint_builder.max_constraint_distance = 1.
-- POSE_GRAPH.constraint_builder.min_score = 0.25


-- -- POSE_GRAPH.constraint_builder.min_score = 0.55
-- POSE_GRAPH.constraint_builder.global_localization_min_score = 0.6
-- POSE_GRAPH.constraint_builder.loop_closure_translation_weight = 1.1e4
-- POSE_GRAPH.constraint_builder.loop_closure_rotation_weight = 1e3



return options
