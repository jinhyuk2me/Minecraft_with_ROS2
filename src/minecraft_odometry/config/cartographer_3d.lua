-- Cartographer 3D configuration for Minecraft with Point Cloud
-- PURE MAPPING MODE: Only generates maps, does NOT publish localization TF transforms
include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link", 
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = false,  -- CRITICAL: Don't publish odom frame TF
  publish_frame_projected_to_2d = false,
  use_pose_extrapolator = true,  -- Enable pose extrapolation for smoother tracking
  use_odometry = true,  -- Use our ground truth odometry for mapping ONLY
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 0,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 1,
  lookup_transform_timeout_sec = 1.0,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.0,
  odometry_sampling_ratio = 1.0,
  fixed_frame_pose_sampling_ratio = 1.0,
  imu_sampling_ratio = 1.0,
  landmarks_sampling_ratio = 1.0,

  -- CRITICAL: TF 퍼블리시를 완전히 비활성화하는 설정
  publish_to_tf = false,
}

-- Use 3D SLAM with point clouds
MAP_BUILDER.use_trajectory_builder_3d = true

-- 3D trajectory builder settings for Minecraft world
TRAJECTORY_BUILDER_3D.min_range = 0.5
TRAJECTORY_BUILDER_3D.max_range = 50.0
TRAJECTORY_BUILDER_3D.num_accumulated_range_data = 1
TRAJECTORY_BUILDER_3D.voxel_filter_size = 0.15

-- High resolution submap for detailed mapping
TRAJECTORY_BUILDER_3D.high_resolution_adaptive_voxel_filter.max_length = 0.9
TRAJECTORY_BUILDER_3D.high_resolution_adaptive_voxel_filter.min_num_points = 150
TRAJECTORY_BUILDER_3D.high_resolution_adaptive_voxel_filter.max_range = 20.

-- Low resolution submap for wide area coverage
TRAJECTORY_BUILDER_3D.low_resolution_adaptive_voxel_filter.max_length = 4.
TRAJECTORY_BUILDER_3D.low_resolution_adaptive_voxel_filter.min_num_points = 200
TRAJECTORY_BUILDER_3D.low_resolution_adaptive_voxel_filter.max_range = 50.

-- IMU settings (relaxed for static/slow movement)
TRAJECTORY_BUILDER_3D.imu_gravity_time_constant = 10.

-- Motion filter adjusted for corridor navigation
TRAJECTORY_BUILDER_3D.motion_filter.max_time_seconds = 5.
TRAJECTORY_BUILDER_3D.motion_filter.max_distance_meters = 0.2
TRAJECTORY_BUILDER_3D.motion_filter.max_angle_radians = math.rad(1.)

-- Enable SLAM with loop closure for better corridor mapping (MAPPING ONLY)
MAP_BUILDER.use_trajectory_builder_3d = true
POSE_GRAPH.optimize_every_n_nodes = 90  -- Enable pose graph optimization every 90 nodes
POSE_GRAPH.constraint_builder.sampling_ratio = 0.003  -- Enable loop closure constraints  
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.55  -- Enable global localization

-- PURE MAPPING MODE: 잘못된 파라미터 제거됨 (위의 provide_odom_frame = false로 충분)

-- Submap parameters for Minecraft environment
TRAJECTORY_BUILDER_3D.submaps.high_resolution = 0.1
TRAJECTORY_BUILDER_3D.submaps.low_resolution = 0.45
TRAJECTORY_BUILDER_3D.submaps.num_range_data = 100
TRAJECTORY_BUILDER_3D.submaps.range_data_inserter.hit_probability = 0.55
TRAJECTORY_BUILDER_3D.submaps.range_data_inserter.miss_probability = 0.49

return options