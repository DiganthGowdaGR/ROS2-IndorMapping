-- ============================================================
-- CARTOGRAPHER 2D SLAM CONFIGURATION (ROS2)
-- Project: Indoor Mapping Robot
-- Description:
-- This configuration defines how Cartographer performs
-- 2D SLAM using LiDAR and odometry.
-- ============================================================

-- Load core Cartographer modules
include "map_builder.lua"
include "trajectory_builder.lua"

-- ============================================================
-- GLOBAL OPTIONS
-- ============================================================

options = {

  -- Core SLAM builders
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,

  -- ========================================================
  -- COORDINATE FRAMES (TF SYSTEM)
  -- ========================================================
  map_frame = "map",                 -- Global reference frame (final map)
  tracking_frame = "robot_footprint", -- Frame used for SLAM tracking
  published_frame = "odom",          -- Frame published to rest of system
  odom_frame = "odom",               -- Odometry reference frame

  -- Frame behavior
  provide_odom_frame = false,        -- Do NOT generate odom internally
  publish_frame_projected_to_2d = true, -- Flatten pose to 2D (ignore Z)

  -- ========================================================
  -- SENSOR CONFIGURATION
  -- ========================================================
  use_odometry = true,               -- Use odometry data
  use_nav_sat = false,               -- No GPS
  use_landmarks = false,             -- No external landmarks

  -- Laser scan settings
  num_laser_scans = 1,               -- One LiDAR sensor
  num_multi_echo_laser_scans = 0,    -- No multi-echo scans
  num_subdivisions_per_laser_scan = 1, -- No scan splitting
  num_point_clouds = 0,              -- No 3D point clouds

  -- ========================================================
  -- TIMING AND PERFORMANCE
  -- ========================================================
  lookup_transform_timeout_sec = 0.2, -- TF lookup timeout
  submap_publish_period_sec = 0.3,    -- Map update rate
  pose_publish_period_sec = 5e-3,     -- Pose update frequency
  trajectory_publish_period_sec = 30e-3, -- Trajectory update

  -- Sampling ratios (1.0 = use all data)
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

-- ============================================================
-- 2D SLAM MODE
-- ============================================================

MAP_BUILDER.use_trajectory_builder_2d = true

-- ============================================================
-- LIDAR (SCAN) PARAMETERS
-- ============================================================

TRAJECTORY_BUILDER_2D.min_range = 0.1     -- Minimum valid distance
TRAJECTORY_BUILDER_2D.max_range = 8.0     -- Maximum sensing distance
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5. -- For unknown space

-- IMU usage
TRAJECTORY_BUILDER_2D.use_imu_data = false -- No IMU used

-- Scan matching (VERY IMPORTANT)
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true

-- ============================================================
-- MOTION FILTER (REDUCE NOISE)
-- ============================================================

TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.05
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.05)
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 0.2

-- Meaning:
-- Ignore small movements to reduce unnecessary computation

-- ============================================================
-- POSE GRAPH OPTIMIZATION
-- ============================================================

POSE_GRAPH.optimize_every_n_nodes = 60

-- Loop closure quality thresholds
POSE_GRAPH.constraint_builder.min_score = 0.62
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.66

-- Meaning:
-- Controls how confidently scans are matched
-- Higher = stricter matching

-- ============================================================
-- FINAL RETURN
-- ============================================================

return options