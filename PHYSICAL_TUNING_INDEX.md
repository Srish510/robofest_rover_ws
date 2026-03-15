# Physical Tuning Parameter Index

This file lists variables/parameters that depend on rover dimensions, sensor mounting, environment, traction, or field conditions.

## 1) Robot Geometry & Kinematics

| File:Line | Parameter | Why it depends on physical setup |
|---|---|---|
| `src/rover_control/rover_control/esp32_bridge.py:33` | `wheelbase_m` | Must match axle spacing for correct steering/differential mapping |
| `src/rover_control/rover_control/esp32_bridge.py:34` | `track_width_m` | Must match left-right wheel spacing for turn-rate mapping |
| `src/rover_control/rover_control/esp32_bridge.py:35` | `max_motor_speed` | Normalization cap depends on motor + gearing + battery |
| `src/rover_control/rover_control/esp32_bridge.py:36` | `max_servo_angle_rad` | Steering hardware limit |
| `src/rover_control/rover_control/esp32_bridge.py:38` | `drive_mode` | Mechanical drive strategy (`differential` vs `servo`) |
| `src/rover_bringup/config/rover_params.yaml:66` | `wheelbase_m` | Robot geometry |
| `src/rover_bringup/config/rover_params.yaml:67` | `track_width_m` | Robot geometry |
| `src/rover_bringup/config/rover_params.yaml:68` | `max_motor_speed` | Motor capability / drivetrain |
| `src/rover_bringup/config/rover_params.yaml:69` | `max_servo_angle_rad` | Steering hardware constraint |
| `src/rover_bringup/launch/navigation.launch.py:139` | `wheelbase_m` | Launch override of geometry |
| `src/rover_bringup/launch/navigation.launch.py:140` | `track_width_m` | Launch override of geometry |
| `src/rover_bringup/launch/rover.launch.py:143` | `wheelbase_m` | Launch override of geometry |
| `src/rover_bringup/launch/rover.launch.py:144` | `track_width_m` | Launch override of geometry |
| `src/rover_bringup/launch/sim_test.launch.py:41` | `wheel_base_m` | Simulated rover geometry |
| `src/rover_bringup/launch/sim_test.launch.py:42` | `track_width_m` | Simulated rover geometry |

## 2) Motion, Safety Distances, and Control Gains

| File:Line | Parameter | Why it depends on physical setup |
|---|---|---|
| `src/rover_control/rover_control/motor_interface.py:25` | `max_linear_speed` | Safe top speed depends on traction + space |
| `src/rover_control/rover_control/motor_interface.py:26` | `max_angular_speed` | Safe turn rate depends on wheelbase/track + floor grip |
| `src/rover_control/rover_control/motor_interface.py:27` | `lane_kp` | Lane controller gain depends on dynamics and camera geometry |
| `src/rover_control/rover_control/motor_interface.py:28` | `lane_kd` | Derivative damping depends on speed/noise |
| `src/rover_control/rover_control/motor_interface.py:29` | `heading_kp` | Heading correction gain depends on chassis response |
| `src/rover_control/rover_control/motor_interface.py:30` | `obstacle_stop_distance` | Must reflect stopping distance + sensor latency |
| `src/rover_control/rover_control/motor_interface.py:31` | `obstacle_slow_distance` | Environment-dependent caution zone |
| `src/rover_control/rover_control/motor_interface.py:32` | `obstacle_avoidance_angular` | Avoidance aggressiveness by space/traction |
| `src/rover_control/rover_control/motor_interface.py:33` | `lane_confidence_threshold` | Depends on lighting and camera quality |
| `src/rover_control/rover_control/odometry_node.py:25` | `speed_scale` | Calibrates commanded vs real speed |
| `src/rover_navigation/rover_navigation/goal_manager.py:42` | `default_forward_speed` | Cruise speed by terrain and corridor width |
| `src/rover_navigation/rover_navigation/goal_manager.py:43` | `checkpoint_approach_speed` | Slow-down speed near checkpoint |
| `src/rover_navigation/rover_navigation/goal_manager.py:45` | `no_lane_timeout_sec` | Lane-loss handling depends on camera/frame rate |
| `src/rover_navigation/rover_navigation/goal_manager.py:46` | `obstacle_critical_dist` | Distance for behavior state transition |
| `src/rover_navigation/rover_navigation/goal_manager.py:51` | `nav2_auto_waypoints_xy` | Route geometry depends on room/track dimensions |
| `src/rover_navigation/rover_navigation/goal_manager.py:58` | `nav2_loop_waypoints` | Mission repetition policy |
| `src/rover_navigation/rover_navigation/goal_manager.py:59` | `nav2_start_delay_sec` | Startup delay for sensor/map readiness |
| `src/rover_bringup/config/rover_params.yaml:76` | `max_linear_speed` | Same as above, config source |
| `src/rover_bringup/config/rover_params.yaml:77` | `max_angular_speed` | Same as above, config source |
| `src/rover_bringup/config/rover_params.yaml:78` | `lane_kp` | Same as above, config source |
| `src/rover_bringup/config/rover_params.yaml:79` | `lane_kd` | Same as above, config source |
| `src/rover_bringup/config/rover_params.yaml:80` | `heading_kp` | Same as above, config source |
| `src/rover_bringup/config/rover_params.yaml:81` | `obstacle_stop_distance` | Same as above, config source |
| `src/rover_bringup/config/rover_params.yaml:82` | `obstacle_slow_distance` | Same as above, config source |
| `src/rover_bringup/config/rover_params.yaml:83` | `obstacle_avoidance_angular` | Same as above, config source |
| `src/rover_bringup/launch/navigation.launch.py:171` | `max_linear_speed` | Launch-time indoor tuning override |
| `src/rover_bringup/launch/navigation.launch.py:172` | `max_angular_speed` | Launch-time indoor tuning override |
| `src/rover_bringup/launch/navigation.launch.py:173` | `obstacle_stop_distance` | Launch-time safety override |
| `src/rover_bringup/launch/navigation.launch.py:174` | `obstacle_slow_distance` | Launch-time safety override |
| `src/rover_bringup/launch/navigation.launch.py:267` | `nav2_auto_waypoints_xy` | Launch-time route geometry override |
| `src/rover_bringup/launch/rover.launch.py:158` | `max_linear_speed` | Launch-time legacy tuning |
| `src/rover_bringup/launch/rover.launch.py:159` | `max_angular_speed` | Launch-time legacy tuning |
| `src/rover_bringup/launch/rover.launch.py:160` | `lane_kp` | Launch-time legacy tuning |
| `src/rover_bringup/launch/rover.launch.py:161` | `lane_kd` | Launch-time legacy tuning |
| `src/rover_bringup/launch/rover.launch.py:163` | `obstacle_stop_distance` | Launch-time legacy safety |
| `src/rover_bringup/launch/rover.launch.py:164` | `obstacle_slow_distance` | Launch-time legacy safety |

## 3) Camera Mounting, Perception, and Environment Sensitivity

| File:Line | Parameter | Why it depends on physical setup |
|---|---|---|
| `src/rover_perception/rover_perception/lane_detector.py:16` | `yellow_h_low` | Lane color threshold varies with paint/lighting |
| `src/rover_perception/rover_perception/lane_detector.py:17` | `yellow_h_high` | Lane color threshold varies with paint/lighting |
| `src/rover_perception/rover_perception/lane_detector.py:18` | `yellow_s_low` | Lighting and camera auto-exposure sensitivity |
| `src/rover_perception/rover_perception/lane_detector.py:19` | `yellow_s_high` | Lighting and camera auto-exposure sensitivity |
| `src/rover_perception/rover_perception/lane_detector.py:20` | `yellow_v_low` | Brightness threshold by scene illumination |
| `src/rover_perception/rover_perception/lane_detector.py:21` | `yellow_v_high` | Brightness threshold by scene illumination |
| `src/rover_perception/rover_perception/lane_detector.py:25` | `camera_height_m` | Must match physical camera mount height |
| `src/rover_perception/rover_perception/lane_detector.py:26` | `camera_pitch_rad` | Must match camera tilt angle |
| `src/rover_perception/rover_perception/obstacle_detector.py:17` | `min_depth_m` | Sensor blind-zone lower bound |
| `src/rover_perception/rover_perception/obstacle_detector.py:18` | `max_depth_m` | Sensor effective range upper bound |
| `src/rover_perception/rover_perception/obstacle_detector.py:19` | `obstacle_height_threshold_m` | Ground/obstacle separation threshold |
| `src/rover_perception/rover_perception/obstacle_detector.py:20` | `ground_plane_tolerance_m` | Surface roughness tolerance |
| `src/rover_perception/rover_perception/obstacle_detector.py:22` | `min_cluster_points` | Noise vs detection sensitivity |
| `src/rover_perception/rover_perception/obstacle_detector.py:24` | `velocity_threshold_m_s` | Dynamic obstacle classification threshold |
| `src/rover_perception/rover_perception/obstacle_detector.py:25` | `depth_scale` | Camera-specific depth unit conversion |
| `src/rover_perception/rover_perception/depth_processor.py:16` | `depth_scale` | Camera-specific depth unit conversion |
| `src/rover_perception/rover_perception/depth_processor.py:17` | `min_depth_m` | Range filtering lower bound |
| `src/rover_perception/rover_perception/depth_processor.py:18` | `max_depth_m` | Range filtering upper bound |
| `src/rover_perception/rover_perception/depth_processor.py:23` | `obstacle_height_min_m` | Ground separation threshold |
| `src/rover_perception/rover_perception/depth_processor.py:24` | `obstacle_height_max_m` | Tall-object cutoff |
| `src/rover_perception/rover_perception/depth_processor.py:25` | `camera_height_m` | Must match sensor mounting |
| `src/rover_bringup/config/rover_params.yaml:28` | `camera_height_m` | Config override for lane projection |
| `src/rover_bringup/config/rover_params.yaml:29` | `camera_pitch_rad` | Config override for lane projection |
| `src/rover_bringup/config/rover_params.yaml:33` | `min_depth_m` | Config override for obstacle detector |
| `src/rover_bringup/config/rover_params.yaml:34` | `max_depth_m` | Config override for obstacle detector |
| `src/rover_bringup/config/rover_params.yaml:35` | `obstacle_height_threshold_m` | Config override for obstacle detector |
| `src/rover_bringup/config/rover_params.yaml:39` | `depth_scale` | Config override for camera scale |
| `src/rover_bringup/config/rover_params.yaml:44` | `min_depth_m` | Config override for depth processor |
| `src/rover_bringup/config/rover_params.yaml:45` | `max_depth_m` | Config override for depth processor |
| `src/rover_bringup/config/rover_params.yaml:52` | `camera_height_m` | Config override for depth processor |

## 4) Costmaps, Map Size, and Terrain Classification

| File:Line | Parameter | Why it depends on physical setup |
|---|---|---|
| `src/rover_navigation/rover_navigation/lane_costmap_layer.py:19` | `costmap_resolution` | Tradeoff between detail and compute |
| `src/rover_navigation/rover_navigation/lane_costmap_layer.py:20` | `costmap_width_m` | Local planning horizon width |
| `src/rover_navigation/rover_navigation/lane_costmap_layer.py:21` | `costmap_height_m` | Local planning horizon length |
| `src/rover_navigation/rover_navigation/lane_costmap_layer.py:22` | `lane_boundary_cost` | Lane boundary penalty aggressiveness |
| `src/rover_navigation/rover_navigation/lane_costmap_layer.py:23` | `outside_lane_cost` | Out-of-lane penalty aggressiveness |
| `src/rover_navigation/rover_navigation/lane_costmap_layer.py:25` | `boundary_width_cells` | Effective lane boundary thickness |
| `src/rover_navigation/rover_navigation/lane_costmap_layer.py:26` | `camera_height_m` | Required for image-to-ground projection |
| `src/rover_navigation/rover_navigation/planner_helper.py:21` | `sector_threshold` | VFH obstacle blocking sensitivity |
| `src/rover_navigation/rover_navigation/planner_helper.py:22` | `max_obstacle_range` | Reactive planner sensing horizon |
| `src/rover_navigation/rover_navigation/planner_helper.py:23` | `safety_margin_m` | Conservative clearance margin |
| `src/rover_perception/rover_perception/depth_processor.py:19` | `voxel_size_m` | Point-cloud sparsity/compute tradeoff |
| `src/rover_perception/rover_perception/depth_processor.py:20` | `map_resolution_m` | Occupancy map cell size |
| `src/rover_perception/rover_perception/depth_processor.py:21` | `map_width_m` | Local map width extent |
| `src/rover_perception/rover_perception/depth_processor.py:22` | `map_height_m` | Local map height extent |
| `src/rover_perception/rover_perception/terrain_mapper.py:16` | `cell_resolution_m` | Terrain grid detail |
| `src/rover_perception/rover_perception/terrain_mapper.py:17` | `map_size_m` | Terrain map extent |
| `src/rover_perception/rover_perception/terrain_mapper.py:18` | `slope_threshold_rad` | Slope classification threshold |
| `src/rover_perception/rover_perception/terrain_mapper.py:19` | `rough_threshold_m` | Roughness classification threshold |
| `src/rover_perception/rover_perception/terrain_mapper.py:20` | `obstacle_height_min_m` | Terrain obstacle min height |
| `src/rover_perception/rover_perception/terrain_mapper.py:21` | `obstacle_height_max_m` | Terrain obstacle max height |
| `src/rover_perception/rover_perception/terrain_mapper.py:22` | `step_height_m` | Step detection threshold |
| `src/rover_perception/rover_perception/terrain_mapper.py:25` | `max_range_m` | Terrain mapping sensing horizon |
| `src/rover_bringup/config/nav2_params.yaml:89` | `xy_goal_tolerance` | Practical stop precision |
| `src/rover_bringup/config/nav2_params.yaml:90` | `yaw_goal_tolerance` | Practical orientation precision |
| `src/rover_bringup/config/nav2_params.yaml:98` | `max_vel_x` | Nav2 local planner speed cap |
| `src/rover_bringup/config/nav2_params.yaml:100` | `max_vel_theta` | Nav2 turn-rate cap |
| `src/rover_bringup/config/nav2_params.yaml:104` | `acc_lim_x` | Acceleration capability constraint |
| `src/rover_bringup/config/nav2_params.yaml:106` | `acc_lim_theta` | Angular acceleration constraint |
| `src/rover_bringup/config/nav2_params.yaml:180` | `width` (local costmap) | Local planning window size |
| `src/rover_bringup/config/nav2_params.yaml:181` | `height` (local costmap) | Local planning window size |
| `src/rover_bringup/config/nav2_params.yaml:182` | `resolution` (local costmap) | Costmap granularity |
| `src/rover_bringup/config/nav2_params.yaml:183` | `robot_radius` (local) | Collision footprint approximation |
| `src/rover_bringup/config/nav2_params.yaml:197` | `max_obstacle_height` | Height filter for obstacles |
| `src/rover_bringup/config/nav2_params.yaml:198` | `min_obstacle_height` | Height filter for obstacles |
| `src/rover_bringup/config/nav2_params.yaml:208` | `obstacle_max_range` | Obstacle detection max range |
| `src/rover_bringup/config/nav2_params.yaml:209` | `obstacle_min_range` | Obstacle detection min range |
| `src/rover_bringup/config/nav2_params.yaml:224` | `cost_scaling_factor` | Inflation aggressiveness |
| `src/rover_bringup/config/nav2_params.yaml:225` | `inflation_radius` | Clearance around obstacles |
| `src/rover_bringup/config/nav2_params.yaml:236` | `robot_radius` (global) | Collision footprint approximation |
| `src/rover_bringup/config/nav2_params.yaml:237` | `resolution` (global costmap) | Global map granularity |
| `src/rover_bringup/config/nav2_params.yaml:263` | `obstacle_max_range` (global) | Global obstacle update range |
| `src/rover_bringup/config/nav2_params.yaml:264` | `obstacle_min_range` (global) | Global obstacle update near cutoff |
| `src/rover_bringup/config/nav2_params.yaml:270` | `cost_scaling_factor` (global) | Global inflation aggressiveness |
| `src/rover_bringup/config/nav2_params.yaml:271` | `inflation_radius` (global) | Global clearance |
| `src/rover_bringup/config/slam.yaml:7` | `cell_resolution_m` | Terrain-map granularity |
| `src/rover_bringup/config/slam.yaml:11` | `slope_threshold_rad` | Terrain slope threshold |
| `src/rover_bringup/config/slam.yaml:13` | `rough_threshold_m` | Terrain roughness threshold |
| `src/rover_bringup/config/slam.yaml:15` | `obstacle_height_min_m` | Terrain obstacle minimum |
| `src/rover_bringup/config/slam.yaml:16` | `obstacle_height_max_m` | Terrain obstacle maximum |
| `src/rover_bringup/config/rtabmap.yaml:44` | `Grid/MaxObstacleHeight` | SLAM grid obstacle filtering |
| `src/rover_bringup/config/rtabmap.yaml:86` | `Icp/VoxelSize` | Registration voxel/downsampling scale |

## 5) Simulation-Specific Physical Condition Parameters

| File:Line | Parameter | Why it depends on physical setup |
|---|---|---|
| `src/rover_perception/rover_perception/mock_camera.py:25` | `lane_sway_amplitude` | Simulates lane curvature/intensity |
| `src/rover_perception/rover_perception/mock_camera.py:26` | `lane_sway_period_sec` | Simulates frequency of lane change |
| `src/rover_perception/rover_perception/mock_camera.py:27` | `show_obstacle` | Toggles obstacle presence |
| `src/rover_perception/rover_perception/mock_camera.py:28` | `obstacle_cycle_sec` | Dynamic obstacle timing |
| `src/rover_perception/rover_perception/mock_camera.py:29` | `obstacle_course` | Scenario geometry/profile |
| `src/rover_bringup/launch/sim_test.launch.py:13` | `obstacle_course` launch arg | Selects simulated course layout |

## 6) Notes on Where to Tune First

1. Start with `src/rover_bringup/config/rover_params.yaml` for most field tuning.
2. Then check launch overrides in `src/rover_bringup/launch/navigation.launch.py` and `src/rover_bringup/launch/rover.launch.py` (they can override YAML defaults).
3. For Nav2 behavior, tune `src/rover_bringup/config/nav2_params.yaml` (especially speed limits, inflation, obstacle ranges, and robot radius).
