# Autonomous Rover - Project Documentation

Complete technical documentation for an autonomous ground rover built with ROS 2 (Humble). The system uses an Intel RealSense D435i camera for perception, an ESP32 microcontroller for low-level motor control and IMU, and a Raspberry Pi 4 running ROS 2 as the main compute platform. The rover autonomously follows yellow lane boundaries, avoids obstacles, scans QR checkpoints, and streams telemetry to a Ground Control Station (GCS) over WiFi.

---

## Table of Contents

- [Architecture Overview](#architecture-overview)
- [Workspace Structure](#workspace-structure)
- [Custom Interfaces (`rover_interfaces`)](#custom-interfaces-rover_interfaces)
- [Packages](#packages)
  - [`rover_perception`](#rover_perception)
  - [`rover_control`](#rover_control)
  - [`rover_navigation`](#rover_navigation)
  - [`rover_comm`](#rover_comm)
  - [`rover_bringup`](#rover_bringup)
- [Launch Files](#launch-files)
- [Configuration Files](#configuration-files)
- [ROS 2 Topic & Service Map](#ros-2-topic--service-map)
- [TF Tree](#tf-tree)

---

## Architecture Overview

```
┌────────────────────────────────────────────────────────────────────┐
│                         Raspberry Pi 4 (ROS 2)                     │
│                                                                    │
│  ┌─────────────┐  ┌──────────────┐  ┌──────────────┐              │
│  │  Perception  │  │   Control    │  │  Navigation  │              │
│  │             │  │              │  │              │              │
│  │ realsense   │─→│ motor_iface  │←─│ goal_manager │              │
│  │ lane_detect │  │ odometry     │  │ lane_costmap │              │
│  │ depth_proc  │  │ esp32_bridge │  │ planner_help │              │
│  │ obstacle_det│  │              │  │              │              │
│  │ terrain_map │  │              │  │              │              │
│  │ qr_scanner  │  │              │  │              │              │
│  └─────────────┘  └──────┬───────┘  └──────────────┘              │
│                          │ UART                                    │
│  ┌───────────────┐       │       ┌───────────────────────┐        │
│  │ Communication │       │       │ SLAM (RTAB-Map)       │        │
│  │ video_stream  │       │       │ rtabmap + visual odom  │        │
│  │ map_stream    │       │       └───────────────────────┘        │
│  │ telemetry     │       │                                        │
│  └───────┬───────┘       │                                        │
│          │ WiFi          │                                        │
└──────────┼───────────────┼────────────────────────────────────────┘
           │               │
     ┌─────▼─────┐   ┌────▼─────┐
     │    GCS    │   │  ESP32   │
     │ (Browser) │   │ Motors   │
     │           │   │ IMU/Batt │
     └───────────┘   └──────────┘
```

**Data flow summary:**
1. **Perception** nodes capture camera frames, detect lanes, obstacles, terrain, and QR codes.
2. **Navigation** uses lane and obstacle data to decide speed and steering via a state machine.
3. **Control** merges navigation commands with lane-keeping PD control and obstacle avoidance, sends 4-wheel drive/steer commands to ESP32 over serial, and publishes odometry.
4. **Communication** streams video (MJPEG), map data (JSON/SSE), and telemetry (JSON/SSE) to a browser-based GCS over HTTP.
5. **SLAM** (RTAB-Map) provides visual odometry, loop closure, and a persistent 3D map.

---

## Workspace Structure

```
rover_ws/
├── src/
│   ├── rover_interfaces/      # Custom ROS 2 message and service definitions
│   │   ├── msg/
│   │   │   ├── Lane.msg
│   │   │   ├── Obstacle.msg
│   │   │   ├── RoverStatus.msg
│   │   │   ├── TerrainCell.msg
│   │   │   ├── TerrainMap.msg
│   │   │   └── TerrainMap3D.msg
│   │   └── srv/
│   │       └── RegisterCheckpoint.srv
│   ├── rover_perception/      # Camera driver, lane/obstacle/terrain/QR detection
│   │   └── rover_perception/
│   │       ├── realsense_node.py
│   │       ├── lane_detector.py
│   │       ├── depth_processor.py
│   │       ├── terrain_mapper.py
│   │       ├── obstacle_detector.py
│   │       ├── qr_scanner.py
│   │       └── mock_camera.py
│   ├── rover_control/         # ESP32 bridge, motor interface, odometry
│   │   └── rover_control/
│   │       ├── esp32_bridge.py
│   │       ├── motor_interface.py
│   │       ├── odometry_node.py
│   │       └── mock_esp32.py
│   ├── rover_navigation/      # Goal management, costmap layers, VFH planner
│   │   └── rover_navigation/
│   │       ├── goal_manager.py
│   │       ├── lane_costmap_layer.py
│   │       └── planner_helper.py
│   ├── rover_comm/            # WiFi streaming - video, maps, telemetry
│   │   └── rover_comm/
│   │       ├── video_stream_node.py
│   │       ├── map_stream_node.py
│   │       └── telemetry_node.py
│   └── rover_bringup/         # Launch files and YAML configuration
│       ├── launch/
│       │   ├── rover.launch.py
│       │   ├── perception_test.launch.py
│       │   ├── sim_test.launch.py
│       │   └── slam_3d.launch.py
│       └── config/
│           ├── rover_params.yaml
│           ├── rtabmap.yaml
│           ├── slam.yaml
│           ├── costmap.yaml
│           └── nav2_params.yaml
├── build/
├── install/
└── log/
```

---

## Custom Interfaces (`rover_interfaces`)

### Messages

#### `Lane.msg`

Lane detection output with boundary points and control errors.

| Field | Type | Description |
|-------|------|-------------|
| `header` | `std_msgs/Header` | Timestamp and frame |
| `left_boundary` | `geometry_msgs/Point[]` | Left lane boundary (image coords) |
| `right_boundary` | `geometry_msgs/Point[]` | Right lane boundary (image coords) |
| `center_line` | `geometry_msgs/Point[]` | Lane center in `base_link` frame (m) |
| `lateral_offset` | `float64` | Offset from lane center (m, +ve = right) |
| `heading_error` | `float64` | Heading error relative to lane (rad) |
| `confidence` | `float64` | Detection confidence [0.0–1.0] |
| `left_detected` | `bool` | Left boundary detected |
| `right_detected` | `bool` | Right boundary detected |

#### `Obstacle.msg`

Per-obstacle detection with 3D position, dynamics, and classification.

| Field | Type | Description |
|-------|------|-------------|
| `header` | `std_msgs/Header` | Timestamp and frame |
| `position` | `geometry_msgs/Point` | 3D position relative to rover (m) |
| `size` | `geometry_msgs/Vector3` | Bounding box dimensions (m) |
| `distance` | `float64` | Distance to obstacle (m) |
| `bearing` | `float64` | Bearing angle (rad, 0 = ahead) |
| `velocity` | `geometry_msgs/Vector3` | Obstacle velocity (m/s) |
| `is_dynamic` | `bool` | Whether obstacle is moving |
| `classification` | `uint8` | `UNKNOWN=0`, `ROCK=1`, `PERSON=2`, `VEHICLE=3`, `OTHER=4` |
| `confidence` | `float64` | Detection confidence [0.0–1.0] |

#### `RoverStatus.msg`

Aggregated rover state for telemetry.

| Field | Type | Description |
|-------|------|-------------|
| `header` | `std_msgs/Header` | Timestamp and frame |
| `servo_angles` | `float32[4]` | Steering servo angles [FL, FR, RL, RR] (rad) |
| `orientation_euler` | `float32[3]` | Roll, pitch, yaw (rad) |
| `linear_acceleration` | `float32[3]` | Accelerometer XYZ (m/s²) |
| `angular_velocity` | `float32[3]` | Gyroscope XYZ (rad/s) |
| `state` | `uint8` | `IDLE=0`, `NAVIGATING=1`, `OBSTACLE_AVOIDANCE=2`, `CHECKPOINT_SCAN=3`, `ERROR=4` |
| `last_checkpoint` | `string` | Last registered checkpoint ID |

#### `TerrainCell.msg`

Single terrain grid cell with full classification data.

| Field | Type | Description |
|-------|------|-------------|
| `x`, `y` | `float32` | Cell center in map frame (m) |
| `elevation` | `float32` | Ground elevation (m) |
| `slope` | `float32` | Slope angle (rad) |
| `roughness` | `float32` | Surface roughness (std dev of height) |
| `terrain_class` | `uint8` | Terrain classification |
| `traversability` | `uint8` | Cost [0–255], 255 = impassable |
| `confidence` | `float32` | Mapping confidence [0.0–1.0] |

#### `TerrainMap.msg`

Local 2D terrain grid from depth processing.

| Field | Type | Description |
|-------|------|-------------|
| `header` | `std_msgs/Header` | Timestamp and frame |
| `resolution` | `float64` | Meters per cell |
| `width`, `height` | `uint32` | Grid dimensions (cells) |
| `origin` | `geometry_msgs/Pose` | Map origin in world frame |
| `terrain_classes` | `uint8[]` | Row-major: `UNKNOWN=0`, `FLAT=1`, `ROUGH=2`, `OBSTACLE=3`, `LANE_BOUNDARY=4` |
| `elevation` | `float32[]` | Elevation grid (m, NaN for unknown) |
| `traversability` | `uint8[]` | Cost grid [0–255] |

#### `TerrainMap3D.msg`

Extended 3D terrain map from SLAM point cloud analysis. Extends `TerrainMap` with slope, roughness, confidence grids, per-cell `TerrainCell` array, and statistics. Adds terrain classes `SLOPE=5` and `STEP=6`.

### Services

#### `RegisterCheckpoint.srv`

Registers a QR checkpoint with the telemetry system.

| Direction | Field | Type | Description |
|-----------|-------|------|-------------|
| Request | `qr_data` | `string` | Decoded QR payload |
| Request | `rover_pose` | `geometry_msgs/Pose` | Rover pose at scan time |
| Request | `timestamp` | `builtin_interfaces/Time` | Detection timestamp |
| Response | `success` | `bool` | Registration success |
| Response | `checkpoint_id` | `string` | Assigned checkpoint ID (e.g. `CP_001`) |
| Response | `message` | `string` | Status message |

---

## Packages

### `rover_perception`

Camera driver and all perception processing nodes. Depends on: `rclpy`, `sensor_msgs`, `geometry_msgs`, `nav_msgs`, `std_msgs`, `cv_bridge`, `rover_interfaces`.

#### 1. `realsense_node`

Publishes camera frames (RGB and Depth) from an Intel RealSense D435i camera, as well as RealSense IMU data.

**Parameters:**

| Parameter | Default | Description |
|-----------|---------|-------------|
| `color_width` | `640` | RGB image width |
| `color_height` | `480` | RGB image height |
| `depth_width` | `640` | Depth image width |
| `depth_height` | `480` | Depth image height |
| `fps` | `30` | Frames per second |
| `enable_imu` | `True` | Whether to publish IMU data |
| `align_depth` | `True` | Whether to align depth to color frame |

**Published Topics:**

| Topic | Type | Description |
|-------|------|-------------|
| `camera/color/image_raw` | `sensor_msgs/Image` | RGB image (`bgr8`) |
| `camera/depth/image_raw` | `sensor_msgs/Image` | Depth image (`16UC1`) |
| `camera/aligned_depth/image_raw` | `sensor_msgs/Image` | Depth aligned to color frame (`16UC1`) |
| `camera/color/camera_info` | `sensor_msgs/CameraInfo` | Color stream intrinsics |
| `camera/depth/camera_info` | `sensor_msgs/CameraInfo` | Depth stream intrinsics |
| `camera/imu` | `sensor_msgs/Imu` | IMU data (if enabled) |

**Notes:**
- Color stream uses `BGR8` for OpenCV compatibility.
- Depth stream uses `16UC1` (millimeters).
- Distortion model is assumed `plumb_bob` — change if using a different lens.

---

#### 2. `lane_detector`

Detects yellow lane boundaries using HSV color filtering and computes lateral offset and heading error for lane-keeping control.

**Parameters:**

| Parameter | Default | Description |
|-----------|---------|-------------|
| `yellow_h_low` | `20` | Yellow hue lower bound (HSV) |
| `yellow_h_high` | `35` | Yellow hue upper bound (HSV) |
| `yellow_s_low` | `80` | Yellow saturation lower bound |
| `yellow_s_high` | `255` | Yellow saturation upper bound |
| `yellow_v_low` | `80` | Yellow value lower bound |
| `yellow_v_high` | `255` | Yellow value upper bound |
| `min_contour_area` | `500` | Minimum contour area for lane marking |
| `roi_top_ratio` | `0.4` | ROI top as image-height ratio (bottom 60% processed) |
| `gaussian_blur_size` | `5` | Gaussian blur kernel (must be odd) |
| `camera_height_m` | `0.3` | Camera height above ground (m) |
| `camera_pitch_rad` | `0.5` | Camera pitch (rad) — reserved for future tuning |

**Published Topics:**

| Topic | Type | Description |
|-------|------|-------------|
| `perception/lane` | `rover_interfaces/msg/Lane` | Lane boundaries, offset, heading error, confidence |
| `perception/lane_debug` | `sensor_msgs/Image` | Debug visualization of detected lanes |

**Subscribed Topics:**

| Topic | Type |
|-------|------|
| `camera/color/image_raw` | `sensor_msgs/Image` |
| `camera/color/camera_info` | `sensor_msgs/CameraInfo` |

**Algorithm:**
1. Crop image to bottom ROI using `roi_top_ratio`.
2. Convert ROI from BGR to HSV and threshold yellow lane pixels.
3. Denoise mask with Gaussian blur and morphology (close + open).
4. Extract contours, reject small regions (`min_contour_area`), split points into left/right by image midpoint.
5. Fit one line per side (`cv2.fitLine`) when enough points exist.
6. Both sides detected → compute lane center, lateral offset, heading error (confidence 0.9).
7. One side → heading only (confidence 0.5). None → confidence 0.0.
8. Publish `Lane` message and debug visualization.

---

#### 3. `depth_processor`

Processes RealSense depth data into point clouds and terrain maps for navigation and mapping. Publishes SLAM-compatible XYZRGB output for RTAB-Map integration.

**Parameters:**

| Parameter | Default | Description |
|-----------|---------|-------------|
| `depth_scale` | `0.001` | Raw depth → meters conversion |
| `min_depth_m` | `0.3` | Minimum valid depth (m) |
| `max_depth_m` | `5.0` | Maximum valid depth (m) |
| `voxel_size_m` | `0.05` | Voxel grid downsampling size (m) |
| `map_resolution_m` | `0.05` | Terrain/occupancy grid cell size (m) |
| `map_width_m` | `10.0` | Local map width (m) |
| `map_height_m` | `10.0` | Local map height (m) |
| `obstacle_height_min_m` | `0.10` | Obstacle lower height threshold (m) |
| `obstacle_height_max_m` | `2.0` | Obstacle upper height threshold (m) |
| `camera_height_m` | `0.3` | Camera height above ground (m) |
| `enable_slam_output` | `True` | Publish RTAB-Map XYZRGB cloud |

**Published Topics:**

| Topic | Type | Description |
|-------|------|-------------|
| `perception/point_cloud` | `PointCloud2` | Downsampled XYZ cloud in `base_link` |
| `perception/terrain_map` | `rover_interfaces/TerrainMap` | Classified local terrain grid |
| `perception/local_costmap` | `OccupancyGrid` | Nav2-compatible occupancy map |
| `perception/slam_point_cloud` | `PointCloud2` | XYZRGB cloud in camera frame for RTAB-Map |

**Subscribed Topics:**

| Topic | Type |
|-------|------|
| `camera/aligned_depth/image_raw` | `sensor_msgs/Image` |
| `camera/color/camera_info` | `sensor_msgs/CameraInfo` |
| `camera/color/image_raw` | `sensor_msgs/Image` |

**Algorithm:**
1. Convert aligned depth to meters, filter to `[min_depth_m, max_depth_m]`, project pixels into 3D camera coordinates using pinhole model.
2. Downsample with voxel grid → publish `perception/point_cloud`.
3. Build subsampled XYZRGB cloud (every 4th pixel) → publish `perception/slam_point_cloud`.
4. Transform to `base_link` ground plane, bin into local grid, classify cells as flat/rough/obstacle.
5. Publish `TerrainMap` and convert traversability to `OccupancyGrid`.

---

#### 4. `terrain_mapper`

3D terrain mapper — ingests RTAB-Map global and local point clouds, accumulates observations in a persistent 2D grid, and publishes terrain classification plus an occupancy grid.

**Parameters:**

| Parameter | Default | Description |
|-----------|---------|-------------|
| `cell_resolution_m` | `0.10` | Grid cell size (m) |
| `map_size_m` | `30.0` | Maximum square map size (m) |
| `slope_threshold_rad` | `0.35` | Slope classification threshold (~20°) |
| `rough_threshold_m` | `0.05` | Roughness classification threshold (m) |
| `obstacle_height_min_m` | `0.15` | Obstacle min height (m) |
| `obstacle_height_max_m` | `2.0` | Obstacle max height (m) |
| `step_height_m` | `0.08` | Step detection threshold (m) |
| `min_points_per_cell` | `3` | Min observations before classifying |
| `update_rate_hz` | `2.0` | Terrain publish rate (Hz) |
| `max_range_m` | `10.0` | Max XY distance for points (m) |

**Published Topics:**

| Topic | Type | Description |
|-------|------|-------------|
| `perception/terrain_map_3d` | `rover_interfaces/TerrainMap3D` | Full 3D terrain grid |
| `slam/occupancy_grid` | `OccupancyGrid` | Nav2-compatible occupancy map |

**Subscribed Topics:**

| Topic | Type | Description |
|-------|------|-------------|
| `rtabmap/cloud_map` | `PointCloud2` | Global cloud (triggers full grid rebuild) |
| `perception/point_cloud` | `PointCloud2` | Local incremental updates |

**Algorithm:**
1. Parse `PointCloud2` into XYZ, filter NaN/Inf.
2. Filter by `max_range_m`, discretize to grid cells, accumulate Z samples.
3. On timer tick: compute bounded extents, crop to `map_size_m`.
4. Per cell (≥ `min_points_per_cell`): compute elevation (median), roughness (std), slope (neighbor gradients), confidence.
5. Classify: obstacle > step > slope > rough > flat. Assign traversability cost.
6. Publish `TerrainMap3D` in `map` frame and `OccupancyGrid`.

---

#### 5. `obstacle_detector`

Detects obstacles from aligned depth images, estimates 3D geometry, and flags dynamic obstacles using frame-to-frame centroid tracking.

**Parameters:**

| Parameter | Default | Description |
|-----------|---------|-------------|
| `min_depth_m` | `0.3` | Min valid depth (m) |
| `max_depth_m` | `5.0` | Max valid depth (m) |
| `obstacle_height_threshold_m` | `0.10` | Height above ground for obstacle (m) |
| `ground_plane_tolerance_m` | `0.05` | Ground estimation tolerance (m) |
| `cluster_distance_m` | `0.15` | Cluster distance (reserved) |
| `min_cluster_points` | `50` | Min contour area for obstacle |
| `dynamic_tracking_window` | `5` | Tracking window (reserved) |
| `velocity_threshold_m_s` | `0.1` | Speed threshold for dynamic flag (m/s) |
| `depth_scale` | `0.001` | Raw depth → meters |

**Published Topics:**

| Topic | Type | Description |
|-------|------|-------------|
| `perception/obstacles` | `rover_interfaces/Obstacle` | Per-obstacle detection messages |
| `perception/obstacle_debug` | `sensor_msgs/Image` | Debug visualization (`bgr8`) |

**Subscribed Topics:**

| Topic | Type |
|-------|------|
| `camera/aligned_depth/image_raw` | `sensor_msgs/Image` |
| `camera/color/camera_info` | `sensor_msgs/CameraInfo` |

**Algorithm:**
1. Convert depth to meters, filter range, back-project to organized 3D cloud.
2. Estimate ground plane from median Y of far points; mark above-ground points as obstacle candidates.
3. Morphological cleanup, extract contours.
4. Per region: compute 3D centroid (median), extents, range, bearing.
5. Match centroids with previous frame; estimate velocity, flag dynamic if speed > threshold.
6. Publish one `Obstacle` message per detection. Debug image: green = static, red = dynamic.

---

#### 6. `qr_scanner`

Detects QR checkpoints from RGB camera frames using `pyzbar`, deduplicates repeated scans, and registers checkpoints via service call.

**Parameters:**

| Parameter | Default | Description |
|-----------|---------|-------------|
| `scan_interval_sec` | `0.5` | Min time between decode attempts |
| `min_qr_size` | `50` | Min QR bounding-box size (px) |
| `deduplicate_timeout_sec` | `30.0` | Deduplication window (s) |

**Published Topics:**

| Topic | Type | Description |
|-------|------|-------------|
| `perception/qr_detected` | `std_msgs/String` | Decoded QR payload |
| `perception/qr_debug` | `sensor_msgs/Image` | Debug frame with QR overlays |

**Subscribed Topics:**

| Topic | Type |
|-------|------|
| `camera/color/image_raw` | `sensor_msgs/Image` |
| `rover/pose` | `geometry_msgs/PoseStamped` |

**Service Client:** `register_checkpoint` (`rover_interfaces/srv/RegisterCheckpoint`)

**Algorithm:**
1. Rate-limit scans. Decode QR codes with `pyzbar`.
2. Filter: QRCODE type only, valid payload, min size check.
3. Deduplicate within `deduplicate_timeout_sec`.
4. New QR → publish to `perception/qr_detected`, call `register_checkpoint` service with QR data, timestamp, and current pose.
5. Publish annotated debug frame.

---

#### 7. `mock_camera`

Generates synthetic color + depth images with yellow lanes, a moving obstacle, and periodic QR-like patterns. Replaces `realsense_node` for testing without hardware.

**Parameters:**

| Parameter | Default | Description |
|-----------|---------|-------------|
| `width` | `640` | Image width |
| `height` | `480` | Image height |
| `fps` | `30` | Frame rate |
| `lane_sway_amplitude` | `40.0` | Lane sway amplitude (px) |
| `lane_sway_period_sec` | `8.0` | Lane sway period (s) |
| `show_obstacle` | `True` | Render moving obstacle |
| `obstacle_cycle_sec` | `6.0` | Obstacle motion cycle (s) |
| `show_qr` | `True` | Render QR placeholders |
| `qr_interval_sec` | `15.0` | QR appearance interval (s) |
| `qr_display_duration_sec` | `3.0` | QR display duration (s) |

**Publishes:** Same topics as `realsense_node` (`camera/color/image_raw`, `camera/depth/image_raw`, `camera/aligned_depth/image_raw`, `camera/color/camera_info`, `camera/depth/camera_info`).

---

### `rover_control`

ESP32 hardware bridge, high-level motor controller, and odometry. Depends on: `rclpy`, `geometry_msgs`, `nav_msgs`, `std_msgs`, `tf2_ros`, `rover_interfaces`.

#### 1. `esp32_bridge`

Bidirectional serial bridge between RPi4 (ROS 2) and ESP32 (low-level motor control, external IMU).

**Serial Protocol (JSON over UART):**
- **TX → ESP32:** `{"cmd": "motor", "fl_spd": 0.5, ..."fl_ang": 0.1, ...}`
- **RX ← ESP32 (IMU):** `{"type": "imu", "ax":…, "gx":…, "roll":…, "pitch":…, "yaw":…}`

No battery or current sensing — the ESP32 only provides IMU data.

**Parameters:**

| Parameter | Default | Description |
|-----------|---------|-------------|
| `serial_port` | `/dev/ttyUSB0` | ESP32 serial port |
| `baud_rate` | `115200` | Serial baud rate |
| `serial_timeout` | `0.05` | Read timeout (s) |
| `wheelbase_m` | `0.3` | Front-to-rear axle distance (m) |
| `track_width_m` | `0.3` | Left-to-right wheel distance (m) |
| `max_motor_speed` | `1.0` | Max motor speed for normalization |
| `max_servo_angle_rad` | `0.5` | Max steering servo angle (rad) |
| `publish_rate_hz` | `50.0` | Data publish rate (Hz) |

**Subscribed Topics:**

| Topic | Type | Description |
|-------|------|-------------|
| `cmd_vel` | `geometry_msgs/Twist` | Velocity commands → 4-wheel drive/steer |

**Published Topics:**

| Topic | Type | Description |
|-------|------|-------------|
| `esp32/imu_raw` | `Float32MultiArray` | `[ax, ay, az, gx, gy, gz, roll, pitch, yaw]` |
| `rover/status` | `rover_interfaces/RoverStatus` | Aggregated rover status |

**Behavior:** Converts `Twist` to 4-wheel speeds and steering angles using Ackermann-style kinematics, sends JSON over serial. Background thread reads ESP32 responses. Stops motors on shutdown.

---

#### 2. `motor_interface`

High-level motion controller that fuses lane-keeping, obstacle avoidance, and navigation commands into a single `cmd_vel` output.

Supports two operating modes:
- **Legacy mode** (`nav2_mode=False`): PD lane-keeping + obstacle avoidance + navigation commands
- **Nav2 safety mode** (`nav2_mode=True`): Subscribes to Nav2's `nav2/cmd_vel` output, applies obstacle-based emergency stop and speed limiting, forwards to `cmd_vel`

**Behavior Priority (Legacy):** Emergency Stop > Obstacle Avoidance > Lane Keeping > Navigation
**Behavior Priority (Nav2):** Emergency Stop > Obstacle Speed Limiting > Nav2 cmd_vel Passthrough

**Parameters:**

| Parameter | Default | Description |
|-----------|---------|-------------|
| `max_linear_speed` | `0.5` | Max linear speed (m/s) |
| `max_angular_speed` | `1.5` | Max angular speed (rad/s) |
| `lane_kp` | `1.2` | Lane PD proportional gain (legacy only) |
| `lane_kd` | `0.3` | Lane PD derivative gain (legacy only) |
| `heading_kp` | `1.0` | Heading correction gain (legacy only) |
| `obstacle_stop_distance` | `0.5` | Full-stop distance (m) |
| `obstacle_slow_distance` | `1.5` | Slow-down distance (m) |
| `obstacle_avoidance_angular` | `0.8` | Avoidance turn rate (rad/s) (legacy only) |
| `lane_confidence_threshold` | `0.3` | Min lane confidence to trust (legacy only) |
| `control_rate_hz` | `20.0` | Control loop rate (Hz) |
| `cmd_timeout_sec` | `0.5` | Data freshness timeout (s) |
| `nav2_mode` | `false` | Enable Nav2 safety mode |

**Subscribed Topics:**

| Topic | Type | Mode |
|-------|------|------|
| `perception/lane` | `rover_interfaces/Lane` | Legacy only |
| `perception/obstacles` | `rover_interfaces/Obstacle` | Both |
| `nav/cmd_vel` | `geometry_msgs/Twist` | Legacy only |
| `nav2/cmd_vel` | `geometry_msgs/Twist` | Nav2 only |
| `rover/e_stop` | `std_msgs/Bool` | Both |

**Published Topics:**

| Topic | Type | Description |
|-------|------|-------------|
| `cmd_vel` | `geometry_msgs/Twist` | Final velocity command |

**Control Logic (Legacy):**
1. E-stop → zero velocity.
2. Closest obstacle < `obstacle_stop_distance` → stop, turn away.
3. Obstacle in slow zone → proportional speed reduction + avoidance angular offset.
4. Lane detected with sufficient confidence → PD controller on lateral offset + heading correction.
5. No lane → relay navigation commands at reduced speed.
6. Clamp output to max speeds.

**Control Logic (Nav2):**
1. E-stop → zero velocity.
2. Closest obstacle < `obstacle_stop_distance` → emergency stop (direct sensor override).
3. Obstacle in slow zone → proportional speed reduction on Nav2's cmd_vel.
4. Forward Nav2's cmd_vel with speed limits applied.

---

#### 3. `odometry_node`

Computes rover odometry from ESP32 external IMU heading and commanded velocity. Publishes `odom → base_link` transforms.

**Parameters:**

| Parameter | Default | Description |
|-----------|---------|-------------|
| `publish_tf` | `True` | Broadcast `odom → base_link` TF |
| `odom_rate_hz` | `50.0` | Odometry update rate (Hz) |
| `speed_scale` | `1.0` | Calibration multiplier for cmd_vel → actual speed |

**Subscribed Topics:**

| Topic | Type | Description |
|-------|------|-------------|
| `esp32/imu_raw` | `Float32MultiArray` | IMU data (yaw from index 8, gyro-z from index 5) |
| `cmd_vel` | `geometry_msgs/Twist` | Commanded linear velocity |

**Published Topics:**

| Topic | Type | Description |
|-------|------|-------------|
| `odom` | `nav_msgs/Odometry` | Odometry (pose + twist) |
| `rover/pose` | `geometry_msgs/PoseStamped` | Current rover pose |

**TF Broadcast:** `odom → base_link`

**Method:** Dead-reckoning using IMU-fused yaw + commanded speed (no wheel encoders). Position integrated each cycle: `x += v·cos(θ)·dt`, `y += v·sin(θ)·dt`.

---

#### 4. `mock_esp32`

Simulates the ESP32 by subscribing to `cmd_vel` and publishing fake IMU data. Replaces `esp32_bridge` for testing without hardware.

**Parameters:**

| Parameter | Default | Description |
|-----------|---------|-------------|
| `wheelbase_m` | `0.3` | Front-to-rear axle distance (m) |
| `track_width_m` | `0.3` | Left-to-right wheel distance (m) |
| `publish_rate_hz` | `50.0` | Publish rate (Hz) |

**Behavior:** Integrates angular velocity into simulated yaw. Publishes simulated IMU with gravity on Z and `RoverStatus`.

---

### `rover_navigation`

Autonomous navigation state machine, lane-based costmap, and reactive planner. Depends on: `rclpy`, `geometry_msgs`, `nav_msgs`, `std_msgs`, `rover_interfaces`.

#### 1. `goal_manager`

Manages autonomous traversal with a state machine: `IDLE → NAVIGATING → CHECKPOINT_SCAN → NAVIGATING → …` (any state can transition to `OBSTACLE_AVOIDANCE` and back).

Supports two operating modes:
- **Legacy mode** (`use_nav2=False`): Publishes Twist commands to `nav/cmd_vel` for the custom navigation stack.
- **Nav2 mode** (`use_nav2=True`): Sends goals to Nav2 via `NavigateToPose` action. Accepts goals from `goal_pose` (rviz2) and `waypoints` (PoseArray) topics. Manages waypoint queuing, checkpoint interruption, and goal resumption.

**Parameters:**

| Parameter | Default | Description |
|-----------|---------|-------------|
| `default_forward_speed` | `0.3` | Normal driving speed (m/s) |
| `checkpoint_approach_speed` | `0.15` | Speed near checkpoints (m/s) |
| `checkpoint_stop_duration_sec` | `3.0` | Stop time at checkpoint (s) |
| `no_lane_timeout_sec` | `2.0` | Time without lane before crawl mode (s) |
| `obstacle_critical_dist` | `0.4` | Obstacle distance for avoidance (m) |
| `control_rate_hz` | `10.0` | Control loop rate (Hz) |
| `use_nav2` | `false` | Enable Nav2 action client mode |

**Subscribed Topics:**

| Topic | Type | Mode |
|-------|------|------|
| `odom` | `nav_msgs/Odometry` | Both |
| `perception/lane` | `rover_interfaces/Lane` | Both |
| `perception/obstacles` | `rover_interfaces/Obstacle` | Both |
| `perception/qr_detected` | `std_msgs/String` | Both |
| `goal_pose` | `geometry_msgs/PoseStamped` | Nav2 only |
| `waypoints` | `geometry_msgs/PoseArray` | Nav2 only |

**Published Topics:**

| Topic | Type | Description |
|-------|------|-------------|
| `nav/cmd_vel` | `geometry_msgs/Twist` | Navigation velocity command (legacy mode) |
| `rover/state` | `std_msgs/String` | Current state name |
| `rover/e_stop` | `std_msgs/Bool` | Emergency stop flag |

**Nav2 Action Client:** `navigate_to_pose` (`nav2_msgs/NavigateToPose`) — sends navigation goals to the Nav2 stack.

**State Machine:**
- **IDLE:** (Nav2 mode) Waiting for goal poses. (Legacy mode) Not used — starts in NAVIGATING.
- **NAVIGATING:** (Nav2) Nav2 handles path following; goal_manager monitors for QR checkpoints. (Legacy) Drive forward at `default_forward_speed`.
- **OBSTACLE_AVOIDANCE:** (Legacy only) Slow to approach speed. Returns when obstacle clears 2× critical distance.
- **CHECKPOINT_SCAN:** Full stop for `checkpoint_stop_duration_sec`. In Nav2 mode, cancels current goal and resumes after scan.

---

#### 2. `lane_costmap_layer`

Publishes a local occupancy grid marking lane boundaries as high-cost zones, keeping the rover between lane lines. Designed to overlay with obstacle costmaps in Nav2.

**Parameters:**

| Parameter | Default | Description |
|-----------|---------|-------------|
| `costmap_resolution` | `0.05` | Grid resolution (m/cell) |
| `costmap_width_m` | `4.0` | Costmap width (m) |
| `costmap_height_m` | `4.0` | Costmap height (m) |
| `lane_boundary_cost` | `100` | Cost on lane boundary cells |
| `outside_lane_cost` | `100` | Cost outside lane |
| `inside_lane_cost` | `0` | Cost inside lane |
| `boundary_width_cells` | `3` | Boundary thickness (cells) |
| `camera_height_m` | `0.3` | Camera height for projection (m) |
| `publish_rate_hz` | `5.0` | Publish rate (Hz) |

**Subscribed:** `perception/lane` → **Published:** `navigation/lane_costmap` (`OccupancyGrid`)

**Method:** Projects image-space lane boundaries to ground plane using approximate pinhole model. For each row, interpolates left/right lane X positions, marks inside/boundary/outside costs.

---

#### 3. `planner_helper`

Merges obstacle and lane costmaps into a unified local costmap. Implements reactive obstacle avoidance using the VFH (Vector Field Histogram) approach.

**Parameters:**

| Parameter | Default | Description |
|-----------|---------|-------------|
| `num_sectors` | `36` | Polar histogram sectors (10° each) |
| `sector_threshold` | `0.3` | Blocked sector threshold |
| `max_obstacle_range` | `3.0` | Max obstacle range for histogram (m) |
| `safety_margin_m` | `0.3` | Safety margin (m) |
| `preferred_direction_rad` | `0.0` | Preferred heading (straight ahead) |
| `publish_rate_hz` | `10.0` | Compute rate (Hz) |

**Subscribed Topics:**

| Topic | Type |
|-------|------|
| `perception/obstacles` | `rover_interfaces/Obstacle` |
| `perception/local_costmap` | `OccupancyGrid` |
| `navigation/lane_costmap` | `OccupancyGrid` |

**Published Topics:**

| Topic | Type | Description |
|-------|------|-------------|
| `navigation/merged_costmap` | `OccupancyGrid` | Merged obstacle + lane costmap |
| `navigation/avoidance_cmd` | `geometry_msgs/Twist` | VFH-based avoidance command |

**VFH Algorithm:** Builds polar obstacle density histogram, decays over time (×0.8). Finds free sectors, selects the one closest to preferred heading. All blocked → turn toward least-blocked sector.

---

### `rover_comm`

WiFi communication nodes for streaming data to a browser-based Ground Control Station. Depends on: `rclpy`, `sensor_msgs`, `nav_msgs`, `geometry_msgs`, `std_msgs`, `cv_bridge`, `rover_interfaces`.

#### 1. `video_stream_node`

Streams live camera feed to GCS using MJPEG over HTTP. Also publishes ROS `CompressedImage`.

**Parameters:**

| Parameter | Default | Description |
|-----------|---------|-------------|
| `stream_port` | `8080` | HTTP server port |
| `jpeg_quality` | `70` | JPEG compression quality |
| `max_fps` | `15` | Max streaming frame rate |
| `resize_width` | `640` | Stream resize width |
| `resize_height` | `480` | Stream resize height |
| `image_topic` | `camera/color/image_raw` | Source image topic |

**HTTP Endpoints:**
- `GET /stream` — MJPEG video stream
- `GET /snapshot` — Single JPEG frame
- `GET /` — HTML page with embedded stream

**Subscribed:** `camera/color/image_raw` → **Published:** `camera/color/compressed` (`CompressedImage`)

---

#### 2. `map_stream_node`

Streams terrain map and occupancy grid data to GCS over HTTP as JSON. Supports Server-Sent Events (SSE) for real-time updates.

**Parameters:**

| Parameter | Default | Description |
|-----------|---------|-------------|
| `map_port` | `8081` | HTTP server port |
| `update_rate_hz` | `2.0` | SSE push rate (Hz) |
| `compress_map` | `True` | Compress map data |

**HTTP Endpoints:**
- `GET /occupancy` — Latest occupancy grid as JSON (base64-encoded data)
- `GET /terrain` — Latest terrain map as JSON (base64-encoded data)
- `GET /stream` — SSE stream with both occupancy and terrain updates

**Subscribed Topics:**

| Topic | Type |
|-------|------|
| `perception/local_costmap` | `OccupancyGrid` |
| `perception/terrain_map` | `rover_interfaces/TerrainMap` |

---

#### 3. `telemetry_node`

Aggregates rover status, checkpoint data, and system health. Serves JSON API to GCS and provides the `register_checkpoint` service.

**Parameters:**

| Parameter | Default | Description |
|-----------|---------|-------------|
| `telemetry_port` | `8082` | HTTP server port |
| `update_rate_hz` | `5.0` | SSE push rate (Hz) |

**HTTP Endpoints:**
- `GET /telemetry` — Full telemetry snapshot (status, pose, checkpoints)
- `GET /checkpoints` — List of registered checkpoints
- `GET /stream` — SSE real-time telemetry stream
- `GET /` — Info page

**Subscribed Topics:**

| Topic | Type |
|-------|------|
| `rover/status` | `rover_interfaces/RoverStatus` |
| `rover/pose` | `geometry_msgs/PoseStamped` |
| `perception/qr_detected` | `std_msgs/String` |

**Service Server:** `register_checkpoint` (`rover_interfaces/srv/RegisterCheckpoint`) — assigns checkpoint IDs (`CP_001`, `CP_002`, …) and stores checkpoint data.

---

### `rover_bringup`

Launch files and configuration for the complete system. Depends on: `rover_interfaces`, `rover_perception`, `rover_control`, `rover_navigation`, `rover_comm`, `rtabmap_ros`, `tf2_ros`, `nav2_bringup`, `nav2_bt_navigator`, `nav2_controller`, `nav2_planner`, `nav2_behaviors`, `nav2_lifecycle_manager`, `nav2_costmap_2d`.

---

## Launch Files

### `rover.launch.py`

**Full system launch** — starts all subsystems: perception, control, navigation, and communication.

**Launch Arguments:**

| Argument | Default | Description |
|----------|---------|-------------|
| `serial_port` | `/dev/ttyUSB0` | ESP32 serial port |
| `video_port` | `8080` | Video stream port |
| `map_port` | `8081` | Map stream port |
| `telemetry_port` | `8082` | Telemetry port |

**Nodes launched:**
- Perception: `realsense_node`, `lane_detector`, `obstacle_detector`, `depth_processor`, `qr_scanner`
- Control: `esp32_bridge`, `motor_interface`, `odometry_node`
- Navigation: `goal_manager`, `lane_costmap_layer`, `planner_helper`
- Communication (2s delayed): `video_stream_node`, `map_stream_node`, `telemetry_node`

```bash
ros2 launch rover_bringup rover.launch.py
ros2 launch rover_bringup rover.launch.py serial_port:=/dev/ttyACM0
```

---

### `perception_test.launch.py`

**Perception-only** — tests camera and detection pipeline without ESP32 or motor control. Includes static TF for `base_link → camera_color_optical_frame` and the video stream node.

```bash
ros2 launch rover_bringup perception_test.launch.py
```

---

### `sim_test.launch.py`

**Simulation test** — runs the full pipeline with `mock_camera` and `mock_esp32` instead of real hardware. Works on any PC.

**Nodes launched:** `mock_camera`, `mock_esp32`, static TF, all perception processing nodes, control nodes (no `esp32_bridge`), navigation nodes, and communication nodes (2s delayed).

```bash
ros2 launch rover_bringup sim_test.launch.py
```

---

### `slam_3d.launch.py`

**3D SLAM + Terrain Mapping** — launches RTAB-Map for visual SLAM with the RealSense D435i, plus the terrain mapper.

**Launch Arguments:**

| Argument | Default | Description |
|----------|---------|-------------|
| `use_sim_time` | `false` | Use simulation clock |
| `localization` | `false` | Localization-only mode (reuse existing map) |
| `rviz` | `false` | Launch RTAB-Map visualization |
| `delete_db` | `false` | Delete map database and start fresh |

**Nodes launched:** Static TFs (base_link → camera_link → optical frames), `rtabmap` (SLAM), `rgbd_odometry` (visual odom), `depth_processor`, `terrain_mapper`, optional `rtabmap_viz`.

```bash
ros2 launch rover_bringup slam_3d.launch.py
ros2 launch rover_bringup slam_3d.launch.py localization:=true
ros2 launch rover_bringup slam_3d.launch.py rviz:=true delete_db:=true
```

---

### `navigation.launch.py`

**Autonomous Navigation (Nav2 + RTAB-Map)** — launches the complete Nav2-based navigation stack. This is the primary launch file for autonomous waypoint navigation with obstacle avoidance and lane boundary enforcement.

**Launch Arguments:**

| Argument | Default | Description |
|----------|---------|-------------|
| `use_sim_time` | `false` | Use simulation clock |
| `localization` | `false` | Localization-only mode (reuse existing RTAB-Map) |
| `serial_port` | `/dev/ttyUSB0` | ESP32 serial port |
| `video_port` | `8080` | Video stream port |
| `map_port` | `8081` | Map stream port |
| `telemetry_port` | `8082` | Telemetry port |
| `delete_db` | `false` | Delete RTAB-Map database and start fresh |

**Architecture:**

```
goal_pose / waypoints → goal_manager (Nav2 action client)
                              ↓
                    Nav2 NavigateToPose action
                              ↓
            bt_navigator → planner_server → controller_server
                              ↓                    ↓
                      Global Costmap          Local Costmap
                    (RTAB-Map map +        (Depth obstacles +
                     obstacles +            lane boundaries +
                     inflation)             inflation)
                                                   ↓
                                           nav2/cmd_vel
                                                   ↓
                                    motor_interface (safety layer)
                                                   ↓
                                              cmd_vel → esp32_bridge
```

**Nodes launched:**
- SLAM (via `slam_3d.launch.py`): `rtabmap`, `rgbd_odometry`, `depth_processor`, `terrain_mapper`, static TFs
- Perception: `realsense_node`, `lane_detector`, `obstacle_detector`, `qr_scanner`
- Control: `esp32_bridge`, `odometry_node`, `motor_interface` (Nav2 safety mode)
- Navigation: `lane_costmap_layer`, `goal_manager` (Nav2 mode)
- Nav2: `controller_server`, `planner_server`, `behavior_server`, `bt_navigator`, `lifecycle_manager_navigation`
- Communication (2s delayed): `video_stream_node`, `map_stream_node`, `telemetry_node`

**Nav2 Costmap Design:**
- **Local costmap** (rolling 5×5m window in `odom` frame):
  - `VoxelLayer`: Depth camera point cloud (`perception/slam_point_cloud`) for obstacle detection with raycasting
  - `StaticLayer` (lane): Lane boundary costmap (`navigation/lane_costmap`) marking yellow lines as lethal
  - `InflationLayer`: Inflates obstacles by 0.55m with cost scaling factor 3.0
- **Global costmap** (full map in `map` frame):
  - `StaticLayer`: RTAB-Map occupancy grid (`rtabmap/grid_map`) as base environment map
  - `ObstacleLayer`: Real-time depth camera obstacles for dynamic obstacle tracking
  - `InflationLayer`: Inflates obstacles for safe global path planning

**Sending Goals:**
```bash
# Launch the full navigation stack
ros2 launch rover_bringup navigation.launch.py

# Send a goal via CLI
ros2 topic pub --once /goal_pose geometry_msgs/PoseStamped \
  '{header: {frame_id: "map"}, pose: {position: {x: 2.0, y: 1.0}, orientation: {w: 1.0}}}'

# Or use rviz2 "Nav2 Goal" button

# Use existing map (localization-only mode)
ros2 launch rover_bringup navigation.launch.py localization:=true
```

---

## Configuration Files

### `rover_params.yaml`

Central parameter file for all nodes. Organized by subsystem (Perception, Control, Navigation, Communication). Loaded by launch files. Contains all tunable parameters documented in the node sections above.

### `rtabmap.yaml`

RTAB-Map visual SLAM configuration optimized for Intel RealSense D435i on a ground rover:
- **Features:** ORB detector (fast for embedded), KD-tree matching
- **Visual Odometry:** Frame-to-Map strategy with guess motion
- **3D Grid:** Depth-based, 0.05m cells, 0.3–5.0m range, normals segmentation
- **Optimizer:** g2o, 3D SLAM with IMU gravity sigma
- **Loop Closure:** Proximity by space, local bundle adjustment on closure
- **Database:** `~/.ros/rtabmap_rover.db`

### `slam.yaml`

Terrain mapper parameters for the SLAM pipeline. Matches the `terrain_mapper` node defaults.

### `nav2_params.yaml`

Complete Nav2 navigation stack configuration. Contains parameters for:
- **bt_navigator**: Behavior tree navigator with all standard BT node plugins
- **controller_server**: DWB local planner configured for 4WD/4WS rover (max 0.5 m/s linear, 1.5 rad/s angular)
- **planner_server**: NavFn global planner with A* search
- **behavior_server**: Recovery behaviors (spin, backup, wait)
- **local_costmap**: Rolling 5×5m window with VoxelLayer (depth obstacles), StaticLayer (lane boundaries), InflationLayer
- **global_costmap**: Full RTAB-Map grid with StaticLayer, ObstacleLayer, InflationLayer

### `costmap.yaml`

Reference file pointing to `nav2_params.yaml` where all costmap layer configuration is defined.

---

## ROS 2 Topic & Service Map

### Perception Topics (Published)

| Topic | Type | Publisher |
|-------|------|----------|
| `camera/color/image_raw` | `Image` | `realsense_node` |
| `camera/depth/image_raw` | `Image` | `realsense_node` |
| `camera/aligned_depth/image_raw` | `Image` | `realsense_node` |
| `camera/color/camera_info` | `CameraInfo` | `realsense_node` |
| `camera/depth/camera_info` | `CameraInfo` | `realsense_node` |
| `camera/imu` | `Imu` | `realsense_node` |
| `perception/lane` | `Lane` | `lane_detector` |
| `perception/lane_debug` | `Image` | `lane_detector` |
| `perception/obstacles` | `Obstacle` | `obstacle_detector` |
| `perception/obstacle_debug` | `Image` | `obstacle_detector` |
| `perception/point_cloud` | `PointCloud2` | `depth_processor` |
| `perception/terrain_map` | `TerrainMap` | `depth_processor` |
| `perception/local_costmap` | `OccupancyGrid` | `depth_processor` |
| `perception/slam_point_cloud` | `PointCloud2` | `depth_processor` |
| `perception/terrain_map_3d` | `TerrainMap3D` | `terrain_mapper` |
| `perception/qr_detected` | `String` | `qr_scanner` |
| `perception/qr_debug` | `Image` | `qr_scanner` |

### Control Topics

| Topic | Type | Publisher |
|-------|------|----------|
| `cmd_vel` | `Twist` | `motor_interface` |
| `esp32/imu_raw` | `Float32MultiArray` | `esp32_bridge` |
| `rover/status` | `RoverStatus` | `esp32_bridge` |
| `odom` | `Odometry` | `odometry_node` |
| `rover/pose` | `PoseStamped` | `odometry_node` |

### Navigation Topics

| Topic | Type | Publisher |
|-------|------|----------|
| `nav/cmd_vel` | `Twist` | `goal_manager` (legacy mode) |
| `nav2/cmd_vel` | `Twist` | `controller_server` (Nav2 mode, remapped) |
| `rover/state` | `String` | `goal_manager` |
| `rover/e_stop` | `Bool` | `goal_manager` |
| `navigation/lane_costmap` | `OccupancyGrid` | `lane_costmap_layer` |
| `navigation/merged_costmap` | `OccupancyGrid` | `planner_helper` |
| `navigation/avoidance_cmd` | `Twist` | `planner_helper` |
| `goal_pose` | `PoseStamped` | External (rviz2 / user) → `goal_manager` |
| `waypoints` | `PoseArray` | External → `goal_manager` |

### Communication Topics

| Topic | Type | Publisher |
|-------|------|----------|
| `camera/color/compressed` | `CompressedImage` | `video_stream_node` |
| `slam/occupancy_grid` | `OccupancyGrid` | `terrain_mapper` |

### Services

| Service | Type | Server |
|---------|------|--------|
| `register_checkpoint` | `RegisterCheckpoint` | `telemetry_node` |

---

## TF Tree

```
map (RTAB-Map)
 └── odom (odometry_node)
      └── base_link
           └── camera_link
                ├── camera_color_optical_frame
                ├── camera_depth_optical_frame
                └── camera_imu_optical_frame
```

- `odom → base_link`: Published by `odometry_node` (dead-reckoning from IMU yaw + cmd_vel).
- `map → odom`: Published by RTAB-Map (corrects drift via loop closures).
- `base_link → camera_*`: Static transforms published in launch files.
  - Camera mounted 0.15m forward, 0.3m above base_link, pitched down ~29° (-0.5 rad).
