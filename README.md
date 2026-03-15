# Autonomous Rover

An autonomous ground rover built with **ROS 2 Humble** that follows yellow lane boundaries, avoids obstacles (static and dynamic), scans QR checkpoints, and streams live telemetry to a browser-based Ground Control Station over WiFi.

## Hardware

| Component | Role |
|-----------|------|
| **Raspberry Pi 4** | Main compute — runs ROS 2 nodes |
| **Intel RealSense D455** | RGB + Depth camera with onboard IMU |
| **Laptop RGB webcam (optional)** | Home testing camera source with monocular depth |
| **ESP32** | Low-level motor control, external IMU (AHRS) |
| **4WD/4WS chassis** | Four independently driven and steered wheels (0.3 m wheelbase, 0.3 m track width) |

## Features

- **Nav2 autonomous navigation** — Waypoint navigation with DWB local planner, NavFn global planner, and behavior tree
- **RTAB-Map visual SLAM** — 3D mapping and localization using RealSense D455 (no wheel encoders needed)
- **Lane boundary enforcement** — Yellow lane lines detected via HSV and injected into Nav2 costmap as lethal zones
- **Obstacle avoidance** — Depth-based 3D obstacle detection in both Nav2 costmap layers and direct sensor emergency stop
- **3D terrain mapping** — Terrain classification (flat / rough / slope / step / obstacle) from SLAM point clouds
- **QR checkpoint scanning** — `pyzbar`-based QR decoding with automatic Nav2 goal interruption and resumption
- **GCS streaming** — MJPEG video (port 8080), JSON map data (port 8081), JSON/SSE telemetry (port 8082)
- **Legacy navigation mode** — Custom PD lane-keeping + VFH reactive planner (no Nav2 required)
- **Simulation mode** — Full pipeline testable on any PC with mock camera and mock ESP32
- **Laptop camera testing mode** — Use webcam + MiDaS/Depth Anything monocular depth to publish RealSense-compatible topics

## Quick Start

### Prerequisites

- ROS 2 Humble
- Python 3 packages: `pyrealsense2`, `opencv-python`, `pyzbar`, `pyserial`, `numpy`
- Optional for laptop monocular depth mode: `torch`, `torchvision`, `transformers`
- ROS 2 packages: `cv_bridge`, `tf2_ros`, `rtabmap_ros`, `nav2_bringup`, `nav2_bt_navigator`, `nav2_controller`, `nav2_planner`, `nav2_behaviors`, `nav2_lifecycle_manager`, `nav2_costmap_2d`

### Install Dependencies

```bash
# System dependencies (required by pyzbar)
sudo apt install libzbar0

# ROS dependencies
rosdep install --from-paths src --ignore-src -r -y

# Python dependencies
pip install -r requirements.txt
```

### Build

```bash
cd ~/rover_ws
colcon build --symlink-install
source install/setup.bash
```

### Run (Real Hardware)

```bash
# Full system (Nav2 autonomous navigation + RTAB-Map SLAM)
ros2 launch rover_bringup navigation.launch.py

# Full system (legacy custom navigation, no Nav2)
ros2 launch rover_bringup rover.launch.py

# Perception only (no motors)
ros2 launch rover_bringup perception_test.launch.py

# 3D SLAM + terrain mapping
ros2 launch rover_bringup slam_3d.launch.py
```

### Run (Simulation — No Hardware Required)

```bash
ros2 launch rover_bringup sim_test.launch.py
```

Sample obstacle-course presets:

```bash
# Recommended sample course (alternating moving obstacles)
ros2 launch rover_bringup sim_test.launch.py obstacle_course:=slalom

# Other built-in courses
ros2 launch rover_bringup sim_test.launch.py obstacle_course:=single_moving
ros2 launch rover_bringup sim_test.launch.py obstacle_course:=chicane
ros2 launch rover_bringup sim_test.launch.py obstacle_course:=center_block
```

### Run (Home Testing with Laptop Camera + Monocular Depth)

```bash
# Perception-only on laptop webcam
ros2 launch rover_bringup perception_test.launch.py camera_source:=laptop camera_index:=0 depth_model:=midas

# Full legacy stack on laptop webcam
ros2 launch rover_bringup rover.launch.py camera_source:=laptop camera_index:=0 depth_model:=midas

# Full Nav2 + SLAM stack on laptop webcam
ros2 launch rover_bringup navigation.launch.py camera_source:=laptop camera_index:=0 depth_model:=midas

# Use Depth Anything instead of MiDaS
ros2 launch rover_bringup perception_test.launch.py camera_source:=laptop depth_model:=depth_anything
```

Supported launch arguments:
- `camera_source:=realsense|laptop`
- `camera_index:=0` (webcam index for laptop mode)
- `depth_model:=midas|depth_anything` (laptop mode only)

### Terminal Launch Strategy Cheat-Sheet

Use these commands directly in terminal to choose the navigation/operation strategy:

```bash
# 1) Nav2 autonomous strategy (primary)
ros2 launch rover_bringup navigation.launch.py

# Nav2 autonomous strategy + localization-only (reuse existing map)
ros2 launch rover_bringup navigation.launch.py localization:=true

# Nav2 autonomous strategy + fresh SLAM map (delete previous DB)
ros2 launch rover_bringup navigation.launch.py delete_db:=true

# Nav2 autonomous strategy using laptop camera
ros2 launch rover_bringup navigation.launch.py camera_source:=laptop camera_index:=0 depth_model:=midas

# 2) Legacy reactive strategy (no Nav2)
ros2 launch rover_bringup rover.launch.py

# Legacy reactive strategy using laptop camera
ros2 launch rover_bringup rover.launch.py camera_source:=laptop camera_index:=0 depth_model:=midas

# 3) Simulation strategy (mock camera + mock ESP32, legacy reactive by default)
ros2 launch rover_bringup sim_test.launch.py

# Simulation with different obstacle-course strategies
ros2 launch rover_bringup sim_test.launch.py obstacle_course:=single_moving
ros2 launch rover_bringup sim_test.launch.py obstacle_course:=slalom
ros2 launch rover_bringup sim_test.launch.py obstacle_course:=chicane
ros2 launch rover_bringup sim_test.launch.py obstacle_course:=center_block

# 4) Perception-only strategy (no motor/nav control)
ros2 launch rover_bringup perception_test.launch.py
ros2 launch rover_bringup perception_test.launch.py camera_source:=laptop camera_index:=0 depth_model:=depth_anything

# 5) SLAM / mapping-only strategy (no full nav control stack)
ros2 launch rover_bringup slam_3d.launch.py
ros2 launch rover_bringup slam_3d.launch.py localization:=true
ros2 launch rover_bringup slam_3d.launch.py delete_db:=true
```

Then open in a browser:
- **Video:** `http://<rover-ip>:8080/stream`
- **Telemetry:** `http://<rover-ip>:8082/telemetry`
- **Map data:** `http://<rover-ip>:8081/stream`

## Architecture

```
┌──────────────────────────────────────────────────────────────┐
│                     Raspberry Pi 4 (ROS 2)                    │
│                                                              │
│  Perception          Control           Navigation            │
│  ┌──────────┐   ┌──────────────┐   ┌───────────────┐        │
│  │ realsense │──→│motor_interface│←──│ goal_manager  │        │
│  │ lane_det  │   │ odometry     │   │ lane_costmap  │        │
│  │ depth_proc│   │ esp32_bridge │   │ planner_helper│        │
│  │ obstacle  │   └──────┬───────┘   └───────────────┘        │
│  │ terrain   │          │ UART                               │
│  │ qr_scanner│          ▼                                    │
│  └──────────┘     ┌──────────┐     Communication             │
│                   │  ESP32   │     ┌─────────────┐           │
│                   │ motors   │     │ video_stream │──→ GCS   │
│                   │ IMU/batt │     │ map_stream   │   (WiFi) │
│                   └──────────┘     │ telemetry    │           │
│                                    └─────────────┘           │
└──────────────────────────────────────────────────────────────┘
```

## Packages

| Package | Description |
|---------|-------------|
| `rover_interfaces` | Custom ROS 2 messages (`Lane`, `Obstacle`, `TerrainMap`, `TerrainMap3D`, `RoverStatus`, `TerrainCell`) and services (`RegisterCheckpoint`) |
| `rover_perception` | Camera drivers (`realsense_node`, `laptop_depth_camera`), lane detection, obstacle detection, depth processing, 3D terrain mapping, QR scanning, mock camera |
| `rover_control` | ESP32 serial bridge, motor interface (PD lane-keeping + obstacle avoidance), odometry, mock ESP32 |
| `rover_navigation` | Goal manager state machine, lane costmap layer, VFH planner helper |
| `rover_comm` | MJPEG video streaming, JSON map streaming, telemetry server with checkpoint registration |
| `rover_bringup` | Launch files and YAML configuration |

## Launch Files

| Launch File | Primary Use | Navigation Strategy | Typical Command |
|-------------|-------------|---------------------|-----------------|
| `navigation.launch.py` | Full autonomous driving stack on real hardware or laptop camera | **Nav2 + RTAB-Map**: NavFn global planner + DWB local planner + BT navigator + lane/depth costmaps + motor safety layer | `ros2 launch rover_bringup navigation.launch.py` |
| `rover.launch.py` | Full stack without Nav2 | **Legacy reactive**: `goal_manager` state machine + `motor_interface` lane/obstacle fusion + optional VFH helper output | `ros2 launch rover_bringup rover.launch.py` |
| `sim_test.launch.py` | No-hardware end-to-end testing with mock sensors and mock ESP32 | **Legacy reactive (simulation)** by default, with selectable obstacle-course presets (`single_moving`, `slalom`, `chicane`, `center_block`) | `ros2 launch rover_bringup sim_test.launch.py obstacle_course:=slalom` |
| `perception_test.launch.py` | Camera and perception validation only | **No motion planning** (lane, obstacle, depth, QR pipeline only) | `ros2 launch rover_bringup perception_test.launch.py camera_source:=laptop` |
| `slam_3d.launch.py` | Mapping/localization and terrain analysis | **SLAM-only**: RTAB-Map + visual odometry + terrain mapping (not full navigation control stack) | `ros2 launch rover_bringup slam_3d.launch.py` |

## Navigation Strategies

### 1) Nav2 Deliberative Navigation (Primary Autonomous Mode)

- Used by: `navigation.launch.py`
- Global planning: `nav2_navfn_planner` (A* enabled)
- Local control: `dwb_core::DWBLocalPlanner`
- Behavior orchestration: `bt_navigator` behavior tree
- Mapping and localization: RTAB-Map (`map -> odom -> base_link`)
- Goal source: internal auto-waypoints (`nav2_auto_waypoints_xy`) by default, with optional external overrides (`/goal_pose`, `/waypoints`)
- Safety layer: `motor_interface` in `nav2_mode=True` applies emergency-stop and obstacle-based speed limiting to `nav2/cmd_vel`

### 2) Legacy Reactive Navigation (Fallback / Lightweight Mode)

- Used by: `rover.launch.py`, `sim_test.launch.py`
- High-level state machine: `goal_manager` (`IDLE`, `NAVIGATING`, `CHECKPOINT_SCAN`, `OBSTACLE_AVOIDANCE`)
- Motion fusion: `motor_interface` combines lane confidence, obstacle distance/bearing, and nav command with this priority:
	`e-stop > obstacle handling > lane guidance > nav command`
- Reactive helper: `planner_helper` computes VFH-style avoidance command and merged local costmap topics for diagnostics/experiments

### 3) Perception-Only / SLAM-Only Workflows

- `perception_test.launch.py`: validates sensing and scene-understanding stack without drive control
- `slam_3d.launch.py`: validates RTAB-Map, TF chain, and terrain mapping outputs without full navigation behavior

## Configuration

All tunable parameters are in `src/rover_bringup/config/rover_params.yaml`. Key parameters to adjust for your hardware:

- **Lane HSV thresholds** — Adjust `yellow_h_low/high`, `yellow_s_low/high`, `yellow_v_low/high` for your lighting
- **Camera height** — Set `camera_height_m` to your actual mounting height
- **ESP32 serial port** — Set `serial_port` (default `/dev/ttyUSB0`)
- **Drive mode** — Set `drive_mode` to `differential` (default) or `servo`
- **ESP32 RX telemetry** — Set `expect_esp32_rx` (`false` default for TX-only)
- **Motor tuning** — Adjust `lane_kp`, `lane_kd`, `max_linear_speed` for your chassis
- **Obstacle distances** — Tune `obstacle_stop_distance` and `obstacle_slow_distance`

## GCS Endpoints

| Endpoint | Port | Description |
|----------|------|-------------|
| `/stream` | 8080 | MJPEG video stream |
| `/snapshot` | 8080 | Single JPEG frame |
| `/occupancy` | 8081 | Occupancy grid JSON |
| `/depth_heatmap` | 8081 | Depth heatmap JSON (base64 JPEG) |
| `/terrain` | 8081 | Alias of `/depth_heatmap` (backward compatibility) |
| `/stream` | 8081 | SSE map updates |
| `/telemetry` | 8082 | Telemetry JSON snapshot |
| `/checkpoints` | 8082 | Registered checkpoints |
| `/qr_detections` | 8082 | Raw QR detections (exact decoded strings) |
| `/stream` | 8082 | SSE telemetry stream |

## Documentation

See [documentation.md](documentation.md) for complete technical documentation including all node parameters, topic maps, message definitions, algorithms, TF tree, and configuration details.

## License

Apache-2.0
