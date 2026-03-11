# Autonomous Rover

An autonomous ground rover built with **ROS 2 Humble** that follows yellow lane boundaries, avoids obstacles (static and dynamic), scans QR checkpoints, and streams live telemetry to a browser-based Ground Control Station over WiFi.

## Hardware

| Component | Role |
|-----------|------|
| **Raspberry Pi 4** | Main compute — runs ROS 2 nodes |
| **Intel RealSense D435i** | RGB + Depth camera with onboard IMU |
| **ESP32** | Low-level motor control, external IMU (AHRS) |
| **4WD/4WS chassis** | Four independently driven and steered wheels (0.3 m wheelbase, 0.3 m track width) |

## Features

- **Nav2 autonomous navigation** — Waypoint navigation with DWB local planner, NavFn global planner, and behavior tree
- **RTAB-Map visual SLAM** — 3D mapping and localization using RealSense D435i (no wheel encoders needed)
- **Lane boundary enforcement** — Yellow lane lines detected via HSV and injected into Nav2 costmap as lethal zones
- **Obstacle avoidance** — Depth-based 3D obstacle detection in both Nav2 costmap layers and direct sensor emergency stop
- **3D terrain mapping** — Terrain classification (flat / rough / slope / step / obstacle) from SLAM point clouds
- **QR checkpoint scanning** — `pyzbar`-based QR decoding with automatic Nav2 goal interruption and resumption
- **GCS streaming** — MJPEG video (port 8080), JSON map data (port 8081), JSON/SSE telemetry (port 8082)
- **Legacy navigation mode** — Custom PD lane-keeping + VFH reactive planner (no Nav2 required)
- **Simulation mode** — Full pipeline testable on any PC with mock camera and mock ESP32

## Quick Start

### Prerequisites

- ROS 2 Humble
- Python 3 packages: `pyrealsense2`, `opencv-python`, `pyzbar`, `pyserial`, `numpy`
- ROS 2 packages: `cv_bridge`, `tf2_ros`, `rtabmap_ros`, `nav2_bringup`, `nav2_bt_navigator`, `nav2_controller`, `nav2_planner`, `nav2_behaviors`, `nav2_lifecycle_manager`, `nav2_costmap_2d`

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
| `rover_perception` | Camera driver (`realsense_node`), lane detection, obstacle detection, depth processing, 3D terrain mapping, QR scanning, mock camera |
| `rover_control` | ESP32 serial bridge, motor interface (PD lane-keeping + obstacle avoidance), odometry, mock ESP32 |
| `rover_navigation` | Goal manager state machine, lane costmap layer, VFH planner helper |
| `rover_comm` | MJPEG video streaming, JSON map streaming, telemetry server with checkpoint registration |
| `rover_bringup` | Launch files and YAML configuration |

## Launch Files

| File | Description |
|------|-------------|
| `navigation.launch.py` | **Nav2 autonomous navigation** — Full stack with Nav2 + RTAB-Map SLAM + all subsystems |
| `rover.launch.py` | Legacy system — custom navigation stack with real hardware |
| `perception_test.launch.py` | Perception pipeline only — camera + detection, no motors |
| `sim_test.launch.py` | Full pipeline with mock camera + mock ESP32 (no hardware needed) |
| `slam_3d.launch.py` | RTAB-Map 3D SLAM + terrain mapping |

## Configuration

All tunable parameters are in `src/rover_bringup/config/rover_params.yaml`. Key parameters to adjust for your hardware:

- **Lane HSV thresholds** — Adjust `yellow_h_low/high`, `yellow_s_low/high`, `yellow_v_low/high` for your lighting
- **Camera height** — Set `camera_height_m` to your actual mounting height
- **ESP32 serial port** — Set `serial_port` (default `/dev/ttyUSB0`)
- **Motor tuning** — Adjust `lane_kp`, `lane_kd`, `max_linear_speed` for your chassis
- **Obstacle distances** — Tune `obstacle_stop_distance` and `obstacle_slow_distance`

## GCS Endpoints

| Endpoint | Port | Description |
|----------|------|-------------|
| `/stream` | 8080 | MJPEG video stream |
| `/snapshot` | 8080 | Single JPEG frame |
| `/occupancy` | 8081 | Occupancy grid JSON |
| `/terrain` | 8081 | Terrain map JSON |
| `/stream` | 8081 | SSE map updates |
| `/telemetry` | 8082 | Telemetry JSON snapshot |
| `/checkpoints` | 8082 | Registered checkpoints |
| `/stream` | 8082 | SSE telemetry stream |

## Documentation

See [documentation.md](documentation.md) for complete technical documentation including all node parameters, topic maps, message definitions, algorithms, TF tree, and configuration details.

## License

Apache-2.0
