"""
Autonomous Navigation Launch File (Nav2 + RTAB-Map).

Launches the complete autonomous navigation stack:
  - RTAB-Map SLAM (via slam_3d.launch.py)
  - Nav2 navigation stack (planner, controller, behavior tree)
  - Perception pipeline (RealSense, lane detection, obstacle detection, QR)
  - Control nodes (ESP32 bridge, odometry, motor interface in Nav2 safety mode)
  - Lane costmap layer (feeds Nav2 local costmap)
  - Goal manager (Nav2 action client mode)
  - Communication nodes (video, map, telemetry streams)

Usage:
  ros2 launch rover_bringup navigation.launch.py
  ros2 launch rover_bringup navigation.launch.py localization:=true  # use existing map
  ros2 launch rover_bringup navigation.launch.py delete_db:=true     # fresh SLAM map
"""
import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, GroupAction, IncludeLaunchDescription,
    SetEnvironmentVariable, TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('rover_bringup')
    nav2_params = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    localization = LaunchConfiguration('localization')
    serial_port = LaunchConfiguration('serial_port')
    video_port = LaunchConfiguration('video_port')
    map_port = LaunchConfiguration('map_port')
    telemetry_port = LaunchConfiguration('telemetry_port')
    delete_db = LaunchConfiguration('delete_db')
    camera_source = LaunchConfiguration('camera_source')
    camera_index = LaunchConfiguration('camera_index')
    depth_model = LaunchConfiguration('depth_model')

    # ─── Launch Arguments ───
    declare_args = [
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('localization', default_value='false',
                              description='Localization-only mode (use existing map)'),
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('video_port', default_value='8080'),
        DeclareLaunchArgument('map_port', default_value='8081'),
        DeclareLaunchArgument('telemetry_port', default_value='8082'),
        DeclareLaunchArgument('delete_db', default_value='false',
                              description='Delete RTAB-Map database and start fresh'),
        DeclareLaunchArgument('camera_source', default_value='laptop',
                      description='Camera source: realsense or laptop'),
        DeclareLaunchArgument('camera_index', default_value='0',
                      description='Laptop webcam index for laptop mode'),
        DeclareLaunchArgument('depth_model', default_value='midas',
                      description='Monocular depth model for laptop mode: midas or depth_anything'),
    ]

    # ─── Include SLAM launch (RTAB-Map + TFs + depth_processor + terrain_mapper) ───
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'slam_3d.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'localization': localization,
            'rviz': 'false',
            'delete_db': delete_db,
            'camera_source': camera_source,
            'camera_index': camera_index,
            'depth_model': depth_model,
        }.items(),
    )

    # ─── Perception Nodes (camera + depth_processor are in slam_3d) ───
    perception_group = GroupAction([
        Node(
            package='rover_perception',
            executable='lane_detector',
            name='lane_detector',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'yellow_h_low': 20,
                'yellow_h_high': 35,
                'yellow_s_low': 80,
                'yellow_s_high': 255,
                'yellow_v_low': 80,
                'yellow_v_high': 255,
                'min_contour_area': 500,
                'roi_top_ratio': 0.4,
                'camera_height_m': 0.3,
            }],
        ),
        Node(
            package='rover_perception',
            executable='obstacle_detector',
            name='obstacle_detector',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'min_depth_m': 0.3,
                'max_depth_m': 5.0,
                'obstacle_height_threshold_m': 0.10,
                'min_cluster_points': 50,
                'velocity_threshold_m_s': 0.1,
            }],
        ),
        Node(
            package='rover_perception',
            executable='qr_scanner',
            name='qr_scanner',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'scan_interval_sec': 0.5,
                'min_qr_size': 50,
                'deduplicate_timeout_sec': 30.0,
            }],
        ),
    ])

    # ─── Control Nodes ───
    control_group = GroupAction([
        Node(
            package='rover_control',
            executable='esp32_bridge',
            name='esp32_bridge',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'serial_port': serial_port,
                'baud_rate': 115200,
                'wheelbase_m': 0.3,
                'track_width_m': 0.3,
                'max_motor_speed': 1.0,
                'max_servo_angle_rad': 0.5,
                'publish_rate_hz': 50.0,
                'drive_mode': 'differential',
                'expect_esp32_rx': False,
            }],
        ),
        Node(
            package='rover_control',
            executable='odometry_node',
            name='odometry_node',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'publish_tf': True,
                'odom_rate_hz': 50.0,
                'speed_scale': 1.0,
            }],
        ),
        # Motor interface in Nav2 safety mode:
        # Subscribes to Nav2's cmd_vel output (remapped to nav2/cmd_vel),
        # applies obstacle-based emergency stop, forwards to cmd_vel.
        Node(
            package='rover_control',
            executable='motor_interface',
            name='motor_interface',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'nav2_mode': True,
                'max_linear_speed': 0.35,
                'max_angular_speed': 1.0,
                'obstacle_stop_distance': 0.5,
                'obstacle_slow_distance': 1.5,
                'control_rate_hz': 20.0,
                'cmd_timeout_sec': 0.5,
            }],
        ),
    ])

    # ─── Lane Costmap Layer (feeds Nav2 local costmap) ───
    lane_costmap_node = Node(
        package='rover_navigation',
        executable='lane_costmap_layer',
        name='lane_costmap_layer',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'costmap_resolution': 0.05,
            'costmap_width_m': 4.0,
            'costmap_height_m': 4.0,
            'lane_boundary_cost': 100,
            'outside_lane_cost': 100,
            'publish_rate_hz': 5.0,
        }],
    )

    # ─── Nav2 Stack ───
    nav2_group = GroupAction([
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            respawn=True,
            respawn_delay=2.0,
            parameters=[nav2_params, {'use_sim_time': use_sim_time}],
            remappings=[('cmd_vel', 'nav2/cmd_vel')],
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            respawn=True,
            respawn_delay=2.0,
            parameters=[nav2_params, {'use_sim_time': use_sim_time}],
        ),
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            respawn=True,
            respawn_delay=2.0,
            parameters=[nav2_params, {'use_sim_time': use_sim_time}],
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            respawn=True,
            respawn_delay=2.0,
            parameters=[nav2_params, {'use_sim_time': use_sim_time}],
        ),
        # Lifecycle manager brings up Nav2 nodes in correct order
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': [
                    'controller_server',
                    'planner_server',
                    'behavior_server',
                    'bt_navigator',
                ],
            }],
        ),
    ])

    # ─── Goal Manager (Nav2 mode) ───
    goal_manager_node = Node(
        package='rover_navigation',
        executable='goal_manager',
        name='goal_manager',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'use_nav2': True,
            'auto_start_nav2_goals': True,
            'nav2_goal_frame': 'map',
            'nav2_auto_waypoints_xy': [0.7, 0.0, 1.0, 0.28, 1.25, 0.0, 1.0, -0.28, 0.7, 0.0],
            'nav2_loop_waypoints': True,
            'nav2_start_delay_sec': 4.0,
            'default_forward_speed': 0.3,
            'checkpoint_approach_speed': 0.15,
            'checkpoint_stop_duration_sec': 3.0,
            'no_lane_timeout_sec': 2.0,
            'obstacle_critical_dist': 0.4,
        }],
    )

    # ─── Communication Nodes (delayed start) ───
    comm_group = TimerAction(
        period=2.0,
        actions=[
            GroupAction([
                Node(
                    package='rover_comm',
                    executable='video_stream_node',
                    name='video_stream_node',
                    output='screen',
                    parameters=[{
                        'stream_port': video_port,
                        'jpeg_quality': 70,
                        'max_fps': 15,
                        'resize_width': 640,
                        'resize_height': 480,
                    }],
                ),
                Node(
                    package='rover_comm',
                    executable='map_stream_node',
                    name='map_stream_node',
                    output='screen',
                    parameters=[{
                        'map_port': map_port,
                        'update_rate_hz': 2.0,
                    }],
                ),
                Node(
                    package='rover_comm',
                    executable='telemetry_node',
                    name='telemetry_node',
                    output='screen',
                    parameters=[{
                        'telemetry_port': telemetry_port,
                        'update_rate_hz': 5.0,
                    }],
                ),
            ]),
        ],
    )

    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        *declare_args,
        slam_launch,
        perception_group,
        control_group,
        lane_costmap_node,
        nav2_group,
        goal_manager_node,
        comm_group,
    ])
