"""
Main launch file for the autonomous rover system.
Launches all subsystems: perception, control, navigation, communication.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Launch arguments
    serial_port_arg = DeclareLaunchArgument(
        'serial_port', default_value='/dev/ttyUSB0',
        description='ESP32 serial port')
    video_port_arg = DeclareLaunchArgument(
        'video_port', default_value='8080',
        description='Video stream HTTP port')
    map_port_arg = DeclareLaunchArgument(
        'map_port', default_value='8081',
        description='Map stream HTTP port')
    telemetry_port_arg = DeclareLaunchArgument(
        'telemetry_port', default_value='8082',
        description='Telemetry HTTP port')

    # ─── Perception Nodes ───
    perception_group = GroupAction([
        Node(
            package='rover_perception',
            executable='realsense_node',
            name='realsense_node',
            output='screen',
            parameters=[{
                'color_width': 640,
                'color_height': 480,
                'depth_width': 640,
                'depth_height': 480,
                'fps': 30,
                'enable_imu': True,
                'align_depth': True,
            }],
        ),
        Node(
            package='rover_perception',
            executable='lane_detector',
            name='lane_detector',
            output='screen',
            parameters=[{
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
                'min_depth_m': 0.3,
                'max_depth_m': 5.0,
                'obstacle_height_threshold_m': 0.10,
                'min_cluster_points': 50,
                'velocity_threshold_m_s': 0.1,
            }],
        ),
        Node(
            package='rover_perception',
            executable='depth_processor',
            name='depth_processor',
            output='screen',
            parameters=[{
                'map_resolution_m': 0.05,
                'map_width_m': 10.0,
                'map_height_m': 10.0,
                'camera_height_m': 0.3,
            }],
        ),
        Node(
            package='rover_perception',
            executable='qr_scanner',
            name='qr_scanner',
            output='screen',
            parameters=[{
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
                'serial_port': LaunchConfiguration('serial_port'),
                'baud_rate': 115200,
                'wheelbase_m': 0.3,
                'track_width_m': 0.3,
                'max_motor_speed': 1.0,
                'max_servo_angle_rad': 0.5,
                'publish_rate_hz': 50.0,
            }],
        ),
        Node(
            package='rover_control',
            executable='motor_interface',
            name='motor_interface',
            output='screen',
            parameters=[{
                'max_linear_speed': 0.5,
                'max_angular_speed': 1.5,
                'lane_kp': 1.2,
                'lane_kd': 0.3,
                'heading_kp': 1.0,
                'obstacle_stop_distance': 0.5,
                'obstacle_slow_distance': 1.5,
                'control_rate_hz': 20.0,
            }],
        ),
        Node(
            package='rover_control',
            executable='odometry_node',
            name='odometry_node',
            output='screen',
            parameters=[{
                'publish_tf': True,
                'odom_rate_hz': 50.0,
                'speed_scale': 1.0,
            }],
        ),
    ])

    # ─── Navigation Nodes ───
    navigation_group = GroupAction([
        Node(
            package='rover_navigation',
            executable='goal_manager',
            name='goal_manager',
            output='screen',
            parameters=[{
                'default_forward_speed': 0.3,
                'checkpoint_approach_speed': 0.15,
                'checkpoint_stop_duration_sec': 3.0,
                'no_lane_timeout_sec': 2.0,
                'obstacle_critical_dist': 0.4,
            }],
        ),
        Node(
            package='rover_navigation',
            executable='lane_costmap_layer',
            name='lane_costmap_layer',
            output='screen',
            parameters=[{
                'costmap_resolution': 0.05,
                'costmap_width_m': 4.0,
                'costmap_height_m': 4.0,
                'lane_boundary_cost': 100,
                'outside_lane_cost': 100,
            }],
        ),
        Node(
            package='rover_navigation',
            executable='planner_helper',
            name='planner_helper',
            output='screen',
            parameters=[{
                'num_sectors': 36,
                'sector_threshold': 0.3,
                'max_obstacle_range': 3.0,
                'safety_margin_m': 0.3,
            }],
        ),
    ])

    # ─── Communication Nodes (delayed start to let perception initialize) ───
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
                        'stream_port': LaunchConfiguration('video_port'),
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
                        'map_port': LaunchConfiguration('map_port'),
                        'update_rate_hz': 2.0,
                    }],
                ),
                Node(
                    package='rover_comm',
                    executable='telemetry_node',
                    name='telemetry_node',
                    output='screen',
                    parameters=[{
                        'telemetry_port': LaunchConfiguration('telemetry_port'),
                        'update_rate_hz': 5.0,
                    }],
                ),
            ])
        ]
    )

    return LaunchDescription([
        serial_port_arg,
        video_port_arg,
        map_port_arg,
        telemetry_port_arg,
        perception_group,
        control_group,
        navigation_group,
        comm_group,
    ])
