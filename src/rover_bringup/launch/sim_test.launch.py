"""
Simulation test launch - runs the FULL rover pipeline with mock camera and
mock ESP32 instead of real hardware. Use on any PC for testing.
"""
from launch import LaunchDescription
from launch.actions import GroupAction, TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    # ─── Mock hardware ───
    mock_group = GroupAction([
        Node(
            package='rover_perception',
            executable='mock_camera',
            name='mock_camera',
            output='screen',
            parameters=[{
                'width': 640,
                'height': 480,
                'fps': 30,
                'show_obstacle': True,
                'obstacle_cycle_sec': 6.0,
                'show_qr': True,
                'qr_interval_sec': 15.0,
                'qr_display_duration_sec': 3.0,
            }],
        ),
        Node(
            package='rover_control',
            executable='mock_esp32',
            name='mock_esp32',
            output='screen',
            parameters=[{
                'wheel_base_m': 0.3,
                'track_width_m': 0.3,
                'publish_rate_hz': 50.0,
            }],
        ),
        # Static TF
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_camera_tf',
            arguments=[
                '0.15', '0', '0.3', '0', '0.5', '0',
                'base_link', 'camera_color_optical_frame'
            ],
        ),
    ])

    # ─── Perception (processing, not camera driver) ───
    perception_group = GroupAction([
        Node(
            package='rover_perception',
            executable='lane_detector',
            name='lane_detector',
            output='screen',
        ),
        Node(
            package='rover_perception',
            executable='obstacle_detector',
            name='obstacle_detector',
            output='screen',
        ),
        Node(
            package='rover_perception',
            executable='depth_processor',
            name='depth_processor',
            output='screen',
        ),
        Node(
            package='rover_perception',
            executable='qr_scanner',
            name='qr_scanner',
            output='screen',
        ),
    ])

    # ─── Control (no real ESP32) ───
    control_group = GroupAction([
        Node(
            package='rover_control',
            executable='motor_interface',
            name='motor_interface',
            output='screen',
        ),
        Node(
            package='rover_control',
            executable='odometry_node',
            name='odometry_node',
            output='screen',
        ),
    ])

    # ─── Navigation ───
    navigation_group = GroupAction([
        Node(
            package='rover_navigation',
            executable='goal_manager',
            name='goal_manager',
            output='screen',
        ),
        Node(
            package='rover_navigation',
            executable='lane_costmap_layer',
            name='lane_costmap_layer',
            output='screen',
        ),
        Node(
            package='rover_navigation',
            executable='planner_helper',
            name='planner_helper',
            output='screen',
        ),
    ])

    # ─── Communication (delayed start) ───
    comm_group = TimerAction(
        period=2.0,
        actions=[
            GroupAction([
                Node(
                    package='rover_comm',
                    executable='video_stream_node',
                    name='video_stream_node',
                    output='screen',
                    parameters=[{'stream_port': 8080}],
                ),
                Node(
                    package='rover_comm',
                    executable='map_stream_node',
                    name='map_stream_node',
                    output='screen',
                    parameters=[{'map_port': 8081}],
                ),
                Node(
                    package='rover_comm',
                    executable='telemetry_node',
                    name='telemetry_node',
                    output='screen',
                    parameters=[{'telemetry_port': 8082}],
                ),
            ])
        ]
    )

    return LaunchDescription([
        mock_group,
        perception_group,
        control_group,
        navigation_group,
        comm_group,
    ])