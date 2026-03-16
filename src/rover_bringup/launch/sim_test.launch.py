"""
Simulation test launch - runs the FULL rover pipeline with mock camera and
mock ESP32 instead of real hardware. Use on any PC for testing.
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    urdf_path = os.path.join(
        get_package_share_directory('rover_description'), 'urdf', 'rover.urdf')
    with open(urdf_path, 'r', encoding='utf-8') as urdf_file:
        robot_description = urdf_file.read()

    obstacle_course_arg = DeclareLaunchArgument(
        'obstacle_course', default_value='slalom',
        description='Mock obstacle course: single_moving, slalom, chicane, center_block')

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
                'obstacle_course': LaunchConfiguration('obstacle_course'),
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
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
            }],
        ),
        # Static TFs for camera optical frames
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_to_color_optical_tf',
            arguments=[
                '0', '0', '0', '-1.5708', '0', '-1.5708',
                'camera_link', 'camera_color_optical_frame'
            ],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_to_depth_optical_tf',
            arguments=[
                '0', '0', '0', '-1.5708', '0', '-1.5708',
                'camera_link', 'camera_depth_optical_frame'
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
            parameters=[{
                'publish_tf': False,
                'imu_source': 'esp32',
                'legacy_imu_topic': 'esp32/imu_raw',
                'child_frame_id': 'camera_link',
            }],
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
        obstacle_course_arg,
        mock_group,
        perception_group,
        control_group,
        navigation_group,
        comm_group,
    ])