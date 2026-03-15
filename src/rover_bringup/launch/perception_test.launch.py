"""
Perception-only launch file for testing camera and detection pipeline
without ESP32 or motor control.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    camera_source = LaunchConfiguration('camera_source')

    return LaunchDescription([
        DeclareLaunchArgument(
            'camera_source',
            default_value='realsense',
            description='Camera source: realsense or laptop'),
        DeclareLaunchArgument(
            'camera_index',
            default_value='0',
            description='Laptop webcam index for laptop mode'),
        DeclareLaunchArgument(
            'depth_model',
            default_value='midas',
            description='Monocular depth model for laptop mode: midas or depth_anything'),
        DeclareLaunchArgument(
            'map_port',
            default_value='8081',
            description='Map/depth heatmap HTTP port'),
        Node(
            package='rover_perception',
            executable='realsense_node',
            name='realsense_node',
            output='both',
            condition=IfCondition(PythonExpression([
                "'", camera_source, "' == 'realsense'"
            ])),
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
            executable='laptop_depth_camera',
            name='laptop_depth_camera',
            output='both',
            condition=IfCondition(PythonExpression([
                "'", camera_source, "' == 'laptop'"
            ])),
            parameters=[{
                'camera_index': LaunchConfiguration('camera_index'),
                'color_width': 640,
                'color_height': 480,
                'fps': 15,
                'depth_fps': 8,
                'depth_model': LaunchConfiguration('depth_model'),
                'midas_model_type': 'MiDaS_small',
                'prefer_fast_model': True,
                'inference_width': 256,
                'min_depth_m': 0.3,
                'max_depth_m': 5.0,
            }],
        ),
        Node(
            package='rover_perception',
            executable='lane_detector',
            name='lane_detector',
            output='both',
        ),
        Node(
            package='rover_perception',
            executable='obstacle_detector',
            name='obstacle_detector',
            output='both',
        ),
        Node(
            package='rover_perception',
            executable='depth_processor',
            name='depth_processor',
            output='both',
        ),
        Node(
            package='rover_perception',
            executable='qr_scanner',
            name='qr_scanner',
            output='both',
        ),
        # Static TF: base_link -> camera_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_camera_tf',
            arguments=[
                '0.15', '0', '0.3', '0', '0.5', '0',
                'base_link', 'camera_color_optical_frame'
            ],
        ),
        # Video stream for viewing
        Node(
            package='rover_comm',
            executable='video_stream_node',
            name='video_stream_node',
            output='both',
            parameters=[{
                'stream_port': 8080,
                'jpeg_quality': 70,
                'max_fps': 15,
            }],
        ),
        Node(
            package='rover_comm',
            executable='map_stream_node',
            name='map_stream_node',
            output='both',
            parameters=[{
                'map_port': LaunchConfiguration('map_port'),
                'update_rate_hz': 5.0,
            }],
        ),
    ])