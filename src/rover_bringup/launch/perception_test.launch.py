"""
Perception-only launch file for testing camera and detection pipeline
without ESP32 or motor control.
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rover_perception',
            executable='realsense_node',
            name='realsense_node',
            output='both',
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
    ])