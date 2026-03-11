"""
3D Terrain Mapping with SLAM launch file.
Launches RTAB-Map for visual SLAM + 3D mapping with the rover's RealSense D435i,
along with the terrain mapper node for terrain analysis.

Usage:
  ros2 launch rover_bringup slam_3d.launch.py
  ros2 launch rover_bringup slam_3d.launch.py localization:=true   # localization-only mode
  ros2 launch rover_bringup slam_3d.launch.py rviz:=true           # with visualization
  ros2 launch rover_bringup slam_3d.launch.py delete_db:=true      # fresh map
"""
import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, GroupAction, OpaqueFunction,
    SetEnvironmentVariable)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration('use_sim_time')
    localization = LaunchConfiguration('localization')
    rviz = LaunchConfiguration('rviz')
    delete_db = LaunchConfiguration('delete_db')
    rtabmap_config = os.path.join(
        get_package_share_directory('rover_bringup'), 'config', 'rtabmap.yaml')
    rover_config = os.path.join(
        get_package_share_directory('rover_bringup'), 'config', 'rover_params.yaml')
    db_path = os.path.expanduser('~/.ros/rtabmap_rover.db')

    # Delete database if requested (for fresh mapping)
    if context.perform_substitution(delete_db) == 'true' and os.path.exists(db_path):
        os.remove(db_path)

    # ─── Static TF: base_link → camera_link ───
    # Camera is mounted 0.15m forward, 0.3m up from base_link, tilted down
    camera_base_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_camera_tf',
        arguments=[
            '--x', '0.15', '--y', '0.0', '--z', '0.3',
            '--roll', '0.0', '--pitch', '-0.5', '--yaw', '0.0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'camera_link'],
        parameters=[{'use_sim_time': use_sim_time}])

    # camera_link → camera_color_optical_frame (RealSense convention)
    camera_optical_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_to_optical_tf',
        arguments=[
            '--x', '0.0', '--y', '0.0', '--z', '0.0',
            '--roll', '-1.5708', '--pitch', '0.0', '--yaw', '-1.5708',
            '--frame-id', 'camera_link',
            '--child-frame-id', 'camera_color_optical_frame'],
        parameters=[{'use_sim_time': use_sim_time}])

    # camera_link → camera_depth_optical_frame (same as color for aligned depth)
    camera_depth_optical_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_to_depth_optical_tf',
        arguments=[
            '--x', '0.0', '--y', '0.0', '--z', '0.0',
            '--roll', '-1.5708', '--pitch', '0.0', '--yaw', '-1.5708',
            '--frame-id', 'camera_link',
            '--child-frame-id', 'camera_depth_optical_frame'],
        parameters=[{'use_sim_time': use_sim_time}])

    # camera_link → camera_imu_optical_frame
    camera_imu_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_to_imu_tf',
        arguments=[
            '--x', '0.0', '--y', '0.0', '--z', '0.0',
            '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
            '--frame-id', 'camera_link',
            '--child-frame-id', 'camera_imu_optical_frame'],
        parameters=[{'use_sim_time': use_sim_time}])

    # ─── RTAB-Map SLAM Node ───
    rtabmap_slam = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[
            rtabmap_config,
            {
                'use_sim_time': use_sim_time,
                'database_path': db_path,
                'Mem/IncrementalMemory':
                    PythonExpression(["'false' if '", localization, "' == 'true' else 'true'"]),
                'Mem/InitWMWithAllNodes':
                    PythonExpression(["'true' if '", localization, "' == 'true' else 'false'"]),
            }],
        remappings=[
            ('rgb/image', 'camera/color/image_raw'),
            ('rgb/camera_info', 'camera/color/camera_info'),
            ('depth/image', 'camera/aligned_depth/image_raw'),
            ('odom', 'odom'),
            ('imu', 'camera/imu'),
            ('grid_map', 'rtabmap/grid_map'),
            ('cloud_map', 'rtabmap/cloud_map'),
            ('mapData', 'rtabmap/mapData'),
        ],
        arguments=['--delete_db_on_start'] if
            context.perform_substitution(delete_db) == 'true' else [])

    # ─── RTAB-Map Visual Odometry (optional, fuses with wheel odom) ───
    rtabmap_odom = Node(
        package='rtabmap_odom',
        executable='rgbd_odometry',
        name='rtabmap_rgbd_odom',
        output='screen',
        parameters=[
            rtabmap_config,
            {
                'use_sim_time': use_sim_time,
                'frame_id': 'base_link',
                'odom_frame_id': 'odom',
                'publish_tf': False,  # wheel odom publishes TF
                'wait_for_transform': 0.2,
                'approx_sync': True,
                'queue_size': 10,
                'Odom/Strategy': '0',
                'Vis/MinInliers': '15',
            }],
        remappings=[
            ('rgb/image', 'camera/color/image_raw'),
            ('rgb/camera_info', 'camera/color/camera_info'),
            ('depth/image', 'camera/aligned_depth/image_raw'),
            ('odom', 'rtabmap/visual_odom'),
        ])

    # ─── Depth Processor (SLAM-aware) ───
    depth_processor = Node(
        package='rover_perception',
        executable='depth_processor',
        name='depth_processor',
        output='screen',
        parameters=[
            rover_config,
            {
                'use_sim_time': use_sim_time,
                'enable_slam_output': True,
            }])

    # ─── 3D Terrain Mapper ───
    terrain_mapper = Node(
        package='rover_perception',
        executable='terrain_mapper',
        name='terrain_mapper',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'cell_resolution_m': 0.10,
            'map_size_m': 30.0,
            'slope_threshold_rad': 0.35,
            'rough_threshold_m': 0.05,
            'obstacle_height_min_m': 0.15,
            'obstacle_height_max_m': 2.0,
            'step_height_m': 0.08,
            'min_points_per_cell': 3,
            'update_rate_hz': 2.0,
            'max_range_m': 10.0,
        }])

    # ─── RTAB-Map Visualization (optional) ───
    rtabmap_viz = Node(
        package='rtabmap_viz',
        executable='rtabmap_viz',
        name='rtabmap_viz',
        output='screen',
        condition=IfCondition(rviz),
        parameters=[
            rtabmap_config,
            {'use_sim_time': use_sim_time}],
        remappings=[
            ('rgb/image', 'camera/color/image_raw'),
            ('rgb/camera_info', 'camera/color/camera_info'),
            ('depth/image', 'camera/aligned_depth/image_raw'),
            ('odom', 'odom'),
        ])

    return [
        camera_base_tf,
        camera_optical_tf,
        camera_depth_optical_tf,
        camera_imu_tf,
        rtabmap_slam,
        rtabmap_odom,
        depth_processor,
        terrain_mapper,
        rtabmap_viz,
    ]


def generate_launch_description():
    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),

        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation clock'),
        DeclareLaunchArgument(
            'localization', default_value='false',
            description='Localization-only mode (use existing map)'),
        DeclareLaunchArgument(
            'rviz', default_value='false',
            description='Launch RTAB-Map visualization'),
        DeclareLaunchArgument(
            'delete_db', default_value='false',
            description='Delete existing map database and start fresh'),

        OpaqueFunction(function=launch_setup),
    ])
