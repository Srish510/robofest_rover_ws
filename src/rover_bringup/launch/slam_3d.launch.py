"""
3D Terrain Mapping with SLAM launch file.
Launches RTAB-Map for visual SLAM + 3D mapping with the rover's RealSense D455,
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
    SetEnvironmentVariable, TimerAction)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration('use_sim_time')
    localization = LaunchConfiguration('localization')
    rviz = LaunchConfiguration('rviz')
    delete_db = LaunchConfiguration('delete_db')
    video_port = LaunchConfiguration('video_port')
    map_port = LaunchConfiguration('map_port')
    telemetry_port = LaunchConfiguration('telemetry_port')
    camera_source = LaunchConfiguration('camera_source')
    camera_index = LaunchConfiguration('camera_index')
    depth_model = LaunchConfiguration('depth_model')
    rtabmap_config = os.path.join(
        get_package_share_directory('rover_bringup'), 'config', 'rtabmap.yaml')
    rover_config = os.path.join(
        get_package_share_directory('rover_bringup'), 'config', 'rover_params.yaml')
    urdf_path = os.path.join(
        get_package_share_directory('rover_description'), 'urdf', 'rover.urdf')
    db_path = os.path.expanduser('~/.ros/rtabmap_rover.db')

    with open(urdf_path, 'r', encoding='utf-8') as urdf_file:
        robot_description = urdf_file.read()

    # Delete database if requested (for fresh mapping)
    if context.perform_substitution(delete_db) == 'true' and os.path.exists(db_path):
        os.remove(db_path)

    # ─── RealSense Camera ───
    realsense = Node(
        package='rover_perception',
        executable='realsense_node',
        name='realsense_node',
        output='screen',
        condition=IfCondition(PythonExpression([
            "'", camera_source, "' == 'realsense'"
        ])),
        parameters=[{
            'use_sim_time': use_sim_time,
            'color_width': 640,
            'color_height': 480,
            'depth_width': 640,
            'depth_height': 480,
            'fps': 30,
            'enable_imu': True,
            'align_depth': True,
        }])

    laptop_depth_camera = Node(
        package='rover_perception',
        executable='laptop_depth_camera',
        name='laptop_depth_camera',
        output='screen',
        condition=IfCondition(PythonExpression([
            "'", camera_source, "' == 'laptop'"
        ])),
        parameters=[{
            'use_sim_time': use_sim_time,
            'camera_index': camera_index,
            'color_width': 640,
            'color_height': 480,
            'fps': 15,
            'depth_fps': 8,
            'depth_model': depth_model,
            'midas_model_type': 'MiDaS_small',
            'prefer_fast_model': True,
            'inference_width': 256,
            'min_depth_m': 0.3,
            'max_depth_m': 5.0,
        }])

    # ─── URDF TF: links and joints from rover_description ───
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description,
        }],
    )

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
    is_localization = context.perform_substitution(localization) == 'true'
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
                'Mem/IncrementalMemory': 'false' if is_localization else 'true',
                'Mem/InitWMWithAllNodes': 'true' if is_localization else 'false',
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
                'frame_id': 'camera_link',
                'odom_frame_id': 'odom',
                'publish_tf': True,
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
            ('odom', 'odom'),
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

    # ─── GCS Communication Nodes (delayed start) ───
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

    return [
        realsense,
        laptop_depth_camera,
        robot_state_publisher_node,
        camera_optical_tf,
        camera_depth_optical_tf,
        camera_imu_tf,
        rtabmap_slam,
        rtabmap_odom,
        depth_processor,
        terrain_mapper,
        rtabmap_viz,
        comm_group,
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
        DeclareLaunchArgument(
            'video_port', default_value='8080',
            description='Video stream HTTP port'),
        DeclareLaunchArgument(
            'map_port', default_value='8081',
            description='Map stream HTTP port'),
        DeclareLaunchArgument(
            'telemetry_port', default_value='8082',
            description='Telemetry stream HTTP port'),
        DeclareLaunchArgument(
            'camera_source', default_value='realsense',
            description='Camera source: realsense or laptop'),
        DeclareLaunchArgument(
            'camera_index', default_value='0',
            description='Laptop webcam index for laptop mode'),
        DeclareLaunchArgument(
            'depth_model', default_value='midas',
            description='Monocular depth model for laptop mode: midas or depth_anything'),

        OpaqueFunction(function=launch_setup),
    ])