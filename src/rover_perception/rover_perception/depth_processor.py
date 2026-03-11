import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose, Point, Quaternion
from cv_bridge import CvBridge
from rover_interfaces.msg import TerrainMap
import numpy as np


class DepthProcessor(Node):
    def __init__(self):
        super().__init__('depth_processor')

        # Parameters
        self.declare_parameter('depth_scale', 0.001)                    #Convert depth from mm to meters
        self.declare_parameter('min_depth_m', 0.3)                      #Minimum valid distance from camera (m)
        self.declare_parameter('max_depth_m', 5.0)                      #Maximum valid distance from camera (m)
        self.declare_parameter('voxel_size_m', 0.05)                    #Size of voxel for downsampling (m)
        self.declare_parameter('map_resolution_m', 0.05)                #Resolution of the occupancy grid (m) {Size of each cell}
        self.declare_parameter('map_width_m', 10.0)                     #Width of the occupancy grid (m)
        self.declare_parameter('map_height_m', 10.0)                    #Height of the occupancy grid (m)
        self.declare_parameter('obstacle_height_min_m', 0.10)           #Minimum height above ground to consider a cell an obstacle (m)
        self.declare_parameter('obstacle_height_max_m', 2.0)            #Maximum height above ground to consider a cell an obstacle (m)
        self.declare_parameter('camera_height_m', 0.3)                  #Height of the camera above the ground (m)
        self.declare_parameter('enable_slam_output', True)              #Whether to publish SLAM-compatible point cloud for RTAB-Map (camera optical frame with RGB)

        self.bridge = CvBridge()
        self.fx = self.fy = self.cx = self.cy = None
        self._last_color_image = None

        # Subscribers
        self.depth_sub = self.create_subscription(                              #RealSense aligned depth image
            Image, 'camera/aligned_depth/image_raw', self.depth_callback, 10)   
        self.info_sub = self.create_subscription(                               #Camera intrinsic parameters
            CameraInfo, 'camera/color/camera_info', self.info_callback, 10)
        self.color_sub = self.create_subscription(                              #Color image for RGBD point cloud generation
            Image, 'camera/color/image_raw', self.color_callback, 10)

        # Publishers
        self.pc_pub = self.create_publisher(PointCloud2, 'perception/point_cloud', 5)               #Downsampled point cloud in base_link frame
        self.terrain_pub = self.create_publisher(TerrainMap, 'perception/terrain_map', 5)           #Custom terrain map message with elevation and traversability
        self.occupancy_pub = self.create_publisher(OccupancyGrid, 'perception/local_costmap', 5)    #Occupancy grid for Nav2 compatibility (0-100 occupied, -1 unknown)
        self.slam_pc_pub = self.create_publisher(PointCloud2, 'perception/slam_point_cloud', 5)     #Full-resolution XYZRGB point cloud in camera optical frame for RTAB-Map visual SLAM

        self.get_logger().info(f'Depth processor initialized {"(SLAM output enabled)" if self.get_parameter("enable_slam_output").value else "(SLAM output disabled)"}')

    def info_callback(self, msg):
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]

    def color_callback(self, msg):          #Cache latest color image for RGBD point cloud generation
        self._last_color_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def depth_callback(self, msg):          #Process incoming depth image to generate point cloud, terrain map, and occupancy grid
        if self.fx is None:
            return

        depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        scale = self.get_parameter('depth_scale').value
        depth_m = depth_image.astype(np.float32) * scale

        h, w = depth_m.shape
        min_d = self.get_parameter('min_depth_m').value
        max_d = self.get_parameter('max_depth_m').value

        # Generate 3D points
        u = np.arange(w).reshape(1, -1).repeat(h, axis=0).astype(np.float32)        #Column indices
        v = np.arange(h).reshape(-1, 1).repeat(w, axis=1).astype(np.float32)        #Row indices

        valid = (depth_m > min_d) & (depth_m < max_d)
        z = depth_m[valid]
        x = ((u[valid] - self.cx) * z) / self.fx                                #Using pinhole camera model to convert pixel coordinates to 3D points in camera frame
        y = ((v[valid] - self.cy) * z) / self.fy

        if len(z) == 0:
            return

        # Downsample using voxel grid
        voxel_size = self.get_parameter('voxel_size_m').value
        points = np.stack([x, y, z], axis=-1)
        voxel_idx = np.floor(points / voxel_size).astype(np.int32)
        _, unique_indices = np.unique(voxel_idx, axis=0, return_index=True)
        points_ds = points[unique_indices]

        # Publish PointCloud2
        self._publish_pointcloud(points_ds, msg.header.stamp)

        # Publish SLAM-compatible XYZRGB point cloud (in camera optical frame)
        if self.get_parameter('enable_slam_output').value:
            self._publish_slam_pointcloud(
                points, depth_m, u, v, valid, msg.header.stamp)

        # Build terrain map and occupancy grid
        self._build_terrain_map(points_ds, msg.header.stamp)


    def _publish_pointcloud(self, points, stamp):    #Publish downsampled point cloud           
        msg = PointCloud2()
        msg.header.stamp = stamp
        msg.header.frame_id = 'base_link'
        msg.height = 1
        msg.width = len(points)
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = 12 * len(points)
        msg.is_dense = True
        msg.data = points.astype(np.float32).tobytes()
        self.pc_pub.publish(msg)


    def _publish_slam_pointcloud(self, points, depth_m, u, v, valid, stamp):
        """Publish XYZRGB point cloud in camera optical frame for RTAB-Map.

        RTAB-Map expects point clouds in the camera optical frame with RGB
        color data for visual feature matching and loop closure detection.
        """
        color_img = self._last_color_image
        h, w = depth_m.shape

        # Subsample for performance (every 4th pixel)
        step = 4
        u_sub = u[::step, ::step]
        v_sub = v[::step, ::step]
        d_sub = depth_m[::step, ::step]
        valid_sub = (d_sub > self.get_parameter('min_depth_m').value) & \
                    (d_sub < self.get_parameter('max_depth_m').value)

        z = d_sub[valid_sub]
        x = ((u_sub[valid_sub] - self.cx) * z) / self.fx
        y = ((v_sub[valid_sub] - self.cy) * z) / self.fy

        if len(z) == 0:
            return

        n = len(z)

        # Get RGB values if color image is available
        if color_img is not None and color_img.shape[:2] == (h, w):
            pix_u = u_sub[valid_sub].astype(np.int32)
            pix_v = v_sub[valid_sub].astype(np.int32)
            pix_u = np.clip(pix_u, 0, w - 1)
            pix_v = np.clip(pix_v, 0, h - 1)
            b = color_img[pix_v, pix_u, 0].astype(np.uint8)
            g = color_img[pix_v, pix_u, 1].astype(np.uint8)
            r = color_img[pix_v, pix_u, 2].astype(np.uint8)
        else:
            r = np.full(n, 128, dtype=np.uint8)
            g = np.full(n, 128, dtype=np.uint8)
            b = np.full(n, 128, dtype=np.uint8)

        # Pack RGB into a single float32 (RTAB-Map convention)
        rgb_packed = np.zeros(n, dtype=np.float32)
        rgb_int = (r.astype(np.uint32) << 16) | \
                  (g.astype(np.uint32) << 8) | \
                  b.astype(np.uint32)
        rgb_packed = rgb_int.view(np.float32)

        # Build XYZRGB buffer
        cloud_data = np.zeros(n, dtype=[
            ('x', np.float32), ('y', np.float32), ('z', np.float32),
            ('rgb', np.float32)])
        cloud_data['x'] = x.astype(np.float32)
        cloud_data['y'] = y.astype(np.float32)
        cloud_data['z'] = z.astype(np.float32)
        cloud_data['rgb'] = rgb_packed

        msg = PointCloud2()
        msg.header.stamp = stamp
        msg.header.frame_id = 'camera_color_optical_frame'
        msg.height = 1
        msg.width = n
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 16
        msg.row_step = 16 * n
        msg.is_dense = True
        msg.data = cloud_data.tobytes()
        self.slam_pc_pub.publish(msg)



    def _build_terrain_map(self, points, stamp):            #Build local terrain map from point cloud
        resolution = self.get_parameter('map_resolution_m').value
        map_w_m = self.get_parameter('map_width_m').value
        map_h_m = self.get_parameter('map_height_m').value
        obs_h_min = self.get_parameter('obstacle_height_min_m').value
        obs_h_max = self.get_parameter('obstacle_height_max_m').value
        cam_h = self.get_parameter('camera_height_m').value

        map_w = int(map_w_m / resolution)
        map_h = int(map_h_m / resolution)

        # Transform camera coords to base_link ground plane:
        # Camera: x=right, y=down, z=forward
        # Base_link: x=forward, z=up
        ground_x = points[:, 2]  # forward
        ground_y = points[:, 0]  # left-right
        ground_z = cam_h - points[:, 1]  # height above ground

        # Grid indices (camera is at center bottom of map)
        gx = ((ground_x / resolution) + map_h // 2).astype(np.int32)
        gy = ((ground_y / resolution) + map_w // 2).astype(np.int32)

        # Filter to valid grid cells
        valid = (gx >= 0) & (gx < map_h) & (gy >= 0) & (gy < map_w)
        gx = gx[valid]
        gy = gy[valid]
        gz = ground_z[valid]

        # Initialize grids
        terrain_classes = np.zeros(map_h * map_w, dtype=np.uint8)  # unknown
        elevation = np.full(map_h * map_w, float('nan'), dtype=np.float32)
        traversability = np.full(map_h * map_w, 0, dtype=np.uint8)

        # Populate grids
        for i in range(len(gx)):
            idx = gx[i] * map_w + gy[i]
            if idx < 0 or idx >= map_h * map_w:
                continue
            h_val = gz[i]

            if np.isnan(elevation[idx]) or h_val > elevation[idx]:
                elevation[idx] = h_val

            if h_val > obs_h_min and h_val < obs_h_max:
                terrain_classes[idx] = TerrainMap.TERRAIN_OBSTACLE
                traversability[idx] = 254
            elif h_val > 0.03:
                if terrain_classes[idx] != TerrainMap.TERRAIN_OBSTACLE:
                    terrain_classes[idx] = TerrainMap.TERRAIN_ROUGH
                    traversability[idx] = max(traversability[idx], 128)
            else:
                if terrain_classes[idx] == 0:
                    terrain_classes[idx] = TerrainMap.TERRAIN_FLAT
                    traversability[idx] = max(traversability[idx], 10)

        # Publish TerrainMap
        terrain_msg = TerrainMap()
        terrain_msg.header.stamp = stamp
        terrain_msg.header.frame_id = 'base_link'
        terrain_msg.resolution = resolution
        terrain_msg.width = map_w
        terrain_msg.height = map_h
        terrain_msg.origin = Pose(
            position=Point(
                x=-(map_h // 2) * resolution,
                y=-(map_w // 2) * resolution,
                z=0.0
            ),
            orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        )
        terrain_msg.terrain_classes = terrain_classes.tolist()
        terrain_msg.elevation = elevation.tolist()
        terrain_msg.traversability = traversability.tolist()
        self.terrain_pub.publish(terrain_msg)

        # Publish OccupancyGrid (for Nav2 compatibility)
        occ_msg = OccupancyGrid()
        occ_msg.header.stamp = stamp
        occ_msg.header.frame_id = 'base_link'
        occ_msg.info.resolution = resolution
        occ_msg.info.width = map_w
        occ_msg.info.height = map_h
        occ_msg.info.origin = terrain_msg.origin

        # Convert traversability to occupancy [-1, 0-100]
        occ_data = np.full(map_h * map_w, -1, dtype=np.int8)
        known = terrain_classes > 0
        occ_data[known] = np.clip(
            (traversability[known].astype(np.int16) * 100) // 255, 0, 100
        ).astype(np.int8)
        occ_msg.data = occ_data.tolist()
        self.occupancy_pub.publish(occ_msg)


def main(args=None):
    rclpy.init(args=args)
    node = DepthProcessor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
