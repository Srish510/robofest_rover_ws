import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose, Point, Quaternion
from rover_interfaces.msg import TerrainMap3D, TerrainCell
import numpy as np
import struct


class TerrainMapper(Node):
    def __init__(self):
        super().__init__('terrain_mapper')

        #Parameters
        self.declare_parameter('cell_resolution_m', 0.10)               #Size of each grid cell in meters
        self.declare_parameter('map_size_m', 30.0)                      #Max size of the terrain map (square) in meters
        self.declare_parameter('slope_threshold_rad', 0.35)             #Slope angle (radians) above which terrain is considered a slope
        self.declare_parameter('rough_threshold_m', 0.05)               #Height variation (meters) above which terrain is considered rough
        self.declare_parameter('obstacle_height_min_m', 0.15)           #Minimum height (meters) above which terrain is considered an obstacle
        self.declare_parameter('obstacle_height_max_m', 2.0)            #Maximum height (meters) below which terrain is considered an obstacle
        self.declare_parameter('step_height_m', 0.08)                   #Height difference (meters) above which terrain is considered a step
        self.declare_parameter('min_points_per_cell', 3)                #Minimum number of points in a cell to consider it valid
        self.declare_parameter('update_rate_hz', 2.0)                   #Update rate (Hz)
        self.declare_parameter('max_range_m', 10.0)                     #Maximum range (meters) to consider points for mapping

        #Subscibers
        self.cloud_sub = self.create_subscription(
            PointCloud2, 'rtabmap/cloud_map',
            self.cloud_map_callback, 5)
        self.local_cloud_sub = self.create_subscription(
            PointCloud2, 'perception/point_cloud',
            self.local_cloud_callback, 5)

        #Publishers
        self.terrain3d_pub = self.create_publisher(TerrainMap3D, 'perception/terrain_map_3d', 5)
        self.occupancy_pub = self.create_publisher(OccupancyGrid, 'slam/occupancy_grid', 5)

        # Accumulated terrain grid (persistent between callbacks)
        self._grid = {}  # (gx, gy) -> list of z values
        self._last_global_update = self.get_clock().now()

        rate = self.get_parameter('update_rate_hz').value
        self.update_timer = self.create_timer(1.0 / rate, self._publish_terrain)

        self.get_logger().info('3D Terrain Mapper initialized')

    def _parse_pointcloud2(self, msg):
        """Extract XYZ points from a PointCloud2 message."""
        field_names = [f.name for f in msg.fields]
        if 'x' not in field_names or 'y' not in field_names or 'z' not in field_names:
            return np.empty((0, 3), dtype=np.float32)

        # Find field offsets
        offsets = {}
        for f in msg.fields:
            if f.name in ('x', 'y', 'z'):
                offsets[f.name] = f.offset

        point_step = msg.point_step
        data = np.frombuffer(msg.data, dtype=np.uint8)
        n_points = msg.width * msg.height

        if n_points == 0 or len(data) < n_points * point_step:
            return np.empty((0, 3), dtype=np.float32)

        points = np.zeros((n_points, 3), dtype=np.float32)
        for i, axis in enumerate(('x', 'y', 'z')):
            offset = offsets[axis]
            # Extract float32 at each point's offset
            starts = np.arange(n_points) * point_step + offset
            points[:, i] = np.frombuffer(
                data, dtype=np.float32,
                count=n_points, offset=0
            ).__array_interface__  # fallback below

        # Efficient structured extraction
        points = np.zeros((n_points, 3), dtype=np.float32)
        raw = bytes(msg.data)
        for i in range(n_points):
            base = i * point_step
            points[i, 0] = struct.unpack_from('f', raw, base + offsets['x'])[0]
            points[i, 1] = struct.unpack_from('f', raw, base + offsets['y'])[0]
            points[i, 2] = struct.unpack_from('f', raw, base + offsets['z'])[0]

        # Filter NaN/Inf
        valid = np.isfinite(points).all(axis=1)
        return points[valid]

    def _parse_pointcloud2_fast(self, msg):
        """Fast XYZ extraction assuming standard float32 XYZ layout."""
        n_points = msg.width * msg.height
        if n_points == 0:
            return np.empty((0, 3), dtype=np.float32)

        # Build a dtype from the fields
        offsets = {}
        for f in msg.fields:
            if f.name in ('x', 'y', 'z'):
                offsets[f.name] = f.offset

        point_step = msg.point_step
        raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(n_points, point_step)

        points = np.zeros((n_points, 3), dtype=np.float32)
        for i, axis in enumerate(('x', 'y', 'z')):
            o = offsets[axis]
            points[:, i] = raw[:, o:o+4].view(np.float32).flatten()

        valid = np.isfinite(points).all(axis=1)
        return points[valid]

    def cloud_map_callback(self, msg):
        """Process RTAB-Map's global 3D cloud map."""
        points = self._parse_pointcloud2_fast(msg)
        if len(points) == 0:
            return

        # Reset grid for full global rebuild
        self._grid = {}
        self._ingest_points(points)
        self._last_global_update = self.get_clock().now()
        self.get_logger().debug(
            f'Global cloud map: {len(points)} points ingested')

    def local_cloud_callback(self, msg):
        """Incrementally add local point cloud observations."""
        points = self._parse_pointcloud2_fast(msg)
        if len(points) == 0:
            return
        self._ingest_points(points)

    def _ingest_points(self, points):
        """Add 3D points into the terrain grid."""
        resolution = self.get_parameter('cell_resolution_m').value
        max_range = self.get_parameter('max_range_m').value

        # Filter by range
        dist_xy = np.sqrt(points[:, 0]**2 + points[:, 1]**2)
        mask = dist_xy < max_range
        points = points[mask]

        if len(points) == 0:
            return

        # Discretize into grid cells (XY plane, Z is height)
        gx = np.floor(points[:, 0] / resolution).astype(np.int32)
        gy = np.floor(points[:, 1] / resolution).astype(np.int32)
        gz = points[:, 2]

        for i in range(len(gx)):
            key = (int(gx[i]), int(gy[i]))
            if key not in self._grid:
                self._grid[key] = []
            self._grid[key].append(float(gz[i]))

    def _publish_terrain(self):
        """Analyze terrain grid and publish TerrainMap3D + OccupancyGrid."""
        if not self._grid:
            return

        resolution = self.get_parameter('cell_resolution_m').value
        slope_thresh = self.get_parameter('slope_threshold_rad').value
        rough_thresh = self.get_parameter('rough_threshold_m').value
        obs_h_min = self.get_parameter('obstacle_height_min_m').value
        obs_h_max = self.get_parameter('obstacle_height_max_m').value
        step_h = self.get_parameter('step_height_m').value
        min_pts = self.get_parameter('min_points_per_cell').value

        # Determine grid bounds
        keys = list(self._grid.keys())
        gx_arr = np.array([k[0] for k in keys])
        gy_arr = np.array([k[1] for k in keys])

        gx_min, gx_max = int(gx_arr.min()), int(gx_arr.max())
        gy_min, gy_max = int(gy_arr.min()), int(gy_arr.max())

        width = gx_max - gx_min + 1
        height = gy_max - gy_min + 1

        # Cap map size to prevent memory issues
        map_size = self.get_parameter('map_size_m').value
        max_cells = int(map_size / resolution)
        if width > max_cells or height > max_cells:
            # Crop to centered region
            cx = (gx_min + gx_max) // 2
            cy = (gy_min + gy_max) // 2
            half = max_cells // 2
            gx_min, gx_max = cx - half, cx + half
            gy_min, gy_max = cy - half, cy + half
            width = gx_max - gx_min + 1
            height = gy_max - gy_min + 1

        # Initialize output grids
        terrain_classes = np.zeros(width * height, dtype=np.uint8)
        elevation_grid = np.full(width * height, float('nan'), dtype=np.float32)
        traversability_grid = np.zeros(width * height, dtype=np.uint8)
        slope_grid = np.full(width * height, float('nan'), dtype=np.float32)
        roughness_grid = np.full(width * height, float('nan'), dtype=np.float32)
        confidence_grid = np.zeros(width * height, dtype=np.float32)

        cells = []
        elevations_valid = []

        for (gx, gy), z_vals in self._grid.items():
            # Check bounds
            lx = gx - gx_min
            ly = gy - gy_min
            if lx < 0 or lx >= width or ly < 0 or ly >= height:
                continue

            idx = lx * height + ly
            z_arr = np.array(z_vals, dtype=np.float32)
            n_pts = len(z_arr)

            if n_pts < min_pts:
                continue

            elev = float(np.median(z_arr))
            std_z = float(np.std(z_arr))
            z_range = float(np.max(z_arr) - np.min(z_arr))

            # Confidence based on number of observations
            conf = min(1.0, n_pts / 50.0)

            # Compute slope from neighbors
            slope_val = self._compute_slope(gx, gy, resolution)

            # Classify terrain
            t_class, trav = self._classify_cell(
                elev, std_z, z_range, slope_val,
                obs_h_min, obs_h_max, slope_thresh,
                rough_thresh, step_h)

            elevation_grid[idx] = elev
            terrain_classes[idx] = t_class
            traversability_grid[idx] = trav
            slope_grid[idx] = slope_val
            roughness_grid[idx] = std_z
            confidence_grid[idx] = conf
            elevations_valid.append(elev)

            cell = TerrainCell()
            cell.x = float((gx + 0.5) * resolution)
            cell.y = float((gy + 0.5) * resolution)
            cell.elevation = elev
            cell.slope = slope_val
            cell.roughness = std_z
            cell.terrain_class = t_class
            cell.traversability = trav
            cell.confidence = conf
            cells.append(cell)

        if not elevations_valid:
            return

        now = self.get_clock().now().to_msg()
        elev_arr = np.array(elevations_valid)

        # Publish TerrainMap3D
        msg = TerrainMap3D()
        msg.header.stamp = now
        msg.header.frame_id = 'map'
        msg.resolution = resolution
        msg.width = width
        msg.height = height
        msg.origin = Pose(
            position=Point(
                x=float(gx_min * resolution),
                y=float(gy_min * resolution),
                z=0.0),
            orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))
        msg.terrain_classes = terrain_classes.tolist()
        msg.elevation = elevation_grid.tolist()
        msg.traversability = traversability_grid.tolist()
        msg.cells = cells
        msg.slope = slope_grid.tolist()
        msg.roughness = roughness_grid.tolist()
        msg.confidence = confidence_grid.tolist()
        msg.min_elevation = float(elev_arr.min())
        msg.max_elevation = float(elev_arr.max())
        msg.mean_elevation = float(elev_arr.mean())
        msg.total_points_processed = sum(len(v) for v in self._grid.values())
        self.terrain3d_pub.publish(msg)

        # Publish OccupancyGrid for Nav2 compatibility
        occ_msg = OccupancyGrid()
        occ_msg.header.stamp = now
        occ_msg.header.frame_id = 'map'
        occ_msg.info.resolution = resolution
        occ_msg.info.width = height  # OccupancyGrid: width=columns
        occ_msg.info.height = width  # height=rows
        occ_msg.info.origin = msg.origin

        occ_data = np.full(width * height, -1, dtype=np.int8)
        known = terrain_classes > 0
        occ_data[known] = np.clip(
            (traversability_grid[known].astype(np.int16) * 100) // 255,
            0, 100).astype(np.int8)
        occ_msg.data = occ_data.tolist()
        self.occupancy_pub.publish(occ_msg)

        self.get_logger().debug(
            f'Terrain map: {len(cells)} cells, '
            f'elev [{msg.min_elevation:.2f}, {msg.max_elevation:.2f}]m')

    def _compute_slope(self, gx, gy, resolution):
        """Compute slope at a grid cell using its neighbors."""
        neighbors = [
            (gx - 1, gy), (gx + 1, gy),
            (gx, gy - 1), (gx, gy + 1)]

        center_vals = self._grid.get((gx, gy))
        if not center_vals:
            return 0.0
        center_elev = float(np.median(center_vals))

        max_gradient = 0.0
        for nx, ny in neighbors:
            n_vals = self._grid.get((nx, ny))
            if not n_vals or len(n_vals) < 2:
                continue
            n_elev = float(np.median(n_vals))
            gradient = abs(n_elev - center_elev) / resolution
            max_gradient = max(max_gradient, gradient)

        return float(np.arctan(max_gradient))

    def _classify_cell(self, elev, roughness, z_range, slope,
                       obs_h_min, obs_h_max, slope_thresh,
                       rough_thresh, step_h):
        """Classify terrain and compute traversability cost."""
        # Obstacle: tall features
        if obs_h_min < elev < obs_h_max and z_range > obs_h_min:
            return TerrainMap3D.TERRAIN_OBSTACLE, 254

        # Step: abrupt height change
        if z_range > step_h and roughness > rough_thresh * 2:
            return TerrainMap3D.TERRAIN_STEP, 200

        # Steep slope
        if slope > slope_thresh:
            cost = min(254, int(128 + (slope / slope_thresh) * 80))
            return TerrainMap3D.TERRAIN_SLOPE, cost

        # Rough terrain
        if roughness > rough_thresh:
            cost = min(200, int(64 + (roughness / rough_thresh) * 80))
            return TerrainMap3D.TERRAIN_ROUGH, cost

        # Flat / traversable
        return TerrainMap3D.TERRAIN_FLAT, 10


def main(args=None):
    rclpy.init(args=args)
    node = TerrainMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
