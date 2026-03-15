"""
Lane costmap layer node - publishes a local costmap that marks lane boundaries
as high-cost zones, keeping the rover between lane lines.
This costmap is designed to be overlaid with the obstacle costmap in Nav2.
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose, Point, Quaternion
from rover_interfaces.msg import Lane
import numpy as np


class LaneCostmapLayer(Node):
    def __init__(self):
        super().__init__('lane_costmap_layer')

        # Parameters
        self.declare_parameter('costmap_resolution', 0.05)
        self.declare_parameter('costmap_width_m', 4.0)
        self.declare_parameter('costmap_height_m', 4.0)
        self.declare_parameter('lane_boundary_cost', 100)
        self.declare_parameter('outside_lane_cost', 100)
        self.declare_parameter('inside_lane_cost', 0)
        self.declare_parameter('boundary_width_cells', 3)
        self.declare_parameter('camera_height_m', 0.3)
        self.declare_parameter('publish_rate_hz', 5.0)

        self.latest_lane = None

        # Subscriber
        self.lane_sub = self.create_subscription(
            Lane, 'perception/lane', self.lane_callback, 10)

        # Publisher
        self.costmap_pub = self.create_publisher(
            OccupancyGrid, 'navigation/lane_costmap', 5)

        # Timer
        rate = self.get_parameter('publish_rate_hz').value
        self.timer = self.create_timer(1.0 / rate, self.publish_costmap)

        self.get_logger().info('Lane costmap layer initialized')

    def lane_callback(self, msg):
        self.latest_lane = msg

    def publish_costmap(self):
        lane = self.latest_lane
        if lane is None:
            return

        res = self.get_parameter('costmap_resolution').value
        width_m = self.get_parameter('costmap_width_m').value
        height_m = self.get_parameter('costmap_height_m').value
        boundary_cost = self.get_parameter('lane_boundary_cost').value
        outside_cost = self.get_parameter('outside_lane_cost').value
        inside_cost = self.get_parameter('inside_lane_cost').value
        bw = self.get_parameter('boundary_width_cells').value

        width = int(width_m / res)
        height = int(height_m / res)

        # Initialize costmap: unknown outside lane
        costmap = np.full((height, width), -1, dtype=np.int8)

        if lane.left_detected and lane.right_detected and len(lane.left_boundary) > 1 and len(lane.right_boundary) > 1:

            cam_h = self.get_parameter('camera_height_m').value

            left_ground = self._image_to_ground(lane.left_boundary, cam_h)
            right_ground = self._image_to_ground(lane.right_boundary, cam_h)

            if left_ground and right_ground:
                # For each row in the costmap, interpolate left and right lane x-positions
                for gy in range(height):
                    # Ground y coordinate (forward distance from rover)
                    ground_forward = gy * res

                    # Find lane x at this forward distance
                    left_x = self._interpolate_lane_x(left_ground, ground_forward)
                    right_x = self._interpolate_lane_x(right_ground, ground_forward)

                    if left_x is None or right_x is None:
                        continue

                    # Ensure left < right
                    if left_x > right_x:
                        left_x, right_x = right_x, left_x

                    # Convert to grid coordinates
                    left_gx = int((left_x / res) + width // 2)
                    right_gx = int((right_x / res) + width // 2)

                    for gx in range(width):
                        if gx < left_gx - bw or gx > right_gx + bw:
                            # Outside lane
                            costmap[gy, gx] = min(100, outside_cost)
                        elif abs(gx - left_gx) <= bw or abs(gx - right_gx) <= bw:
                            # On lane boundary
                            costmap[gy, gx] = min(100, boundary_cost)
                        else:
                            # Inside lane
                            costmap[gy, gx] = max(0, inside_cost)

        # Publish OccupancyGrid
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.info.resolution = res
        msg.info.width = width
        msg.info.height = height
        msg.info.origin = Pose(
            position=Point(x=0.0, y=-(width // 2) * res, z=0.0),
            orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        )
        msg.data = costmap.flatten().tolist()
        self.costmap_pub.publish(msg)

    def _image_to_ground(self, boundary_points, camera_height):
        """Convert image-space boundary points to ground-plane (forward, lateral)."""
        ground_points = []
        # Rough approximation: forward distance ~ camera_height * image_height / (v - vanishing_point)
        # lateral offset ~ forward_distance * (u - image_center) / focal_length
        # Using simplified model
        for pt in boundary_points:
            u = pt.x  # pixel x
            v = pt.y  # pixel y (from top)
            # Avoid division by zero near horizon
            if v < 10:
                continue
            # Approximate ground projection
            forward = camera_height * 480.0 / max(v, 1)  # rough approximation
            lateral = (u - 320.0) * forward / 500.0  # rough with assumed focal length
            if 0 < forward < 10.0:  # reasonable range
                ground_points.append((forward, lateral))
        return ground_points

    def _interpolate_lane_x(self, ground_points, forward_dist):
        """Interpolate lateral position at a given forward distance."""
        if not ground_points:
            return None

        # Sort by forward distance
        sorted_pts = sorted(ground_points, key=lambda p: p[0])

        # Find bracketing points
        for i in range(len(sorted_pts) - 1):
            f0, x0 = sorted_pts[i]
            f1, x1 = sorted_pts[i + 1]
            if f0 <= forward_dist <= f1:
                t = (forward_dist - f0) / (f1 - f0) if (f1 - f0) > 0 else 0.5
                return x0 + t * (x1 - x0)

        # Extrapolate from nearest
        if forward_dist <= sorted_pts[0][0]:
            return sorted_pts[0][1]
        return sorted_pts[-1][1]


def main(args=None):
    rclpy.init(args=args)
    node = LaneCostmapLayer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
