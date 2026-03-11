"""
Planner helper node - assists with path planning by merging obstacle and lane
costmaps into a unified local costmap for the motor interface. Also implements 
reactive obstacle avoidance using the VFH (Vector Field Histogram) approach.
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Twist
from rover_interfaces.msg import Obstacle
import numpy as np
import math


class PlannerHelper(Node):
    def __init__(self):
        super().__init__('planner_helper')

        # Parameters
        self.declare_parameter('num_sectors', 36)
        self.declare_parameter('sector_threshold', 0.3)
        self.declare_parameter('max_obstacle_range', 3.0)
        self.declare_parameter('safety_margin_m', 0.3)
        self.declare_parameter('preferred_direction_rad', 0.0)  # straight ahead
        self.declare_parameter('publish_rate_hz', 10.0)

        self.num_sectors = self.get_parameter('num_sectors').value
        self.sector_size = 2.0 * math.pi / self.num_sectors

        # Polar histogram of obstacle density
        self.histogram = np.zeros(self.num_sectors)
        self.latest_obstacle_costmap = None
        self.latest_lane_costmap = None

        # Subscribers
        self.obstacle_sub = self.create_subscription(
            Obstacle, 'perception/obstacles', self.obstacle_callback, 10)
        self.obstacle_costmap_sub = self.create_subscription(
            OccupancyGrid, 'perception/local_costmap', self.obstacle_costmap_callback, 5)
        self.lane_costmap_sub = self.create_subscription(
            OccupancyGrid, 'navigation/lane_costmap', self.lane_costmap_callback, 5)

        # Publishers
        self.merged_costmap_pub = self.create_publisher(
            OccupancyGrid, 'navigation/merged_costmap', 5)
        self.avoidance_cmd_pub = self.create_publisher(
            Twist, 'navigation/avoidance_cmd', 10)

        # Timer for VFH computation
        rate = self.get_parameter('publish_rate_hz').value
        self.timer = self.create_timer(1.0 / rate, self.compute_and_publish)

        self.get_logger().info('Planner helper initialized')

    def obstacle_callback(self, msg):
        """Update polar histogram with obstacle data."""
        max_range = self.get_parameter('max_obstacle_range').value
        if msg.distance > max_range:
            return

        # Map bearing to sector
        bearing = msg.bearing  # radians, 0 = ahead
        sector = int(((bearing + math.pi) / self.sector_size) % self.num_sectors)

        # Weight by inverse distance (closer = more dangerous)
        weight = 1.0 - (msg.distance / max_range)
        # Dynamic obstacles get extra weight
        if msg.is_dynamic:
            weight *= 1.5
        self.histogram[sector] = max(self.histogram[sector], weight)

    def obstacle_costmap_callback(self, msg):
        self.latest_obstacle_costmap = msg

    def lane_costmap_callback(self, msg):
        self.latest_lane_costmap = msg

    def compute_and_publish(self):
        """Run VFH and merge costmaps."""
        # Decay histogram over time
        self.histogram *= 0.8

        # VFH: Find best gap
        threshold = self.get_parameter('sector_threshold').value
        preferred = self.get_parameter('preferred_direction_rad').value

        # Binary histogram: True = blocked
        blocked = self.histogram > threshold

        # Find free sectors
        free_sectors = np.where(~blocked)[0]

        avoidance_cmd = Twist()

        if len(free_sectors) > 0:
            # Convert sectors to angles
            angles = (free_sectors.astype(float) * self.sector_size) - math.pi

            # Find free sector closest to preferred direction
            angle_diffs = np.abs(angles - preferred)
            best_idx = np.argmin(angle_diffs)
            best_angle = angles[best_idx]

            # Generate avoidance steering command
            avoidance_cmd.angular.z = best_angle * 0.5  # proportional steering
            avoidance_cmd.linear.x = 0.3 * (1.0 - np.max(self.histogram) * 0.5)
        else:
            # All sectors blocked - stop and turn
            avoidance_cmd.linear.x = 0.0
            # Turn toward least-blocked direction
            min_sector = np.argmin(self.histogram)
            min_angle = (min_sector * self.sector_size) - math.pi
            avoidance_cmd.angular.z = math.copysign(0.5, min_angle)

        self.avoidance_cmd_pub.publish(avoidance_cmd)

        # Merge costmaps
        self._merge_costmaps()

    def _merge_costmaps(self):
        """Merge obstacle and lane costmaps into unified local costmap."""
        obs = self.latest_obstacle_costmap
        lane = self.latest_lane_costmap

        if obs is None:
            return

        # Use obstacle costmap as base
        merged = OccupancyGrid()
        merged.header = obs.header
        merged.info = obs.info

        obs_data = np.array(obs.data, dtype=np.int8)

        if lane is not None and lane.info.width == obs.info.width and lane.info.height == obs.info.height:
            lane_data = np.array(lane.data, dtype=np.int8)
            # Merge: take maximum cost (most restrictive)
            merged_data = np.where(
                (obs_data >= 0) & (lane_data >= 0),
                np.maximum(obs_data, lane_data),
                np.where(obs_data >= 0, obs_data, lane_data)
            )
            merged.data = merged_data.astype(np.int8).tolist()
        else:
            merged.data = obs.data

        self.merged_costmap_pub.publish(merged)


def main(args=None):
    rclpy.init(args=args)
    node = PlannerHelper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()