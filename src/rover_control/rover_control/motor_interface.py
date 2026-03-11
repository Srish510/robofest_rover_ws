"""
Motor interface node - high-level motion controller that fuses lane-keeping,
obstacle avoidance, and navigation commands into a single cmd_vel output.

Supports two modes:
  - Legacy mode (nav2_mode=False): PD lane-keeping + obstacle avoidance + nav commands
    Behavior priority: emergency stop > obstacle avoidance > lane keeping > navigation
  - Nav2 mode (nav2_mode=True): Safety layer for Nav2's cmd_vel output
    Subscribes to nav2/cmd_vel, applies obstacle emergency stop, forwards to cmd_vel
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rover_interfaces.msg import Lane, Obstacle
from std_msgs.msg import Bool
import math
import time


class MotorInterface(Node):
    def __init__(self):
        super().__init__('motor_interface')

        # Parameters
        self.declare_parameter('max_linear_speed', 0.5)
        self.declare_parameter('max_angular_speed', 1.5)
        self.declare_parameter('lane_kp', 1.2)
        self.declare_parameter('lane_kd', 0.3)
        self.declare_parameter('heading_kp', 1.0)
        self.declare_parameter('obstacle_stop_distance', 0.5)
        self.declare_parameter('obstacle_slow_distance', 1.5)
        self.declare_parameter('obstacle_avoidance_angular', 0.8)
        self.declare_parameter('lane_confidence_threshold', 0.3)
        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('cmd_timeout_sec', 0.5)
        self.declare_parameter('nav2_mode', False)

        self.max_linear = self.get_parameter('max_linear_speed').value
        self.max_angular = self.get_parameter('max_angular_speed').value
        self.nav2_mode = self.get_parameter('nav2_mode').value

        # State
        self.latest_lane = None
        self.latest_obstacles = []
        self.nav_cmd = Twist()
        self.nav2_cmd = Twist()
        self.nav2_cmd_time = 0.0
        self.e_stop = False
        self.prev_lateral_error = 0.0
        self.prev_time = time.time()
        self.lane_time = 0.0
        self.obstacle_time = 0.0

        # Subscribers
        self.obstacle_sub = self.create_subscription(
            Obstacle, 'perception/obstacles', self.obstacle_callback, 10)
        self.estop_sub = self.create_subscription(
            Bool, 'rover/e_stop', self.estop_callback, 10)

        if self.nav2_mode:
            # Nav2 mode: subscribe to Nav2's remapped cmd_vel output
            self.nav2_cmd_sub = self.create_subscription(
                Twist, 'nav2/cmd_vel', self.nav2_cmd_callback, 10)
        else:
            # Legacy mode: subscribe to lane and nav commands
            self.lane_sub = self.create_subscription(
                Lane, 'perception/lane', self.lane_callback, 10)
            self.nav_cmd_sub = self.create_subscription(
                Twist, 'nav/cmd_vel', self.nav_cmd_callback, 10)

        # Publisher
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Control loop timer
        rate = self.get_parameter('control_rate_hz').value
        self.timer = self.create_timer(1.0 / rate, self.control_loop)

        mode_str = 'Nav2 safety' if self.nav2_mode else 'legacy'
        self.get_logger().info(f'Motor interface initialized ({mode_str} mode)')

    def lane_callback(self, msg):
        self.latest_lane = msg
        self.lane_time = time.time()

    def obstacle_callback(self, msg):
        now = time.time()
        self.latest_obstacles = [
            obs for obs in self.latest_obstacles
            if (now - self.obstacle_time) < 0.5
        ]
        self.latest_obstacles.append(msg)
        self.obstacle_time = now

    def nav_cmd_callback(self, msg):
        self.nav_cmd = msg

    def nav2_cmd_callback(self, msg):
        self.nav2_cmd = msg
        self.nav2_cmd_time = time.time()

    def estop_callback(self, msg):
        self.e_stop = msg.data
        if self.e_stop:
            self.get_logger().warn('Emergency stop activated!')

    def control_loop(self):
        if self.nav2_mode:
            self._control_loop_nav2()
        else:
            self._control_loop_legacy()

    def _control_loop_nav2(self):
        """Nav2 safety mode: forward Nav2 cmd_vel with obstacle emergency stop."""
        cmd = Twist()
        now = time.time()
        timeout = self.get_parameter('cmd_timeout_sec').value

        # Priority 1: Emergency stop
        if self.e_stop:
            self.cmd_pub.publish(cmd)
            return

        # Priority 2: Obstacle emergency stop (direct sensor, not costmap)
        stop_dist = self.get_parameter('obstacle_stop_distance').value
        slow_dist = self.get_parameter('obstacle_slow_distance').value

        min_distance = float('inf')
        closest_bearing = 0.0
        for obs in self.latest_obstacles:
            if abs(obs.bearing) < math.radians(60) and obs.distance < min_distance:
                min_distance = obs.distance
                closest_bearing = obs.bearing

        if min_distance < stop_dist:
            # Emergency stop — obstacle too close
            self.cmd_pub.publish(cmd)
            return

        # Forward Nav2 cmd_vel with speed modulation for nearby obstacles
        if (now - self.nav2_cmd_time) < timeout:
            cmd = Twist()
            cmd.linear.x = self.nav2_cmd.linear.x
            cmd.angular.z = self.nav2_cmd.angular.z

            # Slow down proportionally if obstacle in slow zone
            if min_distance < slow_dist:
                factor = (min_distance - stop_dist) / (slow_dist - stop_dist)
                cmd.linear.x *= max(0.0, factor)

        # Clamp
        cmd.linear.x = max(-self.max_linear, min(self.max_linear, cmd.linear.x))
        cmd.angular.z = max(-self.max_angular, min(self.max_angular, cmd.angular.z))

        self.cmd_pub.publish(cmd)

    def _control_loop_legacy(self):
        """Legacy mode: PD lane-keeping + obstacle avoidance + nav commands."""
        cmd = Twist()
        now = time.time()
        timeout = self.get_parameter('cmd_timeout_sec').value

        # Priority 1: Emergency stop
        if self.e_stop:
            self.cmd_pub.publish(cmd)  # zero velocity
            return

        # Priority 2: Obstacle avoidance
        stop_dist = self.get_parameter('obstacle_stop_distance').value
        slow_dist = self.get_parameter('obstacle_slow_distance').value
        avoidance_angular = self.get_parameter('obstacle_avoidance_angular').value

        closest_obstacle = None
        min_distance = float('inf')

        for obs in self.latest_obstacles:
            if abs(obs.bearing) < math.radians(60) and obs.distance < min_distance:
                min_distance = obs.distance
                closest_obstacle = obs

        obstacle_factor = 1.0

        if closest_obstacle and min_distance < slow_dist:
            if min_distance < stop_dist:
                cmd.linear.x = 0.0
                if closest_obstacle.bearing > 0:
                    cmd.angular.z = avoidance_angular
                else:
                    cmd.angular.z = -avoidance_angular
                self.cmd_pub.publish(cmd)
                return
            else:
                obstacle_factor = (min_distance - stop_dist) / (slow_dist - stop_dist)
                avoid_z = -math.copysign(
                    avoidance_angular * (1.0 - obstacle_factor) * 0.5,
                    closest_obstacle.bearing
                )

        # Priority 3: Lane keeping
        lane_conf_thresh = self.get_parameter('lane_confidence_threshold').value
        kp = self.get_parameter('lane_kp').value
        kd = self.get_parameter('lane_kd').value
        heading_kp = self.get_parameter('heading_kp').value

        lane_angular = 0.0
        has_lane = False

        if self.latest_lane and (now - self.lane_time) < timeout:
            lane = self.latest_lane
            if lane.confidence > lane_conf_thresh:
                has_lane = True
                dt = now - self.prev_time if self.prev_time else 0.05
                dt = max(dt, 0.01)

                error = lane.lateral_offset
                deriv = (error - self.prev_lateral_error) / dt
                self.prev_lateral_error = error

                lane_angular = kp * error + kd * deriv
                lane_angular += heading_kp * lane.heading_error

        self.prev_time = now

        if has_lane:
            base_linear = max(self.nav_cmd.linear.x, self.max_linear * 0.3)
            cmd.linear.x = base_linear * obstacle_factor
            cmd.angular.z = lane_angular
            if closest_obstacle and min_distance < slow_dist:
                cmd.angular.z += avoid_z
        else:
            cmd.linear.x = self.nav_cmd.linear.x * obstacle_factor * 0.5
            cmd.angular.z = self.nav_cmd.angular.z
            if closest_obstacle and min_distance < slow_dist:
                cmd.angular.z += avoid_z

        # Clamp
        cmd.linear.x = max(-self.max_linear, min(self.max_linear, cmd.linear.x))
        cmd.angular.z = max(-self.max_angular, min(self.max_angular, cmd.angular.z))

        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = MotorInterface()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
