"""
Goal manager node - manages waypoint navigation, checkpoint sequencing,
and state machine for autonomous traversal between checkpoints.

Supports two modes:
  - Legacy mode (use_nav2=False): Publishes Twist to nav/cmd_vel (default)
  - Nav2 mode (use_nav2=True): Sends goals to Nav2 via NavigateToPose action

States:
  IDLE -> NAVIGATING -> CHECKPOINT_SCAN -> NAVIGATING -> ...
  Any state -> OBSTACLE_AVOIDANCE -> Previous state

Nav2 mode accepts goal poses via:
  - /goal_pose topic (geometry_msgs/PoseStamped) — e.g. from rviz2
  - /waypoints topic (geometry_msgs/PoseArray) — sequence of goals

Nav2 can also run fully standalone (no rviz2/external goal sources) using
internally configured auto-waypoints.
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist, PoseStamped, PoseArray
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Bool
from rover_interfaces.msg import Lane, Obstacle, RoverStatus
import math


class GoalManager(Node):
    # States
    IDLE = 0
    NAVIGATING = 1
    OBSTACLE_AVOIDANCE = 2
    CHECKPOINT_SCAN = 3

    def __init__(self):
        super().__init__('goal_manager')

        # Parameters
        self.declare_parameter('default_forward_speed', 0.3)
        self.declare_parameter('checkpoint_approach_speed', 0.15)
        self.declare_parameter('checkpoint_stop_duration_sec', 3.0)
        self.declare_parameter('no_lane_timeout_sec', 2.0)
        self.declare_parameter('obstacle_critical_dist', 0.4)
        self.declare_parameter('control_rate_hz', 10.0)
        self.declare_parameter('use_nav2', False)
        self.declare_parameter('auto_start_nav2_goals', True)
        self.declare_parameter('nav2_goal_frame', 'map')
        self.declare_parameter('nav2_auto_waypoints_xy', [
            1.5, 0.0,
            2.5, 0.8,
            3.2, 0.0,
            2.5, -0.8,
            1.5, 0.0,
        ])
        self.declare_parameter('nav2_loop_waypoints', True)
        self.declare_parameter('nav2_start_delay_sec', 4.0)

        self.use_nav2 = self.get_parameter('use_nav2').value

        # State
        self.state = self.IDLE
        self.prev_state = self.IDLE
        self.current_pose = None
        self.latest_lane = None
        self.closest_obstacle_dist = float('inf')
        self.closest_obstacle_bearing = 0.0
        self.last_lane_time = 0.0
        self.checkpoint_stop_start = None
        self.registered_checkpoints = []

        # Nav2 state
        self._nav2_goal_handle = None
        self._nav2_active = False
        self._waypoint_queue = []
        self._waypoint_template = []
        self._pending_goal = None  # goal to resume after checkpoint scan
        self._autonomous_started = False

        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry, 'odom', self.odom_callback, 10)
        self.lane_sub = self.create_subscription(
            Lane, 'perception/lane', self.lane_callback, 10)
        self.obstacle_sub = self.create_subscription(
            Obstacle, 'perception/obstacles', self.obstacle_callback, 10)
        self.qr_sub = self.create_subscription(
            String, 'perception/qr_detected', self.qr_callback, 10)

        # Publishers
        self.nav_cmd_pub = self.create_publisher(Twist, 'nav/cmd_vel', 10)
        self.state_pub = self.create_publisher(String, 'rover/state', 10)
        self.estop_pub = self.create_publisher(Bool, 'rover/e_stop', 10)

        # Nav2 action client
        if self.use_nav2:
            from nav2_msgs.action import NavigateToPose
            self._NavigateToPose = NavigateToPose
            self._nav2_client = ActionClient(
                self, NavigateToPose, 'navigate_to_pose')

            # Accept goals from rviz2 or external sources
            self.goal_pose_sub = self.create_subscription(
                PoseStamped, 'goal_pose', self._goal_pose_callback, 10)
            self.waypoints_sub = self.create_subscription(
                PoseArray, 'waypoints', self._waypoints_callback, 10)

            self._load_auto_waypoints_from_params()
            auto_start = bool(self.get_parameter('auto_start_nav2_goals').value)
            if auto_start and self._waypoint_template:
                delay = max(0.0, float(self.get_parameter('nav2_start_delay_sec').value))
                self._autostart_timer = self.create_timer(delay, self._autostart_nav2_goals)
                self.get_logger().info(
                    f'Goal manager initialized (Nav2 mode) — auto-starting '
                    f'{len(self._waypoint_template)} internal waypoints in {delay:.1f}s')
            else:
                self.get_logger().info(
                    'Goal manager initialized (Nav2 mode) — waiting for goals')
        else:
            # Legacy mode: start navigating immediately
            self.state = self.NAVIGATING
            self.get_logger().info(
                'Goal manager initialized — starting autonomous navigation')

        # Control loop
        rate = self.get_parameter('control_rate_hz').value
        self.timer = self.create_timer(1.0 / rate, self.control_loop)

    # ─── Callbacks ───

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

    def lane_callback(self, msg):
        self.latest_lane = msg
        self.last_lane_time = self.get_clock().now().nanoseconds / 1e9

    def obstacle_callback(self, msg):
        if abs(msg.bearing) < math.radians(45):
            if msg.distance < self.closest_obstacle_dist:
                self.closest_obstacle_dist = msg.distance
                self.closest_obstacle_bearing = msg.bearing

    def qr_callback(self, msg):
        """QR code detected — transition to checkpoint scan state."""
        if msg.data not in self.registered_checkpoints:
            self.registered_checkpoints.append(msg.data)
            self.get_logger().info(f'Checkpoint scanned: {msg.data} '
                                   f'(total: {len(self.registered_checkpoints)})')
            if self.state == self.NAVIGATING:
                self.prev_state = self.state
                self.state = self.CHECKPOINT_SCAN
                self.checkpoint_stop_start = self.get_clock().now().nanoseconds / 1e9
                if self.use_nav2:
                    self._cancel_nav2_goal()

    # ─── Nav2 Goal Management ───

    def _goal_pose_callback(self, msg):
        """Receive a single goal pose (e.g. from rviz2 Nav2 Goal button)."""
        self.get_logger().info(
            f'Received goal: ({msg.pose.position.x:.2f}, '
            f'{msg.pose.position.y:.2f})')
        self._waypoint_queue.clear()
        self._send_nav2_goal(msg)

    def _waypoints_callback(self, msg):
        """Receive a sequence of waypoints to visit."""
        self._waypoint_queue.clear()
        for pose in msg.poses:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose = pose
            self._waypoint_queue.append(ps)
        self.get_logger().info(
            f'Received {len(self._waypoint_queue)} waypoints')
        self._send_next_waypoint()

    def _load_auto_waypoints_from_params(self):
        """Load internal Nav2 waypoints from flat [x1, y1, x2, y2, ...] parameter."""
        frame = str(self.get_parameter('nav2_goal_frame').value)
        values = list(self.get_parameter('nav2_auto_waypoints_xy').value)
        if len(values) < 2 or len(values) % 2 != 0:
            self.get_logger().warn(
                'nav2_auto_waypoints_xy must have an even count >= 2; auto-start disabled')
            self._waypoint_template = []
            return

        waypoints = []
        for i in range(0, len(values), 2):
            ps = PoseStamped()
            ps.header.frame_id = frame
            ps.pose.position.x = float(values[i])
            ps.pose.position.y = float(values[i + 1])
            ps.pose.position.z = 0.0
            ps.pose.orientation.x = 0.0
            ps.pose.orientation.y = 0.0
            ps.pose.orientation.z = 0.0
            ps.pose.orientation.w = 1.0
            waypoints.append(ps)

        self._waypoint_template = waypoints

    def _autostart_nav2_goals(self):
        """Start internal waypoint mission when Nav2 mode is enabled."""
        if self._autonomous_started:
            return
        self._autonomous_started = True
        if hasattr(self, '_autostart_timer'):
            self._autostart_timer.cancel()

        self._refill_waypoint_queue_if_needed()
        if self.state == self.IDLE and self._waypoint_queue:
            self.get_logger().info('Starting autonomous Nav2 waypoint mission')
            self._send_next_waypoint()

    def _refill_waypoint_queue_if_needed(self):
        if self._waypoint_queue:
            return
        if not self._waypoint_template:
            return
        self._waypoint_queue = [wp for wp in self._waypoint_template]

    def _send_next_waypoint(self):
        """Pop the next waypoint from the queue and send it to Nav2."""
        if not self._waypoint_queue and bool(self.get_parameter('nav2_loop_waypoints').value):
            self._refill_waypoint_queue_if_needed()

        if self._waypoint_queue:
            goal = self._waypoint_queue.pop(0)
            self._send_nav2_goal(goal)
        else:
            self.state = self.IDLE
            self._nav2_active = False
            self.get_logger().info('All waypoints completed')

    def _send_nav2_goal(self, pose_stamped):
        """Send a NavigateToPose goal to Nav2."""
        if not self._nav2_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 action server not available')
            return

        goal_msg = self._NavigateToPose.Goal()
        goal_msg.pose = pose_stamped
        goal_msg.behavior_tree = ''

        self._pending_goal = pose_stamped
        self.state = self.NAVIGATING
        self._nav2_active = True

        future = self._nav2_client.send_goal_async(
            goal_msg, feedback_callback=self._nav2_feedback_cb)
        future.add_done_callback(self._nav2_goal_response_cb)

    def _nav2_goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Nav2 goal rejected')
            self._nav2_active = False
            return

        self._nav2_goal_handle = goal_handle
        self.get_logger().info('Nav2 goal accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._nav2_result_cb)

    def _nav2_feedback_cb(self, feedback_msg):
        fb = feedback_msg.feedback
        self.get_logger().debug(
            f'Nav2 progress: ETA {fb.estimated_time_remaining.sec}s, '
            f'dist {fb.distance_remaining:.2f}m',
            throttle_duration_sec=5.0)

    def _nav2_result_cb(self, future):
        result = future.result()
        status = result.status
        self._nav2_goal_handle = None
        self._nav2_active = False

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Nav2 goal reached')
            self._pending_goal = None
            self._send_next_waypoint()
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info('Nav2 goal canceled')
        else:
            self.get_logger().warn(f'Nav2 goal failed with status: {status}')
            self._pending_goal = None
            self._send_next_waypoint()

    def _cancel_nav2_goal(self):
        """Cancel the active Nav2 goal (e.g. for checkpoint scan)."""
        if self._nav2_goal_handle is not None:
            self.get_logger().info('Canceling Nav2 goal for checkpoint scan')
            self._nav2_goal_handle.cancel_goal_async()

    # ─── Control Loop ───

    def control_loop(self):
        cmd = Twist()
        now = self.get_clock().now().nanoseconds / 1e9

        # Publish current state
        state_names = {
            0: 'IDLE', 1: 'NAVIGATING',
            2: 'OBSTACLE_AVOIDANCE', 3: 'CHECKPOINT_SCAN'}
        state_msg = String()
        state_msg.data = state_names.get(self.state, 'UNKNOWN')
        self.state_pub.publish(state_msg)

        if self.use_nav2:
            self._control_loop_nav2(cmd, now)
        else:
            self._control_loop_legacy(cmd, now)

        # Reset obstacle tracking for next cycle
        self.closest_obstacle_dist = float('inf')

    def _control_loop_nav2(self, cmd, now):
        """Control loop for Nav2 mode — Nav2 handles path following."""
        if self.state == self.IDLE:
            return

        elif self.state == self.NAVIGATING:
            # Nav2 controller handles cmd_vel.
            # We only monitor for checkpoint scans (handled in qr_callback).
            pass

        elif self.state == self.CHECKPOINT_SCAN:
            stop_duration = self.get_parameter('checkpoint_stop_duration_sec').value
            if self.checkpoint_stop_start and (now - self.checkpoint_stop_start) > stop_duration:
                self.checkpoint_stop_start = None
                self.get_logger().info('Checkpoint registered — resuming navigation')
                # Re-send the goal that was interrupted
                if self._pending_goal is not None:
                    self._send_nav2_goal(self._pending_goal)
                else:
                    self._send_next_waypoint()

    def _control_loop_legacy(self, cmd, now):
        """Control loop for legacy mode — direct cmd_vel publishing."""
        default_speed = self.get_parameter('default_forward_speed').value
        obstacle_critical = self.get_parameter('obstacle_critical_dist').value
        no_lane_timeout = self.get_parameter('no_lane_timeout_sec').value

        if self.state == self.IDLE:
            self.nav_cmd_pub.publish(cmd)
            return

        elif self.state == self.NAVIGATING:
            if self.closest_obstacle_dist < obstacle_critical:
                self.prev_state = self.state
                self.state = self.OBSTACLE_AVOIDANCE
                self.get_logger().info('Obstacle detected — switching to avoidance')

            if self.latest_lane and self.latest_lane.confidence > 0.3:
                cmd.linear.x = default_speed
            elif (now - self.last_lane_time) < no_lane_timeout:
                cmd.linear.x = default_speed * 0.5
            else:
                cmd.linear.x = default_speed * 0.2
                self.get_logger().warn(
                    'No lane detected — crawling', throttle_duration_sec=5.0)

        elif self.state == self.OBSTACLE_AVOIDANCE:
            approach_speed = self.get_parameter('checkpoint_approach_speed').value
            cmd.linear.x = approach_speed

            if self.closest_obstacle_dist > obstacle_critical * 2.0:
                self.state = self.prev_state
                self.get_logger().info('Obstacle cleared — resuming navigation')

        elif self.state == self.CHECKPOINT_SCAN:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

            stop_duration = self.get_parameter('checkpoint_stop_duration_sec').value
            if self.checkpoint_stop_start and (now - self.checkpoint_stop_start) > stop_duration:
                self.state = self.NAVIGATING
                self.checkpoint_stop_start = None
                self.get_logger().info('Checkpoint registered — resuming navigation')

        self.nav_cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = GoalManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
