
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rover_interfaces.msg import RoverStatus
from std_msgs.msg import Float32MultiArray
import serial
import json
import threading
import math


class ESP32Bridge(Node):
    def __init__(self):
        super().__init__('esp32_bridge')

        # Parameters
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('serial_timeout', 0.05)
        self.declare_parameter('wheelbase_m', 0.3)       # Front-to-rear axle distance
        self.declare_parameter('track_width_m', 0.3)     # Left-to-right wheel distance
        self.declare_parameter('max_motor_speed', 1.0)
        self.declare_parameter('max_servo_angle_rad', 0.5)  # Max steering angle per wheel
        self.declare_parameter('publish_rate_hz', 50.0)
        self.declare_parameter('drive_mode', 'differential')  # 'servo' or 'differential'
        self.declare_parameter('expect_esp32_rx', False)

        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('baud_rate').value
        timeout = self.get_parameter('serial_timeout').value

        # Open serial connection
        try:
            self.serial = serial.Serial(port, baud, timeout=timeout)
            self.get_logger().info(f'Connected to ESP32 on {port} at {baud} baud')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open serial port {port}: {e}')
            self.serial = None

        self.wheelbase = self.get_parameter('wheelbase_m').value
        self.track_width = self.get_parameter('track_width_m').value
        self.max_speed = self.get_parameter('max_motor_speed').value
        self.max_servo_angle = self.get_parameter('max_servo_angle_rad').value
        self.drive_mode = str(self.get_parameter('drive_mode').value).strip().lower()
        if self.drive_mode not in ('servo', 'differential'):
            self.get_logger().warn(
                f"Invalid drive_mode '{self.drive_mode}', falling back to 'differential'")
            self.drive_mode = 'differential'
        self.expect_esp32_rx = bool(self.get_parameter('expect_esp32_rx').value)
        self.get_logger().info(
            f"Drive mode: {self.drive_mode}, ESP32 RX enabled: {self.expect_esp32_rx}")

        # State
        self.latest_imu = None
        self.lock = threading.Lock()

        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 10)

        # Publishers
        self.imu_raw_pub = self.create_publisher(Float32MultiArray, 'esp32/imu_raw', 50)
        self.status_pub = self.create_publisher(RoverStatus, 'rover/status', 10)

        # Serial read thread
        self.running = True
        if self.serial and self.expect_esp32_rx:
            self.read_thread = threading.Thread(target=self._serial_read_loop, daemon=True)
            self.read_thread.start()

        # Publish timer
        rate = self.get_parameter('publish_rate_hz').value
        self.timer = self.create_timer(1.0 / rate, self.publish_data)

    def cmd_vel_callback(self, msg):
        """Convert Twist to 4-wheel speed + servo angle commands and send to ESP32."""
        if not self.serial:
            return

        linear = msg.linear.x
        angular = msg.angular.z

        L = self.wheelbase       # front-to-rear axle distance
        T = self.track_width     # left-to-right wheel distance

        if self.drive_mode == 'differential':
            # Skid-steer mode: fixed steering, wheel-speed differential only.
            left_speed = linear - angular * (T / 2.0)
            right_speed = linear + angular * (T / 2.0)

            fl_angle = fr_angle = rl_angle = rr_angle = 0.0
            fl_speed = rl_speed = left_speed
            fr_speed = rr_speed = right_speed
        else:
            # Compute per-wheel steering angles and speeds using
            # independent 4-wheel steering geometry.
            # Front wheels steer in the commanded direction,
            # rear wheels steer opposite for tighter turning.
            if abs(angular) < 1e-6:
                # Straight driving
                fl_angle = fr_angle = rl_angle = rr_angle = 0.0
                fl_speed = fr_speed = rl_speed = rr_speed = linear
            else:
                # Turning radius from Twist
                R = linear / angular  # signed turn radius to center of rover

                # Ackermann-style per-wheel steering angles
                # Front wheels steer toward the turn, rear wheels steer opposite
                fl_angle = math.atan2(L, R - T / 2.0) - math.pi / 2.0
                fr_angle = math.atan2(L, R + T / 2.0) - math.pi / 2.0
                rl_angle = -fl_angle * 0.5  # Rear counter-steer (partial)
                rr_angle = -fr_angle * 0.5

                # Per-wheel speeds proportional to distance from turn center
                # (outer wheels faster, inner wheels slower)
                r_fl = math.hypot(L / 2.0, R - T / 2.0)
                r_fr = math.hypot(L / 2.0, R + T / 2.0)
                r_rl = math.hypot(L / 2.0, R - T / 2.0)
                r_rr = math.hypot(L / 2.0, R + T / 2.0)
                r_max = max(r_fl, r_fr, r_rl, r_rr, 1e-6)

                base_speed = abs(linear)
                fl_speed = math.copysign(base_speed * r_fl / r_max, linear)
                fr_speed = math.copysign(base_speed * r_fr / r_max, linear)
                rl_speed = math.copysign(base_speed * r_rl / r_max, linear)
                rr_speed = math.copysign(base_speed * r_rr / r_max, linear)

        # Clamp servo angles
        fl_angle = max(-self.max_servo_angle, min(self.max_servo_angle, fl_angle))
        fr_angle = max(-self.max_servo_angle, min(self.max_servo_angle, fr_angle))
        rl_angle = max(-self.max_servo_angle, min(self.max_servo_angle, rl_angle))
        rr_angle = max(-self.max_servo_angle, min(self.max_servo_angle, rr_angle))

        # Normalize speeds to [-1, 1]
        fl_cmd = max(-1.0, min(1.0, fl_speed / self.max_speed))
        fr_cmd = max(-1.0, min(1.0, fr_speed / self.max_speed))
        rl_cmd = max(-1.0, min(1.0, rl_speed / self.max_speed))
        rr_cmd = max(-1.0, min(1.0, rr_speed / self.max_speed))

        command = {
            'cmd': self.drive_mode,
            'fl_spd': round(fl_cmd, 3),
            'fr_spd': round(fr_cmd, 3),
            'rl_spd': round(rl_cmd, 3),
            'rr_spd': round(rr_cmd, 3),
            'fl_ang': round(fl_angle, 4),
            'fr_ang': round(fr_angle, 4),
            'rl_ang': round(rl_angle, 4),
            'rr_ang': round(rr_angle, 4),
        }
        self._serial_write(command)

    def _serial_write(self, data):
        """Send JSON command to ESP32."""
        if not self.serial:
            return
        try:
            line = json.dumps(data) + '\n'
            self.serial.write(line.encode('utf-8'))
        except serial.SerialException as e:
            self.get_logger().error(f'Serial write error: {e}')

    def _serial_read_loop(self):
        """Background thread: read and parse ESP32 serial data."""
        while self.running and self.expect_esp32_rx:
            try:
                if self.serial and self.serial.in_waiting:
                    line = self.serial.readline().decode('utf-8', errors='ignore').strip()
                    if not line:
                        continue
                    try:
                        data = json.loads(line)
                    except json.JSONDecodeError:
                        continue

                    with self.lock:
                        msg_type = data.get('type', '')
                        if msg_type == 'imu':
                            self.latest_imu = data
            except serial.SerialException:
                break
            except Exception as e:
                self.get_logger().debug(f'Serial read error: {e}')

    def publish_data(self):
        """Publish received ESP32 data as ROS messages."""
        with self.lock:
            imu = self.latest_imu
            self.latest_imu = None

        now = self.get_clock().now().to_msg()

        if imu:
            msg = Float32MultiArray()
            msg.data = [
                float(imu.get('ax', 0)), float(imu.get('ay', 0)), float(imu.get('az', 0)),
                float(imu.get('gx', 0)), float(imu.get('gy', 0)), float(imu.get('gz', 0)),
                float(imu.get('roll', 0)), float(imu.get('pitch', 0)), float(imu.get('yaw', 0))
            ]
            self.imu_raw_pub.publish(msg)

        # Publish rover status
        status = RoverStatus()
        status.header.stamp = now
        status.header.frame_id = 'camera_link'
        if imu:
            status.orientation_euler = [
                float(imu.get('roll', 0)),
                float(imu.get('pitch', 0)),
                float(imu.get('yaw', 0))
            ]
            status.linear_acceleration = [
                float(imu.get('ax', 0)),
                float(imu.get('ay', 0)),
                float(imu.get('az', 0))
            ]
            status.angular_velocity = [
                float(imu.get('gx', 0)),
                float(imu.get('gy', 0)),
                float(imu.get('gz', 0))
            ]
        status.state = RoverStatus.STATE_NAVIGATING
        self.status_pub.publish(status)

    def destroy_node(self):
        self.running = False
        if self.serial:
            # Stop all motors before shutdown
            self._serial_write({
                'cmd': 'motor',
                'fl_spd': 0.0, 'fr_spd': 0.0, 'rl_spd': 0.0, 'rr_spd': 0.0,
                'fl_ang': 0.0, 'fr_ang': 0.0, 'rl_ang': 0.0, 'rr_ang': 0.0,
            })
            self.serial.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ESP32Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
