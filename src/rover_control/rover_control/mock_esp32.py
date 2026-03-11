"""
Mock ESP32 node - simulates the ESP32 by subscribing to cmd_vel and publishing
fake IMU data. Replaces esp32_bridge for testing.
Simulates a 4-wheel drive + 4-wheel steer rover.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from rover_interfaces.msg import RoverStatus
import math
import time


class MockESP32(Node):
    def __init__(self):
        super().__init__('mock_esp32')

        self.declare_parameter('wheelbase_m', 0.3)
        self.declare_parameter('track_width_m', 0.3)
        self.declare_parameter('publish_rate_hz', 50.0)

        self.wheelbase = self.get_parameter('wheelbase_m').value
        self.track_width = self.get_parameter('track_width_m').value

        self.last_cmd = Twist()
        self.last_time = time.time()
        self.yaw = 0.0  # Simulated IMU yaw (integrated from angular velocity)

        self.cmd_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_callback, 10)

        self.imu_pub = self.create_publisher(Float32MultiArray, 'esp32/imu_raw', 50)
        self.status_pub = self.create_publisher(RoverStatus, 'rover/status', 10)

        rate = self.get_parameter('publish_rate_hz').value
        self.timer = self.create_timer(1.0 / rate, self.publish_data)

        self.get_logger().info('Mock ESP32 started (4WD/4WS)')

    def cmd_callback(self, msg):
        self.last_cmd = msg

    def publish_data(self):
        now = time.time()
        dt = now - self.last_time
        self.last_time = now
        if dt <= 0:
            dt = 0.02

        linear = self.last_cmd.linear.x
        angular = self.last_cmd.angular.z

        # Integrate yaw from angular velocity (simulates ESP32 AHRS output)
        self.yaw += angular * dt
        self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))

        # Simulate IMU: [ax, ay, az, gx, gy, gz, roll, pitch, yaw]
        imu_msg = Float32MultiArray()
        imu_msg.data = [
            0.0, 0.0, 9.81,                  # accel x, y, z
            0.0, 0.0, float(angular),         # gyro x, y, z
            0.0, 0.0, float(self.yaw)         # roll, pitch, yaw
        ]
        self.imu_pub.publish(imu_msg)

        # Status
        status = RoverStatus()
        status.header.stamp = self.get_clock().now().to_msg()
        status.header.frame_id = 'base_link'
        status.servo_angles = [0.0, 0.0, 0.0, 0.0]
        status.orientation_euler = [0.0, 0.0, float(self.yaw)]
        status.angular_velocity = [0.0, 0.0, float(angular)]
        status.state = RoverStatus.STATE_NAVIGATING
        self.status_pub.publish(status)


def main(args=None):
    rclpy.init(args=args)
    node = MockESP32()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()