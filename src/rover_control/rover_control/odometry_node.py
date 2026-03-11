"""
Odometry node - computes rover odometry from ESP32 external IMU data (heading)
and commanded velocity (cmd_vel). Publishes odom -> base_link transforms.

The rover has 4 independently driven and steered wheels. Without wheel encoders,
linear velocity is estimated from cmd_vel while heading is taken from the IMU's
fused yaw output on the ESP32.
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, PoseStamped, Quaternion, Twist
from std_msgs.msg import Float32MultiArray
from tf2_ros import TransformBroadcaster
import math


class OdometryNode(Node):
    def __init__(self):
        super().__init__('odometry_node')

        # Parameters
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('odom_rate_hz', 50.0)
        self.declare_parameter('speed_scale', 1.0)          # Multiplier to calibrate cmd_vel to actual speed

        self.publish_tf = self.get_parameter('publish_tf').value

        # Odometry state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0           # Updated from IMU yaw
        self.omega = 0.0           # Angular velocity from IMU gyro z
        self.cmd_linear = 0.0      # Commanded linear velocity from cmd_vel
        self.imu_ready = False
        self.prev_time = None

        # Subscribers
        self.imu_sub = self.create_subscription(
            Float32MultiArray, 'esp32/imu_raw', self.imu_callback, 50)
        self.cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 10)

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, 'odom', 50)
        self.pose_pub = self.create_publisher(PoseStamped, 'rover/pose', 50)

        # TF broadcaster
        if self.publish_tf:
            self.tf_broadcaster = TransformBroadcaster(self)

        # Odom update timer
        rate = self.get_parameter('odom_rate_hz').value
        self.timer = self.create_timer(1.0 / rate, self.odom_update)

        self.get_logger().info('Odometry node initialized (IMU + cmd_vel)')

    def imu_callback(self, msg):
        # esp32/imu_raw layout: [ax, ay, az, gx, gy, gz, roll, pitch, yaw]
        if len(msg.data) < 9:
            return
        self.theta = float(msg.data[8])     # Fused yaw from ESP32 AHRS
        self.omega = float(msg.data[5])     # Gyro z (rad/s)
        self.imu_ready = True

    def cmd_vel_callback(self, msg):
        self.cmd_linear = msg.linear.x

    def odom_update(self):
        if not self.imu_ready:
            return

        now = self.get_clock().now()
        if self.prev_time is None:
            self.prev_time = now
            return

        dt = (now - self.prev_time).nanoseconds / 1e9
        self.prev_time = now
        if dt <= 0.0 or dt > 1.0:
            return

        speed_scale = self.get_parameter('speed_scale').value
        v = self.cmd_linear * speed_scale

        # Dead-reckon position using IMU heading + commanded speed
        self.x += v * math.cos(self.theta) * dt
        self.y += v * math.sin(self.theta) * dt

        now_msg = now.to_msg()
        q = self._yaw_to_quaternion(self.theta)

        # Publish Odometry
        odom = Odometry()
        odom.header.stamp = now_msg
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = q
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = self.omega
        self.odom_pub.publish(odom)

        # Publish PoseStamped for QR scanner
        pose_msg = PoseStamped()
        pose_msg.header = odom.header
        pose_msg.pose = odom.pose.pose
        self.pose_pub.publish(pose_msg)

        # Publish TF
        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = now_msg
            t.header.frame_id = 'odom'
            t.child_frame_id = 'base_link'
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            t.transform.rotation = q
            self.tf_broadcaster.sendTransform(t)

    def _yaw_to_quaternion(self, yaw):
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q


def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
