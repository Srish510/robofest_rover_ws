"""
Odometry node - computes rover odometry from IMU data (RealSense by default)
and commanded velocity (cmd_vel). Publishes odometry in odom -> child_frame_id
(camera_link by default).

The rover has 4 independently driven and steered wheels. Without wheel encoders,
linear velocity is estimated from cmd_vel while heading is estimated from IMU
orientation (if available) or integrated gyro z.
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, PoseStamped, Quaternion, Twist
from sensor_msgs.msg import Imu
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
        self.declare_parameter('imu_source', 'realsense')   # realsense or esp32
        self.declare_parameter('imu_topic', 'camera/imu')
        self.declare_parameter('legacy_imu_topic', 'esp32/imu_raw')
        self.declare_parameter('child_frame_id', 'camera_link')

        self.publish_tf = self.get_parameter('publish_tf').value
        imu_source = self.get_parameter('imu_source').value
        self.child_frame_id = self.get_parameter('child_frame_id').value

        # Odometry state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0           # Updated from IMU yaw
        self.omega = 0.0           # Angular velocity from IMU gyro z
        self.cmd_linear = 0.0      # Commanded linear velocity from cmd_vel
        self.imu_ready = False
        self.last_imu_time_sec = None
        self.prev_time = None

        # Subscribers
        if imu_source == 'esp32':
            legacy_imu_topic = self.get_parameter('legacy_imu_topic').value
            self.imu_sub = self.create_subscription(
                Float32MultiArray, legacy_imu_topic, self.imu_esp32_callback, 50)
            self.get_logger().info(f'Using ESP32 IMU topic: {legacy_imu_topic}')
        else:
            imu_topic = self.get_parameter('imu_topic').value
            self.imu_sub = self.create_subscription(
                Imu, imu_topic, self.imu_realsense_callback, 50)
            self.get_logger().info(f'Using RealSense IMU topic: {imu_topic}')

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

    def imu_esp32_callback(self, msg):
        # esp32/imu_raw layout: [ax, ay, az, gx, gy, gz, roll, pitch, yaw]
        if len(msg.data) < 9:
            return
        self.theta = float(msg.data[8])     # Fused yaw from ESP32 AHRS
        self.omega = float(msg.data[5])     # Gyro z (rad/s)
        self.imu_ready = True

    def imu_realsense_callback(self, msg):
        stamp_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if stamp_sec <= 0.0:
            stamp_sec = self.get_clock().now().nanoseconds / 1e9

        dt = None
        if self.last_imu_time_sec is not None:
            dt = stamp_sec - self.last_imu_time_sec
            if dt <= 0.0 or dt > 1.0:
                dt = None
        self.last_imu_time_sec = stamp_sec

        self.omega = float(msg.angular_velocity.z)

        has_orientation = msg.orientation_covariance[0] >= 0.0
        if has_orientation:
            q = msg.orientation
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            self.theta = math.atan2(siny_cosp, cosy_cosp)
        elif dt is not None:
            self.theta += self.omega * dt

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
        odom.child_frame_id = self.child_frame_id
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
            t.child_frame_id = self.child_frame_id
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
