import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
from rover_interfaces.srv import RegisterCheckpoint
from pyzbar.pyzbar import decode as pyzbar_decode
import cv2
import numpy as np


class QRScanner(Node):
    def __init__(self):
        super().__init__('qr_scanner')

        # Parameters
        self.declare_parameter('scan_interval_sec', 0.5)            #Minimum time between scans (seconds)
        self.declare_parameter('min_qr_size', 50)                   #Minimum size (in pixels) of QR code bounding box to consider valid
        self.declare_parameter('deduplicate_timeout_sec', 30.0)     #Time to consider a QR code "already scanned" (seconds)

        self.bridge = CvBridge()
        self.last_scan_time = self.get_clock().now()
        self.scan_interval = self.get_parameter('scan_interval_sec').value

        # Track scanned QR codes to avoid duplicates
        self.scanned_codes = {}  # {qr_data: last_scan_timestamp}
        self.dedup_timeout = self.get_parameter('deduplicate_timeout_sec').value

        # Current pose 
        self.current_pose = None

        # Subscribers
        self.image_sub = self.create_subscription(
            Image, 'camera/color/image_raw', self.image_callback, 10)
        self.pose_sub = self.create_subscription(
            PoseStamped, 'rover/pose', self.pose_callback, 10)

        # Publishers
        self.qr_pub = self.create_publisher(String, 'perception/qr_detected', 10)
        self.debug_pub = self.create_publisher(Image, 'perception/qr_debug', 5)

        # Service client for checkpoint registration
        self.checkpoint_client = self.create_client(
            RegisterCheckpoint, 'register_checkpoint')

        self.get_logger().info('QR scanner initialized (using pyzbar)')

    def pose_callback(self, msg):
        self.current_pose = msg.pose

    def image_callback(self, msg):
        # Rate-limit scanning
        now = self.get_clock().now()
        elapsed = (now - self.last_scan_time).nanoseconds / 1e9
        if elapsed < self.scan_interval:
            return
        self.last_scan_time = now

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        debug_frame = frame.copy()

        # Decode QR codes using pyzbar
        decoded_objects = pyzbar_decode(frame)

        for obj in decoded_objects:
            if obj.type != 'QRCODE':
                continue

            data = obj.data.decode('utf-8', errors='ignore')
            if not data:
                continue

            # Get bounding polygon
            points = obj.polygon
            if len(points) < 4:
                continue

            # Check minimum size
            pts = np.array([(p.x, p.y) for p in points], dtype=np.int32)
            x, y, w, h = cv2.boundingRect(pts)
            min_size = self.get_parameter('min_qr_size').value
            if w < min_size or h < min_size:
                continue

            # Draw bounding box
            cv2.polylines(debug_frame, [pts], True, (0, 255, 0), 3)
            cv2.putText(debug_frame, data, (x, y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            # Check for duplicate scan
            now_sec = now.nanoseconds / 1e9
            if data in self.scanned_codes:
                if (now_sec - self.scanned_codes[data]) < self.dedup_timeout:
                    cv2.putText(debug_frame, "ALREADY SCANNED", (10, 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 165, 255), 2)
                    self._publish_debug(debug_frame, msg.header)
                    return

            # New checkpoint detected
            self.scanned_codes[data] = now_sec
            self.get_logger().info(f'QR checkpoint detected: {data}')

            # Publish QR data
            qr_msg = String()
            qr_msg.data = data
            self.qr_pub.publish(qr_msg)

            # Register checkpoint with GCS
            self._register_checkpoint(data, msg.header.stamp)

            cv2.putText(debug_frame, "CHECKPOINT REGISTERED", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

        self._publish_debug(debug_frame, msg.header)

    def _register_checkpoint(self, qr_data, stamp):
        """Send checkpoint registration request to GCS."""
        if not self.checkpoint_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Checkpoint registration service not available')
            return

        request = RegisterCheckpoint.Request()
        request.qr_data = qr_data
        request.timestamp = stamp
        if self.current_pose:
            request.rover_pose = self.current_pose
        future = self.checkpoint_client.call_async(request)
        future.add_done_callback(self._checkpoint_response_callback)

    def _checkpoint_response_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(
                    f'Checkpoint {response.checkpoint_id} registered: {response.message}')
            else:
                self.get_logger().warn(f'Checkpoint registration failed: {response.message}')
        except Exception as e:
            self.get_logger().error(f'Checkpoint service call failed: {e}')

    def _publish_debug(self, frame, header):
        debug_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        debug_msg.header = header
        self.debug_pub.publish(debug_msg)


def main(args=None):
    rclpy.init(args=args)
    node = QRScanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()