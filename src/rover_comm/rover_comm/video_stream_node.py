"""
Video stream node - streams live camera feed to GCS over WiFi using
MJPEG over HTTP. Lightweight and compatible with any browser-based GCS.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2
import threading
from http.server import HTTPServer, BaseHTTPRequestHandler
import time


class VideoStreamNode(Node):
    def __init__(self):
        super().__init__('video_stream_node')

        # Parameters
        self.declare_parameter('stream_port', 8080)
        self.declare_parameter('jpeg_quality', 70)
        self.declare_parameter('max_fps', 15)
        self.declare_parameter('resize_width', 640)
        self.declare_parameter('resize_height', 480)
        self.declare_parameter('image_topic', 'camera/color/image_raw')

        self.bridge = CvBridge()
        self.latest_frame = None
        self.frame_lock = threading.Lock()
        self.jpeg_quality = self.get_parameter('jpeg_quality').value
        self.resize_w = self.get_parameter('resize_width').value
        self.resize_h = self.get_parameter('resize_height').value
        self.min_frame_interval = 1.0 / self.get_parameter('max_fps').value

        # Subscribe to camera image
        topic = self.get_parameter('image_topic').value
        self.image_sub = self.create_subscription(
            Image, topic, self.image_callback, 10)

        # Also publish compressed image for ROS-based GCS
        self.compressed_pub = self.create_publisher(
            CompressedImage, 'camera/color/compressed', 5)

        # Start HTTP MJPEG server in a background thread
        port = self.get_parameter('stream_port').value
        self._start_http_server(port)

        self.get_logger().info(f'Video stream server started on port {port}')

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Resize for streaming efficiency
        if frame.shape[1] != self.resize_w or frame.shape[0] != self.resize_h:
            frame = cv2.resize(frame, (self.resize_w, self.resize_h))

        # Encode to JPEG
        encode_params = [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality]
        _, jpeg_data = cv2.imencode('.jpg', frame, encode_params)

        with self.frame_lock:
            self.latest_frame = jpeg_data.tobytes()

        # Publish compressed image
        compressed_msg = CompressedImage()
        compressed_msg.header = msg.header
        compressed_msg.format = 'jpeg'
        compressed_msg.data = self.latest_frame
        self.compressed_pub.publish(compressed_msg)

    def _start_http_server(self, port):
        node = self

        class MJPEGHandler(BaseHTTPRequestHandler):
            def do_GET(self):
                if self.path == '/stream':
                    self.send_response(200)
                    self.send_header('Content-Type',
                                     'multipart/x-mixed-replace; boundary=frame')
                    self.send_header('Cache-Control', 'no-cache')
                    self.send_header('Access-Control-Allow-Origin', '*')
                    self.end_headers()
                    try:
                        while rclpy.ok():
                            with node.frame_lock:
                                frame_data = node.latest_frame
                            if frame_data is not None:
                                self.wfile.write(b'--frame\r\n')
                                self.wfile.write(b'Content-Type: image/jpeg\r\n')
                                self.wfile.write(
                                    f'Content-Length: {len(frame_data)}\r\n\r\n'.encode())
                                self.wfile.write(frame_data)
                                self.wfile.write(b'\r\n')
                            time.sleep(node.min_frame_interval)
                    except (BrokenPipeError, ConnectionResetError):
                        pass
                elif self.path == '/snapshot':
                    with node.frame_lock:
                        frame_data = node.latest_frame
                    if frame_data:
                        self.send_response(200)
                        self.send_header('Content-Type', 'image/jpeg')
                        self.send_header('Content-Length', str(len(frame_data)))
                        self.send_header('Access-Control-Allow-Origin', '*')
                        self.end_headers()
                        self.wfile.write(frame_data)
                    else:
                        self.send_response(503)
                        self.end_headers()
                else:
                    self.send_response(200)
                    self.send_header('Content-Type', 'text/html')
                    self.end_headers()
                    self.wfile.write(
                        b'<html><body>'
                        b'<h1>Rover Camera</h1>'
                        b'<img src="/stream" />'
                        b'</body></html>')

            def log_message(self, format, *args):
                pass  # Suppress HTTP logs

        server = HTTPServer(('0.0.0.0', port), MJPEGHandler)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()


def main(args=None):
    rclpy.init(args=args)
    node = VideoStreamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
