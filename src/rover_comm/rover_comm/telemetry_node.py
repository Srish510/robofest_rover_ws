"""
Telemetry node - aggregates rover status, checkpoint data, and system health,
serving it to the GCS as a JSON API and providing the checkpoint registration service.
"""
import rclpy
from rclpy.node import Node
from rover_interfaces.msg import RoverStatus
from rover_interfaces.srv import RegisterCheckpoint
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
import json
import threading
from http.server import HTTPServer, BaseHTTPRequestHandler
import errno
import time
from urllib.parse import urlsplit

try:
    from http.server import ThreadingHTTPServer
except ImportError:
    ThreadingHTTPServer = HTTPServer


class TelemetryNode(Node):
    def __init__(self):
        super().__init__('telemetry_node')

        # Parameters
        self.declare_parameter('telemetry_port', 8082)
        self.declare_parameter('update_rate_hz', 5.0)

        self.data_lock = threading.Lock()
        self.latest_status = None
        self.latest_pose = None
        self.checkpoints = []  # List of registered checkpoints
        self.qr_detections = []  # Recent QR detections

        # Subscribers
        self.status_sub = self.create_subscription(
            RoverStatus, 'rover/status', self.status_callback, 10)
        self.pose_sub = self.create_subscription(
            PoseStamped, 'rover/pose', self.pose_callback, 10)
        self.qr_sub = self.create_subscription(
            String, 'perception/qr_detected', self.qr_callback, 10)

        # Service: checkpoint registration
        self.checkpoint_srv = self.create_service(
            RegisterCheckpoint, 'register_checkpoint', self.register_checkpoint_handler)

        # Start telemetry HTTP server
        port = self.get_parameter('telemetry_port').value
        self._start_telemetry_server(port)

        self.get_logger().info(f'Telemetry server started on port {port}')

    def status_callback(self, msg):
        with self.data_lock:
            self.latest_status = msg

    def pose_callback(self, msg):
        with self.data_lock:
            self.latest_pose = msg

    def qr_callback(self, msg):
        with self.data_lock:
            self.qr_detections.append({
                'data': msg.data,
                'time': time.time()
            })
            # Keep last 50
            if len(self.qr_detections) > 50:
                self.qr_detections = self.qr_detections[-50:]

    def register_checkpoint_handler(self, request, response):
        """Handle checkpoint registration service calls."""
        checkpoint_id = f'CP_{len(self.checkpoints) + 1:03d}'

        checkpoint = {
            'id': checkpoint_id,
            'qr_data': request.qr_data,
            'timestamp': request.timestamp.sec + request.timestamp.nanosec * 1e-9,
            'pose': {
                'x': request.rover_pose.position.x,
                'y': request.rover_pose.position.y,
                'z': request.rover_pose.position.z,
            },
            'registered_at': time.time()
        }

        with self.data_lock:
            self.checkpoints.append(checkpoint)

        self.get_logger().info(
            f'Checkpoint {checkpoint_id} registered: {request.qr_data}')

        response.success = True
        response.checkpoint_id = checkpoint_id
        response.message = f'Checkpoint {checkpoint_id} registered successfully'
        return response

    def _get_telemetry_dict(self):
        """Build telemetry JSON dict."""
        with self.data_lock:
            status = self.latest_status
            pose = self.latest_pose
            checkpoints = list(self.checkpoints)
            qr_detections = list(self.qr_detections)

        last_qr = qr_detections[-1] if qr_detections else None

        telemetry = {
            'timestamp': time.time(),
            'status': None,
            'pose': None,
            'checkpoints': checkpoints,
            'total_checkpoints': len(checkpoints),
            'last_qr_detection': last_qr,
            'qr_detections': qr_detections,
        }

        if status:
            state_names = {0: 'idle', 1: 'navigating', 2: 'obstacle_avoidance',
                           3: 'checkpoint_scan', 4: 'error'}
            telemetry['status'] = {
                'servo_angles': list(status.servo_angles),
                'orientation': list(status.orientation_euler),
                'state': state_names.get(status.state, 'unknown'),
                'last_checkpoint': status.last_checkpoint,
            }

        if pose:
            telemetry['pose'] = {
                'x': pose.pose.position.x,
                'y': pose.pose.position.y,
                'z': pose.pose.position.z,
                'orientation': {
                    'x': pose.pose.orientation.x,
                    'y': pose.pose.orientation.y,
                    'z': pose.pose.orientation.z,
                    'w': pose.pose.orientation.w,
                }
            }

        return telemetry

    @staticmethod
    def _json_default(value):
        """Convert non-standard numeric types (e.g. numpy scalars) for JSON."""
        if hasattr(value, 'item'):
            return value.item()
        if hasattr(value, 'tolist'):
            return value.tolist()
        raise TypeError(f'Object of type {value.__class__.__name__} is not JSON serializable')

    def _start_telemetry_server(self, port):
        node = self

        class ReusableThreadingHTTPServer(ThreadingHTTPServer):
            allow_reuse_address = True
            daemon_threads = True

        class TelemetryHandler(BaseHTTPRequestHandler):
            def do_GET(self):
                request_path = urlsplit(self.path).path

                if request_path == '/telemetry':
                    data = json.dumps(node._get_telemetry_dict(), default=node._json_default)
                    self.send_response(200)
                    self.send_header('Content-Type', 'application/json')
                    self.send_header('Cache-Control', 'no-store, no-cache, must-revalidate, max-age=0')
                    self.send_header('Pragma', 'no-cache')
                    self.send_header('Expires', '0')
                    self.send_header('Access-Control-Allow-Origin', '*')
                    self.end_headers()
                    self.wfile.write(data.encode())

                elif request_path == '/checkpoints':
                    with node.data_lock:
                        data = json.dumps(node.checkpoints, default=node._json_default)
                    self.send_response(200)
                    self.send_header('Content-Type', 'application/json')
                    self.send_header('Cache-Control', 'no-store, no-cache, must-revalidate, max-age=0')
                    self.send_header('Pragma', 'no-cache')
                    self.send_header('Expires', '0')
                    self.send_header('Access-Control-Allow-Origin', '*')
                    self.end_headers()
                    self.wfile.write(data.encode())

                elif request_path == '/qr_detections':
                    with node.data_lock:
                        data = json.dumps(node.qr_detections, default=node._json_default)
                    self.send_response(200)
                    self.send_header('Content-Type', 'application/json')
                    self.send_header('Cache-Control', 'no-store, no-cache, must-revalidate, max-age=0')
                    self.send_header('Pragma', 'no-cache')
                    self.send_header('Expires', '0')
                    self.send_header('Access-Control-Allow-Origin', '*')
                    self.end_headers()
                    self.wfile.write(data.encode())

                elif request_path == '/stream':
                    # SSE for real-time telemetry
                    self.send_response(200)
                    self.send_header('Content-Type', 'text/event-stream')
                    self.send_header('Cache-Control', 'no-cache')
                    self.send_header('Connection', 'keep-alive')
                    self.send_header('X-Accel-Buffering', 'no')
                    self.send_header('Access-Control-Allow-Origin', '*')
                    self.end_headers()
                    self.wfile.write(b'retry: 2000\n\n')
                    self.wfile.flush()
                    rate = node.get_parameter('update_rate_hz').value
                    interval = 1.0 / rate
                    try:
                        while rclpy.ok():
                            data = json.dumps(node._get_telemetry_dict(), default=node._json_default)
                            self.wfile.write(f'data: {data}\n\n'.encode())
                            self.wfile.flush()
                            time.sleep(interval)
                    except (BrokenPipeError, ConnectionResetError):
                        pass
                else:
                    self.send_response(200)
                    self.send_header('Content-Type', 'text/html')
                    self.end_headers()
                    self.wfile.write(
                        b'<html><body><h1>Rover Telemetry</h1>'
                        b'<p>Endpoints: /telemetry, /checkpoints, /qr_detections, /stream</p>'
                        b'</body></html>')

            def log_message(self, format, *args):
                pass

        server = None
        last_exception = None
        for attempt in range(3):
            try:
                server = ReusableThreadingHTTPServer(('0.0.0.0', port), TelemetryHandler)
                break
            except OSError as exc:
                last_exception = exc
                if exc.errno == errno.EADDRINUSE:
                    self.get_logger().warn(
                        f'Telemetry port {port} already in use (attempt {attempt + 1}/3). Retrying...')
                else:
                    self.get_logger().warn(
                        f'Telemetry server bind failed (attempt {attempt + 1}/3): {exc}')
                time.sleep(0.5)

        if server is None:
            raise RuntimeError(
                f'Failed to start telemetry HTTP server on port {port}: {last_exception}')

        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()


def main(args=None):
    rclpy.init(args=args)
    node = TelemetryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
