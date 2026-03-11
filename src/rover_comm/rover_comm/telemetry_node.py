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
import time


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

        telemetry = {
            'timestamp': time.time(),
            'status': None,
            'pose': None,
            'checkpoints': checkpoints,
            'total_checkpoints': len(checkpoints),
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

    def _start_telemetry_server(self, port):
        node = self

        class TelemetryHandler(BaseHTTPRequestHandler):
            def do_GET(self):
                if self.path == '/telemetry':
                    data = json.dumps(node._get_telemetry_dict())
                    self.send_response(200)
                    self.send_header('Content-Type', 'application/json')
                    self.send_header('Access-Control-Allow-Origin', '*')
                    self.end_headers()
                    self.wfile.write(data.encode())

                elif self.path == '/checkpoints':
                    with node.data_lock:
                        data = json.dumps(node.checkpoints)
                    self.send_response(200)
                    self.send_header('Content-Type', 'application/json')
                    self.send_header('Access-Control-Allow-Origin', '*')
                    self.end_headers()
                    self.wfile.write(data.encode())

                elif self.path == '/stream':
                    # SSE for real-time telemetry
                    self.send_response(200)
                    self.send_header('Content-Type', 'text/event-stream')
                    self.send_header('Cache-Control', 'no-cache')
                    self.send_header('Access-Control-Allow-Origin', '*')
                    self.end_headers()
                    rate = node.get_parameter('update_rate_hz').value
                    interval = 1.0 / rate
                    try:
                        while rclpy.ok():
                            data = json.dumps(node._get_telemetry_dict())
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
                        b'<p>Endpoints: /telemetry, /checkpoints, /stream</p>'
                        b'</body></html>')

            def log_message(self, format, *args):
                pass

        server = HTTPServer(('0.0.0.0', port), TelemetryHandler)
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
