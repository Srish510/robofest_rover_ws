"""
Map stream node - sends occupancy grid and colored depth heatmap data to GCS over WiFi.
Uses Server-Sent Events (SSE) for real-time updates.
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import json
import threading
from http.server import HTTPServer, BaseHTTPRequestHandler
import base64

try:
    from http.server import ThreadingHTTPServer
except ImportError:
    ThreadingHTTPServer = HTTPServer


class MapStreamNode(Node):
    def __init__(self):
        super().__init__('map_stream_node')

        # Parameters
        self.declare_parameter('map_port', 8081)
        self.declare_parameter('update_rate_hz', 2.0)
        self.declare_parameter('compress_map', True)

        self.latest_occupancy = None
        self.latest_depth = None
        self.data_lock = threading.Lock()
        self.bridge = CvBridge()

        # Subscribers
        self.occ_sub = self.create_subscription(
            OccupancyGrid, 'perception/local_costmap', self.occupancy_callback, 5)
        self.depth_sub = self.create_subscription(
            Image, 'camera/aligned_depth/image_raw', self.depth_callback, 5)

        # Start HTTP server for map data
        port = self.get_parameter('map_port').value
        self._start_map_server(port)

        self.get_logger().info(f'Map stream server started on port {port}')

    def occupancy_callback(self, msg):
        with self.data_lock:
            self.latest_occupancy = msg

    def depth_callback(self, msg):
        with self.data_lock:
            self.latest_depth = msg

    def _occupancy_to_json(self, msg):
        """Convert OccupancyGrid to JSON-serializable dict."""
        return {
            'type': 'occupancy_grid',
            'timestamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
            'frame_id': msg.header.frame_id,
            'resolution': msg.info.resolution,
            'width': msg.info.width,
            'height': msg.info.height,
            'origin': {
                'x': msg.info.origin.position.x,
                'y': msg.info.origin.position.y,
                'z': msg.info.origin.position.z,
            },
            'data': base64.b64encode(bytes(
                [max(0, v) for v in msg.data]
            )).decode('ascii')
        }

    def _depth_frame_to_json(self, msg):
        """Convert depth image topic to colored heatmap payload for `/depth_frame`."""
        payload = self._depth_heatmap_to_json(msg)
        payload['type'] = 'depth_frame'
        return payload

    def _depth_heatmap_to_json(self, msg):
        """Convert depth image to backward-compatible heatmap JPEG JSON payload."""
        depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

        if depth.dtype == np.uint16:
            depth_m = depth.astype(np.float32) / 1000.0
        elif depth.dtype == np.float32:
            depth_m = depth
        else:
            depth_m = depth.astype(np.float32) * 0.001

        valid = np.isfinite(depth_m) & (depth_m > 0.0)
        if np.any(valid):
            min_v = float(np.min(depth_m[valid]))
            max_v = float(np.max(depth_m[valid]))
            span = max(max_v - min_v, 1e-6)
            norm = np.zeros_like(depth_m, dtype=np.float32)
            norm[valid] = (depth_m[valid] - min_v) / span
            gray = ((1.0 - norm) * 255.0).astype(np.uint8)
            heatmap = cv2.applyColorMap(gray, cv2.COLORMAP_JET)
            heatmap[~valid] = (0, 0, 0)
        else:
            min_v = 0.0
            max_v = 0.0
            heatmap = np.zeros((depth_m.shape[0], depth_m.shape[1], 3), dtype=np.uint8)

        ok, jpg = cv2.imencode('.jpg', heatmap, [int(cv2.IMWRITE_JPEG_QUALITY), 85])
        if not ok:
            raise RuntimeError('Failed to encode depth heatmap JPEG')

        return {
            'type': 'depth_heatmap',
            'timestamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
            'frame_id': msg.header.frame_id,
            'width': int(heatmap.shape[1]),
            'height': int(heatmap.shape[0]),
            'encoding': 'jpeg',
            'unit': 'meters',
            'depth_min_m': min_v,
            'depth_max_m': max_v,
            'image': base64.b64encode(jpg.tobytes()).decode('ascii'),
        }

    def _start_map_server(self, port):
        node = self

        class MapHandler(BaseHTTPRequestHandler):
            def do_GET(self):
                if self.path == '/occupancy':
                    with node.data_lock:
                        data = node.latest_occupancy
                    if data:
                        json_data = json.dumps(node._occupancy_to_json(data))
                        self.send_response(200)
                        self.send_header('Content-Type', 'application/json')
                        self.send_header('Access-Control-Allow-Origin', '*')
                        self.end_headers()
                        self.wfile.write(json_data.encode())
                    else:
                        self.send_response(503)
                        self.end_headers()

                elif self.path == '/depth_frame':
                    with node.data_lock:
                        data = node.latest_depth
                    if data:
                        json_data = json.dumps(node._depth_frame_to_json(data))
                        self.send_response(200)
                        self.send_header('Content-Type', 'application/json')
                        self.send_header('Access-Control-Allow-Origin', '*')
                        self.end_headers()
                        self.wfile.write(json_data.encode())
                    else:
                        self.send_response(503)
                        self.end_headers()

                elif self.path == '/depth_heatmap' or self.path == '/terrain':
                    with node.data_lock:
                        data = node.latest_depth
                    if data:
                        json_data = json.dumps(node._depth_heatmap_to_json(data))
                        self.send_response(200)
                        self.send_header('Content-Type', 'application/json')
                        self.send_header('Access-Control-Allow-Origin', '*')
                        self.end_headers()
                        self.wfile.write(json_data.encode())
                    else:
                        self.send_response(503)
                        self.end_headers()

                elif self.path == '/stream':
                    # SSE (Server-Sent Events) for continuous map updates
                    self.send_response(200)
                    self.send_header('Content-Type', 'text/event-stream')
                    self.send_header('Cache-Control', 'no-cache')
                    self.send_header('Connection', 'keep-alive')
                    self.send_header('Access-Control-Allow-Origin', '*')
                    self.end_headers()
                    import time
                    rate = node.get_parameter('update_rate_hz').value
                    interval = 1.0 / rate
                    try:
                        while rclpy.ok():
                            with node.data_lock:
                                occ = node.latest_occupancy
                                depth = node.latest_depth
                            payload = {}
                            if occ:
                                payload['occupancy'] = node._occupancy_to_json(occ)
                            if depth:
                                depth_heatmap_payload = node._depth_heatmap_to_json(depth)
                                payload['depth_heatmap'] = depth_heatmap_payload
                                payload['depth_frame'] = dict(depth_heatmap_payload, type='depth_frame')
                            if payload:
                                self.wfile.write(
                                    f'data: {json.dumps(payload)}\n\n'.encode())
                                self.wfile.flush()
                            time.sleep(interval)
                    except (BrokenPipeError, ConnectionResetError):
                        pass
                else:
                    self.send_response(200)
                    self.send_header('Content-Type', 'text/html')
                    self.end_headers()
                    self.wfile.write(
                        b'<html><body><h1>Rover Map Server</h1>'
                        b'<p>Endpoints: /occupancy, /depth_frame (/depth_heatmap,/terrain aliases), /stream</p>'
                        b'</body></html>')

            def log_message(self, format, *args):
                pass

        server = ThreadingHTTPServer(('0.0.0.0', port), MapHandler)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()


def main(args=None):
    rclpy.init(args=args)
    node = MapStreamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
