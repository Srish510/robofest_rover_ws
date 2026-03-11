"""
Map stream node - sends terrain map and occupancy grid data to GCS over WiFi.
Uses a lightweight WebSocket server for real-time map updates.
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from rover_interfaces.msg import TerrainMap
import json
import threading
from http.server import HTTPServer, BaseHTTPRequestHandler
import base64


class MapStreamNode(Node):
    def __init__(self):
        super().__init__('map_stream_node')

        # Parameters
        self.declare_parameter('map_port', 8081)
        self.declare_parameter('update_rate_hz', 2.0)
        self.declare_parameter('compress_map', True)

        self.latest_occupancy = None
        self.latest_terrain = None
        self.data_lock = threading.Lock()

        # Subscribers
        self.occ_sub = self.create_subscription(
            OccupancyGrid, 'perception/local_costmap', self.occupancy_callback, 5)
        self.terrain_sub = self.create_subscription(
            TerrainMap, 'perception/terrain_map', self.terrain_callback, 5)

        # Start HTTP server for map data
        port = self.get_parameter('map_port').value
        self._start_map_server(port)

        self.get_logger().info(f'Map stream server started on port {port}')

    def occupancy_callback(self, msg):
        with self.data_lock:
            self.latest_occupancy = msg

    def terrain_callback(self, msg):
        with self.data_lock:
            self.latest_terrain = msg

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

    def _terrain_to_json(self, msg):
        """Convert TerrainMap to JSON-serializable dict."""
        return {
            'type': 'terrain_map',
            'timestamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
            'frame_id': msg.header.frame_id,
            'resolution': msg.resolution,
            'width': msg.width,
            'height': msg.height,
            'origin': {
                'x': msg.origin.position.x,
                'y': msg.origin.position.y,
                'z': msg.origin.position.z,
            },
            'terrain_classes': base64.b64encode(
                bytes(msg.terrain_classes)).decode('ascii'),
            'traversability': base64.b64encode(
                bytes(msg.traversability)).decode('ascii'),
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

                elif self.path == '/terrain':
                    with node.data_lock:
                        data = node.latest_terrain
                    if data:
                        json_data = json.dumps(node._terrain_to_json(data))
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
                    self.send_header('Access-Control-Allow-Origin', '*')
                    self.end_headers()
                    import time
                    rate = node.get_parameter('update_rate_hz').value
                    interval = 1.0 / rate
                    try:
                        while rclpy.ok():
                            with node.data_lock:
                                occ = node.latest_occupancy
                                terr = node.latest_terrain
                            payload = {}
                            if occ:
                                payload['occupancy'] = node._occupancy_to_json(occ)
                            if terr:
                                payload['terrain'] = node._terrain_to_json(terr)
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
                        b'<p>Endpoints: /occupancy, /terrain, /stream</p>'
                        b'</body></html>')

            def log_message(self, format, *args):
                pass

        server = HTTPServer(('0.0.0.0', port), MapHandler)
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
