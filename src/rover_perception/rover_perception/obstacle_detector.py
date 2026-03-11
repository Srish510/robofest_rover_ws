import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from rover_interfaces.msg import Obstacle
from geometry_msgs.msg import Point, Vector3
import cv2
import numpy as np
import time


class ObstacleDetector(Node):
    def __init__(self):
        super().__init__('obstacle_detector')

        # Parameters
        self.declare_parameter('min_depth_m', 0.3)                          #Minimum depth to consider (meters)
        self.declare_parameter('max_depth_m', 5.0)                          #Maximum depth to consider (meters)
        self.declare_parameter('obstacle_height_threshold_m', 0.10)         #Minimum height above ground plane to consider as obstacle (meters)
        self.declare_parameter('ground_plane_tolerance_m', 0.05)            #Tolerance for ground plane estimation (meters)
        self.declare_parameter('cluster_distance_m', 0.15)                  #Maximum distance between points to be considered in the same cluster (meters)
        self.declare_parameter('min_cluster_points', 50)                    #Minimum number of points in a cluster to be considered an obstacle
        self.declare_parameter('dynamic_tracking_window', 5)                #Number of frames to track dynamic obstacles for velocity estimation
        self.declare_parameter('velocity_threshold_m_s', 0.1)               #Minimum velocity (m/s) to consider an obstacle as dynamic
        self.declare_parameter('depth_scale', 0.001)                        #Depth scale to convert raw depth values to meters (depends on camera settings)

        self.bridge = CvBridge()
        self.camera_info = None
        self.fx = self.fy = self.cx = self.cy = None

        # For dynamic obstacle tracking: {obstacle_id: deque of (timestamp, position)}
        self.tracked_obstacles = {}
        self.next_obstacle_id = 0
        self.prev_centroids = []
        self.prev_time = None

        # Subscribers
        self.depth_sub = self.create_subscription(
            Image, 'camera/aligned_depth/image_raw', self.depth_callback, 10)
        self.info_sub = self.create_subscription(
            CameraInfo, 'camera/color/camera_info', self.info_callback, 10)

        # Publishers
        self.obstacle_pub = self.create_publisher(Obstacle, 'perception/obstacles', 10)
        self.debug_pub = self.create_publisher(Image, 'perception/obstacle_debug', 5)

        self.get_logger().info('Obstacle detector initialized')

    def info_callback(self, msg):
        self.camera_info = msg
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]

    def depth_callback(self, msg):
        if self.fx is None:
            return

        depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        depth_scale = self.get_parameter('depth_scale').value
        depth_m = depth_image.astype(np.float32) * depth_scale

        h, w = depth_m.shape
        min_d = self.get_parameter('min_depth_m').value
        max_d = self.get_parameter('max_depth_m').value

        # Create valid depth mask
        valid_mask = (depth_m > min_d) & (depth_m < max_d)

        # Convert depth to 3D point cloud (organized)
        u_coords = np.arange(w).reshape(1, -1).repeat(h, axis=0)
        v_coords = np.arange(h).reshape(-1, 1).repeat(w, axis=1)

        z = depth_m
        x = (u_coords - self.cx) * z / self.fx
        y = (v_coords - self.cy) * z / self.fy

        # Simple ground plane removal: assume camera is roughly level
        # Points above ground plane (in camera frame, y < threshold are obstacles)
        ground_tol = self.get_parameter('ground_plane_tolerance_m').value
        height_thresh = self.get_parameter('obstacle_height_threshold_m').value

        # In the camera frame, y-axis points down. Objects above ground have
        # y values less than the ground plane y.
        # Estimate ground plane as the median y at far distances
        far_mask = valid_mask & (z > 1.0) & (z < 3.0)
        if np.sum(far_mask) > 100:
            ground_y = np.median(y[far_mask])
        else:
            ground_y = np.max(y[valid_mask]) if np.any(valid_mask) else 0.3

        # Obstacle mask: points significantly above ground plane
        obstacle_mask = valid_mask & (y < (ground_y - height_thresh))

        # Convert obstacle mask to uint8 for contour detection
        obs_binary = (obstacle_mask.astype(np.uint8)) * 255

        # Clean up with morphological operations
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7))
        obs_binary = cv2.morphologyEx(obs_binary, cv2.MORPH_CLOSE, kernel)
        obs_binary = cv2.morphologyEx(obs_binary, cv2.MORPH_OPEN, kernel)

        # Find contours (clusters of obstacle pixels)
        contours, _ = cv2.findContours(obs_binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        min_pts = self.get_parameter('min_cluster_points').value
        current_centroids = []
        current_time = time.time()

        # Debug visualization
        debug_frame = cv2.applyColorMap(
            cv2.convertScaleAbs(depth_m, alpha=255.0 / max_d), cv2.COLORMAP_JET)

        for contour in contours:
            if cv2.contourArea(contour) < min_pts:
                continue

            # Bounding rect
            bx, by, bw, bh = cv2.boundingRect(contour)
            roi_mask = obstacle_mask[by:by + bh, bx:bx + bw]

            # Get 3D centroid of obstacle
            z_roi = z[by:by + bh, bx:bx + bw]
            x_roi = x[by:by + bh, bx:bx + bw]
            y_roi = y[by:by + bh, bx:bx + bw]

            obs_z = z_roi[roi_mask]
            obs_x = x_roi[roi_mask]
            obs_y = y_roi[roi_mask]

            if len(obs_z) == 0:
                continue

            cx_3d = float(np.median(obs_x))
            cy_3d = float(np.median(obs_y))
            cz_3d = float(np.median(obs_z))

            # Bounding box in 3D
            size_x = float(np.max(obs_x) - np.min(obs_x))
            size_y = float(np.max(obs_y) - np.min(obs_y))
            size_z = float(np.max(obs_z) - np.min(obs_z))

            distance = float(np.sqrt(cx_3d ** 2 + cy_3d ** 2 + cz_3d ** 2))
            bearing = float(np.arctan2(cx_3d, cz_3d))

            current_centroids.append(np.array([cx_3d, cy_3d, cz_3d]))

            # Check for dynamic obstacle by matching with previous frame centroids
            is_dynamic = False
            velocity = Vector3()
            vel_thresh = self.get_parameter('velocity_threshold_m_s').value

            if self.prev_centroids and self.prev_time:
                dt = current_time - self.prev_time
                if dt > 0:
                    current_pos = np.array([cx_3d, cy_3d, cz_3d])
                    min_dist = float('inf')
                    best_prev = None
                    for prev_c in self.prev_centroids:
                        d = np.linalg.norm(current_pos - prev_c)
                        if d < min_dist:
                            min_dist = d
                            best_prev = prev_c

                    if best_prev is not None and min_dist < 1.0:  # max match distance
                        vel = (current_pos - best_prev) / dt
                        speed = np.linalg.norm(vel)
                        if speed > vel_thresh:
                            is_dynamic = True
                            velocity.x = float(vel[0])
                            velocity.y = float(vel[1])
                            velocity.z = float(vel[2])

            # Build and publish obstacle message
            obs_msg = Obstacle()
            obs_msg.header.stamp = msg.header.stamp
            obs_msg.header.frame_id = 'camera_color_optical_frame'
            obs_msg.position = Point(x=cx_3d, y=cy_3d, z=cz_3d)
            obs_msg.size = Vector3(x=size_x, y=size_y, z=size_z)
            obs_msg.distance = distance
            obs_msg.bearing = bearing
            obs_msg.velocity = velocity
            obs_msg.is_dynamic = is_dynamic
            obs_msg.classification = Obstacle.UNKNOWN
            obs_msg.confidence = min(1.0, len(obs_z) / 500.0)

            self.obstacle_pub.publish(obs_msg)

            # Debug drawing
            color = (0, 0, 255) if is_dynamic else (0, 255, 0)
            cv2.rectangle(debug_frame, (bx, by), (bx + bw, by + bh), color, 2)
            label = f'{distance:.1f}m'
            if is_dynamic:
                label += ' DYN'
            cv2.putText(debug_frame, label, (bx, by - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

        # Update tracking state
        self.prev_centroids = current_centroids
        self.prev_time = current_time

        # Publish debug image
        debug_msg = self.bridge.cv2_to_imgmsg(debug_frame, encoding='bgr8')
        debug_msg.header = msg.header
        self.debug_pub.publish(debug_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
