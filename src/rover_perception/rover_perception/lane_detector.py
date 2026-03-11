import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from rover_interfaces.msg import Lane
from geometry_msgs.msg import Point
import cv2
import numpy as np


class LaneDetector(Node):
    def __init__(self):
        super().__init__('lane_detector')

        # HSV thresholds for yellow lane detection
        self.declare_parameter('yellow_h_low', 20)                          #Yellow hue minimum        
        self.declare_parameter('yellow_h_high', 35)                         #Yellow hue maximum                 
        self.declare_parameter('yellow_s_low', 80)                          #Yellow saturation minimum  
        self.declare_parameter('yellow_s_high', 255)                        #Yellow saturation maximum
        self.declare_parameter('yellow_v_low', 80)                          #Yellow value minimum 
        self.declare_parameter('yellow_v_high', 255)                        #Yellow value maximum
        self.declare_parameter('min_contour_area', 500)                     #Minimum area of contours to consider as lane markings
        self.declare_parameter('roi_top_ratio', 0.4)                        #Top of region of interest as a ratio of image height (0.4 means bottom 60% of image)
        self.declare_parameter('gaussian_blur_size', 5)                     #Size of Gaussian blur kernel (must be odd)
        self.declare_parameter('camera_height_m', 0.3)                      #Height of camera above ground (m)
        self.declare_parameter('camera_pitch_rad', 0.5)                     #Pitch angle of camera (rad)

        self.bridge = CvBridge()
        self.camera_info = None
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        # Subscribers
        self.image_sub = self.create_subscription(
            Image, 'camera/color/image_raw', self.image_callback, 10)
        self.info_sub = self.create_subscription(
            CameraInfo, 'camera/color/camera_info', self.info_callback, 10)

        # Publishers
        self.lane_pub = self.create_publisher(Lane, 'perception/lane', 10)
        self.debug_pub = self.create_publisher(Image, 'perception/lane_debug', 5)

        self.get_logger().info('Lane detector initialized')

    def info_callback(self, msg):
        self.camera_info = msg
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        h, w = frame.shape[:2]

        # Region of interest: bottom portion of image where lane lines appear
        roi_top = int(h * self.get_parameter('roi_top_ratio').value)
        roi = frame[roi_top:h, :]

        # Convert to HSV and threshold for yellow
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        lower_yellow = np.array([
            self.get_parameter('yellow_h_low').value,
            self.get_parameter('yellow_s_low').value,
            self.get_parameter('yellow_v_low').value
        ])
        upper_yellow = np.array([
            self.get_parameter('yellow_h_high').value,
            self.get_parameter('yellow_s_high').value,
            self.get_parameter('yellow_v_high').value
        ])
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        # Morphological cleanup
        blur_size = self.get_parameter('gaussian_blur_size').value
        mask = cv2.GaussianBlur(mask, (blur_size, blur_size), 0)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        min_area = self.get_parameter('min_contour_area').value
        valid_contours = [c for c in contours if cv2.contourArea(c) > min_area]

        # Classify contours as left or right lane
        mid_x = w // 2
        left_points = []
        right_points = []

        for contour in valid_contours:
            M = cv2.moments(contour)
            if M['m00'] == 0:
                continue
            cx_contour = int(M['m10'] / M['m00'])

            # Extract bottom points of the contour for lane fitting
            pts = contour.reshape(-1, 2)
            if cx_contour < mid_x:
                left_points.extend(pts.tolist())
            else:
                right_points.extend(pts.tolist())

        # Build lane message
        lane_msg = Lane()
        lane_msg.header.stamp = msg.header.stamp
        lane_msg.header.frame_id = 'camera_color_optical_frame'

        debug_frame = roi.copy()

        # Fit lines to left and right lane boundary points
        left_line = None
        right_line = None

        if len(left_points) > 10:
            left_pts = np.array(left_points)
            left_line = cv2.fitLine(left_pts, cv2.DIST_L2, 0, 0.01, 0.01)
            lane_msg.left_detected = True
            for pt in left_pts[::5]:
                p = Point()
                p.x = float(pt[0])
                p.y = float(pt[1] + roi_top)
                lane_msg.left_boundary.append(p)
            # Draw on debug
            cv2.polylines(debug_frame, [left_pts.reshape(-1, 1, 2)], False, (0, 255, 0), 2)

        if len(right_points) > 10:
            right_pts = np.array(right_points)
            right_line = cv2.fitLine(right_pts, cv2.DIST_L2, 0, 0.01, 0.01)
            lane_msg.right_detected = True
            for pt in right_pts[::5]:
                p = Point()
                p.x = float(pt[0])
                p.y = float(pt[1] + roi_top)
                lane_msg.right_boundary.append(p)
            cv2.polylines(debug_frame, [right_pts.reshape(-1, 1, 2)], False, (0, 0, 255), 2)

        # Calculate lateral offset and heading error
        if left_line is not None and right_line is not None:
            # Compute lane center at the bottom of the ROI
            roi_h = roi.shape[0]
            left_x_bottom = self._line_x_at_y(left_line, roi_h - 1)
            right_x_bottom = self._line_x_at_y(right_line, roi_h - 1)
            center_x = (left_x_bottom + right_x_bottom) / 2.0
            lane_width_px = abs(right_x_bottom - left_x_bottom)

            # Lateral offset: positive means rover is right of center
            offset_px = mid_x - center_x
            if self.fx and lane_width_px > 0:
                # Approximate ground-plane conversion
                lane_msg.lateral_offset = offset_px * self.get_parameter('camera_height_m').value / self.fy
            else:
                lane_msg.lateral_offset = offset_px / (w / 2.0)

            # Heading error from average of lane line angles
            left_angle = np.arctan2(float(left_line[1]), float(left_line[0]))
            right_angle = np.arctan2(float(right_line[1]), float(right_line[0]))
            avg_angle = (left_angle + right_angle) / 2.0
            lane_msg.heading_error = avg_angle - np.pi / 2.0

            lane_msg.confidence = 0.9

            # Draw center line
            center_top_x = int((self._line_x_at_y(left_line, 0) + self._line_x_at_y(right_line, 0)) / 2.0)
            cv2.line(debug_frame, (int(center_x), roi_h - 1), (center_top_x, 0), (255, 255, 0), 2)
        elif left_line is not None or right_line is not None:
            # Single lane detected - estimate center from visible lane
            line = left_line if left_line is not None else right_line
            angle = np.arctan2(float(line[1]), float(line[0]))
            lane_msg.heading_error = angle - np.pi / 2.0
            lane_msg.lateral_offset = 0.0
            lane_msg.confidence = 0.5
        else:
            lane_msg.confidence = 0.0

        self.lane_pub.publish(lane_msg)

        # Publish debug image
        debug_msg = self.bridge.cv2_to_imgmsg(debug_frame, encoding='bgr8')
        debug_msg.header = msg.header
        self.debug_pub.publish(debug_msg)

    def _line_x_at_y(self, line, y):
        """Given fitLine output [vx, vy, x0, y0], compute x at given y."""
        vx, vy, x0, y0 = float(line[0]), float(line[1]), float(line[2]), float(line[3])
        if abs(vy) < 1e-6:
            return x0
        t = (y - y0) / vy
        return x0 + vx * t


def main(args=None):
    rclpy.init(args=args)
    node = LaneDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
