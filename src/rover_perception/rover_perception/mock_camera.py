"""
Mock camera node - generates synthetic color+depth images with yellow lanes,
a moving obstacle, and periodic QR codes. Replaces the realsense_node for testing.
"""

#NOTE: This code is fully AI-generated and not actually used in the final system, but it can be useful

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
import time


class MockCamera(Node):
    def __init__(self):
        super().__init__('mock_camera')

        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30)
        self.declare_parameter('lane_sway_amplitude', 40.0)
        self.declare_parameter('lane_sway_period_sec', 8.0)
        self.declare_parameter('show_obstacle', True)
        self.declare_parameter('obstacle_cycle_sec', 6.0)
        self.declare_parameter('obstacle_course', 'single_moving')
        self.declare_parameter('show_qr', True)
        self.declare_parameter('qr_interval_sec', 15.0)
        self.declare_parameter('qr_display_duration_sec', 3.0)

        self.w = self.get_parameter('width').value
        self.h = self.get_parameter('height').value
        self.fps = self.get_parameter('fps').value

        self.bridge = CvBridge()
        self.start_time = time.time()

        # Approximate D455 intrinsics
        self.fx = 615.0
        self.fy = 615.0
        self.cx = self.w / 2.0
        self.cy = self.h / 2.0

        self.color_pub = self.create_publisher(Image, 'camera/color/image_raw', 10)
        self.depth_pub = self.create_publisher(Image, 'camera/depth/image_raw', 10)
        self.aligned_depth_pub = self.create_publisher(
            Image, 'camera/aligned_depth/image_raw', 10)
        self.color_info_pub = self.create_publisher(
            CameraInfo, 'camera/color/camera_info', 10)
        self.depth_info_pub = self.create_publisher(
            CameraInfo, 'camera/depth/camera_info', 10)

        self.timer = self.create_timer(1.0 / self.fps, self.publish_frame)
        self.get_logger().info(
            f'Mock camera started: {self.w}x{self.h} @ {self.fps}fps')

    def _draw_obstacle(self, color, depth, cx, cy, half_w, half_h, depth_mm):
        x1 = max(0, int(cx - half_w))
        x2 = min(self.w, int(cx + half_w))
        y1 = max(0, int(cy - half_h))
        y2 = min(self.h, int(cy + half_h))
        if x2 <= x1 or y2 <= y1:
            return

        cv2.rectangle(color, (x1, y1), (x2, y2), (30, 60, 120), -1)
        cv2.rectangle(color, (x1, y1), (x2, y2), (20, 40, 80), 2)
        depth[y1:y2, x1:x2] = int(depth_mm)

    def publish_frame(self):
        now = self.get_clock().now().to_msg()
        t = time.time() - self.start_time

        color_img, depth_img = self._generate_scene(t)

        color_msg = self.bridge.cv2_to_imgmsg(color_img, encoding='bgr8')
        color_msg.header.stamp = now
        color_msg.header.frame_id = 'camera_color_optical_frame'
        self.color_pub.publish(color_msg)

        depth_msg = self.bridge.cv2_to_imgmsg(depth_img, encoding='16UC1')
        depth_msg.header.stamp = now
        depth_msg.header.frame_id = 'camera_depth_optical_frame'
        self.depth_pub.publish(depth_msg)

        aligned_msg = self.bridge.cv2_to_imgmsg(depth_img, encoding='16UC1')
        aligned_msg.header.stamp = now
        aligned_msg.header.frame_id = 'camera_color_optical_frame'
        self.aligned_depth_pub.publish(aligned_msg)

        self._publish_info(self.color_info_pub, 'camera_color_optical_frame', now)
        self._publish_info(self.depth_info_pub, 'camera_depth_optical_frame', now)

    def _generate_scene(self, t):
        w, h = self.w, self.h
        color = np.zeros((h, w, 3), dtype=np.uint8)

        # Grey road
        color[:, :] = (80, 80, 80)

        # Sky
        horizon_y = int(h * 0.35)
        color[0:horizon_y, :] = (180, 130, 100)

        # ─── Yellow lane lines ───
        sway_amp = self.get_parameter('lane_sway_amplitude').value
        sway_period = self.get_parameter('lane_sway_period_sec').value
        sway = sway_amp * math.sin(2.0 * math.pi * t / sway_period)

        lane_half = 150
        center_x = w // 2 + int(sway)
        vanish_y = horizon_y + 10

        for side_offset in [-lane_half, lane_half]:
            side_x = center_x + side_offset
            pts = []
            for row in range(h - 1, vanish_y, -5):
                frac = (h - row) / float(h - vanish_y)
                px = int(w // 2 + (side_x - w // 2) * frac + sway * (1.0 - frac * 0.5))
                pts.append((px, row))
            if len(pts) > 1:
                pts_arr = np.array(pts, dtype=np.int32)
                for i in range(len(pts_arr) - 1):
                    frac_i = (h - pts_arr[i][1]) / float(h - vanish_y)
                    thick = max(2, int(20 * frac_i))
                    cv2.line(color, tuple(pts_arr[i]), tuple(pts_arr[i + 1]),
                             (0, 255, 255), thick)  # yellow BGR

        # ─── Depth image ───
        depth = np.zeros((h, w), dtype=np.uint16)
        for row in range(vanish_y, h):
            frac = (row - vanish_y) / float(h - vanish_y)
            d_m = 5.0 - frac * 4.7
            depth[row, :] = int(d_m * 1000)

        # ─── Obstacle course ───
        if self.get_parameter('show_obstacle').value:
            obs_cycle = max(0.5, float(self.get_parameter('obstacle_cycle_sec').value))
            obs_phase = (t % obs_cycle) / obs_cycle
            course = str(self.get_parameter('obstacle_course').value).strip().lower()

            if course == 'slalom':
                y_positions = [int(h * 0.48), int(h * 0.58), int(h * 0.68)]
                amplitudes = [90, 75, 60]
                phase_offsets = [0.0, 0.33, 0.66]
                depth_values = [2300, 2000, 1700]
                for y, amp, phase_offset, d_mm in zip(
                        y_positions, amplitudes, phase_offsets, depth_values):
                    cx = center_x + int(amp * math.sin(2.0 * math.pi * (obs_phase + phase_offset)))
                    self._draw_obstacle(color, depth, cx, y, 22, 26, d_mm)

            elif course == 'chicane':
                self._draw_obstacle(color, depth, center_x - 95, int(h * 0.54), 32, 34, 2200)
                self._draw_obstacle(color, depth, center_x + 105, int(h * 0.66), 32, 34, 1800)

            elif course == 'center_block':
                self._draw_obstacle(color, depth, center_x, int(h * 0.60), 38, 44, 1700)
                gate_side = -1 if obs_phase < 0.5 else 1
                self._draw_obstacle(color, depth, center_x + gate_side * 120, int(h * 0.74), 26, 28, 1400)

            else:
                # single_moving (default): one obstacle oscillating left-right
                obs_cx = int(center_x + 80 * math.sin(2.0 * math.pi * obs_phase))
                self._draw_obstacle(color, depth, obs_cx, int(h * 0.55), 25, 30, 2000)

            cv2.putText(
                color,
                f'course: {course}',
                (12, 28),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (235, 235, 235),
                2,
                cv2.LINE_AA,
            )

        # ─── QR Code (periodic) ───
        if self.get_parameter('show_qr').value:
            qr_interval = self.get_parameter('qr_interval_sec').value
            qr_dur = self.get_parameter('qr_display_duration_sec').value
            if (t % qr_interval) < qr_dur:
                cp_num = int(t / qr_interval) + 1
                self._draw_qr_placeholder(color, f'CP_{cp_num:03d}',
                                           center_x, int(h * 0.45))

        # Sensor noise
        noise = np.random.randint(-5, 6, color.shape, dtype=np.int16)
        color = np.clip(color.astype(np.int16) + noise, 0, 255).astype(np.uint8)

        return color, depth

    def _draw_qr_placeholder(self, img, text, cx, cy):
        """Draw a simple QR-like pattern. For real QR decoding, use qrencode lib."""
        s = 80
        x1, y1 = max(0, cx - s // 2), max(0, cy - s // 2)
        x2, y2 = min(img.shape[1], x1 + s), min(img.shape[0], y1 + s)
        img[y1:y2, x1:x2] = (255, 255, 255)

        # 3 finder patterns (corners of a QR code)
        for fx, fy in [(x1 + 5, y1 + 5), (x2 - 19, y1 + 5), (x1 + 5, y2 - 19)]:
            cv2.rectangle(img, (fx, fy), (fx + 14, fy + 14), (0, 0, 0), -1)
            cv2.rectangle(img, (fx + 3, fy + 3), (fx + 11, fy + 11), (255, 255, 255), -1)
            cv2.rectangle(img, (fx + 5, fy + 5), (fx + 9, fy + 9), (0, 0, 0), -1)

        cv2.putText(img, text, (x1 + 8, y2 - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 1)

    def _publish_info(self, pub, frame_id, stamp):
        msg = CameraInfo()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.width = self.w
        msg.height = self.h
        msg.distortion_model = 'plumb_bob'
        msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        msg.k = [self.fx, 0.0, self.cx, 0.0, self.fy, self.cy, 0.0, 0.0, 1.0]
        msg.p = [self.fx, 0.0, self.cx, 0.0, 0.0, self.fy, self.cy, 0.0,
                 0.0, 0.0, 1.0, 0.0]
        msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MockCamera()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()