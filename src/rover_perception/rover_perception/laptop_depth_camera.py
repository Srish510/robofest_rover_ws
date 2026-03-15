import time
import sys

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image


class LaptopDepthCameraNode(Node):
    def __init__(self):
        super().__init__('laptop_depth_camera')

        self.declare_parameter('camera_index', 0)
        self.declare_parameter('camera_backend', 'auto')
        self.declare_parameter('auto_find_camera', True)
        self.declare_parameter('max_camera_index', 4)
        self.declare_parameter('color_width', 640)
        self.declare_parameter('color_height', 480)
        self.declare_parameter('fps', 15)
        self.declare_parameter('depth_fps', 5)
        self.declare_parameter('depth_model', 'midas')
        self.declare_parameter('midas_model_type', 'MiDaS_small')
        self.declare_parameter('depth_anything_model', 'LiheYoung/depth-anything-small-hf')
        self.declare_parameter('prefer_fast_model', True)
        self.declare_parameter('inference_width', 256)
        self.declare_parameter('min_depth_m', 0.3)
        self.declare_parameter('max_depth_m', 5.0)

        self.camera_index = int(self.get_parameter('camera_index').value)
        self.camera_backend = str(self.get_parameter('camera_backend').value).lower()
        self.auto_find_camera = bool(self.get_parameter('auto_find_camera').value)
        self.max_camera_index = max(0, int(self.get_parameter('max_camera_index').value))
        self.color_width = int(self.get_parameter('color_width').value)
        self.color_height = int(self.get_parameter('color_height').value)
        self.fps = float(self.get_parameter('fps').value)
        self.depth_fps = max(0.5, float(self.get_parameter('depth_fps').value))
        self.depth_model = str(self.get_parameter('depth_model').value).lower()
        self.midas_model_type = str(self.get_parameter('midas_model_type').value)
        self.depth_anything_model = str(
            self.get_parameter('depth_anything_model').value)
        self.prefer_fast_model = bool(self.get_parameter('prefer_fast_model').value)
        self.inference_width = max(128, int(self.get_parameter('inference_width').value))
        self.min_depth_m = float(self.get_parameter('min_depth_m').value)
        self.max_depth_m = float(self.get_parameter('max_depth_m').value)

        self.bridge = CvBridge()

        self.color_pub = self.create_publisher(Image, 'camera/color/image_raw', 10)
        self.depth_pub = self.create_publisher(Image, 'camera/depth/image_raw', 10)
        self.aligned_depth_pub = self.create_publisher(
            Image, 'camera/aligned_depth/image_raw', 10)
        self.color_info_pub = self.create_publisher(
            CameraInfo, 'camera/color/camera_info', 10)
        self.depth_info_pub = self.create_publisher(
            CameraInfo, 'camera/depth/camera_info', 10)

        self.cap = self._open_camera()
        if self.cap is None:
            raise RuntimeError(
                f'Failed to open laptop camera. Requested index={self.camera_index}, '
                f'backend={self.camera_backend}, auto_find_camera={self.auto_find_camera}')

        self.model_backend = 'fallback'
        self.model = None
        self.model_transform = None
        self.model_device = None
        self.da_processor = None
        self._load_depth_model()

        self.last_depth_u16 = np.full(
            (self.color_height, self.color_width),
            int(self.max_depth_m * 1000.0), dtype=np.uint16)
        self.last_depth_time = 0.0

        self.timer = self.create_timer(1.0 / self.fps, self.capture_and_publish)

        self.get_logger().info(
            f'Laptop depth camera started: {self.color_width}x{self.color_height} @ {self.fps}fps, '
            f'depth={self.model_backend}')

    def _backend_candidates(self):
        if self.camera_backend == 'v4l2':
            return [('v4l2', cv2.CAP_V4L2)]
        if self.camera_backend == 'gstreamer':
            return [('gstreamer', cv2.CAP_GSTREAMER)]
        if self.camera_backend == 'ffmpeg':
            return [('ffmpeg', cv2.CAP_FFMPEG)]
        if self.camera_backend == 'any':
            return [('any', cv2.CAP_ANY)]

        # auto: on Linux prefer v4l2 first to avoid gstreamer webcam pipeline issues
        if sys.platform.startswith('linux'):
            return [
                ('v4l2', cv2.CAP_V4L2),
                ('any', cv2.CAP_ANY),
                ('gstreamer', cv2.CAP_GSTREAMER),
            ]
        return [('any', cv2.CAP_ANY)]

    def _try_open(self, index: int, backend_name: str, backend_code: int):
        cap = cv2.VideoCapture(index, backend_code)
        if not cap.isOpened():
            cap.release()
            return None

        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.color_width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.color_height)
        cap.set(cv2.CAP_PROP_FPS, self.fps)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        ok, frame = cap.read()
        if not ok or frame is None:
            cap.release()
            return None

        self.get_logger().info(
            f'Opened webcam index {index} with backend {backend_name}')
        return cap

    def _open_camera(self):
        backends = self._backend_candidates()
        indices = [self.camera_index]
        if self.auto_find_camera:
            indices.extend([
                index for index in range(self.max_camera_index + 1)
                if index != self.camera_index
            ])

        for index in indices:
            for backend_name, backend_code in backends:
                cap = self._try_open(index, backend_name, backend_code)
                if cap is not None:
                    self.camera_index = index
                    return cap

        self.get_logger().error(
            f'Unable to open any webcam. Tried indices={indices}, '
            f'backends={[name for name, _ in backends]}')
        return None

    def _load_depth_model(self):
        try:
            import torch
        except Exception:
            self.get_logger().warn(
                'torch not available. Falling back to heuristic pseudo-depth.')
            return

        self.model_device = 'cuda' if torch.cuda.is_available() else 'cpu'

        if self.prefer_fast_model:
            if self.depth_model != 'midas':
                self.get_logger().warn(
                    'prefer_fast_model=true: overriding depth_model to midas for better FPS')
                self.depth_model = 'midas'
            if self.midas_model_type != 'MiDaS_small':
                self.get_logger().warn(
                    'prefer_fast_model=true: overriding midas_model_type to MiDaS_small for better FPS')
                self.midas_model_type = 'MiDaS_small'

        if self.depth_model == 'depth_anything':
            try:
                from transformers import AutoImageProcessor, AutoModelForDepthEstimation

                self.da_processor = AutoImageProcessor.from_pretrained(
                    self.depth_anything_model)
                self.model = AutoModelForDepthEstimation.from_pretrained(
                    self.depth_anything_model).to(self.model_device)
                self.model.eval()
                self.model_backend = 'depth_anything'
                self.get_logger().info(
                    f'Depth backend: Depth Anything ({self.depth_anything_model}) on {self.model_device}')
                return
            except Exception as exc:
                self.get_logger().warn(
                    f'Depth Anything load failed: {exc}. Trying MiDaS...')

        try:
            self.model = torch.hub.load('intel-isl/MiDaS', self.midas_model_type)
            self.model.to(self.model_device)
            self.model.eval()
            transforms = torch.hub.load('intel-isl/MiDaS', 'transforms')
            if self.midas_model_type in ('DPT_Large', 'DPT_Hybrid', 'DPT_BEiT_L_512'):
                self.model_transform = transforms.dpt_transform
            else:
                self.model_transform = transforms.small_transform
            self.model_backend = 'midas'
            self.get_logger().info(
                f'Depth backend: MiDaS ({self.midas_model_type}) on {self.model_device}')
        except Exception as exc:
            self.get_logger().warn(
                f'MiDaS load failed: {exc}. Using heuristic pseudo-depth fallback.')

    def _prepare_inference_rgb(self, rgb: np.ndarray) -> np.ndarray:
        h, w = rgb.shape[:2]
        if w <= self.inference_width:
            return rgb
        new_w = self.inference_width
        new_h = max(64, int((h / float(w)) * new_w))
        return cv2.resize(rgb, (new_w, new_h), interpolation=cv2.INTER_AREA)

    def _predict_depth_u16(self, bgr_frame: np.ndarray) -> np.ndarray:
        now = time.time()
        if (now - self.last_depth_time) < (1.0 / self.depth_fps):
            return self.last_depth_u16

        depth_m = None
        rgb = cv2.cvtColor(bgr_frame, cv2.COLOR_BGR2RGB)
        inference_rgb = self._prepare_inference_rgb(rgb)

        if self.model_backend == 'midas':
            try:
                import torch

                input_batch = self.model_transform(inference_rgb).to(self.model_device)
                with torch.no_grad():
                    prediction = self.model(input_batch)
                    prediction = torch.nn.functional.interpolate(
                        prediction.unsqueeze(1),
                        size=rgb.shape[:2],
                        mode='bicubic',
                        align_corners=False,
                    ).squeeze()

                inv_depth = prediction.cpu().numpy().astype(np.float32)
                p2, p98 = np.percentile(inv_depth, (2, 98))
                inv_depth = np.clip((inv_depth - p2) / max(p98 - p2, 1e-6), 0.0, 1.0)
                depth_m = self.min_depth_m + (1.0 - inv_depth) * (
                    self.max_depth_m - self.min_depth_m)
            except Exception as exc:
                self.get_logger().warn(
                    f'MiDaS inference failed ({exc}), switching to fallback depth.')
                self.model_backend = 'fallback'

        elif self.model_backend == 'depth_anything':
            try:
                import torch

                inputs = self.da_processor(images=inference_rgb, return_tensors='pt')
                for key in inputs:
                    inputs[key] = inputs[key].to(self.model_device)

                with torch.no_grad():
                    outputs = self.model(**inputs)
                    predicted = outputs.predicted_depth
                    predicted = torch.nn.functional.interpolate(
                        predicted.unsqueeze(1),
                        size=rgb.shape[:2],
                        mode='bicubic',
                        align_corners=False,
                    ).squeeze(1)

                rel_depth = predicted.squeeze().cpu().numpy().astype(np.float32)
                p2, p98 = np.percentile(rel_depth, (2, 98))
                rel_depth = np.clip((rel_depth - p2) / max(p98 - p2, 1e-6), 0.0, 1.0)
                depth_m = self.min_depth_m + (1.0 - rel_depth) * (
                    self.max_depth_m - self.min_depth_m)
            except Exception as exc:
                self.get_logger().warn(
                    f'Depth Anything inference failed ({exc}), switching to fallback depth.')
                self.model_backend = 'fallback'

        if depth_m is None:
            gray = cv2.cvtColor(bgr_frame, cv2.COLOR_BGR2GRAY).astype(np.float32) / 255.0
            rows = np.linspace(0.0, 1.0, self.color_height, dtype=np.float32)
            row_prior = np.tile(rows.reshape(-1, 1), (1, self.color_width))
            rel = 0.7 * (1.0 - row_prior) + 0.3 * (1.0 - gray)
            rel = np.clip(rel, 0.0, 1.0)
            depth_m = self.min_depth_m + rel * (self.max_depth_m - self.min_depth_m)

        depth_u16 = np.clip(depth_m * 1000.0, 0.0, 65535.0).astype(np.uint16)
        self.last_depth_u16 = depth_u16
        self.last_depth_time = now
        return depth_u16

    def _make_camera_info(self, frame_id: str, stamp):
        msg = CameraInfo()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.width = self.color_width
        msg.height = self.color_height
        msg.distortion_model = 'plumb_bob'
        msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]

        fx = 0.9 * self.color_width
        fy = 0.9 * self.color_width
        cx = self.color_width / 2.0
        cy = self.color_height / 2.0

        msg.k = [fx, 0.0, cx,
                 0.0, fy, cy,
                 0.0, 0.0, 1.0]
        msg.p = [fx, 0.0, cx, 0.0,
                 0.0, fy, cy, 0.0,
                 0.0, 0.0, 1.0, 0.0]
        msg.r = [1.0, 0.0, 0.0,
                 0.0, 1.0, 0.0,
                 0.0, 0.0, 1.0]
        return msg

    def capture_and_publish(self):
        ok, frame = self.cap.read()
        if not ok or frame is None:
            self.get_logger().warn('Failed to read laptop camera frame')
            return

        frame = cv2.resize(frame, (self.color_width, self.color_height))
        depth_u16 = self._predict_depth_u16(frame)

        now = self.get_clock().now().to_msg()

        color_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        color_msg.header.stamp = now
        color_msg.header.frame_id = 'camera_color_optical_frame'
        self.color_pub.publish(color_msg)

        depth_msg = self.bridge.cv2_to_imgmsg(depth_u16, encoding='16UC1')
        depth_msg.header.stamp = now
        depth_msg.header.frame_id = 'camera_depth_optical_frame'
        self.depth_pub.publish(depth_msg)

        aligned_msg = self.bridge.cv2_to_imgmsg(depth_u16, encoding='16UC1')
        aligned_msg.header.stamp = now
        aligned_msg.header.frame_id = 'camera_color_optical_frame'
        self.aligned_depth_pub.publish(aligned_msg)

        color_info = self._make_camera_info('camera_color_optical_frame', now)
        depth_info = self._make_camera_info('camera_depth_optical_frame', now)
        self.color_info_pub.publish(color_info)
        self.depth_info_pub.publish(depth_info)

    def destroy_node(self):
        if self.cap is not None:
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LaptopDepthCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
