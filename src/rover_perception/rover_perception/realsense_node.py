import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, Imu
from cv_bridge import CvBridge
import pyrealsense2 as rs
import numpy as np


class RealsenseNode(Node):
    def __init__(self):
        super().__init__('realsense_node')

        # Parameters
        self.declare_parameter('color_width', 640)      #RGB image width
        self.declare_parameter('color_height', 480)     #RGB image height
        self.declare_parameter('depth_width', 640)      #Depth image width
        self.declare_parameter('depth_height', 480)     #Depth image height
        self.declare_parameter('fps', 30)               #FPS
        self.declare_parameter('enable_imu', True)      #Whether to publish IMU data
        self.declare_parameter('align_depth', True)     #Whether to publish depth aligned to color frame

        self.color_w = self.get_parameter('color_width').value
        self.color_h = self.get_parameter('color_height').value
        self.depth_w = self.get_parameter('depth_width').value
        self.depth_h = self.get_parameter('depth_height').value
        self.fps = self.get_parameter('fps').value
        self.enable_imu = self.get_parameter('enable_imu').value
        self.align_depth = self.get_parameter('align_depth').value


        # Publishers
        self.color_pub = self.create_publisher(Image, 'camera/color/image_raw', 10)                     #RGB image topic
        self.depth_pub = self.create_publisher(Image, 'camera/depth/image_raw', 10)                     #Depth image topic
        self.aligned_depth_pub = self.create_publisher(Image, 'camera/aligned_depth/image_raw', 10)     #Aligned depth image topic
        self.color_info_pub = self.create_publisher(CameraInfo, 'camera/color/camera_info', 10)         #Camera info for color stream
        self.depth_info_pub = self.create_publisher(CameraInfo, 'camera/depth/camera_info', 10)         #Camera info for depth stream
        if self.enable_imu:
            self.imu_pub = self.create_publisher(Imu, 'camera/imu', 10)                                 #IMU data topic

        self.bridge = CvBridge()
        self.pipeline = rs.pipeline()
        self.config = rs.config()


        # Configure streams
        self.config.enable_stream(rs.stream.color, self.color_w, self.color_h, rs.format.bgr8, self.fps)        #BGR Stream for OpenCV compatibility
        self.config.enable_stream(rs.stream.depth, self.depth_w, self.depth_h, rs.format.z16, self.fps)         #Depth stream
        if self.enable_imu:                                                                                     #IMU streams (accelerometer and gyroscope)
            self.config.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, 200)
            self.config.enable_stream(rs.stream.gyro, rs.format.motion_xyz32f, 200)


        # Align object
        self.align = rs.align(rs.stream.color) if self.align_depth else None


        # Start pipeline
        try:
            self.profile = self.pipeline.start(self.config)
            self.get_logger().info('RealSense pipeline started')
        except Exception as e:
            self.get_logger().error(f'Failed to start RealSense: {e}')
            raise

        # Get intrinsics for camera_info
        self._publish_camera_info_cached = {}

        # Timer for frame capture (visual streams)
        self.timer = self.create_timer(1.0 / self.fps, self.capture_frames)


    def capture_frames(self):                       #Main loop to capture frames from the RealSense camera and publish them
        try:
            frames = self.pipeline.wait_for_frames(timeout_ms=1000)
        except RuntimeError:
            self.get_logger().warn('Frame timeout')
            return

        now = self.get_clock().now().to_msg()

        # Align depth to color
        if self.align:
            aligned_frames = self.align.process(frames)
            depth_frame = aligned_frames.get_depth_frame()
            aligned_depth_frame = depth_frame
        else:
            depth_frame = frames.get_depth_frame()
            aligned_depth_frame = None

        color_frame = frames.get_color_frame()

        if not depth_frame or not color_frame:
            return

        # Publish color image
        color_image = np.asanyarray(color_frame.get_data())
        color_msg = self.bridge.cv2_to_imgmsg(color_image, encoding='bgr8')
        color_msg.header.stamp = now
        color_msg.header.frame_id = 'camera_color_optical_frame'
        self.color_pub.publish(color_msg)

        # Publish depth image
        depth_image = np.asanyarray(depth_frame.get_data())
        depth_msg = self.bridge.cv2_to_imgmsg(depth_image, encoding='16UC1')
        depth_msg.header.stamp = now
        depth_msg.header.frame_id = 'camera_depth_optical_frame'
        self.depth_pub.publish(depth_msg)

        # Publish aligned depth
        if aligned_depth_frame:
            aligned_msg = self.bridge.cv2_to_imgmsg(
                np.asanyarray(aligned_depth_frame.get_data()), encoding='16UC1')
            aligned_msg.header.stamp = now
            aligned_msg.header.frame_id = 'camera_color_optical_frame'
            self.aligned_depth_pub.publish(aligned_msg)

        # Publish camera info
        self._publish_camera_info(color_frame, self.color_info_pub, 'camera_color_optical_frame', now)
        self._publish_camera_info(depth_frame, self.depth_info_pub, 'camera_depth_optical_frame', now)

        # Publish IMU if enabled
        if self.enable_imu:
            accel_frame = frames.first_or_default(rs.stream.accel)
            gyro_frame = frames.first_or_default(rs.stream.gyro)
            if accel_frame and gyro_frame:
                imu_msg = Imu()
                imu_msg.header.stamp = now
                imu_msg.header.frame_id = 'camera_imu_optical_frame'
                accel = accel_frame.as_motion_frame().get_motion_data()
                gyro = gyro_frame.as_motion_frame().get_motion_data()
                imu_msg.linear_acceleration.x = float(accel.x)
                imu_msg.linear_acceleration.y = float(accel.y)
                imu_msg.linear_acceleration.z = float(accel.z)
                imu_msg.angular_velocity.x = float(gyro.x)
                imu_msg.angular_velocity.y = float(gyro.y)
                imu_msg.angular_velocity.z = float(gyro.z)
                self.imu_pub.publish(imu_msg)


    def _publish_camera_info(self, frame, publisher, frame_id, stamp):
        intrinsics = frame.profile.as_video_stream_profile().intrinsics
        msg = CameraInfo()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.width = intrinsics.width
        msg.height = intrinsics.height
        msg.distortion_model = 'plumb_bob'                          #Assuming standard distortion model - may need adjustment based on actual lens characteristics
        msg.d = [float(c) for c in intrinsics.coeffs]
        msg.k = [
            intrinsics.fx, 0.0, intrinsics.ppx,
            0.0, intrinsics.fy, intrinsics.ppy,
            0.0, 0.0, 1.0
        ]
        msg.p = [
            intrinsics.fx, 0.0, intrinsics.ppx, 0.0,
            0.0, intrinsics.fy, intrinsics.ppy, 0.0,
            0.0, 0.0, 1.0, 0.0
        ]
        msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        publisher.publish(msg)

    def destroy_node(self):
        self.pipeline.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RealsenseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
