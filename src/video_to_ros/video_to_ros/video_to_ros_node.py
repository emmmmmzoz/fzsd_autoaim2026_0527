#!/usr/bin/env python3
import os
import glob
import yaml

import cv2
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo


def _find_workspace_root():
    """Walk up from the install prefix to find workspace root (works with --symlink-install)."""
    import ament_index_python
    # COLCON_PREFIX_PATH points to install dir
    prefix = os.environ.get('COLCON_PREFIX_PATH', '')
    if prefix and os.path.isdir(prefix):
        return prefix
    # Fallback: resolve from this file's path
    current = os.path.dirname(os.path.abspath(__file__))
    for _ in range(10):
        parent = os.path.dirname(current)
        if parent == current:
            break
        current = parent
        if os.path.isfile(os.path.join(current, 'src', 'video_to_ros', 'setup.py')):
            return current
    return None


def _find_video_in_workspace(video_name):
    """Search {workspace}/videos/ for video_name with any supported extension."""
    ws = _find_workspace_root()
    search_dirs = []
    if ws:
        search_dirs.append(os.path.join(ws, 'videos'))
    search_dirs.append('videos')

    extensions = ['.mp4', '.avi', '.mkv', '.mov', '.webm', '.flv', '.wmv']

    for videos_dir in search_dirs:
        if not os.path.isdir(videos_dir):
            continue
        for ext in extensions:
            candidate = os.path.join(videos_dir, video_name + ext)
            if os.path.isfile(candidate):
                return candidate

    # Also try the video_name as-is (might already have extension)
    for videos_dir in search_dirs:
        if not os.path.isdir(videos_dir):
            continue
        candidate = os.path.join(videos_dir, video_name)
        if os.path.isfile(candidate):
            return candidate

    raise FileNotFoundError(
        f'Cannot find video "{video_name}" in videos/ directory. '
        f'Searched: {search_dirs}. '
        f'Please provide video_path parameter with full path.')


def _resolve_camera_info_path(url):
    """Convert package:// URL or relative path to absolute filesystem path."""
    if not url:
        return None
    if url.startswith('package://'):
        from ament_index_python.packages import get_package_share_directory
        parts = url.replace('package://', '').split('/')
        pkg = parts[0]
        rel = '/'.join(parts[1:])
        return os.path.join(get_package_share_directory(pkg), rel)
    if os.path.isabs(url):
        return url
    return os.path.abspath(url)


class VideoToRos(Node):
    """Publish video frames as ROS2 Image + CameraInfo topics."""

    def __init__(self):
        super().__init__('video_to_ros')

        # -- parameters --
        self.declare_parameter('video_path', '')
        self.declare_parameter('video_name', 'test1')
        self.declare_parameter('fps', 30.0)
        self.declare_parameter('loop', True)
        self.declare_parameter('frame_id', 'aim_camera_optical_frame')
        self.declare_parameter('scale', 1.0)
        self.declare_parameter('camera_info_url', '')

        video_path = self.get_parameter('video_path').get_parameter_value().string_value
        video_name = self.get_parameter('video_name').get_parameter_value().string_value
        self.target_fps = self.get_parameter('fps').get_parameter_value().double_value
        self.loop = self.get_parameter('loop').get_parameter_value().bool_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.scale = self.get_parameter('scale').get_parameter_value().double_value
        camera_info_url = self.get_parameter('camera_info_url').get_parameter_value().string_value

        # -- resolve video path --
        if video_path:
            if not os.path.isfile(video_path):
                raise FileNotFoundError(f'Video file not found: {video_path}')
        else:
            video_path = _find_video_in_workspace(video_name)

        self.get_logger().info(f'Video path: {video_path}')

        # -- load camera info --
        if not camera_info_url:
            from ament_index_python.packages import get_package_share_directory
            camera_info_url = os.path.join(
                get_package_share_directory('rm_vision_bringup'),
                'config', 'camera_info.yaml')
        ci_path = _resolve_camera_info_path(camera_info_url)
        if not os.path.isfile(ci_path):
            raise FileNotFoundError(
                f'CameraInfo file not found: {ci_path}. '
                f'Set camera_info_url parameter to a valid path.')

        self.camera_info_msg = self._load_camera_info(ci_path)

        # -- open video --
        self.cap = cv2.VideoCapture(video_path)
        if not self.cap.isOpened():
            raise RuntimeError(f'Cannot open video: {video_path}')
        self.total_frames = int(self.cap.get(cv2.CAP_PROP_FRAME_COUNT))
        self.src_fps = self.cap.get(cv2.CAP_PROP_FPS)
        if self.src_fps <= 0:
            self.src_fps = 30.0
        actual_fps = min(self.target_fps, self.src_fps) if self.src_fps else self.target_fps
        self.get_logger().info(
            f'Video opened: {self.total_frames} frames, '
            f'source {self.src_fps:.1f} fps, publish {actual_fps:.1f} fps, '
            f'scale={self.scale:.2f}, loop={self.loop}')

        # -- publishers --
        self.bridge = CvBridge()
        self.image_pub = self.create_publisher(Image, '/image_raw', 10)
        self.camera_info_pub = self.create_publisher(CameraInfo, '/camera_info', 10)

        # Apply scale: if scale=0, auto-scale to match CameraInfo width
        if self.scale <= 0:
            vid_w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            self.scale = float(self.camera_info_msg.width) / max(vid_w, 1)
            self.get_logger().info(f'Auto scale: video {vid_w}px -> camera {self.camera_info_msg.width}px, factor={self.scale:.4f}')

        # -- timer --
        period = 1.0 / actual_fps
        self.timer = self.create_timer(period, self._publish_frame)
        self.get_logger().info('VideoToRos ready, publishing on /image_raw + /camera_info')

    def _load_camera_info(self, yaml_path):
        """Parse a standard ROS camera calibration YAML into a CameraInfo message."""
        with open(yaml_path, 'r') as f:
            data = yaml.safe_load(f)

        msg = CameraInfo()
        msg.height = data['image_height']
        msg.width = data['image_width']
        msg.distortion_model = data.get('distortion_model', 'plumb_bob')
        msg.k = data['camera_matrix']['data']
        msg.d = data['distortion_coefficients']['data']
        msg.r = data['rectification_matrix']['data']
        msg.p = data['projection_matrix']['data']
        self.get_logger().info(f'CameraInfo loaded from {yaml_path}: {msg.width}x{msg.height}')
        return msg

    def _publish_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            if self.loop:
                self.get_logger().info('Video ended, looping back to start')
                self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                ret, frame = self.cap.read()
                if not ret:
                    self.get_logger().error('Failed to re-read video after loop')
                    return
            else:
                self.get_logger().info('Video ended (loop=false), shutting down')
                rclpy.shutdown()
                return

        # resize
        if self.scale != 1.0:
            new_w = max(1, int(frame.shape[1] * self.scale))
            new_h = max(1, int(frame.shape[0] * self.scale))
            frame = cv2.resize(frame, (new_w, new_h))

        # publish
        now = self.get_clock().now().to_msg()
        img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        img_msg.header.stamp = now
        img_msg.header.frame_id = self.frame_id

        self.camera_info_msg.header.stamp = now
        self.camera_info_msg.header.frame_id = self.frame_id

        self.image_pub.publish(img_msg)
        self.camera_info_pub.publish(self.camera_info_msg)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = VideoToRos()
        rclpy.spin(node)
    except Exception as e:
        rclpy.logging.get_logger('video_to_ros').fatal(str(e))
        raise
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
