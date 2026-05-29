#!/usr/bin/env python3
import os
import time
import yaml

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo


def _find_workspace_root():
    """Walk up from the install prefix to find workspace root (works with --symlink-install)."""
    import ament_index_python
    prefix = os.environ.get('COLCON_PREFIX_PATH', '')
    if prefix and os.path.isdir(prefix):
        return prefix
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
        self.declare_parameter('respect_source_fps', False)
        self.declare_parameter('profile', False)

        video_path = self.get_parameter('video_path').get_parameter_value().string_value
        video_name = self.get_parameter('video_name').get_parameter_value().string_value
        self.target_fps = self.get_parameter('fps').get_parameter_value().double_value
        self.loop = self.get_parameter('loop').get_parameter_value().bool_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.scale = self.get_parameter('scale').get_parameter_value().double_value
        self.respect_source_fps = self.get_parameter('respect_source_fps').get_parameter_value().bool_value
        self.profile = self.get_parameter('profile').get_parameter_value().bool_value
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

        self._orig_camera_info_msg = self._load_camera_info(ci_path)

        # -- open video --
        self.cap = cv2.VideoCapture(video_path)
        if not self.cap.isOpened():
            raise RuntimeError(f'Cannot open video: {video_path}')
        self.total_frames = int(self.cap.get(cv2.CAP_PROP_FRAME_COUNT))
        self.src_fps = self.cap.get(cv2.CAP_PROP_FPS)
        if self.src_fps <= 0:
            self.src_fps = 30.0

        # -- FPS logic (fixed: respect_source_fps controls clamping) --
        if self.respect_source_fps:
            actual_fps = min(self.target_fps, self.src_fps)
        else:
            actual_fps = self.target_fps

        self.get_logger().info(
            f'Video opened: {self.total_frames} frames, '
            f'source {self.src_fps:.1f} fps, publish {actual_fps:.1f} fps, '
            f'scale={self.scale:.2f}, loop={self.loop}, '
            f'respect_source_fps={self.respect_source_fps}, profile={self.profile}')

        # -- apply scale --
        self._setup_scale()

        # -- camera_info_msg: scaled once at init time (not every frame) --
        self.camera_info_msg = self._build_scaled_camera_info()

        # -- publishers --
        self.bridge = CvBridge()
        self.image_pub = self.create_publisher(Image, '/image_raw', 10)
        self.camera_info_pub = self.create_publisher(CameraInfo, '/camera_info', 10)

        # -- profile state --
        self._profile_frame_count = 0
        self._profile_last_log_time = time.perf_counter()
        self._profile_accum = {'read': 0.0, 'resize': 0.0, 'bridge': 0.0,
                               'publish': 0.0, 'total': 0.0}

        # -- timer --
        period = 1.0 / actual_fps if actual_fps > 0 else 1.0 / 30.0
        self.timer = self.create_timer(period, self._publish_frame)
        self.get_logger().info('VideoToRos ready, publishing on /image_raw + /camera_info')

    # ------------------------------------------------------------------
    # CameraInfo helpers
    # ------------------------------------------------------------------

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
        self.get_logger().info(
            f'CameraInfo loaded from {yaml_path}: {msg.width}x{msg.height}')
        return msg

    def _setup_scale(self):
        """Resolve self.scale and store target dimensions."""
        vid_w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        vid_h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self._orig_video_w = vid_w
        self._orig_video_h = vid_h

        if self.scale <= 0:
            self.scale = float(self._orig_camera_info_msg.width) / max(vid_w, 1)
            self.get_logger().info(
                f'Auto scale: video {vid_w}x{vid_h} -> '
                f'camera {self._orig_camera_info_msg.width}x'
                f'{self._orig_camera_info_msg.height}, factor={self.scale:.4f}')

        if self.scale != 1.0:
            self._target_w = max(1, int(vid_w * self.scale))
            self._target_h = max(1, int(vid_h * self.scale))
        else:
            self._target_w = vid_w
            self._target_h = vid_h

        self.get_logger().info(
            f'Output resolution: {self._target_w}x{self._target_h} '
            f'(scale={self.scale:.4f})')

    def _build_scaled_camera_info(self):
        """Return a CameraInfo message with K/P matrices scaled by self.scale.

        Called once at init — never accumulates rounding error.
        """
        if self.scale == 1.0:
            return self._orig_camera_info_msg

        s = float(self.scale)
        orig = self._orig_camera_info_msg
        msg = CameraInfo()
        msg.header = orig.header
        msg.height = max(1, int(orig.height * s))
        msg.width = max(1, int(orig.width * s))
        msg.distortion_model = orig.distortion_model
        msg.d = orig.d  # distortion is scale-invariant

        # R stays identity / unchanged
        msg.r = orig.r

        # K: 3x3 row-major [fx, 0, cx, 0, fy, cy, 0, 0, 1]
        k = list(orig.k)
        k[0] *= s   # fx
        k[2] *= s   # cx
        k[4] *= s   # fy
        k[5] *= s   # cy
        msg.k = k

        # P: 3x4 row-major [fx', 0, cx', Tx, 0, fy', cy', Ty, 0, 0, 1, 0]
        p = list(orig.p)
        p[0] *= s   # fx'
        p[2] *= s   # cx'
        p[5] *= s   # fy'
        p[6] *= s   # cy'
        msg.p = p

        self.get_logger().info(
            f'Scaled CameraInfo: {orig.width}x{orig.height} -> '
            f'{msg.width}x{msg.height} (factor={s:.4f})')
        self.get_logger().info(
            f'  K: [{k[0]:.2f}, 0, {k[2]:.2f}, 0, {k[4]:.2f}, {k[5]:.2f}]')
        self.get_logger().info(
            f'  P: [{p[0]:.2f}, 0, {p[2]:.2f}, 0, {p[5]:.2f}, {p[6]:.2f}]')

        return msg

    # ------------------------------------------------------------------
    # Profiling helpers
    # ------------------------------------------------------------------

    def _profile_report(self, read_us, resize_us, bridge_us, pub_us, total_us):
        self._profile_accum['read'] += read_us
        self._profile_accum['resize'] += resize_us
        self._profile_accum['bridge'] += bridge_us
        self._profile_accum['publish'] += pub_us
        self._profile_accum['total'] += total_us
        self._profile_frame_count += 1

        now = time.perf_counter()
        elapsed = now - self._profile_last_log_time
        if self._profile_frame_count >= 30 or elapsed >= 1.0:
            n = self._profile_frame_count
            if n == 0:
                return
            r = self._profile_accum['read'] / n
            rz = self._profile_accum['resize'] / n
            b = self._profile_accum['bridge'] / n
            p = self._profile_accum['publish'] / n
            t = self._profile_accum['total'] / n
            fps_eff = 1.0 / (t / 1e6) if t > 0 else 0.0
            self.get_logger().info(
                f'[profile] n={n} | '
                f'read={r:.0f}us resize={rz:.0f}us bridge={b:.0f}us '
                f'publish={p:.0f}us | total={t:.0f}us '
                f'({t/1000:.1f}ms) | effective_max_fps={fps_eff:.1f}')
            self._profile_frame_count = 0
            self._profile_last_log_time = now
            for k in self._profile_accum:
                self._profile_accum[k] = 0.0

    # ------------------------------------------------------------------
    # Main publish callback
    # ------------------------------------------------------------------

    def _publish_frame(self):
        t_total_start = time.perf_counter()

        # -- 1. read --
        t0 = time.perf_counter()
        ret, frame = self.cap.read()
        t_read = time.perf_counter() - t0

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

        # -- 2. resize (skip when scale==1.0) --
        t0 = time.perf_counter()
        if self.scale != 1.0:
            frame = cv2.resize(frame, (self._target_w, self._target_h))
        t_resize = time.perf_counter() - t0

        # -- 3. cv_bridge conversion --
        t0 = time.perf_counter()
        # Ensure C-contiguous for fastest tobytes() path inside cv_bridge
        if not frame.flags['C_CONTIGUOUS']:
            frame = np.ascontiguousarray(frame)
        img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        t_bridge = time.perf_counter() - t0

        # -- 4. stamp & publish --
        t0 = time.perf_counter()
        now = self.get_clock().now().to_msg()
        img_msg.header.stamp = now
        img_msg.header.frame_id = self.frame_id

        self.camera_info_msg.header.stamp = now
        self.camera_info_msg.header.frame_id = self.frame_id

        self.image_pub.publish(img_msg)
        self.camera_info_pub.publish(self.camera_info_msg)
        t_publish = time.perf_counter() - t0

        t_total = time.perf_counter() - t_total_start

        # -- 5. profile report --
        if self.profile:
            self._profile_report(
                t_read * 1e6, t_resize * 1e6, t_bridge * 1e6,
                t_publish * 1e6, t_total * 1e6)


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
