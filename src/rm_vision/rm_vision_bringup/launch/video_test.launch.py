import os
import sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node

sys.path.append(os.path.join(get_package_share_directory('rm_vision_bringup'), 'launch'))


def generate_launch_description():
    from common import launch_params, node_params, robot_state_publisher, tracker_node

    # ----- workspace / video path resolution -----
    share_dir = get_package_share_directory('rm_vision_bringup')
    # install/rm_vision_bringup/share/rm_vision_bringup -> workspace root (4 levels up)
    workspace_root = os.path.dirname(os.path.dirname(
        os.path.dirname(os.path.dirname(share_dir))))
    videos_dir = os.path.join(workspace_root, 'videos')

    def _resolve_video_path(video_name_str, videos_dir=videos_dir):
        """Find video file by name prefix (test1 -> test1.mp4, etc.)."""
        for ext in ['.mp4', '.avi', '.mkv', '.mov', '.webm', '.flv', '.wmv']:
            p = os.path.join(videos_dir, video_name_str + ext)
            if os.path.isfile(p):
                return p
        # Also try as-is (might include extension)
        p = os.path.join(videos_dir, video_name_str)
        if os.path.isfile(p):
            return p
        return ''  # let the node report the error clearly

    default_video = 'test1'
    default_video_path = _resolve_video_path(default_video)

    # camera_info from rm_vision_bringup (same calibration as hik_camera)
    default_ci = os.path.join(
        get_package_share_directory('rm_vision_bringup'), 'config', 'camera_info.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'video', default_value=default_video,
            description='Video name prefix: test1, test2, test3'),
        DeclareLaunchArgument(
            'video_path', default_value=default_video_path,
            description='Full path to video file (overrides video)'),
        DeclareLaunchArgument(
            'loop', default_value='true',
            description='Loop video playback'),
        DeclareLaunchArgument(
            'fps', default_value='30.0',
            description='Publish frame rate'),
        DeclareLaunchArgument(
            'scale', default_value='1.0',
            description='Resize factor (0=auto-match CameraInfo width)'),
        DeclareLaunchArgument(
            'camera_info_url', default_value=default_ci,
            description='Path to camera calibration YAML'),
        DeclareLaunchArgument(
            'use_serial', default_value='true',
            description='Launch serial_driver node'),

        # ---- video input node ----
        Node(
            package='video_to_ros',
            executable='video_to_ros_node',
            name='video_to_ros',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'video_path': LaunchConfiguration('video_path'),
                'video_name': LaunchConfiguration('video'),
                'loop': LaunchConfiguration('loop'),
                'fps': LaunchConfiguration('fps'),
                'scale': LaunchConfiguration('scale'),
                'camera_info_url': LaunchConfiguration('camera_info_url'),
            }],
            on_exit=Shutdown(),
        ),

        # ---- robot state publisher (from common) ----
        robot_state_publisher,

        # ---- armor_detector (standalone, not composable, since video is Python) ----
        Node(
            package='armor_detector',
            executable='armor_detector_node',
            name='armor_detector',
            output='screen',
            emulate_tty=True,
            parameters=[node_params],
            arguments=['--ros-args', '--log-level',
                       'armor_detector:=' + launch_params['detector_log_level']],
        ),

        # ---- tracker (from common) ----
        tracker_node,

        # ---- attacker ----
        Node(
            package='vision_attacker',
            executable='vision_attacker_node',
            name='vision_attacker',
            output='screen',
            emulate_tty=True,
            parameters=[node_params],
        ),

        # ---- serial_driver (optional, can be disabled for pure vision test) ----
        Node(
            package='vision_serial_driver',
            executable='vision_serial_driver_node',
            name='serial_driver',
            output='screen',
            emulate_tty=True,
            parameters=[node_params],
            condition=IfCondition(LaunchConfiguration('use_serial')),
        ),
    ])
