import os
import sys
from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess
sys.path.append(os.path.join(get_package_share_directory('rm_vision_bringup'), 'launch'))


def generate_launch_description():

    from common import node_params, launch_params, robot_state_publisher, tracker_node
    from launch_ros.descriptions import ComposableNode
    from launch_ros.actions import ComposableNodeContainer, Node
    from launch.actions import TimerAction, Shutdown
    from launch import LaunchDescription

    def get_camera_node(package, plugin):
        return ComposableNode(
            package=package,
            plugin=plugin,
            name='camera_node',
            parameters=[node_params],
            extra_arguments=[{'use_intra_process_comms': True}]
        )

    def get_camera_detector_container(camera_node):
        return ComposableNodeContainer(
            name='camera_detector_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                camera_node,
                ComposableNode(
                    package='armor_detector',
                    plugin='rm_auto_aim::ArmorDetectorNode',
                    name='armor_detector',
                    parameters=[node_params],
                    extra_arguments=[{'use_intra_process_comms': True}]
                )
            ],
            output='both',
            emulate_tty=True,
            ros_arguments=['--ros-args', '--log-level',
                           'armor_detector:='+launch_params['detector_log_level']],
            on_exit=Shutdown(),
        )

    hik_camera_node = get_camera_node('hik_camera', 'hik_camera::HikCameraNode')

    camera = launch_params.get('camera', 'hik')

    delay_tracker_node = TimerAction(
        period=2.0,
        actions=[tracker_node],
    )

    serial_driver_node = Node(
        package='vision_serial_driver',
        executable='vision_serial_driver_node',
        parameters=[node_params],
    )

    attacker_node = Node(
        package='vision_attacker',
        executable='vision_attacker_node',
        parameters=[node_params],
    )

    if camera == 'video':
        share_dir = get_package_share_directory('rm_vision_bringup')
        workspace_root = os.path.dirname(os.path.dirname(
            os.path.dirname(os.path.dirname(share_dir))))
        camera_info_path = os.path.join(share_dir, 'config', 'camera_info.yaml')

        return LaunchDescription([
            robot_state_publisher,
            Node(
                package='video_to_ros',
                executable='video_to_ros_node',
                name='video_to_ros',
                output='screen',
                emulate_tty=True,
                parameters=[{
                    'video_name': launch_params.get('video', 'test1'),
                    'video_path': launch_params.get('video_path', ''),
                    'loop': launch_params.get('loop', True),
                    'fps': launch_params.get('fps', 30.0),
                    'scale': launch_params.get('scale', 1.0),
                    'camera_info_url': camera_info_path,
                }],
                on_exit=Shutdown(),
            ),
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
            delay_tracker_node,
            serial_driver_node,
            attacker_node,
        ])
    else:
        cam_detector = get_camera_detector_container(hik_camera_node)
        return LaunchDescription([
            robot_state_publisher,
            cam_detector,
            delay_tracker_node,
            serial_driver_node,
            attacker_node,
        ])
