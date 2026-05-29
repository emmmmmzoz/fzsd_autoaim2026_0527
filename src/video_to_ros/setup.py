from setuptools import setup

package_name = 'video_to_ros'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'setup.cfg']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fzsd',
    maintainer_email='fzsd@example.com',
    description='Video file to ROS2 image publisher for testing',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'video_to_ros_node = video_to_ros.video_to_ros_node:main',
            'angle_error_plotter = video_to_ros.angle_error_plotter:main',
        ],
    },
)
