from setuptools import setup
import os
from glob import glob

package_name = 'lunar_robot_autonomous'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Autonomous navigation for lunar rover using 3D perception',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_obstacle_avoidance = lunar_robot_autonomous.simple_obstacle_avoidance:main',
            'pointcloud_path_planner = lunar_robot_autonomous.pointcloud_path_planner:main',
            'visual_target_nav = lunar_robot_autonomous.visual_target_nav:main',
            'depth_slam_nav = lunar_robot_autonomous.depth_slam_nav:main',
            'camera_scanner = lunar_robot_autonomous.camera_scanner:main',
            'camera_info_publisher = lunar_robot_autonomous.camera_info_publisher:main',
            'simple_click_to_navigate = lunar_robot_autonomous.simple_click_to_navigate:main',
            'camera_rotation_controller = lunar_robot_autonomous.camera_rotation_controller:main',
            'aruco_detector_node = lunar_robot_autonomous.aruco_detector_node:main',
            'unified_navigator = lunar_robot_autonomous.unified_navigator:main',
            'minimal_navigator = lunar_robot_autonomous.minimal_navigator:main',
            'tf_remapper = lunar_robot_autonomous.tf_remapper:main',
            'topic_remapper = lunar_robot_autonomous.topic_remapper:main',
            'static_tf_bridge = lunar_robot_autonomous.static_tf_bridge:main',  # NEW
            'working_navigator = lunar_robot_autonomous.working_navigator:main',  # NEW
        ],
    },
)