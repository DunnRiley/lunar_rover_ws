#!/usr/bin/env python3
"""
Rover System Launch
Launches all necessary nodes for real hardware operation
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # Motor port configurations
    fr_port = LaunchConfiguration('fr_port', default='/dev/ttyUSB0')
    fl_port = LaunchConfiguration('fl_port', default='/dev/ttyUSB1')
    br_port = LaunchConfiguration('br_port', default='/dev/ttyUSB2')
    bl_port = LaunchConfiguration('bl_port', default='/dev/ttyUSB3')
    
    # Get RViz config path
    pkg_description = get_package_share_directory('lunar_robot_description')
    rviz_config = os.path.join(pkg_description, 'config', 'real_hardware_navigation.rviz')
    
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('fr_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('fl_port', default_value='/dev/ttyUSB1'),
        DeclareLaunchArgument('br_port', default_value='/dev/ttyUSB2'),
        DeclareLaunchArgument('bl_port', default_value='/dev/ttyUSB3'),
        
        # Motor Controller (4-Wheel Skid-Steer)
        Node(
            package='lunar_robot_hardware',
            executable='motor_controller_node',
            name='motor_controller',
            parameters=[{
                'fr_port': fr_port,
                'fl_port': fl_port,
                'br_port': br_port,
                'bl_port': bl_port,
                'use_sim_time': use_sim_time,
            }],
            output='screen'
        ),
        
        # Front D435 Camera (RGB + Depth + Point Cloud)
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='camera',
            namespace='camera',
            parameters=[{
                'camera_name': 'camera',
                'enable_depth': True,
                'enable_color': True,
                'enable_infra1': False,
                'enable_infra2': False,
                'pointcloud.enable': True,
                'align_depth.enable': True,
                'enable_sync': True,
                'use_sim_time': use_sim_time,
            }],
            output='screen'
        ),
        
        # Rear T265 Camera (RGB Only)
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='camera_rear',
            namespace='camera_rear',
            parameters=[{
                'camera_name': 'camera_rear',
                'enable_color': True,
                'enable_depth': False,
                'use_sim_time': use_sim_time,
            }],
            output='screen'
        ),
        
        # Unified Navigator (Point-Click Navigation + Obstacle Avoidance)
        Node(
            package='lunar_robot_autonomous',
            executable='unified_navigator',
            name='unified_navigator',
            parameters=[{
                'use_sim_time': use_sim_time,
            }],
            output='screen'
        ),
        
        # RViz Visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        ),
    ])