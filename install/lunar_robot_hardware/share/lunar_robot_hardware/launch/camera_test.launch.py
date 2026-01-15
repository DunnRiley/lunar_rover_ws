#!/usr/bin/env python3
"""
Camera Test Launch File
Launches cameras with proper transforms for RViz visualization
"""

from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directory
    pkg_description = get_package_share_directory('lunar_robot_description')
    rviz_config = os.path.join(pkg_description, 'config', 'real_hardware_navigation.rviz')
    
    # Minimal robot URDF with camera mounts
    minimal_urdf = """<?xml version="1.0"?>
<robot name="lunar_rover">
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.3 0.2"/>
      </geometry>
    </visual>
  </link>
  
  <link name="camera_link"/>
  <joint name="base_to_camera" type="fixed">
    <parent link="base_link"/>
    <child link="camera_link"/>
    <origin xyz="0.15 0 0.2" rpy="0 0 0"/>
  </joint>
  
  <link name="camera_rear_link"/>
  <joint name="base_to_camera_rear" type="fixed">
    <parent link="base_link"/>
    <child link="camera_rear_link"/>
    <origin xyz="-0.15 0 0.2" rpy="0 0 3.14159265359"/>
  </joint>
</robot>
"""
    
    return LaunchDescription([
        # Robot State Publisher (creates base_link and camera transforms)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'robot_description': minimal_urdf,
                'use_sim_time': False
            }],
            output='screen'
        ),
        
        # Static transform: camera_link to depth optical frame
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_link_to_depth_optical',
            arguments=[
                '0', '0', '0',
                '-1.5707963267948966', '0', '-1.5707963267948966',
                'camera_link',
                'camera_depth_optical_frame'
            ],
            output='screen'
        ),
        
        # Static transform: camera_link to color optical frame  
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_link_to_color_optical',
            arguments=[
                '0', '0', '0',
                '-1.5707963267948966', '0', '-1.5707963267948966',
                'camera_link',
                'camera_color_optical_frame'
            ],
            output='screen'
        ),
        
        # Static transform: rear camera_link to color optical frame
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_rear_link_to_color_optical',
            arguments=[
                '0', '0', '0',
                '-1.5707963267948966', '0', '-1.5707963267948966',
                'camera_rear_link',
                'camera_rear_color_optical_frame'
            ],
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
                'depth_module.depth_profile': '640x480x30',
                'rgb_camera.color_profile': '640x480x30',
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
            }],
            output='screen'
        ),
        
        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': False}],
            output='screen'
        ),
    ])