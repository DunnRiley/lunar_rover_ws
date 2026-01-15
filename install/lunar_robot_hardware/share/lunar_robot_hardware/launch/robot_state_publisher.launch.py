#!/usr/bin/env python3
"""
Robot State Publisher for Real Hardware
Creates the base_link frame and publishes robot description
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get URDF file path
    pkg_description = get_package_share_directory('lunar_robot_description')
    urdf_file = os.path.join(pkg_description, 'urdf', 'lunar_robot.urdf')
    
    # If no URDF exists, we'll use a minimal robot description inline
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
        # Robot State Publisher - creates base_link and publishes URDF
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
    ])