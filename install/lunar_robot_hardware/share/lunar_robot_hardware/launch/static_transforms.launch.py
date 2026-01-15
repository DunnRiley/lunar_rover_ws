#!/usr/bin/env python3
"""
Static Transform Publishers for Real Hardware
Publishes transforms between robot base and camera frames
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    Transform tree structure:
    base_link (robot center)
      └─ camera_link (front camera mount point)
           └─ camera_depth_optical_frame (D435 depth sensor frame)
           └─ camera_color_optical_frame (D435 color sensor frame)
      └─ camera_rear_link (rear camera mount point)
           └─ camera_rear_color_optical_frame (T265 color sensor frame)
    """
    
    return LaunchDescription([
        # Base link to front camera mounting point
        # Adjust x, y, z, roll, pitch, yaw based on your physical setup
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_camera_link',
            arguments=[
                '0.15', '0', '0.2',  # x, y, z (meters) - camera is 15cm forward, 20cm up from base
                '0', '0', '0',        # roll, pitch, yaw (radians) - facing forward
                'base_link',
                'camera_link'
            ],
            output='screen'
        ),
        
        # Camera link to D435 depth optical frame
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_link_to_depth_optical',
            arguments=[
                '0', '0', '0',
                '-1.5707963267948966', '0', '-1.5707963267948966',  # -90deg roll, -90deg yaw
                'camera_link',
                'camera_depth_optical_frame'
            ],
            output='screen'
        ),
        
        # Camera link to D435 color optical frame
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_link_to_color_optical',
            arguments=[
                '0', '0', '0',
                '-1.5707963267948966', '0', '-1.5707963267948966',  # -90deg roll, -90deg yaw
                'camera_link',
                'camera_color_optical_frame'
            ],
            output='screen'
        ),
        
        # Base link to rear camera mounting point
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_camera_rear_link',
            arguments=[
                '-0.15', '0', '0.2',  # x, y, z - camera is 15cm backward, 20cm up
                '0', '0', '3.14159265359',  # yaw 180deg (facing backward)
                'base_link',
                'camera_rear_link'
            ],
            output='screen'
        ),
        
        # Rear camera link to T265 color optical frame
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