#!/usr/bin/env python3
"""
FIXED SLAM Navigation Launch
- Proper startup sequence to avoid TF errors
- SLAM Toolbox for 2D mapping
- Works with D435 camera point cloud
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction
import os


def generate_launch_description():
    
    # Robot URDF with complete frame tree
    robot_urdf = """<?xml version="1.0"?>
<robot name="lunar_rover">
  <link name="base_footprint"/>
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.3 0.2"/>
      </geometry>
    </visual>
  </link>
  <joint name="base_footprint_to_base_link" type="fixed">
    <parent link="base_footprint"/>
    <child link="base_link"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
  </joint>
  <link name="camera_link"/>
  <joint name="base_to_camera" type="fixed">
    <parent link="base_link"/>
    <child link="camera_link"/>
    <origin xyz="0.15 0 0.2" rpy="0 0 0"/>
  </joint>
</robot>
"""
    
    # SLAM parameters
    slam_params = {
        'use_sim_time': False,
        'odom_frame': 'odom',
        'map_frame': 'map',
        'base_frame': 'base_footprint',
        'scan_topic': '/scan',
        'mode': 'mapping',
        'map_update_interval': 1.0,
        'resolution': 0.05,
        'max_laser_range': 8.0,
        'minimum_travel_distance': 0.1,
        'minimum_travel_heading': 0.1,
        'transform_publish_period': 0.02,
        'do_loop_closing': True,
        'loop_search_maximum_distance': 3.0,
        'transform_timeout': 0.5,
    }
    
    rviz_config = os.path.join(
        os.path.expanduser('~'),
        'lunar_rover_ws',
        'slam_navigation.rviz'
    )
    
    return LaunchDescription([
        
        # === STEP 1: TF Tree (immediate) ===
        
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'robot_description': robot_urdf,
                'use_sim_time': False
            }],
            output='screen'
        ),
        
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_depth_optical_tf',
            arguments=[
                '0', '0', '0',
                '-1.5707963267948966', '0', '-1.5707963267948966',
                'camera_link', 'camera_depth_optical_frame'
            ]
        ),
        
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_color_optical_tf',
            arguments=[
                '0', '0', '0',
                '-1.5707963267948966', '0', '-1.5707963267948966',
                'camera_link', 'camera_color_optical_frame'
            ]
        ),
        
        # === STEP 2: Odometry (immediate) ===
        
        Node(
            package='lunar_robot_hardware',
            executable='simple_odom_publisher',
            name='simple_odom_publisher',
            output='screen'
        ),
        
        # === STEP 3: Camera (wait 2s for TF) ===
        
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='realsense2_camera',
                    executable='realsense2_camera_node',
                    name='camera',
                    namespace='camera',
                    parameters=[{
                        'camera_name': 'camera',
                        'enable_color': True,
                        'enable_depth': True,
                        'enable_infra1': False,
                        'enable_infra2': False,
                        'align_depth.enable': True,
                        'enable_sync': True,
                        'depth_module.profile': '640x480x30',
                        'rgb_camera.profile': '640x480x30',
                        'pointcloud.enable': True,
                        'use_sim_time': False,
                    }],
                    output='screen'
                )
            ]
        ),
        
        # === STEP 4: Depth to LaserScan (wait 5s for camera) ===
        
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='depthimage_to_laserscan',
                    executable='depthimage_to_laserscan_node',
                    name='depthimage_to_laserscan',
                    parameters=[{
                        'use_sim_time': False,
                        'scan_height': 10,
                        'range_min': 0.3,
                        'range_max': 8.0,
                        'output_frame': 'camera_depth_optical_frame',
                    }],
                    remappings=[
                        ('depth', '/camera/camera/aligned_depth_to_color/image_raw'),
                        ('depth_camera_info', '/camera/camera/aligned_depth_to_color/camera_info'),
                        ('scan', '/scan'),
                    ],
                    output='screen'
                )
            ]
        ),
        
        # === STEP 5: SLAM Toolbox (wait 8s for scan data) ===
        
        TimerAction(
            period=8.0,
            actions=[
                Node(
                    package='slam_toolbox',
                    executable='async_slam_toolbox_node',
                    name='slam_toolbox',
                    parameters=[slam_params],
                    output='screen'
                )
            ]
        ),
        
        # === STEP 6: RViz (wait 10s for map frame) ===
        
        TimerAction(
            period=10.0,
            actions=[
                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
                    parameters=[{'use_sim_time': False}],
                    output='screen'
                )
            ]
        ),
    ])