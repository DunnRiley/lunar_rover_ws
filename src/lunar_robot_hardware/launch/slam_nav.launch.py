#!/usr/bin/env python3
"""
FIXED SLAM Navigation - Proper TF and Startup Order
- Waits for camera before starting converters
- Ensures map frame exists before RViz
- Better error handling
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    robot_description = """<?xml version="1.0"?>
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
    
    slam_params = {
        'use_sim_time': False,
        'odom_frame': 'odom',
        'map_frame': 'map',
        'base_frame': 'base_footprint',
        'scan_topic': '/scan',
        'mode': 'mapping',
        'map_update_interval': 2.0,
        'resolution': 0.05,
        'max_laser_range': 8.0,
        'minimum_travel_distance': 0.2,
        'minimum_travel_heading': 0.2,
        'transform_publish_period': 0.02,
        'do_loop_closing': True,
        'loop_search_maximum_distance': 3.0,
        'transform_timeout': 1.0,  # Increased timeout
    }
    
    rviz_config = os.path.join(
        os.path.expanduser('~'),
        'lunar_rover_ws',
        'slam_navigation.rviz'
    )
    
    ld = LaunchDescription()
    
    # ========== STEP 1: TF TREE (Immediate) ==========
    
    ld.add_action(Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time
        }],
        output='screen'
    ))
    
    # CORRECTED: Use quaternion notation for transforms
    ld.add_action(Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_depth_optical_tf',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--qx', '-0.5', '--qy', '0.5', '--qz', '-0.5', '--qw', '0.5',
            '--frame-id', 'camera_link',
            '--child-frame-id', 'camera_depth_optical_frame'
        ],
        output='screen'
    ))
    
    ld.add_action(Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_color_optical_tf',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--qx', '-0.5', '--qy', '0.5', '--qz', '-0.5', '--qw', '0.5',
            '--frame-id', 'camera_link',
            '--child-frame-id', 'camera_color_optical_frame'
        ],
        output='screen'
    ))
    
    # ========== STEP 2: ODOMETRY (Immediate) ==========
    
    ld.add_action(Node(
        package='lunar_robot_hardware',
        executable='simple_odom_publisher',
        name='simple_odom_publisher',
        output='screen'
    ))
    
    # ========== STEP 3: CAMERA (Immediate) ==========
    
    ld.add_action(Node(
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
            'depth_module.profile': '640x480x15',
            'rgb_camera.profile': '640x480x15',
            'pointcloud.enable': True,
            'use_sim_time': use_sim_time,
        }],
        output='screen'
    ))
    
    # ========== STEP 4: DEPTH TO LASERSCAN (Wait 5s for camera) ==========
    
    ld.add_action(TimerAction(
        period=5.0,
        actions=[
            Node(
                package='depthimage_to_laserscan',
                executable='depthimage_to_laserscan_node',
                name='depthimage_to_laserscan',
                parameters=[{
                    'use_sim_time': use_sim_time,
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
    ))
    
    # ========== STEP 5: SLAM TOOLBOX (Wait 8s for scan data) ==========
    
    ld.add_action(TimerAction(
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
    ))
    
    # ========== STEP 6: RVIZ (Wait 10s for everything) ==========
    
    if os.path.exists(rviz_config):
        ld.add_action(TimerAction(
            period=10.0,
            actions=[
                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    arguments=['-d', rviz_config],
                    parameters=[{'use_sim_time': use_sim_time}],
                    output='screen'
                )
            ]
        ))
    else:
        ld.add_action(TimerAction(
            period=10.0,
            actions=[
                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    parameters=[{'use_sim_time': use_sim_time}],
                    output='screen'
                )
            ]
        ))
    
    return ld