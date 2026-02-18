#!/usr/bin/env python3
"""
Complete Navigation Launch - Arduino Hardware
Includes point-click navigation, cameras, and Arduino motor control
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
import os


def generate_launch_description():
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    arduino_port = LaunchConfiguration('arduino_port', default='/dev/ttyACM0')
    
    # Robot URDF
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
  
  <link name="camera_rear_link"/>
  
  <joint name="base_to_camera_rear" type="fixed">
    <parent link="base_link"/>
    <child link="camera_rear_link"/>
    <origin xyz="-0.15 0 0.2" rpy="0 0 3.14159"/>
  </joint>
</robot>
"""
    
    return LaunchDescription([
        
        # Launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        
        DeclareLaunchArgument(
            'arduino_port',
            default_value='/dev/ttyACM0',
            description='Arduino serial port'
        ),
        
        # ========== STEP 1: TF Tree (Immediate) ==========
        
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'robot_description': robot_urdf,
                'use_sim_time': use_sim_time
            }],
            output='screen'
        ),
        
        # Camera optical frame transforms
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
        
        # Rear camera transforms
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_rear_left_optical_tf',
            arguments=[
                '0', '0', '0',
                '-1.5707963267948966', '0', '-1.5707963267948966',
                'camera_rear_link', 'camera_rear_left_optical_frame'
            ]
        ),
        
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_rear_right_optical_tf',
            arguments=[
                '0.06', '0', '0',
                '-1.5707963267948966', '0', '-1.5707963267948966',
                'camera_rear_link', 'camera_rear_right_optical_frame'
            ]
        ),
        
        # ========== STEP 2: Arduino Motor Controller (Immediate) ==========
        
        Node(
            package='lunar_robot_hardware',
            executable='arduino_motor_controller',
            name='arduino_motor_controller',
            parameters=[{
                'arduino_port': arduino_port,
                'baudrate': 115200,
                'cmd_vel_timeout': 0.5,
                'deadzone_linear': 0.05,
                'deadzone_angular': 0.05,
                'max_motor_speed': 127,
                'use_sim_time': use_sim_time,
            }],
            output='screen'
        ),
        
        # ========== STEP 3: Simple Odometry (Immediate) ==========
        # Note: Replace with encoder-based odometry when available
        
        Node(
            package='lunar_robot_hardware',
            executable='simple_odom_publisher',
            name='simple_odom_publisher',
            parameters=[{
                'use_sim_time': use_sim_time
            }],
            output='screen'
        ),
        
        # ========== STEP 4: Front Camera (Wait 2s for TF) ==========
        
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
                        'use_sim_time': use_sim_time,
                    }],
                    output='screen'
                )
            ]
        ),
        
        # ========== STEP 5: Point-Click Navigator (Wait 5s for camera) ==========
        
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='lunar_robot_autonomous',
                    executable='unified_navigator',
                    name='unified_navigator',
                    parameters=[{
                        'grid_resolution': 0.15,
                        'planning_range': 8.0,
                        'obstacle_threshold': 0.2,
                        'robot_radius': 0.35,
                        'forward_speed': 0.25,  # Conservative for real hardware
                        'turn_speed': 0.4,
                        'goal_tolerance': 0.3,
                        'lookahead_distance': 0.8,
                        'use_sim_time': use_sim_time,
                    }],
                    output='screen'
                )
            ]
        ),
        
        # ========== STEP 6: RViz (Wait 3s) ==========
        
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    arguments=[
                        '-d', os.path.join(
                            os.path.expanduser('~'),
                            'lunar_rover_ws',
                            'hardware_navigation.rviz'
                        )
                    ] if os.path.exists(os.path.join(
                        os.path.expanduser('~'),
                        'lunar_rover_ws',
                        'hardware_navigation.rviz'
                    )) else [],
                    parameters=[{'use_sim_time': use_sim_time}],
                    output='screen'
                )
            ]
        ),
    ])