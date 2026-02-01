#!/usr/bin/env python3
"""
SLAM Navigation Launch - Complete System for ROS2 Jazzy
- SLAM Toolbox for persistent mapping
- Nav2 for navigation (launching nodes directly)
- Depthimage to Laserscan for 2D SLAM from D435
- Multi-waypoint support
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    autostart = LaunchConfiguration('autostart', default='true')
    
    # Get package directory
    pkg_hardware = FindPackageShare('lunar_robot_hardware')
    
    # Config file paths
    slam_params_file = PathJoinSubstitution([
        pkg_hardware, 'config', 'slam_toolbox_params.yaml'
    ])
    
    nav2_params_file = PathJoinSubstitution([
        pkg_hardware, 'config', 'nav2_params.yaml'
    ])
    
    # RViz config path
    rviz_config_file = os.path.join(
        os.path.expanduser('~'),
        'lunar_rover_ws',
        'slam_navigation.rviz'
    )
    
    # Minimal robot URDF with base_footprint for Nav2
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
    
    # Lifecycle node manager names
    lifecycle_nodes = [
        'controller_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
        'velocity_smoother'
    ]
    
    # Build launch description
    ld = LaunchDescription()
    
    # Launch arguments
    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='false'))
    ld.add_action(DeclareLaunchArgument('autostart', default_value='true'))
    
    # ========== TF TREE ==========
    
    ld.add_action(Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': robot_urdf,
            'use_sim_time': use_sim_time
        }],
        output='screen'
    ))
    
    # Camera optical frame transforms
    ld.add_action(Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_depth_optical_tf',
        arguments=[
            '0', '0', '0',
            '-1.5707963267948966', '0', '-1.5707963267948966',
            'camera_link', 'camera_depth_optical_frame'
        ]
    ))
    
    ld.add_action(Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_color_optical_tf',
        arguments=[
            '0', '0', '0',
            '-1.5707963267948966', '0', '-1.5707963267948966',
            'camera_link', 'camera_color_optical_frame'
        ]
    ))
    
    # ========== CAMERAS ==========
    
    # Front D435 Camera
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
            'reconnect_timeout': 6.0, 
            'wait_for_device_timeout': 10.0,  
        }],
        respawn=True,  
        respawn_delay=2.0,  
        output='screen'
    ))

    # ========== DEPTH TO LASERSCAN ==========
    
    ld.add_action(Node(
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
    ))
    
    # ========== SLAM TOOLBOX ==========
    
    ld.add_action(Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[
            slam_params_file,
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    ))
    
    # ========== NAV2 STACK ==========
    
    # Controller Server
    ld.add_action(Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_params_file],
        remappings=[
            ('cmd_vel', '/cmd_vel'),
            ('odom', '/odom')
        ]
    ))
    
    # Planner Server
    ld.add_action(Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_params_file]
    ))
    
    # Behavior Server
    ld.add_action(Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[nav2_params_file]
    ))
    
    # BT Navigator
    ld.add_action(Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_params_file]
    ))
    
    # Waypoint Follower
    ld.add_action(Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[nav2_params_file]
    ))
    
    # Velocity Smoother
    ld.add_action(Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[nav2_params_file],
        remappings=[
            ('cmd_vel', 'cmd_vel_nav'),
            ('cmd_vel_smoothed', 'cmd_vel')
        ]
    ))
    
    # Lifecycle Manager
    ld.add_action(Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'node_names': lifecycle_nodes
        }]
    ))
    
    # ========== WAYPOINT NAVIGATOR ==========
    
    ld.add_action(Node(
        package='lunar_robot_autonomous',
        executable='waypoint_navigator',
        name='waypoint_navigator',
        parameters=[{
            'use_sim_time': use_sim_time,
        }],
        output='screen'
    ))

    # ========== ODOMETRY ==========

    ld.add_action(Node(
        package='lunar_robot_hardware',
        executable='simple_odom_publisher',
        name='simple_odom_publisher',
        output='screen'
    ))
    
    # ========== RVIZ ==========
    # Only launch if config file exists
    
    if os.path.exists(rviz_config_file):
        ld.add_action(Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        ))
    else:
        # Launch RViz without config
        ld.add_action(Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        ))
    
    return ld