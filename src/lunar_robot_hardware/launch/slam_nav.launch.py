#!/usr/bin/env python3
"""
- Continuous SLAM (map updates while navigating)
- No save/reload needed
- One RViz, one launch command
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('lunar_robot_hardware')
    
    # Paths
    urdf_file = os.path.join(pkg_share, 'urdf', 'lunar_rover.urdf')
    rviz_config = os.path.join(pkg_share, 'rviz', 'slam_navigation.rviz')
    
    slam_params = {
        'use_sim_time': False,
        'odom_frame': 'odom',
        'map_frame': 'map',
        'base_frame': 'base_footprint',
        'scan_topic': '/scan',
        'mode': 'mapping',  # Continuous mapping mode
        'debug_logging': False,
        'throttle_scans': 1,
        'transform_publish_period': 0.02,
        'map_update_interval': 2.0,
        'resolution': 0.05,
        'max_laser_range': 8.0,
        'minimum_travel_distance': 0.2,
        'minimum_travel_heading': 0.2,
        'scan_buffer_size': 10,
        'scan_buffer_maximum_scan_distance': 10.0,
        'link_match_minimum_response_fine': 0.1,
        'link_scan_maximum_distance': 1.5,
        'loop_search_maximum_distance': 3.0,
        'do_loop_closing': True,
        'loop_match_minimum_chain_size': 10,
        'loop_match_maximum_variance_coarse': 3.0,
        'loop_match_minimum_response_coarse': 0.35,
        'correlation_search_space_dimension': 0.5,
        'correlation_search_space_resolution': 0.01,
        'correlation_search_space_smear_deviation': 0.1,
        'minimum_time_interval': 0.5,
        'transform_timeout': 0.2,
        'tf_buffer_duration': 30.0,
        'stack_size_to_use': 40000000,
        'solver_plugin': 'solver_plugins::CeresSolver',
        'ceres_linear_solver': 'SPARSE_NORMAL_CHOLESKY',
        'ceres_preconditioner': 'SCHUR_JACOBI',
        'ceres_trust_strategy': 'LEVENBERG_MARQUARDT',
        'ceres_dogleg_type': 'TRADITIONAL_DOGLEG',
        'ceres_loss_function': 'None',
    }
    
    # Nav2 params
    nav2_params = {
        'use_sim_time': False,
        'robot_radius': 0.25,
        'transform_tolerance': 0.5,
        'controller_frequency': 10.0,
        'planner_frequency': 1.0,
        'costmap_update_frequency': 5.0,
        'global_costmap_resolution': 0.05,
        'local_costmap_resolution': 0.05,
    }
    
    ld = LaunchDescription()
    
    # ========== ROBOT DESCRIPTION ==========
    with open(urdf_file, 'r') as f:
        robot_desc = f.read()
    
    ld.add_action(Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    ))
    
    # ========== TF STATIC TRANSFORMS ==========
    ld.add_action(Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_depth_optical_tf',
        arguments=['0', '0', '0', '-0.5', '0.5', '-0.5', '0.5', 
                   'camera_link', 'camera_depth_optical_frame']
    ))
    
    ld.add_action(Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_color_optical_tf',
        arguments=['0', '0', '0', '-0.5', '0.5', '-0.5', '0.5',
                   'camera_link', 'camera_color_optical_frame']
    ))
    
    # ========== REALSENSE CAMERA ==========
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
            'use_sim_time': False,
        }],
        output='screen'
    ))
    
    # ========== DEPTH TO LASERSCAN ==========
    ld.add_action(Node(
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        name='depthimage_to_laserscan',
        remappings=[
            ('depth', '/camera/camera/aligned_depth_to_color/image_raw'),
            ('depth_camera_info', '/camera/camera/aligned_depth_to_color/camera_info'),
        ],
        parameters=[{
            'scan_height': 20,
            'scan_time': 0.033,
            'range_min': 0.3,
            'range_max': 8.0,
            'output_frame': 'camera_depth_optical_frame',
        }],
        output='screen'
    ))
    
    # ========== ODOMETRY ==========
    ld.add_action(Node(
        package='lunar_robot_hardware',
        executable='odom_publisher',
        name='odom_publisher',
        output='screen'
    ))
    
    # ========== SLAM TOOLBOX (CONTINUOUS MAPPING) ==========
    ld.add_action(Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[slam_params],
        output='screen'
    ))
    
    # Lifecycle manager for SLAM Toolbox
    ld.add_action(Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_slam',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': ['slam_toolbox']
        }]
    ))
    
    # ========== NAV2 STACK ==========
    # Using Nav2 bringup with minimal params
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    ld.add_action(IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare('nav2_bringup'),
            'launch',
            'navigation_launch.py'
        ]),
        launch_arguments={
            'use_sim_time': 'false',
            'params_file': PathJoinSubstitution([
                FindPackageShare('lunar_robot_hardware'),
                'config',
                'nav2_params.yaml'
            ]),
            'autostart': 'true',
        }.items()
    ))
    
    # ========== WAYPOINT NAVIGATOR ==========
    ld.add_action(Node(
        package='lunar_robot_autonomous',
        executable='waypoint_navigator',
        name='waypoint_navigator',
        output='screen',
        parameters=[{'use_sim_time': False}]
    ))
    
    # ========== RVIZ ==========
    ld.add_action(Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    ))
    
    return ld