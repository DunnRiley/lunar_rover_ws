#!/usr/bin/env python3
"""
Launch file for unified navigation system
Run this AFTER the simulation is started
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.actions import TimerAction

def generate_launch_description():
    
    pkg_description = FindPackageShare('lunar_robot_description')
    
    # Unified Navigator
    navigator = Node(
        package='lunar_robot_autonomous',
        executable='unified_navigator',
        name='unified_navigator',
        parameters=[{
            'use_sim_time': True,
            'grid_resolution': 0.15,
            'planning_range': 8.0,
            'obstacle_threshold': 1.2,
            'robot_radius': 0.4,
            'forward_speed': 0.25,  # HALF SPEED
            'turn_speed': 0.4,
            'goal_tolerance': 0.5,
            'lookahead_distance': 1.0,
        }],
        output='screen'
    )
    
    # RViz with navigation visualization
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', PathJoinSubstitution([
            pkg_description, 'config', 'navigation_view.rviz'
        ])],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )
    
    return LaunchDescription([
        navigator,
        rviz,
    ])