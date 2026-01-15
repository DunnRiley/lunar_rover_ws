#!/usr/bin/env python3

"""
Complete lunar robot simulation launch file with new motor controller
"""

from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    
    pkg_description = FindPackageShare('lunar_robot_description')
    pkg_gazebo = FindPackageShare('lunar_robot_gazebo')
    pkg_control = FindPackageShare('lunar_robot_control')
    
    # Robot description
    robot_description = Command(['xacro ', PathJoinSubstitution([
        pkg_description, 'urdf', 'lunar_robot.urdf.xacro'
    ])])
    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True
        }],
        output='screen'
    )
    
    # Start Gazebo with lunar environment
    start_gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', PathJoinSubstitution([
            pkg_gazebo, 'worlds', 'lunar_environment.world'
        ])],
        output='screen'
    )
    
    # Spawn robot in Gazebo
    spawn_robot = TimerAction(
        period=3.0,
        actions=[ExecuteProcess(
            cmd=['ros2', 'run', 'ros_gz_sim', 'create',
                 '-name', 'lunar_robot',
                 '-topic', 'robot_description',
                 '-z', '0.5'],
            output='screen'
        )]
    )
    
    # Clock bridge
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )
    
    # Controller manager (loads individual wheel controllers)
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            PathJoinSubstitution([pkg_description, 'config', 'robot_control.yaml'])
        ],
        output='screen'
    )
    
    # Spawn controllers (delayed to allow controller manager to start)
    spawn_controllers = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
                     'joint_state_broadcaster'],
                output='screen'
            ),
            ExecuteProcess(
                cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
                     'front_left_wheel_controller'],
                output='screen'
            ),
            ExecuteProcess(
                cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
                     'front_right_wheel_controller'],
                output='screen'
            ),
            ExecuteProcess(
                cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
                     'rear_left_wheel_controller'],
                output='screen'
            ),
            ExecuteProcess(
                cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
                     'rear_right_wheel_controller'],
                output='screen'
            )
        ]
    )
    
    # Motor controller node (our new C++ controller)
    motor_controller = TimerAction(
        period=6.0,  # Start after controllers are loaded
        actions=[Node(
            package='lunar_robot_control',
            executable='motor_controller_node',
            name='motor_controller_node',
            parameters=[
                PathJoinSubstitution([pkg_control, 'config', 'motor_controller.yaml'])
            ],
            output='screen'
        )]
    )
    
    return LaunchDescription([
        start_gazebo,
        robot_state_publisher,
        clock_bridge,
        spawn_robot,
        controller_manager,
        spawn_controllers,
        motor_controller
    ])