#!/usr/bin/env python3
"""
Simple Teleop-Only Launch File
Launches motor controller + teleop without navigation
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch import conditions


def generate_launch_description():
    # Declare arguments
    arduino_port_arg = DeclareLaunchArgument(
        'arduino_port',
        default_value='/dev/ttyACM0',
        description='Arduino serial port'
    )
    
    use_hold_mode_arg = DeclareLaunchArgument(
        'use_hold_mode',
        default_value='true',
        description='Use hold-to-drive teleop (true) or toggle mode (false)'
    )
    
    # Get configuration
    arduino_port = LaunchConfiguration('arduino_port')
    use_hold_mode = LaunchConfiguration('use_hold_mode')
    
    # Arduino Motor Controller Node
    motor_controller = Node(
        package='lunar_robot_hardware',
        executable='arduino_motor_controller',
        name='arduino_motor_controller',
        output='screen',
        parameters=[{
            'arduino_port': arduino_port,
            'baudrate': 115200,
            'cmd_vel_timeout': 0.5,
        }]
    )
    
    # Teleop Node (hold-to-drive version)
    teleop_hold = Node(
        package='lunar_robot_hardware',
        executable='arduino_teleop_hold',
        name='arduino_teleop',
        output='screen',
        prefix='xterm -e',  # Run in separate terminal window
        condition=conditions.IfCondition(use_hold_mode)
    )
    
    # Teleop Node (original toggle version)
    teleop_toggle = Node(
        package='lunar_robot_hardware',
        executable='arduino_teleop',
        name='arduino_teleop',
        output='screen',
        prefix='xterm -e',  # Run in separate terminal window
        condition=conditions.UnlessCondition(use_hold_mode)
    )
    
    return LaunchDescription([
        arduino_port_arg,
        use_hold_mode_arg,
        motor_controller,
        teleop_hold,
        teleop_toggle,
    ])
