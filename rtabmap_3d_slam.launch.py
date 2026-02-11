#!/usr/bin/env python3
"""
RTAB-Map 3D SLAM Launcher - Standalone Script
Run this directly: python3 launch_rtabmap.py
"""

import os
import sys
import subprocess
import time
import signal

def print_header():
    print("="*70)
    print("  RTAB-Map 3D SLAM Launcher")
    print("="*70)
    print()

def check_dependencies():
    """Check if required packages are installed"""
    print("Checking dependencies...")
    
    # Check RTAB-Map
    result = subprocess.run(
        ['ros2', 'pkg', 'list'],
        capture_output=True,
        text=True
    )
    
    if 'rtabmap_ros' not in result.stdout:
        print("❌ ERROR: rtabmap_ros not installed!")
        print()
        print("Install with:")
        print("  sudo apt update")
        print("  sudo apt install ros-humble-rtabmap-ros")
        print("  OR")
        print("  sudo apt install ros-jazzy-rtabmap-ros")
        return False
    
    print("✓ RTAB-Map installed")
    return True

def kill_existing_nodes():
    """Kill any existing ROS nodes"""
    print("Cleaning up existing nodes...")
    subprocess.run(['pkill', '-9', '-f', 'rtabmap'], stderr=subprocess.DEVNULL)
    subprocess.run(['pkill', '-9', '-f', 'rgbd_odometry'], stderr=subprocess.DEVNULL)
    subprocess.run(['pkill', '-9', '-f', 'realsense'], stderr=subprocess.DEVNULL)
    subprocess.run(['pkill', '-9', '-f', 'robot_state_publisher'], stderr=subprocess.DEVNULL)
    subprocess.run(['pkill', '-9', '-f', 'rviz'], stderr=subprocess.DEVNULL)
    time.sleep(2)
    print("✓ Cleaned up")

def main():
    print_header()
    
    if not check_dependencies():
        sys.exit(1)
    
    kill_existing_nodes()
    
    # Find ROS installation
    if os.path.exists('/opt/ros/jazzy/setup.bash'):
        ros_distro = 'jazzy'
    elif os.path.exists('/opt/ros/humble/setup.bash'):
        ros_distro = 'humble'
    else:
        print("❌ ERROR: No ROS installation found!")
        sys.exit(1)
    
    print(f"Using ROS2 {ros_distro}")
    print()
    
    # Create temporary launch file
    launch_content = """
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction
import os

def generate_launch_description():
    
    robot_urdf = '''<?xml version="1.0"?>
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
</robot>
'''
    
    return LaunchDescription([
        
        # Robot State Publisher
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
        
        # Camera optical frame transforms
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_depth_optical_tf',
            arguments=['0', '0', '0', '-1.5707963267948966', '0', '-1.5707963267948966',
                      'camera_link', 'camera_depth_optical_frame']
        ),
        
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_color_optical_tf',
            arguments=['0', '0', '0', '-1.5707963267948966', '0', '-1.5707963267948966',
                      'camera_link', 'camera_color_optical_frame']
        ),
        
        # Camera
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
        
        # RGB-D Odometry
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='rtabmap_odom',
                    executable='rgbd_odometry',
                    name='rgbd_odometry',
                    parameters=[{
                        'frame_id': 'base_link',
                        'odom_frame_id': 'odom',
                        'publish_tf': True,
                        'subscribe_depth': True,
                        'subscribe_rgb': True,
                        'approx_sync': True,
                        'Odom/Strategy': '0',
                        'Vis/FeatureType': '0',
                        'Vis/MaxFeatures': '400',
                        'use_sim_time': False,
                    }],
                    remappings=[
                        ('rgb/image', '/camera/camera/color/image_raw'),
                        ('rgb/camera_info', '/camera/camera/color/camera_info'),
                        ('depth/image', '/camera/camera/aligned_depth_to_color/image_raw'),
                    ],
                    output='screen'
                )
            ]
        ),
        
        # RTAB-Map
        TimerAction(
            period=8.0,
            actions=[
                Node(
                    package='rtabmap_slam',
                    executable='rtabmap',
                    name='rtabmap',
                    parameters=[{
                        'frame_id': 'base_link',
                        'subscribe_depth': True,
                        'subscribe_rgb': True,
                        'approx_sync': True,
                        'database_path': '~/.ros/rtabmap.db',
                        'Rtabmap/DetectionRate': '1.0',
                        'RGBD/LinearUpdate': '0.1',
                        'RGBD/AngularUpdate': '0.1',
                        'Vis/FeatureType': '0',
                        'Grid/FromDepth': 'true',
                        'use_sim_time': False,
                    }],
                    remappings=[
                        ('rgb/image', '/camera/camera/color/image_raw'),
                        ('rgb/camera_info', '/camera/camera/color/camera_info'),
                        ('depth/image', '/camera/camera/aligned_depth_to_color/image_raw'),
                    ],
                    arguments=['--delete_db_on_start'],
                    output='screen'
                )
            ]
        ),
        
        # RViz
        TimerAction(
            period=10.0,
            actions=[
                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    output='screen'
                )
            ]
        ),
    ])
"""
    
    # Write temporary launch file
    temp_launch = '/tmp/rtabmap_temp.launch.py'
    with open(temp_launch, 'w') as f:
        f.write(launch_content)
    
    print("Starting RTAB-Map 3D SLAM...")
    print()
    print("Timeline:")
    print("  0-2s:  Starting TF tree")
    print("  2-5s:  Starting camera (you should see RGB image)")
    print("  5-8s:  Starting visual odometry")
    print("  8-10s: Starting RTAB-Map SLAM")
    print("  10s+:  RViz opens")
    print()
    print("After 10 seconds:")
    print("  1. Open another terminal")
    print("  2. Run: cd ~/lunar_rover_ws && python3 teleop_keyboard.py")
    print("  3. Rotate camera 360° (T/G keys)")
    print("  4. Drive around slowly (W/A/S/D keys)")
    print()
    print("Press Ctrl+C to stop everything")
    print("="*70)
    print()
    
    # Source ROS and launch
    cmd = f"bash -c 'source /opt/ros/{ros_distro}/setup.bash && ros2 launch {temp_launch}'"
    
    try:
        subprocess.run(cmd, shell=True)
    except KeyboardInterrupt:
        print("\n\nStopping RTAB-Map...")
        kill_existing_nodes()
        print("✓ Stopped")

if __name__ == '__main__':
    main()