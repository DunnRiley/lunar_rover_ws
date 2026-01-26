#!/usr/bin/env python3
"""
STABLE RTAB-Map + Multi-Waypoint Navigation
Fixes:
- Camera USB stability issues
- Proper startup timing
- No odom_info dependency (removes warnings)
- Conservative settings for hardware reliability
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction, ExecuteProcess
from launch.substitutions import LaunchConfiguration
import os


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # Minimal robot URDF
    robot_urdf = """<?xml version="1.0"?>
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
"""
    
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('database_path', default_value='~/.ros/rtabmap.db'),
        
        # ========== STEP 1: TF TREE (Immediate) ==========
        
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
        
        # ========== STEP 2: CAMERA (Wait 2s for TF) ==========
        
        TimerAction(
            period=2.0,
            actions=[
                # Reset USB port before starting camera (helps with disconnects)
                ExecuteProcess(
                    cmd=['bash', '-c', 
                         'echo "Resetting USB..." && '
                         'sudo sh -c "echo 0 > /sys/bus/usb/devices/4-1/authorized" 2>/dev/null || true && '
                         'sleep 0.5 && '
                         'sudo sh -c "echo 1 > /sys/bus/usb/devices/4-1/authorized" 2>/dev/null || true && '
                         'sleep 1'],
                    output='screen',
                    shell=True
                ),
                
                Node(
                    package='realsense2_camera',
                    executable='realsense2_camera_node',
                    name='camera',
                    namespace='camera',
                    parameters=[{
                        'camera_name': 'camera',
                        'camera_namespace': 'camera',
                        'enable_color': True,
                        'enable_depth': True,
                        'enable_infra1': False,
                        'enable_infra2': False,
                        'align_depth.enable': True,
                        'enable_sync': True,
                        
                        # REDUCED RESOLUTION for stability
                        'depth_module.profile': '640x480x15',  # Lower framerate
                        'rgb_camera.profile': '640x480x15',
                        
                        # Point cloud (for visualization, not SLAM)
                        'pointcloud.enable': True,
                        'pointcloud.stream_filter': 2,
                        
                        # USB stability settings
                        'reconnect_timeout': 6.0,
                        'enable_auto_exposure': True,
                        
                        'use_sim_time': use_sim_time,
                    }],
                    respawn=True,  # Auto-restart if crashes
                    respawn_delay=5.0,
                    output='screen'
                )
            ]
        ),
        
        # ========== STEP 3: ODOMETRY (Wait 8s for camera data) ==========
        
        TimerAction(
            period=8.0,
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
                        'approx_sync_max_interval': 0.5,  # Increased tolerance
                        'wait_for_transform': 1.0,
                        'queue_size': 50,  # Larger queue
                        'use_sim_time': use_sim_time,
                        
                        # CRITICAL: Match RTAB-Map feature detector
                        'Odom/Strategy': '0',
                        'Vis/FeatureType': '0',  # GFTT (must match Kp/DetectorStrategy)
                        'Vis/MinInliers': '5',  # Very relaxed
                        'Vis/MaxFeatures': '800',
                        'GFTT/MinDistance': '5',
                        'GFTT/QualityLevel': '0.0001',  # Very sensitive
                        'Odom/ResetCountdown': '1',
                        'Odom/GuessMotion': 'false',
                        'Vis/MaxDepth': '4.0',
                    }],
                    remappings=[
                        ('rgb/image', '/camera/camera/color/image_raw'),
                        ('rgb/camera_info', '/camera/camera/color/camera_info'),
                        ('depth/image', '/camera/camera/aligned_depth_to_color/image_raw'),
                    ],
                    respawn=True,
                    respawn_delay=5.0,
                    output='screen'
                )
            ]
        ),
        
        # ========== STEP 4: RTAB-MAP (Wait 12s for odometry) ==========
        
        TimerAction(
            period=12.0,
            actions=[
                Node(
                    package='rtabmap_slam',
                    executable='rtabmap',
                    name='rtabmap',
                    parameters=[{
                        'frame_id': 'base_link',
                        'odom_frame_id': 'odom',
                        'map_frame_id': 'map',
                        'subscribe_depth': True,
                        'subscribe_rgb': True,
                        'subscribe_odom_info': False,  # CRITICAL: Don't wait for odom_info
                        'approx_sync': True,
                        'wait_for_transform': 1.0,
                        'queue_size': 50,
                        'topic_queue_size': 50,  # Increased from 20
                        'sync_queue_size': 50,  # Match topic_queue_size
                        
                        # Database
                        'database_path': LaunchConfiguration('database_path'),
                        'Mem/IncrementalMemory': 'true',
                        'Mem/InitWMWithAllNodes': 'false',
                        
                        # Slow mapping for 360° scan
                        'Rtabmap/DetectionRate': '1.0',
                        'RGBD/LinearUpdate': '0.15',  # Update every 15cm
                        'RGBD/AngularUpdate': '0.17',  # ~10 degrees
                        
                        # CRITICAL: Match odometry features
                        'Vis/FeatureType': '0',  # GFTT (not 6!)
                        'Kp/DetectorStrategy': '0',  # GFTT
                        'Vis/MaxFeatures': '800',
                        'Vis/MinInliers': '5',
                        'Kp/MaxFeatures': '800',
                        'GFTT/MinDistance': '5',
                        'GFTT/QualityLevel': '0.0001',
                        
                        # Grid map for obstacle avoidance
                        'Grid/FromDepth': 'true',
                        'Grid/MaxObstacleHeight': '0.5',
                        'Grid/MinObstacleHeight': '0.05',
                        'Grid/RangeMax': '5.0',
                        'Grid/CellSize': '0.05',
                        'Grid/DepthDecimation': '4',
                        
                        # 3DoF registration
                        'Reg/Strategy': '1',
                        'Reg/Force3DoF': 'true',
                        'Icp/VoxelSize': '0.05',
                        'Icp/MaxTranslation': '0.3',
                        
                        # Loop closure
                        'RGBD/OptimizeFromGraphEnd': 'false',
                        'RGBD/ProximityBySpace': 'true',
                        'RGBD/ProximityMaxGraphDepth': '50',
                        
                        'use_sim_time': use_sim_time,
                    }],
                    remappings=[
                        ('rgb/image', '/camera/camera/color/image_raw'),
                        ('rgb/camera_info', '/camera/camera/color/camera_info'),
                        ('depth/image', '/camera/camera/aligned_depth_to_color/image_raw'),
                    ],
                    arguments=['--delete_db_on_start'],
                    respawn=False,  # Don't auto-restart RTAB-Map
                    output='screen'
                )
            ]
        ),
        
        # ========== STEP 5: NAVIGATION (Wait 15s for map) ==========
        
        TimerAction(
            period=15.0,
            actions=[
                Node(
                    package='lunar_robot_autonomous',
                    executable='multi_waypoint_navigator',
                    name='multi_waypoint_navigator',
                    parameters=[{
                        'goal_tolerance': 0.3,
                        'forward_speed': 0.2,
                        'turn_speed': 0.3,
                        'lookahead_distance': 0.8,
                        'use_sim_time': use_sim_time,
                    }],
                    output='screen'
                )
            ]
        ),
        
        # ========== STEP 6: RViz (Wait 5s) ==========
        
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    arguments=['-d', os.path.join(
                        os.path.expanduser('~'),
                        'lunar_rover_ws',
                        'rtabmap_navigation.rviz'
                    )] if os.path.exists(os.path.join(
                        os.path.expanduser('~'),
                        'lunar_rover_ws',
                        'rtabmap_navigation.rviz'
                    )) else [],
                    parameters=[{'use_sim_time': use_sim_time}],
                    output='screen'
                )
            ]
        ),
    ])