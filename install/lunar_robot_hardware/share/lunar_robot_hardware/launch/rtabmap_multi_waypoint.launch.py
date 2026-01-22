#!/usr/bin/env python3
"""
FIXED: RTAB-Map with Multi-Waypoint Navigation
- Corrected TF transforms for point cloud visibility
- Fixed odometry topic remapping
- Tuned for stationary initial mapping
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # Minimal URDF with camera link
    minimal_urdf = """<?xml version="1.0"?>
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
        
        # 1. Robot State Publisher (creates base_link)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'robot_description': minimal_urdf,
                'use_sim_time': use_sim_time
            }],
            output='screen'
        ),
        
        # 2. Static TF: base_link -> odom (since we have no wheel encoders)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='odom_to_base',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
            output='screen'
        ),
        
        # 3. Static TF: camera_link -> camera_depth_optical_frame (CRITICAL!)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_to_depth_optical',
            arguments=[
                '0', '0', '0',
                '-1.5707963267948966', '0', '-1.5707963267948966',
                'camera_link', 'camera_depth_optical_frame'
            ],
            output='screen'
        ),
        
        # 4. Static TF: camera_link -> camera_color_optical_frame
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_to_color_optical',
            arguments=[
                '0', '0', '0',
                '-1.5707963267948966', '0', '-1.5707963267948966',
                'camera_link', 'camera_color_optical_frame'
            ],
            output='screen'
        ),
        
        # 5. D435 Camera - STANDARD RESOLUTION (not too low for odometry)
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
                # Use 640x480 for better feature detection
                'depth_module.profile': '640x480x30',
                'rgb_camera.profile': '640x480x30',
                'pointcloud.enable': True,  # Enable for RViz visualization
                'pointcloud.stream_filter': 2,  # Texture filter
                'use_sim_time': use_sim_time,
            }],
            output='screen'
        ),
        
        # 6. RGB-D Odometry
        Node(
            package='rtabmap_odom',
            executable='rgbd_odometry',
            name='rgbd_odometry',
            parameters=[{
                'frame_id': 'camera_link',  # Changed from base_link
                'odom_frame_id': 'odom',
                'publish_tf': True,
                'subscribe_depth': True,
                'subscribe_rgb': True,
                'approx_sync': True,
                'approx_sync_max_interval': 0.1,  # Increased tolerance
                'use_sim_time': use_sim_time,
                
                # TUNED FOR STATIONARY/SLOW MOVEMENT
                'Odom/Strategy': '0',  # Frame-to-Map (better for slow movement)
                'Odom/ResetCountdown': '1',
                'Odom/GuessMotion': 'false',  # Don't assume motion
                'Vis/CorGuessWinSize': '20',
                'Vis/MaxDepth': '4.0',
                'Vis/MinInliers': '5',  # LOWERED from 10 - less strict
                'Vis/MaxFeatures': '1000',  # Increased features
                'Vis/FeatureType': '6',  # GFTT (good for textured scenes)
                'GFTT/MinDistance': '5',
                'GFTT/QualityLevel': '0.001',
            }],
            remappings=[
                ('rgb/image', '/camera/camera/color/image_raw'),
                ('rgb/camera_info', '/camera/camera/color/camera_info'),
                ('depth/image', '/camera/camera/aligned_depth_to_color/image_raw'),
                ('odom', '/rtabmap/odom'),  # Publish to /rtabmap/odom
            ],
            output='screen'
        ),
        
        # 7. RTAB-Map SLAM - CORRECTED TOPIC REMAPPING
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            parameters=[{
                'frame_id': 'camera_link',
                'odom_frame_id': 'odom',
                'map_frame_id': 'map',
                'subscribe_depth': True,
                'subscribe_rgb': True,
                'subscribe_odom_info': True,  # Use odometry quality info
                'approx_sync': True,
                'queue_size': 30,
                
                # Database
                'database_path': LaunchConfiguration('database_path'),
                'Mem/IncrementalMemory': 'true',
                'Mem/InitWMWithAllNodes': 'false',
                
                # Detection rate - for STATIONARY MAPPING
                'Rtabmap/DetectionRate': '0.5',  # Process every 2 seconds
                'RGBD/LinearUpdate': '0.05',  # Small movements
                'RGBD/AngularUpdate': '0.05',
                
                # Features
                'Vis/MaxFeatures': '1000',
                'Vis/MinInliers': '5',  # LOWERED - less strict
                'Kp/MaxFeatures': '1000',
                'Kp/DetectorStrategy': '6',  # GFTT
                
                # Grid map for navigation
                'Grid/FromDepth': 'true',
                'Grid/MaxObstacleHeight': '0.4',
                'Grid/MinObstacleHeight': '0.05',
                'Grid/RangeMax': '5.0',
                'Grid/CellSize': '0.05',
                'Grid/DepthDecimation': '4',
                
                # Registration
                'Reg/Strategy': '1',  # ICP
                'Reg/Force3DoF': 'true',
                'Icp/VoxelSize': '0.05',
                'Icp/MaxTranslation': '0.2',
                
                'use_sim_time': use_sim_time,
            }],
            remappings=[
                ('rgb/image', '/camera/camera/color/image_raw'),
                ('rgb/camera_info', '/camera/camera/color/camera_info'),
                ('depth/image', '/camera/camera/aligned_depth_to_color/image_raw'),
                ('odom', '/rtabmap/odom'),  # CRITICAL FIX: Subscribe to rgbd_odometry output
            ],
            arguments=['--delete_db_on_start'],
            output='screen'
        ),
        
        # 8. Multi-Waypoint Navigator
        Node(
            package='lunar_robot_autonomous',
            executable='multi_waypoint_navigator',
            name='multi_waypoint_navigator',
            parameters=[{
                'goal_tolerance': 0.3,
                'forward_speed': 0.25,
                'turn_speed': 0.4,
                'use_sim_time': use_sim_time,
            }],
            output='screen'
        ),
        
        # 9. RViz
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
        ),
    ])