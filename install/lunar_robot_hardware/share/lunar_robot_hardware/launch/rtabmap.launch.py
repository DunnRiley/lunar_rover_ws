#!/usr/bin/env python3
"""
FIXED RTAB-Map Launch File for D435 Camera
Correct topic remappings and odometry handling
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Parameters
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # Minimal robot URDF
    minimal_urdf = """<?xml version="1.0"?>
<robot name="lunar_rover">
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.3 0.2"/>
      </geometry>
    </visual>
  </link>
</robot>
"""
    
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('localization', default_value='false'),
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
        
        # 2. Transform: base_link -> camera_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_camera_link',
            arguments=[
                '0.15', '0', '0.2',
                '0', '0', '0',
                'base_link',
                'camera_link'
            ],
            output='screen'
        ),
        
        # 3. D435 Camera - NO REMAPPING HERE, camera publishes to its own namespace
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
                'depth_module.profile': '640x480x30',
                'rgb_camera.profile': '640x480x30',
                'use_sim_time': use_sim_time,
            }],
            output='screen'
        ),
        
        # 4. RGB-D Odometry - Creates /odom from camera data
        Node(
            package='rtabmap_odom',
            executable='rgbd_odometry',
            name='rgbd_odometry',
            parameters=[{
                'frame_id': 'base_link',
                'subscribe_depth': True,
                'subscribe_rgb': True,
                'approx_sync': True,
                'use_sim_time': use_sim_time,
                
                # Odometry parameters
                'Odom/Strategy': '0',  # Frame-to-Map
                'Odom/ResetCountdown': '1',
                'Vis/CorGuessWinSize': '20',
                'Vis/MaxDepth': '4.0',
                'Vis/MinInliers': '15',
            }],
            remappings=[
                # Map RealSense topics to what rgbd_odometry expects
                ('rgb/image', '/camera/camera/color/image_raw'),
                ('rgb/camera_info', '/camera/camera/color/camera_info'),
                ('depth/image', '/camera/camera/aligned_depth_to_color/image_raw'),
            ],
            output='screen'
        ),
        
        # 5. RTAB-Map SLAM Node
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            parameters=[{
                'frame_id': 'base_link',
                'subscribe_depth': True,
                'subscribe_rgb': True,
                'subscribe_scan': False,
                'use_action_for_goal': True,
                'approx_sync': True,
                
                # Database
                'database_path': LaunchConfiguration('database_path'),
                'Mem/IncrementalMemory': 'true',
                'Mem/InitWMWithAllNodes': 'false',
                
                # Registration
                'Reg/Strategy': '1',  # ICP
                'Reg/Force3DoF': 'true',
                
                # Visual features
                'RGBD/NeighborLinkRefining': 'true',
                'RGBD/ProximityBySpace': 'true',
                'RGBD/AngularUpdate': '0.01',
                'RGBD/LinearUpdate': '0.01',
                'RGBD/OptimizeFromGraphEnd': 'false',
                
                # Loop closure
                'Kp/MaxDepth': '4.0',
                'Kp/DetectorStrategy': '0',  # GFTT/BRIEF -> 0=GFTT/ORB (works without xfeatures2d)
                
                # Grid mapping
                'Grid/FromDepth': 'false',
                'Grid/MaxObstacleHeight': '0.4',
                'Grid/MinObstacleHeight': '0.1',
                
                # Performance
                'RGBD/OptimizeMaxError': '0.1',
                'Rtabmap/TimeThr': '700',
                'Rtabmap/DetectionRate': '1',
                
                'use_sim_time': use_sim_time,
            }],
            remappings=[
                # Map RealSense topics to what RTAB-Map expects
                ('rgb/image', '/camera/camera/color/image_raw'),
                ('rgb/camera_info', '/camera/camera/color/camera_info'),
                ('depth/image', '/camera/camera/aligned_depth_to_color/image_raw'),
                # Odometry comes from rgbd_odometry node above
            ],
            arguments=['--delete_db_on_start'] if not LaunchConfiguration('localization') else [],
            output='screen'
        ),
        
        # 6. RTAB-Map Visualization
        Node(
            package='rtabmap_viz',
            executable='rtabmap_viz',
            name='rtabmap_viz',
            parameters=[{
                'frame_id': 'base_link',
                'subscribe_depth': True,
                'subscribe_rgb': True,
                'subscribe_odom_info': True,
                'approx_sync': True,
                'use_sim_time': use_sim_time,
            }],
            remappings=[
                ('rgb/image', '/camera/camera/color/image_raw'),
                ('rgb/camera_info', '/camera/camera/color/camera_info'),
                ('depth/image', '/camera/camera/aligned_depth_to_color/image_raw'),
            ],
            output='screen'
        ),
        
        # 7. RViz with RTAB-Map visualization
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