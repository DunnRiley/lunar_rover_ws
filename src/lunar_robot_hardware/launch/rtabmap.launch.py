#!/usr/bin/env python3
"""
OPTIMIZED RTAB-Map Launch - Better Performance
Reduced resolution, faster processing, lighter visualization
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
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
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('localization', default_value='false'),
        DeclareLaunchArgument('database_path', default_value='~/.ros/rtabmap.db'),
        
        # 1. Robot State Publisher
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
            arguments=['0.15', '0', '0.2', '0', '0', '0', 'base_link', 'camera_link'],
            output='screen'
        ),
        
        # 3. D435 Camera - REDUCED RESOLUTION FOR PERFORMANCE
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
                # OPTIMIZED: Lower resolution = faster processing
                'depth_module.profile': '424x240x15',  # Was 640x480x30
                'rgb_camera.profile': '424x240x15',    # Was 640x480x30
                # Disable point cloud publishing (RTAB-Map creates its own)
                'pointcloud.enable': False,
                'use_sim_time': use_sim_time,
            }],
            output='screen'
        ),
        
        # 4. RGB-D Odometry - OPTIMIZED
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
                
                # OPTIMIZED: Faster odometry
                'Odom/Strategy': '0',
                'Odom/ResetCountdown': '1',
                'Vis/CorGuessWinSize': '10',  # Reduced from 20
                'Vis/MaxDepth': '3.0',         # Reduced from 4.0
                'Vis/MinInliers': '10',        # Reduced from 15
                'Vis/MaxFeatures': '400',      # Limit features
            }],
            remappings=[
                ('rgb/image', '/camera/camera/color/image_raw'),
                ('rgb/camera_info', '/camera/camera/color/camera_info'),
                ('depth/image', '/camera/camera/aligned_depth_to_color/image_raw'),
            ],
            output='screen'
        ),
        
        # 5. RTAB-Map SLAM - HIGHLY OPTIMIZED
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
                'queue_size': 30,  # Increased buffer
                
                # Database
                'database_path': LaunchConfiguration('database_path'),
                'Mem/IncrementalMemory': 'true',
                'Mem/InitWMWithAllNodes': 'false',
                
                # PERFORMANCE: Process fewer frames
                'Rtabmap/DetectionRate': '2',       # Process every 2 seconds (was 1)
                'Rtabmap/TimeThr': '0',             # No time limit
                'RGBD/LinearUpdate': '0.05',        # Update every 5cm (was 1cm)
                'RGBD/AngularUpdate': '0.05',       # Update every ~3° (was 0.57°)
                
                # PERFORMANCE: Reduce features
                'Vis/MaxFeatures': '400',           # Max features per frame
                'Vis/MinInliers': '10',             # Min features for valid match
                'Kp/MaxFeatures': '400',            # Max keypoints
                'Kp/DetectorStrategy': '6',         # GFTT (faster than ORB)
                
                # PERFORMANCE: Lighter mapping
                'Grid/FromDepth': 'true',           # Use depth instead of features
                'Grid/MaxObstacleHeight': '0.4',
                'Grid/MinObstacleHeight': '0.05',
                'Grid/RangeMax': '3.0',             # Limit mapping range
                'Grid/CellSize': '0.05',            # 5cm grid cells
                'Grid/DepthDecimation': '4',        # Downsample depth
                
                # PERFORMANCE: Lighter loop closure
                'RGBD/ProximityBySpace': 'false',   # Disable proximity detection
                'RGBD/NeighborLinkRefining': 'false', # Disable refinement
                'RGBD/OptimizeFromGraphEnd': 'false',
                
                # Registration
                'Reg/Strategy': '1',                # ICP
                'Reg/Force3DoF': 'true',           # 2D only
                'Icp/PointToPlane': 'true',
                'Icp/Iterations': '10',             # Reduced iterations
                'Icp/VoxelSize': '0.05',           # 5cm voxels
                'Icp/MaxTranslation': '0.2',
                'Icp/MaxCorrespondenceDistance': '0.1',
                
                'use_sim_time': use_sim_time,
            }],
            remappings=[
                ('rgb/image', '/camera/camera/color/image_raw'),
                ('rgb/camera_info', '/camera/camera/color/camera_info'),
                ('depth/image', '/camera/camera/aligned_depth_to_color/image_raw'),
            ],
            arguments=['--delete_db_on_start'],
            output='screen'
        ),
        
        # 6. RViz - Optimized Config
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(
                os.path.expanduser('~'),
                'lunar_rover_ws',
                'rtabmap_rviz_optimized.rviz'
            )] if os.path.exists(os.path.join(
                os.path.expanduser('~'),
                'lunar_rover_ws',
                'rtabmap_rviz_optimized.rviz'
            )) else [],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        ),
    ])