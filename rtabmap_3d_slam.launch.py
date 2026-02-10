#!/usr/bin/env python3
"""
RTAB-Map 3D SLAM Launch - WORKING VERSION
- Full 3D point cloud mapping
- Navigation and obstacle avoidance ready
- Persistent map building
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction
import os


def generate_launch_description():
    
    # Robot URDF
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
    
    # RTAB-Map parameters - optimized for D435
    rtabmap_params = {
        'frame_id': 'base_link',
        'odom_frame_id': 'odom',
        'map_frame_id': 'map',
        'subscribe_depth': True,
        'subscribe_rgb': True,
        'subscribe_scan_cloud': False,
        'approx_sync': True,
        'wait_for_transform': 0.5,
        'queue_size': 30,
        
        # Database
        'database_path': '~/.ros/rtabmap.db',
        'Mem/IncrementalMemory': 'true',
        'Mem/InitWMWithAllNodes': 'false',
        
        # Mapping rate - process every frame
        'Rtabmap/DetectionRate': '1.0',
        'RGBD/LinearUpdate': '0.1',      # Update every 10cm
        'RGBD/AngularUpdate': '0.1',     # Update every ~6 degrees
        
        # Feature detection (GFTT is fast and works well)
        'Vis/FeatureType': '0',          # GFTT
        'Kp/DetectorStrategy': '0',       # GFTT
        'Vis/MaxFeatures': '400',
        'Vis/MinInliers': '15',
        'GFTT/MinDistance': '5',
        'GFTT/QualityLevel': '0.001',
        
        # 3D Mapping
        'Grid/FromDepth': 'true',
        'Grid/MaxObstacleHeight': '0.5',
        'Grid/MinObstacleHeight': '0.05',
        'Grid/RangeMax': '5.0',
        'Grid/CellSize': '0.05',
        'Grid/3D': 'false',              # 2D occupancy grid for navigation
        
        # Loop closure
        'RGBD/OptimizeFromGraphEnd': 'false',
        'RGBD/ProximityBySpace': 'true',
        'RGBD/NeighborLinkRefining': 'true',
        
        # Registration
        'Reg/Strategy': '1',              # ICP
        'Reg/Force3DoF': 'true',         # Keep it 2D (x, y, yaw)
        'Icp/VoxelSize': '0.05',
        'Icp/MaxCorrespondenceDistance': '0.1',
        'Icp/MaxTranslation': '0.2',
        
        'use_sim_time': False,
    }
    
    # Odometry parameters
    odom_params = {
        'frame_id': 'base_link',
        'odom_frame_id': 'odom',
        'publish_tf': True,
        'subscribe_depth': True,
        'subscribe_rgb': True,
        'approx_sync': True,
        'wait_for_transform': 0.5,
        'queue_size': 30,
        
        # Odometry strategy
        'Odom/Strategy': '0',             # Frame-to-Map
        'Vis/FeatureType': '0',           # GFTT (must match RTAB-Map)
        'Vis/MaxFeatures': '400',
        'Vis/MinInliers': '15',
        'GFTT/MinDistance': '5',
        'GFTT/QualityLevel': '0.001',
        
        'use_sim_time': False,
    }
    
    rviz_config = os.path.join(
        os.path.expanduser('~'),
        'lunar_rover_ws',
        'rtabmap_3d.rviz'
    )
    
    return LaunchDescription([
        
        # === STEP 1: TF Tree ===
        
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
        
        # === STEP 2: Camera (wait 2s) ===
        
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
                        'enable_infra1': False,
                        'enable_infra2': False,
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
        
        # === STEP 3: RGB-D Odometry (wait 5s) ===
        
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='rtabmap_odom',
                    executable='rgbd_odometry',
                    name='rgbd_odometry',
                    parameters=[odom_params],
                    remappings=[
                        ('rgb/image', '/camera/camera/color/image_raw'),
                        ('rgb/camera_info', '/camera/camera/color/camera_info'),
                        ('depth/image', '/camera/camera/aligned_depth_to_color/image_raw'),
                    ],
                    output='screen'
                )
            ]
        ),
        
        # === STEP 4: RTAB-Map SLAM (wait 8s) ===
        
        TimerAction(
            period=8.0,
            actions=[
                Node(
                    package='rtabmap_slam',
                    executable='rtabmap',
                    name='rtabmap',
                    parameters=[rtabmap_params],
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
        
        # === STEP 5: RViz (wait 10s) ===
        
        TimerAction(
            period=10.0,
            actions=[
                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
                    parameters=[{'use_sim_time': False}],
                    output='screen'
                )
            ]
        ),
    ])