#!/bin/bash
# Quick test script to verify camera transforms
# Run this to test if transforms fix the point cloud issue

echo "=========================================="
echo "Camera Transform Test Script"
echo "=========================================="
echo ""
echo "This script will:"
echo "1. Publish base_link frame"
echo "2. Publish camera transforms"
echo "3. Launch D435 camera"
echo "4. Launch RViz"
echo ""
echo "Press Ctrl+C in THIS terminal to stop everything"
echo ""
sleep 2

# Kill any existing ROS nodes
pkill -f ros2
pkill -f rviz2
sleep 1

echo "Starting robot state publisher (creates base_link)..."
ros2 run robot_state_publisher robot_state_publisher \
    --ros-args \
    -p robot_description:="<?xml version='1.0'?>
<robot name='lunar_rover'>
  <link name='base_link'>
    <visual>
      <geometry>
        <box size='0.5 0.3 0.2'/>
      </geometry>
    </visual>
  </link>
  <link name='camera_link'/>
  <joint name='base_to_camera' type='fixed'>
    <parent link='base_link'/>
    <child link='camera_link'/>
    <origin xyz='0.15 0 0.2' rpy='0 0 0'/>
  </joint>
</robot>" \
    -p use_sim_time:=false &

RSP_PID=$!
sleep 2

echo "Publishing camera optical frame transform..."
ros2 run tf2_ros static_transform_publisher \
    0 0 0 \
    -1.5707963267948966 0 -1.5707963267948966 \
    camera_link camera_depth_optical_frame &

TF1_PID=$!
sleep 1

ros2 run tf2_ros static_transform_publisher \
    0 0 0 \
    -1.5707963267948966 0 -1.5707963267948966 \
    camera_link camera_color_optical_frame &

TF2_PID=$!
sleep 1

echo "Starting D435 camera..."
ros2 launch realsense2_camera rs_launch.py \
    camera_name:=camera \
    enable_depth:=true \
    enable_color:=true \
    pointcloud.enable:=true &

CAM_PID=$!
sleep 3

echo "Checking if transforms are working..."
timeout 3 ros2 run tf2_ros tf2_echo base_link camera_depth_optical_frame || echo "Transform check timed out (this is normal)"
sleep 1

echo ""
echo "=========================================="
echo "Starting RViz..."
echo "=========================================="
echo ""
echo "In RViz:"
echo "1. Check that point cloud appears"
echo "2. If not, change Fixed Frame to 'camera_link'"
echo "3. Add PointCloud2 display with topic: /camera/camera/depth/color/points"
echo ""

# Try to launch RViz with config, fallback to plain RViz
if [ -f ~/lunar_robot_ws/src/lunar_robot_description/config/real_hardware_navigation.rviz ]; then
    ros2 run rviz2 rviz2 -d ~/lunar_robot_ws/src/lunar_robot_description/config/real_hardware_navigation.rviz &
else
    ros2 run rviz2 rviz2 &
fi

RVIZ_PID=$!

echo ""
echo "All processes started!"
echo "Press Ctrl+C to stop everything"
echo ""

# Wait for Ctrl+C
trap "echo 'Stopping...'; kill $RSP_PID $TF1_PID $TF2_PID $CAM_PID $RVIZ_PID 2>/dev/null; pkill -f ros2; exit 0" SIGINT
wait