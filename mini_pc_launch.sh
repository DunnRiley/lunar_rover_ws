#!/bin/bash
# ========================================================================
# Mini PC Hardware Launch Script
# Place at: ~/lunar_rover_ws/mini_pc_launch.sh
# Runs: Robot State Publisher, D435 Camera, IFWATER Stereo Camera
# ========================================================================

echo "========================================="
echo "  MINI PC: Starting Hardware Nodes"
echo "========================================="

cd ~/lunar_rover_ws

# Source ROS2
if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
    echo "✓ ROS2 Jazzy"
elif [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
    echo "✓ ROS2 Humble"
else
    echo "✗ No ROS2 installation found!"
    exit 1
fi

# Source workspace (optional - works without it)
if [ -f install/setup.bash ]; then
    source install/setup.bash
    echo "✓ Workspace sourced"
fi

# Set network parameters for multi-machine ROS2
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET

echo "✓ Network: ROS_DOMAIN_ID=42, SUBNET discovery"
echo ""

# ========================================================================
# 1. ROBOT STATE PUBLISHER (Creates TF tree)
# ========================================================================

echo "[1/4] Starting TF Tree (Robot State Publisher)..."

# Complete URDF with front and rear camera mounts
ROBOT_URDF='<?xml version="1.0"?>
<robot name="lunar_rover">
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.3 0.2"/>
      </geometry>
    </visual>
  </link>
  
  <!-- Front D435 Camera -->
  <link name="camera_link"/>
  <joint name="base_to_camera" type="fixed">
    <parent link="base_link"/>
    <child link="camera_link"/>
    <origin xyz="0.15 0 0.2" rpy="0 0 0"/>
  </joint>
  
  <!-- Rear IFWATER Stereo Camera -->
  <link name="camera_rear_link"/>
  <joint name="base_to_camera_rear" type="fixed">
    <parent link="base_link"/>
    <child link="camera_rear_link"/>
    <origin xyz="-0.15 0 0.2" rpy="0 0 3.14159265359"/>
  </joint>
</robot>'

ros2 run robot_state_publisher robot_state_publisher \
  --ros-args \
  -p robot_description:="$ROBOT_URDF" \
  -p use_sim_time:=false &

sleep 2

echo "[2/4] Starting Static Transforms (Camera Optical Frames)..."

# D435 Camera optical frame transforms
ros2 run tf2_ros static_transform_publisher \
  0 0 0 -1.5707963267948966 0 -1.5707963267948966 \
  camera_link camera_depth_optical_frame &

ros2 run tf2_ros static_transform_publisher \
  0 0 0 -1.5707963267948966 0 -1.5707963267948966 \
  camera_link camera_color_optical_frame &

# IFWATER Stereo camera optical frame transforms
ros2 run tf2_ros static_transform_publisher \
  0 0 0 -1.5707963267948966 0 -1.5707963267948966 \
  camera_rear_link camera_rear_left_optical_frame &

ros2 run tf2_ros static_transform_publisher \
  0.06 0 0 -1.5707963267948966 0 -1.5707963267948966 \
  camera_rear_link camera_rear_right_optical_frame &

sleep 1
echo "  ✓ TF Tree ready"
echo ""

# ========================================================================
# 3. FRONT CAMERA (D435 - RGB + Depth + Point Cloud)
# ========================================================================

echo "[3/4] Starting Front Camera (D435)..."

ros2 launch realsense2_camera rs_launch.py \
  camera_name:=camera \
  camera_namespace:=camera \
  enable_depth:=true \
  enable_color:=true \
  pointcloud.enable:=true \
  align_depth.enable:=true \
  enable_sync:=true \
  depth_module.profile:=640x480x30 \
  rgb_camera.profile:=640x480x30 &

CAM_PID=$!
sleep 5

if ps -p $CAM_PID > /dev/null 2>&1; then
    echo "  ✓ D435 camera running (PID $CAM_PID)"
else
    echo "  ✗ D435 camera failed to start!"
fi
echo ""

# ========================================================================
# 4. REAR STEREO CAMERA (IFWATER)
# ========================================================================

echo "[4/4] Starting Rear Stereo Camera (IFWATER)..."

# Find the stereo camera publisher script
if [ -f ~/lunar_rover_ws/DiagnosticAndTesting/stereo_camera_publisher.py ]; then
    STEREO_SCRIPT=~/lunar_rover_ws/DiagnosticAndTesting/stereo_camera_publisher.py
elif [ -f ~/lunar_rover_ws/stereo_camera_publisher.py ]; then
    STEREO_SCRIPT=~/lunar_rover_ws/stereo_camera_publisher.py
else
    echo "  ✗ stereo_camera_publisher.py not found!"
    echo "  Looking in:"
    echo "    - ~/lunar_rover_ws/DiagnosticAndTesting/"
    echo "    - ~/lunar_rover_ws/"
    echo ""
    echo "  Skipping rear camera..."
    STEREO_SCRIPT=""
fi

if [ -n "$STEREO_SCRIPT" ]; then
    echo "  Found: $STEREO_SCRIPT"
    python3 "$STEREO_SCRIPT" \
      --ros-args \
      -p device:=/dev/video32 \
      -p width:=1600 \
      -p height:=600 \
      -p fps:=30 \
      -p publish_rate:=30.0 &
    
    STEREO_PID=$!
    sleep 2
    
    if ps -p $STEREO_PID > /dev/null 2>&1; then
        echo "  ✓ Stereo camera running (PID $STEREO_PID)"
    else
        echo "  ✗ Stereo camera failed to start"
        echo "  Check device /dev/video32 exists: ls -l /dev/video32"
    fi
else
    echo "  ⚠ Skipping stereo camera (script not found)"
fi

echo ""
echo "========================================="
echo "  ✓✓✓ MINI PC HARDWARE READY ✓✓✓"
echo "========================================="
echo ""
echo "Running nodes:"
echo "  • Robot State Publisher"
echo "  • TF Static Transforms"
echo "  • D435 Camera (front)"
if [ -n "$STEREO_SCRIPT" ]; then
    echo "  • IFWATER Stereo Camera (rear)"
fi
echo ""
echo "Network: ROS_DOMAIN_ID=42"
echo ""
echo "From LAPTOP, run:"
echo "  bash ~/lunar_rover_ws/laptop_ros_launch.sh"
echo ""
echo "Press Ctrl+C to stop all nodes"
echo "========================================="

# Wait for all background processes
wait