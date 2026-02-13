#!/bin/bash
# ========================================================================
# Mini PC Launch - ULTRA LOW BANDWIDTH MODE
# For poor WiFi connections - sacrifices quality for smooth streaming
# ========================================================================

echo "========================================="
echo "  MINI PC: Hardware Launch"
echo "  MODE: Ultra Low Bandwidth"
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

# Network config
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET

echo "✓ Network: ROS_DOMAIN_ID=42, SUBNET discovery"
echo ""

# ========================================================================
# 1. ROBOT DESCRIPTION (Inline URDF)
# ========================================================================

echo "[1/4] Starting Robot State Publisher..."

URDF_CONTENT='<?xml version="1.0"?>
<robot name="lunar_rover">
  <link name="base_link"/>
  <link name="camera_link"/>
  <link name="camera_rear_link"/>
  
  <joint name="base_to_camera" type="fixed">
    <parent link="base_link"/>
    <child link="camera_link"/>
    <origin xyz="0.2 0 0.15" rpy="0 0 0"/>
  </joint>
  
  <joint name="base_to_camera_rear" type="fixed">
    <parent link="base_link"/>
    <child link="camera_rear_link"/>
    <origin xyz="-0.2 0 0.15" rpy="0 0 3.14159"/>
  </joint>
</robot>'

ros2 run robot_state_publisher robot_state_publisher \
  --ros-args -p robot_description:="$URDF_CONTENT" &

sleep 2
echo "  ✓ robot_state_publisher running"
echo ""

# ========================================================================
# 2. STATIC TRANSFORMS
# ========================================================================

echo "[2/4] Starting Static Transforms..."

ros2 run tf2_ros static_transform_publisher \
  0 0 0 -1.5707963267948966 0 -1.5707963267948966 \
  camera_link camera_color_optical_frame &

sleep 1
echo "  ✓ TF Tree ready"
echo ""

# ========================================================================
# 3. FRONT CAMERA - MINIMAL BANDWIDTH MODE
# ========================================================================

echo "[3/4] Starting Front Camera (D435) - LOW BANDWIDTH..."
echo "  Resolution: 320x180 @ 6fps"
echo "  Depth: ENABLED (aligned to color)"
echo "  Point Cloud: DISABLED"

ros2 launch realsense2_camera rs_launch.py \
  camera_name:=camera \
  camera_namespace:=camera \
  enable_depth:=true \
  enable_color:=true \
  pointcloud.enable:=false \
  align_depth.enable:=true \
  enable_sync:=true \
  depth_module.profile:=320x180x6 \
  rgb_camera.profile:=320x180x6 &

CAM_PID=$!
sleep 5

if ps -p $CAM_PID > /dev/null 2>&1; then
    echo "  ✓ D435 camera running (color only, 320x180 @ 6fps)"
    echo "  Subscribe to: /camera/camera/color/image_raw/compressed"
else
    echo "  ✗ D435 camera failed to start!"
fi
echo ""

# ========================================================================
# 4. REAR CAMERA - ENABLED WITH STEREO COMBINER
# ========================================================================

echo "[4/4] Starting Rear Stereo Camera (IFWATER)..."

# Find the stereo camera publisher script
if [ -f ~/lunar_rover_ws/DiagnosticAndTesting/stereo_camera_publisher.py ]; then
    STEREO_SCRIPT=~/lunar_rover_ws/DiagnosticAndTesting/stereo_camera_publisher.py
elif [ -f ~/lunar_rover_ws/stereo_camera_publisher.py ]; then
    STEREO_SCRIPT=~/lunar_rover_ws/stereo_camera_publisher.py
else
    echo "  ✗ stereo_camera_publisher.py not found - skipping rear camera"
    STEREO_SCRIPT=""
fi

if [ -n "$STEREO_SCRIPT" ]; then
    echo "  Found: $STEREO_SCRIPT"
    # Low bandwidth: 640x240 @ 6fps instead of 800x300 @ 15fps
    python3 "$STEREO_SCRIPT" \
      --ros-args \
      -p device:=/dev/video32 \
      -p width:=640 \
      -p height:=240 \
      -p fps:=6 \
      -p publish_rate:=6.0 &
    
    sleep 2
    echo "  ✓ Rear camera running (640x240 @ 6fps)"
    
    # Start stereo combiner to create single side-by-side image
    if [ -f ~/lunar_rover_ws/stereo_combiner.py ]; then
        echo "  Starting stereo combiner..."
        python3 ~/lunar_rover_ws/stereo_combiner.py \
          --ros-args \
          -p left_crop_start:=0 \
          -p left_crop_width:=320 \
          -p right_crop_start:=320 \
          -p right_crop_width:=320 &
        
        sleep 1
        echo "  ✓ Stereo combined at: /camera_rear/stereo_combined/compressed"
    else
        echo "  ⚠ stereo_combiner.py not found - using separate left/right"
    fi
else
    echo "  Rear camera disabled"
fi

echo ""

# ========================================================================
# MOTOR CONTROLLER (Uncomment when ready)
# ========================================================================

# echo "Starting Motor Controller..."
# python3 ~/lunar_rover_ws/motor_controller.py &

echo ""
echo "========================================="
echo "  ✓✓✓ MINI PC READY - LOW BANDWIDTH ✓✓✓"
echo "========================================="
echo ""
echo "TOPICS:"
echo "  /camera/camera/color/image_raw/compressed (320x180 @ 6fps)"
echo "  /camera/camera/aligned_depth_to_color/image_raw/compressedDepth"
echo "  /camera_rear/stereo_combined/compressed (640x240 @ 6fps)"
echo "  /tf, /tf_static"
echo ""
echo "BANDWIDTH: ~300-500 KB/sec (light)"
echo ""
echo "OPTIONAL: Enable 5-second buffer for smoother playback"
echo "  (Competition-style 5-second delay)"
echo "  Uncomment the section below to enable buffering"
echo ""

# ========================================================================
# OPTIONAL: IMAGE BUFFER FOR SMOOTH DELAYED PLAYBACK
# Uncomment to enable 5-second buffered playback (like competition delay)
# ========================================================================

if [ -f ~/lunar_rover_ws/image_buffer.py ]; then
    echo "Starting image buffer (5 second delay)..."
    
    # Buffer front color camera
    python3 ~/lunar_rover_ws/image_buffer.py \
      --ros-args \
      -p input_topic:=/camera/camera/color/image_raw/compressed \
      -p output_topic:=/camera/camera/color/buffered/compressed \
      -p buffer_delay_sec:=5.0 \
      -p target_fps:=6.0 &
    
    # Buffer rear stereo
    python3 ~/lunar_rover_ws/image_buffer.py \
      --ros-args \
      -p input_topic:=/camera_rear/stereo_combined/compressed \
      -p output_topic:=/camera_rear/stereo_buffered/compressed \
      -p buffer_delay_sec:=5.0 \
      -p target_fps:=6.0 &
    
    echo "  ✓ Buffered topics available:"
    echo "    /camera/camera/color/buffered/compressed"
    echo "    /camera_rear/stereo_buffered/compressed"
    echo ""
fi

echo "Press Ctrl+C to stop all nodes"
echo "========================================="

wait