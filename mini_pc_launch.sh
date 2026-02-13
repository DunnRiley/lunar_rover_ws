#!/bin/bash
# ========================================================================
# Mini PC Launch - OPTIMIZED FOR WIFI
# Very aggressive compression for smooth streaming over poor WiFi
# ========================================================================

echo "========================================="
echo "  MINI PC: Optimized WiFi Mode"
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
# 1. ROBOT DESCRIPTION
# ========================================================================

echo "[1/5] Starting Robot State Publisher..."

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

echo "[2/5] Starting Static Transforms..."

ros2 run tf2_ros static_transform_publisher \
  0 0 0 -1.5707963267948966 0 -1.5707963267948966 \
  camera_link camera_color_optical_frame &

ros2 run tf2_ros static_transform_publisher \
  0 0 0 -1.5707963267948966 0 -1.5707963267948966 \
  camera_link camera_depth_optical_frame &

sleep 1
echo "  ✓ TF Tree ready"
echo ""

# ========================================================================
# 3. FRONT CAMERA - HIGH RATE CAPTURE
# ========================================================================

echo "[3/5] Starting D435 Camera (high rate capture)..."
echo "  Camera: 424x240 @ 30fps (will be decimated for streaming)"

ros2 launch realsense2_camera rs_launch.py \
  camera_name:=camera \
  camera_namespace:=camera \
  enable_depth:=true \
  enable_color:=true \
  pointcloud.enable:=false \
  align_depth.enable:=true \
  enable_sync:=true \
  depth_module.profile:=424x240x30 \
  rgb_camera.profile:=424x240x30 &

CAM_PID=$!
sleep 5

if ps -p $CAM_PID > /dev/null 2>&1; then
    echo "  ✓ D435 camera running at 30 FPS"
else
    echo "  ✗ D435 camera failed to start!"
fi
echo ""

# ========================================================================
# 4. OPTIMIZED IMAGE PIPELINE - Aggressive Compression & Buffering
# ========================================================================

echo "[4/5] Starting Optimized Image Pipeline..."

if [ -f ~/lunar_rover_ws/optimized_image_pipeline.py ]; then
    # RGB Color - Very aggressive compression
    python3 ~/lunar_rover_ws/optimized_image_pipeline.py \
      --ros-args \
      -p input_topic:=/camera/camera/color/image_raw/compressed \
      -p output_topic:=/camera/camera/color/optimized/compressed \
      -p jpeg_quality:=20 \
      -p decimation:=5 \
      -p buffer_delay_sec:=5.0 \
      -p target_fps:=6.0 \
      -p resize_factor:=1.0 &
    
    sleep 1
    echo "  ✓ RGB pipeline: 30fps → decimate 1/5 → 20% JPEG → buffer → 6fps output"
    
    # Depth - Less aggressive (already grayscale)
    python3 ~/lunar_rover_ws/optimized_image_pipeline.py \
      --ros-args \
      -p input_topic:=/camera/camera/aligned_depth_to_color/image_raw/compressed \
      -p output_topic:=/camera/camera/depth/optimized/compressed \
      -p jpeg_quality:=40 \
      -p decimation:=3 \
      -p buffer_delay_sec:=5.0 \
      -p target_fps:=6.0 &
    
    sleep 1
    echo "  ✓ Depth pipeline: 30fps → decimate 1/3 → 40% JPEG → buffer → 6fps output"
    
else
    echo "  ✗ optimized_image_pipeline.py not found - using basic buffer"
    
    # Fallback to basic buffer
    if [ -f ~/lunar_rover_ws/image_buffer.py ]; then
        python3 ~/lunar_rover_ws/image_buffer.py \
          --ros-args \
          -p input_topic:=/camera/camera/color/image_raw/compressed \
          -p output_topic:=/camera/camera/color/buffered/compressed \
          -p buffer_delay_sec:=5.0 \
          -p target_fps:=6.0 &
        echo "  ✓ Basic buffer enabled"
    fi
fi

echo ""

# ========================================================================
# 5. REAR CAMERA - Optional
# ========================================================================

echo "[5/5] Rear Stereo Camera..."

# Find the stereo camera publisher script
if [ -f ~/lunar_rover_ws/DiagnosticAndTesting/stereo_camera_publisher.py ]; then
    STEREO_SCRIPT=~/lunar_rover_ws/DiagnosticAndTesting/stereo_camera_publisher.py
elif [ -f ~/lunar_rover_ws/stereo_camera_publisher.py ]; then
    STEREO_SCRIPT=~/lunar_rover_ws/stereo_camera_publisher.py
else
    STEREO_SCRIPT=""
fi

if [ -n "$STEREO_SCRIPT" ]; then
    echo "  Starting rear camera (480x180 @ 15fps)..."
    python3 "$STEREO_SCRIPT" \
      --ros-args \
      -p device:=/dev/video32 \
      -p width:=480 \
      -p height:=180 \
      -p fps:=15 \
      -p publish_rate:=15.0 &
    
    sleep 2
    
    # Start stereo combiner
    if [ -f ~/lunar_rover_ws/stereo_combiner.py ]; then
        python3 ~/lunar_rover_ws/stereo_combiner.py \
          --ros-args \
          -p left_crop_start:=0 \
          -p left_crop_width:=240 \
          -p right_crop_start:=240 \
          -p right_crop_width:=240 \
          -p publish_compressed:=true &
        
        sleep 1
        
        # Buffer the combined stereo
        if [ -f ~/lunar_rover_ws/image_buffer.py ]; then
            python3 ~/lunar_rover_ws/image_buffer.py \
              --ros-args \
              -p input_topic:=/camera_rear/stereo_combined/compressed \
              -p output_topic:=/camera_rear/stereo_optimized/compressed \
              -p buffer_delay_sec:=5.0 \
              -p target_fps:=6.0 &
        fi
        
        echo "  ✓ Rear stereo combined and buffered"
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
echo "  ✓✓✓ OPTIMIZED MODE READY ✓✓✓"
echo "========================================="
echo ""
echo "STREAMING STRATEGY:"
echo "  • Camera captures at 30 FPS (smooth local)"
echo "  • Decimates to 1/5 frames (6 FPS)"
echo "  • Re-compresses JPEG at 20% quality (tiny)"
echo "  • Buffers 5 seconds"
echo "  • Streams out at steady 6 FPS"
echo ""
echo "LAPTOP TOPICS (use these in RViz):"
echo "  Front RGB:  /camera/camera/color/optimized/compressed"
echo "  Depth:      /camera/camera/depth/optimized/compressed"
echo "  Rear:       /camera_rear/stereo_optimized/compressed"
echo ""
echo "EXPECTED BANDWIDTH: ~150-300 KB/sec total"
echo "EXPECTED LATENCY: 5 seconds (buffered)"
echo ""
echo "Press Ctrl+C to stop all nodes"
echo "========================================="

wait