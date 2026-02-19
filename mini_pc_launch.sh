#!/bin/bash
# ========================================================================
# Mini PC Launch - FIXED FOR WIFI STREAMING
# - Correct image decode pipeline
# - Toggleable delay buffer (set DELAY_SEC=0 to disable)
# - Clear topic naming
# ========================================================================

echo "========================================="
echo "  MINI PC: Camera Streaming (Fixed)"
echo "========================================="

cd ~/lunar_rover_ws

# ── ROS2 setup ──────────────────────────────────────────────────────────
if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash && echo "✓ ROS2 Jazzy"
elif [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash && echo "✓ ROS2 Humble"
else
    echo "✗ No ROS2 installation found!" && exit 1
fi

[ -f install/setup.bash ] && source install/setup.bash && echo "✓ Workspace sourced"

export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
echo "✓ Network: ROS_DOMAIN_ID=42, SUBNET"
echo ""

# ── Competition delay setting ────────────────────────────────────────────
# Set to 1.0 for competition (1-second delay), 0.0 for testing with no delay
DELAY_SEC=${DELAY_SEC:-0.0}
if [ "$(echo "$DELAY_SEC > 0" | bc -l)" = "1" ]; then
    echo "⏱  COMPETITION MODE: ${DELAY_SEC}s delay buffer ENABLED"
else
    echo "🔵  TEST MODE: No delay (live streaming)"
fi
echo ""

# ── Cleanup old processes ────────────────────────────────────────────────
echo "Stopping any old nodes..."
pkill -f "realsense2_camera_node" 2>/dev/null
pkill -f "optimized_image_pipeline" 2>/dev/null
pkill -f "stereo_camera_publisher" 2>/dev/null
pkill -f "stereo_combiner" 2>/dev/null
pkill -f "robot_state_publisher" 2>/dev/null
pkill -f "static_transform_publisher" 2>/dev/null
sleep 2
echo "✓ Clean"
echo ""

# ── Trap for clean shutdown ──────────────────────────────────────────────
trap 'echo ""; echo "Shutting down mini PC nodes..."; kill 0; exit' SIGINT SIGTERM

# ========================================================================
# 1. ROBOT DESCRIPTION + TF TREE
# ========================================================================
echo "[1/5] Robot State Publisher + TF..."

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

sleep 1

ros2 run tf2_ros static_transform_publisher \
    0 0 0 -1.5707963267948966 0 -1.5707963267948966 \
    camera_link camera_color_optical_frame &

ros2 run tf2_ros static_transform_publisher \
    0 0 0 -1.5707963267948966 0 -1.5707963267948966 \
    camera_link camera_depth_optical_frame &

sleep 1
echo "  ✓ TF tree ready"
echo ""

# ========================================================================
# 2. FRONT CAMERA (D435)
# ========================================================================
echo "[2/5] D435 Front Camera (424x240 @ 30fps)..."

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
echo "  Waiting 6s for camera to start..."
sleep 6

if ps -p $CAM_PID > /dev/null 2>&1; then
    echo "  ✓ D435 running (PID $CAM_PID)"
else
    echo "  ✗ D435 failed to start — check USB connection"
    echo "  Tip: run 'rs-enumerate-devices' to verify camera is detected"
fi
echo ""

# ========================================================================
# 3. FRONT CAMERA STREAMING PIPELINE
#    Reads COMPRESSED topic (realsense publishes this automatically)
#    → decimates → re-encodes at low quality → buffers → outputs
# ========================================================================
echo "[3/5] Front Camera Streaming Pipeline..."
echo "  Input:  /camera/camera/color/image_raw/compressed (30fps)"
echo "  Output: /camera/color/stream/compressed (6fps, low quality)"

if [ ! -f ~/lunar_rover_ws/optimized_image_pipeline.py ]; then
    echo "  ✗ optimized_image_pipeline.py not found in ~/lunar_rover_ws/"
    echo "  Please copy the fixed version there first."
else
    python3 ~/lunar_rover_ws/optimized_image_pipeline.py \
        --ros-args \
        -p input_topic:=/camera/camera/color/image_raw/compressed \
        -p output_topic:=/camera/color/stream/compressed \
        -p input_is_compressed:=true \
        -p jpeg_quality:=25 \
        -p decimation:=5 \
        -p buffer_delay_sec:=$DELAY_SEC \
        -p target_fps:=6.0 \
        -p resize_factor:=1.0 &

    sleep 1

    # Depth pipeline (less aggressive compression, lower rate is fine)
    python3 ~/lunar_rover_ws/optimized_image_pipeline.py \
        --ros-args \
        -p input_topic:=/camera/camera/aligned_depth_to_color/image_raw \
        -p output_topic:=/camera/depth/stream/compressed \
        -p input_is_compressed:=false \
        -p jpeg_quality:=50 \
        -p decimation:=10 \
        -p buffer_delay_sec:=$DELAY_SEC \
        -p target_fps:=3.0 &

    sleep 1
    echo "  ✓ Front camera pipeline running"
fi
echo ""

# ========================================================================
# 4. REAR STEREO CAMERA (USB stereo camera)
# ========================================================================
echo "[4/5] Rear Stereo Camera..."

# Find the stereo publisher script
STEREO_SCRIPT=""
for candidate in \
    ~/lunar_rover_ws/DiagnosticAndTesting/stereo_camera_publisher.py \
    ~/lunar_rover_ws/stereo_camera_publisher.py; do
    [ -f "$candidate" ] && STEREO_SCRIPT="$candidate" && break
done

if [ -n "$STEREO_SCRIPT" ]; then
    echo "  Starting stereo camera (device /dev/video0, 480x180 @ 15fps)..."
    python3 "$STEREO_SCRIPT" \
        --ros-args \
        -p device:=/dev/video0 \
        -p width:=480 \
        -p height:=180 \
        -p fps:=15 \
        -p publish_rate:=15.0 &

    sleep 2

    # Combine left + right into one side-by-side image
    if [ -f ~/lunar_rover_ws/stereo_combiner.py ]; then
        python3 ~/lunar_rover_ws/stereo_combiner.py \
            --ros-args \
            -p left_crop_start:=0 \
            -p left_crop_width:=240 \
            -p right_crop_start:=240 \
            -p right_crop_width:=240 \
            -p publish_compressed:=true &
        sleep 1
    fi

    # Stream the combined stereo image through the pipeline too
    if [ -f ~/lunar_rover_ws/optimized_image_pipeline.py ]; then
        python3 ~/lunar_rover_ws/optimized_image_pipeline.py \
            --ros-args \
            -p input_topic:=/camera_rear/stereo_combined/compressed \
            -p output_topic:=/camera_rear/stream/compressed \
            -p input_is_compressed:=true \
            -p jpeg_quality:=30 \
            -p decimation:=3 \
            -p buffer_delay_sec:=$DELAY_SEC \
            -p target_fps:=6.0 &
        sleep 1
    fi

    echo "  ✓ Rear stereo pipeline running"
else
    echo "  ⚠ stereo_camera_publisher.py not found — rear camera disabled"
fi
echo ""

# ========================================================================
# 5. MOTOR CONTROLLER (Uncomment when ready)
# ========================================================================
echo "[5/5] Motor Controller..."
# ros2 run lunar_robot_hardware arduino_motor_controller &
echo "  (Disabled — uncomment in script to enable)"
echo ""

# ========================================================================
# SUMMARY
# ========================================================================
echo "========================================="
echo "  ✓✓✓ MINI PC READY ✓✓✓"
echo "========================================="
echo ""
echo "DELAY MODE: ${DELAY_SEC}s"
echo ""
echo "TOPICS TO USE IN RVIZ ON LAPTOP:"
echo ""
echo "  Front RGB (color):  /camera/color/stream/compressed"
echo "  Front Depth:        /camera/depth/stream/compressed"
echo "  Rear Stereo:        /camera_rear/stream/compressed"
echo ""
echo "  (All use Transport = 'compressed' in RViz Image display)"
echo ""
echo "BANDWIDTH ESTIMATE:"
echo "  ~150-250 KB/s total at these settings"
echo ""
if [ "$(echo "$DELAY_SEC > 0" | bc -l)" = "1" ]; then
    echo "⏱  Competition delay active: ${DELAY_SEC}s"
    echo "   To change delay: DELAY_SEC=1.0 bash mini_pc_launch.sh"
else
    echo "💡 To enable competition delay:"
    echo "   DELAY_SEC=1.0 bash mini_pc_launch.sh"
fi
echo ""
echo "Press Ctrl+C to stop all nodes"
echo "========================================="

wait