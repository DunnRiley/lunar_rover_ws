#!/bin/bash
# ============================================================================
# RTAB-Map SLAM Launch Script - FIXED VERSION v3
# ============================================================================
# Fixes: Camera crashes, Ctrl+C handling, proper cleanup

WORKSPACE="$HOME/lunar_rover_ws"
LOG_DIR="/tmp/rtabmap_logs"
RVIZ_CONFIG="$WORKSPACE/rtabmap_navigation.rviz"

# Store PIDs for cleanup
declare -a PIDS_TO_KILL

# Cleanup function that actually works
cleanup() {
    echo ""
    echo "=========================================="
    echo "  Stopping RTAB-Map...                   "
    echo "=========================================="
    
    # Kill by PID (more reliable than pkill)
    for pid in "${PIDS_TO_KILL[@]}"; do
        if ps -p "$pid" > /dev/null 2>&1; then
            echo "  Stopping process $pid..."
            kill "$pid" 2>/dev/null || true
        fi
    done
    
    # Backup: pkill if anything remains
    sleep 1
    pkill -9 -f rtabmap 2>/dev/null || true
    pkill -9 -f rgbd_odometry 2>/dev/null || true
    pkill -9 -f realsense 2>/dev/null || true
    pkill -9 -f robot_state_publisher 2>/dev/null || true
    pkill -9 -f static_transform_publisher 2>/dev/null || true
    pkill -9 -f rviz2 2>/dev/null || true
    
    echo "  ✓ Stopped"
    exit 0
}

# Trap Ctrl+C - this is critical!
trap cleanup SIGINT SIGTERM EXIT

echo "========================================================"
echo "    RTAB-Map SLAM System - Starting                    "
echo "========================================================"
echo ""

# Check workspace
if [ ! -d "$WORKSPACE" ]; then
    echo "ERROR: Workspace not found at $WORKSPACE"
    exit 1
fi

cd "$WORKSPACE"

if [ ! -f "install/setup.bash" ]; then
    echo "ERROR: Workspace not built! Run: colcon build"
    exit 1
fi

# Check parameter files
if [ ! -f "rtabmap_odom_params.yaml" ]; then
    echo "ERROR: rtabmap_odom_params.yaml not found in $WORKSPACE"
    echo "Please copy the fixed parameter files here."
    exit 1
fi

if [ ! -f "rtabmap_slam_params.yaml" ]; then
    echo "ERROR: rtabmap_slam_params.yaml not found in $WORKSPACE"
    exit 1
fi

source install/setup.bash

# Create log directory
mkdir -p "$LOG_DIR"
rm -f "$LOG_DIR"/*.log 2>/dev/null || true

echo "Step 1/6: Starting TF tree..."

# Robot URDF
ROBOT_URDF='<?xml version="1.0"?>
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
</robot>'

ros2 run robot_state_publisher robot_state_publisher \
    --ros-args \
    -p robot_description:="$ROBOT_URDF" \
    -p use_sim_time:=false \
    > "$LOG_DIR/robot_state_publisher.log" 2>&1 &
PIDS_TO_KILL+=($!)

ros2 run tf2_ros static_transform_publisher \
    0 0 0 -1.5707963267948966 0 -1.5707963267948966 \
    camera_link camera_depth_optical_frame > "$LOG_DIR/tf_depth.log" 2>&1 &
PIDS_TO_KILL+=($!)

ros2 run tf2_ros static_transform_publisher \
    0 0 0 -1.5707963267948966 0 -1.5707963267948966 \
    camera_link camera_color_optical_frame > "$LOG_DIR/tf_color.log" 2>&1 &
PIDS_TO_KILL+=($!)

ros2 run tf2_ros static_transform_publisher \
    0 0 0 0 0 0 \
    odom base_link > "$LOG_DIR/tf_odom.log" 2>&1 &
PIDS_TO_KILL+=($!)

sleep 2
echo "  ✓ TF tree ready"

echo ""
echo "Step 2/6: Starting RealSense camera..."
echo "  NOTE: If this fails, your camera might not be detected."
echo "  Run: bash diagnose_camera.sh to check camera connection."
echo ""

# Try to launch camera with better error handling
ros2 launch realsense2_camera rs_launch.py \
    camera_name:=camera \
    camera_namespace:=camera \
    enable_depth:=true \
    enable_color:=true \
    pointcloud.enable:=true \
    align_depth.enable:=true \
    enable_sync:=true \
    depth_module.profile:=640x480x30 \
    rgb_camera.profile:=640x480x30 \
    initial_reset:=true \
    > "$LOG_DIR/camera.log" 2>&1 &

CAM_PID=$!
PIDS_TO_KILL+=($CAM_PID)

# Wait and check if camera is actually working
sleep 8

if ! ps -p $CAM_PID > /dev/null 2>&1; then
    echo "  ✗ ERROR: Camera process died immediately!"
    echo ""
    echo "Last 20 lines of camera log:"
    tail -20 "$LOG_DIR/camera.log"
    echo ""
    echo "TROUBLESHOOTING:"
    echo "  1. Check camera connection: lsusb | grep Intel"
    echo "  2. Run diagnostic: bash diagnose_camera.sh"
    echo "  3. Try USB fix: sudo ./fix_camera_usb.sh"
    echo "  4. Test basic camera: bash test_camera_transforms.sh"
    exit 1
fi

# Check if topics exist
CAMERA_OK=0
for i in {1..10}; do
    if ros2 topic list 2>/dev/null | grep -q "/camera/camera/color/image_raw"; then
        CAMERA_OK=1
        break
    fi
    sleep 1
done

if [ $CAMERA_OK -eq 0 ]; then
    echo "  ✗ ERROR: Camera started but no topics published!"
    echo ""
    echo "Last 20 lines of camera log:"
    tail -20 "$LOG_DIR/camera.log"
    echo ""
    echo "Try: bash test_camera_transforms.sh"
    exit 1
fi

echo "  ✓ Camera running and publishing topics"

echo ""
echo "Step 3/6: Starting RGB-D Odometry..."

ros2 run rtabmap_odom rgbd_odometry \
    --ros-args \
    --params-file "$WORKSPACE/rtabmap_odom_params.yaml" \
    --remap rgb/image:=/camera/camera/color/image_raw \
    --remap rgb/camera_info:=/camera/camera/color/camera_info \
    --remap depth/image:=/camera/camera/aligned_depth_to_color/image_raw \
    > "$LOG_DIR/odometry.log" 2>&1 &

ODOM_PID=$!
PIDS_TO_KILL+=($ODOM_PID)

sleep 3

if ! ps -p $ODOM_PID > /dev/null 2>&1; then
    echo "  ✗ ERROR: Odometry failed!"
    tail -20 "$LOG_DIR/odometry.log"
    exit 1
fi

echo "  ✓ Odometry running"

echo ""
echo "Step 4/6: Starting RTAB-Map SLAM..."

ros2 run rtabmap_slam rtabmap \
    --ros-args \
    --params-file "$WORKSPACE/rtabmap_slam_params.yaml" \
    -r delete_db_on_start:=true \
    --remap rgb/image:=/camera/camera/color/image_raw \
    --remap rgb/camera_info:=/camera/camera/color/camera_info \
    --remap depth/image:=/camera/camera/aligned_depth_to_color/image_raw \
    > "$LOG_DIR/rtabmap.log" 2>&1 &

RTABMAP_PID=$!
PIDS_TO_KILL+=($RTABMAP_PID)

sleep 3

if ! ps -p $RTABMAP_PID > /dev/null 2>&1; then
    echo "  ✗ ERROR: RTAB-Map failed!"
    tail -20 "$LOG_DIR/rtabmap.log"
    exit 1
fi

echo "  ✓ RTAB-Map running"

echo ""
echo "Step 5/6: Starting RViz..."

if [ -f "$RVIZ_CONFIG" ]; then
    ros2 run rviz2 rviz2 -d "$RVIZ_CONFIG" > "$LOG_DIR/rviz.log" 2>&1 &
else
    echo "  ⚠ Using default RViz config (rtabmap_navigation.rviz not found)"
    ros2 run rviz2 rviz2 > "$LOG_DIR/rviz.log" 2>&1 &
fi

RVIZ_PID=$!
PIDS_TO_KILL+=($RVIZ_PID)

sleep 2

echo "  ✓ RViz started"

echo ""
echo "========================================================"
echo "    RTAB-Map SLAM READY!                              "
echo "========================================================"
echo ""
echo "System Status:"
echo "  Camera:    PID $CAM_PID"
echo "  Odometry:  PID $ODOM_PID"
echo "  RTAB-Map:  PID $RTABMAP_PID"
echo "  RViz:      PID $RVIZ_PID"
echo ""
echo "Logs: $LOG_DIR/"
echo ""
echo "IN RVIZ:"
echo "  1. Set Fixed Frame to 'map'"
echo "  2. Add PointCloud2: /rtabmap/cloud_map"
echo "  3. Add Map: /rtabmap/grid_map"
echo "  4. Move camera to build map"
echo ""
echo "Check logs:"
echo "  tail -f $LOG_DIR/rtabmap.log"
echo "  tail -f $LOG_DIR/odometry.log"
echo ""
echo "Press Ctrl+C to stop everything"
echo "========================================================"
echo ""

# Keep script running - wait for Ctrl+C
while true; do
    # Check if critical processes died
    if ! ps -p $CAM_PID > /dev/null 2>&1; then
        echo "⚠ ERROR: Camera died! Check $LOG_DIR/camera.log"
        break
    fi
    if ! ps -p $RTABMAP_PID > /dev/null 2>&1; then
        echo "⚠ ERROR: RTAB-Map died! Check $LOG_DIR/rtabmap.log"
        break
    fi
    sleep 5
done

# If we get here, something died - cleanup
cleanup