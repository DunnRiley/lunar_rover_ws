#!/bin/bash
# ============================================================================
#  VISUALIZER: rtabmap_viz
#  Run in a second terminal after slam_launch.py is running.
# ============================================================================

if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
elif [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
else
    echo "✗ No ROS2 installation found" && exit 1
fi

[ -f ~/lunar_rover_ws/install/setup.bash ] && source ~/lunar_rover_ws/install/setup.bash

export ROS_DOMAIN_ID=42
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
export ROS_LOCALHOST_ONLY=0
FASTDDS_XML=/tmp/fastdds_udp_only.xml
[ -f "$FASTDDS_XML" ] && export FASTRTPS_DEFAULT_PROFILES_FILE="$FASTDDS_XML"

echo "========================================="
echo "  SLAM Visualizer (rtabmap_viz)"
echo "========================================="
echo ""

echo "Waiting for RTAB-Map..."
for i in $(seq 1 30); do
    ros2 topic list 2>/dev/null | grep -q "^/rtabmap" && break
    [ $i -eq 30 ] && echo "  ⚠ RTAB-Map not detected — launching anyway"
    sleep 1
done
echo "  ✓ Detected"
echo ""

if ! ros2 pkg list 2>/dev/null | grep -q "^rtabmap_viz$"; then
    echo "✗ rtabmap_viz not found"
    echo "  Install: sudo apt install ros-jazzy-rtabmap-viz"
    exit 1
fi

echo "Launching rtabmap_viz..."
echo ""
echo "CONTROLS:  left-drag=orbit  scroll=zoom  middle-drag=pan"
echo "           View menu → toggle cloud/graph/camera feed"
echo ""
echo "WHAT TO EXPECT:"
echo "  Colored 3D cloud builds as you move camera over textured surfaces."
echo "  Red/green lines = loop closure links (good — means map is consistent)."
echo "  Camera panel shows feature tracks (dots = tracked points)."
echo ""
echo "LOGS: /tmp/slam_viz.log  |  Ctrl+C to exit"
echo "========================================="

# wait_for_transform=0.5  → extra buffer for TF extrapolation errors
# topic/sync queue 30     → handles rate mismatches between color/depth/odom
ros2 run rtabmap_viz rtabmap_viz \
    --ros-args \
    -p subscribe_depth:=true \
    -p subscribe_rgb:=true \
    -p subscribe_odom:=true \
    -p approx_sync:=true \
    -p wait_for_transform:=0.5 \
    -p topic_queue_size:=30 \
    -p sync_queue_size:=30 \
    -p frame_id:=base_link \
    -p odom_frame_id:=odom \
    --remap rgb/image:=/camera/camera/color/image_raw \
    --remap rgb/camera_info:=/camera/camera/color/camera_info \
    --remap depth/image:=/camera/camera/aligned_depth_to_color/image_raw \
    --remap odom:=/odom \
    2>&1 | tee /tmp/slam_viz.log