#!/bin/bash
# ========================================================================
# Laptop ROS2 Launch Script
# Place at: ~/lunar_rover_ws/laptop_ros_launch.sh
# Runs: RViz for visualization
# ========================================================================

echo "========================================="
echo "  LAPTOP: Starting Visualization"
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

# Source workspace (optional)
if [ -f install/setup.bash ]; then
    source install/setup.bash
    echo "✓ Workspace sourced"
fi

# CRITICAL: Set same network parameters as mini PC
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET

echo "✓ Network: ROS_DOMAIN_ID=42, SUBNET discovery"
echo ""

# Wait for miniPC nodes
echo "Waiting 3 seconds for mini PC nodes to start..."
sleep 3

# Check if we can see topics from miniPC
echo ""
echo "Checking connection to mini PC..."
TOPICS=$(ros2 topic list 2>/dev/null | grep -E "/camera|/tf" | wc -l)

if [ $TOPICS -gt 0 ]; then
    echo "✓ Connected! Found $TOPICS camera/tf topics from mini PC"
else
    echo "✗ Cannot see topics from mini PC!"
    echo ""
    echo "Troubleshooting:"
    echo "  1. Is mini PC running? ssh moonpie@138.67.181.222"
    echo "  2. Run on mini PC: bash ~/lunar_rover_ws/mini_pc_launch.sh"
    echo "  3. Check network: ping 138.67.181.222"
    echo "  4. Check ROS_DOMAIN_ID matches (should be 42 on both)"
    echo ""
    echo "Available topics:"
    ros2 topic list
    echo ""
fi

# ========================================================================
# Launch RViz with proper config
# ========================================================================

echo ""
echo "Starting RViz..."

# Check for existing RViz config
if [ -f ~/lunar_rover_ws/hardware_navigation.rviz ]; then
    RVIZ_CONFIG=~/lunar_rover_ws/hardware_navigation.rviz
    echo "  Using config: hardware_navigation.rviz"
elif [ -f ~/lunar_rover_ws/realsense_working.rviz ]; then
    RVIZ_CONFIG=~/lunar_rover_ws/realsense_working.rviz
    echo "  Using config: realsense_working.rviz"
else
    echo "  Using default RViz config"
    RVIZ_CONFIG=""
fi

if [ -n "$RVIZ_CONFIG" ]; then
    ros2 run rviz2 rviz2 -d "$RVIZ_CONFIG" \
      --ros-args -p use_sim_time:=false &
else
    ros2 run rviz2 rviz2 \
      --ros-args -p use_sim_time:=false &
fi

RVIZ_PID=$!
sleep 3

if ps -p $RVIZ_PID > /dev/null 2>&1; then
    echo "  ✓ RViz running (PID $RVIZ_PID)"
else
    echo "  ✗ RViz failed to start"
fi

echo ""
echo "========================================="
echo "  ✓✓✓ LAPTOP VISUALIZATION READY ✓✓✓"
echo "========================================="
echo ""
echo "IN RVIZ:"
echo "  1. Fixed Frame: base_link"
echo "  2. Add displays for:"
echo "     - Image: /camera/camera/color/image_raw (D435 RGB)"
echo "     - Image: /camera/camera/aligned_depth_to_color/image_raw (D435 Depth)"
echo "     - PointCloud2: /camera/camera/depth/color/points (D435 3D)"
echo "     - Image: /camera_rear/left/image_raw (IFWATER Left)"
echo "     - Image: /camera_rear/right/image_raw (IFWATER Right)"
echo ""
echo "To start GUI controls, run in another terminal:"
echo "  python3 ~/lunar_rover_ws/laptop_control_gui.py"
echo ""
echo "Press Ctrl+C to stop"
echo "========================================="

wait