#!/bin/bash
echo "Checking TF tree..."
echo ""

source /opt/ros/jazzy/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash 2>/dev/null

echo "=== AVAILABLE FRAMES ==="
timeout 2 ros2 run tf2_ros tf2_echo map odom 2>&1 | head -20

echo ""
echo "=== TF TREE STRUCTURE ==="
timeout 3 ros2 run tf2_tools view_frames 2>&1
if [ -f frames.pdf ]; then
    echo "TF tree saved to frames.pdf"
    echo "Converting to text..."
    timeout 5 ros2 topic echo --once /tf_static 2>/dev/null | head -30
fi

echo ""
echo "=== CHECKING KEY TRANSFORMS ==="
echo "1. Checking map -> odom:"
timeout 2 ros2 run tf2_ros tf2_echo map odom 2>&1 | grep -E "At time|Transform" | head -5

echo ""
echo "2. Checking odom -> base_footprint:"
timeout 2 ros2 run tf2_ros tf2_echo odom base_footprint 2>&1 | grep -E "At time|Transform" | head -5

echo ""
echo "3. Checking base_link -> camera_link:"
timeout 2 ros2 run tf2_ros tf2_echo base_link camera_link 2>&1 | grep -E "At time|Transform" | head -5

echo ""
echo "4. Checking camera_link -> camera_depth_optical_frame:"
timeout 2 ros2 run tf2_ros tf2_echo camera_link camera_depth_optical_frame 2>&1 | grep -E "At time|Transform" | head -5