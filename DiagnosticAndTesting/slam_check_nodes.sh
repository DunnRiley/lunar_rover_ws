#!/bin/bash
echo "Checking SLAM system nodes..."
echo ""

source /opt/ros/jazzy/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash 2>/dev/null

echo "=== RUNNING NODES ==="
ros2 node list 2>/dev/null

echo ""
echo "=== DEPTHIMAGE_TO_LASERSCAN STATUS ==="
if ros2 node list 2>/dev/null | grep -q depthimage_to_laserscan; then
    echo "✓ depthimage_to_laserscan node is running"
    echo ""
    echo "Node info:"
    ros2 node info /depthimage_to_laserscan 2>/dev/null
else
    echo "✗ depthimage_to_laserscan node NOT running!"
fi

echo ""
echo "=== SLAM_TOOLBOX STATUS ==="
if ros2 node list 2>/dev/null | grep -q slam_toolbox; then
    echo "✓ slam_toolbox node is running"
else
    echo "✗ slam_toolbox node NOT running!"
fi

echo ""
echo "=== CHECKING /scan TOPIC ==="
echo "Subscribers to /scan:"
ros2 topic info /scan 2>/dev/null | grep "Subscription count"

echo ""
echo "Publishers to /scan:"
ros2 topic info /scan 2>/dev/null | grep "Publisher count"

echo ""
echo "Trying to read one message from /scan..."
timeout 3 ros2 topic echo --once /scan 2>&1 | head -20 || echo "No data on /scan topic"