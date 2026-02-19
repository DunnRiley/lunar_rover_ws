#!/bin/bash
# Quick TF Chain Checker

echo "======================================================================"
echo "  TF Chain Diagnostic"
echo "======================================================================"

# Source ROS
if [ -f /opt/ros/jazzy/setup.bash ]; then
    . /opt/ros/jazzy/setup.bash
elif [ -f /opt/ros/humble/setup.bash ]; then
    . /opt/ros/humble/setup.bash
fi

if [ -f ~/lunar_rover_ws/install/setup.bash ]; then
    . ~/lunar_rover_ws/install/setup.bash
fi

echo ""
echo "Available TF Frames:"
echo "----------------------------------------------------------------------"
timeout 3 ros2 run tf2_ros tf2_monitor 2>/dev/null | head -30

echo ""
echo "Checking specific transforms:"
echo "----------------------------------------------------------------------"

echo ""
echo "1. base_link → camera_link:"
if timeout 2 ros2 run tf2_ros tf2_echo base_link camera_link 2>/dev/null | grep -q "Translation"; then
    echo "  ✓ Works"
else
    echo "  ❌ BROKEN - robot_state_publisher issue"
fi

echo ""
echo "2. camera_link → camera_depth_optical_frame:"
if timeout 2 ros2 run tf2_ros tf2_echo camera_link camera_depth_optical_frame 2>/dev/null | grep -q "Translation"; then
    echo "  ✓ Works"
else
    echo "  ❌ BROKEN - static_transform_publisher issue"
fi

echo ""
echo "3. odom → base_link:"
if timeout 2 ros2 run tf2_ros tf2_echo odom base_link 2>/dev/null | grep -q "Translation"; then
    echo "  ✓ Works"
else
    echo "  ❌ BROKEN - rgbd_odometry not publishing TF"
    echo ""
    echo "Checking odometry node:"
    if ros2 node info /rgbd_odometry 2>/dev/null | grep -q "publish_tf"; then
        ros2 node info /rgbd_odometry | grep -A5 "Parameters"
    fi
fi

echo ""
echo "4. Full chain: odom → camera_depth_optical_frame:"
if timeout 2 ros2 run tf2_ros tf2_echo odom camera_depth_optical_frame 2>/dev/null | grep -q "Translation"; then
    echo "  ✓ Works - TF chain is complete!"
else
    echo "  ❌ BROKEN - chain incomplete"
fi

echo ""
echo "======================================================================"
echo "  TF Tree Visualization"
echo "======================================================================"
echo ""
echo "Creating PDF of TF tree..."
timeout 5 ros2 run tf2_tools view_frames 2>/dev/null

if [ -f frames.pdf ]; then
    echo "✓ TF tree saved to: frames.pdf"
    echo "  Open with: xdg-open frames.pdf"
    
    # Also create text version
    if [ -f frames.gv ]; then
        echo ""
        echo "TF Tree (text):"
        cat frames.gv
    fi
else
    echo "⚠️  Could not generate TF tree"
fi

echo ""
echo "======================================================================"
echo "  Quick Diagnosis"
echo "======================================================================"
echo ""

# Count active static transforms
STATIC_COUNT=$(ros2 node list 2>/dev/null | grep -c static_transform_publisher)
echo "Static transform publishers running: $STATIC_COUNT (should be 2)"

# Check if odometry is publishing TF
if ros2 topic echo /tf --once 2>/dev/null | grep -q "odom"; then
    echo "✓ Odometry is publishing to /tf"
else
    echo "❌ Odometry NOT publishing to /tf"
    echo ""
    echo "Check odometry log:"
    echo "  tail /tmp/slam_odom.log"
fi

echo ""
echo "======================================================================"