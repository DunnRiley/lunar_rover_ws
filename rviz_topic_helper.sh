#!/bin/bash
# ========================================================================
# RViz Topic Helper - Shows you exactly which topics to use in RViz
# ========================================================================

echo "========================================="
echo "  RViz Image Topic Helper"
echo "========================================="
echo ""

# Source ROS2
if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
elif [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
fi

export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0

echo "Checking available camera topics..."
echo ""

# Check for buffered topics
BUFFERED=$(ros2 topic list 2>/dev/null | grep "buffered")

if [ -n "$BUFFERED" ]; then
    echo "✓ BUFFERED TOPICS FOUND - Use these for smooth playback:"
    echo ""
    echo "📺 IN RVIZ:"
    echo ""
    echo "1. Click 'Add' button (bottom left)"
    echo ""
    echo "2. Add Front Camera:"
    echo "   Type: Image"
    echo "   Topic: /camera/camera/color/buffered/compressed"
    echo "   Transport: compressed"
    echo ""
    echo "3. Add Rear Stereo:"
    echo "   Type: Image"  
    echo "   Topic: /camera_rear/stereo_buffered/compressed"
    echo "   Transport: compressed"
    echo ""
    echo "4. (Optional) Add Depth:"
    echo "   Type: Image"
    echo "   Topic: /camera/camera/aligned_depth_to_color/image_raw/compressedDepth"
    echo "   Transport: compressedDepth"
    echo ""
    echo "💡 NOTE: Wait 5 seconds after adding displays for buffer to fill"
    echo "   Then you'll get smooth 6 FPS playback!"
    echo ""
    
    # Show buffered topics
    echo "Available buffered topics:"
    ros2 topic list | grep buffered
    
else
    echo "⚠ NO BUFFERED TOPICS - Using regular compressed:"
    echo ""
    echo "IN RVIZ:"
    echo ""
    echo "1. Click 'Add' button (bottom left)"
    echo ""
    echo "2. Add Front Camera:"
    echo "   Type: Image"
    echo "   Topic: /camera/camera/color/image_raw/compressed"
    echo "   Transport: compressed"
    echo ""
    echo "3. Add Rear Stereo Combined:"
    echo "   Type: Image"
    echo "   Topic: /camera_rear/stereo_combined/compressed"
    echo "   Transport: compressed"
    echo ""
    echo "4. (Optional) Add Depth:"
    echo "   Type: Image"
    echo "   Topic: /camera/camera/aligned_depth_to_color/image_raw/compressedDepth"
    echo "   Transport: compressedDepth"
    echo ""
    
    # Show available topics
    echo "Available camera topics:"
    ros2 topic list | grep -E "camera.*compressed|stereo"
fi

echo ""
echo "========================================="
echo "IMPORTANT:"
echo "========================================="
echo ""
echo "1. Make sure 'Transport' is set to 'compressed' or 'compressedDepth'"
echo "2. Do NOT use raw image topics - they are too slow over WiFi"
echo "3. Fixed Frame should be: base_link"
echo ""
echo "To check topic rates:"
echo "  ros2 topic hz /camera/camera/color/buffered/compressed"
echo ""
echo "To check topic bandwidth:"
echo "  ros2 topic bw /camera/camera/color/buffered/compressed"
echo ""
