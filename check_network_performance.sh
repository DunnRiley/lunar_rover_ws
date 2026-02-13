#!/bin/bash
# ========================================================================
# ROS2 Network Performance Checker
# Run on laptop to diagnose slow image streaming
# ========================================================================

echo "========================================="
echo "  ROS2 Network Performance Checker"
echo "========================================="
echo ""

# Check if ROS2 is sourced
if ! command -v ros2 &> /dev/null; then
    echo "✗ ROS2 not found. Source it first:"
    echo "  source /opt/ros/jazzy/setup.bash"
    exit 1
fi

echo "Checking ROS2 topics..."
echo ""

# List all camera topics
echo "Available camera topics:"
ros2 topic list | grep camera | head -20
echo ""

# Check topic frequencies
echo "Checking topic rates (press Ctrl+C after 5 seconds)..."
echo ""

echo "=== RAW IMAGE (expect <1 Hz over WiFi) ==="
timeout 5 ros2 topic hz /camera/camera/color/image_raw 2>/dev/null || echo "  Not available or too slow to measure"
echo ""

echo "=== COMPRESSED IMAGE (expect 10-30 Hz over WiFi) ==="
timeout 5 ros2 topic hz /camera/camera/color/image_raw/compressed 2>/dev/null || echo "  Not available - camera may not be publishing compressed"
echo ""

echo "=== POINT CLOUD (expect <1 Hz over WiFi) ==="
timeout 5 ros2 topic hz /camera/camera/depth/color/points 2>/dev/null || echo "  Not available (disabled by default for bandwidth)"
echo ""

# Check topic bandwidth
echo "========================================="
echo "Topic Bandwidth Analysis:"
echo "========================================="
echo ""

# Check if we can measure bandwidth
if command -v ros2 topic bw &> /dev/null; then
    echo "Measuring bandwidth for 10 seconds..."
    echo ""
    
    echo "RAW COLOR IMAGE:"
    timeout 10 ros2 topic bw /camera/camera/color/image_raw 2>/dev/null || echo "  Not available"
    echo ""
    
    echo "COMPRESSED COLOR IMAGE:"
    timeout 10 ros2 topic bw /camera/camera/color/image_raw/compressed 2>/dev/null || echo "  Not available"
    echo ""
else
    echo "⚠ 'ros2 topic bw' not available"
fi

# Network recommendations
echo ""
echo "========================================="
echo "RECOMMENDATIONS:"
echo "========================================="
echo ""

COMPRESSED_RATE=$(timeout 3 ros2 topic hz /camera/camera/color/image_raw/compressed 2>/dev/null | grep "average rate" | awk '{print $3}')
RAW_RATE=$(timeout 3 ros2 topic hz /camera/camera/color/image_raw 2>/dev/null | grep "average rate" | awk '{print $3}')

if [ -z "$COMPRESSED_RATE" ]; then
    echo "✗ No compressed topics found"
    echo "  → In RViz, use: /camera/camera/color/image_raw/compressed"
    echo "  → Make sure Transport = 'compressed' in Image display settings"
elif (( $(echo "$COMPRESSED_RATE < 5" | bc -l 2>/dev/null || echo 0) )); then
    echo "✗ Compressed topics are slow ($COMPRESSED_RATE Hz)"
    echo "  → Check WiFi signal strength"
    echo "  → Move closer to router or use ethernet"
    echo "  → Reduce resolution in mini_pc_launch.sh"
else
    echo "✓ Compressed topics working well ($COMPRESSED_RATE Hz)"
    echo "  → Continue using compressed transport in RViz"
fi

if [ -n "$RAW_RATE" ] && (( $(echo "$RAW_RATE > 1" | bc -l 2>/dev/null || echo 0) )); then
    echo "⚠ You're subscribed to RAW images ($RAW_RATE Hz)"
    echo "  → Switch to compressed topics for better performance"
fi

echo ""
echo "========================================="
echo "Quick Fix:"
echo "========================================="
echo "1. In RViz, DELETE any raw image displays"
echo "2. Add → Image"
echo "3. Topic: /camera/camera/color/image_raw/compressed"
echo "4. Transport: compressed"
echo ""
echo "This should give you 15-30 FPS instead of <1 FPS"
echo "========================================="