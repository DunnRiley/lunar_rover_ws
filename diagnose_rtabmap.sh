#!/bin/bash
# RTAB-Map Diagnostics - Check what's working

echo "======================================================================"
echo "  RTAB-Map System Diagnostics"
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
echo "1. Running Nodes:"
echo "----------------------------------------------------------------------"
ros2 node list 2>/dev/null || echo "  ❌ No nodes running!"

echo ""
echo "2. Camera Topics:"
echo "----------------------------------------------------------------------"
if ros2 topic list 2>/dev/null | grep -q "/camera/camera/color/image_raw"; then
    echo "  ✓ RGB image: /camera/camera/color/image_raw"
    RATE=$(timeout 3 ros2 topic hz /camera/camera/color/image_raw --window 5 2>&1 | grep 'average rate' || echo "    (rate check timed out)")
    echo "    $RATE"
else
    echo "  ❌ RGB image not publishing"
fi

if ros2 topic list 2>/dev/null | grep -q "/camera/camera/aligned_depth_to_color/image_raw"; then
    echo "  ✓ Depth image: /camera/camera/aligned_depth_to_color/image_raw"
else
    echo "  ❌ Depth image not publishing"
fi

if ros2 topic list 2>/dev/null | grep -q "/camera/camera/depth/color/points"; then
    echo "  ✓ Live point cloud: /camera/camera/depth/color/points"
    RATE=$(timeout 3 ros2 topic hz /camera/camera/depth/color/points --window 5 2>&1 | grep 'average rate' || echo "    (rate check timed out)")
    echo "    $RATE"
else
    echo "  ❌ Live point cloud not publishing"
fi

echo ""
echo "3. Odometry:"
echo "----------------------------------------------------------------------"
if ros2 topic list 2>/dev/null | grep -q "/odom"; then
    echo "  ✓ Odometry: /odom"
    RATE=$(timeout 3 ros2 topic hz /odom --window 5 2>&1 | grep 'average rate' || echo "    (rate check timed out)")
    echo "    $RATE"
    
    # Check if publishing TF
    echo ""
    echo "  Checking if odometry publishes TF..."
    if timeout 2 ros2 topic echo /tf --once 2>/dev/null | grep -q "child_frame_id: \"base_link\""; then
        echo "  ✓ Odometry IS publishing TF (odom → base_link)"
    else
        echo "  ❌ Odometry NOT publishing TF!"
        echo "  This is why RViz can't transform point cloud!"
        echo ""
        echo "  Odometry node parameters:"
        ros2 param list /rgbd_odometry 2>/dev/null | grep publish_tf
        ros2 param get /rgbd_odometry publish_tf 2>/dev/null
    fi
else
    echo "  ❌ Odometry not publishing"
fi

echo ""
echo "4. RTAB-Map Topics:"
echo "----------------------------------------------------------------------"
if ros2 topic list 2>/dev/null | grep -q "/rtabmap/cloud_map"; then
    echo "  ✓ RTAB-Map cloud: /rtabmap/cloud_map"
    echo "    Rate: $(timeout 5 ros2 topic hz /rtabmap/cloud_map --window 5 2>&1 | grep 'average rate' || echo 'waiting for data...')"
else
    echo "  ❌ RTAB-Map cloud not publishing"
fi

if ros2 topic list 2>/dev/null | grep -q "/rtabmap/info"; then
    echo "  ✓ RTAB-Map info: /rtabmap/info"
else
    echo "  ❌ RTAB-Map info not publishing"
fi

echo ""
echo "5. TF Frames:"
echo "----------------------------------------------------------------------"
if ros2 run tf2_ros tf2_echo base_link camera_link 2>/dev/null | head -5 | grep -q "Translation"; then
    echo "  ✓ TF tree is working"
else
    echo "  ⚠️  TF tree may have issues"
fi

echo ""
echo "Available frames:"
ros2 run tf2_ros tf2_monitor 2>/dev/null | head -20

echo ""
echo "6. RTAB-Map Status:"
echo "----------------------------------------------------------------------"
if [ -f /tmp/slam_rtabmap.log ]; then
    echo "Last 20 lines of RTAB-Map log:"
    tail -20 /tmp/slam_rtabmap.log
else
    echo "  ❌ No log file found at /tmp/slam_rtabmap.log"
fi

echo ""
echo "7. Point Cloud Data Check:"
echo "----------------------------------------------------------------------"
echo "Checking if point cloud has data..."

timeout 5 ros2 topic echo /camera/camera/depth/color/points --once 2>/dev/null | head -20 > /tmp/pc_check.txt

if [ -s /tmp/pc_check.txt ]; then
    echo "  ✓ Point cloud data is being published"
    echo "  Sample:"
    head -10 /tmp/pc_check.txt
else
    echo "  ❌ No point cloud data received (camera may not be streaming)"
fi

echo ""
echo "8. Quick Fixes:"
echo "----------------------------------------------------------------------"
echo "If camera not publishing:"
echo "  • Check USB connection"
echo "  • Run: rs-enumerate-devices"
echo "  • Try: sudo bash DiagnosticAndTesting/fix_camera_usb.sh"
echo ""
echo "If RTAB-Map topics not publishing:"
echo "  • Move/rotate the camera"
echo "  • RTAB-Map needs motion to generate map"
echo ""
echo "If 'map' frame doesn't exist:"
echo "  • Normal! RTAB-Map creates 'map' after processing data"
echo "  • Use 'odom' frame in RViz initially"
echo "  • Switch to 'map' after rotating camera 360°"
echo ""
echo "======================================================================"