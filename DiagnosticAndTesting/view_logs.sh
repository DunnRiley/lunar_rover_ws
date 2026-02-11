#!/bin/bash
# View RTAB-Map logs to diagnose issues

echo "======================================================================"
echo "  RTAB-Map Log Viewer"
echo "======================================================================"

if [ ! -f /tmp/rtabmap_slam.log ]; then
    echo "❌ No logs found!"
    echo "Did you run the launcher yet?"
    echo "Run: bash launch_rtabmap_simple.sh"
    exit 1
fi

echo ""
echo "========================================================================"
echo "  RTAB-Map SLAM Log (MOST IMPORTANT)"
echo "========================================================================"
tail -100 /tmp/rtabmap_slam.log

echo ""
echo ""
echo "========================================================================"
echo "  Odometry Log"
echo "========================================================================"
tail -50 /tmp/rtabmap_odom.log

echo ""
echo ""
echo "========================================================================"
echo "  Camera Log"
echo "========================================================================"
tail -50 /tmp/rtabmap_camera.log

echo ""
echo ""
echo "========================================================================"
echo "  Currently Running Nodes"
echo "========================================================================"
source /opt/ros/jazzy/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash 2>/dev/null
ros2 node list 2>/dev/null || echo "No nodes running"

echo ""
echo ""
echo "========================================================================"
echo "  Available Topics"
echo "========================================================================"
ros2 topic list 2>/dev/null | grep -E "camera|rtabmap|odom" || echo "No topics found"

echo ""
echo ""
echo "========================================================================"
echo "  Full Logs Location"
echo "========================================================================"
echo "SLAM:     /tmp/rtabmap_slam.log"
echo "Odometry: /tmp/rtabmap_odom.log"
echo "Camera:   /tmp/rtabmap_camera.log"
echo "RSP:      /tmp/rtabmap_rsp.log"
echo "TF:       /tmp/tf1.log, /tmp/tf2.log"
echo "RViz:     /tmp/rtabmap_rviz.log"