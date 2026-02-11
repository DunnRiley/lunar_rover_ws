#!/bin/bash
# RTAB-Map WITHOUT RViz (for systems with graphics issues)

echo "======================================================================"
echo "  RTAB-Map 3D SLAM - NO RVIZ VERSION"
echo "======================================================================"

# Source ROS
if [ -f /opt/ros/jazzy/setup.bash ]; then
    . /opt/ros/jazzy/setup.bash
    echo "✓ ROS2 Jazzy"
elif [ -f /opt/ros/humble/setup.bash ]; then
    . /opt/ros/humble/setup.bash
    echo "✓ ROS2 Humble"
else
    echo "❌ No ROS2"
    exit 1
fi

# Source workspace
if [ -f ~/lunar_rover_ws/install/setup.bash ]; then
    . ~/lunar_rover_ws/install/setup.bash
fi

echo ""
if ! ros2 pkg list 2>/dev/null | grep -q rtabmap_ros; then
    echo "❌ Install: sudo apt install ros-jazzy-rtabmap-ros"
    exit 1
fi
echo "✓ RTAB-Map installed"

# Clean up
echo ""
echo "Cleaning..."
killall -9 rtabmap rgbd_odometry realsense_node realsense2_camera_node robot_state_publisher static_transform_publisher 2>/dev/null
sleep 2
echo "✓ Clean"

# URDF
cat > /tmp/rover.urdf << 'EOF'
<?xml version="1.0"?>
<robot name="lunar_rover">
  <link name="base_link"/>
  <link name="camera_link"/>
  <joint name="base_to_camera" type="fixed">
    <parent link="base_link"/>
    <child link="camera_link"/>
    <origin xyz="0.15 0 0.2" rpy="0 0 0"/>
  </joint>
</robot>
EOF

echo ""
echo "======================================================================"
echo "  Starting Components (NO RVIZ)"
echo "======================================================================"

echo ""
echo "[1/5] TF tree..."
ros2 run robot_state_publisher robot_state_publisher \
    --ros-args -p robot_description:="$(cat /tmp/rover.urdf)" \
    > /tmp/rtabmap_rsp.log 2>&1 &

ros2 run tf2_ros static_transform_publisher 0 0 0 -1.5708 0 -1.5708 camera_link camera_depth_optical_frame > /tmp/tf1.log 2>&1 &
ros2 run tf2_ros static_transform_publisher 0 0 0 -1.5708 0 -1.5708 camera_link camera_color_optical_frame > /tmp/tf2.log 2>&1 &
sleep 2
echo "    ✓ Ready"

echo ""
echo "[2/5] Camera..."
ros2 launch realsense2_camera rs_launch.py \
    camera_name:=camera \
    camera_namespace:=camera \
    enable_depth:=true \
    enable_color:=true \
    pointcloud.enable:=true \
    align_depth.enable:=true \
    enable_sync:=true \
    > /tmp/rtabmap_camera.log 2>&1 &

CAM_PID=$!
sleep 8

if ! ps -p $CAM_PID > /dev/null; then
    echo "    ❌ Failed!"
    exit 1
fi
echo "    ✓ Running (PID $CAM_PID)"

echo ""
echo "[3/5] Verifying camera..."
for i in {1..15}; do
    if ros2 topic list 2>/dev/null | grep -q "/camera/camera/color/image_raw"; then
        echo "    ✓ Publishing!"
        break
    fi
    sleep 1
done

echo ""
echo "[4/5] Visual odometry..."
ros2 run rtabmap_odom rgbd_odometry \
    --ros-args \
    -p frame_id:=base_link \
    -p odom_frame_id:=odom \
    -p publish_tf:=true \
    -p approx_sync:=true \
    -r rgb/image:=/camera/camera/color/image_raw \
    -r rgb/camera_info:=/camera/camera/color/camera_info \
    -r depth/image:=/camera/camera/aligned_depth_to_color/image_raw \
    > /tmp/rtabmap_odom.log 2>&1 &

ODOM_PID=$!
sleep 5

if ! ps -p $ODOM_PID > /dev/null; then
    echo "    ❌ Failed!"
    exit 1
fi
echo "    ✓ Running (PID $ODOM_PID)"

echo ""
echo "[5/5] RTAB-Map SLAM..."
rm -f ~/.ros/rtabmap.db

ros2 run rtabmap_slam rtabmap \
    --ros-args \
    -p frame_id:=base_link \
    -p approx_sync:=true \
    -p database_path:=~/.ros/rtabmap.db \
    -r rgb/image:=/camera/camera/color/image_raw \
    -r rgb/camera_info:=/camera/camera/color/camera_info \
    -r depth/image:=/camera/camera/aligned_depth_to_color/image_raw \
    > /tmp/rtabmap_slam.log 2>&1 &

SLAM_PID=$!
sleep 5

if ! ps -p $SLAM_PID > /dev/null; then
    echo "    ❌ Failed!"
    exit 1
fi
echo "    ✓ Running (PID $SLAM_PID)"

echo ""
echo "======================================================================"
echo "  ✓✓✓ RTAB-Map RUNNING (NO RVIZ) ✓✓✓"
echo "======================================================================"
echo ""
echo "Processes:"
echo "  • Camera:    PID $CAM_PID"
echo "  • Odometry:  PID $ODOM_PID"
echo "  • RTAB-Map:  PID $SLAM_PID"
echo ""
echo "======================================================================"
echo "  HOW TO USE WITHOUT RVIZ:"
echo "======================================================================"
echo ""
echo "1. START TELEOP (in another terminal):"
echo "   cd ~/lunar_rover_ws"
echo "   python3 teleop_keyboard.py"
echo ""
echo "2. DRIVE AROUND for 30-60 seconds"
echo ""
echo "3. VIEW MAP STATISTICS:"
echo "   ros2 topic echo /rtabmap/info --once"
echo ""
echo "4. CHECK MAP IS BUILDING:"
echo "   ros2 topic hz /rtabmap/cloud_map"
echo "   (Should show ~1 Hz)"
echo ""
echo "5. SAVE THE MAP when done:"
echo "   Press Ctrl+C to stop this script"
echo "   Map saved to: ~/.ros/rtabmap.db"
echo ""
echo "6. VIEW MAP LATER (3D viewer):"
echo "   rtabmap-databaseViewer ~/.ros/rtabmap.db"
echo ""
echo "======================================================================"
echo "  MONITORING COMMANDS (use in another terminal):"
echo "======================================================================"
echo ""
echo "# See how many nodes in map:"
echo "ros2 topic echo /rtabmap/info --once | grep 'nodes:'"
echo ""
echo "# Check map is publishing:"
echo "ros2 topic hz /rtabmap/cloud_map"
echo ""
echo "# Check odometry quality:"
echo "ros2 topic echo /odom_info --once"
echo ""
echo "# List all RTAB-Map topics:"
echo "ros2 topic list | grep rtabmap"
echo ""
echo "======================================================================"
echo ""
echo "Press Ctrl+C to stop and save map"
echo "======================================================================"

# Trap for cleanup
trap 'echo ""; echo "Stopping and saving map..."; killall -9 rtabmap rgbd_odometry realsense_node realsense2_camera_node robot_state_publisher static_transform_publisher 2>/dev/null; echo "✓ Map saved to ~/.ros/rtabmap.db"; exit' INT TERM

# Monitor with map stats
echo ""
echo "Monitoring (checking every 30 seconds)..."
COUNTER=0
while true; do
    sleep 30
    COUNTER=$((COUNTER + 1))
    
    # Check processes
    if ! ps -p $CAM_PID > /dev/null 2>&1; then
        echo "⚠️  Camera died!"
        break
    fi
    
    if ! ps -p $SLAM_PID > /dev/null 2>&1; then
        echo "⚠️  RTAB-Map died!"
        break
    fi
    
    # Show map stats
    echo ""
    echo "[$(date +%H:%M:%S)] Status Update ($(( COUNTER / 2 )) min):"
    
    # Get node count
    NODES=$(ros2 topic echo /rtabmap/info --once 2>/dev/null | grep -A1 "loop_closure_id:" | tail -1 | awk '{print $2}' || echo "?")
    echo "  • Map nodes: $NODES"
    
    # Check if publishing
    if ros2 topic hz /rtabmap/cloud_map --window 5 > /dev/null 2>&1; then
        echo "  • Map publishing: ✓"
    else
        echo "  • Map publishing: ? (drive around!)"
    fi
    
    echo "  • All systems: RUNNING"
done

echo ""
echo "Process died. Cleaning up..."
killall -9 rtabmap rgbd_odometry realsense_node realsense2_camera_node robot_state_publisher static_transform_publisher 2>/dev/null
echo "✓ Map saved to ~/.ros/rtabmap.db"
echo ""
echo "View map: rtabmap-databaseViewer ~/.ros/rtabmap.db"