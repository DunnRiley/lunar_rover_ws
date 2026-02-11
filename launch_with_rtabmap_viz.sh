#!/bin/bash
# RTAB-Map with rtabmap-viz (real-time 3D viewer)
# This is RTAB-Map's own visualizer - should work even if RViz crashes

echo "======================================================================"
echo "  RTAB-Map with rtabmap-viz (Real-time 3D Viewer)"
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
killall -9 rtabmap rgbd_odometry realsense_node realsense2_camera_node robot_state_publisher static_transform_publisher rtabmap-viz 2>/dev/null
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
echo "  Starting System"
echo "======================================================================"

echo ""
echo "[1/6] TF tree..."
ros2 run robot_state_publisher robot_state_publisher \
    --ros-args -p robot_description:="$(cat /tmp/rover.urdf)" \
    > /tmp/rtabmap_rsp.log 2>&1 &

ros2 run tf2_ros static_transform_publisher 0 0 0 -1.5708 0 -1.5708 camera_link camera_depth_optical_frame > /tmp/tf1.log 2>&1 &
ros2 run tf2_ros static_transform_publisher 0 0 0 -1.5708 0 -1.5708 camera_link camera_color_optical_frame > /tmp/tf2.log 2>&1 &
sleep 2
echo "    ✓ Ready"

echo ""
echo "[2/6] Camera..."
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
echo "[3/6] Verifying camera..."
for i in {1..15}; do
    if ros2 topic list 2>/dev/null | grep -q "/camera/camera/color/image_raw"; then
        echo "    ✓ Publishing!"
        break
    fi
    sleep 1
done

echo ""
echo "[4/6] Visual odometry..."
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
echo "[5/6] RTAB-Map SLAM..."
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
echo "[6/6] Starting rtabmap-viz (3D Viewer)..."
echo "    This will open in a few seconds..."
rtabmap-viz > /tmp/rtabmap_viz.log 2>&1 &
VIZ_PID=$!
sleep 3
echo "    ✓ Started (PID $VIZ_PID)"

echo ""
echo "======================================================================"
echo "  ✓✓✓ SYSTEM RUNNING ✓✓✓"
echo "======================================================================"
echo ""
echo "Processes:"
echo "  • Camera:      PID $CAM_PID"
echo "  • Odometry:    PID $ODOM_PID"
echo "  • RTAB-Map:    PID $SLAM_PID"
echo "  • Visualizer:  PID $VIZ_PID"
echo ""
echo "======================================================================"
echo "  IN THE rtabmap-viz WINDOW:"
echo "======================================================================"
echo ""
echo "You should see a 3D window with menus. To view the map:"
echo ""
echo "1. The map will appear automatically as you drive"
echo "2. Use mouse to rotate/zoom the 3D view:"
echo "   • Left click + drag: Rotate"
echo "   • Right click + drag: Pan"
echo "   • Scroll wheel: Zoom"
echo ""
echo "3. View options (menu bar):"
echo "   • View → Cloud → Show cloud map (persistent 3D map)"
echo "   • View → Grid → Show occupancy grid"
echo "   • View → Graph → Show graph (nodes and connections)"
echo ""
echo "======================================================================"
echo "  NOW START TELEOP:"
echo "======================================================================"
echo ""
echo "Open another terminal:"
echo "  cd ~/lunar_rover_ws"
echo "  python3 teleop_keyboard.py"
echo ""
echo "Drive around and watch the 3D map build in rtabmap-viz!"
echo ""
echo "Press Ctrl+C to stop"
echo "======================================================================"

# Trap for cleanup
trap 'echo ""; echo "Stopping..."; killall -9 rtabmap rgbd_odometry realsense_node realsense2_camera_node robot_state_publisher static_transform_publisher rtabmap-viz 2>/dev/null; echo "✓ Stopped"; echo "Map saved to ~/.ros/rtabmap.db"; exit' INT TERM

# Monitor
echo ""
echo "Monitoring..."
COUNTER=0
while true; do
    sleep 30
    COUNTER=$((COUNTER + 1))
    
    if ! ps -p $CAM_PID > /dev/null 2>&1; then
        echo "⚠️  Camera died!"
        break
    fi
    
    if ! ps -p $SLAM_PID > /dev/null 2>&1; then
        echo "⚠️  RTAB-Map died!"
        break
    fi
    
    if ! ps -p $VIZ_PID > /dev/null 2>&1; then
        echo "⚠️  Visualizer closed!"
        break
    fi
    
    echo "[$(date +%H:%M:%S)] All systems running ($(( COUNTER / 2 )) min)"
done

echo ""
echo "Cleaning up..."
killall -9 rtabmap rgbd_odometry realsense_node realsense2_camera_node robot_state_publisher static_transform_publisher rtabmap-viz 2>/dev/null
echo "✓ Stopped"
echo "Map saved to ~/.ros/rtabmap.db"