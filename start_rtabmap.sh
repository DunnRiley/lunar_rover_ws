#!/bin/bash
# RTAB-Map Launcher - WORKING VERSION
# Minimal parameters to avoid type issues

echo "======================================================================"
echo "  RTAB-Map 3D SLAM - WORKING VERSION"
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
    echo "✓ Workspace"
fi

# Check RTAB-Map
echo ""
if ! ros2 pkg list 2>/dev/null | grep -q rtabmap_ros; then
    echo "❌ Install: sudo apt install ros-jazzy-rtabmap-ros"
    exit 1
fi
echo "✓ RTAB-Map installed"

# Clean up
echo ""
echo "Cleaning old processes..."
killall -9 rtabmap rgbd_odometry realsense_node realsense2_camera_node robot_state_publisher static_transform_publisher rviz2 2>/dev/null
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
echo "    Waiting 8 seconds..."
sleep 8

if ! ps -p $CAM_PID > /dev/null; then
    echo "    ❌ Failed! Check: tail /tmp/rtabmap_camera.log"
    exit 1
fi
echo "    ✓ Running (PID $CAM_PID)"

echo ""
echo "[3/6] Verifying camera topics..."
for i in {1..15}; do
    if ros2 topic list 2>/dev/null | grep -q "/camera/camera/color/image_raw"; then
        echo "    ✓ Publishing!"
        break
    fi
    sleep 1
done

if ! ros2 topic list 2>/dev/null | grep -q "/camera/camera/color/image_raw"; then
    echo "    ❌ No topics!"
    exit 1
fi

echo ""
echo "[4/6] Visual odometry (minimal parameters)..."
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
    tail -30 /tmp/rtabmap_odom.log
    exit 1
fi
echo "    ✓ Running (PID $ODOM_PID)"

echo ""
echo "[5/6] RTAB-Map SLAM (minimal parameters)..."
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
    tail -50 /tmp/rtabmap_slam.log
    exit 1
fi
echo "    ✓ Running (PID $SLAM_PID)"

echo ""
echo "[6/6] RViz..."
ros2 run rviz2 rviz2 > /tmp/rtabmap_rviz.log 2>&1 &
sleep 3
echo "    ✓ Started"

echo ""
echo "======================================================================"
echo "  ✓✓✓ ALL SYSTEMS RUNNING ✓✓✓"
echo "======================================================================"
echo ""
echo "Processes:"
echo "  • Camera:    PID $CAM_PID"
echo "  • Odometry:  PID $ODOM_PID"
echo "  • RTAB-Map:  PID $SLAM_PID"
echo ""
echo "======================================================================"
echo "  RVIZ SETUP (DO THIS NOW):"
echo "======================================================================"
echo ""
echo "1. Fixed Frame → Change to: odom"
echo ""
echo "2. Add → PointCloud2"
echo "   Topic: /camera/camera/depth/color/points"
echo "   Color Transformer: RGB8"
echo "   (LIVE camera - appears instantly)"
echo ""
echo "3. Add → PointCloud2"
echo "   Topic: /rtabmap/cloud_map"
echo "   Color Transformer: RGB8"
echo "   (YOUR 3D MAP - appears after driving)"
echo ""
echo "4. Add → Map"
echo "   Topic: /rtabmap/grid_map"
echo ""
echo "======================================================================"
echo "  THEN START TELEOP:"
echo "======================================================================"
echo ""
echo "Open new terminal:"
echo "  cd ~/lunar_rover_ws"
echo "  python3 teleop_keyboard.py"
echo ""
echo "Drive slowly! Map builds after 20-30 seconds of motion."
echo ""
echo "Logs: /tmp/rtabmap_*.log"
echo "View logs: bash view_logs.sh"
echo ""
echo "Press Ctrl+C to stop"
echo "======================================================================"

# Cleanup trap
trap 'echo ""; echo "Stopping..."; killall -9 rtabmap rgbd_odometry realsense_node realsense2_camera_node robot_state_publisher static_transform_publisher rviz2 2>/dev/null; echo "✓ Stopped"; exit' INT TERM

# Monitor
echo ""
echo "Monitoring (checking every 10 seconds)..."
COUNTER=0
while true; do
    sleep 10
    COUNTER=$((COUNTER + 1))
    
    if ! ps -p $CAM_PID > /dev/null 2>&1; then
        echo ""
        echo "⚠️  Camera died! Check: tail /tmp/rtabmap_camera.log"
        break
    fi
    
    if ! ps -p $ODOM_PID > /dev/null 2>&1; then
        echo ""
        echo "⚠️  Odometry died! Check: tail /tmp/rtabmap_odom.log"
        break
    fi
    
    if ! ps -p $SLAM_PID > /dev/null 2>&1; then
        echo ""
        echo "⚠️  RTAB-Map died! Check: tail /tmp/rtabmap_slam.log"
        break
    fi
    
    # Status every minute
    if [ $((COUNTER % 6)) -eq 0 ]; then
        echo ""
        echo "[$(date +%H:%M:%S)] All systems running ($(( COUNTER / 6 )) min)"
    else
        echo -n "."
    fi
done

echo ""
echo "Process died. Stopping all..."
killall -9 rtabmap rgbd_odometry realsense_node realsense2_camera_node robot_state_publisher static_transform_publisher rviz2 2>/dev/null
echo "✓ Stopped"