#!/bin/bash
# Simple SLAM for Visual Waypoint Selection
# No Nav2, no autonomous driving - just SLAM + waypoint clicking

echo "======================================================================"
echo "  Visual Waypoint Selection System"
echo "======================================================================"
echo ""
echo "This system:"
echo "  1. Runs RTAB-Map to build a 3D point cloud map"
echo "  2. Lets you click on the point cloud to select waypoints"
echo "  3. Shows distance to target using visual odometry"
echo "  4. NO autonomous driving (you drive manually)"
echo ""
read -p "Press Enter to start..."

# Source ROS
if [ -f /opt/ros/jazzy/setup.bash ]; then
    . /opt/ros/jazzy/setup.bash
elif [ -f /opt/ros/humble/setup.bash ]; then
    . /opt/ros/humble/setup.bash
else
    echo "❌ No ROS2"
    exit 1
fi

# Source workspace
if [ -f ~/lunar_rover_ws/install/setup.bash ]; then
    . ~/lunar_rover_ws/install/setup.bash
fi

# Clean up
echo ""
echo "Cleaning..."
killall -9 rtabmap rgbd_odometry realsense_node realsense2_camera_node robot_state_publisher static_transform_publisher rviz2 2>/dev/null
sleep 2

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

# Create RViz config for waypoint selection
cat > /tmp/waypoint_rviz.rviz << 'EOF'
Panels:
  - Class: rviz_common/Displays
    Name: Displays
  - Class: rviz_common/Tool Properties
    Name: Tool Properties
Visualization Manager:
  Class: ""
  Displays:
    - Class: rviz_default_plugins/Grid
      Name: Grid
      Reference Frame: <Fixed Frame>
      Plane Cell Count: 20
      Color: 160; 160; 164
    - Class: rviz_default_plugins/TF
      Name: TF
      Enabled: true
      Frames:
        All Enabled: true
      Show Names: true
    - Class: rviz_default_plugins/PointCloud2
      Name: Live Camera Cloud
      Enabled: true
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Best Effort
        Value: /camera/camera/depth/color/points
      Use Fixed Frame: true
      Color Transformer: RGB8
      Size (m): 0.01
      Size (Pixels): 3
      Style: Points
    - Class: rviz_default_plugins/PointCloud2
      Name: RTAB-Map Cloud
      Enabled: true
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /rtabmap/cloud_map
      Use Fixed Frame: true
      Color Transformer: RGB8
      Size (m): 0.02
      Size (Pixels): 3
      Style: Points
    - Class: rviz_default_plugins/MarkerArray
      Name: Waypoint Markers
      Enabled: true
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /waypoint_markers
    - Class: rviz_default_plugins/Marker
      Name: Target Marker
      Enabled: true
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /target_marker
    - Class: rviz_default_plugins/Path
      Name: Odometry Path
      Enabled: true
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /rtabmap/odom_path
      Color: 255; 170; 0
      Line Style: Lines
      Line Width: 0.03
  Global Options:
    Background Color: 48; 48; 48
    Fixed Frame: odom
    Frame Rate: 30
  Tools:
    - Class: rviz_default_plugins/PublishPoint
      Single click: true
      Topic: /clicked_point
  Value: true
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: 5.0
      Enable Stereo Rendering:
        Stereo Eye Separation: 0.06
        Stereo Focal Distance: 1.0
        Swap Stereo Eyes: false
        Value: false
      Focal Point:
        X: 0
        Y: 0
        Z: 0
      Focal Shape Fixed Size: true
      Focal Shape Size: 0.05
      Invert Z Axis: false
      Name: Current View
      Near Clip Distance: 0.01
      Pitch: 0.5
      Target Frame: <Fixed Frame>
      Value: Orbit (rviz_default_plugins)
      Yaw: 0.0
Window Geometry:
  Height: 800
  Width: 1200
EOF

echo ""
echo "======================================================================"
echo "  Starting SLAM System"
echo "======================================================================"

echo ""
echo "[1/5] TF tree..."
ros2 run robot_state_publisher robot_state_publisher \
    --ros-args -p robot_description:="$(cat /tmp/rover.urdf)" \
    > /tmp/slam_rsp.log 2>&1 &

ros2 run tf2_ros static_transform_publisher 0 0 0 -1.5708 0 -1.5708 camera_link camera_depth_optical_frame > /tmp/slam_tf1.log 2>&1 &
ros2 run tf2_ros static_transform_publisher 0 0 0 -1.5708 0 -1.5708 camera_link camera_color_optical_frame > /tmp/slam_tf2.log 2>&1 &
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
    > /tmp/slam_camera.log 2>&1 &

CAM_PID=$!
sleep 8
echo "    ✓ Camera running (PID $CAM_PID)"

echo ""
echo "[3/5] Visual odometry..."
ros2 run rtabmap_odom rgbd_odometry \
    --ros-args \
    -p frame_id:=base_link \
    -p odom_frame_id:=odom \
    -p publish_tf:=true \
    -p approx_sync:=true \
    -r rgb/image:=/camera/camera/color/image_raw \
    -r rgb/camera_info:=/camera/camera/color/camera_info \
    -r depth/image:=/camera/camera/aligned_depth_to_color/image_raw \
    > /tmp/slam_odom.log 2>&1 &

ODOM_PID=$!
sleep 5
echo "    ✓ Odometry running (PID $ODOM_PID)"

echo ""
echo "[4/5] RTAB-Map SLAM..."
rm -f ~/.ros/rtabmap_waypoint.db

ros2 run rtabmap_slam rtabmap \
    --ros-args \
    -p frame_id:=base_link \
    -p approx_sync:=true \
    -p database_path:=~/.ros/rtabmap_waypoint.db \
    -r rgb/image:=/camera/camera/color/image_raw \
    -r rgb/camera_info:=/camera/camera/color/camera_info \
    -r depth/image:=/camera/camera/aligned_depth_to_color/image_raw \
    > /tmp/slam_rtabmap.log 2>&1 &

SLAM_PID=$!
sleep 5

if ! ps -p $SLAM_PID > /dev/null 2>&1; then
    echo "    ❌ RTAB-Map failed to start!"
    echo ""
    echo "Last 30 lines of log:"
    tail -30 /tmp/slam_rtabmap.log
    echo ""
    echo "Cleaning up..."
    killall -9 rtabmap rgbd_odometry realsense_node realsense2_camera_node robot_state_publisher static_transform_publisher rviz2 2>/dev/null
    exit 1
fi

echo "    ✓ RTAB-Map running (PID $SLAM_PID)"

# Verify topics are publishing
echo ""
echo "Verifying RTAB-Map topics..."
sleep 3

if ros2 topic list 2>/dev/null | grep -q "/rtabmap/cloud_map"; then
    echo "    ✓ RTAB-Map topics active"
else
    echo "    ⚠️  RTAB-Map topics not yet visible (will appear after camera movement)"
fi

echo ""
echo "[5/5] Starting RViz with waypoint config..."
echo "    (May take 10-15 seconds to open...)"
rviz2 -d /tmp/waypoint_rviz.rviz > /tmp/slam_rviz.log 2>&1 &
RVIZ_PID=$!
sleep 5

if ps -p $RVIZ_PID > /dev/null 2>&1; then
    echo "    ✓ RViz running (PID $RVIZ_PID)"
else
    echo "    ⚠️  RViz may have crashed (graphics driver issue)"
    echo "    The system will still work, just no visualization"
fi

echo ""
echo "======================================================================"
echo "  ✓✓✓ SYSTEM READY ✓✓✓"
echo "======================================================================"
echo ""
echo "Running:"
echo "  • Camera:    PID $CAM_PID"
echo "  • Odometry:  PID $ODOM_PID"
echo "  • RTAB-Map:  PID $SLAM_PID"
if ps -p $RVIZ_PID > /dev/null 2>&1; then
    echo "  • RViz:      PID $RVIZ_PID"
fi
echo ""
echo "======================================================================"
echo "  STEP 1: BUILD THE MAP"
echo "======================================================================"
echo ""
echo "Open ANOTHER TERMINAL and run teleop:"
echo "  cd ~/lunar_rover_ws"
echo "  python3 teleop_keyboard.py"
echo ""
echo "Then ROTATE CAMERA 360° (T/G keys, 2-3 minutes)"
echo "This builds the 3D point cloud of your room."
echo ""
echo "IN RVIZ you should see:"
echo "  • Live camera point cloud (colorful, moves with camera)"
echo ""
echo "IMPORTANT - IN RVIZ:"
echo "  1. Fixed Frame is currently 'odom'"
echo "  2. After rotating camera, RTAB-Map creates 'map' frame"
echo "  3. Change Fixed Frame to 'map' to see persistent cloud!"
echo "  4. You'll then see both:"
echo "     - Live cloud (moves)"
echo "     - RTAB-Map cloud (stays, accumulates)"
echo ""
echo "Press ENTER when you've completed the 360° scan..."
read

echo ""
echo "======================================================================"
echo "  STEP 2: START WAYPOINT SELECTOR"
echo "======================================================================"
echo ""
echo "Now starting the waypoint selection tool..."
echo ""

if ps -p $RVIZ_PID > /dev/null 2>&1; then
    echo "IN RVIZ:"
    echo "  1. Select the 'Publish Point' tool (top toolbar)"
    echo "  2. Click anywhere on the point cloud"
    echo "  3. The waypoint selector will ask what to do"
    echo ""
    echo "IN THE TERMINAL:"
    echo "  • Choose to add waypoint or set as target"
    echo "  • Distance to target shows in real-time"
    echo "  • Drive manually with teleop to reach targets"
    echo ""
else
    echo "⚠️  RViz not running - cannot select points visually"
    echo "You can still manually enter waypoint coordinates"
fi

echo ""
echo "======================================================================"
echo "  NOW RUN THE WAYPOINT SELECTOR"
echo "======================================================================"
echo ""
echo "In a THIRD TERMINAL, run:"
echo "  cd ~/lunar_rover_ws"
echo "  python3 waypoint_selector.py"
echo ""
echo "That terminal will show distances and let you select waypoints."
echo ""
echo "======================================================================"
echo "  System Running"
echo "======================================================================"
echo ""
echo "Press Ctrl+C to stop everything"

# Monitor
trap 'echo ""; echo "Stopping..."; killall -9 rtabmap rgbd_odometry realsense_node realsense2_camera_node robot_state_publisher static_transform_publisher rviz2 2>/dev/null; echo "✓ Stopped"; exit' INT TERM

while true; do
    sleep 10
    if ! ps -p $SLAM_PID > /dev/null 2>&1; then
        echo ""
        echo "⚠️  RTAB-Map died! Check logs:"
        echo "  tail /tmp/slam_rtabmap.log"
        break
    fi
    if ! ps -p $CAM_PID > /dev/null 2>&1; then
        echo ""
        echo "⚠️  Camera died!"
        break
    fi
done

echo ""
echo "Cleaning up..."
killall -9 rtabmap rgbd_odometry realsense_node realsense2_camera_node robot_state_publisher static_transform_publisher rviz2 2>/dev/null
echo "✓ Stopped"