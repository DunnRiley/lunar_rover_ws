#!/bin/bash
# LIVE Autonomous Navigation
# Build map, navigate, all in one session - no saving needed!

echo "======================================================================"
echo "  LIVE Autonomous Navigation System"
echo "======================================================================"
echo ""
echo "This script:"
echo "  1. Starts RTAB-Map in SLAM mode (builds map live)"
echo "  2. Starts Nav2 (uses live map)"
echo "  3. You rotate camera to see the room"
echo "  4. You set waypoints and navigate - all LIVE!"
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

# Check Nav2
if ! ros2 pkg list | grep -q nav2_bringup; then
    echo "❌ Installing Nav2..."
    sudo apt update
    sudo apt install -y ros-jazzy-nav2-bringup ros-jazzy-navigation2
fi

# Clean up
echo ""
echo "Cleaning old processes..."
killall -9 rtabmap rgbd_odometry realsense_node realsense2_camera_node robot_state_publisher static_transform_publisher 2>/dev/null
sleep 2

# Create Nav2 params (simple, for live use)
cat > /tmp/nav2_params_live.yaml << 'EOF'
controller_server:
  ros__parameters:
    controller_frequency: 10.0
    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      min_vel_x: 0.0
      max_vel_x: 0.3
      max_vel_theta: 0.8
      min_speed_xy: 0.0
      max_speed_xy: 0.3
      acc_lim_x: 1.5
      acc_lim_theta: 2.0
      decel_lim_x: -1.5
      decel_lim_theta: -2.0
      vx_samples: 15
      vtheta_samples: 20
      sim_time: 1.5
      xy_goal_tolerance: 0.2
      critics: ["RotateToGoal", "Oscillation", "BaseObstacle", "GoalAlign", "PathAlign", "PathDist", "GoalDist"]

local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_link
      rolling_window: true
      width: 4
      height: 4
      resolution: 0.05
      robot_radius: 0.25
      plugins: ["obstacle_layer", "inflation_layer"]
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: True
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.5

global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_link
      robot_radius: 0.25
      resolution: 0.05
      plugins: ["static_layer", "inflation_layer"]
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: True
        subscribe_to_updates: true
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.5

planner_server:
  ros__parameters:
    expected_planner_frequency: 10.0
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"
      tolerance: 0.5
      use_astar: false
      allow_unknown: true
EOF

# Create URDF
cat > /tmp/rover.urdf << 'EOF'
<?xml version="1.0"?>
<robot name="lunar_rover">
  <link name="base_footprint"/>
  <link name="base_link"/>
  <joint name="base_footprint_to_base_link" type="fixed">
    <parent link="base_footprint"/>
    <child link="base_link"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
  </joint>
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
    > /tmp/live_rsp.log 2>&1 &

ros2 run tf2_ros static_transform_publisher 0 0 0 -1.5708 0 -1.5708 camera_link camera_depth_optical_frame > /tmp/live_tf1.log 2>&1 &
ros2 run tf2_ros static_transform_publisher 0 0 0 -1.5708 0 -1.5708 camera_link camera_color_optical_frame > /tmp/live_tf2.log 2>&1 &
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
    > /tmp/live_camera.log 2>&1 &

CAM_PID=$!
sleep 8
echo "    ✓ Camera (PID $CAM_PID)"

echo ""
echo "[3/6] Visual odometry..."
ros2 run rtabmap_odom rgbd_odometry \
    --ros-args \
    -p frame_id:=base_link \
    -p odom_frame_id:=odom \
    -p publish_tf:=true \
    -p approx_sync:=true \
    -r rgb/image:=/camera/camera/color/image_raw \
    -r rgb/camera_info:=/camera/camera/color/camera_info \
    -r depth/image:=/camera/camera/aligned_depth_to_color/image_raw \
    > /tmp/live_odom.log 2>&1 &

ODOM_PID=$!
sleep 5
echo "    ✓ Odometry (PID $ODOM_PID)"

echo ""
echo "[4/6] RTAB-Map LIVE SLAM..."
rm -f ~/.ros/rtabmap_live.db

ros2 run rtabmap_slam rtabmap \
    --ros-args \
    -p frame_id:=base_link \
    -p approx_sync:=true \
    -p database_path:=~/.ros/rtabmap_live.db \
    -p Mem/IncrementalMemory:=true \
    -p Grid/FromDepth:=true \
    -p Grid/CellSize:=0.05 \
    -r rgb/image:=/camera/camera/color/image_raw \
    -r rgb/camera_info:=/camera/camera/color/camera_info \
    -r depth/image:=/camera/camera/aligned_depth_to_color/image_raw \
    > /tmp/live_rtabmap.log 2>&1 &

SLAM_PID=$!
sleep 5
echo "    ✓ RTAB-Map SLAM (PID $SLAM_PID)"

echo ""
echo "[5/6] Nav2 navigation..."
ros2 launch nav2_bringup navigation_launch.py \
    params_file:=/tmp/nav2_params_live.yaml \
    use_sim_time:=false \
    > /tmp/live_nav2.log 2>&1 &

NAV_PID=$!
sleep 10
echo "    ✓ Nav2 (PID $NAV_PID)"

echo ""
echo "[6/6] Checking if map is publishing..."
sleep 3
if ros2 topic hz /rtabmap/grid_map --window 3 > /dev/null 2>&1; then
    echo "    ✓ Grid map publishing!"
else
    echo "    ⚠️  Grid map not yet publishing (will start after camera movement)"
fi

echo ""
echo "======================================================================"
echo "  ✓✓✓ SYSTEM READY - LIVE MODE ✓✓✓"
echo "======================================================================"
echo ""
echo "IMPORTANT: The map is building LIVE as you move the camera!"
echo ""
echo "======================================================================"
echo "  STEP 1: BUILD MAP (Do this NOW)"
echo "======================================================================"
echo ""
echo "Open ANOTHER TERMINAL and run:"
echo "  cd ~/lunar_rover_ws"
echo "  python3 teleop_keyboard.py"
echo ""
echo "Then:"
echo "  1. ROTATE CAMERA 360° SLOWLY (T/G keys, 2-3 minutes)"
echo "     - This builds the map of your entire room"
echo "     - The rover sees everything around it"
echo ""
echo "  2. Wait 30 seconds after rotation completes"
echo "     - Let RTAB-Map process the map"
echo ""
echo "  3. Come back to this terminal"
echo ""
echo "======================================================================"
echo "  WAITING FOR YOU TO SCAN THE ROOM..."
echo "======================================================================"
echo ""
echo "Press ENTER when you've rotated 360° and waited 30 seconds..."
read

echo ""
echo "======================================================================"
echo "  STEP 2: AUTONOMOUS NAVIGATION"
echo "======================================================================"
echo ""
echo "The map is ready! Now you can navigate autonomously."
echo ""
echo "Coordinates are in meters from your current position:"
echo "  X: forward(+) / backward(-)"
echo "  Y: left(+) / right(-)"
echo "  Yaw: rotation in radians"
echo ""
echo "Examples:"
echo "  Point 2m forward:  x=2.0, y=0.0, yaw=0.0"
echo "  Point 1m left:     x=0.0, y=1.0, yaw=1.57"
echo "  Point behind you:  x=-1.0, y=0.0, yaw=3.14"
echo ""

# Get waypoints
waypoints=()
while true; do
    echo ""
    echo "--- Waypoint $((${#waypoints[@]} + 1)) ---"
    read -p "X (forward/back in meters): " x
    read -p "Y (left/right in meters): " y
    read -p "Yaw (rotation, 0-6.28): " yaw
    read -p "Description: " desc
    
    waypoints+=("$x,$y,$yaw,$desc")
    echo "✓ Added: $desc at ($x, $y, $yaw)"
    
    read -p "Add another waypoint? (y/n): " more
    if [ "$more" != "y" ]; then
        break
    fi
done

if [ ${#waypoints[@]} -eq 0 ]; then
    echo "No waypoints set!"
    exit 0
fi

echo ""
echo "======================================================================"
echo "  WAYPOINT SUMMARY"
echo "======================================================================"
for i in "${!waypoints[@]}"; do
    IFS=',' read -r x y yaw desc <<< "${waypoints[$i]}"
    echo "$((i+1)). $desc: ($x, $y, $yaw)"
done

echo ""
read -p "Press ENTER to start autonomous navigation..."

echo ""
echo "======================================================================"
echo "  NAVIGATING AUTONOMOUSLY"
echo "======================================================================"

# Navigate to each waypoint using ros2 cli
for i in "${!waypoints[@]}"; do
    IFS=',' read -r x y yaw desc <<< "${waypoints[$i]}"
    
    echo ""
    echo "→ Waypoint $((i+1))/${#waypoints[@]}: $desc"
    echo "  Going to: ($x, $y, $yaw)"
    
    # Calculate quaternion from yaw
    qz=$(echo "scale=6; s($yaw/2)" | bc -l)
    qw=$(echo "scale=6; c($yaw/2)" | bc -l)
    
    # Send goal
    timeout 60 ros2 topic pub --once /goal_pose geometry_msgs/PoseStamped \
        "{header: {frame_id: 'map'}, pose: {position: {x: $x, y: $y, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: $qz, w: $qw}}}" \
        > /dev/null 2>&1
    
    echo "  ✓ Goal sent, robot is navigating..."
    echo "  (Watch the rover move!)"
    
    sleep 15  # Give it time to get close
    
    if [ $((i+1)) -lt ${#waypoints[@]} ]; then
        echo "  Waiting 5 seconds before next waypoint..."
        sleep 5
    fi
done

echo ""
echo "======================================================================"
echo "  ✓ ALL WAYPOINTS SENT!"
echo "======================================================================"
echo ""
echo "The robot should have visited all waypoints."
echo "System is still running - you can send more goals if needed."
echo ""
echo "Press Ctrl+C to stop everything"

# Keep running
trap 'echo ""; echo "Stopping..."; killall -9 rtabmap rgbd_odometry realsense_node realsense2_camera_node robot_state_publisher static_transform_publisher 2>/dev/null; echo "✓ Stopped"; exit' INT TERM

while true; do
    sleep 10
    if ! ps -p $SLAM_PID > /dev/null 2>&1; then
        echo "⚠️  RTAB-Map died!"
        break
    fi
    if ! ps -p $NAV_PID > /dev/null 2>&1; then
        echo "⚠️  Nav2 died!"
        break
    fi
done

echo "System died. Cleaning up..."
killall -9 rtabmap rgbd_odometry realsense_node realsense2_camera_node robot_state_publisher static_transform_publisher 2>/dev/null
echo "✓ Stopped"