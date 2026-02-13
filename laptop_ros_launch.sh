#!/bin/bash
# ========================================================================
# Laptop ROS2 Launch Script
# Place at: ~/lunar_rover_ws/laptop_ros_launch.sh
# Runs: RViz, Teleop, and Navigation Stack
# ========================================================================

echo "========================================="
echo "  LAPTOP: Visualization & Control"
echo "========================================="

cd ~/lunar_rover_ws

# Source ROS2
if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
    echo "✓ ROS2 Jazzy"
elif [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
    echo "✓ ROS2 Humble"
else
    echo "✗ No ROS2 installation found!"
    exit 1
fi

# Source workspace
if [ -f install/setup.bash ]; then
    source install/setup.bash
    echo "✓ Workspace sourced"
fi

# CRITICAL: Set same network parameters as mini PC
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET

echo "✓ Network: ROS_DOMAIN_ID=42, SUBNET discovery"
echo ""

# Wait for miniPC nodes
echo "Waiting 3 seconds for mini PC nodes to start..."
sleep 3

# Check if we can see topics from miniPC
echo ""
echo "Checking connection to mini PC..."
TOPICS=$(ros2 topic list 2>/dev/null | grep -E "/camera|/tf" | wc -l)

if [ $TOPICS -gt 0 ]; then
    echo "✓ Connected! Found $TOPICS camera/tf topics from mini PC"
else
    echo "⚠ Warning: Cannot see topics from mini PC!"
    echo ""
    echo "  Troubleshooting:"
    echo "  1. Is mini PC running? ssh moonpie@138.67.181.222"
    echo "  2. Run on mini PC: bash ~/lunar_rover_ws/mini_pc_launch.sh"
    echo "  3. Check network: ping 138.67.181.222"
    echo "  4. Verify ROS_DOMAIN_ID=42 on both machines"
    echo ""
    echo "  Continuing anyway - you can test locally..."
    echo ""
fi

# Trap to cleanup on exit
trap 'echo ""; echo "Shutting down..."; kill 0' SIGINT SIGTERM

# ========================================================================
# Launch RViz
# ========================================================================

echo ""
echo "Detecting available camera topics..."

# Check if buffered topics are available
BUFFERED_TOPICS=$(ros2 topic list 2>/dev/null | grep -c "buffered" || echo "0")

if [ "$BUFFERED_TOPICS" -gt 0 ]; then
    echo "  ✓ Found buffered topics - using buffered config"
    USE_BUFFERED=true
    
    # Create buffered RViz config if it doesn't exist
    if [ ! -f ~/lunar_rover_ws/rover_buffered.rviz ]; then
        echo "  Creating rover_buffered.rviz config..."
        cat > ~/lunar_rover_ws/rover_buffered_temp.rviz << 'EOF'
Panels:
  - Class: rviz_common/Displays
    Name: Displays
Visualization Manager:
  Displays:
    - Class: rviz_default_plugins/Grid
      Name: Grid
      Value: true
    - Class: rviz_default_plugins/TF
      Name: TF
      Value: true
    - Class: rviz_default_plugins/Image
      Name: Front Camera (Buffered)
      Topic:
        Value: /camera/camera/color/buffered/compressed
      Value: true
    - Class: rviz_default_plugins/Image
      Name: Rear Stereo Combined
      Topic:
        Value: /camera_rear/stereo_buffered/compressed
      Value: true
  Global Options:
    Fixed Frame: base_link
EOF
        RVIZ_CONFIG=~/lunar_rover_ws/rover_buffered_temp.rviz
    else
        RVIZ_CONFIG=~/lunar_rover_ws/rover_buffered.rviz
    fi
else
    echo "  ⚠ No buffered topics found - using compressed topics"
    USE_BUFFERED=false
    
    # Check for existing configs
    if [ -f ~/lunar_rover_ws/hardware_navigation.rviz ]; then
        RVIZ_CONFIG=~/lunar_rover_ws/hardware_navigation.rviz
        echo "  Using config: hardware_navigation.rviz"
    elif [ -f ~/lunar_rover_ws/realsense_working.rviz ]; then
        RVIZ_CONFIG=~/lunar_rover_ws/realsense_working.rviz
        echo "  Using config: realsense_working.rviz"
    else
        echo "  Using default RViz config"
        RVIZ_CONFIG=""
    fi
fi

echo ""
echo "Starting RViz..."

if [ -n "$RVIZ_CONFIG" ]; then
    ros2 run rviz2 rviz2 -d "$RVIZ_CONFIG" \
      --ros-args -p use_sim_time:=false &
else
    ros2 run rviz2 rviz2 \
      --ros-args -p use_sim_time:=false &
fi

RVIZ_PID=$!
sleep 2

if ps -p $RVIZ_PID > /dev/null 2>&1; then
    echo "  ✓ RViz running (PID $RVIZ_PID)"
    
    if [ "$USE_BUFFERED" = true ]; then
        echo ""
        echo "  📺 BUFFERED MODE ACTIVE"
        echo "  Initial 5 second delay, then smooth playback"
        echo ""
        echo "  To add displays in RViz:"
        echo "    Add → Image"
        echo "    Front camera: /camera/camera/color/buffered/compressed"
        echo "    Rear stereo: /camera_rear/stereo_buffered/compressed"
    else
        echo ""
        echo "  To add displays in RViz:"
        echo "    Add → Image"
        echo "    Front camera: /camera/camera/color/image_raw/compressed"
        echo "    Rear stereo: /camera_rear/stereo_combined/compressed"
    fi
else
    echo "  ✗ RViz failed to start"
fi

# ========================================================================
# Launch Keyboard Teleop (optional)
# ========================================================================

echo ""
read -p "Launch keyboard teleop? (y/n, default=n): " LAUNCH_TELEOP
LAUNCH_TELEOP=${LAUNCH_TELEOP:-n}

if [ "$LAUNCH_TELEOP" = "y" ] || [ "$LAUNCH_TELEOP" = "Y" ]; then
    echo "Starting keyboard teleop..."
    echo "  Use WASD or arrow keys to control"
    xterm -e "source /opt/ros/$ROS_DISTRO/setup.bash && ros2 run teleop_twist_keyboard teleop_twist_keyboard" &
    TELEOP_PID=$!
    sleep 1
    if ps -p $TELEOP_PID > /dev/null 2>&1; then
        echo "  ✓ Teleop running (PID $TELEOP_PID)"
    fi
fi

# ========================================================================
# Launch Joy/Controller Teleop (optional)
# ========================================================================

echo ""
read -p "Launch game controller teleop? (y/n, default=n): " LAUNCH_JOY
LAUNCH_JOY=${LAUNCH_JOY:-n}

if [ "$LAUNCH_JOY" = "y" ] || [ "$LAUNCH_JOY" = "Y" ]; then
    echo "Starting joy node and teleop..."
    
    # Launch joy node
    ros2 run joy joy_node &
    JOY_PID=$!
    sleep 1
    
    # Launch teleop_twist_joy
    ros2 run teleop_twist_joy teleop_node &
    JOY_TELEOP_PID=$!
    sleep 1
    
    if ps -p $JOY_PID > /dev/null 2>&1 && ps -p $JOY_TELEOP_PID > /dev/null 2>&1; then
        echo "  ✓ Controller teleop running"
        echo "  Use left stick for linear, right stick for angular"
        echo "  Hold deadman button (usually L1/LB)"
    else
        echo "  ✗ Controller teleop failed - is controller connected?"
    fi
fi

# ========================================================================
# Launch Nav2 (optional)
# ========================================================================

echo ""
read -p "Launch Nav2 navigation stack? (y/n, default=n): " LAUNCH_NAV
LAUNCH_NAV=${LAUNCH_NAV:-n}

if [ "$LAUNCH_NAV" = "y" ] || [ "$LAUNCH_NAV" = "Y" ]; then
    echo "Starting Nav2..."
    
    # Check if nav2 params exist
    if [ -f ~/lunar_rover_ws/nav2_params.yaml ]; then
        ros2 launch nav2_bringup navigation_launch.py \
          params_file:=~/lunar_rover_ws/nav2_params.yaml &
        NAV_PID=$!
        echo "  ✓ Nav2 launching with custom params..."
    else
        echo "  ⚠ No nav2_params.yaml found, using defaults"
        ros2 launch nav2_bringup navigation_launch.py &
        NAV_PID=$!
        echo "  ✓ Nav2 launching with default params..."
    fi
    
    sleep 3
    if ps -p $NAV_PID > /dev/null 2>&1; then
        echo "  ✓ Nav2 running (PID $NAV_PID)"
    fi
fi

echo ""
echo "========================================="
echo "  ✓✓✓ LAPTOP READY ✓✓✓"
echo "========================================="
echo ""
echo "RUNNING NODES:"
ps aux | grep -E "rviz2|teleop|joy|nav2" | grep -v grep
echo ""

# Check again for buffered topics
BUFFERED_CHECK=$(ros2 topic list 2>/dev/null | grep -c "buffered" || echo "0")

if [ "$BUFFERED_CHECK" -gt 0 ]; then
    echo "📺 BUFFERED MODE - Using 5 second delay for smooth playback"
    echo ""
    echo "IN RVIZ - BUFFERED TOPICS (smooth, 5s delay):"
    echo "  1. Fixed Frame: base_link"
    echo "  2. Image displays should use:"
    echo "     Front: /camera/camera/color/buffered/compressed"
    echo "     Rear:  /camera_rear/stereo_buffered/compressed"
    echo "     Depth: /camera/camera/aligned_depth_to_color/image_raw/compressedDepth"
    echo ""
    echo "  💡 Wait 5 seconds for buffer to fill, then smooth 6 FPS playback!"
    echo ""
else
    echo "IN RVIZ - USE COMPRESSED TOPICS FOR BEST PERFORMANCE:"
    echo "  1. Fixed Frame: base_link (or odom for Nav2)"
    echo "  2. Add Image displays using COMPRESSED transport:"
    echo "     Add → Image → /camera/camera/color/image_raw/compressed"
    echo "     (Make sure Transport dropdown = 'compressed')"
    echo ""
    echo "  Other compressed topics:"
    echo "     /camera/camera/aligned_depth_to_color/image_raw/compressedDepth"
    echo "     /camera_rear/stereo_combined/compressed"
    echo ""
    echo "  ⚠ AVOID point cloud over WiFi (very slow)"
    echo "     /camera/camera/depth/color/points (disabled by default)"
    echo ""
fi

echo "AVAILABLE TOPICS:"
ros2 topic list | grep -E "camera|cmd_vel|scan|map|goal|compressed|buffered" | head -15 || echo "  (none yet)"
echo ""
echo "Press Ctrl+C to stop all nodes"
echo "========================================="

wait