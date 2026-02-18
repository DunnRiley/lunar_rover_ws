#!/bin/bash
# ========================================================================
# Laptop ROS2 Launch Script - FIXED
# - Uses correct compressed topics matching mini_pc_launch.sh output
# - Delay toggle: DELAY_SEC env var must match mini PC setting
# ========================================================================

echo "========================================="
echo "  LAPTOP: Visualization & Control"
echo "========================================="

cd ~/lunar_rover_ws

# ── ROS2 setup ──────────────────────────────────────────────────────────
if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash && echo "✓ ROS2 Jazzy"
elif [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash && echo "✓ ROS2 Humble"
else
    echo "✗ No ROS2 installation found!" && exit 1
fi

[ -f install/setup.bash ] && source install/setup.bash && echo "✓ Workspace sourced"

export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
echo "✓ Network: ROS_DOMAIN_ID=42, SUBNET"
echo ""

trap 'echo ""; echo "Shutting down..."; kill 0' SIGINT SIGTERM

# ── Wait for mini PC topics ──────────────────────────────────────────────
echo "Waiting 5s then checking for mini PC topics..."
sleep 5

TOPICS=$(ros2 topic list 2>/dev/null | grep -cE "camera|tf" || true)
if [ "$TOPICS" -gt 0 ]; then
    echo "✓ Connected — found $TOPICS camera/TF topics"
else
    echo "⚠ Cannot see mini PC topics yet."
    echo "  Make sure mini PC is running: bash mini_pc_launch.sh"
    echo "  Verify same WiFi and ROS_DOMAIN_ID=42 on both machines"
    echo "  Continuing anyway..."
fi
echo ""

# ── Generate RViz config with the correct topics ─────────────────────────
RVIZ_CONFIG=~/lunar_rover_ws/laptop_stream.rviz

cat > "$RVIZ_CONFIG" << 'RVIZEOF'
Panels:
  - Class: rviz_common/Displays
    Name: Displays
  - Class: rviz_common/Views
    Name: Views

Visualization Manager:
  Class: ""
  Displays:
    - Alpha: 0.5
      Cell Size: 1.0
      Class: rviz_default_plugins/Grid
      Color: 160; 160; 164
      Enabled: true
      Name: Grid
      Plane Cell Count: 10
      Reference Frame: <Fixed Frame>
      Value: true

    - Class: rviz_default_plugins/TF
      Enabled: true
      Frame Timeout: 15
      Frames:
        All Enabled: true
      Name: TF
      Show Arrows: true
      Show Axes: true
      Show Names: true
      Value: true

    # ── FRONT CAMERA (D435) ──────────────────────────────────────────
    # Topic matches mini_pc_launch.sh output: /camera/color/stream/compressed
    - Class: rviz_default_plugins/Image
      Enabled: true
      Name: "Front Camera (D435)"
      Normalize Range: true
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Best Effort
        Value: /camera/color/stream/compressed
      Value: true

    # ── DEPTH CAMERA ─────────────────────────────────────────────────
    - Class: rviz_default_plugins/Image
      Enabled: true
      Name: "Front Depth"
      Normalize Range: true
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Best Effort
        Value: /camera/depth/stream/compressed
      Value: true

    # ── REAR STEREO ──────────────────────────────────────────────────
    - Class: rviz_default_plugins/Image
      Enabled: true
      Name: "Rear Stereo"
      Normalize Range: true
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Best Effort
        Value: /camera_rear/stream/compressed
      Value: true

    # ── NAVIGATION ───────────────────────────────────────────────────
    - Class: rviz_default_plugins/MarkerArray
      Enabled: true
      Name: "Navigation Goals"
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /goal_markers
      Value: true

    - Alpha: 1
      Buffer Length: 1
      Class: rviz_default_plugins/Path
      Color: 25; 255; 0
      Enabled: true
      Name: "Planned Path"
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /planned_path
      Value: true

  Enabled: true
  Global Options:
    Background Color: 48; 48; 48
    Fixed Frame: base_link
    Frame Rate: 30
  Name: root
  Tools:
    - Class: rviz_default_plugins/Interact
      Hide Inactive Objects: true
    - Class: rviz_default_plugins/MoveCamera
    - Class: rviz_default_plugins/Select
    - Class: rviz_default_plugins/SetGoal
      Topic:
        Value: /goal_pose
    - Class: rviz_default_plugins/PublishPoint
      Single click: true
      Topic:
        Value: /clicked_point
  Value: true
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: 3.5
      Focal Point:
        X: 0.5
        Y: 0
        Z: 0
      Name: Current View
      Pitch: 0.5
      Target Frame: <Fixed Frame>
      Value: Orbit (rviz)
      Yaw: 0

Window Geometry:
  Height: 900
  Width: 1600
  X: 0
  Y: 0
RVIZEOF

echo "✓ RViz config written to: $RVIZ_CONFIG"
echo ""

# ── Launch RViz ──────────────────────────────────────────────────────────
echo "Starting RViz..."
ros2 run rviz2 rviz2 -d "$RVIZ_CONFIG" \
    --ros-args -p use_sim_time:=false &
RVIZ_PID=$!
sleep 3

if ps -p $RVIZ_PID > /dev/null 2>&1; then
    echo "  ✓ RViz running (PID $RVIZ_PID)"
else
    echo "  ✗ RViz failed to start"
fi
echo ""

# ── Optional: Keyboard Teleop ────────────────────────────────────────────
read -p "Launch keyboard teleop? (y/n) [n]: " LAUNCH_TELEOP
if [[ "${LAUNCH_TELEOP,,}" == "y" ]]; then
    xterm -e "bash -c 'source /opt/ros/$ROS_DISTRO/setup.bash && ros2 run teleop_twist_keyboard teleop_twist_keyboard'" &
    echo "  ✓ Teleop launched in new window"
fi

# ── Optional: Game controller ────────────────────────────────────────────
read -p "Launch game controller? (y/n) [n]: " LAUNCH_JOY
if [[ "${LAUNCH_JOY,,}" == "y" ]]; then
    ros2 run joy joy_node &
    sleep 1
    ros2 run teleop_twist_joy teleop_node &
    echo "  ✓ Joy controller launched"
fi

# ── Status ───────────────────────────────────────────────────────────────
echo ""
echo "========================================="
echo "  ✓✓✓ LAPTOP READY ✓✓✓"
echo "========================================="
echo ""
echo "RVIZ IS SHOWING:"
echo "  /camera/color/stream/compressed   ← Front RGB"
echo "  /camera/depth/stream/compressed   ← Depth"
echo "  /camera_rear/stream/compressed    ← Rear stereo"
echo ""
echo "TROUBLESHOOTING — if images don't appear:"
echo "  1. Check topic exists:"
echo "     ros2 topic list | grep stream"
echo "  2. Check rate (should be ~6 Hz):"
echo "     ros2 topic hz /camera/color/stream/compressed"
echo "  3. Verify pipeline is running on mini PC:"
echo "     ssh moonpie@138.67.181.222 'ps aux | grep pipeline'"
echo "  4. If using RViz 'Image' display, Transport should be 'compressed'"
echo "     (RViz auto-detects this from the /compressed suffix)"
echo ""
echo "Press Ctrl+C to stop all nodes"
echo "========================================="

wait