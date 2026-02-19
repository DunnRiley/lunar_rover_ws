#!/bin/bash
# ============================================================================
#  LAPTOP: SLAM Visualization + Nav2 Goal Sending
#
#  Subscribes to topics from the mini PC. No heavy computation here.
#  Includes Nav2 RViz tools so you can click 2D Nav Goals.
#
#  USAGE:
#    bash slam_laptop.sh          → Full visualization (recommended)
#    bash slam_laptop.sh rviz     → RViz only (no scan helper prompt)
# ============================================================================

MODE=${1:-full}

# ── ROS setup ─────────────────────────────────────────────────────────────
if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash && ROS_DISTRO=jazzy
elif [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash && ROS_DISTRO=humble
else
    echo "✗ No ROS2 found" && exit 1
fi
[ -f ~/lunar_rover_ws/install/setup.bash ] && source ~/lunar_rover_ws/install/setup.bash

export ROS_DOMAIN_ID=42
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
export ROS_LOCALHOST_ONLY=0

echo "========================================="
echo "  LAPTOP: SLAM Visualization + Nav2"
echo "========================================="
echo "ROS2: $ROS_DISTRO  |  DOMAIN: $ROS_DOMAIN_ID"
echo ""

# ── Kill old laptop processes ──────────────────────────────────────────────
pkill -f rviz2 2>/dev/null
sleep 1

# ── Wait for mini PC ──────────────────────────────────────────────────────
echo "Waiting for mini PC topics..."
CONNECTED=false
for i in $(seq 1 20); do
    if ros2 topic list 2>/dev/null | grep -qE "^/map$|^/rtabmap/cloud_map$"; then
        CONNECTED=true
        echo "✓ Mini PC connected"
        break
    fi
    echo -n "  ($i/20)..."
    sleep 2
done
[ "$CONNECTED" = false ] && echo "" && echo "⚠  Mini PC not visible — continuing anyway (check WiFi + ROS_DOMAIN_ID)"
echo ""

# ── Write RViz config ─────────────────────────────────────────────────────
RVIZ_CONFIG="$HOME/lunar_rover_ws/slam_nav2_laptop.rviz"

cat > "$RVIZ_CONFIG" << 'RVIZ_EOF'
Panels:
  - Class: rviz_common/Displays
    Name: Displays
  - Class: rviz_common/Views
    Name: Views
  - Class: rviz_common/Tool Properties
    Name: Tool Properties
    Expanded:
      - /2D Pose Estimate1
      - /2D Goal Pose1

Visualization Manager:
  Class: ""
  Global Options:
    Fixed Frame: map
    Background Color: 25; 25; 35
    Frame Rate: 20
  Displays:

    # ── GROUND GRID ──────────────────────────────────────────────────────
    - Class: rviz_default_plugins/Grid
      Name: Grid
      Enabled: true
      Reference Frame: map
      Cell Size: 0.5
      Color: 80; 80; 80
      Plane Cell Count: 60
      Alpha: 0.4
      Line Style:
        Line Width: 0.02
        Value: Lines

    # ── TF ───────────────────────────────────────────────────────────────
    - Class: rviz_default_plugins/TF
      Name: TF
      Enabled: true
      Show Axes: true
      Show Names: true
      Marker Scale: 0.3
      Frame Timeout: 15.0
      Frames:
        All Enabled: false
        map:
          Value: true
        odom:
          Value: true
        base_link:
          Value: true
        base_footprint:
          Value: true
        camera_link:
          Value: true

    # ── ROBOT POSITION (odometry trail) ──────────────────────────────────
    - Class: rviz_default_plugins/Odometry
      Name: Robot Position
      Enabled: true
      Topic:
        Value: /odom
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Best Effort
      Keep: 300
      Shape:
        Alpha: 0.8
        Color: 255; 170; 0
        Head Length: 0.12
        Head Radius: 0.05
        Shaft Length: 0.25
        Shaft Radius: 0.02
        Value: Arrow
      Position Tolerance: 0.05
      Angle Tolerance: 0.05

    # ── 2D OCCUPANCY MAP (from RTAB-Map /map) ────────────────────────────
    - Class: rviz_default_plugins/Map
      Name: "Occupancy Map (Nav2)"
      Enabled: true
      Topic:
        Value: /map
        Depth: 5
        Durability Policy: Transient Local
        History Policy: Keep Last
        Reliability Policy: Reliable
      Color Scheme: map
      Alpha: 0.75
      Draw Behind: true

    # ── 3D POINT CLOUD MAP ────────────────────────────────────────────────
    - Class: rviz_default_plugins/PointCloud2
      Name: "3D Map (RTAB-Map)"
      Enabled: true
      Topic:
        Value: /rtabmap/cloud_map
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
      Size (m): 0.03
      Style: Points
      Color Transformer: RGB8
      Use Fixed Frame: true
      Decay Time: 0.0
      Alpha: 1.0

    # ── LIVE CAMERA FEED ──────────────────────────────────────────────────
    - Class: rviz_default_plugins/Image
      Name: "Front Camera (Live)"
      Enabled: true
      Topic:
        Value: /camera/color/stream/compressed
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Best Effort
      Queue Size: 1
      Transport: compressed
      Normalize Range: false

    # ── LASER SCAN ────────────────────────────────────────────────────────
    - Class: rviz_default_plugins/LaserScan
      Name: "Laser Scan"
      Enabled: true
      Topic:
        Value: /scan
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Best Effort
      Size (m): 0.04
      Style: Spheres
      Color: 255; 255; 0
      Alpha: 0.8
      Decay Time: 0.0

    # ── NAV2: GLOBAL COSTMAP ──────────────────────────────────────────────
    - Class: rviz_default_plugins/Map
      Name: "Global Costmap"
      Enabled: false
      Topic:
        Value: /global_costmap/costmap
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
      Color Scheme: costmap
      Alpha: 0.5
      Draw Behind: false

    # ── NAV2: LOCAL COSTMAP ───────────────────────────────────────────────
    - Class: rviz_default_plugins/Map
      Name: "Local Costmap"
      Enabled: false
      Topic:
        Value: /local_costmap/costmap
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
      Color Scheme: costmap
      Alpha: 0.5
      Draw Behind: false

    # ── NAV2: PLANNED PATH ────────────────────────────────────────────────
    - Class: rviz_default_plugins/Path
      Name: "Nav2 Global Plan"
      Enabled: true
      Topic:
        Value: /plan
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
      Color: 0; 200; 255
      Line Style: Lines
      Line Width: 0.05
      Alpha: 1.0

    # ── NAV2: LOCAL PLAN ─────────────────────────────────────────────────
    - Class: rviz_default_plugins/Path
      Name: "Nav2 Local Plan"
      Enabled: true
      Topic:
        Value: /local_plan
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
      Color: 255; 100; 0
      Line Style: Lines
      Line Width: 0.04
      Alpha: 0.9

    # ── RTAB-MAP: ODOMETRY PATH ───────────────────────────────────────────
    - Class: rviz_default_plugins/Path
      Name: "Odometry Trail"
      Enabled: true
      Topic:
        Value: /rtabmap/odom_path
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
      Color: 255; 150; 0
      Line Style: Lines
      Line Width: 0.03
      Alpha: 0.7

    # ── RTAB-MAP: OPTIMIZED PATH ──────────────────────────────────────────
    - Class: rviz_default_plugins/Path
      Name: "Optimized Path (RTAB-Map)"
      Enabled: false
      Topic:
        Value: /rtabmap/mapPath
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
      Color: 0; 255; 0
      Line Style: Lines
      Line Width: 0.04

    # ── WAYPOINTS ─────────────────────────────────────────────────────────
    - Class: rviz_default_plugins/MarkerArray
      Name: "Waypoints"
      Enabled: true
      Topic:
        Value: /waypoint_markers
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable

    - Class: rviz_default_plugins/Marker
      Name: "Current Target"
      Enabled: true
      Topic:
        Value: /target_marker
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable

    # ── NAV2 FEEDBACK MARKERS ─────────────────────────────────────────────
    - Class: rviz_default_plugins/MarkerArray
      Name: "Nav2 Markers"
      Enabled: true
      Topic:
        Value: /waypoint_markers
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable

  Tools:
    - Class: rviz_default_plugins/Interact
      Name: Interact
    - Class: rviz_default_plugins/MoveCamera
      Name: Move Camera
    - Class: rviz_default_plugins/Select
      Name: Select
    - Class: rviz_default_plugins/FocusCamera
      Name: Focus Camera
    - Class: rviz_default_plugins/Measure
      Name: Measure
    - Class: rviz_default_plugins/SetInitialPose
      Name: 2D Pose Estimate
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /initialpose
    - Class: rviz_default_plugins/SetGoal
      Name: 2D Nav Goal
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /goal_pose
    - Class: rviz_default_plugins/PublishPoint
      Name: Publish Point
      Single click: true
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /clicked_point

  Transformation:
    Current:
      Class: rviz_default_plugins/TF
  Value: true

  Views:
    Current:
      Class: rviz_default_plugins/TopDownOrtho
      Name: Top Down (Map)
      Scale: 60.0
      X: 0.0
      Y: 0.0
      Angle: 0.0
      Enable Stereo Rendering: false
    Saved:
      - Class: rviz_default_plugins/Orbit
        Name: 3D View
        Distance: 8.0
        Focal Point:
          X: 0.0
          Y: 0.0
          Z: 0.0
        Pitch: 0.65
        Yaw: 0.5

Window Geometry:
  Height: 1000
  Width: 1800
  X: 0
  Y: 0
RVIZ_EOF

echo "✓ RViz config → $RVIZ_CONFIG"
echo ""

# ── Start RViz ────────────────────────────────────────────────────────────
echo "Starting RViz..."
rviz2 -d "$RVIZ_CONFIG" 2>/tmp/slam_laptop_rviz.log &
RVIZ_PID=$!
sleep 3

if ps -p $RVIZ_PID >/dev/null 2>&1; then
    echo "✓ RViz running (PID $RVIZ_PID)"
else
    echo "✗ RViz failed — check: tail /tmp/slam_laptop_rviz.log"
fi
echo ""

echo "========================================="
echo "  LAPTOP READY"
echo "========================================="
echo ""
echo "RViz Fixed Frame = 'map'"
echo ""
echo "HOW TO NAVIGATE:"
echo "  1. SLAM Build Phase:"
echo "     Drive the rover (teleop) slowly around the area"
echo "     Watch the 3D map build in RViz"
echo "     Aim for a 360° scan of the environment"
echo ""
echo "  2. Send Nav2 Goals:"
echo "     Select '2D Nav Goal' tool in RViz toolbar"
echo "     Click+drag on the map to set position AND heading"
echo "     Nav2 plans and drives the rover automatically"
echo ""
echo "  3. Set Initial Pose (if localizing):"
echo "     Select '2D Pose Estimate' tool"
echo "     Click where you think the robot currently is"
echo ""
echo "  4. Multi-Waypoint (optional):"
echo "     python3 ~/lunar_rover_ws/waypoint_selector.py"
echo ""
echo "TROUBLESHOOTING:"
echo "  No map?  → Rotate camera, wait 20 s for first keyframe"
echo "  Nav2 not planning?  → Check: ros2 topic echo /map (should have data)"
echo "  Robot not moving?  → Check: ros2 topic echo /cmd_vel"
echo ""
echo "TOPICS FROM MINI PC:"
echo "  /map                     2D occupancy map"
echo "  /rtabmap/cloud_map       3D point cloud"
echo "  /odom                    Robot odometry"
echo "  /scan                    Laser scan"
echo "  /plan                    Nav2 planned path"
echo ""
echo "Press Ctrl+C to stop"
echo "========================================="

if [ "$MODE" = "full" ]; then
    echo ""
    echo "Tip: Run scan helper in a new terminal:"
    echo "  python3 ~/lunar_rover_ws/scan_helper.py"
fi

trap 'echo ""; pkill -f rviz2; exit' SIGINT SIGTERM
wait $RVIZ_PID