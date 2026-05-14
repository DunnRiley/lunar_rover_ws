#!/bin/bash
# ============================================================================
#  MINI PC (or single laptop): RTAB-Map SLAM  — FIXED
#
#  Fixes applied:
#  - Odom/GuessMotion passed as string "false" (was bare bool → crash)
#  - Grid/CellSize, Grid/RangeMax etc. passed as unquoted doubles (were
#    quoted strings → InvalidParameterTypeException crash)
#  - Nav2 removed for proof-of-concept (nav2_route missing in Jazzy base)
#    Re-add once:  sudo apt install ros-jazzy-nav2-bringup ros-jazzy-navigation2
#  - publish_tf on visual odometry set to TRUE for single-machine use
#    (dead-reckoning odom still runs as fallback but visual odom drives TF)
#
#  MODES:
#    bash slam_minipc.sh map          → Fresh map (deletes old DB)
#    bash slam_minipc.sh map keep     → Resume / extend existing map
#    bash slam_minipc.sh localize     → Load saved map, localize only
#
#  MAP FILE: ~/.ros/rtabmap_rover.db
# ============================================================================

MODE=${1:-map}
KEEP_MAP=${2:-""}
DB_PATH="$HOME/.ros/rtabmap_rover.db"

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

# ── FastDDS: disable shared memory transport ──────────────────────────────
# The SHM transport on a single machine often hits stale port-lock files
# (fastrtps_port7027 etc.), killing nodes during startup.  Force UDP-only.
FASTDDS_XML=/tmp/fastdds_udp_only.xml
cat > "$FASTDDS_XML" << 'FASTDDS_EOF'
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
    <transport_descriptors>
        <transport_descriptor>
            <transport_id>UDPv4Transport</transport_id>
            <type>UDPv4</type>
        </transport_descriptor>
    </transport_descriptors>
    <participant profile_name="participant_profile" is_default_profile="true">
        <rtps>
            <userTransports>
                <transport_id>UDPv4Transport</transport_id>
            </userTransports>
            <useBuiltinTransports>false</useBuiltinTransports>
        </rtps>
    </participant>
</profiles>
FASTDDS_EOF
export FASTRTPS_DEFAULT_PROFILES_FILE="$FASTDDS_XML"

# Also clean up any stale SHM port-lock files left from previous runs
rm -f /dev/shm/fastrtps_* /tmp/fastrtps_* 2>/dev/null
echo "  ✓ FastDDS: SHM disabled, stale locks cleared"

# ── Print header ──────────────────────────────────────────────────────────
echo "========================================="
case "$MODE" in
    localize) echo "  SLAM — LOCALIZATION MODE" ;;
    *)
        if [ "$KEEP_MAP" = "keep" ]; then
            echo "  SLAM — MAPPING (RESUME)"
        else
            echo "  SLAM — MAPPING (FRESH)"
        fi ;;
esac
echo "========================================="
echo "ROS2: $ROS_DISTRO  |  DOMAIN: $ROS_DOMAIN_ID"
echo ""

# ── Dependency checks ─────────────────────────────────────────────────────
echo "Checking dependencies..."
MISSING=0

check_pkg() {
    if ! ros2 pkg list 2>/dev/null | grep -q "^$1$"; then
        echo "  ✗ Missing: $1"
        MISSING=$((MISSING+1))
    else
        echo "  ✓ $1"
    fi
}

check_pkg rtabmap_ros
check_pkg rtabmap_slam
check_pkg rtabmap_odom
check_pkg depthimage_to_laserscan
check_pkg realsense2_camera

# Nav2 is optional — warn but don't exit
if ! ros2 pkg list 2>/dev/null | grep -q "^nav2_bringup$"; then
    echo "  ⚠ nav2_bringup not found — Nav2 disabled (mapping still works)"
    echo "    To install: sudo apt install ros-${ROS_DISTRO}-nav2-bringup ros-${ROS_DISTRO}-navigation2"
    NAV2_AVAILABLE=false
else
    echo "  ✓ nav2_bringup"
    NAV2_AVAILABLE=true
fi

if [ $MISSING -gt 0 ]; then
    echo ""
    echo "Install missing packages:"
    echo "  sudo apt install ros-${ROS_DISTRO}-rtabmap-ros \\"
    echo "                   ros-${ROS_DISTRO}-depthimage-to-laserscan"
    exit 1
fi
echo ""

# ── Map file handling ─────────────────────────────────────────────────────
if [ "$MODE" = "localize" ] && [ ! -f "$DB_PATH" ]; then
    echo "✗ No map at $DB_PATH — run mapping first"
    exit 1
fi
[ -f "$DB_PATH" ] && echo "Map file: $DB_PATH ($(du -h "$DB_PATH" | cut -f1))" || echo "No existing map (will create)"
echo ""

# ── Kill old processes ────────────────────────────────────────────────────
echo "Stopping old nodes..."
pkill -f realsense2_camera_node 2>/dev/null
pkill -f rgbd_odometry           2>/dev/null
pkill -f "rtabmap "              2>/dev/null   # space avoids killing this script's grep
pkill -f robot_state_publisher   2>/dev/null
pkill -f static_transform_pub    2>/dev/null
pkill -f depthimage_to_laserscan 2>/dev/null
sleep 2
echo "✓ Clean"
echo ""

trap 'echo ""; echo "Shutting down..."; kill 0; wait; echo "✓ Stopped"; exit' SIGINT SIGTERM

# ── URDF ─────────────────────────────────────────────────────────────────
URDF='<?xml version="1.0"?>
<robot name="lunar_rover">
  <link name="base_link"/>
  <link name="base_footprint"/>
  <joint name="base_footprint_joint" type="fixed">
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
</robot>'

# ── [1/5] TF TREE ─────────────────────────────────────────────────────────
echo "[1/5] TF tree + robot description..."

ros2 run robot_state_publisher robot_state_publisher \
    --ros-args -p robot_description:="$URDF" -p publish_frequency:=50.0 \
    2>/tmp/slam_rsp.log &

sleep 1

# Use quaternion form — avoids RPY parsing quirks across ROS2 versions
ros2 run tf2_ros static_transform_publisher \
    --x 0.0 --y 0.0 --z 0.0 \
    --qx -0.5 --qy 0.5 --qz -0.5 --qw 0.5 \
    --frame-id camera_link \
    --child-frame-id camera_depth_optical_frame \
    2>/dev/null &

ros2 run tf2_ros static_transform_publisher \
    --x 0.0 --y 0.0 --z 0.0 \
    --qx -0.5 --qy 0.5 --qz -0.5 --qw 0.5 \
    --frame-id camera_link \
    --child-frame-id camera_color_optical_frame \
    2>/dev/null &

# Publish an identity map→odom TF so RViz has a valid "map" frame from the
# start, before RTAB-Map creates its first keyframe.
ros2 run tf2_ros static_transform_publisher     --x 0.0 --y 0.0 --z 0.0     --qx 0.0 --qy 0.0 --qz 0.0 --qw 1.0     --frame-id map     --child-frame-id odom     2>/dev/null &

# Wait until robot_state_publisher has actually published the TF tree.
# Without this, rgbd_odometry and rtabmap start before base_link exists.
echo "  Waiting for TF: base_link..."
for i in $(seq 1 20); do
    sleep 1
    if ros2 run tf2_ros tf2_echo base_link camera_link --timeout 0.5 2>/dev/null | grep -q "Translation"; then
        echo "  ✓ TF tree ready (base_link confirmed at ${i}s)"
        break
    fi
    if [ $i -eq 20 ]; then
        echo "  ⚠ TF tree slow — continuing anyway (check: ros2 run tf2_tools view_frames)"
    fi
done
echo ""

# ── [2/5] DEAD-RECKONING ODOMETRY ─────────────────────────────────────────
echo "[2/5] Dead-reckoning odometry (cmd_vel integration)..."

python3 - << 'ODOM_EOF' &
#!/usr/bin/env python3
import rclpy, math
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TransformStamped
from tf2_ros import TransformBroadcaster

class DeadReckonOdom(Node):
    def __init__(self):
        super().__init__('dead_reckoning_odom')
        self.x = self.y = self.th = self.vx = self.vth = 0.0
        self.last = self.get_clock().now()
        self.br = TransformBroadcaster(self)
        self.pub = self.create_publisher(Odometry, '/odom', 50)
        self.create_subscription(Twist, '/cmd_vel', self.cb, 10)
        self.create_timer(0.02, self.update)
        self.get_logger().info('Dead-reckoning odometry running (50 Hz)')

    def cb(self, msg):
        self.vx  = msg.linear.x
        self.vth = msg.angular.z

    def update(self):
        now = self.get_clock().now()
        dt = (now - self.last).nanoseconds / 1e9
        self.last = now
        if dt <= 0 or dt > 0.5:
            return

        dx  = self.vx * math.cos(self.th) * dt
        dy  = self.vx * math.sin(self.th) * dt
        dth = self.vth * dt
        self.x  += dx
        self.y  += dy
        self.th += dth
        while self.th >  math.pi: self.th -= 2*math.pi
        while self.th < -math.pi: self.th += 2*math.pi

        qz = math.sin(self.th/2.0)
        qw = math.cos(self.th/2.0)
        stamp = now.to_msg()

        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'odom'
        t.child_frame_id  = 'base_footprint'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation.z    = qz
        t.transform.rotation.w    = qw
        self.br.sendTransform(t)

        o = Odometry()
        o.header.stamp    = stamp
        o.header.frame_id = 'odom'
        o.child_frame_id  = 'base_footprint'
        o.pose.pose.position.x    = self.x
        o.pose.pose.position.y    = self.y
        o.pose.pose.orientation.z = qz
        o.pose.pose.orientation.w = qw
        o.twist.twist.linear.x  = self.vx
        o.twist.twist.angular.z = self.vth
        o.pose.covariance[0]  = 0.05
        o.pose.covariance[7]  = 0.05
        o.pose.covariance[35] = 0.1
        o.twist.covariance[0] = 0.05
        o.twist.covariance[35]= 0.1
        self.pub.publish(o)

rclpy.init()
node = DeadReckonOdom()
try:
    rclpy.spin(node)
except (KeyboardInterrupt, Exception):
    pass
finally:
    node.destroy_node()
    rclpy.try_shutdown()
ODOM_EOF

ODOM_PID=$!
sleep 2
echo "  ✓ Odometry publishing on /odom (PID $ODOM_PID)"
echo ""

# ── [3/5] CAMERA ─────────────────────────────────────────────────────────
echo "[3/5] D435 Camera (640×480 @ 15 fps)..."

ros2 launch realsense2_camera rs_launch.py \
    camera_name:=camera \
    camera_namespace:=camera \
    enable_depth:=true \
    enable_color:=true \
    enable_infra1:=false \
    enable_infra2:=false \
    pointcloud.enable:=true \
    align_depth.enable:=true \
    enable_sync:=true \
    depth_module.profile:=640x480x15 \
    rgb_camera.profile:=640x480x15 \
    2>/tmp/slam_camera.log &

CAM_PID=$!
echo "  Waiting for camera (10 s)..."
sleep 10

if ! ps -p $CAM_PID >/dev/null 2>&1; then
    echo "  ✗ Camera failed! Check: tail /tmp/slam_camera.log"
    exit 1
fi

# Wait for image topic
for i in $(seq 1 20); do
    ros2 topic list 2>/dev/null | grep -q "/camera/camera/color/image_raw$" && break
    sleep 1
done
ros2 topic list 2>/dev/null | grep -q "/camera/camera/color/image_raw$" \
    && echo "  ✓ Camera publishing (PID $CAM_PID)" \
    || { echo "  ✗ No camera topics! Check: tail /tmp/slam_camera.log"; exit 1; }

# Depth→LaserScan (handy for visualisation in RViz)
ros2 run depthimage_to_laserscan depthimage_to_laserscan_node \
    --ros-args \
    --remap depth/image_raw:=/camera/camera/aligned_depth_to_color/image_raw \
    --remap depth/camera_info:=/camera/camera/depth/camera_info \
    --remap scan:=/scan \
    -p scan_height:=1 \
    -p scan_time:=0.066 \
    -p range_min:=0.2 \
    -p range_max:=5.0 \
    -p output_frame_id:=camera_link \
    2>/tmp/slam_scan.log &
echo "  ✓ Depth→LaserScan on /scan"
echo ""

# ── [4/5] RTAB-MAP VISUAL ODOMETRY ───────────────────────────────────────
echo "[4/5] RTAB-Map RGB-D visual odometry..."

# KEY FIX: all RTAB-Map string parameters must be quoted; bool-typed params
# (Odom/GuessMotion) must also be quoted strings — RTAB-Map reads them as
# strings internally even though they represent booleans.
# publish_tf=true so visual odometry owns the odom→base_link TF on a single
# machine (dead-reckoning above still publishes /odom topic as fallback).

ros2 run rtabmap_odom rgbd_odometry \
    --ros-args \
    -p frame_id:=base_link \
    -p odom_frame_id:=odom \
    -p publish_tf:=true \
    -p approx_sync:=true \
    -p approx_sync_max_interval:=0.15 \
    -p wait_for_transform:=0.3 \
    -p queue_size:=30 \
    -p 'Odom/Strategy:=0' \
    -p 'Vis/FeatureType:=6' \
    -p 'Vis/MaxFeatures:=600' \
    -p 'Vis/MinInliers:=8' \
    -p 'Vis/MaxDepth:=5.0' \
    -p 'Odom/ResetCountdown:=1' \
    -p 'Odom/GuessMotion:=false' \
    --remap rgb/image:=/camera/camera/color/image_raw \
    --remap rgb/camera_info:=/camera/camera/color/camera_info \
    --remap depth/image:=/camera/camera/aligned_depth_to_color/image_raw \
    2>/tmp/slam_vodom.log &

VODOM_PID=$!
echo "  Waiting for visual odometry (6 s)..."
sleep 6

if ! ps -p $VODOM_PID >/dev/null 2>&1; then
    echo "  ✗ Visual odometry crashed!"
    echo "  Last lines of log:"
    tail -10 /tmp/slam_vodom.log
    exit 1
fi
echo "  ✓ Visual odometry running (PID $VODOM_PID)"
echo ""

# ── [5/5] RTAB-MAP SLAM ───────────────────────────────────────────────────
echo "[5/5] RTAB-Map SLAM..."

case "$MODE" in
    localize)
        RTAB_ARGS="--localization"
        MEM_PARAM="-p 'Mem/IncrementalMemory:=false'"
        echo "  Mode: LOCALIZATION"
        ;;
    *)
        if [ "$KEEP_MAP" = "keep" ]; then
            RTAB_ARGS=""
            MEM_PARAM="-p 'Mem/IncrementalMemory:=true'"
            echo "  Mode: MAPPING (resume)"
        else
            RTAB_ARGS="--delete_db_on_start"
            MEM_PARAM="-p 'Mem/IncrementalMemory:=true'"
            echo "  Mode: MAPPING (fresh)"
        fi ;;
esac

# KEY FIX: numeric RTAB-Map params (Grid/CellSize, Grid/RangeMax, etc.) must
# NOT be quoted — they must be bare doubles so ROS2 parses them as double,
# not string.  String-valued params (Reg/Force3DoF, Grid/FromDepth, etc.)
# must still be quoted because RTAB-Map reads them as strings.

ros2 run rtabmap_slam rtabmap \
    --ros-args \
    -p frame_id:=base_link \
    -p odom_frame_id:=odom \
    -p map_frame_id:=map \
    -p subscribe_depth:=true \
    -p subscribe_rgb:=true \
    -p subscribe_odom_info:=false \
    -p approx_sync:=true \
    -p wait_for_transform:=0.3 \
    -p queue_size:=30 \
    -p topic_queue_size:=30 \
    -p database_path:=$DB_PATH \
    -p 'Rtabmap/DetectionRate:=1.0' \
    -p 'RGBD/LinearUpdate:=0.1' \
    -p 'RGBD/AngularUpdate:=0.17' \
    -p 'Vis/FeatureType:=6' \
    -p 'Kp/DetectorStrategy:=6' \
    -p 'Vis/MaxFeatures:=600' \
    -p 'Vis/MinInliers:=8' \
    -p 'Kp/MaxFeatures:=600' \
    -p 'Vis/MaxDepth:=5.0' \
    -p 'Grid/FromDepth:=true' \
    -p 'Grid/MaxObstacleHeight:=0.6' \
    -p 'Grid/MinObstacleHeight:=0.02' \
    -p 'Grid/RangeMax:=5.0' \
    -p 'Grid/CellSize:=0.05' \
    -p 'Grid/DepthDecimation:=2' \
    -p 'Reg/Force3DoF:=true' \
    -p 'RGBD/OptimizeFromGraphEnd:=false' \
    -p 'RGBD/ProximityBySpace:=true' \
    -p 'Mem/NotLinkedNodesKept:=false' \
    $MEM_PARAM \
    --remap rgb/image:=/camera/camera/color/image_raw \
    --remap rgb/camera_info:=/camera/camera/color/camera_info \
    --remap depth/image:=/camera/camera/aligned_depth_to_color/image_raw \
    --remap grid_map:=/map \
    -- $RTAB_ARGS \
    2>/tmp/slam_rtabmap.log &

RTAB_PID=$!
echo "  Waiting for RTAB-Map (polling up to 12s)..."
RTAB_OK=false
for i in $(seq 1 12); do
    sleep 1
    if ! ps -p $RTAB_PID >/dev/null 2>&1; then
        echo ""
        echo "  ✗ RTAB-Map crashed (died after ${i}s)!"
        echo ""
        echo "  ── /tmp/slam_rtabmap.log (last 30 lines) ──"
        tail -30 /tmp/slam_rtabmap.log
        echo ""
        echo "  Common causes:"
        echo "   • SHM port lock  → stale /dev/shm/fastrtps_* (script clears these on start)"
        echo "   • DB locked      → rm ~/.ros/rtabmap_rover.db  then re-run"
        echo "   • Param type err → check for InvalidParameterTypeException above"
        kill 0; wait
        exit 1
    fi
    if grep -q "rtabmap:" /tmp/slam_rtabmap.log 2>/dev/null; then
        RTAB_OK=true
        echo "  ✓ RTAB-Map running (PID $RTAB_PID, confirmed at ${i}s)"
        break
    fi
done
if [ "$RTAB_OK" = "false" ]; then
    echo "  ⚠ RTAB-Map alive but slow to log — continuing (monitor: tail -f /tmp/slam_rtabmap.log)"
fi
echo ""

# ── Nav2 (optional — skip if package missing) ─────────────────────────────
if [ "$NAV2_AVAILABLE" = "true" ]; then
    echo "[+] Nav2 available — skipping for now (add --nav2 flag to enable)"
    echo "    (Nav2 needs a stable map first; build the map, then re-run)"
fi

# ── Summary ───────────────────────────────────────────────────────────────
echo "========================================="
echo "  ✓✓✓ SLAM READY ✓✓✓"
echo "========================================="
echo ""
echo "PIDs:"
echo "  odom    $ODOM_PID"
echo "  camera  $CAM_PID"
echo "  vodom   $VODOM_PID"
echo "  rtabmap $RTAB_PID"
echo ""
echo "KEY TOPICS:"
echo "  /map                   ← occupancy grid (once map builds)"
echo "  /rtabmap/cloud_map     ← 3D point cloud map"
echo "  /odom                  ← odometry"
echo "  /scan                  ← laser scan (from depth)"
echo ""
echo "NOW: open a second terminal and run:"
echo "  bash slam_laptop.sh"
echo ""
echo "THEN: move the camera slowly to build the map."
echo "  Point camera at a textured surface (not a blank wall)."
echo "  After ~5 seconds of movement you should see /rtabmap/cloud_map."
echo ""
echo "TIPS if map doesn't build:"
echo "  tail -f /tmp/slam_vodom.log   ← look for 'failed to find features'"
echo "  tail -f /tmp/slam_rtabmap.log ← look for 'new node added'"
echo ""
echo "LOGS:  /tmp/slam_camera.log  /tmp/slam_rtabmap.log  /tmp/slam_vodom.log"
echo "Map:   $DB_PATH"
echo ""
echo "Press Ctrl+C to stop"
echo "========================================="

# ── Monitor loop ──────────────────────────────────────────────────────────
TICK=0
while true; do
    sleep 5
    TICK=$((TICK+1))

    for NAME_PID in "RTAB-Map:$RTAB_PID" "Camera:$CAM_PID" "VisualOdom:$VODOM_PID"; do
        NAME="${NAME_PID%%:*}"
        PID="${NAME_PID##*:}"
        if ! ps -p $PID >/dev/null 2>&1; then
            echo ""
            echo "⚠  $NAME (PID $PID) died!"
            echo "   Check logs in /tmp/slam_*.log"
            kill 0; wait
            exit 1
        fi
    done

    # Every minute print a heartbeat
    if [ $((TICK % 12)) -eq 0 ]; then
        NODES=$(ros2 node list 2>/dev/null | grep -c "" || echo "?")
        echo "[$(date +%H:%M:%S)] Running — $NODES nodes active"
    fi
done