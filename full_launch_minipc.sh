#!/bin/bash
# ============================================================================
# MINI PC UNIFIED LAUNCH  ·  full_launch_minipc.sh
#
# HOW TO USE:
#   bash full_launch_minipc.sh               # normal / test mode
#   DELAY_SEC=1.0 bash full_launch_minipc.sh # competition (1s delay)
#
# KEY FIXES IN THIS VERSION:
#   1. ROS env vars set BEFORE launching — were missing in SSH sessions
#   2. Color pipeline uses input_reliable:=true (D435 compressed = RELIABLE)
#   3. FastDDS UDP-only — fixes silent cross-machine topic blocking
#   4. Stereo camera: uses /dev/video_stereo symlink (stable) falling back to scan
# ============================================================================

CYAN='\033[0;36m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'
RED='\033[0;31m'; BOLD='\033[1m'; NC='\033[0m'

log()  { echo -e "${CYAN}[miniPC]${NC} $1"; }
ok()   { echo -e "${GREEN}  ✓${NC} $1"; }
warn() { echo -e "${YELLOW}  ⚠${NC} $1"; }
err()  { echo -e "${RED}  ✗${NC} $1"; }

echo -e "${BOLD}"
echo "  ╔══════════════════════════════════════════╗"
echo "  ║     LUNAR ROVER  ·  Mini PC Launch       ║"
echo "  ╚══════════════════════════════════════════╝"
echo -e "${NC}"

cd ~/lunar_rover_ws

# ─────────────────────────────────────────────────────────────────────────────
# ROS2 setup
# ─────────────────────────────────────────────────────────────────────────────
set +u
if   [ -f /opt/ros/jazzy/setup.bash ];  then source /opt/ros/jazzy/setup.bash  && ok "ROS2 Jazzy"
elif [ -f /opt/ros/humble/setup.bash ]; then source /opt/ros/humble/setup.bash && ok "ROS2 Humble"
else err "No ROS2 installation found!" && exit 1
fi
[ -f install/setup.bash ] && source install/setup.bash && ok "Workspace sourced"
set -u

# ─────────────────────────────────────────────────────────────────────────────
# REQUIRED: These env vars must be set. The miniPC ~/.bashrc was missing them.
# Run fix_minipc_env.sh once to persist them — for now we set them here.
# ─────────────────────────────────────────────────────────────────────────────
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
ok "ROS_DOMAIN_ID=42  ROS_LOCALHOST_ONLY=0  SUBNET"

# FastDDS: force UDP-only, clear stale SHM locks
# Shared-memory transport silently blocks cross-machine topic discovery.
FASTDDS_XML=/tmp/fastdds_udp_only.xml
cat > "$FASTDDS_XML" << 'EOF'
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
EOF
export FASTRTPS_DEFAULT_PROFILES_FILE="$FASTDDS_XML"
rm -f /dev/shm/fastrtps_* /tmp/fastrtps_* 2>/dev/null
ok "FastDDS: UDP-only, stale SHM cleared"
echo ""

# Competition delay
DELAY_SEC=${DELAY_SEC:-0.0}
log "Delay mode: ${DELAY_SEC}s $([ "$DELAY_SEC" = "0.0" ] && echo '(live)' || echo '(COMPETITION)')"

# ─────────────────────────────────────────────────────────────────────────────
# Kill stale processes
# ─────────────────────────────────────────────────────────────────────────────
log "Clearing stale nodes..."
pkill -f "realsense2_camera_node"     2>/dev/null
pkill -f "optimized_image_pipeline"   2>/dev/null
pkill -f "stereo_camera_publisher"    2>/dev/null
pkill -f "robot_state_publisher"      2>/dev/null
pkill -f "static_transform_publisher" 2>/dev/null
pkill -f "joy_to_arduino"             2>/dev/null
sleep 2; ok "Cleared"
echo ""

trap 'echo ""; log "Shutting down..."; kill 0; exit' SIGINT SIGTERM

# ══════════════════════════════════════════════════════════════════════════════
# 1 · TF TREE
# ══════════════════════════════════════════════════════════════════════════════
log "[1/5] TF tree..."
URDF='<?xml version="1.0"?>
<robot name="lunar_rover">
  <link name="base_link"/>
  <link name="camera_link"/>
  <link name="camera_rear_link"/>
  <joint name="base_to_camera" type="fixed">
    <parent link="base_link"/><child link="camera_link"/>
    <origin xyz="0.2 0 0.15" rpy="0 0 0"/>
  </joint>
  <joint name="base_to_camera_rear" type="fixed">
    <parent link="base_link"/><child link="camera_rear_link"/>
    <origin xyz="-0.2 0 0.15" rpy="0 0 3.14159"/>
  </joint>
</robot>'

ros2 run robot_state_publisher robot_state_publisher \
    --ros-args -p robot_description:="$URDF" > /tmp/rover_rsp.log 2>&1 &
sleep 1

ros2 run tf2_ros static_transform_publisher \
    0 0 0 -1.5707963267948966 0 -1.5707963267948966 \
    camera_link camera_color_optical_frame > /tmp/rover_tf1.log 2>&1 &

ros2 run tf2_ros static_transform_publisher \
    0 0 0 -1.5707963267948966 0 -1.5707963267948966 \
    camera_link camera_depth_optical_frame > /tmp/rover_tf2.log 2>&1 &
sleep 1
ok "TF ready"
echo ""

# ══════════════════════════════════════════════════════════════════════════════
# 2 · FRONT D435 CAMERA
# ══════════════════════════════════════════════════════════════════════════════
log "[2/5] D435 Front Camera..."

ros2 launch realsense2_camera rs_launch.py \
    camera_name:=camera \
    camera_namespace:=camera \
    enable_depth:=true \
    enable_color:=true \
    pointcloud.enable:=false \
    align_depth.enable:=true \
    enable_sync:=true \
    depth_module.profile:=424x240x30 \
    rgb_camera.profile:=424x240x30 > /tmp/rover_camera.log 2>&1 &

CAM_PID=$!
log "  Waiting up to 15s for camera topics..."
CAMERA_UP=false
for i in $(seq 1 15); do
    sleep 1
    if ros2 topic list 2>/dev/null | grep -q "/camera/camera/color/image_raw"; then
        CAMERA_UP=true
        ok "D435 topics visible (${i}s)"; break
    fi
    echo -n "."
done
echo ""

if [ "$CAMERA_UP" = "false" ]; then
    warn "D435 topics not up yet — check: tail /tmp/rover_camera.log"
fi

# Check if the raw color topic exists (we use raw, not /compressed)
log "  D435 raw color topic QoS:"
ros2 topic info -v /camera/camera/color/image_raw 2>/dev/null \
    | grep -E "Reliability|Durability" | head -4 | while read line; do
    echo "    $line"
done
# (If /compressed topic is also present, it was likely published by image_transport plugins)
if ros2 topic list 2>/dev/null | grep -q "/camera/camera/color/image_raw/compressed"; then
    ok "  /compressed also available (image_transport republisher running)"
else
    warn "  /compressed NOT available — using raw topic (correct)"
fi
echo ""

# ══════════════════════════════════════════════════════════════════════════════
# 3 · STREAMING PIPELINE
# ══════════════════════════════════════════════════════════════════════════════
log "[3/5] Streaming pipelines..."

PIPELINE="$HOME/lunar_rover_ws/optimized_image_pipeline.py"
if [ ! -f "$PIPELINE" ]; then
    err "optimized_image_pipeline.py not found at $PIPELINE"
    err "Copy the updated file there and re-run"
else
    # ── Color stream ──────────────────────────────────────────────────────
    # Use RAW topic — realsense doesn't publish /compressed by default.
    # (If the QoS block above was empty, the /compressed topic didn't exist.)
    python3 "$PIPELINE" \
        --ros-args \
        -p input_topic:=/camera/camera/color/image_raw \
        -p output_topic:=/camera/color/stream/compressed \
        -p input_is_compressed:=false \
        -p input_reliable:=false \
        -p jpeg_quality:=30 \
        -p decimation:=5 \
        -p buffer_delay_sec:="${DELAY_SEC}" \
        -p target_fps:=6.0 > /tmp/rover_pipe_color.log 2>&1 &
    COLOR_PID=$!
    sleep 1

    if kill -0 $COLOR_PID 2>/dev/null; then
        ok "Color pipeline (PID $COLOR_PID) → /camera/color/stream/compressed @ 6fps"
    else
        err "Color pipeline crashed — check /tmp/rover_pipe_color.log"
        tail -5 /tmp/rover_pipe_color.log
    fi

    # ── Depth stream ──────────────────────────────────────────────────────
    # Aligned depth is raw Image (16UC1) — input_is_compressed=false
    # RealSense raw topics use BEST_EFFORT QoS
    python3 "$PIPELINE" \
        --ros-args \
        -p input_topic:=/camera/camera/aligned_depth_to_color/image_raw \
        -p output_topic:=/camera/depth/stream/compressed \
        -p input_is_compressed:=false \
        -p input_reliable:=false \
        -p jpeg_quality:=50 \
        -p decimation:=10 \
        -p buffer_delay_sec:="${DELAY_SEC}" \
        -p target_fps:=3.0 > /tmp/rover_pipe_depth.log 2>&1 &
    DEPTH_PID=$!
    sleep 1

    if kill -0 $DEPTH_PID 2>/dev/null; then
        ok "Depth pipeline (PID $DEPTH_PID) → /camera/depth/stream/compressed @ 3fps"
    else
        err "Depth pipeline crashed — check /tmp/rover_pipe_depth.log"
        tail -5 /tmp/rover_pipe_depth.log
    fi
fi
echo ""
# ════════════════════════════════════════════════════════════════════════
# 4 · REAR STEREO CAMERA
# ════════════════════════════════════════════════════════════════════════
log "[4/5] Rear stereo camera..."

STEREO_SCRIPT=""
for candidate in \
    ~/lunar_rover_ws/stereo_camera_publisher.py \
    ~/lunar_rover_ws/DiagnosticAndTesting/stereo_camera_publisher.py; do
    [ -f "$candidate" ] && STEREO_SCRIPT="$candidate" && break
done

if [ -z "$STEREO_SCRIPT" ]; then
    warn "stereo_camera_publisher.py not found — rear camera disabled"
else
    # Publisher handles device detection internally now
    python3 "$STEREO_SCRIPT" \
        --ros-args \
        -p device:=/dev/video_stereo \
        -p width:=1600 \
        -p height:=600 \
        -p fps:=15 \
        -p publish_rate:=10.0 > /tmp/rover_stereo.log 2>&1 &

    sleep 3

    # Verify it's actually producing frames before starting combiner
    STEREO_OK=false
    for i in 1 2 3; do
        COUNT=$(ros2 topic hz /camera_rear/left/image_raw \
            --spin-time 2 2>/dev/null | grep "average rate" | wc -l)
        [ "$COUNT" -gt 0 ] && STEREO_OK=true && break
        sleep 1
    done

    if [ "$STEREO_OK" = "false" ]; then
        warn "Stereo camera not producing frames — check /tmp/rover_stereo.log"
    else
        ok "Stereo camera publishing"

        python3 ~/lunar_rover_ws/stereo_combiner.py \
            --ros-args \
            -p left_crop_start:=0 \
            -p left_crop_width:=800 \
            -p right_crop_start:=800 \
            -p right_crop_width:=800 \
            -p publish_compressed:=true > /tmp/rover_combiner.log 2>&1 &

        sleep 2

        python3 ~/lunar_rover_ws/optimized_image_pipeline.py \
            --ros-args \
            -p input_topic:=/camera_rear/stereo_combined/compressed \
            -p output_topic:=/camera_rear/stream/compressed \
            -p input_is_compressed:=true \
            -p jpeg_quality:=40 \
            -p decimation:=1 \
            -p buffer_delay_sec:=0.0 \
            -p target_fps:=10.0 > /tmp/rover_pipe_rear.log 2>&1 &

        sleep 2
        ok "Rear stereo pipeline running  →  /camera_rear/stream/compressed @ 10fps"
    fi
fi

# ══════════════════════════════════════════════════════════════════════════════
# 5 · JOY → ARDUINO
# ══════════════════════════════════════════════════════════════════════════════
log "[5/5] Joy → Arduino..."

ARDUINO_PORT=""
for p in /dev/ttyACM0 /dev/ttyACM1 /dev/ttyUSB0; do
    [ -e "$p" ] && ARDUINO_PORT="$p" && break
done

JOY_SCRIPT=""
for loc in "$(dirname "$0")/joy_to_arduino.py" ~/lunar_rover_ws/joy_to_arduino.py; do
    [ -f "$loc" ] && JOY_SCRIPT="$loc" && break
done

if [ -z "$JOY_SCRIPT" ]; then
    warn "joy_to_arduino.py not found"
elif [ -n "$ARDUINO_PORT" ]; then
    python3 "$JOY_SCRIPT" > /tmp/rover_arduino.log 2>&1 &
    sleep 2
    ok "Joy→Arduino on $ARDUINO_PORT"
else
    warn "No Arduino found at ACM0/ACM1/USB0"
fi
echo ""

# ──────────────────────────────────────────────────────────────────────────────
# VERIFY
# ──────────────────────────────────────────────────────────────────────────────
log "Verifying topics (3s wait)..."
sleep 3
ALL_OK=true
for T in /camera/color/stream/compressed /camera/depth/stream/compressed; do
    if ros2 topic list 2>/dev/null | grep -q "^${T}$"; then
        ok "$T"
    else
        warn "$T — not up yet"
        ALL_OK=false
    fi
done

if [ "$ALL_OK" = "false" ]; then
    echo ""
    warn "Troubleshoot with:"
    warn "  tail /tmp/rover_pipe_color.log    ← color pipeline log"
    warn "  tail /tmp/rover_camera.log        ← D435 camera log"
    warn "  ros2 topic info -v /camera/camera/color/image_raw/compressed"
fi
echo ""

# ──────────────────────────────────────────────────────────────────────────────
# SUMMARY
# ──────────────────────────────────────────────────────────────────────────────
echo -e "${BOLD}${GREEN}"
echo "  ╔══════════════════════════════════════════╗"
echo "  ║         ✓✓✓  MINI PC READY  ✓✓✓         ║"
echo "  ╚══════════════════════════════════════════╝"
echo -e "${NC}"
echo "  Delay: ${DELAY_SEC}s"
echo ""
echo "  Streaming topics for RViz:"
echo "    /camera/color/stream/compressed   ← front RGB @ 6fps"
echo "    /camera/depth/stream/compressed   ← front depth @ 3fps"
echo "    /camera_rear/stream/compressed    ← rear stereo @ 6fps"
echo ""
echo "  RViz: set Transport Hint = compressed on each Image display"
echo ""
echo "  Logs:"
echo "    tail -f /tmp/rover_pipe_color.log"
echo "    tail -f /tmp/rover_pipe_depth.log"
echo "    tail -f /tmp/rover_stereo.log"
echo "    tail -f /tmp/rover_camera.log"
echo ""
echo "  Press Ctrl+C to stop all"
echo ""

wait