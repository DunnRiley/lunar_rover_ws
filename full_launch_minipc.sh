#!/bin/bash
# ========================================================================
# MINI PC UNIFIED LAUNCH  ·  full_launch_minipc.sh
# Starts: TF tree, D435 front camera, image pipeline, rear stereo,
#         Arduino motor controller
#
# Usage:
#   bash full_launch_minipc.sh                      # live / test mode
#   DELAY_SEC=1.0 bash full_launch_minipc.sh        # competition delay
# ========================================================================

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

# ── ROS2 setup ──────────────────────────────────────────────────────────
# Temporarily disable 'unbound variable' check — ROS2 setup scripts use unset vars
set +u
if   [ -f /opt/ros/jazzy/setup.bash ];  then source /opt/ros/jazzy/setup.bash  && ok "ROS2 Jazzy"
elif [ -f /opt/ros/humble/setup.bash ]; then source /opt/ros/humble/setup.bash && ok "ROS2 Humble"
else err "No ROS2 installation found!" && exit 1
fi

[ -f install/setup.bash ] && source install/setup.bash && ok "Workspace overlay sourced"
set -u  # re-enable unbound variable check

export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
ok "Network: ROS_DOMAIN_ID=42, SUBNET discovery"
echo ""

# ── Competition delay ────────────────────────────────────────────────────
DELAY_SEC=${DELAY_SEC:-0.0}
if [ "$(echo "$DELAY_SEC > 0" | bc -l 2>/dev/null || echo 0)" = "1" ]; then
    echo -e "${YELLOW}  ⏱  COMPETITION MODE: ${DELAY_SEC}s delay buffer ENABLED${NC}"
else
    log "Test mode: live streaming (no delay)"
fi
echo ""

# ── Kill stale processes ─────────────────────────────────────────────────
log "Stopping stale nodes..."
pkill -f "realsense2_camera_node"     2>/dev/null
pkill -f "optimized_image_pipeline"   2>/dev/null
pkill -f "stereo_camera_publisher"    2>/dev/null
pkill -f "stereo_combiner"            2>/dev/null
pkill -f "robot_state_publisher"      2>/dev/null
pkill -f "static_transform_publisher" 2>/dev/null
pkill -f "arduino_motor_controller"   2>/dev/null
sleep 2
ok "Clean"
echo ""

trap 'echo ""; log "Shutting down all nodes..."; kill 0; exit' SIGINT SIGTERM

# ════════════════════════════════════════════════════════════════════════
# 1 · ROBOT DESCRIPTION + TF TREE
# ════════════════════════════════════════════════════════════════════════
log "[1/5] Robot State Publisher + TF tree..."

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
ok "TF tree ready"
echo ""

# ════════════════════════════════════════════════════════════════════════
# 2 · FRONT D435 CAMERA
# ════════════════════════════════════════════════════════════════════════
log "[2/5] D435 Front Camera (424×240 @ 30 fps)..."

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
log "  Waiting 6s for camera to initialize..."
sleep 6

if ps -p $CAM_PID > /dev/null 2>&1; then
    ok "D435 running (PID $CAM_PID)"
else
    warn "D435 may have failed — check: tail /tmp/rover_camera.log"
    warn "Tip: run 'rs-enumerate-devices' to verify camera is detected"
fi
echo ""

# ════════════════════════════════════════════════════════════════════════
# 3 · FRONT CAMERA STREAMING PIPELINE
# ════════════════════════════════════════════════════════════════════════
log "[3/5] Image streaming pipeline..."

if [ ! -f ~/lunar_rover_ws/optimized_image_pipeline.py ]; then
    warn "optimized_image_pipeline.py not found in ~/lunar_rover_ws/ — skipping"
else
    # Color stream: 6 fps, JPEG q=25
    python3 ~/lunar_rover_ws/optimized_image_pipeline.py \
        --ros-args \
        -p input_topic:=/camera/camera/color/image_raw/compressed \
        -p output_topic:=/camera/color/stream/compressed \
        -p input_is_compressed:=true \
        -p jpeg_quality:=25 \
        -p decimation:=5 \
        -p buffer_delay_sec:=$DELAY_SEC \
        -p target_fps:=6.0 \
        -p resize_factor:=1.0 > /tmp/rover_pipe_color.log 2>&1 &

    sleep 1

    # Depth stream: 3 fps
    python3 ~/lunar_rover_ws/optimized_image_pipeline.py \
        --ros-args \
        -p input_topic:=/camera/camera/aligned_depth_to_color/image_raw \
        -p output_topic:=/camera/depth/stream/compressed \
        -p input_is_compressed:=false \
        -p jpeg_quality:=50 \
        -p decimation:=10 \
        -p buffer_delay_sec:=$DELAY_SEC \
        -p target_fps:=3.0 > /tmp/rover_pipe_depth.log 2>&1 &

    sleep 1
    ok "Image pipeline running  →  /camera/color/stream/compressed @ 6fps"
fi
echo ""

# ════════════════════════════════════════════════════════════════════════
# 4 · REAR STEREO CAMERA
# ════════════════════════════════════════════════════════════════════════
log "[4/5] Rear stereo camera..."

STEREO_SCRIPT=""
for candidate in \
    ~/lunar_rover_ws/DiagnosticAndTesting/stereo_camera_publisher.py \
    ~/lunar_rover_ws/stereo_camera_publisher.py; do
    [ -f "$candidate" ] && STEREO_SCRIPT="$candidate" && break
done

if [ -n "$STEREO_SCRIPT" ]; then
    python3 "$STEREO_SCRIPT" \
        --ros-args \
        -p device:=/dev/video0 \
        -p width:=480 \
        -p height:=180 \
        -p fps:=15 \
        -p publish_rate:=15.0 > /tmp/rover_stereo.log 2>&1 &
    sleep 2

    if [ -f ~/lunar_rover_ws/stereo_combiner.py ]; then
        python3 ~/lunar_rover_ws/stereo_combiner.py \
            --ros-args \
            -p left_crop_start:=0 \
            -p left_crop_width:=240 \
            -p right_crop_start:=240 \
            -p right_crop_width:=240 \
            -p publish_compressed:=true > /tmp/rover_combiner.log 2>&1 &
        sleep 1
    fi

    if [ -f ~/lunar_rover_ws/optimized_image_pipeline.py ]; then
        python3 ~/lunar_rover_ws/optimized_image_pipeline.py \
            --ros-args \
            -p input_topic:=/camera_rear/stereo_combined/compressed \
            -p output_topic:=/camera_rear/stream/compressed \
            -p input_is_compressed:=true \
            -p jpeg_quality:=30 \
            -p decimation:=3 \
            -p buffer_delay_sec:=$DELAY_SEC \
            -p target_fps:=6.0 > /tmp/rover_pipe_rear.log 2>&1 &
        sleep 1
    fi
    ok "Rear stereo pipeline running  →  /camera_rear/stream/compressed @ 6fps"
else
    warn "stereo_camera_publisher.py not found — rear camera disabled"
fi
echo ""

# ════════════════════════════════════════════════════════════════════════
# 5 · JOY → ARDUINO  (direct serial, replaces arduino_motor_controller)
# ════════════════════════════════════════════════════════════════════════
log "[5/5] Joy-to-Arduino bridge..."

ARDUINO_PORT=""
for p in /dev/ttyACM0 /dev/ttyACM1 /dev/ttyUSB0; do
    [ -e "$p" ] && ARDUINO_PORT="$p" && break
done

JOY_SCRIPT=""
for loc in "$(dirname "$0")/joy_to_arduino.py" ~/lunar_rover_ws/joy_to_arduino.py; do
    [ -f "$loc" ] && JOY_SCRIPT="$loc" && break
done

if [ -z "$JOY_SCRIPT" ]; then
    warn "joy_to_arduino.py not found — copy it to ~/lunar_rover_ws/"
    warn "Motor control will NOT work"
elif [ -n "$ARDUINO_PORT" ]; then
    python3 "$JOY_SCRIPT" > /tmp/rover_arduino.log 2>&1 &
    sleep 2
    ok "Joy→Arduino bridge running on $ARDUINO_PORT"
    ok "Subscribing to /joy from laptop · direct serial · 20Hz rate-limited"
else
    warn "No Arduino found at /dev/ttyACM0, /dev/ttyACM1, or /dev/ttyUSB0"
    warn "Motor controller NOT started — check USB cable"
fi
echo ""

# ════════════════════════════════════════════════════════════════════════
# SUMMARY
# ════════════════════════════════════════════════════════════════════════
echo -e "${BOLD}${GREEN}"
echo "  ╔══════════════════════════════════════════╗"
echo "  ║         ✓✓✓  MINI PC READY  ✓✓✓         ║"
echo "  ╚══════════════════════════════════════════╝"
echo -e "${NC}"
echo "  Delay mode : ${DELAY_SEC}s"
echo ""
echo "  Camera streams (RViz on laptop):"
echo "    /camera/color/stream/compressed    (6 fps)"
echo "    /camera/depth/stream/compressed    (3 fps)"
echo "    /camera_rear/stream/compressed     (6 fps)"
echo ""
echo "  Motor control:"
echo "    /cmd_vel  ← laptop publishes Twist, Arduino drives"
echo ""
echo "  Logs:  tail -f /tmp/rover_*.log"
echo ""
[ "$(echo "$DELAY_SEC > 0" | bc -l 2>/dev/null || echo 0)" = "1" ] \
    && echo -e "  ${YELLOW}⏱  Competition delay ${DELAY_SEC}s active${NC}" \
    || echo "  💡 Competition mode: DELAY_SEC=1.0 bash full_launch_minipc.sh"
echo ""
echo "  Press Ctrl+C to stop all nodes"
echo "  ════════════════════════════════════════════"

wait