#!/bin/bash
# ============================================================================
#  MINI PC — FULL ROVER LAUNCH
#  Starts: TF tree, cameras, image pipeline, motor controller
#
#  USAGE:
#    bash full_launch_minipc.sh           → Normal mode (no delay)
#    DELAY_SEC=1.0 bash full_launch_minipc.sh  → Competition delay mode
# ============================================================================

set -euo pipefail

DELAY_SEC=${DELAY_SEC:-0.0}

# ── Colours ──────────────────────────────────────────────────────────────────
RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'
CYAN='\033[0;36m'; BOLD='\033[1m'; NC='\033[0m'

log()  { echo -e "${CYAN}[miniPC]${NC} $*"; }
ok()   { echo -e "${GREEN}  ✓${NC} $*"; }
warn() { echo -e "${YELLOW}  ⚠${NC}  $*"; }
die()  { echo -e "${RED}  ✗${NC} $* — aborting"; kill 0 2>/dev/null; exit 1; }

echo ""
echo -e "${BOLD}╔══════════════════════════════════════════════╗${NC}"
echo -e "${BOLD}║       LUNAR ROVER — MINI PC LAUNCH           ║${NC}"
echo -e "${BOLD}╚══════════════════════════════════════════════╝${NC}"
echo ""

# ── ROS2 setup ───────────────────────────────────────────────────────────────
if   [ -f /opt/ros/jazzy/setup.bash ];  then source /opt/ros/jazzy/setup.bash  && ok "ROS2 Jazzy"
elif [ -f /opt/ros/humble/setup.bash ]; then source /opt/ros/humble/setup.bash && ok "ROS2 Humble"
else die "No ROS2 installation found"; fi

[ -f ~/lunar_rover_ws/install/setup.bash ] && source ~/lunar_rover_ws/install/setup.bash && ok "Workspace sourced"

export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
ok "Network: DOMAIN_ID=42, SUBNET discovery"

if (( $(echo "$DELAY_SEC > 0" | bc -l) )); then
    echo -e "${YELLOW}  ⏱  COMPETITION MODE: ${DELAY_SEC}s delay buffer${NC}"
else
    ok "Test mode: live streaming (no delay)"
fi
echo ""

# ── Kill old processes ────────────────────────────────────────────────────────
log "Stopping stale processes..."
pkill -f "realsense2_camera_node"       2>/dev/null || true
pkill -f "optimized_image_pipeline"     2>/dev/null || true
pkill -f "stereo_camera_publisher"      2>/dev/null || true
pkill -f "stereo_combiner"              2>/dev/null || true
pkill -f "robot_state_publisher"        2>/dev/null || true
pkill -f "static_transform_publisher"   2>/dev/null || true
pkill -f "arduino_motor_controller"     2>/dev/null || true
sleep 2
ok "Clean"
echo ""

trap 'echo ""; log "Shutting down mini PC…"; kill 0 2>/dev/null; wait; log "Done"; exit' \
    SIGINT SIGTERM EXIT

# ── URDF / robot description ──────────────────────────────────────────────────
URDF_CONTENT='<?xml version="1.0"?>
<robot name="lunar_rover">
  <link name="base_link"/>
  <link name="camera_link"/>
  <link name="camera_rear_link"/>
  <joint name="base_to_camera" type="fixed">
    <parent link="base_link"/>
    <child link="camera_link"/>
    <origin xyz="0.2 0 0.15" rpy="0 0 0"/>
  </joint>
  <joint name="base_to_camera_rear" type="fixed">
    <parent link="base_link"/>
    <child link="camera_rear_link"/>
    <origin xyz="-0.2 0 0.15" rpy="0 0 3.14159"/>
  </joint>
</robot>'

# ── [1/5] Robot description + TF ─────────────────────────────────────────────
log "[1/5] Robot State Publisher + TF…"

ros2 run robot_state_publisher robot_state_publisher \
    --ros-args -p robot_description:="$URDF_CONTENT" \
    2>/tmp/rover_rsp.log &

sleep 1

ros2 run tf2_ros static_transform_publisher \
    0 0 0 -1.5707963267948966 0 -1.5707963267948966 \
    camera_link camera_color_optical_frame \
    2>/dev/null &

ros2 run tf2_ros static_transform_publisher \
    0 0 0 -1.5707963267948966 0 -1.5707963267948966 \
    camera_link camera_depth_optical_frame \
    2>/dev/null &

sleep 1
ok "TF tree ready"
echo ""

# ── [2/5] D435 front camera ───────────────────────────────────────────────────
log "[2/5] D435 Front Camera (424x240 @ 30fps)…"

ros2 launch realsense2_camera rs_launch.py \
    camera_name:=camera camera_namespace:=camera \
    enable_depth:=true enable_color:=true \
    pointcloud.enable:=false align_depth.enable:=true \
    enable_sync:=true \
    depth_module.profile:=424x240x30 rgb_camera.profile:=424x240x30 \
    2>/tmp/rover_camera.log &

CAM_PID=$!
log "  Waiting 6s for camera…"
sleep 6

if ps -p $CAM_PID > /dev/null 2>&1; then
    ok "D435 running (PID $CAM_PID)"
else
    warn "D435 failed to start — check USB: tail /tmp/rover_camera.log"
fi
echo ""

# ── [3/5] Front camera streaming pipeline ────────────────────────────────────
log "[3/5] Front Camera Streaming Pipeline…"

PIPE="$HOME/lunar_rover_ws/optimized_image_pipeline.py"
if [ -f "$PIPE" ]; then
    # RGB stream
    python3 "$PIPE" --ros-args \
        -p input_topic:=/camera/camera/color/image_raw/compressed \
        -p output_topic:=/camera/color/stream/compressed \
        -p input_is_compressed:=true \
        -p jpeg_quality:=25 -p decimation:=5 \
        -p buffer_delay_sec:="$DELAY_SEC" -p target_fps:=6.0 \
        2>/tmp/rover_pipe_rgb.log &
    sleep 1

    # Depth stream
    python3 "$PIPE" --ros-args \
        -p input_topic:=/camera/camera/aligned_depth_to_color/image_raw \
        -p output_topic:=/camera/depth/stream/compressed \
        -p input_is_compressed:=false \
        -p jpeg_quality:=50 -p decimation:=10 \
        -p buffer_delay_sec:="$DELAY_SEC" -p target_fps:=3.0 \
        2>/tmp/rover_pipe_depth.log &
    sleep 1
    ok "Front camera pipeline running"
else
    warn "optimized_image_pipeline.py not found — skipping front camera pipeline"
fi
echo ""

# ── [4/5] Rear stereo camera ──────────────────────────────────────────────────
log "[4/5] Rear Stereo Camera…"

STEREO_SCRIPT=""
for c in \
    "$HOME/lunar_rover_ws/DiagnosticAndTesting/stereo_camera_publisher.py" \
    "$HOME/lunar_rover_ws/stereo_camera_publisher.py"; do
    [ -f "$c" ] && STEREO_SCRIPT="$c" && break
done

if [ -n "$STEREO_SCRIPT" ]; then
    python3 "$STEREO_SCRIPT" --ros-args \
        -p device:=/dev/video0 -p width:=480 -p height:=180 \
        -p fps:=15 -p publish_rate:=15.0 \
        2>/tmp/rover_stereo.log &
    sleep 2

    COMBINER="$HOME/lunar_rover_ws/stereo_combiner.py"
    if [ -f "$COMBINER" ]; then
        python3 "$COMBINER" --ros-args \
            -p left_crop_start:=0 -p left_crop_width:=240 \
            -p right_crop_start:=240 -p right_crop_width:=240 \
            -p publish_compressed:=true \
            2>/tmp/rover_combiner.log &
        sleep 1
    fi

    if [ -f "$PIPE" ]; then
        python3 "$PIPE" --ros-args \
            -p input_topic:=/camera_rear/stereo_combined/compressed \
            -p output_topic:=/camera_rear/stream/compressed \
            -p input_is_compressed:=true \
            -p jpeg_quality:=30 -p decimation:=3 \
            -p buffer_delay_sec:="$DELAY_SEC" -p target_fps:=6.0 \
            2>/tmp/rover_pipe_rear.log &
        sleep 1
    fi
    ok "Rear stereo pipeline running"
else
    warn "stereo_camera_publisher.py not found — rear camera disabled"
fi
echo ""

# ── [5/5] Arduino motor controller ───────────────────────────────────────────
log "[5/5] Arduino Motor Controller…"

# Detect Arduino port
ARDUINO_PORT=""
for p in /dev/ttyACM0 /dev/ttyACM1 /dev/ttyUSB0; do
    [ -e "$p" ] && ARDUINO_PORT="$p" && break
done

if [ -n "$ARDUINO_PORT" ]; then
    ros2 run lunar_robot_hardware arduino_motor_controller \
        --ros-args -p arduino_port:="$ARDUINO_PORT" -p baudrate:=115200 \
        2>/tmp/rover_motors.log &
    sleep 2
    ok "Motor controller on $ARDUINO_PORT"
else
    warn "No Arduino detected (/dev/ttyACM0, /dev/ttyACM1, /dev/ttyUSB0)"
    warn "Motor controller NOT started — teleop will have no effect"
fi
echo ""

# ── Summary ───────────────────────────────────────────────────────────────────
echo -e "${BOLD}╔══════════════════════════════════════════════╗${NC}"
echo -e "${BOLD}║       ✓  MINI PC READY                       ║${NC}"
echo -e "${BOLD}╚══════════════════════════════════════════════╝${NC}"
echo ""
echo "  Camera topics:"
echo "    /camera/color/stream/compressed     ← Front RGB  (6 fps)"
echo "    /camera/depth/stream/compressed     ← Front Depth (3 fps)"
echo "    /camera_rear/stream/compressed      ← Rear stereo (6 fps)"
echo ""
echo "  Motor topic: /cmd_vel  →  Arduino"
echo ""
if (( $(echo "$DELAY_SEC > 0" | bc -l) )); then
    echo -e "  ${YELLOW}⏱  Competition delay: ${DELAY_SEC}s${NC}"
fi
echo "  Logs: /tmp/rover_*.log"
echo ""
echo "  Press Ctrl+C to stop all nodes"
echo ""

wait