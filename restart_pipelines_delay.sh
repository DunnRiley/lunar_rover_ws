#!/bin/bash
# ============================================================================
# restart_pipelines_delay.sh  —  runs on MINI PC
#
# Kills the existing optimized_image_pipeline instances and relaunches
# them with a new buffer_delay_sec value.
#
# Usage:
#   bash ~/lunar_rover_ws/restart_pipelines_delay.sh 5.0   # enable delay
#   bash ~/lunar_rover_ws/restart_pipelines_delay.sh 0.0   # disable delay
#
# Called automatically by rover_control_gui.py when the delay toggle
# is flipped.  Also safe to run manually from a terminal.
# ============================================================================

DELAY_SEC="${1:-0.0}"
WS="$HOME/lunar_rover_ws"

# ── Source ROS ───────────────────────────────────────────────────────────────
set +u
if   [ -f /opt/ros/jazzy/setup.bash ];  then source /opt/ros/jazzy/setup.bash
elif [ -f /opt/ros/humble/setup.bash ]; then source /opt/ros/humble/setup.bash
else echo "[delay] ERROR: no ROS2 installation found"; exit 1
fi
[ -f "$WS/install/setup.bash" ] && source "$WS/install/setup.bash"
set -u

export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET

echo "[delay] Stopping existing image pipelines..."
# Kill ALL optimized_image_pipeline instances regardless of topic
pkill -f "optimized_image_pipeline" 2>/dev/null
sleep 1.5

echo "[delay] Starting colour pipeline  (delay=${DELAY_SEC}s)..."
python3 "$WS/optimized_image_pipeline.py" \
    --ros-args \
    -p input_topic:=/camera/camera/color/image_raw \
    -p output_topic:=/camera/color/stream/compressed \
    -p input_is_compressed:=false \
    -p input_reliable:=false \
    -p jpeg_quality:=30 \
    -p decimation:=5 \
    -p buffer_delay_sec:="${DELAY_SEC}" \
    -p target_fps:=6.0 > /tmp/rover_pipe_color.log 2>&1 &

echo "[delay] Starting depth pipeline   (delay=${DELAY_SEC}s)..."
python3 "$WS/optimized_image_pipeline.py" \
    --ros-args \
    -p input_topic:=/camera/camera/aligned_depth_to_color/image_raw \
    -p output_topic:=/camera/depth/stream/compressed \
    -p input_is_compressed:=false \
    -p input_reliable:=false \
    -p jpeg_quality:=50 \
    -p decimation:=10 \
    -p buffer_delay_sec:="${DELAY_SEC}" \
    -p target_fps:=3.0 > /tmp/rover_pipe_depth.log 2>&1 &

sleep 1

# Verify both came up
COLOR_OK=false
DEPTH_OK=false
for i in 1 2 3 4 5; do
    sleep 1
    ros2 topic list 2>/dev/null | grep -q "^/camera/color/stream/compressed$"  && COLOR_OK=true
    ros2 topic list 2>/dev/null | grep -q "^/camera/depth/stream/compressed$"  && DEPTH_OK=true
    [ "$COLOR_OK" = "true" ] && [ "$DEPTH_OK" = "true" ] && break
done

if [ "$COLOR_OK" = "true" ] && [ "$DEPTH_OK" = "true" ]; then
    echo "[delay] ✓ Pipelines running  buffer_delay_sec=${DELAY_SEC}"
else
    echo "[delay] ✗ Pipeline verify failed  color=$COLOR_OK depth=$DEPTH_OK"
    echo "[delay]   Check: tail /tmp/rover_pipe_color.log"
    echo "[delay]   Check: tail /tmp/rover_pipe_depth.log"
    exit 1
fi