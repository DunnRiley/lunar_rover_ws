#!/bin/bash
# ============================================================================
# full_launch_autonomous.sh  —  MINI PC
#
# Starts all nodes needed for autonomous mission sequencer operation.
# Run this INSTEAD of (or alongside) full_launch_minipc.sh.
#
# Usage:
#   bash ~/lunar_rover_ws/full_launch_autonomous.sh
#   bash ~/lunar_rover_ws/full_launch_autonomous.sh --mission mission.yaml
#
# The --mission flag pre-loads a mission file so you can trigger it
# immediately from the GUI or:
#   ros2 topic pub /mission/start std_msgs/msg/Bool "data: true" --once
#
# ── What this starts ─────────────────────────────────────────────────────
#
#   1. nav_arduino_bridge.py
#        Reads DIST_DONE/DIST_TIMEOUT from Arduino Serial.
#        Converts /nav/arduino_dist_cmd to 0xDC serial packets.
#        Publishes /imu/gyro_deg_s and /imu/accel_ms2 from Serial2.
#
#   2. arduino_motor_controller  (lunar_robot_hardware)
#        Translates /cmd_vel Twist messages to DriveLeft/DriveRight packets.
#        Used by the sequencer for turns.
#
#   3. nav_sensor_fusion.py
#        Integrates IMU gyro to /nav/heading_deg.
#        Provides /nav/fused_state JSON for the sequencer.
#
#   4. nav_mission_sequencer.py  (if --mission is given)
#        Loads the mission YAML and waits for /mission/start True.
#
# ── NOT started by this script ───────────────────────────────────────────
#
#   - joy_to_arduino.py  (run separately for joystick control)
#   - Cameras / image pipelines  (run full_launch_minipc.sh for those)
#
# ============================================================================

CYAN='\033[0;36m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'
RED='\033[0;31m'; BOLD='\033[1m'; NC='\033[0m'

log()  { echo -e "${CYAN}[auto]${NC} $1"; }
ok()   { echo -e "${GREEN}  ✓${NC} $1"; }
warn() { echo -e "${YELLOW}  ⚠${NC} $1"; }
err()  { echo -e "${RED}  ✗${NC} $1"; }

# ── Parse args ────────────────────────────────────────────────────────────
MISSION_FILE=""
for arg in "$@"; do
    case "$arg" in
        --mission)  shift; MISSION_FILE="$1"; shift ;;
        --mission=*) MISSION_FILE="${arg#--mission=}" ;;
    esac
done

echo -e "${BOLD}"
echo "  ╔══════════════════════════════════════════════╗"
echo "  ║   LUNAR ROVER  ·  Autonomous Nav Launch      ║"
echo "  ╚══════════════════════════════════════════════╝"
echo -e "${NC}"

WS="$HOME/lunar_rover_ws"
cd "$WS" || { err "Cannot cd to $WS"; exit 1; }

# ── ROS2 setup ────────────────────────────────────────────────────────────
set +u
if   [ -f /opt/ros/jazzy/setup.bash ];  then source /opt/ros/jazzy/setup.bash  && ok "ROS2 Jazzy"
elif [ -f /opt/ros/humble/setup.bash ]; then source /opt/ros/humble/setup.bash && ok "ROS2 Humble"
else err "No ROS2 installation found!"; exit 1; fi
[ -f "$WS/install/setup.bash" ] && source "$WS/install/setup.bash" && ok "Workspace sourced"
set -u

export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
ok "ROS_DOMAIN_ID=42  SUBNET discovery"
echo ""

# ── Kill any stale instances ──────────────────────────────────────────────
log "Clearing stale autonomous nodes..."
pkill -f "nav_arduino_bridge"     2>/dev/null
pkill -f "nav_sensor_fusion"      2>/dev/null
pkill -f "nav_mission_sequencer"  2>/dev/null
pkill -f "arduino_motor_controller" 2>/dev/null
sleep 1

trap 'echo ""; log "Shutting down..."; kill 0; exit' SIGINT SIGTERM

# ── Auto-detect serial ports ──────────────────────────────────────────────
CMD_PORT=$(ls /dev/ttyACM* 2>/dev/null | sort | head -1)
TELEM_PORT=$(ls /dev/ttyUSB* 2>/dev/null | sort | head -1)

if [ -z "$CMD_PORT" ]; then
    warn "No /dev/ttyACM* found — Arduino not connected?"
    warn "  Bridge will start but won't be able to send commands."
    CMD_PORT="/dev/ttyACM0"
else
    ok "Command port: $CMD_PORT"
fi

if [ -z "$TELEM_PORT" ]; then
    warn "No /dev/ttyUSB* found — Serial2 not wired to USB-UART adapter."
    warn "  IMU and actuator encoder data will be unavailable."
    warn "  Turns will still work (IMU on Serial2) but heading feedback won't."
    TELEM_PORT=""
else
    ok "Telemetry port: $TELEM_PORT"
fi
echo ""

# ── 1. Arduino bridge ─────────────────────────────────────────────────────
log "[1/4] Arduino bridge (0xDC distance drive + DIST_DONE listener)..."

BRIDGE_ARGS="--ros-args -p cmd_port:=${CMD_PORT}"
[ -n "$TELEM_PORT" ] && BRIDGE_ARGS="$BRIDGE_ARGS -p telem_port:=${TELEM_PORT}"

python3 "$WS/nav_arduino_bridge.py" $BRIDGE_ARGS > /tmp/rover_bridge.log 2>&1 &
BRIDGE_PID=$!
sleep 2

if kill -0 $BRIDGE_PID 2>/dev/null; then
    ok "nav_arduino_bridge running (PID $BRIDGE_PID)"
    ok "  Logs: tail -f /tmp/rover_bridge.log"
else
    err "nav_arduino_bridge crashed — check /tmp/rover_bridge.log"
    tail -10 /tmp/rover_bridge.log
    exit 1
fi
echo ""

# ── 2. Arduino motor controller ───────────────────────────────────────────
log "[2/4] Arduino motor controller (/cmd_vel → DriveLeft/DriveRight)..."

# The motor controller uses the same serial port as joy_to_arduino.
# If joy_to_arduino is already running, the controller will share the
# /cmd_vel topic but the serial port will conflict.
# For autonomous-only runs: start the controller here.
# For joystick + autonomous: let joy_to_arduino handle serial directly.

if pgrep -f "arduino_motor_controller" > /dev/null 2>&1; then
    ok "arduino_motor_controller already running"
elif pgrep -f "joy_to_arduino" > /dev/null 2>&1; then
    warn "joy_to_arduino is running — it handles serial directly."
    warn "  /cmd_vel turn commands will go through joy_to_arduino's watchdog."
    warn "  arduino_motor_controller NOT started (would conflict on serial port)."
else
    ros2 run lunar_robot_hardware arduino_motor_controller \
        --ros-args -p arduino_port:="${CMD_PORT}" \
        > /tmp/rover_motor_ctrl.log 2>&1 &
    MOTOR_PID=$!
    sleep 2
    if kill -0 $MOTOR_PID 2>/dev/null; then
        ok "arduino_motor_controller running (PID $MOTOR_PID)"
    else
        warn "arduino_motor_controller failed — check /tmp/rover_motor_ctrl.log"
        warn "  Turns may not work. Drive-by-distance (0xDC) is unaffected."
        tail -5 /tmp/rover_motor_ctrl.log
    fi
fi
echo ""

# ── 3. Sensor fusion ──────────────────────────────────────────────────────
log "[3/4] Sensor fusion (IMU heading integration)..."

python3 "$WS/nav_sensor_fusion.py" > /tmp/rover_fusion.log 2>&1 &
FUSION_PID=$!
sleep 2

if kill -0 $FUSION_PID 2>/dev/null; then
    ok "nav_sensor_fusion running (PID $FUSION_PID)"
else
    warn "nav_sensor_fusion failed — check /tmp/rover_fusion.log"
    warn "  Turns will still run but heading feedback won't work."
    tail -5 /tmp/rover_fusion.log
fi
echo ""

# ── 4. Mission sequencer ──────────────────────────────────────────────────
log "[4/4] Mission sequencer..."

SEQ_ARGS=""
if [ -n "$MISSION_FILE" ]; then
    # Resolve path relative to WS if not absolute
    if [[ "$MISSION_FILE" != /* ]]; then
        MISSION_FILE="$WS/$MISSION_FILE"
    fi
    if [ -f "$MISSION_FILE" ]; then
        SEQ_ARGS="--ros-args -p mission_file:=${MISSION_FILE}"
        ok "Mission file: $MISSION_FILE"
    else
        warn "Mission file not found: $MISSION_FILE"
        warn "  Sequencer will start idle — use the GUI to load a file."
    fi
else
    warn "No --mission flag — sequencer starts idle."
    warn "  Load a mission via GUI or:"
    warn "    ros2 topic pub /mission/file std_msgs/msg/String 'data: \"/path/mission.yaml\"' --once"
fi

python3 "$WS/nav_mission_sequencer.py" $SEQ_ARGS > /tmp/rover_sequencer.log 2>&1 &
SEQ_PID=$!
sleep 2

if kill -0 $SEQ_PID 2>/dev/null; then
    ok "nav_mission_sequencer running (PID $SEQ_PID)"
    ok "  Logs: tail -f /tmp/rover_sequencer.log"
else
    err "nav_mission_sequencer crashed — check /tmp/rover_sequencer.log"
    tail -10 /tmp/rover_sequencer.log
fi
echo ""

# ── Verify topics ─────────────────────────────────────────────────────────
log "Verifying ROS topics (3 s)..."
sleep 3

for T in /nav/arduino_done /nav/fused_state /nav/heading_deg; do
    if ros2 topic list 2>/dev/null | grep -q "^${T}$"; then
        ok "$T"
    else
        warn "$T  — not yet (check relevant node)"
    fi
done
echo ""

echo -e "${BOLD}${GREEN}"
echo "  ╔══════════════════════════════════════════════╗"
echo "  ║        ✓  AUTONOMOUS STACK READY  ✓          ║"
echo "  ╚══════════════════════════════════════════════╝"
echo -e "${NC}"

if [ -n "$MISSION_FILE" ] && kill -0 $SEQ_PID 2>/dev/null; then
    echo "  Mission loaded:  $(basename "$MISSION_FILE")"
    echo "  TO START:"
    echo "    ros2 topic pub /mission/start std_msgs/msg/Bool 'data: true' --once"
    echo "    OR click START MISSION in the rover_control_gui"
    echo ""
fi

echo "  Log files:"
echo "    tail -f /tmp/rover_bridge.log"
echo "    tail -f /tmp/rover_fusion.log"
echo "    tail -f /tmp/rover_sequencer.log"
echo ""
echo "  To test distance drive (0.3 m forward):"
echo "    ros2 topic pub /nav/arduino_dist_cmd std_msgs/msg/Float32 'data: 0.3' --once"
echo "    ros2 topic echo /nav/arduino_done"
echo ""
echo "  Press Ctrl+C to stop all"
echo ""

wait