#!/bin/bash
# ========================================================================
# LAPTOP LAUNCH  ·  full_launch_laptop.sh
# Optionally SSH-starts the mini PC, then opens the GUI control panel.
#
# Usage:
#   bash full_launch_laptop.sh                  # GUI only (miniPC already running)
#   bash full_launch_laptop.sh --start-minipc   # SSH-start miniPC first
# ========================================================================

CYAN='\033[0;36m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'
RED='\033[0;31m'; BOLD='\033[1m'; NC='\033[0m'

log()  { echo -e "${CYAN}[laptop]${NC} $1"; }
ok()   { echo -e "${GREEN}  ✓${NC} $1"; }
warn() { echo -e "${YELLOW}  ⚠${NC} $1"; }
err()  { echo -e "${RED}  ✗${NC} $1"; }

# ── CONFIG — edit these if your network changes ──────────────────────────
MINIPC_USER="cheese"
MINIPC_IP="192.168.0.102"
MINIPC_WS="~/lunar_rover_ws"
MINIPC_LAUNCH="bash ~/lunar_rover_ws/full_launch_minipc.sh"
GUI_SCRIPT="$(dirname "$0")/rover_control_gui.py"
TELEOP_SCRIPT="$(dirname "$0")/arduino_teleop_controller.py"
# ─────────────────────────────────────────────────────────────────────────

echo -e "${BOLD}"
echo "  ╔══════════════════════════════════════════╗"
echo "  ║     LUNAR ROVER  ·  Laptop Launch        ║"
echo "  ╚══════════════════════════════════════════╝"
echo -e "${NC}"

# ── ROS2 setup ──────────────────────────────────────────────────────────
set +u  # ROS2 setup scripts use unbound variables
if   [ -f /opt/ros/jazzy/setup.bash ];  then source /opt/ros/jazzy/setup.bash  && ok "ROS2 Jazzy"
elif [ -f /opt/ros/humble/setup.bash ]; then source /opt/ros/humble/setup.bash && ok "ROS2 Humble"
else err "No ROS2 installation found!" && exit 1
fi

[ -f ~/lunar_rover_ws/install/setup.bash ] && \
    source ~/lunar_rover_ws/install/setup.bash && ok "Workspace overlay sourced"
set -u  # re-enable unbound variable check

export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
ok "Network: ROS_DOMAIN_ID=42, SUBNET discovery"
echo ""

# ── Optional: SSH-start mini PC ─────────────────────────────────────────
if [ "$1" = "--start-minipc" ]; then
    log "SSH-starting mini PC at ${MINIPC_USER}@${MINIPC_IP}..."

    # Test SSH connectivity first
    if ssh -o ConnectTimeout=5 -o BatchMode=yes \
           "${MINIPC_USER}@${MINIPC_IP}" "echo ok" > /dev/null 2>&1; then
        ok "SSH connection verified"

        # Launch on miniPC in background (nohup keeps it alive after SSH exits)
        ssh "${MINIPC_USER}@${MINIPC_IP}" \
            "nohup bash ${MINIPC_WS}/full_launch_minipc.sh > /tmp/minipc_launch.log 2>&1 &"

        log "Mini PC launch triggered — waiting 8s for nodes to start..."
        sleep 8
        ok "Mini PC should be up"
    else
        warn "Cannot SSH to ${MINIPC_USER}@${MINIPC_IP}"
        warn "Check: both on same WiFi? SSH keys set up?"
        warn "Continuing anyway — you can start miniPC manually"
    fi
    echo ""
fi

# ── Check for PyQt5 ─────────────────────────────────────────────────────
log "Checking Python dependencies..."
if python3 -c "import PyQt5" 2>/dev/null; then
    ok "PyQt5 available"
else
    warn "PyQt5 not found — installing..."
    pip3 install PyQt5 --break-system-packages 2>/dev/null || \
        sudo apt-get install -y python3-pyqt5 2>/dev/null
    python3 -c "import PyQt5" 2>/dev/null && ok "PyQt5 installed" || \
        { err "PyQt5 install failed. Run: sudo apt install python3-pyqt5"; exit 1; }
fi

# ── Locate scripts ───────────────────────────────────────────────────────
if [ ! -f "$GUI_SCRIPT" ]; then
    GUI_SCRIPT="$(find ~/lunar_rover_ws -name rover_control_gui.py 2>/dev/null | head -1)"
fi
if [ ! -f "$GUI_SCRIPT" ]; then
    err "rover_control_gui.py not found!"
    err "Copy it to the same directory as this script or to ~/lunar_rover_ws/"
    exit 1
fi

if [ ! -f "$TELEOP_SCRIPT" ]; then
    TELEOP_SCRIPT="$(find ~/lunar_rover_ws -name arduino_teleop_controller.py 2>/dev/null | head -1)"
fi

ok "GUI script:    $GUI_SCRIPT"
[ -f "$TELEOP_SCRIPT" ] && ok "Teleop script: $TELEOP_SCRIPT"
echo ""

# ── Check ros-jazzy-joy (or ros-humble-joy) is installed ─────────────────
if ! ros2 pkg list 2>/dev/null | grep -q "^joy$"; then
    warn "joy package not found — installing..."
    sudo apt-get install -y ros-$(ros2 --version 2>/dev/null | grep -oP 'jazzy|humble' | head -1)-joy 2>/dev/null \
        && ok "joy installed" \
        || { err "Could not install joy. Run: sudo apt install ros-jazzy-joy"; }
fi

# ── Start joy_node (controller → /joy topic) ─────────────────────────────
echo -e "${BOLD}${CYAN}"
echo "  ╔══════════════════════════════════════════╗"
echo "  ║       Starting Controller Input          ║"
echo "  ╚══════════════════════════════════════════╝"
echo -e "${NC}"

log "Starting joy_node (USB gamepad → /joy)..."
ros2 run joy joy_node > /tmp/rover_joy.log 2>&1 &
JOY_PID=$!
sleep 1

if kill -0 $JOY_PID 2>/dev/null; then
    ok "joy_node running (PID $JOY_PID)  — plug in controller now if not already"
else
    warn "joy_node failed to start — check: sudo apt install ros-jazzy-joy"
    warn "Controller teleop will not work, but GUI will still open"
fi

# ── Start controller teleop node (/joy → /cmd_vel + /actuator_cmd) ───────
if [ -f "$TELEOP_SCRIPT" ]; then
    log "Starting controller teleop node..."
    python3 "$TELEOP_SCRIPT" > /tmp/rover_teleop.log 2>&1 &
    TELEOP_PID=$!
    sleep 1
    if kill -0 $TELEOP_PID 2>/dev/null; then
        ok "Controller teleop running (PID $TELEOP_PID)"
        ok "  Left stick = drive  ·  Right stick = turn"
        ok "  A=extend  B=retract  Start=e-stop  RB/LB=speed"
    else
        warn "Teleop node failed — check /tmp/rover_teleop.log"
    fi
else
    warn "arduino_teleop_controller.py not found — skipping controller teleop"
fi

# ── Cleanup on exit ───────────────────────────────────────────────────────
cleanup() {
    log "Shutting down..."
    [ -n "$JOY_PID" ]    && kill $JOY_PID    2>/dev/null
    [ -n "$TELEOP_PID" ] && kill $TELEOP_PID 2>/dev/null
}
trap cleanup EXIT INT TERM

# ── Launch GUI ───────────────────────────────────────────────────────────
echo -e "${BOLD}${GREEN}"
echo "  ╔══════════════════════════════════════════╗"
echo "  ║       Launching Mission Control GUI      ║"
echo "  ╚══════════════════════════════════════════╝"
echo -e "${NC}"

exec python3 "$GUI_SCRIPT"