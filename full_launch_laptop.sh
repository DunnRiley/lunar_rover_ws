#!/bin/bash
# ========================================================================
# LAPTOP LAUNCH  ·  full_launch_laptop.sh
# Optionally SSH-starts the mini PC, then opens the GUI control panel
# and starts the navigation goal relay node.
#
# Usage:
#   bash full_launch_laptop.sh                  # GUI only (miniPC already running)
#   bash full_launch_laptop.sh --start-minipc   # SSH-start miniPC first
#   bash full_launch_laptop.sh --no-nav         # skip nav relay
# ========================================================================

CYAN='\033[0;36m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'
RED='\033[0;31m'; BOLD='\033[1m'; NC='\033[0m'

log()  { echo -e "${CYAN}[laptop]${NC} $1"; }
ok()   { echo -e "${GREEN}  ✓${NC} $1"; }
warn() { echo -e "${YELLOW}  ⚠${NC} $1"; }
err()  { echo -e "${RED}  ✗${NC} $1"; }

# ── CONFIG ────────────────────────────────────────────────────────────────
MINIPC_USER="cheese"
MINIPC_IP="192.168.0.102"
MINIPC_WS="~/lunar_rover_ws"
MINIPC_LAUNCH="bash ~/lunar_rover_ws/full_launch_minipc.sh"
GUI_SCRIPT="$(dirname "$0")/rover_control_gui.py"
TELEOP_SCRIPT="$(dirname "$0")/arduino_teleop_controller.py"
NAV_RELAY_SCRIPT="$(dirname "$0")/nav_goal_relay.py"
# ─────────────────────────────────────────────────────────────────────────

# Parse flags
START_MINIPC=false
SKIP_NAV=false
for arg in "$@"; do
    [ "$arg" = "--start-minipc" ] && START_MINIPC=true
    [ "$arg" = "--no-nav"       ] && SKIP_NAV=true
done

echo -e "${BOLD}"
echo "  ╔══════════════════════════════════════════╗"
echo "  ║     LUNAR ROVER  ·  Laptop Launch        ║"
echo "  ╚══════════════════════════════════════════╝"
echo -e "${NC}"

# ── ROS2 setup ────────────────────────────────────────────────────────────
set +u
if   [ -f /opt/ros/jazzy/setup.bash ];  then source /opt/ros/jazzy/setup.bash  && ok "ROS2 Jazzy"
elif [ -f /opt/ros/humble/setup.bash ]; then source /opt/ros/humble/setup.bash && ok "ROS2 Humble"
else err "No ROS2 installation found!" && exit 1
fi
[ -f ~/lunar_rover_ws/install/setup.bash ] && \
    source ~/lunar_rover_ws/install/setup.bash && ok "Workspace overlay sourced"
set -u

export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
ok "Network: ROS_DOMAIN_ID=42, SUBNET discovery"
echo ""

# ── Optional: SSH-start mini PC ──────────────────────────────────────────
if [ "$START_MINIPC" = "true" ]; then
    log "SSH-starting mini PC at ${MINIPC_USER}@${MINIPC_IP}..."

    if ssh -o ConnectTimeout=5 -o BatchMode=yes \
           "${MINIPC_USER}@${MINIPC_IP}" "echo ok" > /dev/null 2>&1; then
        ok "SSH connection verified"

        ssh "${MINIPC_USER}@${MINIPC_IP}" \
            "nohup bash ${MINIPC_WS}/full_launch_minipc.sh > /tmp/minipc_launch.log 2>&1 &"

        log "Mini PC launch triggered — waiting 10s for nodes to start..."
        sleep 10
        ok "Mini PC should be up"
    else
        warn "Cannot SSH to ${MINIPC_USER}@${MINIPC_IP}"
        warn "Check: both on same WiFi? SSH keys set up?"
        warn "Continuing anyway — you can start miniPC manually"
    fi
    echo ""
fi

# ── Check Python deps ─────────────────────────────────────────────────────
log "Checking Python dependencies..."
if python3 -c "import PyQt5" 2>/dev/null; then
    ok "PyQt5 available"
else
    warn "PyQt5 not found — installing..."
    pip3 install PyQt5 --break-system-packages 2>/dev/null || \
        sudo apt-get install -y python3-pyqt5 2>/dev/null
fi

if python3 -c "import cv2" 2>/dev/null; then
    ok "OpenCV available"
else
    warn "OpenCV not found — installing..."
    pip3 install opencv-python --break-system-packages 2>/dev/null || \
        sudo apt-get install -y python3-opencv 2>/dev/null
fi

# ── Locate scripts ────────────────────────────────────────────────────────
if [ ! -f "$GUI_SCRIPT" ]; then
    GUI_SCRIPT="$(find ~/lunar_rover_ws -name rover_control_gui.py 2>/dev/null | head -1)"
fi
if [ ! -f "$TELEOP_SCRIPT" ]; then
    TELEOP_SCRIPT="$(find ~/lunar_rover_ws -name arduino_teleop_controller.py 2>/dev/null | head -1)"
fi
if [ ! -f "$NAV_RELAY_SCRIPT" ]; then
    NAV_RELAY_SCRIPT="$(find ~/lunar_rover_ws -name nav_goal_relay.py 2>/dev/null | head -1)"
fi

# Nav control panel (replaces RViz clicking — more reliable)
NAV_PANEL_SCRIPT="$(dirname "$0")/nav_control_panel.py"
if [ ! -f "$NAV_PANEL_SCRIPT" ]; then
    NAV_PANEL_SCRIPT="$(find ~/lunar_rover_ws -name nav_control_panel.py 2>/dev/null | head -1)"
fi

ok "GUI script:    $GUI_SCRIPT"
[ -f "$TELEOP_SCRIPT"    ] && ok "Teleop script: $TELEOP_SCRIPT"
[ -f "$NAV_RELAY_SCRIPT" ] && ok "Nav relay:     $NAV_RELAY_SCRIPT"
echo ""

# ── Joy node ──────────────────────────────────────────────────────────────
if ! ros2 pkg list 2>/dev/null | grep -q "^joy$"; then
    warn "joy package not found — installing..."
    sudo apt-get install -y ros-$(ros2 --version 2>/dev/null | grep -oP 'jazzy|humble' | head -1)-joy 2>/dev/null
fi

log "Starting joy_node..."
ros2 run joy joy_node > /tmp/rover_joy.log 2>&1 &
JOY_PID=$!
sleep 1
kill -0 $JOY_PID 2>/dev/null && \
    ok "joy_node running (PID $JOY_PID)" || \
    warn "joy_node failed — check: sudo apt install ros-jazzy-joy"

# ── Controller teleop ─────────────────────────────────────────────────────
if [ -f "$TELEOP_SCRIPT" ]; then
    log "Starting controller teleop node..."
    python3 "$TELEOP_SCRIPT" > /tmp/rover_teleop.log 2>&1 &
    TELEOP_PID=$!
    sleep 1
    if kill -0 $TELEOP_PID 2>/dev/null; then
        ok "Controller teleop running (PID $TELEOP_PID)"
    else
        warn "Teleop node failed — check /tmp/rover_teleop.log"
    fi
fi

# ── Navigation goal relay ─────────────────────────────────────────────────
NAV_PID=""
if [ "$SKIP_NAV" = "false" ]; then
    # nav_goal_relay is the fallback RViz-based method (kept for compatibility)
    if [ -f "$NAV_RELAY_SCRIPT" ]; then
        python3 "$NAV_RELAY_SCRIPT" > /tmp/rover_nav_relay.log 2>&1 &
        NAV_PID=$!
        sleep 1
        kill -0 $NAV_PID 2>/dev/null && ok "nav_goal_relay running (PID $NAV_PID)" \
            || warn "nav_goal_relay failed — check /tmp/rover_nav_relay.log"
    fi

    # nav_control_panel: preferred — reliable click-to-navigate GUI
    if [ -f "$NAV_PANEL_SCRIPT" ]; then
        log "Starting nav_control_panel GUI..."
        python3 "$NAV_PANEL_SCRIPT" &
        PANEL_PID=$!
        sleep 2
        if kill -0 $PANEL_PID 2>/dev/null; then
            ok "nav_control_panel GUI open (PID $PANEL_PID)"
        else
            warn "nav_control_panel failed — check that PyQt5 is installed:"
            warn "  pip install PyQt5 --break-system-packages"
        fi
    else
        warn "nav_control_panel.py not found — copy it to ~/lunar_rover_ws/"
        warn "Falling back to RViz Publish Point tool (less reliable)"
    fi
else
    log "Nav relay skipped (--no-nav)"
fi
echo ""

# ── Cleanup on exit ───────────────────────────────────────────────────────
cleanup() {
    log "Shutting down..."
    [ -n "$JOY_PID"    ] && kill $JOY_PID    2>/dev/null
    [ -n "$TELEOP_PID" ] && kill $TELEOP_PID 2>/dev/null
    [ -n "$NAV_PID"    ] && kill $NAV_PID    2>/dev/null
}
trap cleanup EXIT INT TERM

# ── Launch GUI ────────────────────────────────────────────────────────────
echo -e "${BOLD}${GREEN}"
echo "  ╔══════════════════════════════════════════╗"
echo "  ║       Launching Mission Control GUI      ║"
echo "  ╚══════════════════════════════════════════╝"
echo -e "${NC}"

if [ ! -f "$GUI_SCRIPT" ]; then
    err "rover_control_gui.py not found! Copy it to the workspace."
    exit 1
fi

exec python3 "$GUI_SCRIPT"