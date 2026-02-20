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
MINIPC_USER="moonpie"
MINIPC_IP="192.168.0.102"
MINIPC_WS="~/lunar_rover_ws"
MINIPC_LAUNCH="bash ~/lunar_rover_ws/full_launch_minipc.sh"
GUI_SCRIPT="$(dirname "$0")/rover_control_gui.py"
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

# ── Locate GUI script ────────────────────────────────────────────────────
if [ ! -f "$GUI_SCRIPT" ]; then
    GUI_SCRIPT="$(find ~/lunar_rover_ws -name rover_control_gui.py 2>/dev/null | head -1)"
fi

if [ ! -f "$GUI_SCRIPT" ]; then
    err "rover_control_gui.py not found!"
    err "Copy it to the same directory as this script or to ~/lunar_rover_ws/"
    exit 1
fi

ok "GUI script: $GUI_SCRIPT"
echo ""

# ── Launch GUI ───────────────────────────────────────────────────────────
echo -e "${BOLD}${GREEN}"
echo "  ╔══════════════════════════════════════════╗"
echo "  ║       Launching Mission Control GUI      ║"
echo "  ╚══════════════════════════════════════════╝"
echo -e "${NC}"

# Pass ROS env vars into python process
exec python3 "$GUI_SCRIPT"