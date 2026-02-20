#!/bin/bash
# ============================================================================
#  LAPTOP — FULL ROVER LAUNCH
#  Optionally SSH-starts the mini PC, then opens the GUI control panel.
#
#  USAGE:
#    bash full_launch_laptop.sh              → GUI only (miniPC already running)
#    bash full_launch_laptop.sh --start-minipc  → SSH + start miniPC first
# ============================================================================

MINIPC_IP="192.168.0.102"
MINIPC_USER="moonpie"
START_MINIPC=false

for arg in "$@"; do
    [ "$arg" = "--start-minipc" ] && START_MINIPC=true
done

RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'
CYAN='\033[0;36m'; BOLD='\033[1m'; NC='\033[0m'
ok()   { echo -e "${GREEN}  ✓${NC} $*"; }
warn() { echo -e "${YELLOW}  ⚠${NC}  $*"; }
log()  { echo -e "${CYAN}[laptop]${NC} $*"; }

echo ""
echo -e "${BOLD}╔══════════════════════════════════════════════╗${NC}"
echo -e "${BOLD}║       LUNAR ROVER — LAPTOP LAUNCH            ║${NC}"
echo -e "${BOLD}╚══════════════════════════════════════════════╝${NC}"
echo ""

# ── ROS2 setup ───────────────────────────────────────────────────────────────
if   [ -f /opt/ros/jazzy/setup.bash ];  then source /opt/ros/jazzy/setup.bash  && ok "ROS2 Jazzy"
elif [ -f /opt/ros/humble/setup.bash ]; then source /opt/ros/humble/setup.bash && ok "ROS2 Humble"
else echo -e "${RED}✗ No ROS2 found${NC}"; exit 1; fi

[ -f ~/lunar_rover_ws/install/setup.bash ] && source ~/lunar_rover_ws/install/setup.bash && ok "Workspace sourced"

export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
ok "Network: DOMAIN_ID=42, SUBNET"
echo ""

# ── Optionally start MiniPC via SSH ─────────────────────────────────────────
if [ "$START_MINIPC" = "true" ]; then
    log "Starting Mini PC via SSH ($MINIPC_USER@$MINIPC_IP)…"

    if ! ssh -o ConnectTimeout=5 -o BatchMode=yes "$MINIPC_USER@$MINIPC_IP" exit 2>/dev/null; then
        warn "Cannot SSH to Mini PC — make sure it's on and same WiFi"
    else
        # Launch miniPC script in background SSH session (nohup keeps it alive after SSH exits)
        ssh "$MINIPC_USER@$MINIPC_IP" \
            "cd ~/lunar_rover_ws && nohup bash full_launch_minipc.sh > /tmp/rover_minipc_launch.log 2>&1 &"
        ok "Mini PC launch initiated — waiting 8s for it to come up…"
        sleep 8
    fi
    echo ""
fi

# ── Check python dependencies ─────────────────────────────────────────────────
log "Checking Python GUI dependencies…"
python3 -c "import PyQt5" 2>/dev/null && ok "PyQt5 found" || {
    warn "PyQt5 not found — installing…"
    pip install PyQt5 --break-system-packages --quiet || \
    sudo apt-get install -y python3-pyqt5 -q
}

# ── Launch the GUI ────────────────────────────────────────────────────────────
GUI_SCRIPT="$HOME/lunar_rover_ws/rover_control_gui.py"

if [ ! -f "$GUI_SCRIPT" ]; then
    warn "rover_control_gui.py not found at $GUI_SCRIPT"
    warn "Copy rover_control_gui.py to ~/lunar_rover_ws/ first"
    exit 1
fi

log "Launching Rover Control GUI…"
echo ""
python3 "$GUI_SCRIPT" --minipc-ip "$MINIPC_IP" --minipc-user "$MINIPC_USER"