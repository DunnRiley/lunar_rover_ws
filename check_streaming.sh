#!/bin/bash
# ============================================================================
#  CAMERA STREAMING DIAGNOSTIC  ·  check_streaming.sh
#  Run this on EITHER machine to diagnose cross-machine streaming issues.
#
#  Usage:
#    bash check_streaming.sh           # auto-detect role
#    bash check_streaming.sh minipc    # force miniPC role
#    bash check_streaming.sh laptop    # force laptop role
# ============================================================================

CYAN='\033[0;36m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'
RED='\033[0;31m'; BOLD='\033[1m'; NC='\033[0m'

ok()   { echo -e "  ${GREEN}✓${NC} $1"; }
warn() { echo -e "  ${YELLOW}⚠${NC}  $1"; }
err()  { echo -e "  ${RED}✗${NC} $1"; }
hdr()  { echo -e "\n${BOLD}${CYAN}══ $1 ══${NC}"; }

echo -e "${BOLD}"
echo "  ╔══════════════════════════════════════════════╗"
echo "  ║   Camera Streaming Diagnostic                ║"
echo "  ╚══════════════════════════════════════════════╝"
echo -e "${NC}"

# ── Detect role ──────────────────────────────────────────────────────────────
ROLE=${1:-""}
if [ -z "$ROLE" ]; then
    HOSTNAME=$(hostname)
    if [[ "$HOSTNAME" == *"cheese"* ]]; then
        ROLE="minipc"
    else
        ROLE="laptop"
    fi
fi
echo "  Detected role: ${BOLD}${ROLE}${NC}"

# ── ROS setup ────────────────────────────────────────────────────────────────
hdr "ROS2 Environment"
set +u
if   [ -f /opt/ros/jazzy/setup.bash ];  then source /opt/ros/jazzy/setup.bash  && ok "ROS2 Jazzy sourced"
elif [ -f /opt/ros/humble/setup.bash ]; then source /opt/ros/humble/setup.bash && ok "ROS2 Humble sourced"
else err "No ROS2 installation found!" && exit 1
fi
[ -f ~/lunar_rover_ws/install/setup.bash ] && source ~/lunar_rover_ws/install/setup.bash && ok "Workspace sourced"
set -u

# ── Check / set env vars ─────────────────────────────────────────────────────
hdr "Network Environment Variables"

check_env() {
    local var=$1 expected=$2
    local val="${!var:-}"
    if [ -z "$val" ]; then
        warn "$var is NOT SET  (expected: $expected)"
        export "$var"="$expected"
        warn "  → Temporarily set to $expected for this session"
        warn "  → Add to ~/.bashrc:  export $var=$expected"
    elif [ "$val" = "$expected" ]; then
        ok "$var=$val"
    else
        err "$var=$val  (expected: $expected)"
        warn "  → This WILL break cross-machine comms!  Set: export $var=$expected"
    fi
}

check_env ROS_DOMAIN_ID          "42"
check_env ROS_LOCALHOST_ONLY     "0"
check_env ROS_AUTOMATIC_DISCOVERY_RANGE "SUBNET"

export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET

# ── Network connectivity ─────────────────────────────────────────────────────
hdr "Network Connectivity"
MINIPC_IP="192.168.0.102"

LOCAL_IP=$(hostname -I | awk '{print $1}' 2>/dev/null || echo "unknown")
ok "Local IP: $LOCAL_IP"

if ping -c1 -W2 "$MINIPC_IP" > /dev/null 2>&1; then
    ok "miniPC ($MINIPC_IP) is reachable"
else
    err "miniPC ($MINIPC_IP) is NOT reachable"
    warn "  Check: both machines on same WiFi network?"
fi

# Check they're on the same subnet
LOCAL_SUBNET=$(echo "$LOCAL_IP" | cut -d. -f1-3)
MINI_SUBNET=$(echo "$MINIPC_IP" | cut -d. -f1-3)
if [ "$LOCAL_SUBNET" = "$MINI_SUBNET" ]; then
    ok "Same subnet ($LOCAL_SUBNET.*)"
else
    err "Different subnets! Local=$LOCAL_SUBNET.*  MiniPC=$MINI_SUBNET.*"
    err "  SUBNET discovery requires both machines on same /24 subnet"
fi

# ── ROS node/topic discovery ─────────────────────────────────────────────────
hdr "ROS Topic Discovery"

echo "  Waiting 3s for discovery..."
sleep 3

TOPIC_LIST=$(ros2 topic list 2>/dev/null)
echo "  Found $(echo "$TOPIC_LIST" | grep -c '^/') topics total"

# Check critical camera topics
check_topic() {
    local topic=$1
    if echo "$TOPIC_LIST" | grep -q "^${topic}$"; then
        ok "Topic exists: $topic"
        # Check if data is actually flowing
        local hz
        hz=$(ros2 topic hz "$topic" --window 5 2>/dev/null | grep "^average" | awk '{print $3}' || echo "?")
        if [ "$hz" != "?" ] && [ -n "$hz" ]; then
            ok "  → Data flowing at ~${hz} Hz"
        else
            warn "  → Topic exists but NO DATA flowing (check if publisher is running)"
        fi
    else
        err "Topic MISSING: $topic"
    fi
}

echo ""
echo "  Camera source topics (published by miniPC):"
check_topic /camera/camera/color/image_raw/compressed
check_topic /camera/camera/aligned_depth_to_color/image_raw

echo ""
echo "  Streamed topics (published by pipeline, consumed by RViz):"
check_topic /camera/color/stream/compressed
check_topic /camera/depth/stream/compressed
check_topic /camera_rear/stream/compressed

# ── Pipeline process check ───────────────────────────────────────────────────
hdr "Pipeline Processes"

check_proc() {
    local name=$1
    if pgrep -f "$name" > /dev/null 2>&1; then
        local pid
        pid=$(pgrep -f "$name" | head -1)
        ok "$name running (PID $pid)"
    else
        warn "$name NOT running"
    fi
}

check_proc "realsense2_camera_node"
check_proc "optimized_image_pipeline"
check_proc "stereo_camera_publisher"
check_proc "joy_to_arduino"

# ── RViz transport check (laptop only) ───────────────────────────────────────
if [ "$ROLE" = "laptop" ]; then
    hdr "RViz Transport Check"
    
    # Check if image_transport is available
    if ros2 pkg list 2>/dev/null | grep -q "^image_transport$"; then
        ok "image_transport package available"
    else
        err "image_transport NOT found"
        warn "  Install: sudo apt install ros-\$(ros2 --version 2>/dev/null | grep -oP 'jazzy|humble')-image-transport"
    fi
    
    if ros2 pkg list 2>/dev/null | grep -q "^compressed_image_transport$"; then
        ok "compressed_image_transport available"
    else
        err "compressed_image_transport NOT found"
        warn "  Install: sudo apt install ros-\$(ros2 --version 2>/dev/null | grep -oP 'jazzy|humble')-compressed-image-transport"
    fi
fi

# ── Firewall check ───────────────────────────────────────────────────────────
hdr "Firewall"
if command -v ufw &>/dev/null; then
    UFW_STATUS=$(sudo ufw status 2>/dev/null | head -1)
    if echo "$UFW_STATUS" | grep -q "inactive"; then
        ok "ufw is inactive (no blocking)"
    else
        warn "ufw is active — may block ROS DDS traffic"
        warn "  To disable: sudo ufw disable"
        warn "  Or allow: sudo ufw allow from 192.168.0.0/24"
    fi
else
    ok "ufw not installed (no local firewall)"
fi

# ── Summary and recommendations ──────────────────────────────────────────────
hdr "Summary & Next Steps"

echo ""
echo "  If images are not showing in RViz, work through this checklist:"
echo ""
echo "  ${BOLD}On the miniPC:${NC}"
echo "    1. Run:  bash full_launch_minipc.sh"
echo "    2. Verify:  ros2 topic echo /camera/color/stream/compressed --no-arr | head -5"
echo "    3. Should show stamp/frame_id every ~0.17s (6fps)"
echo ""
echo "  ${BOLD}On the laptop:${NC}"
echo "    4. Run:  bash check_streaming.sh laptop"
echo "    5. Verify topics appear in list above"
echo "    6. In RViz → Image display → Topic → set to:"
echo "       /camera/color/stream/compressed"
echo "    7. In RViz → Image display → Transport Hint → set to: compressed"
echo ""
echo "  ${BOLD}Quick test (laptop):${NC}"
echo "    ros2 run image_tools showimage --ros-args \\"
echo "      --remap image:=/camera/color/stream/compressed \\"
echo "      -p transport:=compressed"
echo ""
echo "  ${BOLD}If no topics visible on laptop:${NC}"
echo "    export ROS_DOMAIN_ID=42"
echo "    export ROS_LOCALHOST_ONLY=0"
echo "    export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET"
echo "    ros2 topic list  # should now show miniPC topics"
echo ""
echo "  ${BOLD}Log files (miniPC):${NC}"
echo "    tail -f /tmp/rover_camera.log"
echo "    tail -f /tmp/rover_pipe_color.log"
echo "    tail -f /tmp/rover_pipe_depth.log"
