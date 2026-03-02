#!/bin/bash
# ============================================================================
# fix_minipc_env.sh  —  Run ONCE on the miniPC
# Adds the required ROS2 env vars to ~/.bashrc so they persist across
# SSH sessions (the miniPC was missing ROS_DOMAIN_ID and ROS_LOCALHOST_ONLY).
# ============================================================================

GREEN='\033[0;32m'; YELLOW='\033[1;33m'; CYAN='\033[0;36m'; NC='\033[0m'
ok()   { echo -e "  ${GREEN}✓${NC} $1"; }
warn() { echo -e "  ${YELLOW}already set${NC}: $1"; }
log()  { echo -e "${CYAN}[env fix]${NC} $1"; }

log "Adding ROS env vars to ~/.bashrc..."
echo ""

BASHRC="$HOME/.bashrc"
add_if_missing() {
    local line="$1" label="$2"
    if grep -qF "$line" "$BASHRC" 2>/dev/null; then
        warn "$label"
    else
        echo "" >> "$BASHRC"
        echo "$line" >> "$BASHRC"
        ok "Added: $label"
    fi
}

add_if_missing "export ROS_DOMAIN_ID=42"                     "ROS_DOMAIN_ID=42"
add_if_missing "export ROS_LOCALHOST_ONLY=0"                  "ROS_LOCALHOST_ONLY=0"
add_if_missing "export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET"  "ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET"

echo ""
echo "  Done. Run:  source ~/.bashrc"
echo "  Verify:    echo \$ROS_DOMAIN_ID   (should print 42)"
