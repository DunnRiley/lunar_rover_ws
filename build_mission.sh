#!/bin/bash
# =============================================================================
#  build_mission.sh  —  interactive command-line mission builder
#
#  Run on either the miniPC or laptop.
#  Walks you through adding steps and writes a valid mission.yaml.
#
#  USAGE:
#    bash ~/lunar_rover_ws/build_mission.sh
#    bash ~/lunar_rover_ws/build_mission.sh output.yaml
#    bash ~/lunar_rover_ws/build_mission.sh --quick  # fast preset prompts
# =============================================================================

CYAN='\033[0;36m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'
RED='\033[0;31m'; BOLD='\033[1m'; NC='\033[0m'

WS="$HOME/lunar_rover_ws"
OUTPUT_FILE="${1:-$WS/mission_new.yaml}"
[[ "$OUTPUT_FILE" == --* ]] && OUTPUT_FILE="$WS/mission_new.yaml"

QUICK_MODE=false
[[ "$1" == "--quick" ]] && QUICK_MODE=true && OUTPUT_FILE="$WS/mission_quick.yaml"

clear
echo -e "${BOLD}${CYAN}"
echo "  ╔══════════════════════════════════════════════╗"
echo "  ║      LUNAR ROVER  ·  MISSION BUILDER         ║"
echo "  ╚══════════════════════════════════════════════╝"
echo -e "${NC}"

# ── Mission name ──────────────────────────────────────────────────────────────
read -rp "  Mission name [Unnamed mission]: " MISSION_NAME
MISSION_NAME="${MISSION_NAME:-Unnamed mission}"

STEPS=()   # each entry is a YAML block string

# ── Step builder ──────────────────────────────────────────────────────────────

print_menu() {
    echo ""
    echo -e "  ${BOLD}Add a step:${NC}"
    echo "    1) drive_forward     — drive forward N metres"
    echo "    2) drive_backward    — drive backward N metres"
    echo "    3) arc_turn          — turn by degrees (IMU)"
    echo "    4) actuator_position — move bucket to dig/drive/dump"
    echo "    5) wait              — pause N seconds"
    echo "    6) stop              — stop all motors"
    echo "    7) done              — finish building, write YAML"
    echo "    8) show              — preview current steps"
    echo "    9) remove last       — delete last step"
    echo ""
}

step_count() { echo "${#STEPS[@]}"; }

add_drive_forward() {
    read -rp "    Distance (metres) [1.0]: " D; D="${D:-1.0}"
    read -rp "    Use camera for stop? (y/n) [y]: " UC; UC="${UC:-y}"
    [[ "$UC" == "y" || "$UC" == "Y" ]] && USE_CAM="true" || USE_CAM="false"
    read -rp "    Timeout seconds [25]: " T; T="${T:-25}"
    STEPS+=("    - action: drive_forward
      params:
        distance_m: $D
        use_camera: $USE_CAM
        timeout_s: $T")
    ok "Added: drive_forward ${D}m"
}

add_drive_backward() {
    read -rp "    Distance (metres) [0.5]: " D; D="${D:-0.5}"
    read -rp "    Timeout seconds [15]: " T; T="${T:-15}"
    STEPS+=("    - action: drive_backward
      params:
        distance_m: $D
        timeout_s: $T")
    ok "Added: drive_backward ${D}m"
}

add_arc_turn() {
    echo "    Positive degrees = CCW (left), negative = CW (right)"
    read -rp "    Degrees [90]: " DEG; DEG="${DEG:-90}"
    read -rp "    Turn speed rad/s [0.18]: " SPD; SPD="${SPD:-0.18}"
    read -rp "    Tolerance degrees [5]: " TOL; TOL="${TOL:-5}"
    read -rp "    Timeout seconds [30]: " T; T="${T:-30}"
    STEPS+=("    - action: arc_turn
      params:
        degrees: $DEG
        speed: $SPD
        tolerance_deg: $TOL
        timeout_s: $T")
    ok "Added: arc_turn ${DEG}°"
}

add_actuator() {
    echo "    Targets: dig | drive | dump"
    read -rp "    Target [dig]: " TGT; TGT="${TGT:-dig}"
    read -rp "    Timeout seconds [8]: " T; T="${T:-8}"
    STEPS+=("    - action: actuator_position
      params:
        target: $TGT
        timeout_s: $T")
    ok "Added: actuator_position → $TGT"
}

add_wait() {
    read -rp "    Seconds [1.0]: " S; S="${S:-1.0}"
    STEPS+=("    - action: wait
      params:
        seconds: $S")
    ok "Added: wait ${S}s"
}

add_stop() {
    STEPS+=("    - action: stop
      params: {}")
    ok "Added: stop"
}

show_steps() {
    echo ""
    if [ "${#STEPS[@]}" -eq 0 ]; then
        echo "  (no steps yet)"
        return
    fi
    echo -e "  ${BOLD}Current steps:${NC}"
    for i in "${!STEPS[@]}"; do
        n=$((i+1))
        first_line=$(echo "${STEPS[$i]}" | grep "action:" | head -1 | tr -s ' ' | xargs)
        echo "    $n. $first_line"
    done
    echo ""
}

# ── Quick presets ─────────────────────────────────────────────────────────────

if [ "$QUICK_MODE" = "true" ]; then
    echo -e "  ${BOLD}Quick mode — choose a preset:${NC}"
    echo "    1) Test forward 1m + back 1m"
    echo "    2) Dig + drive to dump zone + dump"
    echo "    3) Custom (guided)"
    read -rp "  Choice [1]: " QC; QC="${QC:-1}"

    case "$QC" in
        1)
            MISSION_NAME="Test forward/back"
            STEPS+=("    - action: drive_forward
      params: {distance_m: 1.0, use_camera: false, timeout_s: 20}")
            STEPS+=("    - action: wait
      params: {seconds: 1.0}")
            STEPS+=("    - action: drive_backward
      params: {distance_m: 1.0, timeout_s: 20}")
            STEPS+=("    - action: stop
      params: {}")
            ;;
        2)
            MISSION_NAME="Dig and dump"
            STEPS+=("    - action: actuator_position
      params: {target: dig, timeout_s: 8}")
            STEPS+=("    - action: drive_forward
      params: {distance_m: 0.8, use_camera: false, timeout_s: 15}")
            STEPS+=("    - action: actuator_position
      params: {target: drive, timeout_s: 8}")
            STEPS+=("    - action: arc_turn
      params: {degrees: 180, speed: 0.18, tolerance_deg: 5, timeout_s: 30}")
            STEPS+=("    - action: drive_forward
      params: {distance_m: 3.0, use_camera: true, timeout_s: 35}")
            STEPS+=("    - action: actuator_position
      params: {target: dump, timeout_s: 8}")
            STEPS+=("    - action: wait
      params: {seconds: 2.0}")
            STEPS+=("    - action: actuator_position
      params: {target: drive, timeout_s: 8}")
            STEPS+=("    - action: stop
      params: {}")
            ;;
        *)
            true  # fall through to interactive
            ;;
    esac
fi

# ── Interactive loop ──────────────────────────────────────────────────────────

if [ "${#STEPS[@]}" -eq 0 ]; then
    while true; do
        print_menu
        echo -n "  Choice [1-9]: "
        read -r choice

        case "$choice" in
            1) add_drive_forward ;;
            2) add_drive_backward ;;
            3) add_arc_turn ;;
            4) add_actuator ;;
            5) add_wait ;;
            6) add_stop ;;
            7) break ;;
            8) show_steps ;;
            9)
                if [ "${#STEPS[@]}" -gt 0 ]; then
                    unset 'STEPS[-1]'
                    ok "Removed last step (${#STEPS[@]} remaining)"
                else
                    warn "No steps to remove"
                fi ;;
            *) warn "Invalid choice: $choice" ;;
        esac
    done
fi

# ── Write YAML ────────────────────────────────────────────────────────────────

show_steps

if [ "${#STEPS[@]}" -eq 0 ]; then
    warn "No steps added — aborting"
    exit 1
fi

echo ""
log "Writing $OUTPUT_FILE…"

{
echo "# Mission YAML — generated by build_mission.sh"
echo "# Run with: bash ~/lunar_rover_ws/run_mission.sh $(basename "$OUTPUT_FILE")"
echo ""
echo "mission:"
echo "  name: \"$MISSION_NAME\""
echo ""
echo "  steps:"
for s in "${STEPS[@]}"; do
    echo "$s"
    echo ""
done
} > "$OUTPUT_FILE"

ok "Saved: $OUTPUT_FILE"
echo ""
echo -e "  ${BOLD}To run this mission:${NC}"
echo "    bash ~/lunar_rover_ws/run_mission.sh $(basename "$OUTPUT_FILE")"
echo ""
echo -e "  ${BOLD}Dry-run validation:${NC}"
echo "    bash ~/lunar_rover_ws/run_mission.sh --dry-run $(basename "$OUTPUT_FILE")"
echo ""