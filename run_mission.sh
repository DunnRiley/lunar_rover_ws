#!/bin/bash
# run_mission.sh  —  MINI PC
# Usage: bash ~/lunar_rover_ws/run_mission.sh [--list|--dry-run] [file.yaml]
#
# Starts nav_arduino_bridge (owns serial), then nav_mission_sequencer,
# then sends the mission. Kills any stale sequencer first.

C1='\033[0;36m'; C2='\033[0;32m'; C3='\033[1;33m'; C4='\033[0;31m'; CN='\033[0m'; CB='\033[1m'
say()  { echo -e "${C1}[mission]${CN} $1"; }
ok()   { echo -e "${C2}  OK${CN} $1"; }
warn() { echo -e "${C3}  WARN${CN} $1"; }
err()  { echo -e "${C4}  ERR${CN} $1"; }

WS="$HOME/lunar_rover_ws"
cd "$WS" || { err "Cannot cd to $WS"; exit 1; }

MISSION_FILE=""; DRY_RUN=false; LIST_MODE=false

for arg in "$@"; do
    case "$arg" in
        --dry-run)  DRY_RUN=true ;;
        --list)     LIST_MODE=true ;;
        -h|--help)  echo "bash run_mission.sh [--list|--dry-run] [file.yaml]"; exit 0 ;;
        *.yaml|*.yml)  MISSION_FILE="$arg" ;;
        *)  [ -f "$arg" ] && MISSION_FILE="$arg" ;;
    esac
done

# ── List ──────────────────────────────────────────────────────────────────────
if [ "$LIST_MODE" = "true" ]; then
    echo ""
    echo -e "${CB}Available YAML files:${CN}"
    found=0
    for f in "$WS"/*.yaml "$WS"/*.yml; do
        [ -f "$f" ] && found=$((found+1)) && echo "  $f"
    done
    [ "$found" -eq 0 ] && echo "  (none in $WS)"
    echo ""; exit 0
fi

# ── Resolve file ──────────────────────────────────────────────────────────────
[ -z "$MISSION_FILE" ] && MISSION_FILE="$WS/mission.yaml" && say "Using default: mission.yaml"
echo "$MISSION_FILE" | grep -qv '^/' && MISSION_FILE="$WS/$MISSION_FILE"
[ -f "$MISSION_FILE" ] || { err "Not found: $MISSION_FILE"; exit 1; }
ok "Mission file: $MISSION_FILE"

# ── Validate ──────────────────────────────────────────────────────────────────
say "Validating YAML..."
if command -v python3 >/dev/null 2>&1; then
    VR=$(MFILE="$MISSION_FILE" python3 - <<'PY'
import sys,os
f=os.environ.get("MFILE","")
try:
    import yaml
    with open(f) as fh: doc=yaml.safe_load(fh)
    m=doc.get("mission",doc); s=m.get("steps",[]); n=m.get("name","unnamed")
    print("OK steps=%d name=%s"%(len(s),n))
except ImportError: print("WARN pyyaml missing")
except Exception as e: print("ERROR "+str(e)); sys.exit(1)
PY
)
    echo "$VR" | grep -q "^ERROR" && { err "YAML: $VR"; exit 1; }
    ok "YAML valid: $VR"
fi

[ "$DRY_RUN" = "true" ] && ok "Dry-run complete" && exit 0

# ── ROS2 setup ────────────────────────────────────────────────────────────────
set +u
[ -f /opt/ros/jazzy/setup.bash ]  && source /opt/ros/jazzy/setup.bash  && ok "ROS2 Jazzy"
[ -f /opt/ros/humble/setup.bash ] && source /opt/ros/humble/setup.bash && ok "ROS2 Humble"
[ -f "$WS/install/setup.bash" ]   && source "$WS/install/setup.bash"   && ok "WS sourced"
set -u
export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=0 ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
ok "ROS_DOMAIN_ID=42"

echo ""
echo -e "${CB}${C1}  LUNAR ROVER MISSION RUN${CN}"
echo "  File: $(basename "$MISSION_FILE")"
echo ""

# ── Bridge (MUST run — it owns the Arduino serial port) ───────────────────────
BRIDGE_RUNNING=$(ros2 node list 2>/dev/null | grep -c "nav_arduino_bridge" || true)
if [ "$BRIDGE_RUNNING" -gt 0 ]; then
    ok "nav_arduino_bridge already running"
else
    say "Starting nav_arduino_bridge.py..."
    python3 "$WS/nav_arduino_bridge.py" > /tmp/rover_bridge.log 2>&1 &
    i=0
    while [ $i -lt 8 ]; do
        i=$((i+1)); sleep 1
        FOUND=$(ros2 node list 2>/dev/null | grep -c "nav_arduino_bridge" || true)
        [ "$FOUND" -gt 0 ] && ok "Bridge ready (${i}s)" && break
        printf "."
    done
    echo ""
fi

# ── Sequencer (always kill and restart — no stale state) ──────────────────────
say "Restarting sequencer (fresh instance)..."
pkill -f "nav_mission_sequencer" 2>/dev/null
sleep 1
python3 "$WS/nav_mission_sequencer.py" > /tmp/rover_sequencer.log 2>&1 &
SEQ_PID=$!

i=0
while [ $i -lt 10 ]; do
    i=$((i+1)); sleep 1
    FOUND=$(ros2 node list 2>/dev/null | grep -c "nav_mission_sequencer" || true)
    [ "$FOUND" -gt 0 ] && ok "Sequencer ready (${i}s)" && break
    printf "."
done
echo ""

# ── Cleanup trap ──────────────────────────────────────────────────────────────
cleanup() {
    echo ""
    say "Aborting..."
    ros2 topic pub --once /mission/start std_msgs/msg/Bool "data: false" 2>/dev/null || true
    sleep 0.3
    ros2 topic pub --once /nav/arduino_cmd std_msgs/msg/Float32MultiArray \
        "data: [255.0,0.0,0.0,0.0]" 2>/dev/null || true
    exit 0
}
trap cleanup INT TERM

# ── Send mission ──────────────────────────────────────────────────────────────
say "Sending file path..."
ros2 topic pub --once /mission/file std_msgs/msg/String \
    "data: '$MISSION_FILE'" 2>/dev/null && ok "File sent" || warn "file send failed"

sleep 0.6

say "Sending START..."
ros2 topic pub --once /mission/start std_msgs/msg/Bool \
    "data: true" 2>/dev/null && ok "Started!" || warn "start send failed"

echo ""
echo -e "${CB}${C2}  Mission running.  Ctrl+C to abort.${CN}"
echo "  Sequencer logs: tail -f /tmp/rover_sequencer.log"
echo "  Bridge logs:    tail -f /tmp/rover_bridge.log"
echo ""

# ── Monitor ───────────────────────────────────────────────────────────────────
START_T=$(date +%s); LAST_KEY=""

while true; do
    LINE=$(ros2 topic echo --once --no-arr /mission/status 2>/dev/null \
           | grep "data:" | head -1 || true)
    if [ -n "$LINE" ]; then
        JSON=$(echo "$LINE" | sed "s/^.*data: //")
        INFO=$(echo "$JSON" | python3 -c "
import sys,json
try:
    d=json.loads(sys.stdin.read().strip().strip(\"'\"))
    print('%s|%s|%s|%s'%(d.get('running',False),d.get('step',0),d.get('total',0),d.get('step_name','')))
except: print('unknown|0|0|')
" 2>/dev/null || echo "unknown|0|0|")
        RUN=$(echo "$INFO"|cut -d'|' -f1)
        STEP=$(echo "$INFO"|cut -d'|' -f2)
        TOTAL=$(echo "$INFO"|cut -d'|' -f3)
        NAME=$(echo "$INFO"|cut -d'|' -f4)
        KEY="${STEP}|${NAME}"
        ELAPSED=$(( $(date +%s) - START_T ))
        if [ "$KEY" != "$LAST_KEY" ]; then
            LAST_KEY="$KEY"
            if [ "$RUN" = "True" ] || [ "$RUN" = "true" ]; then
                echo "  >> Step $((STEP+1))/$TOTAL: $NAME  [${ELAPSED}s]"
            else
                break
            fi
        fi
    fi
    sleep 0.5
done

ELAPSED=$(( $(date +%s) - START_T ))
echo ""
echo -e "${CB}${C2}  Mission complete  (${ELAPSED}s)${CN}"