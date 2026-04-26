#!/usr/bin/env bash
# =============================================================================
# validate_week3.sh — Week 3 Endurance & Validation Test
# =============================================================================
# Reformerz Healthcare | Drone Tracking Pipeline
#
# What this script does:
#   1. Pre-flight checks (MAVROS up? PX4 SITL up? workspace built?)
#   2. Launches the full tracking stack in a tmux session
#   3. Starts logger_node to record session_data.csv
#   4. Runs for ENDURANCE_SECONDS (default 600 = 10 minutes)
#   5. Monitors for OFFBOARD mode drops every 30 seconds (FAIL condition)
#   6. Kills all nodes cleanly after the timer
#   7. Prints a PASS/FAIL endurance report
#   8. Prompts to generate plots
#
# Prerequisites:
#   Terminal A: PX4 SITL running  (make px4_sitl gazebo-classic_iris)
#   Terminal B: MAVROS running    (ros2 launch mavros px4.launch ...)
#   Terminal C: test_image_pub.py running
#   Then run this script in Terminal D.
#
# Usage:
#   chmod +x validate_week3.sh
#   bash ~/ros2_ws/src/my_robot_controller/scripts/validate_week3.sh
#   bash ~/ros2_ws/src/my_robot_controller/scripts/validate_week3.sh --duration 120
# =============================================================================

set -eo pipefail

# ── Configuration ─────────────────────────────────────────────────────────────
ENDURANCE_SECONDS=600          # 10-minute endurance run
MONITOR_INTERVAL=30            # check OFFBOARD status every N seconds
WS=~/ros2_ws
SCRIPTS=$WS/src/my_robot_controller/scripts
SETUP=$WS/install/setup.bash
LOG_DIR=$WS/logs
SESSION="week3_validation"

# ── Parse arguments ───────────────────────────────────────────────────────────
while [[ $# -gt 0 ]]; do
    case $1 in
        --duration) ENDURANCE_SECONDS="$2"; shift 2 ;;
        *) echo "Unknown arg: $1"; exit 1 ;;
    esac
done

# ── Colours ───────────────────────────────────────────────────────────────────
G='\033[0;32m'; R='\033[0;31m'; Y='\033[1;33m'; B='\033[1;34m'; NC='\033[0m'
ok()   { echo -e "${G}[PASS]${NC} $*"; }
warn() { echo -e "${Y}[WARN]${NC} $*"; }
fail() { echo -e "${R}[FAIL]${NC} $*"; }
info() { echo -e "${B}[INFO]${NC} $*"; }

# ── State tracking ────────────────────────────────────────────────────────────
MODE_DROP_COUNT=0
HEARTBEAT_TIMEOUT_COUNT=0
START_EPOCH=$(date +%s)

mkdir -p "$LOG_DIR"
ENDURANCE_LOG="$LOG_DIR/endurance_$(date +%Y%m%d_%H%M%S).log"

log() {
    local ts; ts=$(date +"%H:%M:%S")
    echo "[$ts] $*" | tee -a "$ENDURANCE_LOG"
}

# ── Banner ────────────────────────────────────────────────────────────────────
echo ""
echo -e "${B}+=========================================================================+${NC}"
echo -e "${B}|   WEEK 3 ENDURANCE VALIDATION — Reformerz Healthcare                   |${NC}"
echo -e "${B}|   Duration: ${ENDURANCE_SECONDS}s  ($(( ENDURANCE_SECONDS / 60 )) minutes)                               |${NC}"
echo -e "${B}+=========================================================================+${NC}"
echo ""

# ── Pre-flight checks ─────────────────────────────────────────────────────────
echo "--- Pre-flight Checks ---"

# 1. Workspace built?
if [[ ! -f "$SETUP" ]]; then
    fail "Workspace not built. Run: cd $WS && colcon build --packages-select my_robot_controller"
    exit 1
fi
source "$SETUP"
ok "Workspace sourced"

# 2. MAVROS running? (check /mavros/state topic exists)
if ! timeout 4s ros2 topic echo /mavros/state --once > /dev/null 2>&1; then
    fail "/mavros/state not available — start MAVROS first (Terminal B)"
    exit 1
fi
ok "MAVROS is running"

# 3. PX4 SITL connected?
FCU_CONN=$(timeout 4s ros2 topic echo /mavros/state --once 2>/dev/null | grep -c "connected: true" || true)
if [[ "$FCU_CONN" -eq 0 ]]; then
    fail "FCU not connected — check Terminal A (PX4 SITL)"
    exit 1
fi
ok "PX4 SITL connected"

# 4. test_image_pub running?
if ! ros2 topic hz /camera/image_raw --window 5 2>/dev/null | grep -q "average rate" & sleep 3; kill %1 2>/dev/null; then
    warn "/camera/image_raw not detected — start test_image_pub.py (Terminal C)"
    warn "Continuing anyway — perception will not run without it"
fi

# 5. logger_node.py exists?
if [[ ! -f "$WS/src/my_robot_controller/my_robot_controller/logger_node.py" ]]; then
    fail "logger_node.py not found in my_robot_controller package"
    exit 1
fi
ok "logger_node.py found"

# 6. matplotlib/pandas installed?
if ! python3 -c "import matplotlib, pandas" 2>/dev/null; then
    warn "matplotlib/pandas not found. Installing..."
    pip3 install matplotlib pandas --quiet
fi
ok "Python plotting deps: matplotlib, pandas"

echo ""
info "All pre-flight checks passed. Launching stack in tmux session '$SESSION'..."
echo ""

# ── Launch full stack in tmux ─────────────────────────────────────────────────
tmux kill-session -t "$SESSION" 2>/dev/null || true
sleep 1

tmux new-session -d -s "$SESSION" -x 220 -y 55

# Split: 5 panes
# [0] perception+tracking+control  [1] logger_node
# [2] latency_validator            [3] topic monitor
# [4] status bar
tmux split-window -h  -t "$SESSION:0"
tmux split-window -v  -t "$SESSION:0.0"
tmux split-window -v  -t "$SESSION:0.1"
tmux split-window -v  -t "$SESSION:0.2"

SRC="source $SETUP"

# Pane 0: Full drone tracking stack
tmux send-keys -t "$SESSION:0.0" \
    "$SRC && ros2 launch my_robot_controller drone_tracking.launch.py" Enter

# Pane 1: Logger node (THE FLIGHT RECORDER)
tmux send-keys -t "$SESSION:0.1" \
    "sleep 5 && $SRC && ros2 run my_robot_controller logger_node" Enter

# Pane 2: Latency validator (background stats)
if [[ -f "$SCRIPTS/latency_validator.py" ]]; then
    tmux send-keys -t "$SESSION:0.2" \
        "sleep 8 && $SRC && python3 $SCRIPTS/latency_validator.py" Enter
else
    tmux send-keys -t "$SESSION:0.2" \
        "sleep 8 && $SRC && ros2 topic hz /mavros/setpoint_velocity/cmd_vel" Enter
fi

# Pane 3: Live topic monitor
tmux send-keys -t "$SESSION:0.3" \
    "sleep 6 && $SRC && watch -n 2 'ros2 topic hz /tracking/status --window 20 2>/dev/null'" Enter

# Pane 4: Status bar
tmux send-keys -t "$SESSION:0.4" \
    "echo 'Endurance monitor running in main terminal. Attach: tmux attach -t $SESSION'" Enter

info "tmux session launched. Attach with: tmux attach -t $SESSION"
echo ""

# ── Wait for OFFBOARD + ARMED + stable before starting timer ─────────────────
# The control_node startup sequence is:
#   HEARTBEAT (~2.5s) → REQ_OFFBOARD → REQ_ARM → TAKEOFF (4s) → TRACKING
# We wait until the drone is BOTH in OFFBOARD mode AND armed, then add a
# 15s stabilisation hold so the takeoff completes before the clock starts.
STARTUP_GRACE=150
info "Waiting for OFFBOARD+ARMED steady state (max ${STARTUP_GRACE}s)..."
OFFBOARD_WAIT=0
STABLE_MODE=""
STABLE_ARM=0
while [[ $OFFBOARD_WAIT -lt $STARTUP_GRACE ]]; do
    STATE_RAW=$(timeout 3s ros2 topic echo /mavros/state --once 2>/dev/null || echo "")
    STABLE_MODE=$(echo "$STATE_RAW" | grep -o "mode: [A-Z0-9]*" | awk '{print $2}' || echo "UNKNOWN")
    STABLE_ARM=$(echo  "$STATE_RAW" | grep -c "armed: true" || echo "0")
    echo -ne "\r  Waiting... mode=${STABLE_MODE}  armed=${STABLE_ARM}  (${OFFBOARD_WAIT}s / ${STARTUP_GRACE}s)"
    if [[ "$STABLE_MODE" == "OFFBOARD" && "$STABLE_ARM" -ge 1 ]]; then
        echo ""
        ok "OFFBOARD+ARMED confirmed at ${OFFBOARD_WAIT}s — holding 15s for takeoff to complete..."
        sleep 15
        ok "Stable — starting $((ENDURANCE_SECONDS / 60))-minute endurance clock"
        break
    fi
    sleep 5
    OFFBOARD_WAIT=$((OFFBOARD_WAIT + 5))
done

if [[ "$STABLE_MODE" != "OFFBOARD" ]]; then
    warn "OFFBOARD mode not reached in ${STARTUP_GRACE}s — starting timer anyway"
fi
MODE="$STABLE_MODE"

echo ""
log "=== ENDURANCE TEST STARTED === duration=${ENDURANCE_SECONDS}s ==="

# ── Endurance monitoring loop ─────────────────────────────────────────────────
ELAPSED=0
while [[ $ELAPSED -lt $ENDURANCE_SECONDS ]]; do
    sleep "$MONITOR_INTERVAL"
    ELAPSED=$((ELAPSED + MONITOR_INTERVAL))
    REMAINING=$((ENDURANCE_SECONDS - ELAPSED))

    # Check FCU state — retry once on UNKNOWN to filter WSL2 topic-echo timeouts
    STATE_RAW=$(timeout 4s ros2 topic echo /mavros/state --once 2>/dev/null || echo "")
    MODE=$(echo "$STATE_RAW" | grep -o "mode: [A-Z0-9]*" | awk '{print $2}' || echo "UNKNOWN")
    if [[ "$MODE" == "UNKNOWN" || -z "$MODE" ]]; then
        sleep 2
        STATE_RAW=$(timeout 4s ros2 topic echo /mavros/state --once 2>/dev/null || echo "")
        MODE=$(echo "$STATE_RAW" | grep -o "mode: [A-Z0-9]*" | awk '{print $2}' || echo "UNKNOWN")
    fi
    CONN=$(echo "$STATE_RAW" | grep -c "connected: true" || echo "0")
    ARM=$(echo  "$STATE_RAW" | grep -c "armed: true"     || echo "0")
    STATUS=$(timeout 2s ros2 topic echo /tracking/status --once 2>/dev/null | grep -o "data: '[A-Z]*'" | tr -d "data: '" || echo "UNKNOWN")

    PROGRESS_PCT=$(( ELAPSED * 100 / ENDURANCE_SECONDS ))
    BAR_FILLED=$(( PROGRESS_PCT / 5 ))
    BAR=$(printf '#%.0s' $(seq 1 $BAR_FILLED))$(printf '.%.0s' $(seq $((BAR_FILLED+1)) 20))

    if [[ "$MODE" != "OFFBOARD" ]]; then
        MODE_DROP_COUNT=$((MODE_DROP_COUNT + 1))
        log "MODE DROP #${MODE_DROP_COUNT}: mode=${MODE}  elapsed=${ELAPSED}s"
        fail "OFFBOARD mode dropped! mode=${MODE}  (drop #${MODE_DROP_COUNT})"
    elif [[ "$CONN" -eq 0 ]]; then
        HEARTBEAT_TIMEOUT_COUNT=$((HEARTBEAT_TIMEOUT_COUNT + 1))
        log "HEARTBEAT TIMEOUT #${HEARTBEAT_TIMEOUT_COUNT}: elapsed=${ELAPSED}s"
        warn "FCU heartbeat timeout! (count #${HEARTBEAT_TIMEOUT_COUNT})"
    else
        log "OK mode=${MODE} armed=${ARM} status=${STATUS} elapsed=${ELAPSED}s remaining=${REMAINING}s"
        echo -e "  [${BAR}] ${ELAPSED}s/${ENDURANCE_SECONDS}s  mode=${G}${MODE}${NC}  status=${STATUS}  remaining=${REMAINING}s"
    fi
done

# ── Stop all nodes ────────────────────────────────────────────────────────────
echo ""
info "Timer complete — stopping all nodes..."
log "=== ENDURANCE TEST COMPLETE ==="

tmux send-keys -t "$SESSION:0.0" C-c
sleep 1
tmux send-keys -t "$SESSION:0.1" C-c
sleep 1
tmux send-keys -t "$SESSION:0.2" C-c
sleep 0.5
tmux send-keys -t "$SESSION:0.3" C-c
sleep 0.5

# Give logger_node a moment to flush its CSV
sleep 3

# Find the latest CSV
LATEST_CSV=$(ls -t "$LOG_DIR"/session_*.csv 2>/dev/null | head -1 || echo "")

# ── Final endurance report ────────────────────────────────────────────────────
ACTUAL_DURATION=$(( $(date +%s) - START_EPOCH ))
echo ""
echo -e "${B}+=========================================================================+${NC}"
echo -e "${B}|   WEEK 3 ENDURANCE REPORT — Reformerz Healthcare                       |${NC}"
echo -e "${B}+=========================================================================+${NC}"
echo -e "  Duration ran     : ${ACTUAL_DURATION}s  (target: ${ENDURANCE_SECONDS}s)"
echo -e "  OFFBOARD drops   : ${MODE_DROP_COUNT}  (target: 0)"
echo -e "  Heartbeat timeout: ${HEARTBEAT_TIMEOUT_COUNT}  (target: 0)"
if [[ -n "$LATEST_CSV" ]]; then
    ROWS=$(wc -l < "$LATEST_CSV" || echo "?")
    echo -e "  CSV log          : $LATEST_CSV"
    echo -e "  CSV rows         : $ROWS"
fi
echo -e "  Endurance log    : $ENDURANCE_LOG"
echo ""

if [[ $MODE_DROP_COUNT -eq 0 && $HEARTBEAT_TIMEOUT_COUNT -eq 0 ]]; then
    echo -e "${G}+=========================================================================+${NC}"
    echo -e "${G}|   RESULT: PASS — Zero mode drops over ${ENDURANCE_SECONDS}s endurance run          |${NC}"
    echo -e "${G}|   This is Engineering-Grade evidence for your report.                  |${NC}"
    echo -e "${G}+=========================================================================+${NC}"
    VERDICT="PASS"
else
    echo -e "${R}+=========================================================================+${NC}"
    echo -e "${R}|   RESULT: FAIL — ${MODE_DROP_COUNT} mode drop(s), ${HEARTBEAT_TIMEOUT_COUNT} heartbeat timeout(s)        |${NC}"
    echo -e "${R}|   Check endurance log for timestamps of each failure.                  |${NC}"
    echo -e "${R}+=========================================================================+${NC}"
    VERDICT="FAIL"
fi

echo ""
echo "Full log saved to: $ENDURANCE_LOG"
echo ""

# ── Prompt to generate plots ──────────────────────────────────────────────────
if [[ -n "$LATEST_CSV" ]]; then
    echo -e "${Y}Session data ready: $LATEST_CSV${NC}"
    echo ""
    read -r -p "Generate analysis plots now? [Y/n]: " PLOT_CHOICE
    PLOT_CHOICE="${PLOT_CHOICE:-Y}"
    if [[ "$PLOT_CHOICE" =~ ^[Yy]$ ]]; then
        info "Running generate_plots.py..."
        source "$SETUP"
        python3 "$SCRIPTS/generate_plots.py" --csv "$LATEST_CSV"
    else
        echo ""
        echo "To generate plots later:"
        echo "  source $SETUP"
        echo "  python3 $SCRIPTS/generate_plots.py --csv $LATEST_CSV"
    fi
else
    warn "No session CSV found in $LOG_DIR — logger_node may not have started."
    echo "  Start it manually: ros2 run my_robot_controller logger_node"
fi

echo ""
info "tmux session still running. Attach: tmux attach -t $SESSION"
info "Kill session:                       tmux kill-session -t $SESSION"
echo ""
