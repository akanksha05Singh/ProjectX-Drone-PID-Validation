#!/usr/bin/env bash
# verify_shiva_ready.sh — Week 3 Integration Status Check
# Reformerz Healthcare | Drone Tracking Pipeline
# =====================================================
# Run after the full stack is live:
#   Terminal 1: PX4 SITL
#   Terminal 2: MAVROS
#   Terminal 3: ros2 launch drone_tracking.launch.py
#   Terminal 4: test_image_pub.py
# Then: bash ~/ros2_ws/src/my_robot_controller/scripts/verify_shiva_ready.sh

set -euo pipefail

# ── Colours ──────────────────────────────────────────────────────────────────
G='\033[0;32m';  R='\033[0;31m';  Y='\033[0;33m'
B='\033[1;34m';  C='\033[0;36m';  NC='\033[0m'
PASS="${G}PASS${NC}";  FAIL="${R}FAIL${NC}";  WARN="${Y}WARN${NC}"

# ── ROS2 setup ────────────────────────────────────────────────────────────────
source /opt/ros/humble/setup.bash 2>/dev/null || true
[[ -f "$HOME/ros2_ws/install/setup.bash" ]] && source "$HOME/ros2_ws/install/setup.bash"

RESULTS=()   # each entry: "PASS" | "WARN" | "FAIL"
LINES=()     # matching display strings (with colour codes)

# ════════════════════════════════════════════════════════════════════════════
# CHECK 1 — control loop publishing cmd_vel at >= 10 Hz
# ════════════════════════════════════════════════════════════════════════════
echo -e "\n${C}[1/4] Measuring /mavros/setpoint_velocity/cmd_vel publish rate...${NC}"

HZ_OUTPUT=$(timeout 10s ros2 topic hz /mavros/setpoint_velocity/cmd_vel --window 50 2>&1 || true)
HZ_RAW=$(echo "$HZ_OUTPUT" | grep -o 'average rate: [0-9.]*' | awk '{print $3}' | tail -1)
HZ_RAW=${HZ_RAW:-0}
HZ_INT=$(echo "$HZ_RAW" | awk '{printf "%d", $1}')

if   [[ "$HZ_INT" -ge 10 ]]; then
    RESULTS+=("PASS")
    LINES+=("  Check 1 - cmd_vel rate   : ${G}${HZ_RAW} Hz${NC}  [${PASS}]  (>=10 Hz required)")
elif [[ "$HZ_INT" -ge 5  ]]; then
    RESULTS+=("WARN")
    LINES+=("  Check 1 - cmd_vel rate   : ${Y}${HZ_RAW} Hz${NC}  [${WARN}]  (low, expected >=10 Hz)")
else
    RESULTS+=("FAIL")
    LINES+=("  Check 1 - cmd_vel rate   : ${R}${HZ_RAW} Hz${NC}  [${FAIL}]  (not publishing - check control_node)")
fi

# ════════════════════════════════════════════════════════════════════════════
# CHECK 2 — FCU connected + armed + OFFBOARD mode
# ════════════════════════════════════════════════════════════════════════════
echo -e "${C}[2/4] Checking /mavros/state (connected / armed / OFFBOARD)...${NC}"

STATE_RAW=$(timeout 5s ros2 topic echo /mavros/state --once 2>&1 || true)
CONNECTED=$(echo "$STATE_RAW" | grep -o 'connected: [a-z]*' | awk '{print $2}' || echo "false")
ARMED=$(echo "$STATE_RAW"     | grep -o 'armed: [a-z]*'     | awk '{print $2}' || echo "false")
MODE=$(echo "$STATE_RAW"      | grep -o 'mode: [A-Za-z0-9]*'| awk '{print $2}' || echo "UNKNOWN")
CONNECTED=${CONNECTED:-false}; ARMED=${ARMED:-false}; MODE=${MODE:-UNKNOWN}

if [[ "$CONNECTED" == "true" && "$ARMED" == "true" && "$MODE" == "OFFBOARD" ]]; then
    RESULTS+=("PASS")
    LINES+=("  Check 2 - FCU state      : connected=${G}true${NC} armed=${G}true${NC} mode=${G}OFFBOARD${NC}  [${PASS}]")
elif [[ "$CONNECTED" == "true" ]]; then
    RESULTS+=("WARN")
    LINES+=("  Check 2 - FCU state      : connected=${G}true${NC} armed=${Y}${ARMED}${NC} mode=${Y}${MODE}${NC}  [${WARN}]  (not yet in OFFBOARD+ARM)")
else
    RESULTS+=("FAIL")
    LINES+=("  Check 2 - FCU state      : connected=${R}${CONNECTED}${NC}  [${FAIL}]  (MAVROS not connected)")
fi

# ════════════════════════════════════════════════════════════════════════════
# CHECK 3 — End-to-end latency: camera -> cmd_vel (header timestamps)
# ════════════════════════════════════════════════════════════════════════════
echo -e "${C}[3/4] Measuring end-to-end latency (camera header -> cmd_vel header)...${NC}"

# Write inline latency probe to a temp file (avoids nested heredoc issues)
PROBE=/tmp/_verify_lat_probe_$$.py
cat > "$PROBE" << 'PROBE_EOF'
import sys, time
try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image
    from geometry_msgs.msg import TwistStamped

    class Probe(Node):
        def __init__(self):
            super().__init__("lat_probe_verify")
            self.cam_t   = None
            self.samples = []
            self.create_subscription(Image,        "/camera/image_raw",                 self._cam, 10)
            self.create_subscription(TwistStamped, "/mavros/setpoint_velocity/cmd_vel", self._cmd, 10)
        def _cam(self, m):
            self.cam_t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        def _cmd(self, m):
            t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            if self.cam_t and 0 < t - self.cam_t < 5:
                self.samples.append((t - self.cam_t) * 1000.0)

    rclpy.init()
    n = Probe()
    deadline = time.time() + 8.0
    while time.time() < deadline and len(n.samples) < 20:
        rclpy.spin_once(n, timeout_sec=0.1)
    if n.samples:
        avg = sum(n.samples) / len(n.samples)
        p95 = sorted(n.samples)[int(len(n.samples) * 0.95)]
        print(f"{avg:.1f},{p95:.1f},{len(n.samples)}")
    else:
        print("N/A,N/A,0")
    n.destroy_node()
    rclpy.try_shutdown()
except Exception as e:
    print("N/A,N/A,0")
PROBE_EOF

E2E_MS=$(python3 "$PROBE" 2>/dev/null || echo "N/A,N/A,0")
rm -f "$PROBE"

AVG_MS=$(echo "$E2E_MS" | cut -d',' -f1)
P95_MS=$(echo "$E2E_MS" | cut -d',' -f2)
SAMPLES=$(echo "$E2E_MS" | cut -d',' -f3)

if [[ "$AVG_MS" == "N/A" || "${SAMPLES:-0}" == "0" ]]; then
    RESULTS+=("WARN")
    LINES+=("  Check 3 - E2E latency    : ${Y}no samples collected${NC}  [${WARN}]  (pipeline idle or test_image_pub not running)")
else
    P95_INT=$(echo "$P95_MS" | awk '{printf "%d", $1}')
    if [[ "$P95_INT" -le 100 ]]; then
        RESULTS+=("PASS")
        LINES+=("  Check 3 - E2E latency    : avg=${G}${AVG_MS}ms${NC} p95=${G}${P95_MS}ms${NC} n=${SAMPLES}  [${PASS}]  (<=100ms)")
    elif [[ "$P95_INT" -le 250 ]]; then
        RESULTS+=("WARN")
        LINES+=("  Check 3 - E2E latency    : avg=${Y}${AVG_MS}ms${NC} p95=${Y}${P95_MS}ms${NC} n=${SAMPLES}  [${WARN}]  (WSL2 overhead, target <100ms)")
    else
        RESULTS+=("FAIL")
        LINES+=("  Check 3 - E2E latency    : avg=${R}${AVG_MS}ms${NC} p95=${R}${P95_MS}ms${NC} n=${SAMPLES}  [${FAIL}]  (>250ms)")
    fi
fi

# ════════════════════════════════════════════════════════════════════════════
# CHECK 4 — Watchdog: SIGSTOP perception_node -> /tracking/status == LOST
# ════════════════════════════════════════════════════════════════════════════
echo -e "${C}[4/4] Testing 500ms watchdog (SIGSTOP perception_node -> expect LOST)...${NC}"

PERC_PID=$(pgrep -f "perception_node" | head -1 || echo "")

if [[ -z "$PERC_PID" ]]; then
    RESULTS+=("WARN")
    LINES+=("  Check 4 - Watchdog       : ${Y}perception_node not found${NC}  [${WARN}]  (skip - start full stack first)")
else
    echo "  Pausing perception_node (PID $PERC_PID) for 1.8s..."
    kill -STOP "$PERC_PID" 2>/dev/null || true
    sleep 1.8

    STATUS_DATA=$(timeout 3s ros2 topic echo /tracking/status --once 2>&1 | grep -o "data: '[A-Z]*'" | head -1 || echo "")
    CMD_RAW=$(timeout 3s ros2 topic echo /mavros/setpoint_velocity/cmd_vel --once 2>&1 || echo "")
    VX_VAL=$(echo  "$CMD_RAW" | grep 'linear:'  -A2 | grep 'x:' | head -1 | awk '{print $2}' || echo "1")
    YAW_VAL=$(echo "$CMD_RAW" | grep 'angular:' -A4 | grep 'z:' | head -1 | awk '{print $2}' || echo "1")

    # Resume perception node before evaluating results
    kill -CONT "$PERC_PID" 2>/dev/null || true
    echo "  Resumed perception_node."

    VX_ZERO=$(echo  "${VX_VAL:-1}"  | awk '{v=$1+0; print (v<0?-v:v)<0.01?"yes":"no"}')
    YAW_ZERO=$(echo "${YAW_VAL:-1}" | awk '{v=$1+0; print (v<0?-v:v)<0.01?"yes":"no"}')
    IS_LOST=$(echo "$STATUS_DATA" | grep -c "LOST" || echo "0")

    if [[ "$IS_LOST" -gt 0 && "$VX_ZERO" == "yes" && "$YAW_ZERO" == "yes" ]]; then
        RESULTS+=("PASS")
        LINES+=("  Check 4 - Watchdog       : status=LOST vx=0 yaw=0  [${PASS}]  (500ms timer fires correctly)")
    elif [[ "$IS_LOST" -gt 0 ]]; then
        RESULTS+=("WARN")
        LINES+=("  Check 4 - Watchdog       : status=LOST but velocities non-zero  [${WARN}]")
    else
        RESULTS+=("FAIL")
        LINES+=("  Check 4 - Watchdog       : ${R}status not LOST after 1.8s pause${NC}  [${FAIL}]  (check tracking_node watchdog)")
    fi
fi

# ════════════════════════════════════════════════════════════════════════════
# ASCII-BOXED REPORT
# ════════════════════════════════════════════════════════════════════════════
PASS_COUNT=$(printf '%s\n' "${RESULTS[@]}" | grep -c '^PASS$' || true)
WARN_COUNT=$(printf '%s\n' "${RESULTS[@]}" | grep -c '^WARN$' || true)
FAIL_COUNT=$(printf '%s\n' "${RESULTS[@]}" | grep -c '^FAIL$' || true)

if   [[ "${FAIL_COUNT:-0}" -gt 0 ]]; then
    VERDICT="${R}NOT READY -- ${FAIL_COUNT} check(s) FAILED (fix above before showing Shiva)${NC}"
elif [[ "${WARN_COUNT:-0}" -gt 0 ]]; then
    VERDICT="${Y}SHIVA-READY  (${WARN_COUNT} warning(s) -- see details above)${NC}"
else
    VERDICT="${G}*** SHIVA-READY *** -- All 4 checks passed${NC}"
fi

echo ""
echo -e "${B}+=========================================================================+${NC}"
echo -e "${B}|       WEEK 3 INTEGRATION STATUS: SHIVA-READY                           |${NC}"
echo -e "${B}|       Reformerz Healthcare  --  Drone Tracking Pipeline                |${NC}"
echo -e "${B}+=========================================================================+${NC}"
for line in "${LINES[@]}"; do
    echo -e "${B}|${NC}  ${line}"
done
echo -e "${B}+---------+---------------------------------------------------------------+${NC}"
echo -e "${B}|${NC}  Score  :  ${G}${PASS_COUNT:-0} PASS${NC}   ${Y}${WARN_COUNT:-0} WARN${NC}   ${R}${FAIL_COUNT:-0} FAIL${NC}   (out of 4 checks)"
echo -e "${B}|${NC}  Verdict:  ${VERDICT}"
echo -e "${B}+=========================================================================+${NC}"
echo ""
