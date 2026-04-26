#!/usr/bin/env bash
# =============================================================================
# run_week2.sh — One-script Week 2 Demo for Shiva
# =============================================================================
# Opens 4 tmux panes automatically:
#   Pane 0 (top-left)  : test_image_pub  → /camera/image_raw at 30 Hz
#   Pane 1 (top-right) : perception_node → /detected_objects
#   Pane 2 (bot-left)  : ros2 topic hz   → live FPS readout
#   Pane 3 (bot-right) : ros2 topic echo → live detection labels
#
# After everything is confirmed running, starts the 30-min stress test
# in the SAME terminal you launched this script from.
#
# Requirements: tmux installed (sudo apt install tmux)
#
# Usage:
#   chmod +x ~/ros2_ws/src/my_robot_controller/scripts/run_week2.sh
#   ~/ros2_ws/src/my_robot_controller/scripts/run_week2.sh
# =============================================================================

set -e

WS=~/ros2_ws
SCRIPTS=$WS/src/my_robot_controller/scripts
SETUP=$WS/install/setup.bash

# ── Colour helpers ────────────────────────────────────────────────────────────
RED='\033[0;31m'; GRN='\033[0;32m'; YLW='\033[1;33m'; NC='\033[0m'
ok()   { echo -e "${GRN}[OK]${NC} $*"; }
warn() { echo -e "${YLW}[WARN]${NC} $*"; }
die()  { echo -e "${RED}[ERROR]${NC} $*"; exit 1; }

# ── Pre-flight checks ─────────────────────────────────────────────────────────
echo ""
echo "============================================================"
echo "  Week 2 Pre-flight Checks"
echo "============================================================"

# 1. Workspace built?
[[ -f $SETUP ]] || die "Workspace not built. Run: cd $WS && colcon build --packages-select my_robot_controller"
source $SETUP
ok "Workspace sourced: $SETUP"

# 2. Python dependencies?
python3 -c "import ultralytics, openvino, psutil, cv2" 2>/dev/null \
  || die "Missing pip deps. Run: pip3 install ultralytics openvino psutil opencv-python"
ok "Python deps: ultralytics, openvino, psutil, cv2"

# 3. ROS2 vision_msgs?
python3 -c "from vision_msgs.msg import Detection2DArray" 2>/dev/null \
  || die "Missing vision_msgs. Run: sudo apt install ros-humble-vision-msgs ros-humble-cv-bridge"
ok "ROS2 msgs: vision_msgs, cv_bridge"

# 4. ONNX model ready? (128×128 — fastest on WSL2 CPU)
MODEL_FILE=~/.ros/yolov8n_128.onnx
if [[ -f $MODEL_FILE ]]; then
    ok "ONNX model: $MODEL_FILE"
else
    warn "ONNX model not found at $MODEL_FILE"
    echo "  → Running one-time export at 128×128 (takes ~30 seconds)..."
    python3 - <<'EOF'
from ultralytics import YOLO
from pathlib import Path
import shutil

dst = Path.home() / ".ros" / "yolov8n_128.onnx"
dst.parent.mkdir(parents=True, exist_ok=True)

pt = Path.home() / ".ros" / "yolov8n.pt"
if not pt.exists():
    m = YOLO("yolov8n.pt")
    cwd_pt = Path("yolov8n.pt")
    if cwd_pt.exists():
        shutil.move(str(cwd_pt), str(pt))
else:
    m = YOLO(str(pt))

m.export(format="onnx", half=False, dynamic=False, imgsz=128)

# ultralytics saves beside the .pt or in cwd
for candidate in [pt.with_suffix(".onnx"), Path("yolov8n.onnx")]:
    if candidate.exists():
        shutil.move(str(candidate), str(dst))
        print(f"Saved to {dst}")
        break
else:
    print("WARNING: could not find exported .onnx — node will re-export at startup")
EOF
    [[ -f $MODEL_FILE ]] && ok "ONNX export complete: $MODEL_FILE" \
                         || warn "Export may have landed elsewhere; node will re-export at startup"
fi

# 5. Test image ready?
TEST_IMG=~/.ros/test_detection_image.jpg
if [[ ! -f $TEST_IMG ]]; then
    warn "Test image not cached; test_image_pub.py will download it on first run."
fi

echo ""
echo "============================================================"
echo "  All checks passed. Launching in tmux..."
echo "============================================================"
echo ""

# ── Launch tmux session ───────────────────────────────────────────────────────
SESSION="week2"

# Kill previous session if it exists
tmux kill-session -t $SESSION 2>/dev/null || true
sleep 0.5

tmux new-session -d -s $SESSION -x 220 -y 50

# Split into 4 panes: top-left, top-right, bottom-left, bottom-right
tmux split-window -h   -t $SESSION:0
tmux split-window -v   -t $SESSION:0.0
tmux split-window -v   -t $SESSION:0.1

SOURCE_CMD="source $SETUP"

# ── Pane 0 (top-left): test_image_pub ────────────────────────────────────────
tmux send-keys -t $SESSION:0.0 \
    "$SOURCE_CMD && python3 $SCRIPTS/test_image_pub.py" Enter

# ── Pane 1 (top-right): perception_node ──────────────────────────────────────
tmux send-keys -t $SESSION:0.1 \
    "$SOURCE_CMD && ros2 launch my_robot_controller perception.launch.py" Enter

# Give the node time to load the model before starting monitors
sleep 3

# ── Pane 2 (bottom-left): FPS heartbeat ──────────────────────────────────────
tmux send-keys -t $SESSION:0.2 \
    "$SOURCE_CMD && echo '--- FPS HEARTBEAT ---' && ros2 topic hz /detected_objects" Enter

# ── Pane 3 (bottom-right): Detection echo (AI heartbeat) ─────────────────────
tmux send-keys -t $SESSION:0.3 \
    "$SOURCE_CMD && echo '--- AI HEARTBEAT ---' && ros2 topic echo /detected_objects --field detections" Enter

echo "tmux session '$SESSION' launched."
echo "  Attach to watch: tmux attach -t $SESSION"
echo ""
echo "============================================================"
echo "  Waiting 90 seconds for node warm-up (OpenVINO load"
echo "  + warm-up inference takes up to 60s on first run)..."
echo "  Watch the tmux panes: tmux attach -t week2"
echo "============================================================"
echo ""

# Wait and show countdown so user knows it's not frozen
for i in $(seq 90 -10 10); do
    echo "  Starting stress test in ${i}s  (Ctrl+C to skip wait)"
    sleep 10
done

# Final check: verify /detected_objects is actually publishing before starting
echo ""
echo "Verifying /detected_objects is live..."
source "$SETUP"
DET_HZ=$(timeout 8 ros2 topic hz /detected_objects 2>/dev/null | grep "average rate" | awk '{print $3}' | tr -d ':')
if [[ -z "$DET_HZ" ]]; then
    warn "/detected_objects not detected yet — check tmux panes for errors:"
    warn "  tmux attach -t $SESSION"
    warn "Continuing anyway — if FPS stays 0, the perception_node may have crashed."
else
    ok "/detected_objects is live at ~${DET_HZ} Hz"
fi
echo ""

# ── 30-minute stress test (runs right here in the current terminal) ───────────
python3 $SCRIPTS/stress_test.py

# Exit code of stress_test.py is 0 = PASS, 1 = FAIL
RESULT=$?
echo ""
if [[ $RESULT -eq 0 ]]; then
    echo -e "${GRN}============================================================${NC}"
    echo -e "${GRN}  Week 2 COMPLETE — Screenshot the terminal above for Shiva ${NC}"
    echo -e "${GRN}============================================================${NC}"
else
    echo -e "${RED}============================================================${NC}"
    echo -e "${RED}  Stress test FAILED — see individual metric lines above    ${NC}"
    echo -e "${RED}============================================================${NC}"
fi

echo ""
echo "tmux session still running. To check live view:"
echo "  tmux attach -t $SESSION"
echo "To kill all processes:"
echo "  tmux kill-session -t $SESSION"
