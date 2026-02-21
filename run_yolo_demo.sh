#!/bin/bash
# 一键 YOLO 抓取 demo
# 前提：Isaac Sim 已打开 SPARC_ur5_fixed.usd 并按了 Play
# 用法：./run_yolo_demo.sh [物体名] [盒子号]

set -e

LABEL="${1:-yellow_cube}"
BOX="${2:-1}"
DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$DIR"

# --- 激活 conda ---
CONDA_SH=""
for d in "$HOME/miniconda3" "$HOME/Miniconda3" "$HOME/anaconda3"; do
  [ -f "$d/etc/profile.d/conda.sh" ] && CONDA_SH="$d/etc/profile.d/conda.sh" && break
done
if [ -z "$CONDA_SH" ]; then
  echo "[ERROR] conda not found"; exit 1
fi
source "$CONDA_SH"
conda activate ros2_humble_py310
echo "[OK] conda env: ros2_humble_py310, python: $(python3 --version)"

source /opt/ros/humble/setup.bash
source install/setup.bash 2>/dev/null || true

# --- 日志目录 ---
LOG_DIR="$DIR/.demo_logs"
mkdir -p "$LOG_DIR"

PIDS=()
cleanup() {
  echo ""
  echo "==> Stopping all nodes ..."
  for p in "${PIDS[@]}"; do kill "$p" 2>/dev/null; done
  wait 2>/dev/null
  echo "==> Done. Logs in $LOG_DIR/"
  exit 0
}
trap cleanup SIGINT SIGTERM

echo "============================================"
echo "  YOLO Pick-and-Place Demo"
echo "  Target: $LABEL -> box $BOX"
echo "============================================"

# 1) MoveIt
echo ""
echo "==> [1/5] Starting MoveIt ..."
ros2 launch ur5_moveit_config demo.launch.py > "$LOG_DIR/moveit.log" 2>&1 &
PIDS+=($!)
sleep 6
echo "    MoveIt PID: ${PIDS[-1]}"

# 2) Trajectory relay
echo "==> [2/5] Starting trajectory relay ..."
python3 scripts/trajectory_to_isaac_relay.py > "$LOG_DIR/relay.log" 2>&1 &
PIDS+=($!)
sleep 1
echo "    Relay PID: ${PIDS[-1]}"

# 3) Pick-place node
echo "==> [3/5] Starting pick-place node ..."
ros2 run ur5_moveit_config ur5_pick_place_cpp_r > "$LOG_DIR/pickplace.log" 2>&1 &
PIDS+=($!)
sleep 2
echo "    Pick-place PID: ${PIDS[-1]}"

# 4) YOLO + target publisher
echo "==> [4/5] Starting YOLO + target_publisher ..."
python3 pick_place_chatbot_ui/target_publisher.py > "$LOG_DIR/target_pub.log" 2>&1 &
PIDS+=($!)
ros2 run yolov8obb_object_detection yolov8_obb_publisher > "$LOG_DIR/yolo.log" 2>&1 &
PIDS+=($!)
sleep 4

# 检查 YOLO 是否还活着
if ! kill -0 "${PIDS[-1]}" 2>/dev/null; then
  echo "    [ERROR] YOLO node crashed! Check $LOG_DIR/yolo.log:"
  tail -5 "$LOG_DIR/yolo.log"
  echo ""
  echo "    Fix: pip install 'numpy<2' then re-run."
  cleanup
fi
echo "    YOLO PID: ${PIDS[-1]}"

# 检查 /rgb topic
echo ""
echo "==> Checking /rgb topic ..."
timeout 3 ros2 topic hz /rgb 2>/dev/null | head -2 || echo "    [WARN] /rgb not receiving. Is Isaac Sim playing?"

# 5) 发抓取指令
echo ""
echo "==> [5/5] Sending pick command: $LABEL -> box $BOX"
ros2 topic pub --once /target_class_cmd std_msgs/msg/String "{data: '$LABEL,$BOX'}"

echo ""
echo "==> Command sent!"
echo "==> Watching target_publisher for YOLO detection ..."
echo "    (waiting for YOLO to detect '$LABEL' and publish /target_point)"
echo ""

# 实时显示 target_publisher 和 pick-place 的日志
tail -f "$LOG_DIR/target_pub.log" "$LOG_DIR/pickplace.log" &
PIDS+=($!)

wait
