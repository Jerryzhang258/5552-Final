#!/bin/bash
# 一键 demo：启动 Isaac Sim + MoveIt/RVIZ + 相机画面
# 用法：./run_simple_demo.sh
# Ctrl+C 结束所有进程

DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$DIR"

# --- conda ---
CONDA_SH=""
for d in "$HOME/miniconda3" "$HOME/Miniconda3" "$HOME/anaconda3"; do
  [ -f "$d/etc/profile.d/conda.sh" ] && CONDA_SH="$d/etc/profile.d/conda.sh" && break
done
if [ -z "$CONDA_SH" ]; then echo "[ERROR] conda not found"; exit 1; fi
source "$CONDA_SH"
conda activate ros2_humble_py310
source /opt/ros/humble/setup.bash
source install/setup.bash 2>/dev/null || true

PIDS=()
cleanup() {
  echo ""
  echo "==> Stopping all ..."
  for p in "${PIDS[@]}"; do kill "$p" 2>/dev/null; done
  wait 2>/dev/null
  echo "==> Done."
  exit 0
}
trap cleanup SIGINT SIGTERM

echo "============================================"
echo "  NLP Pick-and-Place Demo"
echo "============================================"
echo ""

# --- 1) Isaac Sim ---
echo "==> [1/3] Starting Isaac Sim ..."
echo "    (takes ~30s to load)"
PYTHONPATH="/opt/isaac:${PYTHONPATH:-}" python -c "
import sys
sys.argv = ['isaacsim', '/opt/isaac/omni/apps/omni.app.full.kit',
            '--ext-folder', '/opt/isaac/isaacsim/exts',
            '--enable', 'isaacsim.ros2.bridge']
from isaacsim import main
main()
" &
PIDS+=($!)
echo "    Isaac Sim PID: ${PIDS[-1]}"
echo ""
echo "    >>> Isaac Sim 启动后请手动操作："
echo "    >>> 1. File -> Open -> SPARC_ur5_fixed.usd"
echo "    >>> 2. Press Play"
echo ""
read -p "    按回车继续（等 Isaac Sim 加载好并按了 Play 后）..."

# --- 2) MoveIt + RVIZ ---
echo ""
echo "==> [2/3] Starting MoveIt + RVIZ ..."
ros2 launch ur5_moveit_config demo.launch.py &
PIDS+=($!)
sleep 6

# --- 3) 相机画面 ---
echo "==> [3/3] Opening camera view ..."
HAS_RGB=$(timeout 3 ros2 topic list 2>/dev/null | grep -c "/rgb")
if [ "$HAS_RGB" -gt 0 ]; then
  ros2 run rqt_image_view rqt_image_view --ros-args -r image:=/rgb &
  PIDS+=($!)
  echo "    Camera view opened."
else
  echo "    /rgb not found. Run isaac_setup_camera_publisher.py in Isaac Script Editor."
fi

echo ""
echo "============================================"
echo "  Demo Ready!"
echo "============================================"
echo ""
echo "  Isaac Sim: table + cubes + UR5 arm"
echo "  RVIZ:      motion planning interface"
echo ""
echo "  In RVIZ -> MotionPlanning:"
echo "    Goal State -> arm_ready -> Plan & Execute"
echo ""
echo "  Press Ctrl+C to stop all."
echo ""

wait
