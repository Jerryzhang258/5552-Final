#!/bin/bash
# Chatbot 网页（http://localhost:8000）需要 fastapi/uvicorn，在 conda 环境里装一次即可

if ! python3 -c "import fastapi" 2>/dev/null; then
  CONDA_SH=""
  for d in "$HOME/miniconda3" "$HOME/miniconda3-latest" "$HOME/Miniconda3" "$HOME/anaconda3"; do
    [ -f "$d/etc/profile.d/conda.sh" ] && CONDA_SH="$d/etc/profile.d/conda.sh" && break
  done
  if [ -n "$CONDA_SH" ]; then
    source "$CONDA_SH"
    conda activate ros2_humble_py310 2>/dev/null || true
  fi
  if ! python3 -c "import fastapi" 2>/dev/null; then
    echo "[ERROR] Chatbot 网页需要 fastapi，当前环境未安装。"
    echo "  用途：浏览器打开 http://localhost:8000 的对话界面依赖 fastapi/uvicorn。"
    echo "  解决（只需做一次）："
    echo "    conda activate ros2_humble_py310"
    echo "    pip install -r requirements.txt"
    echo "  若没有该环境，先创建： conda create -n ros2_humble_py310 python=3.10 -y"
    echo "然后再运行本脚本。"
    exit 1
  fi
  echo "[OK] 已使用 conda 环境: ros2_humble_py310"
fi

WORKSPACE_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$WORKSPACE_ROOT" || exit 1

source /opt/ros/humble/setup.bash
source "$WORKSPACE_ROOT/install/setup.bash"

cd "$(dirname "$0")" || exit 1

# 后台启动 target_publisher
python3 target_publisher.py &
TP_PID=$!

# 后台启动 YOLO 检测
ros2 run yolov8obb_object_detection yolov8_obb_publisher &
YOLO_PID=$!

# 后台启动 cmd_bridge (FastAPI + WebSocket)
python3 cmd_bridge.py &
BRIDGE_PID=$!

sleep 3

# 可选：用 Firefox 打开聊天界面（若无 Firefox 可手动浏览器访问 http://localhost:8000）
if command -v firefox &>/dev/null; then
  firefox --new-window "http://localhost:8000" --width=400 --height=700 &
fi

echo "Chatbot 已启动: http://localhost:8000  按 Ctrl+C 结束并清理子进程。"

cleanup() {
  kill $TP_PID $YOLO_PID $BRIDGE_PID 2>/dev/null
  exit 0
}
trap cleanup SIGINT SIGTERM

wait $BRIDGE_PID
cleanup
