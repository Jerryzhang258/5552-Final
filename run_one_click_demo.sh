#!/bin/bash
# 一键完成：编译 → 启动 Isaac Sim → 启动 RVIZ+MoveIt+UR5 抓放+Chatbot
# 运行：./run_one_click_demo.sh

set -e
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR"

echo "=============================================="
echo "  一键 Demo：编译 + Isaac Sim + ROS 全栈"
echo "=============================================="

# ---------- 1) 编译 ----------
echo ""
echo "[1/4] 编译工作空间（使用系统 Python）..."
source /opt/ros/humble/setup.bash
export PATH="/usr/bin:/usr/local/bin:$PATH"
colcon build --symlink-install
source install/setup.bash
chmod +x main_launch.sh pick_place_chatbot_ui/launch.sh 2>/dev/null || true

# ---------- 2) 启动 Isaac Sim ----------
CONDA_SH=""
for d in "$HOME/miniconda3" "$HOME/miniconda3-latest" "$HOME/Miniconda3" "$HOME/anaconda3"; do
  [ -f "$d/etc/profile.d/conda.sh" ] && CONDA_SH="$d/etc/profile.d/conda.sh" && break
done

echo ""
echo "[2/4] 启动 Isaac Sim（新窗口）..."
if [ -n "$CONDA_SH" ]; then
  gnome-terminal --title "Isaac Sim" -- bash -c "
    source '$CONDA_SH'
    conda activate ros2_humble_py310
    cd '$SCRIPT_DIR' && ./scripts/launch_isaac_sim.sh
    exec bash
  "
else
  echo "    [跳过] 未找到 conda，请手动新开终端运行: ./scripts/launch_isaac_sim.sh"
fi

echo "    → 在 Isaac Sim 窗口中：File → Open → 选择 SPARC_ur5_fixed.usd → 点击 Play"
echo "    → 等待约 25 秒后将自动启动 RVIZ 等..."
sleep 25

# ---------- 3) RVIZ + MoveIt ----------
echo ""
echo "[3/4] 启动 RVIZ + MoveIt + UR5 抓放..."
gnome-terminal --title "RVIZ MoveIt" -- bash -c "
  cd '$SCRIPT_DIR' && source /opt/ros/humble/setup.bash && source install/setup.bash
  ros2 launch ur5_moveit_config demo.launch.py
  exec bash
"
sleep 3
gnome-terminal --title "UR5 Pick Place" -- bash -c "
  cd '$SCRIPT_DIR' && source /opt/ros/humble/setup.bash && source install/setup.bash
  ros2 run ur5_moveit_config ur5_pick_place_cpp_r
  exec bash
"
sleep 2
gnome-terminal --title "Trajectory->Isaac Relay" -- bash -c "
  cd '$SCRIPT_DIR' && source /opt/ros/humble/setup.bash && source install/setup.bash
  python3 scripts/trajectory_to_isaac_relay.py
  exec bash
"
sleep 1

# ---------- 4) Chatbot + YOLO ----------
echo ""
echo "[4/4] 启动 Chatbot（YOLO + 对话）..."
if [ -n "$CONDA_SH" ]; then
  gnome-terminal --title "Chatbot + YOLO" -- bash -c "
    source '$CONDA_SH'
    conda activate ros2_humble_py310
    cd '$SCRIPT_DIR' && ./pick_place_chatbot_ui/launch.sh
    exec bash
  "
else
  echo "    [跳过] 请手动新开终端: conda activate ros2_humble_py310 && cd $SCRIPT_DIR && ./pick_place_chatbot_ui/launch.sh"
fi

echo ""
echo "=============================================="
echo "  一键启动完成"
echo "=============================================="
echo "  1. Isaac Sim：若未打开场景，请 File → Open → SPARC_ur5_fixed.usd → Play"
echo "  2. RVIZ：将机械臂拖到 arm_ready 姿态"
echo "  3. 浏览器打开: http://localhost:8000  用自然语言控制抓取"
echo "=============================================="
