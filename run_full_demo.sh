#!/bin/bash
# 一键完整 demo：编译 + Isaac Sim + RVIZ+MoveIt + UR5 抓放 + 聊天界面（含 YOLO）
# 会在新终端自动启动 Isaac，请在其中打开并播放 SPARC_ur5_fixed.usd

set -e
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR"

# 提前找 conda，供 Isaac 和 Chatbot 用
CONDA_SH=""
for d in "$HOME/miniconda3" "$HOME/miniconda3-latest" "$HOME/Miniconda3" "$HOME/anaconda3"; do
  if [ -f "$d/etc/profile.d/conda.sh" ]; then
    CONDA_SH="$d/etc/profile.d/conda.sh"
    break
  fi
done

echo "==> 工作空间: $SCRIPT_DIR"
echo "==> 编译（使用系统 Python）..."
source /opt/ros/humble/setup.bash
export PATH="/usr/bin:/usr/local/bin:$PATH"
colcon build --symlink-install

echo "==> 加载环境..."
source install/setup.bash

chmod +x main_launch.sh pick_place_chatbot_ui/launch.sh scripts/launch_isaac_sim.sh scripts/check_isaac_arm_chain.sh 2>/dev/null || true

# --- 0) Isaac Sim（先启动，需较长时间加载）---
if [ -n "$CONDA_SH" ]; then
  gnome-terminal --title "Isaac Sim" -- bash -c "
    source '$CONDA_SH'
    conda activate ros2_humble_py310
    cd '$SCRIPT_DIR' && ./scripts/launch_isaac_sim.sh
    exec bash
  "
  echo "==> 已启动 Isaac Sim 终端，等待 25 秒加载..."
  sleep 25
  echo "==> 请在 Isaac 中打开场景 SPARC_ur5_fixed.usd 并点击 Play。"
  echo "==> 若场景里没有绿/红/黄方块：Window -> Script Editor 运行 scripts/isaac_add_placeholder_objects.py，再 File -> Save，方块会保存到该 USD 里。"
else
  echo "==> [WARN] 未找到 conda，请先手动新开终端运行: ./scripts/launch_isaac_sim.sh"
  read -p "     Isaac 启动后按回车继续..."
fi

# --- 1) RVIZ + MoveIt ---
gnome-terminal --title "RVIZ MoveIt" -- bash -c "
  cd '$SCRIPT_DIR' && source /opt/ros/humble/setup.bash && source install/setup.bash
  ros2 launch ur5_moveit_config demo.launch.py
  exec bash
"

sleep 3

# --- 2) UR5 Pick-Place 节点 ---
gnome-terminal --title "UR5 Pick Place" -- bash -c "
  cd '$SCRIPT_DIR' && source /opt/ros/humble/setup.bash && source install/setup.bash
  ros2 run ur5_moveit_config ur5_pick_place_cpp_r
  exec bash
"

sleep 2

# --- 2b) 轨迹中继：MoveIt Plan&Execute (action) -> /isaac_joint_commands -> Isaac Sim ---
gnome-terminal --title "Trajectory->Isaac Relay" -- bash -c "
  cd '$SCRIPT_DIR' && source /opt/ros/humble/setup.bash && source install/setup.bash
  python3 scripts/trajectory_to_isaac_relay.py
  exec bash
"

sleep 1

# --- 3) 聊天界面 + YOLO + 目标发布（需 conda ros2_humble_py310）---
if [ -z "$CONDA_SH" ]; then
  echo "==> [WARN] 未找到 conda，跳过 Chatbot。请手动新开终端执行："
  echo "     conda activate ros2_humble_py310 && cd $SCRIPT_DIR && ./pick_place_chatbot_ui/launch.sh"
else
  gnome-terminal --title "Chatbot + YOLO" -- bash -c "
    source '$CONDA_SH'
    conda activate ros2_humble_py310
    cd '$SCRIPT_DIR' && ./pick_place_chatbot_ui/launch.sh
    exec bash
  "
  echo "==> 已启动 RVIZ+MoveIt、UR5 抓放、Chatbot（YOLO + 对话）。"
fi

echo "==> 在 RVIZ 中将机械臂移到 arm_ready；浏览器打开 http://localhost:8000 使用聊天控制。"
