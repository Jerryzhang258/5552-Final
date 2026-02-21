#!/bin/bash
# 一键跑通 UR5 仿真 demo（不含 Isaac Sim 与 Chatbot）
# 使用前请：1) 在 Isaac Sim 中打开并播放 SPARC.usd  2) 本脚本会编译并启动 RVIZ + MoveIt + 抓放节点

set -e
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR"

echo "==> 工作空间: $SCRIPT_DIR"
echo "==> 编译（使用系统 Python，避免 Conda 下缺 empy）..."
source /opt/ros/humble/setup.bash
# 若在 conda base 下编译，CMake 会用到 conda 的 Python，缺少 ROS 的 empy；此处改用系统 Python
export PATH="/usr/bin:/usr/local/bin:$PATH"
colcon build --symlink-install

echo "==> 加载环境并启动 RVIZ + MoveIt + UR5 抓放..."
source install/setup.bash

chmod +x main_launch.sh
./main_launch.sh

echo "==> 已启动。请在 RVIZ 中将机械臂移动到 arm_ready，再在 Isaac Sim 中观察抓放。"
echo "==> 需要聊天控制时，新开终端执行: cd $SCRIPT_DIR && ./pick_place_chatbot_ui/launch.sh （需先 conda activate ros2_humble_py310）"
