#!/bin/bash
# 机械臂不动时按链路排查：中继是否收到 action、是否在发 /isaac_joint_commands、Isaac 图是否接好

cd "$(dirname "$0")/.." || exit 1
source /opt/ros/humble/setup.bash
source install/setup.bash 2>/dev/null || true

echo "=== 1) 是否有 arm_controller/follow_joint_trajectory action 服务（应有 trajectory_to_isaac_relay）==="
ros2 action list | grep -E "follow_joint_trajectory|arm_controller" || echo "  [无] 请先启动 run_full_demo 或单独运行: python3 scripts/trajectory_to_isaac_relay.py"

echo ""
echo "=== 2) 在 RVIZ 里 Plan & Execute 一次后，看下面 5 秒内是否收到 /isaac_joint_commands ==="
echo "    若收到会打印 joint names 和 position；若无输出说明中继未发或未启动。"
timeout 5 ros2 topic echo /isaac_joint_commands --once 2>/dev/null || echo "  [超时/无数据] 请确认：① 已点 Plan & Execute ② Trajectory->Isaac Relay 终端在运行 ③ 该终端有 'Relayed trajectory' 日志"

echo ""
echo "=== 3) Isaac 侧必须做的（在 Isaac Sim 播放时执行一次并 Save）==="
echo "  Script Editor 里运行: scripts/isaac_debug_graph_connections.py"
echo "  然后菜单 File -> Save，再 Stop+Play 一次。"
