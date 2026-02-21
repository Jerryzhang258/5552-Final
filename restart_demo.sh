#!/bin/bash
# 全部重启：先结束本次 demo 相关进程，再重新跑 run_full_demo.sh

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR"

echo "==> 结束本次 demo 相关进程..."

pkill -f "trajectory_to_isaac_relay" 2>/dev/null && echo "  已结束: trajectory_to_isaac_relay" || true
pkill -f "ur5_pick_place_cpp_r"      2>/dev/null && echo "  已结束: ur5_pick_place_cpp_r" || true
pkill -f "demo.launch.py"             2>/dev/null && echo "  已结束: demo.launch (RVIZ+MoveIt)" || true
pkill -f "cmd_bridge.py"             2>/dev/null && echo "  已结束: cmd_bridge" || true
pkill -f "target_publisher.py"       2>/dev/null && echo "  已结束: target_publisher" || true
pkill -f "yolov8_obb_publisher"       2>/dev/null && echo "  已结束: yolov8_obb_publisher" || true

sleep 2
echo "==> 重新启动完整 demo..."
exec "$SCRIPT_DIR/run_full_demo.sh"
