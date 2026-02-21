#!/bin/bash
# 不用对话界面：直接发 /target_class_cmd 触发 YOLO+target_publisher+ur5_pick_place 抓取。
# 用法: ./scripts/pick_by_label.sh <类别> [盒子号]
# 例:   ./scripts/pick_by_label.sh red_cube 1
# 前提: target_publisher、yolov8_obb_publisher、ur5_pick_place 已在跑，相机有 /rgb，场景里有该物体。

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR/.."
source /opt/ros/humble/setup.bash
source install/setup.bash 2>/dev/null || true

LABEL="${1:-yellow_cube}"
BOX="${2:-1}"
MSG="$LABEL,$BOX"
echo "==> 发布 /target_class_cmd: $MSG （YOLO 检测到 $LABEL 后会发 /target_point，ur5_pick_place 执行抓放）"
ros2 topic pub --once /target_class_cmd std_msgs/msg/String "{data: '$MSG'}"
