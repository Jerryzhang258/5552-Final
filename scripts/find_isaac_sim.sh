#!/bin/bash
# 查找本机 Isaac Sim 安装位置，便于用正确方式启动

echo "=== 查找 Isaac Sim 启动脚本/可执行文件 ==="
for path in \
  "/opt/isaac/isaac-sim.sh" \
  "/opt/isaac/isaac-sim" \
  "/opt/isaac/python.sh" \
  "/opt/isaac/bin/isaac-sim" \
  "$HOME/isaac-sim/isaac-sim.sh" \
  "/usr/local/bin/isaac-sim" \
  "/opt/nvidia/isaac_sim/isaac-sim.sh" \
  "/opt/nvidia/isaac-sim/isaac-sim.sh"; do
  if [ -e "$path" ]; then
    echo "找到: $path"
  fi
done

echo ""
echo "=== 在 /opt 下搜索 *isaac* ==="
find /opt -maxdepth 4 -name "*isaac*" -type f 2>/dev/null | head -20

echo ""
echo "=== 检查 /opt/isaac/bin ==="
ls -la /opt/isaac/bin 2>/dev/null || echo "/opt/isaac/bin 不存在"

echo ""
echo "=== 检查 ov 包目录 ==="
ls -d "$HOME/.local/share/ov/pkg/"* 2>/dev/null || echo "未找到 ov/pkg"

echo ""
echo "=== 桌面/应用快捷方式 (.desktop) ==="
grep -l -i isaac /usr/share/applications/*.desktop 2>/dev/null || true
grep -l -i isaac "$HOME/.local/share/applications/"*.desktop 2>/dev/null || true

echo ""
echo "=== 本机情况 ==="
echo "Isaac Sim 库目录: /opt/isaac (Python 包 isaacsim/omni 等)"
echo "可执行脚本: /opt/isaac/bin/isaacsim (需配合 conda 环境 ros2_humble_py310 与 PYTHONPATH=/opt/isaac)"
echo ""
echo "推荐启动方式（本项目内）："
echo "  conda activate ros2_humble_py310"
echo "  ./scripts/launch_isaac_sim.sh"
echo ""
echo "或从系统应用菜单搜索 Isaac Sim / Omniverse。"
