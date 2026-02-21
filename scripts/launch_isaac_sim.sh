#!/bin/bash
# 用正确环境启动 Isaac Sim（必须用 conda 环境 ros2_humble_py310 + PYTHONPATH=/opt/isaac）

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR/.."

# 必须用 ros2_humble_py310，不能用 base
if command -v conda &>/dev/null; then
  eval "$(conda shell.bash hook 2>/dev/null)"
  conda activate ros2_humble_py310 || {
    echo "请先执行: conda activate ros2_humble_py310"
    echo "然后再运行: $0"
    exit 1
  }
fi

export PYTHONPATH="/opt/isaac:${PYTHONPATH:-}"

# 若直接运行 /opt/isaac/bin/isaacsim 会使用其 shebang 的 Python，但可能缺参数，用下面完整命令
exec python -c "
import sys
sys.argv = [
    'isaacsim',
    '/opt/isaac/omni/apps/omni.app.full.kit',
    '--ext-folder', '/opt/isaac/isaacsim/exts',
    '--enable', 'isaacsim.ros2.bridge'
]
from isaacsim import main
main()
"
