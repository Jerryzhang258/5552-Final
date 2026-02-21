#!/bin/bash
# 从原仓库 https://github.com/sahilrajpurkar03/nlp-pnp-robotic-arm 恢复初始场景 SPARC.usd

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR/.."

URL="https://github.com/sahilrajpurkar03/nlp-pnp-robotic-arm/raw/master/SPARC.usd"

if [ -f "SPARC.usd" ]; then
  cp SPARC.usd "SPARC.usd.bak"
  echo "==> 已备份当前 SPARC.usd 为 SPARC.usd.bak"
fi

echo "==> 正在从原仓库下载 SPARC.usd ..."
if command -v curl &>/dev/null; then
  curl -L -o SPARC.usd "$URL"
elif command -v wget &>/dev/null; then
  wget -O SPARC.usd "$URL"
else
  echo "[ERROR] 需要 curl 或 wget。请安装后重试，或手动下载："
  echo "  $URL"
  exit 1
fi

if [ -f "SPARC.usd" ] && [ -s "SPARC.usd" ]; then
  echo "==> 已恢复初始场景 SPARC.usd。在 Isaac Sim 中 File -> Open -> SPARC.usd 即可。"
else
  echo "[ERROR] 下载失败或文件为空。请检查网络或手动下载。"
  exit 1
fi
