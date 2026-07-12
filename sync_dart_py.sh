#!/bin/bash
# =============================================================
# 一键同步 src/dart_py/ 到 Windows 桌面
# 用法: ./sync_dart_py.sh
# =============================================================

set -euo pipefail

SRC="/home/panshenghe/K230_2027/src/dart_py"
DST="/mnt/c/Users/panshenghe/Desktop/dart_py"

echo "========================================"
echo "  dart_py → 桌面同步脚本"
echo "========================================"
echo "源目录: $SRC"
echo "目标目录: $DST"
echo ""

# 检查源目录是否存在
if [[ ! -d "$SRC" ]]; then
    echo "[错误] 源目录不存在: $SRC"
    exit 1
fi

# 检查目标桌面是否可访问
if [[ ! -d "/mnt/c/Users/panshenghe/Desktop" ]]; then
    echo "[错误] Windows 桌面不可访问，请确认 WSL 挂载正常"
    exit 1
fi

# 创建目标目录（如果不存在）
mkdir -p "$DST"

# rsync 同步（仅同步 .py 和配置文件，排除 __pycache__ 等）
rsync -av --delete \
    --include='*.py' \
    --include='*.json' \
    --include='*.yaml' \
    --include='*.yml' \
    --include='*.toml' \
    --include='*.cfg' \
    --include='*.ini' \
    --include='*/' \
    --exclude='*' \
    --exclude='__pycache__/' \
    --exclude='*.pyc' \
    --exclude='.git/' \
    "$SRC/" "$DST/"

echo ""
echo "✅ 同步完成！桌面 dart_py/ 已更新。"
echo "目标路径: $DST"
