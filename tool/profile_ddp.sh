#!/bin/bash
# DDP 性能 profiling 一条龙脚本：构建 Profile 版 -> perf 采样 -> 生成火焰图
# 运行方式：cd /home/els/code/APAPostProcessor && bash tool/profile_ddp.sh [每数据集重复次数，默认 3]
# 产物：build/Profile/perf.data（原始采样）与 build/Profile/ddp_flame.svg（火焰图，
# 在 Windows 浏览器打开 \\wsl$\... 路径即可交互查看）

set -euo pipefail

PROJECT_ROOT="/home/els/code/APAPostProcessor"
BUILD_DIR="$PROJECT_ROOT/build/Profile"
BINARY="$BUILD_DIR/apa_profile_ddp"
REPEATS="${1:-3}"
# 火焰图脚本目录（Brendan Gregg FlameGraph 的本地拷贝，不随仓库分发）
FLAMEGRAPH_DIR="${FLAMEGRAPH_DIR:-$HOME/tools/FlameGraph}"

# WSL2 下 /usr/bin/perf 是按内核版本号寻址的 wrapper，对 WSL 定制内核
# （*-microsoft-standard-WSL2）永远找不到匹配二进制；真实二进制在
# linux-tools-<ver> 目录下，用通配符取已安装版本，避免硬编码小版本号
PERF_BIN="$(ls /usr/lib/linux-tools-*/perf 2>/dev/null | head -1)"
if [ -z "$PERF_BIN" ]; then
    echo "✗ 未找到 perf 真实二进制（/usr/lib/linux-tools-*/perf），"
    echo "  请先 sudo apt install linux-tools-generic"
    exit 1
fi
if [ ! -f "$FLAMEGRAPH_DIR/flamegraph.pl" ]; then
    echo "✗ 未找到火焰图脚本 $FLAMEGRAPH_DIR/flamegraph.pl，"
    echo "  请 git clone https://github.com/brendangregg/FlameGraph 到该目录"
    exit 1
fi

cd "$PROJECT_ROOT"

# 1. 构建 Profile 版（RelWithDebInfo + -fno-omit-frame-pointer，
#    优化级别贴近 Release，同时保留调试符号与帧指针供栈展开）
echo "==> 构建 apa_profile_ddp（build/Profile）"
cmake --build "$BUILD_DIR" --target apa_profile_ddp -j

# 2. perf 采样：WSL2 无硬件 PMU，只能用软件事件 cpu-clock（上限 999Hz）；
#    --call-graph fp 依赖上一步保留的帧指针做栈展开
echo "==> perf 采样（cpu-clock @999Hz，每数据集重复 $REPEATS 次）"
"$PERF_BIN" record -e cpu-clock -F 999 --call-graph fp \
    -o "$BUILD_DIR/perf.data" -- "$BINARY" "$REPEATS"

# 3. 折叠调用栈并渲染火焰图
echo "==> 生成火焰图 $BUILD_DIR/ddp_flame.svg"
"$PERF_BIN" script -i "$BUILD_DIR/perf.data" | \
    "$FLAMEGRAPH_DIR/stackcollapse-perf.pl" | \
    "$FLAMEGRAPH_DIR/flamegraph.pl" --title "DDP 4-dataset profile" \
    > "$BUILD_DIR/ddp_flame.svg"

echo "✓ 完成：$BUILD_DIR/ddp_flame.svg"
