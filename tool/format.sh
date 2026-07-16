#!/bin/bash
# 使用项目根目录的 .clang-format 对 src/、test/、bench/ 下所有 C++ 文件格式化
# 用法: ./tool/format.sh

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

CLANG_FORMAT="${CLANG_FORMAT:-clang-format}"
STYLE_FILE="${REPO_ROOT}/.clang-format"

if ! command -v "$CLANG_FORMAT" &>/dev/null; then
    echo "Error: $CLANG_FORMAT not found. Install clang-format or set CLANG_FORMAT env."
    exit 1
fi

if [ ! -f "$STYLE_FILE" ]; then
    echo "Error: $STYLE_FILE not found."
    exit 1
fi

echo "Using: $CLANG_FORMAT"
echo "Style: $STYLE_FILE"
echo "Root:  $REPO_ROOT"
echo ""

# 搜集需要格式化的文件：.h .hpp .cpp .t.cpp .inc
mapfile -t FILES < <(
    find "$REPO_ROOT/src" "$REPO_ROOT/test" "$REPO_ROOT/bench" \
        -type f \
        \( -name '*.h' -o -name '*.hpp' -o -name '*.cpp' -o -name '*.t.cpp' -o -name '*.inc' \) \
        2>/dev/null
)

if [ ${#FILES[@]} -eq 0 ]; then
    echo "No C++ files found under src/ test/ bench/"
    exit 0
fi

echo "Formatting ${#FILES[@]} files..."
"$CLANG_FORMAT" -i --style=file --fallback-style=none "${FILES[@]}"
echo "Done."
