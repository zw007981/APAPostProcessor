#!/bin/bash
# NMPC 性能基准测试脚本：对每份数据文件各运行4次，记录优化耗时、路径长度和maneuver段数
# 运行方式：cd /home/els/code/APAPostProcessor && bash tool/bench_nmpc.sh

set -euo pipefail

PROJECT_ROOT="/home/els/code/APAPostProcessor"
BINARY="$PROJECT_ROOT/build/Release/apa_post_processor"
CONFIG="$PROJECT_ROOT/data/config.json"
RUNS=4
RESULTS_FILE="$PROJECT_ROOT/tool/bench_nmpc_results.txt"

# 所有数据文件列表
DATA_FILES=(
    "data/test.json"
    "data/long_park/data6.json"
    "data/mid_park/data3.json"
    "data/rub_park/data1.json"
    "data/rub_park/data7.json"
)

# 备份原始 config.json
cp "$CONFIG" "$CONFIG.bak"

echo "============================================================" | tee "$RESULTS_FILE"
echo "  NMPC 性能基准测试 — $(date '+%Y-%m-%d %H:%M:%S')" | tee -a "$RESULTS_FILE"
echo "  每个数据文件运行 ${RUNS} 次" | tee -a "$RESULTS_FILE"
echo "============================================================" | tee -a "$RESULTS_FILE"
echo "" | tee -a "$RESULTS_FILE"

for data_file in "${DATA_FILES[@]}"; do
    echo "------------------------------------------------------------" | tee -a "$RESULTS_FILE"
    echo "  数据文件: $data_file" | tee -a "$RESULTS_FILE"
    echo "------------------------------------------------------------" | tee -a "$RESULTS_FILE"

    # 修改 config.json 指向当前数据文件
    cat > "$CONFIG" <<EOF
{
  "data_file_path": "$data_file"
}
EOF

    # 检查数据文件是否存在
    if [ ! -f "$PROJECT_ROOT/$data_file" ]; then
        echo "  ⚠ 文件不存在: $data_file，跳过" | tee -a "$RESULTS_FILE"
        continue
    fi

    for ((run=1; run<=RUNS; run++)); do
        echo "    运行 #$run ..."
        # 运行程序，捕获输出
        output=$("$BINARY" 2>&1) || true

        # 提取 PostProcessor result 行
        result_line=$(echo "$output" | grep "PostProcessor result:" || echo "NOT_FOUND")

        if [ "$result_line" = "NOT_FOUND" ]; then
            echo "      ⚠ 未找到结果行" | tee -a "$RESULTS_FILE"
            continue
        fi

        # 提取关键指标
        success=$(echo "$result_line" | grep -oP 'success=\K[^,]+')
        maneuvers=$(echo "$result_line" | grep -oP 'maneuvers=\K[^,]+')
        length=$(echo "$result_line" | grep -oP 'length=\K[^,]+')
        time_ms=$(echo "$result_line" | grep -oP 'time_ms=\K[^,]+')
        used_retry=$(echo "$result_line" | grep -oP 'used_retry=\K[^,]+')
        message=$(echo "$result_line" | grep -oP 'message=\K.*')

        printf "      #%d  success=%-5s  time_ms=%-8s  length=%-8s  maneuvers=%-3s  retry=%-5s  msg=%s\n" \
            "$run" "$success" "$time_ms" "$length" "$maneuvers" "$used_retry" "$message" | tee -a "$RESULTS_FILE"
    done
    echo "" | tee -a "$RESULTS_FILE"
done

# 恢复原始 config.json
cp "$CONFIG.bak" "$CONFIG"
rm -f "$CONFIG.bak"

echo "============================================================" | tee -a "$RESULTS_FILE"
echo "  测试完成！结果保存在: $RESULTS_FILE" | tee -a "$RESULTS_FILE"
echo "============================================================" | tee -a "$RESULTS_FILE"
