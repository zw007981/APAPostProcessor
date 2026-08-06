#!/usr/bin/env bash
# DDP 四数据集固定验收命令：固定 OMP_NUM_THREADS 与变体列表
# （ddp_tune_common.h::BuildVariants 的 {baseline, nomelt_control}），
# 剥离耗时类易变字段后输出可 diff 的机器可读汇总表。
#
# 用法（仓库根目录）：
#   tool/accept_ddp.sh > a.txt && tool/accept_ddp.sh > b.txt && diff a.txt b.txt
# 同一台机器连续两次执行必须逐位一致（耗时字段已剥离，求解数值本身确定）。
# 变更变体列表只在 ddp_tune_common.h 一处维护（单一真值来源）。
set -euo pipefail
cd "$(dirname "$0")/.."
export OMP_NUM_THREADS=4
./build/Release/apa_tune_ddp | sed -E 's/ time_ms=[0-9.eE+-]+//g; s/ sum_time_ms=[0-9.eE+-]+//g'
