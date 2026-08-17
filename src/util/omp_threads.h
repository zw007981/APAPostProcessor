#pragma once

#include <omp.h>

#include <algorithm>

namespace apa_post_processor {
// CMake 选项 APA_OMP_MAX_THREADS 未注入时的默认天花板：短并行区
// 过度 fork 的同步开销远大于收益（历史实测 22 线程使 B 样条平滑
// 从 ~100ms 退化到 ~10s），必须设跨平台硬上限
#ifndef APA_OMP_MAX_THREADS
#define APA_OMP_MAX_THREADS 8
#endif

// 预处理并行区统一线程策略：有效线程数 = min(运行期上限, 编译期
// 天花板) 再按 policy_denominator 缩放、下限 min_threads、最终不
// 低于 1。运行期上限取 omp_get_max_threads()（受 OMP_NUM_THREADS
// 环境变量控制），部署方日常调优无需重新构建；车端构建时可用
// CMake 选项收紧天花板（如 J6M 建议 2~4）
inline int EffectiveOmpThreads(int policy_denominator, int min_threads) {
    const int env_limit = omp_get_max_threads();
    const int clamped = std::min(env_limit, APA_OMP_MAX_THREADS);
    return std::max(1, std::max(min_threads, clamped / policy_denominator));
}
}  // namespace apa_post_processor
