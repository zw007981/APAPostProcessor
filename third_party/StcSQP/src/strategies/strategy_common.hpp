#pragma once

#include <algorithm>
#include <cmath>
#include <string>

#include "../ocp/multi_stage_ocp.h"

namespace stc_SQP {
namespace strategy_internal {

// 校验策略构造时传入的 nbx/nbu/ng/ns 与 OCP 实际维度是否兼容。
// 若不一致，将诊断信息写入 reason 并返回 false。
inline bool validateSolverDimensions(int nbx, int nbu, int ng, int ns, int nx, int nu,
    std::string* reason = nullptr)
{
    auto set_reason = [reason](const char* msg) {
        if (reason != nullptr) {
            *reason = msg;
        }
    };
    if (nx <= 0 || nu <= 0) {
        set_reason("OCP state/control dimensions must be greater than 0");
        return false;
    }
    if (nbx < 0 || nbu < 0 || ng < 0 || ns < 0) {
        set_reason("strategy construction dimensions nbx/nbu/ng/ns must be non-negative");
        return false;
    }
    if (nbx > nx || nbu > nu) {
        set_reason("nbx/nbu cannot exceed OCP state/control dimensions");
        return false;
    }
    set_reason("");
    return true;
}

// 计算 Partial Condensing 后的宏观步数 cond_N。
// - 当 N < short_n_threshold 且 !force_condensing 时返回 -1（无凝聚）。
// - 否则返回 ceil(N / block_size)，并保证至少为 1（N 极小时等价于无凝聚）。
inline int computeCondN(int N, int block_size, int short_n_threshold,
    bool force_condensing = false)
{
    if (!force_condensing && N < short_n_threshold) {
        return -1;
    }
    if (N <= 0 || block_size <= 0) {
        return -1;
    }
    return std::max(1, (N + block_size - 1) / block_size);
}

// 根据 N 与短 N 阈值决定是否启用 OpenMP 并行 linearize。
inline bool computeUseOmp(int N, int short_n_threshold)
{
    return N >= short_n_threshold;
}

// 计算 OCP 每步一般约束的最大维度 ng_max（与 SQP 引擎内部口径一致）。
inline int computeOcpNgMax(const MultiStageOCP& ocp)
{
    int ng_max = 0;
    for (const auto& segment : ocp.segments()) {
        int ng_seg = 0;
        for (const auto& constraint : segment.constraints) {
            ng_seg += constraint ? constraint->ng() : 0;
        }
        ng_max = std::max(ng_max, ng_seg);
    }
    return ng_max;
}

} // namespace strategy_internal
} // namespace stc_SQP
