#pragma once

#include <vector>

#include "core/types.h"

namespace stc_SQP {
// QP 求解结果：保存每一步的状态、控制以及松弛变量
struct QPSolution {
    // 输入时间步数 N、状态维度 nx、控制维度 nu、每阶段软约束松弛变量维度 ns_per_stage，按维度预分配存储
    void resize(int N, int nx, int nu, int ns_per_stage = 0)
    {
        x.assign(N + 1, Vector::Zero(nx));
        u.assign(N, Vector::Zero(nu));
        if (ns_per_stage > 0) {
            // 当前普通约束仅存在于 k = 0..N-1，终端阶段不分配松弛变量
            s.assign(N, Vector::Zero(ns_per_stage));
        } else {
            s.clear();
        }
    }

    // 状态轨迹 x_0 ... x_N
    std::vector<Vector> x;
    // 控制轨迹 u_0 ... u_{N-1}
    std::vector<Vector> u;
    // 软约束松弛变量（按阶段展平）
    std::vector<Vector> s;
};
} // namespace stc_SQP
