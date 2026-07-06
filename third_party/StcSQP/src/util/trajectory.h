#pragma once

#include <vector>

#include "../core/types.h"

namespace stc_SQP {
// 轨迹数据结构：保存多段 OCP 沿时间轴展开后的状态序列与控制序列
// 状态序列长度为 N+1（含末端），控制序列长度为 N
struct Trajectory {
    // 按预测步数 N、状态维度 nx、控制维度 nu 预分配存储
    void resize(int N, int nx, int nu) {
        x.assign(N + 1, Vector::Zero(nx));
        u.assign(N, Vector::Zero(nu));
    }
    // 状态轨迹 x_0 ... x_N
    std::vector<Vector> x;
    // 控制轨迹 u_0 ... u_{N-1}
    std::vector<Vector> u;
};
} // namespace stc_SQP
