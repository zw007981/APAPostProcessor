#pragma once

#include "../ocp/multi_stage_ocp.h"
#include "../util/trajectory.h"

namespace stc_SQP {
// 实现类负责构造内部 QP 求解器与 SQP 引擎，并暴露统一的 solve 入口。
class SolverStrategy {
public:
    virtual ~SolverStrategy() = default;
    // 求解多段 OCP，结果写入 solution
    virtual bool solve(const MultiStageOCP& ocp, const Trajectory& initial_guess,
        Trajectory& solution) = 0;
};
} // namespace stc_SQP
