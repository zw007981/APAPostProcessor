#pragma once

#include "../util/constants.h"
#include "qp_data.h"
#include "qp_solution.h"

namespace stc_SQP {
// QP 求解器状态码。
// 上层 SQP/RTI 循环必须严格检查该状态：只有返回 SUCCESS 时，qp_sol 中的状态/控制/松弛
// 轨迹才是有效且满足约束与动力学的搜索方向（delta_traj_）。非 SUCCESS 时，qp_sol 可能包含
// 未收敛解、不可行解或 NaN 等脏数据；若强行将其叠加到 current_traj_ 上，会污染当前轨迹、
// 破坏约束满足性，并导致后续迭代基于错误状态继续发散。因此任何非 SUCCESS 情况都必须走失败
// 兜底路径，禁止直接应用 delta_traj_。
enum class QPSolverStatus {
    // 求解成功
    SUCCESS = 0,
    // 达到最大迭代次数仍未收敛
    MAX_ITER_REACHED = 1,
    // 迭代过程中出现不可行解
    INFEASIBLE = 2,
    // 迭代过程中出现 NaN
    NAN_IN_SOLUTION = 3,
    // 不合法的输入
    INVALID_ARGUMENT = 4,
    // 未知错误
    UNKNOWN_ERROR = 5
};

// QP求解器纯虚接口
class QPSolver {
public:
    virtual ~QPSolver() = default;
    // 求解QP问题，结果写入qp_sol
    virtual QPSolverStatus solve(const QPData& qp_data, QPSolution& qp_sol) = 0;
    // 设置求解精度
    virtual void setTolerance(double tol) = 0;
    // 设置热启动
    virtual void setWarmStart(const QPSolution& qp_sol) = 0;
    // 返回求解器支持的每步软约束维度；默认 0 表示不支持软约束。
    virtual int slackDim() const { return 0; }
};
} // namespace stc_SQP
