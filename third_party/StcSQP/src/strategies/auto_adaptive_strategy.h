#pragma once

#include "solver_strategy.hpp"

namespace stc_SQP {
// 自动自适应策略：根据 OCP 总步数 N 选择求解策略。
// - 短 N（N < short_N_threshold_）：串行 linearize + HPIPM 直接求解 OCP QP（cond_N = -1）。
// - 长 N（N >= short_N_threshold_）：OpenMP 并行 linearize + HPIPM Partial Condensing。
// 重要：传入 HPIPM 的 cond_N 是凝聚后的宏观步数（N / block_size），不是块大小。
class AutoAdaptiveStrategy : public SolverStrategy {
public:
    // 构造策略。
    // nbx/nbu/ng/ns 用于构造内部 HPIPMQPSolver；block_size 为 Partial Condensing 块大小。
    AutoAdaptiveStrategy(int nbx, int nbu, int ng, int ns, int block_size = 10,
        int short_n_threshold = 50);
    // 根据 N 分发并求解
    bool solve(const MultiStageOCP& ocp, const Trajectory& initial_guess,
        Trajectory& solution) override;

protected:
    // 块大小：Partial Condensing 每块包含的原始步数
    int block_size_ = 10;
    // 短 N 阈值：低于该值禁用 OpenMP 与 Partial Condensing
    int short_n_threshold_ = 50;
    // 构造 HPIPMQPSolver 所需的维度参数
    int nbx_ = 0;
    int nbu_ = 0;
    int ng_ = 0;
    int ns_ = 0;
};
} // namespace stc_SQP
