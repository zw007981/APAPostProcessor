#pragma once

#include "solver_strategy.hpp"

namespace stc_SQP {
// Coarse-to-Fine 手动分层策略选项
struct HierarchicalOptions {
    // 是否启用粗-细两层优化；false 时直接对原始 OCP 做细优化
    bool enable = true;
    // 粗 OCP 总步数；<=0 时自动决定（默认取原步数 1/10）
    int coarse_n = -1;
    // 粗 OCP 期望时间步长；<=0 时忽略
    double coarse_dt = -1.0;
    // 粗优化最大 SQP 迭代次数
    int coarse_max_iter = 5;
    // 细优化最大 SQP 迭代次数
    int fine_max_iter = 2;
};

// 手动分层策略：先求解粗粒度 OCP，再用动力学一致插值得到细 OCP 的初猜，最后细优化。
// 与设计文档 5.3 对应，内部复用 HPIPM + SQPSolver，短/长 N 均走 AutoAdaptive 同款分发逻辑。
class ManualHierarchicalStrategy : public SolverStrategy {
public:
    // nbx/nbu/ng/ns 用于构造内部 HPIPMQPSolver
    ManualHierarchicalStrategy(int nbx, int nbu, int ng, int ns,
        const HierarchicalOptions& opts = HierarchicalOptions(),
        int block_size = 10, int short_n_threshold = 50);

    bool solve(const MultiStageOCP& ocp, const Trajectory& initial_guess,
        Trajectory& solution) override;

protected:
    // 构造并运行一次 SQPSolver，返回是否成功
    bool solveOnce(const MultiStageOCP& ocp, const Trajectory& initial_guess,
        Trajectory& solution, int max_iter) const;

    // 对 coarse_solution 做动力学一致插值，得到 fine_ocp 长度的初始猜测
    Trajectory interpolateDynamicsConsistent(const Trajectory& coarse_solution,
        const MultiStageOCP& coarse_ocp, const MultiStageOCP& fine_ocp) const;
    // 将细初始猜测下采样到粗 OCP 长度，用于粗优化初猜
    Trajectory downsampleTrajectory(const Trajectory& fine,
        const MultiStageOCP& fine_ocp, const MultiStageOCP& coarse_ocp) const;

protected:
    HierarchicalOptions opts_;
    int nbx_ = 0;
    int nbu_ = 0;
    int ng_ = 0;
    int ns_ = 0;
    int block_size_ = 10;
    int short_n_threshold_ = 50;
};

} // namespace stc_SQP
