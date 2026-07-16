#pragma once

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "../core/logger.h"
#include "../core/types.h"
#include "../ocp/multi_stage_ocp.h"
#include "../qp/qp_data.h"
#include "../qp/qp_solution.h"
#include "../qp/qp_solver.h"
#include "../qp/soft_constraint_validation.h"
#include "../util/trajectory.h"

namespace stc_SQP {
// SQP 求解器选项
struct SQPSolverOptions {
    // 最大 SQP 迭代次数
    int max_iter = 10;
    // KKT 残差收敛阈值
    double kkt_tol = 1e-6;
    // 约束违反收敛阈值
    double constr_viol_tol = 1e-6;
    // 平稳性收敛阈值（用于 delta 步长判停）
    double stationarity_tol = 1e-6;
    // 是否启用实时迭代（RTI）；本次 solve 若含换挡点则局部降级为 Full SQP
    bool use_rti = false;
    bool use_line_search = true;
    double reg_min = 1e-12;
    double reg_max = 1e8;
    double reg_factor = 10.0;
    // 全局 Hessian 正则化（Levenberg-Marquardt 风格阻尼）：额外叠加到每个 stage
    // 的 Q/R 对角上（Q += hessian_regularization * I，R 同理，终端 stage 无 R），
    // 独立于 reg_min（reg_min 仅在 stage 完全无代价时才生效，是结构性兜底；
    // 本字段无条件叠加到所有 stage，用于抑制早期迭代的过大步长）。
    // 默认 0.0，不改变既有行为；仅在显式设置为正值时才收紧步长。
    double hessian_regularization = 0.0;
    bool use_slack = true;
    double slack_penalty = 1e4;
    double merit_penalty = 1e4;
    bool use_omp = true;
    int omp_parallel_threshold = 50;
    // 线搜索最小可接受步长
    double line_search_alpha_min = 1e-4;
    // 线搜索回退系数
    double line_search_rho = 0.5;
    // Armijo 常数 c（0 < c < 1）
    double line_search_c = 1e-4;
    // 是否在 Full SQP 循环中跨迭代复用上一次 QP 解作为 HPIPM IPM 热启动；
    // RTI 单步模式不受此开关影响（无上一次迭代可复用）。
    bool use_qp_warm_start = false;
    // HPIPM 原生软约束配置；仅当底层 QP 求解器支持 ns > 0 时生效。
    // 默认 ns=0 表示不启用软约束。
    SoftConstraintConfig soft_constraint_config;
};

// SQP 主求解器：负责外循环线性化、QP 装配与求解、线搜索与流形更新
//   1. QP 变量为 delta（delta_x0 = 0），求解后通过 dynamics()->retract() 更新 theta
//   2. 严格检查 QPSolverStatus，失败时禁止应用 delta_traj_
//   3. 检测到换挡点时本次 solve 局部降级为 Full SQP，并记录 rti_downgraded_
//   4. linearize() 串行/并行按 global_k 填充 A/B/b 与一般约束 C/D/d
//   5. assembleQP() 从 StageSegment 的 cost/bounds 装配 QP
//   6. 若 StageSegment::stage_params 非空，约束求值时显式传入对应 step 的 p；
//      Constraint 接口内不再持有运行时可变参数，消除跨调用隐藏状态与线程安全隐患。
// 注：单次 solve() 内部不重新筛选走廊半空间；走廊重选由上层 ParkingOptimizer 在两次 solve 之间完成。
class SQPSolver {
public:
    // 构造时注入 QP 求解器
    explicit SQPSolver(std::unique_ptr<QPSolver> qp_solver);
    // 主入口：求解多段 OCP。
    // 若 Full SQP 达到 max_iter 仍未收敛，返回 false，但 solution 仍写入当前最新
    // 迭代轨迹（last iterate）；RTI 模式下单步 QP 成功即返回 true。
    bool solve(const MultiStageOCP& ocp, const Trajectory& initial_guess, Trajectory& solution);
    // 注入外部预分配的 QPData，solve() 将复用而非新建。
    // 调用方需保证 QPData 的 (N, nx, nu, ng_max) 与本次 solve 匹配；
    // 传入 nullptr 则回退到内部新建（默认行为）。
    void setExternalQPData(std::unique_ptr<QPData> qp_data);
    // 取出 QPData 所有权（供对象池跨调用复用）。solve() 后调用；
    // 取出后内部 qp_data_ 置空，下一次 solve() 若未重新注入则自动新建。
    std::unique_ptr<QPData> takeQPData();
    // 读写求解选项
    SQPSolverOptions& options() { return options_; }
    const SQPSolverOptions& options() const { return options_; }
    // 本次 solve 是否因换挡点而触发 RTI 降级
    bool rtiDowngraded() const { return rti_downgraded_; }
    // 本次 solve 是否达到收敛（RTI 模式下表示单步 QP 成功）
    bool converged() const { return converged_; }

protected:
    // 单次 SQP 迭代
    bool iterate();
    // 校验 OCP 与初始猜测维度
    bool validateProblem(const MultiStageOCP& ocp, const Trajectory& initial_guess,
        std::string* reason) const;
    // 线性化：填充 qp_data_ 的 A/B/b（动力学）与 C/D/d（一般约束）。
    //     每线程持有独立 Constraint 副本，通过 clone() 避免 CasADi 工作区竞争。
    bool linearize();
    // 装配 QP 数据：cost、box bounds（一般约束已在 linearize 中装配）
    bool assembleQP();
    // 装配代价项到 QP；若输出维度或数值非法返回 false（串行入口）
    bool assembleCost(int global_k, const StageSegment& segment, const Vector& x,
        const Vector& u);
    // 装配代价项到 QP 的实际实现，允许传入外部 cost 列表与 scratch buffer（供并行路径使用）
    bool assembleCostImpl(int global_k, const CostTerm& cost_term,
        const Vector& x, const Vector& u, Vector& cost_q, Vector& cost_r, Matrix& cost_Q,
        Matrix& cost_R, Matrix& cost_S);
    // 装配 box bound 到 QP（delta 语义）
    void assembleBounds(int global_k, const StageSegment& segment, const Vector& x,
        const Vector& u);
    // 单个离散步的线性化：动力学 + 一般约束，写入 qp_data_ 对应 global_k
    bool linearizeStep(int segment_idx, int step_in_segment, int global_k, double dt,
        const std::vector<std::shared_ptr<Constraint>>& constraints, Vector& local_x_next,
        Matrix& local_A, Matrix& local_B);
    // 统一封装的一般约束求值：含异常、维度、有限性检查。
    // 用于 computeConstraintViolation / checkConvergence 等仅需 g 的路径。
    bool evaluateConstraintValue(const Constraint& constraint, const Vector& x,
        const Vector& u, const Vector& p, int global_k, bool in_parallel, Vector& g) const;
    // 统一封装的一般约束线性化：含异常、维度、有限性检查。
    // 用于 linearizeStep 中同时需要 g、Cx、Cu 的路径。
    bool evaluateConstraintLinearization(const Constraint& constraint,
        const Vector& x, const Vector& u, const Vector& p, int global_k, bool in_parallel,
        Vector& g, Matrix& Cx, Matrix& Cu) const;
    // 求解 QP：严格检查 QPSolverStatus，失败时 delta_traj_ 不可用
    bool solveQP();
    bool lineSearch(double& alpha);
    // 计算轨迹的 merit function：cost + merit_penalty * 约束违反（L1）
    double computeMerit(const Trajectory& traj) const;
    // 计算当前 QP 方向的 merit function 方向导数（用于 Armijo 条件）
    double computeDirectionalDerivative() const;
    // 计算轨迹的约束违反量（一般约束 + box bound 的 L1 正部）
    double computeConstraintViolation(const Trajectory& traj) const;
    // 收敛判断：综合 delta 步长、动力学残差、约束与边界违反量
    bool checkConvergence();
    // 流形更新：必须调用 dynamics()->retract() 处理 theta；
    // 若模型回调异常或结果非有限，返回 false。
    bool applyRetraction(const MultiStageOCP& ocp, const Trajectory& current, double alpha,
        const Trajectory& delta, Trajectory& result);

protected:
    // QP 求解器
    std::unique_ptr<QPSolver> qp_solver_;
    // QP 数据容器
    std::unique_ptr<QPData> qp_data_;
    // 外部注入的 QPData（solve() 入口处被移入 qp_data_），避免每次 solve 重新分配内存池
    std::unique_ptr<QPData> external_qp_data_;
    // QP 求解结果缓存（避免每次迭代重复分配）
    QPSolution qp_solution_;
    // 求解选项
    SQPSolverOptions options_;
    // 当前迭代轨迹
    Trajectory current_traj_;
    // QP 求解得到的方向（delta）
    Trajectory delta_traj_;
    // 当前 OCP 指针（solve 期间有效，不拥有所有权）
    const MultiStageOCP* ocp_ = nullptr;
    // 当前迭代计数
    int iter_count_ = 0;
    // assembleCost 的 scratch buffer，避免每次迭代临时分配 q/r/Q/R/S
    Vector cost_q_;
    Vector cost_r_;
    Matrix cost_Q_;
    Matrix cost_R_;
    Matrix cost_S_;
    // linearize 的 scratch buffer，避免每次迭代临时分配 x_next/A/B
    Vector lin_x_next_;
    Matrix lin_A_;
    Matrix lin_B_;
    // computeConstraintViolation / checkConvergence 的约束值 scratch buffer
    mutable Vector mutable_g_;
    // 终端代价评估使用的零控制缓存
    Vector zero_u_;
    // 按 solve() 预分配一次：每线程持有独立 Constraint clone 副本，
    // 避免并行 linearize() 中 CasADiFunction / mutable scratch 的数据竞争。
    std::vector<std::vector<std::vector<std::shared_ptr<Constraint>>>> thread_constraint_clones_;
    // 与 thread_constraint_clones_ 对称的 cost 克隆池，供 assembleQP/assembleCost 并行路径使用。
    std::vector<std::vector<std::shared_ptr<CostTerm>>> thread_cost_clones_;
    // 本次 solve 是否实际按 RTI 模式执行（含换挡点降级后的 Full SQP为 false）
    bool rti_mode_active_ = false;
    // RTI 降级标记
    bool rti_downgraded_ = false;
    // 收敛标记
    bool converged_ = false;
};
} // namespace stc_SQP
