#include <gtest/gtest.h>

#include <Eigen/Cholesky>
#include <cmath>
#include <cstddef>
#include <vector>

#include "core/DDP/bicycle_dynamics.h"
#include "core/DDP/ddp_cost.h"
#include "core/DDP/esdf_constraint.h"
#include "core/DDP/ms_ilqr.h"
#include "spatial/esdf_map.h"
#include "spatial/grid_map.h"
#include "vehicle/vehicle_footprint_model.h"
#include "vehicle/vehicle_params.h"

namespace apa_post_processor {
namespace {

constexpr double kWheelbase = 2.7;
constexpr double kDt = 0.1;

// 按状态分量布局 [x, y, θ, v, a, δ, ω] 构造七维状态
DdpState MakeState(double x, double y, double theta, double v, double a,
                   double delta, double omega) {
    DdpState state;
    state << x, y, theta, v, a, delta, omega;
    return state;
}

// 按控制分量布局 [j, η] 构造二维控制
DdpControl MakeControl(double jerk, double eta) {
    DdpControl control;
    control << jerk, eta;
    return control;
}

// 逐元素对拍（绝对容差）
template <typename TDerivedA, typename TDerivedB>
void ExpectMatrixNear(const Eigen::MatrixBase<TDerivedA>& expected,
                      const Eigen::MatrixBase<TDerivedB>& actual, double tol) {
    ASSERT_EQ(expected.rows(), actual.rows());
    ASSERT_EQ(expected.cols(), actual.cols());
    for (int row = 0; row < expected.rows(); ++row) {
        for (int col = 0; col < expected.cols(); ++col) {
            EXPECT_NEAR(expected(row, col), actual(row, col), tol)
                << "row=" << row << " col=" << col;
        }
    }
}

// 白盒访问：派生类将受保护成员/方法公开给单元测试
class MsIlqrTestAccess : public MsIlqrSolver {
   public:
    using MsIlqrSolver::acceptCandidate;
    using MsIlqrSolver::backward_pass_count_;
    using MsIlqrSolver::backwardPass;
    using MsIlqrSolver::cand_controls_;
    using MsIlqrSolver::cand_states_;
    using MsIlqrSolver::clamped_;
    using MsIlqrSolver::computeJacobians;
    using MsIlqrSolver::config_;
    using MsIlqrSolver::controls_;
    using MsIlqrSolver::convergenceAllowed;
    using MsIlqrSolver::cost_eval_;
    using MsIlqrSolver::defects_;
    using MsIlqrSolver::domain_guard_rejections_;
    using MsIlqrSolver::du_lin_;
    using MsIlqrSolver::dv1_;
    using MsIlqrSolver::dv2_;
    using MsIlqrSolver::dx_lin_;
    using MsIlqrSolver::ec1_;
    using MsIlqrSolver::ec2_;
    using MsIlqrSolver::evaluateNominal;
    using MsIlqrSolver::expectedChange;
    using MsIlqrSolver::feedforward_;
    using MsIlqrSolver::gain_K_;
    using MsIlqrSolver::increaseReg;
    using MsIlqrSolver::is_shooting_;
    using MsIlqrSolver::linear_rollout_count_;
    using MsIlqrSolver::linearRollout;
    using MsIlqrSolver::max_qu_norm_;
    using MsIlqrSolver::merit_mu_;
    using MsIlqrSolver::MsIlqrSolver;
    using MsIlqrSolver::nonlinear_rollout_count_;
    using MsIlqrSolver::nonlinearRollout;
    using MsIlqrSolver::num_steps_;
    using MsIlqrSolver::prepareWorkspace;
    using MsIlqrSolver::q_u_;
    using MsIlqrSolver::q_uu_;
    using MsIlqrSolver::q_x_;
    using MsIlqrSolver::qp_factorization_count_;
    using MsIlqrSolver::rho_reg_;
    using MsIlqrSolver::setNominalTrajectory;
    using MsIlqrSolver::setShootingLookup;
    using MsIlqrSolver::states_;
    using MsIlqrSolver::step_dt_;
    using MsIlqrSolver::total_cost_;
    using MsIlqrSolver::value_S_;
    using MsIlqrSolver::value_s_;
};

// 一致性滚动：从 x0 出发按给定控制全量积分（不注入任何打靶状态），
// 用于构造零缺陷初值
DdpAlignedVec<DdpState> RolloutStates(const BicycleDynamics& dynamics,
                                      const DdpState& x0,
                                      const DdpAlignedVec<DdpControl>& controls,
                                      double dt) {
    DdpAlignedVec<DdpState> states;
    states.reserve(controls.size() + 1);
    states.push_back(x0);
    for (const auto& control : controls) {
        states.push_back(dynamics.step(states.back(), control, dt));
    }
    return states;
}

// 手工构造最小可用参考：求解器只消费位姿/dt/打靶节点集
DdpReference MakeReference(const std::vector<Pose>& poses, double dt,
                           const std::vector<std::size_t>& shooting_nodes) {
    DdpReference reference;
    reference.ds = 0.05;
    reference.dt = dt;
    reference.poses = poses;
    reference.shooting_nodes = shooting_nodes;
    return reference;
}

// 标准圆弧测试问题：零控制一致性滚动得到的名义轨迹，跟踪目标在 y/θ 上
// 错开给定偏移以产生非零代价梯度
struct ArcProblem {
    DdpReference reference;
    DdpAlignedVec<DdpState> states;
    DdpAlignedVec<DdpControl> controls;
};
ArcProblem MakeArcProblem(std::size_t num_steps, const DdpState& x0,
                          double offset_y, double offset_theta,
                          const std::vector<std::size_t>& shooting_nodes) {
    ArcProblem problem;
    problem.controls.resize(num_steps, DdpControl::Zero());
    const BicycleDynamics dynamics(kWheelbase);
    problem.states = RolloutStates(dynamics, x0, problem.controls, kDt);
    std::vector<Pose> poses;
    poses.reserve(num_steps + 1);
    for (const auto& state : problem.states) {
        poses.emplace_back(state(DDP_IDX_X), state(DDP_IDX_Y) + offset_y,
                           state(DDP_IDX_THETA) + offset_theta);
    }
    problem.reference = MakeReference(poses, kDt, shooting_nodes);
    return problem;
}

// 默认零乘子与跟踪权重输入
DdpCostMultiplierState MakeMultipliers(std::size_t num_steps) {
    return DdpCostMultiplierState::MakeZero(num_steps);
}
DdpCostInput MakeCostInput(double tracking_weight) {
    DdpCostInput input;
    input.tracking_weight = tracking_weight;
    input.anneal_exempt_mask = nullptr;
    return input;
}

// 默认求解器配置：宽松盒约束、极小正则化（便于与无正则参照对拍）
MsIlqrConfig MakeConfig() {
    MsIlqrConfig config;
    config.jerk_max = 100.0;
    config.steer_accel_max = 100.0;
    config.reg_initial = 1e-12;
    config.reg_min = 1e-13;
    return config;
}

// 独立 LQR 参照（不经过求解器内部缓存）：对线性化模型 + 二次代价做无约束
// Riccati 回推（含与求解器相同的极小正则化），再做 α=1 线性滚动
struct LqrReference {
    DdpAlignedVec<DdpControl> ff;
    DdpAlignedVec<DdpControlStateHessian> gain;
    DdpAlignedVec<DdpState> dx;
    DdpAlignedVec<DdpControl> du;
};
LqrReference ComputeLqrReference(const BicycleDynamics& dynamics, double dt,
                                 double rho,
                                 const DdpAlignedVec<DdpState>& states,
                                 const DdpAlignedVec<DdpControl>& controls,
                                 const DdpCostEvaluation& evaluation) {
    const std::size_t num_steps = controls.size();
    LqrReference reference;
    reference.ff.resize(num_steps);
    reference.gain.resize(num_steps);
    reference.dx.resize(num_steps + 1);
    reference.du.resize(num_steps);
    DdpAlignedVec<DdpStateJacobian> jac_a(num_steps);
    DdpAlignedVec<DdpControlJacobian> jac_b(num_steps);
    DdpStateHessian value_hessian = evaluation.stages[num_steps].lxx;
    DdpState value_gradient = evaluation.stages[num_steps].lx;
    for (std::size_t k = num_steps; k-- > 0;) {
        dynamics.jacobians(states[k], controls[k], dt, &jac_a[k], &jac_b[k]);
        const auto& stage = evaluation.stages[k];
        const DdpState q_x = stage.lx + jac_a[k].transpose() * value_gradient;
        const DdpControl q_u = stage.lu + jac_b[k].transpose() * value_gradient;
        const DdpStateHessian q_xx =
            stage.lxx + jac_a[k].transpose() * value_hessian * jac_a[k];
        const DdpControlHessian q_uu =
            stage.luu + jac_b[k].transpose() * value_hessian * jac_b[k] +
            rho * DdpControlHessian::Identity();
        const DdpControlStateHessian q_ux =
            stage.lux + jac_b[k].transpose() * value_hessian * jac_a[k];
        const DdpControl ff = -q_uu.ldlt().solve(q_u);
        const DdpControlStateHessian gain = -q_uu.ldlt().solve(q_ux);
        DdpStateHessian next_hessian = q_xx + gain.transpose() * q_uu * gain +
                                       gain.transpose() * q_ux +
                                       q_ux.transpose() * gain;
        next_hessian = 0.5 * (next_hessian + next_hessian.transpose());
        const DdpState next_gradient = q_x + gain.transpose() * q_uu * ff +
                                       gain.transpose() * q_u +
                                       q_ux.transpose() * ff;
        reference.ff[k] = ff;
        reference.gain[k] = gain;
        value_hessian = next_hessian;
        value_gradient = next_gradient;
    }
    reference.dx[0].setZero();
    for (std::size_t k = 0; k < num_steps; ++k) {
        reference.du[k] = reference.ff[k] + reference.gain[k] * reference.dx[k];
        reference.dx[k + 1] =
            jac_a[k] * reference.dx[k] + jac_b[k] * reference.du[k];
    }
    return reference;
}

// 白盒驱动一轮完整准备流程（初始名义建立 -> 代价求值 -> 雅可比），
// 返回 false 表示中途契约异常
void PrepareNominal(MsIlqrTestAccess* solver, const DdpReference& reference,
                    const DdpCostMultiplierState& multipliers,
                    const DdpCostInput& cost_input,
                    const DdpAlignedVec<DdpState>& initial_states,
                    const DdpAlignedVec<DdpControl>& initial_controls) {
    solver->prepareWorkspace(reference.poses.size() - 1);
    solver->setShootingLookup(reference.shooting_nodes);
    solver->setNominalTrajectory(reference, initial_states, initial_controls);
    solver->evaluateNominal(reference, multipliers, cost_input);
    solver->computeJacobians(reference);
}

// LQR 一致性：线性化模型 + 二次代价 + 零缺陷初值下，求解器第一轮产生的
// 搜索方向必须与独立 Riccati 参照（无约束 LQR 解析解）逐元素一致
TEST(MsIlqrTest, LqrDirectionConsistency) {
    const BicycleDynamics dynamics(kWheelbase);
    const DdpCostEvaluator evaluator(DdpCostConfig{}, nullptr);
    const std::size_t num_steps = 8;
    const DdpState x0 = MakeState(0.0, 0.0, 0.0, 0.5, 0.0, 0.05, 0.0);
    ArcProblem problem = MakeArcProblem(num_steps, x0, 0.02, 0.01, {num_steps});
    MsIlqrTestAccess solver(MakeConfig(), &dynamics, &evaluator);
    PrepareNominal(&solver, problem.reference, MakeMultipliers(num_steps),
                   MakeCostInput(10.0), problem.states, problem.controls);
    ASSERT_TRUE(solver.backwardPass());
    solver.linearRollout();
    const LqrReference reference =
        ComputeLqrReference(dynamics, kDt, 1e-12, solver.states_,
                            solver.controls_, solver.cost_eval_);
    for (std::size_t k = 0; k < num_steps; ++k) {
        ExpectMatrixNear(reference.ff[k], solver.feedforward_[k], 1e-8);
        ExpectMatrixNear(reference.gain[k], solver.gain_K_[k], 1e-8);
        ExpectMatrixNear(reference.du[k], solver.du_lin_[k], 1e-8);
    }
    for (std::size_t k = 0; k <= num_steps; ++k) {
        ExpectMatrixNear(reference.dx[k], solver.dx_lin_[k], 1e-8);
    }
    EXPECT_EQ(1, solver.backward_pass_count_);
    EXPECT_EQ(1, solver.linear_rollout_count_);
}

// LQR 一致性（收敛行为）：同一近线性小问题应在一轮内以全牛顿步接受，
// 并在少数几轮内收敛，终态轨迹与 LQR 参照轨迹的差异保持在非线性残差量级
TEST(MsIlqrTest, LqrConvergesWithFullStep) {
    const BicycleDynamics dynamics(kWheelbase);
    const DdpCostEvaluator evaluator(DdpCostConfig{}, nullptr);
    const std::size_t num_steps = 8;
    const DdpState x0 = MakeState(0.0, 0.0, 0.0, 0.5, 0.0, 0.05, 0.0);
    ArcProblem problem = MakeArcProblem(num_steps, x0, 0.02, 0.01, {num_steps});
    MsIlqrTestAccess solver(MakeConfig(), &dynamics, &evaluator);
    const MsIlqrResult result =
        solver.solve(problem.reference, MakeMultipliers(num_steps),
                     MakeCostInput(10.0), problem.states, problem.controls);
    ASSERT_NE(MsIlqrStatus::REGULARIZATION_OVERFLOW, result.status);
    ASSERT_NE(MsIlqrStatus::MAX_ITERATIONS, result.status);
    ASSERT_GE(solver.history().size(), 1U);
    EXPECT_DOUBLE_EQ(1.0, solver.history().front().alpha);
    EXPECT_LE(result.iterations, 5);
    EXPECT_LT(result.final_cost, result.initial_cost);
    const LqrReference reference = ComputeLqrReference(
        dynamics, kDt, 1e-12, problem.states, problem.controls,
        evaluator.evaluate(problem.reference, problem.states, problem.controls,
                           MakeMultipliers(num_steps), MakeCostInput(10.0)));
    for (std::size_t k = 0; k <= num_steps; ++k) {
        const DdpState lqr_state = problem.states[k] + reference.dx[k];
        ExpectMatrixNear(lqr_state, solver.states()[k], 1e-2);
    }
}

// 缺陷收敛：打靶状态注入不一致初值（各段端点错开），迭代后缺陷应归零
// （各分量 < 1e-8），且 merit 按 Armijo 接受逐轮严格下降
TEST(MsIlqrTest, DefectConvergence) {
    const BicycleDynamics dynamics(kWheelbase);
    const DdpCostEvaluator evaluator(DdpCostConfig{}, nullptr);
    const std::size_t num_steps = 20;
    const DdpState x0 = MakeState(0.0, 0.0, 0.0, 0.5, 0.0, 0.04, 0.0);
    ArcProblem problem =
        MakeArcProblem(num_steps, x0, 0.0, 0.0, {5, 10, 15, 20});
    const DdpState offset =
        MakeState(0.15, -0.10, 0.05, 0.02, 0.01, 0.02, 0.01);
    for (const std::size_t node : {5U, 10U, 15U, 20U}) {
        problem.states[node] += offset;
    }
    MsIlqrTestAccess solver(MakeConfig(), &dynamics, &evaluator);
    const MsIlqrResult result =
        solver.solve(problem.reference, MakeMultipliers(num_steps),
                     MakeCostInput(10.0), problem.states, problem.controls);
    ASSERT_NE(MsIlqrStatus::REGULARIZATION_OVERFLOW, result.status);
    EXPECT_GT(result.initial_defect_norm, 0.1);
    EXPECT_LT(result.final_defect_norm, 1e-8);
    for (std::size_t i = 0; i <= num_steps; ++i) {
        EXPECT_LT(solver.defects()[i].cwiseAbs().maxCoeff(), 1e-8)
            << "node=" << i;
    }
    ASSERT_GE(solver.history().size(), 2U);
    for (std::size_t i = 1; i < solver.history().size(); ++i) {
        EXPECT_LT(solver.history()[i].merit, solver.history()[i - 1].merit)
            << "iteration=" << i;
    }
}

// 单缺陷节点专项（off-by-one 对拍）：两段问题、仅节点 1 为打靶节点且携带
// 非零缺陷。手工推导回推：缺陷修正 ẑ = s' + S'd 必须恰好只在节点 1 的
// 左邻步（k=0）生效，节点 2（非打靶）的左邻步（k=1）不得出现修正项
TEST(MsIlqrTest, SingleDefectNodeBackward) {
    const BicycleDynamics dynamics(kWheelbase);
    const DdpCostEvaluator evaluator(DdpCostConfig{}, nullptr);
    const std::size_t num_steps = 2;
    const DdpReference reference = MakeReference(
        {Pose(0.0, 0.0, 0.0), Pose(0.05, 0.0, 0.0), Pose(0.10, 0.01, 0.10)},
        kDt, {1});
    const DdpState x0 = MakeState(0.0, 0.0, 0.0, 0.4, 0.0, 0.03, 0.0);
    const DdpControl u0 = MakeControl(0.01, 0.02);
    const DdpControl u1 = MakeControl(-0.02, 0.01);
    const DdpState x1_integral = dynamics.step(x0, u0, kDt);
    const DdpState offset =
        MakeState(0.10, -0.05, 0.02, 0.06, -0.04, 0.01, -0.03);
    const DdpState x1_injected = x1_integral + offset;
    DdpAlignedVec<DdpState> initial_states = {x0, x1_injected,
                                              DdpState::Zero()};
    const DdpAlignedVec<DdpControl> initial_controls = {u0, u1};
    MsIlqrTestAccess solver(MakeConfig(), &dynamics, &evaluator);
    PrepareNominal(&solver, reference, MakeMultipliers(num_steps),
                   MakeCostInput(10.0), initial_states, initial_controls);
    ExpectMatrixNear(-offset, solver.defects_[1], 1e-12);
    ExpectMatrixNear(DdpState::Zero(), solver.defects_[0], 1e-12);
    ExpectMatrixNear(DdpState::Zero(), solver.defects_[2], 1e-12);
    ASSERT_TRUE(solver.backwardPass());
    const auto& stages = solver.cost_eval_.stages;
    DdpStateJacobian jac_a0, jac_a1;
    DdpControlJacobian jac_b0, jac_b1;
    dynamics.jacobians(x0, u0, kDt, &jac_a0, &jac_b0);
    dynamics.jacobians(x1_injected, u1, kDt, &jac_a1, &jac_b1);
    // 手工回推第 k=1 步：下游节点 2 非打靶（d₂=0），不得出现缺陷修正
    const DdpStateHessian s2_hessian = stages[2].lxx;
    const DdpState s2_gradient = stages[2].lx;
    const DdpState q_x1 = stages[1].lx + jac_a1.transpose() * s2_gradient;
    const DdpControl q_u1 = stages[1].lu + jac_b1.transpose() * s2_gradient;
    const DdpStateHessian q_xx1 =
        stages[1].lxx + jac_a1.transpose() * s2_hessian * jac_a1;
    const DdpControlHessian q_uu1 = stages[1].luu +
                                    jac_b1.transpose() * s2_hessian * jac_b1 +
                                    1e-12 * DdpControlHessian::Identity();
    const DdpControlStateHessian q_ux1 =
        stages[1].lux + jac_b1.transpose() * s2_hessian * jac_a1;
    const DdpControl ff1 = -q_uu1.ldlt().solve(q_u1);
    const DdpControlStateHessian gain1 = -q_uu1.ldlt().solve(q_ux1);
    DdpStateHessian s1_hessian = q_xx1 + gain1.transpose() * q_uu1 * gain1 +
                                 gain1.transpose() * q_ux1 +
                                 q_ux1.transpose() * gain1;
    s1_hessian = 0.5 * (s1_hessian + s1_hessian.transpose());
    const DdpState s1_gradient = q_x1 + gain1.transpose() * q_uu1 * ff1 +
                                 gain1.transpose() * q_u1 +
                                 q_ux1.transpose() * ff1;
    ExpectMatrixNear(q_x1, solver.q_x_[1], 1e-9);
    ExpectMatrixNear(q_u1, solver.q_u_[1], 1e-9);
    ExpectMatrixNear(ff1, solver.feedforward_[1], 1e-9);
    ExpectMatrixNear(gain1, solver.gain_K_[1], 1e-9);
    ExpectMatrixNear(s1_hessian, solver.value_S_[1], 1e-9);
    ExpectMatrixNear(s1_gradient, solver.value_s_[1], 1e-9);
    // 手工回推第 k=0 步：下游节点 1 为打靶节点，修正 ẑ = s₁ + S₁·d₁ 必须
    // 恰好作用在此处（右端索引），任何左端错位都会被以下两式捕获
    const DdpState z0 = s1_gradient + s1_hessian * solver.defects_[1];
    ExpectMatrixNear(stages[0].lx + jac_a0.transpose() * z0, solver.q_x_[0],
                     1e-9);
    ExpectMatrixNear(stages[0].lu + jac_b0.transpose() * z0, solver.q_u_[0],
                     1e-9);
}

// 盒激活场景：收紧跃度盒使最优解 bang-bang，被钳制控制对应的反馈增益行
// 必须恒为零，且 QP 解出的控制恰好贴在盒边界上
TEST(MsIlqrTest, BoxActivationBangBang) {
    const BicycleDynamics dynamics(kWheelbase);
    const DdpCostEvaluator evaluator(DdpCostConfig{}, nullptr);
    const std::size_t num_steps = 10;
    const DdpState x0 = DdpState::Zero();
    ArcProblem problem = MakeArcProblem(num_steps, x0, 0.0, 0.0, {num_steps});
    for (std::size_t k = 0; k <= num_steps; ++k) {
        problem.reference.poses[k] = Pose(0.5, 0.0, 0.0);
    }
    MsIlqrConfig config = MakeConfig();
    config.jerk_max = 0.05;
    MsIlqrTestAccess solver(config, &dynamics, &evaluator);
    const MsIlqrResult result =
        solver.solve(problem.reference, MakeMultipliers(num_steps),
                     MakeCostInput(50.0), problem.states, problem.controls);
    ASSERT_NE(MsIlqrStatus::REGULARIZATION_OVERFLOW, result.status);
    bool has_clamped_jerk = false;
    for (std::size_t k = 0; k < num_steps; ++k) {
        if (!solver.clamped_[k][DDP_IDX_JERK]) {
            continue;
        }
        has_clamped_jerk = true;
        ExpectMatrixNear(Eigen::Matrix<double, 1, DDP_STATE_DIM>::Zero(),
                         solver.gain_K_[k].row(DDP_IDX_JERK), 0.0);
        EXPECT_NEAR(config.jerk_max,
                    std::abs(solver.controls_[k](DDP_IDX_JERK) +
                             solver.feedforward_[k](DDP_IDX_JERK)),
                    1e-9)
            << "step=" << k;
    }
    EXPECT_TRUE(has_clamped_jerk);
}

// 无约束退化一致：同一非线性小问题在超宽盒（永不激活）与默认宽盒下的
// 求解结果必须一致，且全部钳制集为空
TEST(MsIlqrTest, UnconstrainedMatchesWideBox) {
    const BicycleDynamics dynamics(kWheelbase);
    const DdpCostEvaluator evaluator(DdpCostConfig{}, nullptr);
    const std::size_t num_steps = 8;
    const DdpState x0 = MakeState(0.0, 0.0, 0.0, 0.5, 0.0, 0.05, 0.0);
    ArcProblem problem = MakeArcProblem(num_steps, x0, 0.02, 0.01, {num_steps});
    MsIlqrConfig wide_config = MakeConfig();
    MsIlqrTestAccess wide_solver(wide_config, &dynamics, &evaluator);
    const MsIlqrResult wide_result = wide_solver.solve(
        problem.reference, MakeMultipliers(num_steps), MakeCostInput(10.0),
        problem.states, problem.controls);
    MsIlqrConfig free_config = MakeConfig();
    free_config.jerk_max = 1e6;
    free_config.steer_accel_max = 1e6;
    MsIlqrTestAccess free_solver(free_config, &dynamics, &evaluator);
    const MsIlqrResult free_result = free_solver.solve(
        problem.reference, MakeMultipliers(num_steps), MakeCostInput(10.0),
        problem.states, problem.controls);
    ASSERT_EQ(wide_result.status, free_result.status);
    for (std::size_t k = 0; k <= num_steps; ++k) {
        ExpectMatrixNear(wide_solver.states()[k], free_solver.states()[k],
                         1e-12);
    }
    for (std::size_t k = 0; k < num_steps; ++k) {
        ExpectMatrixNear(wide_solver.controls()[k], free_solver.controls()[k],
                         1e-12);
        EXPECT_FALSE(wide_solver.clamped_[k][DDP_IDX_JERK]);
        EXPECT_FALSE(wide_solver.clamped_[k][DDP_IDX_ETA]);
        EXPECT_FALSE(free_solver.clamped_[k][DDP_IDX_JERK]);
        EXPECT_FALSE(free_solver.clamped_[k][DDP_IDX_ETA]);
    }
}

// EC 缓存正确性：同一轮内 EC₁/EC₂ 必须与按缓存方向 δx^l/δu^l 手工汇总
// （含终端项与交叉项）的结果一致；EC(α) 闭式求值不得再触发线性传播；
// 完整 solve 后线性 rollout 次数必须恰好等于后向传递次数
TEST(MsIlqrTest, EcCachingAndRolloutCounts) {
    const BicycleDynamics dynamics(kWheelbase);
    const DdpCostEvaluator evaluator(DdpCostConfig{}, nullptr);
    const std::size_t num_steps = 8;
    const DdpState x0 = MakeState(0.0, 0.0, 0.0, 0.5, 0.0, 0.04, 0.0);
    ArcProblem problem = MakeArcProblem(num_steps, x0, 0.02, 0.01, {4, 8});
    const DdpState offset =
        MakeState(0.10, -0.08, 0.03, 0.02, 0.01, 0.01, 0.01);
    problem.states[4] += offset;
    problem.states[8] += offset;
    MsIlqrTestAccess solver(MakeConfig(), &dynamics, &evaluator);
    PrepareNominal(&solver, problem.reference, MakeMultipliers(num_steps),
                   MakeCostInput(10.0), problem.states, problem.controls);
    ASSERT_TRUE(solver.backwardPass());
    solver.linearRollout();
    double ec1_manual = 0.0;
    double ec2_manual = 0.0;
    for (std::size_t k = 0; k < num_steps; ++k) {
        const auto& stage = solver.cost_eval_.stages[k];
        ec1_manual +=
            stage.lx.dot(solver.dx_lin_[k]) + stage.lu.dot(solver.du_lin_[k]);
        ec2_manual +=
            (solver.dx_lin_[k].transpose() * stage.lxx * solver.dx_lin_[k])
                .value() +
            (2.0 * solver.du_lin_[k].transpose() * stage.lux *
             solver.dx_lin_[k])
                .value() +
            (solver.du_lin_[k].transpose() * stage.luu * solver.du_lin_[k])
                .value();
    }
    const auto& terminal = solver.cost_eval_.stages[num_steps];
    ec1_manual += terminal.lx.dot(solver.dx_lin_[num_steps]);
    ec2_manual += (solver.dx_lin_[num_steps].transpose() * terminal.lxx *
                   solver.dx_lin_[num_steps])
                      .value();
    EXPECT_NEAR(ec1_manual, solver.ec1_, 1e-10);
    EXPECT_NEAR(ec2_manual, solver.ec2_, 1e-10);
    for (const double alpha : {0.13, 0.37, 0.5, 0.8, 1.0}) {
        EXPECT_NEAR(alpha * solver.ec1_ + 0.5 * alpha * alpha * solver.ec2_,
                    solver.expectedChange(alpha), 1e-12);
    }
    const MsIlqrResult result =
        solver.solve(problem.reference, MakeMultipliers(num_steps),
                     MakeCostInput(10.0), problem.states, problem.controls);
    ASSERT_NE(MsIlqrStatus::REGULARIZATION_OVERFLOW, result.status);
    EXPECT_EQ(solver.backward_pass_count_, solver.linear_rollout_count_);
    int total_trials = 0;
    int total_passes = 0;
    for (const auto& record : solver.history()) {
        total_trials += record.line_search_trials;
        total_passes += record.backward_passes;
    }
    EXPECT_EQ(total_trials, solver.nonlinear_rollout_count_);
    EXPECT_EQ(total_passes, solver.backward_pass_count_);
    EXPECT_GE(solver.qp_factorization_count_,
              static_cast<std::int64_t>(solver.backward_pass_count_) *
                  static_cast<std::int64_t>(num_steps));
}

// 非线性 rollout 公式与缺陷缩放：打靶节点状态按 x' = x_int + (α-1)·d̄
// 更新、段内闭环跟踪，接受后缺陷精确缩放为 (1-α)·d̄
TEST(MsIlqrTest, NonlinearRolloutDefectScaling) {
    const BicycleDynamics dynamics(kWheelbase);
    const DdpCostEvaluator evaluator(DdpCostConfig{}, nullptr);
    const std::size_t num_steps = 6;
    const DdpState x0 = MakeState(0.0, 0.0, 0.0, 0.5, 0.0, 0.04, 0.0);
    ArcProblem problem = MakeArcProblem(num_steps, x0, 0.02, 0.01, {3, 6});
    const DdpState offset =
        MakeState(0.10, -0.08, 0.03, 0.02, 0.01, 0.01, 0.01);
    problem.states[3] += offset;
    problem.states[6] += offset;
    MsIlqrTestAccess solver(MakeConfig(), &dynamics, &evaluator);
    PrepareNominal(&solver, problem.reference, MakeMultipliers(num_steps),
                   MakeCostInput(10.0), problem.states, problem.controls);
    ASSERT_TRUE(solver.backwardPass());
    solver.linearRollout();
    const DdpAlignedVec<DdpState> old_states = solver.states_;
    const DdpAlignedVec<DdpControl> old_controls = solver.controls_;
    const DdpAlignedVec<DdpState> old_defects = solver.defects_;
    const double old_defect_norm = solver.defectNorm();
    const double alpha = 0.5;
    solver.nonlinearRollout(alpha, problem.reference,
                            MakeMultipliers(num_steps), MakeCostInput(10.0));
    DdpState expected_state = old_states[0];
    ExpectMatrixNear(old_states[0], solver.cand_states_[0], 1e-12);
    for (std::size_t k = 0; k < num_steps; ++k) {
        const DdpControl expected_control =
            old_controls[k] + alpha * solver.feedforward_[k] +
            solver.gain_K_[k] * (expected_state - old_states[k]);
        ExpectMatrixNear(expected_control, solver.cand_controls_[k], 1e-12);
        expected_state = dynamics.step(expected_state, expected_control, kDt) +
                         (alpha - 1.0) * old_defects[k + 1];
        ExpectMatrixNear(expected_state, solver.cand_states_[k + 1], 1e-12);
    }
    solver.acceptCandidate(alpha);
    for (std::size_t i = 0; i <= num_steps; ++i) {
        ExpectMatrixNear((1.0 - alpha) * old_defects[i], solver.defects_[i],
                         1e-12);
    }
    EXPECT_NEAR((1.0 - alpha) * old_defect_norm, solver.defectNorm(), 1e-12);
}

// ρ_reg 变更后全量重分解：（白盒）ρ_reg 增大后重跑回推，逐步 QP 必须在
// 新 Hessian 上重新分解——分解计数不得复用旧分解；（端到端）静止起点 +
// 超宽盒 + 远距离大转向高权重目标，首轮方向过冲使线搜索耗尽，触发
// 正则化增大并重跑整个后向传递（盒不激活，方向可随 ρ 增大自由收缩）
TEST(MsIlqrTest, RegChangeRebuildsAllQps) {
    const BicycleDynamics dynamics(kWheelbase);
    const DdpCostEvaluator evaluator(DdpCostConfig{}, nullptr);
    const std::size_t num_steps = 8;
    const DdpState x0 = DdpState::Zero();
    ArcProblem problem = MakeArcProblem(num_steps, x0, 0.0, 0.0, {num_steps});
    for (std::size_t k = 0; k <= num_steps; ++k) {
        problem.reference.poses[k] = Pose(1.5, 1.5, 1.57);
    }
    MsIlqrTestAccess solver(MakeConfig(), &dynamics, &evaluator);
    PrepareNominal(&solver, problem.reference, MakeMultipliers(num_steps),
                   MakeCostInput(100.0), problem.states, problem.controls);
    ASSERT_TRUE(solver.backwardPass());
    const std::int64_t factorizations_first = solver.qp_factorization_count_;
    EXPECT_GE(factorizations_first, static_cast<std::int64_t>(num_steps));
    const double rho_before = solver.rho_reg_;
    ASSERT_TRUE(solver.increaseReg());
    EXPECT_GT(solver.rho_reg_, rho_before);
    ASSERT_TRUE(solver.backwardPass());
    EXPECT_GE(solver.qp_factorization_count_ - factorizations_first,
              static_cast<std::int64_t>(num_steps));
    MsIlqrConfig config = MakeConfig();
    config.jerk_max = 1e6;
    config.steer_accel_max = 1e6;
    config.armijo_gamma = 0.8;
    config.max_backtracks = 1;
    config.reg_increase = 100.0;
    config.reg_initial = 1e-6;
    MsIlqrTestAccess e2e_solver(config, &dynamics, &evaluator);
    const MsIlqrResult result =
        e2e_solver.solve(problem.reference, MakeMultipliers(num_steps),
                         MakeCostInput(1e6), problem.states, problem.controls);
    ASSERT_NE(MsIlqrStatus::REGULARIZATION_OVERFLOW, result.status);
    bool has_retry_iteration = false;
    for (const auto& record : e2e_solver.history()) {
        if (record.backward_passes > 1) {
            has_retry_iteration = true;
        }
    }
    EXPECT_TRUE(has_retry_iteration);
    EXPECT_GT(e2e_solver.backward_pass_count_, result.iterations);
    EXPECT_GE(e2e_solver.qp_factorization_count_,
              static_cast<std::int64_t>(e2e_solver.backward_pass_count_) *
                  static_cast<std::int64_t>(num_steps));
}

// 输入契约校验：空指针/非法配置/维度不符/打靶节点越界一律抛
// std::invalid_argument
TEST(MsIlqrTest, InputValidation) {
    const BicycleDynamics dynamics(kWheelbase);
    const DdpCostEvaluator evaluator(DdpCostConfig{}, nullptr);
    EXPECT_THROW(MsIlqrSolver(MakeConfig(), nullptr, &evaluator),
                 std::invalid_argument);
    EXPECT_THROW(MsIlqrSolver(MakeConfig(), &dynamics, nullptr),
                 std::invalid_argument);
    MsIlqrConfig bad_box = MakeConfig();
    bad_box.jerk_max = -1.0;
    EXPECT_THROW(MsIlqrSolver(bad_box, &dynamics, &evaluator),
                 std::invalid_argument);
    MsIlqrConfig bad_reg = MakeConfig();
    bad_reg.reg_min = 10.0;
    bad_reg.reg_max = 1.0;
    EXPECT_THROW(MsIlqrSolver(bad_reg, &dynamics, &evaluator),
                 std::invalid_argument);
    MsIlqrConfig bad_gamma = MakeConfig();
    bad_gamma.armijo_gamma = 1.5;
    EXPECT_THROW(MsIlqrSolver(bad_gamma, &dynamics, &evaluator),
                 std::invalid_argument);
    // merit 上限小于地板 µ₀：地板与上限冲突、语义不自洽，构造拒绝
    MsIlqrConfig bad_mu_max = MakeConfig();
    bad_mu_max.merit_mu_max = 0.5 * bad_mu_max.merit_mu0;
    EXPECT_THROW(MsIlqrSolver(bad_mu_max, &dynamics, &evaluator),
                 std::invalid_argument);
    const std::size_t num_steps = 4;
    ArcProblem problem =
        MakeArcProblem(num_steps, DdpState::Zero(), 0.0, 0.0, {num_steps});
    MsIlqrSolver solver(MakeConfig(), &dynamics, &evaluator);
    DdpAlignedVec<DdpState> bad_states = {DdpState::Zero()};
    EXPECT_THROW(
        solver.solve(problem.reference, MakeMultipliers(num_steps),
                     MakeCostInput(10.0), bad_states, problem.controls),
        std::invalid_argument);
    DdpReference bad_reference = problem.reference;
    bad_reference.shooting_nodes = {num_steps + 3};
    EXPECT_THROW(
        solver.solve(bad_reference, MakeMultipliers(num_steps),
                     MakeCostInput(10.0), problem.states, problem.controls),
        std::invalid_argument);
}

// 正则化溢出上报：reg_max 压到极小值（increaseReg 一次即返回 false），
// 并用静止起点 + 超宽盒 + 远距离大转向高权重目标使线搜索在 α=1 即被拒
// ——必须返回 REGULARIZATION_OVERFLOW。首轮 α=1 是否被接受依赖数值路径
// （跨平台/编译器不保证一致），故按两条路径分别断言 anytime 性质：
// 首轮即溢出则终态保持初始名义值；接受若干轮后溢出则终态恰等于最后
// 一次被接受迭代的记录
TEST(MsIlqrTest, RegularizationOverflowReported) {
    const BicycleDynamics dynamics(kWheelbase);
    const DdpCostEvaluator evaluator(DdpCostConfig{}, nullptr);
    const std::size_t num_steps = 8;
    ArcProblem problem =
        MakeArcProblem(num_steps, DdpState::Zero(), 0.0, 0.0, {num_steps});
    for (std::size_t k = 0; k <= num_steps; ++k) {
        problem.reference.poses[k] = Pose(1.5, 1.5, 1.57);
    }
    MsIlqrConfig config = MakeConfig();
    config.jerk_max = 1e6;
    config.steer_accel_max = 1e6;
    config.armijo_gamma = 0.9;
    config.max_backtracks = 1;
    config.reg_min = 1e-4;
    config.reg_initial = 1e-3;
    config.reg_max = 1e-3;
    MsIlqrTestAccess solver(config, &dynamics, &evaluator);
    const MsIlqrResult result =
        solver.solve(problem.reference, MakeMultipliers(num_steps),
                     MakeCostInput(1e6), problem.states, problem.controls);
    EXPECT_EQ(MsIlqrStatus::REGULARIZATION_OVERFLOW, result.status);
    EXPECT_EQ(result.iterations, static_cast<int>(solver.history().size()));
    if (solver.history().empty()) {
        EXPECT_DOUBLE_EQ(result.initial_cost, result.final_cost);
        EXPECT_DOUBLE_EQ(result.initial_defect_norm, result.final_defect_norm);
    } else {
        EXPECT_LT(result.final_cost, result.initial_cost);
        EXPECT_DOUBLE_EQ(solver.history().back().cost, result.final_cost);
        EXPECT_DOUBLE_EQ(solver.history().back().defect_norm,
                         result.final_defect_norm);
    }
}

// N=1 单步冒烟：最小规模问题应正常完成回推/双 rollout/线搜索全链路，
// 不崩溃且代价下降、快速收敛
TEST(MsIlqrTest, SingleStepProblemSmoke) {
    const BicycleDynamics dynamics(kWheelbase);
    const DdpCostEvaluator evaluator(DdpCostConfig{}, nullptr);
    const std::size_t num_steps = 1;
    const DdpReference reference = MakeReference(
        {Pose(0.05, 0.01, 0.05), Pose(0.1, 0.02, 0.05)}, kDt, {1});
    const DdpState x0 = MakeState(0.0, 0.0, 0.0, 0.3, 0.0, 0.02, 0.0);
    const DdpAlignedVec<DdpControl> controls = {MakeControl(0.2, 0.1)};
    const DdpAlignedVec<DdpState> states =
        RolloutStates(dynamics, x0, controls, kDt);
    MsIlqrTestAccess solver(MakeConfig(), &dynamics, &evaluator);
    const MsIlqrResult result =
        solver.solve(reference, MakeMultipliers(num_steps), MakeCostInput(10.0),
                     states, controls);
    ASSERT_NE(MsIlqrStatus::REGULARIZATION_OVERFLOW, result.status);
    ASSERT_NE(MsIlqrStatus::MAX_ITERATIONS, result.status);
    EXPECT_LT(result.final_cost, result.initial_cost);
    EXPECT_LT(result.final_defect_norm, 1e-12);
    ASSERT_EQ(2U, solver.states().size());
    EXPECT_LE(result.iterations, 5);
}

// 收敛出口的可行性守卫：相对代价/梯度判据达标时若打靶缺陷未愈合
// （‖d‖∞ 超容差），不得判收敛——AL 罚权重极大时增广 Hessian 病态、
// 线搜索只接受微步，会把「没走动的迭代」误报为收敛（尺度盲且不看
// 可行性），放行前必须确认缺陷已愈合
TEST(MsIlqrTest, ConvergenceRequiresFeasibility) {
    const BicycleDynamics dynamics(kWheelbase);
    const DdpCostEvaluator evaluator(DdpCostConfig{}, nullptr);
    MsIlqrTestAccess solver(MakeConfig(), &dynamics, &evaluator);
    // 容差默认与外层缺陷门同量级（1e-3）：缺陷未愈时不允许收敛出口
    solver.defects_.resize(3);
    solver.defects_[0].setZero();
    solver.defects_[1].setZero();
    solver.defects_[2].setZero();
    solver.defects_[1](DDP_IDX_DELTA) = 0.05;
    EXPECT_FALSE(solver.convergenceAllowed());
    solver.defects_[1](DDP_IDX_DELTA) = 1e-4;
    EXPECT_TRUE(solver.convergenceAllowed());
}

// merit 罚权重与 AL 罚量级挂钩（µ_m = max(µ_m0, c·µ_al)，默认 0 = 关闭、
// µ_m 钉住 µ₀）：AL 罚权重达到 1e6 量级时，钉住的 µ_m·‖d‖ 项相对增广
// 代价可忽略，线搜索事实上不再为「修复缺陷」付任何价钱（缺陷修复与
// 代价下降的交换比失衡）；按 AL 量级同步缩放保持交换比恒定——不用
// ‖d‖ 作触发/分母（小缺陷不放大，棘轮爆炸机理不成立），必须配
// merit_mu_max 封顶（c 过大重演「以任意代价歼灭缺陷」的灾难）
TEST(MsIlqrTest, MeritMuScalesWithAlPenalty) {
    const BicycleDynamics dynamics(kWheelbase);
    const DdpCostEvaluator evaluator(DdpCostConfig{}, nullptr);
    MsIlqrConfig config = MakeConfig();
    config.merit_mu0 = 100.0;
    config.merit_mu_max = 1e3;
    MsIlqrTestAccess solver(config, &dynamics, &evaluator);
    // 低于 µ₀：不动（钉住地板）
    solver.raiseMeritMuFloor(1e-3 * 1e4);  // 10 < 100
    EXPECT_DOUBLE_EQ(solver.merit_mu_, 100.0);
    // 高于 µ₀：抬升到 c·µ_al
    solver.raiseMeritMuFloor(1e-3 * 2e5);  // 200
    EXPECT_DOUBLE_EQ(solver.merit_mu_, 200.0);
    // 棘轮只升不降：回调不下降
    solver.raiseMeritMuFloor(1e-3 * 1e4);
    EXPECT_DOUBLE_EQ(solver.merit_mu_, 200.0);
    // 封顶：c·µ_al 超 cap 时截断（防「以任意代价歼灭缺陷」）
    solver.raiseMeritMuFloor(1e-3 * 1e6);  // 1000 = cap
    EXPECT_DOUBLE_EQ(solver.merit_mu_, 1e3);
    solver.raiseMeritMuFloor(1e-2 * 1e6);  // 1e4 > cap
    EXPECT_DOUBLE_EQ(solver.merit_mu_, 1e3);
    // 非有限输入拒绝（防御：不污染 µ_m）
    solver.raiseMeritMuFloor(std::numeric_limits<double>::quiet_NaN());
    EXPECT_DOUBLE_EQ(solver.merit_mu_, 1e3);
    // 非法配置（负比率）构造拒绝
    MsIlqrConfig bad = MakeConfig();
    bad.merit_mu_al_ratio = -1.0;
    EXPECT_THROW(MsIlqrSolver(bad, &dynamics, &evaluator),
                 std::invalid_argument);
}

// L8.3 前向定义域守卫：两个确定性场景——(A) 名义轨迹本身越出
// 「地图 ⊕ margin」时，每个试探候选都携带越界状态、全部回溯拒绝
// （拒绝计数 >0、以 REGULARIZATION_OVERFLOW 诚实失败而非坐标爆炸）；
// (B) 守卫关闭（margin=0）时同一问题拒绝计数为 0、求解正常推进
TEST(MsIlqrTest, DomainGuardRejectsOutOfMapCandidates) {
    const BicycleDynamics dynamics(kWheelbase);
    // 地图 [0,20)²（margin=2.0 ⟹ 域 y≤22）
    const GridMap grid_map(0.1, 200, 200, Position{0.0, 0.0}, {});
    const ESDFMap esdf_map(grid_map);
    const VehicleParams vehicle_params{4.9, 1.9, 2.7, 0.48};
    const VehicleFootprintModel footprint_model(vehicle_params, 233, 2, 1);
    const DdpEsdfConstraint esdf_constraint(esdf_map, footprint_model);
    const DdpCostEvaluator evaluator(DdpCostConfig{}, &esdf_constraint);
    // 名义轨迹：从 y=21.9 以 v=1.0 朝 +y 直行，第 2 步起越出 y≤22——
    // 每个候选（= 名义 + α·修正）都携带越界状态
    const std::size_t num_steps = 20;
    std::vector<Pose> poses;
    poses.reserve(num_steps + 1);
    for (std::size_t k = 0; k <= num_steps; ++k) {
        poses.emplace_back(10.0, 21.9 + 0.1 * static_cast<double>(k), 0.5 * PI);
    }
    const DdpReference reference = MakeReference(poses, kDt, {num_steps});
    const DdpState x0 = MakeState(10.0, 21.9, 0.5 * PI, 1.0, 0.0, 0.0, 0.0);
    DdpAlignedVec<DdpControl> controls;
    controls.resize(num_steps, DdpControl::Zero());
    const DdpAlignedVec<DdpState> states =
        RolloutStates(dynamics, x0, controls, kDt);
    // (A) 守卫开启：越界候选被拒绝（计数 >0），被接受的状态轨迹始终留在
    // 域内（y ≤ 22），无坐标爆炸（参考在域外不可达，收敛与否不作断言）
    {
        MsIlqrConfig config = MakeConfig();
        config.domain_guard_margin = 2.0;
        MsIlqrTestAccess solver(config, &dynamics, &evaluator);
        const MsIlqrResult result =
            solver.solve(reference, MakeMultipliers(num_steps),
                         MakeCostInput(10.0), states, controls);
        EXPECT_GT(result.domain_guard_rejections, 0);
        for (const auto& state : solver.states()) {
            EXPECT_LE(state(DDP_IDX_Y), 22.0 + 1e-9);
            EXPECT_LT(std::abs(state(DDP_IDX_X)), 1e6);
            EXPECT_LT(std::abs(state(DDP_IDX_Y)), 1e6);
        }
    }
    // (B) 守卫关闭：同一问题拒绝计数为 0、求解正常推进
    {
        MsIlqrConfig config = MakeConfig();
        config.domain_guard_margin = 0.0;
        MsIlqrTestAccess solver(config, &dynamics, &evaluator);
        const MsIlqrResult result =
            solver.solve(reference, MakeMultipliers(num_steps),
                         MakeCostInput(10.0), states, controls);
        EXPECT_EQ(result.domain_guard_rejections, 0);
        EXPECT_GT(result.iterations, 0);
    }
    // 非法 margin 构造拒绝
    {
        MsIlqrConfig bad = MakeConfig();
        bad.domain_guard_margin = -1.0;
        EXPECT_THROW(MsIlqrTestAccess(bad, &dynamics, &evaluator),
                     std::invalid_argument);
    }
}

}  // namespace
}  // namespace apa_post_processor
