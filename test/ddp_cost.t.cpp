#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

#include "core/DDP/ddp_cost.h"
#include "core/DDP/esdf_constraint.h"
#include "spatial/esdf_map.h"
#include "spatial/grid_map.h"
#include "vehicle/vehicle_footprint_model.h"
#include "vehicle/vehicle_params.h"

namespace apa_post_processor {
namespace {

// π/2（ESDF 集成场景中平行于墙面的位姿航向）
const double kHalfPi = std::acos(-1.0) / 2.0;

// 对拍辅助：相对误差 < rel_tol，近零分量退化为绝对误差 rel_tol*1e-3
void ExpectComponentClose(double expected, double actual, double rel_tol) {
    EXPECT_LE(std::abs(expected - actual),
              rel_tol * std::max(std::abs(actual), 1e-3))
        << "expected=" << expected << " actual=" << actual;
}

// 矩阵/向量逐元素对拍
template <typename TDerivedA, typename TDerivedB>
void ExpectMatrixClose(const Eigen::MatrixBase<TDerivedA>& expected,
                       const Eigen::MatrixBase<TDerivedB>& actual,
                       double rel_tol) {
    ASSERT_EQ(expected.rows(), actual.rows());
    ASSERT_EQ(expected.cols(), actual.cols());
    for (int row = 0; row < expected.rows(); ++row) {
        for (int col = 0; col < expected.cols(); ++col) {
            ExpectComponentClose(expected(row, col), actual(row, col), rel_tol);
        }
    }
}

// 五点中心差分 [-f(x+2h)+8f(x+h)-8f(x-h)+f(x-2h)]/(12h)，截断误差 O(h^4)
template <typename TFunc>
double FivePointCentral(TFunc&& func, double h) {
    return (-func(2.0 * h) + 8.0 * func(h) - 8.0 * func(-h) + func(-2.0 * h)) /
           (12.0 * h);
}

// 公共测试夹具：N=3 的小型参考轨迹（4 个位姿），状态/控制全部在幅值边界内
// 且与参考位姿有可控偏差，保证平滑/跟踪/终点项都有非零信号
class DdpCostEvaluatorTest : public ::testing::Test {
   protected:
    DdpCostEvaluatorTest() {
        reference_.ds = 0.05;
        reference_.dt = kDt;
        reference_.poses = {Pose{0.0, 0.0, 0.0}, Pose{0.05, 0.0, 0.0},
                            Pose{0.10, 0.01, 0.10}, Pose{0.15, 0.03, 0.15}};
    }

    // 按状态分量布局 [x, y, θ, v, a, δ, ω] 构造一个七维状态
    static DdpState MakeState(double x, double y, double theta, double v,
                              double a, double delta, double omega) {
        DdpState state;
        state << x, y, theta, v, a, delta, omega;
        return state;
    }

    // 全部状态在幅值边界内（λ=0 时幅值 AL 不激活）
    static DdpAlignedVec<DdpState> MakeStates() {
        DdpAlignedVec<DdpState> states;
        states.reserve(kNumSteps + 1);
        states.push_back(MakeState(0.00, 0.000, 0.00, 0.50, 0.20, 0.10, 0.05));
        states.push_back(
            MakeState(0.06, 0.005, 0.02, 0.60, -0.10, 0.12, -0.08));
        states.push_back(MakeState(0.11, 0.015, 0.08, 0.40, 0.15, 0.08, 0.10));
        states.push_back(MakeState(0.16, 0.035, 0.14, 0.05, 0.05, 0.20, 0.00));
        return states;
    }

    static DdpAlignedVec<DdpControl> MakeControls() {
        DdpAlignedVec<DdpControl> controls;
        controls.reserve(kNumSteps);
        controls.push_back(MakeControl(0.30, 0.20));
        controls.push_back(MakeControl(-0.20, 0.10));
        controls.push_back(MakeControl(0.10, -0.15));
        return controls;
    }

    static DdpControl MakeControl(double jerk, double eta) {
        DdpControl control;
        control << jerk, eta;
        return control;
    }

    // 便捷求值：reference 显式传入，便于个别用例改写参考位姿
    static DdpCostEvaluation EvaluateAll(
        const DdpCostEvaluator& evaluator, const DdpReference& reference,
        const DdpAlignedVec<DdpState>& states,
        const DdpAlignedVec<DdpControl>& controls,
        const DdpCostMultiplierState& multipliers, double tracking_weight,
        const std::vector<bool>* mask = nullptr) {
        DdpCostInput input;
        input.tracking_weight = tracking_weight;
        input.anneal_exempt_mask = mask;
        return evaluator.evaluate(reference, states, controls, multipliers,
                                  input);
    }

    // 全轨迹总代价（供有限差分）：扰动由调用方先作用于 states/controls 副本
    static double TotalCost(const DdpCostEvaluator& evaluator,
                            const DdpReference& reference,
                            const DdpAlignedVec<DdpState>& states,
                            const DdpAlignedVec<DdpControl>& controls,
                            const DdpCostMultiplierState& multipliers,
                            double tracking_weight) {
        return EvaluateAll(evaluator, reference, states, controls, multipliers,
                           tracking_weight)
            .total_cost;
    }

    // 全零权重的配置（隔离出单一代价通道做精确手推校验）
    static DdpCostConfig MakeZeroWeightConfig() {
        DdpCostConfig config;
        config.weight_jerk = 0.0;
        config.weight_steer_accel = 0.0;
        config.weight_ref_base = 0.0;
        config.weight_theta = 0.0;
        return config;
    }

    static constexpr std::size_t kNumSteps = 3;
    static constexpr double kDt = 0.1;
    DdpReference reference_;
};

// 测试构造期配置校验：负权重、非有限权重、非正 shift_beta、非正幅值边界
// 均必须抛 std::invalid_argument。因为非法配置会静默污染全部下游求解。
TEST_F(DdpCostEvaluatorTest, ConstructorThrowsOnInvalidConfig) {
    DdpCostConfig config;
    config.weight_jerk = -1.0;
    EXPECT_THROW(DdpCostEvaluator(config, nullptr), std::invalid_argument);
    config.weight_jerk = 1.0;
    config.weight_theta = std::numeric_limits<double>::quiet_NaN();
    EXPECT_THROW(DdpCostEvaluator(config, nullptr), std::invalid_argument);
    config.weight_theta = 5.0;
    config.shift_beta = 0.0;
    EXPECT_THROW(DdpCostEvaluator(config, nullptr), std::invalid_argument);
    config.shift_beta = 0.1;
    config.v_max = 0.0;
    EXPECT_THROW(DdpCostEvaluator(config, nullptr), std::invalid_argument);
    config.v_max = 1.5;
    config.delta_max = -0.5;
    EXPECT_THROW(DdpCostEvaluator(config, nullptr), std::invalid_argument);
    config.delta_max = 0.55;
    config.weight_shift = -0.5;
    EXPECT_THROW(DdpCostEvaluator(config, nullptr), std::invalid_argument);
}

// 测试求值入参维度校验：状态数≠N+1、控制数≠N、幅值乘子尺寸≠5N、
// 豁免掩码尺寸≠N+1、参考位姿不足两个、dt 非正、退火权重非有限，
// 均必须抛 std::invalid_argument——维度错配若静默放行会把导数写串位。
TEST_F(DdpCostEvaluatorTest, EvaluateThrowsOnDimensionMismatch) {
    const DdpCostEvaluator evaluator(DdpCostConfig{}, nullptr);
    const auto states = MakeStates();
    const auto controls = MakeControls();
    const auto multipliers = DdpCostMultiplierState::MakeZero(kNumSteps);
    DdpCostInput input;
    auto bad_states = states;
    bad_states.pop_back();
    EXPECT_THROW(evaluator.evaluate(reference_, bad_states, controls,
                                    multipliers, input),
                 std::invalid_argument);
    auto bad_controls = controls;
    bad_controls.push_back(DdpControl::Zero());
    EXPECT_THROW(evaluator.evaluate(reference_, states, bad_controls,
                                    multipliers, input),
                 std::invalid_argument);
    auto bad_multipliers = multipliers;
    bad_multipliers.amplitude_lambda = Eigen::VectorXd::Zero(14);
    EXPECT_THROW(evaluator.evaluate(reference_, states, controls,
                                    bad_multipliers, input),
                 std::invalid_argument);
    bad_multipliers = multipliers;
    bad_multipliers.amplitude_mu = Eigen::VectorXd::Zero(16);
    EXPECT_THROW(evaluator.evaluate(reference_, states, controls,
                                    bad_multipliers, input),
                 std::invalid_argument);
    const std::vector<bool> bad_mask(3, false);
    EXPECT_THROW(EvaluateAll(evaluator, reference_, states, controls,
                             multipliers, 0.0, &bad_mask),
                 std::invalid_argument);
    DdpReference short_ref;
    short_ref.dt = kDt;
    short_ref.poses = {Pose{0.0, 0.0, 0.0}};
    EXPECT_THROW(
        evaluator.evaluate(short_ref, states, controls, multipliers, input),
        std::invalid_argument);
    DdpReference zero_dt_ref = reference_;
    zero_dt_ref.dt = 0.0;
    EXPECT_THROW(
        evaluator.evaluate(zero_dt_ref, states, controls, multipliers, input),
        std::invalid_argument);
    EXPECT_THROW(
        EvaluateAll(evaluator, reference_, states, controls, multipliers,
                    std::numeric_limits<double>::quiet_NaN()),
        std::invalid_argument);
}

// 测试平滑/跟踪/终点三类二次项的全要素有限差分对拍（相对误差 < 1e-6）。
// 幅值 AL 未激活、换挡代理关闭、ESDF 缺省时全部代价项都是精确二次型，
// GN 形 Hessian 与精确 Hessian 一致：梯度用"总代价五点差分"对拍
// （代价按阶段解耦，∂total/∂x_k 即阶段 k 的 lx），Hessian 用"解析梯度
// 再做中心差分"对拍，同时覆盖 lx/lu/lxx/luu/lux 全部五个输出。
TEST_F(DdpCostEvaluatorTest, QuadraticTermsMatchFiniteDifference) {
    DdpCostConfig config;
    config.weight_jerk = 1.0;
    config.weight_steer_accel = 2.0;
    config.weight_ref_base = 10.0;
    config.weight_theta = 5.0;
    const DdpCostEvaluator evaluator(config, nullptr);
    const auto states = MakeStates();
    const auto controls = MakeControls();
    auto multipliers = DdpCostMultiplierState::MakeZero(kNumSteps);
    multipliers.amplitude_mu.setConstant(3.0);
    multipliers.terminal_lambda << 0.3, -0.2, 0.5, 0.7, -0.4;
    multipliers.terminal_mu << 10.0, 20.0, 30.0, 40.0, 50.0;
    const double tracking_weight = 4.0;
    const auto result = EvaluateAll(evaluator, reference_, states, controls,
                                    multipliers, tracking_weight);
    for (std::size_t k = 0; k <= kNumSteps; ++k) {
        for (int comp = 0; comp < DDP_STATE_DIM; ++comp) {
            auto cost_at = [&](double offset) {
                auto perturbed = states;
                perturbed[k](comp) += offset;
                return TotalCost(evaluator, reference_, perturbed, controls,
                                 multipliers, tracking_weight);
            };
            ExpectComponentClose(FivePointCentral(cost_at, 1e-4),
                                 result.stages[k].lx(comp), 1e-6);
        }
    }
    for (std::size_t k = 0; k < kNumSteps; ++k) {
        for (int comp = 0; comp < DDP_CONTROL_DIM; ++comp) {
            auto cost_at = [&](double offset) {
                auto perturbed = controls;
                perturbed[k](comp) += offset;
                return TotalCost(evaluator, reference_, states, perturbed,
                                 multipliers, tracking_weight);
            };
            ExpectComponentClose(FivePointCentral(cost_at, 1e-4),
                                 result.stages[k].lu(comp), 1e-6);
        }
    }
    for (std::size_t k = 0; k <= kNumSteps; ++k) {
        for (int comp = 0; comp < DDP_STATE_DIM; ++comp) {
            auto grad_at = [&](double offset) {
                auto perturbed = states;
                perturbed[k](comp) += offset;
                return EvaluateAll(evaluator, reference_, perturbed, controls,
                                   multipliers, tracking_weight)
                    .stages[k]
                    .lx;
            };
            const DdpState numeric_col =
                (grad_at(1e-4) - grad_at(-1e-4)) / 2e-4;
            ExpectMatrixClose(numeric_col, result.stages[k].lxx.col(comp),
                              1e-6);
            auto ctrl_grad_at = [&](double offset) {
                auto perturbed = states;
                perturbed[k](comp) += offset;
                return EvaluateAll(evaluator, reference_, perturbed, controls,
                                   multipliers, tracking_weight)
                    .stages[k]
                    .lu;
            };
            const DdpControl numeric_mixed_col =
                (ctrl_grad_at(1e-4) - ctrl_grad_at(-1e-4)) / 2e-4;
            ExpectMatrixClose(numeric_mixed_col, result.stages[k].lux.col(comp),
                              1e-6);
        }
    }
    for (std::size_t k = 0; k < kNumSteps; ++k) {
        for (int comp = 0; comp < DDP_CONTROL_DIM; ++comp) {
            auto grad_at = [&](double offset) {
                auto perturbed = controls;
                perturbed[k](comp) += offset;
                return EvaluateAll(evaluator, reference_, states, perturbed,
                                   multipliers, tracking_weight)
                    .stages[k]
                    .lu;
            };
            const DdpControl numeric_col =
                (grad_at(1e-4) - grad_at(-1e-4)) / 2e-4;
            ExpectMatrixClose(numeric_col, result.stages[k].luu.col(comp),
                              1e-6);
        }
    }
    // 终端阶段不含控制量：控制相关导数恒零
    EXPECT_TRUE(result.stages[kNumSteps].lu.isZero());
    EXPECT_TRUE(result.stages[kNumSteps].luu.isZero());
    EXPECT_TRUE(result.stages[kNumSteps].lux.isZero());
}

// 测试幅值 AL 门控的零贡献行为：不等式未违反（g≤0）且 λ=0 时，
// 值与全部导数必须恒为零（约束完全退出代价）；g=0 边界且 λ=0 时同样为零。
// 这是"未激活约束不得牵引轨迹"的直接钉住。
TEST_F(DdpCostEvaluatorTest, AmplitudeGatingZeroWhenInactive) {
    const DdpCostEvaluator evaluator(MakeZeroWeightConfig(), nullptr);
    const auto states = MakeStates();
    const auto controls = MakeControls();
    auto multipliers = DdpCostMultiplierState::MakeZero(kNumSteps);
    multipliers.amplitude_mu.setConstant(5.0);
    const auto result =
        EvaluateAll(evaluator, reference_, states, controls, multipliers, 0.0);
    for (const auto& stage : result.stages) {
        EXPECT_DOUBLE_EQ(stage.cost_amplitude, 0.0);
        EXPECT_DOUBLE_EQ(stage.totalCost(), 0.0);
        EXPECT_TRUE(stage.lx.isZero());
        EXPECT_TRUE(stage.lu.isZero());
        EXPECT_TRUE(stage.lxx.isZero());
        EXPECT_TRUE(stage.luu.isZero());
        EXPECT_TRUE(stage.lux.isZero());
    }
    // 终端阶段单独显式钉住：末态与参考位姿虽有偏差，但终点 AL 乘子为零、
    // 无 ESDF，终点代价与导数同样必须恒零（不依赖"遍历所有阶段"的间接覆盖）
    const auto& terminal = result.stages[kNumSteps];
    EXPECT_DOUBLE_EQ(terminal.cost_terminal, 0.0);
    EXPECT_DOUBLE_EQ(terminal.totalCost(), 0.0);
    EXPECT_TRUE(terminal.lx.isZero());
    EXPECT_TRUE(terminal.lxx.isZero());
    EXPECT_TRUE(terminal.lu.isZero());
    EXPECT_TRUE(terminal.luu.isZero());
    EXPECT_TRUE(terminal.lux.isZero());
    // g=0 恰在边界（v=v_max）且 λ=0：按门控约定不激活，贡献恒零
    auto boundary_states = MakeStates();
    boundary_states[1](DDP_IDX_V) = 1.5;
    const auto boundary_result = EvaluateAll(
        evaluator, reference_, boundary_states, controls, multipliers, 0.0);
    EXPECT_DOUBLE_EQ(boundary_result.stages[1].cost_amplitude, 0.0);
    EXPECT_TRUE(boundary_result.stages[1].lx.isZero());
    EXPECT_TRUE(boundary_result.stages[1].lxx.isZero());
}

// 测试幅值 AL 违反约束时的手推值/梯度/GN Hessian。
// 因为 v²−v_max² 形态的 GN Hessian μ·(∂g/∂v)² 丢弃了精确 Hessian 中的
// 2(λ+μg) 项（iLQR 层级的标准近似），本用例同时钉住两件事：
// (i) 实现的 Hessian 等于"约束雅可比数值差分按 GN 公式装配"的结果；
// (ii) 解析梯度（精确）的数值差分确实等于精确 Hessian——证明偏差是
// 有意为之的 GN 丢弃项而非实现错误。
TEST_F(DdpCostEvaluatorTest, AmplitudeViolatedConstraintMatchesHandDerived) {
    const DdpCostEvaluator evaluator(MakeZeroWeightConfig(), nullptr);
    auto states = MakeStates();
    states[1](DDP_IDX_V) = 1.6;
    const auto controls = MakeControls();
    auto multipliers = DdpCostMultiplierState::MakeZero(kNumSteps);
    const int row = DDP_AMPLITUDE_CONSTRAINT_DIM * 1 + DDP_AMP_V;
    multipliers.amplitude_lambda(row) = 0.3;
    multipliers.amplitude_mu(row) = 10.0;
    const auto result =
        EvaluateAll(evaluator, reference_, states, controls, multipliers, 0.0);
    const double g = 1.6 * 1.6 - 1.5 * 1.5;
    EXPECT_NEAR(0.3 * g + 0.5 * 10.0 * g * g, result.stages[1].cost_amplitude,
                1e-12);
    EXPECT_NEAR((0.3 + 10.0 * g) * 3.2, result.stages[1].lx(DDP_IDX_V), 1e-12);
    EXPECT_NEAR(10.0 * 3.2 * 3.2, result.stages[1].lxx(DDP_IDX_V, DDP_IDX_V),
                1e-9);
    // 未挂乘子的阶段/分量恒零
    EXPECT_DOUBLE_EQ(result.stages[0].totalCost(), 0.0);
    EXPECT_DOUBLE_EQ(result.stages[2].totalCost(), 0.0);
    EXPECT_DOUBLE_EQ(result.stages[1].lx(DDP_IDX_A), 0.0);
    EXPECT_DOUBLE_EQ(result.stages[1].lxx(DDP_IDX_A, DDP_IDX_A), 0.0);
    // (ii) 解析梯度的数值差分 = 精确 Hessian 2(λ+μg)+μ(2v)² = 109.2
    auto grad_at = [&](double offset) {
        auto perturbed = states;
        perturbed[1](DDP_IDX_V) += offset;
        return EvaluateAll(evaluator, reference_, perturbed, controls,
                           multipliers, 0.0)
            .stages[1]
            .lx(DDP_IDX_V);
    };
    const double exact_hessian = (grad_at(1e-4) - grad_at(-1e-4)) / 2e-4;
    ExpectComponentClose(2.0 * (0.3 + 10.0 * g) + 10.0 * 3.2 * 3.2,
                         exact_hessian, 1e-6);
    EXPECT_GT(
        std::abs(exact_hessian - result.stages[1].lxx(DDP_IDX_V, DDP_IDX_V)),
        1.0);
    // (i) GN 装配：约束雅可比数值差分 ∂g/∂v → μ·(∂g/∂v)²
    auto constraint_at = [&](double offset) {
        const double v = 1.6 + offset;
        return v * v - 1.5 * 1.5;
    };
    const double dg_numeric = FivePointCentral(constraint_at, 1e-4);
    ExpectComponentClose(10.0 * dg_numeric * dg_numeric,
                         result.stages[1].lxx(DDP_IDX_V, DDP_IDX_V), 1e-6);
    // 梯度对拍（解析梯度精确）：总代价数值差分
    auto cost_at = [&](double offset) {
        auto perturbed = states;
        perturbed[1](DDP_IDX_V) += offset;
        return TotalCost(evaluator, reference_, perturbed, controls,
                         multipliers, 0.0);
    };
    ExpectComponentClose(FivePointCentral(cost_at, 1e-4),
                         result.stages[1].lx(DDP_IDX_V), 1e-6);
}

// 测试幅值 AL 的乘子记忆效应：g<0 但 λ>0 时约束仍激活（乘子"记住"了
// 曾经的违反），代价/梯度按 λg+½μg² 与 (λ+μg)·∂g 闭式取值。
TEST_F(DdpCostEvaluatorTest, AmplitudeMultiplierMemoryActivatesBelowBound) {
    const DdpCostEvaluator evaluator(MakeZeroWeightConfig(), nullptr);
    auto states = MakeStates();
    states[1](DDP_IDX_V) = 1.4;
    const auto controls = MakeControls();
    auto multipliers = DdpCostMultiplierState::MakeZero(kNumSteps);
    const int row = DDP_AMPLITUDE_CONSTRAINT_DIM * 1 + DDP_AMP_V;
    multipliers.amplitude_lambda(row) = 0.3;
    multipliers.amplitude_mu(row) = 10.0;
    const auto result =
        EvaluateAll(evaluator, reference_, states, controls, multipliers, 0.0);
    const double g = 1.4 * 1.4 - 1.5 * 1.5;
    EXPECT_LT(g, 0.0);
    EXPECT_NEAR(0.3 * g + 0.5 * 10.0 * g * g, result.stages[1].cost_amplitude,
                1e-12);
    EXPECT_NEAR((0.3 + 10.0 * g) * 2.8, result.stages[1].lx(DDP_IDX_V), 1e-12);
    EXPECT_NEAR(10.0 * 2.8 * 2.8, result.stages[1].lxx(DDP_IDX_V, DDP_IDX_V),
                1e-9);
}

// 测试 δ 双侧线性幅值约束：δ−δ_max≤0 与 −δ−δ_max≤0 的梯度恒为 ±1，
// GN Hessian 与精确 Hessian 一致（线性约束无二阶项），
// 且两侧约束共用同一个状态行不会互相串扰。
TEST_F(DdpCostEvaluatorTest, DeltaLinearConstraintsShareDeltaRow) {
    const DdpCostEvaluator evaluator(MakeZeroWeightConfig(), nullptr);
    const auto controls = MakeControls();
    // δ 正向越界：仅 δ−δ_max 分支激活
    auto states = MakeStates();
    states[0](DDP_IDX_DELTA) = 0.6;
    auto multipliers = DdpCostMultiplierState::MakeZero(kNumSteps);
    multipliers.amplitude_lambda(DDP_AMP_DELTA_POS) = 0.1;
    multipliers.amplitude_mu(DDP_AMP_DELTA_POS) = 7.0;
    const auto pos_result =
        EvaluateAll(evaluator, reference_, states, controls, multipliers, 0.0);
    EXPECT_NEAR(0.1 * 0.05 + 0.5 * 7.0 * 0.0025,
                pos_result.stages[0].cost_amplitude, 1e-12);
    EXPECT_NEAR(0.1 + 7.0 * 0.05, pos_result.stages[0].lx(DDP_IDX_DELTA),
                1e-12);
    auto pos_grad_at = [&](double offset) {
        auto perturbed = states;
        perturbed[0](DDP_IDX_DELTA) += offset;
        return EvaluateAll(evaluator, reference_, perturbed, controls,
                           multipliers, 0.0)
            .stages[0]
            .lx(DDP_IDX_DELTA);
    };
    ExpectComponentClose((pos_grad_at(1e-4) - pos_grad_at(-1e-4)) / 2e-4,
                         pos_result.stages[0].lxx(DDP_IDX_DELTA, DDP_IDX_DELTA),
                         1e-6);
    // δ 负向越界：仅 −δ−δ_max 分支激活，梯度符号翻转
    states[0](DDP_IDX_DELTA) = -0.6;
    multipliers.amplitude_lambda(DDP_AMP_DELTA_POS) = 0.0;
    multipliers.amplitude_mu(DDP_AMP_DELTA_POS) = 0.0;
    multipliers.amplitude_lambda(DDP_AMP_DELTA_NEG) = 0.2;
    multipliers.amplitude_mu(DDP_AMP_DELTA_NEG) = 11.0;
    const auto neg_result =
        EvaluateAll(evaluator, reference_, states, controls, multipliers, 0.0);
    EXPECT_NEAR(0.2 * 0.05 + 0.5 * 11.0 * 0.0025,
                neg_result.stages[0].cost_amplitude, 1e-12);
    EXPECT_NEAR(-(0.2 + 11.0 * 0.05), neg_result.stages[0].lx(DDP_IDX_DELTA),
                1e-12);
    EXPECT_NEAR(11.0, neg_result.stages[0].lxx(DDP_IDX_DELTA, DDP_IDX_DELTA),
                1e-12);
}

// 测试跟踪项角度 wrap：参考角贴近 +π、状态角滑过 ±π 分界线时，
// 代价与梯度必须连续无 2π 跳变（wrap 实现与初值提取/终端约束同源）。
// 若 wrap 缺失，θ=−π+0.05 处的误差会被算成 ≈2π−0.051，代价差 5 个数量级。
TEST_F(DdpCostEvaluatorTest, TrackingAngleWrapAcrossPiSeam) {
    DdpCostConfig config = MakeZeroWeightConfig();
    config.weight_theta = 8.0;
    const DdpCostEvaluator evaluator(config, nullptr);
    auto reference = reference_;
    reference.poses[1] = Pose{0.05, 0.0, PI - 1e-3};
    const auto controls = MakeControls();
    const auto multipliers = DdpCostMultiplierState::MakeZero(kNumSteps);
    // 分界线同侧（θ=π−0.05）：误差 −0.049
    auto states = MakeStates();
    states[1](DDP_IDX_THETA) = PI - 0.05;
    const auto before =
        EvaluateAll(evaluator, reference, states, controls, multipliers, 0.0);
    EXPECT_NEAR(0.5 * 8.0 * 0.049 * 0.049 * kDt, before.stages[1].cost_tracking,
                1e-12);
    EXPECT_NEAR(8.0 * (-0.049) * kDt, before.stages[1].lx(DDP_IDX_THETA),
                1e-12);
    // 跨过分界线（θ=−π+0.05）：wrap 后误差 +0.051 而非 −(2π−0.051)
    states[1](DDP_IDX_THETA) = -PI + 0.05;
    const auto after =
        EvaluateAll(evaluator, reference, states, controls, multipliers, 0.0);
    EXPECT_NEAR(0.5 * 8.0 * 0.051 * 0.051 * kDt, after.stages[1].cost_tracking,
                1e-12);
    EXPECT_NEAR(8.0 * 0.051 * kDt, after.stages[1].lx(DDP_IDX_THETA), 1e-12);
    // 无 2π 跳变：两侧代价/梯度均为小量（若 wrap 缺失，θ=−π+0.05 处
    // 误差会算成 ≈2π−0.051，代价 ~15.8、梯度 ~5.0，比实测大两个数量级）
    EXPECT_LT(std::abs(after.stages[1].cost_tracking), 0.02);
    EXPECT_LT(std::abs(after.stages[1].lx(DDP_IDX_THETA)), 0.05);
    EXPECT_LT(std::abs(before.stages[1].cost_tracking), 0.02);
    EXPECT_LT(std::abs(before.stages[1].lx(DDP_IDX_THETA)), 0.05);
}

// 测试退火掩码：掩码豁免点的 w_ref 恒取 w_ref,0（不随轮次衰减），
// 非豁免点按外部输入的 w_ref(r) 取值——首/末 maneuver 与锚点因此
// 能在退火后期仍保持跟踪压力。
TEST_F(DdpCostEvaluatorTest, AnnealMaskExemptsPointsFromDecay) {
    DdpCostConfig config = MakeZeroWeightConfig();
    config.weight_ref_base = 10.0;
    const DdpCostEvaluator evaluator(config, nullptr);
    const auto states = MakeStates();
    const auto controls = MakeControls();
    const auto multipliers = DdpCostMultiplierState::MakeZero(kNumSteps);
    const std::vector<bool> mask = {true, false, false, false};
    const auto round0 = EvaluateAll(evaluator, reference_, states, controls,
                                    multipliers, 10.0, &mask);
    const auto round3 = EvaluateAll(evaluator, reference_, states, controls,
                                    multipliers, 1.25, &mask);
    // 豁免点（阶段 0）：r=0 与 r=3 的跟踪代价/梯度完全一致
    EXPECT_DOUBLE_EQ(round0.stages[0].cost_tracking,
                     round3.stages[0].cost_tracking);
    EXPECT_TRUE(round3.stages[0].lx.isApprox(round0.stages[0].lx));
    // 非豁免点（阶段 1/2）：按 γ³=0.125 缩放
    const double decay = 1.25 / 10.0;
    for (std::size_t k = 1; k < kNumSteps; ++k) {
        ExpectComponentClose(round0.stages[k].cost_tracking * decay,
                             round3.stages[k].cost_tracking, 1e-12);
        ExpectMatrixClose(round0.stages[k].lx * decay, round3.stages[k].lx,
                          1e-12);
    }
    // 无掩码时全部点都衰减
    const auto round3_nomask =
        EvaluateAll(evaluator, reference_, states, controls, multipliers, 1.25);
    ExpectComponentClose(round0.stages[0].cost_tracking * decay,
                         round3_nomask.stages[0].cost_tracking, 1e-12);
}

// 测试换挡代理门的同号关闭行为：v 与显式一步预测 v⁺=v+a·dt 同号时
// 贡献在数值上为零（σ_β 是光滑门而非硬门，|v|/β 足够大时按指数趋零）；
// 同时钉住纯状态性质——对控制的导数 lu/luu/lux 恒为零。
TEST_F(DdpCostEvaluatorTest, ShiftProxyZeroWhenSameSign) {
    DdpCostConfig config = MakeZeroWeightConfig();
    config.weight_shift = 2.0;
    config.shift_beta = 0.05;
    const DdpCostEvaluator evaluator(config, nullptr);
    const auto controls = MakeControls();
    const auto multipliers = DdpCostMultiplierState::MakeZero(kNumSteps);
    // 全程 v=±1.0、a=0：v⁺=v 同号，|v|/β=20 时门漏损 ~e^(-40)
    for (const double sign : {1.0, -1.0}) {
        auto states = MakeStates();
        for (auto& state : states) {
            state(DDP_IDX_V) = sign * 1.0;
            state(DDP_IDX_A) = 0.0;
        }
        const auto result = EvaluateAll(evaluator, reference_, states, controls,
                                        multipliers, 0.0);
        for (std::size_t k = 0; k < kNumSteps; ++k) {
            EXPECT_LT(result.stages[k].cost_shift, 1e-12);
            EXPECT_LT(result.stages[k].lx.norm(), 1e-12);
            EXPECT_LT(result.stages[k].lxx.norm(), 1e-12);
            EXPECT_TRUE(result.stages[k].lu.isZero());
            EXPECT_TRUE(result.stages[k].luu.isZero());
            EXPECT_TRUE(result.stages[k].lux.isZero());
        }
    }
}

// 测试换挡代理门的变号惩罚与有限差分对拍。
// 门内（v>0、v⁺<0）代价按 ℓ=w_g·[σ(v)σ(−v⁺)+σ(−v)σ(v⁺)] 取手推值；
// 梯度/Hessian（精确二阶，纯状态 (v,a) 块）与数值差分对拍 < 1e-6；
// 显式预测 v⁺=v+a·dt 刻意不代入动力学链，控制导数必须恒零。
TEST_F(DdpCostEvaluatorTest, ShiftProxyMatchesHandDerivedAndFd) {
    DdpCostConfig config = MakeZeroWeightConfig();
    config.weight_shift = 2.0;
    config.shift_beta = 0.1;
    const DdpCostEvaluator evaluator(config, nullptr);
    auto states = MakeStates();
    states[1](DDP_IDX_V) = 0.08;
    states[1](DDP_IDX_A) = -1.0;
    const auto controls = MakeControls();
    const auto multipliers = DdpCostMultiplierState::MakeZero(kNumSteps);
    const auto result =
        EvaluateAll(evaluator, reference_, states, controls, multipliers, 0.0);
    // 手推代价：v⁺=0.08−0.1=−0.02，A=σ(0.8)、D=σ(−0.2)，ℓ=2·(A+D−2AD)
    const double sig_v = 0.5 * (1.0 + std::tanh(0.8));
    const double sig_w = 0.5 * (1.0 + std::tanh(-0.2));
    EXPECT_NEAR(2.0 * (sig_v + sig_w - 2.0 * sig_v * sig_w),
                result.stages[1].cost_shift, 1e-12);
    // 纯状态：控制导数恒零（即使在门内梯度非零处）
    EXPECT_TRUE(result.stages[1].lu.isZero());
    EXPECT_TRUE(result.stages[1].luu.isZero());
    EXPECT_TRUE(result.stages[1].lux.isZero());
    // 梯度对拍（总代价五点差分，门区光滑不跨折点）
    for (const int comp : {DDP_IDX_V, DDP_IDX_A}) {
        auto cost_at = [&](double offset) {
            auto perturbed = states;
            perturbed[1](comp) += offset;
            return TotalCost(evaluator, reference_, perturbed, controls,
                             multipliers, 0.0);
        };
        ExpectComponentClose(FivePointCentral(cost_at, 1e-5),
                             result.stages[1].lx(comp), 1e-6);
    }
    // Hessian 断言：换挡代理的 (v,a) 块做负特征值截断的 PSD 投影（门区
    // 精确二阶导不定，负曲率注入 Riccati 会破坏 GN 假设）——本用例状态
    // （v=0.08、a=−1.0、β=0.1）位于门区，精确块实测不定，投影必须激活。
    // 测试侧从 σ_β 定义独立重算精确块并施加同一投影规则，逐项对照
    const double beta = 0.1;
    const double tanh_v = std::tanh(0.08 / beta);
    const double tanh_w = std::tanh(-0.02 / beta);
    const double dsig_v = (1.0 - tanh_v * tanh_v) / (2.0 * beta);
    const double dsig_w = (1.0 - tanh_w * tanh_w) / (2.0 * beta);
    const double ddsig_v = -tanh_v * (1.0 - tanh_v * tanh_v) / (beta * beta);
    const double ddsig_w = -tanh_w * (1.0 - tanh_w * tanh_w) / (beta * beta);
    const double dp_da = 1.0 - 2.0 * 0.5 * (1.0 + tanh_w);
    const double dp_dd = 1.0 - 2.0 * 0.5 * (1.0 + tanh_v);
    const double h_vv_exact =
        2.0 * (ddsig_v * dp_da + ddsig_w * dp_dd - 4.0 * dsig_v * dsig_w);
    const double h_va_exact =
        2.0 * 0.1 * (ddsig_w * dp_dd - 2.0 * dsig_v * dsig_w);
    const double h_aa_exact = 2.0 * 0.01 * ddsig_w * dp_dd;
    const double trace = h_vv_exact + h_aa_exact;
    const double root = std::hypot(0.5 * (h_vv_exact - h_aa_exact), h_va_exact);
    const double lambda_max = 0.5 * trace + root;
    const double lambda_min = 0.5 * trace - root;
    EXPECT_LT(lambda_min, 0.0);  // 前置：精确块确实不定（投影被激活）
    EXPECT_GT(lambda_max, 0.0);
    double ux = h_va_exact;
    double uy = lambda_max - h_vv_exact;
    const double alt_x = lambda_max - h_aa_exact;
    if (alt_x * alt_x + h_va_exact * h_va_exact > ux * ux + uy * uy) {
        ux = alt_x;
        uy = h_va_exact;
    }
    const double norm = std::hypot(ux, uy);
    ux /= norm;
    uy /= norm;
    const auto& lxx = result.stages[1].lxx;
    EXPECT_NEAR(lambda_max * ux * ux, lxx(DDP_IDX_V, DDP_IDX_V), 1e-6);
    EXPECT_NEAR(lambda_max * ux * uy, lxx(DDP_IDX_V, DDP_IDX_A), 1e-6);
    EXPECT_NEAR(lambda_max * uy * uy, lxx(DDP_IDX_A, DDP_IDX_A), 1e-6);
    // PSD 性质直断：投影后最小特征值非负
    const double out_trace =
        lxx(DDP_IDX_V, DDP_IDX_V) + lxx(DDP_IDX_A, DDP_IDX_A);
    const double out_root = std::hypot(
        0.5 * (lxx(DDP_IDX_V, DDP_IDX_V) - lxx(DDP_IDX_A, DDP_IDX_A)),
        lxx(DDP_IDX_V, DDP_IDX_A));
    EXPECT_GE(0.5 * out_trace - out_root, -1e-12);
}

// 测试逐轮 β 输入通道：DdpCostInput.shift_beta > 0 时覆盖配置静态值
// （外层退火调度的落点——门宽逐轮收窄时同一状态必须给出不同代价）；
// shift_beta = 0 回退配置静态值（既有调用方行为逐位不变）；
// shift_beta < 0 或非有限必须抛 std::invalid_argument（错误调度会静默
// 污染换挡判决的数值形态）
TEST_F(DdpCostEvaluatorTest, ShiftProxyUsesPerRoundInputBeta) {
    DdpCostConfig config = MakeZeroWeightConfig();
    config.weight_shift = 2.0;
    config.shift_beta = 0.1;
    const DdpCostEvaluator evaluator(config, nullptr);
    auto states = MakeStates();
    states[1](DDP_IDX_V) = 0.08;
    states[1](DDP_IDX_A) = -1.0;
    const auto controls = MakeControls();
    const auto multipliers = DdpCostMultiplierState::MakeZero(kNumSteps);
    DdpCostInput input;
    input.tracking_weight = 0.0;
    // 默认（0 = 未指定）：回退配置静态值 0.1，与既有行为逐位一致
    const auto fallback =
        evaluator.evaluate(reference_, states, controls, multipliers, input);
    input.shift_beta = 0.1;
    const auto explicit_same =
        evaluator.evaluate(reference_, states, controls, multipliers, input);
    EXPECT_DOUBLE_EQ(fallback.stages[1].cost_shift,
                     explicit_same.stages[1].cost_shift);
    // 逐轮覆盖：β=0.3（宽门）与 β=0.1 的代价必须不同；宽门在 |v|=0.08
    // 处的门漏损更大、变号惩罚更低
    input.shift_beta = 0.3;
    const auto wide =
        evaluator.evaluate(reference_, states, controls, multipliers, input);
    EXPECT_GT(wide.stages[1].cost_shift, 0.0);
    EXPECT_NE(wide.stages[1].cost_shift, fallback.stages[1].cost_shift);
    // 非法逐轮 β 显式拒绝
    input.shift_beta = -0.1;
    EXPECT_THROW(
        evaluator.evaluate(reference_, states, controls, multipliers, input),
        std::invalid_argument);
    input.shift_beta = std::numeric_limits<double>::quiet_NaN();
    EXPECT_THROW(
        evaluator.evaluate(reference_, states, controls, multipliers, input),
        std::invalid_argument);
}

// 测试换挡代理与门控的互斥：门控计划在场（阶段二模式）时即使
// weight_shift>0，换挡代理也必须整体关闭——阶段二的换挡位置已被符号门/
// 接缝零速等式钉死，代理继续惩罚保留的换挡只会与门控等式对拉
TEST_F(DdpCostEvaluatorTest, ShiftProxySuppressedWhenGatingPlanPresent) {
    DdpCostConfig config = MakeZeroWeightConfig();
    config.weight_shift = 2.0;
    config.shift_beta = 0.1;
    const DdpCostEvaluator evaluator(config, nullptr);
    // 构造 v 在阶段 1→2 变号的状态（换挡代理门内），同时给出门控计划
    auto states = MakeStates();
    states[1](DDP_IDX_V) = 0.3;
    states[1](DDP_IDX_A) = 0.0;
    states[2](DDP_IDX_V) = -0.3;
    states[2](DDP_IDX_A) = 0.0;
    const auto controls = MakeControls();
    // 阶段一模式（无计划）：换挡代理正常生效（前置断言，证明本用例的状态
    // 确实位于代理门内，互斥关闭前后的对照有效）
    const auto stage_one_multipliers =
        DdpCostMultiplierState::MakeZero(kNumSteps);
    DdpCostInput stage_one_input;
    stage_one_input.tracking_weight = 0.0;
    const auto stage_one_eval = evaluator.evaluate(
        reference_, states, controls, stage_one_multipliers, stage_one_input);
    EXPECT_GT(stage_one_eval.stages[1].cost_shift, 0.0);
    // 阶段二模式（计划 + 门控乘子同在场）：换挡代理整体关闭
    DdpGatingPlan plan;
    plan.sign_gate = {0, 0, 0, 0};
    plan.seam_indices = {2};
    plan.seam_lookup = {-1, -1, 0, -1};
    plan.dwell_v_cap = {0.0, 0.0, 0.0, 0.0};
    auto gating_multipliers =
        DdpCostMultiplierState::MakeStageTwoZero(kNumSteps, 1);
    gating_multipliers.gating_sign_mu.setConstant(10.0);
    gating_multipliers.gating_seam_mu.setConstant(10.0);
    gating_multipliers.gating_dwell_mu.setConstant(10.0);
    DdpCostInput gating_input;
    gating_input.tracking_weight = 0.0;
    gating_input.gating_plan = &plan;
    const auto gating_eval = evaluator.evaluate(
        reference_, states, controls, gating_multipliers, gating_input);
    for (const auto& stage : gating_eval.stages) {
        EXPECT_DOUBLE_EQ(stage.cost_shift, 0.0);
    }
    // 门控项确实在场（接缝等式 μ=10、v=-0.3 → ½μv²=0.45），排除「求值
    // 根本没进门控分支」的假阳性
    EXPECT_GT(gating_eval.stages[2].cost_gating, 0.0);
}

// 测试候选待融段的差异化跟踪权重：候选掩码点的跟踪项按
// candidate_tracking_weight 取值（深退火把融化裁决权交还平滑项），
// 权重选择优先级为「豁免掩码（w_ref,0）> 候选掩码 > 普通退火权重」；
// 掩码尺寸不符必须抛 std::invalid_argument
TEST_F(DdpCostEvaluatorTest, MeltCandidateMaskOverridesTrackingWeight) {
    DdpCostConfig config = MakeZeroWeightConfig();
    config.weight_ref_base = 10.0;
    const DdpCostEvaluator evaluator(config, nullptr);
    const auto states = MakeStates();
    const auto controls = MakeControls();
    const auto multipliers = DdpCostMultiplierState::MakeZero(kNumSteps);
    DdpCostInput input;
    input.tracking_weight = 4.0;            // 普通点的退火权重
    input.candidate_tracking_weight = 0.5;  // 候选点的深退火权重
    std::vector<bool> candidate_mask(kNumSteps + 1, false);
    candidate_mask[1] = true;
    input.melt_candidate_mask = &candidate_mask;
    const auto result =
        evaluator.evaluate(reference_, states, controls, multipliers, input);
    // 阶段 1（候选点）：只含候选权重 0.5 的跟踪代价（其余权重均为零）
    const double err_x = states[1](DDP_IDX_X) - reference_.poses[1].x;
    const double err_y = states[1](DDP_IDX_Y) - reference_.poses[1].y;
    EXPECT_NEAR(0.5 * 0.5 * (err_x * err_x + err_y * err_y) * kDt,
                result.stages[1].cost_tracking, 1e-12);
    // 阶段 0/2（非候选点）：按普通退火权重 4.0
    const double err0_x = states[0](DDP_IDX_X) - reference_.poses[0].x;
    const double err0_y = states[0](DDP_IDX_Y) - reference_.poses[0].y;
    EXPECT_NEAR(0.5 * 4.0 * (err0_x * err0_x + err0_y * err0_y) * kDt,
                result.stages[0].cost_tracking, 1e-12);
    // 豁免掩码优先于候选掩码：同点同时被两者覆盖时恒用 w_ref,0
    std::vector<bool> exempt_mask(kNumSteps + 1, false);
    exempt_mask[1] = true;
    input.anneal_exempt_mask = &exempt_mask;
    const auto exempt_result =
        evaluator.evaluate(reference_, states, controls, multipliers, input);
    EXPECT_NEAR(0.5 * 10.0 * (err_x * err_x + err_y * err_y) * kDt,
                exempt_result.stages[1].cost_tracking, 1e-12);
    // 掩码尺寸不符显式拒绝
    const std::vector<bool> bad_mask(2, false);
    input.melt_candidate_mask = &bad_mask;
    EXPECT_THROW(
        evaluator.evaluate(reference_, states, controls, multipliers, input),
        std::invalid_argument);
}

// 测试 δ 奇异区护栏：|δ|≤δ_guard 时代价/梯度/Hessian 恒零（健康解行为
// 逐位不变）；|δ| 越出护栏时按 w·max(0,|δ|−δ_guard)² 手推值激活，
// C¹ 边界连续；梯度/Hessian 与有限差分对拍；护栏阈值不在 (δ_max, π/2)
// 内时构造显式拒绝
TEST_F(DdpCostEvaluatorTest, DeltaGuardActivatesOnlyBeyondThreshold) {
    DdpCostConfig config = MakeZeroWeightConfig();
    config.weight_delta_guard = 3.0;
    config.delta_guard = 0.7;
    const DdpCostEvaluator evaluator(config, nullptr);
    const auto controls = MakeControls();
    const auto multipliers = DdpCostMultiplierState::MakeZero(kNumSteps);
    // 未越界（|δ|=0.6 < 0.7）：全部为零
    auto states = MakeStates();
    states[1](DDP_IDX_DELTA) = 0.6;
    const auto inactive =
        EvaluateAll(evaluator, reference_, states, controls, multipliers, 0.0);
    EXPECT_DOUBLE_EQ(inactive.stages[1].cost_delta_guard, 0.0);
    EXPECT_DOUBLE_EQ(inactive.stages[1].lx(DDP_IDX_DELTA), 0.0);
    EXPECT_DOUBLE_EQ(inactive.stages[1].lxx(DDP_IDX_DELTA, DDP_IDX_DELTA), 0.0);
    // 越界（δ=−0.9）：excess=−0.2 方向沿 −δ；手推代价/梯度/Hessian
    states[1](DDP_IDX_DELTA) = -0.9;
    const auto active =
        EvaluateAll(evaluator, reference_, states, controls, multipliers, 0.0);
    EXPECT_NEAR(active.stages[1].cost_delta_guard, 3.0 * 0.04, 1e-12);
    EXPECT_NEAR(active.stages[1].lx(DDP_IDX_DELTA), 2.0 * 3.0 * 0.2 * (-1.0),
                1e-12);
    EXPECT_NEAR(active.stages[1].lxx(DDP_IDX_DELTA, DDP_IDX_DELTA), 6.0, 1e-12);
    // 梯度 FD 对拍
    auto cost_at = [&](double offset) {
        auto perturbed = states;
        perturbed[1](DDP_IDX_DELTA) += offset;
        return TotalCost(evaluator, reference_, perturbed, controls,
                         multipliers, 0.0);
    };
    ExpectComponentClose(FivePointCentral(cost_at, 1e-5),
                         active.stages[1].lx(DDP_IDX_DELTA), 1e-6);
    // 非法阈值显式拒绝（≤δ_max 或 ≥π/2）
    DdpCostConfig bad = MakeZeroWeightConfig();
    bad.weight_delta_guard = 1.0;
    bad.delta_guard = 0.4;  // < δ_max=0.55
    EXPECT_THROW(DdpCostEvaluator(bad, nullptr), std::invalid_argument);
    bad.delta_guard = 1.6;  // > π/2
    EXPECT_THROW(DdpCostEvaluator(bad, nullptr), std::invalid_argument);
}

// 测试曲率正则项：ℓ_κ = w·(tanδ/L)²（积分型、含 dt）——代价/梯度/精确
// Hessian 的手推值与有限差分对拍；Hessian 处处正定（天然 PSD，直接进
// Riccati 无需投影）；控制导数恒零；权重启用但轴距未注入（=0）时构造
// 显式拒绝（静默放行会让正则项恒零失效）
TEST_F(DdpCostEvaluatorTest, CurvaturePenaltyMatchesHandDerivedAndFd) {
    DdpCostConfig config = MakeZeroWeightConfig();
    config.weight_curvature = 67.0;
    config.wheelbase = 3.0;
    const DdpCostEvaluator evaluator(config, nullptr);
    auto states = MakeStates();
    states[1](DDP_IDX_DELTA) = 0.3;
    const auto controls = MakeControls();
    const auto multipliers = DdpCostMultiplierState::MakeZero(kNumSteps);
    const auto result =
        EvaluateAll(evaluator, reference_, states, controls, multipliers, 0.0);
    // 手推：tan(0.3)=0.30934、sec²(0.3)=1.09571、w·tan²/L²·dt
    const double tan_d = std::tan(0.3);
    const double sec2_d = 1.0 / (std::cos(0.3) * std::cos(0.3));
    const double inv_l2 = 1.0 / 9.0;
    EXPECT_NEAR(result.stages[1].cost_curvature,
                67.0 * tan_d * tan_d * inv_l2 * kDt, 1e-12);
    EXPECT_NEAR(result.stages[1].lx(DDP_IDX_DELTA),
                2.0 * 67.0 * tan_d * sec2_d * inv_l2 * kDt, 1e-12);
    EXPECT_NEAR(result.stages[1].lxx(DDP_IDX_DELTA, DDP_IDX_DELTA),
                2.0 * 67.0 * (sec2_d * sec2_d + 2.0 * tan_d * tan_d * sec2_d) *
                    inv_l2 * kDt,
                1e-12);
    // 纯状态项：控制导数恒零
    EXPECT_TRUE(result.stages[1].lu.isZero());
    EXPECT_TRUE(result.stages[1].luu.isZero());
    EXPECT_TRUE(result.stages[1].lux.isZero());
    // 梯度 FD 对拍（总代价五点差分）
    auto cost_at = [&](double offset) {
        auto perturbed = states;
        perturbed[1](DDP_IDX_DELTA) += offset;
        return TotalCost(evaluator, reference_, perturbed, controls,
                         multipliers, 0.0);
    };
    ExpectComponentClose(FivePointCentral(cost_at, 1e-5),
                         result.stages[1].lx(DDP_IDX_DELTA), 1e-6);
    // Hessian FD 对拍（解析梯度中心差分；精确二阶可直接对拍）
    auto grad_at = [&](double offset) {
        auto perturbed = states;
        perturbed[1](DDP_IDX_DELTA) += offset;
        return EvaluateAll(evaluator, reference_, perturbed, controls,
                           multipliers, 0.0)
            .stages[1]
            .lx(DDP_IDX_DELTA);
    };
    EXPECT_NEAR((grad_at(1e-5) - grad_at(-1e-5)) / 2e-5,
                result.stages[1].lxx(DDP_IDX_DELTA, DDP_IDX_DELTA), 1e-6);
    // δ=0 处代价/梯度为零、Hessian 为 2w/L²·dt > 0（全轨迹最小曲率点
    // 仍有正曲率——正则项在原点附近是凸碗）
    for (auto& state : states) {
        state(DDP_IDX_DELTA) = 0.0;
    }
    const auto at_zero =
        EvaluateAll(evaluator, reference_, states, controls, multipliers, 0.0);
    EXPECT_DOUBLE_EQ(at_zero.stages[1].cost_curvature, 0.0);
    EXPECT_DOUBLE_EQ(at_zero.stages[1].lx(DDP_IDX_DELTA), 0.0);
    EXPECT_NEAR(at_zero.stages[1].lxx(DDP_IDX_DELTA, DDP_IDX_DELTA),
                2.0 * 67.0 / 9.0 * kDt, 1e-12);
    // 启用但轴距未注入：构造显式拒绝
    DdpCostConfig bad = MakeZeroWeightConfig();
    bad.weight_curvature = 10.0;
    bad.wheelbase = 0.0;
    EXPECT_THROW(DdpCostEvaluator(bad, nullptr), std::invalid_argument);
}

// 测试弧长惩罚项：ℓ_v = w_v·v²（积分型、含 dt）——固定 dt 网格下总时长
// 是常数，本项是弧长的同向凸代理、也是代价函数中唯一反对轨迹跑飞
// （绕大圈）的项；手推代价/梯度/Hessian（2w_v>0 天然 PSD）与 FD 对拍；
// 控制导数恒零；v=0 处代价/梯度为零、凸碗 Hessian 2w_v·dt
TEST_F(DdpCostEvaluatorTest, VelocityPenaltyMatchesHandDerivedAndFd) {
    DdpCostConfig config = MakeZeroWeightConfig();
    config.weight_velocity = 0.45;
    const DdpCostEvaluator evaluator(config, nullptr);
    auto states = MakeStates();
    states[1](DDP_IDX_V) = 0.6;
    const auto controls = MakeControls();
    const auto multipliers = DdpCostMultiplierState::MakeZero(kNumSteps);
    const auto result =
        EvaluateAll(evaluator, reference_, states, controls, multipliers, 0.0);
    EXPECT_NEAR(result.stages[1].cost_velocity, 0.45 * 0.36 * kDt, 1e-12);
    EXPECT_NEAR(result.stages[1].lx(DDP_IDX_V), 2.0 * 0.45 * 0.6 * kDt, 1e-12);
    EXPECT_NEAR(result.stages[1].lxx(DDP_IDX_V, DDP_IDX_V), 2.0 * 0.45 * kDt,
                1e-12);
    // 纯状态项：控制导数恒零
    EXPECT_TRUE(result.stages[1].lu.isZero());
    EXPECT_TRUE(result.stages[1].luu.isZero());
    EXPECT_TRUE(result.stages[1].lux.isZero());
    // 梯度 FD 对拍
    auto cost_at = [&](double offset) {
        auto perturbed = states;
        perturbed[1](DDP_IDX_V) += offset;
        return TotalCost(evaluator, reference_, perturbed, controls,
                         multipliers, 0.0);
    };
    ExpectComponentClose(FivePointCentral(cost_at, 1e-5),
                         result.stages[1].lx(DDP_IDX_V), 1e-6);
    // v=0 处凸碗：代价/梯度为零、Hessian 2w_v·dt > 0
    for (auto& state : states) {
        state(DDP_IDX_V) = 0.0;
    }
    const auto at_zero =
        EvaluateAll(evaluator, reference_, states, controls, multipliers, 0.0);
    EXPECT_DOUBLE_EQ(at_zero.stages[1].cost_velocity, 0.0);
    EXPECT_DOUBLE_EQ(at_zero.stages[1].lx(DDP_IDX_V), 0.0);
    EXPECT_NEAR(at_zero.stages[1].lxx(DDP_IDX_V, DDP_IDX_V), 0.9 * kDt, 1e-12);
    // 非法权重显式拒绝
    DdpCostConfig bad = MakeZeroWeightConfig();
    bad.weight_velocity = -1.0;
    EXPECT_THROW(DdpCostEvaluator(bad, nullptr), std::invalid_argument);
}

// 测试终点 AL 等式：c=[x−xg, y−yg, wrap(θ−θg), v, a] 的 λᵀc+½μc² 手推值、
// 梯度 λ+μ∘c、GN Hessian diag(μ)（选择矩阵常量，GN 即精确）；
// δ_N/ω_N 不承载终点约束，对应导数行/列必须恒为零。
TEST_F(DdpCostEvaluatorTest, TerminalAlEqualityMatchesHandDerived) {
    const DdpCostEvaluator evaluator(MakeZeroWeightConfig(), nullptr);
    const auto states = MakeStates();
    const auto controls = MakeControls();
    auto multipliers = DdpCostMultiplierState::MakeZero(kNumSteps);
    multipliers.terminal_lambda << 0.3, -0.2, 0.5, 0.7, -0.4;
    multipliers.terminal_mu << 10.0, 20.0, 30.0, 40.0, 50.0;
    const auto result =
        EvaluateAll(evaluator, reference_, states, controls, multipliers, 0.0);
    const auto& terminal = result.stages[kNumSteps];
    // 终点残差：c=[0.01, 0.005, −0.01, 0.05, 0.05]
    const Eigen::Matrix<double, 5, 1> c =
        (Eigen::Matrix<double, 5, 1>() << 0.01, 0.005, -0.01, 0.05, 0.05)
            .finished();
    EXPECT_NEAR(multipliers.terminal_lambda.dot(c) +
                    0.5 * multipliers.terminal_mu.cwiseProduct(c).dot(c),
                terminal.cost_terminal, 1e-12);
    EXPECT_NEAR(0.3 + 10.0 * 0.01, terminal.lx(DDP_IDX_X), 1e-12);
    EXPECT_NEAR(-0.2 + 20.0 * 0.005, terminal.lx(DDP_IDX_Y), 1e-12);
    EXPECT_NEAR(0.5 + 30.0 * (-0.01), terminal.lx(DDP_IDX_THETA), 1e-12);
    EXPECT_NEAR(0.7 + 40.0 * 0.05, terminal.lx(DDP_IDX_V), 1e-12);
    EXPECT_NEAR(-0.4 + 50.0 * 0.05, terminal.lx(DDP_IDX_A), 1e-12);
    EXPECT_NEAR(10.0, terminal.lxx(DDP_IDX_X, DDP_IDX_X), 1e-12);
    EXPECT_NEAR(30.0, terminal.lxx(DDP_IDX_THETA, DDP_IDX_THETA), 1e-12);
    EXPECT_NEAR(50.0, terminal.lxx(DDP_IDX_A, DDP_IDX_A), 1e-12);
    // δ/ω 方向导数恒零（停稳后前轮转角无物理要求）
    for (const int comp : {DDP_IDX_DELTA, DDP_IDX_OMEGA}) {
        EXPECT_DOUBLE_EQ(terminal.lx(comp), 0.0);
        EXPECT_TRUE(terminal.lxx.row(comp).isZero());
        EXPECT_TRUE(terminal.lxx.col(comp).isZero());
    }
    // 梯度对拍（总代价五点差分）
    for (int comp = 0; comp < DDP_STATE_DIM; ++comp) {
        auto cost_at = [&](double offset) {
            auto perturbed = states;
            perturbed[kNumSteps](comp) += offset;
            return TotalCost(evaluator, reference_, perturbed, controls,
                             multipliers, 0.0);
        };
        ExpectComponentClose(FivePointCentral(cost_at, 1e-4), terminal.lx(comp),
                             1e-6);
    }
    // Hessian 对拍（二次型精确）：解析梯度中心差分
    for (int comp = 0; comp < DDP_STATE_DIM; ++comp) {
        auto grad_at = [&](double offset) {
            auto perturbed = states;
            perturbed[kNumSteps](comp) += offset;
            return EvaluateAll(evaluator, reference_, perturbed, controls,
                               multipliers, 0.0)
                .stages[kNumSteps]
                .lx;
        };
        const DdpState numeric_col = (grad_at(1e-4) - grad_at(-1e-4)) / 2e-4;
        ExpectMatrixClose(numeric_col, terminal.lxx.col(comp), 1e-6);
    }
}

// 测试总代价与逐阶段分解的一致性：total_cost 必须等于各阶段
// totalCost() 的累加（分解值是外层调度/日志的唯一数据源，不允许漂移）。
TEST_F(DdpCostEvaluatorTest, TotalCostEqualsStageSum) {
    DdpCostConfig config;
    config.weight_jerk = 1.0;
    config.weight_steer_accel = 2.0;
    config.weight_ref_base = 10.0;
    config.weight_theta = 5.0;
    const DdpCostEvaluator evaluator(config, nullptr);
    const auto states = MakeStates();
    const auto controls = MakeControls();
    auto multipliers = DdpCostMultiplierState::MakeZero(kNumSteps);
    multipliers.amplitude_mu.setConstant(3.0);
    multipliers.terminal_mu.setConstant(100.0);
    const auto result =
        EvaluateAll(evaluator, reference_, states, controls, multipliers, 4.0);
    double sum = 0.0;
    for (const auto& stage : result.stages) {
        sum += stage.totalCost();
    }
    EXPECT_DOUBLE_EQ(sum, result.total_cost);
    EXPECT_GT(result.total_cost, 0.0);
}

// 测试 ESDF 惩罚的时间轴抽样集成：stride=2 时仅偶数阶段携带 ESDF 代价，
// 未抽样阶段的 ESDF 分量严格为零；终端阶段恒评估（终点避障不抽样）。
// 同时验证 ESDF 导数确实叠加进了阶段 lx/lxx。
TEST_F(DdpCostEvaluatorTest, EsdfPenaltyAppliesOnSampledStagesOnly) {
    // 第 0 列整列占据的墙场（64×64、分辨率 0.125）
    std::vector<Position> cells;
    cells.reserve(64);
    for (int row = 0; row < 64; ++row) {
        cells.emplace_back(Position{0.0, row * 0.125});
    }
    const GridMap grid_map(0.125, 64, 64, Position{0.0, 0.0}, cells);
    const ESDFMap esdf_map(grid_map);
    const VehicleParams veh_params(4.3, 1.8, 2.7, 0.6, 0.8);
    const VehicleFootprintModel footprint_model(veh_params, 233, 2, 1);
    DdpEsdfConstraintConfig esdf_config;
    esdf_config.stride = 2;
    const DdpEsdfConstraint esdf_constraint(esdf_map, footprint_model,
                                            esdf_config);
    const DdpCostEvaluator with_esdf(MakeZeroWeightConfig(), &esdf_constraint);
    const DdpCostEvaluator without_esdf(MakeZeroWeightConfig(), nullptr);
    // 全部状态平行贴墙（圆心 cx=0.5 < r+margin_comf，惩罚激活）
    auto states = MakeStates();
    for (auto& state : states) {
        state(DDP_IDX_X) = 0.5;
        state(DDP_IDX_Y) = 4.0;
        state(DDP_IDX_THETA) = kHalfPi;
    }
    const auto controls = MakeControls();
    const auto multipliers = DdpCostMultiplierState::MakeZero(kNumSteps);
    const auto sampled =
        EvaluateAll(with_esdf, reference_, states, controls, multipliers, 0.0);
    const auto unsampled = EvaluateAll(without_esdf, reference_, states,
                                       controls, multipliers, 0.0);
    // 抽样命中阶段（0、2 与恒评估的终端 3）：ESDF 代价为正且导数非零；
    // 未抽样阶段（1）：ESDF 分量严格为零，与无 ESDF 求值逐位一致
    for (const std::size_t k : {0U, 2U, 3U}) {
        EXPECT_GT(sampled.stages[k].cost_esdf, 0.0) << "stage " << k;
        EXPECT_FALSE(sampled.stages[k].lx.isZero()) << "stage " << k;
        EXPECT_GT(sampled.stages[k].lxx(DDP_IDX_X, DDP_IDX_X), 0.0)
            << "stage " << k;
    }
    EXPECT_DOUBLE_EQ(sampled.stages[1].cost_esdf, 0.0);
    EXPECT_DOUBLE_EQ(sampled.stages[1].totalCost(), 0.0);
    EXPECT_TRUE(sampled.stages[1].lx.isApprox(unsampled.stages[1].lx));
    // 总代价 = Σ ESDF 阶段代价（其余权重全零、乘子全零）
    double esdf_sum = 0.0;
    for (const auto& stage : sampled.stages) {
        esdf_sum += stage.cost_esdf;
    }
    EXPECT_DOUBLE_EQ(esdf_sum, sampled.total_cost);
    EXPECT_DOUBLE_EQ(unsampled.total_cost, 0.0);
}

// 测试 dt 因子约定（回归防火墙）：平滑/跟踪项（积分型代价）随 dt 线性
// 缩放，ESDF 惩罚/幅值 AL/终点 AL（逐阶段点态量）不随 dt 变化。
// 若未来有人误对所有项统一乘 dt（或反向去掉既有 dt 因子），本用例会
// 立即失败——这是回推侧"不得统一乘 dt"约定在求值层的锚点。
TEST_F(DdpCostEvaluatorTest, CostDtScalingConventionIsPinned) {
    // 第 0 列整列占据的墙场（64×64、分辨率 0.125）
    std::vector<Position> cells;
    cells.reserve(64);
    for (int row = 0; row < 64; ++row) {
        cells.emplace_back(Position{0.0, row * 0.125});
    }
    const GridMap grid_map(0.125, 64, 64, Position{0.0, 0.0}, cells);
    const ESDFMap esdf_map(grid_map);
    const VehicleParams veh_params(4.3, 1.8, 2.7, 0.6, 0.8);
    const VehicleFootprintModel footprint_model(veh_params, 233, 2, 1);
    const DdpEsdfConstraint esdf_constraint(esdf_map, footprint_model);
    DdpCostConfig config;
    config.weight_jerk = 1.0;
    config.weight_steer_accel = 2.0;
    config.weight_ref_base = 10.0;
    config.weight_theta = 5.0;
    const DdpCostEvaluator evaluator(config, &esdf_constraint);
    // 全部状态平行贴墙（ESDF 激活），stage 0 速度越界（幅值 AL 激活）
    auto states = MakeStates();
    for (auto& state : states) {
        state(DDP_IDX_X) = 0.5;
        state(DDP_IDX_Y) = 4.0;
        state(DDP_IDX_THETA) = kHalfPi;
    }
    states[0](DDP_IDX_V) = 1.6;
    const auto controls = MakeControls();
    auto multipliers = DdpCostMultiplierState::MakeZero(kNumSteps);
    multipliers.amplitude_mu(DDP_AMP_V) = 7.0;
    multipliers.terminal_mu.setConstant(100.0);
    // 位姿相同、dt 不同（0.1 vs 0.05）的两条参考
    auto ref_fine = reference_;
    ref_fine.dt = 0.05;
    const auto coarse =
        EvaluateAll(evaluator, reference_, states, controls, multipliers, 10.0);
    const auto fine =
        EvaluateAll(evaluator, ref_fine, states, controls, multipliers, 10.0);
    for (std::size_t k = 0; k < kNumSteps; ++k) {
        // 积分型项严格 2 倍（dt 字面量 0.1 == 2*0.05 在 IEEE 下精确成立）
        EXPECT_DOUBLE_EQ(coarse.stages[k].cost_smooth,
                         2.0 * fine.stages[k].cost_smooth)
            << "stage " << k;
        EXPECT_DOUBLE_EQ(coarse.stages[k].cost_tracking,
                         2.0 * fine.stages[k].cost_tracking)
            << "stage " << k;
        // 点态项与 dt 无关，逐位一致
        EXPECT_DOUBLE_EQ(coarse.stages[k].cost_esdf, fine.stages[k].cost_esdf)
            << "stage " << k;
        EXPECT_DOUBLE_EQ(coarse.stages[k].cost_amplitude,
                         fine.stages[k].cost_amplitude)
            << "stage " << k;
    }
    // 负对照：场景确实处于 ESDF/幅值激活区，上述"不变"断言具备判别力
    EXPECT_GT(coarse.stages[0].cost_esdf, 0.0);
    EXPECT_GT(coarse.stages[0].cost_amplitude, 0.0);
    EXPECT_GT(coarse.stages[0].cost_smooth, 0.0);
    EXPECT_DOUBLE_EQ(coarse.stages[kNumSteps].cost_terminal,
                     fine.stages[kNumSteps].cost_terminal);
}

}  // namespace
}  // namespace apa_post_processor
