#include <gtest/gtest.h>

#include <Eigen/Eigenvalues>
#include <cmath>
#include <limits>
#include <vector>

#include "core/iLQR/ilqr_cost.h"
#include "core/iLQR/esdf_constraint.h"
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

// 矩阵/向量逐元素位级相等断言（早停筛选路径与全量路径对拍用）
template <typename TDerivedA, typename TDerivedB>
void ExpectBitwiseEqual(const Eigen::MatrixBase<TDerivedA>& expected,
                        const Eigen::MatrixBase<TDerivedB>& actual) {
    ASSERT_EQ(expected.rows(), actual.rows());
    ASSERT_EQ(expected.cols(), actual.cols());
    EXPECT_TRUE((expected.array() == actual.array()).all());
}

// 五点中心差分 [-f(x+2h)+8f(x+h)-8f(x-h)+f(x-2h)]/(12h)，截断误差 O(h^4)
template <typename TFunc>
double FivePointCentral(TFunc&& func, double h) {
    return (-func(2.0 * h) + 8.0 * func(h) - 8.0 * func(-h) + func(-2.0 * h)) /
           (12.0 * h);
}

// 公共测试夹具：N=3 的小型参考轨迹（4 个位姿），状态/控制全部在幅值边界内
// 且与参考位姿有可控偏差，保证平滑/跟踪/终点项都有非零信号
class iLQRCostEvaluatorTest : public ::testing::Test {
   protected:
    iLQRCostEvaluatorTest() {
        reference_.ds = 0.05;
        reference_.dt = kDt;
        reference_.poses = {Pose{0.0, 0.0, 0.0}, Pose{0.05, 0.0, 0.0},
                            Pose{0.10, 0.01, 0.10}, Pose{0.15, 0.03, 0.15}};
    }

    // 按状态分量布局 [x, y, θ, v, a, δ, ω] 构造一个七维状态
    static iLQRState MakeState(double x, double y, double theta, double v,
                              double a, double delta, double omega) {
        iLQRState state;
        state << x, y, theta, v, a, delta, omega;
        return state;
    }

    // 全部状态在幅值边界内（λ=0 时幅值 AL 不激活）
    static iLQRAlignedVec<iLQRState> MakeStates() {
        iLQRAlignedVec<iLQRState> states;
        states.reserve(kNumSteps + 1);
        states.push_back(MakeState(0.00, 0.000, 0.00, 0.50, 0.20, 0.10, 0.05));
        states.push_back(
            MakeState(0.06, 0.005, 0.02, 0.60, -0.10, 0.12, -0.08));
        states.push_back(MakeState(0.11, 0.015, 0.08, 0.40, 0.15, 0.08, 0.10));
        states.push_back(MakeState(0.16, 0.035, 0.14, 0.05, 0.05, 0.20, 0.00));
        return states;
    }

    static iLQRAlignedVec<iLQRControl> MakeControls() {
        iLQRAlignedVec<iLQRControl> controls;
        controls.reserve(kNumSteps);
        controls.push_back(MakeControl(0.30, 0.20));
        controls.push_back(MakeControl(-0.20, 0.10));
        controls.push_back(MakeControl(0.10, -0.15));
        return controls;
    }

    static iLQRControl MakeControl(double jerk, double eta) {
        iLQRControl control;
        control << jerk, eta;
        return control;
    }

    // 便捷求值：reference 显式传入，便于个别用例改写参考位姿
    static iLQRCostEvaluation EvaluateAll(
        const iLQRCostEvaluator& evaluator, const iLQRReference& reference,
        const iLQRAlignedVec<iLQRState>& states,
        const iLQRAlignedVec<iLQRControl>& controls,
        const iLQRCostMultiplierState& multipliers, double tracking_weight,
        const std::vector<bool>* mask = nullptr) {
        iLQRCostInput input;
        input.tracking_weight = tracking_weight;
        input.anneal_exempt_mask = mask;
        return evaluator.evaluate(reference, states, controls, multipliers,
                                  input);
    }

    // 全轨迹总代价（供有限差分）：扰动由调用方先作用于 states/controls 副本
    static double TotalCost(const iLQRCostEvaluator& evaluator,
                            const iLQRReference& reference,
                            const iLQRAlignedVec<iLQRState>& states,
                            const iLQRAlignedVec<iLQRControl>& controls,
                            const iLQRCostMultiplierState& multipliers,
                            double tracking_weight) {
        return EvaluateAll(evaluator, reference, states, controls, multipliers,
                           tracking_weight)
            .total_cost;
    }

    // 全零权重的配置（隔离出单一代价通道做精确手推校验）
    static iLQRCostConfig MakeZeroWeightConfig() {
        iLQRCostConfig config;
        config.weight_jerk = 0.0;
        config.weight_steer_accel = 0.0;
        config.weight_ref_base = 0.0;
        config.weight_theta = 0.0;
        return config;
    }

    static constexpr std::size_t kNumSteps = 3;
    static constexpr double kDt = 0.1;
    iLQRReference reference_;
};

// 测试构造期配置校验：负权重、非有限权重、非正幅值边界
// 均必须抛 std::invalid_argument。因为非法配置会静默污染全部下游求解。
TEST_F(iLQRCostEvaluatorTest, ConstructorThrowsOnInvalidConfig) {
    iLQRCostConfig config;
    config.weight_jerk = -1.0;
    EXPECT_THROW(iLQRCostEvaluator(config, nullptr), std::invalid_argument);
    config.weight_jerk = 1.0;
    config.weight_theta = std::numeric_limits<double>::quiet_NaN();
    EXPECT_THROW(iLQRCostEvaluator(config, nullptr), std::invalid_argument);
    config.weight_theta = 5.0;
    config.v_max = 0.0;
    EXPECT_THROW(iLQRCostEvaluator(config, nullptr), std::invalid_argument);
    config.v_max = 1.5;
    config.delta_max = -0.5;
    EXPECT_THROW(iLQRCostEvaluator(config, nullptr), std::invalid_argument);
}

// 测试求值入参维度校验：状态数≠N+1、控制数≠N、幅值乘子尺寸≠5N、
// 豁免掩码尺寸≠N+1、参考位姿不足两个、dt 非正、退火权重非有限，
// 均必须抛 std::invalid_argument——维度错配若静默放行会把导数写串位。
TEST_F(iLQRCostEvaluatorTest, EvaluateThrowsOnDimensionMismatch) {
    const iLQRCostEvaluator evaluator(iLQRCostConfig{}, nullptr);
    const auto states = MakeStates();
    const auto controls = MakeControls();
    const auto multipliers = iLQRCostMultiplierState::MakeZero(kNumSteps);
    iLQRCostInput input;
    auto bad_states = states;
    bad_states.pop_back();
    EXPECT_THROW(evaluator.evaluate(reference_, bad_states, controls,
                                    multipliers, input),
                 std::invalid_argument);
    auto bad_controls = controls;
    bad_controls.push_back(iLQRControl::Zero());
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
    iLQRReference short_ref;
    short_ref.dt = kDt;
    short_ref.poses = {Pose{0.0, 0.0, 0.0}};
    EXPECT_THROW(
        evaluator.evaluate(short_ref, states, controls, multipliers, input),
        std::invalid_argument);
    iLQRReference zero_dt_ref = reference_;
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
// 幅值 AL 未激活、ESDF 缺省时全部代价项都是精确二次型，
// GN 形 Hessian 与精确 Hessian 一致：梯度用"总代价五点差分"对拍
// （代价按阶段解耦，∂total/∂x_k 即阶段 k 的 lx），Hessian 用"解析梯度
// 再做中心差分"对拍，同时覆盖 lx/lu/lxx/luu/lux 全部五个输出。
TEST_F(iLQRCostEvaluatorTest, QuadraticTermsMatchFiniteDifference) {
    iLQRCostConfig config;
    config.weight_jerk = 1.0;
    config.weight_steer_accel = 2.0;
    config.weight_ref_base = 10.0;
    config.weight_theta = 5.0;
    const iLQRCostEvaluator evaluator(config, nullptr);
    const auto states = MakeStates();
    const auto controls = MakeControls();
    auto multipliers = iLQRCostMultiplierState::MakeZero(kNumSteps);
    multipliers.amplitude_mu.setConstant(3.0);
    multipliers.terminal_lambda << 0.3, -0.2, 0.5, 0.7, -0.4;
    multipliers.terminal_mu << 10.0, 20.0, 30.0, 40.0, 50.0;
    const double tracking_weight = 4.0;
    const auto result = EvaluateAll(evaluator, reference_, states, controls,
                                    multipliers, tracking_weight);
    for (std::size_t k = 0; k <= kNumSteps; ++k) {
        for (int comp = 0; comp < ILQR_STATE_DIM; ++comp) {
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
        for (int comp = 0; comp < ILQR_CONTROL_DIM; ++comp) {
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
        for (int comp = 0; comp < ILQR_STATE_DIM; ++comp) {
            auto grad_at = [&](double offset) {
                auto perturbed = states;
                perturbed[k](comp) += offset;
                return EvaluateAll(evaluator, reference_, perturbed, controls,
                                   multipliers, tracking_weight)
                    .stages[k]
                    .lx;
            };
            const iLQRState numeric_col =
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
            const iLQRControl numeric_mixed_col =
                (ctrl_grad_at(1e-4) - ctrl_grad_at(-1e-4)) / 2e-4;
            ExpectMatrixClose(numeric_mixed_col, result.stages[k].lux.col(comp),
                              1e-6);
        }
    }
    for (std::size_t k = 0; k < kNumSteps; ++k) {
        for (int comp = 0; comp < ILQR_CONTROL_DIM; ++comp) {
            auto grad_at = [&](double offset) {
                auto perturbed = controls;
                perturbed[k](comp) += offset;
                return EvaluateAll(evaluator, reference_, states, perturbed,
                                   multipliers, tracking_weight)
                    .stages[k]
                    .lu;
            };
            const iLQRControl numeric_col =
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
TEST_F(iLQRCostEvaluatorTest, AmplitudeGatingZeroWhenInactive) {
    const iLQRCostEvaluator evaluator(MakeZeroWeightConfig(), nullptr);
    const auto states = MakeStates();
    const auto controls = MakeControls();
    auto multipliers = iLQRCostMultiplierState::MakeZero(kNumSteps);
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
    boundary_states[1](ILQR_IDX_V) = 1.5;
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
TEST_F(iLQRCostEvaluatorTest, AmplitudeViolatedConstraintMatchesHandDerived) {
    const iLQRCostEvaluator evaluator(MakeZeroWeightConfig(), nullptr);
    auto states = MakeStates();
    states[1](ILQR_IDX_V) = 1.6;
    const auto controls = MakeControls();
    auto multipliers = iLQRCostMultiplierState::MakeZero(kNumSteps);
    const int row = ILQR_AMPLITUDE_CONSTRAINT_DIM * 1 + ILQR_AMP_V;
    multipliers.amplitude_lambda(row) = 0.3;
    multipliers.amplitude_mu(row) = 10.0;
    const auto result =
        EvaluateAll(evaluator, reference_, states, controls, multipliers, 0.0);
    const double g = 1.6 * 1.6 - 1.5 * 1.5;
    EXPECT_NEAR(0.3 * g + 0.5 * 10.0 * g * g, result.stages[1].cost_amplitude,
                1e-12);
    EXPECT_NEAR((0.3 + 10.0 * g) * 3.2, result.stages[1].lx(ILQR_IDX_V), 1e-12);
    EXPECT_NEAR(10.0 * 3.2 * 3.2, result.stages[1].lxx(ILQR_IDX_V, ILQR_IDX_V),
                1e-9);
    // 未挂乘子的阶段/分量恒零
    EXPECT_DOUBLE_EQ(result.stages[0].totalCost(), 0.0);
    EXPECT_DOUBLE_EQ(result.stages[2].totalCost(), 0.0);
    EXPECT_DOUBLE_EQ(result.stages[1].lx(ILQR_IDX_A), 0.0);
    EXPECT_DOUBLE_EQ(result.stages[1].lxx(ILQR_IDX_A, ILQR_IDX_A), 0.0);
    // (ii) 解析梯度的数值差分 = 精确 Hessian 2(λ+μg)+μ(2v)² = 109.2
    auto grad_at = [&](double offset) {
        auto perturbed = states;
        perturbed[1](ILQR_IDX_V) += offset;
        return EvaluateAll(evaluator, reference_, perturbed, controls,
                           multipliers, 0.0)
            .stages[1]
            .lx(ILQR_IDX_V);
    };
    const double exact_hessian = (grad_at(1e-4) - grad_at(-1e-4)) / 2e-4;
    ExpectComponentClose(2.0 * (0.3 + 10.0 * g) + 10.0 * 3.2 * 3.2,
                         exact_hessian, 1e-6);
    EXPECT_GT(
        std::abs(exact_hessian - result.stages[1].lxx(ILQR_IDX_V, ILQR_IDX_V)),
        1.0);
    // (i) GN 装配：约束雅可比数值差分 ∂g/∂v → μ·(∂g/∂v)²
    auto constraint_at = [&](double offset) {
        const double v = 1.6 + offset;
        return v * v - 1.5 * 1.5;
    };
    const double dg_numeric = FivePointCentral(constraint_at, 1e-4);
    ExpectComponentClose(10.0 * dg_numeric * dg_numeric,
                         result.stages[1].lxx(ILQR_IDX_V, ILQR_IDX_V), 1e-6);
    // 梯度对拍（解析梯度精确）：总代价数值差分
    auto cost_at = [&](double offset) {
        auto perturbed = states;
        perturbed[1](ILQR_IDX_V) += offset;
        return TotalCost(evaluator, reference_, perturbed, controls,
                         multipliers, 0.0);
    };
    ExpectComponentClose(FivePointCentral(cost_at, 1e-4),
                         result.stages[1].lx(ILQR_IDX_V), 1e-6);
}

// 测试幅值 AL 的乘子记忆效应：g<0 但 λ>0 时约束仍激活（乘子"记住"了
// 曾经的违反），代价/梯度按 λg+½μg² 与 (λ+μg)·∂g 闭式取值。
TEST_F(iLQRCostEvaluatorTest, AmplitudeMultiplierMemoryActivatesBelowBound) {
    const iLQRCostEvaluator evaluator(MakeZeroWeightConfig(), nullptr);
    auto states = MakeStates();
    states[1](ILQR_IDX_V) = 1.4;
    const auto controls = MakeControls();
    auto multipliers = iLQRCostMultiplierState::MakeZero(kNumSteps);
    const int row = ILQR_AMPLITUDE_CONSTRAINT_DIM * 1 + ILQR_AMP_V;
    multipliers.amplitude_lambda(row) = 0.3;
    multipliers.amplitude_mu(row) = 10.0;
    const auto result =
        EvaluateAll(evaluator, reference_, states, controls, multipliers, 0.0);
    const double g = 1.4 * 1.4 - 1.5 * 1.5;
    EXPECT_LT(g, 0.0);
    EXPECT_NEAR(0.3 * g + 0.5 * 10.0 * g * g, result.stages[1].cost_amplitude,
                1e-12);
    EXPECT_NEAR((0.3 + 10.0 * g) * 2.8, result.stages[1].lx(ILQR_IDX_V), 1e-12);
    EXPECT_NEAR(10.0 * 2.8 * 2.8, result.stages[1].lxx(ILQR_IDX_V, ILQR_IDX_V),
                1e-9);
}

// 测试 δ 双侧线性幅值约束：δ−δ_max≤0 与 −δ−δ_max≤0 的梯度恒为 ±1，
// GN Hessian 与精确 Hessian 一致（线性约束无二阶项），
// 且两侧约束共用同一个状态行不会互相串扰。
TEST_F(iLQRCostEvaluatorTest, DeltaLinearConstraintsShareDeltaRow) {
    const iLQRCostEvaluator evaluator(MakeZeroWeightConfig(), nullptr);
    const auto controls = MakeControls();
    // δ 正向越界：仅 δ−δ_max 分支激活
    auto states = MakeStates();
    states[0](ILQR_IDX_DELTA) = 0.6;
    auto multipliers = iLQRCostMultiplierState::MakeZero(kNumSteps);
    multipliers.amplitude_lambda(ILQR_AMP_DELTA_POS) = 0.1;
    multipliers.amplitude_mu(ILQR_AMP_DELTA_POS) = 7.0;
    const auto pos_result =
        EvaluateAll(evaluator, reference_, states, controls, multipliers, 0.0);
    EXPECT_NEAR(0.1 * 0.05 + 0.5 * 7.0 * 0.0025,
                pos_result.stages[0].cost_amplitude, 1e-12);
    EXPECT_NEAR(0.1 + 7.0 * 0.05, pos_result.stages[0].lx(ILQR_IDX_DELTA),
                1e-12);
    auto pos_grad_at = [&](double offset) {
        auto perturbed = states;
        perturbed[0](ILQR_IDX_DELTA) += offset;
        return EvaluateAll(evaluator, reference_, perturbed, controls,
                           multipliers, 0.0)
            .stages[0]
            .lx(ILQR_IDX_DELTA);
    };
    ExpectComponentClose((pos_grad_at(1e-4) - pos_grad_at(-1e-4)) / 2e-4,
                         pos_result.stages[0].lxx(ILQR_IDX_DELTA, ILQR_IDX_DELTA),
                         1e-6);
    // δ 负向越界：仅 −δ−δ_max 分支激活，梯度符号翻转
    states[0](ILQR_IDX_DELTA) = -0.6;
    multipliers.amplitude_lambda(ILQR_AMP_DELTA_POS) = 0.0;
    multipliers.amplitude_mu(ILQR_AMP_DELTA_POS) = 0.0;
    multipliers.amplitude_lambda(ILQR_AMP_DELTA_NEG) = 0.2;
    multipliers.amplitude_mu(ILQR_AMP_DELTA_NEG) = 11.0;
    const auto neg_result =
        EvaluateAll(evaluator, reference_, states, controls, multipliers, 0.0);
    EXPECT_NEAR(0.2 * 0.05 + 0.5 * 11.0 * 0.0025,
                neg_result.stages[0].cost_amplitude, 1e-12);
    EXPECT_NEAR(-(0.2 + 11.0 * 0.05), neg_result.stages[0].lx(ILQR_IDX_DELTA),
                1e-12);
    EXPECT_NEAR(11.0, neg_result.stages[0].lxx(ILQR_IDX_DELTA, ILQR_IDX_DELTA),
                1e-12);
}

// 测试跟踪项角度 wrap：参考角贴近 +π、状态角滑过 ±π 分界线时，
// 代价与梯度必须连续无 2π 跳变（wrap 实现与初值提取/终端约束同源）。
// 若 wrap 缺失，θ=−π+0.05 处的误差会被算成 ≈2π−0.051，代价差 5 个数量级。
TEST_F(iLQRCostEvaluatorTest, TrackingAngleWrapAcrossPiSeam) {
    iLQRCostConfig config = MakeZeroWeightConfig();
    config.weight_theta = 8.0;
    const iLQRCostEvaluator evaluator(config, nullptr);
    auto reference = reference_;
    reference.poses[1] = Pose{0.05, 0.0, PI - 1e-3};
    const auto controls = MakeControls();
    const auto multipliers = iLQRCostMultiplierState::MakeZero(kNumSteps);
    // 分界线同侧（θ=π−0.05）：误差 −0.049
    auto states = MakeStates();
    states[1](ILQR_IDX_THETA) = PI - 0.05;
    const auto before =
        EvaluateAll(evaluator, reference, states, controls, multipliers, 0.0);
    EXPECT_NEAR(0.5 * 8.0 * 0.049 * 0.049 * kDt, before.stages[1].cost_tracking,
                1e-12);
    EXPECT_NEAR(8.0 * (-0.049) * kDt, before.stages[1].lx(ILQR_IDX_THETA),
                1e-12);
    // 跨过分界线（θ=−π+0.05）：wrap 后误差 +0.051 而非 −(2π−0.051)
    states[1](ILQR_IDX_THETA) = -PI + 0.05;
    const auto after =
        EvaluateAll(evaluator, reference, states, controls, multipliers, 0.0);
    EXPECT_NEAR(0.5 * 8.0 * 0.051 * 0.051 * kDt, after.stages[1].cost_tracking,
                1e-12);
    EXPECT_NEAR(8.0 * 0.051 * kDt, after.stages[1].lx(ILQR_IDX_THETA), 1e-12);
    // 无 2π 跳变：两侧代价/梯度均为小量（若 wrap 缺失，θ=−π+0.05 处
    // 误差会算成 ≈2π−0.051，代价 ~15.8、梯度 ~5.0，比实测大两个数量级）
    EXPECT_LT(std::abs(after.stages[1].cost_tracking), 0.02);
    EXPECT_LT(std::abs(after.stages[1].lx(ILQR_IDX_THETA)), 0.05);
    EXPECT_LT(std::abs(before.stages[1].cost_tracking), 0.02);
    EXPECT_LT(std::abs(before.stages[1].lx(ILQR_IDX_THETA)), 0.05);
}

// 测试退火掩码：掩码豁免点的 w_ref 恒取 w_ref,0（不随轮次衰减），
// 非豁免点按外部输入的 w_ref(r) 取值——首/末 maneuver 与锚点因此
// 能在退火后期仍保持跟踪压力。
TEST_F(iLQRCostEvaluatorTest, AnnealMaskExemptsPointsFromDecay) {
    iLQRCostConfig config = MakeZeroWeightConfig();
    config.weight_ref_base = 10.0;
    const iLQRCostEvaluator evaluator(config, nullptr);
    const auto states = MakeStates();
    const auto controls = MakeControls();
    const auto multipliers = iLQRCostMultiplierState::MakeZero(kNumSteps);
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

// 测试终点 AL 等式：c=[x−xg, y−yg, wrap(θ−θg), v, a] 的 λᵀc+½μc² 手推值、
// 梯度 λ+μ∘c、GN Hessian diag(μ)（选择矩阵常量，GN 即精确）；
// δ_N/ω_N 不承载终点约束，对应导数行/列必须恒为零。
TEST_F(iLQRCostEvaluatorTest, TerminalAlEqualityMatchesHandDerived) {
    const iLQRCostEvaluator evaluator(MakeZeroWeightConfig(), nullptr);
    const auto states = MakeStates();
    const auto controls = MakeControls();
    auto multipliers = iLQRCostMultiplierState::MakeZero(kNumSteps);
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
    EXPECT_NEAR(0.3 + 10.0 * 0.01, terminal.lx(ILQR_IDX_X), 1e-12);
    EXPECT_NEAR(-0.2 + 20.0 * 0.005, terminal.lx(ILQR_IDX_Y), 1e-12);
    EXPECT_NEAR(0.5 + 30.0 * (-0.01), terminal.lx(ILQR_IDX_THETA), 1e-12);
    EXPECT_NEAR(0.7 + 40.0 * 0.05, terminal.lx(ILQR_IDX_V), 1e-12);
    EXPECT_NEAR(-0.4 + 50.0 * 0.05, terminal.lx(ILQR_IDX_A), 1e-12);
    EXPECT_NEAR(10.0, terminal.lxx(ILQR_IDX_X, ILQR_IDX_X), 1e-12);
    EXPECT_NEAR(30.0, terminal.lxx(ILQR_IDX_THETA, ILQR_IDX_THETA), 1e-12);
    EXPECT_NEAR(50.0, terminal.lxx(ILQR_IDX_A, ILQR_IDX_A), 1e-12);
    // δ/ω 方向导数恒零（停稳后前轮转角无物理要求）
    for (const int comp : {ILQR_IDX_DELTA, ILQR_IDX_OMEGA}) {
        EXPECT_DOUBLE_EQ(terminal.lx(comp), 0.0);
        EXPECT_TRUE(terminal.lxx.row(comp).isZero());
        EXPECT_TRUE(terminal.lxx.col(comp).isZero());
    }
    // 梯度对拍（总代价五点差分）
    for (int comp = 0; comp < ILQR_STATE_DIM; ++comp) {
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
    for (int comp = 0; comp < ILQR_STATE_DIM; ++comp) {
        auto grad_at = [&](double offset) {
            auto perturbed = states;
            perturbed[kNumSteps](comp) += offset;
            return EvaluateAll(evaluator, reference_, perturbed, controls,
                               multipliers, 0.0)
                .stages[kNumSteps]
                .lx;
        };
        const iLQRState numeric_col = (grad_at(1e-4) - grad_at(-1e-4)) / 2e-4;
        ExpectMatrixClose(numeric_col, terminal.lxx.col(comp), 1e-6);
    }
}

// 测试总代价与逐阶段分解的一致性：total_cost 必须等于各阶段
// totalCost() 的累加（分解值是外层调度/日志的唯一数据源，不允许漂移）。
TEST_F(iLQRCostEvaluatorTest, TotalCostEqualsStageSum) {
    iLQRCostConfig config;
    config.weight_jerk = 1.0;
    config.weight_steer_accel = 2.0;
    config.weight_ref_base = 10.0;
    config.weight_theta = 5.0;
    const iLQRCostEvaluator evaluator(config, nullptr);
    const auto states = MakeStates();
    const auto controls = MakeControls();
    auto multipliers = iLQRCostMultiplierState::MakeZero(kNumSteps);
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
TEST_F(iLQRCostEvaluatorTest, EsdfPenaltyAppliesOnSampledStagesOnly) {
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
    iLQREsdfConstraintConfig esdf_config;
    esdf_config.stride = 2;
    const iLQREsdfConstraint esdf_constraint(esdf_map, footprint_model,
                                            esdf_config);
    const iLQRCostEvaluator with_esdf(MakeZeroWeightConfig(), &esdf_constraint);
    const iLQRCostEvaluator without_esdf(MakeZeroWeightConfig(), nullptr);
    // 全部状态平行贴墙（圆心 cx=0.5 < r+margin_comf，惩罚激活）
    auto states = MakeStates();
    for (auto& state : states) {
        state(ILQR_IDX_X) = 0.5;
        state(ILQR_IDX_Y) = 4.0;
        state(ILQR_IDX_THETA) = kHalfPi;
    }
    const auto controls = MakeControls();
    const auto multipliers = iLQRCostMultiplierState::MakeZero(kNumSteps);
    const auto sampled =
        EvaluateAll(with_esdf, reference_, states, controls, multipliers, 0.0);
    const auto unsampled = EvaluateAll(without_esdf, reference_, states,
                                       controls, multipliers, 0.0);
    // 抽样命中阶段（0、2 与恒评估的终端 3）：ESDF 代价为正且导数非零；
    // 未抽样阶段（1）：ESDF 分量严格为零，与无 ESDF 求值逐位一致
    for (const std::size_t k : {0U, 2U, 3U}) {
        EXPECT_GT(sampled.stages[k].cost_esdf, 0.0) << "stage " << k;
        EXPECT_FALSE(sampled.stages[k].lx.isZero()) << "stage " << k;
        EXPECT_GT(sampled.stages[k].lxx(ILQR_IDX_X, ILQR_IDX_X), 0.0)
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

// 测试线搜索早停筛选：阈值高于完整代价时筛选不触发，结果与无阈值
// 全量求值逐位一致（总代价、各阶段 ESDF 分量与一/二阶导数）。
TEST_F(iLQRCostEvaluatorTest, EsdfScreenThresholdAboveTotalKeepsFullResult) {
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
    const iLQREsdfConstraint esdf_constraint(esdf_map, footprint_model);
    const iLQRCostEvaluator evaluator(MakeZeroWeightConfig(), &esdf_constraint);
    // 全部状态平行贴墙（ESDF 激活）
    auto states = MakeStates();
    for (auto& state : states) {
        state(ILQR_IDX_X) = 0.5;
        state(ILQR_IDX_Y) = 4.0;
        state(ILQR_IDX_THETA) = kHalfPi;
    }
    const auto controls = MakeControls();
    const auto multipliers = iLQRCostMultiplierState::MakeZero(kNumSteps);
    const auto full = EvaluateAll(evaluator, reference_, states, controls,
                                  multipliers, 0.0);
    iLQRCostInput input;
    input.tracking_weight = 0.0;
    input.screen_cost_threshold = full.total_cost + 1.0;
    const auto screened = evaluator.evaluate(reference_, states, controls,
                                             multipliers, input);
    EXPECT_DOUBLE_EQ(screened.total_cost, full.total_cost);
    EXPECT_FALSE(screened.esdf_screened_out);
    for (std::size_t k = 0; k <= kNumSteps; ++k) {
        EXPECT_DOUBLE_EQ(screened.stages[k].cost_esdf,
                         full.stages[k].cost_esdf)
            << "stage " << k;
        ExpectBitwiseEqual(screened.stages[k].lx, full.stages[k].lx);
        ExpectBitwiseEqual(screened.stages[k].lxx, full.stages[k].lxx);
        EXPECT_DOUBLE_EQ(screened.stages[k].totalCost(),
                         full.stages[k].totalCost())
            << "stage " << k;
    }
}

// 测试线搜索早停筛选：阈值低于廉价小计（不含 ESDF）时 ESDF 求值
// 整段跳过——总代价等于无 ESDF 求值结果（逐位），各阶段 ESDF 分量
// 恒零、导数与无 ESDF 求值一致；同时锚定筛选的数学基础
// 「廉价小计 < 完整代价」（ESDF 代价恒非负）。
TEST_F(iLQRCostEvaluatorTest, EsdfScreenThresholdBelowCheapTotalSkipsEsdf) {
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
    const iLQREsdfConstraint esdf_constraint(esdf_map, footprint_model);
    // 非零权重配置：平滑/跟踪项给出正的廉价小计（全零权重时小计恒 0，
    // 任何正阈值都无法触发筛选，没有判别力）
    iLQRCostConfig config;
    config.weight_jerk = 1.0;
    config.weight_steer_accel = 2.0;
    config.weight_ref_base = 10.0;
    config.weight_theta = 5.0;
    const iLQRCostEvaluator with_esdf(config, &esdf_constraint);
    const iLQRCostEvaluator without_esdf(config, nullptr);
    // 全部状态平行贴墙（ESDF 激活）
    auto states = MakeStates();
    for (auto& state : states) {
        state(ILQR_IDX_X) = 0.5;
        state(ILQR_IDX_Y) = 4.0;
        state(ILQR_IDX_THETA) = kHalfPi;
    }
    const auto controls = MakeControls();
    const auto multipliers = iLQRCostMultiplierState::MakeZero(kNumSteps);
    const auto full = EvaluateAll(with_esdf, reference_, states, controls,
                                  multipliers, 4.0);
    const auto cheap = EvaluateAll(without_esdf, reference_, states, controls,
                                   multipliers, 4.0);
    // 负对照：场景确实处于 ESDF 激活区，廉价/完整两档存在严格间隙
    EXPECT_GT(full.total_cost, cheap.total_cost);
    iLQRCostInput input;
    input.tracking_weight = 4.0;
    // 阈值低于廉价小计 → 筛选必然触发（廉价小计 > 阈值）
    input.screen_cost_threshold = cheap.total_cost - 1.0;
    const auto screened = with_esdf.evaluate(reference_, states, controls,
                                             multipliers, input);
    // 早停返回：总代价与无 ESDF 求值逐位一致，ESDF 分量与导数均未计入
    EXPECT_DOUBLE_EQ(screened.total_cost, cheap.total_cost);
    EXPECT_TRUE(screened.esdf_screened_out);
    for (std::size_t k = 0; k <= kNumSteps; ++k) {
        EXPECT_DOUBLE_EQ(screened.stages[k].cost_esdf, 0.0) << "stage " << k;
        ExpectBitwiseEqual(screened.stages[k].lx, cheap.stages[k].lx);
        ExpectBitwiseEqual(screened.stages[k].lxx, cheap.stages[k].lxx);
    }
}

// 测试 dt 因子约定（回归防火墙）：平滑/跟踪项（积分型代价）随 dt 线性
// 缩放，ESDF 惩罚/幅值 AL/终点 AL（逐阶段点态量）不随 dt 变化。
// 若未来有人误对所有项统一乘 dt（或反向去掉既有 dt 因子），本用例会
// 立即失败——这是回推侧"不得统一乘 dt"约定在求值层的锚点。
TEST_F(iLQRCostEvaluatorTest, CostDtScalingConventionIsPinned) {
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
    const iLQREsdfConstraint esdf_constraint(esdf_map, footprint_model);
    iLQRCostConfig config;
    config.weight_jerk = 1.0;
    config.weight_steer_accel = 2.0;
    config.weight_ref_base = 10.0;
    config.weight_theta = 5.0;
    const iLQRCostEvaluator evaluator(config, &esdf_constraint);
    // 全部状态平行贴墙（ESDF 激活），stage 0 速度越界（幅值 AL 激活）
    auto states = MakeStates();
    for (auto& state : states) {
        state(ILQR_IDX_X) = 0.5;
        state(ILQR_IDX_Y) = 4.0;
        state(ILQR_IDX_THETA) = kHalfPi;
    }
    states[0](ILQR_IDX_V) = 1.6;
    const auto controls = MakeControls();
    auto multipliers = iLQRCostMultiplierState::MakeZero(kNumSteps);
    multipliers.amplitude_mu(ILQR_AMP_V) = 7.0;
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


TEST_F(iLQRCostEvaluatorTest, EsdfScaleMultipliesEsdfChannelLinearly) {
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
    const iLQREsdfConstraint esdf_constraint(esdf_map, footprint_model,
                                            iLQREsdfConstraintConfig{});
    const iLQRCostEvaluator evaluator(MakeZeroWeightConfig(), &esdf_constraint);
    // 全部状态平行贴墙，使 ESDF 惩罚在每个阶段都激活
    auto states = MakeStates();
    for (auto& state : states) {
        state(ILQR_IDX_X) = 0.5;
        state(ILQR_IDX_Y) = 4.0;
        state(ILQR_IDX_THETA) = kHalfPi;
    }
    const auto controls = MakeControls();
    const auto multipliers = iLQRCostMultiplierState::MakeZero(kNumSteps);
    const auto evaluate_with_scale = [&](double esdf_scale) {
        iLQRCostInput input;
        input.tracking_weight = 0.0;
        input.esdf_scale = esdf_scale;
        return evaluator.evaluate(reference_, states, controls, multipliers,
                                  input);
    };
    const auto unit = evaluate_with_scale(1.0);
    constexpr double kScale = 7.5;
    const auto scaled = evaluate_with_scale(kScale);
    ASSERT_GT(unit.total_cost, 0.0);
    EXPECT_DOUBLE_EQ(scaled.total_cost, kScale * unit.total_cost);
    for (std::size_t k = 0; k < unit.stages.size(); ++k) {
        EXPECT_DOUBLE_EQ(scaled.stages[k].cost_esdf,
                         kScale * unit.stages[k].cost_esdf)
            << "stage " << k;
        EXPECT_TRUE(scaled.stages[k].lx.isApprox(kScale * unit.stages[k].lx))
            << "stage " << k;
        EXPECT_TRUE(scaled.stages[k].lxx.isApprox(kScale * unit.stages[k].lxx))
            << "stage " << k;
    }
    // 默认因子恒为 1：不写 esdf_scale 的既有调用方行为逐位不变
    const iLQRCostInput default_input;
    EXPECT_DOUBLE_EQ(default_input.esdf_scale, 1.0);
    // 非法因子必须显式拒绝，而不是静默产出无避障能力的解
    EXPECT_THROW(evaluate_with_scale(0.0), std::invalid_argument);
    EXPECT_THROW(evaluate_with_scale(-1.0), std::invalid_argument);
    EXPECT_THROW(evaluate_with_scale(std::numeric_limits<double>::infinity()),
                 std::invalid_argument);
    EXPECT_THROW(evaluate_with_scale(std::nan("")), std::invalid_argument);
}

}  // namespace
}  // namespace apa_post_processor
