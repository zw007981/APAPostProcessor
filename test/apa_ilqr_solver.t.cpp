#include <gtest/gtest.h>

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <string>
#include <vector>

#include "core/iLQR/apa_ilqr_solver.h"
#include "core/iLQR/ilqr_reference_builder.h"
#include "core/iLQR/esdf_constraint.h"
#include "core/NMPC/vehicle_circle_geometry.h"
#include "core/post_processor.h"
#include "spatial/esdf_map.h"
#include "spatial/grid_map.h"
#include "util/constants.h"
#include "util/data_loader.hpp"
#include "util/path.h"
#include "vehicle/vehicle_footprint_model.h"
#include "vehicle/vehicle_params.h"

namespace apa_post_processor {
namespace {

constexpr double kWheelbase = 2.7;

// 测试用车辆参数：与既有 iLQR 组件单测一致的五参数构造
VehicleParams MakeVehicleParams() {
    return VehicleParams(4.3, 1.8, 2.7, 0.6, 0.8);
}

// 从当前路径末端沿 x 轴追加直线路径点（步长 0.05 m，与 A* 点距一致）
void AppendXLine(Path* path, double x_from, double x_to, double theta) {
    const int count =
        static_cast<int>(std::round(std::abs(x_to - x_from) / 0.05));
    for (int i = 1; i <= count; ++i) {
        const double x = x_from + (x_to - x_from) * i / count;
        path->addPoint({x, 0.0, theta});
    }
}

// 沿 x 轴构造多点路径：segments 为逐段目标 x（θ 恒 0），方向由 x 增减
// 自然形成（x 增 = 前进段、x 减 = 倒退段），分段处即换挡 cusp
Path BuildXPolyline(const std::vector<double>& segments) {
    Path path;
    path.addPoint({segments.front(), 0.0, 0.0});
    for (std::size_t i = 1; i < segments.size(); ++i) {
        AppendXLine(&path, segments[i - 1], segments[i], 0.0);
    }
    path.finalize();
    return path;
}

// 由 Path 构建阶段一前端数据（默认构建配置：0.05 m 重采样、dt=0.1 s、
// 打靶间隔 25 步、盒约束边界同参数表）
iLQRReference BuildReference(const Path& path) {
    return iLQRReferenceBuilder(iLQRConfig{}, MakeVehicleParams())
        .build(path);
}

// 合成小尺度场景的求解配置：这些场景的 J_s′ 量级在 0.1~10 之间，按设计
// 参数表的默认值 μ_min=1e2 标定时 μ⁰ 会被下限 clip 到比问题代价本身大
// 数个量级的罚权重，AL 首轮过冲瞬变会把内层直接淹死；合成场景把 clip
// 下限调低到 1.0（真实数据集 J_s′/‖c‖² 量级在 1e2 以上、不触及下限，
// 生产默认配置不受此覆写影响）
iLQRConfig MakeSyntheticConfig() {
    iLQRConfig config;
    config.outer_mu_min = 1.0;
    // 组件机制测试显式隔离：这些用例验证的是 AL/门控/阶段二机制本身，
    // 与 ALTRO 虚拟控制正交，统一关闭避免默认值变更改变机制语义
    config.inner_use_virtual_control = false;
    return config;
}

// 阶段一求解便捷入口：就地构造求值层与编排器（ESDF 约束可为空 =
// 无地图开阔场景），返回完整求解输出
ApaILQRStageOneResult SolveStageOne(
    const iLQRReference& reference, const iLQREsdfConstraint* esdf_constraint,
    const iLQRConfig& config = MakeSyntheticConfig()) {
    const BicycleDynamics dynamics(kWheelbase);
    const iLQRCostEvaluator cost_evaluator(config, esdf_constraint);
    ApaILQRSolver solver(config, &dynamics, &cost_evaluator);
    return solver.solveStageOne(reference);
}

// 带滞回的符号游程计数：|v| < eps 的样本不计入任何游程，一次变号即
// 新增一个游程（融化后应只剩 1 个游程；保留换挡的场景应观测到多游程）
int CountVelocitySignRuns(const iLQRAlignedVec<iLQRState>& states, double eps) {
    int runs = 0;
    int last_sign = 0;
    for (const auto& state : states) {
        const double v = state(ILQR_IDX_V);
        if (std::abs(v) < eps) {
            continue;
        }
        const int sign = v > 0.0 ? 1 : -1;
        if (sign != last_sign) {
            ++runs;
            last_sign = sign;
        }
    }
    return runs;
}

// 轨迹速度最小值（融化断言：min v 不跌破滞回阈值 = 不变号穿过原 cusp）
double MinVelocity(const iLQRAlignedVec<iLQRState>& states) {
    double min_v = std::numeric_limits<double>::max();
    for (const auto& state : states) {
        min_v = std::min(min_v, state(ILQR_IDX_V));
    }
    return min_v;
}

// 全部状态/控制均为有限值（失败路径不得返回未初始化数据）
bool IsTrajectoryFinite(const ApaILQRStageOneResult& result) {
    for (const auto& state : result.states) {
        for (int i = 0; i < ILQR_STATE_DIM; ++i) {
            if (!std::isfinite(state(i))) {
                return false;
            }
        }
    }
    for (const auto& control : result.controls) {
        for (int i = 0; i < ILQR_CONTROL_DIM; ++i) {
            if (!std::isfinite(control(i))) {
                return false;
            }
        }
    }
    return true;
}

// 与生产质量门同口径的碰撞深度抽检：遍历全部外圆，取
// outer_radius − ESDF(圆心) 的最大值（≤ 0 无碰撞，质量门允许 ≤ 0.02 m）
double MaxCollisionDepth(const iLQRAlignedVec<iLQRState>& states,
                         const ESDFMap& esdf_map,
                         const VehicleFootprintModel& footprint_model) {
    const double outer_radius = footprint_model.getOuterRadius();
    const auto local_centers =
        vehicle_circle_geometry::ExtractLocalCircleCenters(footprint_model,
                                                           CircleType::OUTER);
    double max_depth = 0.0;
    for (const auto& state : states) {
        const double cos_theta = std::cos(state(ILQR_IDX_THETA));
        const double sin_theta = std::sin(state(ILQR_IDX_THETA));
        for (const auto& local : local_centers) {
            const double wx = state(ILQR_IDX_X) + local.x() * cos_theta -
                              local.y() * sin_theta;
            const double wy = state(ILQR_IDX_Y) + local.x() * sin_theta +
                              local.y() * cos_theta;
            max_depth =
                std::max(max_depth, outer_radius - esdf_map.getDist(wx, wy));
        }
    }
    return max_depth;
}

// 测试编排器输入校验：参考位姿不足两个、初值尺寸不符、动力学/求值层
// 为空指针等契约违例必须显式抛出，禁止带着畸形问题进入求解循环
TEST(ApaILQRSolverTest, InvalidInputThrows) {
    const iLQRConfig config;
    const BicycleDynamics dynamics(kWheelbase);
    const iLQRCostEvaluator cost_evaluator(config, nullptr);
    // 空指针构造拒绝
    EXPECT_THROW(ApaILQRSolver(config, nullptr, &cost_evaluator),
                 std::invalid_argument);
    EXPECT_THROW(ApaILQRSolver(config, &dynamics, nullptr),
                 std::invalid_argument);
    ApaILQRSolver solver(config, &dynamics, &cost_evaluator);
    // 位姿不足两个
    iLQRReference reference;
    reference.dt = 0.1;
    reference.poses.emplace_back(0.0, 0.0, 0.0);
    EXPECT_THROW(solver.solveStageOne(reference), std::invalid_argument);
    // 初值尺寸与位姿网格不符（N+1=2 个位姿只给 1 个状态初值）
    reference.poses.emplace_back(0.05, 0.0, 0.0);
    reference.initial_states.push_back(iLQRState::Zero());
    reference.initial_controls.push_back(iLQRControl::Zero());
    EXPECT_THROW(solver.solveStageOne(reference), std::invalid_argument);
}

// 灵魂用例——微 maneuver 融化回归：合成「前进 1.0 m → 倒退 0.15 m →
// 前进 1.0 m」参考（对应设计文档能量论证的「停-倒 0.15 m-停」场景），
// 中间倒退微 maneuver 随退火丧失跟踪拉力后，跃度主导代价应把它压没——
// 阶段一解中 v 全程不变号穿过两个原 cusp（符号游程只剩 1 个），
// 且终点双指标照常达标
TEST(ApaILQRSolverTest, MicroManeuverMeltsWithoutSignChange) {
    const Path path = BuildXPolyline({0.0, 1.0, 0.85, 1.85});
    ASSERT_EQ(path.numManeuvers(), 3);
    const iLQRReference reference = BuildReference(path);
    ASSERT_EQ(reference.maneuvers.size(), 3);
    const auto result = SolveStageOne(reference, nullptr);
    // 联合判据收敛（终点双指标 + 不等式 + 缺陷）
    EXPECT_EQ(result.report.status, ApaILQRStatus::CONVERGED)
        << "outer=" << result.report.outer_iterations
        << " pos_err=" << result.report.terminal_position_error
        << " head_err_deg=" << result.report.terminal_heading_error_deg
        << " ineq=" << result.report.max_amplitude_violation
        << " defect=" << result.report.defect_norm_inf;
    EXPECT_TRUE(IsTrajectoryFinite(result));
    // 融化核心断言：v 全程不变号（不跌破滞回阈值 0.02 m/s），
    // 原两个 cusp 被连续穿过，带滞回符号游程只剩 1 个
    const double min_v = MinVelocity(result.states);
    const int sign_runs = CountVelocitySignRuns(result.states, 0.02);
    EXPECT_GT(min_v, -0.02) << "min_v = " << min_v;
    EXPECT_EQ(sign_runs, 1) << "min_v = " << min_v;
    // 终点照常收敛：位置/朝向达标且静止收尾（v_N/a_N 的 AL 稳态残差
    // 随 μ 增长继续收紧，0.05 与设计文档的驻留速度帽 v_dwell 同语义）
    EXPECT_LE(result.report.terminal_position_error, 0.05);
    EXPECT_LE(result.report.terminal_heading_error_deg, 1.5);
    const iLQRState& final_state = result.states.back();
    EXPECT_NEAR(final_state(ILQR_IDX_V), 0.0, 0.05);
    EXPECT_NEAR(final_state(ILQR_IDX_A), 0.0, 0.05);
    std::cout << "[iLQR-MELT] outer=" << result.report.outer_iterations
              << " inner_total=" << result.report.total_inner_iterations
              << " min_v=" << min_v << " sign_runs=" << sign_runs
              << " pos_err=" << result.report.terminal_position_error
              << std::endl;
}

// 方向性反例——有用 maneuver 不误融：正前方被墙封堵（ESDF 惩罚生效），
// 目标位于起点后方，倒退 maneuver 是终点对齐所必需；即使中间倒退段
// 不豁免退火，AL 渐硬的终点等式也不允许它被压没。验证解保留明显换向
// （min v 显著为负），且墙显著改变轨迹形态（前向峰值被压回）
TEST(ApaILQRSolverTest, UsefulManeuverSurvivesGlobalSoftening) {
    const VehicleParams vehicle_params = MakeVehicleParams();
    const VehicleFootprintModel footprint_model(vehicle_params, 233, 2, 2);
    const double outer_radius = footprint_model.getOuterRadius();
    // 前进 1.0 → 倒退 3.0 → 前进 0.5，目标 (-1.5, 0, θ=0)；倒退 maneuver
    // 承载终点对齐语义，属不可融化的有用段
    const Path path = BuildXPolyline({0.0, 1.0, -2.0, -1.5});
    ASSERT_EQ(path.numManeuvers(), 3);
    const iLQRReference reference = BuildReference(path);
    const auto forward_peak = [](const iLQRAlignedVec<iLQRState>& states) {
        double peak = -std::numeric_limits<double>::max();
        for (const auto& state : states) {
            peak = std::max(peak, state(ILQR_IDX_X));
        }
        return peak;
    };
    // 第一阶段：无墙基线求解，实测优化后轨迹的前向峰值（通常达不到
    // 参考峰值——按实测峰值布墙才能保证墙被轨迹真正触及）
    const auto result_free = SolveStageOne(reference, nullptr);
    ASSERT_EQ(result_free.report.status, ApaILQRStatus::CONVERGED);
    const double peak_free = forward_peak(result_free.states);
    const auto local_centers =
        vehicle_circle_geometry::ExtractLocalCircleCenters(footprint_model,
                                                           CircleType::OUTER);
    double max_local_x = 0.0;
    for (const auto& center : local_centers) {
        max_local_x = std::max(max_local_x, center.x());
    }
    // 第二阶段：墙深入基线轨迹最前外圆 0.05 m（基线解的安全红线惩罚
    // 被激活），重新求解——轨迹必须压回前向峰值才能满足避障
    const double wall_x = peak_free + max_local_x + outer_radius - 0.05;
    std::vector<Position> cells;
    cells.reserve(41);
    for (int i = -20; i <= 20; ++i) {
        cells.emplace_back(Position{wall_x, 0.1 * i});
    }
    const GridMap grid_map(0.1, 300, 200, Position{-15.0, -10.0}, cells);
    const ESDFMap esdf_map(grid_map);
    const iLQREsdfConstraint esdf_constraint(esdf_map, footprint_model);
    const auto result = SolveStageOne(reference, &esdf_constraint);
    EXPECT_EQ(result.report.status, ApaILQRStatus::CONVERGED)
        << "outer=" << result.report.outer_iterations
        << " pos_err=" << result.report.terminal_position_error
        << " head_err_deg=" << result.report.terminal_heading_error_deg
        << " ineq=" << result.report.max_amplitude_violation
        << " defect=" << result.report.defect_norm_inf;
    EXPECT_TRUE(IsTrajectoryFinite(result));
    // 换向保留：min v 显著为负、带滞回符号游程 ≥3（前进→倒退→前进，倒退段
    // 承载终点对齐语义，属不可融化部分）。具体游程数随 ESDF 舒适权重微变：
    // w_comf=1 时实测 3，w_comf=10 时实测 4（更强避障惩罚下多出一段短机动），
    // 故断言不变式而非精确拓扑
    const double min_v = MinVelocity(result.states);
    const int sign_runs = CountVelocitySignRuns(result.states, 0.02);
    EXPECT_LT(min_v, -0.1) << "min_v = " << min_v;
    EXPECT_GE(sign_runs, 3) << "min_v = " << min_v;
    // 碰撞抽检（质量门 0.02 m）与 ESDF 生效证据（前向峰值被墙压回）
    const double max_depth =
        MaxCollisionDepth(result.states, esdf_map, footprint_model);
    const double peak_walled = forward_peak(result.states);
    EXPECT_LE(max_depth, 0.02) << "max_depth = " << max_depth;
    EXPECT_LT(peak_walled, peak_free - 0.03)
        << "peak_walled=" << peak_walled << " peak_free=" << peak_free;
    std::cout << "[iLQR-KEEP] outer=" << result.report.outer_iterations
              << " min_v=" << min_v << " sign_runs=" << sign_runs
              << " max_depth=" << max_depth << " peak_walled=" << peak_walled
              << " peak_free=" << peak_free << std::endl;
}

// 终点收敛：开阔场景直线前进 3 m，终点双指标达标、v_N/a_N → 0（泊车
// 必须静止收尾）；δ_N/ω_N 为自由端点量（停稳后前轮转角无物理要求，
// 留给下游回正逻辑），不做断言
TEST(ApaILQRSolverTest, TerminalStateConvergesWithFreeFinalSteer) {
    const Path path = BuildXPolyline({0.0, 3.0});
    const iLQRReference reference = BuildReference(path);
    const auto result = SolveStageOne(reference, nullptr);
    EXPECT_EQ(result.report.status, ApaILQRStatus::CONVERGED)
        << "outer=" << result.report.outer_iterations
        << " pos_err=" << result.report.terminal_position_error
        << " head_err_deg=" << result.report.terminal_heading_error_deg;
    EXPECT_LE(result.report.terminal_position_error, 0.05);
    EXPECT_LE(result.report.terminal_heading_error_deg, 1.5);
    const iLQRState& final_state = result.states.back();
    EXPECT_NEAR(final_state(ILQR_IDX_V), 0.0, 0.05);
    EXPECT_NEAR(final_state(ILQR_IDX_A), 0.0, 0.05);
    // 终点位置/朝向贴合参考末位姿（双指标的另一侧面）
    const Pose& goal = reference.poses.back();
    EXPECT_NEAR(final_state(ILQR_IDX_X), goal.x, 0.05);
    EXPECT_NEAR(final_state(ILQR_IDX_Y), goal.y, 0.05);
    EXPECT_NEAR(WrapAngle(final_state(ILQR_IDX_THETA) - goal.theta), 0.0,
                1.5 * PI / 180.0);
}

// 阶段一热启动重载（continuation/救援重试的基础设施）：以调用方给定的
// 状态/控制轨迹启动首轮（而非参考自带的前端初值），已收敛解热启动
// 重解应当秒收敛且轮数不超过冷启动；尺寸不符必须显式抛出
TEST(ApaILQRSolverTest, StageOneWarmStartReconvergesQuickly) {
    const Path path = BuildXPolyline({0.0, 1.0, 0.85, 1.85});
    const iLQRReference reference = BuildReference(path);
    const auto cold = SolveStageOne(reference, nullptr);
    ASSERT_EQ(cold.report.status, ApaILQRStatus::CONVERGED);
    const BicycleDynamics dynamics(kWheelbase);
    const iLQRConfig config = MakeSyntheticConfig();
    const iLQRCostEvaluator evaluator(config, nullptr);
    ApaILQRSolver warm_solver(config, &dynamics, &evaluator);
    const auto warm =
        warm_solver.solveStageOne(reference, cold.states, cold.controls);
    EXPECT_EQ(warm.report.status, ApaILQRStatus::CONVERGED)
        << "outer=" << warm.report.outer_iterations
        << " pos_err=" << warm.report.terminal_position_error;
    // 热启动首轮即接近收敛：外层轮数不超过冷启动
    EXPECT_LE(warm.report.outer_iterations, cold.report.outer_iterations);
    EXPECT_LE(warm.report.terminal_position_error, 0.05);
    // 尺寸契约：N+1 / N 不符显式抛出
    auto bad_states = cold.states;
    bad_states.pop_back();
    EXPECT_THROW(
        warm_solver.solveStageOne(reference, bad_states, cold.controls),
        std::invalid_argument);
    auto bad_controls = cold.controls;
    bad_controls.pop_back();
    EXPECT_THROW(
        warm_solver.solveStageOne(reference, cold.states, bad_controls),
        std::invalid_argument);
}

// 内层迭代超限不致命（显式钉住）：把内层迭代上限压到 1，每一轮内层
// 都必然以 MAX_ITERATIONS 退出（不可能收敛），验证外层不因此判败，
// 而是沿用当前迭代点继续推进外层调度——全程无冷重启发生，
// 终点误差跨轮次显著下降
TEST(ApaILQRSolverTest, InnerMaxIterationsIsNotFatal) {
    const Path path = BuildXPolyline({0.0, 3.0});
    const iLQRReference reference = BuildReference(path);
    auto config = MakeSyntheticConfig();
    config.inner_max_iterations = 1;
    const auto result = SolveStageOne(reference, nullptr, config);
    // 外层持续推进：状态码只能是收敛或外层耗尽，绝不为内层失败
    EXPECT_TRUE(result.report.status == ApaILQRStatus::CONVERGED ||
                result.report.status == ApaILQRStatus::MAX_OUTER_ITERATIONS)
        << "status=" << static_cast<int>(result.report.status);
    // 每一轮内层都确实以迭代超限退出（冷重启计数为零：超限不触发兜底）
    ASSERT_FALSE(result.report.history.empty());
    for (const auto& record : result.report.history) {
        EXPECT_EQ(record.inner_status, MsIlqrStatus::MAX_ITERATIONS);
    }
    EXPECT_EQ(result.report.inner_restarts, 0);
    // 外层调度仍在做功：终点误差较首轮显著下降
    const double first_err =
        result.report.history.front().terminal_position_error;
    const double last_err = result.report.terminal_position_error;
    EXPECT_LT(last_err, first_err)
        << "first=" << first_err << " last=" << last_err;
    EXPECT_TRUE(IsTrajectoryFinite(result));
    std::cout << "[iLQR-INNERCAP] outer=" << result.report.outer_iterations
              << " first_err=" << first_err << " last_err=" << last_err
              << std::endl;
}

// 白盒访问：派生类将受保护方法公开给单元测试
class ApaILQRTestAccess : public ApaILQRSolver {
   public:
    using ApaILQRSolver::ApaILQRSolver;
    using ApaILQRSolver::MeritAlHook;
};

// merit 地板的挂钩量真值表（A1 修复）：终端/幅值两组 AL 罚权重必须
// 取大——增广代价由两组共同构成，只挂终端组会在幅值组主导的场景
// 低估「缺陷修复与代价下降的交换比」（data6 即幅值组主导：
// µ_amp ≥ 1e4 而 µ_term ≤ 10135.8，只挂终端时地板不激活）
TEST(ApaILQRSolverTest, MeritAlHookUsesMaxOfBothGroups) {
    // 幅值组主导（data6 形态）：挂钩取幅值组
    EXPECT_DOUBLE_EQ(ApaILQRTestAccess::MeritAlHook(10135.8, 1e5), 1e5);
    // 终端组主导（data3 阶段二病态种子形态）：挂钩取终端组
    EXPECT_DOUBLE_EQ(ApaILQRTestAccess::MeritAlHook(1e6, 1e4), 1e6);
    // 两组相等：取任一（结果相同）
    EXPECT_DOUBLE_EQ(ApaILQRTestAccess::MeritAlHook(1e3, 1e3), 1e3);
    // 两组均为首轮临时值：地板仍低（挂钩不放大首轮弱启动）
    EXPECT_DOUBLE_EQ(ApaILQRTestAccess::MeritAlHook(1.0, 1.0), 1.0);
}

// 冷重启链路（显式钉住）：把 reg_max 压到与 reg_initial 相等（ρ_reg 无
// 任何升级空间），状态初值全部归零与参考拉开 3 m 级误差、首轮跟踪权重
// 拉满——线搜索必然被拒且正则化立即溢出；同一配置下冷重启重试同样
// 溢出。验证：重试计数恰好为 1、状态码为 INNER_SOLVER_FAILED、轨迹仍
// 为最后可用的有限名义值，不挂死、不返回未初始化数据
TEST(ApaILQRSolverTest, InnerOverflowTriggersColdRestartOnce) {
    const Path path = BuildXPolyline({0.0, 3.0});
    iLQRReference reference = BuildReference(path);
    // 状态初值全部归零：与参考位姿拉开 3 m 级初始跟踪误差
    for (auto& state : reference.initial_states) {
        state.setZero();
    }
    auto config = MakeSyntheticConfig();
    config.cost_weight_ref_base = 1e6;
    config.inner_armijo_gamma = 0.9;
    config.inner_max_backtracks = 1;
    config.inner_reg_initial = 1e-3;
    config.inner_reg_max = 1e-3;
    const auto result = SolveStageOne(reference, nullptr, config);
    EXPECT_EQ(result.report.status, ApaILQRStatus::INNER_SOLVER_FAILED);
    EXPECT_EQ(result.report.inner_restarts, 1);
    // 首轮即失败：无完成轮次、无新增量测，终态指标保持初始安全值
    EXPECT_EQ(result.report.outer_iterations, 0);
    EXPECT_TRUE(result.report.history.empty());
    EXPECT_FALSE(result.report.terminal_ok);
    // 轨迹完整有限（anytime 性质：初始名义轨迹仍为合法输出）
    ASSERT_EQ(result.states.size(), reference.poses.size());
    ASSERT_EQ(result.controls.size() + 1, reference.poses.size());
    EXPECT_TRUE(IsTrajectoryFinite(result));
}

// 失败路径：终点朝向要求在 6 s 固定视窗内完成 π 掉头（最小转弯半径下
// 可行弧长预算不足），阶段一必然无法达标。验证在外层迭代上限/内层
// 正则化预算内正常退出（两种失败状态码均为合法出口：外层耗尽或 AL
// 罚权重推到内层病态后的快速失败），结构化诊断（哪类判据未达标 +
// 最终违反度）齐全，不挂死、不返回未初始化数据
TEST(ApaILQRSolverTest, InfeasibleTerminalHeadingFailsWithDiagnostics) {
    // 直线 x 前进 3 m，但 θ 沿线性插值到 π（几何不可行的合成参考）
    Path path;
    constexpr int kPoints = 61;
    for (int i = 0; i < kPoints; ++i) {
        const double ratio = static_cast<double>(i) / (kPoints - 1);
        path.addPoint({3.0 * ratio, 0.0, PI * ratio});
    }
    path.finalize();
    const iLQRReference reference = BuildReference(path);
    // 显式恢复组件独立默认幅值（δ_max=0.55/ω_max=0.5）：本测试锁定
    // 「几何不可行 → 外层迭代后失败且 head_err 大」的出口语义；统一后的
    // iLQRConfig 默认（0.4/0.47728）会让内层第 0 轮直接病态失败、
    // head_err 停留在初始值 0，偏离本测试意图
    iLQRConfig config = MakeSyntheticConfig();
    config.cost_delta_max = 0.55;
    config.cost_omega_max = 0.5;
    const auto result = SolveStageOne(reference, nullptr, config);
    EXPECT_TRUE(result.report.status == ApaILQRStatus::MAX_OUTER_ITERATIONS ||
                result.report.status == ApaILQRStatus::INNER_SOLVER_FAILED)
        << "status=" << static_cast<int>(result.report.status)
        << " head_err_deg=" << result.report.terminal_heading_error_deg;
    // 结构化诊断：终点判据未达标且最终朝向违反度被如实记录
    EXPECT_FALSE(result.report.terminal_ok);
    EXPECT_GT(result.report.terminal_heading_error_deg, 1.5);
    EXPECT_EQ(result.report.outer_iterations,
              static_cast<int>(result.report.history.size()));
    // 轨迹完整且全部有限：任何出口都有安全输出
    ASSERT_EQ(result.states.size(), reference.poses.size());
    ASSERT_EQ(result.controls.size() + 1, reference.poses.size());
    EXPECT_TRUE(IsTrajectoryFinite(result));
    std::cout << "[iLQR-FAIL] outer=" << result.report.outer_iterations
              << " head_err_deg=" << result.report.terminal_heading_error_deg
              << " pos_err=" << result.report.terminal_position_error
              << std::endl;
}

// 真实数据集冒烟：data/mid_park/data3.json 镜像生产路径跑通阶段一
// （RS 换挡点短接 → 前端构建 → AL 外层 + MS-iLQR 内层全链路），解通过
// 碰撞抽检（外圆 ESDF 间隙 ≥ −0.02 m 质量门），轨迹完整有限且联合判据
// 收敛。输入与配置均与生产一致：加载 data/ilqr_config.json 覆盖项并按
// 车辆真值钳制幅值边界；生产权重（w_comf=10）下原始路径直解会内层病态
// 失败，必须先做 RS 短接（9→6 段、长度 24.582→21.131m）才可解。
// 注意：本用例为重型端到端测试（N≈460、十余轮外层 × 数十次内层迭代
// 含 ESDF 求值），Debug 构建下约 50 s；如需快速反馈可用
// --gtest_filter 排除
TEST(ApaILQRSolverTest, RealDatasetStageOneSmoke) {
    ::apa::post_processor::OptimizeRequest request;
    ASSERT_EQ(
        DataLoader::LoadProtoFromJsonFile("data/mid_park/data3.json", request),
        LoadResult::SUCCESS);
    const auto vehicle_params = VehicleParams::FromProto(request.vehicle());
    const Path init_path = Path::FromProto(request.initial_path());
    ASSERT_FALSE(init_path.empty());
    const GridMap grid_map = GridMap::FromProto(request.environment());
    const ESDFMap esdf_map(grid_map);
    const VehicleFootprintModel footprint_model(vehicle_params, 233, 2, 2);
    // 生产配置：加载 data/ilqr_config.json 覆盖项后按车辆真值收紧幅值边界
    // （与 iLQRSolver::optimizeSinglePass 的装配步骤一致）
    nlohmann::json details;
    ASSERT_EQ(DataLoader::LoadJsonFile("data/ilqr_config.json", details),
              LoadResult::SUCCESS);
    iLQRConfig config;
    LoadiLQRConfigOverrides(details, &config);
    config.clampToVehicleParams(vehicle_params);
    // RS 换挡点短接（镜像生产步骤 0）：data3 原始路径在生产权重下阶段一
    // 直解会内层病态失败（INNER_SOLVER_FAILED），短接后的初始轨迹才可解
    Path solver_input = init_path;
    if (config.rs_cap_ratio > 0.0) {
        solver_input = ShortcutShiftPoints(
            init_path, esdf_map, footprint_model, vehicle_params.wheelbase,
            config.reference_delta_max, config);
    }
    // 参考构建与求解使用数据集自带的车辆参数（轴距进入动力学反解）
    const iLQRReference reference =
        iLQRReferenceBuilder(config, vehicle_params)
            .build(solver_input);
    const iLQREsdfConstraint esdf_constraint(esdf_map, footprint_model,
                                             config);
    const BicycleDynamics dynamics(vehicle_params.wheelbase);
    const iLQRCostEvaluator cost_evaluator(config,
                                           &esdf_constraint);
    ApaILQRSolver solver(config, &dynamics, &cost_evaluator);
    const auto result = solver.solveStageOne(reference);
    EXPECT_EQ(result.report.status, ApaILQRStatus::CONVERGED)
        << "outer=" << result.report.outer_iterations
        << " pos_err=" << result.report.terminal_position_error
        << " head_err_deg=" << result.report.terminal_heading_error_deg
        << " ineq=" << result.report.max_amplitude_violation
        << " defect=" << result.report.defect_norm_inf;
    ASSERT_EQ(result.states.size(), reference.poses.size());
    ASSERT_EQ(result.controls.size() + 1, reference.poses.size());
    EXPECT_TRUE(IsTrajectoryFinite(result));
    // 碰撞抽检：全部采样点全部外圆的最大碰撞深度不超过质量门
    const double max_depth =
        MaxCollisionDepth(result.states, esdf_map, footprint_model);
    EXPECT_LE(max_depth, 0.02) << "max_depth = " << max_depth;
    // 终点双指标达标（CONVERGED 的组成部分，显式复检）
    EXPECT_LE(result.report.terminal_position_error, 0.05);
    EXPECT_LE(result.report.terminal_heading_error_deg, 1.5);
    std::cout << "[iLQR-DATA3] N=" << reference.poses.size() - 1
              << " outer=" << result.report.outer_iterations
              << " inner_total=" << result.report.total_inner_iterations
              << " pos_err=" << result.report.terminal_position_error
              << " head_err_deg=" << result.report.terminal_heading_error_deg
              << " max_depth=" << max_depth << std::endl;
}

}  // namespace

// 临时探针（调试用，随诊断结束移除）：data3 阶段一在不同 η_κ 盒下的内层逐迭代轨迹
TEST(ApaILQRSolverTest, ProbeKappaRound0Iterations) {
    ::apa::post_processor::OptimizeRequest request;
    ASSERT_EQ(
        DataLoader::LoadProtoFromJsonFile("data/mid_park/data3.json", request),
        LoadResult::SUCCESS);
    const auto vehicle_params = VehicleParams::FromProto(request.vehicle());
    const Path init_path = Path::FromProto(request.initial_path());
    const GridMap grid_map = GridMap::FromProto(request.environment());
    const ESDFMap esdf_map(grid_map);
    const VehicleFootprintModel footprint_model(vehicle_params, 233, 2, 2);
    const iLQREsdfConstraint esdf_constraint(esdf_map, footprint_model);
    const iLQRReference reference =
        iLQRReferenceBuilder(iLQRConfig{}, vehicle_params)
            .build(init_path);
    for (const double eta_box : {1.0, 0.26}) {
        iLQRConfig config;
        config.inner_steer_accel_max = eta_box;
        const BicycleDynamics dynamics(vehicle_params.wheelbase);
        const iLQRCostEvaluator cost_evaluator(config, &esdf_constraint);
        ApaILQRSolver solver(config, &dynamics, &cost_evaluator);
        const auto result = solver.solveStageOne(reference);
        std::printf("=== eta_box=%.2f status=%d outer=%d\n", eta_box,
                    static_cast<int>(result.report.status),
                    result.report.outer_iterations);
        int shown = 0;
        for (const auto& rec : solver.innerSolver().history()) {
            if (shown++ < 14) {
                std::printf("iter=%d cost=%.4f merit=%.4f defect=%.4e "
                            "alpha=%.3e bp=%d\n",
                            rec.iteration, rec.cost, rec.merit,
                            rec.defect_norm, rec.alpha, rec.backward_passes);
            }
        }
    }
}

// 测试升级机制耗尽判据：只有「内层已达不动点 + 所有仍违反组的 μ 钉在
// 上限且本轮提出升级」才判耗尽；任一组仍有升级空间或内层未收敛都不得
// 提前退出（防止误杀正在靠 μ/λ 升级逼近的解）
TEST(ApaILQRSolverTest, EscalationStuckRequiresAllKnobsExhausted) {
    // 全部满足（含门控违反但已钉死）→ 耗尽
    const EscalationState exhausted{
        false, true, true,   // 终点：违反、请求、钉死
        false, true, true,   // 幅值：违反、请求、钉死
        false, true, true, true, true};
    EXPECT_TRUE(EscalationStuck(exhausted));
    // 门控未开启时门控字段不参与判定
    const EscalationState no_gating{
        false, true, true, false, true, true,
        false, false, false, false, true};
    EXPECT_TRUE(EscalationStuck(no_gating));
    // 内层未收敛 → 不动点论证不成立，不判耗尽
    EscalationState not_stationary = exhausted;
    not_stationary.inner_stationary = false;
    EXPECT_FALSE(EscalationStuck(not_stationary));
    // 终点违反但未请求升级（仍在门控级下降中）→ 不判耗尽
    EscalationState improving = exhausted;
    improving.terminal_wanted = false;
    EXPECT_FALSE(EscalationStuck(improving));
    // 终点违反、请求升级但未钉上限（μ 仍有空间）→ 不判耗尽
    EscalationState unpinned = exhausted;
    unpinned.terminal_pinned = false;
    EXPECT_FALSE(EscalationStuck(unpinned));
    // 幅值组尚有升级空间 → 不判耗尽
    EscalationState amp_unpinned = exhausted;
    amp_unpinned.amplitude_pinned = false;
    EXPECT_FALSE(EscalationStuck(amp_unpinned));
    // 门控违反但未钉上限 → 不判耗尽
    EscalationState gating_unpinned = exhausted;
    gating_unpinned.gating_pinned = false;
    EXPECT_FALSE(EscalationStuck(gating_unpinned));
    // 全绿场景（各组均达标）→ 外层已收敛，判据不触发
    const EscalationState all_ok{
        true, false, false, true, false, false,
        true, false, false, true, true};
    EXPECT_TRUE(EscalationStuck(all_ok));
}
}  // namespace apa_post_processor

