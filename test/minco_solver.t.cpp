#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

#include "core/MINCO/minco_maneuver_segmenter.h"
#include "core/MINCO/minco_preprocessor.h"
#include "core/MINCO/minco_solver.h"
#include "core/NMPC/vehicle_circle_geometry.h"
#include "spatial/esdf_map.h"
#include "spatial/grid_map.h"
#include "util/constants.h"
#include "vehicle/vehicle_footprint_model.h"
#include "vehicle/vehicle_params.h"

namespace apa_post_processor {
namespace {

// 测试用派生类：暴露受保护的问题装配与代价求值入口，供白盒梯度对拍
class MincoSolverTestAccessor : public MincoSolver {
   public:
    using MincoSolver::MincoSolver;
    using MincoSolver::buildProblem;
    using MincoSolver::evaluateCostAndGradient;
};

// 从当前路径末端沿 x 轴追加直线路径点（步长 0.05 m，与 A* 点距一致）
void AppendXLine(Path* path, double x_from, double x_to, double theta) {
    const int count =
        static_cast<int>(std::round(std::abs(x_to - x_from) / 0.05));
    for (int i = 1; i <= count; ++i) {
        const double x = x_from + (x_to - x_from) * i / count;
        path->addPoint({x, 0.0, theta});
    }
}

// 构造从原点出发沿 x 轴的直线前进路径
Path BuildStraightPath(double length) {
    Path path;
    path.addPoint({0.0, 0.0, 0.0});
    AppendXLine(&path, 0.0, length, 0.0);
    path.finalize();
    return path;
}

// 构造 前进1.0m → 后退0.5m 的单次换挡路径
Path BuildGearShiftPath() {
    Path path;
    path.addPoint({0.0, 0.0, 0.0});
    AppendXLine(&path, 0.0, 1.0, 0.0);
    AppendXLine(&path, 1.0, 0.5, 0.0);
    path.finalize();
    return path;
}

// 构造与直墙（第 0 列占据）平行的 y 向直线路径
Path BuildWallParallelPath(double x, double y_from, double y_to, double theta) {
    Path path;
    path.addPoint({x, y_from, theta});
    const int count =
        static_cast<int>(std::round(std::abs(y_to - y_from) / 0.05));
    for (int i = 1; i <= count; ++i) {
        const double y = y_from + (y_to - y_from) * i / count;
        path.addPoint({x, y, theta});
    }
    path.finalize();
    return path;
}

// 无障碍空地图（大范围覆盖车辆包络，原点为地图锚点）
GridMap BuildEmptyGridMap() {
    return GridMap(0.125, 160, 112, Position{-4.0, -6.0}, {});
}

// 直墙地图：第 0 列整列占据；x >= 2*resolution 且远离其余边界圈的区域
// 符号距离场严格为 D=x、梯度 g=(1,0)（与 minco_esdf_penalty.t.cpp 同一场景
// 约定；L8.2 后第 0/63 行与第 63 列亦为占据边界圈，使用本图的测试场景须
// 保证外圆圆心到这些边界圈的距离明显大于到墙面的距离）
GridMap BuildWallGridMap() {
    std::vector<Position> cells;
    cells.reserve(64);
    for (int row = 0; row < 64; ++row) {
        cells.emplace_back(Position{0.0, row * 0.125});
    }
    return GridMap(0.125, 64, 64, Position{0.0, 0.0}, cells);
}

// 真实车辆外圆模型（outer_row_num=1，3 个外圆，r_outer≈1.1505）
VehicleFootprintModel BuildFootprintModel(const VehicleParams& params) {
    return VehicleFootprintModel(params, 233, 2, 1);
}

// 与车辆物理参数一致的运动学配置（轴距 2.7 m）
MincoConfig MakeKinematicsConfig() {
    MincoConfig config;
    config.wheelbase = 2.7;
    return config;
}

// 跑通 分段器 → 预处理器 的公共前置链路，产出主求解器的合法输入
MincoPreprocessorResult RunPreprocessor(
    const Path& path, const Eigen::Vector2d& start_position,
    std::vector<MincoManeuverEstimate>* estimates) {
    *estimates = MincoManeuverSegmenter(MincoConfig{}).segment(path);
    const MincoPreprocessor preprocessor(MakeKinematicsConfig());
    const MincoPreprocessorResult result =
        preprocessor.preprocess(*estimates, start_position);
    EXPECT_TRUE(result.success);
    return result;
}

// 测试合成无障碍直线场景的端到端收敛。
// 因为预处理初值的终点误差已在 1e-3 m 量级（远低于 0.05 m 收敛阈值），
// 主求解器必须在少数外层迭代内以双指标（位置 <= 0.05 m、朝向 <= 1.5°）
// 宣告收敛，且终点硬边界（零速零加速度）精确满足。
TEST(MincoSolverTest, StraightLineConvergesEndToEnd) {
    const GridMap grid_map = BuildEmptyGridMap();
    const ESDFMap esdf_map(grid_map);
    const VehicleParams veh_params(4.3, 1.8, 2.7, 0.6, 0.8);
    const VehicleFootprintModel footprint = BuildFootprintModel(veh_params);
    std::vector<MincoManeuverEstimate> estimates;
    const MincoPreprocessorResult pre_result =
        RunPreprocessor(BuildStraightPath(2.0), {0.0, 0.0}, &estimates);
    const MincoSolver solver(MakeKinematicsConfig());
    const MincoSolverResult result =
        solver.solve(estimates, pre_result, {0.0, 0.0}, esdf_map, footprint);
    EXPECT_TRUE(result.success);
    EXPECT_EQ(result.status, MincoSolverStatus::CONVERGED);
    EXPECT_LE(result.terminal_position_error, 0.05);
    EXPECT_LE(result.terminal_heading_error_deg, 1.5);
    EXPECT_GE(result.outer_iterations, 1);
    EXPECT_EQ(result.outer_iterations, static_cast<int>(result.history.size()));
    ASSERT_GT(result.trajectory.numSegments(), 0);
    // 终点硬边界：速度/角速度均为 0
    const Eigen::Vector2d end_rate =
        result.trajectory.evaluate(result.trajectory.totalDuration(), 1);
    EXPECT_NEAR(end_rate.x(), 0.0, 1e-9);
    EXPECT_NEAR(end_rate.y(), 0.0, 1e-9);
}

// 测试单次换挡两机动场景（全局 K(T) 链条含换挡航点）的端到端收敛。
// 因为换挡点 ṡ=0 在 MINCO 框架内只能以软惩罚近似，收敛后换挡点残余速度
// 必须被压到接近 0 的工程容差内，同时终点双指标满足收敛判据。
TEST(MincoSolverTest, GearShiftConvergesEndToEnd) {
    const GridMap grid_map = BuildEmptyGridMap();
    const ESDFMap esdf_map(grid_map);
    const VehicleParams veh_params(4.3, 1.8, 2.7, 0.6, 0.8);
    const VehicleFootprintModel footprint = BuildFootprintModel(veh_params);
    std::vector<MincoManeuverEstimate> estimates;
    const MincoPreprocessorResult pre_result =
        RunPreprocessor(BuildGearShiftPath(), {0.0, 0.0}, &estimates);
    ASSERT_EQ(estimates.size(), 2U);
    const MincoSolver solver(MakeKinematicsConfig());
    const MincoSolverResult result =
        solver.solve(estimates, pre_result, {0.0, 0.0}, esdf_map, footprint);
    EXPECT_TRUE(result.success);
    EXPECT_EQ(result.status, MincoSolverStatus::CONVERGED);
    EXPECT_LE(result.terminal_position_error, 0.05);
    EXPECT_LE(result.terminal_heading_error_deg, 1.5);
    // 换挡点（首个 Maneuver 的最后一段末端）残余速度检查
    const int cusp_index =
        static_cast<int>(estimates.front().segments.size()) - 1;
    const double cusp_speed =
        std::abs(result.trajectory.evaluateSegment(cusp_index, 1.0, 1).y());
    EXPECT_LT(cusp_speed, 0.05);
}

// 测试 ρ^0 自适应标定公式与手推值对拍。
// 标定公式（MINCO.md 2.5 节）：ρ^0 = clip(J_s' / max(||C_f||², ε_ρ),
// ρ_min, ρ_max)，其中 J_s' 与 C_f 均取首轮内层收敛后的值。把收敛阈值压到
// 数值上不可达的 1e-300 强制跑满外层迭代，用 history 第 0 轮记录手推期望值。
TEST(MincoSolverTest, RhoInitialCalibrationMatchesHandDerived) {
    const GridMap grid_map = BuildEmptyGridMap();
    const ESDFMap esdf_map(grid_map);
    const VehicleParams veh_params(4.3, 1.8, 2.7, 0.6, 0.8);
    const VehicleFootprintModel footprint = BuildFootprintModel(veh_params);
    std::vector<MincoManeuverEstimate> estimates;
    const MincoPreprocessorResult pre_result =
        RunPreprocessor(BuildStraightPath(2.0), {0.0, 0.0}, &estimates);
    MincoConfig config = MakeKinematicsConfig();
    config.terminal_position_tolerance = 1e-300;
    config.max_outer_iterations = 4;
    // 放大安全阀使标定的 ρ^0 落在温和量级（~J_s'/0.1），避免 ρ 指数递增
    // 过早把内层问题拉僵导致 L-BFGS 线搜索失败，保证跑满外层预算
    config.epsilon_rho = 0.1;
    const MincoSolver solver(config);
    const MincoSolverResult result =
        solver.solve(estimates, pre_result, {0.0, 0.0}, esdf_map, footprint);
    ASSERT_EQ(result.history.size(), 4U);
    const MincoOuterIterationRecord& round0 = result.history.front();
    const double expected_rho0 =
        std::min(std::max(round0.j_s_prime /
                              std::max(round0.terminal_violation.squaredNorm(),
                                       config.epsilon_rho),
                          config.rho_min),
                 config.rho_max);
    EXPECT_NEAR(result.rho_initial_calibrated, expected_rho0,
                1e-9 * std::max(1.0, std::abs(expected_rho0)));
    // 首轮更新：λ^1 = λ^0 + ρ^0·C_f^0，且 λ^0 = 0（从纯软惩罚启动）
    const MincoOuterIterationRecord& round1 = result.history[1];
    EXPECT_NEAR(round1.lambda_x,
                result.rho_initial_calibrated * round0.terminal_violation.x(),
                1e-12);
    EXPECT_NEAR(round1.lambda_y,
                result.rho_initial_calibrated * round0.terminal_violation.y(),
                1e-12);
}

// 测试乘子 λ 与惩罚权重 ρ 更新公式的逐轮单步验证（默认无门控、无条件递增）。
// λ^{k+1} = λ^k + ρ^k·C_f^k；ρ^{k+1} = min((1+γ)·ρ^k, ρ_max)。两条公式对
// k>=1 的相邻 history 记录必须严格成立（同一组双精度数参与的相同运算）。
TEST(MincoSolverTest, MultiplierAndRhoUpdatesMatchFormula) {
    const GridMap grid_map = BuildEmptyGridMap();
    const ESDFMap esdf_map(grid_map);
    const VehicleParams veh_params(4.3, 1.8, 2.7, 0.6, 0.8);
    const VehicleFootprintModel footprint = BuildFootprintModel(veh_params);
    std::vector<MincoManeuverEstimate> estimates;
    const MincoPreprocessorResult pre_result =
        RunPreprocessor(BuildStraightPath(2.0), {0.0, 0.0}, &estimates);
    MincoConfig config = MakeKinematicsConfig();
    config.terminal_position_tolerance = 1e-300;
    config.max_outer_iterations = 6;
    config.rho_increase_factor = 0.5;
    // 同 RhoInitialCalibrationMatchesHandDerived：温和 ρ  regime 保证跑满预算
    config.epsilon_rho = 0.1;
    const MincoSolver solver(config);
    const MincoSolverResult result =
        solver.solve(estimates, pre_result, {0.0, 0.0}, esdf_map, footprint);
    ASSERT_EQ(result.history.size(), 6U)
        << "status=" << static_cast<int>(result.status)
        << " pos_err=" << result.terminal_position_error
        << " head_err=" << result.terminal_heading_error_deg;
    // 第 1 轮的 ρ 由标定值无条件递增得到（默认无门控）
    EXPECT_NEAR(result.history[1].rho,
                std::min((1.0 + config.rho_increase_factor) *
                             result.rho_initial_calibrated,
                         config.rho_max),
                1e-9);
    for (std::size_t k = 2; k < result.history.size(); ++k) {
        const MincoOuterIterationRecord& prev = result.history[k - 1];
        const MincoOuterIterationRecord& curr = result.history[k];
        EXPECT_NEAR(curr.lambda_x,
                    prev.lambda_x + prev.rho * prev.terminal_violation.x(),
                    1e-9);
        EXPECT_NEAR(curr.lambda_y,
                    prev.lambda_y + prev.rho * prev.terminal_violation.y(),
                    1e-9);
        EXPECT_NEAR(curr.rho,
                    std::min((1.0 + config.rho_increase_factor) * prev.rho,
                             config.rho_max),
                    1e-9);
    }
}

// 测试充分下降门控（工程增补，默认关闭）启用时的 ρ 更新行为。
// 门控规则：仅当 ||C_f^k|| > κ·||C_f^{k-1}||（未充分下降）时提升 ρ，
// 否则保持 ρ 不变；首轮无上一轮可比较，无条件提升。λ 更新不受门控影响。
TEST(MincoSolverTest, RhoIncreaseGateHoldsWhenEnabled) {
    const GridMap grid_map = BuildEmptyGridMap();
    const ESDFMap esdf_map(grid_map);
    const VehicleParams veh_params(4.3, 1.8, 2.7, 0.6, 0.8);
    const VehicleFootprintModel footprint = BuildFootprintModel(veh_params);
    std::vector<MincoManeuverEstimate> estimates;
    const MincoPreprocessorResult pre_result =
        RunPreprocessor(BuildStraightPath(2.0), {0.0, 0.0}, &estimates);
    MincoConfig config = MakeKinematicsConfig();
    config.terminal_position_tolerance = 1e-300;
    config.max_outer_iterations = 6;
    config.use_rho_increase_gate = true;
    config.rho_gate_kappa = 0.9;
    // 同 RhoInitialCalibrationMatchesHandDerived：温和 ρ regime 保证跑满预算
    config.epsilon_rho = 0.1;
    const MincoSolver solver(config);
    const MincoSolverResult result =
        solver.solve(estimates, pre_result, {0.0, 0.0}, esdf_map, footprint);
    ASSERT_EQ(result.history.size(), 6U)
        << "status=" << static_cast<int>(result.status)
        << " pos_err=" << result.terminal_position_error
        << " head_err=" << result.terminal_heading_error_deg;
    // 首轮无条件提升（从标定值出发）
    EXPECT_NEAR(result.history[1].rho,
                std::min((1.0 + config.rho_increase_factor) *
                             result.rho_initial_calibrated,
                         config.rho_max),
                1e-9);
    for (std::size_t k = 2; k < result.history.size(); ++k) {
        const double violation_curr =
            result.history[k - 1].terminal_violation.norm();
        const double violation_prev =
            result.history[k - 2].terminal_violation.norm();
        const double rho_before = result.history[k - 1].rho;
        const bool expect_increase =
            violation_curr > config.rho_gate_kappa * violation_prev;
        const double expected_rho =
            expect_increase
                ? std::min((1.0 + config.rho_increase_factor) * rho_before,
                           config.rho_max)
                : rho_before;
        EXPECT_NEAR(result.history[k].rho, expected_rho, 1e-9) << "轮次 " << k;
        // λ 更新不受门控影响，始终按 λ + ρ·C_f 执行
        EXPECT_NEAR(
            result.history[k].lambda_x,
            result.history[k - 1].lambda_x +
                rho_before * result.history[k - 1].terminal_violation.x(),
            1e-9);
    }
}

// 测试达到最大外层迭代次数仍未收敛时的显式失败兜底路径。
// 收敛阈值压到 1e-300（数值上不可达）且只给 1 轮外层预算，求解器必须以
// 明确失败状态返回（success=false、status=MAX_OUTER_ITERATIONS），
// 同时保留最后一次内层解的轨迹与指标供诊断，不得抛出或未定义行为。
TEST(MincoSolverTest, MaxOuterIterationsReturnsExplicitFailure) {
    const GridMap grid_map = BuildEmptyGridMap();
    const ESDFMap esdf_map(grid_map);
    const VehicleParams veh_params(4.3, 1.8, 2.7, 0.6, 0.8);
    const VehicleFootprintModel footprint = BuildFootprintModel(veh_params);
    std::vector<MincoManeuverEstimate> estimates;
    const MincoPreprocessorResult pre_result =
        RunPreprocessor(BuildStraightPath(2.0), {0.0, 0.0}, &estimates);
    MincoConfig config = MakeKinematicsConfig();
    config.terminal_position_tolerance = 1e-300;
    config.max_outer_iterations = 1;
    const MincoSolver solver(config);
    MincoSolverResult result;
    EXPECT_NO_THROW(result = solver.solve(estimates, pre_result, {0.0, 0.0},
                                          esdf_map, footprint));
    EXPECT_FALSE(result.success);
    EXPECT_EQ(result.status, MincoSolverStatus::MAX_OUTER_ITERATIONS);
    EXPECT_EQ(result.outer_iterations, 1);
    EXPECT_GT(result.terminal_position_error,
              config.terminal_position_tolerance);
    EXPECT_GT(result.trajectory.numSegments(), 0);
}

// 测试含 MincoEsdfPenalty 的简单有障碍场景：收敛轨迹不侵入 margin_safe 边界。
// 路径与直墙平行且初始间距 1.2 m 落在舒适缓冲带内（r+margin_safe≈1.17 <
// 1.2 < r+margin_comf≈1.25），舒适惩罚必须处于激活状态（否则场景没有验证
// 价值）；收敛后沿轨迹密集采样全部外圆，最小安全余量 d_esdf - r_outer
// 不得小于 margin_safe。
// L8.2 契约下第 0/63 行亦为占据边界圈：路径 y 区间取 [1.7, 3.6]，使全部
// 外圆到下边界圈的距离（>= 1.7-0.09 ≈ 1.62 > 1.2 = 墙距）与到上边界圈的
// 距离（>= 8-0.125-3.6-2.79 ≈ 1.49 > 1.25 = r+margin_comf）都足够远，
// 轨迹沿线场仍由墙面精确主导（与 L8 前场景物理一致），边界圈不参与。
TEST(MincoSolverTest, ObstacleSceneRespectsSafeMargin) {
    const GridMap grid_map = BuildWallGridMap();
    const ESDFMap esdf_map(grid_map);
    const VehicleParams veh_params(4.3, 1.8, 2.7, 0.6, 0.8);
    const VehicleFootprintModel footprint = BuildFootprintModel(veh_params);
    const double wall_x = 1.2;
    std::vector<MincoManeuverEstimate> estimates;
    const MincoPreprocessorResult pre_result =
        RunPreprocessor(BuildWallParallelPath(wall_x, 1.7, 3.6, PI / 2.0),
                        {wall_x, 1.7}, &estimates);
    const MincoSolver solver(MakeKinematicsConfig());
    const MincoSolverResult result =
        solver.solve(estimates, pre_result, {wall_x, 1.7}, esdf_map, footprint);
    ASSERT_TRUE(result.success);
    ASSERT_GE(result.history.size(), 1U);
    // 舒适惩罚在首轮必须处于激活状态（证明场景确实在惩罚带内）
    EXPECT_GT(result.history.front().esdf_penalty, 0.0);
    // 独立验证：以梯形数值积分还原世界坐标（不信任求解器内部积分），
    // 密集采样全部外圆计算最小安全余量
    const std::vector<Eigen::Vector2d> local_centers =
        vehicle_circle_geometry::ExtractLocalCircleCenters(footprint,
                                                           CircleType::OUTER);
    const double outer_radius = footprint.getOuterRadius();
    constexpr int kSamplesPerSegment = 64;
    double min_clearance = std::numeric_limits<double>::infinity();
    Eigen::Vector2d position(wall_x, 1.7);
    for (int i = 0; i < result.trajectory.numSegments(); ++i) {
        const double duration_i = result.trajectory.duration(i);
        Eigen::Vector2d prev_direction(
            std::cos(result.trajectory.evaluateSegment(i, 0.0, 0).x()),
            std::sin(result.trajectory.evaluateSegment(i, 0.0, 0).x()));
        double prev_s_dot = result.trajectory.evaluateSegment(i, 0.0, 1).y();
        for (int j = 1; j <= kSamplesPerSegment; ++j) {
            const double local_time = duration_i * j / kSamplesPerSegment;
            const double theta =
                result.trajectory.evaluateSegment(i, local_time, 0).x();
            const double s_dot =
                result.trajectory.evaluateSegment(i, local_time, 1).y();
            const Eigen::Vector2d direction(std::cos(theta), std::sin(theta));
            position += 0.5 * duration_i / kSamplesPerSegment *
                        (prev_s_dot * prev_direction + s_dot * direction);
            for (const auto& local : local_centers) {
                const Eigen::Vector2d center =
                    position + Eigen::Vector2d(std::cos(theta) * local.x() -
                                                   std::sin(theta) * local.y(),
                                               std::sin(theta) * local.x() +
                                                   std::cos(theta) * local.y());
                min_clearance = std::min(
                    min_clearance,
                    esdf_map.getDist(center.x(), center.y()) - outer_radius);
            }
            prev_direction = direction;
            prev_s_dot = s_dot;
        }
    }
    EXPECT_GE(min_clearance, MincoConfig{}.margin_safe - 1e-9);
}

// 测试解析梯度与中心差分数值梯度对拍。
// 内层目标的梯度链在预处理 J_pre 的基础上新增三个独立环节：跃度闭式二次型
// （含对 T 的 -5/T 显式依赖）、ESDF 逐节点后缀反传（含梯形权重 ∝T 的显式
// 时长梯度）、PHR-ALM 终端项（非零 λ/ρ），任何一环出错都会被有限差分放大；
// 手工构造含换挡与曲率的贴墙场景使 ESDF/物理/换挡惩罚同时处于激活区。
// L8.2 契约下第 0/63 行成为占据边界圈，墙角对角线附近出现「墙面 vs 边界圈」
// 中轴折点：折点两侧各一格内插值梯度场与距离场梯度不一致（插值伪影），
// 1e-6 级对拍必挂。全部 y 坐标较 L8 前整体上移 1.0 m，使所有辛普森节点的
// 外圆圆心都落在墙面主导的精确线性区（cy 比 cx 大 0.3 以上），边界圈不参与。
TEST(MincoSolverTest, AnalyticGradientMatchesFiniteDifference) {
    const GridMap grid_map = BuildWallGridMap();
    const ESDFMap esdf_map(grid_map);
    const VehicleParams veh_params(4.3, 1.8, 2.7, 0.6, 0.8);
    const VehicleFootprintModel footprint = BuildFootprintModel(veh_params);
    // 前进 2 段 + 后退 1 段，贴近直墙（全部外圆在 x∈[0.5, 1.3] 惩罚激活区）。
    // 段时长取 1.4~1.8 s：跃度闭式项 Q/T⁵ 的量级随 T⁻⁵ 急剧放大，过短的段
    // 时长会把代价地形拉得过陡，有限差分的截断误差随之超出 1e-6 验收线
    std::vector<MincoManeuverEstimate> estimates(2);
    estimates[0].direction = Direction::FORWARD;
    estimates[0].start_theta = PI / 2.0 - 0.15;
    estimates[0].start_arc_length = 0.0;
    estimates[0].segments = {
        {{1.05, 2.4}, PI / 2.0 - 0.05, 0.5, 1.6},
        {{1.10, 3.2}, PI / 2.0 + 0.10, 1.0, 1.4},
    };
    estimates[1].direction = Direction::BACKWARD;
    estimates[1].start_theta = PI / 2.0 + 0.10;
    estimates[1].start_arc_length = 1.0;
    estimates[1].segments = {
        {{1.15, 2.8}, PI / 2.0 + 0.05, 0.6, 1.8},
    };
    // 伪造与估计结构一致的预处理输出作为初值（绕过前置链路，隔离被测对象）
    MincoPreprocessorResult pre_result;
    pre_result.success = true;
    pre_result.waypoints = {{PI / 2.0 - 0.05, 0.5}, {PI / 2.0 + 0.10, 1.0}};
    pre_result.durations = {1.6, 1.4, 1.8};
    pre_result.final_arc_length = 0.6;
    // 压低物理上限使运动学惩罚激活；权重取适中量级抑制差分舍入误差
    MincoConfig config = MakeKinematicsConfig();
    config.max_velocity = 0.25;
    config.max_acceleration = 0.3;
    config.max_steer_angle = 0.5;
    config.max_steer_rate = 0.15;
    config.solver_weight_velocity = 10.0;
    config.solver_weight_acceleration = 10.0;
    config.solver_weight_steer_angle = 10.0;
    config.solver_weight_steer_rate = 10.0;
    config.solver_weight_gear_cusp = 10.0;
    config.solver_weight_duration_balance = 2.0;
    config.solver_epsilon_time = 0.05;
    const MincoSolverTestAccessor solver(config);
    const MincoSolverProblem problem =
        solver.buildProblem(estimates, pre_result, {1.05, 1.6});
    ASSERT_EQ(problem.numSegments(), 3);
    ASSERT_EQ(problem.variableCount(), 8);
    const MincoEsdfPenalty penalty(esdf_map, footprint, MincoConfig{});
    // 非零乘子/权重，使 MINCO 终端项的值与梯度都处于激活区
    const MincoMultiplierState multipliers{0.4, -0.3, 37.0};
    const Eigen::VectorXd x0 = problem.initialGuess();
    Eigen::VectorXd analytic(x0.size());
    MincoCostBreakdown breakdown;
    solver.evaluateCostAndGradient(problem, penalty, multipliers, x0, &analytic,
                                   &breakdown);
    EXPECT_GT(breakdown.esdf_penalty, 0.0);
    EXPECT_GT(breakdown.minco_terminal, 0.0);
    for (int k = 0; k < x0.size(); ++k) {
        const double step = 1e-6 * std::max(1.0, std::abs(x0[k]));
        Eigen::VectorXd x_plus = x0;
        Eigen::VectorXd x_minus = x0;
        x_plus[k] += step;
        x_minus[k] -= step;
        Eigen::VectorXd grad_scratch(x0.size());
        MincoCostBreakdown scratch;
        const double f_plus = solver.evaluateCostAndGradient(
            problem, penalty, multipliers, x_plus, &grad_scratch, &scratch);
        const double f_minus = solver.evaluateCostAndGradient(
            problem, penalty, multipliers, x_minus, &grad_scratch, &scratch);
        const double numeric = (f_plus - f_minus) / (2.0 * step);
        EXPECT_NEAR(
            analytic[k], numeric,
            1e-6 * std::max({1.0, std::abs(analytic[k]), std::abs(numeric)}))
            << "决策变量下标 " << k;
    }
}

// 测试非法输入的异常反馈。
// 主求解器的装配同时依赖初值估计与预处理结果两套数据（段数一致、字段
// 有限、时长为正），任何结构性不一致或非有限值都必须在装配期以标准异常
// 明确拒绝，而不是带着垃圾数据进入 L-BFGS。
TEST(MincoSolverTest, InvalidInputsThrow) {
    const GridMap grid_map = BuildEmptyGridMap();
    const ESDFMap esdf_map(grid_map);
    const VehicleParams veh_params(4.3, 1.8, 2.7, 0.6, 0.8);
    const VehicleFootprintModel footprint = BuildFootprintModel(veh_params);
    std::vector<MincoManeuverEstimate> estimates;
    const MincoPreprocessorResult pre_result =
        RunPreprocessor(BuildStraightPath(2.0), {0.0, 0.0}, &estimates);
    const MincoSolver solver(MincoConfig{});
    // 空估计
    EXPECT_THROW(solver.solve({}, pre_result, {0.0, 0.0}, esdf_map, footprint),
                 std::invalid_argument);
    // 非有限起点世界坐标
    EXPECT_THROW(solver.solve(estimates, pre_result,
                              {std::numeric_limits<double>::quiet_NaN(), 0.0},
                              esdf_map, footprint),
                 std::invalid_argument);
    // 预处理结果段数与估计不一致
    auto mismatched_durations = pre_result;
    mismatched_durations.durations.pop_back();
    EXPECT_THROW(solver.solve(estimates, mismatched_durations, {0.0, 0.0},
                              esdf_map, footprint),
                 std::invalid_argument);
    auto mismatched_waypoints = pre_result;
    mismatched_waypoints.waypoints.pop_back();
    EXPECT_THROW(solver.solve(estimates, mismatched_waypoints, {0.0, 0.0},
                              esdf_map, footprint),
                 std::invalid_argument);
    // 预处理结果含非正时长
    auto bad_duration = pre_result;
    bad_duration.durations[0] = 0.0;
    EXPECT_THROW(
        solver.solve(estimates, bad_duration, {0.0, 0.0}, esdf_map, footprint),
        std::invalid_argument);
    // 预处理结果含非有限航点
    auto bad_waypoint = pre_result;
    bad_waypoint.waypoints[0].x() = std::numeric_limits<double>::quiet_NaN();
    EXPECT_THROW(
        solver.solve(estimates, bad_waypoint, {0.0, 0.0}, esdf_map, footprint),
        std::invalid_argument);
    // 目标位姿（末端估计）非有限
    auto bad_target = estimates;
    bad_target.back().segments.back().desired_position.x() =
        std::numeric_limits<double>::infinity();
    EXPECT_THROW(
        solver.solve(bad_target, pre_result, {0.0, 0.0}, esdf_map, footprint),
        std::invalid_argument);
}

// 测试非法配置的构造期校验。
// 收敛判据、外层预算、ρ 标定/更新参数、门控参数、代价权重与 L-BFGS
// 参数直接决定双层循环行为，非法取值必须在构造期显式拒绝。
TEST(MincoSolverTest, InvalidConfigThrows) {
    EXPECT_NO_THROW(const MincoSolver valid_solver(MincoConfig{}));
    MincoConfig config;
    config.terminal_position_tolerance = 0.0;
    EXPECT_THROW(const MincoSolver s(config), std::invalid_argument);
    config = {};
    config.max_outer_iterations = 0;
    EXPECT_THROW(const MincoSolver s(config), std::invalid_argument);
    config = {};
    config.rho_min = 2.0;
    config.rho_max = 1.0;
    EXPECT_THROW(const MincoSolver s(config), std::invalid_argument);
    config = {};
    config.epsilon_rho = 0.0;
    EXPECT_THROW(const MincoSolver s(config), std::invalid_argument);
    config = {};
    config.first_round_rho = -1.0;
    EXPECT_THROW(const MincoSolver s(config), std::invalid_argument);
    config = {};
    config.rho_increase_factor = -0.5;
    EXPECT_THROW(const MincoSolver s(config), std::invalid_argument);
    config = {};
    config.use_rho_increase_gate = true;
    config.rho_gate_kappa = 1.5;
    EXPECT_THROW(const MincoSolver s(config), std::invalid_argument);
    config = {};
    config.weight_jerk_s = -1.0;
    EXPECT_THROW(const MincoSolver s(config), std::invalid_argument);
    config = {};
    config.solver_simpson_subintervals = 3;
    EXPECT_THROW(const MincoSolver s(config), std::invalid_argument);
    config = {};
    config.solver_lbfgs_max_iterations = 0;
    EXPECT_THROW(const MincoSolver s(config), std::invalid_argument);
}

// 测试 config() 只读访问器返回构造时的配置值。
// 因为同模块其它类均提供该访问器供下游读取配置，返回值必须与构造入参
// 逐字段一致。
TEST(MincoSolverTest, ConfigAccessorReturnsConstructionValues) {
    MincoConfig config;
    config.terminal_position_tolerance = 0.02;
    config.max_outer_iterations = 7;
    config.rho_increase_factor = 0.3;
    const MincoSolver solver(config);
    EXPECT_DOUBLE_EQ(solver.config().terminal_position_tolerance, 0.02);
    EXPECT_EQ(solver.config().max_outer_iterations, 7);
    EXPECT_DOUBLE_EQ(solver.config().rho_increase_factor, 0.3);
    EXPECT_DOUBLE_EQ(solver.config().rho_max, 1e6);
}

}  // namespace
}  // namespace apa_post_processor
