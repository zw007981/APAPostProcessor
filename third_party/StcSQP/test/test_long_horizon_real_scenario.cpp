#include <gtest/gtest.h>

#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <optional>
#include <regex>
#include <string>
#include <unistd.h>
#include <vector>

#include "constraints/convex_corridor_constraint.h"
#include "costs/quadratic_tracking.h"
#include "generated/corridor.h"
#include "math/math_util.hpp"
#include "models/bicycle_model_delta.h"
#include "ocp/multi_stage_ocp.h"
#include "problem_updater.h"
#include "qp/hpipm_solver.h"
#include "simple_parking_map.h"
#include "sqp/sqp_algorithm.h"
#include "util/trajectory.h"

using namespace stc_SQP;

namespace {
struct PathPoint {
    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;
};

struct PathRun {
    int start_idx = 0;
    int end_idx = 0;
    double v_sign = 1.0;
};

std::string executableDirectory()
{
    char buf[PATH_MAX];
    const ssize_t len = readlink("/proc/self/exe", buf, sizeof(buf) - 1);
    if (len <= 0) {
        return ".";
    }
    buf[len] = '\0';
    std::string exe(buf);
    const auto pos = exe.find_last_of('/');
    if (pos == std::string::npos) {
        return ".";
    }
    return exe.substr(0, pos);
}

std::optional<double> extractJsonNumber(const std::string& line, const std::string& key)
{
    const std::string pattern = "\"" + key + "\"\\s*:\\s*([-+]?\\d*\\.?\\d+(?:[eE][-+]?\\d+)?)";
    std::regex re(pattern);
    std::smatch match;
    if (std::regex_search(line, match, re)) {
        return std::stod(match[1].str());
    }
    return std::nullopt;
}

std::vector<PathPoint> loadData3InitialPath()
{
    const std::string path = executableDirectory() + "/../../data/data3.json";
    std::ifstream file(path);
    if (!file.is_open()) {
        std::cerr << "Failed to open data3.json: " << path << std::endl;
        return {};
    }

    std::vector<PathPoint> points;
    std::string line;
    std::optional<double> cur_x, cur_y, cur_theta;
    while (std::getline(file, line)) {
        if (auto v = extractJsonNumber(line, "x")) {
            cur_x = v;
        } else if (auto v = extractJsonNumber(line, "y")) {
            cur_y = v;
        } else if (auto v = extractJsonNumber(line, "theta")) {
            cur_theta = v;
            if (cur_x && cur_y) {
                points.push_back({ *cur_x, *cur_y, *cur_theta });
            }
            cur_x = cur_y = cur_theta = std::nullopt;
        }
    }
    return points;
}

std::vector<PathRun> splitPathByDirection(const std::vector<PathPoint>& points)
{
    std::vector<PathRun> runs;
    double prev_sign = 0.0;
    int run_start = 0;
    for (int i = 0; i + 1 < static_cast<int>(points.size()); ++i) {
        const double dx = points[i + 1].x - points[i].x;
        const double dy = points[i + 1].y - points[i].y;
        const double hx = std::cos(points[i].theta), hy = std::sin(points[i].theta);
        const double sign = (dx * hx + dy * hy >= 0.0) ? 1.0 : -1.0;
        if (prev_sign != 0.0 && sign != prev_sign) {
            runs.push_back({ run_start, i, prev_sign });
            run_start = i;
        }
        prev_sign = sign;
    }
    runs.push_back({ run_start, static_cast<int>(points.size()) - 1, prev_sign });
    return runs;
}

struct RealScenario {
    MultiStageOCP ocp;
    Trajectory init_guess;
};

RealScenario buildData3RealMultiSegmentScenario(const std::vector<PathPoint>& points_in,
    int max_steps_per_run = -1, int max_runs = -1)
{
    std::vector<PathPoint> points = points_in;
    const double origin_x = points_in.front().x, origin_y = points_in.front().y;
    for (auto& p : points) {
        p.x -= origin_x;
        p.y -= origin_y;
    }

    RealScenario scenario;
    const int nx = 5, nu = 2;
    const double dt = 0.1, wheelbase = 2.8, v_max = 2.5;
    const double kBoundaryVelocitySlack = 0.02;
    auto dynamics = std::make_shared<BicycleModelDelta>(wheelbase);
    auto runs = splitPathByDirection(points);
    if (max_runs > 0 && static_cast<int>(runs.size()) > max_runs) {
        runs.resize(max_runs);
    }
    if (max_steps_per_run > 0) {
        for (auto& run : runs) {
            run.end_idx = std::min(run.end_idx, run.start_idx + max_steps_per_run);
        }
    }

    scenario.init_guess.x.push_back(
        (Vector(nx) << points[runs.front().start_idx].x, points[runs.front().start_idx].y,
            points[runs.front().start_idx].theta, 0.0, 0.0)
            .finished());

    for (size_t r = 0; r < runs.size(); ++r) {
        const auto& run = runs[r];
        const int n = run.end_idx - run.start_idx;
        double arc_length = 0.0;
        for (int i = run.start_idx; i < run.end_idx; ++i) {
            arc_length += std::hypot(
                points[i + 1].x - points[i].x, points[i + 1].y - points[i].y);
        }
        const double dtheta = math_util::NormalizeAngle(
            points[run.end_idx].theta - points[run.start_idx].theta);
        const double kappa_avg = (arc_length > 1e-6) ? dtheta / arc_length : 0.0;
        const double delta_avg = std::atan(kappa_avg * wheelbase);
        const double total_time = n * dt;
        const double v_peak = (total_time > 1e-6) ? 2.0 * arc_length / total_time : 0.0;

        const int half = n / 2;
        const int global_start = static_cast<int>(scenario.init_guess.x.size()) - 1;
        for (int i = 0; i < n; ++i) {
            const double a = (i < half)
                ? run.v_sign * v_peak / (std::max(1, half) * dt)
                : -run.v_sign * v_peak / (std::max(1, n - half) * dt);
            Vector& x_cur = scenario.init_guess.x[global_start + i];
            x_cur(4) = delta_avg;
            Vector u(nu);
            u << a, 0.0;
            scenario.init_guess.u.push_back(u);
            Vector x_next;
            dynamics->discretize(x_cur, u, dt, run.v_sign, x_next);
            scenario.init_guess.x.push_back(x_next);
        }
    }

    int global_end = 0;
    for (size_t r = 0; r < runs.size(); ++r) {
        const auto& run = runs[r];
        const int n = run.end_idx - run.start_idx;
        global_end += n;

        StageSegment seg;
        seg.dynamics = dynamics;
        seg.N = n;
        seg.dt = dt;
        seg.v_sign = run.v_sign;
        seg.x_min = Vector::Constant(nx, -1e4);
        seg.x_max = Vector::Constant(nx, 1e4);
        seg.x_min(3) = (run.v_sign > 0.0) ? -kBoundaryVelocitySlack : -v_max;
        seg.x_max(3) = (run.v_sign > 0.0) ? v_max : kBoundaryVelocitySlack;
        seg.u_min = Vector::Constant(nu, -3.0);
        seg.u_max = Vector::Constant(nu, 3.0);
        const Vector x_ref = scenario.init_guess.x[global_end];
        seg.cost = std::make_shared<QuadraticTrackingCost>(x_ref,
            Matrix::Identity(nx, nx) * 1e-2, Matrix::Identity(nu, nu) * 1e-2, /*theta_idx=*/2);
        seg.constraints.push_back(std::make_shared<ConvexCorridorConstraint>(nu));
        scenario.ocp.addSegment(seg);
    }
    return scenario;
}

// 计算给定轨迹在当前 OCP 上的总二次跟踪代价（每段按 stage cost k=0..N-1 求和）。
double computeTotalStageCost(const MultiStageOCP& ocp, const Trajectory& traj)
{
    double total = 0.0;
    int global_k = 0;
    for (const auto& segment : ocp.segments()) {
        const auto* cost = dynamic_cast<const QuadraticTrackingCost*>(segment.cost.get());
        if (!cost) {
            continue;
        }
        for (int i = 0; i < segment.N; ++i) {
            double stage_cost = 0.0;
            cost->evaluate(traj.x[global_k + i], traj.u[global_k + i], stage_cost);
            total += stage_cost;
        }
        global_k += segment.N;
    }
    return total;
}
} // namespace

TEST(LongHorizonRealScenario, Data3FirstThreeRunsConvergeWithDocumentedRecipe)
{
    // 测试目的：将 bench 中记录的长 N 真实场景收敛配方迁移为确定性 CI 测试，
    //          验证局部坐标 + HPIPM 容差 1e-4 + 关闭线搜索 + 50 次迭代 + Partial Condensing
    //          对 data3.json 前 3 段/约 150 步真实 BicycleModelDelta+ConvexCorridorConstraint
    //          换挡场景可稳定收敛。
    // 流程：加载 data3.json，切分方向并截断到前 3 段/每段不超过 50 步，构造局部坐标场景，
    //      注入空地图走廊，用 HPIPM 求解，检查解的有限性、约束满足与代价下降。
    // 预期效果：solve 返回 true；解状态全部有限；段边界速度接近 0；最终代价低于初始猜测。
    const auto points = loadData3InitialPath();
    if (points.empty()) {
        GTEST_SKIP() << "data3.json not available, skipping long-horizon real scenario test";
    }

    const int kMaxRuns = 3;
    const int kMaxStepsPerRun = 50;
    RealScenario scenario
        = buildData3RealMultiSegmentScenario(points, kMaxStepsPerRun, kMaxRuns);
    ASSERT_TRUE(scenario.ocp.hasGearShift());

    SimpleParkingMap map;
    UpdaterConfig config;
    config.selection_radius = 200.0;
    config.max_step_displacement = 0.5;
    config.safety_margin = 0.1;
    config.top_k = 10;
    ProblemUpdater updater(config);
    updater.updateOcp(scenario.init_guess, map, scenario.ocp);

    const int N = scenario.ocp.totalSteps();
    const int cond_N = (N + 10 - 1) / 10;
    auto qp_solver = std::make_unique<HPIPMQPSolver>(
        N, scenario.ocp.nx(), scenario.ocp.nu(), scenario.ocp.nx(), scenario.ocp.nu(),
        CORRIDOR_G_DIM, 0, cond_N);
    qp_solver->setTolerance(1e-4);
    SQPSolver solver(std::move(qp_solver));
    solver.options().use_line_search = false;
    solver.options().max_iter = 50;

    Trajectory solution;
    ASSERT_TRUE(solver.solve(scenario.ocp, scenario.init_guess, solution));

    // 解状态全部有限，速度始终在给定 box bound 内。
    int global_k = 0;
    for (const auto& segment : scenario.ocp.segments()) {
        for (int i = 0; i <= segment.N; ++i) {
            const Vector& x = solution.x[global_k + i];
            ASSERT_TRUE(x.allFinite()) << "state at global_k=" << global_k + i << " is non-finite";
            EXPECT_GE(x(3), segment.x_min(3) - 1e-6);
            EXPECT_LE(x(3), segment.x_max(3) + 1e-6);
        }
        global_k += segment.N;
    }

    // 段边界速度接近 0（换挡点）。
    global_k = 0;
    for (const auto& segment : scenario.ocp.segments()) {
        global_k += segment.N;
        if (global_k < static_cast<int>(solution.x.size()) - 1) {
            EXPECT_NEAR(solution.x[global_k](3), 0.0, 5e-2)
                << "boundary velocity at segment transition " << global_k << " should be near zero";
        }
    }

    // 求解后的总阶段代价应低于初始猜测（至少没有显著恶化，允许数值噪声 1%）。
    const double initial_cost = computeTotalStageCost(scenario.ocp, scenario.init_guess);
    const double final_cost = computeTotalStageCost(scenario.ocp, solution);
    EXPECT_LT(final_cost, initial_cost * 1.01 + 1e-6);
}
