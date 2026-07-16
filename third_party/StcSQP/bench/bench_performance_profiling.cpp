// 迁移动机：墙钟计时断言在 CI 高负载下易 flaky，不适合作为硬性单元测试；
//          这里统一改为 Google Benchmark，稳态计时由框架多次采样，
//          原有的串行/并行、策略一致性校验降级为“一次性检查”，不一致时
//          通过 state.SkipWithError() 让对应基准显式失败，保留回归价值。
#include <algorithm>
#include <climits>
#include <cmath>
#include <fstream>
#include <iostream>
#include <memory>
#include <optional>
#include <regex>
#include <string>
#include <vector>

#include <unistd.h>

#include <benchmark/benchmark.h>

#include "constraints/constraint.hpp"
#include "constraints/convex_corridor_constraint.h"
#include "costs/circle_footprint_esdf_penalty_cost.h"
#include "costs/composite_cost.h"
#include "costs/quadratic_tracking.h"
#include "core/vehicle_geometry.h"
#include "examples/parking/circle_obstacle_esdf_map.h"
#include "generated/corridor.h"
#include "math/math_util.hpp"
#include "models/bicycle_model_delta.h"
#include "models/dynamical_system.h"
#include "models/dynamical_system.h"
#include "ocp/multi_stage_ocp.h"
#include "problem_updater.h"
#include "qp/hpipm_solver.h"
#include "simple_parking_map.h"
#include "sqp/sqp_algorithm.h"
#include "strategies/auto_adaptive_strategy.h"
#include "strategies/manual_hierarchical_strategy.h"
#include "util/trajectory.h"

using namespace stc_SQP;

namespace {
// ===================== 基准辅助：一般控制上界约束 g_i = u(idx) - u_max <= 0 =====================
class ControlUpperBoundConstraint : public Constraint {
public:
    ControlUpperBoundConstraint(int control_index, double u_max)
        : control_index_(control_index)
        , u_max_(u_max)
    {
    }
    int ng() const override { return 1; }
    void evaluate(const Vector& x, const Vector& u, const Vector& p, Vector& g) const override
    {
        (void)x;
        (void)p;
        g.resize(1);
        g(0) = u(control_index_) - u_max_;
    }
    void jacobian(const Vector& x, const Vector& u, const Vector& p, Matrix& Cx,
        Matrix& Cu) const override
    {
        (void)x;
        (void)u;
        (void)p;
        Cx.setZero(1, x.size());
        Cu.setZero(1, u.size());
        Cu(0, control_index_) = 1.0;
    }
    std::shared_ptr<Constraint> clone() const override
    {
        return std::make_shared<ControlUpperBoundConstraint>(control_index_, u_max_);
    }
private:
    int control_index_ = 0;
    double u_max_ = 0.0;
};

// ===================== 基准辅助：双积分器线性动力学 =====================
class DoubleIntegrator : public DynamicalSystem {
public:
    DoubleIntegrator(const Matrix& A, const Matrix& B)
        : A_(A)
        , B_(B)
    {
    }
    int nx() const override { return A_.rows(); }
    int nu() const override { return B_.cols(); }
    void evaluate(const Vector& x, const Vector& u, Vector& x_dot) const override
    {
        (void)x;
        (void)u;
        (void)x_dot;
    }
    void discretizeAndLinearize(const Vector& x, const Vector& u, double dt,
        double v_sign, Vector& x_next, Matrix& A, Matrix& B) const override
    {
        (void)dt;
        (void)v_sign;
        x_next = A_ * x + B_ * u;
        A = A_;
        B = B_;
    }

private:
    Matrix A_;
    Matrix B_;
};

// ===================== 基准辅助：简单 n 维线性积分器（用于 Corridor 端到端基准）=====================
class LinearIntegrator : public DynamicalSystem {
public:
    LinearIntegrator(int nx, int nu)
        : nx_(nx)
        , nu_(nu)
    {
    }
    int nx() const override { return nx_; }
    int nu() const override { return nu_; }
    void evaluate(const Vector&, const Vector&, Vector&) const override { }
    void discretizeAndLinearize(const Vector& x, const Vector& u, double dt,
        double v_sign, Vector& x_next, Matrix& A, Matrix& B) const override
    {
        (void)v_sign;
        A.setIdentity(nx_, nx_);
        B.setZero(nx_, nu_);
        const int nmin = std::min(nx_, nu_);
        for (int i = 0; i < nmin; ++i) {
            B(i, i) = dt;
        }
        x_next = A * x + B * u;
    }

private:
    int nx_ = 0;
    int nu_ = 0;
};

// ===================== 基准辅助：构造长步长 LQR 问题 =====================
MultiStageOCP makeLongLqrOcp(int N, double dt)
{
    const int nx = 2, nu = 1;
    Matrix A(nx, nx);
    A << 1.0, dt,
        0.0, 1.0;
    Matrix B(nx, nu);
    B << 0.0,
        dt;
    const Matrix Q = Matrix::Identity(nx, nx);
    const Matrix R = Matrix::Identity(nu, nu) * 0.1;
    const Vector x_ref = Vector::Zero(nx);
    const Vector x_min = Vector::Constant(nx, -1e3);
    const Vector x_max = Vector::Constant(nx, 1e3);
    const Vector u_min = Vector::Constant(nu, -1e3);
    const Vector u_max = Vector::Constant(nu, 1e3);

    StageSegment segment;
    segment.dynamics = std::make_shared<DoubleIntegrator>(A, B);
    segment.cost = std::make_shared<QuadraticTrackingCost>(x_ref, Q, R);
    segment.N = N;
    segment.dt = dt;
    segment.v_sign = 1.0;
    segment.x_min = x_min;
    segment.x_max = x_max;
    segment.u_min = u_min;
    segment.u_max = u_max;
    // 注意：本辅助函数返回纯 LQR，不含一般约束；需要一般约束的调用方自行添加。

    MultiStageOCP ocp;
    ocp.addSegment(segment);
    return ocp;
}

Trajectory makeInitialGuess(int N, int nx, int nu)
{
    Trajectory traj;
    traj.resize(N, nx, nu);
    for (auto& xk : traj.x) {
        xk.setZero();
    }
    for (auto& uk : traj.u) {
        uk.setZero();
    }
    return traj;
}

// 生成与动力学一致的初始猜测：从 x[0] 前向传播，避免线搜索在不可行猜测下失败。
void propagateInitialGuess(const MultiStageOCP& ocp, Trajectory& init_guess)
{
    const int N = ocp.totalSteps();
    for (int k = 0; k < N; ++k) {
        const auto& segment = ocp.segments()[0];
        segment.dynamics->discretize(
            init_guess.x[k], init_guess.u[k], segment.stepSize(k), segment.v_sign,
            init_guess.x[k + 1]);
    }
}

// 判断两条轨迹是否在给定容差内一致，供基准做一次性一致性校验。
bool trajectoriesClose(const Trajectory& a, const Trajectory& b, double tol)
{
    if (a.x.size() != b.x.size() || a.u.size() != b.u.size()) {
        return false;
    }
    for (size_t k = 0; k < a.x.size(); ++k) {
        if ((a.x[k] - b.x[k]).norm() >= tol) {
            return false;
        }
    }
    for (size_t k = 0; k < a.u.size(); ++k) {
        if ((a.u[k] - b.u[k]).norm() >= tol) {
            return false;
        }
    }
    return true;
}

// ===================== 基准辅助：data3.json 真实泊车初始路径加载 =====================
struct TrajectoryPoint {
    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;
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

std::vector<TrajectoryPoint> loadData3InitialPath()
{
    // 基准可执行文件位于 build/Release，项目 data 目录位于 ../../data
    const std::string path = executableDirectory() + "/data/data3.json";
    std::ifstream file(path);
    if (!file.is_open()) {
        std::cerr << "Failed to open data3.json: " << path << std::endl;
        return {};
    }

    std::vector<TrajectoryPoint> points;
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

// 3D 路径积分器：x = [x, y, theta]^T，u = [vx, vy, omega]^T，x_next = x + dt * u
class PathIntegrator : public DynamicalSystem {
public:
    PathIntegrator() = default;
    int nx() const override { return 3; }
    int nu() const override { return 3; }
    void evaluate(const Vector&, const Vector&, Vector&) const override { }
    void discretizeAndLinearize(const Vector& x, const Vector& u, double dt, double v_sign,
        Vector& x_next, Matrix& A, Matrix& B) const override
    {
        (void)v_sign;
        A.setIdentity(3, 3);
        B.setZero(3, 3);
        B(0, 0) = dt;
        B(1, 1) = dt;
        B(2, 2) = dt;
        x_next = A * x + B * u;
    }
};

MultiStageOCP buildData3TrackingOCP(const std::vector<TrajectoryPoint>& points)
{
    const int N = static_cast<int>(points.size()) - 1;
    const int nx = 3;
    const int nu = 3;

    StageSegment seg;
    seg.dynamics = std::make_shared<PathIntegrator>();
    seg.N = N;
    seg.dt = 0.1;
    seg.v_sign = 1.0;
    seg.x_min = Vector::Constant(nx, -1e3);
    seg.x_max = Vector::Constant(nx, 1e3);
    seg.u_min = Vector::Constant(nu, -1e3);
    seg.u_max = Vector::Constant(nu, 1e3);

    // 以终点作为参考，鼓励车辆沿 initial_path 到达目标姿态
    Vector x_ref(nx);
    x_ref << points.back().x, points.back().y, points.back().theta;
    Matrix Q = Matrix::Identity(nx, nx);
    Matrix R = Matrix::Identity(nu, nu) * 0.01;
    seg.cost = std::make_shared<QuadraticTrackingCost>(x_ref, Q, R, /*theta_idx=*/2);

    MultiStageOCP ocp;
    ocp.addSegment(seg);
    return ocp;
}

Trajectory makeData3InitialGuess(const std::vector<TrajectoryPoint>& points)
{
    const int N = static_cast<int>(points.size()) - 1;
    const int nx = 3;
    const int nu = 3;
    const double dt = 0.1;
    Trajectory traj;
    traj.resize(N, nx, nu);

    for (int k = 0; k <= N; ++k) {
        traj.x[k] << points[k].x, points[k].y, points[k].theta;
    }
    for (int k = 0; k < N; ++k) {
        const double dx = points[k + 1].x - points[k].x;
        const double dy = points[k + 1].y - points[k].y;
        const double dtheta = math_util::NormalizeAngle(points[k + 1].theta - points[k].theta);
        traj.u[k] << dx / dt, dy / dt, dtheta / dt;
    }
    return traj;
}

// 构造围绕原点的大正方形凸走廊参数（10 半空间），保证初始零状态在内部。
Vector makeCorridorParameters()
{
    Vector p = Vector::Zero(CORRIDOR_P_DIM);
    constexpr int N_HS = 10;
    const double bounds[N_HS][3] = {
        { 1.0, 0.0, 10.0 }, //  x <= 10
        { -1.0, 0.0, 10.0 }, // -x <= 10
        { 0.0, 1.0, 10.0 }, //  y <= 10
        { 0.0, -1.0, 10.0 }, // -y <= 10
        { 1.0, 1.0, 20.0 },
        { -1.0, 1.0, 20.0 },
        { 1.0, -1.0, 20.0 },
        { -1.0, -1.0, 20.0 },
        { 0.8, 0.6, 14.0 },
        { -0.6, 0.8, 14.0 },
    };
    for (int i = 0; i < N_HS; ++i) {
        p(15 + 2 * i + 0) = bounds[i][0];
        p(15 + 2 * i + 1) = bounds[i][1];
        p(15 + 2 * N_HS + i) = bounds[i][2];
    }
    return p;
}
} // namespace

// ===================== 基准：超长 N LQR + 一般约束，串行无凝聚 vs OpenMP+Partial Condensing =====================
// 目的：度量 N=10000 超长步长场景下两种配置的稳态求解时间，直观体现 Partial Condensing 收益。
static void BM_SerialNoCondensing_LqrN10000(benchmark::State& state)
{
    const int N = 10000;
    const double dt = 0.1;
    MultiStageOCP ocp = makeLongLqrOcp(N, dt);
    ocp.segments()[0].constraints.push_back(std::make_shared<ControlUpperBoundConstraint>(0, 0.5));
    const int nx = ocp.nx(), nu = ocp.nu();
    Trajectory init_guess = makeInitialGuess(N, nx, nu);
    init_guess.x[0] << 5.0, 0.0;
    propagateInitialGuess(ocp, init_guess);

    auto qp_solver = std::make_unique<HPIPMQPSolver>(N, nx, nu, nx, nu, 1, 0, -1);
    SQPSolver solver(std::move(qp_solver));
    solver.options().use_omp = false;
    solver.options().max_iter = 10;
    for (auto _ : state) {
        Trajectory sol;
        const bool ok = solver.solve(ocp, init_guess, sol);
        benchmark::DoNotOptimize(sol);
        if (!ok) {
            state.SkipWithError("Serial no-condensing solve failed");
            break;
        }
    }
}
BENCHMARK(BM_SerialNoCondensing_LqrN10000)->Unit(benchmark::kMillisecond)->Iterations(3);

static void BM_OpenMPPartialCondensing_LqrN10000(benchmark::State& state)
{
    const int N = 10000;
    const double dt = 0.1;
    MultiStageOCP ocp = makeLongLqrOcp(N, dt);
    ocp.segments()[0].constraints.push_back(std::make_shared<ControlUpperBoundConstraint>(0, 0.5));
    const int nx = ocp.nx(), nu = ocp.nu();
    Trajectory init_guess = makeInitialGuess(N, nx, nu);
    init_guess.x[0] << 5.0, 0.0;
    propagateInitialGuess(ocp, init_guess);

    const int block_size = 10;
    const int cond_N = (N + block_size - 1) / block_size;

    // 一次性一致性校验：并行+Partial Condensing 结果必须与串行无凝聚一致，
    // 排除 OpenMP 数据竞争或 Partial Condensing 实现错误。
    {
        Trajectory sol_serial, sol_parallel;
        auto serial_qp = std::make_unique<HPIPMQPSolver>(N, nx, nu, nx, nu, 1, 0, -1);
        SQPSolver serial(std::move(serial_qp));
        serial.options().use_omp = false;
        serial.options().max_iter = 10;
        auto parallel_qp = std::make_unique<HPIPMQPSolver>(N, nx, nu, nx, nu, 1, 0, cond_N);
        SQPSolver parallel(std::move(parallel_qp));
        parallel.options().use_omp = true;
        parallel.options().omp_parallel_threshold = 50;
        parallel.options().max_iter = 10;
        if (!serial.solve(ocp, init_guess, sol_serial)
            || !parallel.solve(ocp, init_guess, sol_parallel)
            || !trajectoriesClose(sol_serial, sol_parallel, 1e-8)) {
            state.SkipWithError("Parallel+Partial Condensing results differ from serial");
            return;
        }
    }

    auto qp_solver = std::make_unique<HPIPMQPSolver>(N, nx, nu, nx, nu, 1, 0, cond_N);
    SQPSolver solver(std::move(qp_solver));
    solver.options().use_omp = true;
    solver.options().omp_parallel_threshold = 50;
    solver.options().max_iter = 10;
    for (auto _ : state) {
        Trajectory sol;
        const bool ok = solver.solve(ocp, init_guess, sol);
        benchmark::DoNotOptimize(sol);
        if (!ok) {
            state.SkipWithError("Parallel solve failed");
            break;
        }
    }
}
BENCHMARK(BM_OpenMPPartialCondensing_LqrN10000)->Unit(benchmark::kMillisecond)->Iterations(3);

// ===================== 基准：含真实 CasADi 凸走廊约束的 OpenMP 并行 linearize =====================
// 目的：度量含 ConvexCorridorConstraint（真实 CasADi 工作区）的 OCP 在 OpenMP 下的求解时间，
//      并一次性校验并行与串行结果一致（排除 CasADi 工作区数据竞争）。
static void BM_OpenMPCorridorConstraint_N200(benchmark::State& state)
{
    const int N = 200;
    const int nx = CORRIDOR_NX;
    const int nu = 2;
    const double dt = 0.1;

    MultiStageOCP ocp;
    StageSegment segment;
    segment.dynamics = std::make_shared<LinearIntegrator>(nx, nu);
    segment.cost = std::make_shared<QuadraticTrackingCost>(
        Vector::Zero(nx), Matrix::Identity(nx, nx), Matrix::Identity(nu, nu) * 0.1);
    segment.N = N;
    segment.dt = dt;
    segment.v_sign = 1.0;
    segment.x_min = Vector::Constant(nx, -1e3);
    segment.x_max = Vector::Constant(nx, 1e3);
    segment.u_min = Vector::Constant(nu, -1e3);
    segment.u_max = Vector::Constant(nu, 1e3);
    const Vector p = makeCorridorParameters();
    segment.constraints.push_back(std::make_shared<ConvexCorridorConstraint>(nu));
    for (int i = 0; i < N; ++i) {
        segment.stage_params.push_back({ p });
    }
    ocp.addSegment(segment);
    const Trajectory init_guess = makeInitialGuess(N, nx, nu);

    // 一次性一致性校验：串行 vs OpenMP。
    {
        Trajectory sol_serial, sol_parallel;
        auto serial_qp = std::make_unique<HPIPMQPSolver>(N, nx, nu, nx, nu, CORRIDOR_G_DIM, 0, -1);
        SQPSolver serial(std::move(serial_qp));
        serial.options().use_omp = false;
        auto parallel_qp = std::make_unique<HPIPMQPSolver>(N, nx, nu, nx, nu, CORRIDOR_G_DIM, 0, -1);
        SQPSolver parallel(std::move(parallel_qp));
        parallel.options().use_omp = true;
        parallel.options().omp_parallel_threshold = 50;
        if (!serial.solve(ocp, init_guess, sol_serial)
            || !parallel.solve(ocp, init_guess, sol_parallel)
            || !trajectoriesClose(sol_serial, sol_parallel, 1e-8)) {
            state.SkipWithError("OpenMP and serial corridor constraint results differ");
            return;
        }
    }

    auto qp_solver = std::make_unique<HPIPMQPSolver>(N, nx, nu, nx, nu, CORRIDOR_G_DIM, 0, -1);
    SQPSolver solver(std::move(qp_solver));
    solver.options().use_omp = true;
    solver.options().omp_parallel_threshold = 50;
    for (auto _ : state) {
        Trajectory sol;
        const bool ok = solver.solve(ocp, init_guess, sol);
        benchmark::DoNotOptimize(sol);
        if (!ok) {
            state.SkipWithError("Corridor constraint parallel solve failed");
            break;
        }
    }
}
BENCHMARK(BM_OpenMPCorridorConstraint_N200)->Unit(benchmark::kMillisecond);

// ===================== 基准：AutoAdaptiveStrategy 长 N 自动选择 Partial Condensing =====================
static void BM_AutoAdaptiveStrategy_LongN200(benchmark::State& state)
{
    const int N = 200;
    const double dt = 0.1;
    MultiStageOCP ocp = makeLongLqrOcp(N, dt);
    const Trajectory init_guess = makeInitialGuess(N, ocp.nx(), ocp.nu());

    // 一次性一致性校验：策略结果应与显式串行求解一致。
    {
        Trajectory sol_adaptive, sol_serial;
        AutoAdaptiveStrategy strategy(ocp.nx(), ocp.nu(), 0, 0, 10, 50);
        auto qp_solver = std::make_unique<HPIPMQPSolver>(
            N, ocp.nx(), ocp.nu(), ocp.nx(), ocp.nu(), 0, 0, -1);
        SQPSolver serial_solver(std::move(qp_solver));
        serial_solver.options().use_omp = false;
        if (!strategy.solve(ocp, init_guess, sol_adaptive)
            || !serial_solver.solve(ocp, init_guess, sol_serial)
            || !trajectoriesClose(sol_adaptive, sol_serial, 1e-10)) {
            state.SkipWithError("AutoAdaptiveStrategy results differ from serial");
            return;
        }
    }

    AutoAdaptiveStrategy strategy(ocp.nx(), ocp.nu(), 0, 0, 10, 50);
    for (auto _ : state) {
        Trajectory sol;
        const bool ok = strategy.solve(ocp, init_guess, sol);
        benchmark::DoNotOptimize(sol);
        if (!ok) {
            state.SkipWithError("AutoAdaptiveStrategy solve failed");
            break;
        }
    }
}
BENCHMARK(BM_AutoAdaptiveStrategy_LongN200)->Unit(benchmark::kMillisecond);

// ===================== 基准：ManualHierarchicalStrategy 粗化+动力学一致插值 =====================
static void BM_ManualHierarchicalStrategy_N200(benchmark::State& state)
{
    const int N = 200;
    const double dt = 0.1;
    MultiStageOCP ocp = makeLongLqrOcp(N, dt);
    Trajectory init_guess = makeInitialGuess(N, ocp.nx(), ocp.nu());
    init_guess.x[0] << 5.0, 0.0;
    propagateInitialGuess(ocp, init_guess);

    // 一次性一致性校验：分层结果与 AutoAdaptive 直接求解一致。
    {
        Trajectory sol_hier, sol_auto;
        HierarchicalOptions opts;
        opts.coarse_n = 20;
        ManualHierarchicalStrategy strategy(ocp.nx(), ocp.nu(), 0, 0, opts);
        AutoAdaptiveStrategy auto_strategy(ocp.nx(), ocp.nu(), 0, 0, 10, 50);
        if (!strategy.solve(ocp, init_guess, sol_hier)
            || !auto_strategy.solve(ocp, init_guess, sol_auto)
            || !trajectoriesClose(sol_hier, sol_auto, 1e-8)) {
            state.SkipWithError("ManualHierarchicalStrategy results differ from AutoAdaptive");
            return;
        }
    }

    HierarchicalOptions opts;
    opts.coarse_n = 20;
    ManualHierarchicalStrategy strategy(ocp.nx(), ocp.nu(), 0, 0, opts);
    for (auto _ : state) {
        Trajectory sol;
        const bool ok = strategy.solve(ocp, init_guess, sol);
        benchmark::DoNotOptimize(sol);
        if (!ok) {
            state.SkipWithError("ManualHierarchicalStrategy solve failed");
            break;
        }
    }
}
BENCHMARK(BM_ManualHierarchicalStrategy_N200)->Unit(benchmark::kMillisecond);

// ===================== 基准：ManualHierarchicalStrategy enable=false 退化为直接细优化 =====================
static void BM_ManualHierarchicalDisable_N200(benchmark::State& state)
{
    const int N = 200;
    const double dt = 0.1;
    MultiStageOCP ocp = makeLongLqrOcp(N, dt);
    Trajectory init_guess = makeInitialGuess(N, ocp.nx(), ocp.nu());
    init_guess.x[0] << 5.0, 0.0;
    propagateInitialGuess(ocp, init_guess);

    // 一次性一致性校验：enable=false 应与显式细优化一致。
    {
        Trajectory sol_hier, sol_fine;
        HierarchicalOptions opts;
        opts.enable = false;
        opts.fine_max_iter = 10;
        ManualHierarchicalStrategy strategy(ocp.nx(), ocp.nu(), 0, 0, opts);
        const int cond_N = (N + 10 - 1) / 10;
        auto qp_solver = std::make_unique<HPIPMQPSolver>(
            N, ocp.nx(), ocp.nu(), ocp.nx(), ocp.nu(), 0, 0, cond_N);
        SQPSolver fine_solver(std::move(qp_solver));
        fine_solver.options().max_iter = 10;
        if (!strategy.solve(ocp, init_guess, sol_hier)
            || !fine_solver.solve(ocp, init_guess, sol_fine)
            || !trajectoriesClose(sol_hier, sol_fine, 1e-8)) {
            state.SkipWithError("ManualHierarchicalStrategy(disable) results differ from fine optimization");
            return;
        }
    }

    HierarchicalOptions opts;
    opts.enable = false;
    opts.fine_max_iter = 10;
    ManualHierarchicalStrategy strategy(ocp.nx(), ocp.nu(), 0, 0, opts);
    for (auto _ : state) {
        Trajectory sol;
        const bool ok = strategy.solve(ocp, init_guess, sol);
        benchmark::DoNotOptimize(sol);
        if (!ok) {
            state.SkipWithError("ManualHierarchicalStrategy(disable) solve failed");
            break;
        }
    }
}
BENCHMARK(BM_ManualHierarchicalDisable_N200)->Unit(benchmark::kMillisecond);

//      （OpenMP + Partial Condensing）下的稳态求解时间；里程碑目标为中位数 < 50ms。
// 只用于隔离度量"裸的 OpenMP + HPIPM Partial Condensing 引擎"本身的可扩展性，
// 不代表真实业务组合（BicycleModelDelta + 走廊/ESDF 约束 + 多段换挡）在同等规模下的
// 性能与收敛性——后者见下方 BM_Data3RealScenario_MultiSegmentBicycleCorridor。
static void BM_Data3InitialPath_N493(benchmark::State& state)
{
    const auto points = loadData3InitialPath();
    if (points.empty()) {
        state.SkipWithError("data3.json load failed or empty");
        return;
    }
    const int N = static_cast<int>(points.size()) - 1;
    if (N != 493) {
        state.SkipWithError("data3.json initial path step count mismatch");
        return;
    }

    const MultiStageOCP ocp = buildData3TrackingOCP(points);
    {
        std::string reason;
        if (!ocp.validate(&reason)) {
            state.SkipWithError("OCP validate failed");
            return;
        }
    }
    const Trajectory init_guess = makeData3InitialGuess(points);

    const int cond_N = (N + 10 - 1) / 10;
    auto qp_solver = std::make_unique<HPIPMQPSolver>(
        N, ocp.nx(), ocp.nu(), ocp.nx(), ocp.nu(), 0, 0, cond_N);
    SQPSolver solver(std::move(qp_solver));
    solver.options().use_omp = true;
    solver.options().omp_parallel_threshold = 50;

    // 预热一次，排除 HPIPM 工作区、clone 池等一次性分配对稳态计时的影响。
    Trajectory warmup_sol;
    if (!solver.solve(ocp, init_guess, warmup_sol)) {
        state.SkipWithError("data3.json warm-up solve failed");
        return;
    }

    for (auto _ : state) {
        Trajectory sol;
        const bool ok = solver.solve(ocp, init_guess, sol);
        benchmark::DoNotOptimize(sol);
        if (!ok) {
            state.SkipWithError("data3.json solve failed");
            break;
        }
    }
}
BENCHMARK(BM_Data3InitialPath_N493)->Unit(benchmark::kMillisecond);

// 目的：BicycleModelDelta（默认运动学模型）+ ConvexCorridorConstraint（真实 ng=40）
// + 由 data3.json::initial_path 真实换向切分出的多段换挡（外部分段，边界 v=0）
// 在 HPIPM + OpenMP + Partial Condensing 下的稳态求解耗时，替代 BM_Data3InitialPath_N493

// 按航向与位移方向的点积符号，将路径切分为方向不变的连续 run（换向点即切分点）。
struct PathRun {
    int start_idx = 0;
    int end_idx = 0;
    double v_sign = 1.0;
};

std::vector<PathRun> splitPathByDirection(const std::vector<TrajectoryPoint>& points)
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

// 由真实换向切分出的多段 OCP + 动力学一致初始猜测：
//   - 每段的 v box bound 按方向收窄（前进 [0,vmax] / 倒车 [-vmax,0]），
//     段边界因此自动收窄为 v=0（见设计文档第 10 节），无需额外切换约束；
//   - 每段的转向角 delta 取该段"总转角 / 总弧长"换算出的平均曲率对应值，
//     初始猜测通过 dynamics->discretize() 以三角形速度剖面（先加速再减速、
//     末态 v≈0）精确传播，保证与 BicycleModelDelta 动力学一致；
//   - 代价项跟踪该段初始猜测自身传播得到的终态（而非原始路径点）：
//     恒定曲率近似下自由传播的终点通常与真实路径终点存在偏差，若代价强行
//     牵引向不可达的真实终点，会使 Full SQP 线搜索找不到下降方向。
//     跟踪"自身传播终态"保证参考点必然可达，同时仍保留真实的段数/步数/
//     弧长/平均曲率结构，足以评估引擎在该规模下的性能与收敛性；
//   - 每段附加 ConvexCorridorConstraint，配合空地图（无墙）令 ng=40 行全部退化为
//     惰性 "0<=0"（数学安全），只贡献真实的计算维度/开销，不引入具体障碍物幅值；
//     【调试记录】曾用"远墙"（intercept 量级 1e4）模拟无约束场景，导致半空间截距
//     与状态量级（约 -150）严重不匹配，使 HPIPM 默认容差下于第 0 次 QP 求解即返回
//     MAX_ITER；改为空地图后该问题消失，是本框架的一个真实数值稳健性发现。
// max_steps_per_run：可选地将每个 run 截断到不超过该步数；max_runs：可选地只取
// 前 max_runs 个 run。两者均用于探测/复现当前实现的收敛规模上限
// （<=0 表示不截断/不限制，使用真实完整段数与段长度）。
struct RealScenario {
    MultiStageOCP ocp;
    Trajectory init_guess;
};

RealScenario buildData3RealMultiSegmentScenario(const std::vector<TrajectoryPoint>& points_in,
    int max_steps_per_run = -1, int max_runs = -1)
{
    // 将路径重新表达为以起点为原点的局部坐标系（标准工程实践：真实部署系统总是在
    // 局部 ego 坐标系下规划，而非原始全局/UTM 坐标），避免绝对坐标幅值（如 -150）
    // 与状态权重/box bound 的相对尺度失配，导致 QP 数值条件数恶化。
    // 【调试记录】曾直接使用 data3.json 的原始全局坐标（约 -150,-10），发现即便单个
    // 真实弯道段（N 低至 25）配合完全无约束的走廊，HPIPM 仍会在第 0 次 QP 求解即
    // 返回 MAX_ITER；改为局部坐标后同一场景稳定收敛。这是本框架当前实现对绝对坐标
    std::vector<TrajectoryPoint> points = points_in;
    const double origin_x = points_in.front().x, origin_y = points_in.front().y;
    for (auto& p : points) {
        p.x -= origin_x;
        p.y -= origin_y;
    }

    RealScenario scenario;
    const int nx = 5, nu = 2;
    const double dt = 0.1, wheelbase = 2.8, v_max = 2.5;
    // 换挡边界 v 的容差带：物理上"约等于 0"而非数学上严格为 0，
    // 避免多个段边界同时被钳死为单点等式约束造成 QP 病态/退化。
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

    // 第一遍：仅传播动力学一致的初始猜测。
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
            // 三角形速度剖面：前 half 步从 0 线性加速到 v_sign*v_peak，
            // 剩余 n-half 步线性减速回到 0，保证段末态 v≈0（配合 box bound 的段边界收窄）。
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

    // 第二遍：按每段初始猜测自身的终态装配代价与约束。
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

// 公共求解逻辑：构造真实场景、装配空地图走廊约束、用 HPIPM+OpenMP+Partial Condensing
// 求解，预热后在稳态循环中计时；solve 失败时通过 state.SkipWithError 显式失败。
//
// 真实多段换挡"这一组合在长 N 下收敛，经隔离实验确认必须同时满足以下三点，
// 缺一不可（详见 buildData3RealMultiSegmentScenario 顶部注释与下方逐条说明）：
//   1) 坐标系：必须使用以起点为原点的局部坐标（buildData3RealMultiSegmentScenario
//      已处理），而非 data3.json 的原始全局坐标（量级 -150）——否则 HPIPM 在默认
//      容差下于第 0 次 QP 求解就返回 MAX_ITER。
//   2) HPIPM 容差：默认容差（ROBUST 预设，约 1e-8）过紧，需放宽到 1e-4 才能在
//      1000 次内部迭代上限内收敛出一个可用的 QP 解。
//   3) 线搜索：即使 1) 2) 都满足，默认的 Armijo 线搜索仍会在真实多段场景下于
//      第 2 次 SQP 迭代判定 QP 方向为"非下降方向"而直接拒绝并使 solve() 失败；
//      必须关闭 use_line_search（直接接受 QP 的 Newton 步）才能继续迭代。
//      这牺牲了线搜索本应提供的下降保证，是当前实现的真实局限，而非最佳实践。
//   4) 迭代预算：默认 max_iter=10 不够，完整 493 步/9 段场景需要约 50 次迭代
//      才能收敛（远多于简化模型基准 BM_Data3InitialPath_N493 隐含的迭代量）。
// 在满足以上条件后，完整真实场景（N=493，9 段真实换挡）可稳定收敛，但耗时约
// 快"与"真实业务问题需要多少次迭代才能收敛"是两个独立的性能维度，后者尚未优化。
void RunData3RealScenarioBenchmark(benchmark::State& state, int max_steps_per_run,
    int max_runs = -1, bool use_partial_condensing = true, bool use_qp_warm_start = false)
{
    const auto points = loadData3InitialPath();
    if (points.empty()) {
        state.SkipWithError("data3.json load failed or empty");
        return;
    }

    RealScenario scenario
        = buildData3RealMultiSegmentScenario(points, max_steps_per_run, max_runs);
    const MultiStageOCP& ocp = scenario.ocp;
    {
        std::string reason;
        if (!ocp.validate(&reason)) {
            state.SkipWithError("Real scenario OCP validate failed: " + reason);
            return;
        }
    }

    SimpleParkingMap map; // 空地图：见 buildData3RealMultiSegmentScenario 顶部注释
    UpdaterConfig config;
    config.selection_radius = 200.0;
    config.max_step_displacement = 0.5;
    config.safety_margin = 0.1;
    config.top_k = 10;
    ProblemUpdater updater(config);
    updater.updateOcp(scenario.init_guess, map, scenario.ocp);

    const int N = ocp.totalSteps();
    const int cond_N = use_partial_condensing ? (N + 10 - 1) / 10 : -1;
    auto qp_solver = std::make_unique<HPIPMQPSolver>(
        N, ocp.nx(), ocp.nu(), ocp.nx(), ocp.nu(), CORRIDOR_G_DIM, 0, cond_N);
    HPIPMQPSolver* hpipm_raw = qp_solver.get();
    qp_solver->setTolerance(1e-4);
    SQPSolver solver(std::move(qp_solver));
    solver.options().use_omp = true;
    solver.options().omp_parallel_threshold = 50;
    solver.options().use_line_search = false;
    solver.options().max_iter = 50;
    solver.options().use_qp_warm_start = use_qp_warm_start;
    // 存在换挡点时 SQPSolver 会自动局部降级为 Full SQP，此处无需显式设置 use_rti。

    Trajectory warmup_sol;
    if (!solver.solve(ocp, scenario.init_guess, warmup_sol)) {
        state.SkipWithError("Real scenario warm-up solve failed (N=" + std::to_string(N) + ")");
        return;
    }

    for (auto _ : state) {
        Trajectory sol;
        const int total_iter_before = hpipm_raw->totalIterations();
        const bool ok = solver.solve(ocp, scenario.init_guess, sol);
        benchmark::DoNotOptimize(sol);
        if (!ok) {
            state.SkipWithError("Real scenario solve failed");
            break;
        }
        state.counters["last_ipm_iter"] = hpipm_raw->lastIterations();
        state.counters["total_ipm_iter"] = hpipm_raw->totalIterations() - total_iter_before;
    }
}

// 完整真实场景：data3.json 全部 9 段真实换挡结构，N=493。见 RunData3RealScenarioBenchmark
// 才能收敛，实测耗时约 800ms 量级，远高于简化模型基准隐含的 <50ms 目标。
static void BM_Data3RealScenario_MultiSegmentBicycleCorridor(benchmark::State& state)
{
    RunData3RealScenarioBenchmark(state, /*max_steps_per_run=*/-1);
}
BENCHMARK(BM_Data3RealScenario_MultiSegmentBicycleCorridor)->Unit(benchmark::kMillisecond);

// 与上一场景完全等价，但开启 Full SQP 跨迭代 QP 热启动，用于对比热启动对总求解耗时
// 与最终 IPM 迭代次数的影响。默认关闭，是否启用由 benchmark 数据驱动决定。
static void BM_Data3RealScenario_MultiSegmentBicycleCorridor_WarmStart(benchmark::State& state)
{
    RunData3RealScenarioBenchmark(state, /*max_steps_per_run=*/-1, /*max_runs=*/-1,
        /*use_partial_condensing=*/true, /*use_qp_warm_start=*/true);
}
BENCHMARK(BM_Data3RealScenario_MultiSegmentBicycleCorridor_WarmStart)
    ->Unit(benchmark::kMillisecond);

// 单个真实弯道段（data3.json 第一段，截断至 N=25）+ ConvexCorridorConstraint + HPIPM，
// 在"完整 493 步"与"短 N 集成测试"之间提供一个中等规模的真实性能锚点。
static void BM_Data3RealScenario_SingleSegmentN25(benchmark::State& state)
{
    RunData3RealScenarioBenchmark(
        state, /*max_steps_per_run=*/25, /*max_runs=*/1, /*use_partial_condensing=*/false);
}
BENCHMARK(BM_Data3RealScenario_SingleSegmentN25)->Unit(benchmark::kMillisecond);

// ===================== 基准：Partial Condensing 处理不能被 block_size 整除的 N =====================
static void BM_PartialCondensingNonDivisibleN53(benchmark::State& state)
{
    const int N = 53;
    const double dt = 0.1;
    MultiStageOCP ocp = makeLongLqrOcp(N, dt);
    ocp.segments()[0].constraints.push_back(std::make_shared<ControlUpperBoundConstraint>(0, 0.5));
    const int nx = ocp.nx(), nu = ocp.nu();
    Trajectory init_guess = makeInitialGuess(N, nx, nu);
    init_guess.x[0] << 5.0, 0.0;
    propagateInitialGuess(ocp, init_guess);

    const int cond_N = (N + 10 - 1) / 10; // 6

    // 一次性一致性校验：cond_N=6（向上取整）应与 cond_N=-1（无凝聚）一致。
    {
        Trajectory sol_no_cond, sol_cond;
        auto no_cond_qp = std::make_unique<HPIPMQPSolver>(N, nx, nu, nx, nu, 1, 0, -1);
        SQPSolver no_cond(std::move(no_cond_qp));
        no_cond.options().use_omp = false;
        auto cond_qp = std::make_unique<HPIPMQPSolver>(N, nx, nu, nx, nu, 1, 0, cond_N);
        SQPSolver cond(std::move(cond_qp));
        cond.options().use_omp = false;
        if (!no_cond.solve(ocp, init_guess, sol_no_cond)
            || !cond.solve(ocp, init_guess, sol_cond)
            || !trajectoriesClose(sol_no_cond, sol_cond, 1e-8)) {
            state.SkipWithError("Partial Condensing result for non-divisible N differs from no-condensing");
            return;
        }
    }

    auto qp_solver = std::make_unique<HPIPMQPSolver>(N, nx, nu, nx, nu, 1, 0, cond_N);
    SQPSolver solver(std::move(qp_solver));
    solver.options().use_omp = false;
    for (auto _ : state) {
        Trajectory sol;
        const bool ok = solver.solve(ocp, init_guess, sol);
        benchmark::DoNotOptimize(sol);
        if (!ok) {
            state.SkipWithError("Partial Condensing solve failed for non-divisible N");
            break;
        }
    }
}
BENCHMARK(BM_PartialCondensingNonDivisibleN53)->Unit(benchmark::kMillisecond);

// ===================== 基准：AutoAdaptiveStrategy 短 N 分支 =====================
static void BM_AutoAdaptiveStrategy_ShortN10(benchmark::State& state)
{
    const int N = 10;
    const double dt = 0.1;
    MultiStageOCP ocp = makeLongLqrOcp(N, dt);
    const int nx = ocp.nx(), nu = ocp.nu();
    Trajectory init_guess = makeInitialGuess(N, nx, nu);
    init_guess.x[0] << 0.0, 0.0;

    AutoAdaptiveStrategy strategy(nx, nu, 0, 0, 10, 50);

    // 一次性校验：短 N 成功求解，且非法输入（空 OCP）安全返回 false。
    {
        Trajectory sol;
        MultiStageOCP empty_ocp;
        Trajectory dummy;
        if (!strategy.solve(ocp, init_guess, sol)
            || strategy.solve(empty_ocp, init_guess, dummy)) {
            state.SkipWithError("AutoAdaptiveStrategy short N or invalid input handling unexpected");
            return;
        }
    }

    for (auto _ : state) {
        Trajectory sol;
        const bool ok = strategy.solve(ocp, init_guess, sol);
        benchmark::DoNotOptimize(sol);
        if (!ok) {
            state.SkipWithError("AutoAdaptiveStrategy short N solve failed");
            break;
        }
    }
}
BENCHMARK(BM_AutoAdaptiveStrategy_ShortN10)->Unit(benchmark::kMillisecond);

// ===================== 基准：ManualHierarchicalStrategy 多段 OCP（含段间边界）=====================
static void BM_ManualHierarchicalStrategy_MultiSegment(benchmark::State& state)
{
    const int N1 = 30, N2 = 25;
    const double dt = 0.1;
    const int nx = 2, nu = 1;

    Matrix A(nx, nx);
    A << 1.0, dt, 0.0, 1.0;
    Matrix B(nx, nu);
    B << 0.0, dt;
    const Matrix Q = Matrix::Identity(nx, nx);
    const Matrix R = Matrix::Identity(nu, nu) * 0.1;
    const Vector x_min = Vector::Constant(nx, -1e3);
    const Vector x_max = Vector::Constant(nx, 1e3);
    const Vector u_min = Vector::Constant(nu, -1e3);
    const Vector u_max = Vector::Constant(nu, 1e3);

    auto makeSegment = [&](int N, const Vector& x_ref) {
        StageSegment seg;
        seg.dynamics = std::make_shared<DoubleIntegrator>(A, B);
        seg.cost = std::make_shared<QuadraticTrackingCost>(x_ref, Q, R);
        seg.N = N;
        seg.dt = dt;
        seg.v_sign = 1.0;
        seg.x_min = x_min;
        seg.x_max = x_max;
        seg.u_min = u_min;
        seg.u_max = u_max;
        return seg;
    };

    MultiStageOCP ocp;
    ocp.addSegment(makeSegment(N1, Vector::Zero(nx)));
    ocp.addSegment(makeSegment(N2, Vector::Zero(nx)));

    Trajectory init_guess;
    init_guess.resize(N1 + N2, nx, nu);
    init_guess.x[0] << 2.0, 0.0;
    for (int k = 0; k < N1 + N2; ++k) {
        const auto [seg_idx, step] = ocp.globalStepToSegment(k);
        const auto& segment = ocp.segments()[seg_idx];
        segment.dynamics->discretize(
            init_guess.x[k], init_guess.u[k], segment.stepSize(step), segment.v_sign,
            init_guess.x[k + 1]);
    }

    HierarchicalOptions opts;
    opts.coarse_n = 10;
    opts.coarse_max_iter = 5;
    opts.fine_max_iter = 5;

    // 一次性一致性校验：多段分层结果与 AutoAdaptive 直接求解一致。
    {
        Trajectory sol_hier, sol_auto;
        ManualHierarchicalStrategy hier_strategy(nx, nu, 0, 0, opts);
        AutoAdaptiveStrategy auto_strategy(nx, nu, 0, 0, 10, 50);
        if (!hier_strategy.solve(ocp, init_guess, sol_hier)
            || !auto_strategy.solve(ocp, init_guess, sol_auto)
            || !trajectoriesClose(sol_hier, sol_auto, 1e-8)) {
            state.SkipWithError("Multi-segment ManualHierarchicalStrategy results differ from AutoAdaptive");
            return;
        }
    }

    ManualHierarchicalStrategy hier_strategy(nx, nu, 0, 0, opts);
    for (auto _ : state) {
        Trajectory sol;
        const bool ok = hier_strategy.solve(ocp, init_guess, sol);
        benchmark::DoNotOptimize(sol);
        if (!ok) {
            state.SkipWithError("Multi-segment ManualHierarchicalStrategy solve failed");
            break;
        }
    }
}
BENCHMARK(BM_ManualHierarchicalStrategy_MultiSegment)->Unit(benchmark::kMillisecond);

// ===================== 基准：含 CircleFootprintEsdfPenaltyCost 的单次 assembleQP 调用 =====================
// 目的：隔离度量组合求值接口 + OpenMP 并行化对 assembleQP 自身的加速收益。
// 通过派生类暴露 protected 的 linearize()/assembleQP()，先完成一次 solve() 预热与 linearize()，
// 再在稳态循环中反复调用 assembleQP()（该函数是幂等的，只覆盖 q/Q/r/R/S 与 bounds）。
class ExposedSQPSolver : public SQPSolver {
public:
    explicit ExposedSQPSolver(std::unique_ptr<QPSolver> qp_solver)
        : SQPSolver(std::move(qp_solver))
    {
    }
    using SQPSolver::linearize;
    using SQPSolver::assembleQP;
};

std::vector<Eigen::Vector2d> makeVehicleCircleCenters(int num_circles)
{
    std::vector<Eigen::Vector2d> centers;
    const double length = vehicle_geometry::kLf + vehicle_geometry::kLr;
    const double radius = vehicle_geometry::kWidth / 2.0;
    const double start = -vehicle_geometry::kLr + radius;
    const double end = vehicle_geometry::kLf - radius;
    const double step = (num_circles > 1) ? (end - start) / (num_circles - 1) : 0.0;
    for (int i = 0; i < num_circles; ++i) {
        centers.emplace_back(start + i * step, 0.0);
    }
    return centers;
}

CircleObstacleEsdfMap makeBenchmarkEsdfMap()
{
    CircleObstacleEsdfMap map;
    // 障碍物贴近车辆路径，使部分圆触发安全裕度违反，从而真实 exercising ESDF 查询分支
    map.addObstacle(Eigen::Vector2d(5.0, 0.5), 1.0);
    map.addObstacle(Eigen::Vector2d(10.0, 0.5), 0.8);
    return map;
}

MultiStageOCP makeEsdfPenaltyOcp(int N, const EsdfMapInterface& map)
{
    const int nx = 3, nu = 3;
    const double dt = 0.1;
    StageSegment seg;
    // 使用简单的 3D 路径积分器，使初始猜测 trivially 动力学一致，焦点放在 cost 装配开销上
    seg.dynamics = std::make_shared<PathIntegrator>();
    {
        std::vector<std::shared_ptr<CostTerm>> terms;
        terms.push_back(std::make_shared<QuadraticTrackingCost>(Vector::Zero(nx),
            Matrix::Identity(nx, nx) * 1e-2, Matrix::Identity(nu, nu) * 1e-2, /*theta_idx=*/2));
        terms.push_back(std::make_shared<CircleFootprintEsdfPenaltyCost>(makeVehicleCircleCenters(5),
            /*circle_radius=*/0.45, /*safety_margin=*/0.05, map, /*penalty_weight=*/1e3));
        seg.cost = std::make_shared<CompositeCost>(std::move(terms));
    }
    seg.N = N;
    seg.dt = dt;
    seg.v_sign = 1.0;
    seg.x_min = Vector::Constant(nx, -1e4);
    seg.x_max = Vector::Constant(nx, 1e4);
    seg.u_min = Vector::Constant(nu, -1e4);
    seg.u_max = Vector::Constant(nu, 1e4);
    MultiStageOCP ocp;
    ocp.addSegment(seg);
    return ocp;
}

Trajectory makeEsdfPenaltyInitialGuess(int N)
{
    const int nx = 3, nu = 3;
    const double dt = 0.1;
    Trajectory traj;
    traj.resize(N, nx, nu);
    // 沿 x 轴匀速前进的 trivially 动力学一致猜测
    for (int k = 0; k <= N; ++k) {
        traj.x[k] << static_cast<double>(k) * dt, 0.0, 0.0;
    }
    for (int k = 0; k < N; ++k) {
        traj.u[k] << 1.0, 0.0, 0.0;
    }
    return traj;
}

// 基线代价：强制 evaluateGradientAndHessian 退化为分别调用 evaluate/gradient/hessian，
// 用于在控制其他变量不变的情况下，单独度量组合求值接口消除重复 ESDF 查询的收益。
class CircleFootprintEsdfPenaltyCostBaseline : public CircleFootprintEsdfPenaltyCost {
public:
    CircleFootprintEsdfPenaltyCostBaseline(std::vector<Eigen::Vector2d> circle_local_positions,
        double circle_radius, double safety_margin, const EsdfMapInterface& map,
        double penalty_weight)
        : CircleFootprintEsdfPenaltyCost(std::move(circle_local_positions), circle_radius,
              safety_margin, map, penalty_weight)
    {
    }
    void evaluateGradientAndHessian(const Vector& x, const Vector& u, double& cost, Vector& q,
        Vector& r, Matrix& Q, Matrix& R, Matrix& S) const override
    {
        evaluate(x, u, cost);
        gradient(x, u, q, r);
        hessian(x, u, Q, R, S);
    }
};

MultiStageOCP makeEsdfPenaltyOcpWithCost(int N, const EsdfMapInterface& map,
    std::shared_ptr<CostTerm> penalty_cost)
{
    const int nx = 3, nu = 3;
    const double dt = 0.1;
    StageSegment seg;
    seg.dynamics = std::make_shared<PathIntegrator>();
    {
        std::vector<std::shared_ptr<CostTerm>> terms;
        terms.push_back(std::make_shared<QuadraticTrackingCost>(Vector::Zero(nx),
            Matrix::Identity(nx, nx) * 1e-2, Matrix::Identity(nu, nu) * 1e-2, /*theta_idx=*/2));
        terms.push_back(penalty_cost);
        seg.cost = std::make_shared<CompositeCost>(std::move(terms));
    }
    seg.N = N;
    seg.dt = dt;
    seg.v_sign = 1.0;
    seg.x_min = Vector::Constant(nx, -1e4);
    seg.x_max = Vector::Constant(nx, 1e4);
    seg.u_min = Vector::Constant(nu, -1e4);
    seg.u_max = Vector::Constant(nu, 1e4);
    MultiStageOCP ocp;
    ocp.addSegment(seg);
    return ocp;
}

static void BM_AssembleQp_EsdfPenaltyCost(benchmark::State& state)
{
    const int N = 200;
    const auto map = makeBenchmarkEsdfMap();
    auto penalty = std::make_shared<CircleFootprintEsdfPenaltyCost>(makeVehicleCircleCenters(5),
        /*circle_radius=*/0.45, /*safety_margin=*/0.05, map, /*penalty_weight=*/1e3);
    const auto ocp = makeEsdfPenaltyOcpWithCost(N, map, penalty);
    auto init_guess = makeEsdfPenaltyInitialGuess(N);

    const int cond_N = (N + 10 - 1) / 10;
    auto qp_solver = std::make_unique<HPIPMQPSolver>(N, 3, 3, 3, 3, 0, 0, cond_N);
    qp_solver->setTolerance(1e-4);
    ExposedSQPSolver solver(std::move(qp_solver));
    solver.options().use_omp = true;
    solver.options().omp_parallel_threshold = 50;
    solver.options().use_line_search = false;
    solver.options().max_iter = 10;

    Trajectory sol;
    if (!solver.solve(ocp, init_guess, sol)) {
        state.SkipWithError("ESDF penalty warm-up solve failed");
        return;
    }
    if (!solver.linearize()) {
        state.SkipWithError("ESDF penalty linearize failed");
        return;
    }

    for (auto _ : state) {
        if (!solver.assembleQP()) {
            state.SkipWithError("ESDF penalty assembleQP failed");
            break;
        }
    }
}
BENCHMARK(BM_AssembleQp_EsdfPenaltyCost)->Unit(benchmark::kMicrosecond);

static void BM_AssembleQp_EsdfPenaltyCost_Serial(benchmark::State& state)
{
    const int N = 200;
    const auto map = makeBenchmarkEsdfMap();
    auto penalty = std::make_shared<CircleFootprintEsdfPenaltyCost>(makeVehicleCircleCenters(5),
        /*circle_radius=*/0.45, /*safety_margin=*/0.05, map, /*penalty_weight=*/1e3);
    const auto ocp = makeEsdfPenaltyOcpWithCost(N, map, penalty);
    auto init_guess = makeEsdfPenaltyInitialGuess(N);

    const int cond_N = (N + 10 - 1) / 10;
    auto qp_solver = std::make_unique<HPIPMQPSolver>(N, 3, 3, 3, 3, 0, 0, cond_N);
    qp_solver->setTolerance(1e-4);
    ExposedSQPSolver solver(std::move(qp_solver));
    solver.options().use_omp = false;
    solver.options().use_line_search = false;
    solver.options().max_iter = 10;

    Trajectory sol;
    if (!solver.solve(ocp, init_guess, sol)) {
        state.SkipWithError("ESDF penalty warm-up solve failed");
        return;
    }
    if (!solver.linearize()) {
        state.SkipWithError("ESDF penalty linearize failed");
        return;
    }

    for (auto _ : state) {
        if (!solver.assembleQP()) {
            state.SkipWithError("ESDF penalty assembleQP failed");
            break;
        }
    }
}
BENCHMARK(BM_AssembleQp_EsdfPenaltyCost_Serial)->Unit(benchmark::kMicrosecond);

static void BM_AssembleQp_EsdfPenaltyCost_SerialBaseline(benchmark::State& state)
{
    const int N = 200;
    const auto map = makeBenchmarkEsdfMap();
    auto penalty = std::make_shared<CircleFootprintEsdfPenaltyCostBaseline>(
        makeVehicleCircleCenters(5), /*circle_radius=*/0.45, /*safety_margin=*/0.05, map,
        /*penalty_weight=*/1e3);
    const auto ocp = makeEsdfPenaltyOcpWithCost(N, map, penalty);
    auto init_guess = makeEsdfPenaltyInitialGuess(N);

    const int cond_N = (N + 10 - 1) / 10;
    auto qp_solver = std::make_unique<HPIPMQPSolver>(N, 3, 3, 3, 3, 0, 0, cond_N);
    qp_solver->setTolerance(1e-4);
    ExposedSQPSolver solver(std::move(qp_solver));
    solver.options().use_omp = false;
    solver.options().use_line_search = false;
    solver.options().max_iter = 10;

    Trajectory sol;
    if (!solver.solve(ocp, init_guess, sol)) {
        state.SkipWithError("ESDF penalty baseline warm-up solve failed");
        return;
    }
    if (!solver.linearize()) {
        state.SkipWithError("ESDF penalty baseline linearize failed");
        return;
    }

    for (auto _ : state) {
        if (!solver.assembleQP()) {
            state.SkipWithError("ESDF penalty baseline assembleQP failed");
            break;
        }
    }
}
BENCHMARK(BM_AssembleQp_EsdfPenaltyCost_SerialBaseline)->Unit(benchmark::kMicrosecond);

// ===================== 基准：-march=native 端到端对比 =====================
// 说明：-march=native 是编译期选项，无法在同一二进制中运行时切换。
// 本基准与 BM_Data3RealScenario_MultiSegmentBicycleCorridor 完全等价，仅名称含 MarchNative，
// 用于在启用 -march=native 的构建中记录数据；基线数据需在关闭该选项的构建中运行同名
// BM_Data3RealScenario_MultiSegmentBicycleCorridor 获取，对比结果记录于 review-log.md。
static void BM_Data3RealScenario_MultiSegmentBicycleCorridor_MarchNative(benchmark::State& state)
{
    RunData3RealScenarioBenchmark(state, /*max_steps_per_run=*/-1);
}
BENCHMARK(BM_Data3RealScenario_MultiSegmentBicycleCorridor_MarchNative)
    ->Unit(benchmark::kMillisecond);
