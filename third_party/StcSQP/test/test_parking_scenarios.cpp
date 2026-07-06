#include <gtest/gtest.h>
#include <limits>

#include "circle_obstacle_esdf_map.h"
#include "constraints/convex_corridor_constraint.h"
#include "constraints/esdf_distance_constraint.h"
#include "costs/quadratic_tracking.h"
#include "generated/corridor.h"
#include "models/bicycle_model_delta.h"
#include "esdf_problem_updater.h"
#include "ocp/multi_stage_ocp.h"
#include "problem_updater.h"
#include "qp/dense_qp_solver.h"
#include "qp/hpipm_solver.h"
#include "simple_parking_map.h"
#include "sqp/sqp_algorithm.h"
#include "util/trajectory.h"

using namespace stc_SQP;

namespace {
// 构造一个"前进-停车-倒车"的两段短 N 真实场景：BicycleModelDelta + 凸走廊约束 + HPIPM。
// 前进段 v 钳制在 [0, v_max]，倒车段钳制在 [-v_max, 0]，两段交界状态因此自动收窄为 v=0，
// 无需引擎提供任何额外的"段间切换约束"（见设计文档第 10 节）。
struct ShortHorizonScenario {
    MultiStageOCP ocp;
    Trajectory init_guess;
    int n1 = 0, n2 = 0, nx = 0, nu = 0;
};

ShortHorizonScenario buildShortHorizonScenario()
{
    ShortHorizonScenario s;
    s.n1 = 15;
    s.n2 = 15;
    s.nx = 5;
    s.nu = 2;
    const double dt = 0.1, v_max = 1.5, a0 = 1.0;
    auto dynamics = std::make_shared<BicycleModelDelta>(2.8);

    auto make_segment = [&](int n, double v_sign) {
        StageSegment seg;
        seg.dynamics = dynamics;
        seg.N = n;
        seg.dt = dt;
        seg.v_sign = v_sign;
        seg.x_min = Vector::Constant(s.nx, -1e3);
        seg.x_max = Vector::Constant(s.nx, 1e3);
        seg.x_min(3) = (v_sign > 0.0) ? 0.0 : -v_max;
        seg.x_max(3) = (v_sign > 0.0) ? v_max : 0.0;
        seg.u_min = Vector::Constant(s.nu, -2.0);
        seg.u_max = Vector::Constant(s.nu, 2.0);
        seg.cost = std::make_shared<QuadraticTrackingCost>(Vector::Zero(s.nx),
            Matrix::Identity(s.nx, s.nx) * 1e-2, Matrix::Identity(s.nu, s.nu) * 1e-2,
            /*theta_idx=*/2);
        seg.constraints.push_back(std::make_shared<ConvexCorridorConstraint>(s.nu));
        return seg;
    };

    s.ocp.addSegment(make_segment(s.n1, 1.0));
    s.ocp.addSegment(make_segment(s.n2, -1.0));

    // 前进段：加速再减速的三角速度剖面，末态 v≈0；倒车段对称（负向）。
    const int total_n = s.n1 + s.n2;
    s.init_guess.resize(total_n, s.nx, s.nu);
    s.init_guess.x[0] << 0.0, 0.0, 0.0, 0.0, 0.0;
    int global_k = 0;
    for (int seg_idx = 0; seg_idx < 2; ++seg_idx) {
        const int n = (seg_idx == 0) ? s.n1 : s.n2;
        const double sign = (seg_idx == 0) ? 1.0 : -1.0;
        const int half = n / 2;
        for (int i = 0; i < n; ++i) {
            const double a = (i < half) ? sign * a0 : -sign * a0;
            Vector u(s.nu);
            u << a, 0.0;
            s.init_guess.u[global_k] = u;
            dynamics->discretize(s.init_guess.x[global_k], u, dt, sign,
                s.init_guess.x[global_k + 1]);
            ++global_k;
        }
    }
    return s;
}
} // namespace

TEST(ParkingScenario, ShortHorizonRealBicycleCorridorGearShiftSolvesWithHpipm)
{
    //          + ConvexCorridorConstraint（经 ProblemUpdater 注入）+ 外部两段换挡
    //          （前进/倒车 box bound 令边界 v=0）+ HPIPMQPSolver——可以端到端求解成功。
    // 流程：构造前进(N=15)+倒车(N=15)共 30 步的两段 OCP，注入宽松走廊约束，
    //      用 HPIPMQPSolver（cond_N=-1，短 N 无需 Partial Condensing）求解。
    // 预期效果：solve 返回 true；因存在换挡点，rtiDowngraded() 为 true；
    //          解轨迹在段边界处的速度分量收敛到 0（数值容差内）。
    ShortHorizonScenario scenario = buildShortHorizonScenario();
    ASSERT_TRUE(scenario.ocp.hasGearShift());

    // 走廊约束设置为远离轨迹的宽松半空间，仅验证管线闭合，不测试避障有效性。
    SimpleParkingMap map;
    {
        Vector n(2);
        n << 1.0, 0.0;
        map.addWall({ n, 100.0 }, Eigen::Vector2d(100.0, -100.0), Eigen::Vector2d(100.0, 100.0));
    }
    UpdaterConfig config;
    config.selection_radius = 200.0;
    config.max_step_displacement = 0.5;
    config.safety_margin = 0.1;
    config.top_k = 10;
    ProblemUpdater updater(config);
    updater.updateOcp(scenario.init_guess, map, scenario.ocp);

    auto qp_solver = std::make_unique<HPIPMQPSolver>(scenario.ocp.totalSteps(), scenario.nx,
        scenario.nu, scenario.nx, scenario.nu, CORRIDOR_G_DIM, 0, -1);
    SQPSolver solver(std::move(qp_solver));
    solver.options().max_iter = 15;
    solver.options().use_rti = true; // 存在换挡点，应被自动降级为 Full SQP

    Trajectory solution;
    const bool ok = solver.solve(scenario.ocp, scenario.init_guess, solution);
    EXPECT_TRUE(ok);
    EXPECT_TRUE(solver.rtiDowngraded());

    const double v_at_boundary = solution.x[scenario.n1](3);
    EXPECT_NEAR(v_at_boundary, 0.0, 1e-6);
}

TEST(ParkingScenario, ShortHorizonRealBicycleCorridorAndEsdfSolvesWithHpipm)
{
    //          仍能通过 HPIPM 端到端求解，证明两种约束可在同一 OCP 中并存。
    // 流程：复用 buildShortHorizonScenario()，为每段追加 EsdfDistanceConstraint；
    //      用 SimpleParkingMap 注入凸走廊，CircleObstacleEsdfMap 注入 ESDF（障碍物远离轨迹）。
    // 预期效果：solve 返回 true；存在换挡点因此 RTI 降级；段边界速度为 0。
    ShortHorizonScenario scenario = buildShortHorizonScenario();
    for (auto& segment : scenario.ocp.segments()) {
        segment.constraints.push_back(std::make_shared<EsdfDistanceConstraint>(0.3));
    }

    SimpleParkingMap map;
    {
        Vector n(2);
        n << 1.0, 0.0;
        map.addWall({ n, 100.0 }, Eigen::Vector2d(100.0, -100.0), Eigen::Vector2d(100.0, 100.0));
    }
    UpdaterConfig config;
    config.selection_radius = 200.0;
    config.max_step_displacement = 0.5;
    config.safety_margin = 0.1;
    config.top_k = 10;
    ProblemUpdater updater(config);
    updater.updateOcp(scenario.init_guess, map, scenario.ocp);

    CircleObstacleEsdfMap esdf_map;
    esdf_map.addObstacle(Eigen::Vector2d(100.0, 100.0), 2.0);
    EsdfProblemUpdater esdf_updater;
    esdf_updater.updateOcp(scenario.init_guess, esdf_map, scenario.ocp);

    const int ng_total = CORRIDOR_G_DIM + EsdfDistanceConstraint::kNumCorners;
    auto qp_solver = std::make_unique<HPIPMQPSolver>(scenario.ocp.totalSteps(), scenario.nx,
        scenario.nu, scenario.nx, scenario.nu, ng_total, 0, -1);
    SQPSolver solver(std::move(qp_solver));
    solver.options().max_iter = 15;
    solver.options().use_rti = true;

    Trajectory solution;
    EXPECT_TRUE(solver.solve(scenario.ocp, scenario.init_guess, solution));
    EXPECT_TRUE(solver.rtiDowngraded());
    EXPECT_NEAR(solution.x[scenario.n1](3), 0.0, 1e-6);
}

TEST(ParkingScenario, ActiveEsdfObstacleKeepsSafeDistanceAfterSolve)
{
    // 测试目的：验证 EsdfDistanceConstraint 不只是"装饰"，在障碍物激活时仍能约束车辆轮廓距离，
    //          且求解结果确实是约束主动引导轨迹偏离（而非恰好满足）。
    // 流程：单段前进场景，v=1.5 恒速直线初始猜测（BicycleModelDelta），障碍物摆在直线路径
    //      中段一侧，使初始猜测在接近障碍物的那几步违反安全裕度，求解后检查所有步骤四个
    //      角点到障碍物的最小距离满足安全裕度，且轨迹产生了明显横向偏移。
    // 预期效果：solve 返回 true；min_corner_distance >= margin - 1e-3；
    //          max_y（横向偏移量）明显大于 0，证明约束确实主动引导了轨迹而非摆设。
    const int N = 30;
    const int nx = 5, nu = 2;
    const double margin = 0.2;
    auto dynamics = std::make_shared<BicycleModelDelta>(2.8);

    Trajectory init_guess;
    init_guess.resize(N, nx, nu);
    init_guess.x[0] << 0.0, 0.0, 0.0, 2.0, 0.0;
    for (int k = 0; k < N; ++k) {
        init_guess.u[k].setZero();
        dynamics->discretize(init_guess.x[k], init_guess.u[k], 0.1, 1.0, init_guess.x[k + 1]);
    }

    StageSegment seg;
    seg.dynamics = dynamics;
    seg.N = N;
    seg.dt = 0.1;
    seg.v_sign = 1.0;
    seg.x_min = Vector::Constant(nx, -1e3);
    seg.x_max = Vector::Constant(nx, 1e3);
    seg.u_min = Vector::Constant(nu, -2.0);
    seg.u_max = Vector::Constant(nu, 2.0);
    seg.cost = std::make_shared<QuadraticTrackingCost>(
        init_guess.x[N], Matrix::Identity(nx, nx), Matrix::Identity(nu, nu) * 0.1, /*theta_idx=*/2);
    seg.constraints.push_back(std::make_shared<EsdfDistanceConstraint>(margin));

    MultiStageOCP ocp;
    ocp.addSegment(seg);

    // 障碍物摆在恒速直线路径中段一侧，使初始猜测在贴近障碍物的那几步违反安全裕度，
    // 求解后必须主动偏移才能恢复裕度。
    CircleObstacleEsdfMap map;
    map.addObstacle(Eigen::Vector2d(6.0, 1.5), 0.3);
    EsdfProblemUpdater updater;
    updater.updateOcp(init_guess, map, ocp);

    SQPSolver solver(std::make_unique<DenseQPSolver>());
    solver.options().max_iter = 30;
    solver.options().use_line_search = false;

    Trajectory solution;
    ASSERT_TRUE(solver.solve(ocp, init_guess, solution));

    const auto corners_local = EsdfDistanceConstraint::cornerLocalPositions();
    double min_distance = std::numeric_limits<double>::max();
    double max_y = 0.0;
    for (int k = 0; k <= N; ++k) {
        const Vector& pose = solution.x[k];
        const double theta = pose(2), c = std::cos(theta), s = std::sin(theta);
        const Eigen::Vector2d center(pose(0), pose(1));
        max_y = std::max(max_y, std::abs(pose(1)));
        for (const auto& local : corners_local) {
            const Eigen::Vector2d corner_world(
                center(0) + c * local(0) - s * local(1), center(1) + s * local(0) + c * local(1));
            min_distance = std::min(min_distance, map.queryDistance(corner_world).distance);
        }
    }
    EXPECT_GE(min_distance, margin - 1e-3);
    EXPECT_GT(max_y, 0.05) << "ESDF constraint should actively steer the trajectory away from the obstacle";
}


TEST(ParkingScenario, TripleGearShiftSolvesWithHpipm)
{
    // 测试目的：验证前进-倒车-前进三段换挡场景可解，且 RTI 因换挡自动降级。
    // 流程：构造 N=10+10+10 的三段 OCP，每段速度 box 按方向钳制，生成动力学一致初始猜测，
    //      注入远离轨迹的凸走廊后用 HPIPM 求解。
    // 预期效果：solve 返回 true；rtiDowngraded 为 true；两个段边界处速度均接近 0。
    const int n = 10;
    const int nx = 5, nu = 2;
    const double dt = 0.1, v_max = 1.5, a0 = 1.0;
    auto dynamics = std::make_shared<BicycleModelDelta>(2.8);

    auto make_segment = [&](double v_sign) {
        StageSegment seg;
        seg.dynamics = dynamics;
        seg.N = n;
        seg.dt = dt;
        seg.v_sign = v_sign;
        seg.x_min = Vector::Constant(nx, -1e3);
        seg.x_max = Vector::Constant(nx, 1e3);
        seg.x_min(3) = (v_sign > 0.0) ? 0.0 : -v_max;
        seg.x_max(3) = (v_sign > 0.0) ? v_max : 0.0;
        seg.u_min = Vector::Constant(nu, -2.0);
        seg.u_max = Vector::Constant(nu, 2.0);
        seg.cost = std::make_shared<QuadraticTrackingCost>(
            Vector::Zero(nx), Matrix::Identity(nx, nx) * 1e-2,
            Matrix::Identity(nu, nu) * 1e-2, /*theta_idx=*/2);
        seg.constraints.push_back(std::make_shared<ConvexCorridorConstraint>(nu));
        return seg;
    };

    MultiStageOCP ocp;
    ocp.addSegment(make_segment(1.0));
    ocp.addSegment(make_segment(-1.0));
    ocp.addSegment(make_segment(1.0));
    ASSERT_TRUE(ocp.hasGearShift());

    Trajectory init_guess;
    init_guess.resize(3 * n, nx, nu);
    init_guess.x[0] << 0.0, 0.0, 0.0, 0.0, 0.0;
    int global_k = 0;
    for (int seg_idx = 0; seg_idx < 3; ++seg_idx) {
        const double sign = (seg_idx == 0 || seg_idx == 2) ? 1.0 : -1.0;
        const int half = n / 2;
        for (int i = 0; i < n; ++i) {
            const double a = (i < half) ? sign * a0 : -sign * a0;
            Vector u(nu);
            u << a, 0.0;
            init_guess.u[global_k] = u;
            dynamics->discretize(init_guess.x[global_k], u, dt, sign, init_guess.x[global_k + 1]);
            ++global_k;
        }
    }

    SimpleParkingMap map;
    {
        Vector normal(2);
        normal << 1.0, 0.0;
        map.addWall({ normal, 100.0 }, Eigen::Vector2d(100.0, -100.0), Eigen::Vector2d(100.0, 100.0));
    }
    UpdaterConfig config;
    config.selection_radius = 200.0;
    config.max_step_displacement = 0.5;
    config.safety_margin = 0.1;
    config.top_k = 10;
    ProblemUpdater updater(config);
    updater.updateOcp(init_guess, map, ocp);

    auto qp_solver = std::make_unique<HPIPMQPSolver>(
        ocp.totalSteps(), nx, nu, nx, nu, CORRIDOR_G_DIM, 0, -1);
    SQPSolver solver(std::move(qp_solver));
    solver.options().max_iter = 15;
    solver.options().use_rti = true;

    Trajectory solution;
    EXPECT_TRUE(solver.solve(ocp, init_guess, solution));
    EXPECT_TRUE(solver.rtiDowngraded());
    EXPECT_NEAR(solution.x[n](3), 0.0, 1e-6);
    EXPECT_NEAR(solution.x[2 * n](3), 0.0, 1e-6);
}

TEST(ParkingScenario, PivotSegmentSteeringConvergesWithThreeSegments)
{
    // 测试目的：验证三段“前进 → 驻车转向（v 钳死为 0、delta_dot 范围放宽） → 倒车”场景可解。
    //          驻车段允许车辆在静止时把方向盘打到较大角度，反向段再沿圆弧后退，
    // 流程：构造 N1=12 前进段 + N2=6 驻车段 + N3=12 倒车段，
    //      前进/倒车段用三角形速度剖面保证边界 v≈0，驻车段用恒定 delta_dot 把方向盘转到 0.6 rad，
    //      每段代价跟踪自身传播得到的终态，用 HPIPMQPSolver 求解。
    // 预期效果：solve 返回 true；两个段边界处速度接近 0；最终航向因倒车圆弧而明显偏离 0。
    const int N1 = 12, N2 = 6, N3 = 12;
    const int nx = 5, nu = 2;
    const double dt = 0.1, L = 2.8, v_peak = 0.8, delta_pivot = 0.6;
    const int N_total = N1 + N2 + N3;
    auto dynamics = std::make_shared<BicycleModelDelta>(L);

    Trajectory init_guess;
    init_guess.resize(N_total, nx, nu);
    init_guess.x[0] << 0.0, 0.0, 0.0, 0.0, 0.0;

    auto propagateTriangular = [&](int start, int n, double sign, double delta) {
        const int half = n / 2;
        for (int i = 0; i < n; ++i) {
            const double a = (i < half)
                ? sign * v_peak / (std::max(1, half) * dt)
                : -sign * v_peak / (std::max(1, n - half) * dt);
            Vector& x = init_guess.x[start + i];
            x(4) = delta;
            init_guess.u[start + i] << a, 0.0;
            dynamics->discretize(x, init_guess.u[start + i], dt, sign, init_guess.x[start + i + 1]);
        }
    };

    // 前进段：直线，v 从 0 → v_peak → 0
    propagateTriangular(0, N1, 1.0, 0.0);
    // 驻车段：v=0，方向盘从当前值匀速转到 delta_pivot
    for (int i = 0; i < N2; ++i) {
        Vector& x = init_guess.x[N1 + i];
        x(3) = 0.0; // v 保持为 0
        init_guess.u[N1 + i] << 0.0, delta_pivot / (N2 * dt);
        dynamics->discretize(x, init_guess.u[N1 + i], dt, 1.0, init_guess.x[N1 + i + 1]);
    }
    // 倒车段：delta 保持 delta_pivot，v 从 0 → -v_peak → 0，沿圆弧后退
    propagateTriangular(N1 + N2, N3, -1.0, delta_pivot);

    auto make_segment = [&](int n, double v_sign, double v_min, double v_max, double delta_min,
                            double delta_max, const Vector& x_ref) {
        StageSegment seg;
        seg.dynamics = dynamics;
        seg.N = n;
        seg.dt = dt;
        seg.v_sign = v_sign;
        seg.x_min = Vector::Constant(nx, -1e3);
        seg.x_max = Vector::Constant(nx, 1e3);
        seg.x_min(3) = v_min;
        seg.x_max(3) = v_max;
        seg.x_min(4) = delta_min;
        seg.x_max(4) = delta_max;
        seg.u_min = Vector::Constant(nu, -5.0);
        seg.u_max = Vector::Constant(nu, 5.0);
        Matrix Q = Matrix::Identity(nx, nx);
        Q(0, 0) = 1.0;
        Q(1, 1) = 1.0;
        Q(2, 2) = 10.0;
        Q(3, 3) = 0.1;
        Q(4, 4) = 0.1;
        seg.cost = std::make_shared<QuadraticTrackingCost>(
            x_ref, Q, Matrix::Identity(nu, nu) * 1e-2, /*theta_idx=*/2);
        return seg;
    };

    MultiStageOCP ocp;
    ocp.addSegment(make_segment(N1, 1.0, 0.0, 1.5, -0.6, 0.6, init_guess.x[N1]));
    ocp.addSegment(make_segment(N2, 1.0, 0.0, 0.0, -0.8, 0.8, init_guess.x[N1 + N2]));
    // 倒车段把方向盘下界收紧到 0.5，确保必须沿圆弧后退而不是直着倒回来
    ocp.addSegment(make_segment(N3, -1.0, -1.5, 0.0, 0.5, 0.6, init_guess.x[N_total]));

    auto qp_solver = std::make_unique<HPIPMQPSolver>(N_total, nx, nu, nx, nu, 0, 0, -1);
    qp_solver->setTolerance(1e-4);
    SQPSolver solver(std::move(qp_solver));
    solver.options().max_iter = 30;
    solver.options().use_line_search = false;

    Trajectory solution;
    EXPECT_TRUE(solver.solve(ocp, init_guess, solution));

    // 段边界速度应接近 0（换挡点/驻车切换点）
    EXPECT_NEAR(solution.x[N1](3), 0.0, 1e-6);
    EXPECT_NEAR(solution.x[N1 + N2](3), 0.0, 1e-6);

    // 倒车圆弧应使最终航向明显偏离 0，并产生可观测的横向位移，
    // 这是驻车段把方向盘打到 0.6 rad 后反向退出的直接证据。
    EXPECT_LT(solution.x[N_total](2), -0.05);
    EXPECT_GT(std::abs(solution.x[N_total](1)), 0.02);

    // 方向盘全程在各自的 box bound 内
    for (int k = 0; k <= N_total; ++k) {
        EXPECT_GE(solution.x[k](4), -0.8 - 1e-9);
        EXPECT_LE(solution.x[k](4), 0.8 + 1e-9);
    }
}

TEST(ParkingScenario, HpipmAndDenseSolverProduceConsistentSolutions)
{
    // 测试目的：验证同一停车场景分别用 HPIPM 与 DenseQPSolver 求解得到一致结果，
    //          作为两种 QP 后端数值一致性的 CI 锁。
    // 流程：复用短 N 双段走廊场景，分别用两种后端求解，比较最终状态与边界速度。
    // 预期效果：两者均成功，状态差异小于 0.1，段边界速度均接近 0。
    ShortHorizonScenario scenario = buildShortHorizonScenario();

    SimpleParkingMap map;
    {
        Vector n(2);
        n << 1.0, 0.0;
        map.addWall({ n, 100.0 }, Eigen::Vector2d(100.0, -100.0), Eigen::Vector2d(100.0, 100.0));
    }
    UpdaterConfig config;
    config.selection_radius = 200.0;
    config.max_step_displacement = 0.5;
    config.safety_margin = 0.1;
    config.top_k = 10;
    ProblemUpdater updater(config);
    updater.updateOcp(scenario.init_guess, map, scenario.ocp);

    auto hpipm_solver = std::make_unique<HPIPMQPSolver>(scenario.ocp.totalSteps(), scenario.nx,
        scenario.nu, scenario.nx, scenario.nu, CORRIDOR_G_DIM, 0, -1);
    SQPSolver solver_h(std::move(hpipm_solver));
    solver_h.options().max_iter = 15;
    solver_h.options().use_line_search = false;
    Trajectory sol_h;
    ASSERT_TRUE(solver_h.solve(scenario.ocp, scenario.init_guess, sol_h));

    SQPSolver solver_d(std::make_unique<DenseQPSolver>());
    solver_d.options().max_iter = 15;
    solver_d.options().use_line_search = false;
    Trajectory sol_d;
    ASSERT_TRUE(solver_d.solve(scenario.ocp, scenario.init_guess, sol_d));

    double max_state_diff = 0.0;
    for (int k = 0; k <= scenario.ocp.totalSteps(); ++k) {
        max_state_diff = std::max(max_state_diff, (sol_h.x[k] - sol_d.x[k]).norm());
    }
    EXPECT_LT(max_state_diff, 0.1);
    EXPECT_NEAR(sol_h.x[scenario.n1](3), 0.0, 1e-6);
    EXPECT_NEAR(sol_d.x[scenario.n1](3), 0.0, 1e-6);
}
