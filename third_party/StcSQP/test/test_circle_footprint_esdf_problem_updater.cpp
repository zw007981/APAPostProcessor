#include <gtest/gtest.h>
#include <limits>

#include "circle_footprint_esdf_problem_updater.h"
#include "circle_obstacle_esdf_map.h"
#include "constraints/circle_footprint_esdf_constraint.h"
#include "costs/quadratic_tracking.h"
#include "models/bicycle_model_delta.h"
#include "ocp/multi_stage_ocp.h"
#include "qp/dense_qp_solver.h"
#include "sqp/sqp_algorithm.h"
#include "util/trajectory.h"

using namespace stc_SQP;

namespace {
// 一组测试用的车身局部圆心坐标：3个圆，覆盖车头、车尾两侧
std::vector<Eigen::Vector2d> testLocalCircles()
{
    return { Eigen::Vector2d(1.5, 0.8), Eigen::Vector2d(1.5, -0.8), Eigen::Vector2d(-1.0, 0.0) };
}

// 构造一段使用BicycleModelDelta的简单前进StageSegment，供本文件的用例复用
StageSegment makeForwardSegment(int N)
{
    const int nx = 5, nu = 2;
    StageSegment seg;
    seg.dynamics = std::make_shared<BicycleModelDelta>(2.8);
    seg.N = N;
    seg.dt = 0.1;
    seg.v_sign = 1.0;
    seg.x_min = Vector::Constant(nx, -1e3);
    seg.x_max = Vector::Constant(nx, 1e3);
    seg.u_min = Vector::Constant(nu, -2.0);
    seg.u_max = Vector::Constant(nu, 2.0);
    Vector x_ref(nx);
    x_ref << 10.0, 0.0, 0.0, 1.0, 0.0;
    seg.cost = std::make_shared<QuadraticTrackingCost>(x_ref, Matrix::Identity(nx, nx), Matrix::Identity(nu, nu) * 0.1, /*theta_idx=*/2);
    return seg;
}

// 根据当前位姿计算第circle个圆心的世界坐标，便于跨用例复用
Eigen::Vector2d computeCircleWorld(
    const std::vector<Eigen::Vector2d>& circles_local, const Vector& pose, int circle)
{
    const double theta = pose(2), c = std::cos(theta), s = std::sin(theta);
    const Eigen::Vector2d center(pose(0), pose(1));
    const Eigen::Vector2d& local = circles_local[circle];
    return Eigen::Vector2d(center(0) + c * local(0) - s * local(1),
        center(1) + s * local(0) + c * local(1));
}
} // namespace

TEST(CircleFootprintEsdfProblemUpdater, RejectsInvalidConstructorArguments)
{
    // 测试目的：验证构造函数拒绝空圆列表与超出kMaxCircles的圆列表
    // 流程：分别用空vector、超量vector构造
    // 预期效果：均抛出std::invalid_argument
    EXPECT_THROW(CircleFootprintEsdfProblemUpdater({}), std::invalid_argument);

    std::vector<Eigen::Vector2d> too_many(CircleFootprintEsdfConstraint::kMaxCircles + 1,
        Eigen::Vector2d(0.0, 0.0));
    // 注意：此处必须用花括号初始化而非圆括号，否则单参数、纯标识符实参会被解析为
    // "most vexing parse"（声明一个名为too_many的局部变量），导致误判为调用默认构造函数
    EXPECT_THROW(CircleFootprintEsdfProblemUpdater{too_many}, std::invalid_argument);
}

TEST(CircleFootprintEsdfProblemUpdater, InjectsDistanceAndGradientMatchingCircleMap)
{
    // 测试目的：验证CircleFootprintEsdfProblemUpdater按各圆心世界坐标查询
    //          CircleObstacleEsdfMap，并把结果正确写入stage_params[i].p的圆参数区间
    // 流程：构造一个圆形障碍物地图，车辆位于已知位姿，调用updateOcp
    // 预期效果：写入的distance/gradient与直接调用map.queryDistance()的结果一致
    const int N = 3;
    const auto circles = testLocalCircles();
    MultiStageOCP ocp;
    ocp.addSegment(makeForwardSegment(N));

    Trajectory traj;
    traj.resize(N, 5, 2);
    for (int k = 0; k <= N; ++k) {
        traj.x[k] << 1.0 * k, 0.0, 0.0, 1.0, 0.0;
    }

    CircleObstacleEsdfMap map;
    map.addObstacle(Eigen::Vector2d(20.0, 0.0), 2.0);

    CircleFootprintEsdfProblemUpdater updater(circles);
    updater.updateOcp(traj, map, ocp);

    for (int k = 0; k < N; ++k) {
        const Vector& p = ocp.segments()[0].stage_params[k].p;
        ASSERT_EQ(p.size(), STAGE_PARAM_DIM);
        const Vector& pose = traj.x[k];
        for (int circle = 0; circle < static_cast<int>(circles.size()); ++circle) {
            const Eigen::Vector2d circle_world = computeCircleWorld(circles, pose, circle);
            const EsdfSample expected = map.queryDistance(circle_world);
            const int base = CircleFootprintEsdfConstraint::kParamStart
                + circle * CircleFootprintEsdfConstraint::kCircleStride;
            EXPECT_NEAR(p(base + 0), expected.distance, 1e-9);
            EXPECT_NEAR(p(base + 1), expected.gradient(0), 1e-9);
            EXPECT_NEAR(p(base + 2), expected.gradient(1), 1e-9);
            EXPECT_NEAR(p(base + 3), circle_world(0), 1e-9);
            EXPECT_NEAR(p(base + 4), circle_world(1), 1e-9);
        }
    }
}

TEST(CircleFootprintEsdfProblemUpdaterIntegration, SqpSolvesWithCircleConstraintAwayFromObstacle)
{
    // 测试目的：验证BicycleModelDelta + CircleFootprintEsdfConstraint（经
    //          CircleFootprintEsdfProblemUpdater注入）+ SQPSolver端到端可解
    //          （远离障碍物，约束不应生效阻碍收敛）
    // 流程：构造与动力学一致的匀速直线初始猜测（u=0，v恒定），参考状态取轨迹终点，
    //      障碍物放在远离该轨迹的位置，构造OCP并调用SQPSolver::solve
    // 预期效果：solve返回true
    const int N = 5;
    const int nx = 5, nu = 2;
    const auto circles = testLocalCircles();
    auto dynamics = std::make_shared<BicycleModelDelta>(2.8);

    Trajectory init_guess;
    init_guess.resize(N, nx, nu);
    init_guess.x[0] << 0.0, 0.0, 0.0, 1.0, 0.0;
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
    seg.cost = std::make_shared<QuadraticTrackingCost>(init_guess.x[N], Matrix::Identity(nx, nx), Matrix::Identity(nu, nu) * 0.1,
        /*theta_idx=*/2);
    seg.constraints.push_back(std::make_shared<CircleFootprintEsdfConstraint>(circles, 0.8, 0.3));

    MultiStageOCP ocp;
    ocp.addSegment(seg);

    CircleObstacleEsdfMap map;
    map.addObstacle(Eigen::Vector2d(100.0, 100.0), 2.0); // 远离轨迹，约束应恒可行

    CircleFootprintEsdfProblemUpdater updater(circles);
    updater.updateOcp(init_guess, map, ocp);

    SQPSolver solver(std::make_unique<DenseQPSolver>());
    solver.options().max_iter = 5;
    Trajectory solution;
    EXPECT_TRUE(solver.solve(ocp, init_guess, solution));
}

TEST(CircleFootprintEsdfProblemUpdater, MultiSegmentInjectionUpdatesEveryStep)
{
    // 测试目的：验证CircleFootprintEsdfProblemUpdater对多段OCP的每一段、每一步都注入圆参数
    // 流程：构造两段不同步长的前进段，生成任意位姿轨迹，调用updateOcp
    // 预期效果：每个segment.stage_params长度均为N，且圆参数区间与直接查询map一致
    const int N1 = 3, N2 = 4;
    const auto circles = testLocalCircles();
    MultiStageOCP ocp;
    ocp.addSegment(makeForwardSegment(N1));
    ocp.addSegment(makeForwardSegment(N2));

    const int total_steps = N1 + N2;
    Trajectory traj;
    traj.resize(total_steps, 5, 2);
    for (int k = 0; k <= total_steps; ++k) {
        traj.x[k] << 0.5 * k, std::sin(0.3 * k), 0.1 * k, 1.0, 0.0;
    }

    CircleObstacleEsdfMap map;
    map.addObstacle(Eigen::Vector2d(50.0, 50.0), 1.0);

    CircleFootprintEsdfProblemUpdater updater(circles);
    updater.updateOcp(traj, map, ocp);

    int global_k = 0;
    for (const auto& segment : ocp.segments()) {
        ASSERT_EQ(static_cast<int>(segment.stage_params.size()), segment.N);
        for (int i = 0; i < segment.N; ++i) {
            const Vector& p = segment.stage_params[i].p;
            ASSERT_EQ(p.size(), STAGE_PARAM_DIM);
            const Vector& pose = traj.x[global_k + i];
            for (int circle = 0; circle < static_cast<int>(circles.size()); ++circle) {
                const Eigen::Vector2d circle_world = computeCircleWorld(circles, pose, circle);
                const EsdfSample expected = map.queryDistance(circle_world);
                const int base = CircleFootprintEsdfConstraint::kParamStart
                    + circle * CircleFootprintEsdfConstraint::kCircleStride;
                EXPECT_NEAR(p(base + 0), expected.distance, 1e-9);
                EXPECT_NEAR(p(base + 3), circle_world(0), 1e-9);
                EXPECT_NEAR(p(base + 4), circle_world(1), 1e-9);
            }
        }
        global_k += segment.N;
    }
}
