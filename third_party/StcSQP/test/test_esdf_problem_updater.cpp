#include <gtest/gtest.h>
#include <limits>

#include "circle_obstacle_esdf_map.h"
#include "constraints/esdf_distance_constraint.h"
#include "costs/quadratic_tracking.h"
#include "esdf_problem_updater.h"
#include "models/bicycle_model_delta.h"
#include "ocp/multi_stage_ocp.h"
#include "qp/dense_qp_solver.h"
#include "sqp/sqp_algorithm.h"
#include "util/trajectory.h"

using namespace stc_SQP;

namespace {
// 构造一段使用 BicycleModelDelta 的简单前进 StageSegment，供本文件的用例复用。
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
    seg.cost = std::make_shared<QuadraticTrackingCost>(
        x_ref, Matrix::Identity(nx, nx), Matrix::Identity(nu, nu) * 0.1, /*theta_idx=*/2);
    return seg;
}

// 根据当前位姿计算第 corner 个角点的世界坐标，便于跨用例复用。
Eigen::Vector2d computeCornerWorld(const Vector& pose, int corner)
{
    const auto corners_local = EsdfDistanceConstraint::cornerLocalPositions();
    const double theta = pose(2), c = std::cos(theta), s = std::sin(theta);
    const Eigen::Vector2d center(pose(0), pose(1));
    const Eigen::Vector2d& local = corners_local[corner];
    return Eigen::Vector2d(center(0) + c * local(0) - s * local(1),
        center(1) + s * local(0) + c * local(1));
}

// 断言 segment.stage_params 中 ESDF 区间与 map.queryDistance(corner_world) 一致。
void assertEsdfParamsMatchMap(const Trajectory& traj, int global_start,
    const CircleObstacleEsdfMap& map, const StageSegment& segment)
{
    for (int i = 0; i < segment.N; ++i) {
        const Vector& p = segment.stage_params[i].p;
        ASSERT_EQ(p.size(), STAGE_PARAM_DIM);
        const Vector& pose = traj.x[global_start + i];
        for (int corner = 0; corner < EsdfDistanceConstraint::kNumCorners; ++corner) {
            const Eigen::Vector2d corner_world = computeCornerWorld(pose, corner);
            const EsdfSample expected = map.queryDistance(corner_world);
            const int base = EsdfDistanceConstraint::kParamStart
                + corner * EsdfDistanceConstraint::kCornerStride;
            EXPECT_NEAR(p(base + 0), expected.distance, 1e-9);
            EXPECT_NEAR(p(base + 1), expected.gradient(0), 1e-9);
            EXPECT_NEAR(p(base + 2), expected.gradient(1), 1e-9);
            EXPECT_NEAR(p(base + 3), corner_world(0), 1e-9);
            EXPECT_NEAR(p(base + 4), corner_world(1), 1e-9);
        }
    }
}
} // namespace

TEST(EsdfProblemUpdater, InjectsDistanceAndGradientMatchingCircleMap)
{
    // 测试目的：验证 EsdfProblemUpdater 按车辆四角点世界坐标查询 CircleObstacleEsdfMap，
    //          并把结果正确写入 stage_params[i].p 的 ESDF 区间
    // 流程：构造一个圆形障碍物地图，车辆位于已知位姿，调用 updateOcp
    // 预期效果：写入的 distance/gradient 与直接调用 map.queryDistance() 的结果一致
    const int N = 3;
    MultiStageOCP ocp;
    ocp.addSegment(makeForwardSegment(N));

    Trajectory traj;
    traj.resize(N, 5, 2);
    for (int k = 0; k <= N; ++k) {
        traj.x[k] << 1.0 * k, 0.0, 0.0, 1.0, 0.0;
    }

    CircleObstacleEsdfMap map;
    map.addObstacle(Eigen::Vector2d(20.0, 0.0), 2.0);

    EsdfProblemUpdater updater;
    updater.updateOcp(traj, map, ocp);

    const auto corners_local = EsdfDistanceConstraint::cornerLocalPositions();
    for (int k = 0; k < N; ++k) {
        const Vector& p = ocp.segments()[0].stage_params[k].p;
        ASSERT_EQ(p.size(), STAGE_PARAM_DIM);
        const Vector& pose = traj.x[k];
        const double theta = pose(2), c = std::cos(theta), s = std::sin(theta);
        const Eigen::Vector2d center(pose(0), pose(1));
        for (int corner = 0; corner < EsdfDistanceConstraint::kNumCorners; ++corner) {
            const Eigen::Vector2d& local = corners_local[corner];
            const Eigen::Vector2d corner_world(center(0) + c * local(0) - s * local(1),
                center(1) + s * local(0) + c * local(1));
            const EsdfSample expected = map.queryDistance(corner_world);
            const int base = EsdfDistanceConstraint::kParamStart
                + corner * EsdfDistanceConstraint::kCornerStride;
            EXPECT_NEAR(p(base + 0), expected.distance, 1e-9);
            EXPECT_NEAR(p(base + 1), expected.gradient(0), 1e-9);
            EXPECT_NEAR(p(base + 2), expected.gradient(1), 1e-9);
            EXPECT_NEAR(p(base + 3), corner_world(0), 1e-9);
            EXPECT_NEAR(p(base + 4), corner_world(1), 1e-9);
        }
    }
}

TEST(EsdfProblemUpdaterIntegration, SqpSolvesWithEsdfConstraintAwayFromObstacle)
{
    // 测试目的：验证 BicycleModelDelta + EsdfDistanceConstraint（经 EsdfProblemUpdater 注入）
    //          + SQPSolver 端到端可解（远离障碍物，约束不应生效阻碍收敛）
    // 流程：构造与动力学一致的匀速直线初始猜测（u=0，v 恒定），参考状态取轨迹终点，
    //      障碍物放在远离该轨迹的位置，构造 OCP 并调用 SQPSolver::solve
    // 预期效果：solve 返回 true
    const int N = 5;
    const int nx = 5, nu = 2;
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
    seg.cost = std::make_shared<QuadraticTrackingCost>(
        init_guess.x[N], Matrix::Identity(nx, nx), Matrix::Identity(nu, nu) * 0.1, /*theta_idx=*/2);
    seg.constraints.push_back(std::make_shared<EsdfDistanceConstraint>(0.3));

    MultiStageOCP ocp;
    ocp.addSegment(seg);

    CircleObstacleEsdfMap map;
    map.addObstacle(Eigen::Vector2d(100.0, 100.0), 2.0); // 远离轨迹，约束应恒可行

    EsdfProblemUpdater updater;
    updater.updateOcp(init_guess, map, ocp);

    SQPSolver solver(std::make_unique<DenseQPSolver>());
    solver.options().max_iter = 5;
    Trajectory solution;
    EXPECT_TRUE(solver.solve(ocp, init_guess, solution));
}

TEST(EsdfProblemUpdater, MultiSegmentInjectionUpdatesEveryStep)
{
    // 测试目的：验证 EsdfProblemUpdater 对多段 OCP 的每一段、每一步都注入 ESDF 参数
    // 流程：构造两段不同步长的前进段，生成任意位姿轨迹，调用 updateOcp
    // 预期效果：每个 segment.stage_params 长度均为 N，且 ESDF 区间与直接查询 map 一致
    const int N1 = 3, N2 = 4;
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
    map.addObstacle(Eigen::Vector2d(5.0, 1.0), 1.5);

    EsdfProblemUpdater updater;
    updater.updateOcp(traj, map, ocp);

    int global_start = 0;
    for (const auto& segment : ocp.segments()) {
        ASSERT_EQ(static_cast<int>(segment.stage_params.size()), segment.N);
        assertEsdfParamsMatchMap(traj, global_start, map, segment);
        global_start += segment.N;
    }
}

TEST(EsdfProblemUpdater, PreservesNonEsdfParameters)
{
    // 测试目的：验证 EsdfProblemUpdater 只覆盖 p[45:65] ESDF 区间，不破坏凸走廊等非 ESDF 槽位
    // 流程：预分配 stage_params 并填入 p[15:45] 标记值，调用 updateOcp
    // 预期效果：p[15:45] 保持不变，p[45:65] 被 map 采样覆盖
    const int N = 3;
    MultiStageOCP ocp;
    ocp.addSegment(makeForwardSegment(N));
    for (auto& segment : ocp.segments()) {
        segment.stage_params.resize(N);
        for (auto& sp : segment.stage_params) {
            sp.p = Vector::Zero(STAGE_PARAM_DIM);
            for (int i = 15; i < 45; ++i) {
                sp.p(i) = 100.0 + i;
            }
        }
    }

    Trajectory traj;
    traj.resize(N, 5, 2);
    for (int k = 0; k <= N; ++k) {
        traj.x[k] << 1.0 * k, 0.0, 0.0, 1.0, 0.0;
    }

    CircleObstacleEsdfMap map;
    map.addObstacle(Eigen::Vector2d(10.0, 0.0), 2.0);

    EsdfProblemUpdater updater;
    updater.updateOcp(traj, map, ocp);

    for (const auto& segment : ocp.segments()) {
        for (const auto& sp : segment.stage_params) {
            const Vector& p = sp.p;
            for (int i = 15; i < 45; ++i) {
                EXPECT_NEAR(p(i), 100.0 + i, 1e-12) << "non-ESDF slot " << i << " was overwritten";
            }
        }
    }
}

TEST(EsdfProblemUpdater, ThrowsIfTrajectoryShorterThanRequired)
{
    // 测试目的：验证当 current_traj.x 长度不足 totalSteps+1 时 updater 抛出异常
    // 流程：构造 N=4 的 OCP，但 traj 只含 4 个状态（应需 5 个），调用 updateOcp
    // 预期效果：抛出 std::invalid_argument
    const int N = 4;
    MultiStageOCP ocp;
    ocp.addSegment(makeForwardSegment(N));

    Trajectory traj;
    traj.resize(N - 1, 5, 2);

    CircleObstacleEsdfMap map;
    EsdfProblemUpdater updater;
    EXPECT_THROW(updater.updateOcp(traj, map, ocp), std::invalid_argument);
}

TEST(EsdfProblemUpdater, ThrowsIfStateDimensionTooSmall)
{
    // 测试目的：验证当某步状态维度小于 3（缺少 x/y/theta）时 updater 抛出异常
    // 流程：构造合法 N 但把 traj.x[2] 换成 2 维向量
    // 预期效果：抛出 std::invalid_argument
    const int N = 3;
    MultiStageOCP ocp;
    ocp.addSegment(makeForwardSegment(N));

    Trajectory traj;
    traj.resize(N, 5, 2);
    for (int k = 0; k <= N; ++k) {
        traj.x[k] << 1.0 * k, 0.0, 0.0, 1.0, 0.0;
    }
    traj.x[2] = Vector::Zero(2);

    CircleObstacleEsdfMap map;
    EsdfProblemUpdater updater;
    EXPECT_THROW(updater.updateOcp(traj, map, ocp), std::invalid_argument);
}

TEST(EsdfProblemUpdater, ThrowsIfStageParamDimensionInvalid)
{
    // 测试目的：验证当 stage_params 已包含非空参数但维度不是 STAGE_PARAM_DIM 时抛出异常
    // 流程：预分配一个 10 维的 p，调用 updateOcp
    // 预期效果：抛出 std::invalid_argument
    const int N = 2;
    MultiStageOCP ocp;
    ocp.addSegment(makeForwardSegment(N));
    ocp.segments()[0].stage_params.resize(N);
    ocp.segments()[0].stage_params[0].p = Vector::Zero(10);

    Trajectory traj;
    traj.resize(N, 5, 2);
    for (int k = 0; k <= N; ++k) {
        traj.x[k] << 0.0, 0.0, 0.0, 1.0, 0.0;
    }

    CircleObstacleEsdfMap map;
    EsdfProblemUpdater updater;
    EXPECT_THROW(updater.updateOcp(traj, map, ocp), std::invalid_argument);
}

TEST(EsdfProblemUpdater, ThrowsIfOcpConfigurationInvalid)
{
    // 测试目的：验证 EsdfProblemUpdater 在 OCP validate 失败时抛出异常
    // 流程：构造 x_min 维度与 nx 不一致的非法 segment，调用 updateOcp
    // 预期效果：抛出 std::invalid_argument
    const int N = 2;
    StageSegment seg = makeForwardSegment(N);
    seg.x_min = Vector::Constant(3, -1e3); // 错误维度，应为 5
    MultiStageOCP ocp;
    ocp.addSegment(seg);

    Trajectory traj;
    traj.resize(N, 5, 2);

    CircleObstacleEsdfMap map;
    EsdfProblemUpdater updater;
    EXPECT_THROW(updater.updateOcp(traj, map, ocp), std::invalid_argument);
}

TEST(EsdfProblemUpdaterIntegration, SqpSolvesWithActiveObstacle)
{
    // 测试目的：验证障碍物贴近规划路径时，EsdfDistanceConstraint 能真正被激活并引导 SQP 避开
    // 流程：构造从 (0,0) 沿 x 轴以 v=2.0 前进 30 步的直线初始猜测，在车辆中心附近放置贴近的圆形障碍物，
    //      经 EsdfProblemUpdater 注入参数后用 DenseQPSolver 求解，检查最终角点最小距离 >= margin
    // 预期效果：solve 返回 true，且所有步骤的四个角点到障碍物的距离均满足安全裕度
    const int N = 30;
    const int nx = 5, nu = 2;
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
    seg.constraints.push_back(std::make_shared<EsdfDistanceConstraint>(0.2));

    MultiStageOCP ocp;
    ocp.addSegment(seg);

    CircleObstacleEsdfMap map;
    // 障碍物贴近轨迹但保证初始猜测仍满足 margin=0.2，从而约束被激活而不导致 QP 不可行
    map.addObstacle(Eigen::Vector2d(6.0, 1.5), 0.3);

    EsdfProblemUpdater updater;
    updater.updateOcp(init_guess, map, ocp);

    SQPSolver solver(std::make_unique<DenseQPSolver>());
    solver.options().max_iter = 30;
    solver.options().use_line_search = false;
    Trajectory solution;
    ASSERT_TRUE(solver.solve(ocp, init_guess, solution));

    const double margin = 0.2;
    double min_distance = std::numeric_limits<double>::max();
    double max_y = 0.0;
    for (int k = 0; k <= N; ++k) {
        max_y = std::max(max_y, std::abs(solution.x[k](1)));
        for (int corner = 0; corner < EsdfDistanceConstraint::kNumCorners; ++corner) {
            min_distance = std::min(min_distance, map.queryDistance(computeCornerWorld(solution.x[k], corner)).distance);
        }
    }
    EXPECT_GE(min_distance, margin - 1e-3);
    EXPECT_GT(max_y, 0.05) << "ESDF constraint should actively steer the trajectory away from the obstacle";
}

TEST(CircleObstacleEsdfMap, EmptyMapReturnsLargeDistanceAndZeroGradient)
{
    // 测试目的：验证未添加障碍物时 queryDistance 返回“很远”的距离与零梯度
    // 流程：查询任意点
    // 预期效果：distance 为 numeric_limits<double>::max()，gradient 为零向量
    CircleObstacleEsdfMap map;
    const EsdfSample sample = map.queryDistance(Eigen::Vector2d(1.0, 2.0));
    EXPECT_EQ(sample.distance, std::numeric_limits<double>::max());
    EXPECT_NEAR(sample.gradient.norm(), 0.0, 1e-12);
}

TEST(CircleObstacleEsdfMap, AddObstacleValidatesInputs)
{
    // 测试目的：验证 addObstacle 拒绝非法输入
    // 流程：分别传入非正半径与非有限圆心
    // 预期效果：均抛出 std::invalid_argument
    CircleObstacleEsdfMap map;
    EXPECT_THROW(map.addObstacle(Eigen::Vector2d(0.0, 0.0), -1.0), std::invalid_argument);
    EXPECT_THROW(map.addObstacle(Eigen::Vector2d(0.0, 0.0), 0.0), std::invalid_argument);
    EXPECT_THROW(map.addObstacle(Eigen::Vector2d(std::numeric_limits<double>::infinity(), 0.0), 1.0),
        std::invalid_argument);
}

TEST(CircleObstacleEsdfMap, OutsideDistanceAndGradient)
{
    // 测试目的：验证点在障碍物外部时 queryDistance 返回正确的符号距离与单位外法向梯度
    // 流程：在 (2,0) 查询圆心 (0,0)、半径 1 的圆
    // 预期效果：distance = 1，gradient = (1,0)
    CircleObstacleEsdfMap map;
    map.addObstacle(Eigen::Vector2d(0.0, 0.0), 1.0);
    const EsdfSample sample = map.queryDistance(Eigen::Vector2d(2.0, 0.0));
    EXPECT_NEAR(sample.distance, 1.0, 1e-12);
    EXPECT_NEAR(sample.gradient(0), 1.0, 1e-12);
    EXPECT_NEAR(sample.gradient(1), 0.0, 1e-12);
}

TEST(CircleObstacleEsdfMap, InsideDistanceAndGradient)
{
    // 测试目的：验证点在障碍物内部时返回负距离，且梯度仍指向外法向
    // 流程：在 (0.5,0) 查询圆心 (0,0)、半径 1 的圆
    // 预期效果：distance = -0.5，gradient = (1,0)
    CircleObstacleEsdfMap map;
    map.addObstacle(Eigen::Vector2d(0.0, 0.0), 1.0);
    const EsdfSample sample = map.queryDistance(Eigen::Vector2d(0.5, 0.0));
    EXPECT_NEAR(sample.distance, -0.5, 1e-12);
    EXPECT_NEAR(sample.gradient(0), 1.0, 1e-12);
    EXPECT_NEAR(sample.gradient(1), 0.0, 1e-12);
}
