#include <vector>

#include <gtest/gtest.h>

#include "constraints/constraint.hpp"
#include "constraints/convex_corridor_constraint.h"
#include "costs/quadratic_tracking.h"
#include "map_interface.h"
#include "models/bicycle_model_kappa.h"
#include "ocp/multi_stage_ocp.h"
#include "problem_updater.h"
#include "qp/dense_qp_solver.h"
#include "simple_parking_map.h"
#include "sqp/sqp_algorithm.h"
#include "util/trajectory.h"

using namespace stc_SQP;

// ===================== 测试辅助：返回固定半空间集合的 Mock 地图 =====================
class FixedHalfSpaceMap : public MapInterface {
public:
    explicit FixedHalfSpaceMap(const std::vector<HalfSpace>& half_spaces)
        : half_spaces_(half_spaces)
    {
    }
    std::vector<HalfSpace> queryHalfSpaces(
        const Vector& pose, double selection_radius, int top_k) const override
    {
        (void)pose;
        (void)selection_radius;
        std::vector<HalfSpace> result;
        const int n = std::min(top_k, static_cast<int>(half_spaces_.size()));
        result.reserve(n);
        for (int i = 0; i < n; ++i) {
            result.push_back(half_spaces_[i]);
        }
        return result;
    }

private:
    std::vector<HalfSpace> half_spaces_;
};

// ===================== 测试辅助：记录 evaluate 收到的显式参数 p 的 Mock 约束 =====================
class RecordingConstraint : public Constraint {
public:
    RecordingConstraint(int nx, int nu)
        : nx_(nx)
        , nu_(nu)
    {
    }
    int ng() const override { return 1; }
    void evaluate(const Vector& x, const Vector& u, const Vector& p, Vector& g) const override
    {
        (void)x;
        (void)u;
        recorded_params_.push_back(p);
        g.resize(1);
        // 恒可行，避免干扰 SQP 收敛；真正的验证点在 recorded_params_
        g(0) = -1.0;
    }
    void jacobian(const Vector& x, const Vector& u, const Vector& p, Matrix& Cx,
        Matrix& Cu) const override
    {
        (void)x;
        (void)u;
        (void)p;
        Cx.setZero(1, nx_);
        Cu.setZero(1, nu_);
    }
    std::shared_ptr<Constraint> clone() const
    {
        auto copy = std::make_shared<RecordingConstraint>(nx_, nu_);
        copy->recorded_params_ = recorded_params_;
        return copy;
    }
    const std::vector<Vector>& recordedParams() const { return recorded_params_; }

private:
    int nx_ = 0;
    int nu_ = 0;
    mutable std::vector<Vector> recorded_params_;
};

// ===================== 测试辅助：evaluate 抛异常，用于验证 SQP 兜底 =====================
class ThrowingEvaluationConstraint : public Constraint {
public:
    ThrowingEvaluationConstraint(int nx, int nu)
        : nx_(nx)
        , nu_(nu)
    {
    }
    int ng() const override { return 1; }
    void evaluate(const Vector& x, const Vector& u, const Vector& p, Vector& g) const override
    {
        (void)x;
        (void)u;
        (void)g;
        if (p.size() > 0) {
            throw std::runtime_error("Deliberate exception in evaluate");
        }
        g.resize(1);
        g(0) = -1.0;
    }
    void jacobian(const Vector& x, const Vector& u, const Vector& p, Matrix& Cx,
        Matrix& Cu) const override
    {
        (void)x;
        (void)u;
        (void)p;
        Cx.setZero(1, nx_);
        Cu.setZero(1, nu_);
    }
    std::shared_ptr<Constraint> clone() const
    {
        return std::make_shared<ThrowingEvaluationConstraint>(nx_, nu_);
    }

private:
    int nx_ = 0;
    int nu_ = 0;
};

// ===================== 测试辅助：根据 pose.x 返回不同截距的地图 =====================
class PoseDependentMap : public MapInterface {
public:
    std::vector<HalfSpace> queryHalfSpaces(
        const Vector& pose, double selection_radius, int top_k) const override
    {
        (void)selection_radius;
        (void)top_k;
        std::vector<HalfSpace> result;
        Vector n(2);
        n << 1.0, 0.0;
        // 截距与 pose.x 线性相关，使不同 step 得到不同 p
        result.push_back({ n, 10.0 + pose(0) });
        return result;
    }
};

// ===================== 测试辅助：构造最小可运行的 MultiStageOCP =====================
MultiStageOCP makeParkingOcp(int N)
{
    MultiStageOCP ocp;
    StageSegment segment;
    segment.dynamics = std::make_shared<BicycleModelKappa>();
    segment.N = N;
    segment.dt = 0.1;
    segment.v_sign = 1.0;
    segment.x_min = Vector::Constant(5, -10.0);
    segment.x_max = Vector::Constant(5, 10.0);
    segment.u_min = Vector::Constant(2, -1.0);
    segment.u_max = Vector::Constant(2, 1.0);
    ocp.addSegment(segment);
    return ocp;
}

Trajectory makeParkingTrajectory(int N)
{
    Trajectory traj;
    traj.resize(N, 5, 2);
    for (int k = 0; k <= N; ++k) {
        traj.x[k] << 1.0 * k, 2.0 * k, 0.1 * k, 0.0, 0.0;
    }
    for (int k = 0; k < N; ++k) {
        traj.u[k].setZero();
    }
    return traj;
}

// ===================== 测试用例 =====================

TEST(ProblemUpdater, ThrowsWhenSelectionRadiusTooSmall)
{
    // 测试目的：验证当 selection_radius <= max_step_displacement + safety_margin 时，
    //          ProblemUpdater 在构造阶段抛出 std::invalid_argument
    // 流程：构造 selection_radius 过小的 UpdaterConfig，创建 ProblemUpdater
    // 预期效果：抛出 std::invalid_argument
    UpdaterConfig config;
    config.selection_radius = 0.5;
    config.max_step_displacement = 0.4;
    config.safety_margin = 0.2;
    config.top_k = 10;

    EXPECT_THROW({ ProblemUpdater updater(config); }, std::invalid_argument);
}

TEST(ProblemUpdater, ThrowsForIllegalTopK)
{
    // 测试目的：验证 top_k 超出 [1, kMaxHalfSpaces] 时构造失败
    // 流程：分别设置 top_k = 0 和 top_k = 11，创建 ProblemUpdater
    // 预期效果：均抛出 std::invalid_argument
    UpdaterConfig config;
    config.selection_radius = 2.0;
    config.max_step_displacement = 0.4;
    config.safety_margin = 0.2;

    config.top_k = 0;
    EXPECT_THROW({ ProblemUpdater updater(config); }, std::invalid_argument);

    config.top_k = ProblemUpdater::kMaxHalfSpaces + 1;
    EXPECT_THROW({ ProblemUpdater updater(config); }, std::invalid_argument);
}

TEST(ProblemUpdater, PacksTopKHalfSpacesIntoParameterVector)
{
    // 测试目的：验证 updateOcp 将地图查询到的 Top-K 半空间正确塞入 150 维参数 p，
    //          且截距位置固定在 p[35:45]，不受运行时 top_k 影响；非凸走廊区间保持不变。
    // 流程：构造有效 OCP 与轨迹，Mock Map 返回 3 个半空间，top_k=10，调用 updateOcp
    // 预期效果：每步 stage_params.p 为 150 维，前 3 个半空间数据落在 p[15:35]/p[35:45]，
    //          p[0:15] 保持原值（本例为 0），p[45:150] 为 0
    UpdaterConfig config;
    config.selection_radius = 2.0;
    config.max_step_displacement = 0.4;
    config.safety_margin = 0.2;
    config.top_k = 10;
    ProblemUpdater updater(config);

    const int N = 3;
    MultiStageOCP ocp = makeParkingOcp(N);
    Trajectory traj = makeParkingTrajectory(N);

    std::vector<HalfSpace> half_spaces;
    Vector n0(2);
    n0 << 1.0, 0.0;
    half_spaces.push_back({ n0, 5.0 });
    Vector n1(2);
    n1 << 0.0, 1.0;
    half_spaces.push_back({ n1, 6.0 });
    Vector n2(2);
    n2 << 0.5, 0.5;
    half_spaces.push_back({ n2, 7.0 });
    FixedHalfSpaceMap map(half_spaces);

    updater.updateOcp(traj, map, ocp);

    ASSERT_EQ(ocp.segments().size(), 1u);
    const StageSegment& segment = ocp.segments()[0];
    ASSERT_EQ(segment.stage_params.size(), 3u);
    for (int i = 0; i < N; ++i) {
        const Vector& p = segment.stage_params[i].p;
        EXPECT_EQ(p.size(), ProblemUpdater::kParameterDim);
        // 半空间 0：法向量在 p[15:17]，截距在 p[35]
        EXPECT_DOUBLE_EQ(p(ProblemUpdater::kHalfSpaceStart + 0), 1.0);
        EXPECT_DOUBLE_EQ(p(ProblemUpdater::kHalfSpaceStart + 1), 0.0);
        EXPECT_DOUBLE_EQ(p(ProblemUpdater::kInterceptStart + 0), 5.0);
        // 半空间 1
        EXPECT_DOUBLE_EQ(p(ProblemUpdater::kHalfSpaceStart + 2), 0.0);
        EXPECT_DOUBLE_EQ(p(ProblemUpdater::kHalfSpaceStart + 3), 1.0);
        EXPECT_DOUBLE_EQ(p(ProblemUpdater::kInterceptStart + 1), 6.0);
        // 半空间 2
        EXPECT_DOUBLE_EQ(p(ProblemUpdater::kHalfSpaceStart + 4), 0.5);
        EXPECT_DOUBLE_EQ(p(ProblemUpdater::kHalfSpaceStart + 5), 0.5);
        EXPECT_DOUBLE_EQ(p(ProblemUpdater::kInterceptStart + 2), 7.0);
        // 未填充的半空间槽位保持 0
        for (int j = 3; j < ProblemUpdater::kMaxHalfSpaces; ++j) {
            EXPECT_DOUBLE_EQ(p(ProblemUpdater::kHalfSpaceStart + 2 * j), 0.0);
            EXPECT_DOUBLE_EQ(p(ProblemUpdater::kHalfSpaceStart + 2 * j + 1), 0.0);
            EXPECT_DOUBLE_EQ(p(ProblemUpdater::kInterceptStart + j), 0.0);
        }
        // 非凸走廊区间 p[0:15] 保持 0
        for (int k = 0; k < ProblemUpdater::kHalfSpaceStart; ++k) {
            EXPECT_DOUBLE_EQ(p(k), 0.0);
        }
        for (int k = ProblemUpdater::kInterceptStart + ProblemUpdater::kMaxHalfSpaces;
             k < ProblemUpdater::kParameterDim; ++k) {
            EXPECT_DOUBLE_EQ(p(k), 0.0);
        }
    }
}

TEST(ProblemUpdater, PreservesNonCorridorParameters)
{
    // 测试目的：验证 updateOcp 只覆盖凸走廊区间 p[15:45]，不破坏调用方已填好的 p[0:15]
    // 流程：预先填充 segment.stage_params[i].p 的 p[0:15]，调用 updateOcp 后检查这些槽位不变，
    //      同时凸走廊区间被正确写入。
    // 预期效果：p[0:15] 保持预设值，p[15:45] 为新的半空间参数。
    UpdaterConfig config;
    config.selection_radius = 2.0;
    config.max_step_displacement = 0.4;
    config.safety_margin = 0.2;
    config.top_k = 10;
    ProblemUpdater updater(config);

    const int N = 1;
    MultiStageOCP ocp = makeParkingOcp(N);
    Trajectory traj = makeParkingTrajectory(N);

    // 预先填入非凸走廊参数：p[0:3] 参考位姿，p[3] 摩擦系数，p[4] 坡度，p[5:15] 动态权重
    ocp.segments()[0].stage_params.resize(N);
    for (int i = 0; i < N; ++i) {
        ocp.segments()[0].stage_params[i].p = Vector::Zero(ProblemUpdater::kParameterDim);
        ocp.segments()[0].stage_params[i].p(0) = 10.0;
        ocp.segments()[0].stage_params[i].p(1) = 20.0;
        ocp.segments()[0].stage_params[i].p(2) = 0.5;
        ocp.segments()[0].stage_params[i].p(3) = 0.8;
        ocp.segments()[0].stage_params[i].p(4) = 0.05;
        for (int k = 5; k < ProblemUpdater::kHalfSpaceStart; ++k) {
            ocp.segments()[0].stage_params[i].p(k) = static_cast<double>(k);
        }
    }

    std::vector<HalfSpace> half_spaces;
    Vector n(2);
    n << 1.0, 0.0;
    half_spaces.push_back({ n, 5.0 });
    FixedHalfSpaceMap map(half_spaces);

    updater.updateOcp(traj, map, ocp);

    const Vector& p = ocp.segments()[0].stage_params[0].p;
    // p[0:15] 应保持原值
    EXPECT_DOUBLE_EQ(p(0), 10.0);
    EXPECT_DOUBLE_EQ(p(1), 20.0);
    EXPECT_DOUBLE_EQ(p(2), 0.5);
    EXPECT_DOUBLE_EQ(p(3), 0.8);
    EXPECT_DOUBLE_EQ(p(4), 0.05);
    for (int k = 5; k < ProblemUpdater::kHalfSpaceStart; ++k) {
        EXPECT_DOUBLE_EQ(p(k), static_cast<double>(k));
    }
    // p[15:45] 应被新的半空间覆盖
    EXPECT_DOUBLE_EQ(p(ProblemUpdater::kHalfSpaceStart + 0), 1.0);
    EXPECT_DOUBLE_EQ(p(ProblemUpdater::kHalfSpaceStart + 1), 0.0);
    EXPECT_DOUBLE_EQ(p(ProblemUpdater::kInterceptStart + 0), 5.0);
}

TEST(ProblemUpdater, ClearsStaleCorridorParametersOnSecondUpdate)
{
    // 测试目的：验证 updateOcp 第二次调用时，上一轮写入的凸走廊槽位会被清零，
    //          避免陈旧半空间参数继续影响约束。
    // 流程：第一次用 3 个半空间更新，第二次用 1 个半空间更新；检查第 2、3 号槽位归零。
    // 预期效果：第二次更新后只有第 0 号半空间槽位非零，其余槽位为 0。
    UpdaterConfig config;
    config.selection_radius = 2.0;
    config.max_step_displacement = 0.4;
    config.safety_margin = 0.2;
    config.top_k = 10;
    ProblemUpdater updater(config);

    const int N = 1;
    MultiStageOCP ocp = makeParkingOcp(N);
    Trajectory traj = makeParkingTrajectory(N);

    std::vector<HalfSpace> three_hs;
    Vector n(2);
    n << 1.0, 0.0;
    three_hs.push_back({ n, 1.0 });
    three_hs.push_back({ n, 2.0 });
    three_hs.push_back({ n, 3.0 });
    FixedHalfSpaceMap map_three(three_hs);
    FixedHalfSpaceMap map_one({ { n, 5.0 } });

    updater.updateOcp(traj, map_three, ocp);
    const Vector& p_first = ocp.segments()[0].stage_params[0].p;
    EXPECT_DOUBLE_EQ(p_first(ProblemUpdater::kInterceptStart + 0), 1.0);
    EXPECT_DOUBLE_EQ(p_first(ProblemUpdater::kInterceptStart + 1), 2.0);
    EXPECT_DOUBLE_EQ(p_first(ProblemUpdater::kInterceptStart + 2), 3.0);

    updater.updateOcp(traj, map_one, ocp);
    const Vector& p_second = ocp.segments()[0].stage_params[0].p;
    EXPECT_DOUBLE_EQ(p_second(ProblemUpdater::kHalfSpaceStart + 0), 1.0);
    EXPECT_DOUBLE_EQ(p_second(ProblemUpdater::kHalfSpaceStart + 1), 0.0);
    EXPECT_DOUBLE_EQ(p_second(ProblemUpdater::kInterceptStart + 0), 5.0);
    for (int j = 1; j < ProblemUpdater::kMaxHalfSpaces; ++j) {
        EXPECT_DOUBLE_EQ(p_second(ProblemUpdater::kInterceptStart + j), 0.0)
            << "intercept slot " << j << " should be zeroed";
        EXPECT_DOUBLE_EQ(p_second(ProblemUpdater::kHalfSpaceStart + 2 * j), 0.0)
            << "normal x slot " << j << " should be zeroed";
        EXPECT_DOUBLE_EQ(p_second(ProblemUpdater::kHalfSpaceStart + 2 * j + 1), 0.0)
            << "normal y slot " << j << " should be zeroed";
    }
}

TEST(ProblemUpdater, TruncatesExcessHalfSpacesToTopK)
{
    // 测试目的：验证当地图返回的半空间数超过 top_k 时只取前 top_k 个，
    //          且固定 ABI 下截距仍位于 p[35:45]
    // 流程：设 top_k=2，Mock Map 返回 5 个半空间
    // 预期效果：p 中只写入前 2 个半空间，第 3 个及以后截距槽位为 0
    UpdaterConfig config;
    config.selection_radius = 2.0;
    config.max_step_displacement = 0.4;
    config.safety_margin = 0.2;
    config.top_k = 2;
    ProblemUpdater updater(config);

    const int N = 1;
    MultiStageOCP ocp = makeParkingOcp(N);
    Trajectory traj = makeParkingTrajectory(N);

    std::vector<HalfSpace> half_spaces;
    for (int i = 0; i < 5; ++i) {
        Vector n(2);
        n << 1.0, 0.0;
        half_spaces.push_back({ n, static_cast<double>(i + 1) });
    }
    FixedHalfSpaceMap map(half_spaces);

    updater.updateOcp(traj, map, ocp);

    const Vector& p = ocp.segments()[0].stage_params[0].p;
    EXPECT_DOUBLE_EQ(p(ProblemUpdater::kHalfSpaceStart + 0), 1.0);
    EXPECT_DOUBLE_EQ(p(ProblemUpdater::kHalfSpaceStart + 1), 0.0);
    EXPECT_DOUBLE_EQ(p(ProblemUpdater::kInterceptStart + 0), 1.0);
    EXPECT_DOUBLE_EQ(p(ProblemUpdater::kHalfSpaceStart + 2), 1.0);
    EXPECT_DOUBLE_EQ(p(ProblemUpdater::kHalfSpaceStart + 3), 0.0);
    EXPECT_DOUBLE_EQ(p(ProblemUpdater::kInterceptStart + 1), 2.0);
    // 被截断的半空间不应写入其截距槽位
    EXPECT_DOUBLE_EQ(p(ProblemUpdater::kInterceptStart + 2), 0.0);
}

TEST(ProblemUpdater, ThrowsWhenOcpHasNoSegment)
{
    // 测试目的：验证 updateOcp 在 OCP 没有任何段时直接抛出异常，避免空跑
    // 流程：构造空 MultiStageOCP，调用 updateOcp
    // 预期效果：抛出 std::invalid_argument
    UpdaterConfig config;
    config.selection_radius = 2.0;
    config.max_step_displacement = 0.4;
    config.safety_margin = 0.2;
    config.top_k = 10;
    ProblemUpdater updater(config);

    MultiStageOCP ocp;
    Trajectory traj;
    traj.resize(0, 5, 2);
    FixedHalfSpaceMap map({});

    EXPECT_THROW([&] { updater.updateOcp(traj, map, ocp); }(), std::invalid_argument);
}

TEST(ProblemUpdater, ThrowsWhenOcpIsInvalid)
{
    // 测试目的：验证 updateOcp 在 OCP 配置非法（如状态边界维度与 nx 不一致）时抛出异常
    // 流程：构造一个 x_min 维度错误的 OCP，调用 updateOcp
    // 预期效果：抛出 std::invalid_argument
    UpdaterConfig config;
    config.selection_radius = 2.0;
    config.max_step_displacement = 0.4;
    config.safety_margin = 0.2;
    config.top_k = 10;
    ProblemUpdater updater(config);

    MultiStageOCP ocp;
    StageSegment segment;
    segment.dynamics = std::make_shared<BicycleModelKappa>();
    segment.N = 1;
    segment.dt = 0.1;
    segment.v_sign = 1.0;
    segment.x_min = Vector::Constant(3, -10.0); // 维度错误
    segment.x_max = Vector::Constant(5, 10.0);
    segment.u_min = Vector::Constant(2, -1.0);
    segment.u_max = Vector::Constant(2, 1.0);
    ocp.addSegment(segment);

    Trajectory traj = makeParkingTrajectory(1);
    FixedHalfSpaceMap map({});

    EXPECT_THROW(updater.updateOcp(traj, map, ocp), std::invalid_argument);
}

TEST(ProblemUpdater, ThrowsWhenStageParamsDimensionInvalid)
{
    // 测试目的：验证 updateOcp 在已有 stage_params.p 维度既不是 0 也不是 150 时抛出异常
    // 流程：构造 OCP 并预填一个维度错误的 p，调用 updateOcp
    // 预期效果：抛出 std::invalid_argument
    UpdaterConfig config;
    config.selection_radius = 2.0;
    config.max_step_displacement = 0.4;
    config.safety_margin = 0.2;
    config.top_k = 10;
    ProblemUpdater updater(config);

    MultiStageOCP ocp = makeParkingOcp(1);
    ocp.segments()[0].stage_params.resize(1);
    ocp.segments()[0].stage_params[0].p = Vector::Zero(10);
    Trajectory traj = makeParkingTrajectory(1);
    FixedHalfSpaceMap map({});

    EXPECT_THROW(updater.updateOcp(traj, map, ocp), std::invalid_argument);
}

TEST(ProblemUpdater, AcceptsPreResizedEmptyStageParams)
{
    // 测试目的：验证调用方预分配 stage_params 为 N 个空 p 时，updateOcp 能正常初始化，
    //          而不是被 validate() 误判为非法配置。
    // 流程：构造 OCP，stage_params.resize(N) 但不填 p；调用 updateOcp 后检查生成的 p。
    // 预期效果：不抛异常，且生成的 p 维度为 kParameterDim，凸走廊区间按地图结果填充。
    UpdaterConfig config;
    config.selection_radius = 2.0;
    config.max_step_displacement = 0.4;
    config.safety_margin = 0.2;
    config.top_k = 10;
    ProblemUpdater updater(config);

    MultiStageOCP ocp = makeParkingOcp(1);
    ocp.segments()[0].stage_params.resize(1); // 空 p 占位
    Trajectory traj = makeParkingTrajectory(1);

    Vector n(2);
    n << 1.0, 0.0;
    FixedHalfSpaceMap map({ { n, 1.0 } });

    EXPECT_NO_THROW(updater.updateOcp(traj, map, ocp));
    ASSERT_EQ(static_cast<int>(ocp.segments()[0].stage_params.size()), 1);
    ASSERT_EQ(ocp.segments()[0].stage_params[0].p.size(), ProblemUpdater::kParameterDim);
    EXPECT_DOUBLE_EQ(ocp.segments()[0].stage_params[0].p(ProblemUpdater::kHalfSpaceStart), 1.0);
    EXPECT_DOUBLE_EQ(ocp.segments()[0].stage_params[0].p(ProblemUpdater::kHalfSpaceStart + 1), 0.0);
    EXPECT_DOUBLE_EQ(ocp.segments()[0].stage_params[0].p(ProblemUpdater::kInterceptStart), 1.0);
}

TEST(ProblemUpdater, ThrowsWhenTrajectoryTooShort)
{
    // 测试目的：验证 current_traj.x 长度不足时 updateOcp 抛出异常
    UpdaterConfig config;
    config.selection_radius = 2.0;
    config.max_step_displacement = 0.4;
    config.safety_margin = 0.2;
    config.top_k = 10;
    ProblemUpdater updater(config);

    MultiStageOCP ocp = makeParkingOcp(3);
    Trajectory traj;
    traj.resize(1, 5, 2); // 仅 2 个状态点，不足 3+1=4 个
    FixedHalfSpaceMap map({});

    EXPECT_THROW(updater.updateOcp(traj, map, ocp), std::invalid_argument);
}

TEST(ProblemUpdater, ThrowsWhenPoseDimensionTooSmall)
{
    // 测试目的：验证 pose 维度小于 3 时 updateOcp 抛出异常
    UpdaterConfig config;
    config.selection_radius = 2.0;
    config.max_step_displacement = 0.4;
    config.safety_margin = 0.2;
    config.top_k = 10;
    ProblemUpdater updater(config);

    MultiStageOCP ocp = makeParkingOcp(1);
    Trajectory traj;
    traj.resize(1, 2, 2); // nx=2 导致 pose 维度不足
    FixedHalfSpaceMap map({});

    EXPECT_THROW(updater.updateOcp(traj, map, ocp), std::invalid_argument);
}

TEST(ProblemUpdater, ThrowsWhenPoseContainsNonFiniteValue)
{
    // 测试目的：验证 current_traj.x 前三维包含非有限值时 updateOcp 抛出异常
    UpdaterConfig config;
    config.selection_radius = 2.0;
    config.max_step_displacement = 0.4;
    config.safety_margin = 0.2;
    config.top_k = 10;
    ProblemUpdater updater(config);

    MultiStageOCP ocp = makeParkingOcp(1);
    Trajectory traj = makeParkingTrajectory(1);
    traj.x[0](1) = std::numeric_limits<double>::quiet_NaN();
    FixedHalfSpaceMap map({});

    EXPECT_THROW(updater.updateOcp(traj, map, ocp), std::invalid_argument);
}

TEST(ProblemUpdater, ThrowsWhenHalfSpaceInvalid)
{
    // 测试目的：验证地图返回非法 HalfSpace（normal 维度非 2 或含非有限值）时抛出异常
    UpdaterConfig config;
    config.selection_radius = 2.0;
    config.max_step_displacement = 0.4;
    config.safety_margin = 0.2;
    config.top_k = 10;
    ProblemUpdater updater(config);

    MultiStageOCP ocp = makeParkingOcp(1);
    Trajectory traj = makeParkingTrajectory(1);

    // normal 维度为 3
    Vector bad_normal(3);
    bad_normal << 1.0, 0.0, 0.0;
    FixedHalfSpaceMap bad_map({ { bad_normal, 1.0 } });
    EXPECT_THROW(updater.updateOcp(traj, bad_map, ocp), std::invalid_argument);

    // 含 NaN
    Vector nan_normal(2);
    nan_normal << 1.0, std::numeric_limits<double>::quiet_NaN();
    FixedHalfSpaceMap nan_map({ { nan_normal, 1.0 } });
    EXPECT_THROW(updater.updateOcp(traj, nan_map, ocp), std::invalid_argument);

    // 零法向量：会静默丢失障碍物，必须拒绝
    Vector zero_normal(2);
    zero_normal << 0.0, 0.0;
    FixedHalfSpaceMap zero_map({ { zero_normal, 0.0 } });
    EXPECT_THROW(updater.updateOcp(traj, zero_map, ocp), std::invalid_argument);
}

TEST(ProblemUpdater, ThrowsForInvalidDistanceConfig)
{
    // 测试目的：验证 UpdaterConfig 中距离/幅值参数非法时构造阶段即被拒绝
    // 流程：构造 selection_radius 非正/非有限、max_step_displacement 或 safety_margin 为负/非有限的配置
    // 预期效果：均抛出 std::invalid_argument
    UpdaterConfig config;
    config.selection_radius = 2.0;
    config.max_step_displacement = 0.4;
    config.safety_margin = 0.2;
    config.top_k = 5;

    // selection_radius 非有限
    {
        UpdaterConfig bad = config;
        bad.selection_radius = std::numeric_limits<double>::infinity();
        EXPECT_THROW([&] { ProblemUpdater updater(bad); }(), std::invalid_argument);
    }
    // selection_radius 非正
    {
        UpdaterConfig bad = config;
        bad.selection_radius = 0.0;
        EXPECT_THROW([&] { ProblemUpdater updater(bad); }(), std::invalid_argument);
    }
    // max_step_displacement 为负
    {
        UpdaterConfig bad = config;
        bad.max_step_displacement = -0.1;
        EXPECT_THROW([&] { ProblemUpdater updater(bad); }(), std::invalid_argument);
    }
    // max_step_displacement 非有限
    {
        UpdaterConfig bad = config;
        bad.max_step_displacement = std::numeric_limits<double>::quiet_NaN();
        EXPECT_THROW([&] { ProblemUpdater updater(bad); }(), std::invalid_argument);
    }
    // safety_margin 为负
    {
        UpdaterConfig bad = config;
        bad.safety_margin = -0.1;
        EXPECT_THROW([&] { ProblemUpdater updater(bad); }(), std::invalid_argument);
    }
    // safety_margin 非有限
    {
        UpdaterConfig bad = config;
        bad.safety_margin = std::numeric_limits<double>::quiet_NaN();
        EXPECT_THROW([&] { ProblemUpdater updater(bad); }(), std::invalid_argument);
    }
}

TEST(SimpleParkingMap, FiltersWallsByGjkContourDistance)
{
    // 测试目的：验证 SimpleParkingMap 基于 GJK 车辆轮廓距离筛选半空间
    // 流程：构造一堵位于 x=5 的垂直墙，车辆在不同 x 位置查询
    // 预期效果：只有在 selection_radius 内的位姿才返回该墙，且按距离排序
    SimpleParkingMap map;
    Vector n(2);
    n << 1.0, 0.0;
    // 墙：x <= 5，边界线段从 (5, -10) 到 (5, 10)
    map.addWall({ n, 5.0 }, Eigen::Vector2d(5.0, -10.0), Eigen::Vector2d(5.0, 10.0));

    // 车辆位于 x=0，车头最前端约 x=3.7，到墙距离约 1.3，在 radius=2 内
    Vector pose(3);
    pose << 0.0, 0.0, 0.0;
    auto hs_near = map.queryHalfSpaces(pose, 2.0, 10);
    EXPECT_EQ(hs_near.size(), 1u);
    EXPECT_DOUBLE_EQ(hs_near[0].intercept, 5.0);

    // radius=1.0 时筛选不到（轮廓距离 > 1.0）
    auto hs_far = map.queryHalfSpaces(pose, 1.0, 10);
    EXPECT_EQ(hs_far.size(), 0u);
}

TEST(SimpleParkingMap, FiniteWallUsesSegmentDistanceNotInfiniteBoundary)
{
    // 测试目的：明确 SimpleParkingMap 的墙是有限线段障碍物，不是无限半空间边界。
    // 流程：构造一堵很短的墙 segment 从 (5, -0.5) 到 (5, 0.5)；
    //      车辆位于 (0, 100)，其到无限边界线 x=5 的距离只有 5（在 radius=6 内），
    //      但到有限线段很远，不应被选中。
    // 预期效果：queryHalfSpaces 不返回该墙。
    SimpleParkingMap map;
    Vector n(2);
    n << 1.0, 0.0;
    map.addWall({ n, 5.0 }, Eigen::Vector2d(5.0, -0.5), Eigen::Vector2d(5.0, 0.5));

    Vector pose(3);
    pose << 0.0, 100.0, 0.0;
    auto hs = map.queryHalfSpaces(pose, 6.0, 10);
    EXPECT_EQ(hs.size(), 0u);
}

TEST(SimpleParkingMap, RejectsInvalidQueryInputs)
{
    // 测试目的：验证 SimpleParkingMap::queryHalfSpaces 对非法输入抛出异常
    SimpleParkingMap map;
    Vector n(2);
    n << 1.0, 0.0;
    map.addWall({ n, 5.0 }, Eigen::Vector2d(5.0, -10.0), Eigen::Vector2d(5.0, 10.0));

    Vector pose(3);
    pose << 0.0, 0.0, 0.0;

    // pose 维度不足
    Vector short_pose(2);
    short_pose << 0.0, 0.0;
    EXPECT_THROW(map.queryHalfSpaces(short_pose, 2.0, 10), std::invalid_argument);

    // pose 含 NaN
    Vector nan_pose(3);
    nan_pose << 0.0, std::numeric_limits<double>::quiet_NaN(), 0.0;
    EXPECT_THROW(map.queryHalfSpaces(nan_pose, 2.0, 10), std::invalid_argument);

    // selection_radius 为负
    EXPECT_THROW(map.queryHalfSpaces(pose, -1.0, 10), std::invalid_argument);

    // selection_radius 为 NaN
    EXPECT_THROW(
        map.queryHalfSpaces(pose, std::numeric_limits<double>::quiet_NaN(), 10),
        std::invalid_argument);

    // top_k 为负
    EXPECT_THROW(map.queryHalfSpaces(pose, 2.0, -1), std::invalid_argument);
}

TEST(SimpleParkingMap, RejectsInvalidWallData)
{
    // 测试目的：验证 SimpleParkingMap::addWall 对非法墙数据抛出异常
    SimpleParkingMap map;
    Vector n(2);
    n << 1.0, 0.0;

    // normal 维度错误
    Vector bad_n(3);
    bad_n << 1.0, 0.0, 0.0;
    EXPECT_THROW(
        map.addWall({ bad_n, 5.0 }, Eigen::Vector2d(5.0, -10.0), Eigen::Vector2d(5.0, 10.0)),
        std::invalid_argument);

    // normal 含 NaN
    Vector nan_n(2);
    nan_n << 1.0, std::numeric_limits<double>::quiet_NaN();
    EXPECT_THROW(
        map.addWall({ nan_n, 5.0 }, Eigen::Vector2d(5.0, -10.0), Eigen::Vector2d(5.0, 10.0)),
        std::invalid_argument);

    // 截距含 NaN
    EXPECT_THROW(
        map.addWall({ n, std::numeric_limits<double>::quiet_NaN() },
            Eigen::Vector2d(5.0, -10.0), Eigen::Vector2d(5.0, 10.0)),
        std::invalid_argument);

    // 端点含 NaN
    EXPECT_THROW(map.addWall({ n, 5.0 }, Eigen::Vector2d(5.0, std::numeric_limits<double>::quiet_NaN()),
                    Eigen::Vector2d(5.0, 10.0)),
        std::invalid_argument);

    // 端点重合（线段退化）
    EXPECT_THROW(map.addWall({ n, 5.0 }, Eigen::Vector2d(5.0, 0.0), Eigen::Vector2d(5.0, 0.0)),
        std::invalid_argument);

    // 零法向量：会静默丢失障碍物，必须拒绝
    Vector zero_n(2);
    zero_n << 0.0, 0.0;
    EXPECT_THROW(
        map.addWall({ zero_n, 0.0 }, Eigen::Vector2d(0.0, -10.0), Eigen::Vector2d(0.0, 10.0)),
        std::invalid_argument);
}

TEST(SimpleParkingMap, RejectsWallNotOnHalfSpaceBoundary)
{
    // 测试目的：验证 addWall 拒绝 half-space 与线段几何不一致的墙，避免 GJK 距离和约束语义冲突
    // 流程：构造 half-space 为 x <= 5，但线段位于 x = 100
    // 预期效果：抛出 std::invalid_argument
    SimpleParkingMap map;
    Vector n(2);
    n << 1.0, 0.0;
    EXPECT_THROW(map.addWall({ n, 5.0 }, Eigen::Vector2d(100.0, -10.0),
                    Eigen::Vector2d(100.0, 10.0)),
        std::invalid_argument);
}

TEST(SimpleParkingMap, ReturnsWallsOrderedByGjkDistance)
{
    // 测试目的：验证 queryHalfSpaces 按 GJK 轮廓距离升序返回，并正确执行 top_k 截断
    // 流程：添加三堵垂直墙，距离车辆由近到远分别为 x=5, 10, 20，设 top_k=2
    // 预期效果：返回截距为 5 和 10 的墙，顺序正确。
    SimpleParkingMap map;
    Vector n(2);
    n << 1.0, 0.0;
    map.addWall({ n, 5.0 }, Eigen::Vector2d(5.0, -10.0), Eigen::Vector2d(5.0, 10.0));
    map.addWall({ n, 10.0 }, Eigen::Vector2d(10.0, -10.0), Eigen::Vector2d(10.0, 10.0));
    map.addWall({ n, 20.0 }, Eigen::Vector2d(20.0, -10.0), Eigen::Vector2d(20.0, 10.0));

    Vector pose(3);
    pose << 0.0, 0.0, 0.0;
    auto hs = map.queryHalfSpaces(pose, 100.0, 2);
    ASSERT_EQ(hs.size(), 2u);
    EXPECT_DOUBLE_EQ(hs[0].intercept, 5.0);
    EXPECT_DOUBLE_EQ(hs[1].intercept, 10.0);
}

TEST(SimpleParkingMap, GjkDistanceUsesRotatedVehicleCorners)
{
    // 测试目的：验证 GJK 筛选使用车辆旋转后的四角点，而非后轴中心距离
    // 流程：同一堵墙 x=3，theta=0 时车身前缘已越过墙（应被选中），
    //      theta=pi/2 时车身沿 y 轴，x 范围仅 [-0.9, 0.9]，在 radius=2 外（不应被选中）。
    // 预期效果：两种朝向返回结果不同。
    SimpleParkingMap map;
    Vector n(2);
    n << 1.0, 0.0;
    map.addWall({ n, 3.0 }, Eigen::Vector2d(3.0, -10.0), Eigen::Vector2d(3.0, 10.0));

    Vector pose(3);
    pose << 0.0, 0.0, 0.0;
    EXPECT_EQ(map.queryHalfSpaces(pose, 2.0, 10).size(), 1u);

    pose << 0.0, 0.0, M_PI / 2.0;
    EXPECT_EQ(map.queryHalfSpaces(pose, 2.0, 10).size(), 0u);
}

// 测试辅助：暴露 protected 的 computeVehicleCorners
class TestableSimpleParkingMap : public SimpleParkingMap {
public:
    using SimpleParkingMap::computeVehicleCorners;
};

TEST(SimpleParkingMap, CornersAgreeWithConvexCorridorConstraint)
{
    // 测试目的：验证 SimpleParkingMap 的车辆角点与 CasADi 生成凸走廊约束使用的角点一致，
    //          防止两边车辆几何常量漂移导致筛选半径与约束语义不一致。
    // 流程：对给定 pose 用 SimpleParkingMap 计算四角点；对每个角点构造一个刚好经过它的半空间，
    //      调用 ConvexCorridorConstraint 评估对应角点的约束值。
    // 预期效果：对应角点的约束值 g 近似为 0。
    TestableSimpleParkingMap map;
    const Vector pose = (Vector(5) << 1.0, 2.0, 0.3, 0.0, 0.0).finished();
    const auto corners = map.computeVehicleCorners(pose);
    ASSERT_EQ(corners.size(), 4u);

    constexpr int nu = 2;
    for (size_t corner_idx = 0; corner_idx < corners.size(); ++corner_idx) {
        Vector p = Vector::Zero(ProblemUpdater::kParameterDim);
        Vector normal(2);
        normal << corners[corner_idx].x(), corners[corner_idx].y();
        const double intercept = normal.dot(corners[corner_idx]);
        p(ProblemUpdater::kHalfSpaceStart + 0) = normal.x();
        p(ProblemUpdater::kHalfSpaceStart + 1) = normal.y();
        p(ProblemUpdater::kInterceptStart + 0) = intercept;

        ConvexCorridorConstraint corridor(nu);
        Vector g;
        corridor.evaluate(pose, Vector::Zero(nu), p, g);
        EXPECT_NEAR(g(static_cast<int>(corner_idx) * ProblemUpdater::kMaxHalfSpaces + 0), 0.0,
            1e-9)
            << "corner " << corner_idx << " differs from ConvexCorridorConstraint corner";
    }
}

TEST(ProblemUpdaterIntegration, StageParametersFlowIntoConvexCorridorConstraint)
{
    // 测试目的：验证 ProblemUpdater -> stage_params -> SQP 装配路径上的约束求值
    //          会随半空间参数变化而变化。
    // 流程：构造带 ConvexCorridorConstraint 的 OCP，用 ProblemUpdater 分别填充
    //      宽松/紧半空间参数，直接对同一个状态调用约束 evaluate。
    // 预期效果：紧半空间下至少有一个角点约束被违反（g > 0），宽松半空间下无违反。
    const int N = 1;
    const int nx = 5, nu = 2;
    MultiStageOCP ocp;
    StageSegment segment;
    segment.dynamics = std::make_shared<BicycleModelKappa>();
    segment.N = N;
    segment.dt = 0.1;
    segment.v_sign = 1.0;
    segment.x_min = Vector::Constant(nx, -10.0);
    segment.x_max = Vector::Constant(nx, 10.0);
    segment.u_min = Vector::Constant(nu, -1.0);
    segment.u_max = Vector::Constant(nu, 1.0);
    segment.constraints.push_back(std::make_shared<ConvexCorridorConstraint>(Vector::Zero(ProblemUpdater::kParameterDim), nu));
    ocp.addSegment(segment);

    Trajectory init_guess;
    init_guess.resize(N, nx, nu);
    // 车辆后轴中心在 x=0，theta=0，前左角点 x ≈ 3.7
    init_guess.x[0] << 0.0, 0.0, 0.0, 0.5, 0.0;
    init_guess.u[0].setZero();

    UpdaterConfig config;
    config.selection_radius = 200.0;
    config.max_step_displacement = 0.1;
    config.safety_margin = 0.05;
    config.top_k = 10;
    ProblemUpdater updater(config);

    auto* corridor = dynamic_cast<const ConvexCorridorConstraint*>(segment.constraints[0].get());
    ASSERT_NE(corridor, nullptr);

    // 场景 A：宽松半空间（x <= 100），无违反
    SimpleParkingMap map_loose;
    {
        Vector n(2);
        n << 1.0, 0.0;
        map_loose.addWall({ n, 100.0 }, Eigen::Vector2d(100.0, -100.0),
            Eigen::Vector2d(100.0, 100.0));
    }
    updater.updateOcp(init_guess, map_loose, ocp);
    Vector g_loose;
    corridor->evaluate(init_guess.x[0], init_guess.u[0],
        ocp.segments()[0].stage_params[0].p, g_loose);
    EXPECT_LE(g_loose.maxCoeff(), 1e-12)
        << "Loose half-space should not produce constraint violation at current state";

    // 场景 B：紧半空间（x <= 3.5），前左角点 x≈3.7 > 3.5，应产生违反
    SimpleParkingMap map_tight;
    {
        Vector n(2);
        n << 1.0, 0.0;
        map_tight.addWall({ n, 3.5 }, Eigen::Vector2d(3.5, -10.0),
            Eigen::Vector2d(3.5, 10.0));
    }
    updater.updateOcp(init_guess, map_tight, ocp);
    Vector g_tight;
    corridor->evaluate(init_guess.x[0], init_guess.u[0],
        ocp.segments()[0].stage_params[0].p, g_tight);
    EXPECT_GT(g_tight.maxCoeff(), 1e-6)
        << "Tight half-space should produce constraint violation at current state";
}

TEST(ProblemUpdaterIntegration, SqpSolverAcceptsStageParameters)
{
    // 测试目的：验证 SQP 主求解器在存在 stage_params 且约束较松时仍能正常收敛，
    //          说明 per-step p 的注入没有破坏 SQP 装配。
    // 流程：构造带 ConvexCorridorConstraint + QuadraticTrackingCost 的 OCP，
    //      用 ProblemUpdater 填充宽松半空间，调用 SQPSolver。
    // 预期效果：solve 返回 true。
    const int N = 5;
    const int nx = 5, nu = 2;
    MultiStageOCP ocp;
    StageSegment segment;
    segment.dynamics = std::make_shared<BicycleModelKappa>();
    segment.N = N;
    segment.dt = 0.1;
    segment.v_sign = 1.0;
    segment.x_min = Vector::Constant(nx, -10.0);
    segment.x_max = Vector::Constant(nx, 10.0);
    segment.u_min = Vector::Constant(nu, -1.0);
    segment.u_max = Vector::Constant(nu, 1.0);
    segment.cost = std::make_shared<QuadraticTrackingCost>(Vector::Zero(nx), Matrix::Identity(nx, nx), Matrix::Identity(nu, nu) * 0.1);
    segment.constraints.push_back(std::make_shared<ConvexCorridorConstraint>(Vector::Zero(ProblemUpdater::kParameterDim), nu));
    ocp.addSegment(segment);

    Trajectory init_guess;
    init_guess.resize(N, nx, nu);
    for (int k = 0; k <= N; ++k) {
        init_guess.x[k] << 0.0, 0.0, 0.0, 0.5, 0.0;
    }
    for (int k = 0; k < N; ++k) {
        init_guess.u[k].setZero();
    }

    SimpleParkingMap map;
    {
        Vector n(2);
        n << 1.0, 0.0;
        map.addWall({ n, 100.0 }, Eigen::Vector2d(100.0, -100.0),
            Eigen::Vector2d(100.0, 100.0));
    }
    UpdaterConfig config;
    config.selection_radius = 200.0;
    config.max_step_displacement = 0.1;
    config.safety_margin = 0.05;
    config.top_k = 10;
    ProblemUpdater updater(config);
    updater.updateOcp(init_guess, map, ocp);

    SQPSolver solver(std::make_unique<DenseQPSolver>());
    solver.options().max_iter = 5;
    Trajectory solution;
    EXPECT_TRUE(solver.solve(ocp, init_guess, solution));
}

TEST(ProblemUpdaterIntegration, SqpInjectsDifferentParametersPerStep)
{
    // 测试目的：直接证明 SQP 内部在按 step 显式传入 Constraint::evaluate(x,u,p) 的 p，
    //          且传入的 p 与 ProblemUpdater 写入的 stage_params[i].p 一致。
    // 流程：构造一个两段的 OCP，使用 RecordingConstraint；ProblemUpdater 通过
    //      PoseDependentMap 为 step 0/1 写入不同截距的 p；调用 SQPSolver；
    //      检查 RecordingConstraint 收到的所有 p 中确实出现了两个不同的截距。
    // 预期效果：记录中至少包含 step 0 的截距 10.0 和 step 1 的截距 11.0。
    const int N = 2;
    const int nx = 5, nu = 2;
    MultiStageOCP ocp;
    StageSegment segment;
    segment.dynamics = std::make_shared<BicycleModelKappa>();
    segment.N = N;
    segment.dt = 0.1;
    segment.v_sign = 1.0;
    segment.x_min = Vector::Constant(nx, -10.0);
    segment.x_max = Vector::Constant(nx, 10.0);
    segment.u_min = Vector::Constant(nu, -1.0);
    segment.u_max = Vector::Constant(nu, 1.0);
    segment.cost = std::make_shared<QuadraticTrackingCost>(Vector::Zero(nx), Matrix::Identity(nx, nx), Matrix::Identity(nu, nu) * 0.1);
    auto recording = std::make_shared<RecordingConstraint>(nx, nu);
    segment.constraints.emplace_back(recording);
    ocp.addSegment(segment);

    Trajectory init_guess;
    init_guess.resize(N, nx, nu);
    init_guess.x[0] << 0.0, 0.0, 0.0, 0.5, 0.0;
    init_guess.x[1] << 1.0, 0.0, 0.0, 0.5, 0.0;
    init_guess.x[2] << 2.0, 0.0, 0.0, 0.5, 0.0;
    for (int k = 0; k < N; ++k) {
        init_guess.u[k].setZero();
    }

    UpdaterConfig config;
    config.selection_radius = 200.0;
    config.max_step_displacement = 0.1;
    config.safety_margin = 0.05;
    config.top_k = 10;
    ProblemUpdater updater(config);
    PoseDependentMap map;
    updater.updateOcp(init_guess, map, ocp);

    // 先确认 ProblemUpdater 确实写了不同的 p
    ASSERT_EQ(ocp.segments()[0].stage_params.size(), 2u);
    const double intercept_step0 = ocp.segments()[0].stage_params[0].p(
        ProblemUpdater::kInterceptStart);
    const double intercept_step1 = ocp.segments()[0].stage_params[1].p(
        ProblemUpdater::kInterceptStart);
    EXPECT_DOUBLE_EQ(intercept_step0, 10.0);
    EXPECT_DOUBLE_EQ(intercept_step1, 11.0);

    SQPSolver solver(std::make_unique<DenseQPSolver>());
    solver.options().max_iter = 5;
    Trajectory solution;
    EXPECT_TRUE(solver.solve(ocp, init_guess, solution));

    // SQP 内部每轮迭代会在 linearize/assemble/merit/convergence 多处求值约束，
    // 因此记录中会出现多次；只要两个 step 的截距都至少出现一次，即证明按 step 显式传参。
    bool saw_step0 = false, saw_step1 = false;
    for (const auto& p : recording->recordedParams()) {
        if (p.size() != ProblemUpdater::kParameterDim) {
            continue;
        }
        const double intercept = p(ProblemUpdater::kInterceptStart);
        if (std::abs(intercept - intercept_step0) < 1e-12) {
            saw_step0 = true;
        }
        if (std::abs(intercept - intercept_step1) < 1e-12) {
            saw_step1 = true;
        }
    }
    EXPECT_TRUE(saw_step0) << "SQP did not inject step 0 p into constraint";
    EXPECT_TRUE(saw_step1) << "SQP did not inject step 1 p into constraint";
}

TEST(ProblemUpdaterIntegration, MultiSegmentInjectionUsesStepInSegment)
{
    // 测试目的：验证 SQP 在多段 OCP 中按 step_in_segment（每段从 0 开始）显式传参 p，
    //          而非误用 global_k 导致越界或读到错段参数。
    // 流程：构造两段 OCP（N1=2, N2=3），每段均含独立的 RecordingConstraint；
    //      轨迹 x 全局递增，PoseDependentMap 使每步 p 的截距唯一；
    //      调用 SQPSolver 并检查两个 RecordingConstraint 收到的 p 与 stage_params 精确对应。
    // 预期效果：solve 返回 true，且每段每个 step 的截距都至少被传入一次。
    const int N1 = 2;
    const int N2 = 3;
    const int nx = 5, nu = 2;

    MultiStageOCP ocp;
    auto makeSegment = [&](int N) {
        StageSegment segment;
        segment.dynamics = std::make_shared<BicycleModelKappa>();
        segment.N = N;
        segment.dt = 0.1;
        segment.v_sign = 1.0;
        segment.x_min = Vector::Constant(nx, -10.0);
        segment.x_max = Vector::Constant(nx, 10.0);
        segment.u_min = Vector::Constant(nu, -1.0);
        segment.u_max = Vector::Constant(nu, 1.0);
        segment.cost = std::make_shared<QuadraticTrackingCost>(Vector::Zero(nx), Matrix::Identity(nx, nx), Matrix::Identity(nu, nu) * 0.1);
        return segment;
    };

    StageSegment seg1 = makeSegment(N1);
    auto recording1 = std::make_shared<RecordingConstraint>(nx, nu);
    seg1.constraints.emplace_back(recording1);
    ocp.addSegment(seg1);

    StageSegment seg2 = makeSegment(N2);
    auto recording2 = std::make_shared<RecordingConstraint>(nx, nu);
    seg2.constraints.emplace_back(recording2);
    ocp.addSegment(seg2);

    Trajectory init_guess;
    init_guess.resize(N1 + N2, nx, nu);
    for (int k = 0; k <= N1 + N2; ++k) {
        // x 取较小递增值，既保证每步截距不同，又使初始猜测接近目标零状态，便于收敛
        init_guess.x[k] << 0.1 * static_cast<double>(k), 0.0, 0.0, 0.5, 0.0;
    }
    for (int k = 0; k < N1 + N2; ++k) {
        init_guess.u[k].setZero();
    }

    UpdaterConfig config;
    config.selection_radius = 200.0;
    config.max_step_displacement = 0.1;
    config.safety_margin = 0.05;
    config.top_k = 10;
    ProblemUpdater updater(config);
    PoseDependentMap map;
    updater.updateOcp(init_guess, map, ocp);

    ASSERT_EQ(ocp.segments()[0].stage_params.size(), static_cast<std::size_t>(N1));
    ASSERT_EQ(ocp.segments()[1].stage_params.size(), static_cast<std::size_t>(N2));

    std::vector<double> expected1, expected2;
    for (int i = 0; i < N1; ++i) {
        expected1.push_back(
            ocp.segments()[0].stage_params[i].p(ProblemUpdater::kInterceptStart));
    }
    for (int i = 0; i < N2; ++i) {
        expected2.push_back(
            ocp.segments()[1].stage_params[i].p(ProblemUpdater::kInterceptStart));
    }

    SQPSolver solver(std::make_unique<DenseQPSolver>());
    solver.options().max_iter = 2;
    Trajectory solution;
    EXPECT_TRUE(solver.solve(ocp, init_guess, solution));

    auto containsAllIntercepts = [](const std::vector<Vector>& recorded,
                                     const std::vector<double>& expected) {
        for (double val : expected) {
            bool found = false;
            for (const auto& p : recorded) {
                if (p.size() == ProblemUpdater::kParameterDim
                    && std::abs(p(ProblemUpdater::kInterceptStart) - val) < 1e-12) {
                    found = true;
                    break;
                }
            }
            EXPECT_TRUE(found) << "Expected intercept " << val << " not found in recorded params";
        }
    };

    containsAllIntercepts(recording1->recordedParams(), expected1);
    containsAllIntercepts(recording2->recordedParams(), expected2);
}

TEST(ProblemUpdaterIntegration, TightCorridorMakesSqpInfeasible)
{
    // 测试目的：验证紧凸走廊约束确实进入 SQP 的 QP/merit 路径并产生可观测行为（失败）。
    // 流程：构造初始状态已违反紧半空间的 OCP（车辆前缘 x≈3.7 > 墙截距 3.5），
    //      由于 SQP 固定 delta_x0=0，第一步约束无法被满足，QP 应不可行。
    // 预期效果：SQPSolver::solve() 返回 false。
    const int N = 1;
    const int nx = 5, nu = 2;
    MultiStageOCP ocp;
    StageSegment segment;
    segment.dynamics = std::make_shared<BicycleModelKappa>();
    segment.N = N;
    segment.dt = 0.1;
    segment.v_sign = 1.0;
    segment.x_min = Vector::Constant(nx, -10.0);
    segment.x_max = Vector::Constant(nx, 10.0);
    segment.u_min = Vector::Constant(nu, -1.0);
    segment.u_max = Vector::Constant(nu, 1.0);
    segment.constraints.push_back(std::make_shared<ConvexCorridorConstraint>(Vector::Zero(ProblemUpdater::kParameterDim), nu));
    ocp.addSegment(segment);

    Trajectory init_guess;
    init_guess.resize(N, nx, nu);
    init_guess.x[0] << 0.0, 0.0, 0.0, 0.0, 0.0;
    init_guess.x[1] << 0.0, 0.0, 0.0, 0.0, 0.0;
    init_guess.u[0].setZero();

    SimpleParkingMap map;
    {
        Vector n(2);
        n << 1.0, 0.0;
        // 墙 x <= 3.5，车辆前缘约 x=3.7，初始状态已违反
        map.addWall({ n, 3.5 }, Eigen::Vector2d(3.5, -10.0), Eigen::Vector2d(3.5, 10.0));
    }
    UpdaterConfig config;
    config.selection_radius = 200.0;
    config.max_step_displacement = 0.1;
    config.safety_margin = 0.05;
    config.top_k = 10;
    ProblemUpdater updater(config);
    updater.updateOcp(init_guess, map, ocp);

    SQPSolver solver(std::make_unique<DenseQPSolver>());
    solver.options().max_iter = 5;
    Trajectory solution;
    EXPECT_FALSE(solver.solve(ocp, init_guess, solution));
}

TEST(ProblemUpdaterIntegration, SetParametersExceptionIsCaughtBySqpSolver)
{
    // 测试目的：验证当某个约束的 evaluate() 抛异常时，SQP 主流程应记录错误并返回 false，
    //          而不是让异常逃逸到调用方。
    // 流程：构造一个只含 ThrowingEvaluationConstraint 的 OCP，并手动填入非空的
    //      stage_params；调用 SQPSolver::solve()。
    // 预期效果：solve 返回 false，且不抛出异常。
    const int N = 2;
    const int nx = 5, nu = 2;
    MultiStageOCP ocp;
    StageSegment segment;
    segment.dynamics = std::make_shared<BicycleModelKappa>();
    segment.N = N;
    segment.dt = 0.1;
    segment.v_sign = 1.0;
    segment.x_min = Vector::Constant(nx, -10.0);
    segment.x_max = Vector::Constant(nx, 10.0);
    segment.u_min = Vector::Constant(nu, -1.0);
    segment.u_max = Vector::Constant(nu, 1.0);
    segment.constraints.push_back(std::make_shared<ThrowingEvaluationConstraint>(nx, nu));
    // 手动填充合法 stage_params，确保 OCP 校验通过
    for (int i = 0; i < N; ++i) {
        segment.stage_params.push_back({ Vector::Zero(ProblemUpdater::kParameterDim) });
    }
    ocp.addSegment(segment);

    Trajectory init_guess;
    init_guess.resize(N, nx, nu);
    for (int k = 0; k <= N; ++k) {
        init_guess.x[k] << 0.0, 0.0, 0.0, 0.0, 0.0;
    }
    for (int k = 0; k < N; ++k) {
        init_guess.u[k].setZero();
    }

    SQPSolver solver(std::make_unique<DenseQPSolver>());
    solver.options().max_iter = 5;
    Trajectory solution;
    EXPECT_FALSE(solver.solve(ocp, init_guess, solution));
}
