#include <cmath>
#include <limits>
#include <stdexcept>

#include <gtest/gtest.h>

#include "core/types.h"
#include "models/dynamical_system.h"
#include "ocp/multi_stage_ocp.h"
#include "ocp/stage_parameters.h"
#include "util/constants.h"

using namespace stc_SQP;

namespace {

// 极简动力学桩：仅用于提供一致的 nx/nu，满足 addSegment 校验
class DummyDynamics : public DynamicalSystem {
public:
    DummyDynamics(int nx, int nu)
        : nx_(nx)
        , nu_(nu)
    {
    }
    int nx() const override { return nx_; }
    int nu() const override { return nu_; }
    void evaluate(const Vector& x, const Vector& u, Vector& x_dot) const override
    {
        (void)x;
        (void)u;
        x_dot.setZero(nx_);
    }
    void discretizeAndLinearize(const Vector& x, const Vector& u, double dt,
        double v_sign, Vector& x_next, Matrix& A, Matrix& B) const override
    {
        (void)u;
        (void)dt;
        (void)v_sign;
        x_next = x;
        A = Matrix::Identity(nx_, nx_);
        B = Matrix::Zero(nx_, nu_);
    }

private:
    int nx_ = 0;
    int nu_ = 0;
};

// 构造一段简单的 OCP 段
StageSegment makeSegment(int N, double dt, double v_sign,
    bool with_stage_params = false)
{
    StageSegment seg;
    seg.dynamics = std::make_shared<DummyDynamics>(/*nx=*/2, /*nu=*/1);
    seg.N = N;
    seg.dt = dt;
    seg.v_sign = v_sign;
    seg.x_min = Vector::Constant(2, -1e3);
    seg.x_max = Vector::Constant(2, 1e3);
    seg.u_min = Vector::Constant(1, -1e3);
    seg.u_max = Vector::Constant(1, 1e3);

    if (with_stage_params) {
        seg.stage_params.resize(N);
        for (int i = 0; i < N; ++i) {
            seg.stage_params[i].p = Vector::Constant(STAGE_PARAM_DIM, static_cast<double>(i));
        }
    }
    return seg;
}

} // namespace

// 测试目的：coarsen 在多段不等时长 OCP 上正确按时间比例分配步数并守恒总时长
// 流程：构造两段 OCP（段时间分别为 1.0s 和 4.0s），期望粗化到 5 步；
//      检查各段粗步数、粗 dt 以及总时长是否保持。
// 预期效果：第一段 1 步、第二段 4 步，每段粗 dt 等于该段总时长除以该段步数，
//          粗 OCP 总时长仍为 5.0s。
TEST(MultiStageOCP, CoarsenPreservesSegmentTimeAndTotalSteps) {
    MultiStageOCP ocp;
    ocp.addSegment(makeSegment(/*N=*/10, /*dt=*/0.1, /*v_sign=*/1.0)); // 1.0 s
    ocp.addSegment(makeSegment(/*N=*/20, /*dt=*/0.2, /*v_sign=*/1.0)); // 4.0 s

    MultiStageOCP coarse = ocp.coarsen(/*coarse_n=*/5, /*coarse_dt=*/-1.0);

    ASSERT_EQ(coarse.segments().size(), 2U);
    EXPECT_EQ(coarse.segments()[0].N, 1);
    EXPECT_EQ(coarse.segments()[1].N, 4);
    EXPECT_EQ(coarse.totalSteps(), 5);

    const double t0 = 1.0;
    const double t1 = 4.0;
    EXPECT_NEAR(coarse.segments()[0].dt, t0 / coarse.segments()[0].N, 1e-12);
    EXPECT_NEAR(coarse.segments()[1].dt, t1 / coarse.segments()[1].N, 1e-12);

    const double coarse_total_time = coarse.segments()[0].N * coarse.segments()[0].dt
        + coarse.segments()[1].N * coarse.segments()[1].dt;
    EXPECT_NEAR(coarse_total_time, t0 + t1, 1e-12);
    EXPECT_TRUE(coarse.validate());
}

// 测试目的：coarsen 对 stage_params 做最近邻下采样
// 流程：构造单段带 stage_params 的 OCP（p 的第一维设为步序号），粗化到 3 步；
//      检查粗段每步的参数是否取自最近邻的原始步。
// 预期效果：粗段 3 步分别对应原索引 0、5、9。
TEST(MultiStageOCP, CoarsenDownsamplesStageParamsNearestNeighbor) {
    MultiStageOCP ocp;
    ocp.addSegment(makeSegment(/*N=*/10, /*dt=*/0.1, /*v_sign=*/1.0,
        /*with_stage_params=*/true));

    MultiStageOCP coarse = ocp.coarsen(/*coarse_n=*/3, /*coarse_dt=*/-1.0);

    ASSERT_EQ(coarse.totalSteps(), 3);
    const auto& cseg = coarse.segments()[0];
    ASSERT_EQ(static_cast<int>(cseg.stage_params.size()), 3);

    // 最近邻下采样索引：j * (N-1) / (coarse_N-1)，再 round
    EXPECT_DOUBLE_EQ(cseg.stage_params[0].p(0), 0.0);
    EXPECT_DOUBLE_EQ(cseg.stage_params[1].p(0), 5.0);
    EXPECT_DOUBLE_EQ(cseg.stage_params[2].p(0), 9.0);
    EXPECT_TRUE(coarse.validate());
}

// 测试目的：coarsen 在 coarse_n <= 0 且 coarse_dt > 0 时按期望步长反推总步数
// 流程：构造总时长 5.0s 的单段 OCP，传入 coarse_n=0、coarse_dt=0.5；
//      检查粗化后总步数是否为 10 且通过 validate。
// 预期效果：target_total = round(5.0 / 0.5) = 10。
TEST(MultiStageOCP, CoarsenUsesCoarseDtBranch) {
    MultiStageOCP ocp;
    ocp.addSegment(makeSegment(/*N=*/50, /*dt=*/0.1, /*v_sign=*/1.0)); // 5.0 s

    MultiStageOCP coarse = ocp.coarsen(/*coarse_n=*/0, /*coarse_dt=*/0.5);

    EXPECT_EQ(coarse.totalSteps(), 10);
    EXPECT_NEAR(coarse.segments()[0].dt, 5.0 / 10, 1e-12);
    EXPECT_TRUE(coarse.validate());
}

// 测试目的：coarsen 对空 OCP 抛出异常
// 流程：构造空 MultiStageOCP 并调用 coarsen。
// 预期效果：抛出 std::invalid_argument。
TEST(MultiStageOCP, CoarsenThrowsOnEmptyOcp) {
    MultiStageOCP ocp;
    EXPECT_THROW(ocp.coarsen(3, -1.0), std::invalid_argument);
}

// 测试目的：coarsen 检测到非法总时长时抛出异常
// 流程：先加入合法段，再非法修改 dt 使总时长非有限，调用 coarsen。
// 预期效果：抛出 std::invalid_argument。
TEST(MultiStageOCP, CoarsenThrowsOnInvalidTotalTime) {
    MultiStageOCP ocp;
    ocp.addSegment(makeSegment(/*N=*/10, /*dt=*/0.1, /*v_sign=*/1.0));
    ocp.segments()[0].dt = std::numeric_limits<double>::quiet_NaN();

    EXPECT_THROW(ocp.coarsen(3, -1.0), std::invalid_argument);
}
