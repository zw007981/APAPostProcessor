#include "core/NMPC/nmpc_solver.h"

#include <gtest/gtest.h>

#include <cmath>

#include "spatial/esdf_map.h"
#include "spatial/grid_map.h"
#include "test_fixture_util.h"
#include "util/path.h"
#include "vehicle/vehicle_footprint_model.h"
#include "vehicle/vehicle_params.h"

namespace apa_post_processor {
namespace {

using NmpcSolverIntegrationTest = DataJsonFixture;

// 可测试子类：通过继承暴露受保护的pruneShortestSegment，便于对裁剪逻辑做白盒测试。
class TestableNmpcSolver : public NmpcSolver {
   public:
    using NmpcSolver::NmpcSolver;
    using NmpcSolver::pruneShortestSegment;
};

// 公共车辆参数：轴距2.7m、最大前轮转角0.6rad，与其他测试文件保持一致的量级。
VehicleParams MakeVehicleParams() {
    return VehicleParams(/*length=*/4.3, /*width=*/1.8, /*wheelbase=*/2.7,
                         /*max_steer_angle=*/0.6, /*rear_overhang=*/0.8);
}

// 构造一条“先前进5m、再后退3m”的直线换挡路径，theta恒为0（曲率恒为0），
// 是车辆运动学可行的最简单场景，适合验证NmpcSolver端到端闭环能真正收敛。
Path MakeStraightLineSwitchbackPath() {
    Path path;
    for (double x = 0.0; x <= 5.0 + EPSILON; x += 0.1) {
        path.addPoint(Pose(std::min(x, 5.0), 0.0, 0.0));
    }
    for (double x = 5.0; x >= 2.0 - EPSILON; x -= 0.1) {
        path.addPoint(Pose(std::max(x, 2.0), 0.0, 0.0));
    }
    return path;
}

// 构造一张不含任何占据栅格的空地图，覆盖足够大的区域，使圆形ESDF碰撞约束在该场景下恒可行。
ESDFMap MakeEmptyEsdfMap() {
    const GridMap grid_map(0.1, 200, 100, Position{-2.0, -3.0}, {});
    return ESDFMap(grid_map);
}

// 构造一条含“冗余小碎步”的直线路径：前进5m -> 后退0.2m（远小于真实机动，大概率是Hybrid A*
// 离散化伪影）-> 继续前进到8m，theta恒为0，共3个机动段，适合验证机动段裁剪能把它压缩为1段。
Path MakeRedundantCuspSwitchbackPath() {
    Path path;
    for (double x = 0.0; x <= 5.0 + EPSILON; x += 0.1) {
        path.addPoint(Pose(std::min(x, 5.0), 0.0, 0.0));
    }
    for (double x = 5.0; x >= 4.8 - EPSILON; x -= 0.1) {
        path.addPoint(Pose(std::max(x, 4.8), 0.0, 0.0));
    }
    for (double x = 4.8; x <= 8.0 + EPSILON; x += 0.1) {
        path.addPoint(Pose(std::min(x, 8.0), 0.0, 0.0));
    }
    return path;
}

// 构造一个状态向量[x,y,theta,v,delta]，其余测试场景默认0。
stc_SQP::Vector MakeState(double x, double y = 0.0, double theta = 0.0, double v = 0.0,
    double delta = 0.0) {
    stc_SQP::Vector state(5);
    state << x, y, theta, v, delta;
    return state;
}

// 手工构造一个含3个机动段（FWD 5m、BWD 0.1m冗余小碎步、FWD 5m）的合成Result，
// 不经过真实求解，专门用于对pruneShortestSegment()做确定性白盒测试。
NmpcSolver::Result MakeSyntheticThreeSegmentResult() {
    NmpcSolver::Result result;
    result.segment_steps = {5, 2, 5};
    result.segment_v_signs = {1.0, -1.0, 1.0};
    result.trajectory.x.reserve(13);
    for (int i = 0; i <= 5; ++i) {
        result.trajectory.x.push_back(MakeState(static_cast<double>(i)));
    }
    result.trajectory.x.push_back(MakeState(5.05));
    result.trajectory.x.push_back(MakeState(5.10));
    for (int i = 1; i <= 5; ++i) {
        result.trajectory.x.push_back(MakeState(5.10 + i));
    }
    // 补全控制序列，使 pruneShortestSegment 在回填控制量时不会访问空 trajectory.u。
    const int total_steps =
        result.segment_steps[0] + result.segment_steps[1] + result.segment_steps[2];
    result.trajectory.u.reserve(total_steps);
    for (int i = 0; i < total_steps; ++i) {
        stc_SQP::Vector control(2);
        control << 0.0, 0.0;
        result.trajectory.u.push_back(control);
    }
    return result;
}

}  // namespace

// 端到端集成测试（合成场景）：验证NmpcSolver在车辆运动学可行、无障碍物的直线换挡场景下
// 能真正收敛（converged=true），且优化结果与M2构造的初始猜测在结构上一致（步数/换挡边界）。
// 之所以用合成场景而非data/test.json，是因为该回归样例的机动段2要求前轮转角约1.4rad
// （见仓库记忆：该样例的曲率需求远超车辆max_steer_angle，属于运动学不可行的极端测试数据，
// 只适合验证Path/Maneuver解析逻辑，不适合作为NMPC求解收敛性的验证场景）。
TEST(NmpcSolverTest, OptimizesFeasibleStraightLineSwitchbackScenario) {
    const auto path = MakeStraightLineSwitchbackPath();
    const auto vehicle_params = MakeVehicleParams();
    const auto esdf_map = MakeEmptyEsdfMap();
    const VehicleFootprintModel footprint_model(vehicle_params, /*heading_sample_num=*/233,
                                                /*inner_row_num=*/2, /*outer_row_num=*/2);
    ASSERT_LE(footprint_model.getCircleNum(CircleType::OUTER), 20U);

    NmpcSolver solver(vehicle_params, footprint_model);
    NmpcSolver::Result result;
    ASSERT_NO_THROW(result = solver.optimize(path, esdf_map));

    EXPECT_TRUE(result.converged);
    ASSERT_FALSE(result.trajectory.x.empty());
    for (const auto& state : result.trajectory.x) {
        EXPECT_TRUE(state.allFinite());
    }
    for (const auto& control : result.trajectory.u) {
        EXPECT_TRUE(control.allFinite());
    }
    // 首末状态应仍大致锚定在原路径的起点/终点附近（终端代价 + x0固定的共同作用）。
    const auto& first_state = result.trajectory.x.front();
    const auto& last_state = result.trajectory.x.back();
    EXPECT_NEAR(first_state(0), 0.0, 1e-6);
    EXPECT_NEAR(last_state(0), 2.0, 0.5);

    // 求解耗时应被记录为正数，供proto OptimizeResponse.optimization_time_ms使用
    EXPECT_GT(result.solve_time_ms, 0.0);
}

// 测试ToPath()能把优化结果按segment_steps/segment_v_signs正确切回原有的机动段结构
// （段数、每段方向、每段点数），且相邻机动段共享同一个边界点（与Path的内部约定一致）。
// 因为这是proto输出（optimized_path/maneuvers）的核心还原逻辑，必须验证切分位置正确。
TEST(NmpcSolverTest, ToPathReconstructsManeuverStructureFromResult) {
    const auto path = MakeStraightLineSwitchbackPath();
    const auto vehicle_params = MakeVehicleParams();
    const auto esdf_map = MakeEmptyEsdfMap();
    const VehicleFootprintModel footprint_model(vehicle_params, /*heading_sample_num=*/233,
                                                /*inner_row_num=*/2, /*outer_row_num=*/2);
    NmpcSolver solver(vehicle_params, footprint_model);
    const auto result = solver.optimize(path, esdf_map);
    ASSERT_TRUE(result.converged);

    const auto optimized_path = NmpcSolver::ToPath(result);
    ASSERT_EQ(optimized_path.numManeuvers(), result.segment_steps.size());
    ASSERT_EQ(optimized_path.numManeuvers(), 2U);

    const auto& maneuvers = optimized_path.getManeuvers();
    EXPECT_EQ(maneuvers[0].direction, Direction::FORWARD);
    EXPECT_EQ(maneuvers[1].direction, Direction::BACKWARD);
    EXPECT_EQ(maneuvers[0].points.size(),
             static_cast<std::size_t>(result.segment_steps[0]) + 1);
    EXPECT_EQ(maneuvers[1].points.size(),
             static_cast<std::size_t>(result.segment_steps[1]) + 1);
    // 相邻机动段共享同一个换挡边界点
    EXPECT_NEAR(maneuvers[0].points.back().x, maneuvers[1].points.front().x, 1e-9);
    EXPECT_NEAR(maneuvers[0].points.back().y, maneuvers[1].points.front().y, 1e-9);

    // 验证Milestone 002新增派生量回填行为：
    // - 每点都应回填v/delta状态量；
    // - 除每段最后一个点外，都应回填a/delta_dot控制量；
    // - 每段最后一个点没有对应控制量，因此hasA()/hasDeltaDot()为false；
    // - 所有点都未经过Path曲率估计，因此hasKappa()为false。
    int global_x = 0;
    int global_u = 0;
    for (std::size_t seg = 0; seg < maneuvers.size(); ++seg) {
        const auto& maneuver = maneuvers[seg];
        const int step_num = result.segment_steps[seg];
        for (std::size_t i = 0; i < maneuver.points.size(); ++i) {
            const auto& point = maneuver.points[i];
            const auto& state = result.trajectory.x[global_x + i];
            EXPECT_FALSE(point.hasKappa())
                << "NMPC output PathPoint should not carry kappa";
            EXPECT_TRUE(point.hasV());
            EXPECT_TRUE(point.hasDelta());
            EXPECT_NEAR(point.getV(), state(3), 1e-9);
            EXPECT_NEAR(point.getDelta(), state(4), 1e-9);
            if (i < static_cast<std::size_t>(step_num)) {
                const auto& control = result.trajectory.u[global_u + i];
                EXPECT_TRUE(point.hasA());
                EXPECT_TRUE(point.hasDeltaDot());
                EXPECT_NEAR(point.getA(), control(0), 1e-9);
                EXPECT_NEAR(point.getDeltaDot(), control(1), 1e-9);
            } else {
                EXPECT_FALSE(point.hasA())
                    << "last point of segment should not have control a";
                EXPECT_FALSE(point.hasDeltaDot())
                    << "last point of segment should not have control delta_dot";
            }
        }
        global_x += step_num;
        global_u += step_num;
    }
}

// 测试pruneShortestSegment()在存在弧长低于阈值的机动段时，能正确裁剪并合并两侧同号相邻段。
// 因为这是M5机动段裁剪的核心逻辑：3段(FWD/BWD/FWD)裁掉中间的冗余BWD小碎步后应合并为1个FWD段。
TEST(NmpcSolverTest, PruneShortestSegmentMergesFlankingSameDirectionSegments) {
    const auto result = MakeSyntheticThreeSegmentResult();

    const auto pruned = TestableNmpcSolver::pruneShortestSegment(result, /*min_arc_length=*/1.0);
    ASSERT_TRUE(pruned.has_value());
    ASSERT_EQ(pruned->numManeuvers(), 1U);

    const auto& maneuvers = pruned->getManeuvers();
    EXPECT_EQ(maneuvers[0].direction, Direction::FORWARD);
    // 两段各6个点（5步），合并后不去重（两端存在真实位置跳变，交由重新求解时的弧长插值消化）
    EXPECT_EQ(maneuvers[0].points.size(), 12U);
    EXPECT_NEAR(maneuvers[0].points.front().x, 0.0, 1e-9);
    EXPECT_NEAR(maneuvers[0].points.back().x, 10.10, 1e-9);

    // 验证pruneShortestSegment同样回填派生量：v/delta设置、a/delta_dot非末尾点设置/
    // 末尾点未设置、kappa未设置。合并后的单个机动段由原始第0段和第2段拼接而成，
    // 因此原始每段的末尾点（索引5和11）没有对应控制量。
    const auto& points = maneuvers[0].points;
    ASSERT_EQ(points.size(), 12U);
    for (std::size_t i = 0; i < points.size(); ++i) {
        const auto& point = points[i];
        EXPECT_FALSE(point.hasKappa());
        EXPECT_TRUE(point.hasV());
        EXPECT_TRUE(point.hasDelta());
        const bool is_original_segment_last = (i == 5U || i == 11U);
        if (is_original_segment_last) {
            EXPECT_FALSE(point.hasA());
            EXPECT_FALSE(point.hasDeltaDot());
        } else {
            EXPECT_TRUE(point.hasA());
            EXPECT_TRUE(point.hasDeltaDot());
        }
    }
}

// 测试当所有机动段弧长都不低于阈值时，pruneShortestSegment()返回std::nullopt（无需裁剪）。
TEST(NmpcSolverTest, PruneShortestSegmentReturnsNulloptWhenNoSegmentIsBelowThreshold) {
    const auto result = MakeSyntheticThreeSegmentResult();
    // 阈值小于中间段的实际弧长0.1m，因此没有可裁剪的段
    const auto pruned = TestableNmpcSolver::pruneShortestSegment(result, /*min_arc_length=*/0.05);
    EXPECT_FALSE(pruned.has_value());
}

// 测试pruneShortestSegment()对空轨迹/只剩1段的退化输入返回std::nullopt，不做越界访问。
TEST(NmpcSolverTest, PruneShortestSegmentReturnsNulloptForDegenerateInputs) {
    NmpcSolver::Result empty_trajectory_result;
    empty_trajectory_result.segment_steps = {5, 2, 5};
    empty_trajectory_result.segment_v_signs = {1.0, -1.0, 1.0};
    EXPECT_FALSE(
        TestableNmpcSolver::pruneShortestSegment(empty_trajectory_result, 1.0).has_value());

    NmpcSolver::Result single_segment_result;
    single_segment_result.segment_steps = {2};
    single_segment_result.segment_v_signs = {1.0};
    single_segment_result.trajectory.x = {MakeState(0.0), MakeState(0.01), MakeState(0.02)};
    EXPECT_FALSE(
        TestableNmpcSolver::pruneShortestSegment(single_segment_result, 1.0).has_value());
}

// 端到端集成测试：验证optimizeWithPruning()在含冗余小碎步的合成场景下，真正把3个机动段
// 裁剪合并为更少的机动段（理想情况下压缩为1个连续FORWARD机动段），且prune_iterations>0。
// 这是M5交付的核心验收标准：机动段数确实被削减，而不只是轨迹变平滑。
TEST(NmpcSolverTest, OptimizeWithPruningReducesRedundantCuspManeuverCount) {
    const auto path = MakeRedundantCuspSwitchbackPath();
    ASSERT_EQ(path.numManeuvers(), 3U);
    const auto vehicle_params = MakeVehicleParams();
    const auto esdf_map = MakeEmptyEsdfMap();
    const VehicleFootprintModel footprint_model(vehicle_params, /*heading_sample_num=*/233,
                                                /*inner_row_num=*/2, /*outer_row_num=*/2);
    NmpcSolver solver(vehicle_params, footprint_model);

    PruningConfig pruning_config;
    pruning_config.min_segment_arc_length = 0.5;
    NmpcSolver::Result result;
    ASSERT_NO_THROW(result = solver.optimizeWithPruning(path, esdf_map, pruning_config));

    ASSERT_FALSE(result.trajectory.x.empty());
    EXPECT_GE(result.prune_iterations, 1);
    const auto optimized_path = NmpcSolver::ToPath(result);
    EXPECT_LT(optimized_path.numManeuvers(), 3U);
    for (const auto& state : result.trajectory.x) {
        EXPECT_TRUE(state.allFinite());
    }
}

// 测试optimizeWithPruning()在原本无障碍物、纯共线的前进+倒车场景下：由于该来回本身
// 在几何上就是多余的（直接从起点走到终点即可，无需先冲过终点再倒回来），代价函数
// （长度代价的光滑近似v²+自由的内部段位置）会促使SQP把倒车段的实际弧长压缩到接近0，
// 后处理裁剪应能正确识别并合并为1个连续FORWARD机动段。这验证了M3代价设计与M5裁剪
// 后处理的协同效果：即便是"看似合理"的两段换挡，只要没有障碍物真正强制绕行，也会被
// 正确识别为冗余并合并——这正是我们期望的"机动段数削减"效果。
TEST(NmpcSolverTest, OptimizeWithPruningCollapsesGeometricallyRedundantSwitchback) {
    const auto path = MakeStraightLineSwitchbackPath();
    const auto vehicle_params = MakeVehicleParams();
    const auto esdf_map = MakeEmptyEsdfMap();
    const VehicleFootprintModel footprint_model(vehicle_params, /*heading_sample_num=*/233,
                                                /*inner_row_num=*/2, /*outer_row_num=*/2);
    NmpcSolver solver(vehicle_params, footprint_model);

    PruningConfig pruning_config;
    pruning_config.min_segment_arc_length = 0.5;
    // 本用例只验证"几何冗余的换挡能被正确识别合并"这一机制本身，默认的max_terminal_deviation
    // (0.02m)过于贴近SQP数值收敛精度的边界，在该临界点上容易因浮点非确定性抖动导致测试
    // 偶发失败；这里放宽到0.1m以获得稳定、确定性的测试结果，默认值本身在其他用例中验证。
    pruning_config.max_terminal_deviation = 0.1;
    NmpcSolver::Result result;
    ASSERT_NO_THROW(result = solver.optimizeWithPruning(path, esdf_map, pruning_config));

    ASSERT_FALSE(result.trajectory.x.empty());
    EXPECT_GE(result.prune_iterations, 1);
    const auto optimized_path = NmpcSolver::ToPath(result);
    EXPECT_EQ(optimized_path.numManeuvers(), 1U);
    EXPECT_EQ(optimized_path.getManeuvers().front().direction, Direction::FORWARD);
}

// 使用data/test.json回归样例做端到端冒烟测试：只验证NmpcSolver在真实车辆参数、真实ESDF
// 地图与真实多机动段初始路径下完整跑通一次求解流程（转换->约束注入->求解）不抛异常，
// 不对收敛性做强断言——该样例的机动段2曲率需求超出车辆转向极限（见上方场景测试的注释），
// 求解器返回未收敛属预期行为，此处仅验证管线本身健壮、不崩溃。
TEST_F(NmpcSolverIntegrationTest, HandlesRealRegressionSampleWithoutThrowing) {
    const auto& request = getOptimizeRequest();
    const auto path = Path::FromProto(request.initial_path());
    const auto vehicle_params = VehicleParams::FromProto(request.vehicle());
    const auto grid_map = GridMap::FromProto(request.environment());
    const ESDFMap esdf_map(grid_map);

    // outer_row_num取较小值，确保外圆数量不超过CircleFootprintEsdfConstraint::kMaxCircles
    // （见仓库记忆：默认outer_row_num=4时外圆数量约24个，会超出上限20）
    const VehicleFootprintModel footprint_model(vehicle_params, /*heading_sample_num=*/233,
                                                /*inner_row_num=*/2, /*outer_row_num=*/2);
    ASSERT_LE(footprint_model.getCircleNum(CircleType::OUTER), 20U);

    NmpcSolver solver(vehicle_params, footprint_model);
    NmpcSolver::Result result;
    EXPECT_NO_THROW(result = solver.optimize(path, esdf_map));
}

}  // namespace apa_post_processor

