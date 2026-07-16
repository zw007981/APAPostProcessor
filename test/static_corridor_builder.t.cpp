#include "preprocessing/static_corridor_builder.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "core/NMPC/vehicle_circle_geometry.h"
#include "spatial/esdf_map.h"
#include "spatial/grid_map.h"
#include "util/position.h"
#include "util/trajectory_point.h"
#include "vehicle/vehicle_footprint_model.h"
#include "vehicle/vehicle_params.h"

namespace apa_post_processor {
namespace {

// 典型车辆参数：与 NMPC / Milestone 008 测试保持一致。
VehicleParams MakeVehicleParams() {
    return VehicleParams(/*length=*/4.3, /*width=*/1.8, /*wheelbase=*/2.7,
                         /*max_steer_angle=*/0.6, /*rear_overhang=*/0.8,
                         /*max_accel=*/1.0, /*max_decel=*/-1.5,
                         /*max_steer_rate=*/0.4);
}

// 默认静态舒适走廊配置。
StaticCorridorBuilderConfig MakeConfig() {
    StaticCorridorBuilderConfig config;
    config.soft_margin = 0.18;
    return config;
}

// 使用 outer_row_num=1 简化外圆几何。
VehicleFootprintModel MakeSimpleFootprintModel() {
    return VehicleFootprintModel(MakeVehicleParams(),
                                 /*heading_sample_num=*/233,
                                 /*inner_row_num=*/2, /*outer_row_num=*/1);
}

// 构造单个路径点作为 Z_ref 元素，默认 v/delta 均为 0。
TrajectoryPoint MakePathPoint(double x, double y, double theta) {
    TrajectoryPoint point(x, y, theta);
    point.setV(0.0);
    point.setDelta(0.0);
    return point;
}

// 场景：soft_margin 为负值或非有限值。
// 触发原因：Round 7 删除 hard_margin 字段后，构造函数只需校验 soft_margin
// 合法性。 预期行为：非法值（负数/NaN）均应在构造时立即抛出 invalid_argument。
TEST(StaticCorridorBuilderTest, ConstructorThrowsOnInvalidMargin) {
    StaticCorridorBuilderConfig config;
    config.soft_margin = -0.1;
    EXPECT_THROW(StaticCorridorBuilder builder(config), std::invalid_argument);
    config.soft_margin = std::numeric_limits<double>::quiet_NaN();
    EXPECT_THROW(StaticCorridorBuilder builder(config), std::invalid_argument);
}

TEST(StaticCorridorBuilderTest, BuildThrowsOnEmptyZRef) {
    const StaticCorridorBuilder builder(MakeConfig());
    const VehicleFootprintModel model = MakeSimpleFootprintModel();
    const GridMap grid_map(1.0, 3, 3, Position{0.0, 0.0}, {Position{0.0, 0.0}});
    const ESDFMap esdf_map(grid_map);
    EXPECT_THROW(builder.build({}, esdf_map, model), std::invalid_argument);
}

TEST(StaticCorridorBuilderTest, BuildThrowsOnMissingVOrDelta) {
    const StaticCorridorBuilder builder(MakeConfig());
    const VehicleFootprintModel model = MakeSimpleFootprintModel();
    const GridMap grid_map(1.0, 3, 3, Position{0.0, 0.0}, {Position{0.0, 0.0}});
    const ESDFMap esdf_map(grid_map);
    std::vector<TrajectoryPoint> z_ref;
    TrajectoryPoint point(1.0, 1.0, 0.0);
    // v/delta 未设置
    z_ref.push_back(point);
    EXPECT_THROW(builder.build(z_ref, esdf_map, model), std::invalid_argument);
}

// 验证当某个外圆圆心落在 ESDF 地图外时，build() 不会静默成功。
// 利用一个刚好让车辆后轴在地图内、但外圆圆心越界的小地图触发该路径。
// 该测试依赖 VehicleFootprintModel 的默认 outer_row_num=1 与车辆长度 4.3m
// 组合， 确保 2×2 地图下外圆必然越界；若 VehicleFootprintModel
// 参数变化，需同步调整地图尺寸。
TEST(StaticCorridorBuilderTest, BuildFailsOnEsdfQueryOutOfBounds) {
    const StaticCorridorBuilder builder(MakeConfig());
    const VehicleFootprintModel model = MakeSimpleFootprintModel();
    // 2x2 地图，范围 [0,2)x[0,2)
    const GridMap grid_map(1.0, 2, 2, Position{0.0, 0.0}, {Position{0.0, 0.0}});
    const ESDFMap esdf_map(grid_map);
    // 车辆后轴放在地图内部 (1.0, 0.5)，但 theta=0 时最前端外圆圆心会超出 x=2
    const std::vector<TrajectoryPoint> z_ref = {MakePathPoint(1.0, 0.5, 0.0)};
    const auto result = builder.build(z_ref, esdf_map, model);
    EXPECT_FALSE(result.success);
    EXPECT_NE(result.status_msg.find("Invalid ESDF sample"), std::string::npos);
}

// 场景：障碍物位于 (0,0) 单栅格，车辆沿 x 轴正向停放在 (2,0,0)。
// 触发原因：验证 Round 7 起唯一保留的舒适 soft 约束——在 Z_ref 处约束 slack
// 应等于 d_ref - R - soft_margin；小扰动后，由走廊系数恢复的线性化距离应与
// 真实 ESDF 距离接近。
// 预期行为：slack 与线性化距离均与解析预期值在容差内一致。
TEST(StaticCorridorBuilderTest,
     LinearizedDistanceMatchesEsdfNearExpansionPoint) {
    const StaticCorridorBuilder builder(MakeConfig());
    const VehicleFootprintModel model = MakeSimpleFootprintModel();
    const GridMap grid_map(1.0, 10, 10, Position{0.0, 0.0},
                           {Position{0.0, 0.0}});
    const ESDFMap esdf_map(grid_map);
    const auto local_centers =
        vehicle_circle_geometry::ExtractLocalCircleCenters(model,
                                                           CircleType::OUTER);
    ASSERT_GE(local_centers.size(), 1U);
    const double local_x = local_centers.front().x();
    const double local_y = local_centers.front().y();
    const std::vector<TrajectoryPoint> z_ref = {MakePathPoint(2.0, 0.0, 0.0)};
    const auto result = builder.build(z_ref, esdf_map, model);
    EXPECT_TRUE(result.success);
    ASSERT_GE(result.constraints.size(), 2U);
    const Eigen::VectorXd z_vec =
        (Eigen::VectorXd(5) << 2.0, 0.0, 0.0, 0.0, 0.0).finished();
    const double expected_cx = 2.0 + local_x;
    const double expected_cy = local_y;
    const double dist_ref =
        esdf_map.getDistAndGrad(expected_cx, expected_cy).first;
    // 唯一的舒适约束行（索引0）
    const auto& soft = result.constraints[0];
    const double soft_slack = soft.d - soft.c.dot(z_vec);
    const double expected_soft_slack =
        dist_ref - model.getOuterRadius() - MakeConfig().soft_margin;
    EXPECT_NEAR(soft_slack, expected_soft_slack, 1e-5);
    // 小扰动后，由走廊系数恢复的线性化距离应与真实 ESDF 距离接近
    Eigen::VectorXd z_perturbed = z_vec;
    z_perturbed(0) += 0.05;
    z_perturbed(1) += 0.03;
    const double linearized_dist = -soft.c.dot(z_perturbed) + soft.d +
                                   model.getOuterRadius() +
                                   MakeConfig().soft_margin;
    const double theta_perturbed = z_perturbed(2);
    const double cx = z_perturbed(0) + std::cos(theta_perturbed) * local_x -
                      std::sin(theta_perturbed) * local_y;
    const double cy = z_perturbed(1) + std::sin(theta_perturbed) * local_x +
                      std::cos(theta_perturbed) * local_y;
    const double true_dist = esdf_map.getDistAndGrad(cx, cy).first;
    EXPECT_NEAR(linearized_dist, true_dist, 5e-3);
}

// 验证超平面法向量对 v、delta 分量恒为 0。
TEST(StaticCorridorBuilderTest, AHasZeroVDeltaComponents) {
    const StaticCorridorBuilder builder(MakeConfig());
    const VehicleFootprintModel model = MakeSimpleFootprintModel();
    const GridMap grid_map(1.0, 10, 10, Position{0.0, 0.0},
                           {Position{0.0, 0.0}});
    const ESDFMap esdf_map(grid_map);
    std::vector<TrajectoryPoint> z_ref;
    TrajectoryPoint point(2.0, 1.0, 0.3);
    point.setV(0.5);
    point.setDelta(0.1);
    z_ref.push_back(point);
    const auto result = builder.build(z_ref, esdf_map, model);
    EXPECT_TRUE(result.success);
    for (const auto& c : result.constraints) {
        EXPECT_DOUBLE_EQ(c.c(3), 0.0);
        EXPECT_DOUBLE_EQ(c.c(4), 0.0);
    }
}

// 障碍物极远时，走廊约束应处于松弛状态（slack = d_ref - R - margin > 0）；
// 障碍物极近时，约束应紧绷甚至不可行（slack < 0）。
TEST(StaticCorridorBuilderTest, NumericalStabilityAtExtremeDistances) {
    const StaticCorridorBuilder builder(MakeConfig());
    const VehicleFootprintModel model = MakeSimpleFootprintModel();
    const auto local_centers =
        vehicle_circle_geometry::ExtractLocalCircleCenters(model,
                                                           CircleType::OUTER);
    ASSERT_GE(local_centers.size(), 1U);
    const double radius = model.getOuterRadius();
    // 极远场景：车辆在 (10,0,0)，障碍物在 (0,0)
    {
        const GridMap grid_map(1.0, 15, 15, Position{0.0, 0.0},
                               {Position{0.0, 0.0}});
        const ESDFMap esdf_map(grid_map);
        const std::vector<TrajectoryPoint> z_ref = {
            MakePathPoint(10.0, 0.0, 0.0)};
        const auto result = builder.build(z_ref, esdf_map, model);
        EXPECT_TRUE(result.success);
        const Eigen::VectorXd z_vec =
            (Eigen::VectorXd(5) << 10.0, 0.0, 0.0, 0.0, 0.0).finished();
        for (const auto& c : result.constraints) {
            const double slack = c.d - c.c.dot(z_vec);
            const double margin = MakeConfig().soft_margin;
            const double circle_local_x = local_centers[c.circle_idx].x();
            const double circle_local_y = local_centers[c.circle_idx].y();
            const double expected_cx = 10.0 + circle_local_x;
            const double expected_cy = circle_local_y;
            const double expected_slack =
                esdf_map.getDistAndGrad(expected_cx, expected_cy).first -
                radius - margin;
            EXPECT_NEAR(slack, expected_slack, 1e-4);
            EXPECT_GT(slack, 0.0);
        }
    }
    // 极近场景：车辆后轴放在使第一个外圆圆心刚好侵入障碍物内部的位置。
    // 此时 center 0 的 d_ref < R + soft_margin；Milestone 023
    // 引入自洽性修正后， 约束在 Z = Z_ref 处的 slack 被钳制为恰好
    // 0（而非负值）——这是有意为之的
    // 行为变更：保证参考点自身永远满足其对应的静态舒适约束，避免 HPIPM 在
    // 第 0 次 SQP 迭代因 Warm Start 自身违反约束而直接判定不可行。
    {
        const double local_x = local_centers.front().x();
        // 让 center 0 的圆心到障碍物 (0,0) 的 ESDF 距离约为 R + soft_margin -
        // 0.1
        const double vehicle_x =
            radius + MakeConfig().soft_margin - 0.1 - local_x;
        const GridMap grid_map(0.01, 500, 500, Position{0.0, 0.0},
                               {Position{0.0, 0.0}});
        const ESDFMap esdf_map(grid_map);
        const std::vector<TrajectoryPoint> z_ref = {
            MakePathPoint(vehicle_x, 0.0, 0.0)};
        const auto result = builder.build(z_ref, esdf_map, model);
        EXPECT_TRUE(result.success);
        const Eigen::VectorXd z_vec =
            (Eigen::VectorXd(5) << vehicle_x, 0.0, 0.0, 0.0, 0.0).finished();
        bool found_clamped = false;
        for (const auto& c : result.constraints) {
            const double slack = c.d - c.c.dot(z_vec);
            const double margin = MakeConfig().soft_margin;
            const double expected_cx =
                vehicle_x + local_centers[c.circle_idx].x();
            const double expected_cy = local_centers[c.circle_idx].y();
            const double dist_ref =
                esdf_map.getDistAndGrad(expected_cx, expected_cy).first;
            // 自洽性修正：violation = max(0, radius + margin - dist_ref)，
            // 未违反时退化为原始公式 dist_ref - radius - margin。
            const double violation = std::max(0.0, radius + margin - dist_ref);
            const double expected_slack =
                dist_ref - radius - margin + violation;
            EXPECT_NEAR(slack, expected_slack, 1e-4);
            // 参考点自身违反安全边界时，slack 恰好被钳制为 0（不为负）。
            if (c.circle_idx == 0) {
                EXPECT_NEAR(slack, 0.0, 1e-4);
                found_clamped = true;
            }
        }
        EXPECT_TRUE(found_clamped);
    }
}

// 非平凡梯度场景：障碍物位于 (0,1)，车辆在 (2,0)，要求法向量的 x、y
// 分量均非零。
TEST(StaticCorridorBuilderTest, NonTrivialGradientDirection) {
    const StaticCorridorBuilder builder(MakeConfig());
    const VehicleFootprintModel model = MakeSimpleFootprintModel();
    const GridMap grid_map(1.0, 5, 5, Position{0.0, 0.0}, {Position{0.0, 1.0}});
    const ESDFMap esdf_map(grid_map);
    const std::vector<TrajectoryPoint> z_ref = {MakePathPoint(2.0, 0.0, 0.0)};
    const auto result = builder.build(z_ref, esdf_map, model);
    EXPECT_TRUE(result.success);
    for (const auto& c : result.constraints) {
        // 法向量应同时包含 x、y 分量（障碍物在斜上方）
        EXPECT_NE(c.c(0), 0.0);
        EXPECT_NE(c.c(1), 0.0);
        EXPECT_DOUBLE_EQ(c.c(3), 0.0);
        EXPECT_DOUBLE_EQ(c.c(4), 0.0);
    }
}

// 验证约束行数与矩阵形式维度正确，且约束按确定性顺序排列。
// Round 7 起每个 (point, circle) 只产生一条舒适行（不再有 hard 行），
// 预期行数 = 点数 * 圆数（而非以前的 *2）。
TEST(StaticCorridorBuilderTest, MatrixFormDimensionsAreCorrect) {
    const StaticCorridorBuilder builder(MakeConfig());
    const VehicleFootprintModel model = MakeSimpleFootprintModel();
    const GridMap grid_map(1.0, 10, 10, Position{0.0, 0.0},
                           {Position{0.0, 0.0}});
    const ESDFMap esdf_map(grid_map);
    std::vector<TrajectoryPoint> z_ref;
    for (int i = 0; i < 3; ++i) {
        z_ref.push_back(MakePathPoint(1.0 + i, 1.0, 0.0));
    }
    const auto result = builder.build(z_ref, esdf_map, model);
    EXPECT_TRUE(result.success);
    const int circle_num =
        static_cast<int>(model.getCircleNum(CircleType::OUTER));
    const int expected_constraints = 3 * circle_num;
    EXPECT_EQ(static_cast<int>(result.constraints.size()),
              expected_constraints);
    EXPECT_EQ(result.c_matrix.rows(), expected_constraints);
    EXPECT_EQ(result.c_matrix.cols(), 5);
    EXPECT_EQ(result.d_vector.size(), expected_constraints);
    // 验证顺序：按 (point_idx, circle_idx) 升序
    for (int i = 1; i < expected_constraints; ++i) {
        const auto& prev = result.constraints[i - 1];
        const auto& curr = result.constraints[i];
        EXPECT_LT(std::tie(prev.point_idx, prev.circle_idx),
                  std::tie(curr.point_idx, curr.circle_idx));
    }
}

// 白盒测试 computeDScalar：通过派生类暴露 protected 方法。
class StaticCorridorBuilderTestAccessor : public StaticCorridorBuilder {
   public:
    using StaticCorridorBuilder::computeDScalar;
    using StaticCorridorBuilder::StaticCorridorBuilder;
};

TEST(StaticCorridorBuilderComputeDScalarTest, ComputesCorrectDScalar) {
    StaticCorridorBuilderTestAccessor accessor(MakeConfig());
    const Eigen::VectorXd a_row =
        (Eigen::VectorXd(5) << 1.0, 2.0, 3.0, 0.0, 0.0).finished();
    const Eigen::VectorXd z_ref =
        (Eigen::VectorXd(5) << 1.0, 1.0, 0.0, 0.0, 0.0).finished();
    const double d = accessor.computeDScalar(5.0, a_row, z_ref, 0.5, 0.1);
    // d = 5.0 - (1+2+0+0+0) - 0.5 - 0.1 = 1.4
    EXPECT_NEAR(d, 1.4, 1e-9);
}

TEST(StaticCorridorBuilderComputeDScalarTest, RejectsInvalidInputs) {
    StaticCorridorBuilderTestAccessor accessor(MakeConfig());
    const Eigen::VectorXd valid =
        (Eigen::VectorXd(5) << 1.0, 1.0, 0.0, 0.0, 0.0).finished();
    EXPECT_THROW(
        accessor.computeDScalar(std::numeric_limits<double>::quiet_NaN(), valid,
                                valid, 0.5, 0.1),
        std::invalid_argument);
    EXPECT_THROW(accessor.computeDScalar(5.0, valid, valid, 0.0, 0.1),
                 std::invalid_argument);
    EXPECT_THROW(accessor.computeDScalar(5.0, valid, valid, 0.5, -0.1),
                 std::invalid_argument);
}

}  // namespace
}  // namespace apa_post_processor
