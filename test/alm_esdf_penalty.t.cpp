#include "core/ALM/alm_esdf_penalty.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "core/NMPC/vehicle_circle_geometry.h"
#include "spatial/esdf_map.h"
#include "spatial/grid_map.h"
#include "vehicle/vehicle_footprint_model.h"
#include "vehicle/vehicle_params.h"

namespace apa_post_processor {
namespace {

// 墙场地图参数：第 0 列整列占据。占据栅格中心的符号距离场在 x>=2*resolution
// 区域严格满足 D=x、g=(1,0)（距离变换在该区域对线性函数精确，有限差分梯度
// 同样精确；第一个自由列因符号距离在墙面处存在折点，其梯度为 1.5 不可用）。
constexpr double kResolution = 0.125;
constexpr int kMapCols = 64;
constexpr int kMapRows = 64;
// π/2（平行于墙面的位姿航向）
const double kHalfPi = std::acos(-1.0) / 2.0;

// 梯度/代价对拍：相对误差 < rel_tol，近零分量退化为绝对误差 rel_tol*1e-6
void ExpectComponentClose(double expected, double actual, double rel_tol) {
    EXPECT_LE(std::abs(expected - actual),
              rel_tol * std::max(std::abs(actual), 1e-6))
        << "expected=" << expected << " actual=" << actual;
}

// 手推参考结果：混合代价 I_obs 与其对 (x,y,θ) 的梯度
struct ReferenceCostGrad {
    double cost{0.0};
    double grad_x{0.0};
    double grad_y{0.0};
    double grad_theta{0.0};
};

// 墙场手推参考：全部圆心落在 x>=2*resolution 时 D=cx、g=(1,0) 精确成立，
// 双重分段惩罚与链式法则可全部用双精度闭式推导（g_y 恒为 0）
ReferenceCostGrad ComputeReference(
    const std::vector<Eigen::Vector2d>& local_centers, double radius, double x,
    double y, double theta, const AlmEsdfPenaltyConfig& config) {
    const double cos_theta = std::cos(theta);
    const double sin_theta = std::sin(theta);
    ReferenceCostGrad ref;
    for (const auto& center : local_centers) {
        const double lx = center.x();
        const double ly = center.y();
        const double cx = x + cos_theta * lx - sin_theta * ly;
        const double c_safe = radius + config.margin_safe - cx;
        const double c_comf = radius + config.margin_comf - cx;
        double factor = 0.0;
        if (c_safe > 0.0) {
            ref.cost += config.weight_safe * c_safe * c_safe * c_safe;
            factor += config.weight_safe * 3.0 * c_safe * c_safe;
        }
        if (c_comf > 0.0) {
            ref.cost += config.weight_comf * c_comf * c_comf * c_comf;
            factor += config.weight_comf * 3.0 * c_comf * c_comf;
        }
        // 圆心随 θ 旋转的几何链式法则：dP_k/dθ = dR/dθ·p_k^local
        const double dcx_dtheta = -sin_theta * lx - cos_theta * ly;
        ref.grad_x -= factor;
        ref.grad_theta -= factor * dcx_dtheta;
    }
    return ref;
}

// 五点中心差分 [-f(x+2h)+8f(x+h)-8f(x-h)+f(x-2h)]/(12h)，截断误差 O(h^4)。
// 说明：ESDFMap 底层为双精度存储/插值，两点中心差分的截断误差 ~h^2 在
// h=1e-3 时即达 1e-6 量级边界；五点中心差分（同样以评估点为中心）把截断
// 压到 O(h^4)，为步长选择留出充足余量，实测相对误差可达 1e-6 以内。
// 注意场景设计必须保证差分模板范围内不跨越 max(0,C)^3 的 C=0 折点，
// 否则差分误差由折点主导（双精度下暴露得尤为明显）。
double NumericGradientComponent(const AlmEsdfPenalty& penalty, double x,
                                double y, double theta, int component,
                                double h) {
    auto cost_at = [&](double offset) {
        double px = x;
        double py = y;
        double pt = theta;
        if (component == 0) {
            px += offset;
        } else if (component == 1) {
            py += offset;
        } else {
            pt += offset;
        }
        return penalty.evaluate(px, py, pt).cost;
    };
    return (-cost_at(2.0 * h) + 8.0 * cost_at(h) - 8.0 * cost_at(-h) +
            cost_at(-2.0 * h)) /
           (12.0 * h);
}

// 公共测试夹具：直墙 ESDF 地图 + 真实车辆外圆模型（outer_row_num=1，3 个外圆，
// 局部圆心 lx∈{-0.0833, 1.35, 2.7833}、ly=0，r_outer≈1.1505）
class AlmEsdfPenaltyTest : public ::testing::Test {
   protected:
    AlmEsdfPenaltyTest()
        : grid_map_(kResolution, kMapCols, kMapRows, Position{0.0, 0.0},
                    BuildWallCells()),
          esdf_map_(grid_map_),
          veh_params_(4.3, 1.8, 2.7, 0.6, 0.8),
          footprint_model_(veh_params_, 233, 2, 1),
          circle_centers_(vehicle_circle_geometry::ExtractLocalCircleCenters(
              footprint_model_, CircleType::OUTER)),
          circle_radius_(footprint_model_.getOuterRadius()) {}

    // 构造第 0 列整列占据的栅格（占据单元为栅格中心坐标序列）
    static std::vector<Position> BuildWallCells() {
        std::vector<Position> cells;
        cells.reserve(kMapRows);
        for (int row = 0; row < kMapRows; ++row) {
            cells.emplace_back(Position{0.0, row * kResolution});
        }
        return cells;
    }

    AlmEsdfPenalty MakePenalty(AlmEsdfPenaltyConfig config = {}) const {
        return AlmEsdfPenalty(esdf_map_, footprint_model_, config);
    }

   protected:
    GridMap grid_map_;
    ESDFMap esdf_map_;
    VehicleParams veh_params_;
    VehicleFootprintModel footprint_model_;
    std::vector<Eigen::Vector2d> circle_centers_;
    double circle_radius_{0.0};
};

// 测试构造期配置校验：负 margin_safe、margin_comf 不大于 margin_safe、
// 负权重、非有限值均必须抛 std::invalid_argument。
// 因为非法配置会静默污染全部下游优化目标，必须在构造期显式拒绝。
TEST_F(AlmEsdfPenaltyTest, ConstructorThrowsOnInvalidConfig) {
    AlmEsdfPenaltyConfig config;
    config.margin_safe = -0.01;
    EXPECT_THROW(MakePenalty(config), std::invalid_argument);
    config.margin_safe = 0.02;
    config.margin_comf = 0.02;
    EXPECT_THROW(MakePenalty(config), std::invalid_argument);
    config.margin_comf = 0.10;
    config.weight_safe = -1.0;
    EXPECT_THROW(MakePenalty(config), std::invalid_argument);
    config.weight_safe = 1.0;
    config.weight_comf = std::numeric_limits<double>::quiet_NaN();
    EXPECT_THROW(MakePenalty(config), std::invalid_argument);
    config.weight_comf = 1.0;
    config.margin_safe = std::numeric_limits<double>::infinity();
    EXPECT_THROW(MakePenalty(config), std::invalid_argument);
}

// 测试舒适缓冲边界之外代价与梯度严格为零。
// 因为双重惩罚是两段外点罚，d >= r+margin_comf 时两段都不应激活。
TEST_F(AlmEsdfPenaltyTest, ReturnsZeroOutsideComfortMargin) {
    const auto penalty = MakePenalty();
    // 平行位姿（θ=π/2）下全部圆心 cx=x0>d 阈值 r+margin_comf
    const double x0 = circle_radius_ + 0.5;
    const auto result = penalty.evaluate(x0, 4.0, kHalfPi);

    EXPECT_DOUBLE_EQ(result.cost, 0.0);
    EXPECT_TRUE(result.gradient.isZero());
}

// 测试 d = r+margin_comf 边界处代价光滑归零。
// 因为 max(0,C)^3 在 C=0 处值与一阶导数都为 0，边界两侧必须 C¹ 连续。
TEST_F(AlmEsdfPenaltyTest, BoundaryAtComfortMarginIsContinuous) {
    const auto penalty = MakePenalty();
    const double x0 = circle_radius_ + penalty.config().margin_comf;
    const auto on_boundary = penalty.evaluate(x0, 4.0, kHalfPi);
    EXPECT_DOUBLE_EQ(on_boundary.cost, 0.0);
    EXPECT_TRUE(on_boundary.gradient.isZero());
    // 边界内侧 1mm：舒适段激活，代价严格为正
    const auto inside = penalty.evaluate(x0 - 1e-3, 4.0, kHalfPi);
    EXPECT_GT(inside.cost, 0.0);
}

// 测试仅舒适段激活时（r+margin_safe < d < r+margin_comf）代价/梯度与手推一致。
// 因为双重机制的关键分段特性是"safe 段不激活、comf 段单独起作用"，必须独立验证。
TEST_F(AlmEsdfPenaltyTest, HandDerivedComfortOnlyZoneMatches) {
    const auto penalty = MakePenalty();
    // 平行位姿全部圆心 cx=x0=r+0.06：C_safe=-0.04<0（safe 段不激活）、
    // C_comf=0.04>0（comf 段激活）
    const double x0 = circle_radius_ + 0.06;
    const auto result = penalty.evaluate(x0, 4.0, kHalfPi);
    const auto ref = ComputeReference(circle_centers_, circle_radius_, x0, 4.0,
                                      kHalfPi, penalty.config());

    // 弱信号区（C_comf=0.04）同样按 1e-6 验收（双精度存储下无量化噪声放大）
    ExpectComponentClose(ref.cost, result.cost, 1e-6);
    ExpectComponentClose(ref.grad_x, result.gradient.x(), 1e-6);
    EXPECT_NEAR(ref.grad_y, result.gradient.y(), 1e-9);
    ExpectComponentClose(ref.grad_theta, result.gradient.z(), 1e-6);
    // safe 段不激活 ⇒ 代价应与"仅 comf 段"解析值严格同构（参考公式中无 safe 项）
    EXPECT_GT(result.cost, 0.0);
}

// 测试两段同时激活时（d < r+margin_safe）代价/梯度与手推一致。
// 因为 safe+comf 混合代价是本模块的核心输出，强信号区容差取 1e-6。
TEST_F(AlmEsdfPenaltyTest, HandDerivedMixedZoneMatches) {
    const auto penalty = MakePenalty();
    // 平行位姿全部圆心 cx=x0=0.35：C_safe≈0.82、C_comf≈0.90，两段均激活
    const double x0 = 0.35;
    const auto result = penalty.evaluate(x0, 4.0, kHalfPi);
    const auto ref = ComputeReference(circle_centers_, circle_radius_, x0, 4.0,
                                      kHalfPi, penalty.config());

    ExpectComponentClose(ref.cost, result.cost, 1e-6);
    ExpectComponentClose(ref.grad_x, result.gradient.x(), 1e-6);
    EXPECT_NEAR(ref.grad_y, result.gradient.y(), 1e-9);
    ExpectComponentClose(ref.grad_theta, result.gradient.z(), 1e-6);
}

// 测试旋转位姿（θ≠0）下代价/梯度与手推一致。
// 因为外圆圆心随 θ 旋转，θ 通道梯度必须包含几何链式法则 dP_k/dθ=dR/dθ·p_k；
// 本用例 θ 梯度量级 ~9.24，若遗漏旋转项将与真值偏差 9 个数量级。
TEST_F(AlmEsdfPenaltyTest, HandDerivedRotatedPoseMatches) {
    const auto penalty = MakePenalty();
    // θ=atan2(0.8,-0.6)（cos=-0.6、sin=0.8，车辆前部朝向墙面）：前圆
    // （lx=2.7833）cx≈0.28 埋入惩罚区，提供强 θ 梯度信号
    const double theta = std::atan2(0.8, -0.6);
    const double x0 = 1.95;
    const auto result = penalty.evaluate(x0, 4.0, theta);
    const auto ref = ComputeReference(circle_centers_, circle_radius_, x0, 4.0,
                                      theta, penalty.config());

    ExpectComponentClose(ref.cost, result.cost, 1e-6);
    ExpectComponentClose(ref.grad_x, result.gradient.x(), 1e-6);
    EXPECT_NEAR(ref.grad_y, result.gradient.y(), 1e-9);
    ExpectComponentClose(ref.grad_theta, result.gradient.z(), 1e-6);
    EXPECT_GT(std::abs(result.gradient.z()), 1.0);
}

// 测试自定义权重与边界值下仍与手推一致。
// 因为 W_safe/W_comf 与 margin_safe/margin_comf 是外层调参的暴露面，非默认
// 配置同样必须满足解析公式。
TEST_F(AlmEsdfPenaltyTest, CustomWeightsAndMarginsMatchHandDerived) {
    AlmEsdfPenaltyConfig config;
    config.margin_safe = 0.05;
    config.margin_comf = 0.20;
    config.weight_safe = 2.0;
    config.weight_comf = 0.5;
    const auto penalty = MakePenalty(config);
    // 平行位姿 x0=0.7：C_safe≈0.50、C_comf≈0.65，两段均激活
    const double x0 = 0.7;
    const auto result = penalty.evaluate(x0, 4.0, kHalfPi);
    const auto ref = ComputeReference(circle_centers_, circle_radius_, x0, 4.0,
                                      kHalfPi, config);

    ExpectComponentClose(ref.cost, result.cost, 1e-6);
    ExpectComponentClose(ref.grad_x, result.gradient.x(), 1e-6);
    EXPECT_NEAR(ref.grad_y, result.gradient.y(), 1e-9);
    ExpectComponentClose(ref.grad_theta, result.gradient.z(), 1e-6);
}

// 测试负角度位姿（θ=-π/4）下代价/梯度与手推一致。
// 因为负角度的 cos/sin 符号组合（cos>0、sin<0）与既有 [0, π/2] 用例不同，
// 旋转链式法则 dP_k/dθ=dR/dθ·p_k 的符号处理必须在该符号组合下同样成立。
TEST_F(AlmEsdfPenaltyTest, HandDerivedNegativeAngleMatches) {
    const auto penalty = MakePenalty();
    // θ=-π/4，x0=0.4：后圆（lx=-0.0833）cx≈0.34 埋入惩罚区
    const double theta = -std::acos(-1.0) / 4.0;
    const double x0 = 0.4;
    const auto result = penalty.evaluate(x0, 4.0, theta);
    const auto ref = ComputeReference(circle_centers_, circle_radius_, x0, 4.0,
                                      theta, penalty.config());

    ExpectComponentClose(ref.cost, result.cost, 1e-6);
    ExpectComponentClose(ref.grad_x, result.gradient.x(), 1e-6);
    EXPECT_NEAR(ref.grad_y, result.gradient.y(), 1e-9);
    ExpectComponentClose(ref.grad_theta, result.gradient.z(), 1e-6);
}

// 测试双重惩罚随侵入深度严格增大（分段行为的方向性验证）。
// 因为优化的避障能力依赖惩罚对"更深侵入"给出严格更大的代价信号。
TEST_F(AlmEsdfPenaltyTest, DeeperIntrusionYieldsStrictlyLargerCost) {
    const auto penalty = MakePenalty();
    const double x0_values[] = {circle_radius_ + 0.08, circle_radius_,
                                circle_radius_ - 0.3};
    double prev_cost = 0.0;
    for (const double x0 : x0_values) {
        const auto result = penalty.evaluate(x0, 4.0, kHalfPi);
        EXPECT_GT(result.cost, prev_cost);
        prev_cost = result.cost;
    }
}

// 测试平行位姿下解析梯度与五点中心差分数值梯度对拍，相对误差 < 1e-6。
// 数值梯度是独立于手推公式的第三方验证：场景取平行深惩罚区（梯度量级
// ~13~18）以稀释 float 场噪声；步长 h=1e-2 为实测噪声/截断权衡最优。
TEST_F(AlmEsdfPenaltyTest, NumericGradientMatchesAnalyticParallel) {
    const auto penalty = MakePenalty();
    const double x0 = 0.35;
    const auto result = penalty.evaluate(x0, 4.0, kHalfPi);

    ExpectComponentClose(result.gradient.x(),
                         NumericGradientComponent(penalty, x0, 4.0, kHalfPi, 0,
                                                  1e-2),
                         1e-6);
    // 墙场不依赖 y：解析与数值梯度都应在零附近（绝对容差）
    EXPECT_NEAR(result.gradient.y(),
                NumericGradientComponent(penalty, x0, 4.0, kHalfPi, 1, 1e-2),
                1e-9);
    ExpectComponentClose(result.gradient.z(),
                         NumericGradientComponent(penalty, x0, 4.0, kHalfPi, 2,
                                                  1e-2),
                         1e-6);
}

// 测试旋转位姿（θ≠0）下解析梯度与数值梯度对拍，相对误差 < 1e-6（含 θ 通道）。
// 这是"若忽略外圆随 θ 旋转的链式法则，梯度会明显偏离数值梯度"的直接验证：
// θ 通道数值梯度 ~9.24，遗漏旋转项时解析值为 0，偏差达 9 个数量级。
TEST_F(AlmEsdfPenaltyTest, NumericGradientMatchesAnalyticRotated) {
    const auto penalty = MakePenalty();
    const double theta = std::atan2(0.8, -0.6);
    const double x0 = 1.95;
    const auto result = penalty.evaluate(x0, 4.0, theta);

    ExpectComponentClose(result.gradient.x(),
                         NumericGradientComponent(penalty, x0, 4.0, theta, 0,
                                                  1e-2),
                         1e-6);
    EXPECT_NEAR(result.gradient.y(),
                NumericGradientComponent(penalty, x0, 4.0, theta, 1, 1e-2),
                1e-9);
    // θ 通道步长 h=2e-3：大力臂（2.78m）下截断项 ~h^4·arm^5 增长快，
    // 实测该步长噪声/截断权衡最优；场景设计保证全部圆心在差分模板
    // 范围内不跨越 max(0,C)^3 的 C=0 折点（否则差分误差由折点主导）
    ExpectComponentClose(result.gradient.z(),
                         NumericGradientComponent(penalty, x0, 4.0, theta, 2,
                                                  2e-3),
                         1e-6);
    // 负对照量级确认：θ 梯度信号本身足够强，上述对拍具备判别力
    EXPECT_GT(std::abs(result.gradient.z()), 1.0);
}

// 测试圆心越出地图时的保守行为。
// 因为 ESDFMap 对越界查询返回 (0, 零梯度) 哨兵，对应圆按最大侵入计
// （cost=W_safe·(r+margin_safe)^3+W_comf·(r+margin_comf)^3），梯度贡献为零。
TEST_F(AlmEsdfPenaltyTest, OutOfMapCircleYieldsConservativePenalty) {
    const auto penalty = MakePenalty();
    // θ=0 位姿 x0=5.5：前圆 cx=8.28>8 越界；其余圆在图内但 d>r+margin_comf
    // 不激活
    const auto result = penalty.evaluate(5.5, 4.0, 0.0);
    const double expected =
        penalty.config().weight_safe *
            std::pow(circle_radius_ + penalty.config().margin_safe, 3) +
        penalty.config().weight_comf *
            std::pow(circle_radius_ + penalty.config().margin_comf, 3);

    EXPECT_NEAR(result.cost, expected, 1e-9);
    EXPECT_TRUE(result.gradient.isZero());
}

}  // namespace
}  // namespace apa_post_processor
