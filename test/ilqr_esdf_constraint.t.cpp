#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <limits>
#include <vector>

#include "core/iLQR/esdf_constraint.h"
#include "core/NMPC/vehicle_circle_geometry.h"
#include "spatial/esdf_map.h"
#include "spatial/grid_map.h"
#include "vehicle/vehicle_footprint_model.h"
#include "vehicle/vehicle_params.h"

namespace apa_post_processor {
namespace {

// 编译期一致性断言（零运行时开销）：钉住头文件声明与冻结接口契约——
// 定长类型维度与配置默认值必须与设计文档一致（生产配置
// data/ilqr_config.json 未覆盖这些字段，即按此默认值运行），声明被意外
// 改动（维度/默认值漂移）时在编译期即失败
static_assert(iLQRStateHessian::RowsAtCompileTime == ILQR_STATE_DIM &&
                  iLQRStateHessian::ColsAtCompileTime == ILQR_STATE_DIM,
              "iLQRStateHessian 必须为 ILQR_STATE_DIM x ILQR_STATE_DIM");

// ESDF 配置默认值一致性：iLQRConfig 平铺后含 std::string 等非字面量成员，
// 编译期 static_assert 不可用，改运行时断言。生产配置 data/ilqr_config.json
// 未覆盖这些字段时即按默认值运行，默认值漂移必须被测试拦截
TEST(iLQRConfigDefaultsTest, EsdfDefaultsMatchProduction) {
    const iLQRConfig config;
    EXPECT_DOUBLE_EQ(config.esdf_margin_safe, 0.02);
    EXPECT_DOUBLE_EQ(config.esdf_margin_comf, 0.20);
    EXPECT_DOUBLE_EQ(config.esdf_weight_safe, 100.0);
    EXPECT_DOUBLE_EQ(config.esdf_weight_comf, 10.0);
    EXPECT_EQ(config.esdf_stride, 1);
}

// 墙场地图参数：第 0 列整列占据；L8.2 契约下最外圈（第 0/63 行与第 63 列）
// 同样被标记占据。占据栅格中心的符号距离场在 x>=2*resolution 且远离其余
// 边界圈的区域严格满足 D=x、g=(1,0)（距离变换在该区域对线性函数精确，
// 有限差分梯度同样精确；第一个自由列因符号距离在墙面处存在折点，其梯度为
// 1.5 不可用，所有测试场景必须避开该折点区单元格）。场景 y 坐标须保证
// 全部圆心到第 0/63 行的距离明显大于到墙面的距离，否则最近特征在墙面与
// 边界圈之间切换（中轴折点），D=x 的解析假设在折点两侧各一格的过渡带内
// 失效。
constexpr double kResolution = 0.125;
constexpr int kMapCols = 64;
constexpr int kMapRows = 64;
// π/2（平行于墙面的位姿航向）
const double kHalfPi = std::acos(-1.0) / 2.0;

// 对拍辅助：相对误差 < rel_tol，近零分量退化为绝对误差 rel_tol*1e-6
void ExpectComponentClose(double expected, double actual, double rel_tol) {
    EXPECT_LE(std::abs(expected - actual),
              rel_tol * std::max(std::abs(actual), 1e-6))
        << "expected=" << expected << " actual=" << actual;
}

// 五点中心差分 [-f(x+2h)+8f(x+h)-8f(x-h)+f(x-2h)]/(12h)，截断误差 O(h^4)
template <typename TFunc>
double FivePointCentral(TFunc&& func, double h) {
    return (-func(2.0 * h) + 8.0 * func(h) - 8.0 * func(-h) + func(-2.0 * h)) /
           (12.0 * h);
}

// 手推参考结果：双 margin 惩罚的代价、对 (x,y,θ) 的梯度与 GN 形 Hessian
struct ReferenceCostGradHess {
    double cost{0.0};
    Eigen::Vector3d gradient{Eigen::Vector3d::Zero()};
    Eigen::Matrix3d hessian{Eigen::Matrix3d::Zero()};
};

// 墙场手推参考：全部圆心落在 x>=2*resolution 时 D=cx、g=(1,0) 精确成立，
// 每圆约束雅可比 ∇C=(-1, 0, -dcx_dθ) 闭式可知；GN 形 Hessian 为
// Σ_active 6·W·C·∇C·∇Cᵀ（丢弃含场曲率的 3·W·C²·∇²C 项）
ReferenceCostGradHess ComputeReference(
    const std::vector<Eigen::Vector2d>& local_centers, double radius, double x,
    double y, double theta, const iLQRConfig& config) {
    const double cos_theta = std::cos(theta);
    const double sin_theta = std::sin(theta);
    ReferenceCostGradHess ref;
    for (const auto& center : local_centers) {
        const double lx = center.x();
        const double ly = center.y();
        const double cx = x + cos_theta * lx - sin_theta * ly;
        const double dcx_dtheta = -sin_theta * lx - cos_theta * ly;
        const Eigen::Vector3d grad_c(-1.0, 0.0, -dcx_dtheta);
        const double c_safe = radius + config.esdf_margin_safe - cx;
        if (c_safe > 0.0) {
            ref.cost += config.esdf_weight_safe * c_safe * c_safe * c_safe;
            ref.gradient +=
                (3.0 * config.esdf_weight_safe * c_safe * c_safe) * grad_c;
            ref.hessian += (6.0 * config.esdf_weight_safe * c_safe) * grad_c *
                           grad_c.transpose();
        }
        const double c_comf = radius + config.esdf_margin_comf - cx;
        if (c_comf > 0.0) {
            ref.cost += config.esdf_weight_comf * c_comf * c_comf * c_comf;
            ref.gradient +=
                (3.0 * config.esdf_weight_comf * c_comf * c_comf) * grad_c;
            ref.hessian += (6.0 * config.esdf_weight_comf * c_comf) * grad_c *
                           grad_c.transpose();
        }
    }
    return ref;
}

// 白盒测试访问器：暴露 protected 的逐圆约束求值与内部圆集，
// 供约束雅可比的有限差分对拍使用
class TestableiLQREsdfConstraint : public iLQREsdfConstraint {
   public:
    using iLQREsdfConstraint::iLQREsdfConstraint;
    using iLQREsdfConstraint::evaluateCircle;
};

// 公共测试夹具：直墙 ESDF 地图 + 真实车辆外圆模型（outer_row_num=1，3 个外圆，
// 局部圆心 lx∈{-0.0833, 1.35, 2.7833}、ly=0，r_outer≈1.1505）
class iLQREsdfConstraintTest : public ::testing::Test {
   protected:
    iLQREsdfConstraintTest()
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

    iLQREsdfConstraint MakeConstraint(
        iLQRConfig config = {}) const {
        return iLQREsdfConstraint(esdf_map_, footprint_model_, config);
    }

    TestableiLQREsdfConstraint MakeTestable(
        iLQRConfig config = {}) const {
        return TestableiLQREsdfConstraint(esdf_map_, footprint_model_, config);
    }

    // 校验 evaluate 结果与手推参考一致（代价/梯度/Hessian 的 (x,y,θ) 块）
    void ExpectMatchesReference(const iLQREsdfPoseCost& actual,
                                const ReferenceCostGradHess& ref,
                                double rel_tol) const {
        ExpectComponentClose(ref.cost, actual.cost, rel_tol);
        ExpectComponentClose(ref.gradient.x(), actual.gradient(ILQR_IDX_X),
                             rel_tol);
        EXPECT_NEAR(ref.gradient.y(), actual.gradient(ILQR_IDX_Y), 1e-9);
        ExpectComponentClose(ref.gradient.z(), actual.gradient(ILQR_IDX_THETA),
                             rel_tol);
        const Eigen::Matrix3d actual_hess =
            actual.hessian.topLeftCorner<3, 3>();
        for (int row = 0; row < 3; ++row) {
            for (int col = 0; col < 3; ++col) {
                ExpectComponentClose(ref.hessian(row, col),
                                     actual_hess(row, col), rel_tol);
            }
        }
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
// 负权重、非有限值、stride=0 均必须抛 std::invalid_argument。
// 因为非法配置会静默污染全部下游优化目标，必须在构造期显式拒绝。
TEST_F(iLQREsdfConstraintTest, ConstructorThrowsOnInvalidConfig) {
    iLQRConfig config;
    config.esdf_margin_safe = -0.01;
    EXPECT_THROW(MakeConstraint(config), std::invalid_argument);
    config.esdf_margin_safe = 0.02;
    config.esdf_margin_comf = 0.02;
    EXPECT_THROW(MakeConstraint(config), std::invalid_argument);
    config.esdf_margin_comf = 0.10;
    config.esdf_weight_safe = -1.0;
    EXPECT_THROW(MakeConstraint(config), std::invalid_argument);
    config.esdf_weight_safe = 1.0;
    config.esdf_weight_comf = std::numeric_limits<double>::quiet_NaN();
    EXPECT_THROW(MakeConstraint(config), std::invalid_argument);
    config.esdf_weight_comf = 1.0;
    config.esdf_stride = 0;
    EXPECT_THROW(MakeConstraint(config), std::invalid_argument);
}

// 测试舒适缓冲边界之外代价、梯度与 Hessian 严格为零。
// 因为双 margin 惩罚是两段外点罚，d >= r+margin_comf 时两段都不应激活。
// y=2.5（而非地图中部 4.0）：L8.2 后 θ=π/2 时前圆 cy=y+2.78 会逼近第 63 行
// 边界圈（y=7.875），y=4.0 时其 d≈1.09<r+margin_comf 被边界圈激活；
// y=2.5 时全部圆心 cy∈[2.42,5.28]，墙面仍是唯一最近特征，场精确 D=cx。
TEST_F(iLQREsdfConstraintTest, ReturnsZeroOutsideComfortMargin) {
    const auto constraint = MakeConstraint();
    // 平行位姿（θ=π/2）下全部圆心 cx=x0>r+margin_comf
    const double x0 = circle_radius_ + 0.5;
    const auto result = constraint.evaluate(x0, 2.5, kHalfPi);

    EXPECT_DOUBLE_EQ(result.cost, 0.0);
    EXPECT_TRUE(result.gradient.isZero());
    EXPECT_TRUE(result.hessian.isZero());
}

// 测试两段 margin 衔接处的 C¹ 连续性。
// 因为 Φ(C)=max(0,C)³ 在 C=0 处值与一阶导数都为 0，d=r+margin_comf 与
// d=r+margin_safe 两个衔接处两侧的代价与梯度必须连续（GN Hessian 含
// max(0,C) 因子，同样随 C→0 连续归零）。
// y=2.5：全部圆心远离第 0/63 行边界圈，场精确 D=cx（见文件头部地图注释）。
TEST_F(iLQREsdfConstraintTest, MarginBoundariesAreC1Continuous) {
    const auto constraint = MakeConstraint();
    const auto& config = constraint.config();
    // 舒适边界：界上严格为零，界内 0.1mm 处梯度量级 ~3·W_comf·(1e-4)²·N_c，
    // 与 W_comf 成正比，故阈值随权重线性缩放（W_comf=10 时实测 ~1.5e-6）
    const double x_comf = circle_radius_ + config.esdf_margin_comf;
    const auto on_comf = constraint.evaluate(x_comf, 2.5, kHalfPi);
    EXPECT_DOUBLE_EQ(on_comf.cost, 0.0);
    EXPECT_TRUE(on_comf.gradient.isZero());
    EXPECT_TRUE(on_comf.hessian.isZero());
    const auto inside_comf = constraint.evaluate(x_comf - 1e-4, 2.5, kHalfPi);
    EXPECT_GT(inside_comf.cost, 0.0);
    EXPECT_LT(inside_comf.gradient.norm(), 1e-6 * config.esdf_weight_comf);
    // 安全边界：两侧代价差为 O(Δx)（梯度有限），两侧梯度差同样为 O(Δx)
    // （C¹：梯度在衔接处连续、仅随 C 线性变化），不会出现 O(1) 跳变；
    // 衔接处舒适段 C_comf=margin_comf-margin_safe 恒正，Hessian ∝ W_comf·C_comf，
    // 故两个阈值同样随权重线性缩放（×2 余量覆盖有限差分系数）
    const double x_safe = circle_radius_ + config.esdf_margin_safe;
    const auto above_safe = constraint.evaluate(x_safe + 1e-4, 2.5, kHalfPi);
    const auto below_safe = constraint.evaluate(x_safe - 1e-4, 2.5, kHalfPi);
    EXPECT_NEAR(above_safe.cost, below_safe.cost, 1e-4 * config.esdf_weight_comf);
    EXPECT_LT((above_safe.gradient - below_safe.gradient).norm(),
              2.0 * 1e-3 * config.esdf_weight_comf);
}

// 测试仅舒适段激活时（r+margin_safe < d < r+margin_comf）代价/梯度/Hessian
// 与手推参考一致。因为双 margin 机制的关键分段特性是"safe 段不激活、
// comf 段单独起作用"，必须独立验证。
TEST_F(iLQREsdfConstraintTest, HandDerivedComfortOnlyZoneMatches) {
    const auto constraint = MakeConstraint();
    // 平行位姿全部圆心 cx=x0=r+0.06：C_safe=-0.04<0（safe 段不激活）、
    // C_comf=0.04>0（comf 段激活）；y=2.5 保证墙面主导（见文件头部注释）
    const double x0 = circle_radius_ + 0.06;
    const auto result = constraint.evaluate(x0, 2.5, kHalfPi);
    const auto ref = ComputeReference(circle_centers_, circle_radius_, x0, 2.5,
                                      kHalfPi, constraint.config());

    ExpectMatchesReference(result, ref, 1e-6);
    EXPECT_GT(result.cost, 0.0);
}

// 测试两段同时激活时（d < r+margin_safe）代价/梯度/Hessian 与手推参考一致。
// 因为 safe+comf 混合惩罚是本模块的核心输出，强信号区容差取 1e-6。
TEST_F(iLQREsdfConstraintTest, HandDerivedMixedZoneMatches) {
    const auto constraint = MakeConstraint();
    // 平行位姿全部圆心 cx=x0=0.35：C_safe≈0.82、C_comf≈0.90，两段均激活
    const double x0 = 0.35;
    const auto result = constraint.evaluate(x0, 4.0, kHalfPi);
    const auto ref = ComputeReference(circle_centers_, circle_radius_, x0, 4.0,
                                      kHalfPi, constraint.config());

    ExpectMatchesReference(result, ref, 1e-6);
}

// 测试旋转位姿（θ≠0）下代价/梯度/Hessian 与手推参考一致。
// 因为外圆圆心随 θ 旋转，θ 通道导数必须包含几何链式法则 dP/dθ=dR/dθ·p_k；
// Hessian 的 (x,θ)/(θ,θ) 块完全由该旋转链式产生，遗漏将与真值差数量级。
TEST_F(iLQREsdfConstraintTest, HandDerivedRotatedPoseMatches) {
    const auto constraint = MakeConstraint();
    // θ=atan2(0.8,-0.6)（cos=-0.6、sin=0.8，车辆前部朝向墙面）：前圆
    // （lx=2.7833）cx≈0.28 埋入惩罚区，提供强 θ 链式信号
    const double theta = std::atan2(0.8, -0.6);
    const double x0 = 1.95;
    const auto result = constraint.evaluate(x0, 4.0, theta);
    const auto ref = ComputeReference(circle_centers_, circle_radius_, x0, 4.0,
                                      theta, constraint.config());

    ExpectMatchesReference(result, ref, 1e-6);
    EXPECT_GT(std::abs(result.hessian(ILQR_IDX_THETA, ILQR_IDX_THETA)), 1.0);
}

// 测试平行位姿下解析梯度与五点中心差分数值梯度对拍，相对误差 < 1e-6。
// 数值梯度是独立于手推公式的第三方验证；场景取深惩罚区且差分模板
// 不跨越 max(0,C)³ 的 C=0 折点（折点主导差分误差，是已标定的坑）。
TEST_F(iLQREsdfConstraintTest, NumericGradientMatchesAnalyticParallel) {
    const auto constraint = MakeConstraint();
    const double x0 = 0.35;
    const auto result = constraint.evaluate(x0, 4.0, kHalfPi);
    auto cost_of = [&](int component) {
        // component 按值捕获：工厂返回的 lambda 在参数析构后才被调用
        return [&, component](double offset) {
            double px = x0;
            double py = 4.0;
            double pt = kHalfPi;
            if (component == 0) {
                px += offset;
            } else if (component == 1) {
                py += offset;
            } else {
                pt += offset;
            }
            return constraint.evaluate(px, py, pt).cost;
        };
    };

    ExpectComponentClose(result.gradient(ILQR_IDX_X),
                         FivePointCentral(cost_of(0), 1e-3), 1e-6);
    EXPECT_NEAR(result.gradient(ILQR_IDX_Y), FivePointCentral(cost_of(1), 1e-3),
                1e-9);
    ExpectComponentClose(result.gradient(ILQR_IDX_THETA),
                         FivePointCentral(cost_of(2), 1e-3), 1e-6);
    // 非 (x,y,θ) 状态分量必须严格为零：惩罚只依赖位姿
    EXPECT_DOUBLE_EQ(result.gradient(ILQR_IDX_V), 0.0);
    EXPECT_DOUBLE_EQ(result.gradient(ILQR_IDX_A), 0.0);
    EXPECT_DOUBLE_EQ(result.gradient(ILQR_IDX_DELTA), 0.0);
    EXPECT_DOUBLE_EQ(result.gradient(ILQR_IDX_OMEGA), 0.0);
}

// 测试旋转位姿下解析梯度与数值梯度对拍，相对误差 < 1e-6（含 θ 通道）。
// 这是"若忽略外圆随 θ 旋转的链式法则，梯度会明显偏离数值梯度"的直接验证。
TEST_F(iLQREsdfConstraintTest, NumericGradientMatchesAnalyticRotated) {
    const auto constraint = MakeConstraint();
    const double theta = std::atan2(0.8, -0.6);
    const double x0 = 1.95;
    const auto result = constraint.evaluate(x0, 4.0, theta);
    auto cost_of = [&](int component) {
        // component 按值捕获：工厂返回的 lambda 在参数析构后才被调用
        return [&, component](double offset) {
            double px = x0;
            double py = 4.0;
            double pt = theta;
            if (component == 0) {
                px += offset;
            } else if (component == 1) {
                py += offset;
            } else {
                pt += offset;
            }
            return constraint.evaluate(px, py, pt).cost;
        };
    };

    ExpectComponentClose(result.gradient(ILQR_IDX_X),
                         FivePointCentral(cost_of(0), 1e-3), 1e-6);
    EXPECT_NEAR(result.gradient(ILQR_IDX_Y), FivePointCentral(cost_of(1), 1e-3),
                1e-9);
    // θ 通道步长取小：大力臂（2.78m）下高阶截断增长快；
    // 场景保证差分模板范围内不跨越 C=0 折点
    ExpectComponentClose(result.gradient(ILQR_IDX_THETA),
                         FivePointCentral(cost_of(2), 2e-4), 1e-6);
}

// 测试 GN 形 Hessian 与"约束雅可比有限差分"装配结果对拍，相对误差 < 1e-6。
// GN Hessian = Σ_active 6·W·C·∇C·∇Cᵀ 不是代价的精确二阶导（丢弃了
// 3·W·C²·∇²C 的场曲率项），不能用"梯度的差分"直接对拍；正确做法是
// 对约束值 C 做数值差分得到 ∇C，再按 GN 公式装配比对——这既验证梯度
// 链式法则（解析 ∇C vs 数值 ∇C），也验证 Hessian 装配（系数 6·W·C
// 与外积结构）。同时验证两段 margin 共享同一约束雅可比。
TEST_F(iLQREsdfConstraintTest, GnHessianMatchesConstraintJacobianFd) {
    const auto constraint = MakeTestable();
    const auto& config = constraint.config();
    // 平行深惩罚区位姿：三段圆全部落在墙场精确区（cx>=2·resolution）
    const double x0 = 0.35;
    const double y0 = 4.0;
    const double theta0 = kHalfPi;
    Eigen::Matrix3d expected_hess = Eigen::Matrix3d::Zero();
    double expected_cost = 0.0;
    for (const auto& center : circle_centers_) {
        auto constraint_of = [&](int component) {
            // component 按值捕获：工厂返回的 lambda 在参数析构后才被调用
            return [&, component](double offset) {
                double px = x0;
                double py = y0;
                double pt = theta0;
                if (component == 0) {
                    px += offset;
                } else if (component == 1) {
                    py += offset;
                } else {
                    pt += offset;
                }
                const double cos_t = std::cos(pt);
                const double sin_t = std::sin(pt);
                return constraint.evaluateCircle(center, cos_t, sin_t, px, py)
                    .c_safe;
            };
        };
        Eigen::Vector3d grad_numeric;
        for (int component = 0; component < 3; ++component) {
            grad_numeric(component) =
                FivePointCentral(constraint_of(component), 1e-4);
        }
        const double cos_t = std::cos(theta0);
        const double sin_t = std::sin(theta0);
        const auto circle =
            constraint.evaluateCircle(center, cos_t, sin_t, x0, y0);
        // 解析约束雅可比 vs 数值约束雅可比（discretize-then-differentiate
        // 在墙场精确区内两者一致）；y 分量解析值恰为 0，FD 残余噪声 ~1e-10，
        // 近零分量按绝对容差对拍、非零分量按相对 1e-6 对拍
        for (int component = 0; component < 3; ++component) {
            EXPECT_NEAR(grad_numeric(component), circle.grad(component),
                        1e-8 + 1e-6 * std::abs(circle.grad(component)));
        }
        // 两段 margin 共享同一约束雅可比（仅边界值不同）
        EXPECT_GT(circle.c_safe, 0.0);
        EXPECT_GT(circle.c_comf, 0.0);
        expected_cost += config.esdf_weight_safe * std::pow(circle.c_safe, 3) +
                         config.esdf_weight_comf * std::pow(circle.c_comf, 3);
        expected_hess += (6.0 * config.esdf_weight_safe * circle.c_safe +
                          6.0 * config.esdf_weight_comf * circle.c_comf) *
                         grad_numeric * grad_numeric.transpose();
    }
    const auto result = constraint.evaluate(x0, y0, theta0);
    ExpectComponentClose(expected_cost, result.cost, 1e-6);
    const Eigen::Matrix3d actual_hess = result.hessian.topLeftCorner<3, 3>();
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            // y 行/列元素解析值恰为 0（FD 残余 ~1e-10），按绝对容差对拍
            EXPECT_NEAR(expected_hess(row, col), actual_hess(row, col),
                        1e-8 + 1e-6 * std::abs(actual_hess(row, col)));
        }
    }
    // Hessian 的非 (x,y,θ) 行/列必须严格为零
    for (int idx = ILQR_IDX_V; idx <= ILQR_IDX_OMEGA; ++idx) {
        EXPECT_TRUE(result.hessian.row(idx).isZero());
        EXPECT_TRUE(result.hessian.col(idx).isZero());
    }
}

// 测试圆心越出地图时的保守行为。
// L8.1 契约下越界圆得到恢复场 d = d_map(p) − s（负值，随穿透深度线性下降）
// 与恒指向图内的非零梯度：惩罚为大的正值，梯度与 GN Hessian 均非零
// （∇C=+x 向，沿负梯度下降把圆拉回图内），「保守惩罚」语义保留且严格强于
// L8 前的 (0, 零梯度) 哨兵（那时 ∇C=0，Hessian 同样为零）。
TEST_F(iLQREsdfConstraintTest, OutOfMapCircleYieldsConservativePenalty) {
    const auto constraint = MakeConstraint();
    const auto& config = constraint.config();
    // θ=0 位姿 x0=5.25：前圆 cx=5.25+2.7833≈8.033 越出东边界（s≈0.033）；
    // 中圆 cx=6.60（到东边界圈 d=1.275>r+margin_comf≈1.2505）与后圆
    // cx≈5.167 均不激活——x0 的可行窗口为 (8-2.7833, 7.875-1.35-1.2505)≈
    // (5.217, 5.274)，取中点附近
    const double x0 = 5.25;
    const double y0 = 4.0;
    const double map_east = kMapCols * kResolution;  // 8.0
    ReferenceCostGradHess ref;
    for (const auto& center : circle_centers_) {
        const double cx = x0 + center.x();  // θ=0、ly=0：cy=y0、dcx/dθ=0
        double d;
        if (cx >= map_east) {
            // L8.1 恢复场：p=(map_east,y0) 索引钳制到东边界格（采样值
            // d=-res），d = -res-s，∇d=(-1,0) 恒指向图内
            d = -kResolution - (cx - map_east);
        } else {
            // 图内东侧线性区：d = 东边界格中心 x − cx = 7.875−cx，∇d=(-1,0)
            d = (map_east - kResolution) - cx;
        }
        // ∇C = -(∇d·∂c/∂pose)：∂d/∂x=-1、dcx/dθ=0 ⟹ ∇C=(1,0,0)
        const Eigen::Vector3d grad_c(1.0, 0.0, 0.0);
        const double c_safe = circle_radius_ + config.esdf_margin_safe - d;
        if (c_safe > 0.0) {
            ref.cost += config.esdf_weight_safe * std::pow(c_safe, 3);
            ref.gradient +=
                (3.0 * config.esdf_weight_safe * c_safe * c_safe) * grad_c;
            ref.hessian += (6.0 * config.esdf_weight_safe * c_safe) * grad_c *
                           grad_c.transpose();
        }
        const double c_comf = circle_radius_ + config.esdf_margin_comf - d;
        if (c_comf > 0.0) {
            ref.cost += config.esdf_weight_comf * std::pow(c_comf, 3);
            ref.gradient +=
                (3.0 * config.esdf_weight_comf * c_comf * c_comf) * grad_c;
            ref.hessian += (6.0 * config.esdf_weight_comf * c_comf) * grad_c *
                           grad_c.transpose();
        }
    }
    const auto result = constraint.evaluate(x0, y0, 0.0);

    // 场景设计自检：前圆确实越界（否则注释中的恢复场推导不成立）
    EXPECT_GT(x0 + circle_centers_.back().x(), map_east);
    ExpectMatchesReference(result, ref, 1e-9);
    // 恢复场梯度指向图内 ⟹ 代价梯度与 Hessian 的 x 分量非零
    EXPECT_GT(result.gradient(ILQR_IDX_X), 0.0);
    EXPECT_GT(result.hessian(ILQR_IDX_X, ILQR_IDX_X), 0.0);
    // 保守性：惩罚严格大于 L8 前 d=0 哨兵对应的全侵入罚
    const double old_sentinel =
        config.esdf_weight_safe * std::pow(circle_radius_ + config.esdf_margin_safe, 3) +
        config.esdf_weight_comf * std::pow(circle_radius_ + config.esdf_margin_comf, 3);
    EXPECT_GT(result.cost, old_sentinel);
}

// 测试 evaluate 与「逐圆全量求值」参照路径在逐位意义下完全一致。
// 因为 evaluate 内部允许对两个 margin 均不活跃的圆跳过梯度插值
// （不活跃圆的代价/梯度/Hessian 贡献恒为零），本测试用未做该跳过的
// evaluateCircle 逐圆累加作为参照，钉住「跳过不改变任何数值」契约；
// 位姿覆盖全不活跃、comf 段、混合区、旋转、部分圆活跃与图外恢复场
// （图外按实心障碍处理，c 恒正，梯度必取——两阶段跳过只发生在图内）。
TEST_F(iLQREsdfConstraintTest, EvaluateMatchesPerCircleFullEvaluationBitExact) {
    const auto constraint = MakeTestable();
    const auto& config = constraint.config();
    // {x, y, theta}；地图 64x64、分辨率 0.125，墙在第 0 列
    const std::vector<std::array<double, 3>> poses = {
        {6.0, 2.5, kHalfPi},
        {circle_radius_ + 0.06, 2.5, kHalfPi},
        {0.35, 4.0, kHalfPi},
        {1.95, 4.0, std::atan2(0.8, -0.6)},
        {1.2, 4.0, 0.0},
        {-0.3, 4.0, 0.0},
    };
    for (const auto& pose : poses) {
        const double px = pose[0];
        const double py = pose[1];
        const double pt = pose[2];
        const double cos_theta = std::cos(pt);
        const double sin_theta = std::sin(pt);
        // 参照累加：逐圆全量求值（evaluateCircle 恒取距离+梯度），
        // 累加表达式与生产 evaluate 逐条对应
        iLQREsdfPoseCost expected;
        Eigen::Vector3d grad3 = Eigen::Vector3d::Zero();
        Eigen::Matrix3d hess3 = Eigen::Matrix3d::Zero();
        for (const auto& center : circle_centers_) {
            const auto circle = constraint.evaluateCircle(
                center, cos_theta, sin_theta, px, py);
            if (circle.c_safe > 0.0) {
                expected.cost += config.esdf_weight_safe * circle.c_safe *
                                 circle.c_safe * circle.c_safe;
                grad3 += (3.0 * config.esdf_weight_safe * circle.c_safe *
                          circle.c_safe) *
                         circle.grad;
                hess3 += (6.0 * config.esdf_weight_safe * circle.c_safe) *
                         (circle.grad * circle.grad.transpose());
            }
            if (circle.c_comf > 0.0) {
                expected.cost += config.esdf_weight_comf * circle.c_comf *
                                 circle.c_comf * circle.c_comf;
                grad3 += (3.0 * config.esdf_weight_comf * circle.c_comf *
                          circle.c_comf) *
                         circle.grad;
                hess3 += (6.0 * config.esdf_weight_comf * circle.c_comf) *
                         (circle.grad * circle.grad.transpose());
            }
        }
        expected.gradient(ILQR_IDX_X) = grad3.x();
        expected.gradient(ILQR_IDX_Y) = grad3.y();
        expected.gradient(ILQR_IDX_THETA) = grad3.z();
        expected.hessian.topLeftCorner<3, 3>() = hess3;
        const auto actual = constraint.evaluate(px, py, pt);
        // 逐位等价：双 margin 三次罚的累加顺序与每个分量都必须零误差
        EXPECT_DOUBLE_EQ(expected.cost, actual.cost) << "pose x=" << px;
        for (int i = 0; i < ILQR_STATE_DIM; ++i) {
            EXPECT_DOUBLE_EQ(expected.gradient(i), actual.gradient(i))
                << "pose x=" << px << " grad i=" << i;
        }
        for (int row = 0; row < ILQR_STATE_DIM; ++row) {
            for (int col = 0; col < ILQR_STATE_DIM; ++col) {
                EXPECT_DOUBLE_EQ(expected.hessian(row, col),
                                 actual.hessian(row, col))
                    << "pose x=" << px << " hess (" << row << "," << col
                    << ")";
            }
        }
    }
}

// 测试时间轴 stride 抽样语义。
// 因为 ESDF 查询是运行时主导开销，stride=2 时只在偶数阶段做避障检查，
// 查询量减半；stride=1 时逐阶段检查。
TEST_F(iLQREsdfConstraintTest, StrideSamplingSkipsIntermediateStages) {
    iLQRConfig config;
    config.esdf_stride = 2;
    const auto constraint = MakeConstraint(config);

    EXPECT_TRUE(constraint.isSampled(0));
    EXPECT_FALSE(constraint.isSampled(1));
    EXPECT_TRUE(constraint.isSampled(2));
    EXPECT_FALSE(constraint.isSampled(3));
    EXPECT_TRUE(constraint.isSampled(400));
    const auto per_step = MakeConstraint();
    EXPECT_TRUE(per_step.isSampled(0));
    EXPECT_TRUE(per_step.isSampled(1));
    EXPECT_TRUE(per_step.isSampled(399));
}

}  // namespace
}  // namespace apa_post_processor
