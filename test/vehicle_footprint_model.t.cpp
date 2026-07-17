#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>
#include <utility>
#include <vector>

#include "util/constants.h"
#include "vehicle/vehicle_footprint_model.h"
#include "vehicle/vehicle_params.h"

namespace apa_post_processor {
namespace {

// 公共测试夹具：复用同一组车辆参数和 VehicleFootprintModel 实例，
// 避免每个测试重复构造模型与查找表。
class VehicleFootprintModelTest : public ::testing::Test {
   protected:
    static constexpr double kLength = 4.3;
    static constexpr double kWidth = 1.8;
    static constexpr double kWheelbase = 2.7;
    static constexpr double kMaxSteerAngle = 0.6;
    static constexpr double kRearOverhang = 0.8;
    static constexpr int kHeadingSampleNum = 233;
    static constexpr int kInnerRowNum = 2;
    static constexpr int kOuterRowNum = 4;
    VehicleFootprintModelTest()
        : veh_params_(kLength, kWidth, kWheelbase, kMaxSteerAngle,
                      kRearOverhang),
          model_(veh_params_, kHeadingSampleNum, kInnerRowNum, kOuterRowNum) {}

    VehicleParams veh_params_;
    VehicleFootprintModel model_;
};

// 辅助工厂函数：按给定参数构造 VehicleFootprintModel，用于异常测试等需要
// 临时实例的场景。
VehicleFootprintModel MakeModel(const VehicleParams& params,
                                int heading_sample_num = 233,
                                int inner_row_num = 2, int outer_row_num = 4) {
    return VehicleFootprintModel{params, heading_sample_num, inner_row_num,
                                 outer_row_num};
}

// 辅助计算函数：根据车身长度、宽度与外圆行数，计算外圆网格列数以及
// 仅保留边界圆后的圆数量。
std::pair<int, int> ComputeOuterBoundaryCount(double length, double width,
                                              int outer_row_num) {
    const auto delta_y = width / static_cast<double>(outer_row_num);
    const auto col_num = static_cast<int>(std::ceil(length / delta_y));
    const auto boundary_count = 2 * (outer_row_num + col_num - 2);
    return {col_num, boundary_count};
}

// 可测试子类：通过继承暴露受保护的查找表，便于验证 theta=0 时插值结果
// 与表中基准圆心是否完全一致。
class TestableVehicleFootprintModel : public VehicleFootprintModel {
   public:
    using VehicleFootprintModel::VehicleFootprintModel;
    const std::vector<Eigen::Vector2d>& GetInnerBaseCenters() const {
        return inner_circle_table_.front();
    }
    const std::vector<Eigen::Vector2d>& GetOuterBaseCenters() const {
        return outer_circle_table_.front();
    }
};

// 测试目标：验证构造函数对非法车辆几何参数的防御能力。
// 测试流程：分别构造 length<=0、width<=0、rear_overhang<0 的 VehicleParams，
//          再通过 MakeModel 尝试创建模型。
// 预期结果：三种情况均抛出 std::invalid_argument。
TEST_F(VehicleFootprintModelTest, ThrowsOnInvalidVehicleParams) {
    const auto invalid_length =
        VehicleParams{0.0, kWidth, kWheelbase, kMaxSteerAngle, kRearOverhang};
    const auto invalid_width =
        VehicleParams{kLength, -1.0, kWheelbase, kMaxSteerAngle, kRearOverhang};
    const auto invalid_rear_overhang =
        VehicleParams{kLength, kWidth, kWheelbase, kMaxSteerAngle, -0.1};
    EXPECT_THROW(MakeModel(invalid_length), std::invalid_argument);
    EXPECT_THROW(MakeModel(invalid_width), std::invalid_argument);
    EXPECT_THROW(MakeModel(invalid_rear_overhang), std::invalid_argument);
}

// 测试目标：验证构造函数对非法离散化参数的防御能力。
// 测试流程：依次传入 heading_sample_num<2、inner_row_num<1、outer_row_num<1
//          的组合构造模型。
// 预期结果：每种非法组合均抛出 std::invalid_argument。
TEST_F(VehicleFootprintModelTest, ThrowsOnInvalidDiscretizationParams) {
    EXPECT_THROW(MakeModel(veh_params_, 1, kInnerRowNum, kOuterRowNum),
                 std::invalid_argument);
    EXPECT_THROW(MakeModel(veh_params_, kHeadingSampleNum, 0, kOuterRowNum),
                 std::invalid_argument);
    EXPECT_THROW(MakeModel(veh_params_, kHeadingSampleNum, kInnerRowNum, -1),
                 std::invalid_argument);
}

// 测试目标：验证 calInterpolatedCenters 在输出容器未预分配时抛出异常。
// 测试流程：构造空的 centers 与 jacobians 容器，调用插值接口。
// 预期结果：抛出 std::invalid_argument。
TEST_F(VehicleFootprintModelTest, ThrowsOnEmptyOutputBuffers) {
    std::vector<Eigen::Vector2d> empty_centers;
    std::vector<Eigen::Matrix<double, 2, 3>> empty_jacobians;
    EXPECT_THROW(model_.calInterpolatedCenters(0.0, 0.0, 0.0, CircleType::INNER,
                                               empty_centers, empty_jacobians),
                 std::invalid_argument);
}

// 测试目标：验证 calInterpolatedCenters 对大小不匹配输出容器的防御。
// 测试流程：先让 jacobians 比 centers 多一个元素，再反过来让 centers
//          比 jacobians 多一个元素，分别调用插值接口。
// 预期结果：两种情况下均抛出 std::invalid_argument。
TEST_F(VehicleFootprintModelTest, ThrowsOnMismatchedOutputBuffers) {
    const auto circle_num = model_.getCircleNum(CircleType::INNER);
    std::vector<Eigen::Vector2d> centers(circle_num);
    std::vector<Eigen::Matrix<double, 2, 3>> jacobians(circle_num + 1);
    EXPECT_THROW(model_.calInterpolatedCenters(0.0, 0.0, 0.0, CircleType::INNER,
                                               centers, jacobians),
                 std::invalid_argument);
    centers.resize(circle_num + 1);
    jacobians.resize(circle_num);
    EXPECT_THROW(model_.calInterpolatedCenters(0.0, 0.0, 0.0, CircleType::INNER,
                                               centers, jacobians),
                 std::invalid_argument);
}

// 测试目标：验证内圈只有一行时半径退化为 0.5*width 且纵向间距为 0。
// 测试流程：构造 inner_row_num=1 的模型，查询内圆半径；再计算 theta=0 时
//          所有内圆圆心，比较相邻圆心的 y 坐标。
// 预期结果：半径等于车宽的一半，所有内圆 y 坐标相同。
TEST_F(VehicleFootprintModelTest, InnerCircleRadiusForSingleRow) {
    const auto params =
        VehicleParams{kLength, kWidth, kWheelbase, kMaxSteerAngle, 0.0};
    const VehicleFootprintModel single_row_model{params, kHeadingSampleNum, 1,
                                                 kOuterRowNum};
    EXPECT_NEAR(single_row_model.getInnerRadius(), 0.5 * kWidth, 1e-12);
    const auto circle_num = single_row_model.getCircleNum(CircleType::INNER);
    std::vector<Eigen::Vector2d> centers(circle_num);
    std::vector<Eigen::Matrix<double, 2, 3>> jacobians(circle_num);
    single_row_model.calInterpolatedCenters(0.0, 0.0, 0.0, CircleType::INNER,
                                            centers, jacobians);
    for (int i = 1; i < circle_num; ++i) {
        EXPECT_NEAR(centers[i].y(), centers[i - 1].y(), 1e-12);
    }
}

// 测试目标：验证极宽/极短车身触发 col_num=1 时无除零风险并输出正确圆心。
// 测试流程：构造 length=0.5、width=2.0、inner_row_num=3 的模型，使内圆列数
//          退化为 1；调用 calInterpolatedCenters 并与解析公式对比。
// 预期结果：模型正常构造，输出 3 个圆心，位置与公式 base_x=R、
//          base_y=-width/2+R+j*delta_y 一致。
TEST_F(VehicleFootprintModelTest,
       HandlesSingleColumnBoundaryWithoutDivisionByZero) {
    const auto params = VehicleParams{0.5, 2.0, 0.3, kMaxSteerAngle, 0.0};
    const VehicleFootprintModel single_col_model{params, kHeadingSampleNum, 3,
                                                 kOuterRowNum};
    EXPECT_EQ(single_col_model.getCircleNum(CircleType::INNER), 3);
    std::vector<Eigen::Vector2d> centers(3);
    std::vector<Eigen::Matrix<double, 2, 3>> jacobians(3);
    EXPECT_NO_THROW(single_col_model.calInterpolatedCenters(
        0.0, 0.0, 0.0, CircleType::INNER, centers, jacobians));
    const auto R = single_col_model.getInnerRadius();
    const auto delta_y = (2.0 - 2.0 * R) / 2.0;
    const auto base_x = R;
    const auto base_y = -1.0 + R;
    EXPECT_NEAR(centers[0].x(), base_x, 1e-12);
    EXPECT_NEAR(centers[0].y(), base_y, 1e-12);
    EXPECT_NEAR(centers[1].x(), base_x, 1e-12);
    EXPECT_NEAR(centers[1].y(), base_y + delta_y, 1e-12);
    EXPECT_NEAR(centers[2].x(), base_x, 1e-12);
    EXPECT_NEAR(centers[2].y(), base_y + 2.0 * delta_y, 1e-12);
}

// 测试目标：验证外圆生成时是否正确剔除车身内部圆，仅保留边界圆。
// 测试流程：使用默认车身参数计算外圆边界圆数量，并与模型实际输出的
//          outer_circles 数量对比。
// 预期结果：model_.getCircleNum(OUTER) 等于 2*(row_num+col_num-2)。
TEST_F(VehicleFootprintModelTest, OuterCircleBoundaryFiltering) {
    const auto [expected_col_num, expected_boundary_count] =
        ComputeOuterBoundaryCount(kLength, kWidth, kOuterRowNum);
    (void)expected_col_num;
    EXPECT_EQ(model_.getCircleNum(CircleType::OUTER), expected_boundary_count);
}

// 测试目标：验证 theta=0 时插值结果与查找表第 0 项完全一致。
// 测试流程：通过 TestableVehicleFootprintModel 获取 inner_circle_table_[0]，
//          调用 calInterpolatedCenters(x,y,0)，对比输出圆心。
// 预期结果：每个输出圆心等于基准圆心加上 (x,y) 平移量。
TEST_F(VehicleFootprintModelTest, ZeroThetaMatchesLookupTable) {
    const TestableVehicleFootprintModel testable_model{
        veh_params_, kHeadingSampleNum, kInnerRowNum, kOuterRowNum};
    const auto& base_centers = testable_model.GetInnerBaseCenters();
    const auto circle_num = testable_model.getCircleNum(CircleType::INNER);
    std::vector<Eigen::Vector2d> centers(circle_num);
    std::vector<Eigen::Matrix<double, 2, 3>> jacobians(circle_num);
    const double x = 1.2, y = 3.4;

    testable_model.calInterpolatedCenters(x, y, 0.0, CircleType::INNER, centers,
                                          jacobians);

    for (int i = 0; i < circle_num; ++i) {
        EXPECT_NEAR(centers[i].x(), x + base_centers[i].x(), 1e-12);
        EXPECT_NEAR(centers[i].y(), y + base_centers[i].y(), 1e-12);
    }
}

// 测试目标：验证 normalizeTheta 对负角和超大角能正确归一化到 [0,2PI) 且不越界。
// 测试流程：分别用 -PI/2、3PI/2、3PI 调用 calInterpolatedCenters，比较
//          -PI/2 与 3PI/2 的输出，并确认 3PI 不崩溃。
// 预期结果：-PI/2 与 3PI/2 的圆心和雅可比完全一致；3PI 调用成功。
TEST_F(VehicleFootprintModelTest, ThetaNormalizationAndWrapAround) {
    const auto circle_num = model_.getCircleNum(CircleType::INNER);
    std::vector<Eigen::Vector2d> neg_centers(circle_num);
    std::vector<Eigen::Vector2d> wrap_centers(circle_num);
    std::vector<Eigen::Matrix<double, 2, 3>> neg_jacobians(circle_num);
    std::vector<Eigen::Matrix<double, 2, 3>> wrap_jacobians(circle_num);

    model_.calInterpolatedCenters(0.0, 0.0, -PI / 2.0, CircleType::INNER,
                                  neg_centers, neg_jacobians);
    model_.calInterpolatedCenters(0.0, 0.0, 3.0 * PI / 2.0, CircleType::INNER,
                                  wrap_centers, wrap_jacobians);

    for (int i = 0; i < circle_num; ++i) {
        EXPECT_NEAR(neg_centers[i].x(), wrap_centers[i].x(), 1e-12);
        EXPECT_NEAR(neg_centers[i].y(), wrap_centers[i].y(), 1e-12);
        EXPECT_NEAR(neg_jacobians[i](0, 2), wrap_jacobians[i](0, 2), 1e-12);
        EXPECT_NEAR(neg_jacobians[i](1, 2), wrap_jacobians[i](1, 2), 1e-12);
    }
    std::vector<Eigen::Vector2d> large_centers(circle_num);
    std::vector<Eigen::Matrix<double, 2, 3>> large_jacobians(circle_num);
    EXPECT_NO_THROW(model_.calInterpolatedCenters(
        0.0, 0.0, 3.0 * PI, CircleType::INNER, large_centers, large_jacobians));
}

// 测试目标：验证内圆雅可比矩阵的形状与数值是否符合设计。
// 测试流程：在 theta=0、x=y=0 处调用 calInterpolatedCenters，遍历每个圆
//          的雅可比矩阵进行断言。
// 预期结果：每个雅可比为 2x3；前两列构成单位阵；第三列为 (-y, x)。
TEST_F(VehicleFootprintModelTest, JacobianShapeAndValues) {
    const auto circle_num = model_.getCircleNum(CircleType::INNER);
    std::vector<Eigen::Vector2d> centers(circle_num);
    std::vector<Eigen::Matrix<double, 2, 3>> jacobians(circle_num);

    model_.calInterpolatedCenters(0.0, 0.0, 0.0, CircleType::INNER, centers,
                                  jacobians);

    for (int i = 0; i < circle_num; ++i) {
        EXPECT_EQ(jacobians[i].rows(), 2);
        EXPECT_EQ(jacobians[i].cols(), 3);
        EXPECT_DOUBLE_EQ(jacobians[i](0, 0), 1.0);
        EXPECT_DOUBLE_EQ(jacobians[i](1, 1), 1.0);
        EXPECT_DOUBLE_EQ(jacobians[i](0, 1), 0.0);
        EXPECT_DOUBLE_EQ(jacobians[i](1, 0), 0.0);
        EXPECT_NEAR(jacobians[i](0, 2), -centers[i].y(), 1e-12);
        EXPECT_NEAR(jacobians[i](1, 2), centers[i].x(), 1e-12);
    }
}

// 测试目标：验证 theta=0 时 x、y 平移仅对圆心做叠加，不影响雅可比。
// 测试流程：分别用 (0,0) 和 (5,-3) 作为后轴中心调用外圆插值，对比圆心
//          差值与雅可比第三列。
// 预期结果：shifted_centers = origin_centers + (5,-3)；theta 偏导不变。
TEST_F(VehicleFootprintModelTest, TranslationPreservesCentersAtZeroTheta) {
    const auto circle_num = model_.getCircleNum(CircleType::OUTER);
    std::vector<Eigen::Vector2d> origin_centers(circle_num);
    std::vector<Eigen::Vector2d> shifted_centers(circle_num);
    std::vector<Eigen::Matrix<double, 2, 3>> origin_jacobians(circle_num);
    std::vector<Eigen::Matrix<double, 2, 3>> shifted_jacobians(circle_num);

    model_.calInterpolatedCenters(0.0, 0.0, 0.0, CircleType::OUTER,
                                  origin_centers, origin_jacobians);
    model_.calInterpolatedCenters(5.0, -3.0, 0.0, CircleType::OUTER,
                                  shifted_centers, shifted_jacobians);

    for (int i = 0; i < circle_num; ++i) {
        EXPECT_NEAR(shifted_centers[i].x(), origin_centers[i].x() + 5.0, 1e-12);
        EXPECT_NEAR(shifted_centers[i].y(), origin_centers[i].y() - 3.0, 1e-12);
        EXPECT_NEAR(shifted_jacobians[i](0, 2), origin_jacobians[i](0, 2),
                    1e-12);
        EXPECT_NEAR(shifted_jacobians[i](1, 2), origin_jacobians[i](1, 2),
                    1e-12);
    }
}

// 测试目标：验证默认模型构造成功后内外圆半径与圆数量均为正。
// 测试流程：使用夹具中的默认模型直接查询半径与圆数量。
// 预期结果：内外半径均大于 0，内外圆数量均大于 0。
TEST_F(VehicleFootprintModelTest, RadiiAndCircleCountsArePositive) {
    EXPECT_GT(model_.getInnerRadius(), 0.0);
    EXPECT_GT(model_.getOuterRadius(), 0.0);
    EXPECT_GT(model_.getCircleNum(CircleType::INNER), 0);
    EXPECT_GT(model_.getCircleNum(CircleType::OUTER), 0);
}

// 测试目标：验证外圆雅可比矩阵同样满足 2x3 形状与 theta 偏导约束。
// 测试流程：在 theta=0、x=y=0 处调用外圆插值，遍历每个外圆的雅可比。
// 预期结果：每个雅可比为 2x3；J(0,0)=J(1,1)=1；第三列为 (-y, x)。
TEST_F(VehicleFootprintModelTest, OuterCircleJacobianShapeAndValues) {
    const auto circle_num = model_.getCircleNum(CircleType::OUTER);
    std::vector<Eigen::Vector2d> centers(circle_num);
    std::vector<Eigen::Matrix<double, 2, 3>> jacobians(circle_num);

    model_.calInterpolatedCenters(0.0, 0.0, 0.0, CircleType::OUTER, centers,
                                  jacobians);

    for (int i = 0; i < circle_num; ++i) {
        EXPECT_EQ(jacobians[i].rows(), 2);
        EXPECT_EQ(jacobians[i].cols(), 3);
        EXPECT_DOUBLE_EQ(jacobians[i](0, 0), 1.0);
        EXPECT_DOUBLE_EQ(jacobians[i](1, 1), 1.0);
        EXPECT_NEAR(jacobians[i](0, 2), -centers[i].y(), 1e-12);
        EXPECT_NEAR(jacobians[i](1, 2), centers[i].x(), 1e-12);
    }
}
}  // namespace
}  // namespace apa_post_processor
