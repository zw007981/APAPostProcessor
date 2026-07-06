#include "core/NMPC/vehicle_circle_geometry.h"

#include <gtest/gtest.h>

#include <constraints/circle_footprint_esdf_constraint.h>

#include <cmath>
#include <stdexcept>

#include "vehicle/vehicle_footprint_model.h"
#include "vehicle/vehicle_params.h"

namespace apa_post_processor {
namespace {

// 公共测试夹具：复用同一组车辆参数与VehicleFootprintModel实例。
class VehicleCircleGeometryTest : public ::testing::Test {
   protected:
    static constexpr double kLength = 4.3;
    static constexpr double kWidth = 1.8;
    static constexpr double kWheelbase = 2.7;
    static constexpr double kMaxSteerAngle = 0.6;
    static constexpr double kRearOverhang = 0.8;
    // 用较小的outer_row_num，使外圆数量落在CircleFootprintEsdfConstraint::kMaxCircles以内
    static constexpr int kOuterRowNum = 1;
    VehicleCircleGeometryTest()
        : veh_params_(kLength, kWidth, kWheelbase, kMaxSteerAngle,
                      kRearOverhang),
          model_(veh_params_, /*heading_sample_num=*/233, /*inner_row_num=*/2,
                kOuterRowNum) {}

    VehicleParams veh_params_;
    VehicleFootprintModel model_;
};

// 测试提取的外圆局部坐标数量与VehicleFootprintModel::getCircleNum一致。
// 因为CircleFootprintEsdfConstraint的圆数量由该局部坐标序列的长度决定，数量对不上会导致约束漏检。
TEST_F(VehicleCircleGeometryTest, ExtractedCircleCountMatchesModel) {
    const auto centers = vehicle_circle_geometry::ExtractLocalCircleCenters(
        model_, CircleType::OUTER);
    EXPECT_EQ(centers.size(), model_.getCircleNum(CircleType::OUTER));
}

// 测试提取结果与直接调用calInterpolatedCenters(0,0,0,...)完全一致。
// 因为theta=0、位置为原点时不存在插值误差，两者必须逐分量相等，验证提取函数没有引入偏差。
TEST_F(VehicleCircleGeometryTest, ExtractedCentersMatchDirectQueryAtOrigin) {
    const auto extracted = vehicle_circle_geometry::ExtractLocalCircleCenters(
        model_, CircleType::OUTER);

    const auto circle_num =
        static_cast<int>(model_.getCircleNum(CircleType::OUTER));
    std::vector<Eigen::Vector2d> direct_centers(circle_num);
    std::vector<Eigen::Matrix<double, 2, 3>> jacobians(circle_num);
    model_.calInterpolatedCenters(0.0, 0.0, 0.0, CircleType::OUTER,
                                  direct_centers, jacobians);

    ASSERT_EQ(extracted.size(), direct_centers.size());
    for (std::size_t i = 0; i < extracted.size(); ++i) {
        EXPECT_TRUE(extracted[i].isApprox(direct_centers[i]));
    }
}

// 测试提取的局部坐标能直接用于构造CircleFootprintEsdfConstraint而不抛异常。
// 因为这是圆形分解模型与StcSQP碰撞约束对接的关键集成点，必须保证维度/数值都合法。
TEST_F(VehicleCircleGeometryTest, ExtractedCentersConstructCircleConstraint) {
    const auto centers = vehicle_circle_geometry::ExtractLocalCircleCenters(
        model_, CircleType::OUTER);

    EXPECT_NO_THROW(stc_SQP::CircleFootprintEsdfConstraint(
        centers, model_.getOuterRadius(), /*safety_margin=*/0.1));
}

}  // namespace
}  // namespace apa_post_processor
