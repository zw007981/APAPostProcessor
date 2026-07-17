#include <gtest/gtest.h>

#include <cmath>
#include <string>

#include "vehicle/vehicle_params.h"

namespace apa_post_processor {
namespace {

::apa::post_processor::VehicleParams BuildVehicleParamsProto(
    double length, double width, double wheelbase, double max_steer_angle,
    double rear_overhang = 0.0, double max_accel = 1.0, double max_decel = -1.5,
    double max_steer_rate = 0.4) {
    ::apa::post_processor::VehicleParams proto;
    proto.set_length(length);
    proto.set_width(width);
    proto.set_wheelbase(wheelbase);
    proto.set_max_steer_angle(max_steer_angle);
    proto.set_rear_overhang(rear_overhang);
    proto.set_max_accel(max_accel);
    proto.set_max_decel(max_decel);
    proto.set_max_steer_rate(max_steer_rate);
    return proto;
}

void ExpectFromProtoThrows(double length, double width, double wheelbase,
                           double max_steer_angle) {
    const auto proto =
        BuildVehicleParamsProto(length, width, wheelbase, max_steer_angle);

    EXPECT_THROW((void)VehicleParams::FromProto(proto), std::invalid_argument);
}

// 测试 VehicleParams 默认构造的场景。
// 因为默认车辆参数用于无输入时的安全初值，所以所有几何字段都应初始化为 0。
TEST(VehicleParamsTest, DefaultConstructInitializesZeroValues) {
    const VehicleParams vehicle_params;

    EXPECT_DOUBLE_EQ(vehicle_params.length, 0.0);
    EXPECT_DOUBLE_EQ(vehicle_params.width, 0.0);
    EXPECT_DOUBLE_EQ(vehicle_params.wheelbase, 0.0);
    EXPECT_DOUBLE_EQ(vehicle_params.max_steer_angle, 0.0);
    EXPECT_DOUBLE_EQ(vehicle_params.rear_overhang, 0.0);
    EXPECT_DOUBLE_EQ(vehicle_params.max_kappa, 0.0);
    EXPECT_DOUBLE_EQ(vehicle_params.max_accel, 1.0);
    EXPECT_DOUBLE_EQ(vehicle_params.max_decel, -1.5);
    EXPECT_DOUBLE_EQ(vehicle_params.max_steer_rate, 0.4);
}

// 测试使用车辆几何参数构造 VehicleParams 的场景。
// 因为调用方会直接传入车辆尺寸和最大转角，所以构造函数应完整保留全部输入值。
TEST(VehicleParamsTest, ConstructWithValuesStoresAllFields) {
    const VehicleParams vehicle_params{4.3, 1.8, 2.7, 0.6, 0.5};

    EXPECT_DOUBLE_EQ(vehicle_params.length, 4.3);
    EXPECT_DOUBLE_EQ(vehicle_params.width, 1.8);
    EXPECT_DOUBLE_EQ(vehicle_params.wheelbase, 2.7);
    EXPECT_DOUBLE_EQ(vehicle_params.max_steer_angle, 0.6);
    EXPECT_DOUBLE_EQ(vehicle_params.rear_overhang, 0.5);
    EXPECT_NEAR(vehicle_params.max_kappa, std::tan(0.6) / 2.7, 1e-12);
    EXPECT_DOUBLE_EQ(vehicle_params.max_accel, 1.0);
    EXPECT_DOUBLE_EQ(vehicle_params.max_decel, -1.5);
    EXPECT_DOUBLE_EQ(vehicle_params.max_steer_rate, 0.4);
}

// 测试最大曲率在正常轴距下的计算场景。
// 因为车辆最大曲率由最大前轮偏角和轴距共同决定，所以 max_kappa 应等于
// tan(max_steer_angle) / wheelbase。
TEST(VehicleParamsTest, MaxKappaCalculatesFromSteerAngleAndWheelbase) {
    const VehicleParams vehicle_params{4.3, 1.8, 2.5, 0.4};

    EXPECT_NEAR(vehicle_params.max_kappa, std::tan(0.4) / 2.5, 1e-12);
}

// 测试轴距为 0 时最大曲率的边界场景。
// 因为 0 轴距会导致除零风险，所以 max_kappa 应被保护性设置为 0。
TEST(VehicleParamsTest, MaxKappaIsZeroWhenWheelbaseIsZero) {
    const VehicleParams vehicle_params{4.3, 1.8, 0.0, 0.4};

    EXPECT_DOUBLE_EQ(vehicle_params.max_kappa, 0.0);
}

// 测试轴距绝对值等于 EPSILON 时最大曲率的边界场景。
// 因为实现只在 abs(wheelbase) 大于 EPSILON 时计算曲率，所以正负 EPSILON
// 都应返回 0。
TEST(VehicleParamsTest, MaxKappaIsZeroWhenWheelbaseAbsEqualsEpsilon) {
    const VehicleParams positive_vehicle_params{4.3, 1.8, EPSILON, 0.4};
    const VehicleParams negative_vehicle_params{4.3, 1.8, -EPSILON, 0.4};

    EXPECT_DOUBLE_EQ(positive_vehicle_params.max_kappa, 0.0);
    EXPECT_DOUBLE_EQ(negative_vehicle_params.max_kappa, 0.0);
}

// 测试轴距刚超过 EPSILON 时最大曲率恢复计算的边界场景。
// 因为该值已经离开除零保护区间，所以 max_kappa 应按 tan(max_steer_angle) /
// wheelbase 计算。
TEST(VehicleParamsTest, MaxKappaCalculatesWhenWheelbaseJustExceedsEpsilon) {
    const double wheelbase = EPSILON * 1.1;
    const VehicleParams vehicle_params{4.3, 1.8, wheelbase, 0.4};

    EXPECT_NEAR(vehicle_params.max_kappa, std::tan(0.4) / wheelbase, 1e-12);
}

// 测试从 protobuf VehicleParams 消息构造业务 VehicleParams 的场景。
// 因为外部配置通过 proto 传入，所以 FromProto
// 应逐字段复制车辆尺寸、最大转角和后悬距。
TEST(VehicleParamsTest, FromProtoBuildsVehicleParamFields) {
    const auto proto = BuildVehicleParamsProto(4.5, 1.9, 2.8, 0.7, 0.3);

    const VehicleParams vehicle_params = VehicleParams::FromProto(proto);

    EXPECT_DOUBLE_EQ(vehicle_params.length, 4.5);
    EXPECT_DOUBLE_EQ(vehicle_params.width, 1.9);
    EXPECT_DOUBLE_EQ(vehicle_params.wheelbase, 2.8);
    EXPECT_DOUBLE_EQ(vehicle_params.max_steer_angle, 0.7);
    EXPECT_DOUBLE_EQ(vehicle_params.rear_overhang, 0.3);
    EXPECT_NEAR(vehicle_params.max_kappa, std::tan(0.7) / 2.8, 1e-12);
    EXPECT_DOUBLE_EQ(vehicle_params.max_accel, 1.0);
    EXPECT_DOUBLE_EQ(vehicle_params.max_decel, -1.5);
    EXPECT_DOUBLE_EQ(vehicle_params.max_steer_rate, 0.4);
}

// 测试 FromProto 拒绝非法车辆参数的场景。
// 因为 proto 是外部输入边界，所以任何小于 EPSILON
// 的尺寸或最大转角、以及负値的后悬距都应抛出异常。
TEST(VehicleParamsTest, FromProtoThrowsWhenAnyDimensionIsInvalid) {
    ExpectFromProtoThrows(0.0, 1.9, 2.8, 0.7);
    ExpectFromProtoThrows(4.5, 0.0, 2.8, 0.7);
    ExpectFromProtoThrows(4.5, 1.9, 0.0, 0.7);
    ExpectFromProtoThrows(4.5, 1.9, 2.8, 0.0);
    ExpectFromProtoThrows(-4.5, 1.9, 2.8, 0.7);
}

// 测试 FromProto 拒绝负値后悬距的场景。
// 因为后悬距表示物理距离，负値无意义，应拒绝这类非法输入。
TEST(VehicleParamsTest, FromProtoThrowsWhenRearOverhangIsNegative) {
    const auto proto = BuildVehicleParamsProto(4.5, 1.9, 2.8, 0.7, -0.1);

    EXPECT_THROW((void)VehicleParams::FromProto(proto), std::invalid_argument);
}

// 测试 FromProto 接受后悬距为 0 的场景。
// 因为后轴正好处于车身末端时后悬距合法为 0，所以不应抛出异常。
TEST(VehicleParamsTest, FromProtoAcceptsZeroRearOverhang) {
    const auto proto = BuildVehicleParamsProto(4.5, 1.9, 2.8, 0.7, 0.0);

    EXPECT_NO_THROW((void)VehicleParams::FromProto(proto));

    const VehicleParams vehicle_params = VehicleParams::FromProto(proto);
    EXPECT_DOUBLE_EQ(vehicle_params.rear_overhang, 0.0);
}

// 测试 VehicleParams 转换为 JSON 字符串的场景。
// 因为日志输出需要稳定字段名和两位小数精度，所以 toString 应输出全部车辆参数的
// JSON 文本。
TEST(VehicleParamsTest, ToStringBuildsJsonText) {
    const VehicleParams vehicle_params{4.321, 1.876, 2.789, 0.654, 0.35};

    EXPECT_EQ(vehicle_params.toString(),
              std::string("{\"length\": 4.32, \"width\": 1.88, "
                          "\"wheelbase\": 2.79, "
                          "\"max_steer_angle\": 0.65, "
                          "\"rear_overhang\": 0.35, "
                          "\"max_accel\": 1.00, "
                          "\"max_decel\": -1.50, "
                          "\"max_steer_rate\": 0.40, "
                          "\"max_kappa\": 0.27}"));
}

}  // namespace
}  // namespace apa_post_processor