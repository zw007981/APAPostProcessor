#include <cmath>

#include <gtest/gtest.h>

#include "../src/math/math_util.hpp"
#include "../src/math/se2.hpp"
#include "../src/math/so2.hpp"

using namespace stc_SQP;

// SO2 测试套件：验证角度规范化与流形 Retract 满足 (-π, π] 周期性
TEST(SO2Test, WrapNormalizesAngleToMinusPiPi)
{
    // 测试目的：验证 math_util::NormalizeAngle 将任意角度映射到 (-π, π] 区间
    // 流程：输入超过 2π 的角度，检查输出是否在预期区间且与原始角度等价
    // 预期效果：math_util::NormalizeAngle(±π) = π，math_util::NormalizeAngle(±3π) = π，math_util::NormalizeAngle(π/2) = π/2
    EXPECT_NEAR(math_util::NormalizeAngle(PI), PI, 1e-12);
    EXPECT_NEAR(math_util::NormalizeAngle(-PI), PI, 1e-12);
    EXPECT_NEAR(math_util::NormalizeAngle(3.0 * PI), PI, 1e-12);
    EXPECT_NEAR(math_util::NormalizeAngle(-3.0 * PI), PI, 1e-12);
    EXPECT_NEAR(math_util::NormalizeAngle(PI / 2.0), PI / 2.0, 1e-12);
}

TEST(SO2Test, RetractIsWrapOfSum)
{
    // 测试目的：验证 so2::Retract 等价于 (angle + delta) 再做 math_util::NormalizeAngle
    // 流程：取多个角度与增量组合，对比两种计算方式
    // 预期效果：so2::Retract(θ, δ) == math_util::NormalizeAngle(θ + δ)
    const std::vector<double> angles = { 0.0, 0.5, PI - 0.1, -PI + 0.1 };
    const std::vector<double> deltas = { 0.0, 0.2, 2.0 * PI, -2.0 * PI };
    for (double angle : angles) {
        for (double delta : deltas) {
            EXPECT_NEAR(so2::Retract(angle, delta), math_util::NormalizeAngle(angle + delta), 1e-12)
                << "angle=" << angle << ", delta=" << delta;
        }
    }
}

TEST(SO2Test, RetractHandlesCrossingPiBoundary)
{
    // 测试目的：验证 so2::Retract 跨过 π/-π 边界时正确回绕
    // 流程：在 π 附近施加正向增量，结果应从 π 跳变到 -π 附近
    // 预期效果：so2::Retract(π - 0.1, 0.3) ≈ -π + 0.2
    const double result = so2::Retract(PI - 0.1, 0.3);
    EXPECT_NEAR(result, -PI + 0.2, 1e-12);
}

// SE2 测试套件：验证二维刚体变换的正确性
TEST(SE2Test, TransformFromPoseBuildsCorrectMatrix)
{
    // 测试目的：验证由位姿构造的齐次变换矩阵与解析公式一致
    // 流程：给定 [x=1, y=2, theta=π/6]，计算 T 并与手算结果比较
    // 预期效果：T(0:2,0:2) 为旋转矩阵，T(0:2,2) 为平移向量
    Vector pose(3);
    pose << 1.0, 2.0, PI / 6.0;
    const Matrix T = se2::TransformFromPose(pose);
    EXPECT_NEAR(T(0, 0), std::cos(PI / 6.0), 1e-12);
    EXPECT_NEAR(T(0, 1), -std::sin(PI / 6.0), 1e-12);
    EXPECT_NEAR(T(1, 0), std::sin(PI / 6.0), 1e-12);
    EXPECT_NEAR(T(1, 1), std::cos(PI / 6.0), 1e-12);
    EXPECT_NEAR(T(0, 2), 1.0, 1e-12);
    EXPECT_NEAR(T(1, 2), 2.0, 1e-12);
    EXPECT_NEAR(T(2, 2), 1.0, 1e-12);
}

TEST(SE2Test, ApplyTransformRotatesAndTranslatesPoint)
{
    // 测试目的：验证 ApplyTransform 对局部点执行正确的旋转+平移
    // 流程：位姿在原点、朝向 π/2，将局部点 [1, 0] 变换到世界坐标
    // 预期效果：世界坐标为 [0, 1]
    Vector pose(3);
    pose << 0.0, 0.0, PI / 2.0;
    Vector p(2);
    p << 1.0, 0.0;
    const Vector world = se2::ApplyTransform(pose, p);
    EXPECT_NEAR(world(0), 0.0, 1e-12);
    EXPECT_NEAR(world(1), 1.0, 1e-12);
}

TEST(SE2Test, ApplyInverseTransformReversesTransform)
{
    // 测试目的：验证逆变换是世界变换的严格反操作
    // 流程：先 ApplyTransform 再 ApplyInverseTransform，应回到原始点
    // 预期效果：逆变换结果与原始点差 < 1e-12
    Vector pose(3);
    pose << 2.0, -1.0, PI / 4.0;
    Vector p(2);
    p << 0.5, -0.3;
    const Vector world = se2::ApplyTransform(pose, p);
    const Vector local = se2::ApplyInverseTransform(pose, world);
    EXPECT_TRUE(local.isApprox(p, 1e-12));
}

TEST(SE2Test, ComposeIsConsistentWithMatrixMultiplication)
{
    // 测试目的：验证位姿组合 compose 与齐次矩阵乘法等价
    // 流程：构造两个位姿，分别用 compose 和 T_a * T_b 计算，比较结果
    // 预期效果：两种方法得到的位姿一致
    Vector pose_a(3), pose_b(3);
    pose_a << 1.0, 0.0, PI / 3.0;
    pose_b << 0.5, -0.2, PI / 6.0;
    const Vector pose_compose = se2::Compose(pose_a, pose_b);
    const Matrix T_a = se2::TransformFromPose(pose_a);
    const Matrix T_b = se2::TransformFromPose(pose_b);
    const Matrix T_compose = T_a * T_b;
    const double theta_expected = std::atan2(T_compose(1, 0), T_compose(0, 0));
    EXPECT_NEAR(pose_compose(0), T_compose(0, 2), 1e-12);
    EXPECT_NEAR(pose_compose(1), T_compose(1, 2), 1e-12);
    EXPECT_NEAR(pose_compose(2), theta_expected, 1e-12);
}

TEST(SE2Test, ComposeWrapsThetaAcrossPiBoundary)
{
    // 测试目的：验证 compose 对组合后的 theta 做 SO2 wrap，结果落在 (-π, π]
    // 流程：取 pose_a.theta = π - 0.1，pose_b.theta = 0.3，组合后应跨过 π 边界并回绕
    // 预期效果：compose().theta = -π + 0.2，且与矩阵乘法得到的等价角度一致
    Vector pose_a(3), pose_b(3);
    pose_a << 0.0, 0.0, PI - 0.1;
    pose_b << 0.0, 0.0, 0.3;
    const Vector pose_compose = se2::Compose(pose_a, pose_b);
    const Matrix T_a = se2::TransformFromPose(pose_a);
    const Matrix T_b = se2::TransformFromPose(pose_b);
    const Matrix T_compose = T_a * T_b;
    const double theta_expected = std::atan2(T_compose(1, 0), T_compose(0, 0));
    EXPECT_NEAR(pose_compose(2), theta_expected, 1e-12);
    EXPECT_GT(pose_compose(2), -PI);
    EXPECT_LE(pose_compose(2), PI);
}
