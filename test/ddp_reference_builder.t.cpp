#include "core/DDP/ddp_reference_builder.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "util/constants.h"
#include "util/data_loader.hpp"

namespace apa_post_processor {
namespace {

// 测试用车辆参数：本组件只消费轴距（δ = atan(L·κ)
// 反解），取与真实数据一致的 3.0 m
VehicleParams MakeVehicleParams() { return VehicleParams{4.9, 1.9, 3.0, 0.48}; }

// 从当前路径末端沿 x 轴追加直线路径点（步长 0.05 m，与 A* 点距一致）
void AppendXLine(Path* path, double x_from, double x_to, double theta) {
    const int count =
        static_cast<int>(std::round(std::abs(x_to - x_from) / 0.05));
    for (int i = 1; i <= count; ++i) {
        const double x = x_from + (x_to - x_from) * i / count;
        path->addPoint({x, 0.0, theta});
    }
}

// 构造 前进 1.0 m → 后退 0.7 m → 前进 0.5 m 的两次换挡路径
// （每段长度均为 0.05 m 的整数倍，cusp 恰好落在重采样网格点上）
Path BuildTwoShiftPath() {
    Path path;
    path.addPoint({0.0, 0.0, 0.0});
    AppendXLine(&path, 0.0, 1.0, 0.0);
    AppendXLine(&path, 1.0, 0.3, 0.0);
    AppendXLine(&path, 0.3, 0.8, 0.0);
    path.finalize();
    return path;
}

// 沿半径 radius、圆心 (0, radius) 的圆弧追加密集路径点：
// 起点为原点、起始朝向 0，切向角即圆心角 phi，采样弧长步长约 0.005 m
void AppendArc(Path* path, double radius, double phi_to) {
    const int count = static_cast<int>(std::round(phi_to * radius / 0.005));
    for (int i = 1; i <= count; ++i) {
        const double phi = phi_to * i / count;
        path->addPoint(
            {radius * std::sin(phi), radius * (1.0 - std::cos(phi)), phi});
    }
}

// 测试等弧长重采样的间距均匀性与端点保持。
// 因为原始点列严格按 0.05 m 排布且总长按 0.05 m 整除，所以重采样结果必须
// 与原点列逐点重合；总长不整除时按全长归一，间距 = L / round(L / 0.05)，
// 首末点仍必须严格保持。
TEST(DdpReferenceBuilderTest, StraightLineResamplesToUniformSpacing) {
    const DdpReferenceBuilder builder(DdpReferenceBuilderConfig{},
                                      MakeVehicleParams());
    Path path;
    path.addPoint({0.0, 0.0, 0.0});
    AppendXLine(&path, 0.0, 2.0, 0.0);
    path.finalize();

    const DdpReference reference = builder.build(path);
    // 2.0 m / 0.05 m = 40 步、41 个参考位姿
    ASSERT_EQ(reference.poses.size(), 41);
    EXPECT_NEAR(reference.ds, 0.05, 1e-12);
    EXPECT_NEAR(reference.dt, 0.1, 1e-15);
    EXPECT_NEAR(reference.poses.front().x, 0.0, 1e-12);
    EXPECT_NEAR(reference.poses.back().x, 2.0, 1e-12);
    for (std::size_t k = 1; k < reference.poses.size(); ++k) {
        const double gap =
            std::hypot(reference.poses[k].x - reference.poses[k - 1].x,
                       reference.poses[k].y - reference.poses[k - 1].y);
        EXPECT_NEAR(gap, reference.ds, 1e-9);
        EXPECT_NEAR(reference.poses[k].theta, 0.0, 1e-12);
    }

    // 总长 1.03 m 不是 0.05 m 的整数倍：全长归一到 21 段，间距 1.03/21
    Path odd_path;
    odd_path.addPoint({0.0, 0.0, 0.0});
    AppendXLine(&odd_path, 0.0, 1.03, 0.0);
    odd_path.finalize();
    const DdpReference odd_reference = builder.build(odd_path);
    ASSERT_EQ(odd_reference.poses.size(), 22);
    EXPECT_NEAR(odd_reference.ds, 1.03 / 21.0, 1e-12);
    EXPECT_NEAR(odd_reference.poses.back().x, 1.03, 1e-9);
}

// 测试含两次换挡路径的 cusp 检测与 maneuver 元数据。
// 因为换挡点即 Path 的 Maneuver 交界，所以 cusp 索引必须落在两段交界处对应的
// 网格点上，maneuver 的 (s_m, Δs_m, Δθ_m) 与网格起止索引必须与手工对拍一致；
// 同时 cusp 不产生任何 v=0 语义，初值速度在 cusp 两侧只是变号而非清零。
TEST(DdpReferenceBuilderTest, ShiftPathProducesCuspsAndManeuverMetadata) {
    const DdpReference reference =
        DdpReferenceBuilder(DdpReferenceBuilderConfig{}, MakeVehicleParams())
            .build(BuildTwoShiftPath());
    // 总长 2.2 m → N = 44 步、45 个位姿
    ASSERT_EQ(reference.poses.size(), 45);
    ASSERT_EQ(reference.maneuvers.size(), 3);
    EXPECT_EQ(reference.maneuvers[0].sign, 1);
    EXPECT_EQ(reference.maneuvers[1].sign, -1);
    EXPECT_EQ(reference.maneuvers[2].sign, 1);
    EXPECT_NEAR(reference.maneuvers[0].delta_s, 1.0, 1e-9);
    EXPECT_NEAR(reference.maneuvers[1].delta_s, 0.7, 1e-9);
    EXPECT_NEAR(reference.maneuvers[2].delta_s, 0.5, 1e-9);
    for (const auto& maneuver : reference.maneuvers) {
        EXPECT_NEAR(maneuver.delta_theta, 0.0, 1e-12);
    }
    EXPECT_EQ(reference.maneuvers[0].begin_index, 0);
    EXPECT_EQ(reference.maneuvers[0].end_index, 20);
    EXPECT_EQ(reference.maneuvers[1].begin_index, 20);
    EXPECT_EQ(reference.maneuvers[1].end_index, 34);
    EXPECT_EQ(reference.maneuvers[2].begin_index, 34);
    EXPECT_EQ(reference.maneuvers[2].end_index, 44);
    // cusp 集 = 两个方向反号边界对应的网格索引
    const std::vector<std::size_t> expected_cusps = {20, 34};
    EXPECT_EQ(reference.cusp_indices, expected_cusps);
    // 初值速度在 cusp 处变号但不为零：cusp 索引归属后一段 maneuver
    EXPECT_NEAR(reference.initial_states[19](DDP_IDX_V), 0.5, 1e-9);
    EXPECT_NEAR(reference.initial_states[20](DDP_IDX_V), -0.5, 1e-9);
    EXPECT_NEAR(reference.initial_states[33](DDP_IDX_V), -0.5, 1e-9);
    EXPECT_NEAR(reference.initial_states[34](DDP_IDX_V), 0.5, 1e-9);
    EXPECT_NEAR(reference.initial_states[44](DDP_IDX_V), 0.5, 1e-9);
}

// 测试匀速直线路径的差分初值与解析值一致。
// 因为等弧长 0.05 m + 固定 dt=0.1 s 意味着名义速度恒为 ±0.5 m/s，
// 直线上 κ=0，所以 v/δ/a/ω 初值必须处处为解析值，控制初值恒为零向量，
// 且状态/控制序列长度必须分别为 N+1 与 N。
TEST(DdpReferenceBuilderTest, StraightLineInitialGuessMatchesAnalyticValues) {
    Path path;
    path.addPoint({0.0, 0.0, 0.0});
    AppendXLine(&path, 0.0, 2.0, 0.0);
    path.finalize();
    const DdpReference reference =
        DdpReferenceBuilder(DdpReferenceBuilderConfig{}, MakeVehicleParams())
            .build(path);
    ASSERT_EQ(reference.initial_states.size(), 41);
    ASSERT_EQ(reference.initial_controls.size(), 40);
    for (const auto& state : reference.initial_states) {
        EXPECT_NEAR(state(DDP_IDX_V), 0.5, 1e-12);
        EXPECT_NEAR(state(DDP_IDX_A), 0.0, 1e-12);
        EXPECT_NEAR(state(DDP_IDX_DELTA), 0.0, 1e-12);
        EXPECT_NEAR(state(DDP_IDX_OMEGA), 0.0, 1e-12);
    }
    for (const auto& control : reference.initial_controls) {
        EXPECT_DOUBLE_EQ(control(0), 0.0);
        EXPECT_DOUBLE_EQ(control(1), 0.0);
    }
}

// 测试定曲率圆弧上的差分初值与解析值一致。
// 因为圆弧上 θ 随弧长严格线性变化（κ = 1/R = 0.2），所以重采样后的
// 差分曲率必须复现解析值：δ = atan(L_base · κ) = atan(0.6)，v 恒为名义值，
// 中心差分得到的 a/ω 在定常 v/δ 下必须为零。
TEST(DdpReferenceBuilderTest, CircularArcInitialGuessMatchesAnalyticCurvature) {
    Path path;
    path.addPoint({0.0, 0.0, 0.0});
    AppendArc(&path, 5.0, 0.5);
    path.finalize();
    const DdpReference reference =
        DdpReferenceBuilder(DdpReferenceBuilderConfig{}, MakeVehicleParams())
            .build(path);
    // 弧长 2.5 m → N = 50 步
    ASSERT_EQ(reference.initial_states.size(), 51);
    const double expected_delta = std::atan(3.0 * 0.2);
    for (const auto& state : reference.initial_states) {
        // 密集弦采样使总长相对理想圆弧短约 4e-8（相对量），容差随之标定
        EXPECT_NEAR(state(DDP_IDX_V), 0.5, 1e-6);
        EXPECT_NEAR(state(DDP_IDX_A), 0.0, 1e-6);
        EXPECT_NEAR(state(DDP_IDX_OMEGA), 0.0, 1e-6);
    }
    // 内部 knot 的 δ 与解析值一致（端点由单侧/拷贝差分得到，同样应一致）
    for (std::size_t k = 0; k < reference.initial_states.size(); ++k) {
        EXPECT_NEAR(reference.initial_states[k](DDP_IDX_DELTA), expected_delta,
                    1e-6)
            << "k = " << k;
    }
}

// 测试越界初值被裁剪进盒约束。
// 因为 cusp 处 v 变号使中心差分产生 |a| = Δv/(2·dt) = 3.0 m/s² 的尖峰，
// 超出 a_max=1.0，所以裁剪后必须钉在边界上；v_max=0.3 小于名义速度 0.5，
// 所以速度初值必须整体压到 ±0.3；曲率 κ=1.0 的圆弧给出 δ=atan(3.0)>δ_max，
// 转角初值必须裁到 δ_max。
TEST(DdpReferenceBuilderTest, InitialGuessIsClippedIntoBox) {
    DdpReferenceBuilderConfig config;
    config.v_max = 0.3;
    const DdpReference reference =
        DdpReferenceBuilder(config, MakeVehicleParams())
            .build(BuildTwoShiftPath());
    for (const auto& state : reference.initial_states) {
        EXPECT_LE(std::abs(state(DDP_IDX_V)), config.v_max + 1e-12);
        EXPECT_LE(std::abs(state(DDP_IDX_A)), config.a_max + 1e-12);
        EXPECT_LE(std::abs(state(DDP_IDX_DELTA)), config.delta_max + 1e-12);
        EXPECT_LE(std::abs(state(DDP_IDX_OMEGA)), config.omega_max + 1e-12);
    }
    EXPECT_NEAR(reference.initial_states[0](DDP_IDX_V), 0.3, 1e-12);
    EXPECT_NEAR(reference.initial_states[20](DDP_IDX_V), -0.3, 1e-12);
    // 第一个 cusp 附近 a 尖峰被裁到 -a_max，第二个 cusp 附近为 +a_max
    EXPECT_NEAR(reference.initial_states[19](DDP_IDX_A), -config.a_max, 1e-12);
    EXPECT_NEAR(reference.initial_states[33](DDP_IDX_A), config.a_max, 1e-12);

    Path sharp_arc;
    sharp_arc.addPoint({0.0, 0.0, 0.0});
    AppendArc(&sharp_arc, 1.0, 0.3);
    sharp_arc.finalize();
    const DdpReference arc_reference =
        DdpReferenceBuilder(DdpReferenceBuilderConfig{}, MakeVehicleParams())
            .build(sharp_arc);
    for (const auto& state : arc_reference.initial_states) {
        EXPECT_LE(std::abs(state(DDP_IDX_DELTA)),
                  DdpReferenceBuilderConfig{}.delta_max + 1e-12);
    }
    EXPECT_NEAR(arc_reference.initial_states[3](DDP_IDX_DELTA),
                DdpReferenceBuilderConfig{}.delta_max, 1e-12);
}

// 测试打靶节点布设：{每 n_s 步} ∪ {全部 cusp} ∪ {末点 N}。
// 因为规则网格与 cusp/末点取并集，所以结果必须升序无重复、包含全部 cusp 与
// 0/N 端点、相邻间隔不超过 n_s；n_s 超过 N 时只剩 {0, cusp…, N}。
TEST(DdpReferenceBuilderTest, ShootingNodesCoverCuspsEndAndIntervalBound) {
    const Path path = BuildTwoShiftPath();
    const DdpReference reference =
        DdpReferenceBuilder(DdpReferenceBuilderConfig{}, MakeVehicleParams())
            .build(path);
    const std::vector<std::size_t> expected_nodes = {0, 20, 25, 34, 44};
    EXPECT_EQ(reference.shooting_nodes, expected_nodes);

    DdpReferenceBuilderConfig wide_config;
    wide_config.shooting_interval = 100;
    const DdpReference wide_reference =
        DdpReferenceBuilder(wide_config, MakeVehicleParams()).build(path);
    const std::vector<std::size_t> expected_wide = {0, 20, 34, 44};
    EXPECT_EQ(wide_reference.shooting_nodes, expected_wide);

    DdpReferenceBuilderConfig dense_config;
    dense_config.shooting_interval = 10;
    const DdpReference dense_reference =
        DdpReferenceBuilder(dense_config, MakeVehicleParams()).build(path);
    const std::vector<std::size_t> expected_dense = {0, 10, 20, 30, 34, 40, 44};
    EXPECT_EQ(dense_reference.shooting_nodes, expected_dense);

    // 通用不变量：升序无重复、含 0 与 N、含全部 cusp、间隔不超 n_s
    const auto& nodes = dense_reference.shooting_nodes;
    EXPECT_TRUE(std::is_sorted(nodes.begin(), nodes.end()));
    EXPECT_EQ(std::adjacent_find(nodes.begin(), nodes.end()), nodes.end());
    EXPECT_EQ(nodes.front(), 0);
    EXPECT_EQ(nodes.back(), 44);
    for (const std::size_t cusp : dense_reference.cusp_indices) {
        EXPECT_NE(std::find(nodes.begin(), nodes.end(), cusp), nodes.end());
    }
    for (std::size_t i = 1; i < nodes.size(); ++i) {
        EXPECT_LE(nodes[i] - nodes[i - 1], dense_config.shooting_interval);
    }
}

// 测试退化输入的失败语义：空路径、单点路径、总长不足一个采样间距、
// 纯原地旋转（零位移）路径都必须抛出 std::invalid_argument 而非崩溃或
// 静默产出无意义结果。
TEST(DdpReferenceBuilderTest, DegeneratePathsThrowInvalidArgument) {
    const DdpReferenceBuilder builder(DdpReferenceBuilderConfig{},
                                      MakeVehicleParams());
    Path empty_path;
    EXPECT_THROW(builder.build(empty_path), std::invalid_argument);

    Path single_point_path;
    single_point_path.addPoint({0.0, 0.0, 0.0});
    single_point_path.finalize();
    EXPECT_THROW(builder.build(single_point_path), std::invalid_argument);

    Path short_path;
    short_path.addPoint({0.0, 0.0, 0.0});
    short_path.addPoint({0.03, 0.0, 0.0});
    short_path.finalize();
    EXPECT_THROW(builder.build(short_path), std::invalid_argument);

    Path pivot_path;
    pivot_path.addPoint({0.0, 0.0, 0.0});
    pivot_path.addPoint({0.0, 0.0, 0.5});
    pivot_path.finalize();
    EXPECT_THROW(builder.build(pivot_path), std::invalid_argument);
}

// 测试跨 ±π 路径的角度处理：θ 差分必须经统一 wrap()，不产生 2π 跳变伪影。
// 因为朝向从 π-0.1 连续增大并跨过 ±π 分界线（每步 +0.005 rad），所以
// 重采样位姿的相邻 wrap 差分必须恒为 +0.005，差分曲率 κ=0.1、
// δ=atan(0.3) 必须处处一致，任何未 wrap 的实现都会在跨界处产生巨幅尖峰。
TEST(DdpReferenceBuilderTest, HeadingWrapAcrossPiHasNoSpuriousArtifacts) {
    Path path;
    for (int k = 0; k <= 40; ++k) {
        const double theta = std::remainder(PI - 0.1 + k * 0.005, 2.0 * PI);
        path.addPoint({-k * 0.05, 0.0, theta});
    }
    path.finalize();
    const DdpReference reference =
        DdpReferenceBuilder(DdpReferenceBuilderConfig{}, MakeVehicleParams())
            .build(path);
    ASSERT_EQ(reference.poses.size(), 41);
    for (std::size_t k = 1; k < reference.poses.size(); ++k) {
        const double dtheta = std::remainder(
            reference.poses[k].theta - reference.poses[k - 1].theta, 2.0 * PI);
        EXPECT_NEAR(dtheta, 0.005, 1e-12) << "k = " << k;
    }
    const double expected_delta = std::atan(3.0 * 0.1);
    for (const auto& state : reference.initial_states) {
        EXPECT_NEAR(state(DDP_IDX_DELTA), expected_delta, 1e-9);
        EXPECT_NEAR(state(DDP_IDX_V), 0.5, 1e-9);
    }
}

// 四份真实数据集的验收测试：Path 均可成功预处理，输出规模与
// DDP.md 2.1 节量级一致（N = round(L/0.05)、打靶节点约 N/n_s+|C|+1 个），
// cusp 数量与 Path 的方向反号边界一一对应，初值全部有限且在盒内。
class DdpReferenceDatasetTest
    : public ::testing::TestWithParam<std::pair<std::string, std::string>> {};

TEST_P(DdpReferenceDatasetTest, RealDatasetPreprocessesToExpectedScale) {
    const auto& [name, file] = GetParam();
    ::apa::post_processor::OptimizeRequest request;
    ASSERT_EQ(DataLoader::LoadProtoFromJsonFile(file, request),
              LoadResult::SUCCESS);
    const auto vehicle_params = VehicleParams::FromProto(request.vehicle());
    const auto init_path = Path::FromProto(request.initial_path());
    ASSERT_FALSE(init_path.empty());
    const DdpReferenceBuilderConfig config;
    const DdpReference reference =
        DdpReferenceBuilder(config, vehicle_params).build(init_path);

    // 规模量级：N = round(L / 0.05)，状态/控制序列长度为 N+1 与 N
    const double total_length = init_path.length();
    const std::size_t expected_n = std::max<std::size_t>(
        1, static_cast<std::size_t>(std::lround(total_length / 0.05)));
    ASSERT_EQ(reference.poses.size(), expected_n + 1);
    ASSERT_EQ(reference.initial_states.size(), expected_n + 1);
    ASSERT_EQ(reference.initial_controls.size(), expected_n);

    // cusp 数量与 Path 的方向反号边界一一对应
    const auto& maneuvers = init_path.getManeuvers();
    auto direction_sign = [](Direction direction) {
        if (direction == Direction::FORWARD) {
            return 1;
        }
        if (direction == Direction::BACKWARD) {
            return -1;
        }
        return 0;
    };
    std::size_t expected_cusps = 0;
    for (std::size_t m = 1; m < maneuvers.size(); ++m) {
        if (direction_sign(maneuvers[m - 1].direction) *
                direction_sign(maneuvers[m].direction) ==
            -1) {
            ++expected_cusps;
        }
    }
    EXPECT_EQ(reference.cusp_indices.size(), expected_cusps);
    EXPECT_GT(reference.cusp_indices.size(), 0);

    // 打靶节点：含 0 与 N、含全部 cusp、间隔不超 n_s、规模约 N/n_s+|C|+1
    const auto& nodes = reference.shooting_nodes;
    ASSERT_FALSE(nodes.empty());
    EXPECT_EQ(nodes.front(), 0);
    EXPECT_EQ(nodes.back(), expected_n);
    EXPECT_TRUE(std::is_sorted(nodes.begin(), nodes.end()));
    EXPECT_EQ(std::adjacent_find(nodes.begin(), nodes.end()), nodes.end());
    for (const std::size_t cusp : reference.cusp_indices) {
        EXPECT_NE(std::find(nodes.begin(), nodes.end(), cusp), nodes.end());
        EXPECT_LT(cusp, expected_n + 1);
    }
    for (std::size_t i = 1; i < nodes.size(); ++i) {
        EXPECT_LE(nodes[i] - nodes[i - 1], config.shooting_interval);
    }
    EXPECT_LE(nodes.size(), expected_n / config.shooting_interval +
                                reference.cusp_indices.size() + 2);

    // 端点保持：首末参考位姿与原始路径一致
    EXPECT_NEAR(reference.poses.front().x, init_path.front().x, 1e-9);
    EXPECT_NEAR(reference.poses.front().y, init_path.front().y, 1e-9);
    EXPECT_NEAR(reference.poses.back().x, init_path.back().x, 1e-9);
    EXPECT_NEAR(reference.poses.back().y, init_path.back().y, 1e-9);
    EXPECT_NEAR(
        std::remainder(reference.poses.back().theta - init_path.back().theta,
                       2.0 * PI),
        0.0, 1e-9);

    // 初值全部有限且裁剪在盒内，控制初值恒为零
    for (const auto& state : reference.initial_states) {
        for (int i = 0; i < DDP_STATE_DIM; ++i) {
            EXPECT_TRUE(std::isfinite(state(i)));
        }
        EXPECT_LE(std::abs(state(DDP_IDX_V)), config.v_max + 1e-12);
        EXPECT_LE(std::abs(state(DDP_IDX_A)), config.a_max + 1e-12);
        EXPECT_LE(std::abs(state(DDP_IDX_DELTA)), config.delta_max + 1e-12);
        EXPECT_LE(std::abs(state(DDP_IDX_OMEGA)), config.omega_max + 1e-12);
    }
    for (const auto& control : reference.initial_controls) {
        EXPECT_DOUBLE_EQ(control(0), 0.0);
        EXPECT_DOUBLE_EQ(control(1), 0.0);
    }
    std::cout << "[DDP-REF] dataset=" << name << " N=" << expected_n
              << " cusps=" << reference.cusp_indices.size()
              << " shooting_nodes=" << nodes.size()
              << " length=" << total_length << std::endl;
}

INSTANTIATE_TEST_SUITE_P(
    FourDatasets, DdpReferenceDatasetTest,
    ::testing::Values(std::make_pair(std::string("data3_mid_park"),
                                     std::string("data/mid_park/data3.json")),
                      std::make_pair(std::string("data1_rub_park"),
                                     std::string("data/rub_park/data1.json")),
                      std::make_pair(std::string("data7_rub_park"),
                                     std::string("data/rub_park/data7.json")),
                      std::make_pair(std::string("data6_long_park"),
                                     std::string("data/long_park/data6.json"))),
    [](const ::testing::TestParamInfo<std::pair<std::string, std::string>>&
           info) { return info.param.first; });

}  // namespace
}  // namespace apa_post_processor
