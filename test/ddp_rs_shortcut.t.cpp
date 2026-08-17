// Reeds-Shepp 换挡点短接（ShortcutShiftPoints）单元测试：冗余绕行的
// 短接消除、碰撞守卫回滚、长度增长守卫回滚、端点保持、关闭时逐位透传、
// 配置校验。
// 与前两级几何前处理的本质区别：cusp 剪枝与曲率投影都严格保持前端给出
// 的同伦类（换挡点位置由前端钉死），本级主动更换同伦类——以 maneuver
// 边界为节点做动态规划全局择优，用有界曲率的 RS 曲线直连节点对，因此
// 每次替换都必须由逐点 ESDF 校验与长度增长上限兑底。
#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "core/DDP/ddp_reference_builder.h"
#include "core/collision_check.h"
#include "spatial/esdf_map.h"
#include "spatial/grid_map.h"
#include "vehicle/vehicle_footprint_model.h"
#include "vehicle/vehicle_params.h"

namespace apa_post_processor {
namespace {

constexpr double kWheelbase = 3.0;
constexpr double kDeltaMax = 0.47728;

// 测试用车辆参数与外圆模型（与 DDP 数据集同源：轴距 3.0 m）
VehicleParams MakeVehicleParams() { return VehicleParams{4.9, 1.9, 3.0, 0.48}; }

VehicleFootprintModel MakeFootprint() {
    return VehicleFootprintModel(MakeVehicleParams(), 233, 2, 1);
}

// 空旷地图（60m×60m、分辨率 0.125，仅远角一个占据点保证距离场良定义）。
// 地图必须比测试路径大出一个车身以上——短接的安全判据要求全部外圆心
// 落在图内，地图太小会让本可接受的候选被越界判据误拒
ESDFMap MakeEmptyMap() {
    const GridMap grid_map(0.125, 480, 480, Position{-30.0, -30.0},
                           std::vector<Position>{Position{-29.9, -29.9}});
    return ESDFMap(grid_map);
}

// 带一道水平障碍墙的地图（y 固定，x 区间逐格占据）
ESDFMap MakeMapWithWall(double y, double x_from, double x_to) {
    std::vector<Position> cells;
    cells.push_back(Position{-29.9, -29.9});
    for (double x = x_from; x <= x_to; x += 0.125) {
        cells.push_back(Position{x, y});
    }
    const GridMap grid_map(0.125, 480, 480, Position{-30.0, -30.0}, cells);
    return ESDFMap(grid_map);
}

DdpRsShortcutConfig MakeConfig(double cap_ratio) {
    DdpRsShortcutConfig config;
    config.cap_ratio = cap_ratio;
    config.collision_margin = 0.02;
    config.max_length_growth = 1.0;
    config.sample_dist = 0.05;
    return config;
}

// 从当前路径末端沿直线追加点（步长 0.05 m，与 A* 点距一致）
void AppendLine(Path* path, double x_to, double y_to, double theta) {
    const double x_from = path->back().x;
    const double y_from = path->back().y;
    const int count = static_cast<int>(std::max(
        1L, std::lround(std::hypot(x_to - x_from, y_to - y_from) / 0.05)));
    for (int i = 1; i <= count; ++i) {
        path->addPoint({x_from + (x_to - x_from) * i / count,
                        y_from + (y_to - y_from) * i / count, theta});
    }
}

// 沿半径 radius 的圆弧追加点（正 radius = 左转），从当前位姿转过
// sweep 弧度；radius·sweep > 0 对应前进
void AppendArc(Path* path, double radius, double sweep) {
    const double x0 = path->back().x;
    const double y0 = path->back().y;
    const double theta0 = path->back().theta;
    const double cx = x0 - radius * std::sin(theta0);
    const double cy = y0 + radius * std::cos(theta0);
    const int count = static_cast<int>(std::max(
        1L, std::lround(std::abs(sweep * radius) / 0.05)));
    for (int i = 1; i <= count; ++i) {
        const double theta = theta0 + sweep * i / count;
        path->addPoint({cx + radius * std::sin(theta),
                        cy - radius * std::cos(theta), theta});
    }
}

// 冗余绕行场景：沿 +x 前进 8 m → 倒退 4 m 拐上去 → 再前进回到 (12,0)。
// 空旷环境下这条 3 段路径存在一条 1 段的短接（直接沿 +x 开到底），
// 前端却因栅格搜索的离散性给出了折返
Path BuildDetourPath() {
    Path path;
    path.addPoint({0.0, 0.0, 0.0});
    AppendLine(&path, 8.0, 0.0, 0.0);
    AppendLine(&path, 4.0, 4.0, 0.0);
    AppendLine(&path, 12.0, 0.0, 0.0);
    path.finalize();
    return path;
}

// U 形绕行场景：前进直行到 (8,0) → 半径 10 的左转半圆到 (8,20,π)
// → 沿 π 航向前进到 (-8,20)。两条臂相隔 20 m，起终点之间存在一条
// 穿中间空旷区的更短连接——短接几何与原路径几何完全分离，因此
// 可以在不影响原路径合法性的前提下单独封死短接区
Path BuildUTurnPath() {
    Path path;
    path.addPoint({0.0, 0.0, 0.0});
    AppendLine(&path, 8.0, 0.0, 0.0);
    AppendArc(&path, 10.0, M_PI);
    AppendLine(&path, -8.0, 20.0, M_PI);
    path.finalize();
    return path;
}

// 测试空旷环境下的冗余绕行被短接消除：换挡段数下降、总长下降、
// 起终点位姿逐位保持（前后缀不参与替换）
TEST(DdpRsShortcutTest, DetourIsShortcutInFreeSpace) {
    const Path input = BuildDetourPath();
    const ESDFMap map = MakeEmptyMap();
    const VehicleFootprintModel footprint = MakeFootprint();
    const Path output = ShortcutShiftPoints(input, map, footprint, kWheelbase,
                                            kDeltaMax, MakeConfig(0.9));
    EXPECT_LT(output.numManeuvers(), input.numManeuvers());
    EXPECT_LT(output.length(), input.length());
    EXPECT_NEAR(output.front().x, input.front().x, 1e-9);
    EXPECT_NEAR(output.front().y, input.front().y, 1e-9);
    EXPECT_NEAR(output.back().x, input.back().x, 1e-9);
    EXPECT_NEAR(output.back().y, input.back().y, 1e-9);
}

// 测试短接曲线的曲率不超过配置上限：短接的全部意义是给下游一条运动学
// 可行的初值，产出一条超限曲线等于把问题从换挡数转嫁到可行性。
// 用 U 形场景（短接确实含弯道）才能真正检验曲率上限
TEST(DdpRsShortcutTest, ShortcutCurvatureRespectsCap) {
    const Path input = BuildUTurnPath();
    const ESDFMap map = MakeEmptyMap();
    const VehicleFootprintModel footprint = MakeFootprint();
    const double cap_ratio = 0.9;
    const Path output = ShortcutShiftPoints(input, map, footprint, kWheelbase,
                                            kDeltaMax, MakeConfig(cap_ratio));
    ASSERT_LT(output.length(), input.length());
    const double kappa_cap = cap_ratio * std::tan(kDeltaMax) / kWheelbase;
    double max_kappa = 0.0;
    Pose previous = output.front();
    bool first = true;
    output.forEach([&](const TrajectoryPoint& point) {
        if (first) {
            first = false;
            previous = point;
            return;
        }
        const double ds = std::hypot(point.x - previous.x, point.y - previous.y);
        if (ds > 1e-6) {
            const double dtheta =
                std::abs(std::remainder(point.theta - previous.theta, 2 * M_PI));
            max_kappa = std::max(max_kappa, dtheta / ds);
        }
        previous = point;
    });
    // 折线离散化本身带来微小放大，容差取 5%
    EXPECT_LE(max_kappa, kappa_cap * 1.05);
}

// 测试碰撞守卫：在 U 形绕行的中间空旷区横一道墙。墙距两条臂均 10 m、
// 距弯道 6 m，因此原路径仍无碰撞；而空旷环境下会被选中的那条
// 穿中间区短接恰好撞墙。守卫失效时输出将直接携带碰撞，因此这里断言
// 的是真实契约——短接不得引入新碰撞，而不是“不允许存在任何短接”
TEST(DdpRsShortcutTest, CollidingShortcutIsRejected) {
    const Path input = BuildUTurnPath();
    const ESDFMap map = MakeMapWithWall(10.0, -9.0, 12.0);
    const VehicleFootprintModel footprint = MakeFootprint();
    const DdpRsShortcutConfig config = MakeConfig(0.9);
    // 前提校验：原路径本身无碰撞，否则输出侧的断言没有意义
    ASSERT_LE(ComputeMaxCollisionDepth(input, map, footprint),
              config.collision_margin);
    // 参照：空旷环境下确实存在一条更短的穿中间区短接
    const Path free_space_output = ShortcutShiftPoints(
        input, MakeEmptyMap(), footprint, kWheelbase, kDeltaMax, config);
    ASSERT_LT(free_space_output.length(), input.length());
    ASSERT_GT(ComputeMaxCollisionDepth(free_space_output, map, footprint),
              config.collision_margin);
    const Path output = ShortcutShiftPoints(input, map, footprint, kWheelbase,
                                            kDeltaMax, config);
    EXPECT_LE(ComputeMaxCollisionDepth(output, map, footprint),
              config.collision_margin);
}

// 测试长度增长守卫：把增长上限设为 0（禁止任何变长）后，只有确实更短
// 的候选才可能被接受，因此输出长度绝不超过输入长度
TEST(DdpRsShortcutTest, LengthGrowthGuardBoundsOutputLength) {
    const Path input = BuildDetourPath();
    const ESDFMap map = MakeEmptyMap();
    const VehicleFootprintModel footprint = MakeFootprint();
    DdpRsShortcutConfig config = MakeConfig(0.9);
    config.max_length_growth = 0.0;
    const Path output =
        ShortcutShiftPoints(input, map, footprint, kWheelbase, kDeltaMax, config);
    EXPECT_LE(output.length(), input.length() + 1e-9);
}

// 测试关闭时逐位透传：cap_ratio=0 必须让本级完全无副作用，这是
// 「新机制默认关闭且关闭时位一致」的硬要求
TEST(DdpRsShortcutTest, DisabledConfigPassesPathThrough) {
    const Path input = BuildDetourPath();
    const ESDFMap map = MakeEmptyMap();
    const VehicleFootprintModel footprint = MakeFootprint();
    DdpRsShortcutConfig off = MakeConfig(0.0);
    const Path passthrough =
        ShortcutShiftPoints(input, map, footprint, kWheelbase, kDeltaMax, off);
    EXPECT_EQ(passthrough.size(), input.size());
    EXPECT_EQ(passthrough.numManeuvers(), input.numManeuvers());
}

// 测试配置校验：比例越界、负裕度、非正采样间距、非正车辆参数都必须
// 显式抛出，静默降级会让上层以为短接已生效
TEST(DdpRsShortcutTest, InvalidConfigThrows) {
    const Path input = BuildDetourPath();
    const ESDFMap map = MakeEmptyMap();
    const VehicleFootprintModel footprint = MakeFootprint();
    DdpRsShortcutConfig bad_ratio = MakeConfig(1.5);
    EXPECT_THROW(ShortcutShiftPoints(input, map, footprint, kWheelbase,
                                     kDeltaMax, bad_ratio),
                 std::invalid_argument);
    DdpRsShortcutConfig bad_margin = MakeConfig(0.9);
    bad_margin.collision_margin = -0.1;
    EXPECT_THROW(ShortcutShiftPoints(input, map, footprint, kWheelbase,
                                     kDeltaMax, bad_margin),
                 std::invalid_argument);
    DdpRsShortcutConfig bad_sample = MakeConfig(0.9);
    bad_sample.sample_dist = 0.0;
    EXPECT_THROW(ShortcutShiftPoints(input, map, footprint, kWheelbase,
                                     kDeltaMax, bad_sample),
                 std::invalid_argument);
    EXPECT_THROW(ShortcutShiftPoints(input, map, footprint, 0.0, kDeltaMax,
                                     MakeConfig(0.9)),
                 std::invalid_argument);
}

// 测试尊重节点出车方向：输入首段为前进，任何被采纳的 RS 直连都必须
// 以前进起步，输出首段方向恒为前进。触发原因是节点出车方向约束是
// 「换挡节点不得凭空变向」语义的载体
TEST(DdpRsShortcutTest, RespectsNodeDepartureDirection) {
    const Path input = BuildDetourPath();
    const ESDFMap map = MakeEmptyMap();
    const VehicleFootprintModel footprint = MakeFootprint();
    const Path output = ShortcutShiftPoints(input, map, footprint, kWheelbase,
                                            kDeltaMax, MakeConfig(0.9));
    ASSERT_EQ(output.getManeuvers().front().direction, Direction::FORWARD);
    EXPECT_NEAR(output.front().x, input.front().x, 1e-9);
    EXPECT_NEAR(output.front().theta, input.front().theta, 1e-9);
}

// 测试重构保持终点位姿：终链为多段（含尖点）RS 时，回溯重构必须保持
// 连接段内部 maneuver 的顺序——整表 reverse 会把段内顺序一并反转、
// 终点漂移到中间尖点处，下游以短接终点为目标的收敛检查会全部失效。
// 预期行为：输出终点与输入终点逐位一致
// 一并反转、终点漂移到中间尖点处，下游以短接终点为目标的收敛检查
// 会全部失效。预期行为：输出终点与输入终点逐位一致
TEST(DdpRsShortcutTest, ReconstructionPreservesTerminalPose) {
    // 前进 10 m 后倒回 (0,0.5)：唯一优于原路径的直连是以尖点收尾的
    // 前进起步 RS 曲线（纯横向偏移构型下同向前进词族不可行）
    Path input;
    input.addPoint({0.0, 0.0, 0.0});
    AppendLine(&input, 10.0, 0.0, 0.0);
    AppendLine(&input, 0.0, 0.5, 0.0);
    input.finalize();
    ASSERT_EQ(input.numManeuvers(), 2u);
    DdpRsShortcutConfig config = MakeConfig(1.0);
    config.max_length_growth = 1.0;
    const ESDFMap map = MakeEmptyMap();
    const VehicleFootprintModel footprint = MakeFootprint();
    const Path output = ShortcutShiftPoints(input, map, footprint, kWheelbase,
                                            kDeltaMax, config);
    // 前提校验：确实用 RS 直连替换了原路径（长度显著缩短），
    // 否则本用例退化为原样透传、测不到重构顺序
    EXPECT_LT(output.length(), input.length() - 0.5);
    EXPECT_NEAR(output.back().x, input.back().x, 1e-9);
    EXPECT_NEAR(output.back().y, input.back().y, 1e-9);
    EXPECT_NEAR(output.back().theta, input.back().theta, 1e-9);
}

}  // namespace
}  // namespace apa_post_processor
