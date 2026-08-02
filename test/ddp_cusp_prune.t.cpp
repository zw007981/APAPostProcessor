// cusp 几何预剪枝（PruneRedundantCusps）单元测试：冗余折返的识别、
// 重连缝合的 ESDF 安全检查与回滚、首/末段保护、配置校验。
// 折返冗余的物理语义：「前进到 A → 倒回 B → 继续前进」中，若 B 落在
// 此前已覆盖的路径区域内（|QB| ≤ α·Δs，Q 为前缀上距 B 最近的点），
// 则 A→B 这段折返不贡献任何新探索——剔除后路径从 Q 直接缝合到 B，
// 三重覆盖区消失、maneuver 数净减 2。
#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "core/DDP/ddp_reference_builder.h"
#include "spatial/esdf_map.h"
#include "spatial/grid_map.h"
#include "vehicle/vehicle_footprint_model.h"
#include "vehicle/vehicle_params.h"

namespace apa_post_processor {
namespace {

// 测试用车辆参数与外圆模型（与 DDP 数据集同源：轴距 3.0 m）
VehicleParams MakeVehicleParams() { return VehicleParams{4.9, 1.9, 3.0, 0.48}; }

VehicleFootprintModel MakeFootprint() {
    return VehicleFootprintModel(MakeVehicleParams(), 233, 2, 1);
}

// 空旷地图（16m×16m、分辨率 0.125，仅远角一个占据点保证距离场良
// 定义；地图必须比测试路径大出一个车身以上——外圆查询越界会按保守
// 侵入处理（getDist 越界返回 0），地图太小会让安全误判为不安全
ESDFMap MakeEmptyMap() {
    const GridMap grid_map(0.125, 128, 128, Position{-8.0, -8.0},
                           std::vector<Position>{Position{-7.9, -7.9}});
    return ESDFMap(grid_map);
}

// 带单个占据点的地图（占据点即障碍物中心）
ESDFMap MakeMapWithObstacle(const Position& obstacle) {
    const GridMap grid_map(0.125, 128, 128, Position{-8.0, -8.0},
                           std::vector<Position>{obstacle});
    return ESDFMap(grid_map);
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

// 冗余折返场景：前进 2.0 m（0→2）→ 倒退 0.4 m（2→1.6）→ 前进 2.0 m
// （1.6→3.6），全程 θ=0。折返终点 (1.6,0) 恰好落在首段已覆盖区域内
// （|QB|=0），剔除并重连后应为单个前进 maneuver、总长 3.6 m
Path BuildRedundantYankPath() {
    Path path;
    path.addPoint({0.0, 0.0, 0.0});
    AppendLine(&path, 2.0, 0.0, 0.0);
    AppendLine(&path, 1.6, 0.0, 0.0);
    AppendLine(&path, 3.6, 0.0, 0.0);
    path.finalize();
    return path;
}

// 测试冗余折返被剔除并缝合：三段 maneuver 合并为一段、总长按三重覆盖
// 区净减、端点位姿不变
TEST(DdpCuspPruneTest, RedundantYankIsPrunedAndRestitched) {
    const ESDFMap esdf_map = MakeEmptyMap();
    const auto footprint = MakeFootprint();
    DdpCuspPruneConfig config;
    config.max_prune_arc = 1.0;
    const Path pruned = PruneRedundantCusps(BuildRedundantYankPath(), esdf_map,
                                            footprint, config);
    EXPECT_EQ(pruned.numManeuvers(), 1U);
    EXPECT_NEAR(pruned.length(), 3.6, 0.15);
    // 端点位姿保持（首/末段受保护，终点语义不变）
    EXPECT_NEAR(pruned.back().x, 3.6, 0.1);
    EXPECT_NEAR(pruned.back().theta, 0.0, 1e-6);
}

// 测试非冗余 maneuver 保留：折返终点落在未覆盖的新区域（横向偏移的
// 真实位移）时，缝合桥与航向轴垂直（自行车模型不可达），航向一致性
// 守卫必须拒绝剔除
TEST(DdpCuspPruneTest, LateralShiftManeuverKeptByHeadingGuard) {
    Path path;
    path.addPoint({0.0, 0.0, 0.0});
    AppendLine(&path, 2.0, 0.0, 0.0);
    // 倒退段斜向移到 (1.5, -0.3)（横向偏移 0.3 m 的真实位移）
    AppendLine(&path, 1.5, -0.3, 0.0);
    AppendLine(&path, 3.5, -0.3, 0.0);
    path.finalize();
    ASSERT_EQ(path.numManeuvers(), 3U);
    const ESDFMap esdf_map = MakeEmptyMap();
    const auto footprint = MakeFootprint();
    DdpCuspPruneConfig config;
    config.max_prune_arc = 1.0;
    const Path pruned = PruneRedundantCusps(path, esdf_map, footprint, config);
    // 缝合桥 (1.5,0)→(1.5,-0.3) 与航向（θ=0）垂直，守卫拒绝剔除
    EXPECT_EQ(pruned.numManeuvers(), 3U);
}

// 测试 ESDF 安全检查回滚：缝合桥/接缝点穿过障碍物时该次剔除必须回滚
// （同伦类保护），其余安全检查通过的剔除照常生效
TEST(DdpCuspPruneTest, ObstacleOnStitchRollsBackPrune) {
    // 障碍物恰好压在缝合点 (1.6, 0) 旁 0.5 m 处——外圆包络必然侵入
    const ESDFMap esdf_map = MakeMapWithObstacle(Position{1.6, 0.5});
    const auto footprint = MakeFootprint();
    DdpCuspPruneConfig config;
    config.max_prune_arc = 1.0;
    const Path path = BuildRedundantYankPath();
    const Path pruned = PruneRedundantCusps(path, esdf_map, footprint, config);
    EXPECT_EQ(pruned.numManeuvers(), 3U) << "障碍压缝合点时必须回滚剔除";
    // 对照：障碍物远离缝合区（(1.6, 5)）时剔除生效
    const ESDFMap clear_map = MakeMapWithObstacle(Position{1.6, 5.0});
    const Path pruned_clear =
        PruneRedundantCusps(path, clear_map, footprint, config);
    EXPECT_EQ(pruned_clear.numManeuvers(), 1U);
}

// 测试首/末段保护：首段与末段无论多短都不参与剔除（它们承载起点状态
// 与终点语义）；阈值 0 = 关闭（输出与输入逐位一致）
TEST(DdpCuspPruneTest, FirstAndLastManeuversProtected) {
    Path path;
    path.addPoint({0.0, 0.0, 0.0});
    AppendLine(&path, 0.4, 0.0, 0.0);   // 首段仅 0.4 m
    AppendLine(&path, -1.6, 0.0, 0.0);  // 中段 2.0 m 倒退（超阈）
    AppendLine(&path, -1.2, 0.0, 0.0);  // 末段仅 0.4 m
    path.finalize();
    ASSERT_EQ(path.numManeuvers(), 3U);
    const ESDFMap esdf_map = MakeEmptyMap();
    const auto footprint = MakeFootprint();
    DdpCuspPruneConfig config;
    config.max_prune_arc = 1.0;
    const Path pruned = PruneRedundantCusps(path, esdf_map, footprint, config);
    // 中段 2.0 m 超阈值，首末段受保护：无剔除
    EXPECT_EQ(pruned.numManeuvers(), 3U);
    // 阈值 0 = 关闭
    DdpCuspPruneConfig off;
    const Path identical =
        PruneRedundantCusps(BuildRedundantYankPath(), esdf_map, footprint, off);
    EXPECT_EQ(identical.numManeuvers(), 3U);
    EXPECT_DOUBLE_EQ(identical.length(), BuildRedundantYankPath().length());
}

// 测试多个交替折返的迭代剔除：data1 式「+1.0 / ∓0.3 微动 ×4 / +1.0」
// 序列中全部冗余微段应被逐次剔除（每次剔除后重扫），最终合并为单段
TEST(DdpCuspPruneTest, MultipleAlternatingYanksPrunedIteratively) {
    Path path;
    path.addPoint({0.0, 0.0, 0.0});
    AppendLine(&path, 1.0, 0.0, 0.0);
    AppendLine(&path, 0.7, 0.0, 0.0);
    AppendLine(&path, 1.0, 0.0, 0.0);
    AppendLine(&path, 0.7, 0.0, 0.0);
    AppendLine(&path, 1.0, 0.0, 0.0);
    AppendLine(&path, 0.7, 0.0, 0.0);
    AppendLine(&path, 2.0, 0.0, 0.0);
    path.finalize();
    ASSERT_EQ(path.numManeuvers(), 7U);
    const ESDFMap esdf_map = MakeEmptyMap();
    const auto footprint = MakeFootprint();
    DdpCuspPruneConfig config;
    config.max_prune_arc = 0.8;
    const Path pruned = PruneRedundantCusps(path, esdf_map, footprint, config);
    EXPECT_EQ(pruned.numManeuvers(), 1U);
    EXPECT_NEAR(pruned.length(), 2.0, 0.15);
}

// 测试非法配置显式拒绝：负阈值/负重叠系数/负碰撞裕度都会让剪枝行为
// 不可预期，必须构造期抛出
TEST(DdpCuspPruneTest, InvalidConfigThrows) {
    const ESDFMap esdf_map = MakeEmptyMap();
    const auto footprint = MakeFootprint();
    const Path path = BuildRedundantYankPath();
    DdpCuspPruneConfig config;
    config.max_prune_arc = -1.0;
    EXPECT_THROW(PruneRedundantCusps(path, esdf_map, footprint, config),
                 std::invalid_argument);
    config = DdpCuspPruneConfig{};
    config.overlap_ratio = 0.0;
    EXPECT_THROW(PruneRedundantCusps(path, esdf_map, footprint, config),
                 std::invalid_argument);
    config = DdpCuspPruneConfig{};
    config.collision_margin = -0.01;
    EXPECT_THROW(PruneRedundantCusps(path, esdf_map, footprint, config),
                 std::invalid_argument);
}

}  // namespace
}  // namespace apa_post_processor
