#pragma once

#include <Eigen/Core>
#include <vector>

#include "../../util/path.h"
#include "alm_maneuver_segmenter.h"
#include "bicycle_kinematics_extractor.h"
#include "minco_trajectory.h"

namespace apa_post_processor {
// 单 Maneuver 融化分类（ALM.md 2.6.3 节硬修剪判据）
enum class AlmMeltClass {
    // 正常机动段（|Δs| 不低于融化弧长阈值）
    NORMAL,
    // 已融化废段（|Δs| 与 |Δθ| 均低于阈值）：物理冗余换挡，压平剔除
    MELTED,
    // 原地掉头式微动段（|Δs| 低于弧长阈值但 |Δθ| 超过阈值）：仅作为信息性
    // 分类标签，采样数据与 NORMAL 段同等对待、不做任何改写；输出中以
    // Direction::PIVOT 方向标签单独保留，不参与合并
    PIVOT,
};
// 机动融化与拓扑修剪配置
struct AlmManeuverMelterConfig {
    // 融化段弧长阈值 |Δs| (m)（ALM.md 2.6.3 节示例量级 0.05 m）
    double melt_arc_threshold = 0.05;
    // PIVOT 判定的朝向变化阈值 |Δθ| (rad)：|Δs| 低于弧长阈值但 |Δθ|
    // 超过此值时为原地掉头式微动，保留为 PIVOT 而非剔除；默认值沿用
    // util::topology_cleaner 的 pivot_delta_threshold 量级（量纲从
    // 前轮转角变化 Δδ 重新标定为 θ-s 参数化下的朝向变化 Δθ）
    double melt_heading_threshold = 0.1;
    // 多项式段离散为 Path 点的每段采样点数（>= 2）
    int samples_per_segment = 16;
};
// 单 Maneuver 的融化判据量与分类结果
struct AlmManeuverMeltInfo {
    // 分类结果
    AlmMeltClass classification = AlmMeltClass::NORMAL;
    // 净弧长位移 |Δs| (m)，θ-s 轨迹精确取值
    double arc_displacement = 0.0;
    // 朝向变化量 |Δθ| (rad)，θ-s 轨迹精确取值（构造上已解缠绕）
    double heading_change = 0.0;
};
// 机动融化与拓扑修剪结果
struct AlmMeltResult {
    // 是否发生了废段剔除
    bool pruned = false;
    // 修剪后的采样路径（经 topology_cleaner 第二遍重建：剔除废段、
    // 同向合并、方向相反段绝不合并）
    Path path;
    // 剔除的废段数
    int removed_count = 0;
    // 保留的 PIVOT 段数（仅分类标签统计；对应段的采样数据不做任何改写）
    int pivot_count = 0;
    // 逐 Maneuver 判据量（与输入估计一一对应）
    std::vector<AlmManeuverMeltInfo> maneuver_infos;
};
// 机动融化与拓扑修剪：ALM 收敛后的拓扑级后处理。融化判据直接取 θ-s
// 参数化下的精确量（Maneuver 首末段端点的 |Δs|/|Δθ|，无需采样近似）；
// 修剪复用 util::topology_cleaner 的两遍分类约定与第二遍重建逻辑
// （UNKNOWN 标记剔除 + 同向合并 + "绝不合并方向相反相邻段"红线），
// 仅按 θ-s 量纲重新标定阈值，不重新实现独立拓扑分类逻辑。
// 首尾段保护：首段决定车辆当前位姿、末段承载 ALM 精确收敛的终点
// x,y,θ 等式约束（ALM.md 2.6.3 节"微调必须保持已收敛的终点等式约束
// 不被扰动"），二者无论判据量如何均只分类为 NORMAL——即使满足融化
// 判据量也只允许参与同向合并，不允许被当作废段剔除，也不允许被重分类
// 为 PIVOT（真实运动方向是首末段的输出语义，PIVOT 标签会丢弃
// FORWARD/BACKWARD 信息并阻断同向合并）。
// PIVOT 段处理：只把 direction 改写为 Direction::PIVOT 标签（单独保留、
// 不参与合并），采样点数据与 NORMAL 段完全同等对待、不做任何改写——
// 连续优化产出的 θ-s 轨迹本身满足 θ̇=v·tanδ/L_base，压平位置/清零速度
// 反而会制造 v≡0 但 θ 变化的自相矛盾状态。
class AlmManeuverMelter {
   public:
    // 构造并校验配置：阈值必须为正有限值、采样数 >= 2，非法抛
    // std::invalid_argument
    explicit AlmManeuverMelter(AlmManeuverMelterConfig config = {});
    // 主入口：融化检测 → 方向标记 → 复用 topology_cleaner 第二遍重建。
    // 输入非法（空估计/空 Maneuver/估计与轨迹段数不一致/非有限起点）抛
    // std::invalid_argument
    AlmMeltResult meltAndPrune(
        const MincoTrajectory& trajectory,
        const std::vector<AlmManeuverEstimate>& estimates,
        const Eigen::Vector2d& start_position,
        const BicycleKinematicsExtractor& kinematics) const;
    // 融化判据（独立可测）：按估计的段结构索引轨迹段，给出逐 Maneuver 的
    // |Δs|/|Δθ| 与分类；输入结构非法抛 std::invalid_argument
    std::vector<AlmManeuverMeltInfo> detectMelting(
        const MincoTrajectory& trajectory,
        const std::vector<AlmManeuverEstimate>& estimates) const;
    // 当前配置（只读）
    const AlmManeuverMelterConfig& config() const { return config_; }

   protected:
    AlmManeuverMelterConfig config_;
};
}  // namespace apa_post_processor
