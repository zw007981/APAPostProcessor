#pragma once

#include <Eigen/Core>
#include <vector>

#include "../../util/maneuver.h"
#include "minco_maneuver_segmenter.h"
#include "bicycle_kinematics_extractor.h"
#include "minco_trajectory.h"

namespace apa_post_processor {
// "MincoTrajectory → 采样 Maneuver 序列"离散化工具（纯函数，无内部状态，
// 多次调用互不干扰）。把 θ-s 参数化的连续多项式轨迹按 Maneuver 结构离散为
// 采样点序列（x,y,θ,v,a,δ,δ̇,t）：世界坐标由 ṡ·(cosθ,sinθ) 梯形积分逐点还原
// （锚定 start_position，跨 Maneuver 连续累积），v/a/δ/δ̇ 由运动学提取器
// 逐点解析，t 取 θ-s 轨迹的全局时刻（真实时间参数化，使运动学校验对
// 离散化产出直接可用）；采用半开区间采样避免段连接处重复点，全局终点单独
// 补在最后一个 Maneuver。方向直接取自估计，不做任何方向推断/改写，也不做
// 任何融化/剔除判断——同一工具既可用于主优化收敛轨迹（优化后），也可用于
// 预处理粗优化轨迹（优化前），保证两条曲线经同一套离散化管线产出、可直接
// 对比。输入非法（结构不一致/非有限起点/采样数 < 2）抛
// std::invalid_argument。
std::vector<Maneuver> SampleMincoTrajectory(
    const MincoTrajectory& trajectory,
    const std::vector<MincoManeuverEstimate>& estimates,
    const Eigen::Vector2d& start_position,
    const BicycleKinematicsExtractor& kinematics, int samples_per_segment);
// 估计与轨迹的段结构一致性校验（独立可测）：空估计、含空 Maneuver 的估计、
// 估计段数总和与轨迹段数不一致均抛 std::invalid_argument
void CheckMincoSampleStructure(
    const MincoTrajectory& trajectory,
    const std::vector<MincoManeuverEstimate>& estimates);
}  // namespace apa_post_processor
