#pragma once

#include <Eigen/Core>
#include <Eigen/StdVector>
#include <cmath>
#include <cstddef>
#include <string>
#include <vector>

#include "../../spatial/esdf_map.h"
#include "../../util/constants.h"
#include "../../util/path.h"
#include "../../util/reeds_shepp.h"
#include "../../vehicle/vehicle_footprint_model.h"
#include "../../vehicle/vehicle_params.h"

namespace apa_post_processor {
// DDP 优化问题的状态/控制维度：七维状态链 [x, y, θ, v, a, δ, ω] 与二维控制 [j,
// η]
inline constexpr int DDP_STATE_DIM = 7;
inline constexpr int DDP_CONTROL_DIM = 2;
// 状态分量布局索引（纯排布约定，与动力学链更新顺序无关）
inline constexpr int DDP_IDX_X = 0;
inline constexpr int DDP_IDX_Y = 1;
inline constexpr int DDP_IDX_THETA = 2;
inline constexpr int DDP_IDX_V = 3;
inline constexpr int DDP_IDX_A = 4;
inline constexpr int DDP_IDX_DELTA = 5;
inline constexpr int DDP_IDX_OMEGA = 6;
using DdpState = Eigen::Matrix<double, DDP_STATE_DIM, 1>;
using DdpControl = Eigen::Matrix<double, DDP_CONTROL_DIM, 1>;
// 定长 Eigen 类型的容器必须配套对齐分配器，保证 packet 矢量化路径可用
template <typename T>
using DdpAlignedVec = std::vector<T, Eigen::aligned_allocator<T>>;
// 角度差分统一归一化到 (-π, π]：初值提取、跟踪代价与终端约束必须使用同一
// wrap 实现，避免跨 ±π 的路径出现 2π 跳变伪影
inline double WrapAngle(double angle) {
    return std::remainder(angle, 2.0 * PI);
}
// 参考轨迹构建配置：盒约束边界作为配置传入，全部初值统一裁剪进盒
struct DdpReferenceBuilderConfig {
    // 等弧长重采样标称间距 (m)：总长不整除时按全长归一，保证间距均匀且覆盖终点
    double sample_dist{DELTA_DIST};
    // 固定离散步长 (s)：dt = 标称间距 / 名义车速，时间段不再是优化变量
    double dt{0.1};
    // 打靶节点规则间隔 n_s（步）：打靶集 = {每 n_s 步} ∪ {cusp} ∪ {末点}
    std::size_t shooting_interval{25};
    // 纵向速度幅值上限 (m/s)，对应平方形态状态约束的边界
    double v_max{1.5};
    // 纵向加速度幅值上限 (m/s²)
    double a_max{1.0};
    // 前轮转角幅值上限 (rad)：默认值取车队车辆物理参数真值
    double delta_max{0.47728};
    // 前轮转角速度幅值上限 (rad/s)：默认值取车辆真值 max_steer_rate=0.4
    double omega_max{0.4};
};
// 重采样网格上的 maneuver 元数据：cusp 仅用于初值提取、打靶节点布设与
// 跟踪权重排布，不产生任何 v=0 硬边界语义（速度允许全程不变号穿过原换挡点）
struct DdpReferenceManeuver {
    // 运动方向符号：+1 前进 / -1 后退 / 0 原地转向或方向未知
    int sign{0};
    // 位移弧长（无符号幅值）(m)
    double delta_s{0.0};
    // 朝向变化（已 wrap 到 (-π, π]）(rad)
    double delta_theta{0.0};
    // 在重采样网格上的起/止点索引（含端点；相邻 maneuver 共享边界点索引）
    std::size_t begin_index{0};
    // 终止索引
    std::size_t end_index{0};
};
// 阶段一求解所需的全部前端数据：参考位姿序列、初值序列、cusp/maneuver
// 元数据与打靶节点集，供内层求解器与外层 AL 循环直接消费
struct DdpReference {
    // 实际重采样间距 (m)（全长归一后可能与标称 sample_dist 略有差异）
    double ds{0.0};
    // 固定离散步长 (s)
    double dt{0.0};
    // N+1 个等弧长参考位姿
    std::vector<Pose> poses;
    // maneuver 元数据序列（与输入 Path 的 maneuver 一一对应）
    std::vector<DdpReferenceManeuver> maneuvers;
    // 换挡尖点（方向反号边界）的网格索引，升序无重复
    std::vector<std::size_t> cusp_indices;
    // 打靶节点集：{每 n_s 步} ∪ {cusp} ∪ {末点 N}，升序无重复
    std::vector<std::size_t> shooting_nodes;
    // N+1 个状态初值 [x, y, θ, v, a, δ, ω]，已全部裁剪进盒约束
    DdpAlignedVec<DdpState> initial_states;
    // N 个控制初值 [j, η]，恒为零向量
    DdpAlignedVec<DdpControl> initial_controls;
    // N 个逐步时长 (s)：构建时一律填 dt（均匀网格）。段级时间重分配启用后
    std::vector<double> step_dt;
    // 第 k 步的时长：step_dt 未填充时退化为均匀网格 dt
    double stepDt(std::size_t k) const {
        return k < step_dt.size() ? step_dt[k] : dt;
    }
};
// RS 换挡点短接配置：允许换挡位姿移动，用有界曲率曲线重连——
// 唯一能突破「换挡点由前端钉死」的几何前处理。编排为以 maneuver
// 边界为节点的动态规划全局择优（2026-08-13 起为唯一实现，贪心
// 逐轮扫描编排已删除）
struct DdpRsShortcutConfig {
    // 曲率上限比（0=关闭）
    double cap_ratio{0.0};
    // ESDF 侵入裕度 + 圆心必须在地图内
    double collision_margin{0.02};
    // 长度增长上限比（以原始输入长度为基准）
    double max_length_growth{0.05};
    // RS 曲线离散采样间距 (m)：碰撞校验与输出重采样共用
    double sample_dist{0.05};
    // DP 代价：每个 maneuver 的固定段价（段数惩罚，越大越倾向少换挡）
    double segment_fixed_cost{8.88};
    // DP 代价：短段惩罚权重（沿用外部混合 A* 参考实现的定价口径）
    double short_segment_weight{1.0};
    // DP 代价：短段判定阈值 (m)，低于该长度按比例加惩罚
    double short_segment_length{2.5};
    // RS 求解逐次耗时记录文件（CSV）：非空时每次 RS 计算追加一行，
    // 供实验对比取证；空 = 关闭（默认，零副作用）
    std::string rs_timing_csv{};
    // RS 耗时记录的分组标签（如数据集名）：随 CSV 行写入，便于回读
    std::string rs_timing_tag{};
};
// RS 换挡点短接：以 maneuver 边界为节点做动态规划全局择优，用有界
// 曲率 RS 曲线直连节点对（出车方向受节点 maneuver 方向约束、逐点
// 碰撞校验、长度守卫），起点与终点位姿必须保持（漂移即拒绝）
Path ShortcutShiftPoints(const Path& path, const ESDFMap& esdf_map,
                         const VehicleFootprintModel& footprint_model,
                         double wheelbase, double delta_max,
                         const DdpRsShortcutConfig& config);

class DdpReferenceBuilder {
   public:
    // 构造时校验配置与车辆参数：采样间距/步长/盒边界/轴距必须为正
    DdpReferenceBuilder(DdpReferenceBuilderConfig config,
                        const VehicleParams& vehicle_params);
    // 消费前端路径，产出阶段一求解所需的全部前端数据；
    // 空路径/单点路径/总长不足一个采样间距时抛出 std::invalid_argument
    DdpReference build(const Path& path) const;
    // 标称重采样间距（只读）：退化诊断等下游判定需要同一来源
    double sampleDist() const { return config_.sample_dist; }

   protected:
    // 配置
    DdpReferenceBuilderConfig config_;
    // 轴距 (m)：δ = atan(L·κ) 反解的唯一车辆参数依赖
    double wheelbase_{0.0};
};
}  // namespace apa_post_processor
