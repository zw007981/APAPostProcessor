#pragma once

#include <Eigen/Core>
#include <Eigen/StdVector>
#include <cmath>
#include <cstddef>
#include <vector>

#include "../../util/constants.h"
#include "../../util/path.h"
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
    // 前轮转角幅值上限 (rad)
    double delta_max{0.55};
    // 前轮转角速度幅值上限 (rad/s)
    double omega_max{0.5};
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
};
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
    DdpReferenceBuilderConfig config_;
    // 轴距 (m)：δ = atan(L·κ) 反解的唯一车辆参数依赖
    double wheelbase_{0.0};
};
}  // namespace apa_post_processor
