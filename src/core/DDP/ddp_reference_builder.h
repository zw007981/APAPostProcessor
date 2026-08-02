#pragma once

#include <Eigen/Core>
#include <Eigen/StdVector>
#include <cmath>
#include <cstddef>
#include <vector>

#include "../../spatial/esdf_map.h"
#include "../../util/constants.h"
#include "../../util/path.h"
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
    // （max_steer_angle=0.47728 rad，对应 κ_max=tanδ_max/L≈0.1724 /m）；
    // 任何更大的配置值都会在编排入口被车辆参数钳回（只准收紧不准放宽）
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
// cusp 几何预剪枝配置：在构建 DDP 参考之前，对混合 A* 的 maneuver 序列
// 做一次纯几何的冗余折返剔除。cusp 集决定打靶节点布设、跟踪权重排布与
// 阶段二接缝门控——「从更少的 cusp 出发」把「跨越离散拓扑变化」的融化
// 负担从连续优化器肩上卸下来（实测：阶段一内部融化在低临界比段上已被
// 滞回吸收，进一步融化受收敛收束与锚点冲突限制）
struct DdpCuspPruneConfig {
    // 冗余折返的弧长阈值 (m)：位移低于该值的内部 maneuver 才进入候选
    // （0 = 关闭，输出与输入逐位一致）
    double max_prune_arc{0.0};
    // 冗余判据系数 α：折返终点 B 距「此前已覆盖路径」最近点 Q 的距离
    // |QB| <= α·Δs 才判冗余——B 落在未探索的新区域即为真实位移，绝不剔除
    double overlap_ratio{1.0};
    // 缝合桥 ESDF 侵入上限 (m)：外圆半径 − ESDF 距离超过该值即回滚该次
    // 剔除（同伦类保护：被剪的段可能是绕障必需）
    double collision_margin{0.02};
};
// 对输入路径做换挡几何预剪枝：逐次把「终点落在已覆盖区域内的冗余折返」
// 整段剔除，并把路径从折返起点附近的已覆盖点 Q 直接缝合到折返终点 B
// （三重覆盖区消失，maneuver 数净减 2/次）。每次剔除前做三道守卫：
// 重叠判据（|QB| ≤ α·Δs）、航向一致性（缝合桥方向与两端航向轴的夹角
// ≈0 或 ≈π，横向桥接对自行车模型不可达）、ESDF 侵入检查（超限即回滚
// 本次剔除）。首/末 maneuver 无论判据量如何均不参与（承载起点状态与
// 终点语义）。配置非法抛 std::invalid_argument
Path PruneRedundantCusps(const Path& path, const ESDFMap& esdf_map,
                         const VehicleFootprintModel& footprint_model,
                         const DdpCuspPruneConfig& config);

// 参考保形曲率投影配置：把前端 A* 参考中隐含曲率超过给定上限的弧段，
// 在保持全部点位、端点航向、换挡点与同伦类的前提下压回可行域（输入
// 超限 2~9% 会使 AL 幅值约束从首轮即激活、求解器一开始就贴着约束边界
// 找空间——这是阶段一病态与阶段二碰撞的上游原因之一）。
// 与已证伪的 V 形折点平滑（L2.1，针对误诊断的「伪影」）不同源：本项
// 针对的是已实测确证的全局性超限，目标是让参考本身进入可行域
struct DdpCurvatureProjectionConfig {
    // 曲率上限相对车辆物理上限的比例（0 = 关闭）：投影目标
    // κ_proj = ratio · tan(δ_max) / L；取 <1（如 0.95）给求解器在 δ 边界
    // 留出平衡余量——参考恰好贴限时 AL/跟踪的边界争斗无求解余量
    double cap_ratio{0.0};
};
// 对输入路径做保形曲率投影：以段曲率 κ_i = wrap(Δθ)/ds（与参考构建器
// δ = atan(L·κ) 反解同一 θ 差分口径）检测超限段，逐 maneuver 执行
// 「钳制 + 航向守恒再分配」——超限段的 κ 钳到 ±κ_cap，被钳掉的带符号
// 航向均匀摊到本 maneuver 的未钳段上（摊派后任何段 |κ| 仍 ≤ κ_cap，
// 否则该 maneuver 整段回滚保持原样）。**只改 θ 不改位置**：端点、
// 换挡点位置与同伦类自动保持（位移零变化，无需 ESDF 回滚守卫——这正是
// 本方案相对位置重构的更安全之处）；每 maneuver 的总航向变化严格守恒，
// maneuver 端点航向逐位不变。整段超限（无可摊容量，如全段贴限的参考）
// 无法投影——此时上游无几何裕度可言，保持原样由下游机制处理。
// 配置非法抛 std::invalid_argument；ratio=0 或无超限段时逐位透传
Path ProjectReferenceCurvature(const Path& path, double wheelbase,
                               double delta_max,
                               const DdpCurvatureProjectionConfig& config);

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
