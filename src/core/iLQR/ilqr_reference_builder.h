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
#include "ilqr_config.h"

namespace apa_post_processor {
// iLQR 优化问题的状态/控制维度：七维状态链 [x, y, θ, v, a, δ, ω] 与二维控制 [j,
// η]
inline constexpr int ILQR_STATE_DIM = 7;
inline constexpr int ILQR_CONTROL_DIM = 2;
// 状态分量布局索引（纯排布约定，与动力学链更新顺序无关）
inline constexpr int ILQR_IDX_X = 0;
inline constexpr int ILQR_IDX_Y = 1;
inline constexpr int ILQR_IDX_THETA = 2;
inline constexpr int ILQR_IDX_V = 3;
inline constexpr int ILQR_IDX_A = 4;
inline constexpr int ILQR_IDX_DELTA = 5;
inline constexpr int ILQR_IDX_OMEGA = 6;
using iLQRState = Eigen::Matrix<double, ILQR_STATE_DIM, 1>;
using iLQRControl = Eigen::Matrix<double, ILQR_CONTROL_DIM, 1>;
// 定长 Eigen 类型的容器必须配套对齐分配器，保证 packet 矢量化路径可用
template <typename T>
using iLQRAlignedVec = std::vector<T, Eigen::aligned_allocator<T>>;
// 角度差分统一归一化到 (-π, π]：初值提取、跟踪代价与终端约束必须使用同一
// wrap 实现，避免跨 ±π 的路径出现 2π 跳变伪影
inline double WrapAngle(double angle) {
    return std::remainder(angle, 2.0 * PI);
}
// 重采样网格上的 maneuver 元数据：cusp 仅用于初值提取、打靶节点布设与
// 跟踪权重排布，不产生任何 v=0 硬边界语义（速度允许全程不变号穿过原换挡点）
struct iLQRReferenceManeuver {
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
struct iLQRReference {
    // 实际重采样间距 (m)（全长归一后可能与标称 sample_dist 略有差异）
    double ds{0.0};
    // 固定离散步长 (s)
    double dt{0.0};
    // N+1 个等弧长参考位姿
    std::vector<Pose> poses;
    // maneuver 元数据序列（与输入 Path 的 maneuver 一一对应）
    std::vector<iLQRReferenceManeuver> maneuvers;
    // 换挡尖点（方向反号边界）的网格索引，升序无重复
    std::vector<std::size_t> cusp_indices;
    // 打靶节点集：{每 n_s 步} ∪ {cusp} ∪ {末点 N}，升序无重复
    std::vector<std::size_t> shooting_nodes;
    // N+1 个状态初值 [x, y, θ, v, a, δ, ω]，已全部裁剪进盒约束
    iLQRAlignedVec<iLQRState> initial_states;
    // N 个控制初值 [j, η]，恒为零向量
    iLQRAlignedVec<iLQRControl> initial_controls;
    // N 个逐步时长 (s)：构建时一律填 dt（均匀网格）。段级时间重分配启用后
    std::vector<double> step_dt;
    // 第 k 步的时长：step_dt 未填充时退化为均匀网格 dt
    double stepDt(std::size_t k) const {
        return k < step_dt.size() ? step_dt[k] : dt;
    }
};
// RS 换挡点短接：以 maneuver 边界为节点做动态规划全局择优，用有界
// 曲率 RS 曲线直连节点对（出车方向受节点 maneuver 方向约束、逐点
// 碰撞校验、长度守卫），起点与终点位姿必须保持（漂移即拒绝）
Path ShortcutShiftPoints(const Path& path, const ESDFMap& esdf_map,
                         const VehicleFootprintModel& footprint_model,
                         double wheelbase, double delta_max,
                         const iLQRConfig& config);

class iLQRReferenceBuilder {
   public:
    // 构造时校验配置与车辆参数：采样间距/步长/盒边界/轴距必须为正
    iLQRReferenceBuilder(const iLQRConfig& config,
                        const VehicleParams& vehicle_params);
    // 消费前端路径，产出阶段一求解所需的全部前端数据；
    // 空路径/单点路径/总长不足一个采样间距时抛出 std::invalid_argument
    iLQRReference build(const Path& path) const;
    // 标称重采样间距（只读）：退化诊断等下游判定需要同一来源
    double sampleDist() const { return config_.reference_sample_dist; }

   protected:
    // 配置
    iLQRConfig config_;
    // 轴距 (m)：δ = atan(L·κ) 反解的唯一车辆参数依赖
    double wheelbase_{0.0};
};
}  // namespace apa_post_processor
