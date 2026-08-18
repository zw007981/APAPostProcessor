#pragma once

#include <Eigen/Core>
#include <vector>

#include "block_tridiagonal_solver.h"

namespace apa_post_processor {
// 单维边界条件：位置/速度/加速度（对应 h=3 时的 0~2 阶导数）
struct MincoBoundaryCondition {
    double pos{0.0};
    double vel{0.0};
    double acc{0.0};
};
// θ-s 运动状态空间的二维边界条件（theta 与 s 各一组）
struct MincoBoundaryCondition2d {
    MincoBoundaryCondition theta;
    MincoBoundaryCondition s;
};
// MINCO 分段多项式轨迹：θ(t) 与 s(t) 各为 M 段 5 阶多项式（每段 6 个归一化
// 系数）。本类封装 K(T)c=b 的装配与块三对角求解、按任意时刻求值及各阶实
// 时间导数、终点弧长 s_f 的 K(T)^{-T} 伴随梯度、以及时间重参数化
// τ↔T 分段光滑双射。
// 装配约定：每段采用局部归一化时间 tau=t/T_i∈[0,1] 构造基函数，K(T) 的行
// 按块 6 行一组组织——块行 0 为起点位置/速度/加速度 + 首段末端位置/1/2 阶
// 导数连续性；中间块行 i 为与上一段的 3/4 阶导数连续性 + 本段两端位置 +
// 本段末端 1/2 阶导数连续性；末块行补上终点位置/速度/加速度。该排列使
// 全部主对角块非奇异，块 Thomas 消元无需选主元。内部航点 d_m 在右端项 b
// 中恰出现两次（相邻两段端点位置各一次），终点位置（如 s_f）固定位于 b
// 的第 6M-3（0 基）个位置，s_f 的伴随梯度反传据此定位该下标。
class MincoTrajectory {
   public:
    // 每段多项式系数个数（5 阶多项式）
    static constexpr int COEFFS_PER_SEG = 6;
    // 块矩阵：6 行，第 i 列为第 i 段系数/右端项
    using CoeffMatrix = Eigen::Matrix<double, COEFFS_PER_SEG, Eigen::Dynamic>;
    MincoTrajectory() = default;
    // 由边界条件、内部航点与段时长构建全部多项式系数。waypoints 含 M-1 个
    // 内部航点（x 分量为 θ、y 分量为 s）；durations 含 M 段时长，必须全部
    // 为正有限值。输入不合法抛 std::invalid_argument；K(T) 奇异抛
    // std::runtime_error。
    void setTrajectory(const MincoBoundaryCondition2d& start,
                       const MincoBoundaryCondition2d& end,
                       const std::vector<Eigen::Vector2d>& waypoints,
                       const std::vector<double>& durations);
    // 求值全局时刻 t 处 θ/s 的 order 阶实时间导数，返回 (θ导数, s导数)。
    // t 允许在 [0, totalDuration] 端点处有数值级微小越界（内部截断），明显
    // 越界抛 std::out_of_range；order 超出 [0, 5] 抛 std::invalid_argument。
    Eigen::Vector2d evaluate(double t, int order) const;
    // 求值指定段内局部时刻（实时间，应在 [0, T_i] 内，允许数值级微小越界）
    // 处 θ/s 的 order 阶实时间导数
    Eigen::Vector2d evaluateSegment(int segment_index, double local_time,
                                    int order) const;
    // 段数 M；未调用 setTrajectory 时为 0
    int numSegments() const { return static_cast<int>(durations_.size()); }
    // 第 i 段时长；越界抛 std::out_of_range
    double duration(int segment_index) const;
    // 轨迹总时长
    double totalDuration() const;
    // θ/s 全部段的归一化系数（6xM，第 i 列为第 i 段系数，tau∈[0,1] 基）
    const CoeffMatrix& coeffsTheta() const { return coeffs_theta_; }
    const CoeffMatrix& coeffsS() const { return coeffs_s_; }
    // 终点弧长 s_f 的伴随梯度：给定 ∂J/∂c_s（与 coeffsS 同形的 6xM 矩阵），
    // 返回 ∂J/∂s_f = (K(T)^{-T} ∂J/∂c_s) 的第 6M-3（0 基）个分量；复用
    // K(T) 的块 LU 消元结果，不重新分解
    double finalArcLengthAdjointGradient(const CoeffMatrix& dJ_over_dcs) const;
    // 通用伴随求解 adj = K(T)^{-T} rhs，供后续优化模块计算 b 内任意变量的
    // 梯度（如内部航点、终点边界）
    CoeffMatrix solveAdjoint(const CoeffMatrix& rhs) const;
    // 归一化时刻 tau_norm∈[0,1] 处 order 阶实时间导数的基函数行（含
    // 1/T^order 缩放）。纯函数、不触碰内部状态；公开供优化模块装配代价对
    // 多项式系数的梯度复用，避免下游重复实现同一公式（与 BSplineSmoother
    // 公开 buildKnotVector/computeBasisAtU 同理）
    static Eigen::Matrix<double, 1, COEFFS_PER_SEG> DerivativeBasisRow(
        double tau_norm, int order, double duration);
    // 时间重参数化分段光滑双射 T=T(τ)（T=1 处一阶连续可导）
    static double TauToDuration(double tau);
    // 双射逆映射 τ=τ(T)，要求 T>0，否则抛 std::invalid_argument
    static double DurationToTau(double duration);
    // dT/dτ 解析导数
    static double TauToDurationDerivative(double tau);
    // dτ/dT 解析导数，要求 T>0，否则抛 std::invalid_argument
    static double DurationToTauDerivative(double duration);

   protected:
    // 装配 K(T) 的下/主/上三条块对角线（输出参数先清空再填充）
    static void AssembleK(const std::vector<double>& durations,
                          std::vector<BlockTridiagonalSolver::Block>& lower,
                          std::vector<BlockTridiagonalSolver::Block>& diagonal,
                          std::vector<BlockTridiagonalSolver::Block>& upper);
    // 装配单维右端项 b（6xM）；dim 取 0 表示 θ 分量、1 表示 s 分量
    static CoeffMatrix AssembleRhs(
        const MincoBoundaryCondition& start, const MincoBoundaryCondition& end,
        const std::vector<Eigen::Vector2d>& waypoints, int dim);
    // 定位全局时刻 t 所属段索引；t 明显越界抛 std::out_of_range
    int locateSegment(double t) const;
    // 校验已初始化且求导阶数在 [0, 5] 内
    void checkEvaluable(int order) const;

   protected:
    // M 段时长
    std::vector<double> durations_;
    // M+1 个累计时刻（首元素恒为 0），供按全局时刻定位段
    std::vector<double> cumulative_durations_;
    // 6xM，θ 各段归一化系数
    CoeffMatrix coeffs_theta_;
    // 6xM，s 各段归一化系数
    CoeffMatrix coeffs_s_;
    // 保留 K(T) 的块 LU 消元结果，供 s_f 伴随梯度与通用伴随求解复用
    BlockTridiagonalSolver solver_;
};
}  // namespace apa_post_processor
