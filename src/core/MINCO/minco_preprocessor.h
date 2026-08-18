#pragma once

#include <Eigen/Core>
#include <vector>

#include "bicycle_kinematics_extractor.h"
#include "minco_config.h"
#include "minco_maneuver_segmenter.h"
#include "minco_trajectory.h"

namespace apa_post_processor {
// 预处理问题装配数据：由 MincoManeuverSegmenter 的初值估计一次性装配，供
// L-BFGS 代价函数反复求值时复用，避免每次求值重新展开。
struct MincoPreprocessorProblem {
    // 起点边界条件（车辆起步为零速零加速度）
    MincoBoundaryCondition2d start;
    // 终点边界条件（s 的位置分量仅为占位，求值时被决策变量 s_f 覆盖）
    MincoBoundaryCondition2d end;
    // 起点世界坐标（世界坐标还原积分的锚点）
    Eigen::Vector2d start_position{0.0, 0.0};
    // M-1 个内部航点初值 (θ, s)
    std::vector<Eigen::Vector2d> initial_waypoints;
    // M 段时长初值
    std::vector<double> initial_durations;
    // 终点弧长 s_f 初值
    double initial_final_arc_length = 0.0;
    // M 个逐段终点跟踪目标 p_w0（世界系）
    std::vector<Eigen::Vector2d> track_positions;
    // 换挡点对应的多项式段下标（该段末端即换挡点，也是内部航点下标）
    std::vector<int> cusp_segment_indices;
    // 段数 M
    int numSegments() const {
        return static_cast<int>(initial_durations.size());
    }
    // 决策变量维数 3M-1：2(M-1) 个航点分量 + M 个时间变量 + 1 个终点弧长
    int variableCount() const { return 3 * numSegments() - 1; }
    // 打包 L-BFGS 初始猜测：[航点(θ,s)..., τ..., s_f]
    Eigen::VectorXd initialGuess() const;
};
// 预处理粗优化结果
struct MincoPreprocessorResult {
    // 整体成功：优化器正常结束且最大段终点跟踪误差不超过配置容差
    bool success = false;
    // L-BFGS 是否正常跑完（未抛异常；语义与 BSplineSmoother 一致）
    bool optimizer_converged = false;
    // 优化后的 θ-s 多项式轨迹（失败时为空轨迹，numSegments()==0）
    MincoTrajectory trajectory;
    // 优化后的 M-1 个内部航点 (θ, s)
    std::vector<Eigen::Vector2d> waypoints;
    // 优化后的 M 段时长
    std::vector<double> durations;
    // 优化后的终点弧长 s_f
    double final_arc_length = 0.0;
    // 优化后各段末端世界坐标（与代价函数同一组辛普森节点积分）
    std::vector<Eigen::Vector2d> segment_end_positions;
    // 最大段终点跟踪误差 (m)
    double max_endpoint_error = 0.0;
    // 最大换挡点残余速度 |ṡ| (m/s)
    double max_cusp_speed = 0.0;
    // L-BFGS 实际迭代次数
    int lbfgs_iterations = 0;
    // 最终代价值
    double final_cost = 0.0;
};
// 预处理粗优化器：两阶段优化流程的第一阶段。以 MincoManeuverSegmenter 的
// 初值估计为输入，在松收敛阈值下求解 J_pre（不含 ESDF 碰撞惩罚与外层
// MINCO 乘子逻辑），把初值拉近前端路径并满足运动学约束，输出供主优化
// 使用的 MINCO 初值。决策变量布局：[2(M-1) 个内部航点 (θ,s) 分量,
// M 个时间重参数化变量 τ_i, 终点弧长 s_f]，梯度经 K(T)^{-T} 伴随反传
// （内部航点/终点弧长）与 K(T) 行对 T 的解析依赖（时间变量）。
class MincoPreprocessor {
   public:
    // 构造并校验配置：非法配置抛 std::invalid_argument（运动学参数的合法性
    // 由 BicycleKinematicsExtractor 构造时校验）
    explicit MincoPreprocessor(const MincoConfig& config);
    // 对初值估计执行 J_pre 粗优化。输入非法（空估计/空段/非有限字段/非正
    // 时长/非有限起点）抛 std::invalid_argument；优化失败显式返回
    // success=false（含 optimizer_converged=false 或跟踪误差超容差），
    // 不静默返回垃圾结果
    MincoPreprocessorResult preprocess(
        const std::vector<MincoManeuverEstimate>& estimates,
        const Eigen::Vector2d& start_position) const;
    // 当前配置（只读）
    const MincoConfig& config() const { return config_; }

   protected:
    // 辛普森节点数据：世界坐标还原积分在同一组固定节点上的求值结果，
    // 代价函数（梯度反传）与结果指标（段末端位置）共享，保证"先离散后
    // 求导"的节点一致性
    struct SimpsonNodeData {
        // 每段 simpson_subintervals+1 个节点的 θ 值
        std::vector<std::vector<double>> node_theta;
        // 每段 simpson_subintervals+1 个节点的 ṡ 值
        std::vector<std::vector<double>> node_s_dot;
        // 每段的辛普森积分位移（世界系）
        std::vector<Eigen::Vector2d> segment_displacements;
    };
    // 装配问题数据并逐项校验输入合法性
    MincoPreprocessorProblem buildProblem(
        const std::vector<MincoManeuverEstimate>& estimates,
        const Eigen::Vector2d& start_position) const;
    // 求值 J_pre 代价与解析梯度（L-BFGS 函数对象；gradient 必须已分配且
    // 维数等于 problem.variableCount()）
    double evaluateCostAndGradient(const MincoPreprocessorProblem& problem,
                                   const Eigen::VectorXd& x,
                                   Eigen::VectorXd* gradient) const;
    // 由决策变量重建 MINCO 轨迹（维数不匹配抛 std::invalid_argument）
    MincoTrajectory buildTrajectory(const MincoPreprocessorProblem& problem,
                                    const Eigen::VectorXd& x) const;
    // 在代价函数同一组辛普森节点上求值各段 θ/ṡ 并积分各段位移
    SimpsonNodeData computeSimpsonNodeData(
        const MincoTrajectory& trajectory) const;
    // 按代价函数同一组辛普森节点积分各段末端世界坐标
    std::vector<Eigen::Vector2d> computeSegmentEndPositions(
        const MincoPreprocessorProblem& problem,
        const MincoTrajectory& trajectory) const;
    // 辛普森单位权重（不含段时长因子），代价函数与指标计算共享同一组权重
    static std::vector<double> SimpsonUnitWeights(int num_subintervals);

   protected:
    MincoConfig config_;
    BicycleKinematicsExtractor kinematics_;
};
}  // namespace apa_post_processor
