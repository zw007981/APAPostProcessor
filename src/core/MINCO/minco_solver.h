#pragma once

#include <Eigen/Core>
#include <vector>

#include "bicycle_kinematics_extractor.h"
#include "minco_config.h"
#include "minco_esdf_penalty.h"
#include "minco_maneuver_segmenter.h"
#include "minco_preprocessor.h"
#include "minco_trajectory.h"

namespace apa_post_processor {
// PHR-ALM 主求解器外层迭代状态
enum class MincoSolverStatus {
    // 终点位置/朝向双指标同时满足收敛判据
    CONVERGED,
    // 达到最大外层迭代次数仍未满足双指标判据（实时系统兜底退出条件）
    MAX_OUTER_ITERATIONS,
    // 内层 L-BFGS 求解抛出异常（返回最后一次成功迭代的轨迹，显式标记失败）
    INNER_LBFGS_FAILED,
    // 由内层解重建 MINCO 轨迹失败（K(T) 奇异等极端数值情况）
    TRAJECTORY_BUILD_FAILED,
};
// 主优化问题装配数据：由初值估计（边界条件/目标位姿/换挡点）与预处理结果
// （内层 L-BFGS 初值：航点/时长/终点弧长）一次性装配，供代价函数反复求值
struct MincoSolverProblem {
    // 起点边界条件（车辆起步为零速零加速度）
    MincoBoundaryCondition2d start;
    // 终点边界条件（θ 取目标朝向、硬边界精确满足；s 的位置分量仅为占位，
    // 求值时被决策变量 s_f 覆盖）
    MincoBoundaryCondition2d end;
    // 起点世界坐标（世界坐标还原积分的锚点）
    Eigen::Vector2d start_position{0.0, 0.0};
    // 终点目标位姿（MINCO 终端项的 x_target/y_target/θ_target）
    Eigen::Vector2d target_position{0.0, 0.0};
    double target_theta = 0.0;
    // M-1 个内部航点初值 (θ, s)，来自预处理结果
    std::vector<Eigen::Vector2d> initial_waypoints;
    // M 段时长初值，来自预处理结果
    std::vector<double> initial_durations;
    // 终点弧长 s_f 初值，来自预处理结果
    double initial_final_arc_length = 0.0;
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
// 内层目标函数依赖的 PHR-ALM 乘子状态（外层每轮更新后注入代价函数）
struct MincoMultiplierState {
    // 终点 x 向乘子 λ_x
    double lambda_x = 0.0;
    // 终点 y 向乘子 λ_y
    double lambda_y = 0.0;
    // 惩罚权重 ρ
    double rho = 0.0;
};
// 内层代价分解：J_s' 与 PHR-ALM 终端项分离（ρ^0 标定只依赖 J_s'）
struct MincoCostBreakdown {
    // 总代价 J_ρ = J_s' + minco_terminal
    double total = 0.0;
    // 基础平滑目标 J_0 与全部不等式软约束惩罚之和（不含 MINCO 终端项）
    double j_s_prime = 0.0;
    // PHR-ALM 增广拉格朗日终端项
    double minco_terminal = 0.0;
    // ESDF 双重安全惩罚分项（J_s' 的组成部分，单独透出供诊断）
    double esdf_penalty = 0.0;
};
// 单轮外层迭代的完整记录（供收敛判定回溯与更新公式单步验证）
struct MincoOuterIterationRecord {
    // 外层迭代序号（0 基）
    int outer_index = 0;
    // 本轮内层求解实际使用的乘子/惩罚权重（更新前的快照）
    double lambda_x = 0.0;
    double lambda_y = 0.0;
    double rho = 0.0;
    // 带符号终点误差 C_f = (x̃_f - x_target, ỹ_f - y_target)
    Eigen::Vector2d terminal_violation{0.0, 0.0};
    // 终点位置误差 e_pos (m)
    double position_error = 0.0;
    // 终点朝向误差 e_heading (deg)
    double heading_error_deg = 0.0;
    // 本轮解的 J_s'（ρ^0 标定的分子）
    double j_s_prime = 0.0;
    // 本轮解的 MINCO 终端项
    double minco_terminal = 0.0;
    // 本轮解的 ESDF 惩罚分项
    double esdf_penalty = 0.0;
    // 本轮解的总代价 J_ρ
    double total_cost = 0.0;
    // 本轮内层 L-BFGS 实际迭代次数
    int lbfgs_iterations = 0;
};
// PHR-ALM 主优化结果
struct MincoSolverResult {
    // 整体成功：双指标收敛判据满足（status == CONVERGED）
    bool success = false;
    // 外层迭代终止状态（失败路径的显式反馈，不静默返回垃圾结果）
    MincoSolverStatus status = MincoSolverStatus::MAX_OUTER_ITERATIONS;
    // 最终 θ-s 多项式轨迹（终态重建失败时为空轨迹）
    MincoTrajectory trajectory;
    // 最终 M-1 个内部航点 (θ, s)
    std::vector<Eigen::Vector2d> waypoints;
    // 最终 M 段时长
    std::vector<double> durations;
    // 最终终点弧长 s_f
    double final_arc_length = 0.0;
    // 最终终点位置误差 e_pos (m)
    double terminal_position_error = 0.0;
    // 最终终点朝向误差 e_heading (deg)
    double terminal_heading_error_deg = 0.0;
    // 最终一轮内层求解实际使用的乘子/惩罚权重
    double lambda_x = 0.0;
    double lambda_y = 0.0;
    double rho = 0.0;
    // ρ^0 自适应标定值（首轮内层收敛后按 J_s'/max(‖C_f‖², ε_ρ) 标定）
    double rho_initial_calibrated = 0.0;
    // 实际执行的外层迭代轮数
    int outer_iterations = 0;
    // 全部外层轮的内层 L-BFGS 累计迭代次数
    int total_lbfgs_iterations = 0;
    // 最终总代价 J_ρ
    double final_cost = 0.0;
    // 最终 J_s'
    double final_j_s_prime = 0.0;
    // 逐轮外层迭代记录
    std::vector<MincoOuterIterationRecord> history;
};
// PHR-ALM 主求解器：两阶段优化流程的第二阶段，与 NmpcSolver 并列的后处理
// 求解器编排入口。以预处理器输出为初值，内层用 L-BFGS 优化决策变量
// [2(M-1) 个内部航点 (θ,s), M 个时间重参数化变量 τ_i, 终点弧长 s_f]，
// 外层按终点双指标判据收敛并更新乘子 λ 与惩罚权重 ρ。梯度链路与预处理
// 器同源：K(T)^{-T} 伴随反传（航点/s_f）+ K(T) 行对 T 的解析依赖（τ），
// 新增跃度闭式二次型、ESDF 逐节点后缀反传与 PHR-ALM 终端项三个环节。
class MincoSolver {
   public:
    // 构造并校验配置：非法配置抛 std::invalid_argument（运动学参数的合法性
    // 由 BicycleKinematicsExtractor 构造时校验，ESDF 惩罚配置的合法性由
    // MincoEsdfPenalty 构造时校验）
    explicit MincoSolver(const MincoConfig& config);
    // 主入口：以初值估计（边界/目标/换挡点）与预处理结果（内层初值）驱动
    // PHR-ALM 双层优化。输入非法（空估计/非有限字段/预处理结果与估计段数
    // 不一致/非正时长）抛 std::invalid_argument；未收敛显式返回
    // success=false 与对应 status，不假设一定收敛
    MincoSolverResult solve(const std::vector<MincoManeuverEstimate>& estimates,
                          const MincoPreprocessorResult& preprocessor_result,
                          const Eigen::Vector2d& start_position,
                          const ESDFMap& esdf_map,
                          const VehicleFootprintModel& footprint_model) const;
    // 当前配置（只读）
    const MincoConfig& config() const { return config_; }

   protected:
    // 辛普森节点数据：世界坐标还原积分在同一组固定节点上的求值结果，
    // 代价函数（梯度反传）与终点指标计算共享，保证"先离散后求导"的
    // 节点一致性
    struct SimpsonNodeData {
        // 每段 simpson_subintervals+1 个节点的 θ 值
        std::vector<std::vector<double>> node_theta;
        // 每段 simpson_subintervals+1 个节点的 ṡ 值
        std::vector<std::vector<double>> node_s_dot;
        // 每段的辛普森积分位移（世界系）
        std::vector<Eigen::Vector2d> segment_displacements;
        // 每段 simpson_subintervals+1 个节点的世界坐标（辛普森累计部分和，
        // 供 ESDF 逐节点求值）
        std::vector<std::vector<Eigen::Vector2d>> node_positions;
    };
    // 终点指标：由同一组辛普森节点产出，供外层收敛判定与 λ/ρ 更新
    struct TerminalMetrics {
        // 末端世界坐标（辛普森积分）
        Eigen::Vector2d end_position{0.0, 0.0};
        // 带符号终点误差 C_f
        Eigen::Vector2d violation{0.0, 0.0};
        // 终点位置误差 e_pos (m)
        double position_error = 0.0;
        // 终点朝向误差 e_heading (deg)
        double heading_error_deg = 0.0;
    };
    // 装配问题数据并逐项校验输入合法性
    MincoSolverProblem buildProblem(
        const std::vector<MincoManeuverEstimate>& estimates,
        const MincoPreprocessorResult& preprocessor_result,
        const Eigen::Vector2d& start_position) const;
    // 求值 J_ρ 代价与解析梯度（L-BFGS 函数对象；gradient 必须已分配且维数
    // 等于 problem.variableCount()；breakdown 可为空指针表示不需要分解）
    double evaluateCostAndGradient(const MincoSolverProblem& problem,
                                   const MincoEsdfPenalty& esdf_penalty,
                                   const MincoMultiplierState& multipliers,
                                   const Eigen::VectorXd& x,
                                   Eigen::VectorXd* gradient,
                                   MincoCostBreakdown* breakdown) const;
    // 由决策变量重建 MINCO 轨迹（维数不匹配抛 std::invalid_argument）
    MincoTrajectory buildTrajectory(const MincoSolverProblem& problem,
                                    const Eigen::VectorXd& x) const;
    // 在代价函数同一组辛普森节点上求值各段 θ/ṡ、段位移与逐节点世界坐标
    // （start_position 为世界坐标还原积分的锚点）
    SimpsonNodeData computeSimpsonNodeData(
        const MincoTrajectory& trajectory,
        const Eigen::Vector2d& start_position) const;
    // 由辛普森节点数据与轨迹求值终点指标（位置/朝向双指标）
    TerminalMetrics computeTerminalMetrics(
        const MincoSolverProblem& problem,
        const MincoTrajectory& trajectory) const;
    // 辛普森单位权重（不含段时长因子），代价函数与指标计算共享同一组权重
    static std::vector<double> SimpsonUnitWeights(int num_subintervals);
    // 角度归一化到 [-π, π]
    static double NormalizeAngle(double angle);
    // 标量裁剪到 [lower, upper]
    static double Clip(double value, double lower, double upper);

   protected:
    MincoConfig config_;
    BicycleKinematicsExtractor kinematics_;
};
}  // namespace apa_post_processor
