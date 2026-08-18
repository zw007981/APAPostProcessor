#pragma once

#include <Eigen/Core>
#include <vector>

#include "alm_esdf_penalty.h"
#include "alm_maneuver_segmenter.h"
#include "alm_preprocessor.h"
#include "bicycle_kinematics_extractor.h"
#include "minco_trajectory.h"

namespace apa_post_processor {
// PHR-ALM 主求解器外层迭代状态
enum class AlmSolverStatus {
    // 终点位置/朝向双指标同时满足收敛判据（ALM.md 2.5 节工业级判定）
    CONVERGED,
    // 达到最大外层迭代次数仍未满足双指标判据（实时系统兜底退出条件）
    MAX_OUTER_ITERATIONS,
    // 内层 L-BFGS 求解抛出异常（返回最后一次成功迭代的轨迹，显式标记失败）
    INNER_LBFGS_FAILED,
    // 由内层解重建 MINCO 轨迹失败（K(T) 奇异等极端数值情况）
    TRAJECTORY_BUILD_FAILED,
};
// PHR-ALM 主优化配置（内外层循环的判据、乘子/惩罚权重更新与内层 L-BFGS
// 求解参数）。内层目标 J_ρ = J_s' + PHR-ALM 终端项，其中 J_s' 为基础平滑
// 目标 J_0（跃度闭式积分 + 时间正则）与全部不等式软约束惩罚（运动学四项、
// 段时长平衡、换挡点软惩罚、ESDF 双重安全惩罚）之和，与设计文档 1.4/2.5 节
// 一一对应。偏离原论文的两处工程选择（均可在配置中切换，注释已注明原因）：
//   - 首轮内层优化以 λ=0、rho=first_round_rho 的纯软惩罚启动，首轮收敛后按
//     2.5 节公式标定 ρ^0 并替换，标定值才是后续 ALM 迭代使用的初始权重；
//   - ρ 的充分下降门控（ALM.md 1.4 节注明并非原论文内容）默认关闭，即按
//     原论文无条件递增；启用后仅当终点误差未充分减小时才提升 ρ。
struct AlmSolverConfig {
    // 终点位置收敛阈值 e_pos (m)（ALM.md 2.5 节：0.05 m）
    double terminal_position_tolerance = 0.05;
    // 终点朝向收敛阈值 e_heading (deg)（ALM.md 2.5 节：1.5°）
    double terminal_heading_tolerance_deg = 1.5;
    // 最大外层迭代次数（达到上限仍未收敛时显式返回失败状态）
    int max_outer_iterations = 20;
    // 首轮内层优化的临时软惩罚权重（λ^0=0 的纯软惩罚轮；首轮收敛后即被
    // 自适应标定的 ρ^0 替换，不参与后续迭代）
    double first_round_rho = 1.0;
    // ρ^0 标定公式的裁剪下界 ρ_min
    double rho_min = 1e-2;
    // ρ^0 标定公式的裁剪上界与递增上限 ρ_max
    double rho_max = 1e6;
    // ρ^0 标定安全阀 ε_ρ：防止 ||C_f||→0 时分母趋零导致 ρ^0 数值爆炸
    double epsilon_rho = 1e-4;
    // ρ 递增系数 γ：ρ^{k+1} = min((1+γ)·ρ^k, ρ_max)
    double rho_increase_factor = 1.0;
    // 充分下降门控开关（工程增补，默认关闭=原论文无条件递增）
    bool use_rho_increase_gate = false;
    // 门控阈值 κ∈(0,1)：||C_f^k|| > κ·||C_f^{k-1}|| 时才提升 ρ
    double rho_gate_kappa = 0.9;
    // 纵向跃度权重 w_s（J_0 抗点头项）。取值依据（四数据集调参记录）：
    // 1.0 时连续压缩不足，data6 路径长度 29.83 m 超既有基线；5.0 与
    // weight_gear_cusp=50 组合后四数据集全部满足合法性三门且段数/长度
    // 全面优于基线（详见 docs/ALM.md 第四章与 docs/interfaces.md 变更记录）
    double weight_jerk_s = 5.0;
    // 角跃度权重 w_θ（J_0 抗方向盘异响项）
    double weight_jerk_theta = 1.0;
    // 时间正则权重 ε_T（J_0 的 ε_T·T_s 项）
    double epsilon_time = 0.01;
    // 纵向速度约束 C_v 惩罚权重
    double weight_velocity = 1000.0;
    // 纵向加速度约束 C_a 惩罚权重
    double weight_acceleration = 1000.0;
    // 前轮最大转角约束 C_δ 惩罚权重。取值依据（换挡曲率补救调参记录）：
    // C_δ 已改造为光滑 hinge 形态（小违反区梯度不消失），15.0 配合
    // weight_gear_cusp_theta=200.0 与 weight_safe=600.0 的组合，四数据集
    // 行驶速度最大 κ 压到物理上限的 ~1.00~1.03 倍、|δ̇|≤0.38 rad/s，
    // 且段数不增于原始路径、长度全面更短（详见 docs/interfaces.md 变更
    // 记录）；继续加大权重对 κ 收益递减且破坏碰撞/预处理收敛。段数压缩
    // 调参（六批次扫描）进一步表明：10.0 在微段融合（fuse_arc=1.5）与
    // 交界平滑修复的组合下保持 κ_ratio ≤ 1.012 与全部合法性，同时换来
    // data1 10→2、data3 9→7、data6 6→4 的段数压缩（15.0 时 data1
    // kin_steer 临界超标、压缩收益持平，故取 10.0）
    double weight_steer_angle = 10.0;
    // 方向盘打角速度约束 C_δ̇ 惩罚权重
    double weight_steer_rate = 1000.0;
    // 换挡点 ṡ² 软惩罚权重（ṡ=0 无法施加为 MINCO 硬边界的工程近似）。
    // 取值依据（四数据集调参记录）：1000.0 时换挡点过死，连续压缩后的
    // 冗余段融化不充分（data7 保持 4 段、data6 长度 29.83 m 超基线）；
    // 50.0 与 weight_jerk_s=5.0 组合后四数据集全部满足合法性三门且
    // 段数/长度全面优于基线（详见 docs/ALM.md 第四章与
    // docs/interfaces.md 变更记录）
    double weight_gear_cusp = 50.0;
    // 换挡点 θ̇² 软惩罚权重（"换挡瞬间停转"：θ-s 独立多项式参数化下
    // ṡ=0 处 θ̇ 无硬边界可施加，以此二次惩罚压低换挡点转向残留，
    // 抑制 κ=tanδ/L 在换挡邻域的尖峰）。取值依据（换挡曲率补救调参
    // 记录）：200.0 在合法化组合（weight_steer_angle=15.0、
    // weight_safe=600.0）下停驻窗口净旋转压至 ~0.02 rad 以内且
    // 段数/长度不劣于原始路径
    double weight_gear_cusp_theta = 200.0;
    // 段时长平衡约束惩罚权重
    double weight_duration_balance = 10.0;
    // 段时长平衡下界系数 ε_low：T_i >= ε_low·mean(T)
    double duration_balance_lower = 0.5;
    // 段时长平衡上界系数 ε_upp：T_i <= ε_upp·mean(T)
    double duration_balance_upper = 2.0;
    // 每段物理约束采样点数（梯形积分节点，含两端，>= 2）
    int physics_samples_per_segment = 5;
    // 每段辛普森积分子区间数（世界坐标还原积分，必须为偶数且 >= 2）
    int simpson_subintervals = 8;
    // 内层 L-BFGS 最大迭代次数（主优化比预处理粗优化更紧）
    int lbfgs_max_iterations = 500;
    // 内层 L-BFGS 梯度范数收敛阈值（绝对）
    double lbfgs_epsilon = 1e-5;
    // 内层 L-BFGS 梯度范数收敛阈值（相对）
    double lbfgs_epsilon_rel = 1e-5;
    // L-BFGS 历史记忆长度
    int lbfgs_m = 10;
    // 线搜索最大试探次数
    int lbfgs_max_linesearch = 20;
    // 线搜索算法：1=Armijo, 2=Wolfe, 3=Strong Wolfe
    int lbfgs_linesearch_algo = 1;
    // Armijo 条件参数
    double lbfgs_ftol = 1e-4;
    // Wolfe 曲率条件参数
    double lbfgs_wolfe = 0.9;
};
// 主优化问题装配数据：由初值估计（边界条件/目标位姿/换挡点）与预处理结果
// （内层 L-BFGS 初值：航点/时长/终点弧长）一次性装配，供代价函数反复求值
struct AlmSolverProblem {
    // 起点边界条件（车辆起步为零速零加速度）
    MincoBoundaryCondition2d start;
    // 终点边界条件（θ 取目标朝向、硬边界精确满足；s 的位置分量仅为占位，
    // 求值时被决策变量 s_f 覆盖）
    MincoBoundaryCondition2d end;
    // 起点世界坐标（世界坐标还原积分的锚点）
    Eigen::Vector2d start_position{0.0, 0.0};
    // 终点目标位姿（ALM 终端项的 x_target/y_target/θ_target）
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
struct AlmMultiplierState {
    // 终点 x 向乘子 λ_x
    double lambda_x = 0.0;
    // 终点 y 向乘子 λ_y
    double lambda_y = 0.0;
    // 惩罚权重 ρ
    double rho = 0.0;
};
// 内层代价分解：J_s' 与 PHR-ALM 终端项分离（ρ^0 标定只依赖 J_s'）
struct AlmCostBreakdown {
    // 总代价 J_ρ = J_s' + alm_terminal
    double total = 0.0;
    // 基础平滑目标 J_0 与全部不等式软约束惩罚之和（不含 ALM 终端项）
    double j_s_prime = 0.0;
    // PHR-ALM 增广拉格朗日终端项
    double alm_terminal = 0.0;
    // ESDF 双重安全惩罚分项（J_s' 的组成部分，单独透出供诊断）
    double esdf_penalty = 0.0;
};
// 单轮外层迭代的完整记录（供收敛判定回溯与更新公式单步验证）
struct AlmOuterIterationRecord {
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
    // 本轮解的 ALM 终端项
    double alm_terminal = 0.0;
    // 本轮解的 ESDF 惩罚分项
    double esdf_penalty = 0.0;
    // 本轮解的总代价 J_ρ
    double total_cost = 0.0;
    // 本轮内层 L-BFGS 实际迭代次数
    int lbfgs_iterations = 0;
};
// PHR-ALM 主优化结果
struct AlmSolverResult {
    // 整体成功：双指标收敛判据满足（status == CONVERGED）
    bool success = false;
    // 外层迭代终止状态（失败路径的显式反馈，不静默返回垃圾结果）
    AlmSolverStatus status = AlmSolverStatus::MAX_OUTER_ITERATIONS;
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
    // ρ^0 自适应标定值（首轮内层收敛后按 2.5 节公式标定）
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
    std::vector<AlmOuterIterationRecord> history;
};
// PHR-ALM 主求解器：两阶段优化流程的第二阶段，与 NmpcSolver 并列的后处理
// 求解器编排入口。以预处理器输出为初值，内层用 L-BFGS 优化决策变量
// [2(M-1) 个内部航点 (θ,s), M 个时间重参数化变量 τ_i, 终点弧长 s_f]，
// 外层按终点双指标判据收敛并更新乘子 λ 与惩罚权重 ρ。梯度链路与预处理
// 器同源：K(T)^{-T} 伴随反传（航点/s_f）+ K(T) 行对 T 的解析依赖（τ），
// 新增跃度闭式二次型、ESDF 逐节点后缀反传与 PHR-ALM 终端项三个环节。
class AlmSolver {
   public:
    // 构造并校验配置：非法配置抛 std::invalid_argument（运动学参数的合法性
    // 由 BicycleKinematicsExtractor 构造时校验，ESDF 惩罚配置的合法性由
    // AlmEsdfPenalty 构造时校验）
    explicit AlmSolver(AlmSolverConfig config = {},
                       BicycleKinematicsConfig kinematics_config = {},
                       AlmEsdfPenaltyConfig esdf_config = {});
    // 主入口：以初值估计（边界/目标/换挡点）与预处理结果（内层初值）驱动
    // PHR-ALM 双层优化。输入非法（空估计/非有限字段/预处理结果与估计段数
    // 不一致/非正时长）抛 std::invalid_argument；未收敛显式返回
    // success=false 与对应 status，不假设一定收敛
    AlmSolverResult solve(const std::vector<AlmManeuverEstimate>& estimates,
                          const AlmPreprocessorResult& preprocessor_result,
                          const Eigen::Vector2d& start_position,
                          const ESDFMap& esdf_map,
                          const VehicleFootprintModel& footprint_model) const;
    // 当前配置（只读）
    const AlmSolverConfig& config() const { return config_; }

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
    AlmSolverProblem buildProblem(
        const std::vector<AlmManeuverEstimate>& estimates,
        const AlmPreprocessorResult& preprocessor_result,
        const Eigen::Vector2d& start_position) const;
    // 求值 J_ρ 代价与解析梯度（L-BFGS 函数对象；gradient 必须已分配且维数
    // 等于 problem.variableCount()；breakdown 可为空指针表示不需要分解）
    double evaluateCostAndGradient(const AlmSolverProblem& problem,
                                   const AlmEsdfPenalty& esdf_penalty,
                                   const AlmMultiplierState& multipliers,
                                   const Eigen::VectorXd& x,
                                   Eigen::VectorXd* gradient,
                                   AlmCostBreakdown* breakdown) const;
    // 由决策变量重建 MINCO 轨迹（维数不匹配抛 std::invalid_argument）
    MincoTrajectory buildTrajectory(const AlmSolverProblem& problem,
                                    const Eigen::VectorXd& x) const;
    // 在代价函数同一组辛普森节点上求值各段 θ/ṡ、段位移与逐节点世界坐标
    // （start_position 为世界坐标还原积分的锚点）
    SimpsonNodeData computeSimpsonNodeData(
        const MincoTrajectory& trajectory,
        const Eigen::Vector2d& start_position) const;
    // 由辛普森节点数据与轨迹求值终点指标（位置/朝向双指标）
    TerminalMetrics computeTerminalMetrics(
        const AlmSolverProblem& problem,
        const MincoTrajectory& trajectory) const;
    // 辛普森单位权重（不含段时长因子），代价函数与指标计算共享同一组权重
    static std::vector<double> SimpsonUnitWeights(int num_subintervals);
    // 角度归一化到 [-π, π]
    static double NormalizeAngle(double angle);
    // 标量裁剪到 [lower, upper]
    static double Clip(double value, double lower, double upper);

   protected:
    AlmSolverConfig config_;
    BicycleKinematicsExtractor kinematics_;
    AlmEsdfPenaltyConfig esdf_config_;
};
}  // namespace apa_post_processor
