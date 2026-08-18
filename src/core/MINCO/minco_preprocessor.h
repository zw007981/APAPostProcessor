#pragma once

#include <Eigen/Core>
#include <vector>

#include "alm_maneuver_segmenter.h"
#include "bicycle_kinematics_extractor.h"
#include "minco_trajectory.h"

namespace apa_post_processor {
// 预处理粗优化配置（两阶段优化流程第一阶段 J_pre 的代价权重与求解参数）。
// J_pre 只包含三类代价（与设计文档"两阶段优化流程：先粗后精"一致）：
//   1) 运动学/加速度等物理约束惩罚（复用 BicycleKinematicsExtractor 的四项
//      防奇异二次形态约束，三次光滑外点罚）；
//   2) 段时长平衡约束惩罚（T_i 贴近均值，避免单段过长导致数值积分失效）；
//   3) 逐段终点对前端锚点 p_w0 的二次跟踪惩罚。
// 另有两项工程加固（均为可配置项，偏离论文字面处已在注释说明原因）：
//   - 换挡点 ṡ² 软惩罚：MincoTrajectory 的内部航点只支持 0 阶（位置）自由
//     变量，无法施加 ṡ=0 硬边界，预处理阶段以二次惩罚近似，使换挡点残余
//     速度收敛到接近 0；
//   - epsilon_time 时间正则：终点跟踪在固定系数下与段时长 T 精确无关
//     （辛普森权重 ∝T 与 ṡ∝1/T 抵消），物理约束可行后 T 方向梯度为零
//     （平坦方向），微小时间正则消除该退化、避免 T 无理由漂移。
struct AlmPreprocessorConfig {
    // 逐段终点对 p_w0 的二次跟踪权重。取值依据（段数压缩调参记录）：
    // 微段融合（fuse_arc=1.5）移除摆动段后，data1 初始结构跨距变大，
    // 10.0 时预处理终点误差 0.12 m 超容差失败；20.0 使四数据集预处理
    // 全部收敛且无其它指标恶化
    double weight_endpoint_track = 20.0;
    // 纵向速度约束 C_v 惩罚权重
    double weight_velocity = 1000.0;
    // 纵向加速度约束 C_a 惩罚权重
    double weight_acceleration = 1000.0;
    // 前轮最大转角约束 C_δ 惩罚权重。取值依据（换挡曲率补救调参记录）：
    // 主求解用 15.0 即可把行驶速度最大 κ 压到物理上限的 ~1.02 倍
    // （|δ̇| 同步验证 <0.4 rad/s）；预处理阶段必须保持 0.0——前端路径
    // 本身以 ~0.186 的 κ 骑行（超上限 ~8%），预处理施加 C_δ 会与终点
    // 跟踪冲突导致收敛失败（详见 docs/interfaces.md 变更记录）
    double weight_steer_angle = 0.0;
    // 方向盘打角速度约束 C_δ̇ 惩罚权重
    double weight_steer_rate = 1000.0;
    // 段时长平衡约束惩罚权重
    double weight_duration_balance = 10.0;
    // 段时长平衡下界系数 ε_low：T_i >= ε_low·mean(T)
    double duration_balance_lower = 0.5;
    // 段时长平衡上界系数 ε_upp：T_i <= ε_upp·mean(T)
    double duration_balance_upper = 2.0;
    // 时间正则权重（ε_T·ΣT_i，消除 T 平坦方向）
    double epsilon_time = 0.01;
    // 换挡点 ṡ² 软惩罚权重
    double weight_gear_cusp = 1000.0;
    // 换挡点 θ̇² 软惩罚权重（"换挡瞬间停转"：θ-s 独立多项式参数化下
    // ṡ=0 处 θ̇ 无硬边界可施加，以此二次惩罚压低换挡点转向残留，
    // 抑制 κ=tanδ/L 在换挡邻域的尖峰）。取值依据（换挡曲率补救调参
    // 记录）：200.0 在合法化组合（solver.weight_steer_angle=15.0、
    // weight_safe=600.0）下停驻窗口净旋转压至 ~0.02 rad 以内且
    // 段数/长度不劣于原始路径
    double weight_gear_cusp_theta = 200.0;
    // 每段物理约束采样点数（梯形积分节点，含两端，>= 2）
    int physics_samples_per_segment = 5;
    // 每段辛普森积分子区间数（世界坐标还原积分，必须为偶数且 >= 2）
    int simpson_subintervals = 8;
    // 成功判据：优化后各段末端世界坐标与 p_w0 的最大允许偏差 (m)。
    // 取值依据（合成场景调参记录）：直线/单次换挡合成场景实测收敛后跟踪
    // 误差在 1e-3 m 量级，0.1 m 给出约两个量级余量；该量级约为标称段长
    // d_seg=0.6 m 的 1/6，作为主优化（位置验收 0.05 m 量级）的 warm start
    // 足够接近前端路径
    double convergence_position_tolerance = 0.1;
    // L-BFGS 最大迭代次数。取值依据：合成场景实测 63（直线）~96（单次
    // 换挡）次迭代收敛，200 给出约 2 倍余量覆盖更多换挡/更长路径的输入
    int lbfgs_max_iterations = 200;
    // L-BFGS 梯度范数收敛阈值（绝对）。取值依据：预处理产出仅是主优化的
    // warm start，实测 1e-4 时合成场景已收敛到亚毫米级跟踪误差，更紧的
    // 阈值只增加迭代数、不改变 warm start 质量；比 NMPC 预处理管线
    // BSplineSmoother 的 1e-5 松一个量级，体现"粗优化"
    double lbfgs_epsilon = 1e-4;
    // L-BFGS 梯度范数收敛阈值（相对）
    double lbfgs_epsilon_rel = 1e-4;
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
// 预处理问题装配数据：由 AlmManeuverSegmenter 的初值估计一次性装配，供
// L-BFGS 代价函数反复求值时复用，避免每次求值重新展开。
struct AlmPreprocessorProblem {
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
struct AlmPreprocessorResult {
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
// 预处理粗优化器：两阶段优化流程的第一阶段。以 AlmManeuverSegmenter 的
// 初值估计为输入，在松收敛阈值下求解 J_pre（不含 ESDF 碰撞惩罚与外层
// ALM 乘子逻辑），把初值拉近前端路径并满足运动学约束，输出供主优化
// 使用的 MINCO 初值。决策变量布局：[2(M-1) 个内部航点 (θ,s) 分量,
// M 个时间重参数化变量 τ_i, 终点弧长 s_f]，梯度经 K(T)^{-T} 伴随反传
// （内部航点/终点弧长）与 K(T) 行对 T 的解析依赖（时间变量）。
class AlmPreprocessor {
   public:
    // 构造并校验配置：非法配置抛 std::invalid_argument（运动学参数的合法性
    // 由 BicycleKinematicsExtractor 构造时校验）
    explicit AlmPreprocessor(AlmPreprocessorConfig config = {},
                             BicycleKinematicsConfig kinematics_config = {});
    // 对初值估计执行 J_pre 粗优化。输入非法（空估计/空段/非有限字段/非正
    // 时长/非有限起点）抛 std::invalid_argument；优化失败显式返回
    // success=false（含 optimizer_converged=false 或跟踪误差超容差），
    // 不静默返回垃圾结果
    AlmPreprocessorResult preprocess(
        const std::vector<AlmManeuverEstimate>& estimates,
        const Eigen::Vector2d& start_position) const;
    // 当前配置（只读）
    const AlmPreprocessorConfig& config() const { return config_; }

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
    AlmPreprocessorProblem buildProblem(
        const std::vector<AlmManeuverEstimate>& estimates,
        const Eigen::Vector2d& start_position) const;
    // 求值 J_pre 代价与解析梯度（L-BFGS 函数对象；gradient 必须已分配且
    // 维数等于 problem.variableCount()）
    double evaluateCostAndGradient(const AlmPreprocessorProblem& problem,
                                   const Eigen::VectorXd& x,
                                   Eigen::VectorXd* gradient) const;
    // 由决策变量重建 MINCO 轨迹（维数不匹配抛 std::invalid_argument）
    MincoTrajectory buildTrajectory(const AlmPreprocessorProblem& problem,
                                    const Eigen::VectorXd& x) const;
    // 在代价函数同一组辛普森节点上求值各段 θ/ṡ 并积分各段位移
    SimpsonNodeData computeSimpsonNodeData(
        const MincoTrajectory& trajectory) const;
    // 按代价函数同一组辛普森节点积分各段末端世界坐标
    std::vector<Eigen::Vector2d> computeSegmentEndPositions(
        const AlmPreprocessorProblem& problem,
        const MincoTrajectory& trajectory) const;
    // 辛普森单位权重（不含段时长因子），代价函数与指标计算共享同一组权重
    static std::vector<double> SimpsonUnitWeights(int num_subintervals);

   protected:
    AlmPreprocessorConfig config_;
    BicycleKinematicsExtractor kinematics_;
};
}  // namespace apa_post_processor
