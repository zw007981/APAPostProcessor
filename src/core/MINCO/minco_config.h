#pragma once

#include "../../util/config.h"

namespace apa_post_processor {
// MINCO 配置
struct MincoConfig : public Config {
    // ===== 运动学映射与物理约束（BicycleKinematicsExtractor）=====
    // 轴距 L_base (m)
    double wheelbase = 2.8;
    // 纵向速度上限 v_max (m/s)
    double max_velocity = 2.0;
    // 纵向加速度上限 a_max (m/s²)
    double max_acceleration = 1.5;
    // 前轮最大转角 δ_max (rad)
    double max_steer_angle = 0.65;
    // 方向盘打角速度上限 δ̇_max (rad/s)
    double max_steer_rate = 0.4;
    // δ/δ̇ 分母正则化常数 ε_g：取 0 时退化为未正则化公式
    double epsilon_g = 1e-8;
    // C_δ 惩罚的光滑 hinge 半宽 ε_h
    double steer_hinge_epsilon = 1e-4;

    // ===== ESDF 双重安全惩罚（MincoEsdfPenalty）=====
    // 硬安全边界 margin_safe (m)
    double margin_safe = 0.02;
    // 硬安全边界惩罚权重 W_safe
    double weight_safe = 2000.0;
    // 舒适缓冲边界 margin_comf (m)
    double margin_comf = 0.2;
    // 舒适安全边界惩罚权重 W_comf
    double weight_comf = 20.0;

    // ===== 前端解析与降采样（MincoManeuverSegmenter）=====
    // 标称段长 d_seg (m)
    double nominal_segment_length = 0.6;
    // 标称行驶速度 (m/s)，用于段时长初值估计
    double nominal_speed = 0.5;
    // 标称转向角速度 (rad/s)，用于原地转向段的时长初值估计
    double nominal_turn_rate = 0.3;
    // 段时长下限 (s)，防止退化输入产生非正时长
    double min_segment_duration = 0.5;
    // 微段融合弧长阈值 (m)：>0 时，方向相反且 |Δs|/|Δθ| 低于阈值的
    // 摆动微段在分段阶段即被移除（邻段吸收/同向合并），0 为关闭；κ
    // 合法化封死 pivot 压缩后段数压缩只能前移到分段结构层，1.5 为最大
    double fuse_arc_threshold = 1.5;
    // 微段融合朝向阈值 (rad)：候选段 |Δθ| 低于该值才可融合，保护真实
    // 转向调整段不被误融
    double fuse_heading_threshold = 0.2;

    // ===== 机动融化与拓扑修剪（MincoManeuverMelter）=====
    // 融化段弧长阈值 |Δs| (m)：低于此值视为物理冗余换挡废段
    double melt_arc_threshold = 0.05;
    // PIVOT 判定朝向阈值 |Δθ| (rad)：弧长低于阈值但朝向超出此值时
    // 为原地掉头式微动，保留为 PIVOT 而非剔除；默认值沿用
    // topology_cleaner 的 pivot_delta_threshold 量级
    double melt_heading_threshold = 0.1;
    // 多项式段离散为 Path 点的每段采样点数（>= 2）
    int samples_per_segment = 16;

    // ===== 预处理粗优化（MincoPreprocessor）=====
    // 逐段终点对 p_w0 的二次跟踪权重
    double weight_endpoint_track = 20.0;
    // 纵向速度约束 C_v 惩罚权重
    double pre_weight_velocity = 1000.0;
    // 纵向加速度约束 C_a 惩罚权重
    double pre_weight_acceleration = 1000.0;
    // 前轮最大转角约束 C_δ 惩罚权重
    double pre_weight_steer_angle = 0.0;
    // 方向盘打角速度约束 C_δ̇ 惩罚权重
    double pre_weight_steer_rate = 1000.0;
    // 段时长平衡约束惩罚权重
    double pre_weight_duration_balance = 10.0;
    // 段时长平衡下界系数 ε_low：T_i >= ε_low·mean(T)
    double pre_duration_balance_lower = 0.5;
    // 段时长平衡上界系数 ε_upp：T_i <= ε_upp·mean(T)
    double pre_duration_balance_upper = 2.0;
    // 时间正则权重（ε_T·ΣT_i，消除 T 平坦方向）
    double pre_epsilon_time = 0.01;
    // 换挡点 ṡ² 软惩罚权重
    double pre_weight_gear_cusp = 1000.0;
    // 换挡点 θ̇² 软惩罚权重：压低转向残留，抑制 κ=tanδ/L
    // 在换挡邻域的尖峰（θ-s 参数化下 ṡ=0 处 θ̇ 无硬边界可施加）
    double pre_weight_gear_cusp_theta = 200.0;
    // 每段物理约束采样点数（梯形积分节点，含两端，>= 2）
    int pre_physics_samples_per_segment = 5;
    // 每段辛普森积分子区间数（世界坐标还原积分，偶数且 >= 2）
    int pre_simpson_subintervals = 8;
    // 成功判据：各段末端世界坐标与 p_w0 的最大允许偏差 (m)。合成
    // 场景实测收敛后跟踪误差在 1e-3 m 量级，0.1 m 给约两个量级余量
    // （主优化位置验收 0.05 m），作为 warm start 足够接近前端路径
    double convergence_position_tolerance = 0.1;
    // L-BFGS 最大迭代次数
    int pre_lbfgs_max_iterations = 200;
    // L-BFGS 梯度范数收敛阈值
    double pre_lbfgs_epsilon = 1e-4;
    // L-BFGS 梯度范数收敛阈值
    double pre_lbfgs_epsilon_rel = 1e-4;
    // L-BFGS 历史记忆长度
    int pre_lbfgs_m = 10;
    // 线搜索最大试探次数
    int pre_lbfgs_max_linesearch = 20;
    // 线搜索算法：1=Armijo, 2=Wolfe, 3=Strong Wolfe
    int pre_lbfgs_linesearch_algo = 1;
    // Armijo 条件参数
    double pre_lbfgs_ftol = 1e-4;
    // Wolfe 曲率条件参数
    double pre_lbfgs_wolfe = 0.9;

    // ===== PHR-ALM 主流程（MincoSolver，solver_ 前缀）=====
    // 终点位置收敛阈值 e_pos (m)
    double terminal_position_tolerance = 0.05;
    // 终点朝向收敛阈值 e_heading (deg)
    double terminal_heading_tolerance_deg = 1.5;
    // 最大外层迭代次数（达到上限仍未收敛时显式返回失败状态）
    int max_outer_iterations = 20;
    // 首轮内层优化的临时软惩罚权重（λ^0=0 的纯软惩罚轮；收敛后即被
    // 自适应标定的 ρ^0 替换）
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
    // 纵向跃度权重 w_s：1.0 时压缩不足（data6 长 29.83 m 超基线），
    // 5.0 与 solver_weight_gear_cusp=50 组合后四数据集全合法
    // 且段数/长度全面优于基线
    double weight_jerk_s = 5.0;
    // 角跃度权重 w_θ（J_0 抗方向盘异响项）
    double weight_jerk_theta = 1.0;
    // 时间正则权重 ε_T（J_0 的 ε_T·T_s 项）
    double solver_epsilon_time = 0.01;
    // 纵向速度约束 C_v 惩罚权重
    double solver_weight_velocity = 1000.0;
    // 纵向加速度约束 C_a 惩罚权重
    double solver_weight_acceleration = 1000.0;
    // 最大转角约束 C_δ 惩罚权重：C_δ 为光滑 hinge 形态，
    // 10.0 配合 solver_weight_gear_cusp_theta=200 与 weight_safe=600
    // 使四数据集行驶 κ ≤ 1.012 倍物理上限、段数压缩、长度全面更短
    double solver_weight_steer_angle = 10.0;
    // 方向盘打角速度约束 C_δ̇ 惩罚权重
    double solver_weight_steer_rate = 1000.0;
    // 换挡点 ṡ² 软惩罚权重（ṡ=0 无硬边界可施加的近似）：
    // 1000.0 时换挡点过死、融化不充分，50.0 与 weight_jerk_s=5 组合
    // 后四数据集全合法且段数/长度优于基线
    double solver_weight_gear_cusp = 50.0;
    // 换挡点 θ̇² 软惩罚权重："换挡瞬间停转"，抑制 κ=tanδ/L 在
    // 换挡邻域的尖峰；200.0 使停驻窗口净旋转压至 ~0.02 rad 以内
    double solver_weight_gear_cusp_theta = 200.0;
    // 段时长平衡约束惩罚权重
    double solver_weight_duration_balance = 10.0;
    // 段时长平衡下界系数 ε_low：T_i >= ε_low·mean(T)
    double solver_duration_balance_lower = 0.5;
    // 段时长平衡上界系数 ε_upp：T_i <= ε_upp·mean(T)
    double solver_duration_balance_upper = 2.0;
    // 每段物理约束采样点数（梯形积分节点，含两端，>= 2）
    int solver_physics_samples_per_segment = 5;
    // 每段辛普森积分子区间数（世界坐标还原积分，偶数且 >= 2）
    int solver_simpson_subintervals = 8;
    // 内层 L-BFGS 最大迭代次数（比预处理粗优化更紧）
    int solver_lbfgs_max_iterations = 500;
    // 内层 L-BFGS 梯度范数收敛阈值（绝对）
    double solver_lbfgs_epsilon = 1e-5;
    // 内层 L-BFGS 梯度范数收敛阈值（相对）
    double solver_lbfgs_epsilon_rel = 1e-5;
    // L-BFGS 历史记忆长度
    int solver_lbfgs_m = 10;
    // 线搜索最大试探次数
    int solver_lbfgs_max_linesearch = 20;
    // 线搜索算法：1=Armijo, 2=Wolfe, 3=Strong Wolfe
    int solver_lbfgs_linesearch_algo = 1;
    // Armijo 条件参数
    double solver_lbfgs_ftol = 1e-4;
    // Wolfe 曲率条件参数
    double solver_lbfgs_wolfe = 0.9;

    // ===== 停驻窗口合法化改写（MincoSteerPadding，pad_ 前缀）=====
    // 停驻判定速度上限 (m/s)：|v| 低于该值的连续点段视为停驻窗口
    double v_epsilon = 0.05;
    // 可冻结窗口的最大净朝向变化 (rad)：净 Δθ 低于该值才允许改写，
    // 冻结把 Δθ 转嫁到下游航向，阈值需与终点航向容差 1.5° 留余量
    // 超阈值的真实旋转窗口保持原样
    double max_freeze_dtheta = 0.02;
    // 最大转角 δ_max (rad)：替换序列的转角限幅（与运动学同名字段）
    double pad_steer_angle = 0.48;
    // 最大转角速度 δ̇_max (rad/s)：替换序列的过渡速率上限，决定窗口
    // 最短分配时长，不足时插入更多点并顺延时间戳
    double pad_steer_rate = 0.4;
    // 替换序列的采样间隔 (s)
    double sample_dt = 0.1;
};
}  // namespace apa_post_processor
