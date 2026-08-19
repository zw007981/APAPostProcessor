#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <string>
#include <vector>

#include "../../util/config.h"
#include "../../util/constants.h"
#include "../../util/trajectory.h"
#include "../../vehicle/vehicle_params.h"

namespace apa_post_processor {
// iLQR配置
struct iLQRConfig : public Config {
    // ===== 参考构建（iLQRReferenceBuilder）=====
    // 等弧长重采样标称间距 (m)
    double reference_sample_dist{DELTA_DIST};
    // 固定离散步长 (s)
    double reference_dt{0.1};
    // 打靶节点规则间隔 n_s（步）
    std::size_t reference_shooting_interval{25};
    // 纵向速度幅值上限 (m/s)
    double reference_v_max{1.5};
    // 纵向加速度幅值上限 (m/s²)
    double reference_a_max{1.0};
    // 前轮转角幅值上限 (rad)
    double reference_delta_max{0.47728};
    // 前轮转角速度幅值上限 (rad/s)
    double reference_omega_max{0.4};

    // ===== RS 基于固定换挡点的前处理（ShortcutShiftPoints）=====
    // 曲率上限比（0=关闭）
    double rs_cap_ratio{0.0};
    // ESDF 侵入裕度 + 圆心必须在地图内 (m)
    double rs_collision_margin{0.02};
    // 长度增长上限比（以原始输入长度为基准）
    double rs_max_length_growth{0.05};
    // RS 曲线离散采样间距 (m)
    double rs_sample_dist{0.05};
    // DP 代价：每 maneuver 固定段价
    double rs_segment_fixed_cost{8.88};
    // DP 代价：短段惩罚权重
    double rs_short_segment_weight{1.0};
    // DP 代价：短段判定阈值 (m)
    double rs_short_segment_length{2.5};
    // RS 逐次耗时记录文件（CSV），空=关闭
    std::string rs_timing_csv{};
    // RS 耗时记录分组标签（如数据集名）
    std::string rs_timing_tag{};

    // ===== 代价（iLQRCostEvaluator）=====
    // 跃度权重
    double cost_weight_jerk{1.0};
    // 转角加加速度权重
    double cost_weight_steer_accel{1.0};
    // 跟踪位置权重基准值
    double cost_weight_ref_base{10.0};
    // 跟踪朝向权重
    double cost_weight_theta{5.0};
    // 速度上限 (m/s)
    double cost_v_max{1.5};
    // 加速度上限 (m/s²)
    double cost_a_max{1.0};
    // 转角速率上限 (rad/s)
    double cost_omega_max{0.5};
    // 前轮转角上限 (rad)
    double cost_delta_max{0.55};

    // ===== 内层 MS-iLQR（MsIlqrSolver）=====
    // 跃度上限 (m/s³)
    double inner_jerk_max{1.5};
    // 转角加加速度上限 (rad/s²)
    double inner_steer_accel_max{1.0};
    // 迭代上限
    int inner_max_iterations{50};
    // 代价变化容差
    double inner_cost_change_tol{1e-6};
    // 梯度范数容差
    double inner_gradient_tol{1e-8};
    // 收敛可行性守卫：缺陷 ‖d‖∞ 容差
    double inner_convergence_defect_tol{1e-3};
    // 正则化初值
    double inner_reg_initial{1e-4};
    // 正则化下界
    double inner_reg_min{1e-9};
    // 正则化上界
    double inner_reg_max{1e9};
    // 正则化增大倍率
    double inner_reg_increase{10.0};
    // 正则化缩小倍率
    double inner_reg_decrease{0.5};
    // Armijo γ
    double inner_armijo_gamma{0.1};
    // 回溯衰减 β
    double inner_backtrack_beta{0.5};
    // 回溯次数上限
    int inner_max_backtracks{50};
    // merit μ₀
    double inner_merit_mu0{10.0};
    // merit-AL 量级挂钩比率 c（0=关闭）
    double inner_merit_mu_al_ratio{0.0};
    // merit μ 上限
    double inner_merit_mu_max{1e9};
    // 定义域守卫外扩 margin (m)，0=关闭
    double inner_domain_guard_margin{2.0};
    // true=ALTRO 虚拟控制增广（默认开启）：首轮 rollout 缺陷恒零，帮助
    // 从不可行初值起步，四数据集长度更短且 data6/data7 升为阶段二收敛
    bool inner_use_virtual_control{true};
    // 虚拟控制软代价权重 R_inf
    double inner_virtual_control_weight{1e4};

    // ===== 外层 AL（AlOuterLoop）=====
    // 外层迭代上限
    int outer_max_outer_iterations{20};
    // 终点位置容差 (m)
    double outer_terminal_position_tol{0.05};
    // 终点朝向容差 (deg)
    double outer_terminal_heading_tol_deg{1.5};
    // 归一化不等式工程容差
    double outer_inequality_tol{0.021};
    // 打靶缺陷容差
    double outer_defect_tol{1e-3};
    // 罚权重下界
    double outer_mu_min{1e2};
    // 罚权重上界
    double outer_mu_max{1e6};
    // 首轮罚权重（弱启动）
    double outer_first_round_mu{1.0};
    // 幅值不等式初始罚权重
    double outer_amplitude_mu_initial{1.0};
    // true=阶段一逐元素门控
    bool outer_amplitude_mu_per_element{true};
    // 阶段二逐元素门控开关
    bool outer_amplitude_mu_per_element_stage_two{false};
    // μ⁰ 标定小量 ε_μ
    double outer_epsilon_mu{1e-4};
    // 充分下降门控 κ
    double outer_mu_gate_kappa{0.9};
    // 罚权重增长倍率 φ
    double outer_mu_growth_factor{10.0};
    // 退火率 γ
    double outer_anneal_gamma{0.5};
    // ESDF 逐轮量级调度增长
    double outer_esdf_scale_growth{1.0};
    // ESDF 逐轮因子上限
    double outer_esdf_scale_max{1.0};

    // ===== 求解编排（ApaILQRSolver）=====
    // 阶段二外层迭代上限
    int stage_two_max_outer_iterations{16};
    // 门控罚权重初值
    double gating_mu_initial{10.0};
    // 门控罚权重上界
    double gating_mu_max{1e6};
    // 门控违反度容差
    double gating_tol{1e-2};

    // ===== ESDF 双 margin 惩罚（iLQREsdfConstraint）=====
    // 安全边界 (m)
    double esdf_margin_safe{0.02};
    // 舒适边界 (m)
    double esdf_margin_comf{0.20};
    // 安全惩罚权重
    double esdf_weight_safe{100.0};
    // 舒适惩罚权重
    double esdf_weight_comf{10.0};
    // 时间轴抽样间隔
    int esdf_stride{1};

    // ===== 后处理与阶段二门控精化（iLQRPostStage）=====
    // 符号游程滞回 (m/s)
    double post_epsilon_v{0.02};
    // 修剪剔除弧长阈值 (m)
    double post_prune_min_arc_length{0.05};
    // 修剪 PIVOT 朝向变化阈值 (rad)
    double post_prune_pivot_heading_threshold{0.1};
    // 驻留窗速度帽 (m/s)
    double post_v_dwell{0.05};
    // 换挡延迟下限 (s)
    double post_shift_delay{0.4};
    // 驻留时长安全余量
    double post_kappa_pad{1.2};
    // 转角速率上限 (rad/s)，与求解配置同源
    double post_omega_max{0.5};
    // 转角加加速度上限 (rad/s²)
    double post_eta_max{1.0};
    // 接缝 |v| 质量指标（记录不否决）
    double post_seam_speed_tol{0.02};
    // 驻留窗端点 |ω| 容差（记录不否决）
    double post_dwell_omega_tol{0.55};
    // v/a 绝对容差（合法性门）
    double post_amplitude_check_tol{0.05};
    // δ/ω 相对容差（合法性门）
    double post_amplitude_check_rel_tol{0.021};
    // j/η 盒过冲探针（记录不否决）
    double post_control_overshoot_tol{0.3};
    // 阶段二跟踪权重地板
    double post_stage_two_min_tracking_weight{0.0};
    // true=阶段一末轮跟踪权重退火到地板时跳过阶段二
    bool post_skip_stage_two_when_weight_exhausted{false};
    // 轨迹合法性校验配置（util 层共享类型，保持嵌套）
    TrajectoryValidationConfig validation{};

    // ===== 双候选编排 =====
    // 开启后同一输入跑两遍完整 iLQR，第二遍退火率改为0.999 (近似不退火) 进行精修，
    // 在某些场景中可以得到更短的路径但代价是在所有场景中增加计算时间
    bool dual_candidate_select{false};

    // 默认构造：固定 merit μ₀=100（自适应规则已证伪）、内层判据收紧到
    // 1e-9（防大 μ 量级误判）、校验航向容差 1.5°，随后同步幅值边界
    iLQRConfig() {
        inner_merit_mu0 = 100.0;
        inner_cost_change_tol = 1e-9;
        validation.max_terminal_heading_error_deg = 1.5;
        synchronizeAmplitudeBounds();
    }
    // 权威幅值边界同步进全部消费方（reference_* → cost_*/post_*）
    void synchronizeAmplitudeBounds() {
        cost_v_max = reference_v_max;
        cost_a_max = reference_a_max;
        cost_delta_max = reference_delta_max;
        cost_omega_max = reference_omega_max;
        post_omega_max = reference_omega_max;
        post_eta_max = inner_steer_accel_max;
    }
    // 以车辆物理参数为上界收紧幅值边界（只收紧、不放宽），并同步
    void clampToVehicleParams(const VehicleParams& vehicle_params) {
        reference_delta_max =
            std::min(reference_delta_max, vehicle_params.max_steer_angle);
        reference_omega_max =
            std::min(reference_omega_max, vehicle_params.max_steer_rate);
        reference_a_max = std::min(
            reference_a_max, std::min(vehicle_params.max_accel,
                                      std::abs(vehicle_params.max_decel)));
        synchronizeAmplitudeBounds();
    }
};
}  // namespace apa_post_processor
