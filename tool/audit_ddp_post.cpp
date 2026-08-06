// DDP 后处理链路四断点审计工具：复刻 PostProcessor::optimizeDdp 的装配
// （同一配置来源 data/ddp_config.json、同一碰撞模型、同一求解流程），但在
// 后处理链路内部逐步手动编排，对四数据集在四个断点逐一转储机器可读的
// 结构化量测（[AUDIT] 前缀行），供后处理/校验口径审计取证：
//   断点0 参考构建：逐 maneuver 元数据与融化临界权重比（[AUDIT-REFMAN]），
//         以及超物理上限的参考曲率伪影定位（[AUDIT-REFKAPPA]）；
//   断点1 阶段一解：逐符号游程 sign/Δs/Δθ/Δδ/max|v|/网格步数/时长 T，
//         以及 cusp 处 |v| 分布（[AUDIT-S1RUN]/[AUDIT-S1CUSP] 行）；
//   断点2 修剪环节：逐 maneuver 的弧长/Δδ/Δθ 判据量、首末保护标记、现行
//         Δδ 判据与 ALM 侧 Δθ 判据的对照分类结果，以及生产修剪的实际
//         分类去向（[AUDIT-PRUNE]/[AUDIT-PRUNE-SUM] 行）；
//   断点3 阶段二解：逐轮外层的终端双指标/幅值违反/缺陷/μ 与最终门控
//         违反度（[AUDIT-S2ROUND]/[AUDIT-S2FINAL] 行）；
//   断点4 驻留与最终输出：逐接缝报告 + 校验清单的全部实测值（不短路，
//         [AUDIT-SEAM]/[AUDIT-CHECK] 行，layer=gate/metric 与生产
//         validateOutput 的门/指标两层口径一致），并对「阶段一解 +
//         修剪 + 驻留插入」降级候选做同一套量测（cand=stage1 行），
//         为分级降级出口提供可输出性证据。
// 生产判定模拟：候选一不可用时降级——阶段二未收敛则 first_fail=
// stage_two_convergence；阶段二收敛时给出首个未过的合法性门（质量指标
// 不参与否决）。阶段一未收敛的数据集只转储断点0/1（生产语义下后续断点
// 不存在）。阶段二跟踪权重与生产同一地板口径（stage_one 末轮退火值与
// post_stage.stage_two_min_tracking_weight 取大）。
// 变体支持：变体矩阵与调参工具共享 ddp_tune_common.h 的同一真值来源；
// 命令行可指定单个变体名（默认 baseline，"all" 跑全部变体）。
// 运行示例（仓库根目录）：
//   ./build/Release/apa_audit_ddp_post [variant_name|all]
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <string>
#include <utility>
#include <vector>

#include "core/DDP/apa_ddp_solver.h"
#include "core/DDP/bicycle_dynamics.h"
#include "core/DDP/ddp_cost.h"
#include "core/DDP/ddp_post_stage.h"
#include "core/DDP/ddp_reference_builder.h"
#include "core/DDP/esdf_constraint.h"
#include "core/NMPC/vehicle_circle_geometry.h"
#include "core/post_processor.h"
#include "spatial/esdf_map.h"
#include "spatial/grid_map.h"
#include "util/config_loader.h"
#include "util/data_loader.hpp"
#include "util/logger.h"
#include "util/topology_cleaner.h"
#include "util/trajectory.h"
#include "vehicle/vehicle_footprint_model.h"
#include "vehicle/vehicle_params.h"

#include "ddp_tune_common.h"

using namespace apa_post_processor;
using ddp_tune::DatasetCase;

namespace {
// 白盒访问器：暴露 protected 的接缝转向需求静态量测，供降级候选（阶段一
// 状态网格）的驻留窗计划复用同一量测规则（不改动被测实现）
class PostStageAuditAccessor : public DdpPostStage {
   public:
    using DdpPostStage::DdpPostStage;
    using DdpPostStage::MeasureSeamDeltaDeltaFromStates;
};

// 断点 1：阶段一解逐符号游程量测（sign/Δs/Δθ/Δδ/max|v|/网格步数/时长 T）
// 与 cusp 处 |v| 分布（游程边界共享点及其 ±1 邻域的最大 |v|，捕捉滞回
// 滞后——边界点按定义已承诺新符号，其 |v|>=ε_v，真实过零点在邻域内）
void DumpStageOneRuns(const std::string& variant, const std::string& dataset,
                      const DdpPostStage& post_stage,
                      const DdpAlignedVec<DdpState>& states, double dt) {
    const auto runs = post_stage.analyzeSignRuns(states);
    for (std::size_t i = 0; i < runs.size(); ++i) {
        const auto& run = runs[i];
        double max_abs_v = 0.0;
        for (std::size_t k = run.begin_index; k <= run.end_index; ++k) {
            max_abs_v = std::max(max_abs_v, std::abs(states[k](DDP_IDX_V)));
        }
        const double delta_delta =
            std::abs(states[run.end_index](DDP_IDX_DELTA) -
                     states[run.begin_index](DDP_IDX_DELTA));
        const std::size_t steps = run.end_index - run.begin_index;
        std::cout << "[AUDIT-S1RUN] variant=" << variant
                  << " dataset=" << dataset << " index=" << i
                  << " sign=" << run.sign << " begin=" << run.begin_index
                  << " end=" << run.end_index << " steps=" << steps
                  << " T=" << steps * dt << " ds=" << run.delta_s
                  << " dtheta=" << run.delta_theta << " ddelta=" << delta_delta
                  << " vmax=" << max_abs_v << std::endl;
    }
    // cusp 速度分布：相邻游程的共享边界点（首次承诺反号的样本）
    for (std::size_t i = 1; i < runs.size(); ++i) {
        const std::size_t boundary = runs[i].begin_index;
        double window_max_v = 0.0;
        const std::size_t lo = boundary > 0 ? boundary - 1 : 0;
        const std::size_t hi = std::min(boundary + 1, states.size() - 1);
        for (std::size_t k = lo; k <= hi; ++k) {
            window_max_v =
                std::max(window_max_v, std::abs(states[k](DDP_IDX_V)));
        }
        std::cout << "[AUDIT-S1CUSP] variant=" << variant
                  << " dataset=" << dataset << " index=" << i
                  << " node=" << boundary
                  << " v_boundary=" << std::abs(states[boundary](DDP_IDX_V))
                  << " v_window_max=" << window_max_v << std::endl;
    }
}

// 断点 2：修剪环节逐 maneuver 判据量与对照分类。对每个 maneuver 转储：
// 弧长、现行 PIVOT 判据量 |Δδ|（首末点前轮转角差）、ALM 侧判据量 |Δθ|
// （游程朝向变化）、首/末保护标记、两种判据各自的分类结果
// （NORMAL/PIVOT/DROP/PROTECT）以及生产修剪的实际分类去向
void DumpPrune(const std::string& variant, const std::string& dataset,
               const DdpPostStage& post_stage,
               const DdpPostStageConfig& config,
               const std::vector<DdpSignRun>& runs,
               const std::vector<Maneuver>& maneuvers_before,
               const std::vector<Maneuver>& maneuvers_after, bool prune_ok) {
    std::size_t dropped = 0;
    std::size_t pivoted = 0;
    for (std::size_t i = 0; i < maneuvers_before.size(); ++i) {
        const auto& maneuver = maneuvers_before[i];
        const double arc = maneuver.length();
        double ddelta = 0.0;
        if (maneuver.points.size() >= 2 && maneuver.points.front().hasDelta() &&
            maneuver.points.back().hasDelta()) {
            ddelta = std::abs(maneuver.points.back().getDelta() -
                              maneuver.points.front().getDelta());
        }
        const double dtheta = std::abs(runs[i].delta_theta);
        const bool endpoint = (i == 0 || i + 1 == maneuvers_before.size());
        const bool tiny = arc < config.prune.min_arc_length;
        // 现行判据（Δδ）与 ALM 对照判据（Δθ）的分类结果并列转储：
        // 首/末段无论判据量如何均受保护（红线），不参与剔除与重分类
        const char* cls_ddelta =
            endpoint ? "PROTECT"
                     : (tiny ? (ddelta > config.prune.pivot_heading_threshold
                                    ? "PIVOT"
                                    : "DROP")
                             : "NORMAL");
        const char* cls_dtheta =
            endpoint ? "PROTECT"
                     : (tiny ? (dtheta > config.prune.pivot_heading_threshold
                                    ? "PIVOT"
                                    : "DROP")
                             : "NORMAL");
        // 生产实际分类：修剪后同下标 maneuver 的方向（UNKNOWN=剔除标记）
        const char* actual = "KEPT";
        if (maneuvers_after[i].direction == Direction::UNKNOWN) {
            actual = "DROPPED";
            ++dropped;
        } else if (maneuvers_after[i].direction == Direction::PIVOT) {
            actual = "PIVOT";
            ++pivoted;
        }
        std::cout << "[AUDIT-PRUNE] variant=" << variant
                  << " dataset=" << dataset << " index=" << i
                  << " sign=" << runs[i].sign << " arc=" << arc
                  << " ddelta=" << ddelta << " dtheta=" << dtheta
                  << " protect=" << (endpoint ? 1 : 0)
                  << " cls_ddelta=" << cls_ddelta
                  << " cls_dtheta=" << cls_dtheta << " actual=" << actual
                  << std::endl;
    }
    std::cout << "[AUDIT-PRUNE-SUM] variant=" << variant
              << " dataset=" << dataset
              << " runs=" << maneuvers_before.size() << " dropped=" << dropped
              << " pivoted=" << pivoted << " prune_ok=" << (prune_ok ? 1 : 0)
              << std::endl;
}

// 断点 3：阶段二逐轮外层历史与最终门控量测转储
void DumpStageTwo(const std::string& variant, const std::string& dataset,
                  const ApaDdpStageTwoResult& stage_two) {
    for (const auto& round : stage_two.report.history) {
        std::cout << "[AUDIT-S2ROUND] variant=" << variant
                  << " dataset=" << dataset
                  << " round=" << round.outer_index
                  << " w_ref=" << round.tracking_weight << " mu=" << round.mu
                  << " base_cost=" << round.base_cost
                  << " aug_cost=" << round.augmented_cost
                  << " term_pos=" << round.terminal_position_error
                  << " term_head=" << round.terminal_heading_error_deg
                  << " ineq=" << round.max_amplitude_violation
                  << " defect=" << round.defect_norm_inf
                  << " inner_status=" << static_cast<int>(round.inner_status)
                  << " inner_iter=" << round.inner_iterations << std::endl;
    }
    std::cout << "[AUDIT-S2FINAL] variant=" << variant
              << " dataset=" << dataset
              << " status=" << static_cast<int>(stage_two.report.status)
              << " outer=" << stage_two.report.outer_iterations
              << " mu0_cal=" << stage_two.report.mu_initial_calibrated
              << " mu_final=" << stage_two.report.mu_final
              << " term_pos=" << stage_two.report.terminal_position_error
              << " term_head=" << stage_two.report.terminal_heading_error_deg
              << " sign_viol=" << stage_two.max_sign_violation
              << " dwell_viol=" << stage_two.max_dwell_violation
              << " seam_speed=" << stage_two.max_seam_speed
              << " gating_ok=" << (stage_two.gating_ok ? 1 : 0) << std::endl;
}

// 单项校验量测记录：判据名 + 实测值 + 阈值 + 是否通过 + 是否合法性门
// （与生产 validateOutput 的门/指标两层口径一致：门不过即不可输出，
// 指标只记录不否决）
struct CheckMeasurement {
    std::string name;
    double measured;
    double threshold;
    bool passed;
    bool gate;
};

// 断点 4 的 12 项校验判据全量量测（不短路）：对一条候选输出轨迹与产生它
// 的状态/控制序列，按现行校验清单的同一公式逐项求值并转储。cand 标记候选
// 来源（stage2 = 阶段二精化候选，stage1 = 阶段一降级候选），hypo=1 表示
// 该候选在生产语义下不会被输出（求解未收敛的最后迭代点，仅供审计）
std::vector<CheckMeasurement> DumpChecks(
    const std::string& variant, const std::string& dataset,
    const std::string& cand, bool hypothetical, const Trajectory& output,
    const DdpAlignedVec<DdpState>& states,
    const DdpAlignedVec<DdpControl>& controls, const TrajectoryPoint& goal,
    const ESDFMap& esdf_map, const VehicleFootprintModel& footprint_model,
    const DdpPostStageConfig& config, const ApaDdpSolverConfig& solver_config,
    const std::vector<DdpSeamReport>& seams, int input_maneuver_count,
    double input_length) {
    std::vector<CheckMeasurement> checks;
    checks.reserve(17);
    // 与生产 validateOutput 同一分层：合法性门（不过即不可输出）与质量
    // 指标（只记录不否决）分开登记，first_fail 只在门集合内判定
    const auto record_gate = [&checks](const std::string& name, double measured,
                                       double threshold, bool passed) {
        checks.push_back(
            CheckMeasurement{name, measured, threshold, passed, true});
    };
    const auto record_metric = [&checks](const std::string& name,
                                         double measured, double threshold,
                                         bool passed) {
        checks.push_back(
            CheckMeasurement{name, measured, threshold, passed, false});
    };
    // ① 碰撞复检 + ② 终点双指标 + ③ 运动学四残差（与生产同一质量门配置）
    const auto validation =
        output.validate(goal, esdf_map, footprint_model, config.validation);
    record_gate("collision", validation.max_intrusion_depth,
                config.validation.max_collision_depth,
                validation.collision_safe);
    record_gate("terminal_position", validation.terminal_position_error,
                config.validation.max_terminal_position_error,
                validation.terminal_position_ok);
    record_gate("terminal_heading", validation.terminal_heading_error_deg,
                config.validation.max_terminal_heading_error_deg,
                validation.terminal_heading_ok);
    record_gate("kinematic_position",
                validation.max_kinematic_position_residual,
                config.validation.max_kinematic_position_residual,
                validation.max_kinematic_position_residual <=
                    config.validation.max_kinematic_position_residual);
    record_gate("kinematic_heading",
                validation.max_kinematic_heading_residual_deg,
                config.validation.max_kinematic_heading_residual_deg,
                validation.max_kinematic_heading_residual_deg <=
                    config.validation.max_kinematic_heading_residual_deg);
    record_gate("kinematic_velocity",
                validation.max_kinematic_velocity_residual,
                config.validation.max_kinematic_velocity_residual,
                validation.max_kinematic_velocity_residual <=
                    config.validation.max_kinematic_velocity_residual);
    record_gate("kinematic_steer", validation.max_kinematic_steer_residual,
                config.validation.max_kinematic_steer_residual,
                validation.max_kinematic_steer_residual <=
                    config.validation.max_kinematic_steer_residual);
    // 速度残差超标时定位 argmax 点对与上下文（驻留窗边沿 vs 轨迹内部），
    // 供"重定时伪影 vs 真实可行性缺陷"的归因判断
    if (validation.max_kinematic_velocity_residual >
        config.validation.max_kinematic_velocity_residual) {
        double worst_residual = 0.0;
        std::size_t worst_pair = 0;
        for (std::size_t i = 0; i + 1 < output.size(); ++i) {
            const auto& p0 = output[i];
            const auto& p1 = output[i + 1];
            if (!p0.hasT() || !p1.hasT() || !p0.hasV() || !p1.hasV() ||
                !p0.hasA() || !p1.hasA()) {
                continue;
            }
            const double dt = p1.getT() - p0.getT();
            if (!(dt > 0.0) || dt > config.validation.max_kinematic_dt) {
                continue;
            }
            const double residual = std::abs(
                (p1.getV() - p0.getV()) - 0.5 * dt * (p0.getA() + p1.getA()));
            if (residual > worst_residual) {
                worst_residual = residual;
                worst_pair = i;
            }
        }
        const auto& q0 = output[worst_pair];
        const auto& q1 = output[worst_pair + 1];
        std::cout << "[AUDIT-KINV] variant=" << variant
                  << " dataset=" << dataset << " cand=" << cand
                  << " pair=" << worst_pair << "/" << output.size()
                  << " residual=" << worst_residual << " t=" << q0.getT()
                  << " v0=" << q0.getV() << " v1=" << q1.getV()
                  << " a0=" << q0.getA() << " a1=" << q1.getA() << std::endl;
    }
    // ④ 状态幅值复检与 ⑤ 控制盒过冲（专项容差）；同步记录控制过冲的
    // argmax 位置与约束类型（j 或 η），供"边界层尖峰 vs 发散级超限"的
    // 归因判断。幅值门与生产 validateOutput 同一拆分口径：v/a 共用绝对
    // 容差；δ 只在行驶点（|v|≥v_dwell）复检、ω 全点复检，均按相对容差
    double amplitude_violation = 0.0;
    double control_violation = 0.0;
    std::size_t control_worst_index = 0;
    std::string control_worst_name;
    // 分量幅值包络（δ/ω 的相对容差拆分需要各自独立的实测分布）
    double amp_v = 0.0;
    double amp_a = 0.0;
    double amp_delta = 0.0;
    double amp_omega = 0.0;
    // δ 相对容差门的行驶点口径（与生产一致：低速/驻留点的 δ 不产生曲率）
    double amp_delta_rel = 0.0;
    for (std::size_t k = 0; k < states.size(); ++k) {
        const DdpState& x = states[k];
        amp_v = std::max(amp_v, std::abs(x(DDP_IDX_V)) - solver_config.cost.v_max);
        amp_a = std::max(amp_a, std::abs(x(DDP_IDX_A)) - solver_config.cost.a_max);
        amp_delta = std::max(amp_delta,
                             std::abs(x(DDP_IDX_DELTA)) -
                                 solver_config.cost.delta_max);
        amp_omega = std::max(amp_omega,
                             std::abs(x(DDP_IDX_OMEGA)) -
                                 solver_config.cost.omega_max);
        amplitude_violation = std::max({amp_v, amp_a});
        if (std::abs(x(DDP_IDX_V)) >= config.v_dwell) {
            amp_delta_rel =
                std::max(amp_delta_rel,
                         std::abs(x(DDP_IDX_DELTA)) /
                                 solver_config.cost.delta_max -
                             1.0);
        }
        if (k < controls.size()) {
            const double jerk_overshoot = std::abs(controls[k](DDP_IDX_JERK)) -
                                          solver_config.inner.jerk_max;
            const double eta_overshoot = std::abs(controls[k](DDP_IDX_ETA)) -
                                         solver_config.inner.steer_accel_max;
            if (jerk_overshoot > control_violation) {
                control_violation = jerk_overshoot;
                control_worst_index = k;
                control_worst_name = "jerk";
            }
            if (eta_overshoot > control_violation) {
                control_violation = eta_overshoot;
                control_worst_index = k;
                control_worst_name = "eta";
            }
        }
    }
    record_gate("amplitude", amplitude_violation, config.amplitude_check_tol,
                amplitude_violation <= config.amplitude_check_tol);
    // δ/ω 相对容差门（生产 validateOutput 的同名门）：δ 取行驶点相对超限，
    // ω 取全点相对超限
    const double amp_omega_rel =
        amp_omega / solver_config.cost.omega_max;
    record_gate("amplitude_delta", amp_delta_rel,
                config.amplitude_check_rel_tol,
                amp_delta_rel <= config.amplitude_check_rel_tol);
    record_gate("amplitude_omega", amp_omega_rel,
                config.amplitude_check_rel_tol,
                amp_omega_rel <= config.amplitude_check_rel_tol);
    std::cout << "[AUDIT-AMPQ] variant=" << variant << " dataset=" << dataset
              << " cand=" << cand << " v=" << amp_v << " a=" << amp_a
              << " delta=" << amp_delta << " omega=" << amp_omega
              << " delta_rel=" << amp_delta / solver_config.cost.delta_max
              << " omega_rel=" << amp_omega_rel << std::endl;
    record_metric("control_amplitude", control_violation,
                  config.control_overshoot_tol,
                  control_violation <= config.control_overshoot_tol);
    if (control_violation > 0.0) {
        std::cout << "[AUDIT-CTRL] variant=" << variant
                  << " dataset=" << dataset << " cand=" << cand
                  << " type=" << control_worst_name
                  << " node=" << control_worst_index << "/" << controls.size()
                  << " overshoot=" << control_violation << std::endl;
    }
    // ⑥ 接缝与驻留完整性子判据（逐接缝量测取最不利值）
    double max_seam_speed = 0.0;
    double min_dwell_margin = 0.0;
    double max_window_speed = 0.0;
    double max_window_end_omega = 0.0;
    bool first_seam = true;
    for (const auto& seam : seams) {
        max_seam_speed = std::max(max_seam_speed, seam.seam_speed);
        max_window_speed = std::max(max_window_speed, seam.window_max_speed);
        max_window_end_omega =
            std::max(max_window_end_omega, seam.window_end_omega);
        const double dwell_margin = seam.dwell_duration - seam.t_dwell;
        if (first_seam) {
            min_dwell_margin = dwell_margin;
            first_seam = false;
        } else {
            min_dwell_margin = std::min(min_dwell_margin, dwell_margin);
        }
    }
    // 无接缝时四项驻留子判据自然满足（量测记 0）；dwell_resteer 是死判据
    // （构造上 T_dwell=κ_pad·max(T_resteer,T_shift) 且 κ_pad≥1 恒成立），
    // 生产已从校验清单移除，审计同步不再登记
    record_metric("seam_zero_speed", max_seam_speed, config.seam_speed_tol,
                  max_seam_speed <= config.seam_speed_tol);
    record_metric("dwell_duration", first_seam ? 0.0 : -min_dwell_margin, 0.0,
                  first_seam || min_dwell_margin >= 0.0);
    record_metric("dwell_window_speed", max_window_speed,
                  config.v_dwell + config.seam_speed_tol,
                  max_window_speed <= config.v_dwell + config.seam_speed_tol);
    record_metric("dwell_window_end_omega", max_window_end_omega,
                  config.dwell_omega_tol,
                  max_window_end_omega <= config.dwell_omega_tol);
    // ⑦ 输出物理方向段数 vs 输入 A* maneuver 数（不增判据，效果指标）
    const int output_runs = output.countDirectionRuns(
        config.epsilon_v, config.prune.min_arc_length);
    record_metric("maneuver_count", static_cast<double>(output_runs),
                  static_cast<double>(input_maneuver_count),
                  output_runs <= input_maneuver_count);
    // 长度比（路径蠕变探针，与生产同一 1.05 参考口径）
    const double length_ratio =
        input_length > 0.0 ? output.length() / input_length : 1.0;
    record_metric("length_ratio", length_ratio, 1.05, length_ratio <= 1.05);
    for (const auto& check : checks) {
        std::cout << "[AUDIT-CHECK] variant=" << variant
                  << " dataset=" << dataset << " cand=" << cand
                  << " hypo=" << (hypothetical ? 1 : 0)
                  << " layer=" << (check.gate ? "gate" : "metric")
                  << " check=" << check.name << " measured=" << check.measured
                  << " threshold=" << check.threshold
                  << " pass=" << (check.passed ? 1 : 0) << std::endl;
    }
    return checks;
}

// 降级候选的驻留窗计划：在阶段一状态网格上，按修剪后保留的游程序列定位
// 接缝（相邻保留游程的共享边界点），窗口半宽与生产门控计划同一公式
// m_j=⌈max(T_resteer,T_shift)/(2dt)⌉，裁剪到 [0,N]、不跨相邻接缝且与
// 前一窗口互不重叠（与生产 buildStageOneSeamPlans 的裁剪语义保持一致，
// 保证逐接缝驻留插入单调装配）
std::vector<DdpSeamPlan> BuildStageOneSeamPlans(
    const DdpAlignedVec<DdpState>& states,
    const std::vector<std::size_t>& seam_indices, double dt,
    const DdpPostStageConfig& config,
    const PostStageAuditAccessor& post_stage) {
    std::vector<DdpSeamPlan> plans;
    plans.reserve(seam_indices.size());
    std::size_t prev_window_end_plus_one = 0;
    for (std::size_t j = 0; j < seam_indices.size(); ++j) {
        const std::size_t seam = seam_indices[j];
        const double delta_delta =
            post_stage.MeasureSeamDeltaDeltaFromStates(states, seam,
                                                       config.v_dwell);
        const double t_resteer = DdpPostStage::ComputeResteerTime(
            delta_delta, config.omega_max, config.eta_max);
        const auto half = static_cast<std::size_t>(std::ceil(
            std::max(t_resteer, config.shift_delay) / (2.0 * dt) - 1e-9));
        const std::size_t next_bound = (j + 1 < seam_indices.size())
                                           ? seam_indices[j + 1] - 1
                                           : states.size() - 1;
        const std::size_t window_begin =
            std::max(seam > half ? seam - half : 0, prev_window_end_plus_one);
        const std::size_t window_end =
            std::min(std::min(seam + half, states.size() - 1), next_bound);
        prev_window_end_plus_one = window_end + 1;
        plans.push_back(DdpSeamPlan{
            seam, window_begin, window_end, delta_delta, t_resteer,
            config.kappa_pad * std::max(t_resteer, config.shift_delay)});
    }
    return plans;
}

// 首轮内层探针（阶段一失败数据集的根因取证）：精确复现外层第 0 轮的
// 内层求解（λ=0、终点 μ=first_round_mu、幅值 μ=amplitude_mu_initial、
// w_ref=w_ref,0、前端初值启动），转储内层首轮终态的 |δ|/|v| 极值节点
// 分布——「首次越界发生在哪里、朝哪个方向逃逸」是区分参考侧伪影与
// 求解器侧奇异区逃逸的直接证据
void DumpRoundZeroProbe(const std::string& variant, const std::string& dataset,
                        const DdpReference& reference,
                        const ApaDdpSolverConfig& config,
                        const BicycleDynamics& dynamics,
                        const DdpCostEvaluator& cost_evaluator) {
    const std::size_t num_steps = reference.poses.size() - 1;
    MsIlqrSolver inner(config.inner, &dynamics, &cost_evaluator);
    auto multipliers = DdpCostMultiplierState::MakeZero(num_steps);
    multipliers.amplitude_mu.setConstant(config.outer.amplitude_mu_initial);
    multipliers.terminal_mu.setConstant(config.outer.first_round_mu);
    AlOuterLoop outer(config.outer, config.cost);
    const auto exempt_mask = outer.makeAnnealExemptMask(reference);
    DdpCostInput input;
    input.tracking_weight = config.cost.weight_ref_base;
    input.anneal_exempt_mask = &exempt_mask;
    const auto result = inner.solve(reference, multipliers, input,
                                    reference.initial_states,
                                    reference.initial_controls);
    // 极值节点：|δ| 超界（或接近奇异区）与 |v| 超界各取 Top-3 转储
    std::vector<std::pair<double, std::size_t>> delta_abs;
    std::vector<std::pair<double, std::size_t>> v_abs;
    delta_abs.reserve(num_steps + 1);
    v_abs.reserve(num_steps + 1);
    for (std::size_t k = 0; k <= num_steps; ++k) {
        delta_abs.emplace_back(
            std::abs(inner.states()[k](DDP_IDX_DELTA)), k);
        v_abs.emplace_back(std::abs(inner.states()[k](DDP_IDX_V)), k);
    }
    const auto dump_top = [&variant, &dataset, &inner](
                              const char* tag,
                              std::vector<std::pair<double, std::size_t>>& values) {
        std::sort(values.begin(), values.end(),
                  [](const auto& lhs, const auto& rhs) {
                      return lhs.first > rhs.first;
                  });
        for (std::size_t i = 0; i < std::min<std::size_t>(3, values.size());
             ++i) {
            const auto [value, k] = values[i];
            const auto& x = inner.states()[k];
            std::cout << tag << " variant=" << variant
                      << " dataset=" << dataset << " rank=" << i
                      << " node=" << k << " abs_value=" << value
                      << " v=" << x(DDP_IDX_V)
                      << " delta=" << x(DDP_IDX_DELTA)
                      << " x=" << x(DDP_IDX_X) << " y=" << x(DDP_IDX_Y)
                      << " theta=" << x(DDP_IDX_THETA) << std::endl;
        }
    };
    dump_top("[AUDIT-R0DELTA]", delta_abs);
    dump_top("[AUDIT-R0V]", v_abs);
    std::cout << "[AUDIT-R0] variant=" << variant << " dataset=" << dataset
              << " status=" << static_cast<int>(result.status)
              << " iterations=" << result.iterations
              << " initial_cost=" << result.initial_cost
              << " final_cost=" << result.final_cost << std::endl;
}

// 断点 0：参考构建量测。逐 maneuver 元数据与融化临界权重比（平衡式标定
// 的数据源：平滑项 ~w_j·Δs²/T⁵ 与跟踪项 ~w_ref·Δs²·n_pts·dt 同阶正比 Δs²，
// 融化裁决只取决于权重比与该段可用时长，临界比 (w_j/w_ref)*=T⁵·n_pts·dt
// 即「该段被融化所需的最小权重比」）；另转储超物理上限的参考曲率伪影
// （κ=wrap(Δθ)/ds 超过 tan(δ_max)/L 的节点，V 形折点伪影的直接量测）
void DumpReference(const std::string& variant, const std::string& dataset,
                   const DdpReference& reference,
                   const DdpReferenceBuilderConfig& config,
                   double wheelbase, double max_steer_angle) {
    for (std::size_t m = 0; m < reference.maneuvers.size(); ++m) {
        const auto& maneuver = reference.maneuvers[m];
        const std::size_t steps = maneuver.end_index - maneuver.begin_index;
        const double duration = steps * reference.dt;
        const double num_pts = static_cast<double>(steps + 1);
        const double t5 = std::pow(duration, 5.0);
        const bool endpoint =
            (m == 0 || m + 1 == reference.maneuvers.size());
        std::cout << "[AUDIT-REFMAN] variant=" << variant
                  << " dataset=" << dataset << " index=" << m
                  << " sign=" << maneuver.sign << " ds=" << maneuver.delta_s
                  << " dtheta=" << maneuver.delta_theta
                  << " begin=" << maneuver.begin_index
                  << " end=" << maneuver.end_index << " steps=" << steps
                  << " T=" << duration
                  << " crit_ratio=" << t5 * num_pts * reference.dt
                  << " protect=" << (endpoint ? 1 : 0) << std::endl;
    }
    // 参考曲率伪影：物理上限 κ_max = tan(max_steer_angle)/L（车辆参数
    // 真值——配置值只允许收紧到该值以内，不允许放宽，故 cap 与配置无关）；
    // 超阈节点逐点转储，另始终转储全网格 Top-5 κ 节点（未超阈时也能看到
    // 曲率分布的峰值位置）
    (void)config;
    const double kappa_cap = std::tan(max_steer_angle) / wheelbase;
    std::vector<std::pair<double, std::size_t>> kappa_all;
    kappa_all.reserve(reference.poses.size());
    for (std::size_t k = 0; k + 1 < reference.poses.size(); ++k) {
        const double kappa =
            std::abs(WrapAngle(reference.poses[k + 1].theta -
                               reference.poses[k].theta)) /
            reference.ds;
        kappa_all.emplace_back(kappa, k);
    }
    std::sort(kappa_all.begin(), kappa_all.end(),
              [](const auto& lhs, const auto& rhs) {
                  return lhs.first > rhs.first;
              });
    const std::size_t dump_count = std::min<std::size_t>(5, kappa_all.size());
    for (std::size_t i = 0; i < dump_count; ++i) {
        const auto [kappa, k] = kappa_all[i];
        std::size_t owner = reference.maneuvers.size();
        for (std::size_t m = 0; m < reference.maneuvers.size(); ++m) {
            if (k >= reference.maneuvers[m].begin_index &&
                k < reference.maneuvers[m].end_index) {
                owner = m;
                break;
            }
        }
        std::cout << "[AUDIT-REFKAPPA] variant=" << variant
                  << " dataset=" << dataset << " rank=" << i
                  << " node=" << k
                  << " kappa=" << kappa << " kappa_cap=" << kappa_cap
                  << " over=" << (kappa > kappa_cap ? 1 : 0)
                  << " x=" << reference.poses[k].x
                  << " y=" << reference.poses[k].y
                  << " theta=" << reference.poses[k].theta << " maneuver="
                  << (owner < reference.maneuvers.size()
                          ? std::to_string(owner)
                          : "boundary")
                  << std::endl;
    }
}

// 参考重锚迭代探针（L4.3 结构变体的机制验证）：把阶段一解重建为新参考
// 并重解——原参考超物理上限时，跟踪项把解往不可达的急弯上拉、ESDF 罚
// 的平衡残余落在侵入侧；重锚后的参考已是「较可行的」上一轮的解，跟踪
// 压力随之解除，ESDF 有机会把解进一步推出侵入。逐轮转储侵入/终点/
// 违反度/缺陷，验证侵入是否单调下降并能在 2~3 轮内落到 0.02 门内
void DumpReanchorProbe(const std::string& variant, const std::string& dataset,
                       const DdpConfig& config,
                       const VehicleParams& vehicle_params,
                       const ESDFMap& esdf_map,
                       const VehicleFootprintModel& footprint_model,
                       ApaDdpSolver& solver, const DdpPostStage& post_stage,
                       const DdpReferenceBuilder& reference_builder,
                       const ApaDdpStageOneResult& stage_one,
                       const Path& init_path) {
    const auto outer_circles =
        vehicle_circle_geometry::ExtractLocalCircleCenters(
            footprint_model, CircleType::OUTER);
    const double outer_radius = footprint_model.getOuterRadius();
    // 阶段一解的碰撞侵入量测（与生产同一口径：外圆半径 − ESDF 距离的全圆
    // 全点最大值）
    const auto measure_intrusion = [&](const DdpAlignedVec<DdpState>& states) {
        double intrusion = 0.0;
        for (const auto& state : states) {
            const double cos_theta = std::cos(state(DDP_IDX_THETA));
            const double sin_theta = std::sin(state(DDP_IDX_THETA));
            for (const auto& local : outer_circles) {
                const double wx = state(DDP_IDX_X) + local.x() * cos_theta -
                                  local.y() * sin_theta;
                const double wy = state(DDP_IDX_Y) + local.x() * sin_theta +
                                  local.y() * cos_theta;
                intrusion = std::max(
                    intrusion, outer_radius - esdf_map.getDist(wx, wy));
            }
        }
        return intrusion;
    };
    std::cout << "[AUDIT-REANCHOR] variant=" << variant
              << " dataset=" << dataset << " iter=0"
              << " intrusion=" << measure_intrusion(stage_one.states)
              << " term_pos=" << stage_one.report.terminal_position_error
              << " ineq=" << stage_one.report.max_amplitude_violation
              << " defect=" << stage_one.report.defect_norm_inf << std::endl;
    ApaDdpStageOneResult current = stage_one;
    Path anchor_path = init_path;
    for (int iter = 1; iter <= 3; ++iter) {
        const auto runs = post_stage.analyzeSignRuns(current.states);
        auto maneuvers = post_stage.buildManeuvers(current.states, runs);
        if (!post_stage.pruneManeuvers(&maneuvers)) {
            break;
        }
        anchor_path = ReconstructPath(maneuvers);
        const DdpReference new_reference =
            reference_builder.build(anchor_path);
        DdpAlignedVec<DdpState> warm_states;
        DdpAlignedVec<DdpControl> warm_controls;
        post_stage.buildStageTwoWarmStart(anchor_path, new_reference,
                                          &warm_states, &warm_controls);
        current = solver.solveStageOne(new_reference, warm_states,
                                       warm_controls);
        std::cout << "[AUDIT-REANCHOR] variant=" << variant
                  << " dataset=" << dataset << " iter=" << iter
                  << " status=" << static_cast<int>(current.report.status)
                  << " intrusion=" << measure_intrusion(current.states)
                  << " term_pos=" << current.report.terminal_position_error
                  << " term_head=" << current.report.terminal_heading_error_deg
                  << " ineq=" << current.report.max_amplitude_violation
                  << " defect=" << current.report.defect_norm_inf
                  << " outer=" << current.report.outer_iterations << std::endl;
        if (current.report.status != ApaDdpStatus::CONVERGED) {
            break;
        }
    }
}

// 单数据集的完整审计流程（复刻 optimizeDdp 装配，断点处手动编排转储）
void AuditSingle(const std::string& variant, const DatasetCase& dataset,
                 const DdpConfig& variant_config) {
    ::apa::post_processor::OptimizeRequest request;
    if (DataLoader::LoadProtoFromJsonFile(dataset.file, request) !=
        LoadResult::SUCCESS) {
        std::cout << "[AUDIT-SUMMARY] variant=" << variant
                  << " dataset=" << dataset.name
                  << " load_failed=1" << std::endl;
        return;
    }
    const auto vehicle_params = VehicleParams::FromProto(request.vehicle());
    // 与生产同一口径：先按车辆物理参数收紧幅值边界、再同步同源字段
    DdpConfig config = variant_config;
    config.clampToVehicleParams(vehicle_params);
    config.synchronizeAmplitudeBounds();
    const VehicleFootprintModel footprint_model(vehicle_params,
                                                /*heading_sample_num=*/233,
                                                /*inner_row_num=*/2,
                                                config.outer_row_num);
    const auto grid_map = GridMap::FromProto(request.environment());
    const ESDFMap esdf_map(grid_map);
    const auto init_path = Path::FromProto(request.initial_path());
    // L8.4/L8.5：越界查询计数复位（求解结束后随 [AUDIT-OOM] 单次汇总）
    esdf_map.resetOutOfMapQueryCount();
    // 装配与生产同一流程：Reeds-Shepp 换挡点短接（默认关闭时逐位透传）→
    // 参考构建 → 求解组件 → 阶段一全局软化求解
    Path solver_input = init_path;
    if (config.rs_shortcut.cap_ratio > 0.0) {
        solver_input = ShortcutShiftPoints(
            solver_input, esdf_map, footprint_model, vehicle_params.wheelbase,
            config.reference.delta_max, config.rs_shortcut);
    }
    const DdpReferenceBuilder reference_builder(config.reference,
                                                vehicle_params);
    const DdpReference reference = reference_builder.build(solver_input);
    DumpReference(variant, dataset.name, reference, config.reference,
                  vehicle_params.wheelbase, vehicle_params.max_steer_angle);
    const BicycleDynamics dynamics(vehicle_params.wheelbase);
    const DdpEsdfConstraint esdf_constraint(esdf_map, footprint_model,
                                            config.esdf);
    const DdpCostEvaluator cost_evaluator(config.solver.cost, &esdf_constraint);
    // 首轮内层探针：仅在阶段一失败取证时需要（数据集体量小、开销可忽略，
    // 全数据集开启以保留跨数据集对照）
    DumpRoundZeroProbe(variant, dataset.name, reference, config.solver,
                       dynamics, cost_evaluator);
    ApaDdpSolver solver(config.solver, &dynamics, &cost_evaluator);
    const auto stage_one = solver.solveStageOne(reference);
    std::cout << "[AUDIT-S1FINAL] variant=" << variant
              << " dataset=" << dataset.name
              << " status=" << static_cast<int>(stage_one.report.status)
              << " outer=" << stage_one.report.outer_iterations
              << " term_pos=" << stage_one.report.terminal_position_error
              << " term_head=" << stage_one.report.terminal_heading_error_deg
              << " ineq=" << stage_one.report.max_amplitude_violation
              << " defect=" << stage_one.report.defect_norm_inf
              << " mu0_cal=" << stage_one.report.mu_initial_calibrated
              << " mu_final=" << stage_one.report.mu_final << std::endl;
    // L8.4 分项归因：越界查询计数（恢复场覆盖的探测量）+ 定义域守卫
    // 拒绝数（被拦住的越界试探量）
    std::cout << "[AUDIT-OOM] variant=" << variant
              << " dataset=" << dataset.name
              << " out_of_map_queries=" << esdf_map.outOfMapQueryCount()
              << " domain_guard_rejections="
              << stage_one.report.domain_guard_rejections << std::endl;
    PostStageAuditAccessor post_stage(config.post_stage, &reference_builder,
                                      &solver, vehicle_params);
    // 断点 1：阶段一解逐符号游程与 cusp 速度分布（含未收敛的最后迭代点）
    DumpStageOneRuns(variant, dataset.name, post_stage, stage_one.states,
                     reference.dt);
    // 参考重锚迭代探针：仅当阶段一收敛时才有意义（未收敛解不进入重锚）。
    // 探针必须用独立的求解器实例——内层求解器的 µ_m/ρ_reg/QP 活动集跨
    // solve 热启动保持，复用主求解器会污染后续阶段二的热启动状态，使
    // 审计的阶段二解偏离生产（实测 data3 阶段二在探针复用下与生产解不同）
    if (stage_one.report.status == ApaDdpStatus::CONVERGED) {
        ApaDdpSolver probe_solver(config.solver, &dynamics, &cost_evaluator);
        DumpReanchorProbe(variant, dataset.name, config, vehicle_params,
                          esdf_map, footprint_model, probe_solver, post_stage,
                          reference_builder, stage_one, init_path);
    }
    if (stage_one.report.status != ApaDdpStatus::CONVERGED) {
        // 溢出/未收敛取证（N0）：失败轮次的 μ/缺陷/违反度终态、内层
        // ρ_reg 与 µ_m 终值、控制盒饱和度、最后一次内层求解的逐迭代
        // 记录（α/代价/缺陷/回推次数/线搜索试步数）、最坏幅值违反的
        // 类型/节点/数值——判定「逐约束罚 vs 信赖域 vs 罚回退」谁更
        // 对症的分水岭是盒饱和度：饱和度高 ⇒ 信赖域；低而 Hessian
        // 病态 ⇒ 逐约束罚
        const auto& inner = solver.innerSolver();
        const auto& solver_config = solver.config();
        std::size_t clamped_j = 0;
        std::size_t clamped_eta = 0;
        for (const auto& u : inner.controls()) {
            if (std::abs(std::abs(u(DDP_IDX_JERK)) -
                         solver_config.inner.jerk_max) < 1e-9) {
                ++clamped_j;
            }
            if (std::abs(std::abs(u(DDP_IDX_ETA)) -
                         solver_config.inner.steer_accel_max) < 1e-9) {
                ++clamped_eta;
            }
        }
        std::cout << "[AUDIT-S1FAIL] variant=" << variant
                  << " dataset=" << dataset.name
                  << " status=" << static_cast<int>(stage_one.report.status)
                  << " outer=" << stage_one.report.outer_iterations
                  << " restarts=" << stage_one.report.inner_restarts
                  << " rho_reg=" << inner.rhoReg()
                  << " merit_mu=" << inner.meritMu()
                  << " clamped_j=" << clamped_j << "/" << inner.controls().size()
                  << " clamped_eta=" << clamped_eta << "/"
                  << inner.controls().size()
                  << " domain_guard_rejections="
                  << stage_one.report.domain_guard_rejections << std::endl;
        // 最后一次内层求解的逐迭代轨迹（仅接受迭代）：α 是否趋于 0、
        // 回推重试是否密集、代价/缺陷是否还在移动
        for (const auto& record : inner.history()) {
            std::cout << "[AUDIT-S1ITER] variant=" << variant
                      << " dataset=" << dataset.name
                      << " iter=" << record.iteration
                      << " alpha=" << record.alpha
                      << " cost=" << record.cost
                      << " defect=" << record.defect_norm
                      << " merit_mu=" << record.merit_mu
                      << " backward_passes=" << record.backward_passes
                      << " line_search_trials=" << record.line_search_trials
                      << std::endl;
        }
        // 最坏幅值违反定位（五种幅值约束共用同一 max 量测，必须展开
        // 分量才能区分「哪个物理量、在哪个节点」顽固违反）
        double worst_violation = 0.0;
        std::size_t worst_index = 0;
        std::string worst_name;
        double worst_value = 0.0;
        for (std::size_t k = 0; k < stage_one.states.size(); ++k) {
            const auto& x = stage_one.states[k];
            const std::array<std::pair<const char*, double>, 5> checks{{
                {"v", x(DDP_IDX_V) * x(DDP_IDX_V) -
                          solver_config.cost.v_max * solver_config.cost.v_max},
                {"a", x(DDP_IDX_A) * x(DDP_IDX_A) -
                          solver_config.cost.a_max * solver_config.cost.a_max},
                {"omega", x(DDP_IDX_OMEGA) * x(DDP_IDX_OMEGA) -
                              solver_config.cost.omega_max *
                                  solver_config.cost.omega_max},
                {"delta+", x(DDP_IDX_DELTA) - solver_config.cost.delta_max},
                {"delta-", -x(DDP_IDX_DELTA) - solver_config.cost.delta_max},
            }};
            for (const auto& [name, g] : checks) {
                if (g > worst_violation) {
                    worst_violation = g;
                    worst_index = k;
                    worst_name = name;
                }
            }
        }
        if (worst_violation > 0.0) {
            const auto& x = stage_one.states[worst_index];
            worst_value = worst_name[0] == 'v' ? x(DDP_IDX_V)
                          : worst_name[0] == 'a'
                              ? x(DDP_IDX_A)
                              : worst_name[0] == 'o' ? x(DDP_IDX_OMEGA)
                                                     : x(DDP_IDX_DELTA);
            std::cout << "[AUDIT-S1WORST] variant=" << variant
                      << " dataset=" << dataset.name
                      << " constraint=" << worst_name << " node=" << worst_index
                      << "/" << stage_one.states.size()
                      << " violation=" << worst_violation
                      << " state_value=" << worst_value
                      << " v=" << x(DDP_IDX_V) << " pose=(" << x(DDP_IDX_X)
                      << "," << x(DDP_IDX_Y) << "," << x(DDP_IDX_THETA) << ")"
                      << std::endl;
        }
        // 生产语义下后续断点不存在（后处理不得带非收敛解进入修剪/阶段二）
        std::cout << "[AUDIT-SUMMARY] variant=" << variant
                  << " dataset=" << dataset.name
                  << " first_fail=stage_one_convergence" << std::endl;
        return;
    }
    const auto& goal_pt = init_path.back();
    const TrajectoryPoint goal(goal_pt.x, goal_pt.y, goal_pt.theta);
    const int input_maneuver_count =
        reference.maneuvers.empty()
            ? static_cast<int>(init_path.numManeuvers())
            : static_cast<int>(reference.maneuvers.size());
    // 断点 2：符号游程 → maneuver → 修剪对照（判据量/分类/实际去向）
    const auto runs = post_stage.analyzeSignRuns(stage_one.states);
    const auto maneuvers_before =
        post_stage.buildManeuvers(stage_one.states, runs);
    auto maneuvers_after = maneuvers_before;
    const bool prune_ok = post_stage.pruneManeuvers(&maneuvers_after);
    DumpPrune(variant, dataset.name, post_stage, config.post_stage, runs,
              maneuvers_before, maneuvers_after, prune_ok);
    if (!prune_ok) {
        std::cout << "[AUDIT-SUMMARY] variant=" << variant
                  << " dataset=" << dataset.name
                  << " first_fail=pivot" << std::endl;
        return;
    }
    // 生产路径：修剪后重采样 → 门控计划 → 阶段二门控重解（热启动）
    const Path pruned_path = ReconstructPath(maneuvers_after);
    const DdpReference stage_two_reference =
        reference_builder.build(pruned_path);
    const auto gating =
        post_stage.buildGatingPlan(stage_two_reference, pruned_path);
    DdpAlignedVec<DdpState> warm_states;
    DdpAlignedVec<DdpControl> warm_controls;
    post_stage.buildStageTwoWarmStart(pruned_path, stage_two_reference,
                                      &warm_states, &warm_controls);
    const double stage_one_final_weight =
        stage_one.report.history.empty()
            ? solver.config().cost.weight_ref_base
            : stage_one.report.history.back().tracking_weight;
    // 跟踪权重地板与生产 DdpPostStage 同一口径（深退火下阶段一末轮权重
    // 可能远低于精化所需的保持量级，生产钳到地板）——审计镜像漏掉这一步
    // 会让阶段二解与生产不一致（实测 data3 阶段二在地板两侧结论相反）
    const double stage_two_weight =
        std::max(stage_one_final_weight,
                 config.post_stage.stage_two_min_tracking_weight);
    auto stage_two = solver.solveStageTwo(
        stage_two_reference, gating.plan, warm_states, warm_controls,
        stage_two_weight, &stage_one.final_multipliers);
    // 断点 3：阶段二逐轮外层历史与最终门控量测
    DumpStageTwo(variant, dataset.name, stage_two);
    // 断点 4 候选一：阶段二输出 + 驻留插入（未收敛时取最后迭代点并标记
    // hypo=1——生产不会输出，仅供审计判断"未收敛解离合法还有多远"）
    const bool stage_two_hypothetical =
        stage_two.report.status != ApaDdpStatus::CONVERGED;
    std::vector<DdpSeamReport> stage_two_seams;
    const Trajectory stage_two_output =
        post_stage.insertDwells(stage_two.states, stage_two_reference.dt,
                                gating.seams, &stage_two_seams);
    for (const auto& seam : stage_two_seams) {
        std::cout << "[AUDIT-SEAM] variant=" << variant
                  << " dataset=" << dataset.name << " cand=stage2"
                  << " index=" << seam.seam_index
                  << " delta_delta=" << seam.delta_delta
                  << " t_resteer=" << seam.t_resteer
                  << " t_dwell=" << seam.t_dwell
                  << " dwell=" << seam.dwell_duration
                  << " seam_speed=" << seam.seam_speed
                  << " window_max_speed=" << seam.window_max_speed
                  << " window_end_omega=" << seam.window_end_omega << std::endl;
    }
    const auto stage_two_checks =
        DumpChecks(variant, dataset.name, "stage2", stage_two_hypothetical,
                   stage_two_output, stage_two.states, stage_two.controls, goal,
                   esdf_map, footprint_model, config.post_stage,
                   solver.config(), stage_two_seams, input_maneuver_count,
                   init_path.length());
    // 断点 4 候选二：阶段一解 + 修剪 + 驻留插入（分级降级的候选形态）——
    // 接缝取修剪后保留游程的共享边界（阶段一网格），窗口与生产同一定宽
    // 公式；量测同一套 12 项判据，为降级出口提供可输出性证据
    std::vector<std::size_t> kept_boundaries;
    kept_boundaries.reserve(maneuvers_after.size());
    for (std::size_t i = 1; i < maneuvers_after.size(); ++i) {
        if (maneuvers_after[i].direction != Direction::UNKNOWN &&
            maneuvers_after[i - 1].direction != Direction::UNKNOWN) {
            kept_boundaries.push_back(runs[i].begin_index);
        }
    }
    const auto stage_one_seam_plans =
        BuildStageOneSeamPlans(stage_one.states, kept_boundaries,
                               reference.dt, config.post_stage, post_stage);
    std::vector<DdpSeamReport> stage_one_seams;
    const Trajectory stage_one_output = post_stage.insertDwells(
        stage_one.states, reference.dt, stage_one_seam_plans, &stage_one_seams);
    for (const auto& seam : stage_one_seams) {
        std::cout << "[AUDIT-SEAM] variant=" << variant
                  << " dataset=" << dataset.name << " cand=stage1"
                  << " index=" << seam.seam_index
                  << " delta_delta=" << seam.delta_delta
                  << " t_resteer=" << seam.t_resteer
                  << " t_dwell=" << seam.t_dwell
                  << " dwell=" << seam.dwell_duration
                  << " seam_speed=" << seam.seam_speed
                  << " window_max_speed=" << seam.window_max_speed
                  << " window_end_omega=" << seam.window_end_omega << std::endl;
    }
    const auto stage_one_checks = DumpChecks(
        variant, dataset.name, "stage1", false, stage_one_output, stage_one.states,
        stage_one.controls, goal, esdf_map, footprint_model, config.post_stage,
        solver.config(), stage_one_seams, input_maneuver_count,
        init_path.length());
    // 生产判定模拟：候选一不可用时降级——阶段二未收敛则
    // first_fail=stage_two_convergence；阶段二收敛时首个未过的合法性门
    // （质量指标不参与否决，与生产 validateOutput 的门/指标分层一致）
    std::string first_fail;
    if (stage_two.report.status != ApaDdpStatus::CONVERGED) {
        first_fail = "stage_two_convergence";
    } else {
        for (const auto& check : stage_two_checks) {
            if (check.gate && !check.passed) {
                first_fail = check.name;
                break;
            }
        }
    }
    const auto count_failed_gates =
        [](const std::vector<CheckMeasurement>& checks) {
            std::size_t failed = 0;
            for (const auto& check : checks) {
                if (check.gate && !check.passed) {
                    ++failed;
                }
            }
            return failed;
        };
    std::cout << "[AUDIT-SUMMARY] variant=" << variant
              << " dataset=" << dataset.name
              << " first_fail=" << (first_fail.empty() ? "none" : first_fail)
              << " stage2_failed_gates=" << count_failed_gates(stage_two_checks)
              << " stage1_failed_gates=" << count_failed_gates(stage_one_checks)
              << " pruned_maneuvers=" << pruned_path.numManeuvers()
              << " stage2_length=" << stage_two_output.length()
              << " stage1_length=" << stage_one_output.length()
              << " input_length=" << init_path.length() << std::endl;
}
}  // namespace

int main(int argc, char** argv) {
    // 求解过程的结构化日志落到 build/log（与调参工具同一约定，避免污染
    // 仓库根 log/ 目录）；审计量测一律走 stdout 的 [AUDIT] 行
    Logger::SetLogDirectory(std::string(PROJECT_ROOT_DIR) + "/build/log");
    Logger::SetConsoleOutputEnabled(false);
    // 命令行可选指定单个变体名（默认 baseline；"all" 跑全部变体）——
    // 变体矩阵与调参工具共享同一真值来源，四断点转储对每个变体可开启
    const std::string filter = (argc > 1) ? argv[1] : "baseline";
    const auto datasets = ddp_tune::BuildDatasets();
    for (const auto& variant : ddp_tune::BuildVariants()) {
        if (filter != "all" && variant.name != filter) {
            continue;
        }
        for (const auto& dataset : datasets) {
            AuditSingle(variant.name, dataset, variant.config);
        }
    }
    return 0;
}
