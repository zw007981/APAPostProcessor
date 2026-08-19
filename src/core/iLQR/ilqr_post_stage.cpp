#include "ilqr_post_stage.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <optional>
#include <stdexcept>
#include <utility>

namespace apa_post_processor {
iLQRPostStage::iLQRPostStage(const iLQRConfig& config,
                           const iLQRReferenceBuilder* reference_builder,
                           ApaILQRSolver* solver,
                           const VehicleParams& vehicle_params)
    : config_(config),
      reference_builder_(reference_builder),
      solver_(solver),
      vehicle_params_(vehicle_params) {
    // 空指针与非法参数会静默污染全部下游判定，必须在构造期显式拒绝
    if (reference_builder_ == nullptr || solver_ == nullptr) {
        throw std::invalid_argument("iLQRPostStage: 参考构建器与求解器必须非空");
    }
    if (!(config_.post_epsilon_v > 0.0) || !(config_.post_v_dwell > 0.0) ||
        !(config_.post_shift_delay > 0.0) || !(config_.post_kappa_pad >= 1.0) ||
        !(config_.post_omega_max > 0.0) || !(config_.post_eta_max > 0.0) ||
        !(config_.post_seam_speed_tol > 0.0) || !(config_.post_dwell_omega_tol > 0.0) ||
        !(config_.post_amplitude_check_tol > 0.0) ||
        !(config_.post_amplitude_check_rel_tol > 0.0)) {
        throw std::invalid_argument(
            "iLQRPostStage: 滞回/驻留/执行器/容差参数必须为正且 κ_pad>=1");
    }
    if (!(config_.post_prune_min_arc_length > 0.0) ||
        !(config_.post_prune_pivot_heading_threshold > 0.0)) {
        throw std::invalid_argument("iLQRPostStage: 融化/修剪判据阈值必须为正");
    }
    if (!(vehicle_params_.wheelbase > 0.0)) {
        throw std::invalid_argument("iLQRPostStage: 车辆轴距必须为正");
    }
}

double iLQRPostStage::ComputeResteerTime(double delta_delta, double omega_max,
                                        double eta_max) {
    if (!(delta_delta >= 0.0) || !(omega_max > 0.0) || !(eta_max > 0.0)) {
        throw std::invalid_argument(
            "iLQRPostStage: T_resteer 参数必须满足 Δδ>=0、ω_max>0、η_max>0");
    }
    // 双积分 bang-bang：三角剖面峰值角速度 √(η·Δδ)，不饱和时
    if (delta_delta <= omega_max * omega_max / eta_max) {
        return 2.0 * std::sqrt(delta_delta / eta_max);
    }
    return delta_delta / omega_max + omega_max / eta_max;
}

std::vector<iLQRSignRun> iLQRPostStage::analyzeSignRuns(
    const iLQRAlignedVec<iLQRState>& states) const {
    if (states.size() < 2) {
        throw std::invalid_argument("iLQRPostStage: 状态数量必须 >= 2");
    }
    // 滞回状态机：|v|<ε_v 的样本不改变已承诺符号（融化残留的速度涟漪
    // 被过滤），仅当反向突破阈值才闭合当前游程并开启新游程
    std::vector<iLQRSignRun> runs;
    runs.reserve(states.size());  // 最坏每点都换向，上限即状态总数
    std::size_t run_begin = 0;
    int current_sign = 0;
    for (std::size_t k = 0; k < states.size(); ++k) {
        const double v = states[k](ILQR_IDX_V);
        int commit = 0;
        if (v >= config_.post_epsilon_v) {
            commit = 1;
        } else if (v <= -config_.post_epsilon_v) {
            commit = -1;
        }
        if (commit == 0) {
            continue;
        }
        if (current_sign == 0) {
            current_sign = commit;
            continue;
        }
        if (commit != current_sign) {
            // 新游程起点即前游程终点（边界点共享，与 maneuver 元数据同约定）
            runs.push_back(iLQRSignRun{current_sign, run_begin, k, 0.0, 0.0});
            run_begin = k;
            current_sign = commit;
        }
    }
    runs.push_back(
        iLQRSignRun{current_sign, run_begin, states.size() - 1, 0.0, 0.0});
    // 段位移与朝向变化量测（PIVOT/剔除判据与诊断报告的数据来源）
    for (auto& run : runs) {
        double arc = 0.0;
        for (std::size_t k = run.begin_index; k < run.end_index; ++k) {
            arc += std::hypot(states[k + 1](ILQR_IDX_X) - states[k](ILQR_IDX_X),
                              states[k + 1](ILQR_IDX_Y) - states[k](ILQR_IDX_Y));
        }
        run.delta_s = arc;
        run.delta_theta = WrapAngle(states[run.end_index](ILQR_IDX_THETA) -
                                    states[run.begin_index](ILQR_IDX_THETA));
    }
    return runs;
}

std::vector<Maneuver> iLQRPostStage::buildManeuvers(
    const iLQRAlignedVec<iLQRState>& states,
    const std::vector<iLQRSignRun>& runs) const {
    std::vector<Maneuver> maneuvers;
    maneuvers.reserve(runs.size());
    for (const auto& run : runs) {
        std::vector<TrajectoryPoint> points;
        points.reserve(run.end_index - run.begin_index + 1);
        for (std::size_t k = run.begin_index; k <= run.end_index; ++k) {
            TrajectoryPoint point(states[k](ILQR_IDX_X), states[k](ILQR_IDX_Y),
                                  states[k](ILQR_IDX_THETA));
            point.setV(states[k](ILQR_IDX_V));
            point.setA(states[k](ILQR_IDX_A));
            point.setDelta(states[k](ILQR_IDX_DELTA));
            point.setDeltaDot(states[k](ILQR_IDX_OMEGA));
            points.push_back(point);
        }
        // 全程未决（符号 0）的轨迹按前进兜底：不产生换挡语义，
        // 首/末段保护会原样保留它
        const Direction direction =
            run.sign < 0 ? Direction::BACKWARD : Direction::FORWARD;
        maneuvers.emplace_back(std::move(points), direction);
    }
    return maneuvers;
}

bool iLQRPostStage::pruneManeuvers(std::vector<Maneuver>* maneuvers) const {
    if (maneuvers == nullptr || maneuvers->empty()) {
        return true;
    }
    // 自有分类（Δθ 语义，与 MINCO 侧 detectMelting 同一物理含义）：极小
    for (std::size_t i = 1; i + 1 < maneuvers->size(); ++i) {
        auto& maneuver = (*maneuvers)[i];
        if (maneuver.length() >= config_.post_prune_min_arc_length) {
            continue;
        }
        if (maneuver.points.size() < 2) {
            continue;
        }
        const double delta_theta = std::abs(WrapAngle(
            maneuver.points.back().theta - maneuver.points.front().theta));
        if (delta_theta > config_.post_prune_pivot_heading_threshold) {
            maneuver.direction = Direction::PIVOT;
        } else {
            maneuver.direction = Direction::UNKNOWN;
        }
    }
    // PIVOT 即失败：动力学一致解在微弧长游程内的 |Δθ| 受 θ̇=v·tanδ/L
    for (std::size_t i = 1; i + 1 < maneuvers->size(); ++i) {
        if ((*maneuvers)[i].direction == Direction::PIVOT) {
            return false;
        }
    }
    return true;
}

iLQRGatingPlanBuild iLQRPostStage::buildGatingPlan(
    const iLQRReference& stage_two_reference, const Path& pruned_path) const {
    const std::size_t num_poses = stage_two_reference.poses.size();
    if (num_poses < 2) {
        throw std::invalid_argument(
            "iLQRPostStage: 阶段二参考位姿数量必须 >= 2");
    }
    iLQRGatingPlanBuild build;
    iLQRGatingPlan& plan = build.plan;
    plan.sign_gate.assign(num_poses, 0);
    plan.seam_lookup.assign(num_poses, -1);
    plan.dwell_v_cap.assign(num_poses, 0.0);
    // 段内符号门：逐 maneuver 按方向符号覆盖其区间（边界点由接缝清零，
    // 归属歧义随之消除）
    for (const auto& maneuver : stage_two_reference.maneuvers) {
        for (std::size_t k = maneuver.begin_index; k <= maneuver.end_index;
             ++k) {
            plan.sign_gate[k] = maneuver.sign;
        }
    }
    plan.seam_indices = stage_two_reference.cusp_indices;
    build.seams.reserve(plan.seam_indices.size());
    const double dt = stage_two_reference.dt;
    const std::size_t num_seams = plan.seam_indices.size();
    // 前一窗口的右边界+1（窗口裁剪的下界）：驻留插入按窗口顺序单调装配，
    std::size_t prev_window_end_plus_one = 0;
    for (std::size_t j = 0; j < num_seams; ++j) {
        const std::size_t seam = plan.seam_indices[j];
        plan.sign_gate[seam] = 0;
        plan.seam_lookup[seam] = static_cast<int>(j);
        // 逐接缝转向需求（从阶段一输出量测：修剪后路径的点携带阶段一
        // v/δ；接缝 j 即修剪后 maneuver j 与 j+1 的边界）
        const double delta_delta = measureSeamDeltaDelta(pruned_path, j);
        const double t_resteer =
            ComputeResteerTime(delta_delta, config_.post_omega_max, config_.post_eta_max);
        // 窗口半宽 m_j=⌈max(T_resteer,T_shift)/(2dt)⌉：静止窗口时长
        // 不小于重转向需求，优化器才能在窗内排出满足 ω/η 边界的摆动
        const auto half = static_cast<std::size_t>(std::ceil(
            std::max(t_resteer, config_.post_shift_delay) / (2.0 * dt) - 1e-9));
        // 窗口裁剪：不越界、不跨相邻接缝（右端不含下一接缝点）、与前一
        const std::size_t next_bound =
            (j + 1 < num_seams) ? plan.seam_indices[j + 1] - 1 : num_poses - 1;
        const std::size_t window_begin =
            std::max(seam > half ? seam - half : 0, prev_window_end_plus_one);
        const std::size_t window_end =
            std::min(std::min(seam + half, num_poses - 1), next_bound);
        prev_window_end_plus_one = window_end + 1;
        for (std::size_t k = window_begin; k <= window_end; ++k) {
            plan.dwell_v_cap[k] = config_.post_v_dwell;
        }
        build.seams.push_back(iLQRSeamPlan{
            seam, window_begin, window_end, delta_delta, t_resteer,
            config_.post_kappa_pad * std::max(t_resteer, config_.post_shift_delay)});
    }
    return build;
}

double iLQRPostStage::measureSeamDeltaDelta(
    const Path& pruned_path, std::size_t seam_maneuver_index) const {
    const auto& maneuvers = pruned_path.getManeuvers();
    if (seam_maneuver_index + 1 >= maneuvers.size()) {
        throw std::invalid_argument(
            "iLQRPostStage: 接缝对应的 maneuver 下标越界");
    }
    // δ_left/δ_right 取接缝前后最后一个 |v|>v_dwell 采样点处的 δ；
    const auto& left = maneuvers[seam_maneuver_index].points;
    const auto& right = maneuvers[seam_maneuver_index + 1].points;
    double delta_left = left.back().hasDelta() ? left.back().getDelta() : 0.0;
    for (std::size_t i = left.size() - 1; i-- > 0;) {
        if (left[i].hasV() && std::abs(left[i].getV()) > config_.post_v_dwell &&
            left[i].hasDelta()) {
            delta_left = left[i].getDelta();
            break;
        }
    }
    double delta_right =
        right.front().hasDelta() ? right.front().getDelta() : 0.0;
    for (std::size_t i = 1; i < right.size(); ++i) {
        if (right[i].hasV() && std::abs(right[i].getV()) > config_.post_v_dwell &&
            right[i].hasDelta()) {
            delta_right = right[i].getDelta();
            break;
        }
    }
    return std::abs(delta_right - delta_left);
}

void iLQRPostStage::buildStageTwoWarmStart(
    const Path& pruned_path, const iLQRReference& stage_two_reference,
    iLQRAlignedVec<iLQRState>* warm_states,
    iLQRAlignedVec<iLQRControl>* warm_controls) const {
    if (warm_states == nullptr || warm_controls == nullptr) {
        throw std::invalid_argument("iLQRPostStage: 热启动输出指针必须非空");
    }
    const std::size_t num_poses = stage_two_reference.poses.size();
    if (num_poses < 2) {
        throw std::invalid_argument(
            "iLQRPostStage: 阶段二参考位姿数量必须 >= 2");
    }
    // 展平修剪后路径（forEach 剔除 maneuver 间共享边界点）并累积弧长，
    // 与参考构建器的重采样使用同一组插值节点（网格点弧长目标严格一致）
    std::vector<TrajectoryPoint> points;
    points.reserve(pruned_path.size());
    pruned_path.forEach(
        [&points](const TrajectoryPoint& point) { points.push_back(point); });
    if (points.size() < 2) {
        throw std::invalid_argument("iLQRPostStage: 修剪后路径点数必须 >= 2");
    }
    // 状态量数据来源链（隐式契约）：buildManeuvers 全量 set v/a/δ/ω →
    for (std::size_t i = 0; i < points.size(); ++i) {
        if (!points[i].hasV() || !points[i].hasA() || !points[i].hasDelta() ||
            !points[i].hasDeltaDot()) {
            throw std::logic_error(
                "iLQRPostStage: 修剪后路径点缺少阶段一状态量（热启动依赖 "
                "v/a/δ/ω，首个缺失点索引 " +
                std::to_string(i) + "）");
        }
    }
    std::vector<double> arc_length(points.size(), 0.0);
    for (std::size_t i = 1; i < points.size(); ++i) {
        arc_length[i] =
            arc_length[i - 1] + std::hypot(points[i].x - points[i - 1].x,
                                           points[i].y - points[i - 1].y);
    }
    const double total_length = arc_length.back();
    const double ds = stage_two_reference.ds;
    // 状态量查表插值：x/y/θ 取参考位姿本身（重采样已含 wrap 处理），
    // v/a/δ/ω 取阶段一解在相同弧长目标处的线性插值
    warm_states->clear();
    warm_states->reserve(num_poses);
    std::size_t segment = 0;
    for (std::size_t k = 0; k < num_poses; ++k) {
        const double target =
            std::min(static_cast<double>(k) * ds, total_length);
        while (segment + 1 < points.size() &&
               arc_length[segment + 1] < target) {
            ++segment;
        }
        double ratio = 0.0;
        if (segment + 1 < points.size()) {
            const double segment_length =
                arc_length[segment + 1] - arc_length[segment];
            ratio = segment_length > 0.0
                        ? (target - arc_length[segment]) / segment_length
                        : 0.0;
        }
        const auto sample = [&points, segment, ratio](auto getter) {
            const double from = getter(points[segment]);
            const double to =
                getter(points[std::min(segment + 1, points.size() - 1)]);
            return from + (to - from) * ratio;
        };
        iLQRState state;
        const Pose& pose = stage_two_reference.poses[k];
        state << pose.x, pose.y, pose.theta,
            sample([](const TrajectoryPoint& p) { return p.getV(); }),
            sample([](const TrajectoryPoint& p) { return p.getA(); }),
            sample([](const TrajectoryPoint& p) { return p.getDelta(); }),
            sample([](const TrajectoryPoint& p) { return p.getDeltaDot(); });
        warm_states->push_back(state);
    }
    // 控制量由 a/ω 差分反解（半隐式链的近似逆），裁剪进控制盒
    const auto& limits = solver_->config();
    const double dt = stage_two_reference.dt;
    warm_controls->clear();
    warm_controls->reserve(num_poses - 1);
    for (std::size_t k = 0; k + 1 < num_poses; ++k) {
        iLQRControl control;
        control(ILQR_IDX_JERK) = std::clamp(
            ((*warm_states)[k + 1](ILQR_IDX_A) - (*warm_states)[k](ILQR_IDX_A)) /
                dt,
            -limits.inner_jerk_max, limits.inner_jerk_max);
        control(ILQR_IDX_ETA) =
            std::clamp(((*warm_states)[k + 1](ILQR_IDX_OMEGA) -
                        (*warm_states)[k](ILQR_IDX_OMEGA)) /
                           dt,
                       -limits.inner_steer_accel_max,
                       limits.inner_steer_accel_max);
        warm_controls->push_back(control);
    }
}

double iLQRPostStage::MeasureSeamDeltaDeltaFromStates(
    const iLQRAlignedVec<iLQRState>& states, std::size_t seam_index,
    double v_dwell) {
    if (seam_index >= states.size()) {
        throw std::invalid_argument("iLQRPostStage: 接缝索引越界");
    }
    double delta_left = states[seam_index](ILQR_IDX_DELTA);
    for (std::size_t k = seam_index; k-- > 0;) {
        if (std::abs(states[k](ILQR_IDX_V)) > v_dwell) {
            delta_left = states[k](ILQR_IDX_DELTA);
            break;
        }
    }
    double delta_right = states[seam_index](ILQR_IDX_DELTA);
    for (std::size_t k = seam_index + 1; k < states.size(); ++k) {
        if (std::abs(states[k](ILQR_IDX_V)) > v_dwell) {
            delta_right = states[k](ILQR_IDX_DELTA);
            break;
        }
    }
    return std::abs(delta_right - delta_left);
}

std::vector<std::size_t> iLQRPostStage::collectKeptSeamIndices(
    const std::vector<iLQRSignRun>& runs,
    const std::vector<Maneuver>& maneuvers) const {
    if (runs.size() != maneuvers.size()) {
        throw std::invalid_argument(
            "iLQRPostStage: 游程与 maneuver 数量必须一致");
    }
    std::vector<std::size_t> seam_indices;
    seam_indices.reserve(maneuvers.size());
    // 相邻游程共享边界点（后一游程的 begin_index）；两侧游程均保留时
    for (std::size_t i = 1; i < maneuvers.size(); ++i) {
        if (maneuvers[i].direction == Direction::UNKNOWN ||
            maneuvers[i - 1].direction == Direction::UNKNOWN) {
            continue;
        }
        seam_indices.push_back(runs[i].begin_index);
    }
    return seam_indices;
}

std::vector<iLQRSeamPlan> iLQRPostStage::buildStageOneSeamPlans(
    const iLQRAlignedVec<iLQRState>& states,
    const std::vector<std::size_t>& seam_indices, double dt) const {
    if (states.size() < 2 || !(dt > 0.0)) {
        throw std::invalid_argument(
            "iLQRPostStage: 状态数量必须 >= 2 且 dt 为正");
    }
    std::vector<iLQRSeamPlan> plans;
    plans.reserve(seam_indices.size());
    // 窗口定宽公式与门控计划同源：m_j=⌈max(T_resteer,T_shift)/(2dt)⌉，
    std::size_t prev_window_end_plus_one = 0;
    for (std::size_t j = 0; j < seam_indices.size(); ++j) {
        const std::size_t seam = seam_indices[j];
        if (seam >= states.size()) {
            throw std::invalid_argument("iLQRPostStage: 接缝索引越界");
        }
        const double delta_delta =
            MeasureSeamDeltaDeltaFromStates(states, seam, config_.post_v_dwell);
        const double t_resteer =
            ComputeResteerTime(delta_delta, config_.post_omega_max, config_.post_eta_max);
        const auto half = static_cast<std::size_t>(std::ceil(
            std::max(t_resteer, config_.post_shift_delay) / (2.0 * dt) - 1e-9));
        const std::size_t next_bound = (j + 1 < seam_indices.size())
                                           ? seam_indices[j + 1] - 1
                                           : states.size() - 1;
        const std::size_t window_begin =
            std::max(seam > half ? seam - half : 0, prev_window_end_plus_one);
        const std::size_t window_end =
            std::min(std::min(seam + half, states.size() - 1), next_bound);
        prev_window_end_plus_one = window_end + 1;
        plans.push_back(iLQRSeamPlan{
            seam, window_begin, window_end, delta_delta, t_resteer,
            config_.post_kappa_pad * std::max(t_resteer, config_.post_shift_delay)});
    }
    return plans;
}

TrajectoryPoint iLQRPostStage::stateToPoint(const iLQRState& x, double t) const {
    // δ 取自收敛解的状态（已过幅值门），tanδ 必有限；断言防御未来
    // 调用方把未收敛/注入状态直接传入
    assert(std::isfinite(x(ILQR_IDX_DELTA)));
    TrajectoryPoint point(x(ILQR_IDX_X), x(ILQR_IDX_Y), x(ILQR_IDX_THETA));
    point.setV(x(ILQR_IDX_V));
    point.setA(x(ILQR_IDX_A));
    point.setDelta(x(ILQR_IDX_DELTA));
    point.setDeltaDot(x(ILQR_IDX_OMEGA));
    // κ=tanδ/L：与 θ̇=v·κ 的运动学关系自洽（含 v 变号）
    point.setKappa(std::tan(x(ILQR_IDX_DELTA)) / vehicle_params_.wheelbase);
    point.setT(t);
    return point;
}

Trajectory iLQRPostStage::insertDwells(
    const iLQRAlignedVec<iLQRState>& states, double dt,
    const std::vector<iLQRSeamPlan>& seams,
    std::vector<iLQRSeamReport>* reports) const {
    if (states.size() < 2 || !(dt > 0.0)) {
        throw std::invalid_argument(
            "iLQRPostStage: 状态数量必须 >= 2 且 dt 为正");
    }
    if (reports == nullptr) {
        throw std::invalid_argument("iLQRPostStage: 接缝报告输出指针必须非空");
    }
    reports->clear();
    reports->reserve(seams.size());
    // 量测/计划阶段：逐接缝重测 Δδ_j、计算 T_dwell 与拉伸量，并生成
    // 校验⑤所需的全部量测（驻留插入前先算好全部时间增量，便于一次装配）
    std::vector<iLQRDwellEdit> edits;
    edits.reserve(seams.size());
    for (const auto& seam : seams) {
        if (seam.window_begin > seam.seam_index ||
            seam.window_end < seam.seam_index ||
            seam.window_end >= states.size()) {
            throw std::invalid_argument(
                "iLQRPostStage: 驻留窗边界必须包含接缝且不越界");
        }
        iLQRSeamReport report;
        report.seam_index = seam.seam_index;
        // Δδ_j 以阶段二最终轨迹重测（与窗口定宽的阶段一量测相互独立）
        report.delta_delta = MeasureSeamDeltaDeltaFromStates(
            states, seam.seam_index, config_.post_v_dwell);
        report.t_resteer = ComputeResteerTime(
            report.delta_delta, config_.post_omega_max, config_.post_eta_max);
        report.t_dwell =
            config_.post_kappa_pad * std::max(report.t_resteer, config_.post_shift_delay);
        report.seam_speed = std::abs(states[seam.seam_index](ILQR_IDX_V));
        double window_speed = 0.0;
        for (std::size_t k = seam.window_begin; k <= seam.window_end; ++k) {
            window_speed =
                std::max(window_speed, std::abs(states[k](ILQR_IDX_V)));
        }
        report.window_max_speed = window_speed;
        report.window_end_omega =
            std::max(std::abs(states[seam.window_begin](ILQR_IDX_OMEGA)),
                     std::abs(states[seam.window_end](ILQR_IDX_OMEGA)));
        const double window_duration =
            static_cast<double>(seam.window_end - seam.window_begin) * dt;
        // 退化守卫：接缝相邻到窗口被裁剪成单点（时长为零）时不做拉伸，
        // 驻留完整性校验会因时长不足判失败并回退（绝不强行 padding）
        if (window_duration <= 0.0) {
            report.dwell_duration = 0.0;
            reports->push_back(report);
            edits.push_back(
                iLQRDwellEdit{seam.window_begin, seam.window_end, 0.0});
            continue;
        }
        // 驻留时长量化到 dt 整数倍（输出网格均匀）：不超过窗口原长时
        // 无需拉伸（窗口已覆盖驻留需求）
        const auto intervals =
            static_cast<std::size_t>(std::ceil(report.t_dwell / dt - 1e-9));
        const double stretched =
            std::max(window_duration, static_cast<double>(intervals) * dt);
        report.dwell_duration = stretched;
        reports->push_back(report);
        edits.push_back(
            iLQRDwellEdit{seam.window_begin, seam.window_end, stretched});
    }
    // 装配阶段：一次遍历完成窗外转化/窗内重定时/时间戳平移
    return assembleRetimedTrajectory(states, dt, edits);
}

Trajectory iLQRPostStage::assembleRetimedTrajectory(
    const iLQRAlignedVec<iLQRState>& states, double dt,
    const std::vector<iLQRDwellEdit>& edits) const {
    // 装配输出轨迹：接缝窗外按状态原样转化（时间戳随前方窗口拉伸量平移），
    std::vector<TrajectoryPoint> points;
    points.reserve(states.size() +
                   edits.size() *
                       (static_cast<std::size_t>(std::ceil(
                            config_.post_kappa_pad * config_.post_shift_delay / dt)) +
                        1));
    std::size_t cursor = 0;
    double time_offset = 0.0;
    for (std::size_t j = 0; j < edits.size(); ++j) {
        const auto& edit = edits[j];
        for (; cursor < edit.window_begin; ++cursor) {
            points.push_back(
                stateToPoint(states[cursor], cursor * dt + time_offset));
        }
        const double window_duration =
            static_cast<double>(edit.window_end - edit.window_begin) * dt;
        const double t_begin = edit.window_begin * dt + time_offset;
        if (edit.stretched_duration <= 0.0) {
            // 退化窗口（单点）：原样透传不做时间拉伸；time_offset 在此
            for (; cursor <= edit.window_end; ++cursor) {
                points.push_back(
                    stateToPoint(states[cursor], cursor * dt + time_offset));
            }
            continue;
        }
        // 边缘斜坡重定时：ρ(τ) 在 [0,R] 从 1 线性升到 r、平台区恒 r、
        const double stretched = edit.stretched_duration;
        const double ramp = std::min(2.0 * dt, 0.5 * window_duration);
        const double ratio = (window_duration - ramp > 1e-9)
                                 ? (stretched - ramp) / (window_duration - ramp)
                                 : stretched / window_duration;
        // σ 的分段常数：σ(R)=R(1+r)/2；σ(W−R)=σ(R)+r(W−2R)
        const double sigma_ramp = ramp * (1.0 + ratio) / 2.0;
        const double sigma_plateau_end =
            sigma_ramp + ratio * (window_duration - 2.0 * ramp);
        // 逆映射 τ=σ⁻¹(t)：t∈[0,σ(R)] 解二次（斜坡区），平台区线性，
        // 末端对称二次；r→1 时退化为 τ=t
        const auto inverse_map = [&](double t) -> std::pair<double, double> {
            if (ratio - 1.0 < 1e-9) {
                return {t, 1.0};
            }
            if (t <= sigma_ramp) {
                // t = τ + (r−1)τ²/(2R) ⇒ τ = R(√(1+2(r−1)t/R)−1)/(r−1)
                const double tau =
                    ramp *
                    (std::sqrt(1.0 + 2.0 * (ratio - 1.0) * t / ramp) - 1.0) /
                    (ratio - 1.0);
                return {tau, 1.0 + (ratio - 1.0) * tau / ramp};
            }
            if (t <= sigma_plateau_end) {
                return {ramp + (t - sigma_ramp) / ratio, ratio};
            }
            // 末端斜坡：t = σ(W−R) + r·u − (r−1)u²/(2R)，u∈[0,R]，取小根
            const double s_end = sigma_plateau_end;
            const double discriminant =
                ratio * ratio - 2.0 * (ratio - 1.0) * (t - s_end) / ramp;
            const double u = ramp *
                             (ratio - std::sqrt(std::max(discriminant, 0.0))) /
                             (ratio - 1.0);
            // 末样本的时间戳量化可超出 σ(W) 最多半个采样步，钳制回窗内
            const double u_clamped = std::min(std::max(u, 0.0), ramp);
            return {window_duration - ramp + u_clamped,
                    ratio - (ratio - 1.0) * u_clamped / ramp};
        };
        const auto intervals =
            static_cast<std::size_t>(std::lround(stretched / dt));
        for (std::size_t i = 0; i <= intervals; ++i) {
            // 新采样时刻 t → 旧窗口时刻 τ 与局部倍率 ρ(τ)；位姿/朝向/前轮
            // 转角按原剖面取值，v/ω/a 随局部时间倍率同比缩放
            const auto [tau, rho] = inverse_map(i * dt);
            const double position = tau / dt;
            const auto base =
                std::min<std::size_t>(static_cast<std::size_t>(position),
                                      edit.window_end - edit.window_begin - 1);
            const double frac = position - static_cast<double>(base);
            const iLQRState& from = states[edit.window_begin + base];
            const iLQRState& to = states[edit.window_begin + base + 1];
            iLQRState sample = from + (to - from) * frac;
            sample(ILQR_IDX_THETA) =
                from(ILQR_IDX_THETA) +
                WrapAngle(to(ILQR_IDX_THETA) - from(ILQR_IDX_THETA)) * frac;
            sample(ILQR_IDX_V) /= rho;
            sample(ILQR_IDX_A) /= rho * rho;
            sample(ILQR_IDX_OMEGA) /= rho;
            points.push_back(stateToPoint(sample, t_begin + i * dt));
        }
        time_offset += stretched - window_duration;
        cursor = edit.window_end + 1;
    }
    for (; cursor < states.size(); ++cursor) {
        points.push_back(
            stateToPoint(states[cursor], cursor * dt + time_offset));
    }
    return Trajectory(std::move(points));
}

bool iLQRPostStage::validateOutput(const Trajectory& output,
                                  const iLQRAlignedVec<iLQRState>& states,
                                  const iLQRAlignedVec<iLQRControl>& controls,
                                  double input_length,
                                  const TrajectoryPoint& goal,
                                  const ESDFMap& esdf_map,
                                  const VehicleFootprintModel& footprint_model,
                                  iLQRPostStageDiagnostics* diagnostics) const {
    if (diagnostics == nullptr) {
        throw std::invalid_argument("iLQRPostStage: 诊断输出指针必须非空");
    }
    diagnostics->gate_checks.clear();
    diagnostics->metric_checks.clear();
    const auto record_gate = [diagnostics](const std::string& name,
                                           double measured, double threshold) {
        diagnostics->gate_checks.push_back(iLQRCheckMeasurement{
            name, measured, threshold, measured <= threshold});
    };
    const auto record_metric = [diagnostics](const std::string& name,
                                             double measured, double threshold,
                                             bool passed) {
        diagnostics->metric_checks.push_back(
            iLQRCheckMeasurement{name, measured, threshold, passed});
    };
    // ==================== 合法性门（任一项不过即不可输出，不得放宽）====
    const auto validation =
        output.validate(goal, esdf_map, footprint_model, config_.validation);
    record_gate("collision", validation.max_intrusion_depth,
                config_.validation.max_collision_depth);
    record_gate("terminal_position", validation.terminal_position_error,
                config_.validation.max_terminal_position_error);
    record_gate("terminal_heading", validation.terminal_heading_error_deg,
                config_.validation.max_terminal_heading_error_deg);
    record_gate("kinematic_position",
                validation.max_kinematic_position_residual,
                config_.validation.max_kinematic_position_residual);
    record_gate("kinematic_heading",
                validation.max_kinematic_heading_residual_deg,
                config_.validation.max_kinematic_heading_residual_deg);
    record_gate("kinematic_velocity",
                validation.max_kinematic_velocity_residual,
                config_.validation.max_kinematic_velocity_residual);
    record_gate("kinematic_steer", validation.max_kinematic_steer_residual,
                config_.validation.max_kinematic_steer_residual);
    // 状态幅值复检（按量分设容差，输出轨迹契约直接消费的量）：
    const auto& solver_config = solver_->config();
    double amp_va = 0.0;
    double amp_delta = 0.0;
    double amp_omega = 0.0;
    for (const auto& x : states) {
        amp_va =
            std::max(amp_va, std::abs(x(ILQR_IDX_V)) - solver_config.cost_v_max);
        amp_va =
            std::max(amp_va, std::abs(x(ILQR_IDX_A)) - solver_config.cost_a_max);
        if (std::abs(x(ILQR_IDX_V)) >= config_.post_v_dwell) {
            amp_delta =
                std::max(amp_delta, std::abs(x(ILQR_IDX_DELTA)) /
                                            solver_config.cost_delta_max -
                                        1.0);
        }
        amp_omega = std::max(
            amp_omega,
            std::abs(x(ILQR_IDX_OMEGA)) / solver_config.cost_omega_max - 1.0);
    }
    record_gate("amplitude", amp_va, config_.post_amplitude_check_tol);
    record_gate("amplitude_delta", amp_delta, config_.post_amplitude_check_rel_tol);
    record_gate("amplitude_omega", amp_omega, config_.post_amplitude_check_rel_tol);
    // ==================== 质量指标（全量记录，不作为回退触发条件）====
    double control_violation = 0.0;
    for (const auto& u : controls) {
        control_violation =
            std::max(control_violation,
                     std::abs(u(ILQR_IDX_JERK)) - solver_config.inner_jerk_max);
        control_violation = std::max(
            control_violation,
            std::abs(u(ILQR_IDX_ETA)) - solver_config.inner_steer_accel_max);
    }
    record_metric("control_amplitude", control_violation,
                  config_.post_control_overshoot_tol,
                  control_violation <= config_.post_control_overshoot_tol);
    // 接缝与驻留完整性子项：阶段二收敛解由门控结构保证（接缝等式/
    double max_seam_speed = 0.0;
    double min_dwell_margin = 0.0;
    double max_window_speed = 0.0;
    double max_window_end_omega = 0.0;
    bool first_seam = true;
    for (const auto& seam : diagnostics->seams) {
        max_seam_speed = std::max(max_seam_speed, seam.seam_speed);
        max_window_speed = std::max(max_window_speed, seam.window_max_speed);
        max_window_end_omega =
            std::max(max_window_end_omega, seam.window_end_omega);
        const double dwell_margin = seam.dwell_duration - seam.t_dwell;
        min_dwell_margin = first_seam
                               ? dwell_margin
                               : std::min(min_dwell_margin, dwell_margin);
        first_seam = false;
    }
    const bool has_seam = !diagnostics->seams.empty();
    record_metric("seam_zero_speed", max_seam_speed, config_.post_seam_speed_tol,
                  max_seam_speed <= config_.post_seam_speed_tol);
    record_metric("dwell_duration", has_seam ? -min_dwell_margin : 0.0, 0.0,
                  !has_seam || min_dwell_margin >= 0.0);
    record_metric("dwell_window_speed", max_window_speed,
                  config_.post_v_dwell + config_.post_seam_speed_tol,
                  max_window_speed <= config_.post_v_dwell + config_.post_seam_speed_tol);
    record_metric("dwell_window_end_omega", max_window_end_omega,
                  config_.post_dwell_omega_tol,
                  max_window_end_omega <= config_.post_dwell_omega_tol);
    // maneuver 数不增（效果指标，记录供方案比较；曾作为合法性门存在
    // 逻辑倒挂——触发时回退目标的 maneuver 数只会更多）
    diagnostics->output_maneuver_count = output.countDirectionRuns(
        config_.post_epsilon_v, config_.post_prune_min_arc_length);
    record_metric("maneuver_count",
                  static_cast<double>(diagnostics->output_maneuver_count),
                  static_cast<double>(diagnostics->input_maneuver_count),
                  diagnostics->output_maneuver_count <=
                      diagnostics->input_maneuver_count);
    // 长度比（路径蠕变探针；1.05 与端到端验收口径一致）
    const double length_ratio =
        input_length > 0.0 ? output.length() / input_length : 1.0;
    record_metric("length_ratio", length_ratio, 1.05, length_ratio <= 1.05);
    // 门结论：首个未过门项填入诊断（全量量测已记录，不短路）
    for (const auto& check : diagnostics->gate_checks) {
        if (!check.passed) {
            diagnostics->failed_check = check.name;
            diagnostics->measured_value = check.measured;
            diagnostics->threshold = check.threshold;
            return false;
        }
    }
    diagnostics->failed_check.clear();
    return true;
}

void iLQRPostStage::makeFallback(iLQRPostStageResult* result,
                                iLQRPostStageStatus status,
                                const std::string& failed_check,
                                double measured, double threshold,
                                const Path& original_path) const {
    result->status = status;
    result->used_fallback = true;
    result->diagnostics.failed_check = failed_check;
    result->diagnostics.measured_value = measured;
    result->diagnostics.threshold = threshold;
    // 回退轨迹：原始 A* 路径经梯形加减速时间参数化补全为可执行轨迹
    // （与生产模块的兜底语义一致，绝不输出半成品轨迹）
    result->trajectory = Trajectory(original_path, vehicle_params_);
}

iLQRPostStageResult iLQRPostStage::run(
    const Path& original_path, const iLQRReference& stage_one_reference,
    const ApaILQRStageOneResult& stage_one_result, const TrajectoryPoint& goal,
    const ESDFMap& esdf_map, const VehicleFootprintModel& footprint_model) {
    iLQRPostStageResult result;
    result.diagnostics.input_maneuver_count =
        stage_one_reference.maneuvers.empty()
            ? static_cast<int>(original_path.numManeuvers())
            : static_cast<int>(stage_one_reference.maneuvers.size());
    // 阶段一未收敛不得带病后处理（融化/修剪/门控均建立在收敛解之上，
    // 此时没有任何候选可评估）
    if (stage_one_result.report.status != ApaILQRStatus::CONVERGED) {
        makeFallback(&result, iLQRPostStageStatus::STAGE_ONE_NOT_CONVERGED,
                     "stage_one_convergence",
                     static_cast<double>(stage_one_result.report.status),
                     static_cast<double>(ApaILQRStatus::CONVERGED),
                     original_path);
        return result;
    }
    // 步骤 1+2：符号游程分析与拓扑修剪（PIVOT 即失败：检出说明解携带
    // 未愈合缺陷，两个候选都由该解构造，不存在可输出的候选）
    const auto runs = analyzeSignRuns(stage_one_result.states);
    auto maneuvers = buildManeuvers(stage_one_result.states, runs);
    if (!pruneManeuvers(&maneuvers)) {
        makeFallback(&result, iLQRPostStageStatus::PIVOT_DETECTED, "pivot",
                     config_.post_prune_pivot_heading_threshold,
                     config_.post_prune_pivot_heading_threshold, original_path);
        return result;
    }
    // 阶段一降级候选（阶段一解 + 修剪 + 驻留插入）的统一构造与
    // 验证：门控开启时前置调用判断能否跳过阶段二，否则沿用原顺序
    // 在阶段二失败后作为降级回退
    struct StageOneCandidate {
        // 构造条件是否满足（状态数 >=2 且参考 dt 合法）
        bool available{false};
        // 构造且通过合法性门时的输出轨迹（未过门则无值）
        std::optional<Trajectory> trajectory;
    };
    const auto build_stage_one_candidate = [&]() {
        StageOneCandidate cand;
        if (stage_one_result.states.size() < 2 ||
            stage_one_reference.dt <= 0.0) {
            return cand;
        }
        cand.available = true;
        const auto seam_indices = collectKeptSeamIndices(runs, maneuvers);
        const auto seam_plans = buildStageOneSeamPlans(
            stage_one_result.states, seam_indices, stage_one_reference.dt);
        Trajectory output =
            insertDwells(stage_one_result.states, stage_one_reference.dt,
                         seam_plans, &result.diagnostics.seams);
        if (validateOutput(output, stage_one_result.states,
                           stage_one_result.controls, original_path.length(),
                           goal, esdf_map, footprint_model,
                           &result.diagnostics)) {
            cand.trajectory = std::move(output);
        }
        return cand;
    };
    // 阶段一末轮跟踪权重：融化调度深退火后的实际量，阶段二精化
    // 的驱动力来源，退火到地板后精化已无可释放的驱动力
    const double stage_one_final_weight =
        stage_one_result.report.history.empty()
            ? solver_->config().cost_weight_ref_base
            : stage_one_result.report.history.back().tracking_weight;
    // 权重耗尽门控：末轮权重已到地板时阶段二精化无驱动力，直接
    // 输出阶段一候选；候选未过合法性门仍须进入阶段二兜底
    const bool weight_skip =
        config_.post_skip_stage_two_when_weight_exhausted &&
        stage_one_final_weight <= config_.post_stage_two_min_tracking_weight;
    if (weight_skip) {
        auto candidate = build_stage_one_candidate();
        if (candidate.trajectory.has_value()) {
            result.status = iLQRPostStageStatus::SUCCESS_STAGE_ONE_ONLY;
            result.used_fallback = false;
            result.trajectory = std::move(*candidate.trajectory);
            return result;
        }
        // 候选不过门：进入阶段二与原有降级链兜底
    }
    // ============ 候选一：阶段二门控精化（必须重解，禁止直接拼接）====
    const Path pruned_path = ReconstructPath(maneuvers);
    iLQRReference stage_two_reference;
    bool stage_two_reference_ok = true;
    try {
        stage_two_reference = reference_builder_->build(pruned_path);
    } catch (const std::invalid_argument&) {
        // 退化诊断精细化：区分"点数不足"与"弧长不足一个重采样间距"
        // 两种子情形（后者典型为融化过度把有用段也压没），供人工排障
        stage_two_reference_ok = false;
        result.diagnostics.degraded_reason = "pruned_path_degenerate";
        if (pruned_path.size() < 2) {
            result.diagnostics.failed_check = "pruned_points";
            result.diagnostics.measured_value =
                static_cast<double>(pruned_path.size());
            result.diagnostics.threshold = 2.0;
        } else {
            result.diagnostics.failed_check = "pruned_length";
            result.diagnostics.measured_value = pruned_path.length();
            result.diagnostics.threshold = reference_builder_->sampleDist();
        }
    }
    if (stage_two_reference_ok) {
        const auto gating = buildGatingPlan(stage_two_reference, pruned_path);
        // 以阶段一解在修剪后网格上的映射热启动（而非前端初值提取的 bang
        iLQRAlignedVec<iLQRState> warm_states;
        iLQRAlignedVec<iLQRControl> warm_controls;
        buildStageTwoWarmStart(pruned_path, stage_two_reference, &warm_states,
                               &warm_controls);
        // 跟踪权重地板（默认 0 = 不启用）：深退火调度下阶段一末轮权重
        // 可能远低于精化所需的保持量级，钳到地板避免门控失稳
        const double stage_two_weight = std::max(
            stage_one_final_weight, config_.post_stage_two_min_tracking_weight);
        auto stage_two = solver_->solveStageTwo(
            stage_two_reference, gating.plan, warm_states, warm_controls,
            stage_two_weight, &stage_one_result.final_multipliers);
        if (stage_two.report.status == ApaILQRStatus::CONVERGED) {
            // 驻留插入（逐接缝时间拉伸 + 时间戳线性重排）后过合法性门
            Trajectory stage_two_output =
                insertDwells(stage_two.states, stage_two_reference.dt,
                             gating.seams, &result.diagnostics.seams);
            result.stage_two = std::move(stage_two);
            if (validateOutput(stage_two_output, result.stage_two->states,
                               result.stage_two->controls,
                               original_path.length(), goal, esdf_map,
                               footprint_model, &result.diagnostics)) {
                result.status = iLQRPostStageStatus::SUCCESS;
                result.used_fallback = false;
                result.trajectory = std::move(stage_two_output);
                return result;
            }
            // 阶段二解不过门：记录失败门项后继续尝试降级候选
            result.diagnostics.degraded_reason =
                "stage_two_gate:" + result.diagnostics.failed_check;
        } else {
            result.stage_two = std::move(stage_two);
            result.diagnostics.degraded_reason = "stage_two_convergence";
        }
    }
    // ============ 候选二：阶段一降级（阶段一解 + 修剪 + 驻留插入）====
    const auto candidate = build_stage_one_candidate();
    if (candidate.trajectory.has_value()) {
        result.status = iLQRPostStageStatus::SUCCESS_STAGE_ONE_ONLY;
        result.used_fallback = false;
        result.trajectory = std::move(*candidate.trajectory);
        return result;
    }
    if (candidate.available) {
        // 降级候选也不过门：两个候选均失败，以降级候选的失败门项回退
        makeFallback(&result, iLQRPostStageStatus::VALIDATION_FAILED,
                     result.diagnostics.failed_check,
                     result.diagnostics.measured_value,
                     result.diagnostics.threshold, original_path);
        return result;
    }
    // 降级候选不可用（状态不足或参考 dt 非法，仅异常注入场景可达）：
    // 以候选一的失败原因回退
    if (result.diagnostics.degraded_reason == "pruned_path_degenerate") {
        makeFallback(&result, iLQRPostStageStatus::PRUNED_PATH_DEGENERATE,
                     result.diagnostics.failed_check,
                     result.diagnostics.measured_value,
                     result.diagnostics.threshold, original_path);
        return result;
    }
    if (result.diagnostics.degraded_reason == "stage_two_convergence") {
        makeFallback(&result, iLQRPostStageStatus::STAGE_TWO_NOT_CONVERGED,
                     "stage_two_convergence",
                     static_cast<double>(result.stage_two->report.status),
                     static_cast<double>(ApaILQRStatus::CONVERGED),
                     original_path);
        return result;
    }
    // 候选一已过门是不可能的（过门即已返回），剩余情形为候选一门不过
    makeFallback(&result, iLQRPostStageStatus::VALIDATION_FAILED,
                 result.diagnostics.failed_check,
                 result.diagnostics.measured_value,
                 result.diagnostics.threshold, original_path);
    return result;
}
}  // namespace apa_post_processor
