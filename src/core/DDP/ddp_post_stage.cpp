#include "ddp_post_stage.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace apa_post_processor {
DdpPostStage::DdpPostStage(DdpPostStageConfig config,
                           const DdpReferenceBuilder* reference_builder,
                           ApaDdpSolver* solver,
                           const VehicleParams& vehicle_params)
    : config_(config),
      reference_builder_(reference_builder),
      solver_(solver),
      vehicle_params_(vehicle_params) {
    // 空指针与非法参数会静默污染全部下游判定，必须在构造期显式拒绝
    if (reference_builder_ == nullptr || solver_ == nullptr) {
        throw std::invalid_argument("DdpPostStage: 参考构建器与求解器必须非空");
    }
    if (!(config_.epsilon_v > 0.0) || !(config_.v_dwell > 0.0) ||
        !(config_.shift_delay > 0.0) || !(config_.kappa_pad >= 1.0) ||
        !(config_.omega_max > 0.0) || !(config_.eta_max > 0.0) ||
        !(config_.seam_speed_tol > 0.0) || !(config_.dwell_omega_tol > 0.0) ||
        !(config_.amplitude_check_tol > 0.0)) {
        throw std::invalid_argument(
            "DdpPostStage: 滞回/驻留/执行器/容差参数必须为正且 κ_pad>=1");
    }
    if (!(vehicle_params_.wheelbase > 0.0)) {
        throw std::invalid_argument("DdpPostStage: 车辆轴距必须为正");
    }
}

double DdpPostStage::ComputeResteerTime(double delta_delta, double omega_max,
                                        double eta_max) {
    if (!(delta_delta >= 0.0) || !(omega_max > 0.0) || !(eta_max > 0.0)) {
        throw std::invalid_argument(
            "DdpPostStage: T_resteer 参数必须满足 Δδ>=0、ω_max>0、η_max>0");
    }
    // 双积分 bang-bang：三角剖面峰值角速度 √(η·Δδ)，不饱和时
    // T=2√(Δδ/η)；饱和后转梯形（加速 ω_max→匀速→减速），
    // T=Δδ/ω_max+ω_max/η_max。切换点 Δδ*=ω²/η 处两式相等（连续）
    if (delta_delta <= omega_max * omega_max / eta_max) {
        return 2.0 * std::sqrt(delta_delta / eta_max);
    }
    return delta_delta / omega_max + omega_max / eta_max;
}

std::vector<DdpSignRun> DdpPostStage::analyzeSignRuns(
    const DdpAlignedVec<DdpState>& states) const {
    if (states.size() < 2) {
        throw std::invalid_argument("DdpPostStage: 状态数量必须 >= 2");
    }
    // 滞回状态机：|v|<ε_v 的样本不改变已承诺符号（融化残留的速度涟漪
    // 被过滤），仅当反向突破阈值才闭合当前游程并开启新游程
    std::vector<DdpSignRun> runs;
    std::size_t run_begin = 0;
    int current_sign = 0;
    for (std::size_t k = 0; k < states.size(); ++k) {
        const double v = states[k](DDP_IDX_V);
        int commit = 0;
        if (v >= config_.epsilon_v) {
            commit = 1;
        } else if (v <= -config_.epsilon_v) {
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
            runs.push_back(DdpSignRun{current_sign, run_begin, k, 0.0, 0.0});
            run_begin = k;
            current_sign = commit;
        }
    }
    runs.push_back(
        DdpSignRun{current_sign, run_begin, states.size() - 1, 0.0, 0.0});
    // 段位移与朝向变化量测（PIVOT/剔除判据与诊断报告的数据来源）
    for (auto& run : runs) {
        double arc = 0.0;
        for (std::size_t k = run.begin_index; k < run.end_index; ++k) {
            arc += std::hypot(states[k + 1](DDP_IDX_X) - states[k](DDP_IDX_X),
                              states[k + 1](DDP_IDX_Y) - states[k](DDP_IDX_Y));
        }
        run.delta_s = arc;
        run.delta_theta = WrapAngle(states[run.end_index](DDP_IDX_THETA) -
                                    states[run.begin_index](DDP_IDX_THETA));
    }
    return runs;
}

std::vector<Maneuver> DdpPostStage::buildManeuvers(
    const DdpAlignedVec<DdpState>& states,
    const std::vector<DdpSignRun>& runs) const {
    std::vector<Maneuver> maneuvers;
    maneuvers.reserve(runs.size());
    for (const auto& run : runs) {
        std::vector<TrajectoryPoint> points;
        points.reserve(run.end_index - run.begin_index + 1);
        for (std::size_t k = run.begin_index; k <= run.end_index; ++k) {
            TrajectoryPoint point(states[k](DDP_IDX_X), states[k](DDP_IDX_Y),
                                  states[k](DDP_IDX_THETA));
            point.setV(states[k](DDP_IDX_V));
            point.setA(states[k](DDP_IDX_A));
            point.setDelta(states[k](DDP_IDX_DELTA));
            point.setDeltaDot(states[k](DDP_IDX_OMEGA));
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

bool DdpPostStage::pruneManeuvers(std::vector<Maneuver>* maneuvers) const {
    if (maneuvers == nullptr || maneuvers->empty()) {
        return true;
    }
    // 首/末 maneuver 保护：分类前先记录原始方向，分类后无条件恢复——
    // 首段承载起点状态、末段承载终点语义，无论判据量如何均不参与剔除
    // 与 PIVOT 重分类（红线，同 ALM 侧 detectMelting 的保护语义）
    const Direction first_direction = maneuvers->front().direction;
    const Direction last_direction = maneuvers->back().direction;
    ClassifyAndResetManeuvers(*maneuvers, config_.cleanup);
    maneuvers->front().direction = first_direction;
    maneuvers->back().direction = last_direction;
    // PIVOT 即失败：默认车辆无钟摆泊车能力，检出原地打轮式微动按求解
    // 失败语义上报（除非人工另有指定），绝不带病输出
    for (std::size_t i = 1; i + 1 < maneuvers->size(); ++i) {
        if ((*maneuvers)[i].direction == Direction::PIVOT) {
            return false;
        }
    }
    return true;
}

DdpGatingPlanBuild DdpPostStage::buildGatingPlan(
    const DdpReference& stage_two_reference, const Path& pruned_path) const {
    const std::size_t num_poses = stage_two_reference.poses.size();
    if (num_poses < 2) {
        throw std::invalid_argument(
            "DdpPostStage: 阶段二参考位姿数量必须 >= 2");
    }
    DdpGatingPlanBuild build;
    DdpGatingPlan& plan = build.plan;
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
    for (std::size_t j = 0; j < num_seams; ++j) {
        const std::size_t seam = plan.seam_indices[j];
        plan.sign_gate[seam] = 0;
        plan.seam_lookup[seam] = static_cast<int>(j);
        // 逐接缝转向需求（从阶段一输出量测：修剪后路径的点携带阶段一
        // v/δ；接缝 j 即修剪后 maneuver j 与 j+1 的边界）
        const double delta_delta = measureSeamDeltaDelta(pruned_path, j);
        const double t_resteer =
            ComputeResteerTime(delta_delta, config_.omega_max, config_.eta_max);
        // 窗口半宽 m_j=⌈max(T_resteer,T_shift)/(2dt)⌉：静止窗口时长
        // 不小于重转向需求，优化器才能在窗内排出满足 ω/η 边界的摆动
        const auto half = static_cast<std::size_t>(std::ceil(
            std::max(t_resteer, config_.shift_delay) / (2.0 * dt) - 1e-9));
        // 窗口裁剪：不越界、不跨相邻接缝（保证逐接缝驻留插入互不重叠）
        const std::size_t prev_bound =
            (j == 0) ? 0 : plan.seam_indices[j - 1] + 1;
        const std::size_t next_bound =
            (j + 1 < num_seams) ? plan.seam_indices[j + 1] - 1 : num_poses - 1;
        const std::size_t window_begin =
            std::max(seam > half ? seam - half : 0, prev_bound);
        const std::size_t window_end =
            std::min(std::min(seam + half, num_poses - 1), next_bound);
        for (std::size_t k = window_begin; k <= window_end; ++k) {
            plan.dwell_v_cap[k] = config_.v_dwell;
        }
        build.seams.push_back(DdpSeamPlan{
            seam, window_begin, window_end, delta_delta, t_resteer,
            config_.kappa_pad * std::max(t_resteer, config_.shift_delay)});
    }
    return build;
}

double DdpPostStage::measureSeamDeltaDelta(
    const Path& pruned_path, std::size_t seam_maneuver_index) const {
    const auto& maneuvers = pruned_path.getManeuvers();
    if (seam_maneuver_index + 1 >= maneuvers.size()) {
        throw std::invalid_argument(
            "DdpPostStage: 接缝对应的 maneuver 下标越界");
    }
    // δ_left/δ_right 取接缝前后最后一个 |v|>v_dwell 采样点处的 δ；
    // 共享边界点属于接缝本身（其 v 已带新 maneuver 的符号），两侧扫描
    // 均从边界点的内侧一点起，避免把后一段的 δ 误作前一段的需求；
    // 全段均低速时退化为边界点 δ（转向需求由相邻段姿态差体现）
    const auto& left = maneuvers[seam_maneuver_index].points;
    const auto& right = maneuvers[seam_maneuver_index + 1].points;
    double delta_left = left.back().hasDelta() ? left.back().getDelta() : 0.0;
    for (std::size_t i = left.size() - 1; i-- > 0;) {
        if (left[i].hasV() && std::abs(left[i].getV()) > config_.v_dwell &&
            left[i].hasDelta()) {
            delta_left = left[i].getDelta();
            break;
        }
    }
    double delta_right =
        right.front().hasDelta() ? right.front().getDelta() : 0.0;
    for (std::size_t i = 1; i < right.size(); ++i) {
        if (right[i].hasV() && std::abs(right[i].getV()) > config_.v_dwell &&
            right[i].hasDelta()) {
            delta_right = right[i].getDelta();
            break;
        }
    }
    return std::abs(delta_right - delta_left);
}

void DdpPostStage::buildStageTwoWarmStart(
    const Path& pruned_path, const DdpReference& stage_two_reference,
    DdpAlignedVec<DdpState>* warm_states,
    DdpAlignedVec<DdpControl>* warm_controls) const {
    if (warm_states == nullptr || warm_controls == nullptr) {
        throw std::invalid_argument("DdpPostStage: 热启动输出指针必须非空");
    }
    const std::size_t num_poses = stage_two_reference.poses.size();
    if (num_poses < 2) {
        throw std::invalid_argument(
            "DdpPostStage: 阶段二参考位姿数量必须 >= 2");
    }
    // 展平修剪后路径（forEach 剔除 maneuver 间共享边界点）并累积弧长，
    // 与参考构建器的重采样使用同一组插值节点（网格点弧长目标严格一致）
    std::vector<TrajectoryPoint> points;
    points.reserve(pruned_path.size());
    pruned_path.forEach(
        [&points](const TrajectoryPoint& point) { points.push_back(point); });
    if (points.size() < 2) {
        throw std::invalid_argument("DdpPostStage: 修剪后路径点数必须 >= 2");
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
        DdpState state;
        const Pose& pose = stage_two_reference.poses[k];
        state << pose.x, pose.y, pose.theta,
            sample([](const TrajectoryPoint& p) { return p.getV(); }),
            sample([](const TrajectoryPoint& p) { return p.getA(); }),
            sample([](const TrajectoryPoint& p) { return p.getDelta(); }),
            sample([](const TrajectoryPoint& p) { return p.getDeltaDot(); });
        warm_states->push_back(state);
    }
    // 控制量由 a/ω 差分反解（半隐式链的近似逆），裁剪进控制盒
    const auto& limits = solver_->config().inner;
    const double dt = stage_two_reference.dt;
    warm_controls->clear();
    warm_controls->reserve(num_poses - 1);
    for (std::size_t k = 0; k + 1 < num_poses; ++k) {
        DdpControl control;
        control(DDP_IDX_JERK) = std::clamp(
            ((*warm_states)[k + 1](DDP_IDX_A) - (*warm_states)[k](DDP_IDX_A)) /
                dt,
            -limits.jerk_max, limits.jerk_max);
        control(DDP_IDX_ETA) =
            std::clamp(((*warm_states)[k + 1](DDP_IDX_OMEGA) -
                        (*warm_states)[k](DDP_IDX_OMEGA)) /
                           dt,
                       -limits.steer_accel_max, limits.steer_accel_max);
        warm_controls->push_back(control);
    }
}

double DdpPostStage::MeasureSeamDeltaDeltaFromStates(
    const DdpAlignedVec<DdpState>& states, std::size_t seam_index,
    double v_dwell) {
    if (seam_index >= states.size()) {
        throw std::invalid_argument("DdpPostStage: 接缝索引越界");
    }
    double delta_left = states[seam_index](DDP_IDX_DELTA);
    for (std::size_t k = seam_index; k-- > 0;) {
        if (std::abs(states[k](DDP_IDX_V)) > v_dwell) {
            delta_left = states[k](DDP_IDX_DELTA);
            break;
        }
    }
    double delta_right = states[seam_index](DDP_IDX_DELTA);
    for (std::size_t k = seam_index + 1; k < states.size(); ++k) {
        if (std::abs(states[k](DDP_IDX_V)) > v_dwell) {
            delta_right = states[k](DDP_IDX_DELTA);
            break;
        }
    }
    return std::abs(delta_right - delta_left);
}

TrajectoryPoint DdpPostStage::stateToPoint(const DdpState& x, double t) const {
    TrajectoryPoint point(x(DDP_IDX_X), x(DDP_IDX_Y), x(DDP_IDX_THETA));
    point.setV(x(DDP_IDX_V));
    point.setA(x(DDP_IDX_A));
    point.setDelta(x(DDP_IDX_DELTA));
    point.setDeltaDot(x(DDP_IDX_OMEGA));
    // κ=tanδ/L：与 θ̇=v·κ 的运动学关系自洽（含 v 变号）
    point.setKappa(std::tan(x(DDP_IDX_DELTA)) / vehicle_params_.wheelbase);
    point.setT(t);
    return point;
}

Trajectory DdpPostStage::insertDwells(
    const DdpAlignedVec<DdpState>& states, double dt,
    const std::vector<DdpSeamPlan>& seams,
    std::vector<DdpSeamReport>* reports) const {
    if (states.size() < 2 || !(dt > 0.0)) {
        throw std::invalid_argument(
            "DdpPostStage: 状态数量必须 >= 2 且 dt 为正");
    }
    if (reports == nullptr) {
        throw std::invalid_argument("DdpPostStage: 接缝报告输出指针必须非空");
    }
    reports->clear();
    reports->reserve(seams.size());
    // 量测/计划阶段：逐接缝重测 Δδ_j、计算 T_dwell 与拉伸量，并生成
    // 校验⑤所需的全部量测（驻留插入前先算好全部时间增量，便于一次装配）
    std::vector<DdpDwellEdit> edits;
    edits.reserve(seams.size());
    for (const auto& seam : seams) {
        if (seam.window_begin > seam.seam_index ||
            seam.window_end < seam.seam_index ||
            seam.window_end >= states.size()) {
            throw std::invalid_argument(
                "DdpPostStage: 驻留窗边界必须包含接缝且不越界");
        }
        DdpSeamReport report;
        report.seam_index = seam.seam_index;
        // Δδ_j 以阶段二最终轨迹重测（与窗口定宽的阶段一量测相互独立）
        report.delta_delta = MeasureSeamDeltaDeltaFromStates(
            states, seam.seam_index, config_.v_dwell);
        report.t_resteer = ComputeResteerTime(
            report.delta_delta, config_.omega_max, config_.eta_max);
        report.t_dwell =
            config_.kappa_pad * std::max(report.t_resteer, config_.shift_delay);
        report.seam_speed = std::abs(states[seam.seam_index](DDP_IDX_V));
        double window_speed = 0.0;
        for (std::size_t k = seam.window_begin; k <= seam.window_end; ++k) {
            window_speed =
                std::max(window_speed, std::abs(states[k](DDP_IDX_V)));
        }
        report.window_max_speed = window_speed;
        report.window_end_omega =
            std::max(std::abs(states[seam.window_begin](DDP_IDX_OMEGA)),
                     std::abs(states[seam.window_end](DDP_IDX_OMEGA)));
        const double window_duration =
            static_cast<double>(seam.window_end - seam.window_begin) * dt;
        // 退化守卫：接缝相邻到窗口被裁剪成单点（时长为零）时不做拉伸，
        // 驻留完整性校验会因时长不足判失败并回退（绝不强行 padding）
        if (window_duration <= 0.0) {
            report.dwell_duration = 0.0;
            reports->push_back(report);
            edits.push_back(
                DdpDwellEdit{seam.window_begin, seam.window_end, 0.0});
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
            DdpDwellEdit{seam.window_begin, seam.window_end, stretched});
    }
    // 装配阶段：一次遍历完成窗外转化/窗内重定时/时间戳平移
    return assembleRetimedTrajectory(states, dt, edits);
}

Trajectory DdpPostStage::assembleRetimedTrajectory(
    const DdpAlignedVec<DdpState>& states, double dt,
    const std::vector<DdpDwellEdit>& edits) const {
    // 装配输出轨迹：接缝窗外为阶段二状态原样转化（时间戳随前方窗口
    // 拉伸量平移），窗内内容按 拉伸时长/窗口原长 线性重定时
    std::vector<TrajectoryPoint> points;
    points.reserve(states.size() +
                   edits.size() *
                       (static_cast<std::size_t>(std::ceil(
                            config_.kappa_pad * config_.shift_delay / dt)) +
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
            // 之后保持不变（窗口时长为零、无拉伸量），后续点的时间戳
            // 平移不受影响
            for (; cursor <= edit.window_end; ++cursor) {
                points.push_back(
                    stateToPoint(states[cursor], cursor * dt + time_offset));
            }
            continue;
        }
        const double ratio = edit.stretched_duration / window_duration;
        const auto intervals =
            static_cast<std::size_t>(std::lround(edit.stretched_duration / dt));
        for (std::size_t i = 0; i <= intervals; ++i) {
            // 新采样时刻 σ → 原窗口时刻 τ=σ/ratio（线性重定时）；
            // 位姿/朝向/前轮转角按原剖面取值，v/ω/a 随时间尺度同比缩放
            const double tau = std::min(i * dt / ratio, window_duration);
            const double position = tau / dt;
            const auto base =
                std::min<std::size_t>(static_cast<std::size_t>(position),
                                      edit.window_end - edit.window_begin - 1);
            const double frac = position - static_cast<double>(base);
            const DdpState& from = states[edit.window_begin + base];
            const DdpState& to = states[edit.window_begin + base + 1];
            DdpState sample = from + (to - from) * frac;
            sample(DDP_IDX_THETA) =
                from(DDP_IDX_THETA) +
                WrapAngle(to(DDP_IDX_THETA) - from(DDP_IDX_THETA)) * frac;
            sample(DDP_IDX_V) /= ratio;
            sample(DDP_IDX_A) /= ratio * ratio;
            sample(DDP_IDX_OMEGA) /= ratio;
            points.push_back(stateToPoint(sample, t_begin + i * dt));
        }
        time_offset += edit.stretched_duration - window_duration;
        cursor = edit.window_end + 1;
    }
    for (; cursor < states.size(); ++cursor) {
        points.push_back(
            stateToPoint(states[cursor], cursor * dt + time_offset));
    }
    return Trajectory(std::move(points));
}

bool DdpPostStage::validateOutput(const Trajectory& output,
                                  const ApaDdpStageTwoResult& stage_two,
                                  const TrajectoryPoint& goal,
                                  const ESDFMap& esdf_map,
                                  const VehicleFootprintModel& footprint_model,
                                  DdpPostStageDiagnostics* diagnostics) const {
    const auto fail = [diagnostics](const std::string& check, double measured,
                                    double threshold) {
        diagnostics->failed_check = check;
        diagnostics->measured_value = measured;
        diagnostics->threshold = threshold;
        return false;
    };
    // ① 碰撞复检 + ② 终点双指标 + ③ 运动学一致性（Trajectory::validate
    // 三门，与 NMPC/ALM 路径共用同一生产质量门）
    const auto validation =
        output.validate(goal, esdf_map, footprint_model, config_.validation);
    if (!validation.collision_safe) {
        return fail("collision", validation.max_intrusion_depth,
                    config_.validation.max_collision_depth);
    }
    if (!validation.terminal_position_ok) {
        return fail("terminal_position", validation.terminal_position_error,
                    config_.validation.max_terminal_position_error);
    }
    if (!validation.terminal_heading_ok) {
        return fail("terminal_heading", validation.terminal_heading_error_deg,
                    config_.validation.max_terminal_heading_error_deg);
    }
    if (!validation.kinematic_feasible) {
        // 四项残差取相对超标最严重的一项作为量化诊断
        const std::pair<const char*, double> residuals[4] = {
            {"kinematic_position",
             validation.max_kinematic_position_residual /
                 config_.validation.max_kinematic_position_residual},
            {"kinematic_heading",
             validation.max_kinematic_heading_residual_deg /
                 config_.validation.max_kinematic_heading_residual_deg},
            {"kinematic_velocity",
             validation.max_kinematic_velocity_residual /
                 config_.validation.max_kinematic_velocity_residual},
            {"kinematic_steer",
             validation.max_kinematic_steer_residual /
                 config_.validation.max_kinematic_steer_residual}};
        const auto* worst = std::max_element(
            std::begin(residuals), std::end(residuals),
            [](const auto& a, const auto& b) { return a.second < b.second; });
        return fail(worst->first, worst->second, 1.0);
    }
    // ④ 控制幅值复检：δ/ω/a/v 状态量与 j/η 控制量全时限值（驻留插入只
    // 缩不增，以阶段二输出为复检对象；限值与求解配置同源）。状态量按
    // AL 平衡态容差复检；控制量按盒过冲专项容差复检——前向滚动按设计
    // 不截断控制，最终序列可经反馈项产生有限盒过冲
    const auto& solver_config = solver_->config();
    double amplitude_violation = 0.0;
    double control_violation = 0.0;
    for (std::size_t k = 0; k < stage_two.states.size(); ++k) {
        const DdpState& x = stage_two.states[k];
        amplitude_violation =
            std::max(amplitude_violation,
                     std::abs(x(DDP_IDX_V)) - solver_config.cost.v_max);
        amplitude_violation =
            std::max(amplitude_violation,
                     std::abs(x(DDP_IDX_A)) - solver_config.cost.a_max);
        amplitude_violation =
            std::max(amplitude_violation,
                     std::abs(x(DDP_IDX_DELTA)) - solver_config.cost.delta_max);
        amplitude_violation =
            std::max(amplitude_violation,
                     std::abs(x(DDP_IDX_OMEGA)) - solver_config.cost.omega_max);
        if (k < stage_two.controls.size()) {
            control_violation =
                std::max(control_violation,
                         std::abs(stage_two.controls[k](DDP_IDX_JERK)) -
                             solver_config.inner.jerk_max);
            control_violation =
                std::max(control_violation,
                         std::abs(stage_two.controls[k](DDP_IDX_ETA)) -
                             solver_config.inner.steer_accel_max);
        }
    }
    if (amplitude_violation > config_.amplitude_check_tol) {
        return fail("amplitude", amplitude_violation,
                    config_.amplitude_check_tol);
    }
    if (control_violation > config_.control_overshoot_tol) {
        return fail("control_amplitude", control_violation,
                    config_.control_overshoot_tol);
    }
    // ⑤ 接缝零速与驻留完整性：逐接缝核对零速、T_dwell≥T_resteer、
    // 实际静止时长、窗内速度帽与窗端 ω 余量
    for (const auto& seam : diagnostics->seams) {
        if (seam.seam_speed > config_.seam_speed_tol) {
            return fail("seam_zero_speed", seam.seam_speed,
                        config_.seam_speed_tol);
        }
        if (seam.t_dwell < seam.t_resteer) {
            return fail("dwell_resteer", seam.t_dwell, seam.t_resteer);
        }
        if (seam.dwell_duration < seam.t_dwell) {
            return fail("dwell_duration", seam.dwell_duration, seam.t_dwell);
        }
        if (seam.window_max_speed > config_.v_dwell + config_.seam_speed_tol) {
            return fail("dwell_window_speed", seam.window_max_speed,
                        config_.v_dwell + config_.seam_speed_tol);
        }
        if (seam.window_end_omega > config_.dwell_omega_tol) {
            return fail("dwell_window_end_omega", seam.window_end_omega,
                        config_.dwell_omega_tol);
        }
    }
    // ⑥ maneuver 数不增：输出物理方向段数不得超过输入 A* maneuver 数
    diagnostics->output_maneuver_count = output.countDirectionRuns(
        config_.epsilon_v, config_.cleanup.min_arc_length);
    if (diagnostics->output_maneuver_count >
        diagnostics->input_maneuver_count) {
        return fail("maneuver_count",
                    static_cast<double>(diagnostics->output_maneuver_count),
                    static_cast<double>(diagnostics->input_maneuver_count));
    }
    return true;
}

void DdpPostStage::makeFallback(DdpPostStageResult* result,
                                DdpPostStageStatus status,
                                std::string failed_check, double measured,
                                double threshold,
                                const Path& original_path) const {
    result->status = status;
    result->used_fallback = true;
    result->diagnostics.failed_check = std::move(failed_check);
    result->diagnostics.measured_value = measured;
    result->diagnostics.threshold = threshold;
    // 回退轨迹：原始 A* 路径经梯形加减速时间参数化补全为可执行轨迹
    // （与生产模块的兜底语义一致，绝不输出半成品轨迹）
    result->trajectory = Trajectory(original_path, vehicle_params_);
}

DdpPostStageResult DdpPostStage::run(
    const Path& original_path, const DdpReference& stage_one_reference,
    const ApaDdpStageOneResult& stage_one_result, const TrajectoryPoint& goal,
    const ESDFMap& esdf_map, const VehicleFootprintModel& footprint_model) {
    DdpPostStageResult result;
    result.diagnostics.input_maneuver_count =
        stage_one_reference.maneuvers.empty()
            ? static_cast<int>(original_path.numManeuvers())
            : static_cast<int>(stage_one_reference.maneuvers.size());
    // 阶段一未收敛不得带病后处理（融化/修剪/门控均建立在收敛解之上）
    if (stage_one_result.report.status != ApaDdpStatus::CONVERGED) {
        makeFallback(&result, DdpPostStageStatus::STAGE_ONE_NOT_CONVERGED,
                     "stage_one_convergence",
                     static_cast<double>(stage_one_result.report.status),
                     static_cast<double>(ApaDdpStatus::CONVERGED),
                     original_path);
        return result;
    }
    // 步骤 1+2：符号游程分析与拓扑修剪（PIVOT 即失败）
    const auto runs = analyzeSignRuns(stage_one_result.states);
    auto maneuvers = buildManeuvers(stage_one_result.states, runs);
    if (!pruneManeuvers(&maneuvers)) {
        makeFallback(&result, DdpPostStageStatus::PIVOT_DETECTED, "pivot",
                     config_.cleanup.pivot_delta_threshold,
                     config_.cleanup.pivot_delta_threshold, original_path);
        return result;
    }
    // 修剪后重采样重排网格（保持 0.05 m 间距与 dt=0.1 s 不变）：
    // 复用前端构建器，cusp/打靶节点/maneuver 元数据与阶段一同源
    const Path pruned_path = ReconstructPath(maneuvers);
    DdpReference stage_two_reference;
    try {
        stage_two_reference = reference_builder_->build(pruned_path);
    } catch (const std::invalid_argument&) {
        // 退化诊断精细化：区分"点数不足"与"弧长不足一个重采样间距"
        // 两种子情形（后者典型为融化过度把有用段也压没），供人工排障
        if (pruned_path.size() < 2) {
            makeFallback(&result, DdpPostStageStatus::PRUNED_PATH_DEGENERATE,
                         "pruned_points",
                         static_cast<double>(pruned_path.size()), 2.0,
                         original_path);
        } else {
            makeFallback(&result, DdpPostStageStatus::PRUNED_PATH_DEGENERATE,
                         "pruned_length", pruned_path.length(),
                         reference_builder_->sampleDist(), original_path);
        }
        return result;
    }
    const auto gating = buildGatingPlan(stage_two_reference, pruned_path);
    // 步骤 3：阶段二门控重解（必须重解，禁止直接拼接修剪结果）——
    // 以阶段一解在修剪后网格上的映射热启动（而非前端初值提取的 bang
    // 速度剖面）；跟踪权重冻结在阶段一末轮的退火后取值（不再退火≠
    // 重置回 w_ref,0：强跟踪会与终端 AL 罚权重形成失衡平衡，实测收敛
    // 轮数翻倍），阶段一无历史（首轮即收敛）时退化为基准权重
    DdpAlignedVec<DdpState> warm_states;
    DdpAlignedVec<DdpControl> warm_controls;
    buildStageTwoWarmStart(pruned_path, stage_two_reference, &warm_states,
                           &warm_controls);
    const double stage_one_final_weight =
        stage_one_result.report.history.empty()
            ? solver_->config().cost.weight_ref_base
            : stage_one_result.report.history.back().tracking_weight;
    auto stage_two = solver_->solveStageTwo(
        stage_two_reference, gating.plan, warm_states, warm_controls,
        stage_one_final_weight, &stage_one_result.final_multipliers);
    if (stage_two.report.status != ApaDdpStatus::CONVERGED) {
        result.stage_two = std::move(stage_two);
        makeFallback(&result, DdpPostStageStatus::STAGE_TWO_NOT_CONVERGED,
                     "stage_two_convergence",
                     static_cast<double>(result.stage_two->report.status),
                     static_cast<double>(ApaDdpStatus::CONVERGED),
                     original_path);
        return result;
    }
    // 步骤 4：驻留插入（逐接缝时间拉伸 + 时间戳线性重排）
    Trajectory output = insertDwells(stage_two.states, stage_two_reference.dt,
                                     gating.seams, &result.diagnostics.seams);
    // 步骤 5：六项校验清单（任一不过 → 回退出口）
    if (!validateOutput(output, stage_two, goal, esdf_map, footprint_model,
                        &result.diagnostics)) {
        result.stage_two = std::move(stage_two);
        makeFallback(&result, DdpPostStageStatus::VALIDATION_FAILED,
                     result.diagnostics.failed_check,
                     result.diagnostics.measured_value,
                     result.diagnostics.threshold, original_path);
        return result;
    }
    // 步骤 6：全部通过，输出最终轨迹
    result.status = DdpPostStageStatus::SUCCESS;
    result.used_fallback = false;
    result.trajectory = std::move(output);
    result.stage_two = std::move(stage_two);
    return result;
}
}  // namespace apa_post_processor
