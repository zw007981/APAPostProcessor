#include "preprocessing_to_ocp_converter.h"

#include <costs/quadratic_tracking.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

#include "bicycle_model_jerk.h"

namespace apa_post_processor {
namespace {
// 判断速度是否足够非零以决定方向符号的阈值
constexpr double kVSignEps = 1e-6;
// 每段最少打靶步数
constexpr int kMinStepsPerSegment = 1;
}  // namespace

PreprocessingToOcpConverter::PreprocessingToOcpConverter(
    const VehicleParams& vehicle_params, const PathToOcpConfig& config)
    : vehicle_params_(vehicle_params), config_(config) {
    if (vehicle_params_.wheelbase <= 0.0 ||
        vehicle_params_.max_steer_angle <= 0.0) {
        throw std::invalid_argument(
            "PreprocessingToOcpConverter: vehicle_params must have positive "
            "wheelbase and max_steer_angle!!!");
    }
    if (config_.dt <= 0.0 || config_.target_peak_speed <= 0.0 ||
        config_.max_speed <= 0.0 || config_.accel_limit <= 0.0 ||
        config_.steer_rate_limit <= 0.0 || config_.pose_bound <= 0.0 ||
        config_.boundary_velocity_slack < 0.0) {
        throw std::invalid_argument(
            "PreprocessingToOcpConverter: config contains non-positive or "
            "negative field that must be positive/non-negative!!!");
    }
}

PreprocessingToOcpConverter::Result PreprocessingToOcpConverter::convert(
    const Path& original_path,
    const PreprocessingPipelineResult& pipe_result) const {
    if (!pipe_result.success) {
        throw std::invalid_argument(
            "PreprocessingToOcpConverter::convert: pipe_result.success is "
            "false, cannot build OCP from failed preprocessing output!!!");
    }
    if (pipe_result.z_ref.size() < 2U) {
        throw std::invalid_argument(
            "PreprocessingToOcpConverter::convert: z_ref must contain at least "
            "2 points!!!");
    }
    const std::size_t n_points = pipe_result.z_ref.size();
    const std::size_t n_steps = n_points - 1;
    if (pipe_result.delta_t.size() != n_steps) {
        throw std::invalid_argument(
            "PreprocessingToOcpConverter::convert: delta_t size (" +
            std::to_string(pipe_result.delta_t.size()) +
            ") must equal z_ref size - 1 (" + std::to_string(n_steps) + ")!!!");
    }
    for (const auto& pt : pipe_result.z_ref) {
        if (!pt.hasV() || !pt.hasDelta() || !pt.hasA() || !pt.hasDeltaDot()) {
            throw std::invalid_argument(
                "PreprocessingToOcpConverter::convert: every z_ref point must "
                "carry v/a/delta/delta_dot!!!");
        }
    }

    const auto boundaries =
        InferSegmentBoundaries(original_path, pipe_result.z_ref);
    {
        const auto& z_last = pipe_result.z_ref.back();
        const auto& goal = original_path.back();
        const double pos_err = std::hypot(z_last.x - goal.x, z_last.y - goal.y);
        const double head_err =
            std::abs(std::remainder(z_last.theta - goal.theta, 2.0 * PI)) *
            RAD2DEG;
        LOG_FMT_INFO(
            "PreprocessingToOcpConverter: z_ref_last=({:.4f},{:.4f},{:.4f}) "
            "goal=({:.4f},{:.4f},{:.4f}) pos_err={:.4f} head_err={:.2f}",
            z_last.x, z_last.y, z_last.theta, goal.x, goal.y, goal.theta,
            pos_err, head_err);
    }

    Result result;
    // 初始猜测按段顺序拼接；相邻段共享端点状态，除第一段外跳过每段起始点
    result.init_guess.x.reserve(n_points);
    result.init_guess.u.reserve(n_steps);
    int total_steps = 0;
    bool first_segment = true;
    for (std::size_t seg_idx = 0; seg_idx < boundaries.size(); ++seg_idx) {
        const bool is_terminal = (seg_idx == boundaries.size() - 1);
        const auto segment =
            buildSegment(pipe_result, boundaries[seg_idx], is_terminal);
        result.ocp.addSegment(segment);
        total_steps += segment.N;
        const auto& boundary = boundaries[seg_idx];
        const int x_start =
            first_segment ? boundary.start_idx : boundary.start_idx + 1;
        for (int i = x_start; i <= boundary.end_idx; ++i) {
            const auto& pt = pipe_result.z_ref[static_cast<std::size_t>(i)];
            // 状态增广：a、delta_dot 初始猜测从微分平坦参考值取，夹紧到物理 box
            const double a_guess = std::clamp(pt.getA(), -config_.accel_limit,
                                              config_.accel_limit);
            const double ddelta_guess =
                std::clamp(pt.getDeltaDot(), -config_.steer_rate_limit,
                           config_.steer_rate_limit);
            stc_SQP::Vector x(7);
            x << pt.x, pt.y, pt.theta, pt.getV(), pt.getDelta(), a_guess,
                ddelta_guess;
            result.init_guess.x.push_back(std::move(x));
        }
        for (int i = boundary.start_idx; i < boundary.end_idx; ++i) {
            const auto& pt = pipe_result.z_ref[static_cast<std::size_t>(i)];
            const auto& pt_next =
                pipe_result.z_ref[static_cast<std::size_t>(i + 1)];
            const double dt_i =
                pipe_result.delta_t[static_cast<std::size_t>(i)];
            // 新控制量 [jerk, ddelta_dot] 的初始猜测：相邻 z_ref 差分近似，夹紧到物理 box
            const double jerk_guess =
                std::clamp((pt_next.getA() - pt.getA()) / dt_i,
                           -config_.max_jerk, config_.max_jerk);
            const double ddelta_dot_guess =
                std::clamp((pt_next.getDeltaDot() - pt.getDeltaDot()) / dt_i,
                           -config_.max_steer_angular_accel,
                           config_.max_steer_angular_accel);
            stc_SQP::Vector u(2);
            u << jerk_guess, ddelta_dot_guess;
            result.init_guess.u.push_back(std::move(u));
        }
        first_segment = false;
    }

    // 截断静态走廊系数，使其行数与 OCP 总步数匹配。
    TruncateCorridor(pipe_result, total_steps, result);
    return result;
}

std::vector<PreprocessingToOcpConverter::SegmentBoundary>
PreprocessingToOcpConverter::InferSegmentBoundaries(
    const Path& original_path, const std::vector<TrajectoryPoint>& z_ref) {
    const int n_points = static_cast<int>(z_ref.size());
    std::vector<int> raw_sign(n_points, 0);
    for (int i = 0; i < n_points; ++i) {
        if (!z_ref[static_cast<std::size_t>(i)].hasV()) {
            continue;
        }
        const double v = z_ref[static_cast<std::size_t>(i)].getV();
        if (std::abs(v) > kVSignEps) {
            raw_sign[i] = (v > 0.0) ? 1 : -1;
        }
    }

    // 全零速度时退化为单段，方向符号从原始 Path 推断。
    bool all_zero = true;
    for (const int sign : raw_sign) {
        if (sign != 0) {
            all_zero = false;
            break;
        }
    }
    if (all_zero) {
        SegmentBoundary boundary;
        boundary.start_idx = 0;
        boundary.end_idx = n_points - 1;
        boundary.v_sign = InferVSign(original_path, z_ref);
        return {boundary};
    }

    std::vector<SegmentBoundary> boundaries;
    int current_sign = 0;
    int segment_start = 0;
    for (int i = 0; i < n_points; ++i) {
        const int sign = raw_sign[i];
        if (sign == 0) {
            // v=0 的转向补丁点合并到当前段中，不单独成段。
            continue;
        }
        if (current_sign == 0) {
            // 首个非零速度点：把前导零也纳入同一段
            current_sign = sign;
            segment_start = 0;
        } else if (sign == current_sign) {
            continue;
        } else {
            // 速度符号翻转，结束上一段并开始新段
            SegmentBoundary boundary;
            boundary.start_idx = segment_start;
            boundary.end_idx = i - 1;
            boundary.v_sign = static_cast<double>(current_sign);
            boundaries.push_back(boundary);
            current_sign = sign;
            segment_start = i - 1;
        }
    }
    if (current_sign != 0) {
        SegmentBoundary boundary;
        boundary.start_idx = segment_start;
        boundary.end_idx = n_points - 1;
        boundary.v_sign = static_cast<double>(current_sign);
        boundaries.push_back(boundary);
    }

    // 保证每段至少包含一个离散步；过短的段向左侧合并，左侧不存在则向右侧合并。
    for (std::size_t i = 0; i < boundaries.size();) {
        const int steps = boundaries[i].end_idx - boundaries[i].start_idx;
        if (steps < kMinStepsPerSegment) {
            if (i + 1 < boundaries.size()) {
                boundaries[i].end_idx = boundaries[i + 1].end_idx;
                boundaries.erase(boundaries.begin() + i + 1);
            } else if (i > 0) {
                boundaries[i - 1].end_idx = boundaries[i].end_idx;
                boundaries.erase(boundaries.begin() + i);
                continue;
            }
            // 只有一段且长度仍不足时无法继续合并，直接保留（输入已保证
            // n_points>=2）。
        }
        ++i;
    }
    return boundaries;
}

stc_SQP::StageSegment PreprocessingToOcpConverter::buildSegment(
    const PreprocessingPipelineResult& pipe_result,
    const SegmentBoundary& boundary, bool is_terminal) const {
    const int start = boundary.start_idx;
    const int end = boundary.end_idx;
    const int segment_steps = end - start;
    if (segment_steps <= 0) {
        throw std::invalid_argument(
            "PreprocessingToOcpConverter::buildSegment: segment must contain "
            "at least one step, got [" +
            std::to_string(start) + ", " + std::to_string(end) + "]!!!");
    }
    const double v_sign = boundary.v_sign;
    if (std::abs(v_sign - 1.0) > 1e-12 && std::abs(v_sign + 1.0) > 1e-12) {
        throw std::invalid_argument(
            "PreprocessingToOcpConverter::buildSegment: v_sign must be +1.0 "
            "or -1.0, got " +
            std::to_string(v_sign) + "!!!");
    }

    stc_SQP::StageSegment segment;
    segment.dynamics =
        std::make_shared<BicycleModelJerk>(vehicle_params_.wheelbase);
    segment.N = segment_steps;
    segment.dt = 0.0;  // dt_array 非空时优先使用 dt_array
    segment.dt_array.assign(pipe_result.delta_t.begin() + start,
                            pipe_result.delta_t.begin() + end);
    segment.v_sign = v_sign;

    // 状态增广后 nx=7, nu=2
    constexpr int kNx = 7, kNu = 2;
    segment.x_min = stc_SQP::Vector::Constant(kNx, -config_.pose_bound);
    segment.x_max = stc_SQP::Vector::Constant(kNx, config_.pose_bound);
    // 按 v_sign 单向压紧速度 box bound
    segment.x_min(3) =
        (v_sign > 0.0) ? -config_.boundary_velocity_slack : -config_.max_speed;
    segment.x_max(3) =
        (v_sign > 0.0) ? config_.max_speed : config_.boundary_velocity_slack;
    segment.x_min(4) = -vehicle_params_.max_steer_angle;
    segment.x_max(4) = vehicle_params_.max_steer_angle;
    // a、ddelta 升级为状态后，原有控制量物理极限平移到状态 box bound
    segment.x_min(5) = -config_.accel_limit;
    segment.x_max(5) = config_.accel_limit;
    segment.x_min(6) = -config_.steer_rate_limit;
    segment.x_max(6) = config_.steer_rate_limit;
    // 新控制量 [jerk, ddelta_dot] 的物理极限
    segment.u_min = (stc_SQP::Vector(kNu) << -config_.max_jerk,
                     -config_.max_steer_angular_accel)
                        .finished();
    segment.u_max = (stc_SQP::Vector(kNu) << config_.max_jerk,
                     config_.max_steer_angular_accel)
                        .finished();

    // 代价函数：仅终端段使用强终端位置/航向权重，其余段仅跟踪速度/舵角，
    // 与 PathToOcpConverter 的语义保持一致，避免多段同时争夺端点导致收敛困难。
    stc_SQP::Matrix Q = stc_SQP::Matrix::Zero(kNx, kNx);
    stc_SQP::Matrix R = stc_SQP::Matrix::Zero(kNu, kNu);
    stc_SQP::Vector x_ref = stc_SQP::Vector::Zero(kNx);
    // 全程目标牵引代价（Milestone 023 六次重构新增，参考论文 Eq.(10) J1）：目标
    // 位姿是整条路径的终点（常量，不随段/步变化），因此无论本段是否终端都先把
    // x_ref(0..2) 设为该常量目标值；权重默认 0.0，未显式开启时对 x_ref 的赋值
    // 不产生任何代价（Q 对应项仍为 0），不影响既有行为。
    const auto& global_target = pipe_result.z_ref.back();
    x_ref(0) = global_target.x;
    x_ref(1) = global_target.y;
    x_ref(2) = global_target.theta;
    Q(0, 0) = config_.global_target_position_weight;
    Q(1, 1) = config_.global_target_position_weight;
    Q(2, 2) = config_.global_target_heading_weight;
    if (is_terminal) {
        // 终端段在全程目标牵引之上叠加更强的终端跟踪权重（两者指向同一个目标
        // 位姿，Q 对角项直接相加即可，无需额外的 x_ref 逻辑）。
        Q(0, 0) += config_.terminal_position_weight;
        Q(1, 1) += config_.terminal_position_weight;
        Q(2, 2) += config_.terminal_heading_weight;
        Q(3, 3) = config_.terminal_speed_weight;
        Q(4, 4) = config_.terminal_steer_weight;
        const auto& terminal_pt =
            pipe_result.z_ref[static_cast<std::size_t>(end)];
        x_ref(3) = terminal_pt.getV();
        x_ref(4) = terminal_pt.getDelta();
        // a、ddelta 的终端参考值取 0（车辆真正静止，呼应 4.3 节"终端静止约束
        // a_N=0"），x_ref 默认已是 0，此处显式写出仅为可读性。
    } else {
        Q(3, 3) = config_.interior_speed_weight;
        Q(4, 4) = config_.interior_steer_weight;
    }
    // J_effort（原控制效果代价）：a、ddelta 升级为状态后，其幅值惩罚自然变为
    // 对这两个新状态分量的标准 Q 代价，语义与此前完全一致（惩罚幅值，非差分）。
    // 对内部段与终端段统一生效，对应此前 R(0,0)/R(1,1) 恒定生效的位置。
    Q(5, 5) = config_.control_effort_accel_weight;
    Q(6, 6) = config_.control_effort_steer_rate_weight;
    // J_smooth（Milestone 023 新增）：对新控制量 jerk、ddelta_dot 施加标准 R
    // 代价，等价于惩罚 (a_{k+1}-a_k)^2 类跨 stage 差分——这是 NMPC 求解本身
    // 真正具备"顺滑消融冗余换挡"压力的核心机制，详见 docs/NMPC.md 6.6 节。
    R(0, 0) = config_.smoothing_jerk_weight;
    R(1, 1) = config_.smoothing_steer_accel_weight;
    segment.cost = std::make_shared<stc_SQP::QuadraticTrackingCost>(
        x_ref, Q, R, /*theta_idx=*/2);

    // stage_params：填充局部步索引、参考航向、时间步长与参考位置，
    // 供信赖域/终端状态约束读取。
    segment.stage_params.resize(static_cast<std::size_t>(segment_steps));
    for (int i = start; i < end; ++i) {
        stc_SQP::StageParameters sp;
        sp.p = stc_SQP::Vector::Zero(stc_SQP::STAGE_PARAM_DIM);
        sp.p(0) = static_cast<double>(i - start);
        sp.p(1) = pipe_result.z_ref[static_cast<std::size_t>(i)].theta;
        sp.p(2) = pipe_result.delta_t[static_cast<std::size_t>(i)];
        sp.p(3) = pipe_result.z_ref[static_cast<std::size_t>(i)].x;
        sp.p(4) = pipe_result.z_ref[static_cast<std::size_t>(i)].y;
        segment.stage_params[static_cast<std::size_t>(i - start)] =
            std::move(sp);
    }
    return segment;
}

void PreprocessingToOcpConverter::TruncateCorridor(
    const PreprocessingPipelineResult& pipe_result, int total_steps,
    Result& result) {
    result.static_corridor_C.resize(0, 0);
    result.static_corridor_d.resize(0);
    if (pipe_result.c_matrix.rows() == 0 || pipe_result.d_vector.size() == 0) {
        return;
    }
    const int n_points = total_steps + 1;
    if (pipe_result.c_matrix.rows() != pipe_result.d_vector.size()) {
        throw std::invalid_argument(
            "PreprocessingToOcpConverter::TruncateCorridor: C_matrix rows (" +
            std::to_string(pipe_result.c_matrix.rows()) +
            ") must match d_vector size (" +
            std::to_string(pipe_result.d_vector.size()) + ")!!!");
    }
    if (pipe_result.c_matrix.rows() % n_points != 0) {
        throw std::invalid_argument(
            "PreprocessingToOcpConverter::TruncateCorridor: C_matrix rows (" +
            std::to_string(pipe_result.c_matrix.rows()) +
            ") is not divisible by z_ref point count (" +
            std::to_string(n_points) + ")!!!");
    }
    const int constraints_per_point = pipe_result.c_matrix.rows() / n_points;
    // constraints_per_point == 0 时等效于无走廊，truncated_rows 为
    // 0，跳过填充。
    const int truncated_rows = constraints_per_point * total_steps;
    if (truncated_rows > 0) {
        // StaticCorridorBuilder 产出的 C_matrix 只有 5
        // 列（x,y,theta,v,delta）， 状态增广（Milestone
        // 023，BicycleModelJerk）后 OCP 状态维度为 7，需要 补齐 2
        // 列全零系数（走廊约束天然与新增的 a、ddelta 状态无关）。
        const int src_cols = static_cast<int>(pipe_result.c_matrix.cols());
        constexpr int kAugmentedNx = 7;
        stc_SQP::Matrix truncated =
            pipe_result.c_matrix.topRows(truncated_rows);
        if (src_cols < kAugmentedNx) {
            stc_SQP::Matrix padded =
                stc_SQP::Matrix::Zero(truncated_rows, kAugmentedNx);
            padded.leftCols(src_cols) = truncated;
            result.static_corridor_C = std::move(padded);
        } else {
            result.static_corridor_C = std::move(truncated);
        }
        result.static_corridor_d = pipe_result.d_vector.head(truncated_rows);
    }
}

double PreprocessingToOcpConverter::InferVSign(
    const Path& original_path, const std::vector<TrajectoryPoint>& z_ref) {
    // 优先从 z_ref 推断：SpeedProfilePlanner 基于弧长方向显式计算 v 的符号，
    // 语义比 Path 的几何启发式方向推断更精确。
    for (const auto& pt : z_ref) {
        if (pt.hasV() && std::abs(pt.getV()) > kVSignEps) {
            return pt.getV() > 0.0 ? 1.0 : -1.0;
        }
    }
    // z_ref 全零速度时 fallback 到原始 Path 的几何方向标注。
    for (const auto& maneuver : original_path.getManeuvers()) {
        if (maneuver.direction == Direction::FORWARD) {
            return 1.0;
        }
        if (maneuver.direction == Direction::BACKWARD) {
            return -1.0;
        }
    }
    return 1.0;
}
}  // namespace apa_post_processor
