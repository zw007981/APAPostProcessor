#include "trajectory.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>

#include "../core/NMPC/vehicle_circle_geometry.h"
#include "../spatial/esdf_map.h"
#include "../vehicle/vehicle_footprint_model.h"
#include "../vehicle/vehicle_params.h"
#include "constants.h"
#include "logger.h"
#include "path.h"

namespace apa_post_processor {
namespace {
// 逐点累计弧长：相邻点欧氏距离累加，重合点（换挡边界的共享点）
// 不前进，保持序列非递减
std::vector<double> ComputeArcLengths(
    const std::vector<TrajectoryPoint>& points) {
    std::vector<double> arc_length(points.size(), 0.0);
    for (std::size_t i = 1; i < points.size(); ++i) {
        arc_length[i] =
            arc_length[i - 1] + std::hypot(points[i].x - points[i - 1].x,
                                           points[i].y - points[i - 1].y);
    }
    return arc_length;
}
// 单点运动方向符号：停驻（|v| 低于阈值）或缺少速度数据时返回 0，
// 调用方据此把它并入当前游程段
int DirectionSignOf(const TrajectoryPoint& point, double v_epsilon) {
    if (!point.hasV() || std::abs(point.getV()) < v_epsilon) {
        return 0;
    }
    return point.getV() > 0.0 ? 1 : -1;
}
// 段内按弧长插值航向角：走最短角差；越界取端点，重合段取右点
double SampleHeadingAt(const std::vector<TrajectoryPoint>& points,
                       const std::vector<double>& arc_length,
                       const PointSpan& span, double s_target) {
    const auto begin_it =
        arc_length.begin() + static_cast<std::ptrdiff_t>(span.begin);
    const auto end_it =
        arc_length.begin() + static_cast<std::ptrdiff_t>(span.end);
    const auto it = std::lower_bound(begin_it, end_it, s_target);
    if (it == begin_it) {
        return points[span.begin].theta;
    }
    if (it == end_it) {
        return points[span.end - 1].theta;
    }
    const std::size_t upper =
        static_cast<std::size_t>(std::distance(arc_length.begin(), it));
    const std::size_t lower = upper - 1;
    const double segment_length = arc_length[upper] - arc_length[lower];
    if (segment_length < EPSILON_PRECISE) {
        return points[upper].theta;
    }
    const double ratio =
        std::clamp((s_target - arc_length[lower]) / segment_length, 0.0, 1.0);
    const double delta =
        std::remainder(points[upper].theta - points[lower].theta, 2.0 * PI);
    return points[lower].theta + ratio * delta;
}
}  // namespace
std::vector<PointSpan> Trajectory::SplitDirectionSpans(
    const std::vector<TrajectoryPoint>& points,
    const WindowKappaConfig& config) {
    if (points.empty()) {
        return {};
    }
    std::vector<PointSpan> spans;
    spans.reserve(points.size());
    std::size_t span_begin = 0;
    int run_sign = 0;
    for (std::size_t i = 0; i < points.size(); ++i) {
        const int sign = DirectionSignOf(points[i], config.v_epsilon);
        if (sign == 0) {
            continue;
        }
        if (run_sign == 0) {
            run_sign = sign;
            continue;
        }
        if (sign != run_sign) {
            spans.push_back({span_begin, i});
            span_begin = i;
            run_sign = sign;
        }
    }
    spans.push_back({span_begin, points.size()});
    return spans;
}

WindowKappaSeries Trajectory::ComputeWindowKappa(
    const std::vector<TrajectoryPoint>& points,
    const std::vector<PointSpan>& spans, const WindowKappaConfig& config) {
    if (!(config.half_window > 0.0) || !(config.min_half_window > 0.0) ||
        config.min_half_window > config.half_window) {
        throw std::invalid_argument(
            "ComputeWindowKappa 收到非法的窗口曲率配置!!!");
    }
    WindowKappaSeries series;
    series.arc_length = ComputeArcLengths(points);
    series.kappa.assign(points.size(),
                        std::numeric_limits<double>::quiet_NaN());
    for (const auto& span : spans) {
        if (span.end > points.size()) {
            throw std::invalid_argument(
                "ComputeWindowKappa 收到越界的方向段区间!!!");
        }
        if (span.begin >= span.end) {
            continue;
        }
        const double span_begin_s = series.arc_length[span.begin];
        const double span_end_s = series.arc_length[span.end - 1];
        for (std::size_t i = span.begin; i < span.end; ++i) {
            const double s = series.arc_length[i];
            const double left = s - span_begin_s;
            const double right = span_end_s - s;
            if (std::max(left, right) < config.min_half_window) {
                continue;
            }
            if (left >= config.half_window && right >= config.half_window) {
                // 双侧充裕：中心差分，截断误差 O(h^2)
                const double before = SampleHeadingAt(
                    points, series.arc_length, span, s - config.half_window);
                const double after = SampleHeadingAt(
                    points, series.arc_length, span, s + config.half_window);
                series.kappa[i] =
                    std::remainder(after - before, 2.0 * PI) /
                    (2.0 * config.half_window);
                continue;
            }
            // 段端单侧差分：以较长一侧的可用弧长为步长。段端一律取
            // 中心差分会在换挡点两侧各留一段空白，低速区密采样时
            // 这些空白会把 κ 曲线切成碎片
            const double step =
                std::min(config.half_window, std::max(left, right));
            const double center =
                SampleHeadingAt(points, series.arc_length, span, s);
            if (left >= right) {
                const double before = SampleHeadingAt(
                    points, series.arc_length, span, s - step);
                series.kappa[i] =
                    std::remainder(center - before, 2.0 * PI) / step;
                continue;
            }
            const double after = SampleHeadingAt(points, series.arc_length,
                                                 span, s + step);
            series.kappa[i] =
                std::remainder(after - center, 2.0 * PI) / step;
        }
    }
    return series;
}

Trajectory::Trajectory(std::vector<TrajectoryPoint> points)
    : points_(std::move(points)) {
    length_cache_.reset();
}

void Trajectory::finalizeKappa(double wheelbase,
                               const WindowKappaConfig& config) {
    if (!std::isfinite(wheelbase) || wheelbase <= 0.0) {
        throw std::invalid_argument(
            "Trajectory::finalizeKappa: wheelbase must be positive and "
            "finite!!!");
    }
    const auto spans = SplitDirectionSpans(points_, config);
    const auto series = ComputeWindowKappa(points_, spans, config);
    for (std::size_t i = 0; i < points_.size(); ++i) {
        // 含 NaN 写入：窗口不足的点覆盖回未设置状态，保证幂等
        points_[i].setKappa(series.kappa[i]);
        if (points_[i].hasDelta()) {
            points_[i].setKappaKinematic(std::tan(points_[i].getDelta()) /
                                         wheelbase);
        }
    }
}

Trajectory::Trajectory(const Path& path, const VehicleParams& vehicle_params,
                       const TimeProfileConfig& time_config) {
    if (!std::isfinite(vehicle_params.wheelbase) ||
        vehicle_params.wheelbase <= 0.0) {
        throw std::invalid_argument(
            "Trajectory: vehicle wheelbase must be positive and finite!!!");
    }
    length_cache_.reset();
    if (path.empty()) {
        return;
    }
    points_.reserve(path.size());
    const double wheelbase = vehicle_params.wheelbase;
    // 时间参数化输入与逐机动段发射区间（δ̇ 段内差分用）
    std::vector<double> arc_lengths;
    std::vector<double> kappas;
    std::vector<int> sigmas;
    std::vector<std::size_t> cusps;
    arc_lengths.reserve(path.size());
    kappas.reserve(path.size());
    sigmas.reserve(path.size());
    cusps.reserve(path.numManeuvers());
    std::vector<std::pair<std::size_t, std::size_t>> maneuver_ranges;
    maneuver_ranges.reserve(path.numManeuvers());
    // 已发射末点的坐标与累计弧长（跨机动段连续累积）
    double prev_x = 0.0;
    double prev_y = 0.0;
    double s = 0.0;
    bool has_prev = false;
    for (const auto& maneuver : path.getManeuvers()) {
        // 运动方向符号：BACKWARD 为负，FORWARD/UNKNOWN/PIVOT 按正处理
        const int sigma = maneuver.direction == Direction::BACKWARD ? -1 : 1;
        const std::size_t range_begin = points_.size();
        // 后续机动段的首点是前段末点的重复（Path::addPoint 构造语义），与
        // Path::forEach 一致跳过；其坐标与已发射末点相同，弧长累积不丢失
        const std::size_t point_begin =
            (has_prev && !maneuver.points.empty()) ? 1 : 0;
        for (std::size_t i = point_begin; i < maneuver.points.size(); ++i) {
            const auto& src = maneuver.points[i];
            if (has_prev) {
                s += std::hypot(src.x - prev_x, src.y - prev_y);
            }
            TrajectoryPoint pt(src.x, src.y, src.theta);
            // 运动方向签名曲率：与 tanδ/L 及轨迹对比视图的约定一致
            const double kappa =
                sigma * (src.hasKappa() ? src.getKappa() : 0.0);
            pt.setKappa(kappa);
            pt.setDelta(std::atan(wheelbase * kappa));
            points_.push_back(std::move(pt));
            arc_lengths.push_back(s);
            kappas.push_back(kappa);
            sigmas.push_back(sigma);
            prev_x = src.x;
            prev_y = src.y;
            has_prev = true;
        }
        if (points_.size() > range_begin) {
            maneuver_ranges.emplace_back(range_begin, points_.size());
        }
    }
    // 换挡点 = 各非末机动段的末个发射点（车速须为 0 的停驻点）
    for (std::size_t i = 0; i + 1 < maneuver_ranges.size(); ++i) {
        cusps.push_back(maneuver_ranges[i].second - 1);
    }
    // "最快走完"前提的梯形加减速时间参数化（配置/纵向极限非法时由
    // ComputeTimeProfile 抛 std::invalid_argument）
    const TimeProfile profile = ComputeTimeProfile(
        arc_lengths, kappas, sigmas, cusps, vehicle_params, time_config);
    for (std::size_t i = 0; i < points_.size(); ++i) {
        points_[i].setV(profile.v[i]);
        points_[i].setA(profile.a[i]);
        points_[i].setT(profile.t[i]);
    }
    // δ̇=dδ/dt：段内差分（内部中心差分、端点单侧差分），分母非正
    // （近重复点时刻不前进）时置 0
    for (const auto& [range_begin, range_end] : maneuver_ranges) {
        for (std::size_t i = range_begin; i < range_end; ++i) {
            const std::size_t lo = (i > range_begin) ? i - 1 : i;
            const std::size_t hi = (i + 1 < range_end) ? i + 1 : i;
            const double dt = profile.t[hi] - profile.t[lo];
            points_[i].setDeltaDot(
                dt > 0.0
                    ? (points_[hi].getDelta() - points_[lo].getDelta()) / dt
                    : 0.0);
        }
    }
    // 交付曲率统一口径：kappa 覆盖上方的 σ 签名窄窗值为加宽窗几何值，
    // kappa_kinematic 由 δ 换算（数值上恰等于该 σ 签名窄窗 κ）——
    // 本构造器产物与三条算法路径交付轨迹的字段语义自此完全一致
    finalizeKappa(vehicle_params.wheelbase);
}

void Trajectory::clear() {
    points_.clear();
    length_cache_.reset();
}

double Trajectory::length() const {
    if (length_cache_.has_value()) {
        return length_cache_.value();
    }
    if (points_.size() < 2) {
        length_cache_ = 0.0;
        return 0.0;
    }
    double total = 0.0;
    for (std::size_t i = 1; i < points_.size(); ++i) {
        total += std::hypot(points_[i].x - points_[i - 1].x,
                            points_[i].y - points_[i - 1].y);
    }
    length_cache_ = total;
    return total;
}

double Trajectory::duration() const {
    if (points_.size() < 2) {
        return 0.0;
    }
    const auto& first = points_.front();
    const auto& last = points_.back();
    if (!first.hasT() || !last.hasT()) {
        return 0.0;
    }
    return last.getT() - first.getT();
}

int Trajectory::countDirectionRuns(double v_epsilon, double min_arc) const {
    // 第一遍：按明确符号切段（停驻点不改变符号、不产生边界，位移归入
    // 当前段），累计各段位移
    struct Episode {
        int sign;
        double arc;
    };
    std::vector<Episode> episodes;
    episodes.reserve(8);
    int current_sign = 0;
    double current_arc = 0.0;
    double prev_x = 0.0;
    double prev_y = 0.0;
    bool has_prev = false;
    for (const auto& pt : points_) {
        double ds = 0.0;
        if (has_prev) {
            ds = std::hypot(pt.x - prev_x, pt.y - prev_y);
        }
        prev_x = pt.x;
        prev_y = pt.y;
        has_prev = true;
        int sign = 0;
        if (pt.hasV() && std::abs(pt.getV()) >= v_epsilon) {
            sign = pt.getV() > 0.0 ? 1 : -1;
        }
        if (sign == 0 || sign == current_sign) {
            current_arc += ds;
            continue;
        }
        episodes.push_back({current_sign, current_arc});
        current_sign = sign;
        current_arc = ds;
    }
    episodes.push_back({current_sign, current_arc});
    // 第二遍：丢弃位移不足的抖动段并合并同号邻段。轨迹首个带符号段
    // （初始方向段）无条件保留——车辆必然从某个方向起步，其位移短只是
    // 观测窗口不足；中段/末段的短位移段按抖动过滤
    int runs = 0;
    int last_sign = 0;
    bool first_episode_kept = false;
    for (const auto& episode : episodes) {
        if (episode.sign == 0) {
            continue;
        }
        if (first_episode_kept && episode.arc < min_arc) {
            continue;
        }
        if (episode.sign != last_sign) {
            ++runs;
            last_sign = episode.sign;
        }
        first_episode_kept = true;
    }
    return runs;
}

TrajectoryPoint& Trajectory::front() {
    if (points_.empty()) {
        throw std::runtime_error("Trajectory::front: trajectory is empty!!!");
    }
    return points_.front();
}

const TrajectoryPoint& Trajectory::front() const {
    if (points_.empty()) {
        throw std::runtime_error("Trajectory::front: trajectory is empty!!!");
    }
    return points_.front();
}

TrajectoryPoint& Trajectory::back() {
    if (points_.empty()) {
        throw std::runtime_error("Trajectory::back: trajectory is empty!!!");
    }
    return points_.back();
}

const TrajectoryPoint& Trajectory::back() const {
    if (points_.empty()) {
        throw std::runtime_error("Trajectory::back: trajectory is empty!!!");
    }
    return points_.back();
}

TrajectoryPoint& Trajectory::operator[](std::size_t i) { return points_[i]; }

const TrajectoryPoint& Trajectory::operator[](std::size_t i) const {
    return points_[i];
}

void Trajectory::push_back(const TrajectoryPoint& pt) {
    length_cache_.reset();
    points_.push_back(pt);
}

void Trajectory::push_back(TrajectoryPoint&& pt) {
    length_cache_.reset();
    points_.push_back(std::move(pt));
}

std::string Trajectory::toString() const {
    std::ostringstream oss;
    oss << "Trajectory{size=" << points_.size() << ", length=" << length()
        << ", duration=" << duration() << "s}";
    return oss.str();
}

TrajectoryValidationResult Trajectory::validate(
    const TrajectoryPoint& goal, const ESDFMap& esdf_map,
    const VehicleFootprintModel& footprint_model,
    const TrajectoryValidationConfig& config) const {
    TrajectoryValidationResult result;
    // 空轨迹不通过任何检查
    if (points_.empty()) {
        result.collision_detail = "trajectory is empty";
        result.terminal_position_detail = "trajectory is empty";
        result.terminal_heading_detail = "trajectory is empty";
        result.kinematic_detail = "trajectory is empty";
        return result;
    }
    // 碰撞安全检查：多圆覆盖模型 + ESDF 距离场
    const double R = footprint_model.getOuterRadius();
    const auto outer_circles =
        vehicle_circle_geometry::ExtractLocalCircleCenters(footprint_model,
                                                           CircleType::OUTER);
    double max_intrusion = 0.0;
    for (const auto& pt : points_) {
        const double c = std::cos(pt.theta);
        const double s = std::sin(pt.theta);
        for (const auto& local : outer_circles) {
            const double wx = pt.x + local.x() * c - local.y() * s;
            const double wy = pt.y + local.x() * s + local.y() * c;
            const double d = esdf_map.getDist(wx, wy);
            const double intrusion = R - d;
            if (intrusion > max_intrusion) {
                max_intrusion = intrusion;
            }
        }
    }
    result.max_intrusion_depth = max_intrusion;
    result.collision_safe = max_intrusion <= config.max_collision_depth;
    if (!result.collision_safe) {
        std::ostringstream oss;
        oss << "max_intrusion_depth=" << max_intrusion
            << "m exceeds threshold=" << config.max_collision_depth << "m";
        result.collision_detail = oss.str();
    }
    // 终点收敛检查：位置 + 航向
    const auto& final_pt = points_.back();
    const double pos_err = std::hypot(final_pt.x - goal.x, final_pt.y - goal.y);
    const double head_err_rad =
        std::abs(std::remainder(final_pt.theta - goal.theta, 2.0 * M_PI));
    const double head_err_deg = head_err_rad * (180.0 / M_PI);
    result.terminal_position_error = pos_err;
    result.terminal_heading_error_deg = head_err_deg;
    result.terminal_position_ok = pos_err <= config.max_terminal_position_error;
    if (!result.terminal_position_ok) {
        std::ostringstream oss;
        oss << "terminal_position_error=" << pos_err
            << "m exceeds threshold=" << config.max_terminal_position_error
            << "m";
        result.terminal_position_detail = oss.str();
    }
    result.terminal_heading_ok =
        head_err_deg <= config.max_terminal_heading_error_deg;
    if (!result.terminal_heading_ok) {
        std::ostringstream oss;
        oss << "terminal_heading_error=" << head_err_deg
            << "° exceeds threshold=" << config.max_terminal_heading_error_deg
            << "°";
        result.terminal_heading_detail = oss.str();
    }
    // 运动学可行性检查：对相邻两点检验自行车模型一阶 ODE 关系在梯形积分
    // 意义下的残差（梯形配点，两端点算术平均近似区间内导数，是直接配点
    // 转录法的标准做法）。只消费 TrajectoryPoint 自带的 v/a/delta/
    // delta_dot/t 与通用运动学关系（ẋ=v·cosθ、θ̇=v·tanδ/L、v̇=a、δ̇=δdot），
    // 不依赖任何求解器内部的动力学模型细节。梯形截断误差为 O(Δt³)，
    // Δt 超过 max_kinematic_dt 的点对残差由截断主导、不携带可行性判别
    // 信号，与时间戳缺失/非递增、对应量未齐备的点对一样按"不证伪"跳过；
    // 全部点对都不可评估时该门记为通过并在 detail 注明跳过原因
    if (points_.size() < 2) {
        // 无相邻点对可比较，真空通过
        result.kinematic_feasible = true;
        result.kinematic_detail = "less than 2 points: no adjacent pairs";
    } else {
        const double wheelbase = footprint_model.getWheelbase();
        int evaluated_pairs = 0;
        double max_pos_res = 0.0;
        double max_head_res = 0.0;
        double max_vel_res = 0.0;
        double max_steer_res = 0.0;
        for (std::size_t i = 0; i + 1 < points_.size(); ++i) {
            const auto& p0 = points_[i];
            const auto& p1 = points_[i + 1];
            if (!p0.hasT() || !p1.hasT()) {
                continue;
            }
            const double dt = p1.getT() - p0.getT();
            if (!(dt > 0.0) || dt > config.max_kinematic_dt) {
                continue;
            }
            ++evaluated_pairs;
            // 位置残差（ẋ=v·cosθ、ẏ=v·sinθ）与航向残差（θ̇=v·tanδ/L）
            if (p0.hasV() && p1.hasV()) {
                const double rx =
                    (p1.x - p0.x) - 0.5 * dt *
                                        (p0.getV() * std::cos(p0.theta) +
                                         p1.getV() * std::cos(p1.theta));
                const double ry =
                    (p1.y - p0.y) - 0.5 * dt *
                                        (p0.getV() * std::sin(p0.theta) +
                                         p1.getV() * std::sin(p1.theta));
                max_pos_res = std::max(max_pos_res, std::hypot(rx, ry));
                if (p0.hasDelta() && p1.hasDelta()) {
                    // Δθ 按 [-π,π] 解缠绕，避免角度跳变误判
                    const double dtheta =
                        std::remainder(p1.theta - p0.theta, 2.0 * M_PI);
                    const double rtheta =
                        dtheta - 0.5 * dt *
                                     (p0.getV() * std::tan(p0.getDelta()) +
                                      p1.getV() * std::tan(p1.getDelta())) /
                                     wheelbase;
                    max_head_res = std::max(max_head_res, std::abs(rtheta));
                }
            }
            // 速度残差（v̇=a）
            if (p0.hasV() && p1.hasV() && p0.hasA() && p1.hasA()) {
                const double rv = (p1.getV() - p0.getV()) -
                                  0.5 * dt * (p0.getA() + p1.getA());
                max_vel_res = std::max(max_vel_res, std::abs(rv));
            }
            // 前轮转角残差（δ̇=δdot）。近零速度点对（|v| 双端低于低速
            // 阈值）跳过：此时 θ̇=v·tanδ/L≈0 与 δ 取值解耦，δ/δ̇ 不承载
            // 运动可行性信号（换挡尖点附近 δ 可在 atan 值域内跳变）
            if (p0.hasV() && p1.hasV() && p0.hasDelta() && p1.hasDelta() &&
                p0.hasDeltaDot() && p1.hasDeltaDot() &&
                (std::abs(p0.getV()) >= config.kinematic_low_speed_epsilon ||
                 std::abs(p1.getV()) >= config.kinematic_low_speed_epsilon)) {
                const double rd =
                    (p1.getDelta() - p0.getDelta()) -
                    0.5 * dt * (p0.getDeltaDot() + p1.getDeltaDot());
                max_steer_res = std::max(max_steer_res, std::abs(rd));
            }
        }
        if (evaluated_pairs == 0) {
            // 没有任何带有效时间戳且 Δt 不超上限的相邻点对，运动学门不证伪
            result.kinematic_feasible = true;
            result.kinematic_detail =
                "kinematic check skipped: no adjacent pairs with valid "
                "timestamps within dt limit";
        } else {
            const double max_head_res_deg = max_head_res * (180.0 / M_PI);
            result.max_kinematic_position_residual = max_pos_res;
            result.max_kinematic_heading_residual_deg = max_head_res_deg;
            result.max_kinematic_velocity_residual = max_vel_res;
            result.max_kinematic_steer_residual = max_steer_res;
            // 从未齐备对应量的分量其最大残差保持 0，自然通过（不证伪）
            std::ostringstream oss;
            if (max_pos_res > config.max_kinematic_position_residual) {
                oss << "max_kinematic_position_residual=" << max_pos_res
                    << "m exceeds threshold="
                    << config.max_kinematic_position_residual << "m; ";
            }
            if (max_head_res_deg > config.max_kinematic_heading_residual_deg) {
                oss << "max_kinematic_heading_residual=" << max_head_res_deg
                    << "deg exceeds threshold="
                    << config.max_kinematic_heading_residual_deg << "deg; ";
            }
            if (max_vel_res > config.max_kinematic_velocity_residual) {
                oss << "max_kinematic_velocity_residual=" << max_vel_res
                    << "m/s exceeds threshold="
                    << config.max_kinematic_velocity_residual << "m/s; ";
            }
            if (max_steer_res > config.max_kinematic_steer_residual) {
                oss << "max_kinematic_steer_residual=" << max_steer_res
                    << "rad exceeds threshold="
                    << config.max_kinematic_steer_residual << "rad; ";
            }
            result.kinematic_detail = oss.str();
            result.kinematic_feasible = result.kinematic_detail.empty();
        }
    }
    // 汇总
    result.all_passed = result.collision_safe && result.terminal_position_ok &&
                        result.terminal_heading_ok && result.kinematic_feasible;
    return result;
}

std::string FormatValidationResult(const TrajectoryValidationResult& result) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(4);
    if (result.all_passed) {
        oss << "[PASS] ";
    } else {
        oss << "[FAIL";
        if (!result.collision_safe) {
            oss << " collision=" << result.max_intrusion_depth << "m>"
                << "threshold";
        }
        if (!result.terminal_position_ok) {
            oss << " pos_err=" << result.terminal_position_error
                << "m>threshold";
        }
        if (!result.terminal_heading_ok) {
            oss << " head_err=" << result.terminal_heading_error_deg
                << "°>threshold";
        }
        if (!result.kinematic_feasible) {
            oss << " kinematic: " << result.kinematic_detail;
        }
        oss << "] ";
    }
    oss << "collision=" << result.max_intrusion_depth << "m "
        << "pos_err=" << result.terminal_position_error << "m "
        << "head_err=" << std::setprecision(2)
        << result.terminal_heading_error_deg << "° "
        << "kinematic_feasible=" << result.kinematic_feasible
        << " max_pos_res=" << result.max_kinematic_position_residual << "m";
    return oss.str();
}
}  // namespace apa_post_processor
