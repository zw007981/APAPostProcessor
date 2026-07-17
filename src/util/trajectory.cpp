#include "trajectory.h"

#include <cmath>
#include <iomanip>
#include <sstream>

#include "../core/NMPC/vehicle_circle_geometry.h"
#include "../spatial/esdf_map.h"
#include "../vehicle/vehicle_footprint_model.h"
#include "logger.h"

namespace apa_post_processor {
Trajectory::Trajectory(std::vector<TrajectoryPoint> points)
    : points_(std::move(points)) {
    length_cache_.reset();
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
    // 汇总
    result.all_passed = result.collision_safe && result.terminal_position_ok &&
                        result.terminal_heading_ok;
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
        oss << "] ";
    }
    oss << "collision=" << result.max_intrusion_depth << "m "
        << "pos_err=" << result.terminal_position_error << "m "
        << "head_err=" << std::setprecision(2)
        << result.terminal_heading_error_deg << "°";
    return oss.str();
}
}  // namespace apa_post_processor
