#include "path.h"

#include <sstream>

namespace apa_post_processor {
Path Path::FromProto(const ::apa::post_processor::Path& path_proto) {
    auto path = Path();
    for (const auto& point : path_proto.points()) {
        path.addPoint(Pose::FromProto(point));
    }
    if (path.empty()) {
        LOG_ERROR(
            "Path constructed from protobuf contains no points, protobuf msg: "
            "{}!!!",
            path_proto.ShortDebugString());
        throw std::invalid_argument(
            "Path constructed from protobuf contains no points!!!");
    }
    path.finalize();
    return path;
}

void Path::toProto(::apa::post_processor::Path* path_proto) const {
    if (path_proto == nullptr) {
        throw std::invalid_argument("Path::toProto received null path_proto");
    }
    path_proto->Clear();
    forEach([path_proto](const PathPoint& point) {
        auto* point_proto = path_proto->add_points();
        point_proto->set_x(point.x);
        point_proto->set_y(point.y);
        point_proto->set_theta(point.theta);
    });
}

void Path::toProto(
    ::google::protobuf::RepeatedPtrField<::apa::post_processor::Maneuver>*
        maneuvers_proto) const {
    if (maneuvers_proto == nullptr) {
        throw std::invalid_argument(
            "Path::toProto received null maneuvers_proto");
    }
    maneuvers_proto->Clear();
    for (const auto& maneuver : maneuvers_) {
        if (maneuver.points.empty()) {
            continue;
        }
        auto* maneuver_proto = maneuvers_proto->Add();
        maneuver_proto->set_direction(
            static_cast<::apa::post_processor::Direction>(maneuver.direction));
        for (const auto& point : maneuver.points) {
            auto* point_proto = maneuver_proto->add_points();
            point_proto->set_x(point.x);
            point_proto->set_y(point.y);
            point_proto->set_theta(point.theta);
        }
    }
}

std::vector<Maneuver>& Path::getManeuvers() {
    length_cache_ = std::nullopt;
    return maneuvers_;
}

void Path::addPoint(Pose point) {
    // 将纯几何 Pose 包装为 PathPoint；kappa 等派生量保持未设置，由 Path 内部曲率估计写入
    PathPoint path_point(point.x, point.y, point.theta);
    // 处理空路径
    if (this->empty()) {
        maneuvers_ = {Maneuver{std::move(path_point), Direction::UNKNOWN}};
        length_cache_ = 0.0;
        return;
    }
    // 计算相较于上一个点的几何差值
    auto& cur_m = maneuvers_.back();
    auto& points = cur_m.points;
    const PathPoint& last_pt = points.back();
    double dist = std::hypot(path_point.x - last_pt.x, path_point.y - last_pt.y),
           delta_theta = std::abs(
               std::remainder(path_point.theta - last_pt.theta, 2.0 * PI));
    // 过近去重与过远线性插值
    if (dist < EPSILON && delta_theta < EPSILON) {
        return;
    } else if (dist > MAX_GAP_DIST) {
        this->addPoint({(last_pt.x + path_point.x) / 2.0,
                        (last_pt.y + path_point.y) / 2.0,
                        last_pt.theta +
                            std::remainder(path_point.theta - last_pt.theta,
                                           2.0 * PI) /
                                2.0});
        this->addPoint(std::move(point));
        return;
    }
    if (length_cache_) {
        *length_cache_ += dist;
    }
    // 基于参考点推算当前点的运动意图
    auto inferFunc = [&](const PathPoint& ref_pt,
                         bool require_min_translation) -> Direction {
        // 若参考点就是上一个点，复用已计算的 dist，避免重复开方
        double ds = (&ref_pt == &last_pt)
                        ? dist
                        : std::hypot(path_point.x - ref_pt.x,
                                     path_point.y - ref_pt.y);
        double dt = std::abs(
                   std::remainder(path_point.theta - ref_pt.theta, 2.0 * PI)),
               dot = (path_point.x - ref_pt.x) * std::cos(ref_pt.theta) +
                     (path_point.y - ref_pt.y) * std::sin(ref_pt.theta);
        // 未知方向需要累计足够位移防噪；已知方向只要纵向投影明确即可识别换挡。
        if (std::abs(dot) >= DOT_EPSILON &&
            (!require_min_translation || ds >= DELTA_DIST)) {
            return dot > 0 ? Direction::FORWARD : Direction::BACKWARD;
        }
        if (ds < DELTA_DIST && dt > PIVOT_THETA_EPSILON) {
            return Direction::PIVOT;
        }
        return Direction::UNKNOWN;
    };
    if (cur_m.direction == Direction::UNKNOWN) {
        // 若当前机动段尚未确定方向且新点的意图明确，则直接加入当前maneuver并更新方向
        if (Direction dir = inferFunc(points.front(), true);
            dir != Direction::UNKNOWN) {
            cur_m.direction = dir;
        }
        points.emplace_back(std::move(path_point));
    } else {
        Direction instant_dir = inferFunc(last_pt, false);
        // 若新点的意图不明确或与当前maneuver方向一致，则直接加入当前maneuver但不更新方向
        if (instant_dir == Direction::UNKNOWN ||
            instant_dir == cur_m.direction) {
            points.emplace_back(std::move(path_point));
        } else {
            // 意图反转则生成新maneuver，注意新maneuver的第一个点是上一个maneuver的最后一个点
            // 曲率计算延后到 finalize() 统一处理，此处不再做临时继承
            maneuvers_.emplace_back(
                std::vector<PathPoint>{last_pt, std::move(path_point)},
                instant_dir);
        }
    }
}

void Path::ComputeMengerCurvature(std::vector<PathPoint>& points,
                                  std::size_t idx) {
    if (idx == 0 || idx + 1 >= points.size()) {
        return;
    }
    const auto prev_idx = Path::CalPrevCurvatureRefIndex(points, idx),
               next_idx = Path::CalNextCurvatureRefIndex(points, idx);
    if (prev_idx == idx || next_idx == idx) {
        points[idx].setKappa(0.0);
        return;
    }
    const auto &prev = points[prev_idx], &curr = points[idx],
               &next = points[next_idx];
    // 向量 a: prev -> curr, 向量 b: curr -> next, 向量 c: prev -> next
    const double dx_a = curr.x - prev.x, dy_a = curr.y - prev.y,
                 dx_b = next.x - curr.x, dy_b = next.y - curr.y,
                 dx_c = dx_a + dx_b, dy_c = dy_a + dy_b;
    // 各边长度的平方
    const double sq_len_a = dx_a * dx_a + dy_a * dy_a,
                 sq_len_b = dx_b * dx_b + dy_b * dy_b,
                 sq_len_c = dx_c * dx_c + dy_c * dy_c;
    const double sq_denom = sq_len_a * sq_len_b * sq_len_c;
    if (sq_denom < CURVATURE_SQ_EPSILON) {
        points[idx].setKappa(0.0);
        return;
    }
    // 向量叉积计算有向面积项，并执行唯一一次 std::sqrt 求最终曲率
    const double cross_prod = dx_a * dy_b - dy_a * dx_b;
    points[idx].setKappa(2.0 * cross_prod / std::sqrt(sq_denom));
}

std::size_t Path::CalPrevCurvatureRefIndex(const std::vector<PathPoint>& points,
                                           std::size_t idx) {
    double accum_dist = 0.0;
    std::size_t prev_idx = idx;
    while (prev_idx > 0 && accum_dist < CURVATURE_WINDOW_DIST) {
        const auto &curr = points[prev_idx], &prev = points[prev_idx - 1];
        accum_dist += std::hypot(curr.x - prev.x, curr.y - prev.y);
        prev_idx--;
    }
    return prev_idx;
}

std::size_t Path::CalNextCurvatureRefIndex(const std::vector<PathPoint>& points,
                                           std::size_t idx) {
    double accum_dist = 0.0;
    std::size_t next_idx = idx;
    while (next_idx + 1 < points.size() && accum_dist < CURVATURE_WINDOW_DIST) {
        const auto &curr = points[next_idx], &next = points[next_idx + 1];
        accum_dist += std::hypot(next.x - curr.x, next.y - curr.y);
        next_idx++;
    }
    return next_idx;
}

void Path::RefreshCurvatureDrafts(std::vector<PathPoint>& points) {
    if (points.size() < 3) {
        return;
    }
    for (std::size_t idx = 1; idx + 1 < points.size(); ++idx) {
        Path::ComputeMengerCurvature(points, idx);
    }
}

void Path::finalize() {
    if (this->empty()) {
        return;
    }
    for (auto& maneuver : maneuvers_) {
        auto& points = maneuver.points;
        Path::RefreshCurvatureDrafts(points);
        if (points.size() >= 3) {
            points.front().setKappa(
                points[1].hasKappa() ? points[1].getKappa() : 0.0);
            points.back().setKappa(
                points[points.size() - 2].hasKappa()
                    ? points[points.size() - 2].getKappa()
                    : 0.0);
        } else if (!points.empty()) {
            // 点不足3个无法计算内部曲率，统一回退到0
            for (auto& point : points) {
                if (!point.hasKappa()) {
                    point.setKappa(0.0);
                }
            }
        }
    }
}

std::size_t Path::size() const {
    return this->empty()
               ? 0
               : std::accumulate(
                     std::next(maneuvers_.begin()), maneuvers_.end(),
                     maneuvers_.front().points.size(),
                     [](std::size_t acc, const Maneuver& m) {
                         return acc +
                                (m.points.empty() ? 0 : m.points.size() - 1);
                     });
}

double Path::length() const {
    if (this->empty()) {
        return 0.0;
    } else if (length_cache_.has_value()) {
        return length_cache_.value();
    } else {
        double len = 0.0;
        this->forEach(
            [&len, prev = (const PathPoint*)nullptr](const PathPoint& p) mutable {
                if (prev) len += std::hypot(p.x - prev->x, p.y - prev->y);
                prev = &p;
            });
        return *(length_cache_ = len);
    }
}

std::string Path::toString() const {
    std::ostringstream oss;
    oss << "{\"maneuvers\": [";
    const char* delim = "";
    for (const auto& maneuver : maneuvers_) {
        oss << delim << maneuver.toString();
        delim = ", ";
    }
    oss << "]}";
    return oss.str();
}
}  // namespace apa_post_processor
