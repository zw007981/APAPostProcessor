#include "geometry.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace stc_SQP {
namespace {
// 二维叉积（标量）
double cross2d(const Eigen::Vector2d& a, const Eigen::Vector2d& b)
{
    return a.x() * b.y() - a.y() * b.x();
}

// 点 p 到线段 ab 的距离
double distancePointToSegment(const Eigen::Vector2d& p,
    const Eigen::Vector2d& a, const Eigen::Vector2d& b)
{
    const Eigen::Vector2d ab = b - a;
    const Eigen::Vector2d ap = p - a;
    const double len2 = ab.squaredNorm();
    if (len2 == 0.0) {
        return (p - a).norm();
    }
    double t = std::max(0.0, std::min(1.0, ap.dot(ab) / len2));
    const Eigen::Vector2d projection = a + t * ab;
    return (p - projection).norm();
}

// 判断点是否在凸多边形内部（边界也算内部），用于检测两凸包是否相交。
bool pointInConvexPolygon(const Eigen::Vector2d& p,
    const std::vector<Eigen::Vector2d>& poly)
{
    const int n = static_cast<int>(poly.size());
    if (n < 3) {
        return false;
    }
    int sign = 0;
    for (int i = 0; i < n; ++i) {
        const Eigen::Vector2d& a = poly[i];
        const Eigen::Vector2d& b = poly[(i + 1) % n];
        const double c = cross2d(b - a, p - a);
        if (c == 0.0) {
            continue;
        }
        const int s = (c > 0.0) ? 1 : -1;
        if (sign == 0) {
            sign = s;
        } else if (sign != s) {
            return false;
        }
    }
    return true;
}

// 单调链算法求凸包，返回逆时针顺序的凸包顶点（不含末尾重复点）。
std::vector<Eigen::Vector2d> convexHull(std::vector<Eigen::Vector2d> points)
{
    const int n = static_cast<int>(points.size());
    if (n <= 1) {
        return points;
    }
    std::sort(points.begin(), points.end(), [](const Eigen::Vector2d& lhs,
                                                   const Eigen::Vector2d& rhs) {
        if (lhs.x() != rhs.x()) {
            return lhs.x() < rhs.x();
        }
        return lhs.y() < rhs.y();
    });
    std::vector<Eigen::Vector2d> hull;
    // 下凸包
    for (int i = 0; i < n; ++i) {
        while (hull.size() >= 2
            && cross2d(hull.back() - hull[hull.size() - 2], points[i] - hull[hull.size() - 2])
                <= 0.0) {
            hull.pop_back();
        }
        hull.push_back(points[i]);
    }
    // 上凸包
    const int lower_size = static_cast<int>(hull.size());
    for (int i = n - 2; i >= 0; --i) {
        while (static_cast<int>(hull.size()) > lower_size
            && cross2d(hull.back() - hull[hull.size() - 2], points[i] - hull[hull.size() - 2])
                <= 0.0) {
            hull.pop_back();
        }
        hull.push_back(points[i]);
    }
    // 移除末尾重复的起始点
    if (!hull.empty()) {
        hull.pop_back();
    }
    return hull;
}

// 原点到凸多边形的最短距离（多边形按逆时针或顺时针排列）。
// 若原点在多边形内部，返回 0。
double distanceOriginToConvexPolygon(const std::vector<Eigen::Vector2d>& poly)
{
    if (poly.empty()) {
        return std::numeric_limits<double>::infinity();
    }
    if (poly.size() == 1) {
        return poly[0].norm();
    }
    if (pointInConvexPolygon(Eigen::Vector2d::Zero(), poly)) {
        return 0.0;
    }
    double min_dist = std::numeric_limits<double>::infinity();
    const int n = static_cast<int>(poly.size());
    for (int i = 0; i < n; ++i) {
        const Eigen::Vector2d& a = poly[i];
        const Eigen::Vector2d& b = poly[(i + 1) % n];
        min_dist = std::min(min_dist, distancePointToSegment(Eigen::Vector2d::Zero(), a, b));
    }
    return min_dist;
}
} // namespace

double gjkConvexDistance2d(
    const std::vector<Eigen::Vector2d>& polygon_a,
    const std::vector<Eigen::Vector2d>& polygon_b)
{
    if (polygon_a.empty() || polygon_b.empty()) {
        throw std::invalid_argument("gjkConvexDistance2d: input polygon cannot be empty");
    }
    // 构造 Minkowski 差 A - B 的顶点集合
    std::vector<Eigen::Vector2d> diff_points;
    diff_points.reserve(polygon_a.size() * polygon_b.size());
    for (const auto& a : polygon_a) {
        for (const auto& b : polygon_b) {
            diff_points.push_back(a - b);
        }
    }
    const std::vector<Eigen::Vector2d> hull = convexHull(std::move(diff_points));
    return distanceOriginToConvexPolygon(hull);
}

double gjkConvexDistance2d(const std::vector<Eigen::Vector2d>& polygon_a,
    const Eigen::Vector2d& seg_start, const Eigen::Vector2d& seg_end)
{
    if (polygon_a.empty()) {
        throw std::invalid_argument("gjkConvexDistance2d: input polygon cannot be empty");
    }
    return gjkConvexDistance2d(polygon_a.data(), polygon_a.size(), seg_start, seg_end);
}

double gjkConvexDistance2d(const Eigen::Vector2d* polygon_a, std::size_t num_points,
    const Eigen::Vector2d& seg_start, const Eigen::Vector2d& seg_end)
{
    if (polygon_a == nullptr || num_points == 0) {
        throw std::invalid_argument("gjkConvexDistance2d: input polygon cannot be empty");
    }
    if (!seg_start.allFinite() || !seg_end.allFinite()) {
        throw std::invalid_argument("gjkConvexDistance2d: line segment endpoints must be finite");
    }
    if ((seg_end - seg_start).squaredNorm() == 0.0) {
        throw std::invalid_argument("gjkConvexDistance2d: line segment endpoints coincide, degenerate");
    }
    // 直接构造 A - segment 的 Minkowski 差，避免调用方创建 polygon_b vector
    std::vector<Eigen::Vector2d> diff_points;
    diff_points.reserve(num_points * 2);
    for (std::size_t i = 0; i < num_points; ++i) {
        diff_points.push_back(polygon_a[i] - seg_start);
        diff_points.push_back(polygon_a[i] - seg_end);
    }
    const std::vector<Eigen::Vector2d> hull = convexHull(std::move(diff_points));
    return distanceOriginToConvexPolygon(hull);
}
} // namespace stc_SQP
