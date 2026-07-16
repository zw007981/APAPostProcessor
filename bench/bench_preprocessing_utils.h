#pragma once

// 供 benchmark 文件复用的预处理管线组装工具函数。
//
// 提取自 bench_adaptive_resampler.cpp、bench_differential_flatness_solver.cpp、
// bench_static_corridor_builder.cpp 中重复的 ~150 行上游管线组装代码，
// 消除跨 benchmark 文件的冗余维护负担。
// bench_bspline_smoother.cpp 因测试的是最上游阶段本身、无需构造上游管线，
// bench_preprocessing_pipeline.cpp 已复用 PreprocessingPipeline 一体化调用，
// 二者不依赖本文件。

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "preprocessing/adaptive_resampler.h"
#include "preprocessing/bspline_smoother.h"
#include "preprocessing/differential_flatness_solver.h"
#include "preprocessing/speed_profile_planner.h"
#include "spatial/esdf_map.h"
#include "util/maneuver.h"

namespace apa_post_processor {
namespace bench_utils {
// 通过派生把 BSplineSmoother 的保护方法暴露给 benchmark，
// 从而由控制点计算密集配点处的几何导数。
class BSplineSmootherBenchmarkAccessor : public BSplineSmoother {
   public:
    using BSplineSmoother::BSplineSmoother;
    using BSplineSmoother::buildKnotVector;
    using BSplineSmoother::computeBasisAtU;
};

// 将机动段方向枚举映射为速度规划使用的 +/-1 符号。
//
// 注意：本函数为简化的 benchmark 专用版本，与生产代码
// PreprocessingPipeline 匿名命名空间中的同名函数语义故意不同
// （PIVOT/UNKNOWN 均返回 1，无日志），
// 因为 benchmark 数据构造场景不会出现真正的 PIVOT 机动段。
inline int BenchDirectionToSign(Direction direction) {
    switch (direction) {
        case Direction::BACKWARD:
            return -1;
        case Direction::FORWARD:
        case Direction::UNKNOWN:
        case Direction::PIVOT:
        default:
            return 1;
    }
}

// 对单个密集配点，使用车身全部子圆查询 ESDF，返回最小距离。
inline double ComputeMinEsdfDistAtPoint(
    const BSplineSmoother::DensePointData& dense_point,
    const ESDFMap& esdf_map) {
    double min_dist = std::numeric_limits<double>::infinity();
    for (const auto& circle : dense_point.circles) {
        const auto [dist, grad] =
            esdf_map.getDistAndGrad(circle.center.x(), circle.center.y());
        (void)grad;
        min_dist = std::min(min_dist, dist);
    }
    return std::isfinite(min_dist) ? min_dist : 0.0;
}

// 由 BSplineSmoother 结果构造速度规划输入：弧长、曲率、最小 ESDF 距离。
// 曲率由控制点与基函数导数解析计算，与 DifferentialFlatnessSolver 保持一致。
inline SpeedProfileInput BuildSpeedProfileInput(
    const BSplineSmoother::Result& smooth_result,
    const BSplineSmootherBenchmarkAccessor& smoother, const ESDFMap& esdf_map) {
    SpeedProfileInput input;
    const auto& dense_points = smooth_result.dense_points;
    const auto control_point_count =
        static_cast<int>(smooth_result.control_points.size());
    const auto knot_vector = smoother.buildKnotVector(control_point_count);
    input.s.reserve(dense_points.size());
    input.kappa.reserve(dense_points.size());
    input.min_esdf_dist.reserve(dense_points.size());

    for (const auto& dpd : dense_points) {
        input.s.push_back(dpd.s);
        input.min_esdf_dist.push_back(ComputeMinEsdfDistAtPoint(dpd, esdf_map));

        BSplineSmoother::BasisPack bp;
        smoother.computeBasisAtU(dpd.u, knot_vector, control_point_count, bp);
        double x_d1 = 0.0;
        double y_d1 = 0.0;
        double x_d2 = 0.0;
        double y_d2 = 0.0;
        for (std::size_t k = 0; k < bp.indices.size(); ++k) {
            const int idx = bp.indices[k];
            const auto& cp = smooth_result.control_points[idx];
            x_d1 += cp.x() * bp.d1[k];
            y_d1 += cp.y() * bp.d1[k];
            x_d2 += cp.x() * bp.d2[k];
            y_d2 += cp.y() * bp.d2[k];
        }
        const double speed_sq = x_d1 * x_d1 + y_d1 * y_d1;
        const double denom = std::max(speed_sq * std::sqrt(speed_sq), 1e-12);
        const double kappa = (x_d1 * y_d2 - y_d1 * x_d2) / denom;
        input.kappa.push_back(kappa);
    }
    return input;
}

// 由 BSplineSmoother 结果构造微分平坦补全输入：几何导数与速度规划结果。
inline DifferentialFlatnessInput BuildDifferentialFlatnessInput(
    const BSplineSmoother::Result& smooth_result,
    const BSplineSmootherBenchmarkAccessor& smoother,
    const SpeedProfileResult& speed_result) {
    DifferentialFlatnessInput input;
    const auto& dense_points = smooth_result.dense_points;
    const auto n_points = dense_points.size();
    const auto control_point_count =
        static_cast<int>(smooth_result.control_points.size());
    const auto knot_vector = smoother.buildKnotVector(control_point_count);

    input.x.reserve(n_points);
    input.y.reserve(n_points);
    input.theta.reserve(n_points);
    input.x_d1.reserve(n_points);
    input.x_d2.reserve(n_points);
    input.x_d3.reserve(n_points);
    input.y_d1.reserve(n_points);
    input.y_d2.reserve(n_points);
    input.y_d3.reserve(n_points);
    input.v.reserve(n_points);
    input.a.reserve(n_points);
    input.t.reserve(n_points);

    for (const auto& dpd : dense_points) {
        BSplineSmoother::BasisPack bp;
        smoother.computeBasisAtU(dpd.u, knot_vector, control_point_count, bp);

        double x = 0.0;
        double y = 0.0;
        double x_d1 = 0.0;
        double y_d1 = 0.0;
        double x_d2 = 0.0;
        double y_d2 = 0.0;
        double x_d3 = 0.0;
        double y_d3 = 0.0;
        for (std::size_t k = 0; k < bp.indices.size(); ++k) {
            const int idx = bp.indices[k];
            const auto& cp = smooth_result.control_points[idx];
            x += cp.x() * bp.values[k];
            y += cp.y() * bp.values[k];
            x_d1 += cp.x() * bp.d1[k];
            y_d1 += cp.y() * bp.d1[k];
            x_d2 += cp.x() * bp.d2[k];
            y_d2 += cp.y() * bp.d2[k];
            x_d3 += cp.x() * bp.d3[k];
            y_d3 += cp.y() * bp.d3[k];
        }

        input.x.push_back(x);
        input.y.push_back(y);
        input.theta.push_back(dpd.theta);
        input.x_d1.push_back(x_d1);
        input.x_d2.push_back(x_d2);
        input.x_d3.push_back(x_d3);
        input.y_d1.push_back(y_d1);
        input.y_d2.push_back(y_d2);
        input.y_d3.push_back(y_d3);
    }

    for (std::size_t i = 0; i < n_points; ++i) {
        input.v.push_back(speed_result.v[i]);
        input.a.push_back(speed_result.a[i]);
        input.t.push_back(speed_result.t[i]);
    }

    return input;
}

// 由微分平坦结果与原始 dense_points 构造自适应重采样输入。
inline AdaptiveResamplerSegmentInput BuildAdaptiveResamplerSegmentInput(
    const BSplineSmoother::Result& smooth_result,
    const DifferentialFlatnessResult& diff_result, Direction direction) {
    AdaptiveResamplerSegmentInput segment;
    segment.states = diff_result.points;
    segment.dense_points = smooth_result.dense_points;
    segment.direction = direction;
    return segment;
}
}  // namespace bench_utils
}  // namespace apa_post_processor
