#include "preprocessing_pipeline.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "../util/logger.h"

namespace apa_post_processor {
namespace {
// 将机动段方向枚举映射为速度规划使用的 +/-1 符号
int DirectionToSign(Direction direction) {
    switch (direction) {
        case Direction::BACKWARD:
            return -1;
        case Direction::FORWARD:
            return 1;
        case Direction::PIVOT:
            return 0;
        case Direction::UNKNOWN:
        default:
            LOG_WARN("DirectionToSign: UNKNOWN direction treated as FORWARD");
            return 1;
    }
}
}  // namespace

PreprocessingPipeline::PreprocessingPipeline(
    const PreprocessingPipelineConfig& config,
    const VehicleParams& vehicle_params,
    const VehicleFootprintModel& footprint_model, const ESDFMap& esdf_map)
    : config_([&config]() {
          // 将跨阶段统一的碰撞安全裕度传播到 BSplineSmoother
          if (config.collision_safety_margin < 0.0 ||
              !std::isfinite(config.collision_safety_margin)) {
              throw std::invalid_argument(
                  "PreprocessingPipelineConfig::collision_safety_margin must "
                  "be "
                  "non-negative finite value, got " +
                  std::to_string(config.collision_safety_margin));
          }
          auto c = config;
          c.bspline.collision_margin = c.collision_safety_margin;
          return c;
      }()),
      vehicle_params_(vehicle_params),
      footprint_model_(footprint_model),
      esdf_map_(esdf_map),
      bspline_smoother_(config_.bspline, vehicle_params, footprint_model,
                        esdf_map),
      speed_planner_(config_.speed),
      diff_flat_solver_(config_.diff_flat),
      adaptive_resampler_(config_.resampler),
      corridor_builder_(config_.corridor) {}

PreprocessingPipelineResult PreprocessingPipeline::run(
    const Path& path, double initial_velocity,
    double initial_steer_angle) const {
    validateInputs(path);
    const auto t_start = std::chrono::steady_clock::now();

    PreprocessingPipelineResult result;
    // 记录本次 run() 实际使用的关键参数
    result.hard_margin_used = config_.collision_safety_margin;
    result.soft_margin_used = config_.corridor.soft_margin;
    result.outer_row_num_used = footprint_model_.getOuterRowNum();
    path.forEach([&result](const TrajectoryPoint& pt) {
        result.original_z_ref.push_back(pt);
    });
    const auto& maneuvers = path.getManeuvers();
    const std::size_t maneuver_count = maneuvers.size();
    std::vector<PerManeuverOutput> per_maneuver_outputs;
    per_maneuver_outputs.reserve(maneuver_count);

    // 阶段耗时累加器
    double total_bspline_ms = 0.0;
    double total_speed_ms = 0.0;
    double total_diff_flat_ms = 0.0;

    // 逐 Maneuver 执行 B 样条平滑 → 速度规划 → 微分平坦
    for (std::size_t i = 0; i < maneuver_count; ++i) {
        const auto& maneuver = maneuvers[i];
        // PIVOT 段无纵向位移，跳过
        if (maneuver.direction == Direction::PIVOT) {
            continue;
        }
        // 仅第一个机动段的起始速度可能非零
        const double seg_initial_v = (i == 0) ? initial_velocity : 0.0;
        // 该段所有密集配点的方向符号一致
        const int dir_sign = DirectionToSign(maneuver.direction);

        auto seg_start = std::chrono::steady_clock::now();
        // Milestone 005: B 样条平滑
        auto smooth_result = bspline_smoother_.smooth(maneuver);
        auto seg_bspline_end = std::chrono::steady_clock::now();
        total_bspline_ms += std::chrono::duration<double, std::milli>(
                                seg_bspline_end - seg_start)
                                .count();

        if (!smooth_result.success) {
            result.status_msg =
                "BSplineSmoother failed at maneuver " + std::to_string(i) +
                ": max_intrusion_depth=" +
                std::to_string(smooth_result.max_intrusion_depth);
            return result;
        }

        // B 样条曲线几何曲率验证
        {
            double max_abs_k = 0.0;
            const auto& dp = smooth_result.dense_points;
            for (std::size_t k = 1; k < dp.size(); ++k) {
                const double ds = dp[k].s - dp[k - 1].s;
                if (ds < 1e-9) continue;
                const double dtheta =
                    std::remainder(dp[k].theta - dp[k - 1].theta, 2.0 * M_PI);
                const double kappa = dtheta / ds;
                if (std::abs(kappa) > max_abs_k) max_abs_k = std::abs(kappa);
            }
            LOG_FMT_INFO(
                "BSpline M{}: arc={:.2f}m pts={} max|κ|={:.4f} (limit={:.2f})",
                i, dp.back().s - dp.front().s, dp.size(), max_abs_k,
                config_.bspline.max_kappa);
        }

        // 构造速度规划输入
        auto speed_input = buildSpeedProfileInput(smooth_result);

        // 为每点生成完整的方向符号数组（单段内所有点方向一致）
        std::vector<int> full_direction_signs(speed_input.s.size(), dir_sign);

        // Milestone 006: 速度规划
        auto speed_result =
            speed_planner_.plan(speed_input, vehicle_params_,
                                full_direction_signs, {}, seg_initial_v);
        auto seg_speed_end = std::chrono::steady_clock::now();
        total_speed_ms += std::chrono::duration<double, std::milli>(
                              seg_speed_end - seg_bspline_end)
                              .count();

        if (!speed_result.success) {
            result.status_msg = "SpeedProfilePlanner failed at maneuver " +
                                std::to_string(i) + ": " +
                                speed_result.status_msg;
            return result;
        }

        // 构造微分平坦输入
        auto diff_input =
            buildDifferentialFlatnessInput(smooth_result, speed_result);

        // Milestone 007: 微分平坦补全
        auto diff_result = diff_flat_solver_.solve(diff_input, vehicle_params_);
        auto seg_diff_end = std::chrono::steady_clock::now();
        total_diff_flat_ms += std::chrono::duration<double, std::milli>(
                                  seg_diff_end - seg_speed_end)
                                  .count();

        if (!diff_result.success) {
            result.status_msg =
                "DifferentialFlatnessSolver failed at maneuver " +
                std::to_string(i) + ": " + diff_result.status_msg;
            return result;
        }

        PerManeuverOutput output;
        output.smooth_result = std::move(smooth_result);
        output.speed_result = std::move(speed_result);
        output.diff_flat_result = std::move(diff_result);
        output.direction = maneuver.direction;
        per_maneuver_outputs.push_back(std::move(output));
    }

    // 构造自适应重采样输入
    std::vector<AdaptiveResamplerSegmentInput> resampler_segments;
    resampler_segments.reserve(maneuver_count);
    for (const auto& output : per_maneuver_outputs) {
        resampler_segments.push_back(buildAdaptiveResamplerSegmentInput(
            output.smooth_result, output.diff_flat_result, output.direction));
    }

    // 透传各阶段中间产物（调试开关）
    if (config_.enable_debug_output) {
        result.debug_maneuver_outputs = std::move(per_maneuver_outputs);
    }

    // 自适应重采样与维数固化
    auto resample_start = std::chrono::steady_clock::now();
    auto resample_result = adaptive_resampler_.resample(
        resampler_segments, vehicle_params_, initial_steer_angle);
    auto resample_end = std::chrono::steady_clock::now();
    const double resample_ms =
        std::chrono::duration<double, std::milli>(resample_end - resample_start)
            .count();

    if (!resample_result.success) {
        result.status_msg =
            "AdaptiveResampler failed: " + resample_result.status_msg;
        return result;
    }

    result.z_ref = std::move(resample_result.points);
    result.delta_t = std::move(resample_result.delta_t);
    result.final_dimension = resample_result.final_dimension;

    // 静态安全走廊构建（可选）
    double corridor_ms = 0.0;
    if (config_.use_static_corridor && !result.z_ref.empty()) {
        auto corridor_start = std::chrono::steady_clock::now();
        auto corridor_result =
            corridor_builder_.build(result.z_ref, esdf_map_, footprint_model_);
        corridor_ms = std::chrono::duration<double, std::milli>(
                          std::chrono::steady_clock::now() - corridor_start)
                          .count();

        if (!corridor_result.success) {
            result.status_msg =
                "StaticCorridorBuilder failed: " + corridor_result.status_msg;
            return result;
        }

        result.c_matrix = std::move(corridor_result.c_matrix);
        result.d_vector = std::move(corridor_result.d_vector);
    }

    const auto t_end = std::chrono::steady_clock::now();
    const double total_ms =
        std::chrono::duration<double, std::milli>(t_end - t_start).count();

    result.success = true;
    result.status_msg = "OK";
    result.time_bspline_ms = total_bspline_ms;
    result.time_speed_ms = total_speed_ms;
    result.time_diff_flat_ms = total_diff_flat_ms;
    result.time_resample_ms = resample_ms;
    result.time_corridor_ms = corridor_ms;
    result.time_total_ms = total_ms;

    return result;
}

SpeedProfileInput PreprocessingPipeline::buildSpeedProfileInput(
    const BSplineSmoother::Result& smooth_result) const {
    SpeedProfileInput input;
    const auto& dense_points = smooth_result.dense_points;
    const auto n_points = dense_points.size();
    if (n_points == 0) {
        return input;
    }
    input.s.reserve(n_points);
    input.kappa.reserve(n_points);
    input.min_esdf_dist.reserve(n_points);

    const auto control_point_count =
        static_cast<int>(smooth_result.control_points.size());
    const auto knot_vector =
        bspline_smoother_.buildKnotVector(control_point_count);

    input.s.reserve(n_points);
    input.kappa.reserve(n_points);
    input.min_esdf_dist.reserve(n_points);

    for (const auto& dpd : dense_points) {
        input.s.push_back(dpd.s);
        input.min_esdf_dist.push_back(computeMinEsdfDistAtPoint(dpd));

        // 从 B 样条基函数导数计算曲率
        BSplineSmoother::BasisPack bp;
        bspline_smoother_.computeBasisAtU(dpd.u, knot_vector,
                                          control_point_count, bp);
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
        // kappa = (x'y'' - y'x'') / (x'^2 + y'^2)^(3/2)
        // 当 speed_sq 极小时（近奇点），曲率不可靠，直接设 0
        const double speed_sq = x_d1 * x_d1 + y_d1 * y_d1;
        constexpr double kMinSpeedSq = 1e-12;
        double kappa = 0.0;
        if (speed_sq > kMinSpeedSq) {
            const double denom = speed_sq * std::sqrt(speed_sq);
            kappa = (x_d1 * y_d2 - y_d1 * x_d2) / denom;
        }
        input.kappa.push_back(kappa);
    }

    return input;
}

DifferentialFlatnessInput PreprocessingPipeline::buildDifferentialFlatnessInput(
    const BSplineSmoother::Result& smooth_result,
    const SpeedProfileResult& speed_result) const {
    DifferentialFlatnessInput input;
    const auto& dense_points = smooth_result.dense_points;
    const auto n_points = dense_points.size();
    if (n_points == 0) {
        return input;
    }

    const auto control_point_count =
        static_cast<int>(smooth_result.control_points.size());
    const auto knot_vector =
        bspline_smoother_.buildKnotVector(control_point_count);

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
        bspline_smoother_.computeBasisAtU(dpd.u, knot_vector,
                                          control_point_count, bp);

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

    // 从速度规划结果拷贝 v/a/t
    const auto result_n = speed_result.v.size();
    for (std::size_t i = 0; i < n_points && i < result_n; ++i) {
        input.v.push_back(speed_result.v[i]);
        input.a.push_back(speed_result.a[i]);
        input.t.push_back(speed_result.t[i]);
    }

    return input;
}

AdaptiveResamplerSegmentInput
PreprocessingPipeline::buildAdaptiveResamplerSegmentInput(
    const BSplineSmoother::Result& smooth_result,
    const DifferentialFlatnessResult& diff_flat_result,
    Direction direction) const {
    AdaptiveResamplerSegmentInput segment;
    segment.states = diff_flat_result.points;
    segment.dense_points = smooth_result.dense_points;
    segment.direction = direction;
    return segment;
}

double PreprocessingPipeline::computeMinEsdfDistAtPoint(
    const BSplineSmoother::DensePointData& dense_point) const {
    // 直接取 smoother 阶段预计算并缓存的 circle.dist，避免对每个子圆重复 ESDF
    // 双线性插值。 BSplineSmoother::validateCollisionFree
    // 中已对同一批密集配点查询并填充了 circle.dist。
    if (dense_point.circles.empty()) {
        return 0.0;
    }
    double min_dist = std::numeric_limits<double>::infinity();
    for (const auto& circle : dense_point.circles) {
        if (circle.dist < min_dist) {
            min_dist = circle.dist;
        }
    }
    // 防御：若所有子圆缓存值均非有限（如地图越界导致的 NaN/Inf），降级为 0.0
    // 并告警。 此行为继承自 ESDFMap 越界哨兵值的已知限制（见
    // docs/known-limitations.md）。
    if (!std::isfinite(min_dist)) {
        LOG_WARN(
            "PreprocessingPipeline: all circle ESDF distances are non-finite "
            "at s={:.3f}, position=({:.3f},{:.3f}); falling back to 0.0",
            dense_point.s, dense_point.position.x(), dense_point.position.y());
        return 0.0;
    }
    return min_dist;
}

void PreprocessingPipeline::validateInputs(const Path& path) const {
    // 调用方应确保 path 已 finalize()，否则各 TrajectoryPoint 的 kappa
    // 全部未设置。 BSplineSmoother 自身不依赖 kappa，因此未 finalize
    // 不会导致崩溃， 但下游若隐式依赖 kappa
    // 会有风险。此处不做硬性检查，仅在注释中说明契约。
    const auto& maneuvers = path.getManeuvers();
    if (maneuvers.empty()) {
        throw std::invalid_argument(
            "PreprocessingPipeline::run: path has no maneuvers");
    }
}
}  // namespace apa_post_processor
