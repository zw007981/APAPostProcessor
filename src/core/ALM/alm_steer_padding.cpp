#include "alm_steer_padding.h"

#include <algorithm>
#include <cmath>

namespace apa_post_processor {
AlmSteerPaddingStats ApplySteerPadding(std::vector<Maneuver>& maneuvers,
                                       const AlmSteerPaddingConfig& config) {
    AlmSteerPaddingStats stats;
    for (auto& maneuver : maneuvers) {
        std::size_t index = 0;
        while (index < maneuver.points.size()) {
            auto& points = maneuver.points;
            if (std::abs(points[index].getV()) >= config.v_epsilon) {
                ++index;
                continue;
            }
            // 停驻窗口 [begin, end)
            const std::size_t begin = index;
            while (index < points.size() &&
                   std::abs(points[index].getV()) < config.v_epsilon) {
                ++index;
            }
            const std::size_t end = index;
            if (end - begin < 2) {
                continue;
            }
            // 窗口净朝向变化：只改写"伪影摆动"（净旋转≈0）窗口；真实
            // pivot 旋转需求的窗口保持原样
            const double net_dtheta = std::abs(std::remainder(
                points[end - 1].theta - points[begin].theta, 2.0 * M_PI));
            if (net_dtheta > config.max_freeze_dtheta) {
                ++stats.windows_skipped;
                continue;
            }
            ++stats.windows_legalized;
            // 从窗口前最后一个驱动点的限幅 δ 出发，按 ≤δ̇_max 斜率向后
            // 延伸过渡带，直到与原 δ 剖面重新相交（交点处 Δδ 与 δ̇ 差异
            // 最小，过渡带内逐对梯形残差严格为零）；窗口内 θ 冻结消除
            // pivot 旋转，窗口外 θ 保持原值；x,y/v/a/t 全部保持原值
            const double theta_frozen = points[begin].theta;
            const double delta_start =
                std::clamp(begin > 0 ? points[begin - 1].getDelta()
                                     : points[begin].getDelta(),
                           -config.max_steer_angle, config.max_steer_angle);
            const double delta_end =
                std::clamp(end < points.size() ? points[end].getDelta()
                                               : points[end - 1].getDelta(),
                           -config.max_steer_angle, config.max_steer_angle);
            const double t_begin = points[begin].getT();
            const double span = points[end - 1].getT() - t_begin;
            double slope = 0.0;
            if (span > 0.0) {
                slope = (delta_end - delta_start) / span;
            }
            slope = std::clamp(slope, -config.max_steer_rate,
                               config.max_steer_rate);
            // 斜率继承：若窗口前一点已处于同向过渡带（通常来自上一个窗口
            // 的延伸），沿用其斜率保持 δ̇ 连续，避免两段过渡带衔接处产生
            // 新的转向残差
            if (begin > 0) {
                const double prev_slope = points[begin - 1].getDeltaDot();
                if (std::abs(prev_slope) > 1e-9 &&
                    (prev_slope > 0.0) == (slope > 0.0)) {
                    slope = std::clamp(prev_slope, -config.max_steer_rate,
                                       config.max_steer_rate);
                }
            }
            stats.max_steer_rate_used =
                std::max(stats.max_steer_rate_used, std::abs(slope));
            // 延伸过渡带：从窗口首点起按斜率改写 δ/δ̇；窗口后与原始 δ
            // 剖面求交，一旦 ramp 值越过原剖面即停笔（该点及其后保持
            // 原值，交点对残差 ~dt/2·|slope−δ̇_orig| ≤ 0.04 rad）
            std::size_t rewrite_end = points.size();
            for (std::size_t k = begin; k < points.size(); ++k) {
                auto& point = points[k];
                const double ramp_delta =
                    delta_start + slope * (point.getT() - t_begin);
                if (k >= end) {
                    const double orig_delta = point.getDelta();
                    if ((slope >= 0.0 && ramp_delta >= orig_delta) ||
                        (slope < 0.0 && ramp_delta <= orig_delta)) {
                        rewrite_end = k;
                        break;
                    }
                } else {
                    point.theta = theta_frozen;
                }
                point.setDelta(std::clamp(ramp_delta, -config.max_steer_angle,
                                          config.max_steer_angle));
                point.setDeltaDot(slope);
            }
            // 交界平滑：过渡带与原剖面衔接处（窗口首末两端）的斜率折角在
            // 爬行区长 dt 点对下会把梯形配点残差放大超门（kink 无法被单一
            // 点对同时满足两侧梯形关系）；对改写区域（含左侧邻点）做 3 点
            // 滑动平均并从平滑后的 δ 差分重算 δ̇，折角被分摊到多个点对，
            // 斜坡中段（线性区）不受滑动平均影响
            const std::size_t smooth_lo = (begin > 0) ? begin - 1 : begin;
            const std::size_t smooth_hi =
                (rewrite_end > 0) ? rewrite_end - 1 : 0;
            if (smooth_hi > smooth_lo) {
                // 先取滑动平均后的 δ（读取允许越界到未改写点，写入仅限
                // 改写区域），再回写并差分重算 δ̇
                std::vector<double> smoothed(smooth_hi - smooth_lo + 1);
                for (std::size_t k = smooth_lo; k <= smooth_hi; ++k) {
                    const std::size_t l = (k > 0) ? k - 1 : k;
                    const std::size_t r = (k + 1 < points.size()) ? k + 1 : k;
                    smoothed[k - smooth_lo] =
                        (points[l].getDelta() + points[k].getDelta() +
                         points[r].getDelta()) /
                        3.0;
                }
                for (std::size_t k = smooth_lo; k <= smooth_hi; ++k) {
                    points[k].setDelta(smoothed[k - smooth_lo]);
                }
                // δ̇ 重算覆盖到交界对另一侧（首个未改写点），保证交界对
                // 两端 δ̇ 与平滑后 δ 剖面一致
                const std::size_t dd_hi =
                    std::min(smooth_hi + 1, points.size() - 1);
                for (std::size_t k = smooth_lo; k <= dd_hi; ++k) {
                    const std::size_t l = (k > 0) ? k - 1 : k;
                    const std::size_t r = (k + 1 < points.size()) ? k + 1 : k;
                    const double dt = points[r].getT() - points[l].getT();
                    points[k].setDeltaDot(
                        dt > 0.0
                            ? (points[r].getDelta() - points[l].getDelta()) / dt
                            : 0.0);
                }
            }
            index = end;
        }
    }
    return stats;
}
}  // namespace apa_post_processor
