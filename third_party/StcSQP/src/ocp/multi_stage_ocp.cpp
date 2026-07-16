#include "multi_stage_ocp.h"

#include <cmath>
#include <limits>
#include <stdexcept>

#include "../util/constants.h"

namespace stc_SQP {
double StageSegment::stepSize(int i) const {
    if (i < 0 || i >= N) {
        throw std::out_of_range("StageSegment::stepSize: index out of range");
    }
    if (!dt_array.empty()) {
        return dt_array[i];
    }
    return dt;
}

void MultiStageOCP::addSegment(const StageSegment& segment) {
    if (segment.N <= 0) {
        throw std::invalid_argument("MultiStageOCP::addSegment: N must be greater than 0");
    }
    if ((!std::isfinite(segment.dt) || segment.dt <= 0.0) && segment.dt_array.empty()) {
        throw std::invalid_argument("MultiStageOCP::addSegment: dt must be a finite positive number");
    }
    if (!segment.dt_array.empty()) {
        if (static_cast<int>(segment.dt_array.size()) != segment.N) {
            throw std::invalid_argument("MultiStageOCP::addSegment: dt_array length must match N");
        }
        for (double dt_i : segment.dt_array) {
            if (!std::isfinite(dt_i) || dt_i <= 0.0) {
                throw std::invalid_argument("MultiStageOCP::addSegment: dt_array elements must be finite positive numbers");
            }
        }
    }
    if (std::abs(segment.v_sign - 1.0) > EPSILON && std::abs(segment.v_sign + 1.0) > EPSILON) {
        throw std::invalid_argument("MultiStageOCP::addSegment: v_sign must be +1.0 or -1.0");
    }
    if (!segments_.empty()) {
        const auto& first = segments_.front();
        if (!segment.dynamics || !first.dynamics) {
            throw std::invalid_argument("MultiStageOCP::addSegment: dynamics cannot be null");
        }
        if (segment.dynamics->nx() != first.dynamics->nx()
            || segment.dynamics->nu() != first.dynamics->nu()) {
            throw std::invalid_argument("MultiStageOCP::addSegment: dynamics dimensions must be consistent across segments");
        }
    } else {
        if (!segment.dynamics) {
            throw std::invalid_argument("MultiStageOCP::addSegment: first segment dynamics cannot be null");
        }
    }
    segments_.push_back(segment);
}

int MultiStageOCP::totalSteps() const {
    int total = 0;
    for (const auto& segment : segments_) {
        total += segment.N;
    }
    return total;
}

bool MultiStageOCP::hasGearShift() const {
    if (segments_.size() < 2) {
        return false;
    }
    for (size_t i = 1; i < segments_.size(); ++i) {
        if (segments_[i].v_sign * segments_[i - 1].v_sign < 0.0) {
            return true;
        }
    }
    return false;
}

int MultiStageOCP::nx() const {
    if (segments_.empty() || !segments_[0].dynamics) {
        return 0;
    }
    return segments_[0].dynamics->nx();
}

int MultiStageOCP::nu() const {
    if (segments_.empty() || !segments_[0].dynamics) {
        return 0;
    }
    return segments_[0].dynamics->nu();
}

bool MultiStageOCP::validate(std::string* reason) const {
    auto set_reason = [reason](const char* msg) {
        if (reason != nullptr) {
            *reason = msg;
        }
    };
    if (segments_.empty()) {
        set_reason("OCP has no segments");
        return false;
    }
    const int nx_expected = nx();
    const int nu_expected = nu();
    if (nx_expected <= 0 || nu_expected <= 0) {
        set_reason("OCP state/control dimensions invalid");
        return false;
    }
    for (size_t s = 0; s < segments_.size(); ++s) {
        const auto& segment = segments_[s];
        if (!segment.dynamics) {
            set_reason("segment with null dynamics");
            return false;
        }
        if (segment.dynamics->nx() != nx_expected || segment.dynamics->nu() != nu_expected) {
            set_reason("dynamics dimensions inconsistent across segments");
            return false;
        }
        if (segment.N <= 0) {
            set_reason("segment with N <= 0");
            return false;
        }
        if ((!std::isfinite(segment.dt) || segment.dt <= 0.0) && segment.dt_array.empty()) {
            set_reason("segment with non-finite-positive dt and empty dt_array");
            return false;
        }
        if (!segment.dt_array.empty()) {
            if (static_cast<int>(segment.dt_array.size()) != segment.N) {
                set_reason("segment with dt_array length not matching N");
                return false;
            }
            for (int i = 0; i < segment.N; ++i) {
                if (std::isnan(segment.dt_array[i]) || std::isinf(segment.dt_array[i])
                    || segment.dt_array[i] <= 0.0) {
                    set_reason("segment with invalid dt_array element (<=0 / nan / inf)");
                    return false;
                }
            }
        }
        if (std::abs(segment.v_sign - 1.0) > EPSILON
            && std::abs(segment.v_sign + 1.0) > EPSILON) {
            set_reason("segment with invalid v_sign");
            return false;
        }
        if (!segment.stage_params.empty()) {
            if (static_cast<int>(segment.stage_params.size()) != segment.N) {
                set_reason("stage_params length must match N");
                return false;
            }
            const int expected_p_dim = STAGE_PARAM_DIM;
            for (int i = 0; i < segment.N; ++i) {
                if (segment.stage_params[i].p.size() != expected_p_dim) {
                    set_reason("stage_params parameter vector dimension must be STAGE_PARAM_DIM");
                    return false;
                }
                if (!segment.stage_params[i].p.allFinite()) {
                    set_reason("stage_params parameter vector contains non-finite values");
                    return false;
                }
            }
        }
        for (const auto& constraint : segment.constraints) {
            if (!constraint) {
                set_reason("segment contains null constraint");
                return false;
            }
            const int ng = constraint->ng();
            if (ng <= 0) {
                set_reason("constraint dimension ng <= 0; add no constraint instead");
                return false;
            }
        }
        if (segment.x_min.size() != nx_expected || segment.x_max.size() != nx_expected) {
            set_reason("state bound dimension inconsistent with nx");
            return false;
        }
        if (segment.u_min.size() != nu_expected || segment.u_max.size() != nu_expected) {
            set_reason("control bound dimension inconsistent with nu");
            return false;
        }
        for (int i = 0; i < nx_expected; ++i) {
            const double lb = segment.x_min(i), ub = segment.x_max(i);
            // 允许单边无界：lb=-inf 或 ub=+inf；其余含 inf 情况均非法
            if (std::isnan(lb) || std::isnan(ub) || lb > ub
                || (std::isinf(lb) && lb > 0.0)
                || (std::isinf(ub) && ub < 0.0)
                || (std::isinf(lb) && std::isinf(ub) && lb == ub)) {
                set_reason("state bound invalid or contains illegal infinite bound");
                return false;
            }
        }
        for (int i = 0; i < nu_expected; ++i) {
            const double lb = segment.u_min(i), ub = segment.u_max(i);
            if (std::isnan(lb) || std::isnan(ub) || lb > ub
                || (std::isinf(lb) && lb > 0.0)
                || (std::isinf(ub) && ub < 0.0)
                || (std::isinf(lb) && std::isinf(ub) && lb == ub)) {
                set_reason("control bound invalid or contains illegal infinite bound");
                return false;
            }
        }
    }
    // 相邻段状态边界必须有非空交集，否则段间边界状态无可行域
    for (size_t s = 1; s < segments_.size(); ++s) {
        const auto& prev = segments_[s - 1];
        const auto& next = segments_[s];
        for (int i = 0; i < nx_expected; ++i) {
            if (std::max(prev.x_min(i), next.x_min(i))
                > std::min(prev.x_max(i), next.x_max(i))) {
                set_reason("adjacent segment state bounds have no intersection");
                return false;
            }
        }
    }
    set_reason("");
    return true;
}

MultiStageOCP MultiStageOCP::coarsen(int coarse_n, double coarse_dt) const {
    if (segments_.empty()) {
        throw std::invalid_argument("MultiStageOCP::coarsen: cannot coarsen empty OCP");
    }

    // 计算每段与总时长（支持 dt_array）
    std::vector<double> seg_time(segments_.size());
    double total_time = 0.0;
    for (size_t s = 0; s < segments_.size(); ++s) {
        const auto& seg = segments_[s];
        double t = 0.0;
        if (!seg.dt_array.empty()) {
            for (double dt_i : seg.dt_array) {
                t += dt_i;
            }
        } else {
            t = seg.N * seg.dt;
        }
        seg_time[s] = t;
        total_time += t;
    }
    if (!std::isfinite(total_time) || total_time <= 0.0) {
        throw std::invalid_argument("MultiStageOCP::coarsen: total duration invalid");
    }

    const int fine_total = totalSteps();
    int target_total = coarse_n;
    if (target_total <= 0 && coarse_dt > 0.0) {
        target_total = static_cast<int>(std::round(total_time / coarse_dt));
    }
    if (target_total <= 0) {
        // 默认粗化到原步数的 1/10，至少保留 1 步
        target_total = std::max(1, fine_total / 10);
    }
    target_total = std::max(1, std::min(target_total, fine_total));

    // 先预计算每段粗化步数：前 S-1 段按时间比例分配，最后一段做截断修正，
    // 避免循环体与修正逻辑出现重复的舍入公式。
    const int nseg = static_cast<int>(segments_.size());
    std::vector<int> coarse_Ns(nseg);
    int used = 0;
    for (int s = 0; s < nseg - 1; ++s) {
        const double ratio = seg_time[s] / total_time;
        coarse_Ns[s] = std::max(1, static_cast<int>(std::round(target_total * ratio)));
        used += coarse_Ns[s];
    }
    // 最后一段修正：保证总步数不超过 target_total；若段数超过目标步数，
    // 实际粗化步数以段数为下限（每段至少 1 步）。
    coarse_Ns[nseg - 1] = std::max(1, target_total - used);

    MultiStageOCP coarse_ocp;
    for (int s = 0; s < nseg; ++s) {
        const auto& seg = segments_[s];
        const int coarse_N = coarse_Ns[s];

        StageSegment cseg = seg;
        cseg.N = coarse_N;
        cseg.dt = seg_time[s] / coarse_N;
        cseg.dt_array.clear();

        // stage_params 若存在，按最近邻下采样到粗步数
        if (!seg.stage_params.empty()) {
            cseg.stage_params.clear();
            cseg.stage_params.reserve(coarse_N);
            for (int j = 0; j < coarse_N; ++j) {
                int orig_idx = 0;
                if (coarse_N > 1) {
                    orig_idx = static_cast<int>(std::round(
                        j * static_cast<double>(seg.N - 1) / (coarse_N - 1)));
                }
                orig_idx = std::max(0, std::min(seg.N - 1, orig_idx));
                cseg.stage_params.push_back(seg.stage_params[orig_idx]);
            }
        }
        coarse_ocp.addSegment(cseg);
    }
    return coarse_ocp;
}

std::pair<int, int> MultiStageOCP::globalStepToSegment(int global_k) const {
    if (global_k < 0) {
        return { -1, -1 };
    }
    int accumulated = 0;
    for (size_t s = 0; s < segments_.size(); ++s) {
        const int Ns = segments_[s].N;
        if (global_k < accumulated + Ns) {
            return { static_cast<int>(s), global_k - accumulated };
        }
        accumulated += Ns;
    }
    return { -1, -1 };
}
} // namespace stc_SQP
