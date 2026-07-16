#include "static_corridor_linear_constraint.h"

#include <stdexcept>

namespace apa_post_processor {
StaticCorridorLinearConstraint::StaticCorridorLinearConstraint(
    const stc_SQP::Matrix& C, const stc_SQP::Vector& d, int global_start_idx,
    int constraints_per_step, int segment_steps, bool skip_last_step)
    : C_(C),
      d_(d),
      global_start_idx_(global_start_idx),
      constraints_per_step_(constraints_per_step),
      segment_steps_(segment_steps),
      skip_last_step_(skip_last_step) {
    if (constraints_per_step_ <= 0) {
        throw std::invalid_argument(
            "StaticCorridorLinearConstraint: constraints_per_step must be "
            "positive, got " +
            std::to_string(constraints_per_step_));
    }
    if (global_start_idx_ < 0) {
        throw std::invalid_argument(
            "StaticCorridorLinearConstraint: global_start_idx must be "
            "non-negative, got " +
            std::to_string(global_start_idx_));
    }
    if (segment_steps_ <= 0) {
        throw std::invalid_argument(
            "StaticCorridorLinearConstraint: segment_steps must be positive, "
            "got " +
            std::to_string(segment_steps_));
    }
    if (C_.cols() < 5) {
        throw std::invalid_argument(
            "StaticCorridorLinearConstraint: C must have at least 5 columns "
            "(x,y,theta,v,delta,...), got " +
            std::to_string(C_.cols()));
    }
    if (C_.rows() != d_.size()) {
        throw std::invalid_argument("StaticCorridorLinearConstraint: C rows (" +
                                    std::to_string(C_.rows()) +
                                    ") must match d size (" +
                                    std::to_string(d_.size()) + ")");
    }
    if (C_.rows() == 0) {
        throw std::invalid_argument(
            "StaticCorridorLinearConstraint: C must have at least one row");
    }
}

int StaticCorridorLinearConstraint::ng() const { return constraints_per_step_; }

void StaticCorridorLinearConstraint::evaluate(const stc_SQP::Vector& x,
                                              const stc_SQP::Vector& u,
                                              const stc_SQP::Vector& p,
                                              stc_SQP::Vector& g) const {
    (void)u;
    // 检查 p 维度
    if (p.size() < 1) {
        throw std::invalid_argument(
            "StaticCorridorLinearConstraint: p must be non-empty with p(0) = "
            "local_step_index, got size " +
            std::to_string(p.size()));
    }
    const int local_step = static_cast<int>(p(0));
    validateStepIndex(p, local_step);
    g.resize(constraints_per_step_);
    if (skip_last_step_ && local_step == segment_steps_ - 1) {
        g.setConstant(-1.0);
        return;
    }
    const int global_step = global_start_idx_ + local_step;
    const int row_start = global_step * constraints_per_step_;
    const int row_end = row_start + constraints_per_step_;
    // 对每个约束行计算 C.row(r) * x - d(r) <= 0
    for (int r = row_start; r < row_end; ++r) {
        const int local_r = r - row_start;
        g(local_r) = C_.row(r).dot(x) - d_(r);
    }
}

void StaticCorridorLinearConstraint::jacobian(const stc_SQP::Vector& x,
                                              const stc_SQP::Vector& u,
                                              const stc_SQP::Vector& p,
                                              stc_SQP::Matrix& Cx,
                                              stc_SQP::Matrix& Cu) const {
    (void)x;
    (void)u;
    // 检查 p 维度
    if (p.size() < 1) {
        throw std::invalid_argument(
            "StaticCorridorLinearConstraint: p must be non-empty with p(0) = "
            "local_step_index, got size " +
            std::to_string(p.size()));
    }
    const int local_step = static_cast<int>(p(0));
    validateStepIndex(p, local_step);
    // Cx 列数取 C_ 的实际列数
    Cx = stc_SQP::Matrix::Zero(constraints_per_step_, C_.cols());
    Cu = stc_SQP::Matrix::Zero(constraints_per_step_, 2);
    if (skip_last_step_ && local_step == segment_steps_ - 1) {
        return;
    }
    const int global_step = global_start_idx_ + local_step;
    const int row_start = global_step * constraints_per_step_;
    // Cx = C_matrix 对应行块, Cu = 0 (走廊约束仅依赖状态，与控制量无关)
    Cx = C_.middleRows(row_start, constraints_per_step_);
}

std::shared_ptr<stc_SQP::Constraint> StaticCorridorLinearConstraint::clone()
    const {
    return std::make_shared<StaticCorridorLinearConstraint>(
        C_, d_, global_start_idx_, constraints_per_step_, segment_steps_,
        skip_last_step_);
}

void StaticCorridorLinearConstraint::validateStepIndex(
    const stc_SQP::Vector& p, int local_step_idx) const {
    // p 非空校验已由 evaluate()/jacobian() 在访问 p(0) 前完成，
    // 此处不再重复；validateStepIndex 仅负责步索引边界有效性。
    (void)p;
    if (local_step_idx < 0) {
        throw std::invalid_argument(
            "StaticCorridorLinearConstraint: local_step_idx must be "
            "non-negative, got " +
            std::to_string(local_step_idx));
    }
    const int global_step = global_start_idx_ + local_step_idx;
    const int max_row =
        global_step * constraints_per_step_ + constraints_per_step_;
    if (max_row > C_.rows()) {
        throw std::invalid_argument(
            "StaticCorridorLinearConstraint: step " +
            std::to_string(global_step) + " requires rows up to " +
            std::to_string(max_row) + " but C has only " +
            std::to_string(C_.rows()) + " rows");
    }
}
}  // namespace apa_post_processor
