#include "position_trust_region_constraint.h"

#include <stdexcept>

namespace apa_post_processor {
PositionTrustRegionConstraint::PositionTrustRegionConstraint(
    double delta_xy_max)
    : delta_xy_max_(delta_xy_max) {
    if (delta_xy_max_ < 0.0 || !std::isfinite(delta_xy_max_)) {
        throw std::invalid_argument(
            "PositionTrustRegionConstraint: delta_xy_max must be "
            "non-negative finite, got " +
            std::to_string(delta_xy_max_));
    }
}

int PositionTrustRegionConstraint::ng() const { return 4; }

void PositionTrustRegionConstraint::evaluate(const stc_SQP::Vector& x,
                                             const stc_SQP::Vector& u,
                                             const stc_SQP::Vector& p,
                                             stc_SQP::Vector& g) const {
    (void)u;
    validateInputs(x, p);
    const double x_ref = p(3);
    const double y_ref = p(4);
    g.resize(4);
    // x upper/lower bounds
    g(0) = x(0) - x_ref - delta_xy_max_;
    g(1) = x_ref - delta_xy_max_ - x(0);
    // y upper/lower bounds
    g(2) = x(1) - y_ref - delta_xy_max_;
    g(3) = y_ref - delta_xy_max_ - x(1);
}

void PositionTrustRegionConstraint::jacobian(const stc_SQP::Vector& x,
                                             const stc_SQP::Vector& u,
                                             const stc_SQP::Vector& p,
                                             stc_SQP::Matrix& Cx,
                                             stc_SQP::Matrix& Cu) const {
    (void)u;
    validateInputs(x, p);
    // Cx 按 x.size() 动态构造
    Cx = stc_SQP::Matrix::Zero(4, x.size());
    Cu = stc_SQP::Matrix::Zero(4, 2);
    // dg0/dx = 1, dg1/dx = -1
    Cx(0, 0) = 1.0;
    Cx(1, 0) = -1.0;
    // dg2/dy = 1, dg3/dy = -1
    Cx(2, 1) = 1.0;
    Cx(3, 1) = -1.0;
}

std::shared_ptr<stc_SQP::Constraint> PositionTrustRegionConstraint::clone()
    const {
    return std::make_shared<PositionTrustRegionConstraint>(delta_xy_max_);
}

void PositionTrustRegionConstraint::validateInputs(
    const stc_SQP::Vector& x, const stc_SQP::Vector& p) const {
    if (x.size() < 5) {
        throw std::invalid_argument(
            "PositionTrustRegionConstraint: x must be at least "
            "5-dimensional (x,y,theta,v,delta,...), got " +
            std::to_string(x.size()));
    }
    if (p.size() < 5) {
        throw std::invalid_argument(
            "PositionTrustRegionConstraint: p must have size >= 5 with "
            "p(3)=x_ref, p(4)=y_ref, got size " +
            std::to_string(p.size()));
    }
}
}  // namespace apa_post_processor
