#include "theta_trust_region_constraint.h"

#include <stdexcept>

namespace apa_post_processor {
ThetaTrustRegionConstraint::ThetaTrustRegionConstraint(double delta_theta_max)
    : delta_theta_max_(delta_theta_max) {
    if (delta_theta_max_ < 0.0 || !std::isfinite(delta_theta_max_)) {
        throw std::invalid_argument(
            "ThetaTrustRegionConstraint: delta_theta_max must be "
            "non-negative finite, got " +
            std::to_string(delta_theta_max_));
    }
}

int ThetaTrustRegionConstraint::ng() const { return 2; }

void ThetaTrustRegionConstraint::evaluate(const stc_SQP::Vector& x,
                                          const stc_SQP::Vector& u,
                                          const stc_SQP::Vector& p,
                                          stc_SQP::Vector& g) const {
    (void)u;
    validateInputs(x, p);
    // p(1) = theta_ref
    const double theta_ref = p(1);
    g.resize(2);
    // theta - (theta_ref + delta_theta_max) <= 0
    g(0) = x(2) - theta_ref - delta_theta_max_;
    // (theta_ref - delta_theta_max) - theta <= 0
    g(1) = theta_ref - delta_theta_max_ - x(2);
}

void ThetaTrustRegionConstraint::jacobian(const stc_SQP::Vector& x,
                                          const stc_SQP::Vector& u,
                                          const stc_SQP::Vector& p,
                                          stc_SQP::Matrix& Cx,
                                          stc_SQP::Matrix& Cu) const {
    (void)u;
    validateInputs(x, p);
    // Cx 按 x.size() 动态构造
    Cx = stc_SQP::Matrix::Zero(2, x.size());
    Cu = stc_SQP::Matrix::Zero(2, 2);
    // dg0/dtheta = 1
    Cx(0, 2) = 1.0;
    // dg1/dtheta = -1
    Cx(1, 2) = -1.0;
}

std::shared_ptr<stc_SQP::Constraint> ThetaTrustRegionConstraint::clone() const {
    return std::make_shared<ThetaTrustRegionConstraint>(delta_theta_max_);
}

void ThetaTrustRegionConstraint::validateInputs(
    const stc_SQP::Vector& x, const stc_SQP::Vector& p) const {
    if (x.size() < 5) {
        throw std::invalid_argument(
            "ThetaTrustRegionConstraint: x must be at least 5-dimensional "
            "(x,y,theta,v,delta,...), got " +
            std::to_string(x.size()));
    }
    if (p.size() < 2) {
        throw std::invalid_argument(
            "ThetaTrustRegionConstraint: p must have size >= 2 with p(1) = "
            "theta_ref, got size " +
            std::to_string(p.size()));
    }
}
}  // namespace apa_post_processor
