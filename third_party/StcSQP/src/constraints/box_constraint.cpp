#include "box_constraint.h"

#include <stdexcept>

namespace stc_SQP {
BoxConstraint::BoxConstraint(const Vector& x_min, const Vector& x_max,
    const Vector& u_min, const Vector& u_max)
    : x_min_(x_min)
    , x_max_(x_max)
    , u_min_(u_min)
    , u_max_(u_max)
{
    if (x_min_.size() != x_max_.size()) {
        throw std::invalid_argument("BoxConstraint: x_min and x_max dimensions do not match");
    }
    if (u_min_.size() != u_max_.size()) {
        throw std::invalid_argument("BoxConstraint: u_min and u_max dimensions do not match");
    }
    for (int i = 0; i < x_min_.size(); ++i) {
        if (x_min_(i) > x_max_(i)) {
            throw std::invalid_argument("BoxConstraint: x_min has components greater than x_max");
        }
    }
    for (int i = 0; i < u_min_.size(); ++i) {
        if (u_min_(i) > u_max_(i)) {
            throw std::invalid_argument("BoxConstraint: u_min has components greater than u_max");
        }
    }
}

int BoxConstraint::ng() const { return 2 * x_min_.size() + 2 * u_min_.size(); }

void BoxConstraint::evaluate(const Vector& x, const Vector& u, const Vector& p, Vector& g) const
{
    (void)p;
    validateInputDimensions(x, u);
    const int nx = x_min_.size();
    const int nu = u_min_.size();
    g.resize(ng());
    g.head(nx) = x_min_ - x;
    g.segment(nx, nx) = x - x_max_;
    g.segment(2 * nx, nu) = u_min_ - u;
    g.tail(nu) = u - u_max_;
}

std::shared_ptr<Constraint> BoxConstraint::clone() const
{
    return std::make_shared<BoxConstraint>(x_min_, x_max_, u_min_, u_max_);
}

void BoxConstraint::jacobian(const Vector& x, const Vector& u, const Vector& p,
    Matrix& Cx, Matrix& Cu) const
{
    (void)p;
    validateInputDimensions(x, u);
    const int nx = x_min_.size();
    const int nu = u_min_.size();
    const int ng_val = ng();
    Cx = Matrix::Zero(ng_val, nx);
    Cu = Matrix::Zero(ng_val, nu);
    // x_min - x <= 0: Cx = -I
    Cx.topRows(nx) = -Matrix::Identity(nx, nx);
    // x - x_max <= 0: Cx = +I
    Cx.middleRows(nx, nx) = Matrix::Identity(nx, nx);
    // u_min - u <= 0: Cu = -I
    Cu.middleRows(2 * nx, nu) = -Matrix::Identity(nu, nu);
    // u - u_max <= 0: Cu = +I
    Cu.bottomRows(nu) = Matrix::Identity(nu, nu);
}

void BoxConstraint::validateInputDimensions(const Vector& x, const Vector& u) const
{
    if (x.size() != x_min_.size()) {
        throw std::invalid_argument("BoxConstraint: x dimension does not match x_min");
    }
    if (u.size() != u_min_.size()) {
        throw std::invalid_argument("BoxConstraint: u dimension does not match u_min");
    }
}
} // namespace stc_SQP
