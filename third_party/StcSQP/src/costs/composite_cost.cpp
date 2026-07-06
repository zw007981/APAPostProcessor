#include "composite_cost.h"

#include <stdexcept>

namespace stc_SQP {
CompositeCost::CompositeCost(std::vector<std::shared_ptr<CostTerm>> terms)
    : terms_(std::move(terms))
{
    if (terms_.empty()) {
        throw std::invalid_argument("CompositeCost: terms cannot be empty");
    }
    for (const auto& term : terms_) {
        if (!term) {
            throw std::invalid_argument("CompositeCost: terms cannot contain null pointers");
        }
    }
}

void CompositeCost::evaluate(const Vector& x, const Vector& u, double& cost) const
{
    cost = 0.0;
    double term_cost = 0.0;
    for (const auto& term : terms_) {
        term->evaluate(x, u, term_cost);
        cost += term_cost;
    }
}

void CompositeCost::gradient(const Vector& x, const Vector& u, Vector& q, Vector& r) const
{
    q = Vector::Zero(x.size());
    r = Vector::Zero(u.size());
    Vector term_q, term_r;
    for (const auto& term : terms_) {
        term->gradient(x, u, term_q, term_r);
        q += term_q;
        r += term_r;
    }
}

void CompositeCost::hessian(const Vector& x, const Vector& u, Matrix& Q, Matrix& R, Matrix& S) const
{
    Q = Matrix::Zero(x.size(), x.size());
    R = Matrix::Zero(u.size(), u.size());
    S = Matrix::Zero(u.size(), x.size());
    Matrix term_Q, term_R, term_S;
    for (const auto& term : terms_) {
        term->hessian(x, u, term_Q, term_R, term_S);
        Q += term_Q;
        R += term_R;
        S += term_S;
    }
}
} // namespace stc_SQP
