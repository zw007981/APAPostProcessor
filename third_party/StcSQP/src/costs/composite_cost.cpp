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
            throw std::invalid_argument("CompositeCost: terms cannot contain null pointer");
        }
    }
}

void CompositeCost::addTerm(std::shared_ptr<CostTerm> term)
{
    if (!term) {
        throw std::invalid_argument("CompositeCost::addTerm: term cannot be null");
    }
    terms_.push_back(std::move(term));
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

void CompositeCost::evaluateGradientAndHessian(const Vector& x, const Vector& u, double& cost,
    Vector& q, Vector& r, Matrix& Q, Matrix& R, Matrix& S) const
{
    const int nx = x.size();
    const int nu = u.size();
    cost = 0.0;
    q = Vector::Zero(nx);
    r = Vector::Zero(nu);
    Q = Matrix::Zero(nx, nx);
    R = Matrix::Zero(nu, nu);
    S = Matrix::Zero(nu, nx);
    double term_cost = 0.0;
    Vector term_q(nx), term_r(nu);
    Matrix term_Q(nx, nx), term_R(nu, nu), term_S(nu, nx);
    for (const auto& term : terms_) {
        term->evaluateGradientAndHessian(x, u, term_cost, term_q, term_r, term_Q, term_R, term_S);
        cost += term_cost;
        q += term_q;
        r += term_r;
        Q += term_Q;
        R += term_R;
        S += term_S;
    }
}

std::shared_ptr<CostTerm> CompositeCost::clone() const
{
    std::vector<std::shared_ptr<CostTerm>> cloned_terms;
    cloned_terms.reserve(terms_.size());
    for (const auto& term : terms_) {
        cloned_terms.push_back(term->clone());
    }
    return std::make_shared<CompositeCost>(std::move(cloned_terms));
}
} // namespace stc_SQP
