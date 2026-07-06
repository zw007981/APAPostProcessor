#include "convex_corridor_constraint.h"

#include <stdexcept>
#include <string>

namespace stc_SQP {
ConvexCorridorConstraint::ConvexCorridorConstraint(int nu)
    : nu_(nu)
    , func_(reinterpret_cast<casadi::FunctionPointer>(corridor),
          reinterpret_cast<casadi::WorkSizeFunction>(corridor_work), 2, 2)
{
    if (nu_ < 0) {
        throw std::invalid_argument(
            "ConvexCorridorConstraint: control dimension nu cannot be negative, got " + std::to_string(nu_));
    }
    cx_dummy_.resize(CORRIDOR_CX_NNZ);
    g_tmp_.resize(CORRIDOR_G_DIM);
    arg_scratch_.resize(2);
    res_scratch_.resize(2);
}

ConvexCorridorConstraint::ConvexCorridorConstraint(const Vector& p, int nu)
    : ConvexCorridorConstraint(nu)
{
    validateParameters(p);
}

int ConvexCorridorConstraint::ng() const { return CORRIDOR_G_DIM; }

void ConvexCorridorConstraint::evaluate(const Vector& x, const Vector& u, const Vector& p,
    Vector& g) const
{
    validateInputDimensions(x, u);
    validateParameters(p);
    g.resize(CORRIDOR_G_DIM);
    doEvaluate(x, p, g, cx_dummy_.data());
}

void ConvexCorridorConstraint::jacobian(const Vector& x, const Vector& u, const Vector& p,
    Matrix& Cx, Matrix& Cu) const
{
    validateInputDimensions(x, u);
    validateParameters(p);
    Cx.resize(CORRIDOR_G_DIM, CORRIDOR_NX);
    Cu = Matrix::Zero(CORRIDOR_G_DIM, nu_);
    doEvaluate(x, p, g_tmp_, Cx.data());
}

void ConvexCorridorConstraint::evaluateAndJacobian(const Vector& x, const Vector& u,
    const Vector& p, Vector& g, Matrix& Cx, Matrix& Cu) const
{
    validateInputDimensions(x, u);
    validateParameters(p);
    g.resize(CORRIDOR_G_DIM);
    Cx.resize(CORRIDOR_G_DIM, CORRIDOR_NX);
    Cu = Matrix::Zero(CORRIDOR_G_DIM, nu_);
    doEvaluate(x, p, g, Cx.data());
}

std::shared_ptr<Constraint> ConvexCorridorConstraint::clone() const
{
    return std::make_shared<ConvexCorridorConstraint>(nu_);
}

void ConvexCorridorConstraint::validateInputDimensions(const Vector& x, const Vector& u) const
{
    if (x.size() != CORRIDOR_NX) {
        throw std::invalid_argument(
            "ConvexCorridorConstraint: state x dimension must be " + std::to_string(CORRIDOR_NX)
            + ", got " + std::to_string(x.size()));
    }
    if (u.size() != nu_) {
        throw std::invalid_argument(
            "ConvexCorridorConstraint: control u dimension must be " + std::to_string(nu_)
            + ", got " + std::to_string(u.size()));
    }
}

void ConvexCorridorConstraint::validateParameters(const Vector& p)
{
    if (p.size() != CORRIDOR_P_DIM) {
        throw std::invalid_argument(
            "ConvexCorridorConstraint: parameter p dimension must be " + std::to_string(CORRIDOR_P_DIM)
            + ", got " + std::to_string(p.size()));
    }
}

void ConvexCorridorConstraint::doEvaluate(const Vector& x, const Vector& p, Vector& g,
    double* cx_buffer) const
{
    arg_scratch_[0] = x.data();
    arg_scratch_[1] = p.data();
    res_scratch_[0] = g.data();
    res_scratch_[1] = cx_buffer;
    func_(arg_scratch_, res_scratch_);
}
} // namespace stc_SQP
