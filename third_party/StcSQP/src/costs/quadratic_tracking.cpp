#include "quadratic_tracking.h"

#include <cmath>
#include <stdexcept>
#include <string>

#include <Eigen/Eigenvalues>

#include "math/math_util.hpp"

namespace stc_SQP {

QuadraticTrackingCost::QuadraticTrackingCost(const Vector& x_ref, const Matrix& Q, const Matrix& R,
    int theta_idx)
    : x_ref_(x_ref)
    , Q_(Q)
    , R_(R)
    , theta_idx_(theta_idx)
{
    const int nx = x_ref_.size();
    if (Q_.rows() != nx || Q_.cols() != nx) {
        throw std::invalid_argument("QuadraticTrackingCost: Q dimension must be nx x nx");
    }
    if (R_.rows() != R_.cols()) {
        throw std::invalid_argument("QuadraticTrackingCost: R must be square");
    }
    if (!Q_.isApprox(Q_.transpose())) {
        throw std::invalid_argument("QuadraticTrackingCost: Q must be symmetric");
    }
    if (!R_.isApprox(R_.transpose())) {
        throw std::invalid_argument("QuadraticTrackingCost: R must be symmetric");
    }
    validatePositiveSemidefinite(Q_, "Q");
    validatePositiveSemidefinite(R_, "R");
    if (theta_idx_ < -1 || theta_idx_ >= nx) {
        throw std::invalid_argument("QuadraticTrackingCost: theta_idx out of range");
    }
}

void QuadraticTrackingCost::evaluate(const Vector& x, const Vector& u, double& cost) const
{
    validateInputDimensions(x, u);
    const Vector dx = computeStateError(x);
    cost = 0.5 * dx.dot(Q_ * dx) + 0.5 * u.dot(R_ * u);
}

void QuadraticTrackingCost::gradient(const Vector& x, const Vector& u, Vector& q, Vector& r) const
{
    validateInputDimensions(x, u);
    const Vector dx = computeStateError(x);
    q = Q_ * dx;
    r = R_ * u;
}

void QuadraticTrackingCost::hessian(const Vector& x, const Vector& u, Matrix& Q, Matrix& R, Matrix& S) const
{
    validateInputDimensions(x, u);
    (void)x;
    (void)u;
    Q = Q_;
    R = R_;
    S = Matrix::Zero(R_.cols(), x_ref_.size());
}

void QuadraticTrackingCost::validatePositiveSemidefinite(const Matrix& M, const char* name) const
{
    Eigen::SelfAdjointEigenSolver<Matrix> solver(M);
    if (solver.eigenvalues().minCoeff() < -EPSILON_PRECISE) {
        throw std::invalid_argument(std::string("QuadraticTrackingCost: ") + name + " must be positive semidefinite");
    }
}

void QuadraticTrackingCost::validateInputDimensions(const Vector& x, const Vector& u) const
{
    if (x.size() != x_ref_.size()) {
        throw std::invalid_argument("QuadraticTrackingCost: x dimension does not match x_ref");
    }
    if (u.size() != R_.cols()) {
        throw std::invalid_argument("QuadraticTrackingCost: u dimension does not match R");
    }
}

Vector QuadraticTrackingCost::computeStateError(const Vector& x) const
{
    Vector dx = x - x_ref_;
    if (theta_idx_ >= 0) {
        dx(theta_idx_) = math_util::NormalizeAngle(x(theta_idx_) - x_ref_(theta_idx_));
    }
    return dx;
}
} // namespace stc_SQP
