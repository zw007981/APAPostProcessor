#include <cmath>
#include <memory>
#include <vector>

#include <gtest/gtest.h>

#include "circle_obstacle_esdf_map.h"
#include "costs/circle_footprint_esdf_penalty_cost.h"
#include "costs/composite_cost.h"
#include "costs/quadratic_tracking.h"

using namespace stc_SQP;

// 测试目的：验证 QuadraticTrackingCost::clone() 生成的副本与原对象在 evaluate/gradient/hessian
// 以及组合求值接口上数值完全一致。
// 流程：构造原代价、clone 出副本；对同一 (x,u) 分别调用两种接口并比较结果。
// 预期效果：副本与原对象的所有输出在浮点容差内相等。
TEST(QuadraticTrackingCost, ClonePreservesNumerics)
{
    const int nx = 3, nu = 2;
    Vector x_ref(nx);
    x_ref << 1.0, 2.0, 0.5;
    Matrix Q = Matrix::Identity(nx, nx) * 2.0;
    Matrix R = Matrix::Identity(nu, nu) * 0.5;
    QuadraticTrackingCost original(x_ref, Q, R, /*theta_idx=*/2);
    auto cloned = original.clone();
    ASSERT_NE(cloned, nullptr);

    Vector x(nx), u(nu);
    x << 1.1, 1.9, 0.6;
    u << 0.2, -0.1;

    double cost_orig = 0.0, cost_clone = 0.0;
    Vector q_orig(nx), r_orig(nu), q_clone(nx), r_clone(nu);
    Matrix Q_out_orig(nx, nx), R_out_orig(nu, nu), S_out_orig(nu, nx);
    Matrix Q_out_clone(nx, nx), R_out_clone(nu, nu), S_out_clone(nu, nx);

    original.evaluate(x, u, cost_orig);
    cloned->evaluate(x, u, cost_clone);
    EXPECT_DOUBLE_EQ(cost_orig, cost_clone);

    original.gradient(x, u, q_orig, r_orig);
    cloned->gradient(x, u, q_clone, r_clone);
    EXPECT_TRUE(q_orig.isApprox(q_clone));
    EXPECT_TRUE(r_orig.isApprox(r_clone));

    original.hessian(x, u, Q_out_orig, R_out_orig, S_out_orig);
    cloned->hessian(x, u, Q_out_clone, R_out_clone, S_out_clone);
    EXPECT_TRUE(Q_out_orig.isApprox(Q_out_clone));
    EXPECT_TRUE(R_out_orig.isApprox(R_out_clone));
    EXPECT_TRUE(S_out_orig.isApprox(S_out_clone));

    double combo_cost_orig = 0.0, combo_cost_clone = 0.0;
    original.evaluateGradientAndHessian(x, u, combo_cost_orig, q_orig, r_orig, Q_out_orig,
        R_out_orig, S_out_orig);
    cloned->evaluateGradientAndHessian(x, u, combo_cost_clone, q_clone, r_clone, Q_out_clone,
        R_out_clone, S_out_clone);
    EXPECT_DOUBLE_EQ(combo_cost_orig, combo_cost_clone);
    EXPECT_TRUE(q_orig.isApprox(q_clone));
    EXPECT_TRUE(r_orig.isApprox(r_clone));
    EXPECT_TRUE(Q_out_orig.isApprox(Q_out_clone));
    EXPECT_TRUE(R_out_orig.isApprox(R_out_clone));
    EXPECT_TRUE(S_out_orig.isApprox(S_out_clone));
}

// 测试目的：验证 CircleFootprintEsdfPenaltyCost 的组合求值接口与分别调用
// evaluate/gradient/hessian 在数值上完全等价，从而确保 assembleCost 切换后行为不变。
// 流程：构造地图与代价，使部分圆违反安全裕度；分别通过两种模式计算 cost/q/r/Q/R/S 并比较。
// 预期效果：两种模式的所有输出在浮点容差内相等。
TEST(CircleFootprintEsdfPenaltyCost, EvaluateGradientAndHessianMatchesSeparateCalls)
{
    CircleObstacleEsdfMap map;
    map.addObstacle(Eigen::Vector2d(0.5, 0.0), 0.3);
    map.addObstacle(Eigen::Vector2d(-0.5, 0.0), 0.2);

    std::vector<Eigen::Vector2d> circles;
    circles.emplace_back(0.0, 0.0);
    circles.emplace_back(1.0, 0.0);
    CircleFootprintEsdfPenaltyCost cost(circles, /*circle_radius=*/0.2,
        /*safety_margin=*/0.05, map, /*penalty_weight=*/1e3);

    Vector x(5), u(2);
    x << 0.0, 0.0, 0.0, 0.0, 0.0;
    u << 0.0, 0.0;

    double cost_sep = 0.0, cost_combo = 0.0;
    Vector q_sep(5), r_sep(2), q_combo(5), r_combo(2);
    Matrix Q_sep(5, 5), R_sep(2, 2), S_sep(2, 5);
    Matrix Q_combo(5, 5), R_combo(2, 2), S_combo(2, 5);

    cost.evaluate(x, u, cost_sep);
    cost.gradient(x, u, q_sep, r_sep);
    cost.hessian(x, u, Q_sep, R_sep, S_sep);

    cost.evaluateGradientAndHessian(x, u, cost_combo, q_combo, r_combo, Q_combo, R_combo, S_combo);

    EXPECT_DOUBLE_EQ(cost_sep, cost_combo);
    EXPECT_TRUE(q_sep.isApprox(q_combo));
    EXPECT_TRUE(r_sep.isApprox(r_combo));
    EXPECT_TRUE(Q_sep.isApprox(Q_combo));
    EXPECT_TRUE(R_sep.isApprox(R_combo));
    EXPECT_TRUE(S_sep.isApprox(S_combo));
}

// 测试目的：验证 CompositeCost 的组合求值接口与分别调用各子代价的累加结果一致，
// 且 clone 后仍保持数值等价。
// 流程：构造由 QuadraticTrackingCost 与 CircleFootprintEsdfPenaltyCost 组成的 CompositeCost；
// 分别用两种模式求值并比较；再 clone 一次重复比较。
// 预期效果：组合求值输出等于分别调用之和；clone 副本与原对象输出一致。
TEST(CompositeCost, EvaluateGradientAndHessianAndClone)
{
    CircleObstacleEsdfMap map;
    map.addObstacle(Eigen::Vector2d(0.5, 0.0), 0.3);

    std::vector<Eigen::Vector2d> circles;
    circles.emplace_back(0.0, 0.0);
    CircleFootprintEsdfPenaltyCost penalty(circles, /*circle_radius=*/0.2,
        /*safety_margin=*/0.05, map, /*penalty_weight=*/1e3);

    const int nx = 5, nu = 2;
    Vector x_ref = Vector::Zero(nx);
    QuadraticTrackingCost tracking(x_ref, Matrix::Identity(nx, nx) * 1e-2,
        Matrix::Identity(nu, nu) * 1e-2, /*theta_idx=*/2);

    std::vector<std::shared_ptr<CostTerm>> terms;
    terms.push_back(tracking.clone());
    terms.push_back(penalty.clone());
    CompositeCost composite(terms);

    Vector x(nx), u(nu);
    x << 0.0, 0.0, 0.0, 0.0, 0.0;
    u << 0.0, 0.0;

    // 分别调用 evaluate/gradient/hessian 求和
    double sum_cost = 0.0;
    Vector sum_q = Vector::Zero(nx), sum_r = Vector::Zero(nu);
    Matrix sum_Q = Matrix::Zero(nx, nx), sum_R = Matrix::Zero(nu, nu), sum_S = Matrix::Zero(nu, nx);
    for (const auto& term : terms) {
        double c = 0.0;
        Vector q(nx), r(nu);
        Matrix Q(nx, nx), R(nu, nu), S(nu, nx);
        term->evaluate(x, u, c);
        term->gradient(x, u, q, r);
        term->hessian(x, u, Q, R, S);
        sum_cost += c;
        sum_q += q;
        sum_r += r;
        sum_Q += Q;
        sum_R += R;
        sum_S += S;
    }

    // 组合求值
    double combo_cost = 0.0;
    Vector combo_q(nx), combo_r(nu);
    Matrix combo_Q(nx, nx), combo_R(nu, nu), combo_S(nu, nx);
    composite.evaluateGradientAndHessian(x, u, combo_cost, combo_q, combo_r, combo_Q, combo_R,
        combo_S);

    EXPECT_DOUBLE_EQ(sum_cost, combo_cost);
    EXPECT_TRUE(sum_q.isApprox(combo_q));
    EXPECT_TRUE(sum_r.isApprox(combo_r));
    EXPECT_TRUE(sum_Q.isApprox(combo_Q));
    EXPECT_TRUE(sum_R.isApprox(combo_R));
    EXPECT_TRUE(sum_S.isApprox(combo_S));

    // clone 后数值仍应一致
    auto cloned = composite.clone();
    ASSERT_NE(cloned, nullptr);
    double cloned_cost = 0.0;
    Vector cloned_q(nx), cloned_r(nu);
    Matrix cloned_Q(nx, nx), cloned_R(nu, nu), cloned_S(nu, nx);
    cloned->evaluateGradientAndHessian(x, u, cloned_cost, cloned_q, cloned_r, cloned_Q, cloned_R,
        cloned_S);
    EXPECT_DOUBLE_EQ(combo_cost, cloned_cost);
    EXPECT_TRUE(combo_q.isApprox(cloned_q));
    EXPECT_TRUE(combo_r.isApprox(cloned_r));
    EXPECT_TRUE(combo_Q.isApprox(cloned_Q));
    EXPECT_TRUE(combo_R.isApprox(cloned_R));
    EXPECT_TRUE(combo_S.isApprox(cloned_S));
}

// 测试目的：验证 CircleFootprintEsdfPenaltyCost::clone() 的语义：
// 1) 副本与原对象数值等价（独立性）；2) 两者共享同一 EsdfMapInterface 引用。
// 流程：构造地图与原始代价，clone 出副本；先比较一次数值输出；再向地图添加新障碍物
// （改变共享 map 状态）后，比较副本与原始代价的输出仍保持一致（均受同一 map 影响）。
// 预期效果：clone 前后副本与原对象输出一致；map 变化后两者仍同步变化。
TEST(CircleFootprintEsdfPenaltyCost, ClonePreservesReferenceToSharedMap)
{
    CircleObstacleEsdfMap map;
    map.addObstacle(Eigen::Vector2d(0.5, 0.0), 0.3);

    std::vector<Eigen::Vector2d> circles;
    circles.emplace_back(0.0, 0.0);
    CircleFootprintEsdfPenaltyCost original(circles, /*circle_radius=*/0.2,
        /*safety_margin=*/0.05, map, /*penalty_weight=*/1e3);
    auto cloned = original.clone();
    ASSERT_NE(cloned, nullptr);

    Vector x(5), u(2);
    x << 0.0, 0.0, 0.0, 0.0, 0.0;
    u << 0.0, 0.0;

    double cost_orig = 0.0, cost_clone = 0.0;
    Vector q_orig(5), r_orig(2), q_clone(5), r_clone(2);
    Matrix Q_orig(5, 5), R_orig(2, 2), S_orig(2, 5);
    Matrix Q_clone(5, 5), R_clone(2, 2), S_clone(2, 5);

    original.evaluateGradientAndHessian(x, u, cost_orig, q_orig, r_orig, Q_orig, R_orig, S_orig);
    cloned->evaluateGradientAndHessian(x, u, cost_clone, q_clone, r_clone, Q_clone, R_clone,
        S_clone);
    EXPECT_DOUBLE_EQ(cost_orig, cost_clone);
    EXPECT_TRUE(q_orig.isApprox(q_clone));
    EXPECT_TRUE(Q_orig.isApprox(Q_clone));

    // 修改共享 map 状态，clone 副本应继续与原对象保持数值一致
    map.addObstacle(Eigen::Vector2d(-0.5, 0.0), 0.4);
    original.evaluateGradientAndHessian(x, u, cost_orig, q_orig, r_orig, Q_orig, R_orig, S_orig);
    cloned->evaluateGradientAndHessian(x, u, cost_clone, q_clone, r_clone, Q_clone, R_clone,
        S_clone);
    EXPECT_DOUBLE_EQ(cost_orig, cost_clone);
    EXPECT_TRUE(q_orig.isApprox(q_clone));
    EXPECT_TRUE(Q_orig.isApprox(Q_clone));
}
