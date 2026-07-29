#include <gtest/gtest.h>

#include <nlohmann/json.hpp>
#include <type_traits>

#include "core/post_processor.h"

namespace apa_post_processor {
namespace {

// 派生配置对象必须保持可拷贝/可移动（基类 protected 特殊成员不阻碍派生类的
// 隐式特殊成员），供场景层与编排层按值传递与局部复制
static_assert(std::is_copy_constructible_v<DdpConfig>);
static_assert(std::is_copy_assignable_v<DdpConfig>);
static_assert(std::is_move_constructible_v<DdpConfig>);

// 测试场景：默认构造的 DdpConfig。
// 预期行为：承载 DDP.md 2.5 节参数表建议默认值（w_j=1.0、w_eta=1.0、
// w_ref,0=10.0、w_theta=5.0、gamma_anneal=0.5、n_s=25、mu_min/mu_max/phi=
// 1e2/1e6/10、容差 0.05 m/1.5°/1e-3、内层/外层迭代上限 50/20、v_max=1.5、
// a_max=1.0、delta_max=0.55、omega_max=0.5、j_max=1.5、eta_max=1.0、
// margin_safe/margin_comf=0.02/0.10、stride=1、epsilon_v=0.02、v_dwell=0.05、
// T_shift=0.4、kappa_pad=1.2），且编排层默认标定（merit_kappa_d=1e9、
// merit_mu0=100、cost_change_tol=1e-9）不被 DdpConfig 构造破坏。
TEST(DdpConfigTest, DefaultsMatchDesignParameterTable) {
    const DdpConfig config;
    // 参考构建：重采样间距/固定步长/打靶间隔与初值裁剪盒边界
    EXPECT_DOUBLE_EQ(config.reference.sample_dist, 0.05);
    EXPECT_DOUBLE_EQ(config.reference.dt, 0.1);
    EXPECT_EQ(config.reference.shooting_interval, 25);
    EXPECT_DOUBLE_EQ(config.reference.v_max, 1.5);
    EXPECT_DOUBLE_EQ(config.reference.a_max, 1.0);
    EXPECT_DOUBLE_EQ(config.reference.delta_max, 0.55);
    EXPECT_DOUBLE_EQ(config.reference.omega_max, 0.5);
    // 内层 MS-iLQR：控制盒与迭代上限；编排层标定值（merit 钉住/容差收紧）
    EXPECT_DOUBLE_EQ(config.solver.inner.jerk_max, 1.5);
    EXPECT_DOUBLE_EQ(config.solver.inner.steer_accel_max, 1.0);
    EXPECT_EQ(config.solver.inner.max_iterations, 50);
    EXPECT_DOUBLE_EQ(config.solver.inner.cost_change_tol, 1e-9);
    EXPECT_DOUBLE_EQ(config.solver.inner.merit_mu0, 100.0);
    EXPECT_DOUBLE_EQ(config.solver.inner.merit_kappa_d, 1e9);
    // 外层 AL：迭代上限/双指标/缺陷容差/罚权重调度/退火率
    EXPECT_EQ(config.solver.outer.max_outer_iterations, 20);
    EXPECT_DOUBLE_EQ(config.solver.outer.terminal_position_tol, 0.05);
    EXPECT_DOUBLE_EQ(config.solver.outer.terminal_heading_tol_deg, 1.5);
    EXPECT_DOUBLE_EQ(config.solver.outer.defect_tol, 1e-3);
    EXPECT_DOUBLE_EQ(config.solver.outer.mu_min, 1e2);
    EXPECT_DOUBLE_EQ(config.solver.outer.mu_max, 1e6);
    EXPECT_DOUBLE_EQ(config.solver.outer.mu_growth_factor, 10.0);
    EXPECT_DOUBLE_EQ(config.solver.outer.anneal_gamma, 0.5);
    // 代价权重：平滑主项基准 w_j=1.0、w_eta=1.0、w_ref,0=10.0、w_theta=5.0、
    // 换挡代理默认关闭
    EXPECT_DOUBLE_EQ(config.solver.cost.weight_jerk, 1.0);
    EXPECT_DOUBLE_EQ(config.solver.cost.weight_steer_accel, 1.0);
    EXPECT_DOUBLE_EQ(config.solver.cost.weight_ref_base, 10.0);
    EXPECT_DOUBLE_EQ(config.solver.cost.weight_theta, 5.0);
    EXPECT_DOUBLE_EQ(config.solver.cost.weight_shift, 0.0);
    EXPECT_DOUBLE_EQ(config.solver.cost.shift_beta, 0.1);
    // 阶段二编排：外层上限 8、门控 mu 初值 10、门控容差 1e-2
    EXPECT_EQ(config.solver.stage_two_max_outer_iterations, 8);
    EXPECT_DOUBLE_EQ(config.solver.gating_mu_initial, 10.0);
    EXPECT_DOUBLE_EQ(config.solver.gating_mu_max, 1e6);
    EXPECT_DOUBLE_EQ(config.solver.gating_tol, 1e-2);
    // ESDF 双 margin 惩罚
    EXPECT_DOUBLE_EQ(config.esdf.margin_safe, 0.02);
    EXPECT_DOUBLE_EQ(config.esdf.margin_comf, 0.10);
    EXPECT_DOUBLE_EQ(config.esdf.weight_safe, 100.0);
    EXPECT_DOUBLE_EQ(config.esdf.weight_comf, 1.0);
    EXPECT_EQ(config.esdf.stride, 1);
    // 后处理与阶段二门控精化
    EXPECT_DOUBLE_EQ(config.post_stage.epsilon_v, 0.02);
    EXPECT_DOUBLE_EQ(config.post_stage.v_dwell, 0.05);
    EXPECT_DOUBLE_EQ(config.post_stage.shift_delay, 0.4);
    EXPECT_DOUBLE_EQ(config.post_stage.kappa_pad, 1.2);
    EXPECT_DOUBLE_EQ(config.post_stage.seam_speed_tol, 0.02);
    EXPECT_DOUBLE_EQ(config.post_stage.amplitude_check_tol, 0.05);
    EXPECT_DOUBLE_EQ(config.post_stage.control_overshoot_tol, 0.3);
}

// 测试场景：默认构造与显式调用 synchronizeAmplitudeBounds。
// 预期行为：状态幅值边界（v_max/a_max/delta_max/omega_max）的唯一权威来源是
// reference 子配置、eta_max 的唯一权威来源是 inner.steer_accel_max——默认构造
// 后 cost/post_stage 的同源字段已与权威来源一致；改写权威来源并同步后，全部
// 消费方跟随更新（Config 双源缺口历史教训的前置防御）。
TEST(DdpConfigTest, AmplitudeBoundsSyncFromSingleSource) {
    DdpConfig config;
    EXPECT_DOUBLE_EQ(config.solver.cost.v_max, config.reference.v_max);
    EXPECT_DOUBLE_EQ(config.solver.cost.a_max, config.reference.a_max);
    EXPECT_DOUBLE_EQ(config.solver.cost.delta_max, config.reference.delta_max);
    EXPECT_DOUBLE_EQ(config.solver.cost.omega_max, config.reference.omega_max);
    EXPECT_DOUBLE_EQ(config.post_stage.omega_max, config.reference.omega_max);
    EXPECT_DOUBLE_EQ(config.post_stage.eta_max,
                     config.solver.inner.steer_accel_max);
    config.reference.v_max = 2.0;
    config.reference.a_max = 1.2;
    config.reference.delta_max = 0.6;
    config.reference.omega_max = 0.7;
    config.solver.inner.steer_accel_max = 1.3;
    config.synchronizeAmplitudeBounds();
    EXPECT_DOUBLE_EQ(config.solver.cost.v_max, 2.0);
    EXPECT_DOUBLE_EQ(config.solver.cost.a_max, 1.2);
    EXPECT_DOUBLE_EQ(config.solver.cost.delta_max, 0.6);
    EXPECT_DOUBLE_EQ(config.solver.cost.omega_max, 0.7);
    EXPECT_DOUBLE_EQ(config.post_stage.omega_max, 0.7);
    EXPECT_DOUBLE_EQ(config.post_stage.eta_max, 1.3);
}

// 测试场景：JSON 显式给出全部可映射字段（区别于默认值）。
// 预期行为：每个字段都被正确读入；reference 节的幅值键同步进 cost/post_stage
// 同源字段，inner.steer_accel_max 同步进 post_stage.eta_max。
TEST(DdpConfigTest, LoadFromJsonOverridesAllFields) {
    const auto details = nlohmann::json::parse(R"json({
        "algorithm": "ddp",
        "reference": {
            "sample_dist": 0.08, "dt": 0.2, "shooting_interval": 10,
            "v_max": 2.0, "a_max": 1.2, "delta_max": 0.6, "omega_max": 0.7
        },
        "solver": {
            "stage_two_max_outer_iterations": 5,
            "gating_mu_initial": 20.0, "gating_mu_max": 1e5, "gating_tol": 0.05,
            "inner": {
                "jerk_max": 2.5, "steer_accel_max": 1.3, "max_iterations": 30,
                "cost_change_tol": 1e-8, "gradient_tol": 1e-7,
                "reg_initial": 1e-3, "reg_min": 1e-8, "reg_max": 1e8,
                "reg_increase": 5.0, "reg_decrease": 0.6,
                "armijo_gamma": 0.2, "backtrack_beta": 0.4, "max_backtracks": 40,
                "merit_mu0": 50.0, "merit_rho": 0.6, "merit_kappa_d": 1e-5,
                "inter_segment_weight": 0.2
            },
            "outer": {
                "max_outer_iterations": 15, "terminal_position_tol": 0.08,
                "terminal_heading_tol_deg": 2.0, "inequality_tol": 0.05,
                "defect_tol": 1e-4, "mu_min": 10.0, "mu_max": 1e5,
                "first_round_mu": 2.0, "amplitude_mu_initial": 3.0,
                "epsilon_mu": 1e-3, "mu_gate_kappa": 0.8,
                "mu_growth_factor": 5.0, "anneal_gamma": 0.6
            },
            "cost": {
                "weight_jerk": 2.0, "weight_steer_accel": 3.0,
                "weight_ref_base": 20.0, "weight_theta": 8.0,
                "weight_shift": 1.0, "shift_beta": 0.2
            }
        },
        "esdf": {
            "margin_safe": 0.03, "margin_comf": 0.15,
            "weight_safe": 200.0, "weight_comf": 2.0, "stride": 2
        },
        "post_stage": {
            "epsilon_v": 0.03, "v_dwell": 0.08, "shift_delay": 0.5,
            "kappa_pad": 1.5, "seam_speed_tol": 0.03, "dwell_omega_tol": 0.2,
            "amplitude_check_tol": 0.08, "control_overshoot_tol": 0.4
        }
    })json");
    DdpConfig config;
    LoadDdpConfigOverrides(details, &config);
    EXPECT_DOUBLE_EQ(config.reference.sample_dist, 0.08);
    EXPECT_DOUBLE_EQ(config.reference.dt, 0.2);
    EXPECT_EQ(config.reference.shooting_interval, 10);
    EXPECT_DOUBLE_EQ(config.reference.v_max, 2.0);
    EXPECT_DOUBLE_EQ(config.reference.a_max, 1.2);
    EXPECT_DOUBLE_EQ(config.reference.delta_max, 0.6);
    EXPECT_DOUBLE_EQ(config.reference.omega_max, 0.7);
    EXPECT_EQ(config.solver.stage_two_max_outer_iterations, 5);
    EXPECT_DOUBLE_EQ(config.solver.gating_mu_initial, 20.0);
    EXPECT_DOUBLE_EQ(config.solver.gating_mu_max, 1e5);
    EXPECT_DOUBLE_EQ(config.solver.gating_tol, 0.05);
    EXPECT_DOUBLE_EQ(config.solver.inner.jerk_max, 2.5);
    EXPECT_DOUBLE_EQ(config.solver.inner.steer_accel_max, 1.3);
    EXPECT_EQ(config.solver.inner.max_iterations, 30);
    EXPECT_DOUBLE_EQ(config.solver.inner.cost_change_tol, 1e-8);
    EXPECT_DOUBLE_EQ(config.solver.inner.gradient_tol, 1e-7);
    EXPECT_DOUBLE_EQ(config.solver.inner.reg_initial, 1e-3);
    EXPECT_DOUBLE_EQ(config.solver.inner.reg_min, 1e-8);
    EXPECT_DOUBLE_EQ(config.solver.inner.reg_max, 1e8);
    EXPECT_DOUBLE_EQ(config.solver.inner.reg_increase, 5.0);
    EXPECT_DOUBLE_EQ(config.solver.inner.reg_decrease, 0.6);
    EXPECT_DOUBLE_EQ(config.solver.inner.armijo_gamma, 0.2);
    EXPECT_DOUBLE_EQ(config.solver.inner.backtrack_beta, 0.4);
    EXPECT_EQ(config.solver.inner.max_backtracks, 40);
    EXPECT_DOUBLE_EQ(config.solver.inner.merit_mu0, 50.0);
    EXPECT_DOUBLE_EQ(config.solver.inner.merit_rho, 0.6);
    EXPECT_DOUBLE_EQ(config.solver.inner.merit_kappa_d, 1e-5);
    EXPECT_DOUBLE_EQ(config.solver.inner.inter_segment_weight, 0.2);
    EXPECT_EQ(config.solver.outer.max_outer_iterations, 15);
    EXPECT_DOUBLE_EQ(config.solver.outer.terminal_position_tol, 0.08);
    EXPECT_DOUBLE_EQ(config.solver.outer.terminal_heading_tol_deg, 2.0);
    EXPECT_DOUBLE_EQ(config.solver.outer.inequality_tol, 0.05);
    EXPECT_DOUBLE_EQ(config.solver.outer.defect_tol, 1e-4);
    EXPECT_DOUBLE_EQ(config.solver.outer.mu_min, 10.0);
    EXPECT_DOUBLE_EQ(config.solver.outer.mu_max, 1e5);
    EXPECT_DOUBLE_EQ(config.solver.outer.first_round_mu, 2.0);
    EXPECT_DOUBLE_EQ(config.solver.outer.amplitude_mu_initial, 3.0);
    EXPECT_DOUBLE_EQ(config.solver.outer.epsilon_mu, 1e-3);
    EXPECT_DOUBLE_EQ(config.solver.outer.mu_gate_kappa, 0.8);
    EXPECT_DOUBLE_EQ(config.solver.outer.mu_growth_factor, 5.0);
    EXPECT_DOUBLE_EQ(config.solver.outer.anneal_gamma, 0.6);
    EXPECT_DOUBLE_EQ(config.solver.cost.weight_jerk, 2.0);
    EXPECT_DOUBLE_EQ(config.solver.cost.weight_steer_accel, 3.0);
    EXPECT_DOUBLE_EQ(config.solver.cost.weight_ref_base, 20.0);
    EXPECT_DOUBLE_EQ(config.solver.cost.weight_theta, 8.0);
    EXPECT_DOUBLE_EQ(config.solver.cost.weight_shift, 1.0);
    EXPECT_DOUBLE_EQ(config.solver.cost.shift_beta, 0.2);
    EXPECT_DOUBLE_EQ(config.esdf.margin_safe, 0.03);
    EXPECT_DOUBLE_EQ(config.esdf.margin_comf, 0.15);
    EXPECT_DOUBLE_EQ(config.esdf.weight_safe, 200.0);
    EXPECT_DOUBLE_EQ(config.esdf.weight_comf, 2.0);
    EXPECT_EQ(config.esdf.stride, 2);
    EXPECT_DOUBLE_EQ(config.post_stage.epsilon_v, 0.03);
    EXPECT_DOUBLE_EQ(config.post_stage.v_dwell, 0.08);
    EXPECT_DOUBLE_EQ(config.post_stage.shift_delay, 0.5);
    EXPECT_DOUBLE_EQ(config.post_stage.kappa_pad, 1.5);
    EXPECT_DOUBLE_EQ(config.post_stage.seam_speed_tol, 0.03);
    EXPECT_DOUBLE_EQ(config.post_stage.dwell_omega_tol, 0.2);
    EXPECT_DOUBLE_EQ(config.post_stage.amplitude_check_tol, 0.08);
    EXPECT_DOUBLE_EQ(config.post_stage.control_overshoot_tol, 0.4);
    // 幅值边界经 reference/inner 单一来源同步进全部消费方
    EXPECT_DOUBLE_EQ(config.solver.cost.v_max, 2.0);
    EXPECT_DOUBLE_EQ(config.solver.cost.a_max, 1.2);
    EXPECT_DOUBLE_EQ(config.solver.cost.delta_max, 0.6);
    EXPECT_DOUBLE_EQ(config.solver.cost.omega_max, 0.7);
    EXPECT_DOUBLE_EQ(config.post_stage.omega_max, 0.7);
    EXPECT_DOUBLE_EQ(config.post_stage.eta_max, 1.3);
}

// 测试场景：JSON 仅含 algorithm 路由字段（无任何 DDP 专有节）。
// 预期行为：全部字段回落到 2.5 节建议默认值（与默认构造逐字段一致）。
TEST(DdpConfigTest, LoadFromJsonKeepsDefaultsWhenAbsent) {
    const auto details =
        nlohmann::json::parse(R"json({"algorithm": "ddp"})json");
    DdpConfig config;
    LoadDdpConfigOverrides(details, &config);
    const DdpConfig fresh;
    EXPECT_DOUBLE_EQ(config.reference.sample_dist, fresh.reference.sample_dist);
    EXPECT_DOUBLE_EQ(config.reference.dt, fresh.reference.dt);
    EXPECT_EQ(config.reference.shooting_interval,
              fresh.reference.shooting_interval);
    EXPECT_DOUBLE_EQ(config.reference.v_max, fresh.reference.v_max);
    EXPECT_DOUBLE_EQ(config.solver.inner.jerk_max, fresh.solver.inner.jerk_max);
    EXPECT_EQ(config.solver.inner.max_iterations,
              fresh.solver.inner.max_iterations);
    EXPECT_DOUBLE_EQ(config.solver.inner.merit_kappa_d,
                     fresh.solver.inner.merit_kappa_d);
    EXPECT_EQ(config.solver.outer.max_outer_iterations,
              fresh.solver.outer.max_outer_iterations);
    EXPECT_DOUBLE_EQ(config.solver.outer.mu_min, fresh.solver.outer.mu_min);
    EXPECT_DOUBLE_EQ(config.solver.outer.anneal_gamma,
                     fresh.solver.outer.anneal_gamma);
    EXPECT_DOUBLE_EQ(config.solver.cost.weight_jerk,
                     fresh.solver.cost.weight_jerk);
    EXPECT_DOUBLE_EQ(config.solver.cost.weight_ref_base,
                     fresh.solver.cost.weight_ref_base);
    EXPECT_DOUBLE_EQ(config.solver.cost.v_max, fresh.solver.cost.v_max);
    EXPECT_DOUBLE_EQ(config.esdf.margin_safe, fresh.esdf.margin_safe);
    EXPECT_EQ(config.esdf.stride, fresh.esdf.stride);
    EXPECT_DOUBLE_EQ(config.post_stage.epsilon_v, fresh.post_stage.epsilon_v);
    EXPECT_DOUBLE_EQ(config.post_stage.omega_max, fresh.post_stage.omega_max);
    EXPECT_DOUBLE_EQ(config.post_stage.eta_max, fresh.post_stage.eta_max);
}

// 测试场景：JSON 在 cost/post_stage 节写入同源幅值键（违反单一来源约定）。
// 预期行为：这些键不被消费，最终取值仍以 reference/inner 权威来源为准
// （JSON 层同样杜绝双源缺口）。
TEST(DdpConfigTest, LoadFromJsonIgnoresShadowedAmplitudeKeys) {
    const auto details = nlohmann::json::parse(R"json({
        "algorithm": "ddp",
        "reference": {"v_max": 2.0, "omega_max": 0.7},
        "solver": {
            "inner": {"steer_accel_max": 1.3},
            "cost": {"v_max": 9.9, "a_max": 9.9, "delta_max": 9.9,
                     "omega_max": 9.9}
        },
        "post_stage": {"omega_max": 9.9, "eta_max": 9.9}
    })json");
    DdpConfig config;
    LoadDdpConfigOverrides(details, &config);
    EXPECT_DOUBLE_EQ(config.solver.cost.v_max, 2.0);
    EXPECT_DOUBLE_EQ(config.solver.cost.a_max, config.reference.a_max);
    EXPECT_DOUBLE_EQ(config.solver.cost.delta_max, config.reference.delta_max);
    EXPECT_DOUBLE_EQ(config.solver.cost.omega_max, 0.7);
    EXPECT_DOUBLE_EQ(config.post_stage.omega_max, 0.7);
    EXPECT_DOUBLE_EQ(config.post_stage.eta_max, 1.3);
}

// 测试场景：空指针调用。
// 预期行为：抛 std::invalid_argument（与 LoadBaseConfigOverrides 同一约定）。
TEST(DdpConfigTest, LoadFromJsonRejectsNullConfig) {
    const auto details =
        nlohmann::json::parse(R"json({"algorithm": "ddp"})json");
    EXPECT_THROW(LoadDdpConfigOverrides(details, nullptr),
                 std::invalid_argument);
}

}  // namespace
}  // namespace apa_post_processor
