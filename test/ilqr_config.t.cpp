#include <gtest/gtest.h>

#include <nlohmann/json.hpp>
#include <type_traits>

#include "core/post_processor.h"

namespace apa_post_processor {
namespace {

// 派生配置对象必须保持可拷贝/可移动（基类 protected 特殊成员不阻碍派生类的
// 隐式特殊成员），供场景层与编排层按值传递与局部复制
static_assert(std::is_copy_constructible_v<iLQRConfig>);
static_assert(std::is_copy_assignable_v<iLQRConfig>);
static_assert(std::is_move_constructible_v<iLQRConfig>);

// 测试场景：默认构造的 iLQRConfig。
// 预期行为：承载 iLQR.md 2.5 节参数表建议默认值（w_j=1.0、w_eta=1.0、
// w_ref,0=10.0、w_theta=5.0、gamma_anneal=0.5、n_s=25、mu_min/mu_max/phi=
// 1e2/1e6/10、容差 0.05 m/1.5°/1e-3、内层/外层迭代上限 50/20、v_max=1.5、
// a_max=1.0、delta_max/omega_max 与车队车辆物理参数真值一致（0.47728 rad /
// 0.4 rad/s——曾被放大为 0.55/0.5 导致输出对真实车辆不可执行的口径缺陷，
// 现以车辆真值为默认并由 clampToVehicleParams 强制只准收紧）、j_max=1.5、
// eta_max=1.0、margin_safe/margin_comf=0.02/0.20、stride=1、epsilon_v=0.02、
// v_dwell=0.05、T_shift=0.4、kappa_pad=1.2），且编排层默认标定
// （merit_mu0=100、cost_change_tol=1e-9）不被 iLQRConfig 构造破坏。
TEST(iLQRConfigTest, DefaultsMatchDesignParameterTable) {
    const iLQRConfig config;
    // 参考构建：重采样间距/固定步长/打靶间隔与初值裁剪盒边界
    EXPECT_DOUBLE_EQ(config.reference_sample_dist, 0.05);
    EXPECT_DOUBLE_EQ(config.reference_dt, 0.1);
    EXPECT_EQ(config.reference_shooting_interval, 25);
    EXPECT_DOUBLE_EQ(config.reference_v_max, 1.5);
    EXPECT_DOUBLE_EQ(config.reference_a_max, 1.0);
    EXPECT_DOUBLE_EQ(config.reference_delta_max, 0.47728);
    EXPECT_DOUBLE_EQ(config.reference_omega_max, 0.4);
    // 内层 MS-iLQR：控制盒与迭代上限；编排层标定值（merit 钉住/容差收紧）
    EXPECT_DOUBLE_EQ(config.inner_jerk_max, 1.5);
    EXPECT_DOUBLE_EQ(config.inner_steer_accel_max, 1.0);
    EXPECT_EQ(config.inner_max_iterations, 50);
    EXPECT_DOUBLE_EQ(config.inner_cost_change_tol, 1e-9);
    EXPECT_DOUBLE_EQ(config.inner_merit_mu0, 100.0);
    // 外层 AL：迭代上限/双指标/缺陷容差/罚权重调度/退火率
    EXPECT_EQ(config.outer_max_outer_iterations, 20);
    EXPECT_DOUBLE_EQ(config.outer_terminal_position_tol, 0.05);
    EXPECT_DOUBLE_EQ(config.outer_terminal_heading_tol_deg, 1.5);
    EXPECT_DOUBLE_EQ(config.outer_defect_tol, 1e-3);
    EXPECT_DOUBLE_EQ(config.outer_mu_min, 1e2);
    EXPECT_DOUBLE_EQ(config.outer_mu_max, 1e6);
    EXPECT_DOUBLE_EQ(config.outer_mu_growth_factor, 10.0);
    EXPECT_DOUBLE_EQ(config.outer_anneal_gamma, 0.5);
    // 代价权重：平滑主项基准 w_j=1.0、w_eta=1.0、w_ref,0=10.0、w_theta=5.0
    EXPECT_DOUBLE_EQ(config.cost_weight_jerk, 1.0);
    EXPECT_DOUBLE_EQ(config.cost_weight_steer_accel, 1.0);
    EXPECT_DOUBLE_EQ(config.cost_weight_ref_base, 10.0);
    EXPECT_DOUBLE_EQ(config.cost_weight_theta, 5.0);
    // 阶段二编排：外层上限 16（四数据集标定值：真实长视窗下 8 轮预算
    // 不足，data3/data7 在 10~11 轮收敛）、门控 mu 初值 10、门控容差 1e-2
    EXPECT_EQ(config.stage_two_max_outer_iterations, 16);
    EXPECT_DOUBLE_EQ(config.gating_mu_initial, 10.0);
    EXPECT_DOUBLE_EQ(config.gating_mu_max, 1e6);
    EXPECT_DOUBLE_EQ(config.gating_tol, 1e-2);
    // ESDF 双 margin 惩罚
    EXPECT_DOUBLE_EQ(config.esdf_margin_safe, 0.02);
    EXPECT_DOUBLE_EQ(config.esdf_margin_comf, 0.20);
    EXPECT_DOUBLE_EQ(config.esdf_weight_safe, 100.0);
    EXPECT_DOUBLE_EQ(config.esdf_weight_comf, 10.0);
    EXPECT_EQ(config.esdf_stride, 1);
    // 后处理与阶段二门控精化
    EXPECT_DOUBLE_EQ(config.post_epsilon_v, 0.02);
    EXPECT_DOUBLE_EQ(config.post_v_dwell, 0.05);
    EXPECT_DOUBLE_EQ(config.post_shift_delay, 0.4);
    EXPECT_DOUBLE_EQ(config.post_kappa_pad, 1.2);
    EXPECT_DOUBLE_EQ(config.post_seam_speed_tol, 0.02);
    EXPECT_DOUBLE_EQ(config.post_amplitude_check_tol, 0.05);
    EXPECT_DOUBLE_EQ(config.post_control_overshoot_tol, 0.3);
}

// 测试场景：幅值边界按车辆物理参数钳制（只准收紧、不准放宽）。
// 预期行为：clampToVehicleParams 把超出车辆物理上限的 δ/ω 边界钳到
// VehicleParams 真值（δ_max ≤ max_steer_angle、ω_max ≤ max_steer_rate、
// a_max ≤ min(max_accel, |max_decel|)），已更紧的取值保持不变；钳制必须
// 发生在 synchronizeAmplitudeBounds 之前（先收紧权威来源、再同步给全部
// 消费方），v_max 无对应车辆物理字段、不钳制。
TEST(iLQRConfigTest, ClampToVehicleParamsOnlyTightens) {
    const VehicleParams vehicle_params{5.0, 1.9, 3.0, 0.47728, 1.1};
    // 超出车辆物理上限的配置被钳回真值
    iLQRConfig inflated;
    inflated.reference_delta_max = 0.55;  // > 0.47728
    inflated.reference_omega_max = 0.5;   // > 0.4
    inflated.reference_a_max = 2.0;       // > min(1.5, 3.0)
    inflated.reference_v_max = 2.0;       // 无车辆字段，不钳制
    inflated.clampToVehicleParams(vehicle_params);
    inflated.synchronizeAmplitudeBounds();
    EXPECT_DOUBLE_EQ(inflated.reference_delta_max, 0.47728);
    EXPECT_DOUBLE_EQ(inflated.reference_omega_max, 0.4);
    EXPECT_DOUBLE_EQ(inflated.reference_a_max, 1.5);
    EXPECT_DOUBLE_EQ(inflated.reference_v_max, 2.0);
    // 同步后全部消费方跟随钳制值
    EXPECT_DOUBLE_EQ(inflated.cost_delta_max, 0.47728);
    EXPECT_DOUBLE_EQ(inflated.cost_omega_max, 0.4);
    EXPECT_DOUBLE_EQ(inflated.post_omega_max, 0.4);
    // 已更紧的取值保持不变（只收紧、不放宽）
    iLQRConfig tight;
    tight.reference_delta_max = 0.4;
    tight.reference_omega_max = 0.3;
    tight.reference_a_max = 0.8;
    tight.clampToVehicleParams(vehicle_params);
    EXPECT_DOUBLE_EQ(tight.reference_delta_max, 0.4);
    EXPECT_DOUBLE_EQ(tight.reference_omega_max, 0.3);
    EXPECT_DOUBLE_EQ(tight.reference_a_max, 0.8);
}

// 测试场景：默认构造与显式调用 synchronizeAmplitudeBounds。
// 预期行为：状态幅值边界（v_max/a_max/delta_max/omega_max）的唯一权威来源是
// reference 子配置、eta_max 的唯一权威来源是 inner.steer_accel_max——默认构造
// 后 cost/post_stage 的同源字段已与权威来源一致；改写权威来源并同步后，全部
// 消费方跟随更新（Config 双源缺口历史教训的前置防御）。
TEST(iLQRConfigTest, AmplitudeBoundsSyncFromSingleSource) {
    iLQRConfig config;
    EXPECT_DOUBLE_EQ(config.cost_v_max, config.reference_v_max);
    EXPECT_DOUBLE_EQ(config.cost_a_max, config.reference_a_max);
    EXPECT_DOUBLE_EQ(config.cost_delta_max, config.reference_delta_max);
    EXPECT_DOUBLE_EQ(config.cost_omega_max, config.reference_omega_max);
    EXPECT_DOUBLE_EQ(config.post_omega_max, config.reference_omega_max);
    EXPECT_DOUBLE_EQ(config.post_eta_max,
                     config.inner_steer_accel_max);
    config.reference_v_max = 2.0;
    config.reference_a_max = 1.2;
    config.reference_delta_max = 0.6;
    config.reference_omega_max = 0.7;
    config.inner_steer_accel_max = 1.3;
    config.synchronizeAmplitudeBounds();
    EXPECT_DOUBLE_EQ(config.cost_v_max, 2.0);
    EXPECT_DOUBLE_EQ(config.cost_a_max, 1.2);
    EXPECT_DOUBLE_EQ(config.cost_delta_max, 0.6);
    EXPECT_DOUBLE_EQ(config.cost_omega_max, 0.7);
    EXPECT_DOUBLE_EQ(config.post_omega_max, 0.7);
    EXPECT_DOUBLE_EQ(config.post_eta_max, 1.3);
}

// 测试场景：JSON 显式给出全部可映射字段（区别于默认值）。
// 预期行为：每个字段都被正确读入；reference 节的幅值键同步进 cost/post_stage
// 同源字段，inner.steer_accel_max 同步进 post_stage.eta_max。
TEST(iLQRConfigTest, LoadFromJsonOverridesAllFields) {
    const auto details = nlohmann::json::parse(R"json({
        "algorithm": "ilqr",
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
                "merit_mu0": 50.0, "merit_mu_max": 1e3,
                "domain_guard_margin": 3.0, "merit_mu_al_ratio": 1e-3,
                "convergence_defect_tol": 2e-3
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
                "weight_ref_base": 20.0, "weight_theta": 8.0
            }
        },
        "esdf": {
            "margin_safe": 0.03, "margin_comf": 0.15,
            "weight_safe": 200.0, "weight_comf": 2.0, "stride": 2
        },
        "post_stage": {
            "epsilon_v": 0.03, "v_dwell": 0.08, "shift_delay": 0.5,
            "kappa_pad": 1.5, "seam_speed_tol": 0.03, "dwell_omega_tol": 0.2,
            "amplitude_check_tol": 0.08, "amplitude_check_rel_tol": 0.03,
            "control_overshoot_tol": 0.4,
            "stage_two_min_tracking_weight": 0.01
        },
        "dual_candidate_select": true
    })json");
    iLQRConfig config;
    LoadiLQRConfigOverrides(details, &config);
    EXPECT_DOUBLE_EQ(config.reference_sample_dist, 0.08);
    EXPECT_DOUBLE_EQ(config.reference_dt, 0.2);
    EXPECT_EQ(config.reference_shooting_interval, 10);
    EXPECT_DOUBLE_EQ(config.reference_v_max, 2.0);
    EXPECT_DOUBLE_EQ(config.reference_a_max, 1.2);
    EXPECT_DOUBLE_EQ(config.reference_delta_max, 0.6);
    EXPECT_DOUBLE_EQ(config.reference_omega_max, 0.7);
    EXPECT_EQ(config.stage_two_max_outer_iterations, 5);
    EXPECT_DOUBLE_EQ(config.gating_mu_initial, 20.0);
    EXPECT_DOUBLE_EQ(config.gating_mu_max, 1e5);
    EXPECT_DOUBLE_EQ(config.gating_tol, 0.05);
    EXPECT_DOUBLE_EQ(config.inner_jerk_max, 2.5);
    EXPECT_DOUBLE_EQ(config.inner_steer_accel_max, 1.3);
    EXPECT_EQ(config.inner_max_iterations, 30);
    EXPECT_DOUBLE_EQ(config.inner_cost_change_tol, 1e-8);
    EXPECT_DOUBLE_EQ(config.inner_gradient_tol, 1e-7);
    EXPECT_DOUBLE_EQ(config.inner_reg_initial, 1e-3);
    EXPECT_DOUBLE_EQ(config.inner_reg_min, 1e-8);
    EXPECT_DOUBLE_EQ(config.inner_reg_max, 1e8);
    EXPECT_DOUBLE_EQ(config.inner_reg_increase, 5.0);
    EXPECT_DOUBLE_EQ(config.inner_reg_decrease, 0.6);
    EXPECT_DOUBLE_EQ(config.inner_armijo_gamma, 0.2);
    EXPECT_DOUBLE_EQ(config.inner_backtrack_beta, 0.4);
    EXPECT_EQ(config.inner_max_backtracks, 40);
    EXPECT_DOUBLE_EQ(config.inner_merit_mu0, 50.0);
    EXPECT_DOUBLE_EQ(config.inner_merit_mu_max, 1e3);
    EXPECT_DOUBLE_EQ(config.inner_domain_guard_margin, 3.0);
    EXPECT_DOUBLE_EQ(config.inner_merit_mu_al_ratio, 1e-3);
    EXPECT_DOUBLE_EQ(config.inner_convergence_defect_tol, 2e-3);
    EXPECT_EQ(config.outer_max_outer_iterations, 15);
    EXPECT_DOUBLE_EQ(config.outer_terminal_position_tol, 0.08);
    EXPECT_DOUBLE_EQ(config.outer_terminal_heading_tol_deg, 2.0);
    EXPECT_DOUBLE_EQ(config.outer_inequality_tol, 0.05);
    EXPECT_DOUBLE_EQ(config.outer_defect_tol, 1e-4);
    EXPECT_DOUBLE_EQ(config.outer_mu_min, 10.0);
    EXPECT_DOUBLE_EQ(config.outer_mu_max, 1e5);
    EXPECT_DOUBLE_EQ(config.outer_first_round_mu, 2.0);
    EXPECT_DOUBLE_EQ(config.outer_amplitude_mu_initial, 3.0);
    EXPECT_DOUBLE_EQ(config.outer_epsilon_mu, 1e-3);
    EXPECT_DOUBLE_EQ(config.outer_mu_gate_kappa, 0.8);
    EXPECT_DOUBLE_EQ(config.outer_mu_growth_factor, 5.0);
    EXPECT_DOUBLE_EQ(config.outer_anneal_gamma, 0.6);
    EXPECT_DOUBLE_EQ(config.cost_weight_jerk, 2.0);
    EXPECT_DOUBLE_EQ(config.cost_weight_steer_accel, 3.0);
    EXPECT_DOUBLE_EQ(config.cost_weight_ref_base, 20.0);
    EXPECT_DOUBLE_EQ(config.cost_weight_theta, 8.0);
    EXPECT_DOUBLE_EQ(config.esdf_margin_safe, 0.03);
    EXPECT_DOUBLE_EQ(config.esdf_margin_comf, 0.15);
    EXPECT_DOUBLE_EQ(config.esdf_weight_safe, 200.0);
    EXPECT_DOUBLE_EQ(config.esdf_weight_comf, 2.0);
    EXPECT_EQ(config.esdf_stride, 2);
    EXPECT_DOUBLE_EQ(config.post_epsilon_v, 0.03);
    EXPECT_DOUBLE_EQ(config.post_v_dwell, 0.08);
    EXPECT_DOUBLE_EQ(config.post_shift_delay, 0.5);
    EXPECT_DOUBLE_EQ(config.post_kappa_pad, 1.5);
    EXPECT_DOUBLE_EQ(config.post_seam_speed_tol, 0.03);
    EXPECT_DOUBLE_EQ(config.post_dwell_omega_tol, 0.2);
    EXPECT_DOUBLE_EQ(config.post_amplitude_check_tol, 0.08);
    EXPECT_DOUBLE_EQ(config.post_amplitude_check_rel_tol, 0.03);
    EXPECT_DOUBLE_EQ(config.post_control_overshoot_tol, 0.4);
    EXPECT_DOUBLE_EQ(config.post_stage_two_min_tracking_weight, 0.01);
    EXPECT_TRUE(config.dual_candidate_select);
    // 幅值边界经 reference/inner 单一来源同步进全部消费方
    EXPECT_DOUBLE_EQ(config.cost_v_max, 2.0);
    EXPECT_DOUBLE_EQ(config.cost_a_max, 1.2);
    EXPECT_DOUBLE_EQ(config.cost_delta_max, 0.6);
    EXPECT_DOUBLE_EQ(config.cost_omega_max, 0.7);
    EXPECT_DOUBLE_EQ(config.post_omega_max, 0.7);
    EXPECT_DOUBLE_EQ(config.post_eta_max, 1.3);
}

// 测试场景：JSON 仅含 algorithm 路由字段（无任何 iLQR 专有节）。
// 预期行为：全部字段回落到 2.5 节建议默认值（与默认构造逐字段一致）。
TEST(iLQRConfigTest, LoadFromJsonKeepsDefaultsWhenAbsent) {
    const auto details =
        nlohmann::json::parse(R"json({"algorithm": "ilqr"})json");
    iLQRConfig config;
    LoadiLQRConfigOverrides(details, &config);
    const iLQRConfig fresh;
    EXPECT_DOUBLE_EQ(config.reference_sample_dist, fresh.reference_sample_dist);
    EXPECT_DOUBLE_EQ(config.reference_dt, fresh.reference_dt);
    EXPECT_EQ(config.reference_shooting_interval,
              fresh.reference_shooting_interval);
    EXPECT_DOUBLE_EQ(config.reference_v_max, fresh.reference_v_max);
    EXPECT_DOUBLE_EQ(config.inner_jerk_max, fresh.inner_jerk_max);
    EXPECT_EQ(config.inner_max_iterations,
              fresh.inner_max_iterations);
    EXPECT_EQ(config.outer_max_outer_iterations,
              fresh.outer_max_outer_iterations);
    EXPECT_DOUBLE_EQ(config.outer_mu_min, fresh.outer_mu_min);
    EXPECT_DOUBLE_EQ(config.outer_anneal_gamma,
                     fresh.outer_anneal_gamma);
    EXPECT_DOUBLE_EQ(config.cost_weight_jerk,
                     fresh.cost_weight_jerk);
    EXPECT_DOUBLE_EQ(config.cost_weight_ref_base,
                     fresh.cost_weight_ref_base);
    EXPECT_DOUBLE_EQ(config.cost_v_max, fresh.cost_v_max);
    EXPECT_DOUBLE_EQ(config.esdf_margin_safe, fresh.esdf_margin_safe);
    EXPECT_EQ(config.esdf_stride, fresh.esdf_stride);
    EXPECT_DOUBLE_EQ(config.post_epsilon_v, fresh.post_epsilon_v);
    EXPECT_DOUBLE_EQ(config.post_omega_max, fresh.post_omega_max);
    EXPECT_DOUBLE_EQ(config.post_eta_max, fresh.post_eta_max);
}

// 测试场景：JSON 在 cost/post_stage 节写入同源幅值键（违反单一来源约定）。
// 预期行为：这些键不被消费，最终取值仍以 reference/inner 权威来源为准
// （JSON 层同样杜绝双源缺口）。
TEST(iLQRConfigTest, LoadFromJsonIgnoresShadowedAmplitudeKeys) {
    const auto details = nlohmann::json::parse(R"json({
        "algorithm": "ilqr",
        "reference": {"v_max": 2.0, "omega_max": 0.7},
        "solver": {
            "inner": {"steer_accel_max": 1.3},
            "cost": {"v_max": 9.9, "a_max": 9.9, "delta_max": 9.9,
                     "omega_max": 9.9}
        },
        "post_stage": {"omega_max": 9.9, "eta_max": 9.9}
    })json");
    iLQRConfig config;
    LoadiLQRConfigOverrides(details, &config);
    EXPECT_DOUBLE_EQ(config.cost_v_max, 2.0);
    EXPECT_DOUBLE_EQ(config.cost_a_max, config.reference_a_max);
    EXPECT_DOUBLE_EQ(config.cost_delta_max, config.reference_delta_max);
    EXPECT_DOUBLE_EQ(config.cost_omega_max, 0.7);
    EXPECT_DOUBLE_EQ(config.post_omega_max, 0.7);
    EXPECT_DOUBLE_EQ(config.post_eta_max, 1.3);
}

// 测试场景：空指针调用。
// 预期行为：抛 std::invalid_argument（与 LoadBaseConfigOverrides 同一约定）。
TEST(iLQRConfigTest, LoadFromJsonRejectsNullConfig) {
    const auto details =
        nlohmann::json::parse(R"json({"algorithm": "ilqr"})json");
    EXPECT_THROW(LoadiLQRConfigOverrides(details, nullptr),
                 std::invalid_argument);
}

// 测试场景：JSON 含未知字段（各节与顶层混入未登记的键）。
// 预期行为：未知键被静默忽略（加载器只消费登记键），已知字段正常读入、
// 其余保持默认——钉住"忽略而非报错"的容错语义，防止未来误改为严格模式
// 后生产配置因新增键而崩溃
TEST(iLQRConfigTest, LoadFromJsonIgnoresUnknownFields) {
    const auto details = nlohmann::json::parse(R"json({
        "algorithm": "ilqr",
        "unknown_top_level": 123,
        "reference": {"v_max": 2.0, "unknown_ref": "x"},
        "solver": {"unknown_solver": true},
        "esdf": {"weight_safe": 150.0, "unknown_esdf": 1.5},
        "post_stage": {"unknown_post": null}
    })json");
    iLQRConfig config;
    LoadiLQRConfigOverrides(details, &config);
    EXPECT_DOUBLE_EQ(config.reference_v_max, 2.0);
    EXPECT_DOUBLE_EQ(config.esdf_weight_safe, 150.0);
    const iLQRConfig fresh;
    EXPECT_DOUBLE_EQ(config.reference_dt, fresh.reference_dt);
    EXPECT_DOUBLE_EQ(config.cost_weight_jerk,
                     fresh.cost_weight_jerk);
    EXPECT_DOUBLE_EQ(config.post_epsilon_v, fresh.post_epsilon_v);
}

// 测试场景：JSON 已知字段给错类型（如数值字段写成字符串）。
// 预期行为：nlohmann 类型转换失败抛 json::exception（类型错误是配置
// 作者笔误，必须显式失败而非静默纠错）；加载器自身的容错语义不受影响
TEST(iLQRConfigTest, LoadFromJsonThrowsOnWrongFieldType) {
    const auto details = nlohmann::json::parse(R"json({
        "algorithm": "ilqr",
        "reference": {"v_max": "not_a_number"}
    })json");
    iLQRConfig config;
    EXPECT_THROW(LoadiLQRConfigOverrides(details, &config),
                 nlohmann::json::exception);
}

}  // namespace
}  // namespace apa_post_processor
