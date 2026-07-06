"""共享符号与维度定义，所有 CasADi 生成脚本共用。"""

import casadi as ca

# 自行车模型（曲率版）状态维度
NX_KAPPA = 5
NU_KAPPA = 2

# 自行车模型（前轮转角版）状态维度
NX_DELTA = 5
NU_DELTA = 2

# 通用状态维度（凸走廊等仅依赖 [x, y, theta] 的模块使用）
NX = 5

# 车辆参数（典型家用轿车，单位 m）
WHEELBASE = 2.8  # 轴距
L_F = 3.7  # 后轴到前保险杠距离（车身前缘）
L_R = 1.0  # 后轴到后保险杠距离（车身后缘）
W = 1.8  # 车宽

# 通用参数 p 维度
P_DIM = 150


def make_kappa_dynamics():
    """曲率控制版自行车模型：状态 [x, y, theta, v, kappa]，控制 [a, kappa_dot]。"""
    x = ca.SX.sym("x", NX_KAPPA)
    u = ca.SX.sym("u", NU_KAPPA)
    theta = x[2]
    v = x[3]
    kappa = x[4]
    a = u[0]
    kappa_dot = u[1]
    f = ca.vertcat(
        v * ca.cos(theta),
        v * ca.sin(theta),
        v * kappa,
        a,
        kappa_dot,
    )
    return x, u, f


def make_delta_dynamics():
    """前轮转角控制版自行车模型：状态 [x, y, theta, v, delta]，控制 [a, delta_dot]。"""
    x = ca.SX.sym("x", NX_DELTA)
    u = ca.SX.sym("u", NU_DELTA)
    theta = x[2]
    v = x[3]
    delta = x[4]
    a = u[0]
    delta_dot = u[1]
    f = ca.vertcat(
        v * ca.cos(theta),
        v * ca.sin(theta),
        v * ca.tan(delta) / WHEELBASE,
        a,
        delta_dot,
    )
    return x, u, f
