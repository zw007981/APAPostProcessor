"""生成凸走廊约束 C 代码：4 个车辆角点 × 10 个半空间。

输出：
    corridor.c / corridor.h / vehicle_geometry.h

函数签名：
    corridor(arg=[x(5), p(150)], res=[g(40), Cx(40x5)])

其中 g 的每一行对应一个车辆角点在一个半空间下的约束值 A_i^T p_world - b_i；
Cx 是 g 对 x 的完整 Jacobian，含 theta 的非线性项（旋转矩阵 R 的导数）。
为保证 ABI 清晰，Cx 通过 ca.densify() 输出为稠密 40×5 矩阵。
"""

import os

import casadi as ca
from common import L_F, L_R, NX, P_DIM, W

# 车辆角点在车身坐标系下的局部坐标（原点为后轴中心，x 向前，y 向左）
CORNERS_LOCAL = [
    ca.DM([L_F, W / 2]),  # 前左
    ca.DM([L_F, -W / 2]),  # 前右
    ca.DM([-L_R, W / 2]),  # 后左
    ca.DM([-L_R, -W / 2]),  # 后右
]

N_CORNERS = 4
N_HS = 10
G_DIM = N_CORNERS * N_HS  # 40
CX_NNZ = G_DIM * NX  # 200（稠密输出）


def _assert_signature(func: ca.Function):
    """回归检查：验证生成函数的输入输出维度与头文件注释一致。"""
    assert func.n_in() == 2, f"{func.name()}: expected 2 inputs, got {func.n_in()}"
    assert (
        func.size1_in(0) == NX
    ), f"{func.name()}: x dimension expected {NX}, got {func.size1_in(0)}"
    assert (
        func.size1_in(1) == P_DIM
    ), f"{func.name()}: p dimension expected {P_DIM}, got {func.size1_in(1)}"
    assert func.n_out() == 2, f"{func.name()}: expected 2 outputs, got {func.n_out()}"
    assert func.size1_out(0) == G_DIM, f"{func.name()}: g dimension expected {G_DIM}"
    assert (
        func.size1_out(1) == G_DIM and func.size2_out(1) == NX
    ), f"{func.name()}: Cx dimension expected {G_DIM}x{NX}"


def generate(output_dir: str):
    os.makedirs(output_dir, exist_ok=True)

    x = ca.SX.sym("x", NX)
    p = ca.SX.sym("p", P_DIM)

    # 从 p 解包 10 个 2D 半空间参数：A_flat(20) + b(10)
    A_flat = p[15 : 15 + N_HS * 2]
    b = p[15 + N_HS * 2 : 15 + N_HS * 3]

    theta = x[2]
    R = ca.vertcat(
        ca.horzcat(ca.cos(theta), -ca.sin(theta)),
        ca.horzcat(ca.sin(theta), ca.cos(theta)),
    )

    g = ca.SX.zeros(G_DIM)
    for corner_idx in range(N_CORNERS):
        p_local = CORNERS_LOCAL[corner_idx]
        p_world = ca.vertcat(x[0], x[1]) + R @ p_local
        for hs_idx in range(N_HS):
            A_i = ca.reshape(A_flat[hs_idx * 2 : (hs_idx + 1) * 2], 2, 1)
            row = corner_idx * N_HS + hs_idx
            g[row] = ca.dot(A_i, p_world) - b[hs_idx]

    # 关键：生成完整 Jacobian Cx = dg/dx（含 theta 非线性项），
    # 并显式稠密化，使生成代码按列主序输出完整的 40x5 矩阵。
    Cx = ca.densify(ca.jacobian(g, x))

    f = ca.Function("corridor", [x, p], [g, Cx])
    _assert_signature(f)

    tmp_c = "corridor.c"
    f.generate(tmp_c)
    os.replace(tmp_c, os.path.join(output_dir, tmp_c))

    with open(os.path.join(output_dir, "corridor.h"), "w") as fh:
        fh.write(f"""#ifndef CORRIDOR_H
#define CORRIDOR_H
#ifdef __cplusplus
extern "C" {{
#endif

// ABI 常量：由 autogen/generate_corridor.py 生成，C++ 与测试统一引用
#define CORRIDOR_NX    {NX}
#define CORRIDOR_P_DIM {P_DIM}
#define CORRIDOR_G_DIM {G_DIM}
#define CORRIDOR_CX_NNZ {CX_NNZ}

int corridor(const double** arg, double** res, long long* iw, double* w, int mem);
int corridor_work(long long* sz_arg, long long* sz_res, long long* sz_iw, long long* sz_w);
const long long* corridor_sparsity_out(long long i);
// arg: [x(CORRIDOR_NX), p(CORRIDOR_P_DIM)]
// res: [g(CORRIDOR_G_DIM), Cx(CORRIDOR_G_DIM x CORRIDOR_NX) 稠密列主序，共 CORRIDOR_CX_NNZ 个 double]

#ifdef __cplusplus
}}
#endif
#endif
""")

    # 同时生成车辆几何常量头，保证 C++ 与 CasADi 使用同一事实源
    with open(os.path.join(output_dir, "vehicle_geometry.h"), "w") as fh:
        fh.write(f"""#ifndef VEHICLE_GEOMETRY_H
#define VEHICLE_GEOMETRY_H

namespace stc_SQP {{
namespace vehicle_geometry {{
// 后轴到前保险杠距离（车身前缘）
constexpr double kLf = {L_F};
// 后轴到后保险杠距离（车身后缘）
constexpr double kLr = {L_R};
// 车宽
constexpr double kWidth = {W};
}} // namespace vehicle_geometry
}} // namespace stc_SQP

#endif
""")


if __name__ == "__main__":
    project_root = os.path.dirname(os.path.abspath(__file__))
    generate(os.path.join(project_root, "..", "src", "generated"))
