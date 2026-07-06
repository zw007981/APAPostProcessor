"""生成自行车模型动力学 C 代码（含 f、A、B），供 C++ CasADiWrapper 调用。"""

import os
import casadi as ca
from common import (make_kappa_dynamics, make_delta_dynamics,
                    NX_KAPPA, NU_KAPPA, NX_DELTA, NU_DELTA)


def _assert_signature(func: ca.Function, nx: int, nu: int, is_jac: bool):
    """回归检查：验证生成函数的输入输出维度与头文件注释一致。"""
    assert func.n_in() == 2, f"{func.name()}: expected 2 inputs, got {func.n_in()}"
    assert func.size1_in(0) == nx, f"{func.name()}: x dimension expected {nx}, got {func.size1_in(0)}"
    assert func.size1_in(1) == nu, f"{func.name()}: u dimension expected {nu}, got {func.size1_in(1)}"
    if is_jac:
        assert func.n_out() == 3, f"{func.name()}: expected 3 outputs, got {func.n_out()}"
        assert func.size1_out(0) == nx, f"{func.name()}: f dimension expected {nx}"
        assert func.size1_out(1) == nx and func.size2_out(1) == nx, \
            f"{func.name()}: A dimension expected {nx}x{nx}"
        assert func.size1_out(2) == nx and func.size2_out(2) == nu, \
            f"{func.name()}: B dimension expected {nx}x{nu}"
    else:
        assert func.n_out() == 1, f"{func.name()}: expected 1 output, got {func.n_out()}"
        assert func.size1_out(0) == nx, f"{func.name()}: f dimension expected {nx}"


def _generate_one(output_dir: str, name: str, x, u, f, nx: int, nu: int):
    """为单一模型生成 dynamics.c / dynamics_jac.c 及对应头文件。"""
    A = ca.jacobian(f, x)
    B = ca.jacobian(f, u)

    func_f = ca.Function(name, [x, u], [f])
    func_jac = ca.Function(f'{name}_jac', [x, u], [f, A, B])
    _assert_signature(func_f, nx, nu, is_jac=False)
    _assert_signature(func_jac, nx, nu, is_jac=True)

    # CasADi 以文件名作为生成符号前缀，路径中含 ../ 会导致命名检查失败，
    # 因此先在当前目录生成，再移动到目标目录。
    tmp_f = f'{name}.c'
    tmp_jac = f'{name}_jac.c'
    func_f.generate(tmp_f)
    func_jac.generate(tmp_jac)
    os.replace(tmp_f, os.path.join(output_dir, tmp_f))
    os.replace(tmp_jac, os.path.join(output_dir, tmp_jac))

    with open(os.path.join(output_dir, f'{name}.h'), 'w') as fh:
        fh.write(f"""#ifndef {name.upper()}_H
#define {name.upper()}_H
#ifdef __cplusplus
extern "C" {{
#endif
int {name}(const double** arg, double** res, long long* iw, double* w, int mem);
int {name}_work(long long* sz_arg, long long* sz_res, long long* sz_iw, long long* sz_w);
// arg: [x({nx}), u({nu})]
// res: [f({nx})]
#ifdef __cplusplus
}}
#endif
#endif
""")

    with open(os.path.join(output_dir, f'{name}_jac.h'), 'w') as fh:
        fh.write(f"""#ifndef {name.upper()}_JAC_H
#define {name.upper()}_JAC_H
#ifdef __cplusplus
extern "C" {{
#endif
int {name}_jac(const double** arg, double** res, long long* iw, double* w, int mem);
int {name}_jac_work(long long* sz_arg, long long* sz_res, long long* sz_iw, long long* sz_w);
const long long* {name}_jac_sparsity_out(long long i);
// arg: [x({nx}), u({nu})]
// res: [f({nx}), A({nx}x{nx}), B({nx}x{nu})]
#ifdef __cplusplus
}}
#endif
#endif
""")


def generate(output_dir: str):
    os.makedirs(output_dir, exist_ok=True)

    x_k, u_k, f_k = make_kappa_dynamics()
    _generate_one(output_dir, 'dynamics_kappa', x_k, u_k, f_k, NX_KAPPA, NU_KAPPA)

    x_d, u_d, f_d = make_delta_dynamics()
    _generate_one(output_dir, 'dynamics_delta', x_d, u_d, f_d, NX_DELTA, NU_DELTA)


if __name__ == '__main__':
    project_root = os.path.dirname(os.path.abspath(__file__))
    generate(os.path.join(project_root, '..', 'src', 'generated'))
