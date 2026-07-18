# C++ SQP 轨迹优化框架设计文档

**设计哲学**：SQP 引擎必须是**纯粹的数值优化器**，对物理世界"一无所知"。它只认识固定维度的矩阵 $A, B, C, d$ 和梯度 $q, r$。所有业务逻辑（障碍物筛选、车辆几何、地图语义）通过**通用参数向量 `p`** 注入，由外部业务层负责。

> **`Constraint`/`CostTerm` 去虚拟化评估说明**：本文档第 3.4/3.5/3.6 节及以下多处以虚函数多态 / `clone()` 描述 `Constraint`/`CostTerm` 接口，这是当前代码的现状。曾评估将其迁移到 closed-set `std::variant` 的去虚拟化方案，但 `third_party/StcSQP/bench/bench_performance_profiling.cpp` 的 Release 端到端 benchmark 未观测到可解释的收益；最终结论为**不合并**，已将相关实验代码从工作树移除，保留本文档描述的虚函数基线。评估详情见主仓库 `docs/interfaces.md`/`docs/known-limitations.md` 对应记录。

---

## 1. 架构总览

```
┌─────────────────────────────────────────────┐
│  Layer 4: Application (业务层)                │
│  examples/parking/                            │
│  - ProblemUpdater: Top-K 筛选、参数填充、完备性断言 │
│  - ParkingOptimizer: 场景组装、RTI 降级判断      │
└──────────────────┬────────────────────────────┘
                   │
┌──────────────────▼────────────────────────────┐
│  Layer 3: SQP Algorithm (数值优化引擎)        │
│  sqp/ + strategies/                           │
│  - 外循环: 线性化(OpenMP) → 线搜索 → 收敛判断    │
│  - 内循环: HPIPM 求解 (Partial Condensing + 软约束) │
│  - 策略: AutoAdaptive / ManualHierarchical      │
└──────────────────┬────────────────────────────┘
                   │
┌──────────────────▼────────────────────────────┐
│  Layer 2: NLP / OCP (问题描述与离散化)         │
│  ocp/ + nlp/                                  │
│  - MultiStageOCP: 多段问题（支持多段拼接与换挡检测；段间切换约束尚未实现）│
│  - StageParameters: 通用参数 p (固定维度)        │
│  - Discretizer:  打靶法离散化为 NLP              │
└──────────────────┬────────────────────────────┘
                   │
┌──────────────────▼────────────────────────────┐
│  Layer 1: QP Solver (凸优化内核)                │
│  qp/                                          │
│  - HPIPM: O(N) Riccati + Partial Condensing    │
│  - OSQP:  备用稀疏 QP                           │
│  - Dense: Eigen LDLT 调试器                     │
└──────────────────┬────────────────────────────┘
                   │
┌──────────────────▼────────────────────────────┐
│  Layer 0: Core & Math & Models (基础设施)      │
│  core/ + math/ + models/ + costs/ + constraints/ │
│  - CasADi 包装器 (预分配 iw/w, 支持线程克隆)      │
│  - SO2/SE2 流形运算 (含 retract)                 │
│  - 自行车模型 (手写 + 生成)                      │
└─────────────────────────────────────────────┘
```

---

## 2. 目录结构

```
StcSQP/
├── CMakeLists.txt
├── cmake/
│   ├── FindHPIPM.cmake
│   └── CasADiCodegen.cmake          # 支持 COMMON_DEPS 追踪
├── autogen/                         # CasADi 离线代码生成（Python）
│   ├── common.py                    # 共享符号：车辆参数、状态命名、p 维度分配
│   ├── generate_dynamics.py         # 生成动力学 f, A, B
│   ├── generate_costs.py            # 生成成本梯度、Hessian
│   ├── generate_corridor.py         # 生成凸走廊约束 g, Cx（含 4 角点 × K 半空间）
│   └── generate_switching.py        # 生成段间切换约束
├── src/                             # 核心 C++ 源码
│   ├── main.cpp
│   ├── core/
│   │   ├── types.h                  # Vector, Matrix, aligned_allocator
│   │   ├── config.hpp               # 编译开关
│   │   ├── options.hpp              # SQPSolverOptions, StageOptions
│   │   ├── timer.h + timer.cpp
│   │   └── logger.h
│   ├── math/                        # math 保持 header-only
│   │   ├── math_util.hpp            # 通用数学工具（角度 wrap 等）
│   │   ├── so2.hpp                  # SO2 流形 retract
│   │   └── se2.hpp                  # 位姿变换
│   ├── models/
│   │   ├── dynamical_system.h       # 纯虚接口（含 retract）
│   │   ├── bicycle_model_delta.h + bicycle_model_delta.cpp   # 前轮转角控制版，本项目默认运动学模型
│   │   ├── bicycle_model_kappa.h + bicycle_model_kappa.cpp   # 曲率控制版，备选
│   │   └── casadi_wrapper.h + casadi_wrapper.cpp
│   ├── generated/                   # CasADi 输出（.gitignore），保持生成格式不变
│   │   ├── dynamics.c / dynamics.h
│   │   ├── dynamics_jac.c / dynamics_jac.h
│   │   ├── corridor.c / corridor.h
│   │   └── switching.c / switching.h
│   ├── util/
│       ├── constants.h              # 统一常量管理（PI、EPSILON 等）
│       ├── trajectory.h + trajectory.cpp
│       ├── reference_line.h + reference_line.cpp
│       └── geometry.h + geometry.cpp  # OBB/GJK仅用于离线验证/凸走廊构造，不进入SQP
│   ├── costs/
│   │   ├── cost_term.hpp                # 纯虚接口
│   │   ├── quadratic_tracking.h + quadratic_tracking.cpp
│   │   ├── control_regularization.h + control_regularization.cpp
│   │   ├── control_rate_regularization.h + control_rate_regularization.cpp
│   │   └── composite_cost.h + composite_cost.cpp
│   ├── constraints/
│   │   ├── constraint.hpp               # 纯虚接口（含 supports_slack）
│   │   ├── box_constraint.h + box_constraint.cpp
│   │   ├── dynamics_constraint.h + dynamics_constraint.cpp
│   │   ├── convex_corridor_constraint.h + convex_corridor_constraint.cpp  # 包装 CasADi 生成的 corridor.c
│   │   └── slack_variable.h + slack_variable.cpp
│   ├── integrators/
│   │   ├── integrator.hpp               # 纯虚接口
│   │   ├── forward_euler.h + forward_euler.cpp
│   │   └── runge_kutta4.h + runge_kutta4.cpp
│   ├── ocp/
│   │   ├── stage.h + stage.cpp          # 单阶段定义（含 StageParameters）
│   │   ├── ocp.h + ocp.cpp
│   │   ├── multi_stage_ocp.h + multi_stage_ocp.cpp  # 多段 OCP（支持拼接与换挡检测；切换约束尚未实现）
│   │   └── discretizer.h + discretizer.cpp
│   ├── nlp/
│   │   ├── nlp.h + nlp.cpp
│   │   ├── nlp_solution.hpp             # 解结构体，较短
│   │   ├── nlp_dims.hpp                 # 维度结构体，较短
│   │   ├── residual.h + residual.cpp
│   │   └── constraint_violation.h + constraint_violation.cpp
│   ├── qp/
│   │   ├── qp_data.h + qp_data.cpp      # 对齐内存池 + Eigen::Map + 软约束
│   │   ├── qp_solution.hpp              # 解结构体，较短
│   │   ├── qp_solver.hpp                # 纯虚接口，返回 QPSolverStatus
│   │   ├── hpipm_solver.h + hpipm_solver.cpp  # HPIPM 包装器（Partial Condensing + 软约束）
│   │   ├── osqp_solver.h + osqp_solver.cpp    # 备用
│   │   └── dense_qp_solver.h + dense_qp_solver.cpp  # Eigen LDLT 调试器
│   ├── sqp/
│   │   ├── sqp_algorithm.h + sqp_algorithm.cpp  # 主求解器（含 RTI 降级逻辑）
│   │   ├── line_search.hpp              # 纯虚接口
│   │   ├── armijo_line_search.h + armijo_line_search.cpp
│   │   ├── hessian_approximation.hpp    # 纯虚接口
│   │   ├── gauss_newton.h + gauss_newton.cpp
│   │   ├── regularization.h + regularization.cpp
│   │   ├── feasibility_restoration.h + feasibility_restoration.cpp
│   │   └── merit_function.h + merit_function.cpp
│   └── strategies/                      # 长 N 分层策略
│   │   ├── solver_strategy.hpp          # 纯虚接口
│   │   ├── auto_adaptive_strategy.h + auto_adaptive_strategy.cpp  # 自动选择单阶段 / Coarse-to-Fine
│   │   └── manual_hierarchical_strategy.h + manual_hierarchical_strategy.cpp
├── examples/
│   └── parking/
│       ├── problem_updater.h + problem_updater.cpp  # 业务层：Top-K 筛选 + 参数填充 + 完备性断言
│       ├── parking_optimizer.h + parking_optimizer.cpp
│       └── main.cpp
└── test/
    ├── test_math.cpp
    ├── test_bicycle_model.cpp
    ├── test_integrator.cpp
    ├── test_corridor_constraint.cpp
    ├── test_qp_solvers.cpp
    ├── test_sqp_convergence.cpp
    ├── test_memory_pool.cpp
    └── test_parking.cpp
```

---

## 3. 核心模块接口定义

### 3.1 状态与控制类型（对齐分配器）

```cpp
// src/core/types.h
#pragma once
#include <Eigen/Dense>
#include <Eigen/StdVector>

namespace stc_SQP {

using Vector = Eigen::VectorXd;
using Matrix = Eigen::MatrixXd;

// 【关键】使用 Eigen 对齐分配器，保证 SIMD（AVX2/AVX512）安全
template<typename T>
using AlignedVector = std::vector<T, Eigen::aligned_allocator<T>>;

using Index = int;

} // namespace stc_SQP
```

### 3.2 动力学模型（含流形 Retraction）

```cpp
// src/models/dynamical_system.h
#pragma once
#include "core/types.h"

namespace stc_SQP {

class DynamicalSystem {
public:
    virtual ~DynamicalSystem() = default;

    virtual int nx() const = 0;
    virtual int nu() const = 0;

    // 连续动力学
    virtual void evaluate(const Vector& x, const Vector& u, Vector& xdot) const = 0;

    // 离散化 + 灵敏度
    virtual void discretizeAndLinearize(const Vector& x, const Vector& u,
                                        double dt,
                                        Vector& x_next,
                                        Matrix& A, Matrix& B) const = 0;

    // 【强制】流形更新：x_new = x ⊕ (alpha * delta)
    // 默认线性相加。BicycleModel 必须重写，对 theta 做 SO2::retract
    virtual void retract(const Vector& x, double alpha, const Vector& delta,
                         Vector& x_new) const {
        x_new = x + alpha * delta;
    }
};

} // namespace stc_SQP
```

### 3.3 通用参数接口（Static Dimensions, Dynamic Parameters）

```cpp
// ocp/stage.hpp
#pragma once
#include "core/types.h"

namespace stc_SQP {

// 极其纯粹：SQP 引擎只认这个定长向量
// 语义由外部 CasADi 脚本定义，C++ 框架不解释
struct StageParameters {
    Vector p;  // 固定长度（如 150 维），构造时预分配
};

struct StageSegment {
    std::shared_ptr<DynamicalSystem> dynamics;
    std::shared_ptr<CostTerm> cost;
    std::vector<std::shared_ptr<Constraint>> constraints;
    
    int N;           // 该段离散步数
    std::vector<double> dt_array;  // 每步独立步长（预留，默认均匀）
    double v_sign;   // +1.0 前进, -1.0 后退
    
    // 每步通用参数（运行时更新）
    std::vector<StageParameters> stage_params;
    
    Vector x_min, x_max;
    Vector u_min, u_max;
};

} // namespace stc_SQP
```

### 3.4 约束接口（软约束支持）

```cpp
// constraints/constraint.hpp
#pragma once
#include "core/types.h"

namespace stc_SQP {

class Constraint {
public:
    virtual ~Constraint() = default;

    // 约束维度
    virtual int ng() const = 0;

    // 评估：g(x, u, p) <= 0
    // p 由调用方显式传入；约束对象内部不持有运行时可变参数，避免隐藏状态与线程竞争
    virtual void evaluate(const Vector& x, const Vector& u, const Vector& p,
        Vector& g) const = 0;

    // Jacobian：Cx = dg/dx, Cu = dg/du（p 作为已知参数，不进入求导）
    // 对凸走廊约束，Cx 含 theta 非线性，必须通过 CasADi 生成函数计算
    virtual void jacobian(const Vector& x, const Vector& u, const Vector& p,
        Matrix& Cx, Matrix& Cu) const = 0;

    // 一次调用同时得到约束值与 Jacobian；默认实现为 evaluate + jacobian
    virtual void evaluateAndJacobian(const Vector& x, const Vector& u, const Vector& p,
        Vector& g, Matrix& Cx, Matrix& Cu) const
    {
        evaluate(x, u, p, g);
        jacobian(x, u, p, Cx, Cu);
    }

    // 是否支持 HPIPM 软约束
    virtual bool supportsSlack() const { return false; }

    // 创建独立副本，供多线程并行线性化使用。
    // 实现类必须深拷贝所有内部状态；对含 CasADiFunction 的约束，clone 必须重新分配独立工作区。
    virtual std::shared_ptr<Constraint> clone() const = 0;
};

} // namespace stc_SQP
```

### 3.5 代价项接口（组合求值 + 线程克隆）

```cpp
// costs/cost_term.hpp
#pragma once
#include <memory>
#include "core/types.h"

namespace stc_SQP {

class CostTerm {
public:
    virtual ~CostTerm() = default;

    // 标量代价 cost = L(x, u)
    virtual void evaluate(const Vector& x, const Vector& u, double& cost) const = 0;

    // 梯度：q = dL/dx, r = dL/du
    virtual void gradient(const Vector& x, const Vector& u,
        Vector& q, Vector& r) const = 0;

    // Hessian：Q = d²L/dx², R = d²L/du², S = d²L/(du dx)
    virtual void hessian(const Vector& x, const Vector& u,
        Matrix& Q, Matrix& R, Matrix& S) const = 0;

    // 一次调用同时得到 cost/梯度/Hessian；默认实现转发到上述三个方法。
    // 对内部持有昂贵计算（如 ESDF 地图查询）的代价，覆写此方法可消除重复计算。
    virtual void evaluateGradientAndHessian(const Vector& x, const Vector& u,
        double& cost, Vector& q, Vector& r,
        Matrix& Q, Matrix& R, Matrix& S) const
    {
        evaluate(x, u, cost);
        gradient(x, u, q, r);
        hessian(x, u, Q, R, S);
    }

    // 创建独立副本，供多线程并行 assembleQP/assembleCost 使用。
    // 含非拥有引用（如 EsdfMapInterface）的代价只需拷贝引用本身，
    // 不需要深拷贝地图数据（与 Constraint::clone() 约定一致）。
    virtual std::shared_ptr<CostTerm> clone() const = 0;
};

} // namespace stc_SQP
```

### 3.6 QP 数据（对齐内存池 + 偏移量填充 + 软约束）

```cpp
// qp/qp_data.hpp
#pragma once
#include <vector>
#include <memory>
#include <Eigen/Core>
#include "core/types.h"

namespace stc_SQP {

// 软约束配置（HPIPM 原生支持）
// 当前实现：一般约束语义为 C x <= d，Dense LDLT 参考求解器仅实现上界 slack（Zu/zu）。
// 下界 slack（Zl/zl）接口已预留，待业务需要时扩展。
struct SoftConstraintConfig {
    int ns;              // 软约束维度
    Vector Zl, Zu;       // L2 权重
    Vector zl, zu;       // L1 权重（Exact Penalty）
    std::vector<int> idxs; // 允许软化的约束索引
};

struct QPData {
    int N = 0;
    int nx = 0;
    int nu = 0;
    int ng_max = 0;

    // 【关键】对齐内存池，禁止拷贝/移动
    AlignedVector<double> memory_pool_;
    
    // Eigen::Map 视图（不拥有内存）
    std::vector<Eigen::Map<Matrix>> A;
    std::vector<Eigen::Map<Matrix>> B;
    std::vector<Eigen::Map<Matrix>> Q;
    std::vector<Eigen::Map<Matrix>> R;
    std::vector<Eigen::Map<Matrix>> S;
    std::vector<Eigen::Map<Vector>> b;
    std::vector<Eigen::Map<Vector>> q;
    std::vector<Eigen::Map<Vector>> r;
    std::vector<Eigen::Map<Matrix>> C;   // ng_max x nx
    std::vector<Eigen::Map<Vector>> d;   // ng_max
    std::vector<Eigen::Map<Vector>> lbx, ubx;
    std::vector<Eigen::Map<Vector>> lbu, ubu;

    // 软约束
    std::unique_ptr<SoftConstraintConfig> soft_config;

    // 构造
    QPData() = default;
    QPData(int N, int nx, int nu, int ng_max);
    
    // 【关键】彻底禁止拷贝和移动（自引用结构）
    QPData(const QPData&) = delete;
    QPData& operator=(const QPData&) = delete;
    QPData(QPData&&) = delete;
    QPData& operator=(QPData&&) = delete;

    void reset();  // 归零，不重新分配

    // 原始指针（供 HPIPM C API）
    double* raw_A(int k) { return A[k].data(); }
};

} // namespace stc_SQP
```

```cpp
// qp/qp_data.cpp
#include "qp/qp_data.hpp"

namespace stc_SQP {

// 【关键】将 double 数量向上取整到 4 的倍数（32 字节对齐，满足 AVX2）
// 若未来使用 AVX512，改为 (num + 7) & ~7
inline size_t AlignSize(size_t num_doubles) {
    return (num_doubles + 3) & ~3;
}

QPData::QPData(int N, int nx, int nu, int ng_max) 
    : N(N), nx(nx), nu(nu), ng_max(ng_max) {
    
    // 【关键】所有尺寸计算都必须经过 AlignSize，确保偏移量对齐
    size_t total = 0;
    
    total += N * AlignSize(nx * nx);      // A
    total += N * AlignSize(nx * nu);      // B
    total += (N + 1) * AlignSize(nx * nx); // Q
    total += N * AlignSize(nu * nu);      // R
    total += N * AlignSize(nu * nx);      // S
    total += N * AlignSize(nx);           // b
    total += (N + 1) * AlignSize(nx);     // q
    total += N * AlignSize(nu);         // r
    total += N * AlignSize(ng_max * nx);  // C
    total += N * AlignSize(ng_max);       // d
    total += 2 * (N + 1) * AlignSize(nx); // lbx, ubx
    total += 2 * N * AlignSize(nu);       // lbu, ubu
    
    memory_pool_.resize(total, 0.0);
    
    // 【关键】映射时必须使用 AlignSize 计算偏移，确保每个 Map 都 32 字节对齐
    size_t offset = 0;
    
    A.reserve(N);
    for (int k = 0; k < N; ++k) {
        A.emplace_back(&memory_pool_[offset], nx, nx);
        offset += AlignSize(nx * nx);
    }
    
    B.reserve(N);
    for (int k = 0; k < N; ++k) {
        B.emplace_back(&memory_pool_[offset], nx, nu);
        offset += AlignSize(nx * nu);
    }
    
    Q.reserve(N + 1);
    for (int k = 0; k < N + 1; ++k) {
        Q.emplace_back(&memory_pool_[offset], nx, nx);
        offset += AlignSize(nx * nx);
    }
    
    R.reserve(N);
    for (int k = 0; k < N; ++k) {
        R.emplace_back(&memory_pool_[offset], nu, nu);
        offset += AlignSize(nu * nu);
    }
    
    S.reserve(N);
    for (int k = 0; k < N; ++k) {
        S.emplace_back(&memory_pool_[offset], nu, nx);
        offset += AlignSize(nu * nx);
    }
    
    b.reserve(N);
    for (int k = 0; k < N; ++k) {
        b.emplace_back(&memory_pool_[offset], nx);
        offset += AlignSize(nx);
    }
    
    q.reserve(N + 1);
    for (int k = 0; k < N + 1; ++k) {
        q.emplace_back(&memory_pool_[offset], nx);
        offset += AlignSize(nx);
    }
    
    r.reserve(N);
    for (int k = 0; k < N; ++k) {
        r.emplace_back(&memory_pool_[offset], nu);
        offset += AlignSize(nu);
    }
    
    C.reserve(N);
    for (int k = 0; k < N; ++k) {
        C.emplace_back(&memory_pool_[offset], ng_max, nx);
        offset += AlignSize(ng_max * nx);
    }
    
    d.reserve(N);
    for (int k = 0; k < N; ++k) {
        d.emplace_back(&memory_pool_[offset], ng_max);
        offset += AlignSize(ng_max);
    }
    
    lbx.reserve(N + 1);
    ubx.reserve(N + 1);
    for (int k = 0; k < N + 1; ++k) {
        lbx.emplace_back(&memory_pool_[offset], nx);
        offset += AlignSize(nx);
        ubx.emplace_back(&memory_pool_[offset], nx);
        offset += AlignSize(nx);
    }
    
    lbu.reserve(N);
    ubu.reserve(N);
    for (int k = 0; k < N; ++k) {
        lbu.emplace_back(&memory_pool_[offset], nu);
        offset += AlignSize(nu);
        ubu.emplace_back(&memory_pool_[offset], nu);
        offset += AlignSize(nu);
    }
    
    // 防御性断言
    assert(offset == total && "Memory pool allocation mismatch!");
}

void QPData::reset() {
    std::fill(memory_pool_.begin(), memory_pool_.end(), 0.0);
}

} // namespace stc_SQP
```

### 3.7 QP 求解器状态（失败兜底）

```cpp
// qp/qp_solver.hpp
#pragma once
#include "qp_data.hpp"
#include "qp_solution.hpp"

namespace stc_SQP {

enum class QPSolverStatus {
    SUCCESS = 0,
    MAX_ITER_REACHED,
    INFEASIBLE,
    NAN_IN_SOLUTION,
    INVALID_ARGUMENT,
    UNKNOWN_ERROR
};

class QPSolver {
public:
    virtual ~QPSolver() = default;
    virtual QPSolverStatus solve(const QPData& qp_data, QPSolution& qp_sol) = 0;
    virtual void setTolerance(double tol) = 0;
    // 设置下一次 solve() 的热启动初值；具体求解器可选择是否实现
    virtual void setWarmStart(const QPSolution& qp_sol) = 0;
};

} // namespace stc_SQP
```

### 3.8 HPIPM 求解器（Partial Condensing + 软约束 + 跨迭代热启动）

```cpp
// qp/hpipm_solver.h
#pragma once
#include "qp_solver.h"

namespace stc_SQP {

class HPIPMQPSolver : public QPSolver {
public:
    // cond_N: Partial Condensing 后的宏观步数（N2）
    // cond_N = -1 表示禁用（直接用 OCP QP）
    // cond_N = 0 表示 Full Condensing（尚未实现）
    // 【Agent 注意】cond_N 是宏观步数，不是块大小！
    // 例如 N=100, block_size=10, 则 cond_N = 10
    // 当前支持 cond_N = -1（无凝聚）以及 0 < cond_N < N（Partial Condensing，仅当 ns=0 时启用）；
    // ns > 0 时自动回退到无凝聚路径，保证软约束求解正确性。
    HPIPMQPSolver(int N, int nx, int nu, int nbx, int nbu, int ng,
        int ns, int cond_N = -1);
    ~HPIPMQPSolver();

    HPIPMQPSolver(const HPIPMQPSolver&) = delete;
    HPIPMQPSolver& operator=(const HPIPMQPSolver&) = delete;

    // 软约束通过 qp_data.soft_config 传入，solve() 内部校验并设置
    QPSolverStatus solve(const QPData& qp_data, QPSolution& qp_sol) override;
    void setTolerance(double tol) override;
    // 设置下一次 solve() 的 IPM 热启动 primal 初值；维度不匹配时内部自动清空缓存，退化为冷启动
    void setWarmStart(const QPSolution& qp_sol) override;
    // 读取上一次 solve() 的 HPIPM IPM 内部迭代次数
    int lastIterations() const;
    // 读取自构造以来所有成功 solve() 的 IPM 内部迭代次数累计值
    int totalIterations() const;

protected:
    QPSolverStatus mapHpipmStatus(int status) const;

private:
    struct Impl;
    std::unique_ptr<Impl> pimpl_;
};

} // namespace stc_SQP
```

**热启动说明**：
- `setWarmStart()` 将传入的 `QPSolution`（含 `x`/`u`/`s`）写入 HPIPM 内部 `qp_sol`；
- `solve()` 在每次调用入口先重置 `warm_start` 标志为 0，仅在缓存有效且维度匹配时开启为 1，避免 stale 标志污染；
- 求解成功后用本次解自动更新缓存，求解失败或不可行时强制清空缓存；
- Partial Condensing 路径下，原始 OCP 解先写入 `qp_sol`，再通过 `d_part_cond_qp_cond_sol` 凝聚到 `cond_qp_sol` 作为热启动初值；
- 当前 `QPSolution` 不携带对偶变量，因此仅实现 primal-only 热启动（HPIPM warm_start 模式 1）；
- `lastIterations()`/`totalIterations()` 通过 HPIPM C API `d_ocp_qp_solver_get_iter` / `d_ocp_qp_ipm_get_iter` 读取，用于 benchmark 数据驱动决策。

### 3.9 CasADi 包装器（预分配工作区 + 线程克隆）

```cpp
// src/models/casadi_wrapper.h
#pragma once
#include <vector>

namespace stc_SQP {
namespace casadi {

class CasADiFunction {
public:
    CasADiFunction(void* handle, int n_in, int n_out);
    ~CasADiFunction();
    
    CasADiFunction(const CasADiFunction&) = delete;
    CasADiFunction& operator=(const CasADiFunction&) = delete;
    
    // 允许移动（但内部 iw/w 随对象移动）
    CasADiFunction(CasADiFunction&&) noexcept;
    CasADiFunction& operator=(CasADiFunction&&) noexcept;

    void operator()(const std::vector<const double*>& arg,
                    std::vector<double*>& res);

    // 【线程安全】克隆独立实例（每个线程一个）
    CasADiFunction clone() const;

private:
    void* handle_ = nullptr;
    int n_in_ = 0, n_out_ = 0;
    std::vector<int> iw_;
    std::vector<double> w_;
    
    void query_work_sizes();
};

} // namespace casadi
} // namespace stc_SQP
```

### 3.10 SQP 主求解器（RTI 降级 + 策略模式 + assembleQP 并行）

```cpp
// sqp/sqp_algorithm.hpp
#pragma once
#include <memory>
#include "core/types.h"

namespace stc_SQP {

struct SQPSolverOptions {
    int max_iter = 10;
    double kkt_tol = 1e-6;
    double constr_viol_tol = 1e-6;
    double stationarity_tol = 1e-6;

    bool use_rti = false;  // 【警告】泊车含换挡点时必须 false
    bool use_line_search = true;

    double reg_min = 1e-12;
    double reg_max = 1e8;
    double reg_factor = 10.0;

    bool use_slack = true;
    double slack_penalty = 1e4;

    // 是否启用 OpenMP 并行 linearize
    bool use_omp = true;
    // OpenMP 并行阈值
    int omp_parallel_threshold = 50;
    // 线搜索最小可接受步长
    double line_search_alpha_min = 1e-4;
    // 线搜索回退系数
    double line_search_rho = 0.5;
    // Armijo 常数
    double line_search_c = 1e-4;
    // merit function L1 罚重
    double merit_penalty = 1e4;
    // 是否在 Full SQP 循环中跨迭代复用上一次 QP 解作为 HPIPM IPM 热启动；
    // RTI 单步模式不受此开关影响（无上一次迭代可复用）。默认 false。
    bool use_qp_warm_start = false;
};

class SQPSolver {
public:
    explicit SQPSolver(std::unique_ptr<QPSolver> qp_solver);

    bool solve(const MultiStageOCP& ocp,
               const Trajectory& initial_guess,
               Trajectory& solution);

    SQPSolverOptions& options() { return options_; }
    const SQPSolverOptions& options() const { return options_; }

    bool rtiDowngraded() const;
    bool converged() const;

protected:
    bool iterate();
    bool validateProblem(const MultiStageOCP& ocp,
        const Trajectory& initial_guess, std::string* reason) const;
    bool linearize();           // 含 OpenMP 并行分发与 clone 池复用
    bool assembleQP();
    bool assembleCost(int global_k, const StageSegment& segment,
        const Vector& x, const Vector& u);
    void assembleBounds(int global_k, const StageSegment& segment,
        const Vector& x, const Vector& u);
    bool linearizeStep(int segment_idx, int step_in_segment, int global_k,
        double dt, const std::vector<std::shared_ptr<Constraint>>& constraints,
        Vector& local_x_next, Matrix& local_A, Matrix& local_B);
    bool evaluateConstraintValue(const Constraint& constraint, const Vector& x,
        const Vector& u, const Vector& p, int global_k, bool in_parallel,
        Vector& g) const;
    bool evaluateConstraintLinearization(const Constraint& constraint,
        const Vector& x, const Vector& u, const Vector& p, int global_k,
        bool in_parallel, Vector& g, Matrix& Cx, Matrix& Cu) const;
    bool solveQP();
    bool lineSearch(double& alpha);
    double computeMerit(const Trajectory& traj) const;
    double computeDirectionalDerivative() const;
    double computeConstraintViolation(const Trajectory& traj) const;
    bool checkConvergence();
    bool applyRetraction(const MultiStageOCP& ocp, const Trajectory& current,
        double alpha, const Trajectory& delta, Trajectory& result);

    std::unique_ptr<QPSolver> qp_solver_;
    std::unique_ptr<QPData> qp_data_;
    QPSolution qp_solution_;
    SQPSolverOptions options_;
    Trajectory current_traj_;
    Trajectory delta_traj_;
    const MultiStageOCP* ocp_ = nullptr;
    int iter_count_ = 0;
    Vector cost_q_, cost_r_, mutable_g_, zero_u_;
    Matrix cost_Q_, cost_R_, cost_S_;
    Vector lin_x_next_;
    Matrix lin_A_, lin_B_;
    std::vector<std::vector<std::vector<std::shared_ptr<Constraint>>>>
        thread_constraint_clones_;
    // 与 thread_constraint_clones_ 对称的 cost 克隆池，供 assembleQP/assembleCost 并行路径使用
    std::vector<std::vector<std::shared_ptr<CostTerm>>> thread_cost_clones_;
    bool rti_mode_active_ = false;
    bool rti_downgraded_ = false;
    bool converged_ = false;
};

} // namespace stc_SQP
```

**实现说明**：策略模式在实现中采用“策略持有 `SQPSolver`”的反向依赖——
`AutoAdaptiveStrategy` / `ManualHierarchicalStrategy` 内部构造 `HPIPMQPSolver` 与 `SQPSolver`，
按自身逻辑配置 `cond_N`、`use_omp`、迭代次数等参数后调用 `SQPSolver::solve()`。
这一反向避免了 `SQPSolver` 依赖策略抽象，简化了对象生命周期，与 5.1 的策略接口仍然兼容。

### 3.11 业务层：ProblemUpdater（完备性断言）

```cpp
// examples/parking/problem_updater.hpp
#pragma once
#include "ocp/multi_stage_ocp.hpp"

namespace stc_SQP {

struct UpdaterConfig {
    // 【语义关键】selection_radius 必须是基于 GJK 的
    // "障碍物到车辆矩形轮廓的精确距离"，而非后轴中心距离
    double selection_radius;      
    
    double max_step_displacement; // Proximity Bounds 幅度（Δp_max）
    double safety_margin;         // 额外安全裕度
    int top_k;                    // 最大约束数（静态维度）
};

class ProblemUpdater {
public:
    explicit ProblemUpdater(const UpdaterConfig& config);

    void updateOcp(const Trajectory& current_traj,
                   const MapInterface& map,
                   MultiStageOCP& ocp);

private:
    UpdaterConfig config_;
    
    // 【关键】完备性断言：筛选半径必须大于单步最大位移 + 安全裕度
    // 前提：selection_radius 基于 GJK 轮廓距离（非中心距离）
    void assert_completeness() const {
        assert(config_.selection_radius > 
               config_.max_step_displacement + config_.safety_margin &&
               "FATAL: 筛选半径过小，数学完备性被破坏！"
               "确保 selection_radius 基于 GJK 轮廓距离！");
    }
};

} // namespace stc_SQP
```

#### ProblemUpdater 使用契约

- `updateOcp()` 入口会先调用 `ocp.validate()`，只接受合法 OCP。
- 调用前 `stage_params` 允许为空；也允许预分配为 `N` 个空 `p`，`ProblemUpdater` 会将其归一化为空并按无既有参数处理。
- 若 `stage_params` 已包含非空参数，则每步 `p` 必须为 `STAGE_PARAM_DIM`（150）维且全部有限，`ProblemUpdater` 仅覆盖其中的凸走廊区间 `p[15:45]`，其余槽位保持不变。

#### SimpleParkingMap 语义边界

`SimpleParkingMap` 是 最小可用地图实现，其语义边界必须明确：

- **每面墙由一条有限线段与一个无限半空间共同描述**：
  - 有限线段 `start -> end` 仅用于 GJK 轮廓距离计算，即障碍物到车辆矩形四条边的精确最短距离；
  - 无限半空间 `(normal, intercept)` 才是最终注入 `Constraint` 的凸化约束，车辆在该半空间的可行侧。
- **筛选阶段使用线段距离，返回结果是半空间**：`queryHalfSpaces()` 按车辆四角点包络到各墙线段的 GJK 距离排序并取 Top-K，但返回给 `ProblemUpdater` 的是对应无限半空间，因此：
  - 被截断的远端墙不影响结果；
  - 同一半空间可能对应很长的墙段，GJK 距离仍按有限线段计算，而不是到无限边界直线的距离。
- **生产系统替换提示**：真实地图应当提供车辆包络到障碍物（多边形/曲线）的精确距离，并输出等价或近似的凸半空间；`SimpleParkingMap` 的“有限线段 = 无限半空间边界”只是教学示例，不表示墙只影响线段覆盖区域。

---

## 4. CasADi 代码生成规范

### 4.1 通用参数 `p` 的语义定义（common.py）

```python
# autogen/common.py
# 定义 p 的维度分配（所有脚本共享）
P_DIM = 150

# p[0:3]   : x_ref, y_ref, theta_ref (Tracking Cost)
# p[3:5]   : 摩擦系数, 坡度 (Dynamics)
# p[5:15]  : 动态权重 (Cost)
# p[15:45] : 10 个凸走廊半空间 (A: 10x2=20, b: 10) -> 实际按需分配
# p[45:150]: 预留
```

### 4.2 凸走廊约束生成（含 4 角点 × K 半空间）

```python
# autogen/generate_corridor.py
import casadi as ca
from common import NX, P_DIM, L_f, L_r, W

def generate_corridor(output_dir: str):
    x = ca.SX.sym('x', NX)      # 状态 [x, y, theta, v, delta]（前轮转角控制版 BicycleModelDelta，本项目默认运动学模型）
    p = ca.SX.sym('p', P_DIM)   # 参数
    
    # 从 p 解包半空间参数
    A_flat = p[15:15+20]        # 10 个 2D 法向量，展平存储
    b = p[15+20:15+30]          # 10 个截距
    
    # 车辆 4 个角点在车身坐标系下的局部坐标
    CORNERS_LOCAL = [
        ca.DM([L_f,  W/2]),   # FL
        ca.DM([L_f, -W/2]),   # FR
        ca.DM([-L_r, W/2]),   # RL
        ca.DM([-L_r,-W/2]),   # RR
    ]
    
    N_CORNERS = 4
    N_HS = 10
    G_DIM = N_CORNERS * N_HS    # 40
    
    g = ca.SX.zeros(G_DIM)
    
    # 旋转矩阵（含 theta 非线性）
    theta = x[2]
    R = ca.vertcat(
        ca.horzcat(ca.cos(theta), -ca.sin(theta)),
        ca.horzcat(ca.sin(theta),  ca.cos(theta))
    )
    
    for corner_idx in range(N_CORNERS):
        p_local = CORNERS_LOCAL[corner_idx]
        p_world = ca.vertcat(x[0], x[1]) + R @ p_local
        
        for hs_idx in range(N_HS):
            A_i = ca.reshape(A_flat[hs_idx*2 : (hs_idx+1)*2], 2, 1)
            g[corner_idx * N_HS + hs_idx] = ca.dot(A_i, p_world) - b[hs_idx]
    
    # 【关键】生成完整 Jacobian Cx = dg/dx（含 theta 非线性）
    Cx = ca.jacobian(g, x)
    
    f = ca.Function('corridor', [x, p], [g, Cx])
    ca.generate_code(f, f'{output_dir}/corridor.c')
    
    # 生成头文件说明维度
    with open(f'{output_dir}/corridor.h', 'w') as f_h:
        f_h.write(f"""
#ifndef CORRIDOR_H
#define CORRIDOR_H
#ifdef __cplusplus
extern "C" {{
#endif
void corridor(const double** arg, double** res, int* iw, double* w, int mem);
// arg: [x(5), p(150)]
// res: [g(40), Cx(40x5)=200]
#ifdef __cplusplus
}}
#endif
#endif
""")
```

**Agent 强制注意**：生成的 `g` 维度是 40（4 角点 × 10 半空间），不是 10。HPIPM 的 `ng` 每步必须设为 40。

---

## 5. 长 N 策略与 HPIPM 配置

### 5.1 策略模式接口

```cpp
// strategies/solver_strategy.hpp
#pragma once
#include "core/types.h"

namespace stc_SQP {

class MultiStageOCP;
class Trajectory;

class SolverStrategy {
public:
    virtual ~SolverStrategy() = default;
    virtual bool solve(const MultiStageOCP& ocp, 
                       const Trajectory& initial_guess,
                       Trajectory& solution) = 0;
};

} // namespace stc_SQP
```

### 5.2 自动自适应策略

```cpp
// strategies/auto_adaptive_strategy.hpp
#pragma once
#include "solver_strategy.hpp"

namespace stc_SQP {

class AutoAdaptiveStrategy : public SolverStrategy {
public:
    bool solve(const MultiStageOCP& ocp, 
               const Trajectory& initial_guess,
               Trajectory& solution) override {
        int N = ocp.total_steps();
        
        if (N < 50) {
            // 短 N：串行 linearize + HPIPM OCP QP（无 condensing）
            return single_sqp_solve(ocp, initial_guess, solution, 
                                    /*cond_N=*/-1, /*use_omp=*/false);
        } else {
            // 长 N：OpenMP 并行 + Partial Condensing
            // 【Agent 注意】cond_N 是宏观步数，不是块大小！
            int block_size = 10;
            int cond_N = (N + block_size - 1) / block_size;  // 向上取整
            return single_sqp_solve(ocp, initial_guess, solution,
                                    cond_N, /*use_omp=*/true);
        }
    }
};

} // namespace stc_SQP
```

### 5.3 手动分层策略（Coarse-to-Fine）

```cpp
// strategies/manual_hierarchical_strategy.hpp
#pragma once
#include "solver_strategy.hpp"

namespace stc_SQP {

struct HierarchicalOptions {
    bool enable = true;
    int coarse_n = -1;            // -1 = 自动
    double coarse_dt = -1.0;      // -1 = 自动
    int coarse_max_iter = 5;
    int fine_max_iter = 2;
};

class ManualHierarchicalStrategy : public SolverStrategy {
public:
    explicit ManualHierarchicalStrategy(const HierarchicalOptions& opts);
    
    bool solve(const MultiStageOCP& ocp, 
               const Trajectory& initial_guess,
               Trajectory& solution) override {
        // 1. 构建粗 OCP
        auto coarse_ocp = ocp.coarsen(opts_.coarse_n, opts_.coarse_dt);
        
        // 2. 粗优化
        Trajectory coarse_solution;
        if (!sqp_solve(coarse_ocp, initial_guess, coarse_solution, 
                       opts_.coarse_max_iter))
            return false;
        
        // 3. 动力学一致插值（非简单线性！）
        Trajectory fine_guess = interpolate_dynamics_consistent(
            coarse_solution, ocp.total_steps(), ocp.dynamics());
        
        // 4. 细优化
        return sqp_solve(ocp, fine_guess, solution, opts_.fine_max_iter);
    }
};

} // namespace stc_SQP
```

### 5.4 HPIPM cond_N 语义（Agent 强制注释）

```cpp
// 【Agent 强制理解】
// HPIPM 的 Partial Condensing 中：
// - N  = 原始打靶步数（如 100）
// - cond_N（N2）= 凝聚后的宏观步数
// 
// 若 block_size = 10，则 cond_N = N / block_size = 10
// 传入 HPIPM 的是 cond_N = 10，不是 block_size = 10
//
// 若 N = 53，block_size = 10，则 cond_N = 6（向上取整）
// HPIPM 内部自动处理最后一块（3 步）的边界
```

---

## 6. 内存与线程安全

### 6.1 OpenMP 并行 linearize（线程隔离）

```cpp
bool SQPSolver::linearize() {
    const int N = ocp_.total_steps();
    
    if (N < options_.omp_parallel_threshold) {
        // 短 N：串行
        for (int i = 0; i < N; ++i) {
            linearize_stage(i);
        }
    } else {
        // 长 N：OpenMP 并行
        // 【关键】每个线程持有独立的 CasADi 实例
        #pragma omp parallel
        {
            auto f_local = casadi_dynamics_.clone();
            auto g_local = casadi_corridor_.clone();
            
            #pragma omp for schedule(static, 32)
            for (int i = 0; i < N; ++i) {
                linearize_stage(i, f_local, g_local);
            }
        }
    }
    return true;
}
```

### 6.2 QPData 内存布局（零拷贝 + 偏移量对齐）

已在 3.5 节完整展示，核心要点：
- `AlignSize(num) = (num + 3) & ~3`，确保每个矩阵起始地址 32 字节对齐
- 所有 `total` 计算和 `offset` 累加都必须经过 `AlignSize`
- 防御性断言 `offset == total`

---

## 7. 测试策略

| 测试文件 | 测试内容 | 通过标准 |
|---------|---------|---------|
| `test_math.cpp` | SO2 retract、角度 wrap | 误差 < 1e-12 |
| `test_bicycle_model.cpp` | 手写 vs CasADi vs 数值差分 | Jacobian 相对误差 < 1e-6 |
| `test_integrator.cpp` | RK4 精度 | 位置误差 < 1e-4 |
| `test_corridor_constraint.cpp` | 4 角点 × 10 半空间，含 theta 非线性 | Jacobian 与数值差分误差 < 1e-6 |
| `test_qp_solvers.cpp` | HPIPM vs Dense LDLT | 解的 L2 误差 < 1e-8 |
| `test_memory_pool.cpp` | 对齐、零拷贝、raw 指针、偏移量 32 字节对齐 | 内存地址连续，SIMD 安全 |
| `test_sqp_convergence.cpp` | 无约束 LQR | 1 次迭代收敛到解析解 |
| `test_parking.cpp` | 端到端泊车 | 无碰撞、求解时间 < 20ms（短 N）/ < 50ms（长 N） |

> **性能与计时类用例不放在单元测试中**：墙钟计时断言（如长 N 场景 < 50ms）在 CI 高负载下易 flaky，不适合作为 GTest 硬断言。所有性能剖析与串行/并行、策略一致性基准统一迁移到 Google Benchmark 目标 `bench_SQP_solver`（源文件 `bench/bench_performance_profiling.cpp`）。基准内部仍保留一次性一致性校验：串行 vs OpenMP+Partial Condensing、`AutoAdaptiveStrategy`/`ManualHierarchicalStrategy` 与显式求解一致等，不一致时通过 `state.SkipWithError()` 显式失败，保留回归价值。运行方式：`./bench_SQP_solver`（可用 `--benchmark_filter` 选跑单项）。

---

## 9. Agent 强制指令（避坑清单）

1. **四角点维度**：`corridor.c` 生成的 `g` 必须是 40 维（4 角点 × 10 半空间），`Cx` 是 40×5。HPIPM 的 `ng` 每步设为 40。
2. **筛选半径语义**：`selection_radius` 必须基于 **GJK 轮廓距离**（障碍物到车辆矩形边缘），而非后轴中心距离。否则完备性断言失效。
3. **cond_N 语义**：HPIPM 的 `cond_N` 是**凝聚后的宏观步数**（`N / block_size`），不是块大小。例如 N=100, block_size=10, cond_N=10。
4. **Jacobian 非线性**：凸走廊约束的 `Cx` **不是常数**，含 $\theta$ 的非线性项。必须通过 CasADi 生成函数每次迭代重算。
5. **内存对齐**：`QPData` 的 `memory_pool_` 使用 `Eigen::aligned_allocator<double>`，且**所有偏移量必须通过 `AlignSize` 填充到 32 字节边界**。禁止直接 `offset += nx * nx`。
6. **角度安全**：任何涉及 `theta` 的更新必须走 `retract()`，禁止裸 `+=`。
7. **线程安全**：OpenMP 并行时，每个线程必须持有独立的 `CasADiFunction` 实例（通过 `clone()`）。
8. **求解器失败兜底**：`solve_qp()` 返回非 `SUCCESS` 时，`delta_traj_` 必须视为不可用，禁止应用到 `current_traj_`。
9. **RTI 限制**：泊车含换挡点时 `use_rti` 必须设为 `false`。检测到换挡点时自动降级为 Full SQP。
10. **CMake 依赖**：修改 `common.py` 后必须触发重新生成，通过 `COMMON_DEPS` 保证。

---

## 10. 应用场景与调用范式（短 N / 长 N）

本节总结本框架要解决的**实际问题**、两种典型输入形态（短 N / 长 N）的差异，以及对应的调用范式伪代码，供业务层（`examples/parking/` 及后续接入方）参考。

### 10.1 核心问题定位

本框架解决的是**泊车场景下的轨迹优化（NMPC）问题**，与常规行车 NMPC 的本质区别在于：

- 路径**天然由多段方向不同的 maneuver 拼接而成**（前进/倒车交替），而非单一方向的连续轨迹；
- 换挡点处车速必须为 0，但**换挡点的检测与分段完全由外部业务层完成**，SQP 引擎本身不做自动分段，只负责把业务层拼好的 `MultiStageOCP`（多段 `StageSegment` 序列）求解出来；
- 车辆只需运动学自行车模型（本项目默认 `BicycleModelDelta`，状态 `[x, y, theta, v, delta]`），不涉及动力学（质量、惯量、侧偏刚度等）；
- 障碍物信息可能来自两种地图形式：**ESDF 距离场**（给定位置到最近障碍物的距离+梯度，新增）或 **SFC 凸走廊**（半空间集合，已实现），两者都通过 `Constraint` 接口 + `p` 向量注入，SQP 引擎不区分。

### 10.2 两种典型输入形态对比

| 维度 | 短 N（几十步） | 长 N（三四百步） |
|------|---------------|------------------|
| 典型用途 | 滚动时域 MPC，单次求解只覆盖当前及后续 1~2 个换挡段 | 离线/一次性求解完整泊车轨迹（如 `data3.json` 的 9 段换挡） |
| 求解器配置 | `cond_N = -1`（无 Partial Condensing）或省略，`use_omp = false` | `cond_N = ceil(N / block_size)`（Partial Condensing），`use_omp = true` |
| 推荐策略 | `AutoAdaptiveStrategy` 短分支，或直接构造 `SQPSolver` + `HPIPMQPSolver(cond_N=-1)` | `AutoAdaptiveStrategy` 长分支，或 `ManualHierarchicalStrategy`（Coarse-to-Fine 两阶段） |
| 段数量级 | 通常 1~2 段（当前滚动窗口内的换挡） | 完整路径的全部段（如 8~9 次换挡） |
| RTI | 若窗口内无换挡可尝试 `use_rti=true`（引擎会在检测到换挡时自动局部降级为 Full SQP） | 通常直接 Full SQP（`use_rti=false`），换挡段数多，RTI 收益有限 |
| 性能验收基准 | 新增：真实模型+约束的短 N 集成测试 | 新增：真实模型+约束的长 N 专项基准（替代当前简化的 `BM_Data3InitialPath_N493`） |

### 10.3 调用范式伪代码

```cpp
// ============ 业务层：从原始搜索路径构造多段 OCP（框架不做自动分段） ============
MultiStageOCP buildParkingOcp(const std::vector<PathPoint>& raw_path,
                              const MapInterface& map /* 或 EsdfMapInterface，新增 */) {
    // 1. 换向检测 + 外部分段（业务层职责，见前述 splitPathByDirection）
    auto runs = splitPathByDirection(raw_path);
    auto model = std::make_shared<BicycleModelDelta>(/*wheelbase=*/2.8); // 默认运动学模型

    MultiStageOCP ocp;
    for (const auto& run : runs) {
        StageSegment seg = buildSegment(run, model);   // 按方向设置 v 的 box bound（换挡处自动 v=0）
        seg.constraints.push_back(buildObstacleConstraint(map)); // 走廊(SFC) 或 ESDF 约束，二选一/共存
        ocp.addSegment(seg);
    }
    return ocp;
}

// ============ 短 N：滚动时域调用（每个控制周期截取局部窗口求解） ============
bool solveShortHorizon(const MultiStageOCP& windowed_ocp, const Trajectory& warm_start,
                       Trajectory& solution) {
    auto qp = std::make_unique<HPIPMQPSolver>(
        windowed_ocp.totalSteps(), windowed_ocp.nx(), windowed_ocp.nu(),
        /*nbx=*/windowed_ocp.nx(), /*nbu=*/windowed_ocp.nu(), /*ng=*/kNg, /*ns=*/0,
        /*cond_N=*/-1);                       // 短 N 不启用 Partial Condensing
    SQPSolver solver(std::move(qp));
    solver.options().use_omp = false;         // N 小，串行 linearize 即可
    solver.options().use_rti = true;          // 窗口内若无换挡可享受 RTI；有换挡会被自动降级
    return solver.solve(windowed_ocp, warm_start, solution);
}

// ============ 长 N：一次性求解完整多段泊车轨迹 ============
bool solveLongHorizon(const MultiStageOCP& full_ocp, const Trajectory& warm_start,
                      Trajectory& solution) {
    const int N = full_ocp.totalSteps();          // 如 493
    const int block_size = 10;
    const int cond_N = (N + block_size - 1) / block_size;  // 宏观步数，不是块大小

    auto qp = std::make_unique<HPIPMQPSolver>(
        N, full_ocp.nx(), full_ocp.nu(), full_ocp.nx(), full_ocp.nu(), /*ng=*/kNg, /*ns=*/0,
        cond_N);                                   // 长 N 启用 Partial Condensing
    SQPSolver solver(std::move(qp));
    solver.options().use_omp = true;               // OpenMP 并行 linearize
    solver.options().omp_parallel_threshold = 50;
    solver.options().use_rti = false;              // 多段换挡场景直接 Full SQP
    return solver.solve(full_ocp, warm_start, solution);
    // 若单次 Full SQP 迭代成本过高，可改用 ManualHierarchicalStrategy 做 Coarse-to-Fine
}
```

