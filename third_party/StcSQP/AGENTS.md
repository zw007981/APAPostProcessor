# Agent 工作指南

## 关键文档索引

本项目所有 Agent 必须首先阅读并遵循以下两份文档：

- **设计文档**：`.github/design_document.md`
  - 描述 StcSQP 的整体架构、模块接口、CasADi 代码生成规范、HPIPM 配置以及 Agent 强制指令（避坑清单）。
- **C++ 编程规范**：`.github/copilot-instructions.md`
  - 规定本仓库的 C++17 编码风格、命名规范、注释语言（中文优先）、垂直空白密度、访问控制（`protected` 优先）、RAII、异常处理、现代 C++ 特性等强制要求。

本 `AGENTS.md` 仅作为上述两份文档的**摘要与补充**，若存在冲突，以原始文档为准。

## 项目定位

StcSQP 是一个面向自动驾驶轨迹优化的**量产级 C++ SQP 数值优化框架**。核心设计哲学是：

- **SQP 引擎对物理世界“一无所知”**：只识别固定维度的矩阵与梯度。
- **静态维度，动态参数**：所有业务语义通过通用参数向量 `p` 注入，C++ 层不解释语义。
- **极简、高效、内存安全**：遵循 C++17 标准，优先使用 RAII、预分配、Eigen Map 视图，禁止裸指针。

## 强制避坑清单（来自设计文档第 9 章）

1. **四角点维度**：凸走廊约束 `g` 为 40 维（4 角点 × 10 半空间），`Cx` 为 40×5；HPIPM 每步 `ng = 40`。
2. **筛选半径语义**：`selection_radius` 必须基于 **GJK 车辆轮廓距离**，而非后轴中心距离。
3. **cond_N 语义**：HPIPM 的 `cond_N` 是凝聚后的**宏观步数**（`N / block_size`），不是块大小。
4. **Jacobian 非线性**：凸走廊 `Cx` 含 `theta` 非线性项，每次迭代需通过 CasADi 函数重算。
5. **内存对齐**：详见下文【内存对齐专节】。
6. **角度安全**：任何 `theta` 更新必须走 `retract()`，禁止裸 `+=`。
7. **线程安全**：OpenMP 并行时，每个线程必须持有独立的 `CasADiFunction` 实例（通过 `clone()`）。
8. **求解器失败兜底**：`solve_qp()` 非 `SUCCESS` 时，`delta_traj_` 不可用，禁止应用到 `current_traj_`。
9. **RTI 限制**：泊车含换挡点时 `use_rti = false`，检测到换挡点自动降级为 Full SQP。
10. **CMake 依赖**：修改 `common.py` 后需通过 `COMMON_DEPS` 触发重新生成。

## 内存对齐专节（设计文档第 9 章第 5 点）

### 为什么必须对齐

`QPData` 的 `memory_pool_` 是一块连续的双精度浮点内存池，上面通过 `Eigen::Map` 映射出大量矩阵/向量视图。Eigen 在开启向量化（SSE/AVX2/AVX512）时，会对满足对齐条件的地址使用 SIMD 加载/存储指令：

- AVX2 要求 32 字节对齐。
- AVX512 要求 64 字节对齐。

如果 `Eigen::Map` 的起始地址未对齐，轻则触发未对齐访问的性能惩罚，重则直接产生 **Segfault（段错误）**。在实时轨迹优化场景中，这种崩溃是不可接受的。

### 对齐策略

设计文档要求使用 `AlignSize(num_doubles)` 将每个块的**元素个数**向上取整到 4 的倍数：

```cpp
inline size_t AlignSize(size_t num_doubles) {
    return (num_doubles + 3) & ~3;  // 4 doubles = 32 bytes
}
```

由于每个 `double` 占 8 字节，4 个 double 正好是 32 字节，满足 AVX2 对齐要求。若未来启用 AVX512，应改为 `(num + 7) & ~7`（64 字节）。

### 必须同时出现的两个地方

1. **总容量计算**：每个矩阵/向量块占用的元素数量都要经过 `AlignSize`。
2. **偏移量推进**：每次 `offset += ...` 也必须经过 `AlignSize`。

禁止直接写 `offset += nx * nx` 或 `offset += nx`，否则后续 `Eigen::Map` 的起始地址可能落在非 32 字节边界上。

### 示例

```cpp
size_t total = 0;
total += N * AlignSize(nx * nx);      // A
total += N * AlignSize(nx * nu);      // B
total += N * AlignSize(ng_max * nx);  // C (一般约束状态 Jacobian)
total += N * AlignSize(ng_max * nu);  // D (一般约束控制 Jacobian)
// ... 其它块

memory_pool_.resize(total, 0.0);

size_t offset = 0;
A.reserve(N);                         // 预留精确容量，避免重分配触发 Map 的拷贝/移动
for (int k = 0; k < N; ++k) {
    A.emplace_back(&memory_pool_[offset], nx, nx);
    offset += AlignSize(nx * nx);     // 必须对齐推进
}
// ... 其它块

if (offset != total) {
    throw std::logic_error("Memory pool allocation mismatch");
}
```

### 分配器选择

`memory_pool_` 的类型是：

```cpp
AlignedVector<double> memory_pool_;
```

其中 `AlignedVector` 定义为：

```cpp
template<typename T>
using AlignedVector = std::vector<T, Eigen::aligned_allocator<T>>;
```

`Eigen::aligned_allocator` 保证 `std::vector` 内部缓冲区的起始地址满足 Eigen 的对齐要求（通常至少 16/32 字节）。结合 `AlignSize` 的偏移填充，池中每个 `Eigen::Map` 块都能获得独立的对齐起始地址。


### 拷贝语义

`QPData` 包含自引用的 `Eigen::Map` 视图（指向 `memory_pool_` 内部），因此必须彻底禁止拷贝和移动：

```cpp
QPData(const QPData&) = delete;
QPData& operator=(const QPData&) = delete;
QPData(QPData&&) = delete;
QPData& operator=(QPData&&) = delete;
```

若误用值语义，Map 会指向已被释放或移动的内存，导致段错误。

### 设计约定（补充）

1. **角度规范化**：`math_util::Wrap` 统一将角度映射到 **(-π, π]**；`±π` 统一表示为 `π`，避免同一角度存在两个等价表示。
2. **SE2 位姿组合**：`se2::Compose` 对组合后的 `theta` 调用 `math_util::Wrap`，保证位姿表示唯一性。
3. **离散状态传播中的 theta**：`BicycleModelKappa` 与 `BicycleModelDelta` 的 `discretizeAndLinearize()` 在 RK4 积分结束后对 `x_next` 的 `theta` 维度调用 `math_util::Wrap`，确保预测状态不越界。SQP 外层对状态的正式更新仍必须通过 `so2::Retract()` 完成。
4. **CasADi 生成代码维度检查**：`autogen/generate_dynamics.py` 在生成 C 代码后通过 `_assert_signature` 断言输入输出维度，防止未来模型维度变化时静默生成错误头注释。

### 代码风格补充约定

1. **命名空间**：统一使用最外层命名空间 `stc_SQP`。
2. **常量管理**：数学与工程常量统一从 `src/util/constants.h` 获取。
3. **`static constexpr` 常量命名**：与 `.github/copilot-instructions.md` 保持一致，`static constexpr` 的整型/浮点/枚举常量使用 `k` 前缀小驼峰（如 `kParameterDim`、`kVehicleLf`），运行期静态成员变量使用 `MACRO_CASE`。
3. **相关变量合并初始化**：当多个变量在语义上紧密相关且由同类表达式初始化时，优先写在一行并用逗号分隔。例如：
   ```cpp
   const double c = std::cos(theta), s = std::sin(theta);
   const double dx = p(0) - pose(0), dy = p(1) - pose(1);
   ```
   这样比拆成多行更能体现变量间的耦合关系。

### 单元测试注释规范（强制）

本项目所有 GTest 单元测试用例必须包含**中文注释**，明确说明以下三点：

1. **测试目的**：该用例要验证什么功能或约束。
2. **测试流程**：用例经过哪些步骤（构造数据、调用接口、做何种比较）。
3. **预期效果**：期望的输出、误差范围或系统行为。

注释应直接放在 `TEST(...)` 或 `TEST_F(...)` 宏之后、具体代码之前，格式如下：

```cpp
TEST(Math, WrapNormalizesAngle) {
    // 测试目的：验证 wrap 函数将任意角度映射到 [-π, π] 区间
    // 流程：输入超过 2π 的角度，检查输出是否在预期区间且与原始角度等价
    // 预期效果：wrap(3π) = π，wrap(-3π) = -π
    EXPECT_NEAR(...);
}
```

### 测试要求

对应单元测试 `test_memory_pool.cpp` 需要验证：

- 内存池中各 `Eigen::Map` 的起始地址满足 32 字节对齐。
- `raw_A(k)`、`raw_B(k)` 等返回的裸指针与 Map 数据一致。
- `reset()` 归零后池内数据全为 0，但不重新分配内存。
- 构造后 `offset == total` 断言成立。
