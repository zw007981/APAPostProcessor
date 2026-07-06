---
applyTo: "CMakeLists.txt,**/*.cmake"
description: "CMake 构建脚本约定，源自参考样例提炼，逐条遵循"
---

# CMake 构建规范

## 1. 章节化与注释密度

- 用 `# ==========================================` 分割线 + 编号小节标题（如"0. 全局与编译配置"）组织整个 `CMakeLists.txt`，保持长文件可读。
- 每个关键设置（如 ccache、`-march=native`、Ninja job pool）都必须用中文注释说明"为什么这样做"，不能只写"做了什么"。

## 2. 目录与目标（Target）约定

- 源码目录：`src/`（核心业务代码）；`main.cpp` 单独剔除后，其余源码打包为 `OBJECT` 库（如 `xxx_core_lib`），主程序只链接该库，避免重复编译。
- 单元测试目录：`test/`（**单数**，不是 `tests/`）；入口文件固定为 `test/main.t.cpp`，其余每个测试文件采用**后缀风格** `<被测对象>.t.cpp`（如 `pose.t.cpp`、`kinematic_model.t.cpp`），通过 `file(GLOB_RECURSE ... "test/*.cpp" "test/*.t.cpp")` 收集。**不建议**改成前缀风格（如 `test_pose.cpp`）：后缀风格能让 `pose.h`/`pose.cpp`/`pose.t.cpp` 在目录里按字母序天然聚在一起，定位一个模块对应的测试更快；`bench/` 采用前缀风格是历史原因，两者不必强行对称。
- 性能压测目录：`bench/`，与 `test/` 同级；入口文件 `bench/main.bench.cpp`，其余压测文件命名 `bench/bench_*.cpp`。
- 第三方库若需要源码内置，放在 `third_party/`，用 `add_subdirectory` 引入；缺失时用 `FATAL_ERROR` 明确报错，不要静默失败。

## 3. 依赖处理策略

- **必需依赖**（如 Protobuf/OpenCV/Eigen3）：`find_package(... REQUIRED)`，缺失直接终止配置。
- **可选依赖**（如 GTest/Google Benchmark）：`find_package(... QUIET)` + 条件判断，缺失时打印 `WARNING` 并优雅跳过对应 target，**禁止因为可选依赖缺失导致整个配置失败**。

## 4. 编译优化项（需保留注释说明原因）

- 优先启用 `ccache`（`find_program(CCACHE_PROGRAM ccache)`），找到则设置 `CMAKE_CXX_COMPILER_LAUNCHER`。
- 用 `-fmacro-prefix-map` 把 `__FILE__` 映射为工程内相对路径，需同时处理绝对路径（Make 等生成器）与相对路径（Ninja 等生成器）两种场景。
- x86_64 平台默认开启 `-march=native`（发挥 AVX2/FMA），需注释说明"如需跨平台部署可关闭此选项"。
- Ninja 生成器下用 `JOB_POOLS` 限制并发编译数（如默认 `nproc - 6`，避免把开发机资源占满），逻辑需保留注释解释来源与目的。

## 5. 默认构建类型

未指定 `CMAKE_BUILD_TYPE` 时默认 `Release`。

---

> 本文件基于一份参考 `CMakeLists.txt` 提炼，如与实际项目需求冲突，以讨论后的结论为准并同步更新本文件。
