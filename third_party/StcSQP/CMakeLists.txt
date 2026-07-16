cmake_minimum_required(VERSION 3.15)
project(StcSQP VERSION 1.0.0 LANGUAGES C CXX)

# ==========================================
# 0. 全局与编译配置
# ==========================================
# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 默认构建类型
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# 启用 ccache 编译缓存加速
find_program(CCACHE_PROGRAM ccache)

if(CCACHE_PROGRAM)
    message(STATUS "Found ccache: ${CCACHE_PROGRAM} - Compilation cache enabled.")
    set(CMAKE_CXX_COMPILER_LAUNCHER "${CCACHE_PROGRAM}")
else()
    message(STATUS "ccache not found - Proceeding without cache.")
endif()

# ==========================================
# 1. 查找并引入第三方库
# ==========================================
# 1.1 必须的库
find_package(Eigen3 REQUIRED)
message(STATUS "Found Eigen3: ${Eigen3_VERSION}")

# 1.2 HPIPM + BLASFEO（QP 求解器依赖）
# 优先从环境变量/CMake 变量读取源码路径；未设置时回退到旧默认路径，
# 保持现有 CI/本地工作流兼容，同时允许他人在不修改 CMakeLists 的情况下覆盖。
if(NOT BLASFEO_SRC_DIR)
    set(BLASFEO_SRC_DIR "$ENV{BLASFEO_SRC_DIR}" CACHE PATH "BLASFEO source directory")
endif()
if(NOT HPIPM_SRC_DIR)
    set(HPIPM_SRC_DIR "$ENV{HPIPM_SRC_DIR}" CACHE PATH "HPIPM source directory")
endif()
if(NOT BLASFEO_SRC_DIR)
    set(BLASFEO_SRC_DIR "/home/els/download/blasfeo" CACHE PATH "BLASFEO source directory" FORCE)
endif()
if(NOT HPIPM_SRC_DIR)
    set(HPIPM_SRC_DIR "/home/els/download/hpipm" CACHE PATH "HPIPM source directory" FORCE)
endif()

if(EXISTS ${BLASFEO_SRC_DIR}/CMakeLists.txt AND EXISTS ${HPIPM_SRC_DIR}/CMakeLists.txt)
    message(STATUS "Found BLASFEO source: ${BLASFEO_SRC_DIR}")
    message(STATUS "Found HPIPM source: ${HPIPM_SRC_DIR}")

    # BLASFEO 配置：关闭测试/示例，使用 GENERIC 目标保证可移植性
    set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)
    set(BLASFEO_EXAMPLES OFF CACHE BOOL "" FORCE)
    set(BLASFEO_TESTING OFF CACHE BOOL "" FORCE)
    set(BLASFEO_BENCHMARKS OFF CACHE BOOL "" FORCE)
    set(TARGET GENERIC CACHE STRING "" FORCE)
    add_subdirectory(${BLASFEO_SRC_DIR} ${CMAKE_CURRENT_BINARY_DIR}/blasfeo_build)

    # HPIPM 配置：使用已存在的 blasfeo 目标，不启用测试
    set(TARGET GENERIC CACHE STRING "" FORCE)
    set(HPIPM_TESTING OFF CACHE BOOL "" FORCE)
    set(HPIPM_FIND_BLASFEO OFF CACHE BOOL "" FORCE)
    set(BLASFEO_PATH ${BLASFEO_SRC_DIR} CACHE STRING "" FORCE)
    add_subdirectory(${HPIPM_SRC_DIR} ${CMAKE_CURRENT_BINARY_DIR}/hpipm_build)

    set(HPIPM_AVAILABLE TRUE)
else()
    message(WARNING "HPIPM/BLASFEO source not found - HPIPM solver will be DISABLED.")
    set(HPIPM_AVAILABLE FALSE)
endif()

# 1.3 OpenMP（并行 linearize 依赖）
find_package(OpenMP)
if(OpenMP_CXX_FOUND)
    message(STATUS "Found OpenMP: ${OpenMP_CXX_VERSION} - Parallel linearize will be ENABLED.")
else()
    message(WARNING "OpenMP not found - Parallel linearize will be DISABLED.")
endif()

# 1.4 可选的库 (google test / benchmark)
find_package(GTest)
find_package(benchmark QUIET)

if(GTest_FOUND)
    message(STATUS "Found GTest: ${GTEST_VERSION} - Unit tests will be ENABLED.")
    enable_testing()
    include(GoogleTest)
else()
    message(WARNING "GTest not found - Unit tests will be DISABLED.")
endif()

if(benchmark_FOUND)
    message(STATUS "Found Google Benchmark - Benchmarks will be ENABLED.")
else()
    message(STATUS "Google Benchmark not found - Benchmarks will be DISABLED.")
endif()

# 引入 CasADi 代码生成封装模块（含 COMMON_DEPS 追踪）
include(cmake/CasADiCodegen.cmake)

# ==========================================
# 2. CasADi 代码生成（动力学）
# ==========================================
set(GENERATED_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src/generated)
set(GENERATED_SRCS
    ${GENERATED_DIR}/dynamics_kappa.c
    ${GENERATED_DIR}/dynamics_kappa_jac.c
    ${GENERATED_DIR}/dynamics_delta.c
    ${GENERATED_DIR}/dynamics_delta_jac.c
    ${GENERATED_DIR}/corridor.c
)

# 动力学代码生成：通过封装函数自动追踪 COMMON_DEPS
stc_sqp_add_casadi_codegen(
    generate_dynamics_code
    ${CMAKE_CURRENT_SOURCE_DIR}/autogen/generate_dynamics.py
    "${GENERATED_DIR}/dynamics_kappa.c;${GENERATED_DIR}/dynamics_kappa_jac.c;${GENERATED_DIR}/dynamics_delta.c;${GENERATED_DIR}/dynamics_delta_jac.c"
)

# 凸走廊约束代码生成：通过封装函数自动追踪 COMMON_DEPS
stc_sqp_add_casadi_codegen(
    generate_corridor_code
    ${CMAKE_CURRENT_SOURCE_DIR}/autogen/generate_corridor.py
    "${GENERATED_DIR}/corridor.c;${GENERATED_DIR}/corridor.h;${GENERATED_DIR}/vehicle_geometry.h"
)

# ==========================================
# 3. 源码打包与依赖解耦
# ==========================================
# 提取 src 下所有 cpp（core/math/models/util 等），但剔除 main.cpp，作为核心业务代码
file(GLOB_RECURSE CORE_SRCS CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/examples/parking/*.cpp"
)
list(REMOVE_ITEM CORE_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp)
list(APPEND CORE_SRCS ${GENERATED_SRCS})

# 没有 HPIPM 源码时，排除依赖 HPIPM/BLASFEO 头文件的源文件
if(NOT HPIPM_AVAILABLE)
    list(REMOVE_ITEM CORE_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/src/qp/hpipm_solver.cpp)
endif()

# 将核心业务代码编译为 Object 库
add_library(stc_SQP_core_lib OBJECT ${CORE_SRCS})
add_dependencies(stc_SQP_core_lib generate_dynamics_code generate_corridor_code)

# 为核心库配置头文件路径和第三方依赖
target_include_directories(stc_SQP_core_lib PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/examples/parking
)
target_link_libraries(stc_SQP_core_lib PUBLIC
    Eigen3::Eigen
)

# 强制 Eigen 按 32 字节对齐分配，保证 QPData 内存池起始地址满足 AVX2 SIMD 要求
target_compile_definitions(stc_SQP_core_lib PUBLIC EIGEN_MAX_ALIGN_BYTES=32)

# 链接 OpenMP；代码统一使用编译器内建 _OPENMP 判断，无需额外宏。
if(OpenMP_CXX_FOUND)
    target_link_libraries(stc_SQP_core_lib PUBLIC OpenMP::OpenMP_CXX)
endif()

if(HPIPM_AVAILABLE)
    target_link_libraries(stc_SQP_core_lib PUBLIC hpipm blasfeo)
endif()

# x86 平台启用 -march=native 以使用 AVX2/FMA 等指令，与 Eigen 32 字节对齐配合释放 SIMD 潜能
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|AMD64|amd64")
    target_compile_options(stc_SQP_core_lib PUBLIC -march=native)
    message(STATUS "Enabling -march=native for stc_SQP_core_lib x86_64 SIMD optimizations.")
endif()

# ==========================================
# 3. 生成最终可执行文件
# ==========================================
# 主程序：仅包含 main.cpp，并链接已编译好的核心库
add_executable(SQP_solver ${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp)
target_link_libraries(SQP_solver PRIVATE stc_SQP_core_lib)

# 单元测试：仅在找到 GTest 时生成
if(GTest_FOUND)
    # 打包 test 文件夹下所有的测试代码 (包含 main.t.cpp)
    file(GLOB_RECURSE TEST_SRCS CONFIGURE_DEPENDS "test/*.cpp" "test/*.t.cpp")

    # 没有 HPIPM 时排除依赖其接口的测试文件
    if(NOT HPIPM_AVAILABLE)
        list(REMOVE_ITEM TEST_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/test/test_qp_solvers.cpp)
    endif()

    add_executable(test_SQP_solver ${TEST_SRCS})

    # 链接核心库与测试框架
    target_link_libraries(test_SQP_solver PRIVATE stc_SQP_core_lib)

    if(TARGET GTest::gmock)
        target_link_libraries(test_SQP_solver PRIVATE GTest::gmock)
    elseif(TARGET GTest::gtest)
        target_link_libraries(test_SQP_solver PRIVATE GTest::gtest)
    else()
        target_link_libraries(test_SQP_solver PRIVATE gmock gtest pthread)
    endif()

    gtest_discover_tests(test_SQP_solver XML_OUTPUT_DIR ${CMAKE_CURRENT_BINARY_DIR}/Testing)
endif()

# Benchmark：仅在找到 Google Benchmark 时生成
if(benchmark_FOUND)
    # 自动包含 main.bench.cpp 以及 bench 文件夹下所有以 bench 开头的 cpp 文件
    file(GLOB BENCH_SRCS CONFIGURE_DEPENDS
        "bench/main.bench.cpp"
        "bench/bench_*.cpp"
    )

    if(BENCH_SRCS)
        # 没有 HPIPM 时排除依赖其接口的性能基准
        if(NOT HPIPM_AVAILABLE)
            list(REMOVE_ITEM BENCH_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/bench/bench_performance_profiling.cpp)
        endif()

        add_executable(bench_SQP_solver ${BENCH_SRCS})
        target_include_directories(bench_SQP_solver PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
        target_link_libraries(bench_SQP_solver PRIVATE stc_SQP_core_lib benchmark::benchmark)

    # 将 data3.json 复制到 benchmark 可执行文件目录，确保运行时能找到
    add_custom_command(TARGET bench_SQP_solver POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            ${CMAKE_CURRENT_SOURCE_DIR}/data/data3.json
            $<TARGET_FILE_DIR:bench_SQP_solver>/data/data3.json
        COMMENT "Copying data3.json for benchmark")
    endif()
endif()

# ==========================================
# 5. Ninja 动态并发池优化 (Ninja Pool Hack)
# ==========================================
include(ProcessorCount)
ProcessorCount(NPROC)

if(NPROC EQUAL 0)
    set(BUILD_JOBS 1)
else()
    if(NPROC GREATER 6)
        math(EXPR BUILD_JOBS "${NPROC} - 6")
    else()
        set(BUILD_JOBS 1)
    endif()
endif()

message(STATUS "=======================================================")
message(STATUS "Current Build Type       : ${CMAKE_BUILD_TYPE}")
message(STATUS "System CPU Cores (nproc) : ${NPROC}")
message(STATUS "Locked Parallel Jobs     : ${BUILD_JOBS}")
message(STATUS "=======================================================")

if(CMAKE_GENERATOR MATCHES "Ninja")
    message(STATUS "Detected Ninja Generator! Applying dynamic job pool...")
    set_property(GLOBAL PROPERTY JOB_POOLS dynamic_pool=${BUILD_JOBS})
    set(CMAKE_JOB_POOL_COMPILE dynamic_pool)
    set(CMAKE_JOB_POOL_LINK dynamic_pool)
endif()