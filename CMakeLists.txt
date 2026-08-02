cmake_minimum_required(VERSION 3.15)
project(APAPostProcessor VERSION 1.0.0 LANGUAGES CXX)

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

# 将 __FILE__ 映射为工程内相对路径
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    # 1. 应对绝对路径的情况 (Make 等生成器常用)
    add_compile_options("$<$<COMPILE_LANGUAGE:CXX>:-fmacro-prefix-map=${CMAKE_CURRENT_SOURCE_DIR}/=>")
    add_compile_options("$<$<COMPILE_LANGUAGE:CXX>:-fmacro-prefix-map=../../=>")

    # 2. 应对相对路径的情况 (Ninja 等生成器常用)
    file(RELATIVE_PATH REL_SRC_DIR ${CMAKE_CURRENT_BINARY_DIR} ${CMAKE_CURRENT_SOURCE_DIR})

    if(REL_SRC_DIR)
        # 统一去掉可能的尾部斜杠，避免拼接后出现 ..// 这种无效前缀
        string(REGEX REPLACE "/+$" "" REL_SRC_DIR "${REL_SRC_DIR}")
        add_compile_options("$<$<COMPILE_LANGUAGE:CXX>:-fmacro-prefix-map=${REL_SRC_DIR}/=>")
    endif()
endif()

# ==========================================
# 1. 查找并引入第三方库
# ==========================================
# 1.1 必须的库
# 优先 CONFIG 模式（源码安装的新版 protobuf 自带 protobuf-config.cmake），
# 找不到时回退 CMake 内置 Module 模式（适配 apt 安装的 libprotobuf-dev，无 config 文件）
find_package(Protobuf CONFIG QUIET)
if(NOT Protobuf_FOUND)
    find_package(Protobuf MODULE REQUIRED)
endif()
message(STATUS "Found Protobuf: ${Protobuf_VERSION}")

find_package(OpenCV REQUIRED)
message(STATUS "Found OpenCV: ${OpenCV_VERSION}")

find_package(Eigen3 REQUIRED)
message(STATUS "Found Eigen3: ${Eigen3_VERSION}")

find_package(nlohmann_json REQUIRED)
message(STATUS "Found nlohmann_json: ${nlohmann_json_VERSION}")

find_package(OpenMP REQUIRED)
message(STATUS "Found OpenMP: ${OpenMP_CXX_VERSION}")

set(QUILL_LOCAL_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third_party/quill)

if(NOT EXISTS ${QUILL_LOCAL_DIR}/CMakeLists.txt)
    message(FATAL_ERROR "Quill not found at ${QUILL_LOCAL_DIR}. Please download quill source first.")
endif()

add_subdirectory(${QUILL_LOCAL_DIR})
message(STATUS "Included local Quill logging library.")

# 1.3 本地 SQP 数值优化框架（NMPC 求解器依赖）
set(STC_SQP_LOCAL_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third_party/StcSQP)

if(NOT EXISTS ${STC_SQP_LOCAL_DIR}/CMakeLists.txt)
    message(FATAL_ERROR "StcSQP not found at ${STC_SQP_LOCAL_DIR}. Please check third_party submodule.")
endif()

add_subdirectory(${STC_SQP_LOCAL_DIR})
message(STATUS "Included local StcSQP optimization framework.")

# 1.4 本地 OSQP 二次规划求解器
set(OSQP_LOCAL_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third_party/osqp)

if(NOT EXISTS ${OSQP_LOCAL_DIR}/CMakeLists.txt)
    message(FATAL_ERROR "OSQP not found at ${OSQP_LOCAL_DIR}. Please clone osqp source first.")
endif()

# 关闭 OSQP 自带的单元测试、打印、性能计数与 Ctrl-C 中断，避免引入无关依赖与终端副作用
set(UNITTESTS OFF CACHE BOOL "" FORCE)
set(PRINTING OFF CACHE BOOL "" FORCE)
set(PROFILING OFF CACHE BOOL "" FORCE)
set(CTRLC OFF CACHE BOOL "" FORCE)
set(ENABLE_MKL_PARDISO OFF CACHE BOOL "" FORCE)

add_subdirectory(${OSQP_LOCAL_DIR})
message(STATUS "Included local OSQP QP solver.")

# 1.5 本地 LBFGSpp 无约束/箱式约束优化库（header-only，仅依赖 Eigen3）
set(LBFGSPP_LOCAL_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third_party/LBFGSpp)

if(NOT EXISTS ${LBFGSPP_LOCAL_DIR}/CMakeLists.txt)
    message(FATAL_ERROR "LBFGSpp not found at ${LBFGSPP_LOCAL_DIR}. Please clone LBFGSpp source first.")
endif()

add_subdirectory(${LBFGSPP_LOCAL_DIR})
message(STATUS "Included local LBFGSpp optimization library (header-only).")

# 1.2 可选的库 (google test / benchmark)
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

# ==========================================
# 2. Proto 协议代码生成与打包
# ==========================================
set(PROTO_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/proto/apa_post_process.proto
    ${CMAKE_CURRENT_SOURCE_DIR}/proto/nmpc_config.proto
)

add_library(apa_proto_lib STATIC)
protobuf_generate(TARGET apa_proto_lib PROTOS ${PROTO_FILES})
target_link_libraries(apa_proto_lib PUBLIC protobuf::libprotobuf)
target_include_directories(apa_proto_lib PUBLIC
    ${CMAKE_CURRENT_BINARY_DIR}
    ${CMAKE_CURRENT_BINARY_DIR}/proto
)

# ==========================================
# 3. 源码打包与依赖解耦
# ==========================================
# 提取 src 下所有 cpp，但剔除 main.cpp，作为核心业务代码
file(GLOB_RECURSE CORE_SRCS CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")
list(REMOVE_ITEM CORE_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp)

# 将核心业务代码编译为 Object 库
add_library(apa_core_lib OBJECT ${CORE_SRCS})

# 为核心库配置头文件路径和第三方依赖
target_include_directories(apa_core_lib PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(apa_core_lib PUBLIC
    apa_proto_lib
    quill::quill
    stc_SQP_core_lib
    osqpstatic
    lbfgspp
    ${OpenCV_LIBS}
    Eigen3::Eigen
    nlohmann_json::nlohmann_json
    OpenMP::OpenMP_CXX
)

# 项目根目录宏：供代码中定位 fig/、log/ 等资源目录
target_compile_definitions(apa_core_lib PUBLIC
    PROJECT_ROOT_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
)

# x86 平台启用 -march=native 以使用 AVX2/FMA 等指令，释放 SIMD 潜能
# 如需部署到不同 CPU，可关闭此选项或指定目标架构
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|AMD64|amd64")
    target_compile_options(apa_core_lib PUBLIC -march=native)
    message(STATUS "Enabling -march=native for x86_64 SIMD optimizations.")
endif()

# DDP 完整二阶动力学项（true MS-DDP）实验开关：默认 OFF（Gauss-Newton/
# iLQR，丢弃 s'·f_** 张量红项）。开启后内层回推补回完整二阶收缩，设计
# 预期是换取局部二次收敛（Li 2023 Fig.3 实测结论）；副作用是 Q_uu 更
# 频繁失去正定性、ρ_reg 触发率上升。必须 PUBLIC 传播：开关改变
# bicycle_dynamics.h 的 constexpr 状态，所有包含该头的编译单元必须一致
option(APA_DDP_FULL_HESSIAN "Enable full second-order dynamics terms (true MS-DDP) in the DDP backward pass" OFF)
if(APA_DDP_FULL_HESSIAN)
    target_compile_definitions(apa_core_lib PUBLIC DDP_ENABLE_FULL_HESSIAN)
    message(STATUS "DDP full Hessian (true MS-DDP) backward pass: ENABLED")
endif()

# ==========================================
# 4. 生成最终可执行文件
# ==========================================
# 主程序：仅包含 main.cpp，并链接已编译好的核心库
# 注：stc_SQP_core_lib 同为 OBJECT 库，OBJECT 库之间的链接不会向下一级可执行文件传递目标文件，
# 因此需要在最终可执行文件上显式再次链接，避免出现未定义引用。
add_executable(apa_post_processor ${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp)
target_link_libraries(apa_post_processor PRIVATE apa_core_lib stc_SQP_core_lib)

# 单元测试：仅在找到 GTest 时生成
if(GTest_FOUND)
    # 打包 test 文件夹下所有的测试代码 (包含 main.t.cpp)
    file(GLOB_RECURSE TEST_SRCS CONFIGURE_DEPENDS "test/*.cpp" "test/*.t.cpp")

    add_executable(test_apa_post_processor ${TEST_SRCS})

    # 链接核心库与测试框架；stc_SQP_core_lib 需显式再次链接，理由同主程序
    target_link_libraries(test_apa_post_processor PRIVATE apa_core_lib stc_SQP_core_lib)

    if(TARGET GTest::gmock)
        target_link_libraries(test_apa_post_processor PRIVATE GTest::gmock)
    elseif(TARGET GTest::gtest)
        target_link_libraries(test_apa_post_processor PRIVATE GTest::gtest)
    else()
        target_link_libraries(test_apa_post_processor PRIVATE gmock gtest pthread)
    endif()

    gtest_discover_tests(test_apa_post_processor XML_OUTPUT_DIR ${CMAKE_CURRENT_BINARY_DIR}/Testing)
endif()

# Benchmark：仅在找到 Google Benchmark 时生成
if(benchmark_FOUND)
    # 自动包含 main.bench.cpp 以及 bench 文件夹下所有以 bench 开头的 cpp 文件
    file(GLOB BENCH_SRCS CONFIGURE_DEPENDS
        "bench/main.bench.cpp"
        "bench/bench_*.cpp"
    )

    if(BENCH_SRCS)
        add_executable(bench_apa_post_processor ${BENCH_SRCS})
        target_include_directories(bench_apa_post_processor PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
        # stc_SQP_core_lib 需显式再次链接，理由同主程序
        target_link_libraries(bench_apa_post_processor PRIVATE apa_core_lib stc_SQP_core_lib benchmark::benchmark)
    endif()
endif()

# ==========================================
# 4.5 调参工具（开发期手动运行，不参与测试/压测）
# ==========================================
# DDP 四数据集调参驱动：逐变体 × 逐数据集执行 PostProcessor::optimizeDdp 并
# 输出机器可读评价行，供调参标定与回归判断；stc_SQP_core_lib 需显式再次
# 链接，理由同主程序
add_executable(apa_tune_ddp ${CMAKE_CURRENT_SOURCE_DIR}/tool/tune_ddp.cpp)
target_link_libraries(apa_tune_ddp PRIVATE apa_core_lib stc_SQP_core_lib)

# ALM 四数据集调参驱动：逐变体 × 逐数据集执行 PostProcessor::optimizeAlm 并
# 输出机器可读评价行；L8 ESDF 边界语义修复跨 ALM/DDP 双路径生效，ALM 回归
# 依赖本目标（此前缺失，补齐）
add_executable(apa_tune_alm ${CMAKE_CURRENT_SOURCE_DIR}/tool/tune_alm.cpp)
target_link_libraries(apa_tune_alm PRIVATE apa_core_lib stc_SQP_core_lib)

# DDP 后处理链路四断点审计驱动：复刻 optimizeDdp 装配并逐步手动编排，对
# 阶段一解/修剪环节/阶段二解/驻留与最终输出四个断点转储全量结构化量测
# （12 项校验判据不短路），供口径审计取证；stc_SQP_core_lib 需显式再次
# 链接，理由同主程序
add_executable(apa_audit_ddp_post ${CMAKE_CURRENT_SOURCE_DIR}/tool/audit_ddp_post.cpp)
target_link_libraries(apa_audit_ddp_post PRIVATE apa_core_lib stc_SQP_core_lib)

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