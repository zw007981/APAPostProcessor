# cmake/CasADiCodegen.cmake
# 封装 CasADi Python 代码生成脚本为 CMake 自定义命令。
# 关键约定：
#   - COMMON_DEPS 为所有生成脚本共享的 autogen/common.py；修改它必须触发全部重新生成。
#   - 自定义命令的工作目录为项目根目录，保证 Python 脚本的相对路径一致。

# 所有生成脚本共享的依赖：common.py 中的车辆参数 / p 维度分配会影响生成代码的 ABI。
set(STC_SQP_COMMON_DEPS ${CMAKE_CURRENT_SOURCE_DIR}/autogen/common.py)

# stc_sqp_add_casadi_codegen
# 封装一个 CasADi 代码生成脚本。
#
# 参数：
#   name          : 自定义目标名称（如 generate_dynamics_code）
#   script        : Python 生成脚本路径（如 ${CMAKE_CURRENT_SOURCE_DIR}/autogen/generate_dynamics.py）
#   output_files  : 生成产物文件列表（绝对路径）
#
# 行为：
#   - 自动将 COMMON_DEPS（autogen/common.py）追加到 DEPENDS。
#   - 自动将 script 本身追加到 DEPENDS。
#   - 创建同名 add_custom_target，可被核心库依赖。
function(stc_sqp_add_casadi_codegen name script output_files)
    # 校验输出文件列表非空
    list(LENGTH output_files output_count)
    if(output_count EQUAL 0)
        message(FATAL_ERROR "stc_sqp_add_casadi_codegen: output_files cannot be empty")
    endif()

    # 聚合依赖：脚本自身 + 共享 common.py
    set(command_depends ${script} ${STC_SQP_COMMON_DEPS})

    add_custom_command(
        OUTPUT ${output_files}
        COMMAND python3 ${script}
        DEPENDS ${command_depends}
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        COMMENT "Generating CasADi C code via ${script}"
    )

    add_custom_target(${name} DEPENDS ${output_files})
endfunction()
