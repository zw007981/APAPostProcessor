#pragma once

#include <stdexcept>
#include <vector>

#include "../core/types.h"

namespace stc_SQP {
namespace casadi {
    // CasADi 生成函数指针类型（casadi_int 在生成代码中等价于 long long）
    using FunctionPointer = int (*)(const double**, double**, long long*, double*, int);
    using WorkSizeFunction = int (*)(long long*, long long*, long long*, long long*);

    // CasADi 生成 C 代码的轻量包装器。
    // 职责：
    //   1. 在构造时查询并预分配 iw/w 工作区，避免每次调用时动态分配。
    //   2. 提供与生成函数签名一致的 operator() 调用接口。
    //   3. 通过 clone() 创建拥有独立工作区的副本，供多线程并发安全使用。
    // 注意：arg/res 中的指针由调用方拥有生命周期，本类不管理输入输出数据。
    class CasADiFunction {
    public:
        // 构造包装器。
        // func       : 生成函数入口（如 dynamics_kappa）
        // work_func  : 对应 _work 函数，用于查询 iw/w 大小
        // n_in/n_out : 输入/输出参数个数（arg/res 的期望长度）
        CasADiFunction(FunctionPointer func, WorkSizeFunction work_func, int n_in, int n_out);
        // 默认析构；iw_/w_ 由 vector 自动释放，无需额外清理。
        ~CasADiFunction() = default;
        // 拷贝构造：func_/work_func_ 可共享，iw_/w_ 重新分配独立缓冲区，
        // 因此副本与原实例可并发调用（多线程场景仍建议通过独立副本访问）。
        CasADiFunction(const CasADiFunction&);
        CasADiFunction& operator=(const CasADiFunction&);
        // 移动构造：转移 iw_/w_ 所有权，避免重新查询工作区大小。
        CasADiFunction(CasADiFunction&&) noexcept;
        // 移动赋值：转移 iw_/w_ 所有权。
        CasADiFunction& operator=(CasADiFunction&&) noexcept;
        // 调用生成函数，该函数不是线程安全的，多线程并发必须通过 clone() 持有独立实例
        void operator()(const std::vector<const double*>& arg, std::vector<double*>& res) const;
        // 创建独立工作区的副本。func_ 与 work_func_ 本身无状态可共享；
        // iw_/w_ 重新分配，保证两个实例并发调用时互不干扰。
        CasADiFunction clone() const;
        // 输入参数个数（arg 的期望长度）
        int n_in() const { return n_in_; }
        // 输出参数个数（res 的期望长度）
        int n_out() const { return n_out_; }
        // 整数工作区 iw 的当前大小，仅用于调试或测试验证。
        int iw_size() const { return static_cast<int>(iw_.size()); }
        // 浮点工作区 w 的当前大小，仅用于调试或测试验证。
        int w_size() const { return static_cast<int>(w_.size()); }

    protected:
        // 查询 _work 函数并预分配 iw_/w_ 缓冲区。
        void queryWorkSizes();
        // CasADi 生成函数入口指针
        FunctionPointer func_ = nullptr;
        // 对应 _work 函数，用于查询工作区大小
        WorkSizeFunction work_func_ = nullptr;
        // 输入参数个数
        int n_in_ = 0;
        // 输出参数个数
        int n_out_ = 0;
        // CasADi 整数工作区（integer workspace），调用时会被覆盖，故声明为 mutable
        mutable std::vector<long long> iw_;
        // CasADi 浮点工作区（real workspace），调用时会被覆盖，故声明为 mutable
        mutable std::vector<double> w_;
    };
} // namespace casadi
} // namespace stc_SQP
