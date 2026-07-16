#include "casadi_wrapper.h"

namespace stc_SQP {
namespace casadi {
    CasADiFunction::CasADiFunction(FunctionPointer func, WorkSizeFunction work_func, int n_in,
        int n_out)
        : func_(func)
        , work_func_(work_func)
        , n_in_(n_in)
        , n_out_(n_out)
    {
        if (func_ == nullptr || work_func_ == nullptr) {
            throw std::invalid_argument("CasADiFunction: function pointer cannot be null");
        }
        queryWorkSizes();
    }

    CasADiFunction::CasADiFunction(CasADiFunction&&) noexcept = default;

    CasADiFunction& CasADiFunction::operator=(CasADiFunction&&) noexcept = default;

    CasADiFunction::CasADiFunction(const CasADiFunction& other)
        : func_(other.func_)
        , work_func_(other.work_func_)
        , n_in_(other.n_in_)
        , n_out_(other.n_out_)
    {
        queryWorkSizes();
    }

    CasADiFunction& CasADiFunction::operator=(const CasADiFunction& other)
    {
        if (this != &other) {
            func_ = other.func_;
            work_func_ = other.work_func_;
            n_in_ = other.n_in_;
            n_out_ = other.n_out_;
            queryWorkSizes();
        }
        return *this;
    }

    void CasADiFunction::queryWorkSizes()
    {
        long long sz_arg = 0;
        long long sz_res = 0;
        long long sz_iw = 0;
        long long sz_w = 0;
        const int ret = work_func_(&sz_arg, &sz_res, &sz_iw, &sz_w);
        if (ret != 0) {
            throw std::runtime_error("CasADiFunction: failed to query work sizes");
        }
        iw_.resize(static_cast<size_t>(sz_iw), 0);
        w_.resize(static_cast<size_t>(sz_w), 0.0);
    }

    void CasADiFunction::operator()(const std::vector<const double*>& arg,
        std::vector<double*>& res) const
    {
        if (static_cast<int>(arg.size()) != n_in_) {
            throw std::invalid_argument("CasADiFunction: number of input arguments mismatch");
        }
        if (static_cast<int>(res.size()) != n_out_) {
            throw std::invalid_argument("CasADiFunction: number of output arguments mismatch");
        }
        // 构造局部非 const 副本，使 data() 类型与 CasADi C API 完全匹配，
        // 避免对调用方容器进行 const_cast。
        std::vector<const double*> arg_buffer = arg;
        const int ret = func_(arg_buffer.data(), res.data(), iw_.data(), w_.data(), 0);
        if (ret != 0) {
            throw std::runtime_error("CasADiFunction: function execution returned error code");
        }
    }

    CasADiFunction CasADiFunction::clone() const
    {
        CasADiFunction copy(func_, work_func_, n_in_, n_out_);
        return copy;
    }
} // namespace casadi
} // namespace stc_SQP
