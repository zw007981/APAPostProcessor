#include "qp_data.h"

#include <algorithm>
#include <stdexcept>

namespace stc_SQP {
QPData::QPData(int N, int nx, int nu, int ng_max)
    : N(N)
    , nx(nx)
    , nu(nu)
    , ng_max(ng_max) {
    if (N <= 0 || nx <= 0 || nu <= 0 || ng_max < 0) {
        throw std::invalid_argument(
            "QPData: N/nx/nu must be positive, ng_max must be non-negative");
    }
    // 总容量：每一块都按 AlignSize 向上取整到 4 个 double（32 字节）
    size_t total = 0;
    total += N * AlignSize(nx * nx); // A
    total += N * AlignSize(nx * nu); // B
    total += (N + 1) * AlignSize(nx * nx); // Q
    total += N * AlignSize(nu * nu); // R
    total += N * AlignSize(nu * nx); // S
    total += N * AlignSize(nx); // b
    total += (N + 1) * AlignSize(nx); // q
    total += N * AlignSize(nu); // r
    total += N * AlignSize(ng_max * nx); // C
    total += N * AlignSize(ng_max * nu); // D (general constraint w.r.t. u)
    total += N * AlignSize(ng_max); // d
    total += 2 * (N + 1) * AlignSize(nx); // lbx, ubx
    total += 2 * N * AlignSize(nu); // lbu, ubu
    memory_pool_.resize(total, 0.0);
    // 映射视图：Eigen::Map 没有默认构造函数，因此先 reserve 精确容量，
    // 再通过 emplace_back 原地构造每个 Map。由于容量已预留，不会发生重分配，
    // 避免触发 Map 的拷贝/移动。每次偏移推进必须调用 AlignSize，保证每个 Map
    // 起始地址 32 字节对齐。
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
    D.reserve(N);
    for (int k = 0; k < N; ++k) {
        D.emplace_back(&memory_pool_[offset], ng_max, nu);
        offset += AlignSize(ng_max * nu);
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
    // 构造期不变量：若 total 与 offset 不一致，说明内存池布局计算存在错误，必须立即失败
    if (offset != total) {
        throw std::logic_error("QPData: memory pool allocation mismatch");
    }
}

void QPData::reset() {
    std::fill(memory_pool_.begin(), memory_pool_.end(), 0.0);
}

bool QPData::hasValidContainerSizes() const {
    const bool n_stage_ok =
        static_cast<int>(A.size()) == N &&
        static_cast<int>(B.size()) == N &&
        static_cast<int>(b.size()) == N &&
        static_cast<int>(R.size()) == N &&
        static_cast<int>(S.size()) == N &&
        static_cast<int>(r.size()) == N &&
        static_cast<int>(lbu.size()) == N &&
        static_cast<int>(ubu.size()) == N &&
        static_cast<int>(C.size()) == N &&
        static_cast<int>(D.size()) == N &&
        static_cast<int>(d.size()) == N;
    const bool np1_stage_ok =
        static_cast<int>(Q.size()) == N + 1 &&
        static_cast<int>(q.size()) == N + 1 &&
        static_cast<int>(lbx.size()) == N + 1 &&
        static_cast<int>(ubx.size()) == N + 1;
    return n_stage_ok && np1_stage_ok;
}
} // namespace stc_SQP
