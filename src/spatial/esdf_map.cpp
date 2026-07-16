#include "esdf_map.h"

#include <vector>

namespace apa_post_processor {
ESDFMap::ESDFMap(const GridMap& grid_map) {
    if (grid_map.getResolution() <= EPSILON) {
        LOG_FMT_ERROR("Invalid ESDFMap resolution: {}. Must be positive!!!",
                      grid_map.getResolution());
        throw std::invalid_argument("ESDFMap resolution must be positive!!!");
    }
    resolution_ = grid_map.getResolution();
    inv_resolution_ = 1.0 / resolution_;
    width_ = grid_map.getWidth();
    height_ = grid_map.getHeight();
    origin_ = grid_map.getOrigin();
    size_ = width_ * height_;
    if (width_ <= 0 || height_ <= 0) {
        LOG_FMT_ERROR(
            "Invalid ESDFMap width/height: {}/{}. Must be positive!!!", width_,
            height_);
        throw std::invalid_argument("ESDFMap width/height must be positive!!!");
    }
    const auto& occ_data = grid_map.getOccupancyData();
    if (occ_data.size() != size_) {
        LOG_FMT_ERROR("Invalid ESDFMap occupancy data size: {}, expected {}!!!",
                      occ_data.size(), size_);
        throw std::logic_error("ESDFMap occupancy data size is invalid!!!");
    }
    const auto signed_distance =
        BuildSignedDistData(occ_data, width_, height_, resolution_);
    cell_data_.resize(size_);
    distance_data_.resize(size_);
    for (GridIndex i = 0; i < size_; ++i) {
        cell_data_[i].dist = signed_distance[i];
        distance_data_[i] = signed_distance[i];
    }
    calGradField();
}

std::pair<double, Eigen::Vector2d> ESDFMap::getDistAndGrad(double x,
                                                           double y) const {
    if (!this->inMap(x, y)) {
        LOG_FMT_WARN("ESDFMap query out of bounds: ({}, {})!", x, y);
        return {0.0, Eigen::Vector2d::Zero()};
    }
    // 双线性插值：从 AoS 缓存取四个角点，用 float 做 lerp，返回前转 double
    const auto params = this->calBilinearParams(x, y);
    const auto& cell_bl = cell_data_[params.idx_bl];
    const auto& cell_br = cell_data_[params.idx_br];
    const auto& cell_tl = cell_data_[params.idx_tl];
    const auto& cell_tr = cell_data_[params.idx_tr];
    const auto col_ratio = params.col_ratio;
    const auto row_ratio = params.row_ratio;
    const auto dist_row_lower =
        cell_bl.dist + (cell_br.dist - cell_bl.dist) * col_ratio;
    const auto dist_row_upper =
        cell_tl.dist + (cell_tr.dist - cell_tl.dist) * col_ratio;
    const auto dist =
        dist_row_lower + (dist_row_upper - dist_row_lower) * row_ratio;
    const auto grad_x_row_lower =
        cell_bl.grad_x + (cell_br.grad_x - cell_bl.grad_x) * col_ratio;
    const auto grad_x_row_upper =
        cell_tl.grad_x + (cell_tr.grad_x - cell_tl.grad_x) * col_ratio;
    const auto grad_x =
        grad_x_row_lower + (grad_x_row_upper - grad_x_row_lower) * row_ratio;
    const auto grad_y_row_lower =
        cell_bl.grad_y + (cell_br.grad_y - cell_bl.grad_y) * col_ratio;
    const auto grad_y_row_upper =
        cell_tl.grad_y + (cell_tr.grad_y - cell_tl.grad_y) * col_ratio;
    const auto grad_y =
        grad_y_row_lower + (grad_y_row_upper - grad_y_row_lower) * row_ratio;
    return {static_cast<double>(dist),
            Eigen::Vector2d(static_cast<double>(grad_x),
                            static_cast<double>(grad_y))};
}

void ESDFMap::getDistAndGradBatch(const double* xs, const double* ys, int n,
                                  double* dists_out, double* grad_x_out,
                                  double* grad_y_out) const {
    // 参数结构体：每个查询存储 4 个 int 索引 + 2 个 float 权重
    struct alignas(32) BatchParam {
        int idx_bl, idx_br, idx_tl, idx_tr;
        float col_ratio, row_ratio;
        int valid;
    };
    // 栈上限 12（对应最多 12 个外圆），超出时回退到 std::vector
    constexpr int kStackMax = 12;
    BatchParam stack_buf[kStackMax];
    std::vector<BatchParam> heap_buf;
    BatchParam* params = stack_buf;
    if (n > kStackMax) {
        heap_buf.reserve(static_cast<std::size_t>(n));
        heap_buf.resize(static_cast<std::size_t>(n));
        params = heap_buf.data();
    }

    // 第一遍：计算所有点的双线性参数并检查越界
    const double res = 1.0 / inv_resolution_;  // 恢复 resolution_（物理米/格）
    const double ox = origin_.x;
    const double oy = origin_.y;
    const int w = width_;
    const int h = height_;
    const double map_x_max = ox + static_cast<double>(w) * res;
    const double map_y_max = oy + static_cast<double>(h) * res;
    for (int i = 0; i < n; ++i) {
        const double x = xs[i];
        const double y = ys[i];
        if (x >= ox && x < map_x_max && y >= oy && y < map_y_max) {
            const auto bp = calBilinearParams(x, y);
            params[i].idx_bl = bp.idx_bl;
            params[i].idx_br = bp.idx_br;
            params[i].idx_tl = bp.idx_tl;
            params[i].idx_tr = bp.idx_tr;
            params[i].col_ratio = bp.col_ratio;
            params[i].row_ratio = bp.row_ratio;
            params[i].valid = 1;
        } else {
            params[i].valid = 0;
        }
    }

    // 第二遍：对所有有效点做双线性插值（纯浮点，编译器可自动向量化）
    const EsdfCell* cell_base = cell_data_.data();
    for (int i = 0; i < n; ++i) {
        if (!params[i].valid) {
            dists_out[i] = 0.0;
            grad_x_out[i] = 0.0;
            grad_y_out[i] = 0.0;
            continue;
        }
        const auto& p = params[i];
        const auto& cell_bl = cell_base[p.idx_bl];
        const auto& cell_br = cell_base[p.idx_br];
        const auto& cell_tl = cell_base[p.idx_tl];
        const auto& cell_tr = cell_base[p.idx_tr];
        const float cr = p.col_ratio;
        const float rr = p.row_ratio;
        const float dist_lo = cell_bl.dist + (cell_br.dist - cell_bl.dist) * cr;
        const float dist_hi = cell_tl.dist + (cell_tr.dist - cell_tl.dist) * cr;
        const float gx_lo =
            cell_bl.grad_x + (cell_br.grad_x - cell_bl.grad_x) * cr;
        const float gx_hi =
            cell_tl.grad_x + (cell_tr.grad_x - cell_tl.grad_x) * cr;
        const float gy_lo =
            cell_bl.grad_y + (cell_br.grad_y - cell_bl.grad_y) * cr;
        const float gy_hi =
            cell_tl.grad_y + (cell_tr.grad_y - cell_tl.grad_y) * cr;
        dists_out[i] = static_cast<double>(dist_lo + (dist_hi - dist_lo) * rr);
        grad_x_out[i] = static_cast<double>(gx_lo + (gx_hi - gx_lo) * rr);
        grad_y_out[i] = static_cast<double>(gy_lo + (gy_hi - gy_lo) * rr);
    }
}

double ESDFMap::getDist(double x, double y) const {
    if (!this->inMap(x, y)) {
        LOG_FMT_WARN("ESDFMap query out of bounds: ({}, {})!", x, y);
        return 0.0;
    }
    // 仅对距离做双线性插值。使用独立 distance_data_ 保持 4-byte stride，
    // 避免 AoS 布局对纯距离查询的 cache 密度损失。
    const auto params = this->calBilinearParams(x, y);
    const auto col_ratio = params.col_ratio;
    const auto row_ratio = params.row_ratio;
    const PhysicalDist* dist_data = distance_data_.data();
    const auto dist_row_lower =
        dist_data[params.idx_bl] +
        (dist_data[params.idx_br] - dist_data[params.idx_bl]) * col_ratio;
    const auto dist_row_upper =
        dist_data[params.idx_tl] +
        (dist_data[params.idx_tr] - dist_data[params.idx_tl]) * col_ratio;
    const auto dist =
        dist_row_lower + (dist_row_upper - dist_row_lower) * row_ratio;
    return static_cast<double>(dist);
}

// 调用方必须保证 (x, y) 在地图范围内。static_cast<int> 对正数截断等价于 floor。
ESDFMap::BilinearParams ESDFMap::calBilinearParams(double x, double y) const {
    auto params = BilinearParams();
    const auto col_float =
        static_cast<float>((x - origin_.x) * inv_resolution_);
    const auto row_float =
        static_cast<float>((y - origin_.y) * inv_resolution_);
    params.col_lower = std::clamp(static_cast<int>(col_float), 0, width_ - 1);
    params.row_lower = std::clamp(static_cast<int>(row_float), 0, height_ - 1);
    params.col_upper = std::min(params.col_lower + 1, width_ - 1);
    params.row_upper = std::min(params.row_lower + 1, height_ - 1);
    // 预计算四个角点的一维索引
    const auto lower_offset = params.row_lower * width_;
    const auto upper_offset = params.row_upper * width_;
    params.idx_bl = lower_offset + params.col_lower;
    params.idx_br = lower_offset + params.col_upper;
    params.idx_tl = upper_offset + params.col_lower;
    params.idx_tr = upper_offset + params.col_upper;
    // 计算双线性插值的权重系数
    params.col_ratio = (params.col_lower == params.col_upper)
                           ? 0.0f
                           : (col_float - static_cast<float>(params.col_lower));
    params.row_ratio = (params.row_lower == params.row_upper)
                           ? 0.0f
                           : (row_float - static_cast<float>(params.row_lower));
    return params;
}

void ESDFMap::calGradField() {
    // 退化尺寸下梯度无意义，直接清零。
    if (width_ <= 1 || height_ <= 1) {
        for (auto& cell : cell_data_) {
            cell.grad_x = 0.0f;
            cell.grad_y = 0.0f;
        }
        return;
    }
    // 基于有限差分法计算梯度，边界使用单侧差分，内部使用中心差分
    // 边界 grad_x approx (DistField(x+1,y) - DistField(x,y)) / res
    // 内部 grad_x approx (DistField(x+1,y) - DistField(x-1,y)) / (2 * res)
    const auto INV_RES = static_cast<PhysicalDist>(inv_resolution_),
               HALF_INV_RES = INV_RES * 0.5f;
    // AoS 缓存基准地址，跨步读取 dist、回写 grad_x/grad_y。
    EsdfCell* cell_base = cell_data_.data();
#pragma omp parallel for schedule(dynamic, 8)
    for (int row = 0; row < height_; ++row) {
        const auto ROW_INV_RES =
            (row == 0 || row == height_ - 1) ? INV_RES : HALF_INV_RES;
        // 从 AoS 缓存中提取出当前行及上下相邻行的首地址
        const EsdfCell *cell_row_cur = cell_base + row * width_,
                       *cell_row_prev =
                           cell_base + std::max(0, row - 1) * width_,
                       *cell_row_next =
                           cell_base + std::min(row + 1, height_ - 1) * width_;
        // 需要求解的梯度行地址
        EsdfCell* cell_row_out = cell_base + row * width_;
        // 左边界
        int col = 0;
        cell_row_out[col].grad_x =
            (cell_row_cur[col + 1].dist - cell_row_cur[col].dist) * INV_RES;
        cell_row_out[col].grad_y =
            (cell_row_next[col].dist - cell_row_prev[col].dist) * ROW_INV_RES;
        // 内部区域
        for (col = 1; col < width_ - 1; col++) {
            cell_row_out[col].grad_x =
                (cell_row_cur[col + 1].dist - cell_row_cur[col - 1].dist) *
                HALF_INV_RES;
            cell_row_out[col].grad_y =
                (cell_row_next[col].dist - cell_row_prev[col].dist) *
                ROW_INV_RES;
        }
        // 右边界
        col = width_ - 1;
        cell_row_out[col].grad_x =
            (cell_row_cur[col].dist - cell_row_cur[col - 1].dist) * INV_RES;
        cell_row_out[col].grad_y =
            (cell_row_next[col].dist - cell_row_prev[col].dist) * ROW_INV_RES;
    }
}

namespace {
// 剥离业务处理纯粹的数组数学运算（SIMD 与 CPUID 动态分发）。
struct SimdMath {
    // 逐元素计算：out[i] = (sqrt(a[i]) - sqrt(b[i])) * scale
    static void ElementWiseSqrtSubMul(const SquaredDist* a,
                                      const SquaredDist* b, PhysicalDist scale,
                                      PhysicalDist* out, int size);
};

// 处理纯粹的二维/一维网格距离变换。
struct DistanceTransform {
    // 分块转置的块大小。16x16 个 int32_t 约 1KB，可较好地驻留在 L1 Cache 中，
    // 在减少 TLB Miss 与 Cache Miss 的同时避免块过小导致调度开销过大。
    static constexpr int BLOCK_SIZE = 16;

    // 计算二维欧氏距离平方场
    static std::unique_ptr<SquaredDist[]> BuildSquaredDistField(
        const std::vector<uint8_t>& occ_data, int width, int height,
        bool target_is_occupied);
    // 沿行方向批量执行 1D 距离变换，调用方负责提供 parabola_indices /
    // breakpoints 缓存，并决定是否并行。
    static void RunDistanceTransformByRows(const SquaredDist* input,
                                           int row_count, int col_count,
                                           SquaredDist* output,
                                           int* parabola_indices,
                                           std::int64_t* breakpoints);
    // 完全整数化的 Felzenszwalb & Huttenlocher 一维平方距离变换
    static void RunDistanceTransform1D(const SquaredDist* input,
                                       SquaredDist* output, int size,
                                       int* parabola_indices,
                                       std::int64_t* breakpoints);
    // 转置 row-major 矩阵，让原列扫描转化为连续行扫描以减少 cache miss
    static void Transpose(const SquaredDist* input, SquaredDist* output,
                          int row_count, int col_count);
};

// =========================================================================
// SimdMath：逐元素 sqrt/sub/mul 的 SIMD 与 Fallback 实现
// =========================================================================

// AVX2 实现必须作为独立函数存在，因为 __attribute__((target("avx2,fma")))
// 只能作用于函数级。如果在 SimdMath::ElementWiseSqrtSubMul 内部直接写 AVX2
// 内联汇编并给整个函数加 target 属性，则在不支持 AVX2 的 CPU 上连函数入口的
// prologue 都可能触发非法指令异常。因此保留一个极小的 AVX2 helper，由无 target
// 属性的 ElementWiseSqrtSubMul 在 CPUID 检测后调用。
#if defined(__GNUC__) || defined(__clang__)
__attribute__((target("avx2,fma")))
#endif
void ElementWiseSqrtSubMul_AVX2(const SquaredDist* a, const SquaredDist* b,
                                PhysicalDist scale, PhysicalDist* out,
                                int size) {
#include <immintrin.h>

    const __m256 scale_vec = _mm256_set1_ps(scale);
    int i = 0;
    // 每次处理 8 个 float：int32 -> float -> sqrt -> sub -> mul
    for (; i <= size - 8; i += 8) {
        const __m256i a_i =
            _mm256_loadu_si256(reinterpret_cast<const __m256i*>(a + i));
        const __m256i b_i =
            _mm256_loadu_si256(reinterpret_cast<const __m256i*>(b + i));

        __m256 a_vec = _mm256_cvtepi32_ps(a_i);
        __m256 b_vec = _mm256_cvtepi32_ps(b_i);

        a_vec = _mm256_sqrt_ps(a_vec);
        b_vec = _mm256_sqrt_ps(b_vec);

        const __m256 diff_vec = _mm256_sub_ps(a_vec, b_vec);
        const __m256 result_vec = _mm256_mul_ps(diff_vec, scale_vec);

        _mm256_storeu_ps(out + i, result_vec);
    }
    // 处理尾部剩余元素
    for (; i < size; ++i) {
        out[i] = (std::sqrt(static_cast<PhysicalDist>(a[i])) -
                  std::sqrt(static_cast<PhysicalDist>(b[i]))) *
                 scale;
    }
}

void SimdMath::ElementWiseSqrtSubMul(const SquaredDist* a, const SquaredDist* b,
                                     PhysicalDist scale, PhysicalDist* out,
                                     int size) {
    // CPUID 检测与标量 Fallback 直接内联在此函数中，只有 AVX2 路径保留独立
    // helper。
    static const bool has_avx2 = []() -> bool {
#if defined(__x86_64__) || defined(_M_X64) || defined(__i386__) || \
    defined(_M_IX86)
#if defined(__GNUC__) || defined(__clang__)
        return __builtin_cpu_supports("avx2");
#else
        return false;
#endif
#else
        return false;
#endif
    }();

    if (has_avx2) {
        ElementWiseSqrtSubMul_AVX2(a, b, scale, out, size);
    } else {
        for (int i = 0; i < size; ++i) {
            out[i] = (std::sqrt(static_cast<PhysicalDist>(a[i])) -
                      std::sqrt(static_cast<PhysicalDist>(b[i]))) *
                     scale;
        }
    }
}

// =========================================================================
// DistanceTransform：二维/一维网格距离变换
// =========================================================================

std::unique_ptr<SquaredDist[]> DistanceTransform::BuildSquaredDistField(
    const std::vector<uint8_t>& occ_data, int width, int height,
    bool target_is_occupied) {
    const auto max_squared_distance = static_cast<SquaredDist>(
        1LL * width * width + 1LL * height * height + 1LL);

    const auto expected_size =
        static_cast<GridIndex>(width) * static_cast<GridIndex>(height);

    // 使用 new T[size] 分配未初始化的原始内存，避免 std::vector(size)
    // 的强制零初始化
    std::unique_ptr<SquaredDist[]> buffer_a(new SquaredDist[expected_size]);
    std::unique_ptr<SquaredDist[]> buffer_b(new SquaredDist[expected_size]);

    const auto max_dim = std::max(width, height);

    // 小地图时并行域启动开销可能超过收益，阈值设为 8192 个像素
    const auto use_parallel = expected_size > 8192U;

    if (!use_parallel) {
        std::vector<int> parabola_indices(static_cast<GridIndex>(max_dim), 0);
        std::vector<std::int64_t> breakpoints(
            static_cast<GridIndex>(max_dim + 1), 0);

        // 阶段 0: 构建初始代价
        for (GridIndex index = 0; index < occ_data.size(); ++index) {
            const auto is_occupied = occ_data[index] != 0U;
            buffer_a[index] =
                is_occupied == target_is_occupied ? 0 : max_squared_distance;
        }

        // 阶段 1: 沿 X 方向（逐行）执行 1D 距离变换
        RunDistanceTransformByRows(buffer_a.get(), height, width,
                                   buffer_b.get(), parabola_indices.data(),
                                   breakpoints.data());

        // 阶段 2: 转置矩阵，将 Y 方向的列扫描转换为物理内存上的连续行扫描
        Transpose(buffer_b.get(), buffer_a.get(), height, width);

        // 阶段 3: 沿 Y 方向（现已转置为行）执行 1D 距离变换
        RunDistanceTransformByRows(buffer_a.get(), width, height,
                                   buffer_b.get(), parabola_indices.data(),
                                   breakpoints.data());

        // 阶段 4: 转置回原始地图坐标系
        Transpose(buffer_b.get(), buffer_a.get(), width, height);

        return buffer_a;
    }

    // 大地图：把 X 扫描、转置、Y 扫描、转置回全部包进一个 OpenMP 并行域
    // 只在 BuildSquaredDistField 内部启动一次并行域，避免 4 次启动/同步开销
    // 同时预分配所有线程的局部 buffer，避免每个并行域内重复堆分配
    const auto available_threads = omp_get_num_procs();
    const auto max_threads = std::min(available_threads, 8);
    const auto num_threads = std::max(1, max_threads);

    const auto parabola_buffer_size =
        static_cast<GridIndex>(num_threads) * static_cast<GridIndex>(max_dim);
    const auto breakpoint_buffer_size = static_cast<GridIndex>(num_threads) *
                                        static_cast<GridIndex>(max_dim + 1);
    std::vector<int> thread_parabola_indices(parabola_buffer_size, 0);
    std::vector<std::int64_t> thread_breakpoints(breakpoint_buffer_size, 0);

#pragma omp parallel num_threads(num_threads)
    {
        const auto thread_id = omp_get_thread_num();
        int* parabola_indices =
            thread_parabola_indices.data() + thread_id * max_dim;
        std::int64_t* breakpoints =
            thread_breakpoints.data() + thread_id * (max_dim + 1);

        // 阶段 0: 构建初始代价（并行化，每个线程写自己的一块）
#pragma omp for
        for (GridIndex index = 0; index < expected_size; ++index) {
            const auto is_occupied = occ_data[index] != 0U;
            buffer_a[index] =
                is_occupied == target_is_occupied ? 0 : max_squared_distance;
        }

        // 阶段 1: 沿 X 方向（逐行）执行 1D 距离变换
#pragma omp for schedule(dynamic, 8)
        for (int row = 0; row < height; ++row) {
            const auto offset = static_cast<GridIndex>(row * width);
            RunDistanceTransform1D(buffer_a.get() + offset,
                                   buffer_b.get() + offset, width,
                                   parabola_indices, breakpoints);
        }

        // 阶段 2: 转置矩阵（height x width -> width x height）
#pragma omp for collapse(2)
        for (int row_block = 0; row_block < height;
             row_block += DistanceTransform::BLOCK_SIZE) {
            for (int col_block = 0; col_block < width;
                 col_block += DistanceTransform::BLOCK_SIZE) {
                const int row_end =
                    std::min(row_block + DistanceTransform::BLOCK_SIZE, height);
                const int col_end =
                    std::min(col_block + DistanceTransform::BLOCK_SIZE, width);
                for (int row = row_block; row < row_end; ++row) {
                    const SquaredDist* src =
                        buffer_b.get() + row * width + col_block;
                    SquaredDist* dst =
                        buffer_a.get() + col_block * height + row;
                    for (int col = col_block; col < col_end; ++col) {
                        *dst = *src;
                        ++src;
                        dst += height;
                    }
                }
            }
        }

        // 阶段 3: 沿 Y 方向（现已转置为行）执行 1D 距离变换
#pragma omp for schedule(dynamic, 8)
        for (int row = 0; row < width; ++row) {
            const auto offset = static_cast<GridIndex>(row * height);
            RunDistanceTransform1D(buffer_a.get() + offset,
                                   buffer_b.get() + offset, height,
                                   parabola_indices, breakpoints);
        }

        // 阶段 4: 转置回原始地图坐标系（width x height -> height x width）
#pragma omp for collapse(2)
        for (int row_block = 0; row_block < width;
             row_block += DistanceTransform::BLOCK_SIZE) {
            for (int col_block = 0; col_block < height;
                 col_block += DistanceTransform::BLOCK_SIZE) {
                const int row_end =
                    std::min(row_block + DistanceTransform::BLOCK_SIZE, width);
                const int col_end =
                    std::min(col_block + DistanceTransform::BLOCK_SIZE, height);
                for (int row = row_block; row < row_end; ++row) {
                    const SquaredDist* src =
                        buffer_b.get() + row * height + col_block;
                    SquaredDist* dst = buffer_a.get() + col_block * width + row;
                    for (int col = col_block; col < col_end; ++col) {
                        *dst = *src;
                        ++src;
                        dst += width;
                    }
                }
            }
        }
    }

    return buffer_a;
}

void DistanceTransform::RunDistanceTransformByRows(const SquaredDist* input,
                                                   int row_count, int col_count,
                                                   SquaredDist* output,
                                                   int* parabola_indices,
                                                   std::int64_t* breakpoints) {
    if (row_count <= 0 || col_count <= 0) {
        return;
    }

    for (int row = 0; row < row_count; ++row) {
        const auto offset = static_cast<GridIndex>(row * col_count);
        RunDistanceTransform1D(input + offset, output + offset, col_count,
                               parabola_indices, breakpoints);
    }
}

void DistanceTransform::RunDistanceTransform1D(const SquaredDist* input,
                                               SquaredDist* output, int size,
                                               int* parabola_indices,
                                               std::int64_t* breakpoints) {
    if (size <= 0) {
        return;  // 卫语句保护边界情况
    }

    // 完全整数化的 Felzenszwalb & Huttenlocher 1D 平方距离变换。
    // breakpoints 中存储的是 2 倍交点的整数下取整（即 2*s），
    // 因此第二阶段比较时使用 2*q，避免任何浮点运算。
    // 退栈判断使用 int64_t 交叉相乘，消除浮点除法。

    int stack_top = 0;
    parabola_indices[0] = 0;
    breakpoints[0] = std::numeric_limits<std::int64_t>::min();
    breakpoints[1] = std::numeric_limits<std::int64_t>::max();

    auto get_val = [input](int x) -> std::int64_t {
        return 1LL * input[x] + 1LL * x * x;
    };

    // 阶段 1：从左到右扫描，构建抛物线的下包络线
    for (int q = 1; q < size; ++q) {
        const auto val_q = get_val(q);

        // 如果新抛物线 q 与栈顶抛物线 p 的交点落在 p 的生效区间左侧，
        // 则 p 被完全遮挡，退栈。
        while (stack_top > 0) {
            const int p = parabola_indices[stack_top];
            const auto val_p = get_val(p);
            const auto q_minus_p = 1LL * (q - p);

            // 2 * intersection(q, p) <= breakpoints[stack_top] ?
            // (val_q - val_p) / (q - p) <= breakpoints[stack_top]
            // => val_q - val_p <= breakpoints[stack_top] * (q - p)
            const auto val_diff = val_q - val_p;
            const auto threshold = breakpoints[stack_top] * q_minus_p;

            if (val_diff > threshold) {
                break;  // 找到有效交点，停止退栈
            }
            --stack_top;
        }

        // 将新抛物线压入栈顶，并记录它的生效区间起点
        const int p = parabola_indices[stack_top];
        const auto val_p = get_val(p);
        const auto q_minus_p = 1LL * (q - p);
        const auto two_s = (val_q - val_p) / q_minus_p;

        ++stack_top;
        parabola_indices[stack_top] = q;
        breakpoints[stack_top] = two_s;
        breakpoints[stack_top + 1] = std::numeric_limits<std::int64_t>::max();
    }

    // 阶段 2：通过查询下包络线，计算每个网格点对应的最小距离平方
    stack_top = 0;
    for (int q = 0; q < size; ++q) {
        const auto two_q = 2LL * q;
        // 如果查询点越过了当前抛物线的右边界，则移动到下一条抛物线
        while (breakpoints[stack_top + 1] < two_q) {
            ++stack_top;
        }

        const int nearest_index = parabola_indices[stack_top];
        const auto delta = q - nearest_index;

        output[q] = static_cast<SquaredDist>(1LL * delta * delta +
                                             1LL * input[nearest_index]);
    }
}

void DistanceTransform::Transpose(const SquaredDist* input, SquaredDist* output,
                                  int row_count, int col_count) {
    // 分块转置：将大矩阵拆成 16x16 的小块，使 input/output 都能驻留 L1 Cache
    // 减少 TLB Miss 和 Cache Miss
    for (int row_block = 0; row_block < row_count;
         row_block += DistanceTransform::BLOCK_SIZE) {
        const int row_end =
            std::min(row_block + DistanceTransform::BLOCK_SIZE, row_count);
        for (int col_block = 0; col_block < col_count;
             col_block += DistanceTransform::BLOCK_SIZE) {
            const int col_end =
                std::min(col_block + DistanceTransform::BLOCK_SIZE, col_count);
            for (int row = row_block; row < row_end; ++row) {
                const SquaredDist* src = input + row * col_count + col_block;
                SquaredDist* dst = output + col_block * row_count + row;
                for (int col = col_block; col < col_end; ++col) {
                    *dst = *src;
                    ++src;
                    dst += row_count;
                }
            }
        }
    }
}
}  // namespace

std::vector<PhysicalDist> ESDFMap::BuildSignedDistData(
    const std::vector<uint8_t>& occ_data, int width, int height,
    double resolution) {
    // 计算 D_out^2 与 D_in^2。
    const auto dist_field_out_squared =
        DistanceTransform::BuildSquaredDistField(occ_data, width, height, true);
    const auto dist_field_in_squared = DistanceTransform::BuildSquaredDistField(
        occ_data, width, height, false);
    // 计算得到最终符号距离
    const auto size = static_cast<int>(occ_data.size());
    std::vector<PhysicalDist> signed_distance(occ_data.size());
    SimdMath::ElementWiseSqrtSubMul(
        dist_field_out_squared.get(), dist_field_in_squared.get(),
        static_cast<PhysicalDist>(resolution), signed_distance.data(), size);
    return signed_distance;
}
}  // namespace apa_post_processor
