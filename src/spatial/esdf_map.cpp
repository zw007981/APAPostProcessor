#include "esdf_map.h"

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
    distance_data_ =
        BuildSignedDistData(occ_data, width_, height_, resolution_);
    calGradField();
}

std::pair<double, Eigen::Vector2d> ESDFMap::getDistAndGrad(double x,
                                                           double y) const {
    if (!this->inMap(x, y)) {
        LOG_FMT_WARN("ESDFMap query out of bounds: ({}, {})!", x, y);
        return {0.0, Eigen::Vector2d::Zero()};
    }
    // 执行双线性插值获取距离值与梯度
    const auto params = this->calBilinearParams(x, y);
    const double dist = this->calBilinearVal(distance_data_, params);
    Eigen::Vector2d grad;
    grad.x() = this->calBilinearVal(grad_x_data_, params);
    grad.y() = this->calBilinearVal(grad_y_data_, params);
    return {dist, grad};
}

double ESDFMap::getDist(double x, double y) const {
    if (!this->inMap(x, y)) {
        LOG_FMT_WARN("ESDFMap query out of bounds: ({}, {})!", x, y);
        return 0.0;
    }
    return this->calBilinearVal(distance_data_, this->calBilinearParams(x, y));
}

bool ESDFMap::inMap(double x, double y) const {
    const auto max_x = origin_.x + static_cast<double>(width_) * resolution_;
    const auto max_y = origin_.y + static_cast<double>(height_) * resolution_;
    return x >= origin_.x && y >= origin_.y && x < max_x && y < max_y;
}

ESDFMap::BilinearParams ESDFMap::calBilinearParams(double x, double y) const {
    auto params = BilinearParams();
    // 根据坐标计算x,y方向上的前后栅格
    const auto col_float = (x - origin_.x) * inv_resolution_,
               row_float = (y - origin_.y) * inv_resolution_;
    params.col_lower =
        std::clamp(static_cast<int>(std::floor(col_float)), 0, width_ - 1);
    params.row_lower =
        std::clamp(static_cast<int>(std::floor(row_float)), 0, height_ - 1);
    params.col_upper = std::min(params.col_lower + 1, width_ - 1);
    params.row_upper = std::min(params.row_lower + 1, height_ - 1);
    // 计算双线性插值的权重系数
    auto calRatioFunc = [](double val, int lower, int upper) -> double {
        return (lower == upper) ? 0.0 : (val - static_cast<double>(lower));
    };
    params.col_ratio =
        calRatioFunc(col_float, params.col_lower, params.col_upper);
    params.row_ratio =
        calRatioFunc(row_float, params.row_lower, params.row_upper);
    return params;
}

double ESDFMap::calBilinearVal(const std::vector<PhysicalDist>& data,
                               const BilinearParams& params) const {
    // 获取指定行列索引的值
    auto fetchFunc = [&](int row, int col) -> double {
        return static_cast<double>(data[getIndex(row, col)]);
    };
    // 线性插值，等价于a * (1 - t) + b * t
    constexpr auto lerpFunc = [](double a, double b, double t) -> double {
        return a + (b - a) * t;
    };
    // 分别在下方和上方沿着X方向进行插值
    const auto val_bottom =
                   lerpFunc(fetchFunc(params.row_upper, params.col_lower),
                            fetchFunc(params.row_upper, params.col_upper),
                            params.col_ratio),
               val_top = lerpFunc(fetchFunc(params.row_lower, params.col_lower),
                                  fetchFunc(params.row_lower, params.col_upper),
                                  params.col_ratio);
    // 沿着Y方向进行最终插值
    return lerpFunc(val_top, val_bottom, params.row_ratio);
}

void ESDFMap::calGradField() {
    grad_x_data_.assign(size_, 0.0f);
    grad_y_data_.assign(size_, 0.0f);
    if (width_ <= 1 || height_ <= 1) {
        return;
    }
    // 基于有限差分法计算梯度，边界使用单侧差分，内部使用中心差分
    // 边界 grad_x approx (DistField(x+1,y) - DistField(x,y)) / res
    // 内部 grad_x approx (DistField(x+1,y) - DistField(x-1,y)) / (2 * res)
    const auto INV_RES = static_cast<PhysicalDist>(inv_resolution_),
               HALF_INV_RES = INV_RES * 0.5f;
    // 距离场只读基准地址
    const PhysicalDist* dist_base = distance_data_.data();
    // x方向梯度场基准地址
    PhysicalDist* grad_x_base = grad_x_data_.data();
    // y方向梯度场基准地址
    PhysicalDist* grad_y_base = grad_y_data_.data();
#pragma omp parallel for schedule(dynamic, 8)
    for (int row = 0; row < height_; ++row) {
        const auto ROW_INV_RES =
            (row == 0 || row == height_ - 1) ? INV_RES : HALF_INV_RES;
        // 从距离场中提取出当前行及上下相邻行的首地址
        const PhysicalDist *dist_row_cur = dist_base + row * width_,
                           *dist_row_prev =
                               dist_base + std::max(0, row - 1) * width_,
                           *dist_row_next =
                               dist_base +
                               std::min(row + 1, height_ - 1) * width_;
        // 需要求解的梯度行地址
        PhysicalDist *grad_x_row = grad_x_base + row * width_,
                     *grad_y_row = grad_y_base + row * width_;
        // 左边界
        int col = 0;
        grad_x_row[col] = (dist_row_cur[col + 1] - dist_row_cur[col]) * INV_RES;
        grad_y_row[col] =
            (dist_row_next[col] - dist_row_prev[col]) * ROW_INV_RES;
        // 内部区域
        for (col = 1; col < width_ - 1; col++) {
            grad_x_row[col] =
                (dist_row_cur[col + 1] - dist_row_cur[col - 1]) * HALF_INV_RES;
            grad_y_row[col] =
                (dist_row_next[col] - dist_row_prev[col]) * ROW_INV_RES;
        }
        // 右边界
        col = width_ - 1;
        grad_x_row[col] = (dist_row_cur[col] - dist_row_cur[col - 1]) * INV_RES;
        grad_y_row[col] =
            (dist_row_next[col] - dist_row_prev[col]) * ROW_INV_RES;
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
