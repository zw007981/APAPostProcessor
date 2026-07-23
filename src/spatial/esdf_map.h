#pragma once

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <numeric>
#include <stdexcept>
#include <utility>
#include <vector>

#include "../util/constants.h"
#include "../util/logger.h"
#include "grid_map.h"

namespace apa_post_processor {
// 栅格一维索引
using GridIndex = std::size_t;
// 平方距离存储类型
using SquaredDist = std::int32_t;
// 物理距离缓存元素类型
using PhysicalDist = double;

// Euclidean Signed Distance Field (ESDF) 地图
class ESDFMap {
    // 双线性插值参数
    struct BilinearParams {
        BilinearParams() = default;
        int col_lower{0};
        int col_upper{0};
        int row_lower{0};
        int row_upper{0};
        // 四角点索引
        int idx_bl{0};
        int idx_br{0};
        int idx_tl{0};
        int idx_tr{0};
        // 插值权重
        double col_ratio{0.0};
        double row_ratio{0.0};
    };

   public:
    // 基于占据栅格地图构建ESDF
    explicit ESDFMap(const GridMap& grid_map);
    // 查询符号距离与梯度
    std::pair<double, Eigen::Vector2d> getDistAndGrad(double x, double y) const;
    // 批量查询符号距离与梯度
    void getDistAndGradBatch(const double* xs, const double* ys, int n,
                             double* dists_out, double* grad_x_out,
                             double* grad_y_out) const;
    // 查询符号距离（正值 = 障碍物外部）
    double getDist(double x, double y) const;
    double getResolution() const { return resolution_; }
    int getWidth() const { return width_; }
    int getHeight() const { return height_; }
    // 地图原点
    Position getOrigin() const { return origin_; }
    // 栅格总数
    GridIndex size() const { return size_; }
    // 判断坐标是否在地图范围内
    bool inMap(double x, double y) const {
        return x >= origin_.x && x < origin_.x + width_ * resolution_ &&
               y >= origin_.y && y < origin_.y + height_ * resolution_;
    }

   protected:
    // 单个栅格的符号距离与梯度 (AoS)
    struct ESDFCell {
        PhysicalDist dist{0.0};
        PhysicalDist grad_x{0.0};
        PhysicalDist grad_y{0.0};
    };

   protected:
    int getIndex(int row, int col) const { return row * width_ + col; }
    // 计算双线性插值参数
    BilinearParams calBilinearParams(double x, double y) const;
    // 计算梯度场
    void calGradField();
    // 构建符号距离
    static std::vector<PhysicalDist> BuildSignedDistData(
        const std::vector<uint8_t>& occ_data, int width, int height,
        double res);

   protected:
    // 网格分辨率 (m)
    double resolution_{0.1};
    // 分辨率倒数
    double inv_resolution_{10.0};
    // 宽度 (列)
    int width_{0};
    // 高度 (行)
    int height_{0};
    // 栅格总数
    GridIndex size_{0};
    Position origin_;
    // AoS 距离与梯度缓存
    std::vector<ESDFCell> cell_data_;
    // 紧凑距离缓存（供 getDist 高效访问）
    std::vector<PhysicalDist> distance_data_;
};
}  // namespace apa_post_processor
