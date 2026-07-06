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
// 栅格一维索引：用于 row-major 数组访问，避免裸 size_t 噪音。
using GridIndex = std::size_t;
// 平方距离的内部存储类型。F&H 算法在整数网格上天然为整数运算，
// 使用 int32_t 可让 1D 扫描完全在 ALU 上完成，避免 FPU 参与。
using SquaredDist = std::int32_t;
// 物理距离缓存元素类型（符号距离、梯度分量）。
using PhysicalDist = float;

// Euclidean Signed Distance Field (ESDF)地图类
class ESDFMap {
    // 用于双线性插值的参数
    struct BilinearParams {
        BilinearParams() = default;
        // 将坐标映射到网格后，x方向的下界索引
        int col_lower{0};
        // x方向的上界索引
        int col_upper{0};
        // y方向的下界索引
        int row_lower{0};
        // y方向的上界索引
        int row_upper{0};
        // x方向上相较于下界的偏移量，这里作为x方向上的插值权重
        double col_ratio{0.0};
        // y方向上的插值权重
        double row_ratio{0.0};
    };

   public:
    // 基于占据栅格地图构建ESDF
    explicit ESDFMap(const GridMap& grid_map);
    // 输入位置返回离最近障碍物的符号距离与梯度
    std::pair<double, Eigen::Vector2d> getDistAndGrad(double x, double y) const;
    // 输入位置返回离最近障碍物的符号距离，正值表示位于障碍物外部
    double getDist(double x, double y) const;
    // 获取地图分辨率
    double getResolution() const { return resolution_; }
    // 获取地图宽度（像素列数）
    int getWidth() const { return width_; }
    // 获取地图高度（像素行数）
    int getHeight() const { return height_; }
    // 获取地图原点（物理坐标系下左下角坐标），供调用方在查询前判断/裁剪坐标范围
    Position getOrigin() const { return origin_; }
    // 获取栅格个数
    GridIndex size() const { return size_; }

   protected:
    // 判断物理坐标是否落在GridMap定义的地图范围内
    bool inMap(double x, double y) const;
    // 根据行列索引计算一维数组索引
    int getIndex(int row, int col) const { return row * width_ + col; }
    // 根据位置计算双线性插值参数
    BilinearParams calBilinearParams(double x, double y) const;
    // 对标量场执行双线性插值
    double calBilinearVal(const std::vector<PhysicalDist>& data,
                          const BilinearParams& params) const;
    // 基于符号距离计算梯度场
    void calGradField();
    // 构建最终符号距离：D_out - D_in
    static std::vector<PhysicalDist> BuildSignedDistData(
        const std::vector<uint8_t>& occ_data, int width, int height,
        double res);

   protected:
    // 网格分辨率（单位m）
    double resolution_{0.1};
    // 网格分辨率的倒数
    double inv_resolution_{10.0};
    // 地图宽度（像素列数）
    int width_{0};
    // 地图高度（像素行数）
    int height_{0};
    // 栅格个数
    GridIndex size_{0};
    // 地图原点位置
    Position origin_;
    // 符号距离缓存，index = row * width_ + col
    std::vector<PhysicalDist> distance_data_;
    // x方向物理梯度缓存
    std::vector<PhysicalDist> grad_x_data_;
    // y方向物理梯度缓存
    std::vector<PhysicalDist> grad_y_data_;
};
}  // namespace apa_post_processor
