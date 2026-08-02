#pragma once

#include <Eigen/Core>
#include <algorithm>
#include <atomic>
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

// Euclidean Signed Distance Field (ESDF) 地图。
// 边界语义（M011 L8 修复后的行为契约）：
// - 图内：EDT 之前把最外圈栅格标记为占据（L8.2）——距离场在接近边界时
//   自然衰减到 ~0，边界在图内一侧即产生斥力；可用区域每边缩小一个栅格
// - 图外：按实心障碍处理（L8.1）——p = clamp(q, 地图矩形)，
//   d(q) = d_map(p) − ‖q−p‖（随穿透深度线性下降），∇d = (p−q)/‖q−p‖
//   恒指向图内；与 L8.2 叠加后全场在边界上构造性连续。恰在边界上
//   （‖q−p‖≈0）时回落图内场值（连续延拓，不再返回零梯度）
// - 越界查询不再逐条告警（L8.5）：恢复场使小幅越界成为合法探测，逐次
//   告警曾单次运行刷 8 万行日志、把本缺陷掩盖了三个 Milestone；改为
//   原子计数 + 编排层单次汇总
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
    // 原子计数成员使隐式拷贝/移动被删除；显式补齐（计数按值复制/转移，
    // 不与原对象共享）
    ESDFMap(const ESDFMap& other)
        : resolution_(other.resolution_),
          inv_resolution_(other.inv_resolution_),
          width_(other.width_),
          height_(other.height_),
          size_(other.size_),
          origin_(other.origin_),
          cell_data_(other.cell_data_),
          distance_data_(other.distance_data_) {
        out_of_map_queries_.store(
            other.out_of_map_queries_.load(std::memory_order_relaxed),
            std::memory_order_relaxed);
    }
    ESDFMap& operator=(const ESDFMap& other) {
        if (this != &other) {
            resolution_ = other.resolution_;
            inv_resolution_ = other.inv_resolution_;
            width_ = other.width_;
            height_ = other.height_;
            size_ = other.size_;
            origin_ = other.origin_;
            cell_data_ = other.cell_data_;
            distance_data_ = other.distance_data_;
            out_of_map_queries_.store(
                other.out_of_map_queries_.load(std::memory_order_relaxed),
                std::memory_order_relaxed);
        }
        return *this;
    }
    ESDFMap(ESDFMap&& other) noexcept
        : resolution_(other.resolution_),
          inv_resolution_(other.inv_resolution_),
          width_(other.width_),
          height_(other.height_),
          size_(other.size_),
          origin_(other.origin_),
          cell_data_(std::move(other.cell_data_)),
          distance_data_(std::move(other.distance_data_)) {
        out_of_map_queries_.store(
            other.out_of_map_queries_.load(std::memory_order_relaxed),
            std::memory_order_relaxed);
    }
    ESDFMap& operator=(ESDFMap&& other) noexcept {
        if (this != &other) {
            resolution_ = other.resolution_;
            inv_resolution_ = other.inv_resolution_;
            width_ = other.width_;
            height_ = other.height_;
            size_ = other.size_;
            origin_ = other.origin_;
            cell_data_ = std::move(other.cell_data_);
            distance_data_ = std::move(other.distance_data_);
            out_of_map_queries_.store(
                other.out_of_map_queries_.load(std::memory_order_relaxed),
                std::memory_order_relaxed);
        }
        return *this;
    }
    // 查询符号距离与梯度（图外语义见类注释的边界契约）
    std::pair<double, Eigen::Vector2d> getDistAndGrad(double x, double y) const;
    // 批量查询符号距离与梯度
    void getDistAndGradBatch(const double* xs, const double* ys, int n,
                             double* dists_out, double* grad_x_out,
                             double* grad_y_out) const;
    // 查询符号距离（正值 = 障碍物外部；图外 = −穿透深度，见类注释）
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
    // 越界查询计数（L8.5，原子）：恢复场使小幅越界成为合法探测，逐次告警
    // 已移除；计数由编排层在求解前后读取差值并单次汇总
    std::size_t outOfMapQueryCount() const {
        return out_of_map_queries_.load(std::memory_order_relaxed);
    }
    // 计数器是 mutable 诊断状态（不构成逻辑状态），const 语义下允许复位
    void resetOutOfMapQueryCount() const {
        out_of_map_queries_.store(0, std::memory_order_relaxed);
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
    // 图内场的双线性插值实现（调用方必须保证 (x,y) 在图内或为边界的
    // 钳制点——calBilinearParams 的索引钳制使后者同样安全）
    double interpolateDist(double x, double y) const;
    std::pair<double, Eigen::Vector2d> interpolateDistAndGrad(double x,
                                                              double y) const;
    // L8.1 图外恢复场：p = clamp(q, 地图矩形)，d = d_map(p) − ‖q−p‖，
    // ∇d = (p−q)/‖q−p‖（恒指向图内）；s≈0（恰在边界）回落图内场值
    std::pair<double, Eigen::Vector2d> recoveredFieldOutside(double x,
                                                             double y) const;

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
    // 越界查询计数（L8.5）：查询路径可多线程并行（批量/OMP），必须原子
    mutable std::atomic<std::size_t> out_of_map_queries_{0};
};
}  // namespace apa_post_processor
