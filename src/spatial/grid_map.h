#pragma once

#include <algorithm>
#include <cmath>
#include <vector>

#include "../util/constants.h"
#include "../util/logger.h"
#include "../util/pose.h"
#include "../util/position.h"
#include "apa_post_process.pb.h"

namespace apa_post_processor {
// 栅格地图类
class GridMap {
   public:
    // 使用分辨率、尺寸、原点和占据信息构造GridMap对象
    // origin 为第 0 行第 0 列栅格的中心
    GridMap(double resolution, int width, int height, const Position& origin,
            const std::vector<Position>& cells);
    // 从proto消息构造GridMap对象
    static GridMap FromProto(const ::apa::post_processor::GridMap& proto);
    // 根据物理坐标计算一维数组索引，非法时返回-1
    inline int getIndex(double x, double y) const {
        const auto col_double =
            (x - origin_.x) * inv_resolution_ + INDEX_EPSILON;
        const auto row_double =
            (y - origin_.y) * inv_resolution_ + INDEX_EPSILON;
        const auto col = fastFloor(col_double);
        const auto row = fastFloor(row_double);
        if (row < 0 || col < 0 || row >= height_ || col >= width_) {
            return -1;
        }
        return getIndex(row, col);
    }
    // 根据行列索引计算一维数组索引
    inline int getIndex(int row, int col) const { return row * width_ + col; }
    // 根据行列索引获取栅格中心的坐标
    inline Position getPosition(int row, int col) const {
        const auto clamped_row = std::clamp(row, 0, height_ - 1),
                   clamped_col = std::clamp(col, 0, width_ - 1);
        return Position{origin_.x + static_cast<double>(clamped_col) * resolution_,
                        origin_.y + static_cast<double>(clamped_row) * resolution_};
    }
    // 基于物理坐标查询占用状态
    inline bool isOccupied(double x, double y) const {
        const auto index = getIndex(x, y);
        if (index < 0) {
            return true;
        }
        return data_[static_cast<std::size_t>(index)] != 0U;
    }
    // 基于内存索引查询占用状态
    inline bool isOccupied(int row, int col) const {
        if (row < 0 || col < 0 || row >= height_ || col >= width_) {
            return true;
        }
        return data_[static_cast<std::size_t>(getIndex(row, col))] != 0U;
    }
    // 获取占据信息的只读引用，可供ESDF构建时直接访问
    const auto& getOccupancyData() const { return data_; }
    // 获取地图分辨率
    double getResolution() const { return resolution_; }
    // 获取地图宽度（像素列数）
    int getWidth() const { return width_; }
    // 获取地图高度（像素行数）
    int getHeight() const { return height_; }
    // 获取栅格数量
    std::size_t size() const { return data_.size(); }
    // 获取地图原点位置
    const Position& getOrigin() const { return origin_; }

   protected:
    // 网格分辨率（单位m）
    double resolution_{0.1};
    // 网格分辨率的倒数
    double inv_resolution_{10.0};
    // 地图宽度 (像素列数)
    int width_{0};
    // 地图高度 (像素行数)
    int height_{0};
    // 地图原点位置（第 0 行第 0 列栅格的中心）
    Position origin_;
    // 网格占据信息，index = row * width_ + col，0=free, 1=occupied
    std::vector<uint8_t> data_;
    // 用于抵消浮点计算误差，避免 floor 在网格边界处错误跌落
    static constexpr double INDEX_EPSILON = 1e-9;
    // 快速向下取整，仅覆盖本类索引映射所需的常规有限数值路径
    static int fastFloor(double value) {
        const auto truncated = static_cast<int>(value);
        return truncated -
               static_cast<int>(value < static_cast<double>(truncated));
    }
};
}  // namespace apa_post_processor