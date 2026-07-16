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
// 栅格地图
class GridMap {
   public:
    // 使用分辨率、尺寸和占据信息构造
    GridMap(double resolution, int width, int height, const Position& origin,
            const std::vector<Position>& cells);
    // 基于protobuf消息构造
    static GridMap FromProto(const ::apa::post_processor::GridMap& proto);
    // 根据物理坐标计算索引，非法时返回-1
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
    inline int getIndex(int row, int col) const { return row * width_ + col; }
    // 根据行列索引获取栅格中心坐标
    inline Position getPosition(int row, int col) const {
        const auto clamped_row = std::clamp(row, 0, height_ - 1),
                   clamped_col = std::clamp(col, 0, width_ - 1);
        return Position{
            origin_.x + static_cast<double>(clamped_col) * resolution_,
            origin_.y + static_cast<double>(clamped_row) * resolution_};
    }
    // 查询占用状态（物理坐标）
    inline bool isOccupied(double x, double y) const {
        const auto index = getIndex(x, y);
        if (index < 0) {
            return true;
        }
        return data_[static_cast<std::size_t>(index)] != 0U;
    }
    // 查询占用状态（行列索引）
    inline bool isOccupied(int row, int col) const {
        if (row < 0 || col < 0 || row >= height_ || col >= width_) {
            return true;
        }
        return data_[static_cast<std::size_t>(getIndex(row, col))] != 0U;
    }
    const auto& getOccupancyData() const { return data_; }
    double getResolution() const { return resolution_; }
    int getWidth() const { return width_; }
    int getHeight() const { return height_; }
    // 栅格总数
    std::size_t size() const { return data_.size(); }
    const Position& getOrigin() const { return origin_; }

   protected:
    // 网格分辨率 (m)
    double resolution_{0.1};
    // 分辨率倒数
    double inv_resolution_{10.0};
    // 宽度 (列)
    int width_{0};
    // 高度 (行)
    int height_{0};
    // 原点（第0行第0列栅格中心）
    Position origin_;
    // 占据信息 (0=free, 1=occupied)
    std::vector<uint8_t> data_;
    // 浮点舍入容差
    static constexpr double INDEX_EPSILON = 1e-9;
    // 快速向下取整
    static int fastFloor(double value) {
        const auto truncated = static_cast<int>(value);
        return truncated -
               static_cast<int>(value < static_cast<double>(truncated));
    }
};
}  // namespace apa_post_processor