#pragma once

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <vector>

#include "../util/constants.h"
#include "../util/logger.h"
#include "vehicle_params.h"

namespace apa_post_processor {
// 圆的类型
enum class CircleType {
    // 车辆内部圆
    INNER,
    // 车辆外轮廓圆
    OUTER
};

// 基于圆形近似的车辆占据模型，提供不同航向角下的圆心查找表和雅可比计算接口
class VehicleFootprintModel {
   public:
    // 使用车辆参数和离散配置构造VehicleFootprintModel实例
    VehicleFootprintModel(const VehicleParams& veh_params,
                          int heading_sample_num = 233, int inner_row_num = 2,
                          int outer_row_num = 4);
    // 输入后轴中心的位姿与圆的种类计算各个圆心的坐标，
    // 以及各个圆心相对于此位姿的2x3雅可比矩阵
    void calInterpolatedCenters(
        double x, double y, double theta, CircleType type,
        std::vector<Eigen::Vector2d>& centers,
        std::vector<Eigen::Matrix<double, 2, 3>>& jacobians) const;
    // 获取内圆半径
    double getInnerRadius() const { return inner_radius_; }
    // 获取外圆半径
    double getOuterRadius() const { return outer_radius_; }
    // 获取指定类型圆的数量
    std::size_t getCircleNum(CircleType type) const {
        const auto& table = (type == CircleType::INNER) ? inner_circle_table_
                                                        : outer_circle_table_;
        return table.empty() ? 0U : table.front().size();
    }

   protected:
    // 将角度归一化到[0, 2PI)
    static double normalizeTheta(double theta) {
        constexpr double TWO_PI = 2.0 * PI, INV_TWO_PI = 1.0 / TWO_PI;
        theta -= std::floor(theta * INV_TWO_PI) * TWO_PI;
        if (theta < 0.0) {
            theta += TWO_PI;
        }
        return theta;
    }
    // 构建查找表，角度范围为[0, 2PI)。
    void buildLookupTable();
    // 生成后轴位于原点且航向为theta时的内外圈圆心集合。
    void generateCirclesAtOrigin(double theta,
                                 std::vector<Eigen::Vector2d>& inner_circles,
                                 std::vector<Eigen::Vector2d>& outer_circles);

   protected:
    // 车辆基础物理参数
    const VehicleParams veh_params_;
    // 在360度的航向角范围内离散采样数
    const int heading_sample_num_{233};
    // 航向离散分辨率(rad)
    const double heading_resolution_{2.0 * PI};
    // 内圈沿车长方向的离散行数
    const int inner_row_num_{2};
    // 外圈沿车长方向的离散行数
    const int outer_row_num_{4};
    // 内圈缓存表：[heading_index][circle_index] -> [x,y]
    std::vector<std::vector<Eigen::Vector2d>> inner_circle_table_;
    // 外圈缓存表：[heading_index][circle_index] -> [x,y]
    std::vector<std::vector<Eigen::Vector2d>> outer_circle_table_;
    // 内圆半径缓存
    double inner_radius_{0.0};
    // 外圆半径缓存
    double outer_radius_{0.0};
};
}  // namespace apa_post_processor