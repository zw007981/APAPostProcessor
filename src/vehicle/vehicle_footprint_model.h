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
// 圆类型
enum class CircleType { INNER, OUTER };

// 基于圆形近似的车辆占据模型
class VehicleFootprintModel {
   public:
    // 使用车辆参数构造
    VehicleFootprintModel(const VehicleParams& veh_params,
                          int heading_sample_num = 233, int inner_row_num = 2,
                          int outer_row_num = 4);
    // 计算圆心坐标及雅可比矩阵
    void calInterpolatedCenters(
        double x, double y, double theta, CircleType type,
        std::vector<Eigen::Vector2d>& centers,
        std::vector<Eigen::Matrix<double, 2, 3>>& jacobians) const;
    // 获取内圆半径
    double getInnerRadius() const { return inner_radius_; }
    // 获取外圆半径
    double getOuterRadius() const { return outer_radius_; }
    // 获取车辆轴距 (m)（运动学关系 θ̇=v·tanδ/L 中的 L）
    double getWheelbase() const { return veh_params_.wheelbase; }
    // 获取指定类型圆的数量
    std::size_t getCircleNum(CircleType type) const {
        const auto& table = (type == CircleType::INNER) ? inner_circle_table_
                                                        : outer_circle_table_;
        return table.empty() ? 0U : table.front().size();
    }
    // 获取外圈沿车长方向的离散行数
    int getOuterRowNum() const { return outer_row_num_; }

   protected:
    // 角度归一化到[0, 2PI)
    static double normalizeTheta(double theta) {
        constexpr double TWO_PI = 2.0 * PI, INV_TWO_PI = 1.0 / TWO_PI;
        theta -= std::floor(theta * INV_TWO_PI) * TWO_PI;
        if (theta < 0.0) {
            theta += TWO_PI;
        }
        return theta;
    }
    // 构建查找表
    void buildLookupTable();
    // 生成原点处给定航向的圆心集合
    void generateCirclesAtOrigin(double theta,
                                 std::vector<Eigen::Vector2d>& inner_circles,
                                 std::vector<Eigen::Vector2d>& outer_circles);

   protected:
    const VehicleParams veh_params_;
    // 航向角离散采样数
    const int heading_sample_num_{233};
    // 航向离散分辨率 (rad)
    const double heading_resolution_{2.0 * PI};
    // 内圈行数
    const int inner_row_num_{2};
    // 外圈行数
    const int outer_row_num_{4};
    // 内圈缓存表
    std::vector<std::vector<Eigen::Vector2d>> inner_circle_table_;
    // 外圈缓存表
    std::vector<std::vector<Eigen::Vector2d>> outer_circle_table_;
    // 内圆半径
    double inner_radius_{0.0};
    // 外圆半径
    double outer_radius_{0.0};
};
}  // namespace apa_post_processor