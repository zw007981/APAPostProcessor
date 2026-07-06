#include "vehicle_circle_geometry.h"

namespace apa_post_processor {
namespace vehicle_circle_geometry {
std::vector<Eigen::Vector2d> ExtractLocalCircleCenters(
    const VehicleFootprintModel& footprint_model, CircleType type) {
    const auto circle_num =
        static_cast<int>(footprint_model.getCircleNum(type));
    std::vector<Eigen::Vector2d> centers(circle_num);
    std::vector<Eigen::Matrix<double, 2, 3>> jacobians(circle_num);
    // 位姿取原点、航向为0，calInterpolatedCenters在该处退化为精确的车身局部坐标，无插值误差
    footprint_model.calInterpolatedCenters(0.0, 0.0, 0.0, type, centers,
                                           jacobians);
    return centers;
}
}  // namespace vehicle_circle_geometry
}  // namespace apa_post_processor
