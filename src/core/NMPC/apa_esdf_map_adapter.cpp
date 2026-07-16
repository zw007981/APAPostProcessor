#include "apa_esdf_map_adapter.h"

#include <algorithm>

namespace apa_post_processor {
ApaEsdfMapAdapter::ApaEsdfMapAdapter(const ESDFMap& esdf_map)
    : esdf_map_(esdf_map) {}

stc_SQP::EsdfSample ApaEsdfMapAdapter::queryDistance(
    const Eigen::Vector2d& point) const {
    // 仅当查询点越界时才裁剪坐标，地图内的点原样透传。
    // 越界时裁剪到边界内侧，避免 distance=0 的退化情况使 SQP 失去梯度方向。
    const auto origin = esdf_map_.getOrigin();
    const double resolution = esdf_map_.getResolution();
    const double max_x =
        origin.x + static_cast<double>(esdf_map_.getWidth()) * resolution;
    const double max_y =
        origin.y + static_cast<double>(esdf_map_.getHeight()) * resolution;
    const bool in_map = point.x() >= origin.x && point.y() >= origin.y &&
                        point.x() < max_x && point.y() < max_y;
    double query_x = point.x();
    double query_y = point.y();
    if (!in_map) {
        // 内缩半个分辨率，确保裁剪后的点严格落在地图有效范围内
        const double inset = resolution * 0.5;
        query_x = std::clamp(point.x(), origin.x + inset, max_x - inset);
        query_y = std::clamp(point.y(), origin.y + inset, max_y - inset);
    }
    const auto [distance, gradient] =
        esdf_map_.getDistAndGrad(query_x, query_y);
    return stc_SQP::EsdfSample{distance, gradient};
}
}  // namespace apa_post_processor
