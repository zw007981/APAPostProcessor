#include "apa_esdf_map_adapter.h"

#include <algorithm>

namespace apa_post_processor {
ApaEsdfMapAdapter::ApaEsdfMapAdapter(const ESDFMap& esdf_map)
    : esdf_map_(esdf_map) {}

stc_SQP::EsdfSample ApaEsdfMapAdapter::queryDistance(
    const Eigen::Vector2d& point) const {
    // 仅当查询点越界时才裁剪坐标，地图内的点原样透传（与底层ESDFMap查询结果逐位一致，
    // 见apa_esdf_map_adapter.t.cpp的QueryDistanceMatchesUnderlyingEsdfMap/
    // QueryDistanceConsistentAcrossMultiplePoints）。ESDFMap::getDistAndGrad对越界
    // 查询固定返回(距离=0, 梯度=0)（属地图类自身既有契约，不应改动），但对本代价项而言，
    // distance=0意味着"正贴在障碍物表面"，配合零梯度会让SQP完全失去把圆心拉回地图内部
    // 的方向信息：一旦线性化让某个圆心跑出地图（无解bug修复后，早期迭代中容易发生），
    // 代价会一直卡在高位却没有梯度可降，实测会导致轨迹严重发散（如data7.json长度
    // 18.7m→33.6m）。裁剪到边界内侧最近点后再查询，可以拿到边界附近真实的距离/梯度，
    // 形成把圆心推回地图内部的有效梯度。
    const auto origin = esdf_map_.getOrigin();
    const double resolution = esdf_map_.getResolution();
    const double max_x = origin.x + static_cast<double>(esdf_map_.getWidth()) * resolution;
    const double max_y = origin.y + static_cast<double>(esdf_map_.getHeight()) * resolution;
    const bool in_map = point.x() >= origin.x && point.y() >= origin.y &&
                        point.x() < max_x && point.y() < max_y;
    double query_x = point.x();
    double query_y = point.y();
    if (!in_map) {
        // 内缩半个分辨率，确保裁剪后的点严格落在地图有效范围内（避免刚好落在上边界
        // 导致的浮点误差再次判定越界）
        const double inset = resolution * 0.5;
        query_x = std::clamp(point.x(), origin.x + inset, max_x - inset);
        query_y = std::clamp(point.y(), origin.y + inset, max_y - inset);
    }
    const auto [distance, gradient] =
        esdf_map_.getDistAndGrad(query_x, query_y);
    return stc_SQP::EsdfSample{distance, gradient};
}
}  // namespace apa_post_processor

