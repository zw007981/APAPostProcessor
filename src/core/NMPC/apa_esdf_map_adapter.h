#pragma once

#include <map_interface.h>

#include <Eigen/Core>

#include "../../spatial/esdf_map.h"

namespace apa_post_processor {
// 把 ESDFMap 包装为 StcSQP EsdfMapInterface，供圆形分解碰撞约束使用。
class ApaEsdfMapAdapter : public stc_SQP::EsdfMapInterface {
   public:
    // 持有 ESDFMap 的常量引用。
    explicit ApaEsdfMapAdapter(const ESDFMap& esdf_map);
    // 实现 EsdfMapInterface 的距离查询接口。
    stc_SQP::EsdfSample queryDistance(
        const Eigen::Vector2d& point) const override;

   protected:
    // 底层 ESDF 地图引用
    const ESDFMap& esdf_map_;
};
}  // namespace apa_post_processor
