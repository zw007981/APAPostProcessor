#pragma once

#include <Eigen/Core>

#include <map_interface.h>

#include "../../spatial/esdf_map.h"

namespace apa_post_processor {
// 适配器：把apa_post_processor::ESDFMap包装为StcSQP的EsdfMapInterface，
// 使圆形分解ESDF碰撞约束（CircleFootprintEsdfConstraint）与业务层ESDF地图对接
class ApaEsdfMapAdapter : public stc_SQP::EsdfMapInterface {
   public:
    // 持有ESDFMap的常量引用，调用方需保证其生命周期覆盖本适配器的整个使用周期
    explicit ApaEsdfMapAdapter(const ESDFMap& esdf_map);
    // 实现EsdfMapInterface：把ESDFMap::getDistAndGrad的结果转换为EsdfSample
    stc_SQP::EsdfSample queryDistance(
        const Eigen::Vector2d& point) const override;

   protected:
    // 底层ESDF地图，非空引用，生命周期由调用方保证
    const ESDFMap& esdf_map_;
};
}  // namespace apa_post_processor
