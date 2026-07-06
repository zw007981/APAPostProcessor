#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iterator>
#include <numeric>
#include <optional>
#include <type_traits>
#include <utility>
#include <vector>

#include "apa_post_process.pb.h"
#include "constants.h"
#include "maneuver.h"
#include "path_point.h"
#include "pose.h"
#include "type_traits.h"

namespace apa_post_processor {
namespace path {
// 判断类型是否为存放 Pose 的可迭代序列
// 不放开给 PathPoint 序列，避免调用方误以为可以自行提供有意义的曲率/状态量。
template <typename TContainer>
inline constexpr bool is_pose_sequence_v =
    util::is_iterable_v<std::decay_t<TContainer>> &&
    std::is_same_v<util::iterable_value_t<std::decay_t<TContainer>>, Pose>;
}  // namespace path

class Path {
   protected:
    // 允许的最大物理步长 (m)，超过则触发递归线性插值。
    static constexpr double MAX_GAP_DIST = 4.0 * DELTA_DIST;
    // 纵向运动阈值 (m)，低于此值认为无明确前后移动。
    static constexpr double DOT_EPSILON = 0.01;
    // 旋转阈值 (3度)，用于判定原地掉头/钟摆。
    static constexpr double PIVOT_THETA_EPSILON = 3.0 * DEG2RAD;
    // 曲率差分窗口阈值，在计算曲率时要求据前后参考点间的距离尽量达到此值以减少抖动。
    static constexpr double CURVATURE_WINDOW_DIST = 0.3;
    // 曲率退化保护阈值，避免参考点过近时除以极小数。
    static constexpr double CURVATURE_SQ_EPSILON =
        EPSILON_PRECISE * EPSILON_PRECISE;

   public:
    Path() = default;
    // 基于任意 Pose 序列的模板构造函数，遍历序列并依次调用 addPoint 添加路径点
    template <typename TContainer,
              std::enable_if_t<path::is_pose_sequence_v<TContainer>, int> = 0>
    explicit Path(TContainer&& poses) {
        for (const auto& pose : poses) {
            this->addPoint(pose);
        }
        this->finalize();
    }
    // 基于protobuf消息构造Path结构体
    static Path FromProto(const ::apa::post_processor::Path& path_proto);
    // 导出为扁平化的Pose序列
    void toProto(::apa::post_processor::Path* path_proto) const;
    // 导出为嵌套的Maneuver序列
    void toProto(
        ::google::protobuf::RepeatedPtrField<::apa::post_processor::Maneuver>*
            maneuvers_proto) const;
    // 添加路径点（调用方只需提供几何Pose，内部包装为PathPoint并自行计算曲率）
    void addPoint(Pose point);
    // 路径收尾，统一处理曲率计算
    void finalize();
    // 判断路径是否为空
    bool empty() const {
        return maneuvers_.empty() || maneuvers_.back().points.empty();
    }
    // 获取maneuver的数量
    std::size_t numManeuvers() const {
        return this->empty() ? 0 : maneuvers_.size();
    }
    // 获取可供修改的maneuver信息
    std::vector<Maneuver>& getManeuvers();
    // 获取常量maneuver信息
    const std::vector<Maneuver>& getManeuvers() const { return maneuvers_; }
    // 获取路径点的数量
    std::size_t size() const;
    // 获取路径总长度
    double length() const;
    // 常量遍历路径点的方法，使用方式类似于
    // path.forEach([](const PathPoint& pt) {std::cout << pt.x; });
    template <typename Func>
    void forEach(Func&& func) const {
        if (this->empty()) {
            return;
        }
        // 第一段完全遍历
        for (const auto& pt : maneuvers_.front().points) {
            func(pt);
        }
        // 后续段跳过首个重复点
        for (auto m_it = std::next(maneuvers_.begin());
             m_it != maneuvers_.end(); ++m_it) {
            if (m_it->points.size() <= 1) {
                continue;
            }
            for (auto pt_it = std::next(m_it->points.begin());
                 pt_it != m_it->points.end(); ++pt_it) {
                func(*pt_it);
            }
        }
    }
    // 把路径信息转化为json格式的字符串
    std::string toString() const;
    // 获取第一个路径点
    const PathPoint& front() const {
        if (this->empty()) {
            throw std::runtime_error(
                "Path is empty, cannot access front point!!!");
        }
        return maneuvers_.front().points.front();
    }
    // 获取最后一个路径点
    const PathPoint& back() const {
        if (this->empty()) {
            throw std::runtime_error(
                "Path is empty, cannot access back point!!!");
        }
        return maneuvers_.back().points.back();
    }

   protected:
    // 使用外接圆法计算点序列中指定索引点的有向曲率
    static void ComputeMengerCurvature(std::vector<PathPoint>& points,
                                       std::size_t idx);
    // 从指定索引向前寻找曲率计算的前向参考点
    static std::size_t CalPrevCurvatureRefIndex(
        const std::vector<PathPoint>& points, std::size_t idx);
    // 从指定索引向后寻找曲率计算的后向参考点
    static std::size_t CalNextCurvatureRefIndex(
        const std::vector<PathPoint>& points, std::size_t idx);
    // 对单个机动段的所有内部点统一计算曲率
    static void RefreshCurvatureDrafts(std::vector<PathPoint>& points);

   protected:
    // 此路径中的机动段序列
    std::vector<Maneuver> maneuvers_;
    // 路径长度缓存
    mutable std::optional<double> length_cache_{0.0};
};
}  // namespace apa_post_processor
