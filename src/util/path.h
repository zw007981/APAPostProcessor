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
#include "pose.h"
#include "trajectory_point.h"
#include "type_traits.h"

namespace apa_post_processor {
namespace path {
// 判断类型是否为Pose可迭代序列（排除TrajectoryPoint，避免曲率误用）
template <typename TContainer>
inline constexpr bool is_pose_sequence_v =
    util::is_iterable_v<std::decay_t<TContainer>> &&
    std::is_same_v<util::iterable_value_t<std::decay_t<TContainer>>, Pose>;
}  // namespace path

class Path {
   protected:
    // 最大物理步长 (m)
    static constexpr double MAX_GAP_DIST = 4.0 * DELTA_DIST;
    // 纵向运动阈值 (m)
    static constexpr double DOT_EPSILON = 0.01;
    // 旋转阈值 (rad)
    static constexpr double PIVOT_THETA_EPSILON = 3.0 * DEG2RAD;

   public:
    Path() = default;
    // 基于Pose序列构造
    template <typename TContainer,
              std::enable_if_t<path::is_pose_sequence_v<TContainer>, int> = 0>
    explicit Path(TContainer&& poses) {
        for (const auto& pose : poses) {
            this->addPoint(pose);
        }
        this->finalize();
    }
    // 基于protobuf消息构造
    static Path FromProto(const ::apa::post_processor::Path& path_proto);
    // 导出为扁平Pose序列
    void toProto(::apa::post_processor::Path* path_proto) const;
    // 导出为嵌套Maneuver序列
    void toProto(
        ::google::protobuf::RepeatedPtrField<::apa::post_processor::Maneuver>*
            maneuvers_proto) const;
    // 添加路径点（自动计算曲率）
    void addPoint(Pose point);
    // 收尾：统一计算曲率
    void finalize();
    // 路径是否为空
    bool empty() const {
        return maneuvers_.empty() || maneuvers_.back().points.empty();
    }
    // maneuver数量
    std::size_t numManeuvers() const {
        return this->empty() ? 0 : maneuvers_.size();
    }
    std::vector<Maneuver>& getManeuvers();
    const std::vector<Maneuver>& getManeuvers() const { return maneuvers_; }
    // 路径点总数
    std::size_t size() const;
    // 路径总长度
    double length() const;
    // 遍历所有路径点
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
    // 转化为JSON字符串
    std::string toString() const;
    // 首个路径点
    const TrajectoryPoint& front() const {
        if (this->empty()) {
            throw std::runtime_error(
                "Path is empty, cannot access front point!!!");
        }
        return maneuvers_.front().points.front();
    }
    // 末个路径点
    const TrajectoryPoint& back() const {
        if (this->empty()) {
            throw std::runtime_error(
                "Path is empty, cannot access back point!!!");
        }
        return maneuvers_.back().points.back();
    }

   protected:
    // 角度差分法计算有向曲率
    static void RefreshCurvatureDrafts(std::vector<TrajectoryPoint>& points);

   protected:
    std::vector<Maneuver> maneuvers_;
    // 路径长度缓存
    mutable std::optional<double> length_cache_{0.0};
};
}  // namespace apa_post_processor
