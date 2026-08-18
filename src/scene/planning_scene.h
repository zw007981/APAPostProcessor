#pragma once

#include <memory>
#include <string>

#include "../core/NMPC/nmpc_config.h"
#include "../core/post_processor.h"
#include "../spatial/esdf_map.h"
#include "../spatial/grid_map.h"
#include "../util/path.h"
#include "../util/trajectory.h"
#include "../vehicle/vehicle_footprint_model.h"
#include "../vehicle/vehicle_params.h"

namespace apa_post_processor {
// 规划场景基类持有场景公共资源，派生类负责传入自己的Config并实现具体的优化链路
class PlanningScene {
   public:
    virtual ~PlanningScene() = default;
    PlanningScene(const PlanningScene&) = delete;
    PlanningScene& operator=(const PlanningScene&) = delete;
    // 场景工厂：读取场景配置文件中的 config_details_path，按算法配置详情
    // JSON 的 "algorithm" 字段（"minco"/"nmpc"/"ilqr"）路由到对应算法场景并完成
    // 加载；文件缺失/字段缺失/算法无法识别时返回 nullptr（错误已记日志）。
    // 生产入口据此按配置运行时选择算法，无需改动代码
    static std::unique_ptr<PlanningScene> LoadFromFile(
        const std::string& config_file_path);
    // 执行优化（由派生类实现）
    virtual PostProcessorResult optimize() = 0;
    // 加载算法配置详情文件（由派生类实现）
    virtual void loadConfigDetails(const std::string& config_details_path) = 0;
    // 最近一次优化产出的轨迹（未执行 optimize() 或优化失败时为空），
    // 供生产入口算法无关地绘制"初始 vs 优化后"对比图
    virtual const Trajectory& optimizedTraj() const = 0;
    // 算法名（"MINCO"/"NMPC"，绘图标签与日志用）
    virtual std::string algorithmName() const = 0;
    // 生成优化摘要：优化前后路径长度与机动段数变化、优化耗时；未执行
    // optimize() 或优化失败时不含优化后指标并注明原因
    std::string optimizeSummary() const;
    // 打印优化摘要（成功记 LOG_INFO，未执行/失败记 LOG_ERROR）
    void printOptimizeSummary() const;
    // 车辆参数
    const VehicleParams& vehicleParams() const { return vehicle_params_; }
    // 车辆圆形分解模型
    const VehicleFootprintModel& footprintModel() const {
        return *footprint_model_;
    }
    // ESDF 距离场地图
    const ESDFMap& esdfMap() const { return *esdf_map_; }
    // 栅格地图
    const GridMap& gridMap() const { return *grid_map_; }
    // 初始路径（来自 protobuf 输入）
    const Path& initPath() const { return init_path_; }
    // 通用配置（可读写，实际指向派生类 Config 实例）
    Config& config() { return *config_; }
    const Config& config() const { return *config_; }
    // 最近一次 optimize() 的结果
    const PostProcessorResult& lastResult() const { return last_result_; }
    // 预处理产出的轨迹
    const Trajectory& preprocessedTraj() const { return preprocessed_traj_; }

   protected:
    // 派生类构造时传入自己的 Config 实例
    explicit PlanningScene(std::unique_ptr<Config> config);
    // 从配置文件加载场景数据，失败返回 false
    bool init(const std::string& config_file_path);

   protected:
    // 最近一次优化结果
    PostProcessorResult last_result_;
    // 预处理轨迹（带时间戳）
    Trajectory preprocessed_traj_;
    // 优化后轨迹（带时间戳）
    Trajectory optimized_traj_;

   private:
    // 通用配置（实际指向 NMPCConfig 等派生类型）
    std::unique_ptr<Config> config_;
    // 车辆运动学参数
    VehicleParams vehicle_params_;
    // 栅格地图（障碍物占据）
    std::unique_ptr<GridMap> grid_map_;
    // ESDF 距离场
    std::unique_ptr<ESDFMap> esdf_map_;
    // 车辆圆形分解 footprint 模型
    std::unique_ptr<VehicleFootprintModel> footprint_model_;
    // 初始几何路径
    Path init_path_;
};
}  // namespace apa_post_processor
