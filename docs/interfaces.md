# 核心接口契约（冻结清单）

> 本文件登记项目中"一旦确定不可随意更改"的核心抽象接口。每一条只记录**设计意图 + 指向实际头文件的链接**，
> 具体签名以链接的头文件为唯一真值，**禁止在本文件里复制代码片段**，避免和头文件不同步。

## 使用方式

- Dev Agent 在实现/修改核心接口前，必须先读本文件确认该接口是否已冻结。
- Review Agent 审查"需求对齐"维度时，以本文件登记的接口为基准之一。
- 新增/变更已冻结接口，必须先在下方"变更记录"登记原因，再动手改代码；未登记的接口签名变更视为违反 [.agents/rules.md](../.agents/rules.md) 红线。

## 已冻结接口清单

| 接口 | 设计意图（一句话） | 头文件链接 | 状态 |
|---|---|---|---|
| `Position` | 纯二维坐标，用于栅格原点/中心等只需位置的场景 | [src/util/position.h](../src/util/position.h) | 已冻结 |
| `Pose` | 纯几何位姿（位置+朝向），**不携带任何路径规划派生量**，用于车辆瞬时位姿等通用场景 | [src/util/pose.h](../src/util/pose.h) | 已冻结 |
| `PathPoint` | `Pose` + 一组"不一定存在"的派生量（曲率 `kappa`、状态量 `v`/`delta`、控制量 `a`/`delta_dot`），代表"路径规划/轨迹优化输出的轨迹点"；这些派生量默认值均为 NaN（代表未提供），只能通过 `hasXxx()`/`getXxx()`/`setXxx()` 三件套访问，`getXxx()` 在未设置时抛出 `std::logic_error`；proto 层不携带这些派生量（详见下方 proto 兼容性说明） | [src/util/trajectory_point.h](../src/util/trajectory_point.h) | 已冻结 |
| `Maneuver` | 同一运动方向下的 `PathPoint` 有序序列，`Path` 的最小分段单元 | [src/util/maneuver.h](../src/util/maneuver.h) | 已冻结 |
| `Path` | 由 `Maneuver` 序列组成的路径，封装逐点追加时的去重/插值/方向推断，曲率估计在 `finalize()` 中统一批量完成 | [src/util/path.h](../src/util/path.h) | 已冻结 |
| `VehicleParams` | 车辆物理参数（长宽轴距/最大转向角/纵向加减速极限/最大转向角速度），派生最大曲率 `max_kappa` | [src/vehicle/vehicle_params.h](../src/vehicle/vehicle_params.h) | 已冻结 |
| `VehicleFootprintModel` | 用内/外两组圆的查找表近似车身占据区域，按航向角离散化 | [src/vehicle/vehicle_footprint_model.h](../src/vehicle/vehicle_footprint_model.h) | 已冻结 |
| `GridMap` | 离散占据栅格，提供物理坐标与栅格索引的双向映射 | [src/spatial/grid_map.h](../src/spatial/grid_map.h) | 已冻结 |
| `ESDFMap` | 基于 `GridMap` 构建的欧式符号距离场，提供双线性插值的距离与梯度查询 | [src/spatial/esdf_map.h](../src/spatial/esdf_map.h) | 已冻结 |
| `ApaEsdfMapAdapter` | 把 `ESDFMap` 适配为 `third_party/StcSQP` 的 `stc_SQP::EsdfMapInterface`，衔接业务地图与通用求解引擎 | [src/core/NMPC/apa_esdf_map_adapter.h](../src/core/NMPC/apa_esdf_map_adapter.h) | 已冻结 |
| `vehicle_circle_geometry::ExtractLocalCircleCenters` | 从 `VehicleFootprintModel` 提取车身坐标系下的圆心局部坐标，供 `CircleFootprintEsdfConstraint` 使用 | [src/core/NMPC/vehicle_circle_geometry.h](../src/core/NMPC/vehicle_circle_geometry.h) | 已冻结 |
| `PathToOcpConverter` | 把 `Path` 转换为 `MultiStageOCP` 描述与初始猜测轨迹；已拆分为 `computeSegmentProfiles()` / `generateInitialGuess()` / `buildOcp()` 三个独立入口，`convert()` 保持向后兼容 | [src/core/NMPC/path_to_ocp_converter.h](../src/core/NMPC/path_to_ocp_converter.h) | 已冻结 |
| `SegmentProfile` | 单段 OCP 离散化描述（N/dt/dt_array/v_sign/is_terminal），衔接 `PathToOcpConverter` 的 OCP 装配与初始猜测生成 | [src/core/NMPC/path_to_ocp_converter.h](../src/core/NMPC/path_to_ocp_converter.h) | 已冻结 |
| `NmpcSolver` | 编排 Path→OCP 转换、StcSQP 求解与求解后机动段裁剪，返回优化轨迹与分段信息；新增 `optimize(MultiStageOCP, Trajectory, ESDFMap)` 扩展点，供预装配 OCP/初始猜测的场景直接接入 | [src/core/NMPC/nmpc_solver.h](../src/core/NMPC/nmpc_solver.h) | 已冻结 |
| `BSplineSmoother` | 对单个 `Maneuver` 拟合四次 B 样条曲线，输出控制点、弧长重参数化表与密集配点缓存；输入 `Maneuver` + `ESDFMap` + `VehicleFootprintModel`，构造函数与 `smooth()` 为唯一外部入口 | [src/preprocessing/bspline_smoother.h](../src/preprocessing/bspline_smoother.h) | 已冻结 |
| `BSplineSmootherConfig` | `BSplineSmoother` 的配置结构体，字段默认值与来源见 `docs/default_params.md` | [src/preprocessing/bspline_smoother.h](../src/preprocessing/bspline_smoother.h) | 已冻结 |
| `SpeedProfilePlanner` | 基于空间域 $v^2$ 凸优化的纵向速度规划器：输入密集配点（`s`/`kappa`/`min_esdf_dist`）、方向符号与可选尖点，输出带符号的速度/加速度/时间戳 | [src/preprocessing/speed_profile_planner.h](../src/preprocessing/speed_profile_planner.h) | 已冻结 |
| `SpeedProfilePlannerConfig` | `SpeedProfilePlanner` 的配置结构体，字段默认值与来源见 `docs/default_params.md` | [src/preprocessing/speed_profile_planner.h](../src/preprocessing/speed_profile_planner.h) | 已冻结 |
| `SpeedProfileInput` | `SpeedProfilePlanner` 的输入结构体，每个密集配点对应弧长、曲率与最小 ESDF 距离 | [src/preprocessing/speed_profile_planner.h](../src/preprocessing/speed_profile_planner.h) | 已冻结 |
| `SpeedProfileResult` | `SpeedProfilePlanner` 的输出结构体，包含求解状态、带符号速度/加速度/时间戳与原始 QP 变量 $b=v^2$ | [src/preprocessing/speed_profile_planner.h](../src/preprocessing/speed_profile_planner.h) | 已冻结 |
| `DifferentialFlatnessSolver` | 基于微分平坦性质，由 B 样条几何导数与速度剖面解析反推前轮偏角 `delta` 及其变化率 `delta_dot`，输出完整 NMPC 状态/控制序列 | [src/preprocessing/differential_flatness_solver.h](../src/preprocessing/differential_flatness_solver.h) | 已冻结 |
| `DifferentialFlatnessSolverConfig` | `DifferentialFlatnessSolver` 的配置结构体，字段默认值见 `src/preprocessing/differential_flatness_solver.h` | [src/preprocessing/differential_flatness_solver.h](../src/preprocessing/differential_flatness_solver.h) | 已冻结 |
| `DifferentialFlatnessInput` | `DifferentialFlatnessSolver` 的输入结构体，每个密集配点对应 B 样条几何导数与速度规划结果 | [src/preprocessing/differential_flatness_solver.h](../src/preprocessing/differential_flatness_solver.h) | 已冻结 |
| `DifferentialFlatnessResult` | `DifferentialFlatnessSolver` 的输出结构体，包含完整 `PathPoint` 状态/控制序列 | [src/preprocessing/differential_flatness_solver.h](../src/preprocessing/differential_flatness_solver.h) | 已冻结 |
| `AdaptiveResamplerConfig` | `AdaptiveResampler` 的配置结构体，字段默认值见 `src/preprocessing/adaptive_resampler.h` | [src/preprocessing/adaptive_resampler.h](../src/preprocessing/adaptive_resampler.h) | 已冻结 |
| `AdaptiveResamplerSegmentInput` | `AdaptiveResampler` 的单段输入结构体，聚合 `BSplineSmoother::DensePointData` 与 `DifferentialFlatnessSolver` 产出的完整状态/控制序列 | [src/preprocessing/adaptive_resampler.h](../src/preprocessing/adaptive_resampler.h) | 已冻结 |
| `AdaptiveResamplerResult` | `AdaptiveResampler` 的输出结构体，包含固定维数打靶点序列与对应 `delta_t` 静态参数向量 | [src/preprocessing/adaptive_resampler.h](../src/preprocessing/adaptive_resampler.h) | 已冻结 |
| `AdaptiveResampler` | 轨迹自适应重采样与维数固化器：将预处理管线前序阶段（B 样条平滑/速度规划/微分平坦解算）产出的密集序列压缩至 NMPC 固定维数，并在弧长域处理完成后注入换挡/起始转向对齐补丁 | [src/preprocessing/adaptive_resampler.h](../src/preprocessing/adaptive_resampler.h) | 已冻结 |
| `StaticCorridorBuilderConfig` | `StaticCorridorBuilder` 的配置结构体，字段 `hard_margin`/`soft_margin` 默认值来源见 `docs/default_params.md` | [src/preprocessing/static_corridor_builder.h](../src/preprocessing/static_corridor_builder.h) | 已冻结 |
| `StaticCorridorBuilder` | 静态安全走廊构建器：以自适应重采样阶段产出的固定维数 Z_ref 为泰勒展开基准点，一次性查询 ESDF 距离与梯度，输出标准线性不等式 `C_matrix`/`d_vector`（含 hard/soft 双边界） | [src/preprocessing/static_corridor_builder.h](../src/preprocessing/static_corridor_builder.h) | 已冻结 |
| `PreprocessingPipelineConfig` | 预处理管线配置，聚合五个预处理阶段的 Config 结构体 + `use_static_corridor` 过渡开关 + `collision_safety_margin` 跨阶段统一数值容差（默认 0，外圆自身提供安全缓冲；仅依赖 `kCollisionEpsilon` 兜底浮点误差）+ `enable_debug_output` 调试数据透传开关（默认 `false`） | [src/preprocessing/preprocessing_pipeline.h](../src/preprocessing/preprocessing_pipeline.h) | 已冻结 |
| `PreprocessingPipeline` | 预处理管线组装器：按 Maneuver 顺序依次调用 B 样条平滑、速度规划、微分平坦解算、自适应重采样、静态走廊构建五个阶段，输出固定维数 Z_ref/U_ref、delta_t 序列与静态走廊系数；`enable_debug_output` 开启时透传各阶段中间产物 | [src/preprocessing/preprocessing_pipeline.h](../src/preprocessing/preprocessing_pipeline.h) | 已冻结 |
| `PreprocessingPipelineResult` | `PreprocessingPipeline::run()` 的输出结构体，包含 Z_ref、delta_t、C_matrix/d_vector、各阶段耗时分解、`debug_maneuver_outputs`（仅在 `enable_debug_output=true` 时填充）、`original_z_ref`（Subplot 1 原始路径对比）、`hard_margin_used`/`soft_margin_used`/`outer_row_num_used`（可视化侧与管线参数保持一致） | [src/preprocessing/preprocessing_pipeline.h](../src/preprocessing/preprocessing_pipeline.h) | 已冻结 |
| `PreprocessingToOcpConverter` | 把 `PreprocessingPipelineResult`（`z_ref`/`delta_t`）转换为可直接喂给 `NmpcSolver` 的 `MultiStageOCP` + `Trajectory` Warm Start；同步按 OCP 总步数截断 `c_matrix`/`d_vector` | [src/core/NMPC/preprocessing_to_ocp_converter.h](../src/core/NMPC/preprocessing_to_ocp_converter.h) | 已完成 |
| `DetailSeriesData` | `Visualizer` 的 Details 图序列缓存结构体；已扩展 `v`/`a`/`delta`/`delta_dot`/`delta_t`/`slack_true`/`slack_calc` 字段，支持预处理管线诊断数据 | [src/util/visualizer.hpp](../src/util/visualizer.hpp) | 已冻结 |
| `Visualizer` | 可视化器：新增 `plotPipelineDiagnostics()` 公开入口，渲染预处理管线 2x2 诊断子图；`plotPipelineDiagnostics()` 新增可选 `VehicleFootprintModel*` 参数，传入后 index-domain 子图绘制 `Z_ref` 每点处所有外圆的最小安全余量 `min(d_esdf - r)`；新增 `drawDetailSubplotDualY()` 双 Y 轴子图模板方法 | [src/util/visualizer.hpp](../src/util/visualizer.hpp) | 已冻结 |

> 以下模块目录当前为空文件（规划中，尚未实现），暂不登记接口：`src/spatial/sfc_corridor.h/.cpp`、`src/core/obb_inflator.h/.cpp`、`src/scene/planning_scene.h/.cpp`。待其实现时再补充登记。

## StcSQP 框架内部接口（联合治理）

> `third_party/StcSQP` 本身按第三方依赖对待、不计入本仓库业务代码规范（见 [docs/architecture.md](architecture.md)
> 第 4 节），下表只登记本仓库侧对 `third_party/StcSQP` 主动修改的那几个内部接口，作为本仓库侧的变更留痕；
> 这些接口的权威定义与设计意图仍以 `third_party/StcSQP/design_document.md`/`AGENTS.md` 为准，
> 两边文档必须同步更新，不能只登记在一处。

| 接口 | 设计意图（一句话） | 头文件链接 | 状态 |
|---|---|---|---|
| `stc_SQP::CostTerm` | 代价项求值纯虚接口；新增组合求值方法 `evaluateGradientAndHessian()`（默认转发到既有 `evaluate()`+`gradient()`+`hessian()`，向后兼容）与 `clone()`（供 `assembleQP()` 并行化使用）；`CircleFootprintEsdfPenaltyCost`/`CompositeCost`/`QuadraticTrackingCost` 均已实现对应方法 | [third_party/StcSQP/src/costs/cost_term.hpp](../third_party/StcSQP/src/costs/cost_term.hpp) | 已完成 |
| `stc_SQP::Constraint` | 约束求值纯虚接口；已评估 `std::variant` 去虚拟化方案，benchmark 无收益，决定不合入，保留虚函数机制 | [third_party/StcSQP/src/constraints/constraint.hpp](../third_party/StcSQP/src/constraints/constraint.hpp) | 已评估不采纳 |
| `stc_SQP::HPIPMQPSolver` | HPIPM 求解器包装；已将 `setWarmStart()` 从空实现改为真正的 IPM 跨迭代热启动，新增 `lastIterations()`/`totalIterations()` 读取 IPM 迭代统计 | [third_party/StcSQP/src/qp/hpipm_solver.h](../third_party/StcSQP/src/qp/hpipm_solver.h) | 已完成 |
| `stc_SQP::SQPSolver` | Full SQP 主循环；已为 `assembleQP()`/`assembleCost()` 补充 OMP 并行路径，并在 `solveQP()` 中接入跨迭代热启动调用与 `use_qp_warm_start` 配置开关 | [third_party/StcSQP/src/sqp/sqp_algorithm.h](../third_party/StcSQP/src/sqp/sqp_algorithm.h) | 已完成 |
| `stc_SQP::StageSegment` | 单段 OCP 描述；已实现 `std::variant` 改造并 benchmark，无收益，已评估不采纳，保留原有 `shared_ptr<Constraint/CostTerm>` 字段类型 | [third_party/StcSQP/src/ocp/multi_stage_ocp.h](../third_party/StcSQP/src/ocp/multi_stage_ocp.h) | 已评估不采纳 |

## proto 兼容性说明

`proto/apa_post_process.proto` 中 `Path`/`Maneuver` message 携带的路径点始终只使用
`apa::post_processor::Pose`（`x`/`y`/`theta`），**不新增字段承载 `PathPoint` 的派生量**
（`kappa`/`v`/`delta`/`a`/`delta_dot`）。这些派生量目前只在进程内产生和消费，不需要
持久化或跨进程传递；C++ 侧的 `Path::FromProto`/`toProto` 负责在 `Pose`（proto 反序列化产物）
与 `PathPoint`（`Path`/`Maneuver` 的内部存储类型）之间转换。若未来确有需要持久化某个派生量，
应先按 [.agents/instructions/dependency-policy.md](../.agents/instructions/dependency-policy.md)
之外的正常 proto 变更流程评估，再更新本节。

## 变更记录

| 日期 | 接口 | 变更内容 | 原因 |
|---|---|---|---|
| 2026-07-06 | `Pose` | 移除 `kappa` 字段，恢复为纯 `{x, y, theta}` | 曲率是路径规划语境下的派生量，不应耦合进通用位姿类型 |
| 2026-07-06 | `PathPoint` | 从空壳继承类补充为真正独立的路径点类型：新增 `kappa`/`v`/`delta`/`a`/`delta_dot` 五个派生量，默认值均为 `std::numeric_limits<double>::quiet_NaN()`，通过 `hasXxx()`/`getXxx()`/`setXxx()` 三件套访问（`getXxx()` 未设置时抛 `std::logic_error`），改用 `class`（而非 `struct`）+ `protected` 字段体现"不一定存在"的封装语义 | 承接原本错误挂在 `Pose` 上的曲率职责；补充 NMPC 优化轨迹的状态/控制量（`v`/`delta`/`a`/`delta_dot`），使 `NmpcSolver` 输出不再丢弃这些信息；用 has/get 强制调用方在使用前确认可用性，避免误用哨兵值 |
| 2026-07-06 | `Maneuver` | 模板构造函数收窄为只接受 `PathPoint`/`std::vector<PathPoint>`，不再接受裸 `Pose`/`std::vector<Pose>` | 避免调用方误以为可以自行提供有意义的 `kappa`，让类型系统在编译期堵住误用 |
| 2026-07-06 | `Path` | `front()`/`back()`/`forEach` 回调对外暴露类型从 `Pose` 改为 `PathPoint`；内部曲率估计静态方法操作 `std::vector<PathPoint>&` | 与 `Maneuver` 存储类型变更保持一致 |
| 2026-07-06 | `Path` | 曲率计算时机从 `addPoint()` 增量刷新改为 `finalize()` 统一批量计算；移除 `refreshTailCurvature()` | 避免"草稿曲率/最终曲率"两套状态并存的复杂度，简化曲率估计算法本身 |
| 2026-07-06 | `Pose` | 移除 `kappa` 字段后正式冻结为纯 `{x, y, theta}` | 曲率职责已迁移到 `PathPoint`，通用位姿类型不再携带路径规划派生量 |
| 2026-07-06 | `PathPoint` | 在 `Path`/`Maneuver`/`NMPC`/`Visualizer` 中全面接入 | `Path`/`Maneuver` 存储类型从 `Pose` 切换为 `PathPoint`，NMPC 输出回填 v/delta/a/delta_dot |
| 2026-07-06 | `Maneuver` | 模板构造函数收窄为只接受 `PathPoint`/`std::vector<PathPoint>` 后冻结 | 避免调用方绕过 `Path` 曲率估计自行提供 `kappa` |
| 2026-07-06 | `PathToOcpConverter` | `buildCumulativeArcLength`/`interpolateAtArcLength` 改用 `PathPoint` 后冻结 | 与 `Maneuver` 存储类型保持一致 |
| 2026-07-06 | `NmpcSolver` | `ToPath`/`pruneShortestSegment` 改用 `PathPoint` 并回填 v/delta/a/delta_dot 后冻结 | 优化结果的状态/控制量不再丢失 |
| 2026-07-06 | `Path` | `addPoint()` 不再刷新曲率，移除 `refreshTailCurvature()`；`finalize()` 统一批量计算所有 `Maneuver` 的曲率 | 避免"草稿曲率/最终曲率"两套状态并存，简化曲率估计算法 |
| 2026-07-06 | `Path` | Pose/PathPoint 重构收尾审计通过，将 `Path` 接口状态更新为"已冻结" | 此前几轮重构引入的变更已落地并验证，接口进入稳定状态 |
| 2026-07-06 | `BSplineSmoother` / `BSplineSmootherConfig` | 新增四次 B 样条分段平滑器接口，登记构造函数与 `smooth()` 主入口；配置字段默认值来源在对应设计文档中记录 | 预处理管线首个落地接口，为后续速度规划、微分平坦、自适应重采样等阶段提供统一曲线/弧长表/密集配点输入 |
| 2026-07-06 | `VehicleParams` | 新增 `max_accel`/`max_decel`/`max_steer_rate` 字段，proto 同步为 optional 字段；`FromProto` 对缺失字段使用默认安全值 | 纵向速度规划需要加减速 box bound，`max_steer_rate` 提前落地供换挡原地打轮补丁复用 |
| 2026-07-06 | `SpeedProfilePlanner` / `SpeedProfilePlannerConfig` / `SpeedProfileInput` / `SpeedProfileResult` | 新增空间域 $v^2$ 凸优化速度规划接口；配置、输入、输出结构体一并登记 | 为预处理管线提供纵向速度规划能力，衔接 BSplineSmoother 输出的密集配点 |
| 2026-07-06 | `DifferentialFlatnessSolver` / `DifferentialFlatnessSolverConfig` / `DifferentialFlatnessInput` / `DifferentialFlatnessResult` | 新增微分平坦解析补全接口；配置、输入、输出结构体一并登记 | 为预处理管线提供状态/控制量解析补全能力，衔接 BSplineSmoother 几何导数与 SpeedProfilePlanner 速度剖面 |
| 2026-07-06 | `AdaptiveResampler` / `AdaptiveResamplerConfig` / `AdaptiveResamplerSegmentInput` / `AdaptiveResamplerResult` | 新增轨迹自适应重采样与维数固化接口；配置、输入、输出结构体一并登记 | 为预处理管线提供固定维数打靶点序列生成能力，衔接 BSplineSmoother 密集配点、SpeedProfilePlanner 速度剖面与 DifferentialFlatnessSolver 状态/控制量 |
| 2026-07-07 | `StaticCorridorBuilder` / `StaticCorridorBuilderConfig` | 新增静态安全走廊构建器接口，登记 `build()` 主入口与 `hard_margin`/`soft_margin` 配置字段 | 为预处理管线提供一次性 ESDF 线性化走廊生成能力，替代 NMPC 热循环中动态 ESDF 查询 |
| 2026-07-07 | `BSplineSmoother` | 将 `buildKnotVector()` 与 `computeBasisAtU()` 从 protected 提升为 public | 预处理管线需要在构造 SpeedProfileInput / DifferentialFlatnessInput 时从平滑结果中提取 B 样条几何导数；这两个方法是纯工具函数，不涉及内部状态变更，提升访问级别不会破坏封装 |
| 2026-07-07 | `PreprocessingPipelineConfig` / `PreprocessingPipeline` / `PreprocessingPipelineResult` | 新增预处理管线组装器接口，登记 `run()` 主入口、配置聚合结构体与结果结构体 | 为 NMPC 提供端到端的 Warm Start 生成能力，把此前五个独立预处理阶段串成一条可复用的管线 |
| 2026-07-07 | `NmpcSolverConfig` | 新增 `static_corridor_C`（`std::optional<Eigen::MatrixXd>`）与 `static_corridor_d`（`std::optional<Eigen::VectorXd>`）字段，标记 `esdf_penalty_weight` 为过渡期废弃字段 | 静态线性走廊（NMPC.md 3.5 节）与现有动态 ESDF 软代价是替代关系；过渡期保留 `esdf_penalty_weight` 作为兜底开关，当 `static_corridor_C` 未提供时仍使用动态 ESDF 软代价维持向后兼容。完整的 HPIPM 静态走廊约束注入留待后续战术改动 |
| 2026-07-07 | `StaticCorridorBuilderConfig` | `hard_margin` 默认值从 0.08（8cm）改为 0.05（5cm），增加交叉引用注释指向 `BSplineSmootherConfig::collision_margin` | 碰撞安全裕度统一：`hard_margin` 与 `collision_margin` 代表同一物理量 $d_{margin}$，此前各自独立配置默认值（5cm vs 8cm）存在跨阶段数值不自洽风险 |
| 2026-07-07 | `BSplineSmootherConfig` | `collision_margin` 注释补充交叉引用，指向 `StaticCorridorBuilderConfig::hard_margin` 与 `PreprocessingPipelineConfig::collision_safety_margin` | 字段默认值本身未变（已是 5cm），仅补充注释防止未来有人单方面修改其中一个而不同步另一个 |
| 2026-07-07 | `PreprocessingPipelineConfig` | 新增 `collision_safety_margin` 字段（默认 0.05），作为跨阶段统一安全裕度的唯一权威来源；`PreprocessingPipeline` 构造函数自动将其传播到 `bspline.collision_margin` 与 `corridor.hard_margin` | 消除此前 `BSplineSmoother` 与 `StaticCorridorBuilder` 各自独立配置安全裕度（5cm vs 8cm）的不自洽风险，提供单一配置入口 |
| 2026-07-07 | `NmpcSolverConfig` | 新增 `max_theta_deviation_from_ref` 字段（默认 0.0，即关闭信赖域约束）；启用时建议初始值 0.06 rad，按真实数据集调参确定最终取值 | 静态走廊一阶泰勒线性化只在状态偏离 $Z_{ref}$ 较小时成立，此前设计未显式约束该偏差范围 |
| 2026-07-07 | `NmpcSolver` | `optimize()` 内部新增信赖域约束（`ThetaTrustRegionConstraint`）与静态走廊约束（`StaticCorridorLinearConstraint`）的注入逻辑；静态走廊与动态 ESDF 软代价为替代关系，当 `static_corridor_C/d` 提供且数据一致时使用静态走廊，否则兜底使用 `esdf_penalty_weight` 动态查询机制 | HPIPM 层静态走廊约束注入 + 信赖域 box 约束，两个前置条件已同时落地 |
| 2026-07-07 | `PathToOcpConverter` | `buildSegment()` 新增 `stage_params` 填充逻辑（p(0)=局部步索引，p(1)=参考航向 theta_ref），供下游约束类按步读取参数 | 为信赖域约束与静态走廊约束提供 per-step 上下文（theta_ref / 步索引） |
| 2026-07-07 | `PreprocessingPipelineConfig` / `PreprocessingPipelineResult` / `PreprocessingPipeline` | 新增调试数据透传：`PreprocessingPipelineConfig::enable_debug_output`（默认 `false`）、`PreprocessingPipelineResult::debug_maneuver_outputs`、`PipelineDebugManeuverOutput`；`PreprocessingPipeline::run()` 开启开关时移动中间产物到结果 | 为 `Visualizer::plotPipelineDiagnostics` 提供各阶段中间产物，默认关闭不影响生产路径内存占用 |
| 2026-07-07 | `Visualizer` / `DetailSeriesData` | 扩展 `DetailSeriesData` 动力学/控制/时间/走廊松弛量字段；新增 `Visualizer::plotPipelineDiagnostics()` 2x2 诊断入口与 `drawDetailSubplotDualY()` 双 Y 轴子图模板方法 | 为预处理管线全断面离线排障提供统一可视化视图 |
| 2026-07-07 | `PreprocessingPipelineResult` / `VehicleFootprintModel` | 复审修复：`PreprocessingPipelineResult` 新增 `original_z_ref`、`hard_margin_used`、`soft_margin_used`、`outer_row_num_used`；`VehicleFootprintModel` 新增 `getOuterRowNum()` | 使 Visualizer 侧的原始路径绘制、slack_true / 隔离墙计算与 PreprocessingPipeline 实际使用的参数保持一致 |
| 2026-07-08 | `Visualizer` | 预处理管线诊断视图布局从 2x3 重构为 2x2：第一列完整显示空间几何；第二列按 s-domain / index-domain 分上下两行，每行内多条曲线垂直堆叠且各自拥有独立 Y 轴；移除原 Subplot 6 静态走廊 slack 对比图 | 避免不同量级信号共享 Y 轴导致的互相压缩，提升 spatial 图的空间利用率 |
| 2026-07-10 | `PreprocessingPipelineConfig` / `BSplineSmootherConfig` / `StaticCorridorBuilderConfig` / `NmpcSolverConfig` / `AdaptiveResamplerConfig` | 碰撞检测重大更新：移除物理安全裕度（`collision_safety_margin`/`collision_margin`/`hard_margin`/`esdf_safety_margin`/`obstacle_density_margin` 默认值从 5cm/5cm/5cm/20cm/5cm 统一改为 **0**）；`collision_validation_tolerance` 从 2cm 改为 `1e-4`（0.1mm，适配三次方碰撞惩罚的梯度消失）；新增 `kCollisionEpsilon = 1e-6` 常量用于浮点安全比较。`soft_margin`（18cm）独立保留用于降低乘员压迫感 | 外圆（Outer Circles）本身已超出车辆矩形轮廓边界，无需额外叠加物理安全裕度；`soft_margin` 继续提供舒适缓冲。详见 [docs/NMPC.md](NMPC.md) 3.1 节"碰撞检测数值容差的设计原则" |
| 2026-07-10 | `Visualizer` | `plotPipelineDiagnostics()` 新增可选 `VehicleFootprintModel*` 参数；index-domain 子图在传入 footprint_model 时绘制 `Z_ref` 每点处所有外圆的最小安全余量 `min(d_esdf - r)`，并叠加 `0m` 与 `soft_margin` 参考线，未传入 footprint_model 时回退到绘制 `delta_t` | 满足离线排障时对每个预处理采样点安全余量的直接观测需求；保持向后兼容，旧调用方无需改动 |
| 2026-07-08 | `ESDFMap` | 内部实现优化（SoA→AoS 数据布局、双线性插值索引预计算、`std::floor`→`static_cast<int>`、内部计算 float 化），**公开接口签名不变**（`getDistAndGrad`/`getDist` 返回类型保持 `std::pair<double, Eigen::Vector2d>`/`double`） | 查询热路径性能优化，不影响冻结接口契约，下游调用方无需任何改动 |
| 2026-07-09 | `BSplineSmootherConfig` | `lbfgs_max_iterations` 默认值从 100 收紧到 80，`lbfgs_max_linesearch` 默认值从 100 收紧到 20 | 按调参-验证-回滚流程实测：100/100 基线 → OMP 并行化 → 单变量单调收紧；`max_iterations` 在 50 时触发测试失败，在 60 时于四数据集之一的 data7 出现 199.44um 侵入深度回退，最终确定为 80/20；`max_linesearch` 在 20 时仍满足 Safety Gate |
| 2026-07-09 | `stc_SQP::CostTerm` | 新增 `evaluateGradientAndHessian()` 组合求值接口（默认向后兼容转发）与 `clone()` 纯虚方法；`CircleFootprintEsdfPenaltyCost`/`CompositeCost` 覆写组合求值以消除重复 ESDF 查询，`QuadraticTrackingCost` 实现 `clone()`；`SQPSolver::assembleCost()` 优先调用组合求值接口 | 为 `assembleQP()` OpenMP 并行化提供与 `Constraint::clone()` 对称的 cost 克隆能力，并消除 `CircleFootprintEsdfPenaltyCost` 每步每次 SQP 迭代 2~3 次的 ESDF 重复查询 |
| 2026-07-09 | `stc_SQP::SQPSolver` | 新增 `thread_cost_clones_` 与 per-thread `CostScratch`，`assembleQP()`/`assembleCost()` 补齐与 `linearize()` 对称的 OpenMP 并行路径（沿用 `options_.omp_parallel_threshold` 阈值） | 将 `linearize()` 已有的 OMP 并行基础设施复用到 cost 装配阶段 |
| 2026-07-09 | `stc_SQP::HPIPMQPSolver` | `setWarmStart()` 从空实现改为真正的 HPIPM IPM primal 热启动；`solve()` 内部自动维护上一次成功解的缓存，维度不匹配或求解失败时退化/清空为冷启动；新增 `lastIterations()`/`totalIterations()` 用于读取 IPM 迭代统计 | 在 Full SQP 循环中跨迭代复用上一次 QP 解作为 IPM 初值，降低 IPM 迭代次数 |
| 2026-07-09 | `stc_SQP::SQPSolver` | `SQPSolverOptions` 新增 `use_qp_warm_start` 开关（默认 `false`）；`solveQP()` 在 Full SQP 非首次迭代且开关开启时调用 `qp_solver_->setWarmStart(qp_solution_)` | 将跨迭代热启动接入 Full SQP 主循环；RTI 单步模式不受影响 |
| 2026-07-09 | `PathToOcpConverter` / `SegmentProfile` | 拆分 `PathToOcpConverter` 职责。新增 `SegmentProfile` 结构体描述单段离散化；新增 `computeSegmentProfiles()` / `generateInitialGuess()` / `buildOcp()` 三个公开入口；`convert()` 内部调用三者并保持原有行为；`buildSegment()` 签名改为接受 `SegmentProfile` + `terminal_x_ref` + `theta_refs`，与初始猜测来源解耦 | 为后续接入预处理管线产出的 Z_ref/U_ref/非均匀 delta_t 提供清晰扩展点，避免复制粘贴约束注入逻辑 |
| 2026-07-09 | `NmpcSolver` | 新增 `optimize(const MultiStageOCP&, const Trajectory&, const ESDFMap&)` 重载；原 `optimize(Path, ESDFMap)` 内部调用 `PathToOcpConverter::convert()` 后转发到该重载；约束注入（信赖域/静态走廊/动态 ESDF 软代价）与求解逻辑下沉到 `solveOcp()` 共享 | 提供不修改公开 `Path` 入口即可注入预装配 OCP/初始猜测的最小侵入式扩展点 |
| 2026-07-09 | `PreprocessingToOcpConverter` | 新增预处理管线输出 → OCP 转换器，直接消费 `PreprocessingPipelineResult` 的 `z_ref`/`delta_t`，把非均匀步长通过 `StageSegment::dt_array` 接入；同时按 OCP 总步数截断 `c_matrix`/`d_vector`，与 `NmpcSolver` 的 `expected_corridor_rows` 对齐 | 把预处理管线产出真正当作 Warm Start 喂给 `NmpcSolver`，替代 `PathToOcpConverter` 的简化三次多项式猜测 |
| 2026-07-09 | `src/main.cpp` | 生产入口切换为 `Path → PreprocessingPipeline → PreprocessingToOcpConverter → NmpcSolver` 完整链路；新增"NMPC 失败但预处理成功时回退到预处理轨迹"兜底，日志/`OptimizeResponse.message` 明确区分四态 | 完成预处理管线与 NMPC 的端到端接线，满足用户需求"如果优化失败但是预处理成功可以考虑用预处理的路径" |
| 2026-07-09 | `AdaptiveResampler::assembleFinalTrajectory` | 修复相邻常规段空间连续但无换挡补丁时，最终 `delta_t.size()` 比 `points.size() - 1` 少 `segment_count - 1` 的问题；当相邻段终点/起点距离小于 1e-3m 时自动补一条连接 `delta_t` | 保证真实泊车数据（段间共享换挡点）下 `PreprocessingPipelineResult.delta_t` 长度与 `z_ref` 严格匹配，使下游 `PreprocessingToOcpConverter` 可直接消费非均匀 `dt_array` |
| 2026-07-09 | `PreprocessingToOcpConverter` | 按 `z_ref` 速度符号将单段 OCP 拆分为多段 `MultiStageOCP`；每段独立生成 `dt_array`、stage_params、终端跟踪代价与单向速度箱约束；静态走廊系数按总 OCP 步数截断 | 解决多 maneuver 真实数据上 NMPC 首迭代 QP 不可行问题，允许 Warm Start 在保持方向一致的各段内满足箱约束 |
| 2026-07-09 | `PostProcessor` / `AdaptiveRetryConfig` | 新增 `PostProcessor` 完整链路封装与 `AdaptiveRetryConfig`；默认参数 NMPC 完全失败时，临时拉大 `AdaptiveResamplerConfig::nominal_step_s`（默认 ×2、×3）重试，并在最后一次重试关闭静态走廊作为兜底；所有重试只修改局部配置副本 | 满足用户需求"预处理间隔拉长重试、用完必须恢复默认值"，并保证调用方默认配置不被污染；解决 data6.json 静态走廊与 Warm Start 首迭代不可行问题 |
| 2026-07-15 | `NmpcSolverConfig::esdf_penalty_weight` | 语义从"与 `static_corridor_C/d` 互斥的替代机制"改为"与静态走廊协同的梯度引导机制"——`NmpcSolver::solveOcp()` 内部触发条件从 `!use_static_corridor && esdf_penalty_weight > 0` 放宽为 `esdf_penalty_weight > 0`（不再要求走廊未启用）。默认值仍为 0（关闭），需按数据集显式调参开启 | 用户明确需求：静态走廊硬约束保证"每次 SQP 迭代无论是否收敛都安全（不穿模）"，ESDF 直接代价提供真实非线性梯度以加速/改善收敛，两者互不冲突（代价只影响目标函数，不影响可行域），不应互斥 |
| 2026-07-19 | `src/core/ALM/`（新模块，规划中） | 新增"待实现：ALM 模块核心接口"一节，登记 `BlockTridiagonalSolver`/`MincoTrajectory`/`BicycleKinematicsExtractor`/`AlmManeuverSegmenter`/`AlmEsdfPenalty`/`AlmPreprocessor`/`AlmSolver` 等规划中接口的目标头文件路径与设计意图，对应 [docs/ALM.md](ALM.md) 与 [docs/milestones.md](milestones.md) `milestone-001`~`milestone-008` | 与 [docs/architecture.md](architecture.md) 3.7 节同步：ALM 是与 NMPC 并列的第二条后处理算法路径，尚未创建任何源文件，先落地接口规划供 Milestone 拆分与后续 Dev Agent 实现参考 |
| 2026-07-15 | `NmpcSolver::solveOcp` | `ng_max` 计算从手工按约束类型硬编码行数累加改为直接调用 `stc_SQP::strategy_internal::computeOcpNgMax(mutable_ocp)`，对实际已装配的各段约束求和取最大值 | 手工累加公式曾遗漏"某类终端约束实际是否被注入"的条件判断（`terminal_position_error_threshold`/`terminal_heading_error_threshold_deg` <= 0 时 `TerminalPoseBoxConstraint` 不再注入，但原公式仍无条件加 6 行），导致 `qp_data.ng_max` 与 HPIPM 构造时的 `ng` 不一致，触发 `QPSolverStatus::INVALID_ARGUMENT`（诊断 data6.json 时复现）；改为动态求和后消除整类潜在的维度不一致 bug |
| 2026-07-15 | `StaticCorridorBuilder::computeDScalar` | 内部实现新增"自洽性修正"：当参考点 `Z_ref` 自身已违反安全边界（`dist_ref < radius + margin`，多发生于 `BSplineSmoother` 碰撞预推未能完全消除侵入的极端困难场景）时，为该约束行补偿等量违反深度 `violation = max(0, radius + margin - dist_ref)`，使约束在 `Z = Z_ref` 处恰好取等号而非直接不可行；`dist_ref >= radius + margin` 时 `violation = 0`，与原公式完全一致，不影响既有已验证场景 | 原公式在参考点已侵入时会产出连 Warm Start 自身都无法满足的硬约束，导致 HPIPM 在第 0 次 SQP 迭代即失败；修正后保证"线性化安全裕度不允许比参考点当前状态更差"，即使无法达到理想安全裕度也不会让 HPIPM 因起点不可行而直接拒绝求解 |
| 2026-07-15 | `PostProcessor::runSingleAttempt` | 内部实现修复：NMPC 求解失败或碰撞超标回退到预处理轨迹的两条分支，此前直接返回未经拓扑清洗的 `Path`，现统一改为调用与 NMPC 成功路径共享的 `applyTopologyCleanup` 辅助 lambda | 修复回归 bug：预处理轨迹中的换挡/起始转向零速补丁段会被 `Path::finalize()` 按方向变化拆分成比原始路径更多的 maneuver（实测 data1.json 曾回退产出 12 段，超过初始 10 段），直接违背"机动段数不劣化"的验收要求 |
| 2026-07-15 | `NmpcSolverConfig::max_iter` | 默认值从 1000 收紧回 300 | 已在四数据集上验证 300 次迭代内可获得可用结果；过大上限（1000/2000）会把单次求解耗时推高到分钟级且未观察到额外解质量收益，拖慢调参迭代反馈速度 |
| 2026-07-15 | `NmpcSolver::solveOcp` / `NmpcSolverConfig` | `ThetaTrustRegionConstraint`/`PositionTrustRegionConstraint` 的注入条件从"必须启用静态走廊"改为"仅取决于配置值本身 > 0"，任意走廊模式（静态/迭代 ESDF）下均生效；`max_position_deviation_from_ref` 默认值从 0.0（关闭）改为 0.15（历史调参验证值） | 修复"NMPC 优化后轨迹在内部机动段呈折线/直线"的视觉缺陷：内部机动段代价矩阵仅跟踪 v/delta、不跟踪 x/y/theta，唯一能把中间打靶点约束在参考曲线附近的机制此前被硬编码为仅静态走廊模式下生效，导致回退到迭代走廊时完全不受约束 |
| 2026-07-15 | `PathToOcpConverter` | `buildSegment()` 签名新增 `x_refs`/`y_refs` 参数（`protected` 内部方法，非公开冻结接口），与已有 `theta_refs` 并列传入，用于填充 `stage_params.p(3)/p(4)` | 此前该入口从未填充 p(3)/p(4)，若调用方启用 `PositionTrustRegionConstraint` 会错误地把轨迹约束到坐标原点附近；修复后与 `PreprocessingToOcpConverter` 行为一致 |
| 2026-07-15 | `NmpcSolver::solveOcp` / `NmpcSolverConfig` | 移除 `TerminalPoseBoxConstraint`/`TerminalFinalStateConstraint` 的注入逻辑，`use_terminal_final_state_constraint` 字段随之删除；终端精度改为完全依赖终端跟踪代价（`terminal_position_weight`/`terminal_heading_weight = 1e5`）。`terminal_position_error_threshold`/`terminal_heading_error_threshold_deg` 字段保留，语义收窄为"仅质量门阈值"，不再驱动任何硬约束注入 | 用户明确要求"放开一切束缚"重新设计：诊断确认静态走廊 + 终端位姿盒 + 终端状态硬约束 + 信赖域四类硬约束同时作用于终端区域是 HPIPM `UNKNOWN_ERROR` 的主要诱因之一；移除终端硬约束后 `data1`/`data3` 不再触发该错误，求解成功率与速度显著提升，详见 `docs/NMPC.md` 6.6 节 |
| 2026-07-15 | `PostProcessor::runSingleAttempt`（剪枝内部实现） | 剪枝候选的求解从单一配置改为多组局部配置重试（沿用与 `solveFullPipeline` 相同的 `PreprocessingPipelineConfig` 参数化），任一组成功即接受 | 诊断发现剪枝失败的真正瓶颈在预处理层（B样条平滑）而非 NMPC/HPIPM，本改动作为兜底保留，主要修复见下一行 `BSplineSmoother` 变更 |
| 2026-07-15 | `BSplineSmoother`（内部实现，非公开签名变更） | L-BFGS 精修失败时的回退目标从"碰撞预推之前的 `initial_x`"改为"碰撞预推之后的检查点 `prepushed_x`"；"双倍碰撞权重重试"步骤的出发点保持 `initial_x` 不变 | 修复真实 bug：预推能把新合并几何的侵入深度从米级压到毫米级，但精修失败时错误地把这部分进度整体丢弃，导致碰撞校验永远针对未逃生的原始高侵入深度判定失败，使预处理管线在"实际已经足够安全"的场景下无谓拒绝、连累 NMPC 根本没有机会运行。详见 `docs/NMPC.md` 6.6.3/6.6.4 节与 `docs/known-limitations.md` 对应条目 |
| 2026-07-15 | `PostProcessor::runSingleAttempt` | 彻底删除机动段剪枝循环（`RemoveShortestManeuver` 调用、`prune_retry_configs`、`solvePrunedWithRetries`、四重门禁判定），简化为单次 `solveFullPipeline` 调用 | 用户明确要求放弃"预处理层盲剪+全链路重跑验证"的段数削减机制（已证明效果不佳），改由 NMPC 内生顺滑机制（见下一行）承担段数削减 |
| 2026-07-15 | `util::topology_cleaner`：`RemoveShortestManeuver` | 函数本体删除（`ClassifyAndResetManeuvers`/`ReconstructPath` 保留） | 同上；该函数仅服务于已删除的剪枝循环，无其它调用方 |
| 2026-07-15 | `PreprocessingToOcpConverter` / `NmpcSolver` | 状态维度从 5 扩展为 7：新增 `BicycleModelJerk` 动力学模型（本项目自定义，不修改 `third_party/StcSQP`），`a`、`delta_dot` 从控制量升级为状态量，新增控制量 `[jerk, ddelta_dot]`；`ThetaTrustRegionConstraint`/`PositionTrustRegionConstraint`/`StaticCorridorLinearConstraint`/`IterativeCorridorConstraint` 的 `Cx` 构造改为按 `x.size()` 动态适配（兼容 5 维/7 维两种入口） | 用户要求真正实现文档 4.2 节设计的 $J_{smooth}$（跨 stage 控制量差分代价），让 NMPC 求解本身具备"熔化冗余换挡"的顺滑压力，取代预处理层剪枝；控制量升阶（rate-as-input）是实现跨 stage 差分代价的标准工业界做法，无需交叉 Hessian 或修改第三方框架 |
| 2026-07-15 | `PathToOcpConfig`（`path_to_ocp_converter.h`） | 新增 `max_jerk=10.0`、`max_steer_angular_accel=5.0`、`smoothing_jerk_weight=1e-1`、`smoothing_steer_accel_weight=1e-1` 四个字段（仅 `PreprocessingToOcpConverter` 消费）；`accel_limit`/`steer_rate_limit` 复用为状态 box bound（语义不变，物理极限从控制量平移到状态量） | 承接上一行状态增广改动 |
| 2026-07-15 | `IterativeCorridorConstraint`/`StaticCorridorLinearConstraint`（内部实现） | 修复两处硬编码 `Cx = Matrix::Zero(ng, 5)` 的遗留 bug，改为按实际状态维度动态构造 | 状态增广后暴露的真实 bug，此前从未被端到端测试覆盖（现有测试只检查转换结果维度，未真正驱动一次完整求解），详见 `docs/known-limitations.md` 对应条目 |
| 2026-07-15 | `NmpcSolverConfig`/`PositionTrustRegionConstraint`/`NmpcSolver::solveOcp` | 位置信赖域从硬约束改为 HPIPM 原生软约束（纯 L2 二次跟踪代价）：新增 `position_tracking_weight` 字段（对应 $W_x=W_y$）；`max_position_deviation_from_ref` 语义从"硬约束宽度"改为"软代价死区宽度"，默认值从 0.15 改为 0.01；`PositionTrustRegionConstraint` 构造函数校验从"必须 > 0"放宽为"必须 >= 0" | 理论分析发现硬信赖域是非黑即白的可行性门槛，会一票否决"偏移量略超阈值但更优"的机动段压缩方案，直接阻断 $J_{smooth}$ 的段数削减效果；改为软代价后单元测试验证了理论（合成场景轨迹压缩 65.5%），但四数据集实测仍未达成段数削减且 `data7` 终点精度出现回归，详见 `docs/NMPC.md` 6.8 节 |
| 2026-07-15 | `NmpcSolverConfig`/`ThetaTrustRegionConstraint`/`NmpcSolver::solveOcp` | 航向信赖域同样从硬约束改为 HPIPM 原生软约束（与位置跟踪同一套机制）：新增 `theta_tracking_weight` 字段（对应 $W_\theta$）；`ThetaTrustRegionConstraint` 构造函数校验从"必须 > 0"放宽为"必须 >= 0" | 不再依赖静态走廊，航向信赖域"服务走廊线性化有效性"的存在理由随之消失，改为与位置跟踪一致的软代价跟踪，详见 `docs/NMPC.md` 6.9 节 |
| 2026-07-15 | `PreprocessingPipelineConfig::use_static_corridor` / `PostProcessor::AdaptiveRetryConfig::use_static_corridor_flags` | 默认值分别从 `true`/`{true, false}` 改为 `false`/`{false, false}` | 默认弃用静态走廊，NMPC 侧回退到每轮重新线性化的 `IterativeCorridorConstraint` 作为唯一碰撞安全硬约束来源（保留迭代走廊为硬约束）；四数据集实测未实现段数削减，但意外修复了 `data3` 长期存在的 HPIPM `UNKNOWN_ERROR`，详见 `docs/NMPC.md` 6.9.3 节 |
| 2026-07-15 | `NmpcSolverConfig::position_tracking_weight`/`theta_tracking_weight` | 默认值从 `1e4` 改为 `0.0`（默认完全关闭 $J_{process}$ 持续跟踪参考轨迹 $Z_{ref}$ 的机制，代码/`Constraint` 类本身保留） | 经论文对照验证：持续跟踪包含冗余换挡的粗参考轨迹本身，与"熔化冗余换挡"目标方向相反；此前阶段只软化了该机制的强制程度，从未质疑其存在的合理性，详见 `docs/NMPC.md` 6.10 节 |
| 2026-07-15 | `PathToOcpConfig`（`path_to_ocp_converter.h`） | 新增 `global_target_position_weight=1e-3`、`global_target_heading_weight=1e-3` 两个字段：对**每一个**打靶步（含终端段）施加向**常量**停车目标位姿的二次牵引代价，复用已有的 `QuadraticTrackingCost`；`PreprocessingToOcpConverter::buildSegment`/`PathToOcpConverter::buildSegment`（后者新增 `global_target_x_ref` 参数）均已接入 | 参考 Zhang et al.《Automatic parking trajectory planning in narrow spaces based on Hybrid A* and NMPC》(Sci Rep 2025) Eq.(10) 的 $J_1$ 空间占用代价设计；因目标位姿是常量，不需要逐步变化的 `StageParameters::p`，无需扩展 `third_party/StcSQP` 接口、无需状态增广、无需改动走廊，详见 `docs/NMPC.md` 6.10 节 |
| 2026-07-15 | `tool/tune_post_processor.cpp`：`TuneVariant` | 新增 `global_target_position_weight`/`global_target_heading_weight`/`interior_speed_weight` 三个字段并接入 `RunSingleDataset`；`BuildVariants()` 从单一 `"base"` 变体扩展为 14 组权重扫描变体 | 支撑一轮四数据集广泛调参，验证得到跨数据集安全的默认权重 0.001，详见 `docs/NMPC.md` 6.10.4 节 |

## 待办：NMPC 安全机制重构相关的计划中接口变更

| 计划变更 | 接口 | 原因 |
|---|---|---|
| `data6.json` 静态走廊路径首迭代 HPIPM `UNKNOWN_ERROR`（状态码 5）根因排查 | `NmpcSolver`、`third_party/StcSQP::HPIPMQPSolver` | 已排除 ng_max 维度不一致、终端约束、信赖域约束三类假设（三者关闭后仍复现同一 `UNKNOWN_ERROR`），根因仍未定位，疑似 HPIPM IPM 内部数值条件问题；按 debug-circuit-breaker 约定暂停继续排查，留待后续专项调试（未解决，见 `docs/known-limitations.md`） |

## 待办：预处理管线相关的计划中接口变更

| 计划变更 | 接口 | 原因 |
|---|---|---|
| ~~碰撞安全裕度参数统一~~ | ~~`BSplineSmootherConfig`、`StaticCorridorBuilderConfig`~~ | ✅ 已完成 |
| ~~静态走廊信赖域约束注入~~ | ~~`NmpcSolverConfig`、`NmpcSolver`~~ | ✅ 已完成。`max_theta_deviation_from_ref` 字段已新增（默认 0.0=关闭）；`ThetaTrustRegionConstraint` 约束类已实现并通过单元测试；`NmpcSolver::optimize()` 已接线 |
| ~~HPIPM 层静态走廊约束注入~~ | ~~`NmpcSolver`~~ | ✅ 已完成。`StaticCorridorLinearConstraint` 约束类已实现并通过单元测试；`NmpcSolver::optimize()` 在 `static_corridor_C/d` 提供时自动注入，否则兜底使用 `esdf_penalty_weight` 动态 ESDF 软代价 |
| ~~`main.cpp` 生产入口接入 `PreprocessingPipeline`~~ | ~~`src/main.cpp`、`NmpcSolver`~~ | ✅ 已完成：`main.cpp` 已切换为 `Path → PreprocessingPipeline → PreprocessingToOcpConverter → NmpcSolver` 完整链路，并实现了"NMPC 失败但预处理成功时回退到预处理轨迹"的四态判定 |
| ~~StcSQP 热循环基础设施优化（`-march=native` + `CostTerm` 组合求值 + `assembleQP` 并行化）~~ | ~~`stc_SQP::CostTerm`、`stc_SQP::SQPSolver`、`third_party/StcSQP/CMakeLists.txt`~~ | ✅ 已完成。三处内部实现优化均已落地：`CostTerm` 新增 `evaluateGradientAndHessian()`/`clone()`，`CircleFootprintEsdfPenaltyCost`/`CompositeCost`/`QuadraticTrackingCost` 实现对应方法；`SQPSolver` 新增 `thread_cost_clones_` 与 per-thread `CostScratch`；`stc_SQP_core_lib` 在 x86_64 平台启用 `-march=native` |
| HPIPM IPM 跨迭代热启动 | `stc_SQP::HPIPMQPSolver`、`stc_SQP::SQPSolver` | ✅ 已完成。已实现 primal-only 跨迭代热启动与 `use_qp_warm_start` 开关；benchmark 显示在 `BM_Data3RealScenario_MultiSegmentBicycleCorridor` 场景下总 IPM 迭代数与总耗时均未出现可观测收益，因此默认关闭，作为可选实验开关保留 |
| ~~`Constraint`/`CostTerm` 虚函数多态迁移到 `std::variant`~~ | ~~`stc_SQP::StageSegment`、`stc_SQP::Constraint`、`stc_SQP::CostTerm`~~ | ✅ 已评估不采纳。Release 基准 `std::variant` 802ms vs 虚函数 726ms 处于同一量级无收益；结合 closed-set 维护成本，最终决定不合入，保留原有虚函数机制 |
| ~~`lbfgs_max_iterations`/`lbfgs_max_linesearch` 默认值收紧~~ | ~~`BSplineSmootherConfig`~~ | ✅ 已完成。`lbfgs_max_iterations` 100→80，`lbfgs_max_linesearch` 100→20，详见上方变更记录 |
| ~~预处理管线可视化诊断入口~~ | ~~`Visualizer`（新增 `plotPipelineDiagnostics`）、`DetailSeriesData`（扩展 `v`/`a`/`delta`/`delta_dot`/`delta_t`/`slack_true`/`slack_calc` 字段）、`PreprocessingPipelineConfig`（新增 `enable_debug_output` 开关）、`PreprocessingPipelineResult`（新增 `debug_maneuver_outputs` 容器）~~ | ✅ 已完成。`plotPipelineDiagnostics` 已按 2x3 子图渲染；`slack_true` 与 `slack_calc` 在真实数据集上数值吻合；单元测试覆盖开关行为、空输入死区保护、双 Y 轴量程、OBB 包络点数、slack 对比 |

## 待办：NMPC 算法最终重构相关的计划中接口变更

> 本表登记"NMPC 算法最终重构"系列尚未实现的接口变更意图；具体类名/方法签名由 Dev Agent 在实施时最终确定，
> 落地后需将本表对应行改为完成态（比照上一张表的 ✅ 划线惯例），并在下方"变更记录"补充正式条目，不得只在本表登记而不同步。

| 计划变更 | 接口 | 原因 |
|---|---|---|
| ~~`NmpcSolver` 新增接受预装配 OCP/初始猜测的扩展点~~ | ~~`NmpcSolver`（`src/core/NMPC/nmpc_solver.h`）~~ | ✅ 已完成：新增 `optimize(const MultiStageOCP&, const Trajectory&, const ESDFMap&)` 重载，约束注入与求解逻辑下沉到 `solveOcp()` 共享 |
| ~~`PathToOcpConverter` 职责边界复核（拆分"OCP 结构装配"与"初始猜测生成"）~~ | ~~`PathToOcpConverter`（`src/core/NMPC/path_to_ocp_converter.h`）~~ | ✅ 已完成：新增 `SegmentProfile` + `computeSegmentProfiles()` / `generateInitialGuess()` / `buildOcp()`，使 OCP 结构装配与初始猜测生成解耦 |
| ~~新增"预处理管线输出 → OCP"转换路径，消费非均匀 `delta_t`~~ | ~~新增类型（具体命名由 Dev Agent 确定），`stc_SQP::StageSegment::dt_array`~~ | ✅ 已完成：新增 `PreprocessingToOcpConverter`，把 `PreprocessingPipelineResult` 的 `z_ref`/`delta_t`/`c_matrix`/`d_vector` 直接作为 Warm Start，通过 `StageSegment::dt_array` 接入非均匀步长 |
| ~~`main.cpp`/`NmpcSolver` 的"NMPC 失败回退预处理轨迹"三/四态判定~~ | ~~`src/main.cpp`、`NmpcSolver`~~ | ✅ 已完成：`main.cpp` 中实现"NMPC 收敛"/"NMPC 未收敛但仍返回最新迭代"/"回退到预处理轨迹"/"整体失败"四态，日志与 `OptimizeResponse.message` 明确区分 |
| ~~自适应间隔重试机制（"拉长间隔重试、用完必须恢复默认值"）~~ | ~~`PreprocessingPipelineConfig`/`BSplineSmootherConfig::dense_step_dist` 或 `AdaptiveResamplerConfig::nominal_step_s`，新增 `PostProcessor`/`AdaptiveRetryConfig`~~ | ✅ 已完成：实测选定 `AdaptiveResamplerConfig::nominal_step_s` 作为重试调节对象（`dense_step_dist` 放大易触发 `dt_array` 非正异常）；`PostProcessor::optimize()` 内部使用局部配置副本，调用方默认配置保持不变；单元测试 `PostProcessorTest.DoesNotLeakConfigAfterRetry` 直接验证 |

## 待实现：ALM 模块核心接口（新模块，milestone-001~008 规划）

> 与 [docs/architecture.md](architecture.md) 3.7 节对应：`src/core/ALM/` 是与 `src/core/NMPC/` 并列的第二条
> 后处理算法路径，目前**尚未创建任何源文件**，下表登记的是 Milestone 拆分阶段的接口设计意图与目标头文件路径，
> **状态一律为"规划中"，不是"已冻结"**——本表不受 [.agents/rules.md](../.agents/rules.md)
> "接口一经冻结禁止擅自修改签名"红线约束，Dev Agent 在对应 Milestone 实现时可按实际情况调整具体方法签名，
> 但类名与核心职责应与本表保持一致（如需变更，先在此处登记原因）。每个 Milestone 完成并通过 Review 收敛后，
> 应将对应行从本表移除并正式登记进上方"已冻结接口清单"，同步补充"变更记录"（比照既有"待办"表格的 ✅ 划线惯例）。

| 接口 | 设计意图（一句话） | 计划头文件 | 对应 ALM.md 章节 | 对应 Milestone |
|---|---|---|---|---|
| `BlockTridiagonalSolver` | 固定 6x6（$h=3$）块的块 Thomas 算法通用求解器，栈上分配、$O(M)$ 前向消元+回代，供 `MincoTrajectory` 装配 $K(T)c=b$ 时复用，替代 `Eigen::SparseMatrix`+`SparseLU` | `src/core/ALM/block_tridiagonal_solver.h` | 2.6.2 | milestone-001 |
| `MincoTrajectory` | $\theta_i(t)/s_i(t)$ 多项式段表示；封装 $K(T)$ 装配（含局部归一化时间 $\tau=t/T_i$）、边界条件注入、系数 $c$ ↔ 中间控制点 $^w\sigma'$/时间 $T$ 转换、$\tau\leftrightarrow T$ 分段光滑双射及其导数、终点弧长 $s_f$ 的 $K(T)^{-T}$ 伴随梯度 | `src/core/ALM/minco_trajectory.h` | 1.2 | milestone-001 |
| `BicycleKinematicsExtractor` | 从 `MincoTrajectory` 的 $\theta,s$ 各阶导数解析阿克曼状态/控制量 $v,a,\delta,\dot\delta$（含 $\epsilon_g$ 分母正则化，值与梯度一致处理）与防奇异二次形态约束惩罚 $\mathcal{C}_v/\mathcal{C}_a/\mathcal{C}_\delta/\mathcal{C}_{\dot\delta}$ 及各自解析梯度 | `src/core/ALM/bicycle_kinematics_extractor.h` | 2.2 / 2.3 | milestone-002 |
| `AlmManeuverSegmenter` | 复用 `Path`/`Maneuver`，实现换挡打断（宏观段，$\dot s_k=0$ 硬边界识别）与空间等距降采样（微观段），产出初始 $M$ 段估计（$p_{w0}^m$/$s_m$/$\theta_m$/$\tau_m$） | `src/core/ALM/alm_maneuver_segmenter.h` | 2.1 | milestone-003 |
| `AlmEsdfPenalty` | 复用 `ESDFMap`/`VehicleFootprintModel` 外圆集合，实现 `margin_safe`/`margin_comf` 双重松弛罚函数 $\mathcal{C}_{safe}/\mathcal{C}_{comf}$、混合代价 $\mathcal{I}_{obs}$ 与对 $x,y,\theta$ 的解析梯度反传 | `src/core/ALM/alm_esdf_penalty.h` | 2.4 | milestone-004 |
| `AlmPreprocessor` | 两阶段优化流程的第一阶段：基于运动学/加速度/段时长平衡约束 + 逐段终点跟踪惩罚构建 $\mathcal{J}_{pre}$，松收敛阈值下把初值拉近前端路径 | `src/core/ALM/alm_preprocessor.h` | 1.2 / 2.1 | milestone-005 |
| `AlmSolver` | 与 `NmpcSolver` 并列的求解器编排入口（类名已定案，与仓库命名惯例 `XxxSolver` 一致）：内层 L-BFGS 优化 $(^w\sigma',\tau,s_f)$（复用 `third_party/LBFGSpp`）+ 外层 PHR-ALM 乘子/惩罚权重更新、$\rho^0$ 自适应标定、位置/朝向双指标收敛判据，输出满足终点精度与无碰撞/无奇异要求的解析轨迹 | `src/core/ALM/alm_solver.h` | 1.4 / 2.5 | milestone-006 |
| 机动融化拓扑修剪钩子 | 融化收敛后的 REV 段压平/PIVOT 保留判据，直接复用 `util::topology_cleaner` 两遍分类算法与判据结构（仅重新标定量纲/阈值），"绝不合并方向相反相邻段"红线与 NMPC 侧一致；具体挂载点（`AlmSolver` 内部方法 or 独立类）由 Dev Agent 在 milestone-007 实现时确定 | 待定（`src/core/ALM/` 下，具体文件名由 milestone-007 确定） | 2.6.3 | milestone-007 |
| `PostProcessor` 接入 ALM 路径的扩展点 | 让已冻结的 `PostProcessor` 新增可选走 `AlmSolver` 的路径（与现有 NMPC 路径并列，非替换），具体方法签名/开关形式由 Dev Agent 在 milestone-008 实现时依据当时的 `PostProcessor` 实际接口确定，避免此处提前臆造签名与实际冻结接口冲突 | `src/core/post_processor.h`（已冻结文件，milestone-008 需在此基础上扩展并同步登记变更记录） | 全部 | milestone-008 |


