# APAPostProcessor

APA路径规划后处理器

## 1. 构建与运行

### 1.1 环境要求

- **编译器**：支持 C++17 的 GCC（≥8）或 Clang（≥10）
- **CMake**：≥ 3.5
- **Ninja**（可选，推荐）：构建加速

### 1.2 第三方依赖

| 依赖 | 安装方式 | 备注 |
|---|---|---|
| **Protobuf** | ≥ 3.15（需支持 proto3 `optional` 关键字） | 必须 |
| **OpenCV** | `apt install libopencv-dev` | 必须 |
| **Eigen3** | `apt install libeigen3-dev` | 必须 |
| **nlohmann\_json** | `apt install nlohmann-json3-dev` | 必须 |
| **OpenMP** | 随编译器安装（GCC/Clang 自带） | 必须 |
| **CasADi (Python)** | `pip3 install casadi`（用于 StcSQP 动力学 / 走廊 C 代码生成） | 必须 |
| **Python** | ≥ 3.8（CasADi 脚本需 `python3`） | 必须 |
| **GTest** | `apt install libgtest-dev` | 可选（无则跳过单元测试） |
| **Google Benchmark** | `apt install libbenchmark-dev` | 可选（无则跳过性能测试） |
| **ccache** | `apt install ccache` | 可选（编译缓存加速） |

以下库已内置在 `third_party/` 中，无需手动下载：

| 库 | 位置 | 用途 |
|---|---|---|
| **BLASFEO** | `third_party/blasfeo/` | 高性能线性代数（HPIPM 依赖） |
| **HPIPM** | `third_party/hpipm/` | 结构 QP 求解器（StcSQP 底层后端） |
| **OSQP** | `third_party/osqp/` | 二次规划求解器 |
| **StcSQP** | `third_party/StcSQP/` | SQP 数值优化框架 |
| **LBFGSpp** | `third_party/LBFGSpp/` | 无约束/箱式约束优化 |
| **Quill** | `third_party/quill/` | 异步日志库 |

### 1.4 运行主程序

```bash
# Debug 版
./build/Debug/apa_post_processor

# Release 版
./build/Release/apa_post_processor
```

主程序默认读取 `data/config.json` 中 `data_file_path` 指定的输入数据，处理后输出优化轨迹。

### 1.5 运行单元测试和性能基准测试

编译后运行

```bash
test_apa_post_processor
bench_apa_post_processor
```

---

## 2. 项目结构

```
.
├── src/            # 核心源码
│   ├── core/       # NMPC 求解器、后处理器
│   ├── preprocessing/  # 预处理管线（B样条/速度规划/走廊）
│   ├── spatial/    # ESDF 地图、网格
│   ├── util/       # 数据加载、日志、路径工具
│   └── vehicle/    # 车辆参数与覆盖模型
├── test/           # 单元测试（GTest）
├── bench/          # 性能基准（Google Benchmark）
├── tool/           # 辅助脚本
├── data/           # 测试数据集
├── docs/           # 设计文档
├── proto/          # Protobuf 协议定义
└── third_party/    # 第三方源码
```

## NMPC

基于NMPC思想优化HybridA*等方法生成的初始轨迹，这里提供了三个场景四个用例：

**long_park**：长距离泊车场景，数据集为 `data/long_park/data6.json` 此处的碰撞检测使用的是ESDF地图理论上来说可以绕过障碍物收敛到全局最优解。如下图所示这里的红色轨迹为初始轨迹，蓝色轨迹为优化后的轨迹：

![long_park_alter](fig/dat6_alter.png)

但是这样的风险在于如果未能收敛此时的结果有可能与障碍物发生碰撞甚至卡在障碍物中间，为了避免这种风险使得在绝大多数未能收敛的情况下也能保证安全性引入了静态走廊作为约束，这也主动限制了优化的自由度只能在初始轨迹的同伦类中进行优化，在这个场景中就不能主动选择从右侧绕开障碍物，优化效果不佳：

![long_park](fig/data6.png)

**mid_park**：中等距离泊车场景，数据集为 `data/mid_park/data3.json`，同样的这里的红色轨迹为初始轨迹，蓝色轨迹为优化后的轨迹，可以看到在这个场景中成功实现了maneuver段数的削减：

![mid_park](fig/data3.png)

**rub_park**：这里有两个数据集，分别为 `data/rub_park/data1.json` 和 `data/rub_park/data7.json`。可以看到在下面的data1场景中虽然maneuver段数也有削减但是最后两段不够平滑，主要是因为这里的优化目标设置中尽快到达终点的权重较大：

![rub_park_data1](fig/data1.png)

data7中的效果如下图所示，maneuver段数由原来的6段削减为5段：

![rub_park_data7](fig/data7.png)
