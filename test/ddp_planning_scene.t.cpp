#include "scene/ddp_planning_scene.h"

#include <gtest/gtest.h>
#include <unistd.h>

#include <filesystem>
#include <fstream>
#include <string>

#include "scene/planning_scene.h"

namespace apa_post_processor {
namespace {

// 测试辅助：把 JSON 文本写入临时文件并返回路径（调用方负责删除）；
// 文件名带进程号与标签避免并行测试互踩
std::string WriteTempJson(const std::string& tag, const std::string& text) {
    const auto path =
        std::filesystem::temp_directory_path() /
        ("apa_ddp_scene_" + tag + "_" +
         std::to_string(static_cast<long long>(::getpid())) + ".json");
    std::ofstream ofs(path);
    ofs << text;
    return path.string();
}

// 测试辅助：按算法详情 JSON 文本构造场景并清理临时文件（数据文件默认为
// 轻量数据集 data/test.json）
std::unique_ptr<PlanningScene> LoadSceneWithDetails(
    const std::string& details_text) {
    const std::string details_path = WriteTempJson("details", details_text);
    const std::string config_path = WriteTempJson(
        "config", std::string("{\"data_file_path\": \"") +
                      std::string(PROJECT_ROOT_DIR) +
                      "/data/test.json\", \"config_details_path\": \"" +
                      details_path + "\"}");
    auto scene = PlanningScene::LoadFromFile(config_path);
    std::filesystem::remove(config_path);
    std::filesystem::remove(details_path);
    return scene;
}

}  // namespace

// 测试场景：详情 JSON 的 algorithm 字段为 "ddp" 且配置 outer_row_num=3。
// 预期行为：工厂路由到 DDP 场景，外圆行数透传为 3（基类覆盖项与
// ALM/NMPC 场景同一解析入口）。
TEST(DdpPlanningSceneTest, FactoryRoutesToDdpSceneWithConfiguredOuterRowNum) {
    auto scene =
        LoadSceneWithDetails("{\"algorithm\": \"ddp\", \"outer_row_num\": 3}");
    ASSERT_NE(scene, nullptr);
    EXPECT_EQ(scene->algorithmName(), "DDP");
    EXPECT_EQ(scene->footprintModel().getOuterRowNum(), 3);
}

// 测试场景：详情 JSON 配置 DDP 专有字段（reference/solver.outer 节）。
// 预期行为：专有字段经 loadConfigDetails 生效，且 reference 节的幅值键
// 经单一来源同步进 cost 同源字段。
TEST(DdpPlanningSceneTest, AppliesDdpSpecificOverrides) {
    auto scene = LoadSceneWithDetails(
        "{\"algorithm\": \"ddp\", "
        "\"reference\": {\"shooting_interval\": 10, \"v_max\": 2.0}, "
        "\"solver\": {\"outer\": {\"mu_min\": 5.0}}}");
    ASSERT_NE(scene, nullptr);
    const auto* ddp_scene = dynamic_cast<const DDPPlanningScene*>(scene.get());
    ASSERT_NE(ddp_scene, nullptr);
    EXPECT_EQ(ddp_scene->ddpConfig().reference.shooting_interval, 10);
    EXPECT_DOUBLE_EQ(ddp_scene->ddpConfig().reference.v_max, 2.0);
    EXPECT_DOUBLE_EQ(ddp_scene->ddpConfig().solver.outer.mu_min, 5.0);
    // 幅值边界单一来源同步：cost.v_max 跟随 reference.v_max
    EXPECT_DOUBLE_EQ(ddp_scene->ddpConfig().solver.cost.v_max, 2.0);
}

// 测试场景：详情 JSON 未配置任何 DDP 专有字段。
// 预期行为：专有字段全部保持 2.5 节建议默认值。
TEST(DdpPlanningSceneTest, KeepsDdpDefaultsWhenDetailsAbsent) {
    auto scene = LoadSceneWithDetails("{\"algorithm\": \"ddp\"}");
    ASSERT_NE(scene, nullptr);
    const auto* ddp_scene = dynamic_cast<const DDPPlanningScene*>(scene.get());
    ASSERT_NE(ddp_scene, nullptr);
    const DdpConfig fresh;
    EXPECT_EQ(ddp_scene->ddpConfig().reference.shooting_interval,
              fresh.reference.shooting_interval);
    EXPECT_DOUBLE_EQ(ddp_scene->ddpConfig().solver.outer.mu_min,
                     fresh.solver.outer.mu_min);
    EXPECT_DOUBLE_EQ(ddp_scene->ddpConfig().solver.cost.weight_ref_base,
                     fresh.solver.cost.weight_ref_base);
    EXPECT_DOUBLE_EQ(ddp_scene->ddpConfig().post_stage.kappa_pad,
                     fresh.post_stage.kappa_pad);
}

// 测试场景：loadConfigDetails 收到不存在的文件路径。
// 预期行为：记错误日志并保持默认配置，不抛异常、不崩溃。
TEST(DdpPlanningSceneTest, MissingDetailsFileKeepsDefaults) {
    auto scene = LoadSceneWithDetails("{\"algorithm\": \"ddp\"}");
    ASSERT_NE(scene, nullptr);
    auto* ddp_scene = dynamic_cast<DDPPlanningScene*>(scene.get());
    ASSERT_NE(ddp_scene, nullptr);
    ASSERT_NO_THROW(
        ddp_scene->loadConfigDetails("/nonexistent/ddp_config.json"));
    const DdpConfig fresh;
    EXPECT_DOUBLE_EQ(ddp_scene->ddpConfig().solver.outer.mu_min,
                     fresh.solver.outer.mu_min);
    EXPECT_EQ(ddp_scene->ddpConfig().reference.shooting_interval,
              fresh.reference.shooting_interval);
}

// 测试场景：未执行 optimize() 时生成优化摘要。
// 预期行为：摘要注明"未执行或失败"，不含优化后指标（算法无关基类机制对
// DDP 场景同样生效）。
TEST(DdpPlanningSceneTest, OptimizeSummaryHandlesNotExecuted) {
    auto scene = LoadSceneWithDetails("{\"algorithm\": \"ddp\"}");
    ASSERT_NE(scene, nullptr);
    const auto summary = scene->optimizeSummary();
    EXPECT_NE(summary.find("failed or not executed"), std::string::npos);
    EXPECT_EQ(summary.find("->"), std::string::npos);
}

}  // namespace apa_post_processor
