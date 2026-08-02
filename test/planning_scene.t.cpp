#include <gtest/gtest.h>

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
        ("apa_scene_" + tag + "_" +
         std::to_string(static_cast<long long>(::getpid())) + ".json");
    std::ofstream ofs(path);
    ofs << text;
    return path.string();
}

// 测试辅助：组装一份场景配置（详情文件路径可指定，数据文件默认为轻量
// 数据集 data/test.json），返回场景配置文件路径
std::string WriteSceneConfig(const std::string& details_path,
                             const std::string& data_file = "/data/test.json") {
    return WriteTempJson(
        "config", std::string("{\"data_file_path\": \"") +
                      std::string(PROJECT_ROOT_DIR) + data_file +
                      "\", \"config_details_path\": \"" + details_path + "\"}");
}

// 测试辅助：按算法详情 JSON 文本构造场景并清理临时文件
std::unique_ptr<PlanningScene> LoadSceneWithDetails(
    const std::string& details_text,
    const std::string& data_file = "/data/test.json") {
    const std::string details_path = WriteTempJson("details", details_text);
    const std::string config_path = WriteSceneConfig(details_path, data_file);
    auto scene = PlanningScene::LoadFromFile(config_path);
    std::filesystem::remove(config_path);
    std::filesystem::remove(details_path);
    return scene;
}

}  // namespace

// 测试场景：详情 JSON 的 algorithm 字段为 "alm" 且配置 outer_row_num=3。
// 预期行为：工厂路由到 ALM 场景，外圆行数透传为 3。
TEST(PlanningSceneTest, FactoryRoutesToAlmSceneWithConfiguredOuterRowNum) {
    auto scene =
        LoadSceneWithDetails("{\"algorithm\": \"alm\", \"outer_row_num\": 3}");
    ASSERT_NE(scene, nullptr);
    EXPECT_EQ(scene->algorithmName(), "ALM");
    EXPECT_EQ(scene->footprintModel().getOuterRowNum(), 3);
}

// 测试场景：详情 JSON 的 algorithm 字段为 "nmpc"，未配置 outer_row_num。
// 预期行为：工厂路由到 NMPC 场景，外圆行数取 Config 基类默认值 4。
TEST(PlanningSceneTest, FactoryRoutesToNmpcSceneWithDefaultOuterRowNum) {
    auto scene = LoadSceneWithDetails("{\"algorithm\": \"nmpc\"}");
    ASSERT_NE(scene, nullptr);
    EXPECT_EQ(scene->algorithmName(), "NMPC");
    EXPECT_EQ(scene->footprintModel().getOuterRowNum(), 4);
}

// 测试场景：NMPC 详情 JSON 配置 outer_row_num=3。
// 预期行为：基类覆盖项经 loadConfigDetails 生效，外圆行数透传为 3（与
// ALM 场景同一解析入口，防止 proto 路由静默忽略基类字段）。
TEST(PlanningSceneTest, NmpcSceneAppliesBaseConfigOverrides) {
    auto scene =
        LoadSceneWithDetails("{\"algorithm\": \"nmpc\", \"outer_row_num\": 3}");
    ASSERT_NE(scene, nullptr);
    EXPECT_EQ(scene->algorithmName(), "NMPC");
    EXPECT_EQ(scene->footprintModel().getOuterRowNum(), 3);
}

// 测试场景：ALM 详情 JSON 未配置 outer_row_num。
// 预期行为：外圆行数取 Config 基类默认值 4。
TEST(PlanningSceneTest, OuterRowNumDefaultsToConfigDefaultWhenAbsent) {
    auto scene = LoadSceneWithDetails("{\"algorithm\": \"alm\"}");
    ASSERT_NE(scene, nullptr);
    EXPECT_EQ(scene->footprintModel().getOuterRowNum(), 4);
}

// 测试场景：详情 JSON 的 algorithm 字段无法识别。
// 预期行为：工厂返回 nullptr。
TEST(PlanningSceneTest, FactoryReturnsNullptrForUnknownAlgorithm) {
    auto scene = LoadSceneWithDetails("{\"algorithm\": \"bogus\"}");
    EXPECT_EQ(scene, nullptr);
}

// 测试场景：场景配置缺少 config_details_path 字段。
// 预期行为：工厂返回 nullptr。
TEST(PlanningSceneTest, FactoryReturnsNullptrWhenDetailsPathAbsent) {
    const std::string config_path = WriteTempJson(
        "config", std::string("{\"data_file_path\": \"") +
                      std::string(PROJECT_ROOT_DIR) + "/data/test.json\"}");
    auto scene = PlanningScene::LoadFromFile(config_path);
    std::filesystem::remove(config_path);
    EXPECT_EQ(scene, nullptr);
}

// 测试场景：未执行 optimize() 时生成优化摘要。
// 预期行为：摘要注明"未执行或失败"，不含优化后指标。
TEST(PlanningSceneTest, OptimizeSummaryHandlesNotExecuted) {
    auto scene = LoadSceneWithDetails("{\"algorithm\": \"alm\"}");
    ASSERT_NE(scene, nullptr);
    const auto summary = scene->optimizeSummary();
    EXPECT_NE(summary.find("failed or not executed"), std::string::npos);
    EXPECT_EQ(summary.find("->"), std::string::npos);
}

// 测试场景：优化失败时生成优化摘要（data/test.json 的退化路径使 ALM
// 预处理无法收敛，真实失败分支）。
// 预期行为：摘要注明失败原因与耗时，不含优化后指标。
TEST(PlanningSceneTest, OptimizeSummaryHandlesOptimizationFailure) {
    auto scene = LoadSceneWithDetails("{\"algorithm\": \"alm\"}");
    ASSERT_NE(scene, nullptr);
    const auto result = scene->optimize();
    ASSERT_FALSE(result.success);
    const auto summary = scene->optimizeSummary();
    EXPECT_NE(summary.find("failed"), std::string::npos);
    EXPECT_NE(summary.find("time_ms="), std::string::npos);
    EXPECT_EQ(summary.find("->"), std::string::npos);
}

// 测试场景：优化成功后生成优化摘要（data3 真实数据集，ALM 可收敛）。
// 预期行为：摘要含优化前后路径长度、机动段数变化与耗时。
TEST(PlanningSceneTest, OptimizeSummaryReportsBeforeAfterMetrics) {
    auto scene = LoadSceneWithDetails("{\"algorithm\": \"alm\"}",
                                      "/data/mid_park/data3.json");
    ASSERT_NE(scene, nullptr);
    const auto result = scene->optimize();
    ASSERT_TRUE(result.success) << result.message;
    const auto summary = scene->optimizeSummary();
    EXPECT_NE(summary.find("length="), std::string::npos);
    EXPECT_NE(summary.find("->"), std::string::npos);
    EXPECT_NE(summary.find("maneuvers="), std::string::npos);
    EXPECT_NE(summary.find("time_ms="), std::string::npos);
}

}  // namespace apa_post_processor
