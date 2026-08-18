#include <gtest/gtest.h>

#include <nlohmann/json.hpp>

#include "core/post_processor.h"

namespace apa_post_processor {
namespace {

// 测试场景：JSON 仅含部分 ESDF 字段。
// 预期行为：显式出现的字段被覆盖，未出现的字段保持构造默认值。
TEST(MincoConfigTest, LoadFromJsonOverridesOnlyPresentFields) {
    const auto details = nlohmann::json::parse(R"json({
        "algorithm": "minco",
        "esdf": {
            "margin_safe": 0.03, "weight_safe": 800.0
        }
    })json");
    MincoConfig config;
    LoadMincoConfigOverrides(details, &config);
    EXPECT_DOUBLE_EQ(config.margin_safe, 0.03);
    EXPECT_DOUBLE_EQ(config.weight_safe, 800.0);
    const MincoConfig fresh;
    EXPECT_DOUBLE_EQ(config.margin_comf, fresh.margin_comf);
    EXPECT_DOUBLE_EQ(config.weight_comf, fresh.weight_comf);
}

// 测试场景：JSON 覆盖全部四个 ESDF 字段。
// 预期行为：四个字段全部被覆盖为 JSON 值。
TEST(MincoConfigTest, LoadFromJsonOverridesAllEsdfFields) {
    const auto details = nlohmann::json::parse(R"json({
        "esdf": {
            "margin_safe": 0.05, "margin_comf": 0.2,
            "weight_safe": 900.0, "weight_comf": 3.0
        }
    })json");
    MincoConfig config;
    LoadMincoConfigOverrides(details, &config);
    EXPECT_DOUBLE_EQ(config.margin_safe, 0.05);
    EXPECT_DOUBLE_EQ(config.margin_comf, 0.2);
    EXPECT_DOUBLE_EQ(config.weight_safe, 900.0);
    EXPECT_DOUBLE_EQ(config.weight_comf, 3.0);
}

// 测试场景：空指针调用。
// 预期行为：抛 std::invalid_argument（与 LoadiLQRConfigOverrides 同一约定）。
TEST(MincoConfigTest, LoadFromJsonRejectsNullConfig) {
    const auto details =
        nlohmann::json::parse(R"json({"algorithm": "minco"})json");
    EXPECT_THROW(LoadMincoConfigOverrides(details, nullptr),
                 std::invalid_argument);
}

// 测试场景：JSON 含未知字段（各节与顶层混入未登记的键）。
// 预期行为：未知键被静默忽略（加载器只消费登记键），已知字段正常读入、
// 其余保持默认——钉住"忽略而非报错"的容错语义
TEST(MincoConfigTest, LoadFromJsonIgnoresUnknownFields) {
    const auto details = nlohmann::json::parse(R"json({
        "algorithm": "minco",
        "unknown_top_level": 123,
        "esdf": {"weight_safe": 150.0, "unknown_esdf": 1.5}
    })json");
    MincoConfig config;
    LoadMincoConfigOverrides(details, &config);
    EXPECT_DOUBLE_EQ(config.weight_safe, 150.0);
    const MincoConfig fresh;
    EXPECT_DOUBLE_EQ(config.margin_safe, fresh.margin_safe);
}

// 测试场景：JSON 已知字段给错类型（如数值字段写成字符串）。
// 预期行为：nlohmann 类型转换失败抛 json::exception（类型错误是配置
// 作者笔误，必须显式失败而非静默纠错）
TEST(MincoConfigTest, LoadFromJsonThrowsOnWrongFieldType) {
    const auto details = nlohmann::json::parse(R"json({
        "esdf": {"margin_safe": "not_a_number"}
    })json");
    MincoConfig config;
    EXPECT_THROW(LoadMincoConfigOverrides(details, &config),
                 nlohmann::json::exception);
}

}  // namespace
}  // namespace apa_post_processor
