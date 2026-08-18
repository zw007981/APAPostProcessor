#pragma once

#include <string>

#include "config.h"

namespace apa_post_processor {
// 由算法配置详情 JSON 文件加载通用 Config 基类字段的覆盖项：仅覆盖显式
// 出现的字段，未出现的字段保持原值（当前映射：outer_row_num）。按约定
// 每个算法有专属配置详情 JSON（如 minco_config.json/nmpc_config.json），
// 本函数承载其中算法无关的基类字段解析，供场景层与调参工具共用，
// 避免多处解析漂移。文件缺失/格式错误返回 false（调用方决定降级策略）
bool LoadBaseConfigOverrides(const std::string& config_details_path,
                             Config* config);
}  // namespace apa_post_processor
