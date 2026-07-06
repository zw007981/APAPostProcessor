#pragma once

#include <vector>

#include "qp_data.h"

namespace stc_SQP {
namespace soft_constraint_validation {
    // 校验软约束配置维度与索引合法性，ng_max：普通约束维度上限
    inline bool validate(const SoftConstraintConfig& cfg, int ng_max)
    {
        if (ng_max < 0) {
            return false;
        }
        const int ns = cfg.ns;
        // 当 ns == 0 时，要求 idxs/Zl/Zu/zl/zu 全部为空，避免配置被静默忽略
        if (ns == 0) {
            const bool all_empty =
                cfg.idxs.empty() &&
                cfg.Zl.size() == 0 &&
                cfg.Zu.size() == 0 &&
                cfg.zl.size() == 0 &&
                cfg.zu.size() == 0;
            return all_empty;
        }
        // 非空配置要求各权重/索引容器长度均等于 ns
        const bool sizes_match =
            static_cast<int>(cfg.idxs.size()) == ns &&
            cfg.Zl.size() == ns &&
            cfg.Zu.size() == ns &&
            cfg.zl.size() == ns &&
            cfg.zu.size() == ns;
        if (!sizes_match) {
            return false;
        }
        // 检查索引越界与重复
        std::vector<char> seen(ng_max, 0);
        for (int idx : cfg.idxs) {
            if (idx < 0 || idx >= ng_max) {
                return false;
            }
            if (seen[idx]) {
                // 重复索引会导致 slack 映射被静默覆盖
                return false;
            }
            seen[idx] = 1;
        }
        return true;
    }

    // 额外校验 cfg.ns 与期望的 expected_ns 一致
    inline bool validate(const SoftConstraintConfig& cfg, int ng_max, int expected_ns)
    {
        if (cfg.ns != expected_ns) {
            return false;
        }
        return validate(cfg, ng_max);
    }

    // 直接从 QPData 读取配置与 ng_max 进行校验
    inline bool validate(const QPData& qp_data)
    {
        if (qp_data.soft_config == nullptr) {
            return true;
        }
        return validate(*qp_data.soft_config, qp_data.ng_max);
    }
} // namespace soft_constraint_validation
} // namespace stc_SQP
