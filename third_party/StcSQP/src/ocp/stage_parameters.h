#pragma once

#include "../core/types.h"

namespace stc_SQP {
// 阶段通用参数：SQP 引擎只把它当作固定长度向量，具体语义由 CasADi 生成函数解释
struct StageParameters {
    // 固定长度的参数向量 p
    Vector p;
};
} // namespace stc_SQP
