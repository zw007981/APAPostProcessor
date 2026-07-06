#pragma once

#include <gtest/gtest.h>

#include "util/data_loader.hpp"

namespace apa_post_processor {

// 公共测试夹具：统一加载 data/test.json 并提供 proto 访问入口。
// 因为多个模块测试都依赖同一份回归样例，集中封装可避免重复读取与样板代码扩散。
class DataJsonFixture : public ::testing::Test {
   protected:
    void SetUp() override
    {
        const auto load_result = DataLoader::LoadProtoFromJsonFile(
            "data/test.json", optimize_request_);
        ASSERT_EQ(load_result, LoadResult::SUCCESS);
    }

    const ::apa::post_processor::OptimizeRequest& getOptimizeRequest() const
    {
        return optimize_request_;
    }

   protected:
    ::apa::post_processor::OptimizeRequest optimize_request_;
};

}  // namespace apa_post_processor
