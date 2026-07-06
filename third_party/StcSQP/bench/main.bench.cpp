#include <iostream>

#include <benchmark/benchmark.h>

#include "core/logger.h"

namespace {

// 将日志级别转换为可读字符串，仅用于本基准可执行文件的 std::cout 打印。
const char* ToLogLevelString(stc_SQP::LogLevel level)
{
    switch (level) {
    case stc_SQP::LogLevel::TRACE:
        return "TRACE";
    case stc_SQP::LogLevel::DEBUG:
        return "DEBUG";
    case stc_SQP::LogLevel::INFO:
        return "INFO";
    case stc_SQP::LogLevel::WARN:
        return "WARN";
    case stc_SQP::LogLevel::ERROR:
        return "ERROR";
    }
    return "UNKNOWN";
}

} // namespace

int main(int argc, char** argv)
{
    // 基准可执行文件同样把库日志打到 std::cout，便于 SkipWithError 场景下定位失败原因；
    // 库本身默认静默，这里是宿主（基准程序）主动注入的策略，与 test/main.t.cpp 保持一致。
    stc_SQP::Logger::SetSink([](stc_SQP::LogLevel level, const std::string& msg) {
        std::cout << "[" << ToLogLevelString(level) << "] " << msg << std::endl;
    });
    benchmark::Initialize(&argc, argv);
    if (benchmark::ReportUnrecognizedArguments(argc, argv)) {
        return 1;
    }
    benchmark::RunSpecifiedBenchmarks();
    benchmark::Shutdown();
    return 0;
}
