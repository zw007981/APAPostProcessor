#include <iostream>

#include "core/logger.h"

namespace {

// 将日志级别转换为可读字符串，仅用于本可执行文件的 std::cout 打印。
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

int main() {
    // 主程序直接把库日志打到 std::cout，不依赖任何具体日志框架；
    // 库本身默认静默（NullSink），这里是宿主（本可执行文件）主动注入的策略。
    stc_SQP::Logger::SetSink([](stc_SQP::LogLevel level, const std::string& msg) {
        std::cout << "[" << ToLogLevelString(level) << "] " << msg << std::endl;
    });
    return 0;
}
