#pragma once

#include <functional>
#include <mutex>
#include <sstream>
#include <string>
#include <utility>

namespace stc_SQP {

// 日志级别：仅供宿主 sink 内部按需过滤/着色，本库自身不做任何级别判断。
enum class LogLevel {
    TRACE,
    DEBUG,
    INFO,
    WARN,
    ERROR
};

// 日志输出回调：宿主自行决定把日志转发到 Quill/glog/std::cout，或直接丢弃。
using LogSink = std::function<void(LogLevel level, const std::string& msg)>;

// 极简日志门面：本库不绑定任何具体日志实现（不依赖 Quill/glog 等第三方库）。
// 默认 sink 为空，即“静默”——作为纯数学 SQP 引擎，不应替宿主决定日志落地方式。
// 宿主通过 SetSink() 注入自己的日志系统，未注入时所有 LOG_* 调用均为空操作。
class Logger final {
public:
    // 注入宿主日志 sink；传入空 std::function 等价于恢复静默。
    static void SetSink(LogSink sink)
    {
        std::lock_guard<std::mutex> lock(MUTEX);
        SINK = std::move(sink);
    }
    // 把一条日志分发给当前 sink；未设置 sink 时什么都不做。
    static void Log(LogLevel level, const std::string& msg)
    {
        LogSink sink_copy;
        {
            std::lock_guard<std::mutex> lock(MUTEX);
            sink_copy = SINK;
        }
        if (sink_copy) {
            sink_copy(level, msg);
        }
    }
    // 拼接日志参数，避免调用方为了简单追加内容手写占位符。
    template <typename... TArgs>
    static std::string BuildMessage(TArgs&&... args)
    {
        std::ostringstream oss;
        (AppendMessagePart(oss, std::forward<TArgs>(args)), ...);
        return oss.str();
    }

protected:
    // 保护 SINK 读写的互斥锁。
    static inline std::mutex MUTEX {};
    // 当前生效的日志 sink，默认空即静默。
    static inline LogSink SINK {};

    template <typename TArg>
    static void AppendMessagePart(std::ostringstream& oss, TArg&& arg)
    {
        oss << std::forward<TArg>(arg);
    }
};

} // namespace stc_SQP

#undef LOG_TRACE
#define LOG_TRACE(...) \
    ::stc_SQP::Logger::Log(::stc_SQP::LogLevel::TRACE, \
        ::stc_SQP::Logger::BuildMessage(__VA_ARGS__))
#undef LOG_DEBUG
#define LOG_DEBUG(...) \
    ::stc_SQP::Logger::Log(::stc_SQP::LogLevel::DEBUG, \
        ::stc_SQP::Logger::BuildMessage(__VA_ARGS__))
#undef LOG_INFO
#define LOG_INFO(...) \
    ::stc_SQP::Logger::Log(::stc_SQP::LogLevel::INFO, \
        ::stc_SQP::Logger::BuildMessage(__VA_ARGS__))
#undef LOG_WARN
#define LOG_WARN(...) \
    ::stc_SQP::Logger::Log(::stc_SQP::LogLevel::WARN, \
        ::stc_SQP::Logger::BuildMessage(__VA_ARGS__))
#undef LOG_ERROR
#define LOG_ERROR(...) \
    ::stc_SQP::Logger::Log(::stc_SQP::LogLevel::ERROR, \
        ::stc_SQP::Logger::BuildMessage(__VA_ARGS__))
