#pragma once

// 关闭 Quill 自带的非前缀 LOG_* 宏，避免和本项目统一宏名冲突。
#define QUILL_DISABLE_NON_PREFIXED_MACROS

#include <quill/Backend.h>
#include <quill/Frontend.h>
#include <quill/LogMacros.h>
#include <quill/Logger.h>
#include <quill/sinks/ConsoleSink.h>
#include <quill/sinks/NullSink.h>
#include <quill/sinks/RotatingFileSink.h>

#include <algorithm>
#include <ctime>
#include <exception>
#include <filesystem>
#include <mutex>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace apa_post_processor {

class Logger final {
   public:
    // 设置日志目录（logger初始化后不可变更）
    static bool SetLogDirectory(const std::string& log_dir) {
        std::lock_guard<std::mutex> lock(CONFIG_MUTEX);
        if (LOGGER != nullptr) {
            return false;
        }
        if (!log_dir.empty()) {
            LOG_DIR = log_dir;
        }
        return true;
    }
    // 设置logger名称（logger初始化后不可变更）
    static bool SetLoggerName(const std::string& logger_name) {
        std::lock_guard<std::mutex> lock(CONFIG_MUTEX);
        if (LOGGER != nullptr) {
            return false;
        }
        if (!logger_name.empty()) {
            LOGGER_NAME = logger_name;
        }
        return true;
    }
    // 设置是否输出到终端（logger初始化后不可变更）
    static bool SetConsoleOutputEnabled(bool enabled) {
        std::lock_guard<std::mutex> lock(CONFIG_MUTEX);
        if (LOGGER != nullptr) {
            return false;
        }
        CONSOLE_OUTPUT_ENABLED = enabled;
        return true;
    }
    // 获取全局logger，首次调用自动初始化
    static quill::Logger* Get() {
        std::call_once(INIT_ONCE, []() {
            quill::BackendOptions backend_options;
            backend_options.check_printable_char = {};
            quill::Backend::start(backend_options);

            std::string target_dir;
            std::string target_logger_name;
            bool console_output_enabled = true;
            {
                std::lock_guard<std::mutex> lock(CONFIG_MUTEX);
                target_dir = LOG_DIR;
                target_logger_name = LOGGER_NAME;
                console_output_enabled = CONSOLE_OUTPUT_ENABLED;
            }

            auto console_sink =
                quill::Frontend::create_or_get_sink<quill::ConsoleSink>(
                    "console_sink");
            auto fallback_sink =
                console_output_enabled
                    ? console_sink
                    : quill::Frontend::create_or_get_sink<quill::NullSink>(
                          "null_sink");
            try {
                std::filesystem::create_directories(target_dir);
                const std::string log_prefix =
                    BuildLogFilePrefix(target_logger_name);
                CleanupOldLogFiles(target_dir, log_prefix);
                const std::string log_file =
                    (std::filesystem::path(target_dir) /
                     BuildLogFileName(target_logger_name))
                        .string();

                quill::RotatingFileSinkConfig sink_config;
                sink_config.set_open_mode('w');
                sink_config.set_rotation_max_file_size(ROTATION_MAX_FILE_SIZE);
                sink_config.set_max_backup_files(MAX_BACKUP_FILES);
                sink_config.set_overwrite_rolled_files(true);

                auto file_sink = quill::Frontend::create_or_get_sink<
                    quill::RotatingFileSink>(log_file, sink_config,
                                             quill::FileEventNotifier{});

                if (console_output_enabled) {
                    LOGGER = quill::Frontend::create_or_get_logger(
                        target_logger_name,
                        {std::move(file_sink), std::move(console_sink)},
                        quill::PatternFormatterOptions{
                            "%(time) [%(thread_id)-%(log_level_short_code)] "
                            "%(source_location) %(message)",
                            "%Y-%m-%d %H:%M:%S.%Qms",
                            quill::Timezone::LocalTime});
                } else {
                    LOGGER = quill::Frontend::create_or_get_logger(
                        target_logger_name, std::move(file_sink),
                        quill::PatternFormatterOptions{
                            "%(time) [%(thread_id)-%(log_level_short_code)] "
                            "%(source_location) %(message)",
                            "%Y-%m-%d %H:%M:%S.%Qms",
                            quill::Timezone::LocalTime});
                }
            } catch (const std::exception&) {
                LOGGER = quill::Frontend::create_or_get_logger(
                    target_logger_name, std::move(fallback_sink),
                    quill::PatternFormatterOptions{
                        "%(time) [%(thread_id)-%(log_level_short_code)] "
                        "%(source_location) %(message)",
                        "%Y-%m-%d %H:%M:%S.%Qms", quill::Timezone::LocalTime});
            }

            LOGGER->set_log_level(quill::LogLevel::TraceL3);
        });

        return LOGGER;
    }
    // 主动刷新日志
    static void Flush() {
        quill::Logger* logger = Get();
        if (logger != nullptr) {
            logger->flush_log();
        }
    }
    // 拼接日志参数
    template <typename... TArgs>
    static std::string BuildMessage(TArgs&&... args) {
        std::ostringstream oss;
        (AppendMessagePart(oss, std::forward<TArgs>(args)), ...);
        return oss.str();
    }

   protected:
    static constexpr const char* DEFAULT_LOG_DIR = "log";
    static constexpr const char* DEFAULT_LOGGER_NAME = "log";
    static constexpr const char* LOG_FILE_SUFFIX = ".log";
    // 单文件最大大小 10MB
    static constexpr size_t ROTATION_MAX_FILE_SIZE = 10U * 1024U * 1024U;
    // 最多保留文件数
    static constexpr uint32_t MAX_BACKUP_FILES = 4U;
    static inline std::once_flag INIT_ONCE{};
    static inline std::mutex CONFIG_MUTEX{};
    static inline std::string LOG_DIR{DEFAULT_LOG_DIR};
    static inline std::string LOGGER_NAME{DEFAULT_LOGGER_NAME};
    static inline bool CONSOLE_OUTPUT_ENABLED{true};
    static inline quill::Logger* LOGGER{nullptr};
    // 生成日志文件名前缀
    static std::string BuildLogFilePrefix(const std::string& logger_name) {
        std::string file_prefix =
            logger_name.empty() ? DEFAULT_LOGGER_NAME : logger_name;
        if (file_prefix.back() != '_') {
            file_prefix.push_back('_');
        }
        return file_prefix;
    }
    // 清理过旧日志文件
    static void CleanupOldLogFiles(const std::string& log_dir,
                                   const std::string& log_prefix) {
        std::vector<std::filesystem::path> log_files;
        for (const auto& entry : std::filesystem::directory_iterator(log_dir)) {
            if (!entry.is_regular_file()) {
                continue;
            }
            const std::string file_name = entry.path().filename().string();
            if (file_name.rfind(log_prefix, 0) != 0) {
                continue;
            }
            if (entry.path().extension() != LOG_FILE_SUFFIX) {
                continue;
            }
            log_files.emplace_back(entry.path());
        }
        if (log_files.size() < MAX_BACKUP_FILES) {
            return;
        }
        std::sort(log_files.begin(), log_files.end(),
                  [](const auto& lhs, const auto& rhs) {
                      return lhs.filename().string() < rhs.filename().string();
                  });
        const size_t delete_count = log_files.size() - MAX_BACKUP_FILES + 1U;
        for (size_t i = 0; i < delete_count; ++i) {
            std::error_code ec;
            std::filesystem::remove(log_files[i], ec);
        }
    }
    static std::string BuildLogFileName(const std::string& logger_name) {
        const std::string file_prefix = BuildLogFilePrefix(logger_name);
        const std::time_t now = std::time(nullptr);
        const std::tm local_tm = *std::localtime(&now);
        char ts_buf[16] = {0};
        std::strftime(ts_buf, sizeof(ts_buf), "%Y%m%d%H%M%S", &local_tm);
        return file_prefix + ts_buf + LOG_FILE_SUFFIX;
    }
    template <typename TArg>
    static void AppendMessagePart(std::ostringstream& oss, TArg&& arg) {
        oss << std::forward<TArg>(arg);
    }
};

}  // namespace apa_post_processor

#undef LOG_TRACE
#define LOG_TRACE(...)                             \
    QUILL_LOG_TRACE_L1(                            \
        ::apa_post_processor::Logger::Get(), "{}", \
        ::apa_post_processor::Logger::BuildMessage(__VA_ARGS__))
#undef LOG_DEBUG
#define LOG_DEBUG(...)                                         \
    QUILL_LOG_DEBUG(::apa_post_processor::Logger::Get(), "{}", \
                    ::apa_post_processor::Logger::BuildMessage(__VA_ARGS__))
#undef LOG_INFO
#define LOG_INFO(...)                                         \
    QUILL_LOG_INFO(::apa_post_processor::Logger::Get(), "{}", \
                   ::apa_post_processor::Logger::BuildMessage(__VA_ARGS__))
#undef LOG_WARN
#define LOG_WARN(...)                                            \
    QUILL_LOG_WARNING(::apa_post_processor::Logger::Get(), "{}", \
                      ::apa_post_processor::Logger::BuildMessage(__VA_ARGS__))
#undef LOG_ERROR
#define LOG_ERROR(...)                                         \
    QUILL_LOG_ERROR(::apa_post_processor::Logger::Get(), "{}", \
                    ::apa_post_processor::Logger::BuildMessage(__VA_ARGS__))
#define LOG_FMT_TRACE(fmt, ...) \
    QUILL_LOG_TRACE_L1(::apa_post_processor::Logger::Get(), fmt, ##__VA_ARGS__)
#define LOG_FMT_DEBUG(fmt, ...) \
    QUILL_LOG_DEBUG(::apa_post_processor::Logger::Get(), fmt, ##__VA_ARGS__)
#define LOG_FMT_INFO(fmt, ...) \
    QUILL_LOG_INFO(::apa_post_processor::Logger::Get(), fmt, ##__VA_ARGS__)
#define LOG_FMT_WARN(fmt, ...) \
    QUILL_LOG_WARNING(::apa_post_processor::Logger::Get(), fmt, ##__VA_ARGS__)
#define LOG_FMT_ERROR(fmt, ...) \
    QUILL_LOG_ERROR(::apa_post_processor::Logger::Get(), fmt, ##__VA_ARGS__)
