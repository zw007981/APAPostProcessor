#pragma once

#include <fstream>
#include <google/protobuf/message.h>
#include <google/protobuf/util/json_util.h>
#include <iostream>
#include <nlohmann/json.hpp>
#include <sstream>
#include <string>
#include <type_traits>
#include <vector>

#include "apa_post_process.pb.h"
#include "logger.h"

namespace apa_post_processor {
// 加载结果枚举类
enum class LoadResult {
    // 加载成功
    SUCCESS = 0,
    // 文件不存在
    FILE_NOT_FOUND = 1,
    // 文件格式错误
    FILE_PARSE_ERROR = 2,
    // 不存在指定字段
    FIELD_NOT_FOUND = 3,
    // 其他未知错误
    UNKNOWN_ERROR = 4
};

// 数据加载器类，负责从指定地址读取文件并做必要的解析
class DataLoader {
   public:
    // 从指定文件路径加载json文件并解析到json_obj中，返回加载结果状态
    static LoadResult LoadJsonFile(const std::string& file_path,
                                   nlohmann::json& json_obj) {
        std::ifstream ifs;
        std::string matched_path;
        for (const auto& candidate_path : GetCandidatePaths(file_path)) {
            ifs = std::ifstream(candidate_path);
            if (ifs.is_open()) {
                matched_path = candidate_path;
                break;
            }
        }
        if (!ifs.is_open()) {
            LOG_FMT_ERROR("Unable to open file {}!!!", file_path);
            return LoadResult::FILE_NOT_FOUND;
        }
        LOG_FMT_INFO("Successfully opened file {}.", matched_path);
        try {
            ifs >> json_obj;
        } catch (const nlohmann::json::parse_error& e) {
            LOG_FMT_ERROR("Failed to parse json file {}, reason: {}!!!",
                          matched_path, e.what());
            return LoadResult::FILE_PARSE_ERROR;
        } catch (const std::exception& e) {
            LOG_FMT_ERROR(
                "Unknown error while loading json file {}, reason: {}!!!",
                matched_path, e.what());
            return LoadResult::UNKNOWN_ERROR;
        }
        return LoadResult::SUCCESS;
    }
    // 从指定位置加载json文件并转化为预设的protobuf消息，返回加载结果状态
    template <typename T>
    static LoadResult LoadProtoFromJsonFile(const std::string& file_path,
                                            T& proto_msg) {
        static_assert(std::is_base_of<google::protobuf::Message, T>::value,
                      "T must derive from google::protobuf::Message");
        nlohmann::json json_obj;
        const auto load_result = LoadJsonFile(file_path, json_obj);
        if (load_result != LoadResult::SUCCESS) {
            return load_result;
        }
        const auto json_str = json_obj.dump();
        google::protobuf::util::JsonParseOptions parse_options;
        parse_options.ignore_unknown_fields = true;
        const auto status =
            google::protobuf::util::JsonStringToMessage(json_str, &proto_msg,
                                                        parse_options);
        if (!status.ok()) {
            LOG_FMT_ERROR(
                "Failed to convert json file {} to protobuf, reason: {}!!!",
                file_path, status.ToString());
            return LoadResult::FILE_PARSE_ERROR;
        }
        return LoadResult::SUCCESS;
    }

   protected:
    // 考虑到实际使用时可执行程序可能在build目录下，这里提供多级相对路径的候选列表
    // 以增加文件加载的鲁棒性，但还是优先命中原始路径。
    static std::vector<std::string> GetCandidatePaths(
        const std::string& base_path) {
        return {base_path, "../" + base_path, "../../" + base_path};
    }
};
}  // namespace apa_post_processor