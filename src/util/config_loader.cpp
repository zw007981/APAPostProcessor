#include "config_loader.h"

#include <stdexcept>

#include "data_loader.hpp"
#include "logger.h"

namespace apa_post_processor {
bool LoadBaseConfigOverrides(const std::string& config_details_path,
                             Config* config) {
    if (config == nullptr) {
        throw std::invalid_argument(
            "LoadBaseConfigOverrides received null config!!!");
    }
    auto details = nlohmann::json();
    if (DataLoader::LoadJsonFile(config_details_path, details) !=
        LoadResult::SUCCESS) {
        return false;
    }
    if (details.contains("outer_row_num")) {
        config->outer_row_num = details["outer_row_num"].get<int>();
    }
    return true;
}
}  // namespace apa_post_processor
