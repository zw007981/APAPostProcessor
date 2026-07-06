#include "grid_map.h"

namespace apa_post_processor {
GridMap::GridMap(double resolution, int width, int height,
                 const Position& origin, const std::vector<Position>& cells)
    : resolution_(resolution),
      inv_resolution_(resolution > 0.0 ? 1.0 / resolution : 0.0),
      width_(width),
      height_(height),
      origin_(origin) {
    if (resolution <= EPSILON) {
        LOG_FMT_ERROR("Invalid GridMap resolution: {}. Must be positive!!!",
                      resolution);
        throw std::invalid_argument("GridMap resolution must be positive!!!");
    }
    if (width <= 0 || height <= 0) {
        LOG_FMT_ERROR(
            "Invalid GridMap width/height: {}/{}. Must be positive!!!", width,
            height);
        throw std::invalid_argument("GridMap width/height must be positive!!!");
    }
    const auto expected_size =
        static_cast<std::size_t>(width_) * static_cast<std::size_t>(height_);
    data_.assign(expected_size, static_cast<uint8_t>(0));
    if (cells.empty()) {
        return;
    }
    for (const auto& cell_center : cells) {
        const auto index = this->getIndex(cell_center.x, cell_center.y);
        if (index < 0) {
            continue;
        }
        data_[static_cast<std::size_t>(index)] = 1;
    }
}

GridMap GridMap::FromProto(const ::apa::post_processor::GridMap& proto) {
    std::vector<Position> cells;
    cells.reserve(static_cast<std::size_t>(proto.cells_size()));
    for (const auto& cell_proto : proto.cells()) {
        cells.emplace_back(Position::FromProto(cell_proto));
    }
    return GridMap(proto.resolution(), proto.width(), proto.height(),
                   Position::FromProto(proto.origin()), cells);
}
}  // namespace apa_post_processor