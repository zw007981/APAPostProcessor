#include "vehicle_footprint_model.h"

namespace apa_post_processor {
VehicleFootprintModel::VehicleFootprintModel(const VehicleParams& veh_params,
                                             int heading_sample_num,
                                             int inner_row_num,
                                             int outer_row_num)
    : veh_params_(veh_params),
      heading_sample_num_(heading_sample_num),
      inner_row_num_(inner_row_num),
      outer_row_num_(outer_row_num),
      heading_resolution_(
          heading_sample_num >= 2 ? (2.0 * PI / heading_sample_num) : 0.0) {
    if (heading_sample_num_ < 2) {
        LOG_FMT_ERROR(
            "VehicleFootprintModel invalid heading_sample_num={}, requires >= "
            "2",
            heading_sample_num_);
        throw std::invalid_argument(
            "VehicleFootprintModel heading_sample_num must be >= 2");
    }
    if (inner_row_num_ < 1 || outer_row_num_ < 1) {
        LOG_FMT_ERROR(
            "VehicleFootprintModel invalid row num, inner_row_num={}, "
            "outer_row_num={}, requires >= 1",
            inner_row_num_, outer_row_num_);
        throw std::invalid_argument(
            "VehicleFootprintModel row num must be >= 1");
    }
    if (veh_params_.length <= EPSILON || veh_params_.width <= EPSILON ||
        veh_params_.rear_overhang < 0.0) {
        LOG_FMT_ERROR(
            "VehicleFootprintModel received invalid vehicle params: {}",
            veh_params_.toString());
        throw std::invalid_argument(
            "VehicleFootprintModel received invalid vehicle dimensions");
    }
    buildLookupTable();
}

void VehicleFootprintModel::calInterpolatedCenters(
    double x, double y, double theta, CircleType type,
    std::vector<Eigen::Vector2d>& centers,
    std::vector<Eigen::Matrix<double, 2, 3>>& jacobians) const {
    const auto circle_num = this->getCircleNum(type);
    if (static_cast<int>(centers.size()) != circle_num ||
        static_cast<int>(jacobians.size()) != circle_num) {
        LOG_FMT_ERROR(
            "calInterpolatedCenters requires pre-sized output buffers, "
            "centers.size()={}, jacobians.size()={}, expected={}!!!",
            centers.size(), jacobians.size(), circle_num);
        throw std::invalid_argument(
            "calInterpolatedCenters requires pre-sized output buffers!!!");
    }
    // 1. 将输入航向角严格映射到[0, 2π)的值域空间
    // 2. 将整个360°空间等分为N个Bucket，计算当前航向对应的精确浮点Bucket
    // 3. 向下取整获取基准Bucket的位置
    // 4. 残差 Δθ = 真实航向角 - 基准 Bucket
    const double norm_theta = normalizeTheta(theta),
                 exact_bucket = norm_theta / heading_resolution_,
                 base_bucket_float = std::floor(exact_bucket),
                 delta_theta =
                     norm_theta - base_bucket_float * heading_resolution_;
    // 5. 取模得到最终的Bucket索引
    const auto bucket_idx = static_cast<std::size_t>(base_bucket_float) %
                            static_cast<std::size_t>(heading_sample_num_);
    const auto& circle_table =
        (type == CircleType::INNER) ? inner_circle_table_ : outer_circle_table_;
    const auto& base_centers = circle_table[bucket_idx];
    for (int i = 0; i < circle_num; ++i) {
        // 基于一阶泰勒展开进行近似，在delta_theta较小的前提下足够精确
        const double local_x = base_centers[i].x() -
                               base_centers[i].y() * delta_theta,
                     local_y = base_centers[i].y() +
                               base_centers[i].x() * delta_theta;
        centers[i].x() = x + local_x;
        centers[i].y() = y + local_y;
        jacobians[i].setZero();
        jacobians[i](0, 0) = 1.0;
        jacobians[i](1, 1) = 1.0;
        jacobians[i](0, 2) = -base_centers[i].y();
        jacobians[i](1, 2) = base_centers[i].x();
    }
}

void VehicleFootprintModel::buildLookupTable() {
    inner_circle_table_.resize(static_cast<std::size_t>(heading_sample_num_));
    outer_circle_table_.resize(static_cast<std::size_t>(heading_sample_num_));
    for (int i = 0; i < heading_sample_num_; i++) {
        const double theta = static_cast<double>(i) * heading_resolution_;
        auto& inner_row = inner_circle_table_[i];
        auto& outer_row = outer_circle_table_[i];
        generateCirclesAtOrigin(theta, inner_row, outer_row);
    }
}

void VehicleFootprintModel::generateCirclesAtOrigin(
    double theta, std::vector<Eigen::Vector2d>& inner_circles,
    std::vector<Eigen::Vector2d>& outer_circles) {
    const double length = veh_params_.length, width = veh_params_.width,
                 rear_overhang = veh_params_.rear_overhang;
    const double cos_theta = std::cos(theta), sin_theta = std::sin(theta);
    // [x,y]绕原点旋转theta角度后的坐标
    auto rotateFunc = [&](double x, double y) {
        return Eigen::Vector2d(x * cos_theta - y * sin_theta,
                               x * sin_theta + y * cos_theta);
    };
    // -----------------------------------------------------------------------
    // 1. 内部圆 (Inner Circles)
    // 求半径R使得这些圆紧贴车身矩形边界，且对角线上的相邻圆相切。
    // 横向两相邻圆的间距为：Δx = (length - 2 * R) / (col_num - 1)
    // 纵向两相邻圆的间距为：Δy = (width - 2 * R) / (row_num - 1)
    // 对角相邻圆相切引入约束：Δx^2 + Δy^2 = (2R)^2
    // 展开可得关于 R 的一元二次方程：A*R^2 + B*R + C = 0
    // 其中P = 1 / (row_num - 1)^2, Q = 1 / (col_num - 1)^2
    // A = P + Q - 1, B = -(P * width + Q * length),
    // C = 0.25 * (P * width^2 + Q * length^2)
    // -----------------------------------------------------------------------
    auto row_num = inner_row_num_,
         col_num = static_cast<int>(std::ceil(length / width * inner_row_num_));
    inner_circles.clear();
    inner_circles.reserve(row_num * col_num);
    double delta_x = 0.0, delta_y = 0.0, R = 0.5 * width;
    double Q = (col_num > 1) ? 1.0 / ((col_num - 1) * (col_num - 1)) : 0.0;
    // 只有一行时半径由车宽直接决定，且纵向间距为0
    if (row_num > 1) {
        double P = 1.0 / ((row_num - 1) * (row_num - 1));
        // 构造一元二次方程的 A, B, C 系数并求解
        double A = P + Q - 1.0, B = -(P * width + Q * length),
               C = 0.25 * (P * width * width + Q * length * length);
        double delta = std::max(B * B - 4.0 * A * C, 0.0);
        R = 0.5 * (-B - std::sqrt(delta)) / A;
        delta_y = (width - 2.0 * R) / (row_num - 1);
    }
    if (col_num > 1) {
        delta_x = (length - 2.0 * R) / (col_num - 1);
    }
    inner_radius_ = R;
    // 按序号生成内圆坐标并旋转，以theta=0时车辆右后角的那个圆为基准圆
    double base_x = -rear_overhang + R, base_y = -0.5 * width + R;
    for (int i = 0; i < col_num; i++) {
        for (int j = 0; j < row_num; j++) {
            inner_circles.emplace_back(
                rotateFunc(base_x + i * delta_x, base_y + j * delta_y));
        }
    }
    // -----------------------------------------------------------------------
    // 2. 外部轮廓圆 (Outer Circles)
    // 将车身切分为网格，以网格对角线的一半作为外圆的半径
    // -----------------------------------------------------------------------
    row_num = outer_row_num_, delta_y = width / row_num,
    col_num = static_cast<int>(std::ceil(length / delta_y)),
    delta_x = length / col_num;
    outer_circles.clear();
    outer_circles.reserve(row_num * col_num);
    outer_radius_ = 0.5 * std::sqrt(delta_x * delta_x + delta_y * delta_y);
    // 按序号生成外圆坐标并旋转，以theta=0时车辆右后角的那个圆为基准圆
    base_x = -rear_overhang + 0.5 * delta_x;
    base_y = -0.5 * width + 0.5 * delta_y;
    for (int i = 0; i < col_num; i++) {
        for (int j = 0; j < row_num; j++) {
            // 只保留轮廓边界上的圆
            if ((i != 0) && (i != col_num - 1) && (j != 0) &&
                (j != row_num - 1)) {
                continue;
            }
            outer_circles.emplace_back(
                rotateFunc(base_x + i * delta_x, base_y + j * delta_y));
        }
    }
    outer_circles.shrink_to_fit();
}
}  // namespace apa_post_processor
