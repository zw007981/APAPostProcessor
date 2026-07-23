#include <gtest/gtest.h>

#include <cmath>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <random>
#include <vector>

#include "spatial/esdf_map.h"

namespace apa_post_processor {
namespace {

// 测试ESDF构造后保留GridMap的基础尺寸参数。
// 因为轨迹优化会同时依赖距离场和栅格几何信息，ESDF必须继承原地图尺度。
TEST(ESDFMapTest, ConstructorCopiesGridMapMetaData) {
    const GridMap grid_map(0.5, 4, 3, Position{-1.0, 2.0},
                           {Position{0.0, 2.5}});
    const ESDFMap esdf_map(grid_map);

    EXPECT_DOUBLE_EQ(esdf_map.getResolution(), 0.5);
    EXPECT_EQ(esdf_map.getWidth(), 4);
    EXPECT_EQ(esdf_map.getHeight(), 3);
}

// 测试单障碍物场景下F&H能给出精确欧氏距离。
// 通过连续查询在栅格中心/角点采样，锁定sqrt(2)对角线距离。
TEST(ESDFMapTest, SingleObstacleBuildsExactEuclideanOutsideDistance) {
    const GridMap grid_map(0.5, 3, 3, Position{0.0, 0.0}, {Position{0.5, 0.5}});
    const ESDFMap esdf_map(grid_map);

    EXPECT_DOUBLE_EQ(esdf_map.getDistAndGrad(0.5, 0.5).first, -0.5);
    EXPECT_DOUBLE_EQ(esdf_map.getDistAndGrad(1.0, 0.5).first, 0.5);
    EXPECT_NEAR(esdf_map.getDistAndGrad(1.0, 1.0).first, std::sqrt(2.0) * 0.5,
                1e-5);
}

// 测试多栅格障碍物内部为负距离。
// 因为优化器需要负距离把初始碰撞点推出障碍物，内部距离符号不能反向。
TEST(ESDFMapTest, OccupiedRegionReturnsNegativeInsideDistance) {
    const std::vector<Position> cells{Position{0.0, 0.0}, Position{1.0, 0.0},
                                      Position{0.0, 1.0}, Position{1.0, 1.0}};
    const GridMap grid_map(1.0, 4, 4, Position{0.0, 0.0}, cells);
    const ESDFMap esdf_map(grid_map);

    EXPECT_DOUBLE_EQ(esdf_map.getDistAndGrad(0.0, 0.0).first, -2.0);
    EXPECT_DOUBLE_EQ(esdf_map.getDistAndGrad(1.0, 1.0).first, -1.0);
    EXPECT_DOUBLE_EQ(esdf_map.getDistAndGrad(2.0, 1.0).first, 1.0);
}

// 测试连续查询会对距离与梯度同时做双线性插值。
// 因为下游数值优化在非栅格点工作，插值结果直接影响代价和雅可比稳定性。
TEST(ESDFMapTest, QueryUsesBilinearInterpolationForDistanceAndGradient) {
    const GridMap grid_map(1.0, 3, 3, Position{0.0, 0.0}, {Position{0.0, 0.0}});
    const ESDFMap esdf_map(grid_map);
    const auto [dist, grad] = esdf_map.getDistAndGrad(1.5, 1.5);

    const auto expected_distance =
        (std::sqrt(2.0) + std::sqrt(5.0) + std::sqrt(5.0) + std::sqrt(8.0)) *
        0.25;
    const auto expected_grad_x =
        ((std::sqrt(5.0) - 1.0) * 0.5 + (std::sqrt(5.0) - std::sqrt(2.0)) +
         (std::sqrt(8.0) - 2.0) * 0.5 + (std::sqrt(8.0) - std::sqrt(5.0))) *
        0.25;
    const auto expected_grad_y = expected_grad_x;

    EXPECT_NEAR(dist, expected_distance, 1e-5);
    EXPECT_NEAR(grad.x(), expected_grad_x, 1e-5);
    EXPECT_NEAR(grad.y(), expected_grad_y, 1e-5);
}

// 测试底层存储与插值计算为双精度：非栅格点查询与双精度手推双线性公式在
// 1e-13 内一致（float 存储的量化噪声 ~1e-7 会立即暴露）。
// 因为下游 ALM 数值梯度对拍需要 1e-6 量级精度，float 量化噪声是唯一瓶颈。
TEST(ESDFMapTest, BilinearQueryMaintainsDoublePrecision) {
    const GridMap grid_map(1.0, 3, 3, Position{0.0, 0.0}, {Position{0.0, 0.0}});
    const ESDFMap esdf_map(grid_map);
    // 查询点 (1.2345678901234567, 1.03125)：两个方向的双线性权重均非平凡
    // （col_ratio 非 2 的幂友好值，float 截断会产生可观测误差）
    const double query_x = 1.2345678901234567;
    const double query_y = 1.03125;
    const auto [dist, grad] = esdf_map.getDistAndGrad(query_x, query_y);

    const double col_ratio = query_x - 1.0;
    const double row_ratio = query_y - 1.0;
    const double sqrt2 = std::sqrt(2.0);
    const double sqrt5 = std::sqrt(5.0);
    const double sqrt8 = std::sqrt(8.0);
    const double dist_row_lower = sqrt2 + (sqrt5 - sqrt2) * col_ratio;
    const double dist_row_upper = sqrt5 + (sqrt8 - sqrt5) * col_ratio;
    const double expected_dist =
        dist_row_lower + (dist_row_upper - dist_row_lower) * row_ratio;
    // 梯度场由距离的有限差分构成（内部中心差分、边界单侧差分）：
    // cell(1,1): ((sqrt5-1)/2, (sqrt5-1)/2)；cell(1,2): 右边界 (sqrt5-sqrt2, (sqrt8-2)/2)
    // cell(2,1): 下边界 ((sqrt8-2)/2, sqrt5-sqrt2)；cell(2,2): 角点 (sqrt8-sqrt5, sqrt8-sqrt5)
    const double gx_bl = (sqrt5 - 1.0) * 0.5;
    const double gx_br = sqrt5 - sqrt2;
    const double gx_tl = (sqrt8 - 2.0) * 0.5;
    const double gx_tr = sqrt8 - sqrt5;
    const double gy_bl = gx_bl;
    const double gy_br = gx_tl;
    const double gy_tl = gx_br;
    const double gy_tr = gx_tr;
    const double expected_gx =
        (gx_bl + (gx_br - gx_bl) * col_ratio) * (1.0 - row_ratio) +
        (gx_tl + (gx_tr - gx_tl) * col_ratio) * row_ratio;
    const double expected_gy =
        (gy_bl + (gy_br - gy_bl) * col_ratio) * (1.0 - row_ratio) +
        (gy_tl + (gy_tr - gy_tl) * col_ratio) * row_ratio;

    EXPECT_NEAR(dist, expected_dist, 1e-13);
    EXPECT_NEAR(grad.x(), expected_gx, 1e-13);
    EXPECT_NEAR(grad.y(), expected_gy, 1e-13);
}

// 测试仅查询符号距离与 getDistAndGrad 返回的距离一致。
TEST(ESDFMapTest, GetDistanceMatchesDistAndGrad) {
    const GridMap grid_map(1.0, 3, 3, Position{0.0, 0.0}, {Position{0.0, 0.0}});
    const ESDFMap esdf_map(grid_map);
    EXPECT_DOUBLE_EQ(esdf_map.getDist(1.5, 1.5),
                     esdf_map.getDistAndGrad(1.5, 1.5).first);
}

// 测试越界连续查询返回安全默认值。
// 因为地图外区域缺少可靠观测，ESDF不应向优化器提供虚假的方向信息。
TEST(ESDFMapTest, QueryOutOfBoundsReturnsDefaultResult) {
    const GridMap grid_map(1.0, 3, 3, Position{0.0, 0.0}, {Position{0.0, 0.0}});
    const ESDFMap esdf_map(grid_map);
    const auto [dist, grad] = esdf_map.getDistAndGrad(3.0, 1.0);

    EXPECT_DOUBLE_EQ(dist, 0.0);
    EXPECT_DOUBLE_EQ(grad.x(), 0.0);
    EXPECT_DOUBLE_EQ(grad.y(), 0.0);
}

// 测试越界距离查询返回安全默认值。
TEST(ESDFMapTest, GetDistanceOutOfBoundsReturnsDefault) {
    const GridMap grid_map(1.0, 3, 3, Position{0.0, 0.0}, {Position{0.0, 0.0}});
    const ESDFMap esdf_map(grid_map);
    EXPECT_DOUBLE_EQ(esdf_map.getDist(3.0, 1.0), 0.0);
}

// 测试 ESDFMap 构造函数在输入 GridMap 参数非法时抛出 std::invalid_argument。
// 分辨率非正或地图尺寸为 0 都属于不可恢复的构造错误，必须显式失败。
TEST(ESDFMapTest, ConstructorThrowsOnInvalidParameters) {
    EXPECT_THROW(
        {
            const GridMap grid_map(0.0, 3, 3, Position{0.0, 0.0}, {});
            const ESDFMap esdf_map(grid_map);
        },
        std::invalid_argument);
    EXPECT_THROW(
        {
            const GridMap grid_map(-0.5, 3, 3, Position{0.0, 0.0}, {});
            const ESDFMap esdf_map(grid_map);
        },
        std::invalid_argument);
    EXPECT_THROW(
        {
            const GridMap grid_map(1.0, 0, 3, Position{0.0, 0.0}, {});
            const ESDFMap esdf_map(grid_map);
        },
        std::invalid_argument);
    EXPECT_THROW(
        {
            const GridMap grid_map(1.0, 3, 0, Position{0.0, 0.0}, {});
            const ESDFMap esdf_map(grid_map);
        },
        std::invalid_argument);
}

// 测试地图四周边界上的梯度使用单侧差分，而不是越界的中心差分。
// 边界导数方向对约束投影至关重要，错误差分会导致优化器在边界处得到虚假梯度。
TEST(ESDFMapTest, GradientAtBoundariesUsesSingleSidedDifference) {
    const GridMap grid_map(1.0, 4, 4, Position{0.0, 0.0},
                           {Position{0.0, 0.0}, Position{1.0, 0.0},
                            Position{0.0, 1.0}, Position{1.0, 1.0}});
    const ESDFMap esdf_map(grid_map);
    constexpr double kTol = 1e-5;
    const auto [left_dist, left_grad] = esdf_map.getDistAndGrad(0.0, 1.0);
    const double left_expected =
        (esdf_map.getDist(1.0, 1.0) - esdf_map.getDist(0.0, 1.0)) / 1.0;
    EXPECT_NEAR(left_grad.x(), left_expected, kTol);
    const auto [right_dist, right_grad] = esdf_map.getDistAndGrad(3.0, 1.0);
    const double right_expected =
        (esdf_map.getDist(3.0, 1.0) - esdf_map.getDist(2.0, 1.0)) / 1.0;
    EXPECT_NEAR(right_grad.x(), right_expected, kTol);
    const auto [bottom_dist, bottom_grad] = esdf_map.getDistAndGrad(1.0, 0.0);
    const double bottom_expected =
        (esdf_map.getDist(1.0, 1.0) - esdf_map.getDist(1.0, 0.0)) / 1.0;
    EXPECT_NEAR(bottom_grad.y(), bottom_expected, kTol);
    const auto [top_dist, top_grad] = esdf_map.getDistAndGrad(1.0, 3.0);
    const double top_expected =
        (esdf_map.getDist(1.0, 3.0) - esdf_map.getDist(1.0, 2.0)) / 1.0;
    EXPECT_NEAR(top_grad.y(), top_expected, kTol);
}

// 测试全自由地图的距离值为算法使用的最大距离，且梯度为 0。
// 没有障碍物时 D_out 退化为最大平方距离，D_in 为 0，因此全场距离恒定。
TEST(ESDFMapTest, FullyFreeMapCalculatesCorrectDistances) {
    const GridMap grid_map(0.5, 3, 3, Position{0.0, 0.0}, {});
    const ESDFMap esdf_map(grid_map);
    const double expected_distance =
        std::sqrt(3.0 * 3.0 + 3.0 * 3.0 + 1.0) * 0.5;
    EXPECT_NEAR(esdf_map.getDist(0.0, 0.0), expected_distance, 1e-5);
    EXPECT_NEAR(esdf_map.getDist(1.0, 1.0), expected_distance, 1e-5);
    const auto [dist, grad] = esdf_map.getDistAndGrad(1.0, 1.0);
    EXPECT_NEAR(dist, expected_distance, 1e-5);
    EXPECT_NEAR(grad.x(), 0.0, 1e-5);
    EXPECT_NEAR(grad.y(), 0.0, 1e-5);
}

// 测试全占据地图的距离值为恒定的最大负距离，且梯度为 0。
// 所有栅格都是障碍物时，D_out 为 0，D_in 退化为最大平方距离，符号距离全场相同。
TEST(ESDFMapTest, FullyOccupiedMapCalculatesCorrectDistances) {
    const std::vector<Position> cells{
        Position{0.0, 0.0}, Position{1.0, 0.0}, Position{2.0, 0.0},
        Position{0.0, 1.0}, Position{1.0, 1.0}, Position{2.0, 1.0},
        Position{0.0, 2.0}, Position{1.0, 2.0}, Position{2.0, 2.0}};
    const GridMap grid_map(1.0, 3, 3, Position{0.0, 0.0}, cells);
    const ESDFMap esdf_map(grid_map);
    const double expected_distance = -std::sqrt(3.0 * 3.0 + 3.0 * 3.0 + 1.0);
    EXPECT_NEAR(esdf_map.getDist(1.0, 1.0), expected_distance, 1e-4);
    const auto [dist, grad] = esdf_map.getDistAndGrad(1.0, 1.0);
    EXPECT_NEAR(dist, expected_distance, 1e-4);
    EXPECT_NEAR(grad.x(), 0.0, 1e-5);
    EXPECT_NEAR(grad.y(), 0.0, 1e-5);
}

// 测试在刚好映射到网格点（插值 ratio 为 0.0）的物理坐标上查询没有插值误差。
// 此时查询值应严格等于底层 distance_data_，避免双线性插值引入的浮点偏差。
TEST(ESDFMapTest, QueryAtExactGridCenterHasNoInterpolationError) {
    const GridMap grid_map(1.0, 4, 4, Position{0.0, 0.0},
                           {Position{0.0, 0.0}, Position{1.0, 0.0},
                            Position{0.0, 1.0}, Position{1.0, 1.0}});
    const ESDFMap esdf_map(grid_map);
    EXPECT_DOUBLE_EQ(esdf_map.getDist(3.0, 0.0), 2.0);
    const auto [dist, grad] = esdf_map.getDistAndGrad(3.0, 0.0);
    EXPECT_DOUBLE_EQ(dist, 2.0);
}

// 验证自己造的轮子 F&H ESDF 与 OpenCV distanceTransform 在 50x50
// 随机地图上结果是否一致， 使用固定种子和
// 25%占用率生成地图，逐栅格对比最近障碍物距离。
TEST(ESDFMapTest, ESDFMatchesOpenCVDistanceTransform) {
    constexpr int kWidth = 50;
    constexpr int kHeight = 50;
    constexpr double kResolution = 0.2;
    constexpr double kOccupiedProbability = 0.25;
    constexpr std::uint32_t kSeed = 42U;
    const std::size_t cell_count =
        static_cast<std::size_t>(kWidth) * static_cast<std::size_t>(kHeight);
    std::mt19937 rng(kSeed);
    std::bernoulli_distribution is_occupied(kOccupiedProbability);
    std::vector<uint8_t> occupancy(cell_count, 0U);
    std::vector<Position> occupied_cells;
    occupied_cells.reserve(cell_count);
    cv::Mat outside_mask(kHeight, kWidth, CV_8UC1, cv::Scalar(255));
    cv::Mat inside_mask(kHeight, kWidth, CV_8UC1, cv::Scalar(0));
    for (int row = 0; row < kHeight; ++row) {
        for (int col = 0; col < kWidth; ++col) {
            const std::size_t index =
                static_cast<std::size_t>(row * kWidth + col);
            if (!is_occupied(rng)) {
                continue;
            }
            occupancy[index] = 1U;
            occupied_cells.emplace_back(static_cast<double>(col) * kResolution,
                                        static_cast<double>(row) * kResolution);
            outside_mask.at<uint8_t>(row, col) = 0U;
            inside_mask.at<uint8_t>(row, col) = 255U;
        }
    }
    if (occupied_cells.empty()) {
        occupancy[0] = 1U;
        occupied_cells.emplace_back(0.0, 0.0);
        outside_mask.at<uint8_t>(0, 0) = 0U;
        inside_mask.at<uint8_t>(0, 0) = 255U;
    }
    if (occupied_cells.size() == cell_count) {
        occupancy.back() = 0U;
        occupied_cells.pop_back();
        outside_mask.at<uint8_t>(kHeight - 1, kWidth - 1) = 255U;
        inside_mask.at<uint8_t>(kHeight - 1, kWidth - 1) = 0U;
    }
    const GridMap grid_map(kResolution, kWidth, kHeight, Position{0.0, 0.0},
                           occupied_cells);
    const ESDFMap esdf_map(grid_map);
    cv::Mat outside_distance;
    cv::Mat inside_distance;
    cv::Mat signed_distance;
    cv::distanceTransform(outside_mask, outside_distance, cv::DIST_L2,
                          cv::DIST_MASK_PRECISE, CV_32F);
    cv::distanceTransform(inside_mask, inside_distance, cv::DIST_L2,
                          cv::DIST_MASK_PRECISE, CV_32F);
    cv::subtract(outside_distance, inside_distance, signed_distance);
    signed_distance *= static_cast<float>(kResolution);
    for (int row = 0; row < kHeight; ++row) {
        for (int col = 0; col < kWidth; ++col) {
            const double x = static_cast<double>(col) * kResolution;
            const double y = static_cast<double>(row) * kResolution;
            const double opencv_value =
                static_cast<double>(signed_distance.at<float>(row, col));
            EXPECT_NEAR(esdf_map.getDist(x, y), opencv_value, 1e-4)
                << "getDist mismatch at row=" << row << " col=" << col;
            EXPECT_NEAR(esdf_map.getDistAndGrad(x, y).first, opencv_value, 1e-4)
                << "getDistAndGrad mismatch at row=" << row << " col=" << col;
        }
    }
}

// 验证大于并行阈值的随机地图仍与 OpenCV 精确距离变换一致。
// 该场景会触发 ESDF 内部 OpenMP 路径，避免并行缓存交换错误被小图测试漏掉。
TEST(ESDFMapTest, ParallelESDFMatchesOpenCVDistanceTransform) {
    constexpr int kWidth = 100;
    constexpr int kHeight = 100;
    constexpr double kResolution = 0.1;
    constexpr double kOccupiedProbability = 0.2;
    constexpr std::uint32_t kSeed = 2025U;
    const std::size_t cell_count =
        static_cast<std::size_t>(kWidth) * static_cast<std::size_t>(kHeight);
    std::mt19937 rng(kSeed);
    std::bernoulli_distribution is_occupied(kOccupiedProbability);
    std::vector<Position> occupied_cells;
    occupied_cells.reserve(cell_count);
    cv::Mat outside_mask(kHeight, kWidth, CV_8UC1, cv::Scalar(255));
    cv::Mat inside_mask(kHeight, kWidth, CV_8UC1, cv::Scalar(0));
    for (int row = 0; row < kHeight; ++row) {
        for (int col = 0; col < kWidth; ++col) {
            if (!is_occupied(rng)) {
                continue;
            }
            occupied_cells.emplace_back(static_cast<double>(col) * kResolution,
                                        static_cast<double>(row) * kResolution);
            outside_mask.at<uint8_t>(row, col) = 0U;
            inside_mask.at<uint8_t>(row, col) = 255U;
        }
    }
    if (occupied_cells.empty()) {
        occupied_cells.emplace_back(0.0, 0.0);
        outside_mask.at<uint8_t>(0, 0) = 0U;
        inside_mask.at<uint8_t>(0, 0) = 255U;
    }
    const GridMap grid_map(kResolution, kWidth, kHeight, Position{0.0, 0.0},
                           occupied_cells);
    const ESDFMap esdf_map(grid_map);
    cv::Mat outside_distance;
    cv::Mat inside_distance;
    cv::Mat signed_distance;
    cv::distanceTransform(outside_mask, outside_distance, cv::DIST_L2,
                          cv::DIST_MASK_PRECISE, CV_32F);
    cv::distanceTransform(inside_mask, inside_distance, cv::DIST_L2,
                          cv::DIST_MASK_PRECISE, CV_32F);
    cv::subtract(outside_distance, inside_distance, signed_distance);
    signed_distance *= static_cast<float>(kResolution);
    for (int row = 0; row < kHeight; ++row) {
        for (int col = 0; col < kWidth; ++col) {
            const double x = static_cast<double>(col) * kResolution;
            const double y = static_cast<double>(row) * kResolution;
            const double opencv_value =
                static_cast<double>(signed_distance.at<float>(row, col));
            EXPECT_NEAR(esdf_map.getDist(x, y), opencv_value, 1e-4)
                << "parallel getDist mismatch at row=" << row << " col=" << col;
        }
    }
}

// 测试非零原点下物理坐标到栅格索引的平移映射正确。
// calBilinearParams 使用 (x - origin_)
// 计算索引，原点偏移不能影响相对距离与梯度。
TEST(ESDFMapTest, DistanceAndGradientAreInvariantUnderOriginTranslation) {
    const GridMap zero_origin_map(0.5, 3, 3, Position{0.0, 0.0},
                                  {Position{0.5, 0.5}});
    const ESDFMap zero_origin_esdf(zero_origin_map);
    const GridMap shifted_map(0.5, 3, 3, Position{1.0, -2.0},
                              {Position{1.5, -1.5}});
    const ESDFMap shifted_esdf(shifted_map);
    constexpr double kTol = 1e-5;
    const auto [dist_a, grad_a] = zero_origin_esdf.getDistAndGrad(1.0, 1.0);
    const auto [dist_b, grad_b] = shifted_esdf.getDistAndGrad(2.0, -1.0);
    EXPECT_NEAR(dist_a, dist_b, kTol);
    EXPECT_NEAR(grad_a.x(), grad_b.x(), kTol);
    EXPECT_NEAR(grad_a.y(), grad_b.y(), kTol);
    EXPECT_DOUBLE_EQ(zero_origin_esdf.getDist(0.5, 0.5),
                     shifted_esdf.getDist(1.5, -1.5));
    EXPECT_NEAR(zero_origin_esdf.getDist(1.0, 0.5),
                shifted_esdf.getDist(2.0, -1.5), kTol);
}

// 测试 width=1 的退化地图仍能正确计算距离场，且梯度场因尺寸过小而安全置 0。
TEST(ESDFMapTest, DegenerateOneByNMapComputesDistanceButZeroGradient) {
    const GridMap grid_map(1.0, 1, 5, Position{0.0, 0.0}, {Position{0.0, 0.0}});
    const ESDFMap esdf_map(grid_map);
    EXPECT_DOUBLE_EQ(esdf_map.getDist(0.0, 0.0), -1.0);
    EXPECT_DOUBLE_EQ(esdf_map.getDist(0.0, 1.0), 1.0);
    EXPECT_DOUBLE_EQ(esdf_map.getDist(0.0, 2.0), 2.0);
    const auto [dist, grad] = esdf_map.getDistAndGrad(0.0, 2.0);
    EXPECT_DOUBLE_EQ(grad.x(), 0.0);
    EXPECT_DOUBLE_EQ(grad.y(), 0.0);
}

// 测试 height=1 的退化地图仍能正确计算距离场，且梯度场因尺寸过小而安全置 0。
TEST(ESDFMapTest, DegenerateNByOneMapComputesDistanceButZeroGradient) {
    const GridMap grid_map(1.0, 5, 1, Position{0.0, 0.0}, {Position{0.0, 0.0}});
    const ESDFMap esdf_map(grid_map);
    EXPECT_DOUBLE_EQ(esdf_map.getDist(0.0, 0.0), -1.0);
    EXPECT_DOUBLE_EQ(esdf_map.getDist(1.0, 0.0), 1.0);
    EXPECT_DOUBLE_EQ(esdf_map.getDist(2.0, 0.0), 2.0);
    const auto [dist, grad] = esdf_map.getDistAndGrad(2.0, 0.0);
    EXPECT_DOUBLE_EQ(grad.x(), 0.0);
    EXPECT_DOUBLE_EQ(grad.y(), 0.0);
}

// ===== getDistAndGradBatch 批量查询测试 =====

// 测试目标：批量查询与逐点 getDistAndGrad 在随机采样点上完全一致。
// 测试流程：从双障碍物地图中随机采样 100 个点，分别用批量和单点查询，
//   比较距离与梯度的 x/y 分量。
// 预期结果：所有点距离误差 <= 1e-10，梯度 x/y 误差 <= 1e-10。
TEST(ESDFMapTest, BatchQueryMatchesIndividualQueriesOnRandomPoints) {
    const GridMap grid_map(0.2, 20, 20, Position{0.0, 0.0},
                           {Position{1.0, 1.0}, Position{2.5, 2.5}});
    const ESDFMap esdf_map(grid_map);
    std::mt19937 rng(77U);
    std::uniform_real_distribution<double> coord_dist(0.0, 3.8);
    constexpr int kNumPoints = 100;
    std::vector<double> xs(kNumPoints);
    std::vector<double> ys(kNumPoints);
    for (int i = 0; i < kNumPoints; ++i) {
        xs[i] = coord_dist(rng);
        ys[i] = coord_dist(rng);
    }
    std::vector<double> batch_dists(kNumPoints);
    std::vector<double> batch_gx(kNumPoints);
    std::vector<double> batch_gy(kNumPoints);
    esdf_map.getDistAndGradBatch(xs.data(), ys.data(), kNumPoints,
                                 batch_dists.data(), batch_gx.data(),
                                 batch_gy.data());
    for (int i = 0; i < kNumPoints; ++i) {
        const auto [dist, grad] = esdf_map.getDistAndGrad(xs[i], ys[i]);
        EXPECT_NEAR(batch_dists[i], dist, 1e-10)
            << "dist mismatch at i=" << i << " (" << xs[i] << "," << ys[i]
            << ")";
        EXPECT_NEAR(batch_gx[i], grad.x(), 1e-10)
            << "grad_x mismatch at i=" << i;
        EXPECT_NEAR(batch_gy[i], grad.y(), 1e-10)
            << "grad_y mismatch at i=" << i;
    }
}

// 测试目标：批量查询在全部越界的极端情况下安全返回默认值。
// 测试流程：构造 4×4 地图，取远离地图的 5 个坐标点做批量查询。
// 预期结果：所有点 dist=0, grad_x=0, grad_y=0（与 getDistAndGrad
// 越界行为一致）。
TEST(ESDFMapTest, BatchQueryAllOutOfBoundsReturnsDefaults) {
    const GridMap grid_map(1.0, 4, 4, Position{0.0, 0.0}, {Position{1.0, 1.0}});
    const ESDFMap esdf_map(grid_map);
    constexpr int kN = 5;
    const double xs[kN] = {-1.0, 5.0, 10.0, -5.0, 100.0};
    const double ys[kN] = {-1.0, 5.0, 10.0, 5.0, -100.0};
    double dists[kN], gx[kN], gy[kN];
    esdf_map.getDistAndGradBatch(xs, ys, kN, dists, gx, gy);
    for (int i = 0; i < kN; ++i) {
        EXPECT_DOUBLE_EQ(dists[i], 0.0) << "dist should be 0 at i=" << i;
        EXPECT_DOUBLE_EQ(gx[i], 0.0) << "grad_x should be 0 at i=" << i;
        EXPECT_DOUBLE_EQ(gy[i], 0.0) << "grad_y should be 0 at i=" << i;
    }
}

// 测试目标：批量查询在部分越界、部分合法的混合场景下正确处理。
// 测试流程：构造 3×3 小地图，取 4 个点：2 个在界内（栅格中心）、2 个在界外。
// 预期结果：界内点返回正确距离值；界外点返回 (0,0,0)。
TEST(ESDFMapTest, BatchQueryMixedInBoundsAndOutOfBounds) {
    const GridMap grid_map(1.0, 3, 3, Position{0.0, 0.0}, {Position{0.0, 0.0}});
    const ESDFMap esdf_map(grid_map);
    constexpr int kN = 4;
    // 点0: 越界（x<0）；点1: 界内(1.5,1.5)；点2: 界内(2.0,1.0)；点3:
    // 越界（y>3）
    const double xs[kN] = {-1.0, 1.5, 2.0, 0.5};
    const double ys[kN] = {0.5, 1.5, 1.0, 5.0};
    double dists[kN], gx[kN], gy[kN];
    esdf_map.getDistAndGradBatch(xs, ys, kN, dists, gx, gy);
    // 点0 越界
    EXPECT_DOUBLE_EQ(dists[0], 0.0);
    EXPECT_DOUBLE_EQ(gx[0], 0.0);
    EXPECT_DOUBLE_EQ(gy[0], 0.0);
    // 点1 界内
    const auto [d1, grad1] = esdf_map.getDistAndGrad(1.5, 1.5);
    EXPECT_NEAR(dists[1], d1, 1e-10);
    EXPECT_NEAR(gx[1], grad1.x(), 1e-10);
    EXPECT_NEAR(gy[1], grad1.y(), 1e-10);
    // 点2 界内
    const auto [d2, grad2] = esdf_map.getDistAndGrad(2.0, 1.0);
    EXPECT_NEAR(dists[2], d2, 1e-10);
    EXPECT_NEAR(gx[2], grad2.x(), 1e-10);
    EXPECT_NEAR(gy[2], grad2.y(), 1e-10);
    // 点3 越界
    EXPECT_DOUBLE_EQ(dists[3], 0.0);
    EXPECT_DOUBLE_EQ(gx[3], 0.0);
    EXPECT_DOUBLE_EQ(gy[3], 0.0);
}

// 测试目标：批量查询 n=1 的单点边界情况与 getDistAndGrad 一致。
// 测试流程：对有障碍物的地图取单点做批量查询。
// 预期结果：距离和梯度与 getDistAndGrad 完全一致。
TEST(ESDFMapTest, BatchQuerySinglePoint) {
    const GridMap grid_map(1.0, 4, 4, Position{0.0, 0.0},
                           {Position{1.0, 1.0}, Position{2.0, 2.0}});
    const ESDFMap esdf_map(grid_map);
    const double x = 1.5, y = 2.5;
    double dist, gx, gy;
    esdf_map.getDistAndGradBatch(&x, &y, 1, &dist, &gx, &gy);
    const auto [expected_d, expected_g] = esdf_map.getDistAndGrad(x, y);
    EXPECT_NEAR(dist, expected_d, 1e-10);
    EXPECT_NEAR(gx, expected_g.x(), 1e-10);
    EXPECT_NEAR(gy, expected_g.y(), 1e-10);
}

// 测试目标：批量查询在栅格中心点（插值 ratio=0）给出精确值。
// 测试流程：取 4 个恰好落在栅格中心的坐标，批量查询。
// 预期结果：距离值与 getDist 一致（无插值误差），梯度值与 getDistAndGrad 一致。
TEST(ESDFMapTest, BatchQueryAtGridCentersExact) {
    const GridMap grid_map(1.0, 4, 4, Position{0.0, 0.0},
                           {Position{0.0, 0.0}, Position{1.0, 0.0},
                            Position{0.0, 1.0}, Position{1.0, 1.0}});
    const ESDFMap esdf_map(grid_map);
    constexpr int kN = 4;
    // 四个栅格中心：(0,0)障碍物内；(2,0)自由；(0,2)自由；(2,2)自由
    const double xs[kN] = {0.0, 2.0, 0.0, 2.0};
    const double ys[kN] = {0.0, 0.0, 2.0, 2.0};
    double dists[kN], gx[kN], gy[kN];
    esdf_map.getDistAndGradBatch(xs, ys, kN, dists, gx, gy);
    for (int i = 0; i < kN; ++i) {
        EXPECT_DOUBLE_EQ(dists[i], esdf_map.getDist(xs[i], ys[i]))
            << "dist mismatch at i=" << i;
        const auto [_, grad] = esdf_map.getDistAndGrad(xs[i], ys[i]);
        EXPECT_NEAR(gx[i], grad.x(), 1e-10) << "grad_x mismatch at i=" << i;
        EXPECT_NEAR(gy[i], grad.y(), 1e-10) << "grad_y mismatch at i=" << i;
    }
}

// 测试目标：批量查询在地图最大尺寸（n=12，对应 12 个外圆）下结果一致。
// 测试流程：从随机地图中随机采样 12 个点，对比批量与单点查询。
// 预期结果：全部 12 个点距离和梯度与 getDistAndGrad 一致。
TEST(ESDFMapTest, BatchQueryMaxBatchSize12) {
    const GridMap grid_map(0.1, 30, 30, Position{0.0, 0.0},
                           {Position{1.0, 1.0}, Position{2.0, 2.0}});
    const ESDFMap esdf_map(grid_map);
    std::mt19937 rng(123U);
    std::uniform_real_distribution<double> coord_dist(0.1, 2.8);
    constexpr int kN = 12;
    double xs[kN], ys[kN];
    for (int i = 0; i < kN; ++i) {
        xs[i] = coord_dist(rng);
        ys[i] = coord_dist(rng);
    }
    double dists[kN], gx[kN], gy[kN];
    esdf_map.getDistAndGradBatch(xs, ys, kN, dists, gx, gy);
    for (int i = 0; i < kN; ++i) {
        const auto [dist, grad] = esdf_map.getDistAndGrad(xs[i], ys[i]);
        EXPECT_NEAR(dists[i], dist, 1e-10) << "dist mismatch at i=" << i << " ("
                                           << xs[i] << "," << ys[i] << ")";
        EXPECT_NEAR(gx[i], grad.x(), 1e-10) << "grad_x mismatch at i=" << i;
        EXPECT_NEAR(gy[i], grad.y(), 1e-10) << "grad_y mismatch at i=" << i;
    }
}

// 测试目标：批量查询在全自由地图（无障碍物）上返回全场恒定距离且梯度近零。
// 测试流程：构造 5×5 无障碍物地图，批量查询 8 个点。
// 预期结果：所有点距离一致（全自由场恒定），梯度模长 ≈ 0。
TEST(ESDFMapTest, BatchQueryFullyFreeMapConstantDistance) {
    const GridMap grid_map(1.0, 5, 5, Position{0.0, 0.0}, {});
    const ESDFMap esdf_map(grid_map);
    constexpr int kN = 8;
    const double xs[kN] = {0.5, 1.5, 2.5, 3.5, 0.5, 1.5, 2.5, 3.5};
    const double ys[kN] = {0.5, 0.5, 0.5, 0.5, 3.5, 3.5, 3.5, 3.5};
    double dists[kN], gx[kN], gy[kN];
    esdf_map.getDistAndGradBatch(xs, ys, kN, dists, gx, gy);
    const double expected_dist = esdf_map.getDist(2.0, 2.0);
    for (int i = 0; i < kN; ++i) {
        EXPECT_NEAR(dists[i], expected_dist, 1e-5)
            << "all free-space distances should be equal at i=" << i;
        EXPECT_NEAR(gx[i], 0.0, 1e-5)
            << "grad_x should be ~0 in free space at i=" << i;
        EXPECT_NEAR(gy[i], 0.0, 1e-5)
            << "grad_y should be ~0 in free space at i=" << i;
    }
}

}  // namespace
}  // namespace apa_post_processor
