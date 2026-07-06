#include <gtest/gtest.h>

#include <stdexcept>
#include <vector>

#include "spatial/grid_map.h"
#include "test_fixture_util.h"

namespace apa_post_processor {
namespace {

// GridMap 测试夹具，复用统一的 data/test.json 加载流程。
class GridMapTest : public DataJsonFixture {
   protected:
    GridMap loadGridMapFromTestJson() const
    {
        return GridMap::FromProto(getOptimizeRequest().environment());
    }
};

// 测试 GridMap 可从 test.json 的 environment 字段正确构造。
// 因为线上入口通过 DataLoader 反序列化请求，单测应覆盖同一路径并校验核心尺寸配置。
TEST_F(GridMapTest, BuildFromTestJsonEnvironmentSuccessfully)
{
    const GridMap grid_map = loadGridMapFromTestJson();

    EXPECT_DOUBLE_EQ(grid_map.getResolution(), 0.1);
    EXPECT_EQ(grid_map.getWidth(), 200);
    EXPECT_EQ(grid_map.getHeight(), 200);
    EXPECT_EQ(grid_map.getOccupancyData().size(), static_cast<std::size_t>(40000));
}

// 测试 test.json 中障碍物坐标被正确映射为占据栅格。
// 因为后续碰撞检测完全依赖栅格占据数据，必须确保 JSON 的 cells 字段被精确写入。
TEST_F(GridMapTest, OccupiedCellsFromTestJsonAreMarkedCorrectly)
{
    const GridMap grid_map = loadGridMapFromTestJson();

    EXPECT_TRUE(grid_map.isOccupied(3.0, 1.0));
    EXPECT_TRUE(grid_map.isOccupied(3.0, 1.1));
    EXPECT_TRUE(grid_map.isOccupied(3.0, 1.2));
    EXPECT_TRUE(grid_map.isOccupied(3.0, 1.3));
    EXPECT_FALSE(grid_map.isOccupied(2.9, 1.0));
    EXPECT_FALSE(grid_map.isOccupied(3.1, 1.0));
}

// 测试 getIndex(物理坐标) 在 test.json 地图上能映射到预期一维索引。
// 因为坐标映射是碰撞查询的核心路径，索引结果必须与地图参数严格一致。
TEST_F(GridMapTest, GetIndexByWorldMapsToExpectedFlattenIndexOnTestJsonMap)
{
    const GridMap grid_map = loadGridMapFromTestJson();

    EXPECT_EQ(grid_map.getIndex(-10.0, -10.0), 0);
    EXPECT_EQ(grid_map.getIndex(3.0, 1.0), 22130);
    EXPECT_EQ(grid_map.getIndex(9.99, 9.99), 39999);
}

// 测试 getIndex(物理坐标) 在 test.json 地图越界时返回 -1。
// 因为上层根据 -1 判断地图外点位，所以边界外坐标必须统一返回非法索引。
TEST_F(GridMapTest, GetIndexByWorldReturnsMinusOneWhenOutOfBoundsOnTestJsonMap)
{
    const GridMap grid_map = loadGridMapFromTestJson();

    EXPECT_EQ(grid_map.getIndex(-10.1, 0.1), -1);
    EXPECT_EQ(grid_map.getIndex(10.0, 0.1), -1);
    EXPECT_EQ(grid_map.getIndex(0.1, 10.0), -1);
}

// 测试 isOccupied 在物理坐标越界时采用保守占用策略。
// 因为路径规划通常将地图外视为不可通行区域，所以越界查询应返回 occupied 以避免越界穿越。
TEST_F(GridMapTest, IsOccupiedByWorldTreatsOutOfBoundsAsOccupied)
{
    const GridMap grid_map = loadGridMapFromTestJson();

    EXPECT_TRUE(grid_map.isOccupied(-10.1, 0.2));
    EXPECT_TRUE(grid_map.isOccupied(10.0, 0.2));
    EXPECT_TRUE(grid_map.isOccupied(0.2, 10.0));
}

// 测试构造函数在分辨率非法时抛出异常。
// 因为非正分辨率会导致坐标映射无意义且存在除零风险，所以必须在构造阶段拒绝。
TEST_F(GridMapTest, ConstructorThrowsWhenResolutionIsNotPositive)
{
    EXPECT_THROW((void)GridMap(0.0, 2.0, 2.0, Position{0.0, 0.0}, {}),
                 std::invalid_argument);
    EXPECT_THROW((void)GridMap(-0.1, 2.0, 2.0, Position{0.0, 0.0}, {}),
                 std::invalid_argument);
}

// 测试构造函数在宽高非法（非正）时抛出异常。
// 因为内部占据数组大小依赖 width*height，非正尺寸会导致地图语义无效，所以应在构造阶段拒绝。
TEST_F(GridMapTest, ConstructorThrowsWhenWidthOrHeightIsNotPositive)
{
    EXPECT_THROW((void)GridMap(0.5, 0, 2, Position{0.0, 0.0}, {}),
                 std::invalid_argument);
    EXPECT_THROW((void)GridMap(0.5, 4, 0, Position{0.0, 0.0}, {}),
                 std::invalid_argument);
    EXPECT_THROW((void)GridMap(0.5, -1, 2, Position{0.0, 0.0}, {}),
                 std::invalid_argument);
    EXPECT_THROW((void)GridMap(0.5, 4, -1, Position{0.0, 0.0}, {}),
                 std::invalid_argument);
}

// 测试通过 DataLoader + FromProto 的整链路加载场景。
// 因为系统入口是 json -> proto -> GridMap，这里验证链路末端的占据结果与样例一致。
TEST_F(GridMapTest, FromProtoBuildsGridMapFromTestJsonEnvironment)
{
    const GridMap grid_map = loadGridMapFromTestJson();

    EXPECT_TRUE(grid_map.isOccupied(3.0, 1.0));
    EXPECT_FALSE(grid_map.isOccupied(3.2, 1.0));
}

// 测试空白区域映射为未占用。
// 因为碰撞检测若把空白区域误判为障碍会触发假阳性，所以必须验证典型自由空间坐标返回 false。
TEST_F(GridMapTest, FreeSpaceIsCorrectlyMapped)
{
    const GridMap grid_map = loadGridMapFromTestJson();

    EXPECT_FALSE(grid_map.isOccupied(0.0, 0.0));
    EXPECT_FALSE(grid_map.isOccupied(-1.0, -1.0));
    EXPECT_FALSE(grid_map.isOccupied(2.0, 1.0));
}

// 测试障碍物区域映射为占用。
// 因为地图中障碍点是碰撞检测的核心输入，必须确认样例中障碍栅格不会被漏标。
TEST_F(GridMapTest, ObstacleCellsAreCorrectlyMapped)
{
    const GridMap grid_map = loadGridMapFromTestJson();

    EXPECT_TRUE(grid_map.isOccupied(3.0, 1.0));
    EXPECT_TRUE(grid_map.isOccupied(3.0, 1.1));
    EXPECT_TRUE(grid_map.isOccupied(3.0, 1.2));
    EXPECT_TRUE(grid_map.isOccupied(3.0, 1.3));
}

// 测试贴边坐标在浮点误差下仍映射到正确栅格。
// 因为 x/y 位于网格边界附近时最容易触发 floor 精度陷阱，所以这里验证边界稳定性和不越界行为。
TEST_F(GridMapTest, BoundaryExactnessTest)
{
    const GridMap grid_map = loadGridMapFromTestJson();

    EXPECT_EQ(grid_map.getIndex(-10.0 + 0.1, -10.0 + 0.1), 201);
    EXPECT_EQ(grid_map.getIndex(9.999999999, 9.999999999), 39999);
    EXPECT_EQ(grid_map.getIndex(10.0, 9.999999999), -1);
}

// 测试 getPosition 返回栅格中心坐标。
// origin 本身即为第 0 行第 0 列栅格的中心，因此 getPosition(0, 0) 应严格等于 origin。
TEST_F(GridMapTest, GetPositionReturnsCellCenter)
{
    const GridMap grid_map = loadGridMapFromTestJson();

    const auto position = grid_map.getPosition(0, 0);
    EXPECT_DOUBLE_EQ(position.x, grid_map.getOrigin().x);
    EXPECT_DOUBLE_EQ(position.y, grid_map.getOrigin().y);
}

// 测试 getPosition 对非零原点和不同分辨率仍返回正确中心。
// 因为栅格地图的原点与分辨率不固定，反推公式必须同时依赖两者。
TEST(GridMapPositionTest, GetPositionHandlesNonZeroOriginAndResolution)
{
    const GridMap grid_map(0.5, 4, 4, Position{-1.0, 2.0}, {});

    const auto position = grid_map.getPosition(2, 3);
    EXPECT_DOUBLE_EQ(position.x, -1.0 + 3.0 * 0.5);
    EXPECT_DOUBLE_EQ(position.y, 2.0 + 2.0 * 0.5);
}

// 测试 getPosition 与 getIndex 在合法栅格内互相自洽。
// 因为中心点必然落在对应栅格内，所以由索引反推的坐标应能映射回原索引。
TEST(GridMapPositionTest, GetPositionAndGetIndexAreConsistent)
{
    const GridMap grid_map(1.0, 5, 5, Position{0.0, 0.0},
                           {Position{2.0, 2.0}, Position{3.0, 2.0}});

    for (int row = 0; row < grid_map.getHeight(); ++row) {
        for (int col = 0; col < grid_map.getWidth(); ++col) {
            const auto position = grid_map.getPosition(row, col);
            EXPECT_EQ(grid_map.getIndex(position.x, position.y),
                      grid_map.getIndex(row, col));
        }
    }
}

// 测试 getPosition 可用于正确查询占据状态。
// 因为栅格中心是 occupancy 的代表点，返回中心后查询 isOccupied 应与 data_ 一致。
TEST(GridMapPositionTest, GetPositionCanQueryOccupancyCorrectly)
{
    const std::vector<Position> cells{Position{2.0, 2.0}, Position{3.0, 2.0}};
    const GridMap grid_map(1.0, 5, 5, Position{0.0, 0.0}, cells);

    const auto occupied_position = grid_map.getPosition(2, 2);
    EXPECT_TRUE(grid_map.isOccupied(occupied_position.x, occupied_position.y));

    const auto free_position = grid_map.getPosition(0, 0);
    EXPECT_FALSE(grid_map.isOccupied(free_position.x, free_position.y));
}

// 测试 getPosition 对越界行列索引进行 clamp。
// 因为外部调用可能传入边界外索引，安全做法是把结果限制在地图范围内而不是返回非法坐标。
TEST(GridMapPositionTest, GetPositionClampsOutOfBoundsIndices)
{
    const GridMap grid_map(1.0, 3, 3, Position{0.0, 0.0}, {});

    const auto bottom_left = grid_map.getPosition(-1, -1);
    EXPECT_DOUBLE_EQ(bottom_left.x, 0.0);
    EXPECT_DOUBLE_EQ(bottom_left.y, 0.0);

    const auto top_right = grid_map.getPosition(100, 100);
    EXPECT_DOUBLE_EQ(top_right.x, 2.0);
    EXPECT_DOUBLE_EQ(top_right.y, 2.0);

    const auto clamped_partial = grid_map.getPosition(-5, 1);
    EXPECT_DOUBLE_EQ(clamped_partial.x, 1.0);
    EXPECT_DOUBLE_EQ(clamped_partial.y, 0.0);
}

}  // namespace
}  // namespace apa_post_processor
