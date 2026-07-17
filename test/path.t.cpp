#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <vector>

#include "util/constants.h"
#include "util/path.h"
#include "util/trajectory_point.h"

namespace apa_post_processor {
namespace {

constexpr double TEST_MAX_GAP_DIST = 4.0 * DELTA_DIST;

void ExpectPoseNear(const Pose& pose, const Pose& expected) {
    EXPECT_NEAR(pose.x, expected.x, EPSILON);
    EXPECT_NEAR(pose.y, expected.y, EPSILON);
    EXPECT_NEAR(pose.theta, expected.theta, EPSILON);
}

Pose BuildPoseWithRawTheta(double x, double y, double theta) {
    Pose pose{x, y, 0.0};
    pose.theta = theta;
    return pose;
}

std::vector<TrajectoryPoint> CollectPathPoints(const Path& path) {
    std::vector<TrajectoryPoint> points;
    points.reserve(path.size());
    path.forEach([&points](const TrajectoryPoint& point) {
        points.emplace_back(point);
    });
    return points;
}

::apa::post_processor::Pose* AddProtoPoint(::apa::post_processor::Path& proto,
                                           double x, double y, double theta) {
    auto* point = proto.add_points();
    point->set_x(x);
    point->set_y(y);
    point->set_theta(theta);
    return point;
}

// 测试默认构造 Path 的状态查询场景。
// 因为空路径没有任何有效机动段，所以 empty、numManeuvers 和 size
// 都应返回空状态。
TEST(PathTest, DefaultConstructIsEmpty) {
    const Path path;

    EXPECT_TRUE(path.empty());
    EXPECT_EQ(path.numManeuvers(), 0U);
    EXPECT_EQ(path.size(), 0U);
}

// 测试基于 Pose 序列的模板构造函数。
// 因为该构造需要等价于依次 addPoint 后再 finalize，所以结果应与手动构建一致。
TEST(PathTest, ConstructFromPoseSequence) {
    const std::vector<Pose> poses = {
        Pose{0.0, 0.0, 0.0},
        Pose{1.0, 0.0, 0.0},
        Pose{2.0, 0.0, 0.0},
    };

    Path constructed_path(poses);

    Path manual_path;
    for (const auto& pose : poses) {
        manual_path.addPoint(pose);
    }
    manual_path.finalize();

    ASSERT_FALSE(constructed_path.empty());
    ASSERT_EQ(constructed_path.numManeuvers(), manual_path.numManeuvers());
    ASSERT_EQ(constructed_path.size(), manual_path.size());
    const auto constructed_points = CollectPathPoints(constructed_path);
    const auto manual_points = CollectPathPoints(manual_path);
    for (std::size_t i = 0; i < constructed_points.size(); ++i) {
        ExpectPoseNear(constructed_points[i], manual_points[i]);
        EXPECT_NEAR(constructed_points[i].getKappa(),
                    manual_points[i].getKappa(), EPSILON);
    }
}

// 测试空 Pose 序列构造 Path 的行为。
// 因为空序列不会产生任何路径点，所以结果应为空路径。
TEST(PathTest, ConstructFromEmptyPoseSequence) {
    const std::vector<Pose> poses;
    const Path path(poses);
    EXPECT_TRUE(path.empty());
    EXPECT_EQ(path.numManeuvers(), 0U);
    EXPECT_EQ(path.size(), 0U);
}

// 测试尾部机动段为空时的状态查询场景。
// 因为最后一个 Maneuver
// 缺少路径点意味着路径不可用，所以查询接口应统一视为空路径。
TEST(PathTest, EmptyTailManeuverMakesPathEmpty) {
    Path path;
    path.getManeuvers().emplace_back();

    EXPECT_TRUE(path.empty());
    EXPECT_EQ(path.numManeuvers(), 0U);
    EXPECT_EQ(path.size(), 0U);
}

// 测试非 const 访问器允许直接维护机动段的场景。
// 因为部分测试和内部流程需要白盒构造状态，所以 getManeuvers 应返回可修改引用。
TEST(PathTest, MutableGetManeuversAllowsStateConstruction) {
    Path path;
    path.getManeuvers().emplace_back(TrajectoryPoint{0.0, 0.0, 0.0},
                                     Direction::FORWARD);

    ASSERT_FALSE(path.empty());
    ASSERT_EQ(path.getManeuvers().size(), 1U);
    EXPECT_EQ(path.getManeuvers().at(0).direction, Direction::FORWARD);
}

// 测试 const 访问器只读观察机动段的场景。
// 因为调用方会在只读上下文统计路径结构，所以 const getManeuvers
// 应保留完整状态。
TEST(PathTest, ConstGetManeuversReadsExistingState) {
    Path path;
    path.addPoint(Pose{0.0, 0.0, 0.0});
    path.addPoint(Pose{2.0 * DELTA_DIST, 0.0, 0.0});
    const Path& const_path = path;

    const auto& maneuvers = const_path.getManeuvers();

    ASSERT_EQ(maneuvers.size(), 1U);
    EXPECT_EQ(maneuvers.at(0).direction, Direction::FORWARD);
}

// 测试单段路径的 size 计数场景。
// 因为单个 Maneuver 没有尖点复用问题，所以 size 应等于该段真实路径点数量。
TEST(PathTest, SizeCountsSingleManeuverPoints) {
    Path path;
    path.addPoint(Pose{0.0, 0.0, 0.0});
    path.addPoint(Pose{2.0 * DELTA_DIST, 0.0, 0.0});
    path.addPoint(Pose{3.0 * DELTA_DIST, 0.0, 0.0});

    EXPECT_EQ(path.numManeuvers(), 1U);
    EXPECT_EQ(path.size(), 3U);
}

// 测试多段路径的 size 计数场景。
// 因为换挡尖点会同时出现在前后两个 Maneuver，所以总点数应跳过后续段的重复首点。
TEST(PathTest, SizeSkipsDuplicatedCuspPointsAcrossManeuvers) {
    Path path;
    const TrajectoryPoint first{0.0, 0.0, 0.0};
    const TrajectoryPoint cusp{0.1, 0.0, 0.0};
    const TrajectoryPoint target{0.0, 0.0, 0.0};
    path.getManeuvers().emplace_back(std::vector<TrajectoryPoint>{first, cusp},
                                     Direction::FORWARD);
    path.getManeuvers().emplace_back(std::vector<TrajectoryPoint>{cusp, target},
                                     Direction::BACKWARD);

    ASSERT_EQ(path.numManeuvers(), 2U);
    ASSERT_EQ(path.getManeuvers().at(0).points.size(), 2U);
    ASSERT_EQ(path.getManeuvers().at(1).points.size(), 2U);
    EXPECT_EQ(path.size(), 3U);
}

// 测试空路径与单点路径的长度查询场景。
// 因为少于两个有效路径点时不存在轨迹线段，所以 length 应返回 0。
TEST(PathTest, LengthReturnsZeroForEmptyAndSinglePointPath) {
    Path path;

    EXPECT_DOUBLE_EQ(path.length(), 0.0);
    ASSERT_NO_THROW(path.addPoint(Pose{1.0, 2.0, 0.3}));
    EXPECT_DOUBLE_EQ(path.length(), 0.0);
}

// 测试 addPoint 在缓存有效时顺手增量维护路径长度的场景。
// 因为实时流程会高频读取
// length，所以追加新点后应直接得到最新长度且重复点不增加距离。
TEST(PathTest, LengthUpdatesIncrementallyWhenAddingPoints) {
    Path path;

    ASSERT_NO_THROW(path.addPoint(Pose{0.0, 0.0, 0.0}));
    EXPECT_DOUBLE_EQ(path.length(), 0.0);
    ASSERT_NO_THROW(path.addPoint(Pose{3.0, 4.0, 0.0}));
    EXPECT_DOUBLE_EQ(path.length(), 5.0);
    ASSERT_NO_THROW(path.addPoint(Pose{3.0, 4.0, 0.0}));
    EXPECT_DOUBLE_EQ(path.length(), 5.0);
    ASSERT_NO_THROW(path.addPoint(Pose{6.0, 8.0, 0.0}));
    EXPECT_DOUBLE_EQ(path.length(), 10.0);
}

// 测试外部直接修改底层 Maneuver 后的长度重算场景。
// 因为优化器可能通过可变引用改写路径，所以 getManeuvers 接触后 length
// 应无感重算缓存。
TEST(PathTest, LengthRecomputesAfterMutableManeuversInvalidatesCache) {
    Path path;
    ASSERT_NO_THROW(path.addPoint(Pose{0.0, 0.0, 0.0}));
    ASSERT_NO_THROW(path.addPoint(Pose{3.0, 4.0, 0.0}));
    EXPECT_DOUBLE_EQ(path.length(), 5.0);

    auto& maneuvers = path.getManeuvers();
    maneuvers.at(0).points.emplace_back(TrajectoryPoint{6.0, 8.0, 0.0});

    EXPECT_DOUBLE_EQ(path.length(), 10.0);
}

// 测试多段路径长度跳过重复尖点的场景。
// 因为 length 复用 forEach
// 的连续路径视图，所以换挡尖点只应参与一次相邻距离计算。
TEST(PathTest, LengthSkipsDuplicatedCuspAcrossManeuvers) {
    Path path;
    const TrajectoryPoint first{0.0, 0.0, 0.0};
    const TrajectoryPoint cusp{1.0, 0.0, 0.0};
    const TrajectoryPoint target{1.0, 2.0, 0.0};
    path.getManeuvers().emplace_back(std::vector<TrajectoryPoint>{first, cusp},
                                     Direction::FORWARD);
    path.getManeuvers().emplace_back(std::vector<TrajectoryPoint>{cusp, target},
                                     Direction::BACKWARD);

    EXPECT_DOUBLE_EQ(path.length(), 3.0);
}

// 测试 forEach 遍历空路径的场景。
// 因为空路径没有有效点，回调不应被调用，避免调用方拿到无意义数据。
TEST(PathTest, ForEachDoesNothingForEmptyPath) {
    Path path;
    std::size_t callback_count = 0U;

    path.forEach(
        [&callback_count](const TrajectoryPoint&) { ++callback_count; });

    EXPECT_EQ(callback_count, 0U);
}

// 测试 forEach 遍历多段路径并跳过重复尖点的场景。
// 因为对外遍历应呈现连续路径，所以换挡点只应出现一次且顺序保持不变。
TEST(PathTest, ForEachTraversesPathInOrderWithoutDuplicatedCusp) {
    Path path;
    const TrajectoryPoint first{0.0, 0.0, 0.0};
    const TrajectoryPoint cusp{0.1, 0.0, 0.0};
    const TrajectoryPoint target{0.0, 0.0, 0.0};
    path.getManeuvers().emplace_back(std::vector<TrajectoryPoint>{first, cusp},
                                     Direction::FORWARD);
    path.getManeuvers().emplace_back(std::vector<TrajectoryPoint>{cusp, target},
                                     Direction::BACKWARD);

    const auto points = CollectPathPoints(path);

    ASSERT_EQ(points.size(), 3U);
    ExpectPoseNear(points.at(0), first);
    ExpectPoseNear(points.at(1), cusp);
    ExpectPoseNear(points.at(2), target);
}

// 测试 forEach 在后续机动段只有尖点时的遍历场景。
// 因为这种段没有新增轨迹点，所以遍历应跳过该段，防止重复输出尖点。
TEST(PathTest, ForEachSkipsTrailingManeuverWithOnlyCuspPoint) {
    Path path;
    const TrajectoryPoint first{0.0, 0.0, 0.0};
    const TrajectoryPoint cusp{1.0, 0.0, 0.0};
    path.getManeuvers().emplace_back(std::vector<TrajectoryPoint>{first, cusp},
                                     Direction::FORWARD);
    path.getManeuvers().emplace_back(std::vector<TrajectoryPoint>{cusp},
                                     Direction::BACKWARD);

    const auto points = CollectPathPoints(path);

    ASSERT_EQ(points.size(), 2U);
    ExpectPoseNear(points.at(0), first);
    ExpectPoseNear(points.at(1), cusp);
    EXPECT_EQ(path.size(), 2U);
}

// 测试空 Path 转换为 JSON 字符串的场景。
// 因为空路径仍需要稳定日志结构，所以 toString 应输出空 maneuvers 数组。
TEST(PathTest, ToStringBuildsJsonTextForEmptyPath) {
    const Path path;

    EXPECT_EQ(path.toString(), std::string("{\"maneuvers\": []}"));
}

// 测试单段 Path 转换为 JSON 字符串的场景。
// 因为 Path 负责按顺序组织 Maneuver，所以单段输出应直接复用 Maneuver 的 JSON
// 表达。
TEST(PathTest, ToStringBuildsJsonTextForSingleManeuver) {
    Path path;
    path.getManeuvers().emplace_back(
        std::vector<TrajectoryPoint>{{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}},
        Direction::FORWARD);

    EXPECT_EQ(path.toString(),
              std::string("{\"maneuvers\": [{\"direction\": \"FORWARD\", "
                          "\"points\": [{\"x\": 0.00, \"y\": 0.00, "
                          "\"theta\": 0.00}, {\"x\": 1.00, "
                          "\"y\": 0.00, \"theta\": 0.00}]}]}"));
}

// 测试多段 Path 转换为 JSON 字符串的场景。
// 因为日志需要保留换挡边界，所以 toString 应按 Maneuver 原始顺序输出每一段。
TEST(PathTest, ToStringBuildsJsonTextForMultipleManeuvers) {
    Path path;
    const TrajectoryPoint first{0.0, 0.0, 0.0};
    const TrajectoryPoint cusp{1.0, 0.0, 0.0};
    const TrajectoryPoint target{0.5, 0.0, 0.0};
    path.getManeuvers().emplace_back(std::vector<TrajectoryPoint>{first, cusp},
                                     Direction::FORWARD);
    path.getManeuvers().emplace_back(std::vector<TrajectoryPoint>{cusp, target},
                                     Direction::BACKWARD);

    EXPECT_EQ(path.toString(),
              std::string("{\"maneuvers\": [{\"direction\": \"FORWARD\", "
                          "\"points\": [{\"x\": 0.00, \"y\": 0.00, "
                          "\"theta\": 0.00}, {\"x\": 1.00, "
                          "\"y\": 0.00, \"theta\": 0.00}]}, "
                          "{\"direction\": \"BACKWARD\", \"points\": "
                          "[{\"x\": 1.00, \"y\": 0.00, "
                          "\"theta\": 0.00}, {\"x\": 0.50, "
                          "\"y\": 0.00, \"theta\": 0.00}]}]}"));
}

// 测试空 Path 导出为扁平 protobuf 路径的场景。
// 因为响应字段需要可重复复用同一个 proto 对象，所以 toProto
// 应先清空旧点再输出空序列。
TEST(PathTest, ToProtoFlatClearsAndExportsEmptyPath) {
    const Path path;
    ::apa::post_processor::Path path_proto;
    AddProtoPoint(path_proto, 9.0, 9.0, 9.0);

    ASSERT_NO_THROW(path.toProto(&path_proto));

    EXPECT_EQ(path_proto.points_size(), 0);
}

// 测试多段 Path 导出为扁平 protobuf 路径的场景。
// 因为 optimized_path 只表达连续位姿序列，所以换挡尖点只应导出一次。
TEST(PathTest, ToProtoFlatExportsPointsWithoutDuplicatedCusp) {
    Path path;
    const TrajectoryPoint first{0.0, 0.0, 0.0};
    const TrajectoryPoint cusp{1.0, 0.0, 0.1};
    const TrajectoryPoint target{0.5, 0.0, 0.2};
    path.getManeuvers().emplace_back(std::vector<TrajectoryPoint>{first, cusp},
                                     Direction::FORWARD);
    path.getManeuvers().emplace_back(std::vector<TrajectoryPoint>{cusp, target},
                                     Direction::BACKWARD);
    ::apa::post_processor::Path path_proto;

    ASSERT_NO_THROW(path.toProto(&path_proto));

    ASSERT_EQ(path_proto.points_size(), 3);
    ExpectPoseNear(Pose::FromProto(path_proto.points(0)), first);
    ExpectPoseNear(Pose::FromProto(path_proto.points(1)), cusp);
    ExpectPoseNear(Pose::FromProto(path_proto.points(2)), target);
}

// 测试多段 Path 导出为嵌套 protobuf 机动段的场景。
// 因为 maneuvers 字段需要保留换挡边界，所以每段方向和段内尖点都应原样输出。
TEST(PathTest, ToProtoNestedExportsManeuversWithDirectionsAndCusp) {
    Path path;
    const TrajectoryPoint first{0.0, 0.0, 0.0};
    const TrajectoryPoint cusp{1.0, 0.0, 0.1};
    const TrajectoryPoint target{0.5, 0.0, 0.2};
    path.getManeuvers().emplace_back(std::vector<TrajectoryPoint>{first, cusp},
                                     Direction::FORWARD);
    path.getManeuvers().emplace_back(std::vector<TrajectoryPoint>{cusp, target},
                                     Direction::BACKWARD);
    ::google::protobuf::RepeatedPtrField<::apa::post_processor::Maneuver>
        maneuvers_proto;

    ASSERT_NO_THROW(path.toProto(&maneuvers_proto));

    ASSERT_EQ(maneuvers_proto.size(), 2);
    EXPECT_EQ(maneuvers_proto.Get(0).direction(),
              ::apa::post_processor::FORWARD);
    EXPECT_EQ(maneuvers_proto.Get(1).direction(),
              ::apa::post_processor::BACKWARD);
    ASSERT_EQ(maneuvers_proto.Get(0).points_size(), 2);
    ASSERT_EQ(maneuvers_proto.Get(1).points_size(), 2);
    ExpectPoseNear(Pose::FromProto(maneuvers_proto.Get(0).points(0)), first);
    ExpectPoseNear(Pose::FromProto(maneuvers_proto.Get(0).points(1)), cusp);
    ExpectPoseNear(Pose::FromProto(maneuvers_proto.Get(1).points(0)), cusp);
    ExpectPoseNear(Pose::FromProto(maneuvers_proto.Get(1).points(1)), target);
}

// 测试 Path 导出 protobuf 时拒绝空输出指针的场景。
// 因为调用方传入空指针属于接口误用，所以应抛出异常避免静默崩溃。
TEST(PathTest, ToProtoThrowsWhenOutputPointerIsNull) {
    const Path path;

    EXPECT_THROW(
        path.toProto(static_cast<::apa::post_processor::Path*>(nullptr)),
        std::invalid_argument);
    EXPECT_THROW(path.toProto(static_cast<::google::protobuf::RepeatedPtrField<
                                  ::apa::post_processor::Maneuver>*>(nullptr)),
                 std::invalid_argument);
}

// 测试 FromProto 接收空 protobuf 路径的非法输入场景。
// 因为空 proto 无法构成有效 Path，所以 FromProto 应抛出 invalid_argument
// 阻断脏数据。
TEST(PathTest, FromProtoThrowsWhenProtoHasNoPoints) {
    const ::apa::post_processor::Path proto;

    EXPECT_THROW((void)Path::FromProto(proto), std::invalid_argument);
}

// 测试 FromProto 从 protobuf 路径构造单段 Path 的场景。
// 因为外部数据以点序列传入，所以 FromProto 应逐点调用 addPoint
// 并保留推断出的方向。
TEST(PathTest, FromProtoBuildsPathAndInfersDirection) {
    ::apa::post_processor::Path proto;
    AddProtoPoint(proto, 0.0, 0.0, 0.0);
    AddProtoPoint(proto, 2.0 * DELTA_DIST, 0.0, 0.0);

    const Path path = Path::FromProto(proto);

    ASSERT_FALSE(path.empty());
    ASSERT_EQ(path.numManeuvers(), 1U);
    EXPECT_EQ(path.size(), 2U);
    EXPECT_EQ(path.getManeuvers().at(0).direction, Direction::FORWARD);
}

// 测试 FromProto 会复用 addPoint 的去重逻辑。
// 因为 protobuf 可能包含重复采样点，所以构造后的 Path 不应保留完全重合点。
TEST(PathTest, FromProtoDeduplicatesRepeatedPoints) {
    ::apa::post_processor::Path proto;
    AddProtoPoint(proto, 0.0, 0.0, 0.0);
    AddProtoPoint(proto, 0.0, 0.0, 0.0);
    AddProtoPoint(proto, 2.0 * DELTA_DIST, 0.0, 0.0);

    const Path path = Path::FromProto(proto);

    ASSERT_EQ(path.numManeuvers(), 1U);
    EXPECT_EQ(path.size(), 2U);
    EXPECT_EQ(path.getManeuvers().at(0).direction, Direction::FORWARD);
}

// 测试 FromProto 在路径发生换挡时构造多段 Maneuver 的场景。
// 因为 FromProto 依赖 addPoint
// 维护状态机，所以倒车输入应生成第二个机动段并保留尖点。
TEST(PathTest, FromProtoBuildsMultipleManeuversWhenDirectionChanges) {
    ::apa::post_processor::Path proto;
    AddProtoPoint(proto, 0.0, 0.0, 0.0);
    AddProtoPoint(proto, 5.0, 0.0, 0.0);
    AddProtoPoint(proto, 3.0, 0.0, 0.0);

    const Path path = Path::FromProto(proto);

    ASSERT_EQ(path.numManeuvers(), 2U);
    EXPECT_GT(path.size(), 3U);
    EXPECT_EQ(path.getManeuvers().at(0).direction, Direction::FORWARD);
    EXPECT_EQ(path.getManeuvers().at(1).direction, Direction::BACKWARD);
    ExpectPoseNear(path.getManeuvers().at(1).points.front(),
                   Pose{5.0, 0.0, 0.0});
    const auto points = CollectPathPoints(path);
    ASSERT_EQ(points.size(), path.size());
    ExpectPoseNear(points.front(), Pose{0.0, 0.0, 0.0});
    ExpectPoseNear(points.back(), Pose{3.0, 0.0, 0.0});
}

// 测试空路径首次添加点的场景。
// 因为 addPoint 负责初始化生命周期，所以空 Path 应生成一个 UNKNOWN
// 机动段并保存首点。
TEST(PathAddPointTest, EmptyPathCreatesUnknownManeuverWithSinglePoint) {
    Path path;
    const Pose point{1.0, 2.0, 0.3};

    ASSERT_NO_THROW(path.addPoint(point));

    const auto& maneuvers = path.getManeuvers();
    ASSERT_EQ(maneuvers.size(), 1U);
    EXPECT_EQ(maneuvers.at(0).direction, Direction::UNKNOWN);
    ASSERT_EQ(maneuvers.at(0).points.size(), 1U);
    ExpectPoseNear(maneuvers.at(0).points.at(0), point);
}

// 测试尾部机动段缺少路径点的损坏状态。
// 因为 empty() 会把这种状态视为空路径，所以 addPoint 应丢弃脏数据并重新初始化。
TEST(PathAddPointTest, CorruptedEmptyTailManeuverIsReinitialized) {
    Path path;
    path.getManeuvers().emplace_back();
    const Pose point{1.0, -2.0, -0.2};

    ASSERT_TRUE(path.empty());
    ASSERT_NO_THROW(path.addPoint(point));

    const auto& maneuvers = path.getManeuvers();
    ASSERT_EQ(maneuvers.size(), 1U);
    EXPECT_EQ(maneuvers.at(0).direction, Direction::UNKNOWN);
    ASSERT_EQ(maneuvers.at(0).points.size(), 1U);
    ExpectPoseNear(maneuvers.at(0).points.at(0), point);
}

// 测试连续添加完全相同点的去重场景。
// 因为重复轨迹点不会提供新的运动信息，所以第二次 addPoint 应直接忽略。
TEST(PathAddPointTest, IdenticalPointIsDeduplicated) {
    Path path;
    const Pose point{0.0, 0.0, 0.0};

    ASSERT_NO_THROW(path.addPoint(point));
    ASSERT_NO_THROW(path.addPoint(point));

    const auto& points = path.getManeuvers().at(0).points;
    EXPECT_EQ(points.size(), 1U);
}

// 测试距离和角度都小于 EPSILON 的噪声点。
// 因为这种偏差属于浮点或采样抖动，所以 addPoint 应保持原路径点数量不变。
TEST(PathAddPointTest, EpsilonNoisePointIsDeduplicated) {
    Path path;
    const Pose first{0.0, 0.0, 0.0};
    const Pose noise{0.5 * EPSILON, 0.0, 0.5 * EPSILON};

    ASSERT_NO_THROW(path.addPoint(first));
    ASSERT_NO_THROW(path.addPoint(noise));

    const auto& points = path.getManeuvers().at(0).points;
    EXPECT_EQ(points.size(), 1U);
}

// 测试相邻点距离略大于最大物理步长的插值场景。
// 因为距离只需要一次二分即可达标，所以最终应新增一个中点和一个目标点。
TEST(PathAddPointTest, SingleSplitInterpolationAddsMidPointAndTarget) {
    Path path;
    const Pose first{0.0, 0.0, 0.0};
    const Pose target{1.1 * TEST_MAX_GAP_DIST, 0.0, 0.0};

    ASSERT_NO_THROW(path.addPoint(first));
    ASSERT_NO_THROW(path.addPoint(target));

    const auto& points = path.getManeuvers().at(0).points;
    ASSERT_EQ(points.size(), 3U);
    ExpectPoseNear(points.at(1), Pose{0.55 * TEST_MAX_GAP_DIST, 0.0, 0.0});
    ExpectPoseNear(points.back(), target);
}

// 测试超长距离输入触发深度递归插值的场景。
// 因为 addPoint 必须保证物理连续性，所以所有最终相邻点间距都应小于最大步长。
TEST(PathAddPointTest, DeepInterpolationKeepsAllAdjacentDistancesBelowMaxGap) {
    Path path;
    const Pose first{0.0, 0.0, 0.0};
    const Pose target{3.5 * TEST_MAX_GAP_DIST, 0.0, 0.0};

    ASSERT_NO_THROW(path.addPoint(first));
    ASSERT_NO_THROW(path.addPoint(target));

    const auto& points = path.getManeuvers().at(0).points;
    ASSERT_GT(points.size(), 3U);
    ExpectPoseNear(points.front(), first);
    ExpectPoseNear(points.back(), target);
    for (std::size_t point_idx = 1U; point_idx < points.size(); ++point_idx) {
        const double dist =
            std::hypot(points.at(point_idx).x - points.at(point_idx - 1U).x,
                       points.at(point_idx).y - points.at(point_idx - 1U).y);
        EXPECT_LT(dist, TEST_MAX_GAP_DIST);
    }
}

// 测试明显正向纵向位移的意图推断场景。
// 因为位移投影为正且超过阈值，所以 UNKNOWN 机动段应确认为 DRIVE。
TEST(PathAddPointTest, ClearForwardMotionUpdatesUnknownToDrive) {
    Path path;

    ASSERT_NO_THROW(path.addPoint(Pose{0.0, 0.0, 0.0}));
    ASSERT_NO_THROW(path.addPoint(Pose{2.0 * DELTA_DIST, 0.0, 0.0}));

    const auto& maneuver = path.getManeuvers().at(0);
    EXPECT_EQ(maneuver.direction, Direction::FORWARD);
    EXPECT_EQ(maneuver.points.size(), 2U);
}

// 测试明显反向纵向位移的意图推断场景。
// 因为位移投影为负且超过阈值，所以 UNKNOWN 机动段应确认为 REVERSE。
TEST(PathAddPointTest, ClearBackwardMotionUpdatesUnknownToReverse) {
    Path path;

    ASSERT_NO_THROW(path.addPoint(Pose{0.0, 0.0, 0.0}));
    ASSERT_NO_THROW(path.addPoint(Pose{-2.0 * DELTA_DIST, 0.0, 0.0}));

    const auto& maneuver = path.getManeuvers().at(0);
    EXPECT_EQ(maneuver.direction, Direction::BACKWARD);
    EXPECT_EQ(maneuver.points.size(), 2U);
}

// 测试步长不足且没有明显旋转的微小位移场景。
// 因为运动意图还不稳定，所以当前机动段方向应继续保持 UNKNOWN。
TEST(PathAddPointTest, MicroMovementKeepsDirectionUnknown) {
    Path path;

    ASSERT_NO_THROW(path.addPoint(Pose{0.0, 0.0, 0.0}));
    ASSERT_NO_THROW(path.addPoint(Pose{0.5 * DELTA_DIST, 0.0, 0.0}));

    EXPECT_EQ(path.getManeuvers().at(0).direction, Direction::UNKNOWN);
}

// 测试纯横向侧滑且没有明显旋转的场景。
// 因为纵向投影落入死区，所以 addPoint 不应把横向噪声误判为前进或后退。
TEST(PathAddPointTest, PureLateralSlipKeepsDirectionUnknown) {
    Path path;

    ASSERT_NO_THROW(path.addPoint(Pose{0.0, 0.0, 0.0}));
    ASSERT_NO_THROW(path.addPoint(Pose{0.0, 1.5 * DELTA_DIST, 0.0}));

    EXPECT_EQ(path.getManeuvers().at(0).direction, Direction::UNKNOWN);
}

// 测试坐标不动但航向角明显变化的原地旋转场景。
// 因为距离不足但转角超过阈值，所以 UNKNOWN 机动段应确认为 PIVOT。
TEST(PathAddPointTest, InPlaceRotationUpdatesUnknownToPivot) {
    Path path;

    ASSERT_NO_THROW(path.addPoint(Pose{0.0, 0.0, 0.0}));
    ASSERT_NO_THROW(path.addPoint(Pose{0.0, 0.0, 0.1}));

    EXPECT_EQ(path.getManeuvers().at(0).direction, Direction::PIVOT);
}

// 测试多次小步前进依赖首点累积距离确立方向的场景。
// 因为 UNKNOWN 阶段以首点为锚点，所以第三个小步点应让方向瞬间确认为 DRIVE。
TEST(PathAddPointTest, AnchorAccumulationEventuallyInfersDrive) {
    Path path;

    ASSERT_NO_THROW(path.addPoint(Pose{0.0, 0.0, 0.0}));
    ASSERT_NO_THROW(path.addPoint(Pose{0.4 * DELTA_DIST, 0.0, 0.0}));
    EXPECT_EQ(path.getManeuvers().at(0).direction, Direction::UNKNOWN);
    ASSERT_NO_THROW(path.addPoint(Pose{0.8 * DELTA_DIST, 0.0, 0.0}));
    EXPECT_EQ(path.getManeuvers().at(0).direction, Direction::UNKNOWN);
    ASSERT_NO_THROW(path.addPoint(Pose{1.2 * DELTA_DIST, 0.0, 0.0}));

    EXPECT_EQ(path.getManeuvers().at(0).direction, Direction::FORWARD);
}

// 测试已经处于 DRIVE 时输入单独看不出方向的噪声点。
// 因为运动中的 UNKNOWN 点应被当前机动段吸收，所以不应生成新的 Maneuver。
TEST(PathAddPointTest, NoiseDuringDriveIsAppendedWithoutNewManeuver) {
    Path path;

    ASSERT_NO_THROW(path.addPoint(Pose{0.0, 0.0, 0.0}));
    ASSERT_NO_THROW(path.addPoint(Pose{2.0 * DELTA_DIST, 0.0, 0.0}));
    ASSERT_NO_THROW(
        path.addPoint(Pose{2.0 * DELTA_DIST, 0.5 * DELTA_DIST, 0.0}));

    const auto& maneuvers = path.getManeuvers();
    ASSERT_EQ(maneuvers.size(), 1U);
    EXPECT_EQ(maneuvers.at(0).direction, Direction::FORWARD);
    EXPECT_EQ(maneuvers.at(0).points.size(), 3U);
}

// 测试 DRIVE 运动中出现明确倒车意图的换挡场景。
// 因为方向反转需要保留尖点连续性，所以新 Maneuver 的首点必须复用上一段末点。
TEST(PathAddPointTest, DriveToReverseCreatesNewManeuverWithCuspPoint) {
    Path path;
    const Pose cusp{5.0, 0.0, 0.0};
    const Pose target{3.0, 0.0, 0.0};

    ASSERT_NO_THROW(path.addPoint(Pose{0.0, 0.0, 0.0}));
    ASSERT_NO_THROW(path.addPoint(cusp));
    ASSERT_NO_THROW(path.addPoint(target));

    const auto& maneuvers = path.getManeuvers();
    ASSERT_EQ(maneuvers.size(), 2U);
    EXPECT_EQ(maneuvers.at(1).direction, Direction::BACKWARD);
    ASSERT_FALSE(maneuvers.at(1).points.empty());
    ExpectPoseNear(maneuvers.at(1).points.front(), cusp);
    ExpectPoseNear(maneuvers.at(1).points.back(), target);
}

// 测试已知前进段中出现略小于默认采样间隔的明确倒车步长。
// 因为换挡点附近常有小步采样，所以已知方向应依赖纵向投影识别反向意图。
TEST(PathAddPointTest, DriveToReverseSplitsOnSubDeltaStep) {
    Path path;
    const Pose cusp{2.0 * DELTA_DIST, 0.0, 0.0};
    const Pose target{1.2 * DELTA_DIST, 0.0, 0.0};

    ASSERT_NO_THROW(path.addPoint(Pose{0.0, 0.0, 0.0}));
    ASSERT_NO_THROW(path.addPoint(cusp));
    ASSERT_NO_THROW(path.addPoint(target));

    const auto& maneuvers = path.getManeuvers();
    ASSERT_EQ(maneuvers.size(), 2U);
    EXPECT_EQ(maneuvers.at(0).direction, Direction::FORWARD);
    EXPECT_EQ(maneuvers.at(1).direction, Direction::BACKWARD);
    ExpectPoseNear(maneuvers.at(1).points.front(), cusp);
    ExpectPoseNear(maneuvers.at(1).points.back(), target);
}

// 测试已知倒车段中出现略小于默认采样间隔的明确前进步长。
// 因为真实轨迹采样会有浮点误差，所以已知方向的换挡检测不能被 DELTA_DIST
// 硬阈值吞掉。
TEST(PathAddPointTest, ReverseToDriveSplitsOnSubDeltaStep) {
    Path path;
    const Pose cusp{-2.0 * DELTA_DIST, 0.0, 0.0};
    const Pose target{-1.2 * DELTA_DIST, 0.0, 0.0};

    ASSERT_NO_THROW(path.addPoint(Pose{0.0, 0.0, 0.0}));
    ASSERT_NO_THROW(path.addPoint(cusp));
    ASSERT_NO_THROW(path.addPoint(target));

    const auto& maneuvers = path.getManeuvers();
    ASSERT_EQ(maneuvers.size(), 2U);
    EXPECT_EQ(maneuvers.at(0).direction, Direction::BACKWARD);
    EXPECT_EQ(maneuvers.at(1).direction, Direction::FORWARD);
    ExpectPoseNear(maneuvers.at(1).points.front(), cusp);
    ExpectPoseNear(maneuvers.at(1).points.back(), target);
}

// 测试 DRIVE 运动切换为原地旋转的换挡场景。
// 因为平动和 PIVOT 是不同意图，所以应生成新 Maneuver 并复用上一段末点作为尖点。
TEST(PathAddPointTest, DriveToPivotCreatesNewManeuverWithCuspPoint) {
    Path path;
    const Pose cusp{5.0, 0.0, 0.0};
    const Pose target{5.0, 0.0, 1.0};

    ASSERT_NO_THROW(path.addPoint(Pose{0.0, 0.0, 0.0}));
    ASSERT_NO_THROW(path.addPoint(cusp));
    ASSERT_NO_THROW(path.addPoint(target));

    const auto& maneuvers = path.getManeuvers();
    ASSERT_EQ(maneuvers.size(), 2U);
    EXPECT_EQ(maneuvers.at(1).direction, Direction::PIVOT);
    ExpectPoseNear(maneuvers.at(1).points.front(), cusp);
    ExpectPoseNear(maneuvers.at(1).points.back(), target);
}

// 测试从原地旋转切回平动行驶的换挡场景。
// 因为 PIVOT 结束后车辆会重新发生纵向位移，所以应再次生成新 Maneuver
// 并保留尖点。
TEST(PathAddPointTest, PivotToDriveCreatesNewManeuverWithCuspPoint) {
    Path path;
    const Pose pivot_start{0.0, 0.0, 0.0};
    const Pose pivot_end{0.0, 0.0, 1.0};
    const Pose drive_target{2.0 * DELTA_DIST * std::cos(1.0),
                            2.0 * DELTA_DIST * std::sin(1.0), 1.0};

    ASSERT_NO_THROW(path.addPoint(pivot_start));
    ASSERT_NO_THROW(path.addPoint(pivot_end));
    ASSERT_NO_THROW(path.addPoint(drive_target));

    const auto& maneuvers = path.getManeuvers();
    ASSERT_EQ(maneuvers.size(), 2U);
    EXPECT_EQ(maneuvers.at(0).direction, Direction::PIVOT);
    EXPECT_EQ(maneuvers.at(1).direction, Direction::FORWARD);
    ASSERT_FALSE(maneuvers.at(1).points.empty());
    ExpectPoseNear(maneuvers.at(1).points.front(), pivot_end);
    ExpectPoseNear(maneuvers.at(1).points.back(), drive_target);
}

// 测试航向角跨越 pi 与 -pi 边界时的方向推断场景。
// 因为 addPoint 使用 remainder 计算角差，所以极小回绕不应被误判为 PIVOT。
TEST(PathAddPointTest, AngleWrapAroundDoesNotTriggerFalsePivot) {
    Path path;

    ASSERT_NO_THROW(path.addPoint(Pose{0.0, 0.0, 3.14}));
    ASSERT_NO_THROW(path.addPoint(Pose{-2.0 * DELTA_DIST, 0.0, -3.14}));

    EXPECT_EQ(path.getManeuvers().at(0).direction, Direction::FORWARD);
}

// 测试插值时航向角跨越 pi 与 -pi 边界的场景。
// 因为中点角度应沿最短角差平滑过渡，所以不应退化成接近 0.02rad 的算术平均。
TEST(PathAddPointTest, InterpolationUsesWrappedAngleForMidPoints) {
    Path path;
    const Pose first = BuildPoseWithRawTheta(0.0, 0.0, 3.14);
    const Pose target = BuildPoseWithRawTheta(10.0 * DELTA_DIST, 0.0, -3.10);

    ASSERT_NO_THROW(path.addPoint(first));
    ASSERT_NO_THROW(path.addPoint(target));

    std::vector<TrajectoryPoint> points;
    points.reserve(path.size());
    path.forEach([&points](const TrajectoryPoint& point) {
        points.emplace_back(point);
    });

    ASSERT_GT(points.size(), 3U);
    EXPECT_GT(std::abs(points.at(1).theta), 3.0);
    EXPECT_GT(std::abs(points.at(points.size() / 2U).theta), 3.0);
    EXPECT_GT(std::abs(points.at(1).theta - 0.02), 1.0);
    ExpectPoseNear(points.front(), first);
    ExpectPoseNear(points.back(), target);
}

// 测试纯直线路径的流式曲率计算。
// 因为三点共线时叉乘为 0，所以所有点（含插值生成的中间点）的曲率应安全保持为
// 0。
TEST(PathCurvatureTest, StraightLineCurvature) {
    Path path;
    for (std::size_t i = 0; i < 5; ++i) {
        ASSERT_NO_THROW(path.addPoint(Pose{0.5 * i, 0.0, 0.0}));
    }
    ASSERT_NO_THROW(path.finalize());

    const auto points = CollectPathPoints(path);
    ASSERT_GE(points.size(), 5U);
    for (const auto& point : points) {
        EXPECT_DOUBLE_EQ(point.getKappa(), 0.0);
    }
}

// 测试等半径圆弧路径的流式曲率计算。
// 因为外接圆法对圆上三点能恢复精确曲率，所以中间点曲率应接近 1/R 且左转为正。
TEST(PathCurvatureTest, ConstantRadiusArc) {
    constexpr double R = 5.0;
    constexpr double kappa_expected = 1.0 / R;
    // 采用小弧长步进，避免触发 MAX_GAP_DIST 插值，确保点严格落在圆上
    constexpr double arc_delta = 0.02;
    Path path;
    for (std::size_t i = 0; i < 5; ++i) {
        const double theta_center = -0.5 * PI + i * arc_delta;
        const double x = R * std::cos(theta_center);
        const double y = R + R * std::sin(theta_center);
        const double heading = theta_center + 0.5 * PI;
        ASSERT_NO_THROW(path.addPoint(Pose{x, y, heading}));
    }
    ASSERT_NO_THROW(path.finalize());

    const auto points = CollectPathPoints(path);
    ASSERT_EQ(points.size(), 5U);
    for (std::size_t i = 1; i + 1 < points.size(); ++i) {
        EXPECT_NEAR(points.at(i).getKappa(), kappa_expected, 1e-3);
        EXPECT_GT(points.at(i).getKappa(), 0.0);
    }
    EXPECT_NEAR(points.front().getKappa(), kappa_expected, 1e-3);
    EXPECT_NEAR(points.back().getKappa(), kappa_expected, 1e-3);
}

// 测试换挡尖点处两段 Maneuver 的曲率隔离。
// 因为换挡后当前段不会再有下一点，所以前段末尾点必须继承前段倒数第二点曲率，且不能出现无穷或
// NaN。
TEST(PathCurvatureTest, GearShiftCuspIsolation) {
    constexpr double R = 5.0;
    // 小弧长步进，避免插值破坏圆弧几何
    constexpr double arc_delta = 0.02;
    Path path;
    // 前进段：圆上 4 个点，左转曲率为正
    for (std::size_t i = 0; i < 4; ++i) {
        const double theta_center = -0.5 * PI + i * arc_delta;
        const double x = R * std::cos(theta_center);
        const double y = R + R * std::sin(theta_center);
        const double heading = theta_center + 0.5 * PI;
        ASSERT_NO_THROW(path.addPoint(Pose{x, y, heading}));
    }
    // 后退段：沿 heading 反方向直线后退 3 个点
    const auto& forward_maneuver = path.getManeuvers().at(0);
    const Pose& cusp = forward_maneuver.points.back();
    const double backward_step = 0.1;
    const double cos_heading = std::cos(cusp.theta);
    const double sin_heading = std::sin(cusp.theta);
    for (std::size_t i = 1; i <= 3; ++i) {
        const double x = cusp.x - i * backward_step * cos_heading;
        const double y = cusp.y - i * backward_step * sin_heading;
        ASSERT_NO_THROW(path.addPoint(Pose{x, y, cusp.theta}));
    }
    ASSERT_NO_THROW(path.finalize());

    ASSERT_EQ(path.numManeuvers(), 2U);
    const auto& maneuvers = path.getManeuvers();
    const auto& forward_points = maneuvers.at(0).points;
    const auto& backward_points = maneuvers.at(1).points;
    ASSERT_GE(forward_points.size(), 2U);
    ASSERT_GE(backward_points.size(), 2U);

    const double cusp_kappa = forward_points.back().getKappa();
    const double prev_kappa =
        forward_points[forward_points.size() - 2].getKappa();
    EXPECT_DOUBLE_EQ(cusp_kappa, prev_kappa);
    EXPECT_TRUE(std::isfinite(cusp_kappa));
    EXPECT_FALSE(std::isnan(cusp_kappa));

    // 前进段中间点应接近圆弧曲率，后退段直线点应为 0，验证两段互不干扰
    for (std::size_t i = 1; i + 1 < forward_points.size(); ++i) {
        EXPECT_NEAR(forward_points.at(i).getKappa(), 1.0 / R, 1e-3);
    }
    for (const auto& point : backward_points) {
        EXPECT_NEAR(point.getKappa(), 0.0, 1e-12);
    }
}

// 测试长距离输入触发递归插值后的曲率计算。
// 因为插值产生的中间点也会走完整 addPoint
// 流程，所以所有点都应被计算且不会崩溃。
TEST(PathCurvatureTest, RecursiveInterpolationCurvature) {
    Path path;
    ASSERT_NO_THROW(path.addPoint(Pose{0.0, 0.0, 0.0}));
    ASSERT_NO_THROW(path.addPoint(Pose{20.0, 0.0, 0.0}));
    ASSERT_NO_THROW(path.finalize());

    const auto points = CollectPathPoints(path);
    ASSERT_GT(points.size(), 2U);
    for (const auto& point : points) {
        EXPECT_TRUE(std::isfinite(point.getKappa()));
        EXPECT_FALSE(std::isnan(point.getKappa()));
    }
}

// 测试 5cm 采样下坐标量化锯齿不会被短窗口外接圆放大成高频曲率毛刺。
// 因为物理窗口会跨越多个采样点，所以轻微横向抖动下曲率应保持在合理范围内。
TEST(PathCurvatureTest, QuantizedPolylineNoiseIsSmoothedByPhysicalWindow) {
    Path path;
    constexpr std::size_t num_points = 80;
    for (std::size_t i = 0; i < num_points; ++i) {
        const double x = static_cast<double>(i) * DELTA_DIST;
        const double y = (i % 2U == 0U) ? 0.001 : -0.001;
        ASSERT_NO_THROW(path.addPoint(Pose{x, y, 0.0}));
    }
    ASSERT_NO_THROW(path.finalize());

    const auto points = CollectPathPoints(path);
    ASSERT_EQ(points.size(), num_points);
    constexpr std::size_t boundary_window_points = 8;
    for (std::size_t i = boundary_window_points;
         i + boundary_window_points < points.size(); ++i) {
        const auto& point = points.at(i);
        EXPECT_TRUE(std::isfinite(point.getKappa()));
        EXPECT_LT(std::abs(point.getKappa()), 0.05);
    }
}

// 测试超短路径在 finalize 后的鲁棒性。
// 因为点数不足时无法计算曲率，所以不应越界，kapp 保持初始 0 即可。
TEST(PathCurvatureTest, ShortManeuverRobustness) {
    Path path;
    ASSERT_NO_THROW(path.addPoint(Pose{0.0, 0.0, 0.0}));
    // 步长小于 MAX_GAP_DIST，避免插值，确保仅有两个原始点
    ASSERT_NO_THROW(path.addPoint(Pose{0.1, 0.0, 0.0}));
    ASSERT_NO_THROW(path.finalize());

    const auto& points = path.getManeuvers().at(0).points;
    ASSERT_EQ(points.size(), 2U);
    EXPECT_DOUBLE_EQ(points.at(0).getKappa(), 0.0);
    EXPECT_DOUBLE_EQ(points.at(1).getKappa(), 0.0);
}

// 测试 S 型曲线的曲率符号变化与拐点特性。
// 揉库轨迹常见从左转到右转的连续切换，验证曲率符号的严格反转与拐点归零。
TEST(PathCurvatureTest, SCurveInflectionPoint) {
    Path path;
    constexpr double dx = 0.05;
    constexpr std::size_t num_points =
        static_cast<std::size_t>(2.0 * PI / dx) + 1U;

    for (std::size_t i = 0; i < num_points; ++i) {
        const double x = i * dx;
        const double y = std::sin(x);
        const double heading = std::atan(std::cos(x));
        ASSERT_NO_THROW(path.addPoint(Pose{x, y, heading}));
    }
    ASSERT_NO_THROW(path.finalize());

    const auto points = CollectPathPoints(path);
    ASSERT_GT(points.size(), 10U);

    double max_negative_kappa = 0.0;
    double max_positive_kappa = 0.0;
    double inflection_min_abs_kappa = 1.0;

    // 从索引 1 遍历到倒数第 2 个点，避开首尾继承点进行纯数学校验
    for (std::size_t i = 1; i < points.size() - 1; ++i) {
        const auto& point = points[i];
        // 1. 左半段 (0, PI) 验证：严格要求全区段为右转（曲率为负）
        if (point.x > 0.1 && point.x < PI - 0.1) {
            EXPECT_LT(point.getKappa(), 0.0)
                << "Curvature should be negative in (0, PI) at x = " << point.x;
            max_negative_kappa = std::min(max_negative_kappa, point.getKappa());
        }
        // 2. 右半段 (PI, 2PI) 验证：严格要求全区段为左转（曲率为正）
        if (point.x > PI + 0.1 && point.x < 2.0 * PI - 0.1) {
            EXPECT_GT(point.getKappa(), 0.0)
                << "Curvature should be positive in (PI, 2PI) at x = "
                << point.x;
            max_positive_kappa = std::max(max_positive_kappa, point.getKappa());
        }
        // 3. 拐点 (x = PI) 附近验证
        if (std::abs(point.x - PI) < dx * 1.5) {
            inflection_min_abs_kappa =
                std::min(inflection_min_abs_kappa, std::abs(point.getKappa()));
        }
    }
    // 校验极值是否接近理论值 (-1 和 1)
    EXPECT_LT(max_negative_kappa, -0.8);
    EXPECT_GT(max_positive_kappa, 0.8);
    // 拐点曲率应极小 (由于离散步长 dx=0.05，理论上应小于 0.05)
    EXPECT_LT(inflection_min_abs_kappa, 0.05);
    // 端点曲率由独立角度差分计算（前/后向差商），不再简单复制邻点值。
    // 对光滑曲线应与邻点接近，但不要求完全相等。
    EXPECT_NEAR(points.front().getKappa(), points[1].getKappa(), 1e-2);
    EXPECT_NEAR(points.back().getKappa(), points[points.size() - 2].getKappa(),
                1e-2);
}

// 测试 addPoint 过程中不提前计算曲率。
// 因为曲率计算时机已延后到 finalize()，所以调用 finalize() 之前所有点的 kappa
// 都应保持未设置状态。
TEST(PathCurvatureTest, CurvatureIsUnsetBeforeFinalize) {
    Path path;
    for (std::size_t i = 0; i < 5; ++i) {
        // 使用小于 DELTA_DIST 的步长，避免触发插值，确保点数可控
        ASSERT_NO_THROW(path.addPoint(Pose{0.5 * DELTA_DIST * i, 0.0, 0.0}));
    }

    const auto& points = path.getManeuvers().at(0).points;
    ASSERT_EQ(points.size(), 5U);
    for (const auto& point : points) {
        EXPECT_FALSE(point.hasKappa())
            << "kappa should not be set before finalize()";
    }
}

// 测试 finalize() 对所有机动段统一批量计算曲率。
// 因为 addPoint 不再刷新曲率，换挡产生的每个 maneuver 都应在 finalize() 后得到
// 内部点的曲率，且首尾端点通过继承获得曲率。
TEST(PathCurvatureTest, FinalizeComputesCurvatureForAllManeuvers) {
    constexpr double R = 5.0;
    constexpr double arc_delta = 0.02;
    Path path;
    // 前进段：圆上 4 个点
    for (std::size_t i = 0; i < 4; ++i) {
        const double theta_center = -0.5 * PI + i * arc_delta;
        const double x = R * std::cos(theta_center);
        const double y = R + R * std::sin(theta_center);
        const double heading = theta_center + 0.5 * PI;
        ASSERT_NO_THROW(path.addPoint(Pose{x, y, heading}));
    }
    // 后退段：沿 heading 反方向直线后退 3 个点
    const auto& forward_maneuver = path.getManeuvers().at(0);
    const Pose& cusp = forward_maneuver.points.back();
    const double backward_step = 0.1;
    const double cos_heading = std::cos(cusp.theta);
    const double sin_heading = std::sin(cusp.theta);
    for (std::size_t i = 1; i <= 3; ++i) {
        const double x = cusp.x - i * backward_step * cos_heading;
        const double y = cusp.y - i * backward_step * sin_heading;
        ASSERT_NO_THROW(path.addPoint(Pose{x, y, cusp.theta}));
    }
    ASSERT_NO_THROW(path.finalize());

    ASSERT_EQ(path.numManeuvers(), 2U);
    const auto& maneuvers = path.getManeuvers();
    const auto& forward_points = maneuvers.at(0).points;
    const auto& backward_points = maneuvers.at(1).points;
    ASSERT_GE(forward_points.size(), 2U);
    ASSERT_GE(backward_points.size(), 2U);

    for (const auto& point : forward_points) {
        EXPECT_TRUE(point.hasKappa())
            << "forward maneuver point should have kappa after finalize()";
    }
    for (const auto& point : backward_points) {
        EXPECT_TRUE(point.hasKappa())
            << "backward maneuver point should have kappa after finalize()";
    }
    EXPECT_NEAR(forward_points.front().getKappa(), forward_points[1].getKappa(),
                EPSILON);
    EXPECT_NEAR(forward_points.back().getKappa(),
                forward_points[forward_points.size() - 2].getKappa(), EPSILON);
    EXPECT_NEAR(backward_points.front().getKappa(),
                backward_points[1].getKappa(), EPSILON);
    EXPECT_NEAR(backward_points.back().getKappa(),
                backward_points[backward_points.size() - 2].getKappa(),
                EPSILON);
}
}  // namespace
}  // namespace apa_post_processor
