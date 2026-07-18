#pragma once

#include <algorithm>
#include <array>
#include <atomic>
#include <cctype>
#include <charconv>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <ctime>
#include <filesystem>
#include <functional>
#include <initializer_list>
#include <iomanip>
#include <iterator>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <optional>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <tuple>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <vector>

#include "../spatial/esdf_map.h"
#include "../spatial/grid_map.h"
#include "../vehicle/vehicle_footprint_model.h"
#include "../vehicle/vehicle_params.h"
#include "path.h"
#include "pose.h"
#include "trajectory.h"
#include "type_traits.h"

namespace apa_post_processor::visualizer {
using PlotStyle = std::unordered_map<std::string, std::string>;

template <typename T>
struct is_shared_ptr : std::false_type {};
template <typename T>
struct is_shared_ptr<std::shared_ptr<T>> : std::true_type {};
template <typename T>
inline constexpr bool is_shared_ptr_v = is_shared_ptr<T>::value;
template <typename T>
struct is_reference_wrapper : std::false_type {};
template <typename T>
struct is_reference_wrapper<std::reference_wrapper<T>> : std::true_type {};
template <typename T>
inline constexpr bool is_reference_wrapper_v = is_reference_wrapper<T>::value;

struct Pen {
    static constexpr const char* BLACK = "#000000";
    static constexpr const char* WHITE = "#FFFFFF";
    static constexpr const char* GRAY = "#D3D3D3";
    static constexpr const char* RED = "#FF0000";
    static constexpr const char* YELLOW = "#FFC400";
    static constexpr const char* BLUE = "#00AEFF";
    static constexpr const char* GREEN = "#00A35A";
    static constexpr const char* ORANGE = "#FF6600";
    static constexpr const char* PURPLE = "#800080";
};

// 视场与坐标变换核心：保持与 VisualizerAVP 相同的世界坐标自适应思路。
class AuspexMatrix {
   public:
    AuspexMatrix() = default;
    // 将位于[dist_min,dist_max]内的距离值基于幂变化t=1-(1+d/10)^-2映射为RGB值，在近处保持高分辨率
    static std::tuple<int, int, int> DistToRGB(double dist,
                                               double dist_min = 0.0,
                                               double dist_max = 66.0) {
        // 根据t值从这条路径上选择RGB颜色
        static constexpr std::array<std::array<int, 3>, 8> RGB_PATH = {{
            {{0, 0, 0}},       // 0: Black
            {{255, 0, 0}},     // 1: Red
            {{255, 255, 0}},   // 2: Yellow
            {{0, 255, 0}},     // 3: Green
            {{0, 255, 255}},   // 4: Cyan
            {{0, 0, 255}},     // 5: Blue
            {{255, 0, 255}},   // 6: Magenta
            {{255, 255, 255}}  // 7: White
        }};
        constexpr auto NUM_EDGES = static_cast<int>(RGB_PATH.size()) - 1;
        // 计算dist_min, dist_max对应的原始t值
        const double t_min = 1.0 - std::pow(1.0 + dist_min / 10.0, -2.0),
                     t_max = 1.0 - std::pow(1.0 + dist_max / 10.0, -2.0);
        if (t_max <= t_min + 1e-6) {
            return std::make_tuple(0, 0, 0);
        }
        // 计算当前距离的原始t值再映射到[0,1]之间
        double t_raw = 1.0 - std::pow(1.0 + dist / 10.0, -2.0),
               t = (t_raw - t_min) / (t_max - t_min);
        t = std::clamp(t, 0.0, 1.0);
        // 将t映射到RGB路径上，计算当前t对应的边索引和插值因子
        double pos = t * NUM_EDGES;
        int idx = std::min(static_cast<int>(pos), NUM_EDGES - 1);
        double frac = pos - idx;
        // 确定t处于哪条边之后通过线性插值的方法计算RGB值
        auto lerpChannelFunc = [frac](int start, int end) {
            return std::clamp(
                static_cast<int>(std::round((1.0 - frac) * start + frac * end)),
                0, 255);
        };
        return std::make_tuple(
            lerpChannelFunc(RGB_PATH[idx][0], RGB_PATH[idx + 1][0]),
            lerpChannelFunc(RGB_PATH[idx][1], RGB_PATH[idx + 1][1]),
            lerpChannelFunc(RGB_PATH[idx][2], RGB_PATH[idx + 1][2]));
    }
    // 重置本帧坐标范围，默认中心用于无数据时保存空图排障。
    void resetDrawState(double meter_per_pixel) {
        min_x_ = std::numeric_limits<double>::max();
        min_y_ = std::numeric_limits<double>::max();
        max_x_ = std::numeric_limits<double>::lowest();
        max_y_ = std::numeric_limits<double>::lowest();
        world_center_x_ = 0.0;
        world_center_y_ = 0.0;
        meters_per_pixel_x_ = meter_per_pixel;
        meters_per_pixel_y_ = meter_per_pixel;
        has_auto_pan_anchor_ = false;
    }
    // 使用单点扩展世界坐标包围盒。
    void updateRng(double x, double y) {
        min_x_ = std::min(min_x_, x);
        max_x_ = std::max(max_x_, x);
        min_y_ = std::min(min_y_, y);
        max_y_ = std::max(max_y_, y);
    }
    // 更新自动平移锚点，通常使用自车后轴中心。
    void updateAutoPanAnchor(double x, double y) {
        auto_pan_anchor_x_ = x;
        auto_pan_anchor_y_ = y;
        has_auto_pan_anchor_ = true;
    }
    // 判断当前包围盒是否有效。
    bool hasValidRange() const { return min_x_ <= max_x_ && min_y_ <= max_y_; }
    // 判断点是否落在当前可视范围外扩区域内。
    bool inRng(double x, double y) const {
        return !hasValidRange() ||
               (min_x_ - default_dist_ <= x && x <= max_x_ + default_dist_ &&
                min_y_ - default_dist_ <= y && y <= max_y_ + default_dist_);
    }
    // 根据已收集范围更新坐标变换，axis equal 开启时保持 x/y 同比例。
    void updateTransformFromRange(int canvas_w, int canvas_h,
                                  double view_padding_ratio,
                                  double min_dynamic_meter_per_pixel,
                                  double max_dynamic_meter_per_pixel) {
        if (!hasValidRange()) {
            return;
        }
        const double range_center_x = (min_x_ + max_x_) * 0.5;
        const double range_center_y = (min_y_ + max_y_) * 0.5;
        const double span_x = std::max(max_x_ - min_x_, 2.0 * default_dist_);
        const double span_y = std::max(max_y_ - min_y_, 2.0 * default_dist_);
        const double raw_mpp_x =
            (span_x * view_padding_ratio) / static_cast<double>(canvas_w);
        const double raw_mpp_y =
            (span_y * view_padding_ratio) / static_cast<double>(canvas_h);
        const double equal_mpp = std::clamp(std::max(raw_mpp_x, raw_mpp_y),
                                            min_dynamic_meter_per_pixel,
                                            max_dynamic_meter_per_pixel);
        if (axis_equal_enabled_) {
            meters_per_pixel_x_ = equal_mpp;
            meters_per_pixel_y_ = equal_mpp;
        } else {
            meters_per_pixel_x_ =
                std::clamp(raw_mpp_x, min_dynamic_meter_per_pixel,
                           max_dynamic_meter_per_pixel);
            meters_per_pixel_y_ =
                std::clamp(raw_mpp_y, min_dynamic_meter_per_pixel,
                           max_dynamic_meter_per_pixel);
        }
        double target_center_x = range_center_x;
        double target_center_y = range_center_y;
        if (auto_pan_enabled_ && has_auto_pan_anchor_) {
            const double view_half_x =
                static_cast<double>(canvas_w) * 0.5 * meters_per_pixel_x_;
            const double view_half_y =
                static_cast<double>(canvas_h) * 0.5 * meters_per_pixel_y_;
            const double inner_half_x = view_half_x * auto_pan_inner_ratio_;
            const double inner_half_y = view_half_y * auto_pan_inner_ratio_;
            const double anchor_dx = auto_pan_anchor_x_ - target_center_x;
            const double anchor_dy = auto_pan_anchor_y_ - target_center_y;
            if (anchor_dx > inner_half_x) {
                target_center_x += anchor_dx - inner_half_x;
            } else if (anchor_dx < -inner_half_x) {
                target_center_x += anchor_dx + inner_half_x;
            }
            if (anchor_dy > inner_half_y) {
                target_center_y += anchor_dy - inner_half_y;
            } else if (anchor_dy < -inner_half_y) {
                target_center_y += anchor_dy + inner_half_y;
            }
        }
        world_center_x_ = target_center_x;
        world_center_y_ = target_center_y;
    }
    // 世界坐标转像素坐标。
    cv::Point worldToPixel(double x, double y, int canvas_w,
                           int canvas_h) const {
        return cv::Point(static_cast<int>(std::lround(
                             static_cast<double>(canvas_w) * 0.5 +
                             (x - world_center_x_) / meters_per_pixel_x_)),
                         static_cast<int>(std::lround(
                             static_cast<double>(canvas_h) * 0.5 -
                             (y - world_center_y_) / meters_per_pixel_y_)));
    }
    // 像素 x 转世界 x。
    double pixelToWorldX(int pixel_x, int canvas_w) const {
        return world_center_x_ + (static_cast<double>(pixel_x) -
                                  static_cast<double>(canvas_w) * 0.5) *
                                     meters_per_pixel_x_;
    }
    // 像素 y 转世界 y。
    double pixelToWorldY(int pixel_y, int canvas_h) const {
        return world_center_y_ - (static_cast<double>(pixel_y) -
                                  static_cast<double>(canvas_h) * 0.5) *
                                     meters_per_pixel_y_;
    }
    // 设置缺省显示半径。
    void setDefaultDist(double default_dist) { default_dist_ = default_dist; }
    // 设置是否保持 x/y 等比例。
    void setAxisEqualEnabled(bool enabled) { axis_equal_enabled_ = enabled; }
    // 设置是否启用自动平移。
    void setAutoPanEnabled(bool enabled) { auto_pan_enabled_ = enabled; }
    // 设置自动平移内区比例。
    void setAutoPanInnerRatio(double ratio) { auto_pan_inner_ratio_ = ratio; }
    // 读取世界中心 x。
    double centerX() const { return world_center_x_; }
    // 读取世界中心 y。
    double centerY() const { return world_center_y_; }
    // 读取缺省显示半径。
    double defaultDist() const { return default_dist_; }

   protected:
    // 世界坐标包围盒极值。
    double min_x_{std::numeric_limits<double>::max()};
    double max_x_{std::numeric_limits<double>::lowest()};
    double min_y_{std::numeric_limits<double>::max()};
    double max_y_{std::numeric_limits<double>::lowest()};
    // 视图中心与米像素比。
    double world_center_x_{0.0};
    double world_center_y_{0.0};
    double meters_per_pixel_x_{0.08};
    double meters_per_pixel_y_{0.08};
    double default_dist_{6.66};
    double auto_pan_anchor_x_{0.0};
    double auto_pan_anchor_y_{0.0};
    double auto_pan_inner_ratio_{0.35};
    // 当前项目轨迹与车体需要保持几何比例，默认开启 axis equal。
    bool axis_equal_enabled_{true};
    bool auto_pan_enabled_{true};
    bool has_auto_pan_anchor_{false};
};

// 渲染指令调度中枢：保存阶段统一按 z_order 回放。
class LogicManifold {
   public:
    enum class ZOrder : int {
        BACKGROUND = 0,
        GRID = 1,
        TRAJECTORY = 2,
        EGO_VEHICLE = 3,
        TEXT_AND_LEGEND = 4,
    };
    struct DrawCommand {
        ZOrder z_order{ZOrder::TRAJECTORY};
        std::uint64_t sequence{0};
        std::function<void()> draw_func;
    };
    LogicManifold() = default;
    // 入队一条延迟绘制指令。
    void enqueue(ZOrder z_order, std::function<void()> draw_func) {
        render_queue_.emplace_back(
            DrawCommand{z_order, render_command_seq_++, std::move(draw_func)});
    }
    // 执行全部指令，保证同层绘制顺序稳定。
    void executeAll() {
        if (render_queue_.empty()) {
            return;
        }
        std::stable_sort(render_queue_.begin(), render_queue_.end(),
                         [](const DrawCommand& lhs, const DrawCommand& rhs) {
                             if (lhs.z_order != rhs.z_order) {
                                 return static_cast<int>(lhs.z_order) <
                                        static_cast<int>(rhs.z_order);
                             }
                             return lhs.sequence < rhs.sequence;
                         });
        is_replaying_ = true;
        for (const auto& command : render_queue_) {
            if (command.draw_func) {
                command.draw_func();
            }
        }
        is_replaying_ = false;
        clear();
    }
    // 清空队列并重置回放状态。
    void clear() {
        render_queue_.clear();
        render_command_seq_ = 0;
        is_replaying_ = false;
    }
    // 查询是否正在回放。
    bool isReplaying() const { return is_replaying_; }
    // 查询是否存在待绘制内容。
    bool isEmpty() const { return render_queue_.empty(); }

   protected:
    // 延迟渲染队列。
    std::vector<DrawCommand> render_queue_;
    // 入队序号用于同 z_order 下稳定排序。
    std::uint64_t render_command_seq_{0};
    // 回放状态用于避免重复入队。
    bool is_replaying_{false};
};

// 底层像素显像与 UI 排版引擎：管理 Mat、图例与文本。
class LumenSanctum {
   public:
    // 构造白底 BGR 画布。
    LumenSanctum(int width, int height)
        : canvas_(height, width, CV_8UC3, cv::Scalar(255, 255, 255)) {}
    // 清空画布与布局缓存。
    void clearCanvas() {
        canvas_.setTo(cv::Scalar(255, 255, 255));
        legend_entries_.clear();
        legend_label_set_.clear();
        occupied_text_bboxes_.clear();
    }
    // 获取可写画布引用。
    cv::Mat& mutableCanvas() { return canvas_; }
    // 获取只读画布引用。
    const cv::Mat& canvas() const { return canvas_; }
    // 画布宽度。
    int width() const { return canvas_.cols; }
    // 画布高度。
    int height() const { return canvas_.rows; }
    // 绘制线段。
    void drawLine(const cv::Point& p0, const cv::Point& p1,
                  const cv::Scalar& color, int thickness = 1) {
        cv::line(canvas_, p0, p1, color, thickness, cv::LINE_AA);
    }
    // 绘制矩形。
    void drawRectangle(const cv::Point& pt0, const cv::Point& pt1,
                       const cv::Scalar& color, int thickness = 1,
                       int line_type = cv::LINE_AA, int shift = 0) {
        cv::rectangle(canvas_, pt0, pt1, color, thickness, line_type, shift);
    }
    // 绘制圆点。
    void drawCircle(const cv::Point& center, int radius,
                    const cv::Scalar& color, int thickness = -1) {
        cv::circle(canvas_, center, radius, color, thickness, cv::LINE_AA);
    }
    // 绘制折线组。
    void drawPolylines(
        const std::vector<std::vector<cv::Point>>& polyline_group, bool closed,
        const cv::Scalar& color, int thickness) {
        cv::polylines(canvas_, polyline_group, closed, color, thickness,
                      cv::LINE_AA);
    }
    // 直接绘制文本。
    void putRawText(const std::string& text, cv::Point origin,
                    double font_scale, const cv::Scalar& color,
                    int thickness = 1) {
        cv::putText(canvas_, text, origin, cv::FONT_HERSHEY_SIMPLEX, font_scale,
                    color, thickness, cv::LINE_AA);
    }
    // 将 overlay 以 alpha 融合到当前画布。
    void blendOverlay(const cv::Mat& overlay, double alpha) {
        cv::addWeighted(overlay, alpha, canvas_, 1.0 - alpha, 0.0, canvas_);
    }
    // 写图：dpi_scale>1 时先高质量缩放再落盘。
    bool writeImage(const std::string& path, double dpi_scale) const {
        if (dpi_scale <= 1.0 + 1e-9) {
            return cv::imwrite(path, canvas_);
        }
        const int output_w = std::max(
            1, static_cast<int>(
                   std::lround(static_cast<double>(canvas_.cols) * dpi_scale)));
        const int output_h = std::max(
            1, static_cast<int>(
                   std::lround(static_cast<double>(canvas_.rows) * dpi_scale)));
        cv::Mat output_canvas;
        cv::resize(canvas_, output_canvas, cv::Size(output_w, output_h), 0.0,
                   0.0, cv::INTER_LANCZOS4);
        return cv::imwrite(path, output_canvas);
    }
    // 智能文本绘制：按占位盒规避明显重叠。
    bool putSmartText(const std::string& text, cv::Point origin,
                      double font_scale, const cv::Scalar& color,
                      bool check_overlap = true, int padding = 4) {
        if (IsBlankText(text)) {
            return true;
        }
        int baseline = 0;
        constexpr int text_thickness = 1;
        const cv::Size text_size =
            cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, font_scale,
                            text_thickness, &baseline);
        cv::Rect text_rect(origin.x - padding,
                           origin.y - text_size.height - padding,
                           text_size.width + 2 * padding,
                           text_size.height + baseline + 2 * padding);
        text_rect &= cv::Rect(0, 0, width(), height());
        if (text_rect.width <= 0 || text_rect.height <= 0) {
            return false;
        }
        if (check_overlap) {
            for (const auto& occupied_rect : occupied_text_bboxes_) {
                if ((text_rect & occupied_rect).area() > 0) {
                    return false;
                }
            }
        }
        cv::putText(canvas_, text, origin, cv::FONT_HERSHEY_SIMPLEX, font_scale,
                    color, text_thickness, cv::LINE_AA);
        occupied_text_bboxes_.push_back(text_rect);
        return true;
    }
    // 登记图例项。
    void addLegend(const std::string& label, const cv::Scalar& color) {
        if (IsBlankText(label) ||
            legend_label_set_.find(label) != legend_label_set_.end()) {
            return;
        }
        legend_label_set_.insert(label);
        legend_entries_.emplace_back(color, label);
    }
    // 绘制当前图例。
    void drawLegend() {
        if (legend_entries_.empty()) {
            return;
        }
        const int start_x = 20;
        int y = 30;
        for (const auto& [color, label_text] : legend_entries_) {
            drawLine(cv::Point(start_x, y), cv::Point(start_x + 28, y), color,
                     2);
            putSmartText(label_text, cv::Point(start_x + 36, y + 5), 0.55,
                         cv::Scalar(10, 10, 10), false);
            y += 24;
        }
    }
    // 判定文本是否为空或全空白。
    static bool IsBlankText(const std::string& text) {
        if (text.empty()) {
            return true;
        }
        return std::all_of(text.begin(), text.end(), [](unsigned char ch) {
            return std::isspace(ch) != 0;
        });
    }

   protected:
    // 主画布。
    cv::Mat canvas_;
    // 图例条目。
    std::vector<std::pair<cv::Scalar, std::string>> legend_entries_;
    // 图例文本去重集合。
    std::set<std::string> legend_label_set_;
    // 文本占位盒缓存。
    std::vector<cv::Rect> occupied_text_bboxes_;
};
}  // namespace apa_post_processor::visualizer

namespace apa_post_processor {
class Visualizer {
   public:
    using Style = visualizer::PlotStyle;
    using ZOrder = visualizer::LogicManifold::ZOrder;
    using DrawCommand = visualizer::LogicManifold::DrawCommand;
    // 输入秒级间隔，内部统一转换为毫秒，负值表示不限制。
    static int NormalizeMinIntervalMsFromSec(double draw_interval_sec) {
        if (!std::isfinite(draw_interval_sec) || draw_interval_sec < 0.0) {
            return -1;
        }
        const double raw_ms = draw_interval_sec * 1000.0;
        if (raw_ms <= 0.0) {
            return 0;
        }
        const double max_ms =
            static_cast<double>(std::numeric_limits<int>::max());
        if (raw_ms >= max_ms) {
            return std::numeric_limits<int>::max();
        }
        return static_cast<int>(std::llround(raw_ms));
    }
    Visualizer(const Visualizer&) = delete;
    Visualizer& operator=(const Visualizer&) = delete;
    Visualizer(Visualizer&&) = delete;
    Visualizer& operator=(Visualizer&&) = delete;
    // 构造可视化实例，默认输出白底、统一栅格与高清图。
    explicit Visualizer(const std::string& instance_name = "DefaultFig",
                        double min_draw_interval_sec = -1.0, double dpi = 2.0,
                        bool enable_grid = true, double default_dist = 6.6)
        : instance_name_(instance_name),
          title_(instance_name),
          save_timestamp_(GetCurrentWallTimeString()),
          min_draw_interval_ms_(
              NormalizeMinIntervalMsFromSec(min_draw_interval_sec)),
          output_dpi_scale_(NormalizeOutputDpiScale(dpi)),
          grid_enabled_(enable_grid),
          auspex_(),
          manifold_(),
          sanctum_(CANVAS_WIDTH_PX, CANVAS_HEIGHT_PX) {
        auspex_.setDefaultDist(default_dist);
    }
    ~Visualizer() = default;
    // 清空画布与本帧绘图状态。
    Visualizer& clear() {
        resetDrawState();
        return *this;
    }
    // 设置标题，空字符串会被忽略。
    Visualizer& setTitle(const std::string& title) {
        if (!title.empty()) {
            title_ = title;
        }
        return *this;
    }
    // 设置实例级限频参数（秒），内部按毫秒保存。
    Visualizer& setMinDrawIntervalSec(double min_draw_interval_sec) {
        min_draw_interval_ms_ =
            NormalizeMinIntervalMsFromSec(min_draw_interval_sec);
        return *this;
    }
    // 动态开关统一栅格。
    Visualizer& setGridEnabled(bool enabled) {
        grid_enabled_ = enabled;
        return *this;
    }
    // 设置坐标轴标签，空字符串会被忽略。
    Visualizer& setAxisLabels(const std::string& x_axis_label,
                              const std::string& y_axis_label) {
        if (!x_axis_label.empty()) {
            x_axis_label_ = x_axis_label;
        }
        if (!y_axis_label.empty()) {
            y_axis_label_ = y_axis_label;
        }
        return *this;
    }
    // 自动平移开关：开启后让锚点尽量留在视野内区。
    Visualizer& setAutoPanEnabled(bool enabled) {
        auspex_.setAutoPanEnabled(enabled);
        return *this;
    }
    // 自动平移内区比例：范围 (0, 0.5)。
    Visualizer& setAutoPanInnerRatio(double ratio) {
        auspex_.setAutoPanInnerRatio(NormalizeAutoPanInnerRatio(ratio));
        return *this;
    }
    // 坐标轴比例开关，默认开启 axis equal。
    Visualizer& setAxisEqualEnabled(bool enabled) {
        auspex_.setAxisEqualEnabled(enabled);
        return *this;
    }
    // 入队绘制指令，实际绘制在 save() 阶段统一完成。
    Visualizer& enqueueDrawCommand(ZOrder z_order,
                                   std::function<void()> draw_func) {
        manifold_.enqueue(z_order, std::move(draw_func));
        return *this;
    }
    // 默认绘制占用栅格，若传入lambda函数则按每个栅格到最近障碍物的距离着色
    Visualizer& plotGridMap(
        const GridMap& grid_map,
        std::function<double(double, double)> distToNearestObsFunc = {}) {
        const double resolution = grid_map.getResolution();
        const Position& origin = grid_map.getOrigin();
        const double max_x =
            origin.x + static_cast<double>(grid_map.getWidth()) * resolution;
        const double max_y =
            origin.y + static_cast<double>(grid_map.getHeight()) * resolution;
        if (!hasValidRange()) {
            // 地图通常远大于调试关注区域；仅在没有焦点时才用整图兜底建视场。
            updateRng(origin.x, origin.y);
            updateRng(max_x, max_y);
        }
        std::vector<cv::Point2d> centers;
        std::vector<cv::Scalar> colors;
        centers.reserve(grid_map.size());
        colors.reserve(grid_map.size());
        for (int row = 0; row < grid_map.getHeight(); ++row) {
            for (int col = 0; col < grid_map.getWidth(); ++col) {
                const auto center_pos = grid_map.getPosition(row, col);
                if (!distToNearestObsFunc) {
                    // 无距离函数时只收集障碍物栅格的中心；colors 保持为空，
                    // 绘制阶段检测到 colors 为空会回退为深色线框模式
                    if (!grid_map.isOccupied(row, col)) {
                        continue;
                    }
                    centers.emplace_back(center_pos.x, center_pos.y);
                } else {
                    // 有距离函数时为每个栅格收集中心和对应颜色
                    if (!inRng(center_pos.x, center_pos.y)) {
                        continue;
                    }
                    const double dist =
                        distToNearestObsFunc(center_pos.x, center_pos.y);
                    const auto [r, g, b] =
                        visualizer::AuspexMatrix::DistToRGB(dist, 0.0, 66.0);
                    centers.emplace_back(center_pos.x, center_pos.y);
                    // OpenCV 使用 BGR 顺序
                    colors.emplace_back(cv::Scalar(b, g, r));
                }
            }
        }
        enqueueDrawCommand(
            ZOrder::GRID,
            [this, centers = std::move(centers), colors = std::move(colors),
             resolution]() { drawGridCells(centers, colors, resolution); });
        return *this;
    }
    // 绘制 Path 轨迹折线。
    Visualizer& plotPath(const Path& path, bool is_focal = false,
                         Style style = {}) {
        std::vector<cv::Point2d> polyline;
        polyline.reserve(path.size());
        path.forEach([this, &polyline](const TrajectoryPoint& point) {
            polyline.emplace_back(point.x, point.y);
        });
        if (is_focal || !hasValidRange()) {
            for (const auto& point : polyline) {
                updateRng(point.x, point.y);
            }
            updateTransformFromRange();
        }
        EnsureStyles(style, {{"color", visualizer::Pen::BLUE},
                             {"linewidth", "2"},
                             {"label", "Path"}});
        drawPolyline(polyline, style, cv::Scalar(255, 174, 0), false, false,
                     ZOrder::TRAJECTORY);
        ++drawn_paths_count_;
        return *this;
    }
    // 兼容仅传样式的旧调用形式。
    Visualizer& plotPath(const Path& path, Style style) {
        return plotPath(path, false, std::move(style));
    }
    // 统一 Path 提取入口：支持 Path、Maneuver、Pose 容器、指针与智能指针。
    template <typename T>
    Visualizer& plotPath(const T& input, bool is_focal = false,
                         Style style = {}) {
        Path path;
        if (tryExtractPath(input, path)) {
            return plotPath(path, is_focal, std::move(style));
        }
        tryUnrollIterable(input, [&](const auto& item) {
            this->plotPath(item, is_focal, style);
        });
        return *this;
    }
    // 聚焦自车，用于确定显示范围与坐标变换。
    Visualizer& focusOnVehicle(const Pose& pose,
                               const VehicleParams& vehicle_params) {
        updateAutoPanAnchor(pose.x, pose.y);
        const auto footprint = buildVehicleFootprint(pose, vehicle_params);
        for (const auto& vertex : footprint) {
            updateRng(vertex.x, vertex.y);
        }
        updateRng(pose.x, pose.y);
        updateTransformFromRange();
        return *this;
    }
    // 绘制车辆轮廓，矩形表示车身，圆点表示后轴中心。
    Visualizer& plotVehicle(const Pose& pose,
                            const VehicleParams& vehicle_params,
                            bool is_focal = false, Style style = {}) {
        updateAutoPanAnchor(pose.x, pose.y);
        const auto footprint = buildVehicleFootprint(pose, vehicle_params);
        if (is_focal || !hasValidRange()) {
            for (const auto& vertex : footprint) {
                updateRng(vertex.x, vertex.y);
            }
            updateRng(pose.x, pose.y);
            updateTransformFromRange();
        }
        EnsureStyles(style, {{"color", visualizer::Pen::RED},
                             {"linewidth", "2"},
                             {"label", "Vehicle"}});
        drawPolyline(footprint, style, cv::Scalar(0, 0, 255), true, false,
                     ZOrder::EGO_VEHICLE);
        const auto color = styleColor(style, cv::Scalar(0, 0, 255));
        enqueueDrawCommand(ZOrder::EGO_VEHICLE, [this, pose, color]() {
            sanctum_.drawCircle(worldToPixel(pose.x, pose.y), 5, color, -1);
        });
        return *this;
    }
    // 兼容仅传样式的车辆绘制调用形式。
    Visualizer& plotVehicle(const Pose& pose,
                            const VehicleParams& vehicle_params, Style style) {
        return plotVehicle(pose, vehicle_params, false, std::move(style));
    }
    // 生成 Details 图层所需序列数据。
    template <typename TInput>
    Visualizer& plotPathDetails(const TInput& input, Style style = {}) {
        Path path;
        if (tryExtractPath(input, path)) {
            appendDetailSeriesFromPath(path, style);
            return *this;
        }
        tryUnrollIterable(input, [&](const auto& item) {
            this->plotPathDetails(item, style);
        });
        return *this;
    }
    // 绘制轨迹；draw_start_end 为 true 时在空间子图额外标注起终点位姿：
    // 橘黄色为起点（后轴中心圆点 + 车身轮廓），绿色为终点。
    Visualizer& plotTrajectory(
        const Trajectory& traj, const VehicleParams& vehicle_params,
        const VehicleFootprintModel* footprint_model = nullptr,
        const ESDFMap* esdf_map = nullptr, const GridMap* grid_map = nullptr,
        bool draw_swept_area = false, bool draw_start_end = false,
        Style style = {}) {
        if (style.find("color") == style.end()) {
            static int ti = 0;
            static constexpr std::array<const char*, 8> P = {
                "#00AEFF", "#FF0000", "#00A35A", "#FFC400",
                "#800080", "#FF6600", "#008080", "#CC0099"};
            style["color"] = P[ti++ % P.size()];
        }
        EnsureStyles(style, {{"linewidth", "2"}});
        trajectory_plots_.push_back({traj, vehicle_params, style,
                                     draw_swept_area, draw_start_end,
                                     footprint_model, esdf_map, grid_map});
        return *this;
    }
    // 保存图像，返回 OpenCV 写图结果。
    bool save(const std::string& dir_name = "fig") {
        if (!ShouldRenderForInstance(instance_name_, min_draw_interval_ms_)) {
            return false;
        }
        bool saved_any = false;
        const bool has_env_output = !manifold_.isEmpty();
        const bool has_detail_output = !detail_series_.empty();
        if (has_env_output) {
            sanctum_.clearCanvas();
            updateTransformFromRange();
            if (grid_enabled_) {
                drawUnifiedGrid();
            }
            executeRenderQueue();
            sanctum_.putSmartText(title_, cv::Point(18, CANVAS_HEIGHT_PX - 18),
                                  0.6, cv::Scalar(20, 20, 20), false, 2);
            sanctum_.drawLegend();
            drawGridCoordinateLabels();
            saved_any = saveCurrentCanvas(dir_name, title_, GetNextSaveNum()) ||
                        saved_any;
        }
        if (has_detail_output) {
            Visualizer details_visualizer(instance_name_, -1.0,
                                          output_dpi_scale_, false,
                                          auspex_.defaultDist());
            details_visualizer.save_timestamp_ = save_timestamp_;
            details_visualizer.title_ = instance_name_ + "Details";
            details_visualizer.detail_series_ = detail_series_;
            details_visualizer.renderDetailsCanvas();
            saved_any =
                details_visualizer.saveCurrentCanvas(
                    dir_name, details_visualizer.title_, GetNextSaveNum()) ||
                saved_any;
            if (!details_visualizer.last_save_path_.empty()) {
                last_save_path_ = details_visualizer.last_save_path_;
            }
        }
        if (!trajectory_plots_.empty()) {
            Visualizer tv(instance_name_, -1.0, output_dpi_scale_, false,
                          auspex_.defaultDist());
            tv.save_timestamp_ = save_timestamp_;
            tv.title_ = instance_name_ + "Trajectory";
            tv.trajectory_plots_ = std::move(trajectory_plots_);
            tv.renderTrajectoryComparisonCanvas();
            saved_any =
                tv.saveCurrentCanvas(dir_name, tv.title_, GetNextSaveNum()) ||
                saved_any;
            if (!tv.last_save_path_.empty())
                last_save_path_ = tv.last_save_path_;
        }
        return saved_any;
    }
    // 读取最近一次保存路径，便于测试或上层记录产物。
    const std::string& lastSavePath() const { return last_save_path_; }

   protected:
    static constexpr int CANVAS_WIDTH_PX = 3600;
    static constexpr int CANVAS_HEIGHT_PX = 2100;
    static constexpr int GRID_STEP_PX = 120;
    static constexpr double OUTPUT_DPI_SCALE_DEFAULT = 1.6;
    static constexpr double OUTPUT_DPI_SCALE_MAX = 4.0;
    static constexpr double METER_PER_PIXEL = 0.08;
    static constexpr double MIN_DYNAMIC_METER_PER_PIXEL = 0.002;
    static constexpr double MAX_DYNAMIC_METER_PER_PIXEL = 2.0;
    static constexpr double VIEW_PADDING_RATIO = 1.2;
    static double normalizeAnglePi(double theta) {
        double t = std::fmod(theta + PI, 2.0 * PI);
        if (t < 0.0) t += 2.0 * PI;
        return t - PI;
    }
    template <typename MapFunc>
    static std::vector<std::vector<cv::Point>> splitFinitePolylines(
        const std::vector<double>& xs, const std::vector<double>& ys,
        MapFunc&& map_to_pixel) {
        std::vector<std::vector<cv::Point>> groups;
        if (xs.size() != ys.size()) return groups;
        std::vector<cv::Point> cur;
        cur.reserve(xs.size());
        for (std::size_t i = 0; i < xs.size(); ++i) {
            if (!std::isfinite(xs[i]) || !std::isfinite(ys[i])) {
                if (cur.size() >= 2) groups.push_back(std::move(cur));
                cur.clear();
                continue;
            }
            cur.emplace_back(map_to_pixel(xs[i], ys[i]));
        }
        if (cur.size() >= 2) groups.push_back(std::move(cur));
        return groups;
    }
    template <typename W2P>
    void drawSweptAreaImpl(const std::vector<double>& xs,
                           const std::vector<double>& ys,
                           const std::vector<double>& headings,
                           const VehicleParams& vp, const Style& style,
                           W2P&& w2p) {
        if (xs.size() < 2) return;
        const double rx = -vp.rear_overhang;
        const double fx = vp.length - vp.rear_overhang;
        const double hw = vp.width * 0.5;
        const std::vector<cv::Point2d> lc = {
            {fx, hw}, {fx, -hw}, {rx, -hw}, {rx, hw}};

        // 最朴素实现：对每个有效路径点绘制车辆 footprint 轮廓线。
        std::vector<std::vector<cv::Point>> outlines;
        outlines.reserve(xs.size());
        for (std::size_t i = 0; i < xs.size(); ++i) {
            if (!std::isfinite(xs[i]) || !std::isfinite(ys[i]) ||
                !std::isfinite(headings[i])) {
                continue;
            }
            const double c = std::cos(headings[i]), s = std::sin(headings[i]);
            std::vector<cv::Point> q;
            q.reserve(4);
            bool ok = true;
            for (const auto& l : lc) {
                const double wx = xs[i] + l.x * c - l.y * s;
                const double wy = ys[i] + l.x * s + l.y * c;
                if (!std::isfinite(wx) || !std::isfinite(wy)) {
                    ok = false;
                    break;
                }
                q.emplace_back(w2p(wx, wy));
            }
            if (ok) outlines.push_back(std::move(q));
        }
        if (outlines.size() < 2) return;

        const cv::Scalar color = styleColor(style, cv::Scalar(180, 180, 180));
        const int thickness = std::max(1, styleLineWidth(style, 1));
        // 使用半透明叠加：重叠轮廓线自然形成扫过区域密度感，无填充、无
        // morphology、无布尔运算，完全可控。
        cv::Mat overlay = sanctum_.canvas().clone();
        for (const auto& q : outlines) {
            const std::vector<std::vector<cv::Point>> group = {q};
            cv::polylines(overlay, group, /*isClosed=*/true, color, thickness,
                          cv::LINE_8);
        }
        sanctum_.blendOverlay(overlay, styleAlpha(style, 0.35));
        addLegend(style, color);
    }
    void drawSweptAreaImpl(const std::vector<double>& xs,
                           const std::vector<double>& ys,
                           const std::vector<double>& headings,
                           const VehicleParams& vp, const Style& style) {
        drawSweptAreaImpl(
            xs, ys, headings, vp, style,
            [this](double x, double y) { return worldToPixel(x, y); });
    }
    static constexpr double AUTO_PAN_INNER_RATIO = 0.35;
    // Details 图序列缓存。
    struct DetailSeriesData {
        std::string label;
        cv::Scalar color{cv::Scalar(0, 0, 0)};
        int line_width{2};
        std::vector<double> index;
        std::vector<double> x;
        std::vector<double> y;
        std::vector<double> curvature;
        std::vector<double> heading;
        std::vector<double> ref_s;
        std::vector<double> lateral_d;
        std::vector<double> v;
        std::vector<double> a;
        std::vector<double> delta;
        std::vector<double> delta_dot;
        std::vector<double> delta_t;
        std::vector<double> time;
        std::vector<double> slack_true;
        std::vector<double> slack_calc;
        std::vector<std::size_t> shift_indices;
    };
    // 实例级门控状态。
    struct InstanceGateState {
        std::optional<std::chrono::steady_clock::time_point> last_render;
    };
    struct TrajectoryPlotEntry {
        Trajectory points;
        VehicleParams vehicle_params;
        Style style;
        bool draw_swept_area;
        bool draw_start_end;
        const VehicleFootprintModel* footprint_model;
        const ESDFMap* esdf_map;
        const GridMap* grid_map;
    };
    struct TrajRenderCtx {
        DetailSeriesData series;
        const TrajectoryPlotEntry* entry;
    };
    // 获取当前墙钟时间字符串，用于输出文件命名。
    static std::string GetCurrentWallTimeString() {
        const auto now = std::chrono::system_clock::to_time_t(
            std::chrono::system_clock::now());
        std::tm local_tm{};
#if defined(_WIN32)
        localtime_s(&local_tm, &now);
#else
        localtime_r(&now, &local_tm);
#endif
        std::stringstream ss;
        ss << std::put_time(&local_tm, "%Y%m%d%H%M%S");
        return ss.str();
    }
    // 全局递增保存序号，避免同秒多图文件名冲突。
    static std::uint64_t GetNextSaveNum() {
        static std::atomic<std::uint64_t> save_num_counter{1};
        return save_num_counter.fetch_add(1);
    }
    // 实例级门控锁。
    static std::mutex& GetInstanceGateMutex() {
        static std::mutex gate_mutex;
        return gate_mutex;
    }
    // 实例级门控状态表。
    static std::unordered_map<std::string, InstanceGateState>&
    GetInstanceGateStates() {
        static std::unordered_map<std::string, InstanceGateState> gate_states;
        return gate_states;
    }
    // 实例级限频判定。
    static bool ShouldRenderForInstance(const std::string& instance_name,
                                        int min_draw_interval_ms) {
        if (instance_name.empty() || min_draw_interval_ms < 0) {
            return true;
        }
        const auto now = std::chrono::steady_clock::now();
        std::lock_guard<std::mutex> lock(GetInstanceGateMutex());
        auto& state = GetInstanceGateStates()[instance_name];
        if (!state.last_render ||
            std::chrono::duration_cast<std::chrono::milliseconds>(
                now - *state.last_render)
                    .count() >= min_draw_interval_ms) {
            state.last_render = now;
            return true;
        }
        return false;
    }
    // 样式项缺失时写入默认值。
    static void EnsureStyle(Style& style, const std::string& key,
                            const std::string& value) {
        style.try_emplace(key, value);
    }
    // 批量写入默认样式，保持调用端和 VisualizerAVP 风格一致。
    static void EnsureStyles(
        Style& style,
        std::initializer_list<std::pair<const char*, const char*>> defaults) {
        for (const auto& [key, value] : defaults) {
            EnsureStyle(style, key, value);
        }
    }
    // 归一化输出清晰度缩放。
    static double NormalizeOutputDpiScale(double dpi_scale) {
        if (!std::isfinite(dpi_scale)) {
            return OUTPUT_DPI_SCALE_DEFAULT;
        }
        return std::clamp(dpi_scale, 1.0, OUTPUT_DPI_SCALE_MAX);
    }
    // 归一化自动平移内区比例，避免非法配置导致视图抖动。
    static double NormalizeAutoPanInnerRatio(double ratio) {
        if (!std::isfinite(ratio)) {
            return AUTO_PAN_INNER_RATIO;
        }
        return std::clamp(ratio, 0.05, 0.49);
    }
    // 字符串转小写。
    static std::string ToLower(std::string value) {
        std::transform(value.begin(), value.end(), value.begin(),
                       [](unsigned char ch) {
                           return static_cast<char>(std::tolower(ch));
                       });
        return value;
    }
    // 解析颜色字符串，支持 HEX 和常见颜色名。
    static cv::Scalar ParseColor(std::string_view color,
                                 const cv::Scalar& fallback) {
        if (color.size() == 7 && color[0] == '#') {
            int r = 0;
            int g = 0;
            int b = 0;
            if (std::from_chars(color.data() + 1, color.data() + 3, r, 16).ec ==
                    std::errc() &&
                std::from_chars(color.data() + 3, color.data() + 5, g, 16).ec ==
                    std::errc() &&
                std::from_chars(color.data() + 5, color.data() + 7, b, 16).ec ==
                    std::errc()) {
                return cv::Scalar(b, g, r);
            }
        }
        using ColorTriplet = std::array<double, 3>;
        static constexpr std::array<std::pair<std::string_view, ColorTriplet>,
                                    9>
            COLOR_MAP{{
                {"red", {0.0, 0.0, 255.0}},
                {"green", {0.0, 163.0, 90.0}},
                {"blue", {255.0, 174.0, 0.0}},
                {"yellow", {0.0, 196.0, 255.0}},
                {"purple", {128.0, 0.0, 128.0}},
                {"black", {0.0, 0.0, 0.0}},
                {"white", {255.0, 255.0, 255.0}},
                {"gray", {128.0, 128.0, 128.0}},
                {"grey", {128.0, 128.0, 128.0}},
            }};
        const std::string lowered = ToLower(std::string(color));
        const auto iter = std::find_if(
            COLOR_MAP.begin(), COLOR_MAP.end(),
            [&](const auto& entry) { return entry.first == lowered; });
        return iter != COLOR_MAP.end()
                   ? cv::Scalar(iter->second[0], iter->second[1],
                                iter->second[2])
                   : fallback;
    }
    // 解析正整数参数。
    static int ParsePositiveInt(const std::string& value, int fallback) {
        try {
            return std::max(1, std::stoi(value));
        } catch (...) {
            return fallback;
        }
    }
    // 解析透明度参数。
    static double ParseAlpha(const std::string& value, double fallback) {
        try {
            return std::clamp(std::stod(value), 0.0, 1.0);
        } catch (...) {
            return fallback;
        }
    }
    // 根据样式读取属性。
    template <typename TValue, typename TParser>
    TValue getStyleProp(const Style& style, const std::string& key,
                        const TValue& fallback, TParser parser) const {
        const auto iter = style.find(key);
        if (iter != style.end()) {
            return parser(iter->second, fallback);
        }
        return fallback;
    }
    // 从样式读取颜色。
    cv::Scalar styleColor(const Style& style,
                          const cv::Scalar& default_color) const {
        return getStyleProp(style, "color", default_color, ParseColor);
    }
    // 从样式读取线宽。
    int styleLineWidth(const Style& style, int default_width = 2) const {
        return getStyleProp(style, "linewidth", default_width,
                            ParsePositiveInt);
    }
    // 从样式读取透明度。
    double styleAlpha(const Style& style, double default_alpha = 1.0) const {
        return getStyleProp(style, "alpha", default_alpha, ParseAlpha);
    }
    // 图例去重并登记。
    void addLegend(const Style& style, const cv::Scalar& color) {
        const auto iter = style.find("label");
        if (iter == style.end()) {
            return;
        }
        sanctum_.addLegend(iter->second, color);
    }
    // 保存当前画布并记录路径。
    bool saveCurrentCanvas(const std::string& dir_name,
                           const std::string& title, std::uint64_t save_num) {
        std::filesystem::path dir(dir_name);
        if (!std::filesystem::exists(dir)) {
            std::filesystem::create_directories(dir);
        }
        std::stringstream ss;
        ss << dir_name << "/" << save_num << title << "_" << save_timestamp_
           << ".png";
        last_save_path_ = ss.str();
        return sanctum_.writeImage(last_save_path_, output_dpi_scale_);
    }
    // 执行延迟队列。
    void executeRenderQueue() { manifold_.executeAll(); }
    // 判断当前范围是否有效。
    bool hasValidRange() const { return auspex_.hasValidRange(); }
    // 判断点是否在当前视野外扩范围内。
    bool inRng(double x, double y) const { return auspex_.inRng(x, y); }
    // 更新世界坐标范围。
    void updateRng(double x, double y) { auspex_.updateRng(x, y); }
    // 更新自动平移锚点。
    void updateAutoPanAnchor(double x, double y) {
        auspex_.updateAutoPanAnchor(x, y);
    }
    // 根据已收集范围刷新坐标变换。
    void updateTransformFromRange() {
        auspex_.updateTransformFromRange(
            CANVAS_WIDTH_PX, CANVAS_HEIGHT_PX, VIEW_PADDING_RATIO,
            MIN_DYNAMIC_METER_PER_PIXEL, MAX_DYNAMIC_METER_PER_PIXEL);
    }
    // 世界坐标转像素坐标。
    cv::Point worldToPixel(double x, double y) const {
        return auspex_.worldToPixel(x, y, CANVAS_WIDTH_PX, CANVAS_HEIGHT_PX);
    }
    // 像素 x 转世界 x。
    double pixelToWorldX(int pixel_x) const {
        return auspex_.pixelToWorldX(pixel_x, CANVAS_WIDTH_PX);
    }
    // 像素 y 转世界 y。
    double pixelToWorldY(int pixel_y) const {
        return auspex_.pixelToWorldY(pixel_y, CANVAS_HEIGHT_PX);
    }
    // 重置本帧绘图状态。
    void resetDrawState() {
        drawn_paths_count_ = 0;
        auspex_.resetDrawState(METER_PER_PIXEL);
        sanctum_.clearCanvas();
        manifold_.clear();
        last_save_path_.clear();
        detail_series_.clear();
    }
    // 编译期递归容器解包器：统一 iterable 输入展开逻辑。
    template <typename TInput, typename TAction>
    bool tryUnrollIterable(const TInput& input, TAction action) {
        (void)action;
        using DecayedT = std::decay_t<TInput>;
        if constexpr (util::is_iterable_v<DecayedT> &&
                      !std::is_same_v<DecayedT, std::string> &&
                      !std::is_same_v<DecayedT, Path> &&
                      !std::is_same_v<DecayedT, Maneuver>) {
            for (const auto& item : input) {
                std::invoke(action, item);
            }
            return true;
        }
        return false;
    }
    // 统一 Path 提取入口：使用 C++17 编译期分派收敛重载。
    template <typename T>
    bool tryExtractPath(const T& input, Path& path) const {
        using DecayedT = std::decay_t<T>;
        if constexpr (visualizer::is_reference_wrapper_v<DecayedT>) {
            return tryExtractPath(input.get(), path);
        } else if constexpr (std::is_pointer_v<DecayedT> ||
                             visualizer::is_shared_ptr_v<DecayedT>) {
            return input != nullptr ? tryExtractPath(*input, path) : false;
        } else if constexpr (std::is_same_v<DecayedT, Path>) {
            if (input.empty()) {
                return false;
            }
            const auto& maneuvers = input.getManeuvers();
            auto& output_maneuvers = path.getManeuvers();
            output_maneuvers.reserve(maneuvers.size());
            for (const auto& maneuver : maneuvers) {
                if (maneuver.points.empty()) {
                    continue;
                }
                output_maneuvers.emplace_back(maneuver.points,
                                              maneuver.direction);
            }
            return !path.empty();
        } else if constexpr (std::is_same_v<DecayedT, Maneuver>) {
            if (input.points.empty()) {
                return false;
            }
            path.getManeuvers().emplace_back(input.points, input.direction);
            return true;
        } else if constexpr (std::is_same_v<DecayedT,
                                            std::vector<TrajectoryPoint>>) {
            if (input.empty()) {
                return false;
            }
            path.getManeuvers().emplace_back(input, Direction::UNKNOWN);
            return true;
        } else if constexpr (std::is_same_v<DecayedT, Trajectory>) {
            if (input.empty()) {
                return false;
            }
            path.getManeuvers().emplace_back(input.points(),
                                             Direction::UNKNOWN);
            return true;
        } else if constexpr (std::is_same_v<DecayedT, TrajectoryPoint>) {
            path.getManeuvers().emplace_back(input, Direction::UNKNOWN);
            return true;
        } else if constexpr (std::is_same_v<DecayedT, std::vector<Pose>>) {
            if (input.empty()) {
                return false;
            }
            std::vector<TrajectoryPoint> path_points;
            path_points.reserve(input.size());
            for (const auto& pose : input) {
                path_points.emplace_back(pose);
            }
            path.getManeuvers().emplace_back(std::move(path_points),
                                             Direction::UNKNOWN);
            return true;
        } else if constexpr (std::is_same_v<DecayedT, Pose>) {
            path.getManeuvers().emplace_back(TrajectoryPoint(input),
                                             Direction::UNKNOWN);
            return true;
        }
        return false;
    }
    // 从 Path 收集扁平点序列。
    std::vector<TrajectoryPoint> collectPathPoints(const Path& path) const {
        std::vector<TrajectoryPoint> points;
        points.reserve(path.size());
        path.forEach([&points](const TrajectoryPoint& point) {
            points.emplace_back(point);
        });
        return points;
    }
    // 从单条 Path 追加一组 Details 曲线数据，按 maneuver 拆分并标记换挡点。
    void appendDetailSeriesFromPath(const Path& path, const Style& style) {
        if (path.empty()) {
            return;
        }
        const auto& maneuvers = path.getManeuvers();
        std::size_t total_points = 0;
        for (const auto& maneuver : maneuvers) {
            total_points += maneuver.points.size();
        }
        DetailSeriesData series;
        if (auto label_iter = style.find("label");
            label_iter != style.end() && !label_iter->second.empty()) {
            series.label = label_iter->second;
        } else {
            series.label = "Path" + std::to_string(drawn_paths_count_);
        }
        series.color = styleColor(style, cv::Scalar(128, 128, 128));
        series.line_width = styleLineWidth(style, 2);
        reserveDetailSeries(series, total_points);
        double accum_s = 0.0;
        bool has_prev_point = false;
        Pose prev_pose;
        std::size_t flat_index = 0;
        for (std::size_t maneuver_index = 0; maneuver_index < maneuvers.size();
             ++maneuver_index) {
            const auto& points = maneuvers[maneuver_index].points;
            if (points.empty()) {
                continue;
            }
            // 当前 maneuver
            // 与前一段的衔接点即为换挡点，记录其在扁平序列中的索引。
            if (maneuver_index > 0 && !points.empty()) {
                series.shift_indices.push_back(flat_index);
            }
            for (std::size_t point_index = 0; point_index < points.size();
                 ++point_index) {
                const auto& point = points[point_index];
                if (has_prev_point) {
                    accum_s += std::hypot(point.x - prev_pose.x,
                                          point.y - prev_pose.y);
                }
                has_prev_point = true;
                prev_pose = point;
                appendDetailPoint(series, point,
                                  point.hasKappa() ? point.getKappa() : 0.0,
                                  accum_s, 0.0);
                ++flat_index;
            }
        }
        if (!series.index.empty()) {
            detail_series_.emplace_back(std::move(series));
            ++drawn_paths_count_;
        }
    }
    // 预留 Details 序列容量。
    static void reserveDetailSeries(DetailSeriesData& series,
                                    std::size_t size) {
        series.index.reserve(size);
        series.x.reserve(size);
        series.y.reserve(size);
        series.curvature.reserve(size);
        series.heading.reserve(size);
        series.ref_s.reserve(size);
        series.lateral_d.reserve(size);
        series.v.reserve(size);
        series.a.reserve(size);
        series.delta.reserve(size);
        series.delta_dot.reserve(size);
        series.delta_t.reserve(size);
        series.time.reserve(size);
        series.slack_true.reserve(size);
        series.slack_calc.reserve(size);
        series.shift_indices.reserve(size);
    }
    // 追加单个 Details 采样点。
    static void appendDetailPoint(DetailSeriesData& series,
                                  const TrajectoryPoint& point,
                                  double curvature, double ref_s,
                                  double lateral_d) {
        series.index.push_back(static_cast<double>(series.index.size()));
        series.x.push_back(point.x);
        series.y.push_back(point.y);
        series.curvature.push_back(curvature);
        series.heading.push_back(point.theta);
        series.ref_s.push_back(ref_s);
        series.lateral_d.push_back(lateral_d);
    }
    // 生成车辆矩形轮廓，坐标原点约定为后轴中心。
    std::vector<cv::Point2d> buildVehicleFootprint(
        const Pose& pose, const VehicleParams& vehicle_params) const {
        if (vehicle_params.length <= 0.0 || vehicle_params.width <= 0.0 ||
            vehicle_params.rear_overhang < 0.0 ||
            vehicle_params.rear_overhang > vehicle_params.length) {
            throw std::invalid_argument(
                "VehicleParams contains invalid dimensions for visualization");
        }
        const double rear_x = -vehicle_params.rear_overhang;
        const double front_x =
            vehicle_params.length - vehicle_params.rear_overhang;
        const double half_width = vehicle_params.width * 0.5;
        const double cos_theta = std::cos(pose.theta);
        const double sin_theta = std::sin(pose.theta);
        std::vector<cv::Point2d> local_vertices{{front_x, half_width},
                                                {front_x, -half_width},
                                                {rear_x, -half_width},
                                                {rear_x, half_width}};
        std::vector<cv::Point2d> world_vertices;
        world_vertices.reserve(local_vertices.size());
        for (const auto& vertex : local_vertices) {
            world_vertices.emplace_back(
                pose.x + vertex.x * cos_theta - vertex.y * sin_theta,
                pose.y + vertex.x * sin_theta + vertex.y * cos_theta);
        }
        return world_vertices;
    }
    // 统一绘制栅格。
    // colors 为空时：表示未传入距离函数，仅对 centers
    // 中的障碍物栅格绘制深色线框； colors 非空时：与 centers
    // 一一对应，按颜色填充每个栅格，形成距离场热力图。
    void drawGridCells(const std::vector<cv::Point2d>& centers,
                       const std::vector<cv::Scalar>& colors,
                       double resolution) {
        if (centers.empty()) {
            return;
        }
        const double half_size = resolution * 0.5;
        // 无颜色信息：绘制深色线框，避免密集障碍物糊成黑块
        if (colors.empty()) {
            constexpr int sub_pixel_shift = 4;
            constexpr double sub_pixel_scale =
                static_cast<double>(1 << sub_pixel_shift);
            const cv::Scalar cell_color(45, 45, 45);
            for (const auto& center : centers) {
                if (!inRng(center.x, center.y)) {
                    continue;
                }
                const cv::Point min_point =
                    worldToPixel(center.x - half_size, center.y + half_size);
                const cv::Point max_point =
                    worldToPixel(center.x + half_size, center.y - half_size);
                const cv::Point pt0(
                    static_cast<int>(std::lround(
                        static_cast<double>(min_point.x) * sub_pixel_scale)),
                    static_cast<int>(std::lround(
                        static_cast<double>(min_point.y) * sub_pixel_scale)));
                const cv::Point pt1(
                    static_cast<int>(std::lround(
                        static_cast<double>(max_point.x) * sub_pixel_scale)),
                    static_cast<int>(std::lround(
                        static_cast<double>(max_point.y) * sub_pixel_scale)));
                sanctum_.drawRectangle(pt0, pt1, cell_color, 1, cv::LINE_AA,
                                       sub_pixel_shift);
            }
            return;
        }
        // 有颜色信息：按距离场颜色填充每个栅格
        for (std::size_t i = 0; i < centers.size(); ++i) {
            const auto& center = centers[i];
            if (!inRng(center.x, center.y)) {
                continue;
            }
            const cv::Point min_point =
                worldToPixel(center.x - half_size, center.y + half_size);
            const cv::Point max_point =
                worldToPixel(center.x + half_size, center.y - half_size);
            sanctum_.drawRectangle(min_point, max_point, colors[i], -1,
                                   cv::LINE_8, 0);
        }
    }
    // 将世界坐标折线映射到像素坐标。
    std::vector<cv::Point> projectWorldPolylineToPixel(
        const std::vector<cv::Point2d>& world_polyline) const {
        std::vector<cv::Point> pixel_polyline;
        pixel_polyline.reserve(world_polyline.size());
        for (const auto& point : world_polyline) {
            pixel_polyline.emplace_back(worldToPixel(point.x, point.y));
        }
        return pixel_polyline;
    }
    // 立即绘制折线并按需绘制 marker。
    void drawPolylineImmediate(const std::vector<cv::Point>& polyline,
                               const Style& style,
                               const cv::Scalar& default_color, bool closed,
                               bool draw_marker) {
        if (polyline.size() < 2) {
            return;
        }
        const auto color = styleColor(style, default_color);
        const int thickness = styleLineWidth(style, 2);
        const double alpha = styleAlpha(style, 1.0);
        const std::vector<std::vector<cv::Point>> polyline_group{polyline};
        if (alpha >= 0.999) {
            sanctum_.drawPolylines(polyline_group, closed, color, thickness);
        } else {
            cv::Mat overlay = sanctum_.canvas().clone();
            cv::polylines(overlay, polyline_group, closed, color, thickness,
                          cv::LINE_AA);
            sanctum_.blendOverlay(overlay, alpha);
        }
        if (draw_marker) {
            for (const auto& point : polyline) {
                sanctum_.drawCircle(point, 3, color, -1);
            }
        }
        addLegend(style, color);
    }
    // 绘制折线：普通阶段入队，回放阶段立即绘制。
    void drawPolyline(const std::vector<cv::Point2d>& world_polyline,
                      const Style& style, const cv::Scalar& default_color,
                      bool closed, bool draw_marker, ZOrder z_order) {
        if (world_polyline.size() < 2) {
            return;
        }
        if (manifold_.isReplaying()) {
            drawPolylineImmediate(projectWorldPolylineToPixel(world_polyline),
                                  style, default_color, closed, draw_marker);
            return;
        }
        auto world_polyline_copy = world_polyline;
        auto style_copy = style;
        enqueueDrawCommand(
            z_order,
            [this, world_polyline_data = std::move(world_polyline_copy),
             style_data = std::move(style_copy), default_color, closed,
             draw_marker]() {
                drawPolylineImmediate(
                    projectWorldPolylineToPixel(world_polyline_data),
                    style_data, default_color, closed, draw_marker);
            });
    }
    // 根据可视距离计算坐标标注间隔。
    static double ComputeCoordinateInterval(double max_visible_distance,
                                            int target_ticks = 6) {
        if (!std::isfinite(max_visible_distance) ||
            max_visible_distance <= 1e-6) {
            return 1.0;
        }
        const double raw_interval =
            max_visible_distance /
            static_cast<double>(std::max(1, target_ticks));
        const double magnitude =
            std::pow(10.0, std::floor(std::log10(raw_interval)));
        const double fraction = raw_interval / magnitude;
        return magnitude *
               (fraction <= 1.5
                    ? 1.0
                    : (fraction <= 3.0 ? 2.0 : (fraction <= 7.0 ? 5.0 : 10.0)));
    }
    // 计算网格起始坐标。
    static double ComputeGridStartCoord(double min_coord, double interval) {
        return std::ceil(min_coord / interval) * interval;
    }
    // 根据间隔估算小数精度。
    static int ComputeCoordinatePrecision(double interval) {
        if (!std::isfinite(interval) || interval <= 1e-12 ||
            interval >= 1.0 - 1e-9) {
            return 0;
        }
        return static_cast<int>(std::round(-std::floor(std::log10(interval))));
    }
    // 格式化坐标标签。
    static std::string FormatNumber(double value, int precision) {
        if (std::abs(value) < 1e-9) {
            value = 0.0;
        }
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(precision) << value;
        return oss.str();
    }
    // 绘制 Details 子图，负责局部范围估计、网格、标签和曲线绘制。
    template <typename TXGetter, typename TYGetter>
    void drawDetailSubplot(const cv::Rect& rect, const std::string& title,
                           const std::string& x_label,
                           const std::string& y_label, TXGetter x_getter,
                           TYGetter y_getter, bool equal_axis = false) {
        const int left_pad = 72;
        const int right_pad = 18;
        const int top_pad = 54;
        const int bottom_pad = 44;
        const cv::Rect plot_rect(rect.x + left_pad, rect.y + top_pad,
                                 rect.width - left_pad - right_pad,
                                 rect.height - top_pad - bottom_pad);
        if (plot_rect.width <= 10 || plot_rect.height <= 10) {
            return;
        }
        sanctum_.drawRectangle(rect.tl(), rect.br(), cv::Scalar(225, 225, 225),
                               1);
        bool has_data = false;
        double min_x = std::numeric_limits<double>::max();
        double max_x = std::numeric_limits<double>::lowest();
        double min_y = std::numeric_limits<double>::max();
        double max_y = std::numeric_limits<double>::lowest();
        for (const auto& series : detail_series_) {
            const auto& xs = std::invoke(x_getter, series);
            const auto& ys = std::invoke(y_getter, series);
            if (xs.size() < 2 || xs.size() != ys.size()) {
                continue;
            }
            has_data = true;
            auto updateRangeFinite = [](double& min_val, double& max_val,
                                        const std::vector<double>& values) {
                for (double v : values) {
                    if (std::isfinite(v)) {
                        min_val = std::min(min_val, v);
                        max_val = std::max(max_val, v);
                    }
                }
            };
            updateRangeFinite(min_x, max_x, xs);
            updateRangeFinite(min_y, max_y, ys);
        }
        if (!has_data) {
            sanctum_.putRawText(title, cv::Point(rect.x + 8, rect.y + 18), 0.5,
                                cv::Scalar(40, 40, 40), 1);
            return;
        }
        normalizePlotRange(min_x, max_x, min_y, max_y, plot_rect, equal_axis);
        const double x_span = max_x - min_x;
        const double y_span = max_y - min_y;
        const auto map_to_pixel = [&](double x, double y) {
            const double nx = (x - min_x) / x_span;
            const double ny = (y - min_y) / y_span;
            return cv::Point(
                plot_rect.x +
                    static_cast<int>(std::lround(nx * plot_rect.width)),
                plot_rect.y + static_cast<int>(
                                  std::lround((1.0 - ny) * plot_rect.height)));
        };
        const bool is_xy_subplot = (x_getter == &DetailSeriesData::x) &&
                                   (y_getter == &DetailSeriesData::y);
        drawDetailGrid(plot_rect, min_x, max_x, min_y, max_y);
        for (const auto& series : detail_series_) {
            const auto& xs = std::invoke(x_getter, series);
            const auto& ys = std::invoke(y_getter, series);
            if (xs.size() < 2 || xs.size() != ys.size()) {
                continue;
            }
            std::vector<cv::Point> polyline;
            polyline.reserve(xs.size());
            for (std::size_t index = 0; index < xs.size(); ++index) {
                polyline.emplace_back(map_to_pixel(xs[index], ys[index]));
            }
            const std::vector<std::vector<cv::Point>> polyline_group{polyline};
            cv::polylines(sanctum_.mutableCanvas(), polyline_group, false,
                          series.color, series.line_width, cv::LINE_AA);
            // 多条路径叠加时换挡点不能喧宾夺主，只用同色圆点提示 maneuver
            // 边界。
            for (const auto shift_idx : series.shift_indices) {
                if (shift_idx >= xs.size()) {
                    continue;
                }
                sanctum_.drawCircle(map_to_pixel(xs[shift_idx], ys[shift_idx]),
                                    8, series.color, -1);
            }
        }
        sanctum_.putRawText(title, cv::Point(rect.x + 8, rect.y + 18), 0.5,
                            cv::Scalar(35, 35, 35), 1);
        sanctum_.putRawText(x_label,
                            cv::Point(plot_rect.x + plot_rect.width / 2 - 24,
                                      plot_rect.y + plot_rect.height + 34),
                            0.42, cv::Scalar(80, 80, 80), 1);
        sanctum_.putRawText(y_label, cv::Point(rect.x + 10, rect.y + 40), 0.42,
                            cv::Scalar(80, 80, 80), 1);
    }
    // 重绘 Details 画布，布局 2x2 子图。
    void renderDetailsCanvas() {
        sanctum_.clearCanvas();
        const int outer_margin = 16;
        const int gap = 12;
        const int half_w = (CANVAS_WIDTH_PX - 2 * outer_margin - gap) / 2;
        const int half_h = (CANVAS_HEIGHT_PX - 2 * outer_margin - gap) / 2;
        const cv::Rect rect_xy(outer_margin, outer_margin, half_w, half_h);
        const cv::Rect rect_heading(outer_margin + half_w + gap, outer_margin,
                                    half_w, half_h);
        const cv::Rect rect_s(outer_margin, outer_margin + half_h + gap, half_w,
                              half_h);
        const cv::Rect rect_kappa(outer_margin + half_w + gap,
                                  outer_margin + half_h + gap, half_w, half_h);
        drawDetailSubplot(rect_xy, "x-y Trajectory", "x(m)", "y(m)",
                          &DetailSeriesData::x, &DetailSeriesData::y, true);
        drawDetailSubplot(rect_heading, "Heading", "index", "heading(rad)",
                          &DetailSeriesData::index, &DetailSeriesData::heading);
        drawDetailSubplot(rect_s, "Arc Length", "index", "s(m)",
                          &DetailSeriesData::index, &DetailSeriesData::ref_s);
        drawDetailSubplot(rect_kappa, "Curvature", "index", "kappa(1/m)",
                          &DetailSeriesData::index,
                          &DetailSeriesData::curvature);
        int legend_y = 30;
        for (const auto& series : detail_series_) {
            if (visualizer::LumenSanctum::IsBlankText(series.label)) {
                continue;
            }
            sanctum_.drawLine(cv::Point(CANVAS_WIDTH_PX - 260, legend_y),
                              cv::Point(CANVAS_WIDTH_PX - 228, legend_y),
                              series.color, 2);
            sanctum_.putRawText(series.label,
                                cv::Point(CANVAS_WIDTH_PX - 220, legend_y + 5),
                                0.45, cv::Scalar(30, 30, 30), 1);
            legend_y += 22;
        }
    }
    // 归一化子图数据范围，必要时扩展为等比例显示。
    static void normalizePlotRange(double& min_x, double& max_x, double& min_y,
                                   double& max_y, const cv::Rect& plot_rect,
                                   bool equal_axis,
                                   double padding_ratio = 0.05) {
        if (std::abs(max_x - min_x) < 1e-9) {
            max_x += 1.0;
            min_x -= 1.0;
        }
        if (std::abs(max_y - min_y) < 1e-9) {
            max_y += 1.0;
            min_y -= 1.0;
        }
        if (equal_axis) {
            const double data_w = max_x - min_x;
            const double data_h = max_y - min_y;
            const double data_ratio = data_w / data_h;
            const double plot_ratio =
                static_cast<double>(plot_rect.width) / plot_rect.height;
            if (data_ratio > plot_ratio) {
                const double extra = (data_w / plot_ratio - data_h) * 0.5;
                min_y -= extra;
                max_y += extra;
            } else {
                const double extra = (data_h * plot_ratio - data_w) * 0.5;
                min_x -= extra;
                max_x += extra;
            }
        }
        const double pad_x = std::max(1e-6, (max_x - min_x) * padding_ratio);
        const double pad_y = std::max(1e-6, (max_y - min_y) * padding_ratio);
        min_x -= pad_x;
        max_x += pad_x;
        min_y -= pad_y;
        max_y += pad_y;
    }
    // 绘制 Details 子图网格与坐标标签。
    void drawDetailGrid(const cv::Rect& plot_rect, double min_x, double max_x,
                        double min_y, double max_y) {
        const cv::Scalar grid_color(238, 238, 238);
        const cv::Scalar axis_color(205, 205, 205);
        const cv::Scalar text_color(120, 120, 120);
        const double x_span = max_x - min_x;
        const double y_span = max_y - min_y;
        const double x_interval = ComputeCoordinateInterval(x_span, 5);
        const double y_interval = ComputeCoordinateInterval(y_span, 5);
        const int x_precision = ComputeCoordinatePrecision(x_interval);
        const int y_precision = ComputeCoordinatePrecision(y_interval);
        for (double value = ComputeGridStartCoord(min_x, x_interval);
             value <= max_x + x_interval * 0.5; value += x_interval) {
            const double nx = (value - min_x) / x_span;
            const int x = plot_rect.x +
                          static_cast<int>(std::lround(nx * plot_rect.width));
            sanctum_.drawLine(cv::Point(x, plot_rect.y),
                              cv::Point(x, plot_rect.y + plot_rect.height),
                              grid_color, 1);
            sanctum_.putRawText(
                FormatNumber(value, x_precision),
                cv::Point(x - 18, plot_rect.y + plot_rect.height + 16), 0.35,
                text_color, 1);
        }
        for (double value = ComputeGridStartCoord(min_y, y_interval);
             value <= max_y + y_interval * 0.5; value += y_interval) {
            const double ny = (value - min_y) / y_span;
            const int y =
                plot_rect.y +
                static_cast<int>(std::lround((1.0 - ny) * plot_rect.height));
            sanctum_.drawLine(cv::Point(plot_rect.x, y),
                              cv::Point(plot_rect.x + plot_rect.width, y),
                              grid_color, 1);
            sanctum_.putRawText(FormatNumber(value, y_precision),
                                cv::Point(plot_rect.x - 56, y + 4), 0.35,
                                text_color, 1);
        }
        sanctum_.drawRectangle(plot_rect.tl(), plot_rect.br(), axis_color, 1);
    }
    // 绘制统一世界坐标栅格。
    void drawUnifiedGrid() {
        const cv::Scalar grid_color(238, 238, 238);
        const cv::Scalar axis_color(205, 205, 205);
        if (hasValidRange()) {
            const double min_world_x = pixelToWorldX(0);
            const double max_world_x = pixelToWorldX(CANVAS_WIDTH_PX - 1);
            const double min_world_y = pixelToWorldY(CANVAS_HEIGHT_PX - 1);
            const double max_world_y = pixelToWorldY(0);
            const double x_interval = ComputeCoordinateInterval(
                std::abs(max_world_x - min_world_x), 8);
            const double y_interval = ComputeCoordinateInterval(
                std::abs(max_world_y - min_world_y), 10);
            for (double world_x =
                     ComputeGridStartCoord(min_world_x, x_interval);
                 world_x <= max_world_x + x_interval * 0.5;
                 world_x += x_interval) {
                const int pixel_x = worldToPixel(world_x, auspex_.centerY()).x;
                sanctum_.drawLine(cv::Point(pixel_x, 0),
                                  cv::Point(pixel_x, CANVAS_HEIGHT_PX - 1),
                                  grid_color, 1);
            }
            for (double world_y =
                     ComputeGridStartCoord(min_world_y, y_interval);
                 world_y <= max_world_y + y_interval * 0.5;
                 world_y += y_interval) {
                const int pixel_y = worldToPixel(auspex_.centerX(), world_y).y;
                sanctum_.drawLine(cv::Point(0, pixel_y),
                                  cv::Point(CANVAS_WIDTH_PX - 1, pixel_y),
                                  grid_color, 1);
            }
            if (min_world_x <= 0.0 && 0.0 <= max_world_x) {
                const int axis_x = worldToPixel(0.0, auspex_.centerY()).x;
                sanctum_.drawLine(cv::Point(axis_x, 0),
                                  cv::Point(axis_x, CANVAS_HEIGHT_PX - 1),
                                  axis_color, 1);
            }
            if (min_world_y <= 0.0 && 0.0 <= max_world_y) {
                const int axis_y = worldToPixel(auspex_.centerX(), 0.0).y;
                sanctum_.drawLine(cv::Point(0, axis_y),
                                  cv::Point(CANVAS_WIDTH_PX - 1, axis_y),
                                  axis_color, 1);
            }
            return;
        }
        for (int x = 0; x < CANVAS_WIDTH_PX; x += GRID_STEP_PX) {
            sanctum_.drawLine(cv::Point(x, 0),
                              cv::Point(x, CANVAS_HEIGHT_PX - 1), grid_color,
                              1);
        }
        for (int y = 0; y < CANVAS_HEIGHT_PX; y += GRID_STEP_PX) {
            sanctum_.drawLine(cv::Point(0, y),
                              cv::Point(CANVAS_WIDTH_PX - 1, y), grid_color, 1);
        }
    }
    // 在统一栅格上标注世界坐标。
    void drawGridCoordinateLabels() {
        if (!hasValidRange()) {
            return;
        }
        const cv::Scalar text_color(130, 130, 130);
        const double min_world_x = pixelToWorldX(0);
        const double max_world_x = pixelToWorldX(CANVAS_WIDTH_PX - 1);
        const double min_world_y = pixelToWorldY(CANVAS_HEIGHT_PX - 1);
        const double max_world_y = pixelToWorldY(0);
        const double x_interval =
            ComputeCoordinateInterval(std::abs(max_world_x - min_world_x), 8);
        const double y_interval =
            ComputeCoordinateInterval(std::abs(max_world_y - min_world_y), 10);
        const int x_precision = ComputeCoordinatePrecision(x_interval);
        const int y_precision = ComputeCoordinatePrecision(y_interval);
        if (!x_axis_label_.empty()) {
            sanctum_.putSmartText(
                x_axis_label_,
                cv::Point(CANVAS_WIDTH_PX - 120, CANVAS_HEIGHT_PX - 8), 0.5,
                text_color, false, 2);
        }
        if (!y_axis_label_.empty()) {
            sanctum_.putSmartText(y_axis_label_, cv::Point(6, 18), 0.5,
                                  text_color, false, 2);
        }
        for (double world_x = ComputeGridStartCoord(min_world_x, x_interval);
             world_x <= max_world_x + x_interval * 0.5; world_x += x_interval) {
            const int pixel_x = worldToPixel(world_x, auspex_.centerY()).x;
            if (pixel_x < 20 || pixel_x > CANVAS_WIDTH_PX - 80) {
                continue;
            }
            sanctum_.putSmartText(
                FormatNumber(world_x, x_precision),
                cv::Point(pixel_x - 18, CANVAS_HEIGHT_PX - 34), 0.45,
                text_color, true, 2);
        }
        for (double world_y = ComputeGridStartCoord(min_world_y, y_interval);
             world_y <= max_world_y + y_interval * 0.5; world_y += y_interval) {
            const int pixel_y = worldToPixel(auspex_.centerX(), world_y).y;
            if (pixel_y < 24 || pixel_y > CANVAS_HEIGHT_PX - 45) {
                continue;
            }
            sanctum_.putSmartText(FormatNumber(world_y, y_precision),
                                  cv::Point(8, pixel_y - 4), 0.45, text_color,
                                  true, 2);
        }
    }
    // 实例名：用于默认标题。
    const std::string instance_name_;
    // 当前标题与保存时间戳。
    std::string title_;
    std::string save_timestamp_;
    // 坐标轴标签。
    std::string x_axis_label_{"X(m)"};
    std::string y_axis_label_{"Y(m)"};
    // 最近一次保存路径，方便调用方记录产物。
    std::string last_save_path_;
    // 实例级最小绘图间隔与已绘路径计数。
    int min_draw_interval_ms_{-1};
    int drawn_paths_count_{0};
    // 输出清晰度缩放。
    double output_dpi_scale_{OUTPUT_DPI_SCALE_DEFAULT};
    // 是否绘制统一栅格。
    bool grid_enabled_{true};
    std::vector<TrajectoryPlotEntry> trajectory_plots_;
    std::vector<DetailSeriesData> detail_series_;
    // 视场变换管理器。
    visualizer::AuspexMatrix auspex_;
    // 延迟绘制队列。
    visualizer::LogicManifold manifold_;
    // 画布与布局管理器。
    visualizer::LumenSanctum sanctum_;

    // === 轨迹比较视图 ===
    // 从 Trajectory 构建 DetailSeriesData。
    static DetailSeriesData buildDetailSeriesFromTrajPoints(
        const Trajectory& traj, const std::string& label,
        const cv::Scalar& color, int line_width, const VehicleParams& vp) {
        DetailSeriesData series;
        series.label = label;
        series.color = color;
        series.line_width = line_width;
        if (traj.empty()) {
            return series;
        }
        const std::size_t n = traj.size();
        reserveDetailSeries(series, n);
        const double wheelbase = vp.wheelbase;
        double arc = 0.0;
        for (std::size_t i = 0; i < n; ++i) {
            const auto& pp = traj[i];
            series.index.push_back(static_cast<double>(i));
            series.x.push_back(pp.x);
            series.y.push_back(pp.y);
            series.heading.push_back(normalizeAnglePi(pp.theta));
            series.ref_s.push_back(arc);
            series.time.push_back(pp.hasT() ? pp.getT() : NAN);
            series.v.push_back(pp.hasV() ? pp.getV() : NAN);
            series.a.push_back(pp.hasA() ? pp.getA() : NAN);
            series.delta.push_back(pp.hasDelta() ? pp.getDelta() : NAN);
            series.delta_dot.push_back(pp.hasDeltaDot() ? pp.getDeltaDot()
                                                        : NAN);
            if (pp.hasKappa()) {
                series.curvature.push_back(pp.getKappa());
            } else if (pp.hasDelta() && std::abs(wheelbase) > EPSILON) {
                series.curvature.push_back(std::tan(pp.getDelta()) / wheelbase);
            } else {
                series.curvature.push_back(NAN);
            }
            if (i + 1 < n) {
                arc += std::hypot(traj[i + 1].x - pp.x, traj[i + 1].y - pp.y);
            }
        }
        return series;
    }
    // 轨迹比较视图主入口：左栏空间几何 + 右栏 t-domain 信号堆叠。
    void renderTrajectoryComparisonCanvas() {
        sanctum_.clearCanvas();
        if (trajectory_plots_.empty()) {
            return;
        }
        const int margin = 16;
        const int gap = 12;
        const int cell_w = (CANVAS_WIDTH_PX - 2 * margin - gap) / 2;
        const int cell_h = CANVAS_HEIGHT_PX - 2 * margin;
        const cv::Rect rect_left(margin, margin, cell_w, cell_h);
        const cv::Rect rect_right(margin + cell_w + gap, margin, cell_w,
                                  cell_h);
        // 为每条轨迹构建渲染上下文。
        std::vector<TrajRenderCtx> ctxs;
        ctxs.reserve(trajectory_plots_.size());
        for (const auto& entry : trajectory_plots_) {
            if (entry.points.empty()) {
                continue;
            }
            const auto clr = styleColor(entry.style, cv::Scalar(0, 0, 0));
            const std::string lb =
                entry.style.count("label") ? entry.style.at("label") : "Traj";
            ctxs.push_back(
                {buildDetailSeriesFromTrajPoints(entry.points, lb, clr,
                                                 styleLineWidth(entry.style, 2),
                                                 entry.vehicle_params),
                 &entry});
        }
        if (ctxs.empty()) {
            return;
        }
        drawTrajectorySpatialSubplot(rect_left, ctxs);
        // 右栏：信号堆叠，按信号类型分组，同类型不同轨迹同框对比。
        // 第一步：为每条轨迹构造各信号 band。
        std::vector<std::vector<DetailSeriesData>> traj_bands;
        traj_bands.reserve(ctxs.size());
        constexpr double kRadToDeg = 180.0 / PI;
        for (const auto& ctx : ctxs) {
            const auto& d = ctx.series;
            // 从速度符号翻转检测 cusp 索引。
            std::vector<std::size_t> cusps;
            for (std::size_t i = 1; i < d.v.size(); ++i) {
                if (std::isfinite(d.v[i - 1]) && std::isfinite(d.v[i]) &&
                    (d.v[i - 1] > 0.0) != (d.v[i] > 0.0)) {
                    cusps.push_back(i);
                }
            }
            // 构造带 cusp 断点的 band。
            auto make_band = [&](const std::string& suffix,
                                 const std::vector<double>& y_vals) {
                DetailSeriesData band;
                band.label = suffix;
                band.color = d.color;
                band.line_width = d.line_width;
                const std::size_t n_raw = y_vals.size();
                const std::size_t n_cusps = cusps.size();
                band.index.reserve(n_raw + n_cusps);
                band.v.reserve(n_raw + n_cusps);
                band.shift_indices.reserve(n_cusps);
                std::size_t cusp_idx = 0;
                for (std::size_t i = 0; i < n_raw; ++i) {
                    while (cusp_idx < n_cusps && cusps[cusp_idx] == i) {
                        band.index.push_back((d.index[i - 1] + d.index[i]) *
                                             0.5);
                        band.v.push_back(NAN);
                        ++cusp_idx;
                    }
                    if (cusp_idx > 0 && i == cusps[cusp_idx - 1]) {
                        band.shift_indices.push_back(band.index.size());
                    }
                    band.index.push_back(d.index[i]);
                    band.v.push_back(y_vals[i]);
                }
                return band;
            };
            std::vector<double> heading_deg = d.heading;
            for (auto& h : heading_deg) {
                h *= kRadToDeg;
            }
            std::vector<DetailSeriesData> bands;
            bands.reserve(7);
            bands.push_back(make_band("kappa(1/m)", d.curvature));
            bands.push_back(make_band("heading(deg)", heading_deg));
            bands.push_back(make_band("v(m/s)", d.v));
            bands.push_back(make_band("a(m/s2)", d.a));
            bands.push_back(make_band("delta(rad)", d.delta));
            bands.push_back(make_band("delta_dot(rad/s)", d.delta_dot));
            // 安全余量：decomposed circle 方法。
            if (ctx.entry->footprint_model && ctx.entry->esdf_map) {
                const double outer_r =
                    ctx.entry->footprint_model->getOuterRadius();
                const auto circle_count =
                    ctx.entry->footprint_model->getCircleNum(CircleType::OUTER);
                std::vector<Eigen::Vector2d> centers(circle_count);
                std::vector<Eigen::Matrix<double, 2, 3>> jacobians(
                    circle_count);
                std::vector<double> margins;
                margins.reserve(d.x.size());
                for (std::size_t i = 0; i < d.x.size(); ++i) {
                    ctx.entry->footprint_model->calInterpolatedCenters(
                        d.x[i], d.y[i], d.heading[i], CircleType::OUTER,
                        centers, jacobians);
                    double min_margin = INFINITY;
                    for (const auto& ct : centers) {
                        min_margin = std::min(
                            min_margin,
                            ctx.entry->esdf_map->getDist(ct.x(), ct.y()) -
                                outer_r);
                    }
                    if (!std::isfinite(min_margin)) {
                        min_margin = 0.0;
                    }
                    margins.push_back(min_margin);
                }
                bands.push_back(make_band("min_margin(m)", margins));
            }
            traj_bands.push_back(std::move(bands));
        }
        // 第二步：按信号类型 pivot，同类型多条轨迹合并到同一 band。
        std::vector<std::vector<DetailSeriesData>> grouped_bands;
        std::vector<std::string> signal_order;
        {
            std::map<std::string, std::vector<DetailSeriesData>> signal_groups;
            for (auto& bands : traj_bands) {
                for (auto& band : bands) {
                    const std::string& name = band.label;
                    if (signal_groups.find(name) == signal_groups.end()) {
                        signal_order.push_back(name);
                    }
                    signal_groups[name].push_back(std::move(band));
                }
            }
            grouped_bands.reserve(signal_order.size());
            for (const auto& name : signal_order) {
                grouped_bands.push_back(std::move(signal_groups[name]));
            }
        }
        const std::size_t n_bands = grouped_bands.size();
        using MemPtr = std::vector<double> DetailSeriesData::*;
        std::vector<MemPtr> y_getters(n_bands, &DetailSeriesData::v);
        std::vector<std::string> y_labels = signal_order;
        // 仅为 min_margin band 添加 y=0 红色虚线参考线。
        std::vector<std::vector<std::pair<double, cv::Scalar>>> band_refs(
            n_bands);
        for (std::size_t i = 0; i < n_bands; ++i) {
            if (y_labels[i].find("min_margin") != std::string::npos) {
                band_refs[i].push_back({0.0, cv::Scalar(0, 0, 255)});
                break;
            }
        }
        const int y_ticks = std::max(2, 5 - static_cast<int>(ctxs.size()));
        drawDetailSubplotStacked(grouped_bands, y_getters, y_labels, rect_right,
                                 "Signal by Index", "index",
                                 &DetailSeriesData::index, false, 0.0, 0.0, {},
                                 y_ticks, band_refs);
    }
    // 查找序列中首/末 x、y、heading 均为有限值的索引；若无有效点，
    // 首索引保持为序列长度，调用方据此跳过。
    static std::pair<std::size_t, std::size_t> FindStartEndIndices(
        const DetailSeriesData& series) {
        const std::size_t n = series.x.size();
        std::size_t first = n;
        std::size_t last = n;
        for (std::size_t i = 0; i < n; ++i) {
            if (!std::isfinite(series.x[i]) || !std::isfinite(series.y[i]) ||
                !std::isfinite(series.heading[i])) {
                continue;
            }
            if (first == n) {
                first = i;
            }
            last = i;
        }
        return {first, last};
    }
    // 轨迹比较空间子图：障碍物背景 + 所有轨迹线 + swept area。
    void drawTrajectorySpatialSubplot(const cv::Rect& rect,
                                      const std::vector<TrajRenderCtx>& ctxs) {
        const int left_pad = 72;
        const int right_pad = 18;
        const int top_pad = 54;
        const int bottom_pad = 44;
        const cv::Rect plot_rect(rect.x + left_pad, rect.y + top_pad,
                                 rect.width - left_pad - right_pad,
                                 rect.height - top_pad - bottom_pad);
        // 白底 + 外框。
        sanctum_.drawRectangle(rect.tl(), rect.br(), cv::Scalar(255, 255, 255),
                               -1);
        sanctum_.drawRectangle(rect.tl(), rect.br(), cv::Scalar(225, 225, 225),
                               1);
        if (plot_rect.width <= 10 || plot_rect.height <= 10) {
            return;
        }
        // 计算所有轨迹与 swept area 的数据范围。
        double min_x = INFINITY;
        double max_x = -INFINITY;
        double min_y = INFINITY;
        double max_y = -INFINITY;
        for (const auto& ctx : ctxs) {
            for (std::size_t i = 0; i < ctx.series.x.size(); ++i) {
                min_x = std::min(min_x, ctx.series.x[i]);
                max_x = std::max(max_x, ctx.series.x[i]);
                min_y = std::min(min_y, ctx.series.y[i]);
                max_y = std::max(max_y, ctx.series.y[i]);
            }
            if (ctx.entry->draw_swept_area) {
                const double half_diag =
                    std::hypot(ctx.entry->vehicle_params.length,
                               ctx.entry->vehicle_params.width) *
                    0.6;
                for (std::size_t i = 0; i < ctx.series.x.size(); ++i) {
                    min_x = std::min(min_x, ctx.series.x[i] - half_diag);
                    max_x = std::max(max_x, ctx.series.x[i] + half_diag);
                    min_y = std::min(min_y, ctx.series.y[i] - half_diag);
                    max_y = std::max(max_y, ctx.series.y[i] + half_diag);
                }
            }
            if (ctx.entry->draw_start_end) {
                // 起终点车身轮廓需纳入视场范围，避免轮廓被图框裁剪。
                const auto [start_idx, end_idx] =
                    FindStartEndIndices(ctx.series);
                if (start_idx < ctx.series.x.size()) {
                    for (const std::size_t idx : {start_idx, end_idx}) {
                        const auto corners = buildVehicleFootprint(
                            Pose{ctx.series.x[idx], ctx.series.y[idx],
                                 ctx.series.heading[idx]},
                            ctx.entry->vehicle_params);
                        for (const auto& corner : corners) {
                            min_x = std::min(min_x, corner.x);
                            max_x = std::max(max_x, corner.x);
                            min_y = std::min(min_y, corner.y);
                            max_y = std::max(max_y, corner.y);
                        }
                    }
                }
            }
        }
        if (min_x > max_x) {
            return;
        }
        normalizePlotRange(min_x, max_x, min_y, max_y, plot_rect, true);
        const double x_span = max_x - min_x;
        const double y_span = max_y - min_y;
        const auto world_to_pixel = [&](double x, double y) {
            return cv::Point(
                plot_rect.x + static_cast<int>(std::lround(
                                  (x - min_x) / x_span * plot_rect.width)),
                plot_rect.y +
                    static_cast<int>(std::lround((1.0 - (y - min_y) / y_span) *
                                                 plot_rect.height)));
        };
        // 背景栅格地图。
        const GridMap* active_grid_map = nullptr;
        for (const auto& ctx : ctxs) {
            if (ctx.entry->grid_map) {
                active_grid_map = ctx.entry->grid_map;
                break;
            }
        }
        if (active_grid_map) {
            const double resolution = active_grid_map->getResolution();
            const double half_res = resolution * 0.5;
            for (int row = 0; row < active_grid_map->getHeight(); ++row) {
                for (int col = 0; col < active_grid_map->getWidth(); ++col) {
                    if (!active_grid_map->isOccupied(row, col)) {
                        continue;
                    }
                    const auto pos = active_grid_map->getPosition(row, col);
                    if (pos.x < min_x || pos.x > max_x || pos.y < min_y ||
                        pos.y > max_y) {
                        continue;
                    }
                    sanctum_.drawRectangle(
                        world_to_pixel(pos.x - half_res, pos.y + half_res),
                        world_to_pixel(pos.x + half_res, pos.y - half_res),
                        cv::Scalar(45, 45, 45), 1);
                }
            }
        }
        drawDetailGrid(plot_rect, min_x, max_x, min_y, max_y);
        // swept area：用每个路径点的车辆 footprint 轮廓线表达扫过区域。
        for (const auto& ctx : ctxs) {
            if (ctx.entry->draw_swept_area) {
                drawSweptAreaImpl(ctx.series.x, ctx.series.y,
                                  ctx.series.heading, ctx.entry->vehicle_params,
                                  {{"color", "#C8C8C8"}, {"alpha", "0.35"}},
                                  world_to_pixel);
            }
        }
        // 轨迹线。
        for (const auto& ctx : ctxs) {
            const auto groups = splitFinitePolylines(ctx.series.x, ctx.series.y,
                                                     world_to_pixel);
            if (!groups.empty()) {
                drawPolylinesClipped(sanctum_, groups, plot_rect,
                                     ctx.series.color, ctx.series.line_width);
            }
        }
        // 起终点位姿标记：橘黄色起点 + 绿色终点（后轴中心圆点 + 车身轮廓），
        // 绘制在轨迹线之上以便辨识。
        const cv::Scalar start_color =
            ParseColor(visualizer::Pen::ORANGE, cv::Scalar(0, 102, 255));
        const cv::Scalar end_color =
            ParseColor(visualizer::Pen::GREEN, cv::Scalar(90, 163, 0));
        bool start_end_drawn = false;
        for (const auto& ctx : ctxs) {
            if (!ctx.entry->draw_start_end) {
                continue;
            }
            const auto [start_idx, end_idx] = FindStartEndIndices(ctx.series);
            if (start_idx >= ctx.series.x.size()) {
                continue;
            }
            const auto draw_marker = [&](std::size_t idx,
                                         const cv::Scalar& color) {
                const auto corners = buildVehicleFootprint(
                    Pose{ctx.series.x[idx], ctx.series.y[idx],
                         ctx.series.heading[idx]},
                    ctx.entry->vehicle_params);
                std::vector<cv::Point> outline;
                outline.reserve(corners.size() + 1);
                for (const auto& corner : corners) {
                    outline.emplace_back(world_to_pixel(corner.x, corner.y));
                }
                // 闭合轮廓：首尾相接形成完整车身矩形。
                outline.emplace_back(outline.front());
                drawPolylinesClipped(sanctum_, {outline}, plot_rect, color, 2);
                sanctum_.drawCircle(
                    world_to_pixel(ctx.series.x[idx], ctx.series.y[idx]), 5,
                    color, -1);
            };
            draw_marker(start_idx, start_color);
            // 单点轨迹起点与终点重合，只画起点，避免绿色覆盖橘黄色。
            if (end_idx != start_idx) {
                draw_marker(end_idx, end_color);
            }
            start_end_drawn = true;
        }
        // 图例。
        std::vector<std::pair<std::string, cv::Scalar>> legend_entries;
        for (const auto& ctx : ctxs) {
            legend_entries.emplace_back(ctx.series.label, ctx.series.color);
        }
        if (start_end_drawn) {
            // 标注起终点颜色含义：橘黄色=起点位姿，绿色=终点位姿。
            legend_entries.emplace_back("Start Pose", start_color);
            legend_entries.emplace_back("End Pose", end_color);
        }
        drawDetailSubplotLegend(plot_rect, legend_entries);
        sanctum_.putRawText("Spatial Geometry",
                            cv::Point(rect.x + 8, rect.y + 18), 0.5,
                            cv::Scalar(35, 35, 35), 1);
        sanctum_.putRawText("x(m)",
                            cv::Point(plot_rect.x + plot_rect.width / 2 - 24,
                                      plot_rect.y + plot_rect.height + 34),
                            0.42, cv::Scalar(80, 80, 80), 1);
        sanctum_.putRawText("y(m)", cv::Point(rect.x + 10, rect.y + 40), 0.42,
                            cv::Scalar(80, 80, 80), 1);
    }
    // 将多组折线按 clip_rect 逐段裁剪绘制。
    static void drawPolylinesClipped(
        visualizer::LumenSanctum& sanctum,
        const std::vector<std::vector<cv::Point>>& groups,
        const cv::Rect& clip_rect, const cv::Scalar& color, int thickness) {
        for (const auto& group : groups) {
            for (std::size_t i = 0; i + 1 < group.size(); ++i) {
                cv::Point p0 = group[i];
                cv::Point p1 = group[i + 1];
                if (cv::clipLine(clip_rect, p0, p1)) {
                    sanctum.drawLine(p0, p1, color, thickness);
                }
            }
        }
    }
    // 在子图右上角绘制小图例。
    void drawDetailSubplotLegend(
        const cv::Rect& plot_rect,
        const std::vector<std::pair<std::string, cv::Scalar>>& entries,
        int x_offset = 0) {
        if (entries.empty()) {
            return;
        }
        constexpr int kLineWidth = 24;
        constexpr int kLineHeight = 12;
        constexpr double kFontScale = 0.32;
        const cv::Scalar kTextColor(40, 40, 40);
        int total_height = 0;
        int max_width = 0;
        for (const auto& [label, clr] : entries) {
            if (visualizer::LumenSanctum::IsBlankText(label)) {
                continue;
            }
            (void)clr;
            int baseline = 0;
            max_width = std::max(
                max_width, cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX,
                                           kFontScale, 1, &baseline)
                               .width);
            total_height += kLineHeight + 2;
        }
        if (total_height == 0) {
            return;
        }
        const int legend_w = kLineWidth + max_width + 14;
        const int legend_h = total_height + 4;
        const int x = plot_rect.x + plot_rect.width - legend_w - 6 - x_offset;
        int y = plot_rect.y + 8;
        // 浅灰填充底框。
        sanctum_.drawRectangle(cv::Point(x - 2, y - 2),
                               cv::Point(x + legend_w + 2, y + legend_h - 2),
                               cv::Scalar(245, 245, 245), -1);
        sanctum_.drawRectangle(cv::Point(x - 2, y - 2),
                               cv::Point(x + legend_w + 2, y + legend_h - 2),
                               cv::Scalar(220, 220, 220), 1);
        for (const auto& [label, clr] : entries) {
            if (visualizer::LumenSanctum::IsBlankText(label)) {
                continue;
            }
            sanctum_.drawLine(cv::Point(x, y + kLineHeight / 2),
                              cv::Point(x + kLineWidth, y + kLineHeight / 2),
                              clr, 2);
            sanctum_.putRawText(
                label, cv::Point(x + kLineWidth + 4, y + kLineHeight / 2 + 3),
                kFontScale, kTextColor, 1);
            y += kLineHeight + 2;
        }
    }
    // 多曲线垂直堆叠子图：共享 X 轴，支持每 band 内叠加多条曲线同框对比。
    // band_series_list[b] 是第 b 个 band 的曲线列表（同 band 内曲线共享 Y
    // 轴量纲）。
    template <typename TXGetter, typename TYGetter>
    void drawDetailSubplotStacked(
        const std::vector<std::vector<DetailSeriesData>>& band_series_list,
        const std::vector<TYGetter>& y_getters,
        const std::vector<std::string>& y_labels, const cv::Rect& rect,
        const std::string& title, const std::string& x_label, TXGetter x_getter,
        bool use_fixed_y = false, double fixed_min_y = 0.0,
        double fixed_max_y = 0.0,
        const std::vector<std::pair<double, cv::Scalar>>& reference_lines = {},
        int y_target_ticks = 3,
        const std::vector<std::vector<std::pair<double, cv::Scalar>>>&
            band_refs = {}) {
        const int n_bands = static_cast<int>(band_series_list.size());
        if (n_bands == 0 || n_bands != static_cast<int>(y_getters.size()) ||
            n_bands != static_cast<int>(y_labels.size())) {
            return;
        }
        // 白底 + 外框。
        sanctum_.drawRectangle(rect.tl(), rect.br(), cv::Scalar(255, 255, 255),
                               -1);
        sanctum_.drawRectangle(rect.tl(), rect.br(), cv::Scalar(225, 225, 225),
                               1);
        // 统一 X 范围：遍历所有 band 内所有 series。
        double min_x = INFINITY;
        double max_x = -INFINITY;
        bool has_data = false;
        for (int i = 0; i < n_bands; ++i) {
            for (const auto& series : band_series_list[i]) {
                const auto& xs = std::invoke(x_getter, series);
                if (xs.size() < 2) {
                    continue;
                }
                has_data = true;
                const auto [cur_min, cur_max] =
                    std::minmax_element(xs.begin(), xs.end());
                min_x = std::min(min_x, *cur_min);
                max_x = std::max(max_x, *cur_max);
            }
        }
        if (!has_data) {
            sanctum_.putRawText(title, cv::Point(rect.x + 8, rect.y + 18), 0.5,
                                cv::Scalar(40, 40, 40), 1);
            return;
        }
        // 总标题。
        if (!visualizer::LumenSanctum::IsBlankText(title)) {
            sanctum_.putRawText(title, cv::Point(rect.x + 8, rect.y + 14), 0.5,
                                cv::Scalar(35, 35, 35), 1);
        }
        const int left_pad = 58;
        const int right_pad = 72;
        const int title_height = 22;
        const int bottom_axis_h = 26;
        const int band_gap = 2;
        const int inner_pad = 2;
        const int band_h = (rect.height - title_height - bottom_axis_h -
                            (n_bands - 1) * band_gap) /
                           n_bands;
        for (int band_idx = 0; band_idx < n_bands; ++band_idx) {
            const cv::Rect band_rect(
                rect.x, rect.y + title_height + band_idx * (band_h + band_gap),
                rect.width, band_h);
            const int plot_y = band_rect.y + inner_pad;
            const int plot_h =
                band_rect.height - inner_pad -
                (band_idx == n_bands - 1 ? bottom_axis_h : inner_pad);
            const cv::Rect plot_rect(band_rect.x + left_pad, plot_y,
                                     band_rect.width - left_pad - right_pad,
                                     plot_h);
            if (plot_rect.width <= 10 || plot_rect.height <= 10) {
                continue;
            }
            const auto& band_series = band_series_list[band_idx];
            if (band_series.empty()) {
                continue;
            }
            // 计算 Y 范围：遍历 band 内所有 series。
            double min_y = INFINITY;
            double max_y = -INFINITY;
            bool band_has_data = false;
            for (const auto& series : band_series) {
                const auto& ys = std::invoke(y_getters[band_idx], series);
                if (ys.size() < 2) {
                    continue;
                }
                band_has_data = true;
                for (double y : ys) {
                    if (std::isfinite(y)) {
                        min_y = std::min(min_y, y);
                        max_y = std::max(max_y, y);
                    }
                }
            }
            if (!band_has_data) {
                continue;
            }
            if (use_fixed_y) {
                min_y = fixed_min_y;
                max_y = fixed_max_y;
            }
            for (const auto& [ref_y, ref_color] : reference_lines) {
                min_y = std::min(min_y, ref_y);
                max_y = std::max(max_y, ref_y);
            }
            double plot_min_x = min_x;
            double plot_max_x = max_x;
            normalizePlotRange(plot_min_x, plot_max_x, min_y, max_y, plot_rect,
                               false, 0.08);
            const double x_span = plot_max_x - plot_min_x;
            const double y_span = max_y - min_y;
            const auto map_to_pixel = [&](double x, double y) {
                return cv::Point(
                    plot_rect.x +
                        static_cast<int>(std::lround((x - plot_min_x) / x_span *
                                                     plot_rect.width)),
                    plot_rect.y +
                        static_cast<int>(std::lround(
                            (1.0 - (y - min_y) / y_span) * plot_rect.height)));
            };
            const cv::Scalar kGridColor(238, 238, 238);
            const cv::Scalar kAxisColor(205, 205, 205);
            const cv::Scalar kTextColor(120, 120, 120);
            // 水平网格与 Y 刻度。
            const double y_interval =
                ComputeCoordinateInterval(y_span, y_target_ticks);
            const int y_precision = ComputeCoordinatePrecision(y_interval);
            for (double value = ComputeGridStartCoord(min_y, y_interval);
                 value <= max_y + y_interval * 0.5; value += y_interval) {
                const double ny = (value - min_y) / y_span;
                const int y = plot_rect.y + static_cast<int>(std::lround(
                                                (1.0 - ny) * plot_rect.height));
                sanctum_.drawLine(cv::Point(plot_rect.x, y),
                                  cv::Point(plot_rect.x + plot_rect.width, y),
                                  kGridColor, 1);
                sanctum_.putRawText(FormatNumber(value, y_precision),
                                    cv::Point(plot_rect.x - 54, y + 3), 0.28,
                                    kTextColor, 1);
            }
            // 仅最底部 band 绘制 X 轴刻度。
            if (band_idx == n_bands - 1) {
                const double x_interval = ComputeCoordinateInterval(x_span, 5);
                const int x_precision = ComputeCoordinatePrecision(x_interval);
                for (double value =
                         ComputeGridStartCoord(plot_min_x, x_interval);
                     value <= plot_max_x + x_interval * 0.5;
                     value += x_interval) {
                    const double nx = (value - plot_min_x) / x_span;
                    const int x =
                        plot_rect.x +
                        static_cast<int>(std::lround(nx * plot_rect.width));
                    sanctum_.drawLine(
                        cv::Point(x, plot_rect.y),
                        cv::Point(x, plot_rect.y + plot_rect.height),
                        kGridColor, 1);
                    sanctum_.putRawText(
                        FormatNumber(value, x_precision),
                        cv::Point(x - 18, plot_rect.y + plot_rect.height + 10),
                        0.28, kTextColor, 1);
                }
                sanctum_.putRawText(
                    x_label,
                    cv::Point(plot_rect.x + plot_rect.width / 2 - 24,
                              band_rect.y + band_rect.height - 2),
                    0.38, cv::Scalar(80, 80, 80), 1);
            }
            // 边框。
            sanctum_.drawRectangle(plot_rect.tl(), plot_rect.br(), kAxisColor,
                                   1);
            // 绘制 band 内每条曲线。
            for (const auto& series : band_series) {
                const auto& xs = std::invoke(x_getter, series);
                const auto& ys = std::invoke(y_getters[band_idx], series);
                if (xs.size() < 2 || xs.size() != ys.size()) {
                    continue;
                }
                const auto groups = splitFinitePolylines(xs, ys, map_to_pixel);
                if (!groups.empty()) {
                    drawPolylinesClipped(sanctum_, groups, plot_rect,
                                         series.color, series.line_width);
                }
                // 换挡点标记。
                for (const auto shift_idx : series.shift_indices) {
                    if (shift_idx >= xs.size()) {
                        continue;
                    }
                    const auto pt = map_to_pixel(xs[shift_idx], ys[shift_idx]);
                    if (pt.x >= plot_rect.x &&
                        pt.x < plot_rect.x + plot_rect.width &&
                        pt.y >= plot_rect.y &&
                        pt.y < plot_rect.y + plot_rect.height) {
                        sanctum_.drawCircle(pt, 3, cv::Scalar(80, 80, 80), -1);
                    }
                }
            }
            // 参考线：per-band 优先，其次全局 reference_lines。
            const auto& refs = (band_idx < static_cast<int>(band_refs.size()) &&
                                !band_refs[band_idx].empty())
                                   ? band_refs[band_idx]
                                   : reference_lines;
            for (const auto& [ref_y, ref_color] : refs) {
                if (ref_y < min_y || ref_y > max_y) {
                    continue;
                }
                const double ny = (ref_y - min_y) / y_span;
                const int ref_y_pixel =
                    plot_rect.y + static_cast<int>(std::lround(
                                      (1.0 - ny) * plot_rect.height));
                // 红色虚线：每 8px 画一段 4px 实线。
                for (int x = plot_rect.x; x < plot_rect.x + plot_rect.width;
                     x += 8) {
                    sanctum_.drawLine(
                        cv::Point(x, ref_y_pixel),
                        cv::Point(
                            std::min(x + 4, plot_rect.x + plot_rect.width),
                            ref_y_pixel),
                        ref_color, 1);
                }
                sanctum_.putRawText(
                    FormatNumber(ref_y, ComputeCoordinatePrecision(y_interval)),
                    cv::Point(plot_rect.x + plot_rect.width + 4,
                              ref_y_pixel + 3),
                    0.26, ref_color, 1);
            }
            // Band 标签（中性灰色，表示信号类型）。
            sanctum_.putRawText(
                y_labels[band_idx],
                cv::Point(plot_rect.x + plot_rect.width - 4, plot_rect.y + 10),
                0.30, cv::Scalar(80, 80, 80), 1);
        }
    }
};
}  // namespace apa_post_processor