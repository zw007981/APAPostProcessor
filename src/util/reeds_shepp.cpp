#include "reeds_shepp.h"

#include <algorithm>
#include <cmath>
#include <initializer_list>
#include <stdexcept>
#include <utility>

#include "constants.h"

namespace apa_post_processor {
namespace {
// 半圆与整圆的归一化常量（归一化尺度下转弯半径为 1，故圆心角即弧长）
constexpr double HALF_PI = 0.5 * PI;
// 闭式解的可行性判据容差：解析式在临界构型上会产生 ~1e-16 量级的负零，
// 直接用 >= 0 判据会误杀这些实际可行的解
constexpr double FEASIBILITY_TOL = 1e-12;

// 角度归一化到 [-π, π]
double Mod2Pi(double angle) { return std::remainder(angle, 2.0 * PI); }

// 直角坐标转极坐标，返回 (半径, 幅角)
std::pair<double, double> Polar(double x, double y) {
    return {std::hypot(x, y), std::atan2(y, x)};
}

// CCCC 族闭式解共用的中间量（Reeds-Shepp 原文公式 8.5/8.6）：由两段中间
// 圆弧的转角 u、v 反解首尾两段的转角 tau、omega
std::pair<double, double> TauOmega(double u, double v, double xi, double eta,
                                   double phi) {
    const double delta = Mod2Pi(u - v);
    const double a = std::sin(u) - std::sin(delta);
    const double b = std::cos(u) - std::cos(delta) - 1.0;
    const double t1 = std::atan2(eta * a - xi * b, xi * a + eta * b);
    const double t2 = 2.0 * (std::cos(delta) - std::cos(v) - std::cos(u)) + 3.0;
    const double tau = (t2 < 0.0) ? Mod2Pi(t1 + PI) : Mod2Pi(t1);
    return {tau, Mod2Pi(tau - u + v - phi)};
}

// 闭式解的三元结果：是否有解 + 三段基元的转角/长度
struct RsTriple {
    bool ok{false};
    double t{0.0};
    double u{0.0};
    double v{0.0};
};

// L+S+L+ 型（CSC 同向，公式 8.1）
RsTriple LpSpLp(double x, double y, double phi) {
    const auto [u, t] = Polar(x - std::sin(phi), y - 1.0 + std::cos(phi));
    if (t < -FEASIBILITY_TOL) {
        return {};
    }
    const double v = Mod2Pi(phi - t);
    if (v < -FEASIBILITY_TOL) {
        return {};
    }
    return {true, t, u, v};
}

// L+S+R+ 型（CSC 异向，公式 8.2）
RsTriple LpSpRp(double x, double y, double phi) {
    const auto [u1, t1] = Polar(x + std::sin(phi), y - 1.0 - std::cos(phi));
    const double u1_sq = u1 * u1;
    if (u1_sq < 4.0) {
        return {};
    }
    const double u = std::sqrt(u1_sq - 4.0);
    const double t = Mod2Pi(t1 + std::atan2(2.0, u));
    const double v = Mod2Pi(t - phi);
    if (t < -FEASIBILITY_TOL || v < -FEASIBILITY_TOL) {
        return {};
    }
    return {true, t, u, v};
}

// L+R-L 型（CCC，公式 8.3/8.4）
RsTriple LpRmL(double x, double y, double phi) {
    const double xi = x - std::sin(phi);
    const double eta = y - 1.0 + std::cos(phi);
    const auto [u1, theta] = Polar(xi, eta);
    if (u1 > 4.0) {
        return {};
    }
    const double u = -2.0 * std::asin(0.25 * u1);
    const double t = Mod2Pi(theta + 0.5 * u + PI);
    const double v = Mod2Pi(phi - t + u);
    if (t < -FEASIBILITY_TOL || u > FEASIBILITY_TOL) {
        return {};
    }
    return {true, t, u, v};
}

// L+R+L-R- 型（CCCC，公式 8.7）：两段中间弧转角互为相反数
RsTriple LpRupLumRm(double x, double y, double phi) {
    const double xi = x + std::sin(phi);
    const double eta = y - 1.0 - std::cos(phi);
    const double rho = 0.25 * (2.0 + std::hypot(xi, eta));
    if (rho > 1.0) {
        return {};
    }
    const double u = std::acos(rho);
    const auto [t, v] = TauOmega(u, -u, xi, eta, phi);
    if (t < -FEASIBILITY_TOL || v > FEASIBILITY_TOL) {
        return {};
    }
    return {true, t, u, v};
}

// L+R-L-R+ 型（CCCC，公式 8.8）：两段中间弧转角相同
RsTriple LpRumLumRp(double x, double y, double phi) {
    const double xi = x + std::sin(phi);
    const double eta = y - 1.0 - std::cos(phi);
    const double rho = (20.0 - xi * xi - eta * eta) / 16.0;
    if (rho < 0.0 || rho > 1.0) {
        return {};
    }
    const double u = -std::acos(rho);
    if (u < -HALF_PI) {
        return {};
    }
    const auto [t, v] = TauOmega(u, u, xi, eta, phi);
    if (t < -FEASIBILITY_TOL || v < -FEASIBILITY_TOL) {
        return {};
    }
    return {true, t, u, v};
}

// L+R-S-L- 型（CCSC 同向收尾，公式 8.9）：中间隐含一段 -π/2 的圆弧
RsTriple LpRmSmLm(double x, double y, double phi) {
    const double xi = x - std::sin(phi);
    const double eta = y - 1.0 + std::cos(phi);
    const auto [rho, theta] = Polar(xi, eta);
    if (rho < 2.0) {
        return {};
    }
    const double r = std::sqrt(rho * rho - 4.0);
    const double u = 2.0 - r;
    const double t = Mod2Pi(theta + std::atan2(r, -2.0));
    const double v = Mod2Pi(phi - HALF_PI - t);
    if (t < -FEASIBILITY_TOL || u > FEASIBILITY_TOL || v > FEASIBILITY_TOL) {
        return {};
    }
    return {true, t, u, v};
}

// L+R-S-R- 型（CCSC 异向收尾，公式 8.10）
RsTriple LpRmSmRm(double x, double y, double phi) {
    const double xi = x + std::sin(phi);
    const double eta = y - 1.0 - std::cos(phi);
    const auto [rho, theta] = Polar(-eta, xi);
    if (rho < 2.0) {
        return {};
    }
    const double t = theta;
    const double u = 2.0 - rho;
    const double v = Mod2Pi(t + HALF_PI - phi);
    if (t < -FEASIBILITY_TOL || u > FEASIBILITY_TOL || v > FEASIBILITY_TOL) {
        return {};
    }
    return {true, t, u, v};
}

// L+R-S-L-R+ 型（CCSCC，公式 8.11）：首尾各隐含一段 -π/2 的圆弧
RsTriple LpRmSLmRp(double x, double y, double phi) {
    const double xi = x + std::sin(phi);
    const double eta = y - 1.0 - std::cos(phi);
    const auto [rho, theta] = Polar(xi, eta);
    if (rho < 2.0) {
        return {};
    }
    const double u = 4.0 - std::sqrt(rho * rho - 4.0);
    if (u > FEASIBILITY_TOL) {
        return {};
    }
    const double t = Mod2Pi(std::atan2((4.0 - u) * xi - 2.0 * eta,
                                       -2.0 * xi + (u - 4.0) * eta));
    const double v = Mod2Pi(t - phi);
    if (t < -FEASIBILITY_TOL || v < -FEASIBILITY_TOL) {
        return {};
    }
    return {true, t, u, v};
}

// 最短候选累积器：逐个吞入候选词，只保留归一化总长最小的一条
class RsBestKeeper {
   public:
    void consider(std::initializer_list<RsSegment> segments) {
        double total = 0.0;
        for (const auto& segment : segments) {
            total += std::fabs(segment.length);
        }
        if (best_.valid && total >= best_.normalized_length) {
            return;
        }
        best_.num_segments = static_cast<int>(segments.size());
        best_.normalized_length = total;
        best_.valid = true;
        int index = 0;
        for (const auto& segment : segments) {
            best_.segments[index] = segment;
            ++index;
        }
    }
    const RsPath& best() const { return best_; }

   protected:
    RsPath best_{};
};

// 构造基元的语法糖：把转向类型与有符号长度打包
RsSegment MakeSegment(RsSteer steer, double length) { return {steer, length}; }

// L/R 互换（reflect 对称：把绕 x 轴镜像后的解翻译回原问题）
RsSteer Mirror(RsSteer steer) {
    if (steer == RsSteer::LEFT) {
        return RsSteer::RIGHT;
    }
    if (steer == RsSteer::RIGHT) {
        return RsSteer::LEFT;
    }
    return RsSteer::STRAIGHT;
}

// CSC 族：直行段夹在两段圆弧之间
void EnumerateCsc(double x, double y, double phi, RsBestKeeper* keeper) {
    // 四种对称变换共用的登记器：pattern 为原始（未镜像）转向序列，
    // mirrored 为真时整体做 L/R 互换
    const auto add3 = [&](const RsTriple& r, std::array<RsSteer, 3> pattern,
                          bool mirrored, double sign) {
        if (!r.ok) {
            return;
        }
        const auto s = [&](int i) {
            return mirrored ? Mirror(pattern[i]) : pattern[i];
        };
        keeper->consider({MakeSegment(s(0), sign * r.t),
                          MakeSegment(s(1), sign * r.u),
                          MakeSegment(s(2), sign * r.v)});
    };
    constexpr std::array<RsSteer, 3> LSL{
        {RsSteer::LEFT, RsSteer::STRAIGHT, RsSteer::LEFT}};
    constexpr std::array<RsSteer, 3> LSR{
        {RsSteer::LEFT, RsSteer::STRAIGHT, RsSteer::RIGHT}};
    add3(LpSpLp(x, y, phi), LSL, false, 1.0);
    add3(LpSpLp(-x, y, -phi), LSL, false, -1.0);
    add3(LpSpLp(x, -y, -phi), LSL, true, 1.0);
    add3(LpSpLp(-x, -y, phi), LSL, true, -1.0);
    add3(LpSpRp(x, y, phi), LSR, false, 1.0);
    add3(LpSpRp(-x, y, -phi), LSR, false, -1.0);
    add3(LpSpRp(x, -y, -phi), LSR, true, 1.0);
    add3(LpSpRp(-x, -y, phi), LSR, true, -1.0);
}

// CCC 族：三段圆弧相接，另需枚举「反向行驶同一条曲线」的时序倒置解
void EnumerateCcc(double x, double y, double phi, RsBestKeeper* keeper) {
    const auto add3 = [&](const RsTriple& r, bool mirrored, double sign,
                          bool reversed) {
        if (!r.ok) {
            return;
        }
        const auto s = [&](RsSteer steer) {
            return mirrored ? Mirror(steer) : steer;
        };
        // 时序倒置时首尾两段的转角互换（中间段自洽）
        const double a = reversed ? r.v : r.t;
        const double c = reversed ? r.t : r.v;
        keeper->consider({MakeSegment(s(RsSteer::LEFT), sign * a),
                          MakeSegment(s(RsSteer::RIGHT), sign * r.u),
                          MakeSegment(s(RsSteer::LEFT), sign * c)});
    };
    add3(LpRmL(x, y, phi), false, 1.0, false);
    add3(LpRmL(-x, y, -phi), false, -1.0, false);
    add3(LpRmL(x, -y, -phi), true, 1.0, false);
    add3(LpRmL(-x, -y, phi), true, -1.0, false);
    // 起终点互换后的等价问题（backwards 对称）
    const double xb = x * std::cos(phi) + y * std::sin(phi);
    const double yb = x * std::sin(phi) - y * std::cos(phi);
    add3(LpRmL(xb, yb, phi), false, 1.0, true);
    add3(LpRmL(-xb, yb, -phi), false, -1.0, true);
    add3(LpRmL(xb, -yb, -phi), true, 1.0, true);
    add3(LpRmL(-xb, -yb, phi), true, -1.0, true);
}

// CCCC 族：四段圆弧相接，中间两段转角大小相同
void EnumerateCccc(double x, double y, double phi, RsBestKeeper* keeper) {
    const auto add4 = [&](const RsTriple& r, bool mirrored, double sign,
                          double mid_sign) {
        if (!r.ok) {
            return;
        }
        const auto s = [&](RsSteer steer) {
            return mirrored ? Mirror(steer) : steer;
        };
        keeper->consider({MakeSegment(s(RsSteer::LEFT), sign * r.t),
                          MakeSegment(s(RsSteer::RIGHT), sign * r.u),
                          MakeSegment(s(RsSteer::LEFT), sign * mid_sign * r.u),
                          MakeSegment(s(RsSteer::RIGHT), sign * r.v)});
    };
    add4(LpRupLumRm(x, y, phi), false, 1.0, -1.0);
    add4(LpRupLumRm(-x, y, -phi), false, -1.0, -1.0);
    add4(LpRupLumRm(x, -y, -phi), true, 1.0, -1.0);
    add4(LpRupLumRm(-x, -y, phi), true, -1.0, -1.0);
    add4(LpRumLumRp(x, y, phi), false, 1.0, 1.0);
    add4(LpRumLumRp(-x, y, -phi), false, -1.0, 1.0);
    add4(LpRumLumRp(x, -y, -phi), true, 1.0, 1.0);
    add4(LpRumLumRp(-x, -y, phi), true, -1.0, 1.0);
}

// CCSC 族：两段圆弧 + 直行段 + 一段圆弧，其中一段圆弧恒为 ±π/2
void EnumerateCcsc(double x, double y, double phi, RsBestKeeper* keeper) {
    // 正序解：C(t) C(∓π/2) S(u) C(v)，第三段转向由 tail 指定
    const auto add_forward = [&](const RsTriple& r, RsSteer tail, bool mirrored,
                                 double sign) {
        if (!r.ok) {
            return;
        }
        const auto s = [&](RsSteer steer) {
            return mirrored ? Mirror(steer) : steer;
        };
        keeper->consider({MakeSegment(s(RsSteer::LEFT), sign * r.t),
                          MakeSegment(s(RsSteer::RIGHT), sign * -HALF_PI),
                          MakeSegment(RsSteer::STRAIGHT, sign * r.u),
                          MakeSegment(s(tail), sign * r.v)});
    };
    // 倒序解（起终点互换）：C(v) S(u) C(∓π/2) C(t)
    const auto add_backward = [&](const RsTriple& r, RsSteer head,
                                  bool mirrored, double sign) {
        if (!r.ok) {
            return;
        }
        const auto s = [&](RsSteer steer) {
            return mirrored ? Mirror(steer) : steer;
        };
        keeper->consider({MakeSegment(s(head), sign * r.v),
                          MakeSegment(RsSteer::STRAIGHT, sign * r.u),
                          MakeSegment(s(RsSteer::RIGHT), sign * -HALF_PI),
                          MakeSegment(s(RsSteer::LEFT), sign * r.t)});
    };
    add_forward(LpRmSmLm(x, y, phi), RsSteer::LEFT, false, 1.0);
    add_forward(LpRmSmLm(-x, y, -phi), RsSteer::LEFT, false, -1.0);
    add_forward(LpRmSmLm(x, -y, -phi), RsSteer::LEFT, true, 1.0);
    add_forward(LpRmSmLm(-x, -y, phi), RsSteer::LEFT, true, -1.0);
    add_forward(LpRmSmRm(x, y, phi), RsSteer::RIGHT, false, 1.0);
    add_forward(LpRmSmRm(-x, y, -phi), RsSteer::RIGHT, false, -1.0);
    add_forward(LpRmSmRm(x, -y, -phi), RsSteer::RIGHT, true, 1.0);
    add_forward(LpRmSmRm(-x, -y, phi), RsSteer::RIGHT, true, -1.0);
    const double xb = x * std::cos(phi) + y * std::sin(phi);
    const double yb = x * std::sin(phi) - y * std::cos(phi);
    add_backward(LpRmSmLm(xb, yb, phi), RsSteer::LEFT, false, 1.0);
    add_backward(LpRmSmLm(-xb, yb, -phi), RsSteer::LEFT, false, -1.0);
    add_backward(LpRmSmLm(xb, -yb, -phi), RsSteer::LEFT, true, 1.0);
    add_backward(LpRmSmLm(-xb, -yb, phi), RsSteer::LEFT, true, -1.0);
    add_backward(LpRmSmRm(xb, yb, phi), RsSteer::RIGHT, false, 1.0);
    add_backward(LpRmSmRm(-xb, yb, -phi), RsSteer::RIGHT, false, -1.0);
    add_backward(LpRmSmRm(xb, -yb, -phi), RsSteer::RIGHT, true, 1.0);
    add_backward(LpRmSmRm(-xb, -yb, phi), RsSteer::RIGHT, true, -1.0);
}

// CCSCC 族：直行段两侧各夹一对圆弧，内侧两段恒为 ±π/2
void EnumerateCcscc(double x, double y, double phi, RsBestKeeper* keeper) {
    const auto add5 = [&](const RsTriple& r, bool mirrored, double sign) {
        if (!r.ok) {
            return;
        }
        const auto s = [&](RsSteer steer) {
            return mirrored ? Mirror(steer) : steer;
        };
        keeper->consider({MakeSegment(s(RsSteer::LEFT), sign * r.t),
                          MakeSegment(s(RsSteer::RIGHT), sign * -HALF_PI),
                          MakeSegment(RsSteer::STRAIGHT, sign * r.u),
                          MakeSegment(s(RsSteer::LEFT), sign * -HALF_PI),
                          MakeSegment(s(RsSteer::RIGHT), sign * r.v)});
    };
    add5(LpRmSLmRp(x, y, phi), false, 1.0);
    add5(LpRmSLmRp(-x, y, -phi), false, -1.0);
    add5(LpRmSLmRp(x, -y, -phi), true, 1.0);
    add5(LpRmSLmRp(-x, -y, phi), true, -1.0);
}

// 沿单段基元推进位姿：normalized_step 为归一化有符号步长
Pose AdvancePose(const Pose& pose, RsSteer steer, double normalized_step,
                 double turning_radius) {
    if (steer == RsSteer::STRAIGHT) {
        const double ds = normalized_step * turning_radius;
        return Pose{pose.x + ds * std::cos(pose.theta),
                    pose.y + ds * std::sin(pose.theta), pose.theta};
    }
    // 左转曲率为正、右转为负；圆弧积分的闭式解对正负步长同样成立
    const double turn_sign = (steer == RsSteer::LEFT) ? 1.0 : -1.0;
    const double d_theta = turn_sign * normalized_step;
    const double new_theta = pose.theta + d_theta;
    return Pose{pose.x + turn_sign * turning_radius *
                             (std::sin(new_theta) - std::sin(pose.theta)),
                pose.y + turn_sign * turning_radius *
                             (std::cos(pose.theta) - std::cos(new_theta)),
                new_theta};
}
}  // namespace

int RsPath::numCusps() const {
    int cusps = 0;
    double prev_sign = 0.0;
    for (int i = 0; i < num_segments; ++i) {
        const double length = segments[i].length;
        if (std::fabs(length) <= FEASIBILITY_TOL) {
            continue;
        }
        const double sign = (length > 0.0) ? 1.0 : -1.0;
        if (prev_sign != 0.0 && sign != prev_sign) {
            ++cusps;
        }
        prev_sign = sign;
    }
    return cusps;
}

RsPath ComputeShortestReedsShepp(const Pose& start, const Pose& goal,
                                 double turning_radius) {
    if (!(turning_radius > 0.0)) {
        throw std::invalid_argument(
            "ComputeShortestReedsShepp: 转弯半径必须为正");
    }
    // 归一化到「起点位于原点、航向为 0、转弯半径为 1」的标准问题
    const double dx = (goal.x - start.x) / turning_radius;
    const double dy = (goal.y - start.y) / turning_radius;
    const double cos_theta = std::cos(start.theta);
    const double sin_theta = std::sin(start.theta);
    const double x = dx * cos_theta + dy * sin_theta;
    const double y = -dx * sin_theta + dy * cos_theta;
    const double phi = Mod2Pi(goal.theta - start.theta);
    RsBestKeeper keeper;
    EnumerateCsc(x, y, phi, &keeper);
    EnumerateCcc(x, y, phi, &keeper);
    EnumerateCccc(x, y, phi, &keeper);
    EnumerateCcsc(x, y, phi, &keeper);
    EnumerateCcscc(x, y, phi, &keeper);
    return keeper.best();
}

std::vector<RsSamplePoint> SampleReedsShepp(const RsPath& path,
                                            const Pose& start,
                                            double turning_radius,
                                            double sample_dist) {
    if (!path.valid) {
        throw std::invalid_argument("SampleReedsShepp: 输入 RS 路径无解");
    }
    if (!(turning_radius > 0.0) || !(sample_dist > 0.0)) {
        throw std::invalid_argument(
            "SampleReedsShepp: 转弯半径与采样步长必须为正");
    }
    std::vector<RsSamplePoint> samples;
    // 预估容量：总弧长除以步长，另加各段端点与首点的余量
    samples.reserve(static_cast<std::size_t>(
                        path.arcLength(turning_radius) / sample_dist) +
                    path.segments.size() + 2);
    samples.push_back({start, true});
    Pose cursor = start;
    for (int i = 0; i < path.num_segments; ++i) {
        const auto& segment = path.segments[i];
        if (std::fabs(segment.length) <= FEASIBILITY_TOL) {
            continue;
        }
        const bool forward = segment.length > 0.0;
        // 首点的行驶方向应取第一段有效基元的方向
        if (samples.size() == 1) {
            samples.front().forward = forward;
        }
        const double arc = std::fabs(segment.length) * turning_radius;
        const int steps = std::max(1, static_cast<int>(std::ceil(
                                          arc / sample_dist)));
        const double step = segment.length / static_cast<double>(steps);
        for (int k = 0; k < steps; ++k) {
            cursor = AdvancePose(cursor, segment.steer, step, turning_radius);
            samples.push_back({cursor, forward});
        }
    }
    return samples;
}
}  // namespace apa_post_processor
