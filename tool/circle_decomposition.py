#!/usr/bin/env python3

"""
车辆圆分解可视化脚本
演示 VehicleFootprintModel 中内圆（INNER）和外圆（OUTER）的几何覆盖效果。
  - 车辆尺寸与 data/*.json 中的实际参数一致
  - 额外圆（additional circles）仅在本脚本中用于可视化参考，C++ 求解器不使用
"""

import math

import matplotlib.patches as patches
import matplotlib.pyplot as plt


class VehicleCircleDecomposition:
    """车辆圆分解：与 C++ VehicleFootprintModel 算法一致。"""

    def __init__(self):
        # ── 车辆尺寸参数（与 data 文件中的实际参数一致） ──
        self.vehicle_length = 5.0       # 车长 (m)
        self.vehicle_width = 1.9        # 车宽 (m)
        self.wheelbase = 3.0            # 轴距 (m)，仅用于标注后轴位置
        self.rear_overhang = 1.1        # 后轴到后保险杠距离 (m)

        # ── 圆分解配置（与 C++ 运行时一致） ──
        # 注意：VehicleFootprintModel 声明默认 outer_row_num=4，
        # 但 planning_scene.cpp 构造时显式传入 outer_row_num=2
        self.inner_row_num = 2
        self.outer_row_num = 2           # ← 实际运行时为 2

        # 存储计算结果
        self.inner_circles: list[tuple[float, float, float, str]] = []
        self.outer_circles: list[tuple[float, float, float, str]] = []
        self.outer_radius: float = 0.0

    # ═══════════════════════════════════════════════════════════════
    # 内圆分解（INNER）
    # 解一元二次方程求最优半径 R，使圆紧贴车身矩形边界、
    # 对角线相邻圆相切，完整覆盖车身内部。
    # 对应 C++: VehicleFootprintModel::generateCirclesAtOrigin() 的 INNER 分支
    # ═══════════════════════════════════════════════════════════════
    def decompose_inner_circles(self):
        col_num = math.ceil(
            self.vehicle_length / self.vehicle_width * self.inner_row_num
        )
        q = 1.0 / ((col_num - 1) ** 2) if col_num > 1 else 0.0

        if self.inner_row_num == 1:
            radius = self.vehicle_width * 0.5
            chord_row = 0.0
        else:
            p = 1.0 / ((self.inner_row_num - 1) ** 2)
            a = p + q - 1.0
            b = -(p * self.vehicle_width + q * self.vehicle_length)
            c = 0.25 * (p * self.vehicle_width**2 + q * self.vehicle_length**2)
            delta = max(b**2 - 4 * a * c, 0.0)
            radius = 0.5 * (-b - math.sqrt(delta)) / a
            chord_row = math.sqrt(p) * (0.5 * self.vehicle_width - radius) * 2

        chord_col = (
            math.sqrt(q) * (0.5 * self.vehicle_length - radius) * 2
            if col_num > 1
            else 0.0
        )

        # 基准点（后轴坐标系）：左后侧第一个内切圆
        base_x = -self.rear_overhang + radius
        base_y = 0.5 * self.vehicle_width - radius

        self.inner_circles.clear()
        for j in range(self.inner_row_num):
            for i in range(col_num):
                cx = base_x + i * chord_col
                cy = base_y - j * chord_row
                self.inner_circles.append((cx, cy, radius, "Inner"))

    # ═══════════════════════════════════════════════════════════════
    # 外圆分解（OUTER）
    # 将车身切分为网格，以网格对角线一半为外圆半径。
    # 仅保留轮廓边界上的圆（i==0, i==col-1, j==0, j==row-1），
    # 内部网格的圆被跳过，形成空心矩形轮廓。
    # 对应 C++: VehicleFootprintModel::generateCirclesAtOrigin() 的 OUTER 分支
    # ═══════════════════════════════════════════════════════════════
    def decompose_outer_circles(self):
        chord_row = self.vehicle_width / self.outer_row_num
        col_num = math.ceil(self.vehicle_length / chord_row)
        chord_col = self.vehicle_length / col_num

        self.outer_radius = 0.5 * math.sqrt(chord_col**2 + chord_row**2)

        base_x = -self.rear_overhang + chord_col * 0.5
        base_y = 0.5 * self.vehicle_width - chord_row * 0.5

        self.outer_circles.clear()
        for j in range(self.outer_row_num):
            for i in range(col_num):
                # 仅保留边界上的圆（与 C++ 一致）
                if (i != 0) and (i != col_num - 1) and (j != 0) and (j != self.outer_row_num - 1):
                    continue
                cx = base_x + i * chord_col
                cy = base_y - j * chord_row
                self.outer_circles.append((cx, cy, self.outer_radius, "Outer"))

    # ═══════════════════════════════════════════════════════════════
    # 外圆超出 OBB 分析
    # ═══════════════════════════════════════════════════════════════
    def calc_outer_overhang(self) -> dict:
        chord_row = self.vehicle_width / self.outer_row_num
        col_num = math.ceil(self.vehicle_length / chord_row)
        chord_col = self.vehicle_length / col_num
        r = self.outer_radius

        obb_front = self.vehicle_length - self.rear_overhang
        obb_rear = -self.rear_overhang
        obb_left = self.vehicle_width * 0.5
        obb_right = -self.vehicle_width * 0.5

        base_x = -self.rear_overhang + chord_col * 0.5
        base_y = 0.5 * self.vehicle_width - chord_row * 0.5
        center_x_min = base_x
        center_x_max = base_x + (col_num - 1) * chord_col
        center_y_max = base_y
        center_y_min = base_y - (self.outer_row_num - 1) * chord_row

        outer_front = center_x_max + r
        outer_rear = center_x_min - r
        outer_left = center_y_max + r
        outer_right = center_y_min - r

        return {
            "front": outer_front - obb_front,
            "rear": obb_rear - outer_rear,
            "left": outer_left - obb_left,
            "right": obb_right - outer_right,
            "radius": r,
            "chord_col": chord_col,
            "chord_row": chord_row,
            "col_num": col_num,
            "obb_front": obb_front,
            "obb_rear": obb_rear,
            "obb_left": obb_left,
            "obb_right": obb_right,
            "outer_front": outer_front,
            "outer_rear": outer_rear,
            "outer_left": outer_left,
            "outer_right": outer_right,
        }

    def print_overhang_report(self):
        d = self.calc_outer_overhang()
        print("=" * 64)
        print("  Outer Circles 超出车辆 OBB 轮廓分析")
        print("=" * 64)
        print(f"  车辆: {self.vehicle_length:.2f}×{self.vehicle_width:.2f} m, "
              f"后悬={self.rear_overhang:.2f} m, "
              f"outer_row_num={self.outer_row_num}")
        print(f"  外圆半径 r       = {d['radius']:.5f} m")
        print(f"  网格间距 (dx, dy) = ({d['chord_col']:.3f}, {d['chord_row']:.3f}) m")
        print(f"  列数 (纵向)       = {d['col_num']}")
        print(f"  行数 (横向)       = {self.outer_row_num}")
        print(f"  外圆总数          = {len(self.outer_circles)}")
        print(f"  内圆总数          = {len(self.inner_circles)}")
        print("-" * 64)
        print(f"  {'方向':<8} {'OBB 边界 (m)':>14} {'外圆外延 (m)':>14} {'超出量 (m)':>12}")
        print("-" * 64)
        print(f"  {'前方':<8} {d['obb_front']:>14.5f} {d['outer_front']:>14.5f} {d['front']:>+12.5f}")
        print(f"  {'后方':<8} {d['obb_rear']:>14.5f} {d['outer_rear']:>14.5f} {d['rear']:>+12.5f}")
        print(f"  {'左侧':<8} {d['obb_left']:>14.5f} {d['outer_left']:>14.5f} {d['left']:>+12.5f}")
        print(f"  {'右侧':<8} {d['obb_right']:>14.5f} {d['outer_right']:>14.5f} {d['right']:>+12.5f}")
        print("-" * 64)
        if abs(d["front"] - d["rear"]) < 1e-6 and abs(d["left"] - d["right"]) < 1e-6:
            print("  ✓ 前后对称、左右对称")
        print(f"  NMPC ng_max = {len(self.outer_circles)}（每步不等式约束数）")
        print("=" * 64)

    # ═══════════════════════════════════════════════════════════════
    # 可视化
    # ═══════════════════════════════════════════════════════════════
    def plot_decomposition(self, save_path: str = "circle_decomposition_plot.png"):
        fig, ax = plt.subplots(figsize=(14, 7))

        # 1. 车辆矩形轮廓（透明填充，避免实心色块遮挡内/外圆的显示细节）
        rect_x = -self.rear_overhang
        rect_y = -self.vehicle_width / 2
        car_rect = patches.Rectangle(
            (rect_x, rect_y),
            self.vehicle_length,
            self.vehicle_width,
            linewidth=2,
            edgecolor="black",
            facecolor="none",
            zorder=5,
            label="Vehicle OBB",
        )
        ax.add_patch(car_rect)

        # 2. 后轴中心
        ax.plot(0, 0, "k+", markersize=14, markeredgewidth=2, zorder=10,
                label="Rear Axle Center (0,0)")

        # 3. 外圆 (绿色虚线 —— NMPC 碰撞约束用)
        for i, (cx, cy, r, _) in enumerate(self.outer_circles):
            label = f"Outer ({len(self.outer_circles)} circles, NMPC ng)" if i == 0 else ""
            circle = patches.Circle(
                (cx, cy), r, color="green", alpha=0.12, ec="green",
                ls="--", lw=1.5, zorder=2, label=label,
            )
            ax.add_patch(circle)

        # 4. 内圆 (红色半透明实线 —— 碰撞检测/验证用)
        for i, (cx, cy, r, _) in enumerate(self.inner_circles):
            label = f"Inner ({len(self.inner_circles)} circles, validation)" if i == 0 else ""
            circle = patches.Circle(
                (cx, cy), r, color="red", alpha=0.20, ec="red",
                lw=1.5, zorder=1, label=label,
            )
            ax.add_patch(circle)

        # 视图设置
        margin = 1.0
        ax.set_aspect("equal")
        ax.set_xlim(-self.rear_overhang - margin,
                     self.vehicle_length - self.rear_overhang + margin)
        ax.set_ylim(-self.vehicle_width / 2 - margin,
                     self.vehicle_width / 2 + margin)
        ax.set_xlabel("X (m) — Forward →")
        ax.set_ylabel("Y (m) — Lateral")
        ax.set_title(
            f"Vehicle Circle Decomposition  "
            f"(L={self.vehicle_length}  W={self.vehicle_width}  "
            f"rear_overhang={self.rear_overhang}  "
            f"outer_row={self.outer_row_num}  inner_row={self.inner_row_num})",
            fontsize=12,
        )

        # 去重图例
        handles, labels = ax.get_legend_handles_labels()
        by_label = dict(zip(labels, handles))
        ax.legend(by_label.values(), by_label.keys(), loc="upper right",
                  bbox_to_anchor=(1.42, 1), fontsize=9)

        ax.grid(True, linestyle=":", alpha=0.5)
        plt.tight_layout()
        plt.savefig(save_path, dpi=300, bbox_inches="tight")
        print(f"\n示意图已保存为 '{save_path}'")
        plt.show()


if __name__ == "__main__":
    dec = VehicleCircleDecomposition()

    dec.decompose_inner_circles()
    dec.decompose_outer_circles()

    dec.print_overhang_report()
    dec.plot_decomposition()
