import math

import matplotlib.patches as patches
import matplotlib.pyplot as plt


# 用于演示用若干圆覆盖车辆 OBB 的方法
class VehicleCircleDecomposition:
    def __init__(self):
        # 车辆基础尺寸参数 (单位: 米)
        self.vehicle_length = 5.0  # 车长
        self.vehicle_width = 2.0  # 车宽
        self.vehicle_lr = 1.0  # 后轴到后保险杠的距离 (根据C++代码推断)

        # 分解配置
        self.inner_row_num = 2
        self.outer_row_num = 4
        self.keep_outer_circle_all = False

        # 存储计算出的圆心和半径 [(x, y, r, type_name), ...]
        self.inner_circles = []
        self.outer_circles = []
        self.additional_circles = []

    def decompose_inner_circles(self):
        """将车辆OBB分解为若干内圆"""
        column_num = math.ceil(
            self.vehicle_length / self.vehicle_width * self.inner_row_num
        )

        q = 1.0 / ((column_num - 1) ** 2)
        if self.inner_row_num == 1:
            radius = self.vehicle_width * 0.5
            chord_row = 0
        else:
            p = 1.0 / ((self.inner_row_num - 1) ** 2)
            a = p + q - 1.0
            b = -(p * self.vehicle_width + q * self.vehicle_length)
            c = 0.25 * (p * self.vehicle_width**2 + q * self.vehicle_length**2)
            delta = b**2 - 4 * a * c
            radius = 0.5 * (-b - math.sqrt(delta)) / a
            chord_row = math.sqrt(p) * (0.5 * self.vehicle_width - radius) * 2

        chord_col = math.sqrt(q) * (0.5 * self.vehicle_length - radius) * 2

        # 基于后轴坐标系计算基准点 (左后侧第一个内切圆)
        base_x = -self.vehicle_lr + radius
        base_y = 0.5 * self.vehicle_width - radius

        for j in range(self.inner_row_num):
            for i in range(column_num):
                cx = base_x + i * chord_col
                cy = base_y - j * chord_row
                self.inner_circles.append((cx, cy, radius, "Inner"))

    def decompose_outer_circles(self):
        """将车辆OBB分解为若干外圆"""
        chord_row = self.vehicle_width / self.outer_row_num
        column_num = math.ceil(self.vehicle_length / chord_row)
        chord_col = self.vehicle_length / column_num

        radius = 0.5 * math.sqrt(chord_col**2 + chord_row**2)

        # 基于后轴坐标系计算基准点
        base_x = -self.vehicle_lr + chord_col * 0.5
        base_y = 0.5 * self.vehicle_width - chord_row * 0.5

        for j in range(self.outer_row_num):
            for i in range(column_num):
                # 如果 keep_outer_circle_all = False，则只保留边缘的圆
                if not self.keep_outer_circle_all:
                    if (
                        (i != 0)
                        and (i != column_num - 1)
                        and (j != 0)
                        and (j != self.outer_row_num - 1)
                    ):
                        continue
                cx = base_x + i * chord_col
                cy = base_y - j * chord_row
                self.outer_circles.append((cx, cy, radius, "Outer"))

    def gen_additional_circles(self):
        """在必要时生成额外圆以增加安全性"""
        # 四个角点圆
        front_x = self.vehicle_length - self.vehicle_lr
        rear_x = -self.vehicle_lr
        left_y = self.vehicle_width * 0.5
        right_y = -self.vehicle_width * 0.5

        vertices = [
            (front_x, left_y, 0.26, "Front Corner"),  # 左前 LF
            (front_x, right_y, 0.26, "Front Corner"),  # 右前 RF
            (rear_x, left_y, 0.25, "Rear Corner"),  # 左后 LR
            (rear_x, right_y, 0.25, "Rear Corner"),  # 右后 RR
        ]
        self.additional_circles.extend(vertices)

        # 前保险杠正中间圆
        self.additional_circles.append((front_x, 0, 0.31, "Front Bump"))
        # 后保险杠正中间圆
        self.additional_circles.append((rear_x, 0, 0.25, "Rear Bump"))
        # 后轴两端圆 (后轴中心在0,0)
        self.additional_circles.append((0, left_y, 0.25, "Rear Axle"))
        self.additional_circles.append((0, right_y, 0.25, "Rear Axle"))

    def calc_outer_overhang(self):
        """计算外圆超出车辆 OBB 轮廓的距离（前/后/左/右），返回字典"""
        # 车辆 OBB 边界（后轴坐标系）
        obb_front = self.vehicle_length - self.vehicle_lr  # 车头 x
        obb_rear = -self.vehicle_lr  # 车尾 x
        obb_left = self.vehicle_width * 0.5  # 左侧 y
        obb_right = -self.vehicle_width * 0.5  # 右侧 y

        # 外圆参数（与 decompose_outer_circles 一致）
        chord_row = self.vehicle_width / self.outer_row_num
        column_num = math.ceil(self.vehicle_length / chord_row)
        chord_col = self.vehicle_length / column_num
        radius = 0.5 * math.sqrt(chord_col**2 + chord_row**2)

        # 外圆中心 x 范围：base_x + [0, column_num-1] * chord_col
        base_x = -self.vehicle_lr + chord_col * 0.5
        center_x_min = base_x  # i=0
        center_x_max = base_x + (column_num - 1) * chord_col  # i=column_num-1

        # 外圆中心 y 范围：base_y - [0, outer_row_num-1] * chord_row
        base_y = 0.5 * self.vehicle_width - chord_row * 0.5
        center_y_max = base_y  # j=0
        center_y_min = (
            base_y - (self.outer_row_num - 1) * chord_row
        )  # j=outer_row_num-1

        # 外圆在各方向上的最大外延
        outer_front = center_x_max + radius
        outer_rear = center_x_min - radius
        outer_left = center_y_max + radius
        outer_right = center_y_min - radius

        # 超出量 = 外圆外延 - OBB 边界（正值 = 超出）
        overhang_front = outer_front - obb_front
        overhang_rear = obb_rear - outer_rear  # obb_rear < outer_rear 时为正
        overhang_left = outer_left - obb_left
        overhang_right = obb_right - outer_right  # obb_right > outer_right 时为正

        return {
            "front": overhang_front,
            "rear": overhang_rear,
            "left": overhang_left,
            "right": overhang_right,
            "radius": radius,
            "chord_col": chord_col,
            "chord_row": chord_row,
            "column_num": column_num,
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
        """打印外圆超出 OBB 的详细报告"""
        d = self.calc_outer_overhang()
        print("=" * 64)
        print("  Outer Circles 超出车辆 OBB 轮廓分析")
        print("=" * 64)
        print(f"  外圆半径 r       = {d['radius']:.5f} m")
        print(f"  网格间距 (dx, dy) = ({d['chord_col']:.3f}, {d['chord_row']:.3f}) m")
        print(f"  列数 (纵向)       = {d['column_num']}")
        print(f"  行数 (横向)       = {self.outer_row_num}")
        print("-" * 64)
        print(
            f"  {'方向':<8} {'OBB 边界 (m)':>14} {'外圆外延 (m)':>14} {'超出量 (m)':>12}"
        )
        print("-" * 64)
        print(
            f"  {'前方':<8} {d['obb_front']:>14.5f} {d['outer_front']:>14.5f} {d['front']:>+12.5f}"
        )
        print(
            f"  {'后方':<8} {d['obb_rear']:>14.5f} {d['outer_rear']:>14.5f} {d['rear']:>+12.5f}"
        )
        print(
            f"  {'左侧':<8} {d['obb_left']:>14.5f} {d['outer_left']:>14.5f} {d['left']:>+12.5f}"
        )
        print(
            f"  {'右侧':<8} {d['obb_right']:>14.5f} {d['outer_right']:>14.5f} {d['right']:>+12.5f}"
        )
        print("-" * 64)
        # 对称性校验
        if abs(d["front"] - d["rear"]) < 1e-6 and abs(d["left"] - d["right"]) < 1e-6:
            print("  ✓ 前后对称、左右对称（符合预期）")
        if abs(d["front"] - d["left"]) < 1e-6:
            print("  ✓ 纵向与横向超出量相等（因为 chord_col == chord_row）")
        print("=" * 64)

    def plot_decomposition(self):
        """绘制车辆轮廓与分解圆"""
        fig, ax = plt.subplots(figsize=(12, 6))

        # 1. 绘制车辆矩形轮廓 (黑色实线)
        rect_x = -self.vehicle_lr
        rect_y = -self.vehicle_width / 2
        car_rect = patches.Rectangle(
            (rect_x, rect_y),
            self.vehicle_length,
            self.vehicle_width,
            linewidth=2,
            edgecolor="black",
            facecolor="none",
            zorder=5,
            label="Vehicle Footprint",
        )
        ax.add_patch(car_rect)

        # 2. 标出后轴中心
        ax.plot(
            0,
            0,
            "k+",
            markersize=15,
            markeredgewidth=2,
            zorder=10,
            label="Rear Axle Center",
        )

        # 3. 绘制外圆 (浅绿色，虚线边缘，偏保守外扩覆盖)
        for i, (cx, cy, r, _) in enumerate(self.outer_circles):
            label = "Outer Circles (Edge)" if i == 0 else ""
            circle = patches.Circle(
                (cx, cy),
                r,
                color="green",
                alpha=0.15,
                ec="green",
                ls="--",
                lw=1.5,
                zorder=1,
                label=label,
            )
            ax.add_patch(circle)

        # 4. 绘制内圆 (蓝色，实线边缘，偏内缩覆盖)
        for i, (cx, cy, r, _) in enumerate(self.inner_circles):
            label = "Inner Circles" if i == 0 else ""
            circle = patches.Circle(
                (cx, cy),
                r,
                color="blue",
                alpha=0.25,
                ec="blue",
                lw=1.5,
                zorder=2,
                label=label,
            )
            ax.add_patch(circle)

        # 5. 绘制额外圆 (红色，实线边缘，用于补角和保险杠)
        for i, (cx, cy, r, ctype) in enumerate(self.additional_circles):
            label = "Additional Circles" if i == 0 else ""
            circle = patches.Circle(
                (cx, cy),
                r,
                color="red",
                alpha=0.4,
                ec="darkred",
                lw=1.5,
                zorder=3,
                label=label,
            )
            ax.add_patch(circle)
            # 在额外圆中心打个小点并标注类型(可选)
            ax.plot(cx, cy, "r.", markersize=5, zorder=4)

        # 视图设置
        ax.set_aspect("equal")
        ax.set_xlim(-self.vehicle_lr - 1.0, self.vehicle_length - self.vehicle_lr + 1.0)
        ax.set_ylim(-self.vehicle_width / 2 - 1.0, self.vehicle_width / 2 + 1.0)
        ax.set_xlabel("X (m) - Forward")
        ax.set_ylabel("Y (m) - Lateral")
        ax.set_title(
            f"Vehicle Circle Decomposition\n(Length={self.vehicle_length}m, Width={self.vehicle_width}m, LR={self.vehicle_lr}m)",
            fontsize=14,
        )

        # 整理图例以避免重复
        handles, labels = ax.get_legend_handles_labels()
        by_label = dict(zip(labels, handles))
        ax.legend(
            by_label.values(),
            by_label.keys(),
            loc="upper right",
            bbox_to_anchor=(1.35, 1),
        )

        plt.grid(True, linestyle=":", alpha=0.6)
        plt.tight_layout()

        # 保存图片到本地
        plt.savefig("circle_decomposition_plot.png", dpi=300, bbox_inches="tight")
        print("示意图已生成并保存为 'circle_decomposition_plot.png'")
        plt.show()


if __name__ == "__main__":
    decomposition = VehicleCircleDecomposition()

    # 按照C++逻辑进行分解
    decomposition.decompose_inner_circles()
    decomposition.decompose_outer_circles()
    decomposition.gen_additional_circles()

    # 打印外圆超出 OBB 的详细分析报告
    decomposition.print_overhang_report()

    # 执行绘图
    decomposition.plot_decomposition()
