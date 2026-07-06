import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
import math

class VehicleCircleDecomposition:
    def __init__(self):
        # 车辆基础尺寸参数 (单位: 米)
        self.vehicle_length = 5.0    # 车长
        self.vehicle_width = 2.0     # 车宽
        self.vehicle_lr = 1.0        # 后轴到后保险杠的距离 (根据C++代码推断)
        
        # 分解配置
        self.inner_row_num = 2
        self.outer_row_num = 4
        self.keep_outer_circle_all = False
        
        # 存储计算出的圆心和半径 [(x, y, r, type_name), ...]
        self.inner_circles = []
        self.outer_circles = []
        self.additional_circles = []

    def decompose_inner_circles(self):
        """复刻 C++ 的 decomposeEgoToInnerCircles 逻辑"""
        column_num = math.ceil(self.vehicle_length / self.vehicle_width * self.inner_row_num)
        
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
        """复刻 C++ 的 decomposeEgoToOuterCircles 逻辑"""
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
                    if (i != 0) and (i != column_num - 1) and (j != 0) and (j != self.outer_row_num - 1):
                        continue
                cx = base_x + i * chord_col
                cy = base_y - j * chord_row
                self.outer_circles.append((cx, cy, radius, "Outer"))

    def gen_additional_circles(self):
        """复刻 C++ 的 genAdditionalCircles 逻辑"""
        # 四个角点圆
        front_x = self.vehicle_length - self.vehicle_lr
        rear_x = -self.vehicle_lr
        left_y = self.vehicle_width * 0.5
        right_y = -self.vehicle_width * 0.5
        
        vertices = [
            (front_x, left_y, 0.26, "Front Corner"),  # 左前 LF
            (front_x, right_y, 0.26, "Front Corner"), # 右前 RF
            (rear_x, left_y, 0.25, "Rear Corner"),    # 左后 LR
            (rear_x, right_y, 0.25, "Rear Corner")    # 右后 RR
        ]
        self.additional_circles.extend(vertices)
        
        # 前保险杠正中间圆
        self.additional_circles.append((front_x, 0, 0.31, "Front Bump"))
        # 后保险杠正中间圆
        self.additional_circles.append((rear_x, 0, 0.25, "Rear Bump"))
        # 后轴两端圆 (后轴中心在0,0)
        self.additional_circles.append((0, left_y, 0.25, "Rear Axle"))
        self.additional_circles.append((0, right_y, 0.25, "Rear Axle"))

    def plot_decomposition(self):
        """绘制车辆轮廓与分解圆"""
        fig, ax = plt.subplots(figsize=(12, 6))
        
        # 1. 绘制车辆矩形轮廓 (黑色实线)
        rect_x = -self.vehicle_lr
        rect_y = -self.vehicle_width / 2
        car_rect = patches.Rectangle((rect_x, rect_y), self.vehicle_length, self.vehicle_width, 
                                     linewidth=2, edgecolor='black', facecolor='none', zorder=5, label='Vehicle Footprint')
        ax.add_patch(car_rect)
        
        # 2. 标出后轴中心
        ax.plot(0, 0, 'k+', markersize=15, markeredgewidth=2, zorder=10, label='Rear Axle Center')

        # 3. 绘制外圆 (浅绿色，虚线边缘，偏保守外扩覆盖)
        for i, (cx, cy, r, _) in enumerate(self.outer_circles):
            label = 'Outer Circles (Edge)' if i == 0 else ""
            circle = patches.Circle((cx, cy), r, color='green', alpha=0.15, ec='green', ls='--', lw=1.5, zorder=1, label=label)
            ax.add_patch(circle)

        # 4. 绘制内圆 (蓝色，实线边缘，偏内缩覆盖)
        for i, (cx, cy, r, _) in enumerate(self.inner_circles):
            label = 'Inner Circles' if i == 0 else ""
            circle = patches.Circle((cx, cy), r, color='blue', alpha=0.25, ec='blue', lw=1.5, zorder=2, label=label)
            ax.add_patch(circle)

        # 5. 绘制额外圆 (红色，实线边缘，用于补角和保险杠)
        for i, (cx, cy, r, ctype) in enumerate(self.additional_circles):
            label = 'Additional Circles' if i == 0 else ""
            circle = patches.Circle((cx, cy), r, color='red', alpha=0.4, ec='darkred', lw=1.5, zorder=3, label=label)
            ax.add_patch(circle)
            # 在额外圆中心打个小点并标注类型(可选)
            ax.plot(cx, cy, 'r.', markersize=5, zorder=4)

        # 视图设置
        ax.set_aspect('equal')
        ax.set_xlim(-self.vehicle_lr - 1.0, self.vehicle_length - self.vehicle_lr + 1.0)
        ax.set_ylim(-self.vehicle_width / 2 - 1.0, self.vehicle_width / 2 + 1.0)
        ax.set_xlabel('X (m) - Forward')
        ax.set_ylabel('Y (m) - Lateral')
        ax.set_title(f'Vehicle Circle Decomposition\n(Length={self.vehicle_length}m, Width={self.vehicle_width}m, LR={self.vehicle_lr}m)', fontsize=14)
        
        # 整理图例以避免重复
        handles, labels = ax.get_legend_handles_labels()
        by_label = dict(zip(labels, handles))
        ax.legend(by_label.values(), by_label.keys(), loc='upper right', bbox_to_anchor=(1.35, 1))
        
        plt.grid(True, linestyle=':', alpha=0.6)
        plt.tight_layout()
        
        # 保存图片到本地
        plt.savefig('circle_decomposition_plot.png', dpi=300, bbox_inches='tight')
        print("示意图已生成并保存为 'circle_decomposition_plot.png'")
        plt.show()

if __name__ == "__main__":
    decomposition = VehicleCircleDecomposition()
    
    # 按照C++逻辑进行分解
    decomposition.decompose_inner_circles()
    decomposition.decompose_outer_circles()
    decomposition.gen_additional_circles()
    
    # 执行绘图
    decomposition.plot_decomposition()