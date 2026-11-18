#!/usr/bin/env python3
import os
import csv
import math
import matplotlib.pyplot as plt

def load_trajectory(filepath):
    times, xs, ys, yaws = [], [], [], []
    target_x, target_y = -0.5, -0.5
    if not os.path.exists(filepath):
        return None
    with open(filepath, 'r', encoding='utf-8') as f:
        reader = csv.DictReader(f)
        for row in reader:
            times.append(float(row['time_sec']))
            xs.append(float(row['x']))
            ys.append(float(row['y']))
            yaws.append(float(row['yaw']))
            target_x = float(row['target_x'])
            target_y = float(row['target_y'])
    return {'time': times, 'x': xs, 'y': ys, 'yaw': yaws, 'tx': target_x, 'ty': target_y}

def calc_cumulative_dist(xs, ys):
    cum_dist = [0.0]
    total = 0.0
    for i in range(1, len(xs)):
        total += math.hypot(xs[i] - xs[i-1], ys[i] - ys[i-1])
        cum_dist.append(total)
    return cum_dist

def main():
    base_dir = os.path.dirname(os.path.abspath(__file__))
    nav2_file = os.path.join(base_dir, "trajectory_nav2.csv")
    naive_file = os.path.join(base_dir, "trajectory_naive.csv")

    nav2_data = load_trajectory(nav2_file)
    naive_data = load_trajectory(naive_file)

    if not nav2_data or not naive_data:
        print("❌ 請確認 trajectory_nav2.csv 與 trajectory_naive.csv 皆存在於同目錄下！")
        return

    fig, axs = plt.subplots(2, 2, figsize=(15, 11))
    fig.suptitle("Autonomous Mobile Robot (AMR) Navigation Algorithm Benchmark", fontsize=16, fontweight='bold')

    # --- 1. 左上圖：2D 空間軌跡對比圖 (各自標示起點與理想直線) ---
    ax1 = axs[0, 0]
    ax1.set_title("1. 2D Spatial Trajectory Comparison", fontsize=13, fontweight='bold')
    ax1.set_xlabel("X Position (m)", fontsize=11)
    ax1.set_ylabel("Y Position (m)", fontsize=11)
    ax1.grid(True, linestyle='--', alpha=0.6)
    ax1.set_aspect('equal', adjustable='datalim')

    # 畫 Naive 軌跡與理想直線
    ax1.plot([naive_data['x'][0], naive_data['tx']], [naive_data['y'][0], naive_data['ty']],
             color='#e74c3c', linestyle=':', alpha=0.5, label="Naive Ideal Line (0.83m)")
    ax1.plot(naive_data['x'], naive_data['y'], color='#e74c3c', linewidth=2.0, label="Baseline (Naive): 3.10m")
    ax1.scatter(naive_data['x'][0], naive_data['y'][0], color='#c0392b', marker='o', s=90, zorder=5, label="Naive Start")
    ax1.scatter(naive_data['x'][-1], naive_data['y'][-1], color='#c0392b', marker='X', s=100, zorder=5)

    # 畫 Nav2 軌跡與理想直線
    ax1.plot([nav2_data['x'][0], nav2_data['tx']], [nav2_data['y'][0], nav2_data['ty']],
             color='#2ecc71', linestyle=':', alpha=0.6, label="Nav2 Ideal Line (1.31m)")
    ax1.plot(nav2_data['x'], nav2_data['y'], color='#27ae60', linewidth=2.8, label="Proposed (Nav2): 1.46m")
    ax1.scatter(nav2_data['x'][0], nav2_data['y'][0], color='#1e8449', marker='s', s=90, zorder=5, label="Nav2 Start")
    ax1.scatter(nav2_data['x'][-1], nav2_data['y'][-1], color='#1e8449', marker='P', s=100, zorder=5)

    # 畫目標點與 0.22m 抵達判定圈
    tx, ty = nav2_data['tx'], nav2_data['ty']
    ax1.scatter(tx, ty, color='gold', edgecolor='black', marker='*', s=260, zorder=6, label="Target B (-0.5, -0.5)")
    circle = plt.Circle((tx, ty), 0.22, color='gold', alpha=0.2, label="Stop Threshold (0.22m)")
    ax1.add_patch(circle)
    ax1.legend(loc='best', fontsize=9)

    # --- 2. 右上圖：距離目標點之收斂曲線 ---
    ax2 = axs[0, 1]
    ax2.set_title("2. Distance to Target vs. Time (Convergence)", fontsize=13, fontweight='bold')
    ax2.set_xlabel("Time (s)", fontsize=11)
    ax2.set_ylabel("Distance to Target B (m)", fontsize=11)
    ax2.grid(True, linestyle='--', alpha=0.6)

    rem_naive = [math.hypot(naive_data['tx'] - x, naive_data['ty'] - y) for x, y in zip(naive_data['x'], naive_data['y'])]
    rem_nav2 = [math.hypot(nav2_data['tx'] - x, nav2_data['ty'] - y) for x, y in zip(nav2_data['x'], nav2_data['y'])]

    ax2.plot(naive_data['time'], rem_naive, color='#e74c3c', linewidth=2.2, label="Baseline (Naive) - 70.3s")
    ax2.plot(nav2_data['time'], rem_nav2, color='#27ae60', linewidth=2.5, label="Proposed (Nav2) - 24.9s")
    ax2.axhline(0.22, color='gray', linestyle='--', linewidth=1.2, label="Brake Threshold (0.22m)")
    ax2.legend(loc='best')

    # --- 3. 左下圖：累積行駛距離隨時間變化曲線 ---
    ax3 = axs[1, 0]
    ax3.set_title("3. Cumulative Traveled Distance Over Time", fontsize=13, fontweight='bold')
    ax3.set_xlabel("Time (s)", fontsize=11)
    ax3.set_ylabel("Cumulative Distance (m)", fontsize=11)
    ax3.grid(True, linestyle='--', alpha=0.6)

    cum_naive = calc_cumulative_dist(naive_data['x'], naive_data['y'])
    cum_nav2 = calc_cumulative_dist(nav2_data['x'], nav2_data['y'])

    ax3.plot(naive_data['time'], cum_naive, color='#e74c3c', linewidth=2.2, label=f"Baseline (Naive): {cum_naive[-1]:.2f} m")
    ax3.plot(nav2_data['time'], cum_nav2, color='#27ae60', linewidth=2.5, label=f"Proposed (Nav2): {cum_nav2[-1]:.2f} m")
    ax3.legend(loc='best')

    # --- 4. 右下圖：關鍵效能指標百分比/數值長條圖 ---
    ax4 = axs[1, 1]
    ax4.set_title("4. Key Performance Metrics Comparison", fontsize=13, fontweight='bold')
    metrics = ['Time (s)', 'Total Dist (m)', 'Rot Effort (rad)', 'Detour Ratio (%)']
    naive_vals = [70.30, 3.10, 18.61, 272.9]
    nav2_vals = [24.90, 1.46, 6.27, 11.7]

    x_pos = range(len(metrics))
    width = 0.35
    bars1 = ax4.bar([p - width/2 for p in x_pos], naive_vals, width=width, color='#e74c3c', label='Baseline (Naive)')
    bars2 = ax4.bar([p + width/2 for p in x_pos], nav2_vals, width=width, color='#27ae60', label='Proposed (Nav2)')

    ax4.set_xticks(list(x_pos))
    ax4.set_xticklabels(metrics, fontsize=10, fontweight='bold')
    ax4.grid(True, axis='y', linestyle='--', alpha=0.6)
    ax4.legend()

    # 在長條圖上方標註數值
    for bar in bars1 + bars2:
        yval = bar.get_height()
        ax4.text(bar.get_x() + bar.get_width()/2.0, yval + 3, f'{yval:.1f}', ha='center', va='bottom', fontsize=9)
    ax4.set_ylim(0, 310)

    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    out_img = os.path.join(base_dir, "algorithm_comparison_report.png")
    plt.savefig(out_img, dpi=300)
    print(f"✅ 四合一專題報告圖表已成功生成：{out_img}")
    plt.show()

if __name__ == '__main__':
    main()