#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav2_msgs.action import NavigateToPose
from tf2_ros import Buffer, TransformListener
import math
import time
import sys
import csv
import os

class AlgorithmBenchmark(Node):
    def __init__(self, mode, target_x, target_y):
        super().__init__('algorithm_benchmark')
        self.mode = mode  # 'nav2' 或 'naive'
        self.target_x = target_x
        self.target_y = target_y

        # 通訊介面
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # TF 定位監聽
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 雷達環境感知變數
        self.front_dist = 99.9
        self.left_dist = 99.9
        self.right_dist = 99.9

        # 實驗數據與軌跡紀錄變數
        self.start_time = None
        self.start_x = None
        self.start_y = None
        self.last_x = None
        self.last_y = None
        self.last_yaw = None
        self.total_distance = 0.0
        self.rotation_effort = 0.0
        self.is_running = False
        self.is_finished = False

        # Nav2 控制柄 (用於抵達時強制取消任務)
        self.nav2_goal_sent = False
        self.nav2_goal_handle = None

        # 軌跡清單：[經過秒數, x, y, yaw, 目標x, 目標y]
        self.trajectory_log = []

        # 10Hz 主迴圈
        self.timer = self.create_timer(0.1, self.UpdateLoop)
        self.get_logger().info(f"📊 演算法評測啟動！當前模式: [{self.mode.upper()}] -> 目標點: ({target_x}, {target_y})")

    def scan_callback(self, msg):
        ranges = [r if (not math.isinf(r) and not math.isnan(r) and r > 0.05) else 10.0 for r in msg.ranges]
        n = len(ranges)
        if n == 0:
            return

        front_idx = int(n * 0.08)
        front_cone = ranges[:front_idx] + ranges[-front_idx:]
        left_cone = ranges[front_idx : int(n * 0.25)]
        right_cone = ranges[int(n * 0.75) : -front_idx]

        self.front_dist = min(front_cone) if front_cone else 10.0
        self.left_dist = min(left_cone) if left_cone else 10.0
        self.right_dist = min(right_cone) if right_cone else 10.0

    def get_robot_pose(self):
        for frame in ['base_footprint', 'base_link']:
            try:
                t = self.tf_buffer.lookup_transform('map', frame, rclpy.time.Time())
                x = t.transform.translation.x
                y = t.transform.translation.y
                q = t.transform.rotation
                siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
                cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
                yaw = math.atan2(siny_cosp, cosy_cosp)
                return x, y, yaw
            except Exception:
                continue
        return None, None, None

    def emergency_stop_robot(self):
        """強制煞車機制：取消 Nav2 並連續發送 1 秒鐘的歸零速度，確保硬體底盤完全靜止"""
        self.get_logger().info("🛑 觸發強制煞車機制！正在鎖死馬達...")

        # 1. 若為 Nav2 模式，先取消導航任務，防止 Nav2 繼續發送速度
        if self.mode == 'nav2' and self.nav2_goal_handle is not None:
            try:
                self.nav2_goal_handle.cancel_goal_async()
            except Exception:
                pass

        # 2. 連續發送 15 次速度歸零訊號 (持續約 1.5 秒)，確保不會因為掉包而繼續滑行
        stop_twist = Twist()
        stop_twist.linear.x = 0.0
        stop_twist.linear.y = 0.0
        stop_twist.linear.z = 0.0
        stop_twist.angular.x = 0.0
        stop_twist.angular.y = 0.0
        stop_twist.angular.z = 0.0

        for _ in range(15):
            self.cmd_vel_pub.publish(stop_twist)
            time.sleep(0.1)

    def UpdateLoop(self):
        if self.is_finished:
            return

        rx, ry, ryaw = self.get_robot_pose()
        if rx is None:
            return

        # 初始化起點與計時器
        if not self.is_running:
            self.start_time = time.time()
            self.start_x, self.start_y = rx, ry
            self.last_x, self.last_y, self.last_yaw = rx, ry, ryaw
            self.is_running = True
            self.get_logger().info(f"🟢 定位成功！起點座標: ({rx:.3f}, {ry:.3f})，開始計時與記錄軌跡！")

        now_elapsed = time.time() - self.start_time

        # 1. 累計行駛距離與轉向消耗
        step_dist = math.hypot(rx - self.last_x, ry - self.last_y)
        self.total_distance += step_dist
        
        angle_diff = abs(self.normalize_angle(ryaw - self.last_yaw))
        self.rotation_effort += angle_diff
        self.last_x, self.last_y, self.last_yaw = rx, ry, ryaw

        # 記錄當前時間點的軌跡座標
        self.trajectory_log.append([
            round(now_elapsed, 2),
            round(rx, 4),
            round(ry, 4),
            round(ryaw, 4),
            self.target_x,
            self.target_y
        ])

        # 2. 檢查是否抵達終點 B (距離小於 0.22 公尺判定為抵達)
        dist_to_goal = math.hypot(self.target_x - rx, self.target_y - ry)
        if dist_to_goal < 0.22:
            self.finish_benchmark(success=True, final_x=rx, final_y=ry, final_err=dist_to_goal)
            return

        # 超時保護 (超過 120 秒視為卡死失敗)
        if now_elapsed > 120.0:
            self.finish_benchmark(success=False, final_x=rx, final_y=ry, final_err=dist_to_goal)
            return

        # 3. 根據選擇的演算法執行移動
        if self.mode == 'nav2':
            self.run_nav2_algorithm()
        elif self.mode == 'naive':
            self.run_naive_straight_obstacle_avoidance(rx, ry, ryaw)

    def run_nav2_algorithm(self):
        if not self.nav2_goal_sent:
            if not self.nav_client.wait_for_server(timeout_sec=1.0):
                self.get_logger().warn("等待 Nav2 伺服器中...")
                return
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose.header.frame_id = 'map'
            goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
            goal_msg.pose.pose.position.x = self.target_x
            goal_msg.pose.pose.position.y = self.target_y
            goal_msg.pose.pose.orientation.w = 1.0

            send_future = self.nav_client.send_goal_async(goal_msg)
            send_future.add_done_callback(self.nav2_goal_response_cb)
            self.nav2_goal_sent = True
            self.get_logger().info("🚀 已將目標點交由 Nav2 進行全局路徑規劃！")

    def nav2_goal_response_cb(self, future):
        try:
            self.nav2_goal_handle = future.result()
        except Exception:
            pass

    def run_naive_straight_obstacle_avoidance(self, rx, ry, ryaw):
        twist = Twist()
        goal_yaw = math.atan2(self.target_y - ry, self.target_x - rx)
        yaw_error = self.normalize_angle(goal_yaw - ryaw)

        safe_distance = 0.45

        if self.front_dist < safe_distance:
            twist.linear.x = 0.0
            if self.left_dist > self.right_dist:
                twist.angular.z = 0.5
            else:
                twist.angular.z = -0.5
            self.get_logger().info(f"⚠️ [初階避障觸發] 前方障礙物 ({self.front_dist:.2f}m)！強制轉向閃避中...", throttle_duration_sec=1.0)
        elif self.left_dist < 0.30:
            twist.linear.x = 0.08
            twist.angular.z = -0.35
        elif self.right_dist < 0.30:
            twist.linear.x = 0.08
            twist.angular.z = 0.35
        else:
            if abs(yaw_error) > 0.35:
                twist.linear.x = 0.0
                twist.angular.z = 0.45 if yaw_error > 0 else -0.45
            else:
                twist.linear.x = 0.18
                twist.angular.z = 0.8 * yaw_error

        self.cmd_vel_pub.publish(twist)

    def save_data_to_csv(self, success, elapsed_time, avg_speed, straight_dist, final_x, final_y, final_err):
        """將軌跡座標與統計結果匯出成 CSV 檔案"""
        base_dir = os.path.dirname(os.path.abspath(__file__))
        traj_filename = os.path.join(base_dir, f"trajectory_{self.mode}.csv")
        summary_filename = os.path.join(base_dir, "benchmark_summary.csv")

        # 1. 儲存詳細軌跡點 (供畫圖對比使用)
        with open(traj_filename, mode='w', newline='', encoding='utf-8') as f:
            writer = csv.writer(f)
            writer.writerow(['time_sec', 'x', 'y', 'yaw', 'target_x', 'target_y'])
            writer.writerows(self.trajectory_log)

        # 2. 追加儲存實驗總表
        file_exists = os.path.isfile(summary_filename)
        with open(summary_filename, mode='a', newline='', encoding='utf-8') as f:
            writer = csv.writer(f)
            if not file_exists:
                writer.writerow([
                    '模式(Mode)', '狀態(Success)', '花費時間(s)', '實際總距離(m)',
                    '直線最短距離(m)', '路徑多繞比例(%)', '平均速度(m/s)',
                    '轉向消耗(rad)', '起點(X,Y)', '目標點(X,Y)', '最終停妥點(X,Y)', '終點誤差(m)'
                ])
            detour_ratio = ((self.total_distance - straight_dist) / straight_dist * 100.0) if straight_dist > 0 else 0.0
            writer.writerow([
                self.mode.upper(),
                'SUCCESS' if success else 'FAILED',
                f"{elapsed_time:.2f}",
                f"{self.total_distance:.3f}",
                f"{straight_dist:.3f}",
                f"{detour_ratio:.1f}%",
                f"{avg_speed:.3f}",
                f"{self.rotation_effort:.2f}",
                f"({self.start_x:.2f}, {self.start_y:.2f})",
                f"({self.target_x:.2f}, {self.target_y:.2f})",
                f"({final_x:.2f}, {final_y:.2f})",
                f"{final_err:.3f}"
            ])

        return traj_filename, summary_filename

    def finish_benchmark(self, success, final_x, final_y, final_err):
        self.is_finished = True
        elapsed_time = time.time() - self.start_time
        avg_speed = self.total_distance / elapsed_time if elapsed_time > 0 else 0.0
        straight_dist = math.hypot(self.target_x - self.start_x, self.target_y - self.start_y)

        # 執行強制連發煞車
        self.emergency_stop_robot()

        # 煞車後再讀取一次最終靜止座標
        stop_x, stop_y, _ = self.get_robot_pose()
        if stop_x is not None:
            final_x, final_y = stop_x, stop_y
            final_err = math.hypot(self.target_x - final_x, self.target_y - final_y)

        # 儲存 CSV
        traj_file, sum_file = self.save_data_to_csv(
            success, elapsed_time, avg_speed, straight_dist, final_x, final_y, final_err
        )

        print("\n" + "="*60)
        print(f"🏁 演算法實驗結果報表 - 模式：【{self.mode.upper()}】")
        print("="*60)
        print(f"📌 任務狀態     : {'✅ 成功抵達並煞停' if success else '❌ 失敗 (超時或卡死)'}")
        print(f"📍 起點座標     : ({self.start_x:.3f}, {self.start_y:.3f})")
        print(f"🎯 目標點座標   : ({self.target_x:.3f}, {self.target_y:.3f}) [直線距離: {straight_dist:.3f} m]")
        print(f"🛑 最終停妥座標 : ({final_x:.3f}, {final_y:.3f}) [距離目標誤差: {final_err:.3f} m]")
        print("-" * 60)
        print(f"⏱️ 總花費時間   : {elapsed_time:.2f} 秒")
        print(f"📏 實際行駛距離 : {self.total_distance:.3f} 公尺")
        print(f"🚀 平均移動速度 : {avg_speed:.3f} m/s")
        print(f"🔄 累積轉向消耗 : {self.rotation_effort:.2f} rad (數值越小代表路徑越平滑)")
        print("-" * 60)
        print(f"💾 軌跡座標已匯出至 : {traj_file}")
        print(f"📊 統計總表已更新至 : {sum_file}")
        print("="*60 + "\n")

        self.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

    @staticmethod
    def normalize_angle(angle):
        while angle > math.pi: angle -= 2.0 * math.pi
        while angle < -math.pi: angle += 2.0 * math.pi
        return angle


def main():
    print("==================================================")
    print("   AMR 演算法效能與軌跡對比評測系統 (含煞車與匯出)  ")
    print("==================================================")
    print("請選擇要測試的演算法模式：")
    print("  [1] 我們原本的演算法 (Nav2 全局路徑規劃 + 局部避障)")
    print("  [2] 對照組初階演算法 (直線朝目標前進 + 雷達反應式避障)")
    choice = input("請輸入數字 (1 或 2): ").strip()

    mode = 'nav2' if choice == '1' else 'naive'

    target_x = float(input("請輸入目標點 B 的 X 座標 (公尺，例如 -0.5): "))
    target_y = float(input("請輸入目標點 B 的 Y 座標 (公尺，例如 -0.5): "))

    rclpy.init()
    node = AlgorithmBenchmark(mode, target_x, target_y)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n🚨 實驗手動中斷！正在緊急煞車...")
        node.emergency_stop_robot()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()