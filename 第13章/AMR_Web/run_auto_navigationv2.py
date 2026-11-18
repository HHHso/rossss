#!/usr/bin/env python3
import os
import signal
import subprocess
import time
import sys
import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap
from tf2_ros import Buffer, TransformListener
import numpy as np
import scipy.ndimage as ndimage

class SmartAutoExplorer(Node):
    def __init__(self):
        super().__init__('smart_auto_explorer')

        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        self.clear_global_costmap = self.create_client(ClearEntireCostmap, '/global_costmap/clear_entirely_global_costmap')
        self.clear_local_costmap = self.create_client(ClearEntireCostmap, '/local_costmap/clear_entirely_local_costmap')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.map_data = None
        self.goal_handle = None
        self.current_goal = None
        self.blacklist = []
        self.consecutive_failures = 0

        # 狀態機設計
        self.state = 'INITIATING'
        self.state_start_time = time.time()
        self.nav_start_time = 0.0

        # 計時器設為 2Hz (0.5秒)，完全不阻塞主執行緒
        self.timer = self.create_timer(0.5, self.control_loop)
        self.get_logger().info("🧠 全新非阻塞防卡死大腦已啟動！等待地圖載入...")

    def map_callback(self, msg):
        self.map_data = msg
        if self.state == 'INITIATING':
            self.get_logger().info("🗺️ 已接收到地圖，開始進行環境掃描...")
            self.set_state('INIT_SPIN')

    def get_robot_pose(self):
        for frame in ['base_footprint', 'base_link']:
            try:
                trans = self.tf_buffer.lookup_transform('map', frame, rclpy.time.Time())
                return trans.transform.translation.x, trans.transform.translation.y
            except Exception:
                continue
        return None, None

    def clear_costmaps(self):
        req = ClearEntireCostmap.Request()
        if self.clear_local_costmap.service_is_ready():
            self.clear_local_costmap.call_async(req)
        if self.clear_global_costmap.service_is_ready():
            self.clear_global_costmap.call_async(req)
        self.get_logger().info("🧹 已清除 Costmap 殘影")

    def set_state(self, new_state):
        self.state = new_state
        self.state_start_time = time.time()

    def find_frontiers(self):
        if self.map_data is None: return []

        w, h = self.map_data.info.width, self.map_data.info.height
        res = self.map_data.info.resolution
        ox, oy = self.map_data.info.origin.position.x, self.map_data.info.origin.position.y

        grid = np.array(self.map_data.data, dtype=np.int8).reshape((h, w))
        free_mask = (grid == 0)
        unknown_mask = (grid == -1)
        obstacle_mask = (grid > 50)

        dilated_unknown = ndimage.binary_dilation(unknown_mask, iterations=1)
        frontier_mask = free_mask & dilated_unknown

        # 障礙物膨脹 35公分，避免找死角
        safety_cells = max(1, int(0.35 / res))
        dilated_obs = ndimage.binary_dilation(obstacle_mask, iterations=safety_cells)
        frontier_mask = frontier_mask & (~dilated_obs)

        labeled, num_features = ndimage.label(frontier_mask)
        frontiers = []

        for i in range(1, num_features + 1):
            y_idx, x_idx = np.where(labeled == i)
            if len(x_idx) >= 12: # 過濾小雜訊
                mean_x, mean_y = np.mean(x_idx), np.mean(y_idx)
                dists = (x_idx - mean_x)**2 + (y_idx - mean_y)**2
                best_idx = np.argmin(dists)
                wx = ox + (x_idx[best_idx] + 0.5) * res
                wy = oy + (y_idx[best_idx] + 0.5) * res
                frontiers.append((wx, wy, len(x_idx)))

        return frontiers

    def control_loop(self):
        now = time.time()

        if self.state == 'INITIATING':
            return 

        elif self.state == 'INIT_SPIN':
            if now - self.state_start_time < 3.5:
                twist = Twist()
                twist.angular.z = 0.5
                self.cmd_vel_pub.publish(twist)
            else:
                self.get_logger().info("✅ 初始掃描完成，開始尋找未知區域...")
                self.set_state('EXPLORE')
            return

        elif self.state == 'RECOVERY_SPIN':
            if now - self.state_start_time < 2.5:
                twist = Twist()
                twist.angular.z = -0.6
                self.cmd_vel_pub.publish(twist)
            else:
                self.get_logger().info("✅ 脫困動作完成，重新尋路...")
                self.set_state('EXPLORE')
            return

        elif self.state == 'EXPLORE':
            if not self.nav_client.wait_for_server(timeout_sec=0.1): return
            robot_x, robot_y = self.get_robot_pose()
            if robot_x is None: return

            frontiers = self.find_frontiers()
            if not frontiers:
                self.get_logger().info("🎉 找不到更多未知邊界，建圖任務完成！")
                return

            best_goal = None
            min_score = float('inf')

            for fx, fy, size in frontiers:
                if any(math.hypot(fx - bx, fy - by) < 0.5 for bx, by in self.blacklist):
                    continue

                dist = math.hypot(fx - robot_x, fy - robot_y)
                if dist < 0.4: continue

                score = dist - (min(size, 60) * 0.05)
                if score < min_score:
                    min_score = score
                    best_goal = (fx, fy)

            if best_goal is None:
                self.get_logger().warn("⚠️ 所有邊界都在黑名單，清空黑名單並強制脫困！")
                self.blacklist.clear()
                self.clear_costmaps()
                self.set_state('RECOVERY_SPIN')
                return

            self.send_goal(best_goal[0], best_goal[1], robot_x, robot_y)

        elif self.state == 'NAVIGATING':
            if now - self.nav_start_time > 20.0:
                self.get_logger().warn("⏰ 導航逾時卡死！強制切換下一個目標。")
                if self.goal_handle:
                    self.goal_handle.cancel_goal_async()
                if self.current_goal:
                    self.blacklist.append(self.current_goal)
                self.clear_costmaps()
                self.set_state('RECOVERY_SPIN')

    def send_goal(self, tx, ty, rx, ry):
        self.nav_start_time = time.time()
        self.current_goal = (tx, ty)

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = tx
        goal_msg.pose.pose.position.y = ty

        yaw = math.atan2(ty - ry, tx - rx)
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.get_logger().info(f"🚀 前往新區域 -> X: {tx:.2f}, Y: {ty:.2f}")
        future = self.nav_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_cb)
        self.set_state('NAVIGATING')

    def goal_response_cb(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn("❌ 目標被 Nav2 拒絕。")
            if self.current_goal: self.blacklist.append(self.current_goal)
            self.set_state('EXPLORE')
            return

        self.goal_handle = handle
        res_future = handle.get_result_async()
        res_future.add_done_callback(self.goal_result_cb)

    def goal_result_cb(self, future):
        if self.state != 'NAVIGATING': return 
        status = future.result().status
        if status == 4:
            self.get_logger().info("✅ 抵達探索點！")
            self.consecutive_failures = 0
            if self.current_goal: self.blacklist.append(self.current_goal)
            self.set_state('EXPLORE')
        else:
            self.consecutive_failures += 1
            if self.current_goal: self.blacklist.append(self.current_goal)
            
            if self.consecutive_failures >= 2:
                self.clear_costmaps()
                self.set_state('RECOVERY_SPIN')
                self.consecutive_failures = 0
            else:
                self.set_state('EXPLORE')

def main():
    print("==================================================")
    print("  ROS 2 Humble - 光達自走車全自動導航建圖系統 (V3) ")
    print("==================================================")

    ros_env = "source /opt/ros/humble/setup.bash && source /home/playrobot/ros2_ws/install/setup.bash"

    print("[1/4] 啟動自走車硬體與雷射里程計...")
    hardware_cmd = f"{ros_env} && ros2 launch /home/playrobot/Desktop/AMR_範例包/第13章/robot_start.launch.py"
    hardware_proc = subprocess.Popen(hardware_cmd, shell=True, executable='/bin/bash', start_new_session=True)
    time.sleep(5)

    print("[2/4] 啟動 SLAM 建圖系統 (等待地圖座標生成)...")
    # 改回你原本最穩定不會報錯的寫法
    slam_cmd = f"{ros_env} && ros2 launch slam_toolbox online_async_launch.py"
    slam_proc = subprocess.Popen(slam_cmd, shell=True, executable='/bin/bash', start_new_session=True)
    time.sleep(8)

    print(">> [系統介入] 發送微動指令逼迫 SLAM 產生 map 座標系...")
    # 解決 Nav2 "frame map does not exist" 當機的核心關鍵
    wake_cmd = f"{ros_env} && ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{{angular: {{z: 0.3}}}}'"
    subprocess.run(wake_cmd, shell=True, executable='/bin/bash', timeout=2)
    time.sleep(4)

    print("[3/4] 啟動 Nav2 導航避障大腦...")
    nav2_params_path = "/home/playrobot/Desktop/AMR_範例包/第14_15章（共用nav）/playrobot_robot_nav2/nav2_params.yaml"
    nav_cmd = f"{ros_env} && ros2 launch nav2_bringup navigation_launch.py params_file:={nav2_params_path}"
    nav_proc = subprocess.Popen(nav_cmd, shell=True, executable='/bin/bash', start_new_session=True)
    time.sleep(6)

    print("[4/4] 啟動 RViz2 (已套用 OpenGL 防報錯機制)...")
    rviz_config_path = "/home/playrobot/Desktop/AMR_範例包/第14_15章（共用nav）/playrobot_robot_nav2/slam_and_nav.rviz"
    
    # 加上 export LIBGL_ALWAYS_SOFTWARE=1 來繞過顯示卡驅動造成的 GLSL 報錯
    rviz_cmd = f"export LIBGL_ALWAYS_SOFTWARE=1 && {ros_env} && ros2 run rviz2 rviz2 -d {rviz_config_path}"
    
    rviz_proc = subprocess.Popen(rviz_cmd, shell=True, executable='/bin/bash', start_new_session=True)
    time.sleep(3)

    print("\n[系統提示] 底層環境已就緒！防卡死自動探索大腦開始運作...")
    rclpy.init()
    explorer_node = SmartAutoExplorer()

    try:
        rclpy.spin(explorer_node)
    except KeyboardInterrupt:
        print("\n[緊急介入] 偵測到 Ctrl+C，正在啟動安全煞車與關機程序...")
    finally:
        try:
            if explorer_node.goal_handle: explorer_node.goal_handle.cancel_goal_async()
            explorer_node.destroy_node()
            rclpy.shutdown()
        except: pass

        for proc in [nav_proc]:
            try: os.killpg(os.getpgid(proc.pid), signal.SIGINT)
            except: pass
        time.sleep(1)

        print(">> 發送強制煞車指令...")
        stop_cmd = f"{ros_env} && ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{{linear: {{x: 0.0, y: 0.0, z: 0.0}}, angular: {{x: 0.0, y: 0.0, z: 0.0}}}}'"
        try: subprocess.run(stop_cmd, shell=True, executable='/bin/bash', timeout=2)
        except: pass

        print(">> 關閉硬體通訊與視覺化節點...")
        for proc in [rviz_proc, slam_proc, hardware_proc]:
            try: os.killpg(os.getpgid(proc.pid), signal.SIGINT)
            except: pass
        time.sleep(2)

        os.system("pkill -f ros2")
        os.system("killall -9 _ros2_daemon 2>/dev/null")
        print("\n✅ 關機完成。")

if __name__ == '__main__':
    main()