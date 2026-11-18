#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
from vision_msgs.msg import Detection2DArray
import threading
import yaml
import math
import time

class YoloVisionMonitor(Node):
    def __init__(self):
        super().__init__('yolo_vision_monitor')
        # 訂閱 YOLO 的辨識結果 (請確認你們的 Topic 名稱是否為 /yolo/detections)
        self.subscription = self.create_subscription(
            Detection2DArray,
            '/yolo/detections',
            self.yolo_callback,
            10)
        
        self.emergency_stop = False
        self.current_seen_objects = []
        
        # 設定危險黑名單 (遇到這些且靠太近就煞車)
        self.danger_classes = ['person', 'dog', 'cat', 'chair']
        self.image_area = 640 * 480  # 假設 Webcam 解析度

    def yolo_callback(self, msg):
        seen = []
        self.emergency_stop = False

        for detection in msg.detections:
            obj_class = detection.id
            box_area = detection.bbox.size_x * detection.bbox.size_y
            seen.append(obj_class)
            
            # 【視覺避障防線】：如果是危險物，且佔據畫面超過 20% (代表貼在車頭前)
            if obj_class in self.danger_classes and (box_area / self.image_area) > 0.20:
                self.emergency_stop = True
                self.get_logger().warning(f"🚨 視覺警報！前方極近距離出現 {obj_class}！")

        # 更新當前看到的物品清單 (去除重複)
        self.current_seen_objects = list(set(seen))

def euler_to_quaternion(yaw):
    q_z = math.sin(yaw / 2.0)
    q_w = math.cos(yaw / 2.0)
    return q_z, q_w

def main():
    rclpy.init()
    
    # 啟動 YOLO 監聽背景執行緒
    yolo_monitor = YoloVisionMonitor()
    spin_thread = threading.Thread(target=rclpy.spin, args=(yolo_monitor,), daemon=True)
    spin_thread.start()

    navigator = BasicNavigator()
    
    # 讀取語義地圖 (需確保同目錄下有 semantic_map.yaml)
    try:
        with open('semantic_map.yaml', 'r', encoding='utf-8') as f:
            semantic_data = yaml.safe_load(f)
        location_dict = {loc['name']: loc for loc in semantic_data['locations']}
    except FileNotFoundError:
        print("❌ 找不到 semantic_map.yaml")
        return

    print("=== 🚀 YOLO 語義視覺導航系統已連線 ===")
    navigator.waitUntilNav2Active()

    while True:
        print("\n-----------------------------------")
        target_name = input(f"📍 請輸入目的地 {list(location_dict.keys())} (輸入 q 離開): ").strip()
        
        if target_name.lower() == 'q':
            break
        if target_name not in location_dict:
            print("⚠️ 查無此地點。")
            continue

        loc = location_dict[target_name]
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = float(loc['x'])
        goal_pose.pose.position.y = float(loc['y'])
        q_z, q_w = euler_to_quaternion(float(loc['yaw']))
        goal_pose.pose.orientation.z, goal_pose.pose.orientation.w = q_z, q_w

        print(f"出發前往：{target_name}...")
        navigator.goToPose(goal_pose)

        # 導航監控迴圈
        while not navigator.isTaskComplete():
            # 檢查 YOLO 是否觸發緊急煞車
            if yolo_monitor.emergency_stop:
                print("🛑 YOLO 視覺攔截：前方有障礙物，已取消導航！")
                navigator.cancelTask()
                break
            time.sleep(0.5)

        # 任務結束後的語義回報
        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print(f"🎉 成功抵達 {target_name}！")
            time.sleep(1) # 給相機一秒鐘對焦
            print(f"👁️ 視覺回報：我在這裡看到了 {yolo_monitor.current_seen_objects if yolo_monitor.current_seen_objects else '空無一物'}")

    rclpy.shutdown()

if __name__ == '__main__':
    main()