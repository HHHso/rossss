#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import speech_recognition as sr
import threading
import time

# 目標點清單 (請依據你的實際地圖座標修改)
WAYPOINTS = {
    "起點":   {"x": 0.0,   "y": 0.0,   "z": 0.0, "w": 1.0},
    "沙發":   {"x": 3.67,  "y": -0.41, "z": 0.0, "w": 1.0},
    "大門":   {"x": 3.73,  "y": -5.38, "z": 0.0, "w": 1.0},
    "路由機": {"x": -1.86, "y": -4.11, "z": 0.0, "w": 1.0}
}

# 全域變數：作為「指令信箱」，用來暫存最新收到的語音或網頁指令
pending_command = None

def match_target(text):
    """比對收到的文字是否包含目標點關鍵字"""
    if "停止" in text or "停" in text or "煞車" in text or "STOP" in text.upper():
        return "STOP"
    for name in WAYPOINTS.keys():
        if name in text:
            return name
    return None

def send_goal(navigator, name):
    """將目標地點封裝成 PoseStamped 格式並發送給 Nav2"""
    coords = WAYPOINTS[name]
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = coords['x']
    goal_pose.pose.position.y = coords['y']
    goal_pose.pose.orientation.z = coords['z']
    goal_pose.pose.orientation.w = coords['w']

    print(f"🚀 出發前往【{name}】 (座標: x={coords['x']}, y={coords['y']})")
    navigator.goToPose(goal_pose)

# ==========================================
# 1. 網路監聽器 (只負責接收網頁 /web_cmd 指令，並放入信箱)
# ==========================================
class WebCmdListener(Node):
    def __init__(self):
        super().__init__('web_cmd_listener')
        self.subscription = self.create_subscription(
            String,
            '/web_cmd',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        global pending_command
        text = msg.data
        print(f"\n🌐 接收到網頁指令: 「{text}」")
        pending_command = text  # 將指令放入信箱，讓主程式去處理

# ==========================================
# 2. 語音辨識執行緒 (在背景無限循環聽麥克風，聽到指令就放入信箱)
# ==========================================
def voice_worker():
    global pending_command
    recognizer = sr.Recognizer()
    mic = sr.Microphone()
    
    with mic as source:
        recognizer.adjust_for_ambient_noise(source, duration=0.5)
        print("\n🎤 麥克風已就緒，請隨時說出指令（或點擊網頁按鈕）...")

    while rclpy.ok():
        try:
            with mic as source:
                # 設定短時間的 timeout，避免程式卡死在監聽狀態
                audio = recognizer.listen(source, timeout=1.0, phrase_time_limit=4)
            text = recognizer.recognize_google(audio, language="zh-TW")
            if text:
                print(f"\n🗣 實體語音辨識: 「{text}」")
                pending_command = text  # 將指令放入信箱，讓主程式去處理
        except sr.WaitTimeoutError:
            continue
        except sr.UnknownValueError:
            continue
        except Exception:
            continue

# ==========================================
# 3. 主程式 (指揮中心：專門負責檢查信箱，並指揮 Nav2 導航)
# ==========================================
def main():
    global pending_command
    rclpy.init()
    
    navigator = BasicNavigator()
    print("⏳ 等待 Nav2 導航大腦連線...")
    navigator.waitUntilNav2Active()
    print("✅ Nav2 連線成功，語音/網頁雙控系統準備就緒！")

    # 啟動網頁監聽節點
    cmd_node = WebCmdListener()

    # 啟動背景語音執行緒
    voice_thread = threading.Thread(target=voice_worker)
    voice_thread.daemon = True
    voice_thread.start()

    try:
        # 主迴圈：不斷檢查「指令信箱」
        while rclpy.ok():
            # 讓 ROS 2 處理一次網頁訊息 (最多等 0.1 秒)
            rclpy.spin_once(cmd_node, timeout_sec=0.1)

            # 若信箱內有新的指令，則取出執行，避免執行緒打架
            if pending_command:
                command = pending_command
                target = match_target(command)
                pending_command = None  # 取出後立刻清空信箱

                if target == "STOP":
                    print("🛑 收到緊急停止指令！取消任務...")
                    navigator.cancelTask()
                elif target:
                    send_goal(navigator, target)
                else:
                    print(f"⚠️️ 指令未包含已知地點，或聽不懂: {command}")

    except KeyboardInterrupt:
        print("\n使用者中斷，關閉導航程式。")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()