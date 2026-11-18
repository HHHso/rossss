#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from flask import Flask, render_template
from flask_socketio import SocketIO, emit
import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
import threading
import time
import math
import yaml
import os
from PIL import Image

app = Flask(__name__)
socketio = SocketIO(app, cors_allowed_origins="*")

navigator = None
nav_active = False

# ==========================================
# 🗺️ 全自動地圖參數讀取 (Auto-load from YAML & Image)
# ==========================================
MAP_YAML_PATH = 'map.yaml'
MAP_IMAGE_PATH = 'static/ocauseco.png'

try:
    # 自動讀取 map.yaml 中的 resolution 與 origin
    with open(MAP_YAML_PATH, 'r') as f:
        map_data = yaml.safe_load(f)
        MAP_RESOLUTION = float(map_data['resolution'])
        MAP_ORIGIN_X = float(map_data['origin'][0])
        MAP_ORIGIN_Y = float(map_data['origin'][1])
    
    # 自動讀取圖片的原始像素大小
    with Image.open(MAP_IMAGE_PATH) as img:
        MAP_WIDTH_PX, MAP_HEIGHT_PX = img.size

    print(f"🗺️ 地圖自動載入成功！")
    print(f"   - 圖片尺寸: {MAP_WIDTH_PX}x{MAP_HEIGHT_PX}")
    print(f"   - 解析度: {MAP_RESOLUTION}, 原點: ({MAP_ORIGIN_X}, {MAP_ORIGIN_Y})")
except Exception as e:
    print(f"❌ 地圖資料讀取失敗，請確認 map.yaml 與 static/ocauseco.png 是否存在。錯誤: {e}")

# ==========================================

def init_ros2():
    global navigator
    rclpy.init()
    navigator = BasicNavigator()
    print("⏳ 等待 Nav2 導航大腦連線...")
    navigator.waitUntilNav2Active()
    print("✅ Nav2 連線成功！")
    while rclpy.ok():
        rclpy.spin_once(navigator, timeout_sec=0.1)

def pixel_ratio_to_meters(rx, ry):
    """將網頁點擊的比例 (0~1) 轉換為 ROS 2 真實世界座標 (公尺)"""
    # 1. 將比例還原成真實像素座標
    pixel_x = rx * MAP_WIDTH_PX
    pixel_y = ry * MAP_HEIGHT_PX
    
    # 2. 轉換為 ROS 座標 (圖片 Y 軸向下，ROS Y 軸向上，因此 Y 軸需反轉計算)
    ros_x = (pixel_x * MAP_RESOLUTION) + MAP_ORIGIN_X
    ros_y = ((MAP_HEIGHT_PX - pixel_y) * MAP_RESOLUTION) + MAP_ORIGIN_Y
    return ros_x, ros_y

def run_navigation_task(waypoints_data, algorithm):
    global nav_active
    nav_active = True
    start_time = time.time()
    
    # TODO: 這裡未來可透過 ROS 2 參數服務 (Parameter Service) 切換 A* 或 Dijkstra
    print(f"⚙️ 準備使用演算法: {algorithm}")

    poses = []
    for wp in waypoints_data:
        x, y = pixel_ratio_to_meters(wp['rx'], wp['ry'])
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = navigator.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.w = 1.0 # 預設抵達時車頭朝向
        poses.append(pose)
        print(f"📍 轉換座標 [{wp['name']}]: 像素比例({wp['rx']:.2f}, {wp['ry']:.2f}) -> ROS({x:.2f}, {y:.2f})")
    
    # 執行 A -> B -> C 連續導航
    navigator.goThroughPoses(poses)

    # 監控導航進度與計時器
    while not navigator.isTaskComplete() and nav_active:
        elapsed = int(time.time() - start_time)
        mins, secs = divmod(elapsed, 60)
        time_str = f"{mins:02d}分{secs:02d}秒"
        
        feedback = navigator.getFeedback()
        if feedback:
            current_target = waypoints_data[feedback.current_waypoint]['name']
            progress_str = f"前往 {current_target}"
        else:
            progress_str = "規劃中..."

        socketio.emit('nav_status', {'timer': time_str, 'progress': progress_str})
        time.sleep(1.0)

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        socketio.emit('nav_status', {'progress': "✅ 完成導航"})
        print("🎉 導航任務圓滿完成")
    elif result == TaskResult.CANCELED:
        socketio.emit('nav_status', {'progress': "🛑 導航已終止"})
        print("🛑 導航任務已取消")

    nav_active = False

@app.route('/')
def index():
    return render_template('index.html')

@socketio.on('start_nav')
def handle_start_nav(data):
    waypoints = data.get('waypoints', [])
    algo = data.get('algorithm', 'astar')
    
    if waypoints and navigator:
        threading.Thread(target=run_navigation_task, args=(waypoints, algo), daemon=True).start()

@socketio.on('stop_nav')
def handle_stop_nav():
    global nav_active
    nav_active = False
    if navigator:
        navigator.cancelTask()

if __name__ == '__main__':
    ros2_thread = threading.Thread(target=init_ros2, daemon=True)
    ros2_thread.start()
    socketio.run(app, host='0.0.0.0', port=5000)