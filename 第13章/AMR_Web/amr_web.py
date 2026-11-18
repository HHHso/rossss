#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import math
import os
import shlex
import subprocess
import time

from flask import Flask, render_template_string, jsonify

app = Flask(__name__)

HOME = os.path.expanduser("~")
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
ROS_SETUP = "source /opt/ros/humble/setup.bash"
AMR_DIR = os.path.join(HOME, "Desktop", "AMR_範例包")

# 車子「起點」在 map 座標系的位置：(x [m], y [m], yaw [rad])
# 取得方式：在 RViz2 手動 2D Pose Estimate 一次，對齊後執行
#   ros2 topic echo /amcl_pose --once
# 讀 position.x / position.y，yaw = 2 * atan2(orientation.z, orientation.w)
START_POSE = (0.0, 0.0, 0.0)

HTML_TEMPLATE = """
<!DOCTYPE html>
<html lang="zh-TW">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>AMR 自走車控制中心</title>
    <style>
        body { font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif; background-color: #121212; color: #ffffff; padding: 20px; max-width: 800px; margin: 0 auto; }
        h1 { text-align: center; color: #4CAF50; border-bottom: 2px solid #333; padding-bottom: 10px; }
        .grid-container { display: grid; grid-template-columns: 1fr 1fr; gap: 20px; margin-top: 20px; }
        .panel { background-color: #1e1e1e; padding: 20px; border-radius: 10px; box-shadow: 0 4px 6px rgba(0,0,0,0.3); }
        h2 { font-size: 18px; color: #aaaaaa; margin-top: 0; margin-bottom: 15px; }
        button { width: 100%; padding: 12px; margin-bottom: 10px; font-size: 16px; font-weight: bold; border: none; border-radius: 5px; cursor: pointer; transition: 0.2s; }

        .btn-start { background-color: #2196F3; color: white; }
        .btn-start:hover { background-color: #1976D2; }

        .btn-init { background-color: #9C27B0; color: white; }
        .btn-init:hover { background-color: #7B1FA2; }

        .btn-align { background-color: #009688; color: white; }
        .btn-align:hover { background-color: #00796B; }

        .btn-nav { background-color: #FF9800; color: white; }
        .btn-nav:hover { background-color: #F57C00; }

        .btn-stop { background-color: #F44336; color: white; grid-column: span 2; margin-top: 10px; }
        .btn-stop:hover { background-color: #D32F2F; }

        #console { background-color: #000; color: #00FF00; padding: 15px; border-radius: 5px; font-family: monospace; margin-top: 20px; min-height: 50px; }
        .hint-text { font-size: 14px; color: #ffeb3b; margin-bottom: 10px; line-height: 1.5; }
    </style>
</head>
<body>

    <h1>🤖 AMR 自走車控制中心</h1>

    <div class="grid-container">
        <!-- 系統啟動區 -->
        <div class="panel">
            <h2>步驟一：系統模組啟動</h2>
            <button class="btn-start" onclick="sendCommand('/start/nav')">1. 啟動 Nav2 導航大腦</button>
            <button class="btn-start" onclick="sendCommand('/start/voice')">2. 啟動語音辨識系統</button>
            <button class="btn-start" onclick="sendCommand('/start/yolo')">3. 啟動 YOLOv8 視覺</button>
        </div>

        <!-- 定位與導航區 -->
        <div class="panel">
            <h2>步驟二：全自動定位與導航指令</h2>

            <div class="hint-text">
                💡 <b>免手動定位提示：</b><br>
                1. 車子停在起點 → 按「回到起點自動定位」（最快、最穩）。<br>
                2. 車子被搬到不明位置 → 按「全自動定位」，程式會灑粒子、旋轉並檢查是否收斂。<br>
                3. 若車子被地圖黑塊卡死不動，點擊「清除殘影」。
            </div>

            <button class="btn-init" onclick="sendCommand('/set_initial_pose')">🎯 1. 回到起點自動定位 (免手動)</button>
            <button class="btn-align" onclick="sendCommand('/auto_localize')">🤖 2. 全自動定位 (灑粒子＋旋轉＋檢查收斂)</button>
            <button class="btn-align" style="background-color: #607D8B;" onclick="sendCommand('/clear_costmap')">🧹 3. 清除地圖殘影 (解除黑色區塊卡死)</button>

            <hr style="border-color: #333; margin: 15px 0;">

            <button class="btn-nav" onclick="sendCommand('/goto/沙發')">前往【沙發】</button>
            <button class="btn-nav" onclick="sendCommand('/goto/大門')">前往【大門】</button>
            <button class="btn-nav" onclick="sendCommand('/goto/路由機')">前往【路由機】</button>
            <button class="btn-nav" onclick="sendCommand('/goto/起點')">返回【起點】</button>
        </div>

        <!-- 緊急停止 -->
        <button class="btn-stop" onclick="sendCommand('/kill_all')">🛑 緊急停止 / 一鍵關閉所有系統</button>
    </div>

    <h3>系統狀態：</h3>
    <div id="console">等待指令...</div>

    <script>
        function sendCommand(api_url) {
            const consoleDiv = document.getElementById('console');
            consoleDiv.textContent = "傳送指令中...";

            fetch(api_url)
                .then(response => response.json())
                .then(data => {
                    let time = new Date().toLocaleTimeString();
                    let color = data.status === 'ERROR' || data.status === 'WARNING' ? 'red' : '#00FF00';
                    consoleDiv.innerHTML = '';
                    const span = document.createElement('span');
                    span.style.color = color;
                    span.textContent = `[${time}] ${data.status}  ${data.msg}`;
                    consoleDiv.appendChild(span);
                })
                .catch(error => {
                    consoleDiv.innerHTML = '';
                    const span = document.createElement('span');
                    span.style.color = 'red';
                    span.textContent = `[錯誤] 無法連線到車子: ${error}`;
                    consoleDiv.appendChild(span);
                });
        }
    </script>
</body>
</html>
"""


# ==========================================
# 小工具
# ==========================================
def is_running(pattern):
    """用 pgrep 判斷某個程式是否已在執行（避免重複啟動）"""
    return subprocess.run(["pgrep", "-f", pattern], capture_output=True).returncode == 0


def run_ros(cmd):
    """背景執行一段 ROS 2 指令（不開終端機）。不用 shell=True 拼字串。"""
    return subprocess.Popen(["bash", "-c", f"{ROS_SETUP} && {cmd}"])


def run_in_terminal(cmd, workdir=None):
    """在新的 gnome-terminal 裡執行，方便看 log。"""
    inner = ROS_SETUP
    if workdir:
        inner += f" && cd {shlex.quote(workdir)}"
    inner += f" && {cmd}; exec bash"
    subprocess.Popen(["gnome-terminal", "--", "bash", "-c", inner])


def start_module(label, pattern, workdir, cmd, done_msg):
    if is_running(pattern):
        return jsonify({"status": "WARNING", "msg": f"{label} 已經在執行中，不需要重複啟動。"})
    run_in_terminal(cmd, workdir)
    return jsonify({"status": "SUCCESS", "msg": done_msg})


def initial_pose_cmd():
    """組出對 /initialpose 發佈起點位置的指令（等同在 RViz2 按 2D Pose Estimate）。"""
    x, y, yaw = START_POSE
    cov = [0.0] * 36
    cov[0] = 0.25      # x 變異數
    cov[7] = 0.25      # y 變異數
    cov[35] = 0.0685   # yaw 變異數
    msg = {
        "header": {"frame_id": "map"},
        "pose": {
            "pose": {
                "position": {"x": x, "y": y, "z": 0.0},
                "orientation": {"x": 0.0, "y": 0.0,
                                "z": math.sin(yaw / 2), "w": math.cos(yaw / 2)},
            },
            "covariance": cov,
        },
    }
    # JSON 是 YAML 的子集，可以直接餵給 ros2 topic pub；timeout 避免 AMCL 沒開時卡住
    return ("timeout 15 ros2 topic pub --once /initialpose "
            f"geometry_msgs/msg/PoseWithCovarianceStamped {shlex.quote(json.dumps(msg))}")


# ==========================================
# 前端頁面
# ==========================================
@app.route('/')
def index():
    return render_template_string(HTML_TEMPLATE)


# ==========================================
# 系統模組啟動
# ==========================================
@app.route('/start/nav')
def start_nav():
    return start_module(
        "Nav2", "playrobot_navigation.launch.py",
        os.path.join(AMR_DIR, "第14_15章（共用nav）", "playrobot_robot_nav2"),
        "ros2 launch playrobot_navigation.launch.py",
        "Nav2 導航大腦已啟動。等待地圖載入完成後，請執行步驟二。")


@app.route('/start/voice')
def start_voice():
    return start_module(
        "語音辨識", "voice_nav.py",
        os.path.join(AMR_DIR, "第13章"),
        "python3 voice_nav.py",
        "語音/網頁雙控系統已啟動！")


@app.route('/start/yolo')
def start_yolo():
    return start_module(
        "YOLOv8", "YOLO_ROS.py",
        os.path.join(AMR_DIR, "第16章", "YOLOv8_ROS"),
        "python3 YOLO_ROS.py",
        "YOLOv8 視覺辨識已啟動")


# ==========================================
# 自動定位
# ==========================================
@app.route('/set_initial_pose')
def set_initial_pose():
    run_ros(initial_pose_cmd())
    x, y, yaw = START_POSE
    return jsonify({"status": "SUCCESS",
                    "msg": f"🎯 已把定位設為起點 (x={x}, y={y}, yaw={yaw})，請觀察 RViz2 雷射點是否貼齊地圖。"})


@app.route('/auto_localize')
def auto_localize():
    script = os.path.join(BASE_DIR, "auto_localize.py")
    if not os.path.exists(script):
        return jsonify({"status": "ERROR", "msg": f"找不到 {script}"})
    if is_running("auto_localize.py"):
        return jsonify({"status": "WARNING", "msg": "自動定位正在執行中。"})
    run_in_terminal(f"python3 {shlex.quote(script)}")
    return jsonify({"status": "SUCCESS",
                    "msg": "🤖 全自動定位已啟動，請看新開的終端機輸出，車子會原地旋轉。"})


# ==========================================
# 清除 Costmap 殘影 (解除卡死狀態)
# ==========================================
@app.route('/clear_costmap')
def clear_costmap():
    # 用 ; 而不是 &&：local 失敗時 global 仍然會清
    run_ros(
        'ros2 service call /local_costmap/clear_entirely_local_costmap '
        'nav2_msgs/srv/ClearEntireCostmap "{}"; '
        'ros2 service call /global_costmap/clear_entirely_global_costmap '
        'nav2_msgs/srv/ClearEntireCostmap "{}"'
    )
    return jsonify({"status": "SUCCESS", "msg": "🧹 已強制清除地圖殘影，請重試導航指令！"})


# ==========================================
# 網頁模擬語音訊號發送
# ==========================================
@app.route('/goto/<target>')
def go_to(target):
    run_ros(f"timeout 10 ros2 topic pub --once /web_cmd std_msgs/msg/String \"{{data: '{target}'}}\"")
    return jsonify({"status": "SUCCESS", "msg": f"🌐 網路指令已發送：前往【{target}】"})


# ==========================================
# 一鍵關閉
# ==========================================
@app.route('/kill_all')
def kill_all():
    # 不用 shell=True：pkill -f 會比對完整指令列，
    # 連「執行這串指令的 sh」自己也會被殺掉，後面的指令就不會執行。
    patterns = ["auto_localize.py", "voice_nav.py", "YOLO_ROS.py", "ros2 launch"]
    for p in patterns:                       # 先用 SIGINT 讓 launch 有機會優雅關掉子節點
        subprocess.run(["pkill", "-INT", "-f", p])
    time.sleep(3)
    for p in patterns:                       # 還沒關的才強制砍
        subprocess.run(["pkill", "-KILL", "-f", p])
    subprocess.run(["pkill", "-KILL", "-x", "rviz2"])
    return jsonify({"status": "WARNING", "msg": "🛑 已關閉所有 ROS 2 行程與模組！"})


if __name__ == '__main__':
    print("========================================")
    print("🚀 AMR Web 控制台已啟動！")
    print("請在同一網路下的裝置瀏覽器輸入: http://<車子的IP>:5000")
    print("========================================")
    app.run(host='0.0.0.0', port=5000, debug=False)
