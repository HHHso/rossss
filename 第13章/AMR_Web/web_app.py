#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from flask import Flask, render_template_string, jsonify
import subprocess

app = Flask(__name__)

# 用來記錄目前正在跑的終端機行程
active_processes = {}

# 前端網頁的 HTML 與 CSS 設計
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
                1. 點擊「全域尋找」，RViz2 會灑滿彩色粒子。<br>
                2. 點擊「旋轉 360 度」，粒子會自動收斂並對齊特徵。<br>
                3. 若車子被地圖黑塊卡死不動，點擊「清除殘影」。
            </div>
            
            <button class="btn-init" onclick="sendCommand('/global_loc')">🌍 1. 全域尋找自己 (灑滿粒子)</button>
            <button class="btn-align" onclick="sendCommand('/auto_align')">🔄 2. 自動對齊特徵 (原地旋轉 360 度)</button>
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
            consoleDiv.innerHTML = "傳送指令中...";
            
            fetch(api_url)
                .then(response => response.json())
                .then(data => {
                    let time = new Date().toLocaleTimeString();
                    let color = data.status === 'ERROR' || data.status === 'WARNING' ? 'red' : '#00FF00';
                    consoleDiv.innerHTML = `<span style="color:${color};">[${time}] ${data.status} <br> ${data.msg}</span>`;
                })
                .catch(error => {
                    consoleDiv.innerHTML = `<span style="color:red;">[錯誤] 無法連線到車子: ${error}</span>`;
                });
        }
    </script>
</body>
</html>
"""

@app.route('/')
def index():
    return render_template_string(HTML_TEMPLATE)

@app.route('/start/nav')
def start_nav():
    cmd = 'gnome-terminal -- bash -c "source /opt/ros/humble/setup.bash && cd ~/Desktop/AMR_範例包/第14_15章（共用nav）/playrobot_robot_nav2 && ros2 launch playrobot_navigation.launch.py; exec bash"'
    active_processes['nav'] = subprocess.Popen(cmd, shell=True)
    return jsonify({"status": "SUCCESS", "msg": "Nav2 導航大腦已啟動。等待地圖載入後，請執行步驟二。"})

@app.route('/start/voice')
def start_voice():
    cmd = 'gnome-terminal -- bash -c "source /opt/ros/humble/setup.bash && cd /home/playrobot/Desktop/AMR_範例包/第13章 && python3 voice_nav.py; exec bash"'
    active_processes['voice'] = subprocess.Popen(cmd, shell=True)
    return jsonify({"status": "SUCCESS", "msg": "語音/網頁雙控系統已啟動！"})

@app.route('/start/yolo')
def start_yolo():
    # 1. 啟動攝影機節點 (負責擷取實體鏡頭並發布到 RGB_raw)
    cmd_pub = 'gnome-terminal -- bash -c "source /opt/ros/humble/setup.bash && cd ~/Desktop/AMR_範例包/第16章/YOLOv8_ROS && python3 img_pub.py; exec bash"'
    subprocess.Popen(cmd_pub, shell=True)
    
    # 2. 啟動 YOLO 辨識與控制節點 (負責接收畫面、尋找水瓶並發送馬達指令)
    cmd_yolo = 'gnome-terminal -- bash -c "source /opt/ros/humble/setup.bash && cd ~/Desktop/AMR_範例包/第16章/YOLOv8_ROS && python3 yolov2.py; exec bash"'
    active_processes['yolo'] = subprocess.Popen(cmd_yolo, shell=True)
    
    return jsonify({"status": "SUCCESS", "msg": "📷 攝影機與 YOLOv8 系統已雙雙啟動！"})

# ==========================================
# 定位與對齊指令
# ==========================================
@app.route('/global_loc')
def global_loc():
    cmd = '''gnome-terminal -- bash -c "source /opt/ros/humble/setup.bash && ros2 service call /reinitialize_global_localization std_srvs/srv/Empty; exec bash"'''
    subprocess.Popen(cmd, shell=True)
    return jsonify({"status": "SUCCESS", "msg": "🌍 已啟動全域尋找！請接著點擊「旋轉 360 度」讓車子認出環境。"})

@app.route('/auto_align')
def auto_align():
    cmd = 'gnome-terminal -- bash -c "source /opt/ros/humble/setup.bash && ros2 action send_goal /spin nav2_msgs/action/Spin \\"{target_yaw: 6.28}\\"; exec bash"'
    subprocess.Popen(cmd, shell=True)
    return jsonify({"status": "SUCCESS", "msg": "🔄 已送出旋轉指令，請觀察特徵線是否對齊。"})

# ==========================================
# 清除 Costmap 殘影 (解除卡死狀態)
# ==========================================
@app.route('/clear_costmap')
def clear_costmap():
    # 同時清除 local 和 global 代價地圖，使用正確的 nav2_msgs/srv/ClearEntireCostmap 型態
    cmd = '''
    source /opt/ros/humble/setup.bash && \
    ros2 service call /local_costmap/clear_entirely_local_costmap nav2_msgs/srv/ClearEntireCostmap "{}" && \
    ros2 service call /global_costmap/clear_entirely_global_costmap nav2_msgs/srv/ClearEntireCostmap "{}"
    '''
    subprocess.Popen(cmd, shell=True, executable='/bin/bash')
    return jsonify({"status": "SUCCESS", "msg": "🧹 已強制清除地圖殘影，請重試導航指令！"})

# ==========================================
# 網頁模擬語音訊號發送
# ==========================================
@app.route('/goto/<target>')
def go_to(target):
    cmd = f'''
    source /opt/ros/humble/setup.bash && \
    ros2 topic pub --once /web_cmd std_msgs/msg/String "{{data: '{target}'}}"
    '''
    subprocess.Popen(cmd, shell=True, executable='/bin/bash')
    return jsonify({"status": "SUCCESS", "msg": f"🌐 網路指令已發送：前往【{target}】"})

@app.route('/kill_all')
def kill_all():
    cmd = "pkill -9 -f ros2; pkill -9 -f rviz2; killall -9 _ros2_daemon; pkill -9 -f voice_nav.py; pkill -9 -f YOLO_ROS.py"
    subprocess.run(cmd, shell=True)
    active_processes.clear()
    return jsonify({"status": "WARNING", "msg": "🛑 已強制關閉所有 ROS 2 行程與終端機！"})

if __name__ == '__main__':
    print("========================================")
    print("🚀 AMR Web 控制台已啟動！")
    print("請在同一網路下的裝置瀏覽器輸入: http://<車子的IP>:5000")
    print("========================================")
    app.run(host='0.0.0.0', port=5000, debug=False)