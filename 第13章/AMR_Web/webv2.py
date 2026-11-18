#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from flask import Flask, render_template_string, jsonify, Response, request
import subprocess
import time

app = Flask(__name__)

active_processes = {}
latest_frame = None
latest_lidar = {
    "front": 9.99, "left": 9.99, "right": 9.99, "back": 9.99,
    "danger": False, "points": []
}
radar_config = {
    "flip_lidar": False,     # 光達前後反轉校正
    "motor_enabled": True    # 是否讓 YOLO 控制馬達轉動
}

HTML_TEMPLATE = """
<!DOCTYPE html>
<html lang="zh-TW">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>AMR AI 哨兵與倒車雷達控制中心</title>
    <style>
        body { font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif; background-color: #121212; color: #ffffff; padding: 20px; max-width: 1100px; margin: 0 auto; }
        h1 { text-align: center; color: #4CAF50; border-bottom: 2px solid #333; padding-bottom: 10px; margin-bottom: 20px; }
        .grid-container { display: grid; grid-template-columns: 1.2fr 1fr; gap: 20px; }
        .panel { background-color: #1e1e1e; padding: 20px; border-radius: 10px; box-shadow: 0 4px 6px rgba(0,0,0,0.4); }
        h2 { font-size: 18px; color: #cccccc; margin-top: 0; margin-bottom: 12px; display: flex; justify-content: space-between; align-items: center; }
        button { width: 100%; padding: 12px; margin-bottom: 10px; font-size: 15px; font-weight: bold; border: none; border-radius: 6px; cursor: pointer; transition: 0.2s; }
        
        .btn-start { background-color: #2196F3; color: white; }
        .btn-start:hover { background-color: #1976D2; }
        .btn-init { background-color: #9C27B0; color: white; }
        .btn-align { background-color: #009688; color: white; }
        .btn-nav { background-color: #FF9800; color: white; }
        .btn-tool { background-color: #37474F; color: #ECEFF1; font-size: 14px; padding: 8px; }
        .btn-tool:hover { background-color: #455A64; }
        .btn-stop { background-color: #F44336; color: white; grid-column: span 2; font-size: 18px; padding: 15px; }
        .btn-stop:hover { background-color: #D32F2F; }
        
        #video_feed { width: 100%; border-radius: 8px; border: 2px solid #333; background-color: #000; min-height: 280px; }
        
        /* 倒車雷達 HUD 樣式 */
        .radar-box { text-align: center; position: relative; }
        #radarCanvas { background: radial-gradient(circle, #1a261e 0%, #0d1110 100%); border-radius: 50%; border: 2px solid #2e4c38; box-shadow: 0 0 15px rgba(0,255,100,0.1); }
        .dist-grid { display: grid; grid-template-columns: repeat(4, 1fr); gap: 8px; margin-top: 12px; }
        .dist-card { background: #141414; padding: 8px 4px; border-radius: 6px; border: 1px solid #333; text-align: center; }
        .dist-label { font-size: 12px; color: #888; }
        .dist-val { font-size: 18px; font-weight: bold; margin-top: 4px; color: #4CAF50; }
        .danger-banner { background-color: #d32f2f; color: white; padding: 6px 12px; border-radius: 4px; font-size: 14px; animation: blink 0.6s infinite alternate; display: none; }
        @keyframes blink { from { opacity: 1; } to { opacity: 0.3; } }

        #console { background-color: #000; color: #00FF00; padding: 15px; border-radius: 5px; font-family: monospace; margin-top: 20px; min-height: 45px; }
    </style>
</head>
<body>

    <h1>控制中心</h1>

    <div class="grid-container">
        <!-- 左上：YOLOv8 即時視覺監控 -->
        <div class="panel" style="text-align: center;">
            <h2>
                <span>YOLOv8 監控</span>
                <span id="danger_badge" class="danger-banner"> 前方障礙物！</span>
            </h2>
            <img id="video_feed" src="/video_feed" alt="等待 YOLOv8 啟動...">
            <div style="display: flex; gap: 10px; margin-top: 10px;">
                <button id="btn_motor" class="btn-tool" onclick="toggleMotor()">YOLO開啟中</button>
                <button id="btn_flip" class="btn-tool" onclick="toggleFlip()">光達車頭方向：正常 (0°)</button>
            </div>
        </div>

        <!-- 右上：汽車倒車雷達 HUD -->
        <div class="panel radar-box">
            <h2><span>雷達數據</span> <span style="font-size:12px; color:#4CAF50;">煞車線: 35cm</span></h2>
            <canvas id="radarCanvas" width="260" height="260"></canvas>
            
            <div class="dist-grid">
                <div class="dist-card">
                    <div class="dist-label">🔼 正前方</div>
                    <div id="d_front" class="dist-val">-- m</div>
                </div>
                <div class="dist-card">
                    <div class="dist-label">◀️ 左側</div>
                    <div id="d_left" class="dist-val">-- m</div>
                </div>
                <div class="dist-card">
                    <div class="dist-label">▶️ 右側</div>
                    <div id="d_right" class="dist-val">-- m</div>
                </div>
                <div class="dist-card">
                    <div class="dist-label">🔽 正後方</div>
                    <div id="d_back" class="dist-val">-- m</div>
                </div>
            </div>
        </div>

        <!-- 左下：步驟一 -->
        <div class="panel">
            <h2>步驟一：系統模組啟動</h2>
            <button class="btn-start" onclick="sendCommand('/start/nav')">1. 啟動 Nav2 導航大腦 (含底盤與光達)</button>
            <button class="btn-start" onclick="sendCommand('/start/voice')">2. 啟動語音辨識系統</button>
            <button class="btn-start" onclick="sendCommand('/start/yolo')">3. 啟動 YOLOv8 視覺哨兵與雷達連線</button>
        </div>

        <!-- 右下：步驟二 -->
        <div class="panel">
            <h2>步驟二：全自動定位與導航</h2>
            <button class="btn-align" onclick="sendCommand('/auto_align')">  1. 自動對齊特徵 (旋轉 360 度)</button>
            <button class="btn-align" style="background-color: #607D8B;" onclick="sendCommand('/clear_costmap')"> 2. 清除地圖殘影 (解除卡死)</button>
            <hr style="border-color: #333; margin: 12px 0;">
            <div style="display: flex; gap: 10px;">
                <button class="btn-nav" onclick="sendCommand('/goto/沙發')">前往【沙發】</button>
                <button class="btn-nav" onclick="sendCommand('/goto/起點')">返回【起點】</button>
            </div>
        </div>

        <button class="btn-stop" onclick="sendCommand('/kill_all')">🛑 停止 </button>
    </div>

    <div id="console">系統待命中...</div>

    <script>
        function sendCommand(api_url) {
            const consoleDiv = document.getElementById('console');
            consoleDiv.innerHTML = "傳送指令中...";
            fetch(api_url)
                .then(response => response.json())
                .then(data => {
                    let time = new Date().toLocaleTimeString();
                    let color = data.status === 'ERROR' || data.status === 'WARNING' ? '#ff5252' : '#00FF00';
                    consoleDiv.innerHTML = `<span style="color:${color};">[${time}] ${data.status}：${data.msg}</span>`;
                })
                .catch(error => {
                    consoleDiv.innerHTML = `<span style="color:red;">[錯誤] 無法連線: ${error}</span>`;
                });
        }

        function toggleFlip() {
            fetch('/toggle_flip').then(r => r.json()).then(d => {
                document.getElementById('btn_flip').innerText = d.flip_lidar ? " 光達車頭方向：正常" : " 光達車頭方向：已反轉 ";
            });
        }

        function toggleMotor() {
            fetch('/toggle_motor').then(r => r.json()).then(d => {
                const btn = document.getElementById('btn_motor');
                if (d.motor_enabled) {
                    btn.innerText = "🏃 哨兵追擊馬達：開啟中 (點擊暫停轉動)";
                    btn.style.backgroundColor = "#37474F";
                } else {
                    btn.innerText = "⏸️ 哨兵追擊馬達：已暫停 (僅監控畫面與雷達)";
                    btn.style.backgroundColor = "#E65100";
                }
            });
        }

        // ==========================================
        // 繪製汽車倒車雷達 HUD
        // ==========================================
        const canvas = document.getElementById('radarCanvas');
        const ctx = canvas.getContext('2d');
        const cx = canvas.width / 2;
        const cy = canvas.height / 2;
        const maxRange = 2.0; // 雷達畫面最大顯示半徑 2.0 公尺
        const scale = (cx - 15) / maxRange;

        function getColor(dist) {
            if (dist < 0.38) return '#FF1744'; // 危險紅
            if (dist < 0.80) return '#FF9100'; // 警戒橘黃
            return '#00E676';                  // 安全綠
        }

        function updateDistText(id, val) {
            const el = document.getElementById(id);
            if (val >= 9.0) {
                el.innerText = "> 3.0m";
                el.style.color = "#00E676";
            } else {
                el.innerText = val.toFixed(2) + " m";
                el.style.color = getColor(val);
            }
        }

        function drawSectorArc(startDeg, endDeg, dist) {
            const clampedDist = Math.min(dist, maxRange);
            const r = Math.max(22, clampedDist * scale);
            const color = getColor(dist);
            
            // Canvas 0度在右邊，我們讓車頭正前方朝上 (-90度)
            const startRad = (startDeg - 90) * Math.PI / 180;
            const endRad = (endDeg - 90) * Math.PI / 180;

            ctx.beginPath();
            ctx.arc(cx, cy, r, startRad, endRad);
            ctx.strokeStyle = color;
            ctx.lineWidth = dist < 0.38 ? 8 : 5;
            ctx.stroke();
        }

        function renderRadar(data) {
            ctx.clearRect(0, 0, canvas.width, canvas.height);

            // 1. 畫同心圓距離標尺 (0.35m 紅線、1.0m、2.0m)
            [0.35, 1.0, 2.0].forEach(d => {
                ctx.beginPath();
                ctx.arc(cx, cy, d * scale, 0, Math.PI * 2);
                ctx.strokeStyle = (d === 0.35) ? 'rgba(255, 23, 68, 0.45)' : 'rgba(255, 255, 255, 0.12)';
                ctx.lineWidth = 1;
                ctx.stroke();
            });

            // 2. 畫十字準線
            ctx.beginPath();
            ctx.moveTo(cx, 0); ctx.lineTo(cx, canvas.height);
            ctx.moveTo(0, cy); ctx.lineTo(canvas.width, cy);
            ctx.strokeStyle = 'rgba(255, 255, 255, 0.08)';
            ctx.stroke();

            // 3. 畫四向倒車雷達感應波紋 (前、右、後、左)
            drawSectorArc(-30, 30, data.front);   // 前方
            drawSectorArc(45, 135, data.right);   // 右側
            drawSectorArc(150, 210, data.back);   // 後方
            drawSectorArc(-135, -45, data.left);  // 左側

            // 4. 畫 360 度真實雷射點雲 (白色/青色光點)
            if (data.points) {
                ctx.fillStyle = '#00E5FF';
                data.points.forEach(pt => {
                    const angle = pt[0]; // 弧度 (0 為正前方，逆時針為正)
                    const dist = pt[1];
                    if (dist <= maxRange) {
                        // 轉換至 Canvas 座標 (車頭朝上，左正右負)
                        const px = cx - Math.sin(angle) * (dist * scale);
                        const py = cy - Math.cos(angle) * (dist * scale);
                        ctx.beginPath();
                        ctx.arc(px, py, 2.2, 0, Math.PI * 2);
                        ctx.fill();
                    }
                });
            }

            // 5. 畫正中央車體與車頭箭頭
            ctx.fillStyle = data.danger ? '#FF1744' : '#2196F3';
            ctx.beginPath();
            ctx.roundRect(cx - 12, cy - 16, 24, 32, 5);
            ctx.fill();
            // 車頭白色小箭頭
            ctx.fillStyle = '#FFFFFF';
            ctx.beginPath();
            ctx.moveTo(cx, cy - 14);
            ctx.lineTo(cx - 6, cy - 4);
            ctx.lineTo(cx + 6, cy - 4);
            ctx.closePath();
            ctx.fill();

            // 6. 更新下方數字與危險橫幅
            updateDistText('d_front', data.front);
            updateDistText('d_left', data.left);
            updateDistText('d_right', data.right);
            updateDistText('d_back', data.back);
            document.getElementById('danger_badge').style.display = data.danger ? 'inline-block' : 'none';
        }

        // 每 120 毫秒拉取最新光達雷達數據
        setInterval(() => {
            fetch('/lidar_data')
                .then(r => r.json())
                .then(data => renderRadar(data))
                .catch(() => {});
        }, 120);
    </script>
</body>
</html>
"""

@app.route('/')
def index():
    return render_template_string(HTML_TEMPLATE)

@app.route('/upload_frame', methods=['POST'])
def upload_frame():
    global latest_frame
    latest_frame = request.data
    return 'OK'

@app.route('/upload_lidar', methods=['POST'])
def upload_lidar():
    global latest_lidar
    latest_lidar = request.json
    return jsonify(radar_config)

@app.route('/lidar_data')
def get_lidar_data():
    return jsonify(latest_lidar)

@app.route('/toggle_flip')
def toggle_flip():
    radar_config["flip_lidar"] = not radar_config["flip_lidar"]
    return jsonify(radar_config)

@app.route('/toggle_motor')
def toggle_motor():
    radar_config["motor_enabled"] = not radar_config["motor_enabled"]
    return jsonify(radar_config)

@app.route('/video_feed')
def video_feed():
    def generate():
        global latest_frame
        while True:
            if latest_frame:
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + latest_frame + b'\r\n')
            time.sleep(0.04)
    return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')

# ==========================================
# 注意：拿掉了尾端的 "; exec bash"，讓程式結束時視窗能自動關閉！
# ==========================================
@app.route('/start/nav')
def start_nav():
    cmd = 'gnome-terminal -- bash -c "source /opt/ros/humble/setup.bash && cd ~/Desktop/AMR_範例包/第14_15章（共用nav）/playrobot_robot_nav2 && ros2 launch playrobot_navigation.launch.py"'
    active_processes['nav'] = subprocess.Popen(cmd, shell=True)
    return jsonify({"status": "SUCCESS", "msg": "Nav2 導航大腦與光達底盤已啟動。"})

@app.route('/start/voice')
def start_voice():
    cmd = 'gnome-terminal -- bash -c "source /opt/ros/humble/setup.bash && cd /home/playrobot/Desktop/AMR_範例包/第13章 && python3 voice_nav.py"'
    active_processes['voice'] = subprocess.Popen(cmd, shell=True)
    return jsonify({"status": "SUCCESS", "msg": "語音控制系統已啟動！"})

@app.route('/start/yolo')
def start_yolo():
    cmd_pub = 'gnome-terminal -- bash -c "source /opt/ros/humble/setup.bash && cd ~/Desktop/AMR_範例包/第16章/YOLOv8_ROS && python3 img_pub.py"'
    subprocess.Popen(cmd_pub, shell=True)
    time.sleep(1)
    cmd_yolo = 'gnome-terminal -- bash -c "source /opt/ros/humble/setup.bash && cd ~/Desktop/AMR_範例包/第16章/YOLOv8_ROS && python3 yolov2.py"'
    active_processes['yolo'] = subprocess.Popen(cmd_yolo, shell=True)
    return jsonify({"status": "SUCCESS", "msg": "📷 YOLOv8 與倒車雷達已啟動！"})

@app.route('/global_loc')
def global_loc():
    cmd = 'source /opt/ros/humble/setup.bash && ros2 service call /reinitialize_global_localization std_srvs/srv/Empty'
    subprocess.Popen(cmd, shell=True, executable='/bin/bash')
    return jsonify({"status": "SUCCESS", "msg": "🌍 已灑滿粒子！請接著點擊「旋轉 360 度」。"})

@app.route('/auto_align')
def auto_align():
    cmd = 'source /opt/ros/humble/setup.bash && ros2 action send_goal /spin nav2_msgs/action/Spin "{target_yaw: 6.28}"'
    subprocess.Popen(cmd, shell=True, executable='/bin/bash')
    return jsonify({"status": "SUCCESS", "msg": "🔄 已送出旋轉 360 度指令。"})

@app.route('/clear_costmap')
def clear_costmap():
    cmd = '''
    source /opt/ros/humble/setup.bash && \
    ros2 service call /local_costmap/clear_entirely_local_costmap nav2_msgs/srv/ClearEntireCostmap "{}" && \
    ros2 service call /global_costmap/clear_entirely_global_costmap nav2_msgs/srv/ClearEntireCostmap "{}"
    '''
    subprocess.Popen(cmd, shell=True, executable='/bin/bash')
    return jsonify({"status": "SUCCESS", "msg": "🧹 已強制清除地圖殘影！"})

@app.route('/goto/<target>')
def go_to(target):
    cmd = f'''
    source /opt/ros/humble/setup.bash && \
    ros2 topic pub --once /web_cmd std_msgs/msg/String "{{data: '{target}'}}"
    '''
    subprocess.Popen(cmd, shell=True, executable='/bin/bash')
    return jsonify({"status": "SUCCESS", "msg": f"🌐 前往【{target}】"})

# ==========================================
# 🛑 終極安全停機：先殺上層大腦 -> 發送 0 速度鎖死馬達 -> 最後關底盤與視窗
# ==========================================
@app.route('/kill_all')
def kill_all():
    # 1. 先停掉所有會發送移動指令的上層程式與相機 (保留 playrobot_base 底盤通訊活著)
    step1_kill_brains = (
        "pkill -SIGINT -f yolov2.py; "
        "pkill -9 -f img_pub.py; "
        "pkill -9 -f voice_nav.py; "
        "pkill -9 -f controller_server; "
        "pkill -9 -f bt_navigator; "
        "pkill -9 -f planner_server"
    )
    subprocess.run(step1_kill_brains, shell=True)
    time.sleep(0.3)

    # 2. 趁底盤 playrobot_base 還活著，強制發送歸零速度給 Arduino 煞車
    step2_brake = (
        "source /opt/ros/humble/setup.bash && "
        "ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "
        "'{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'"
    )
    try:
        subprocess.run(step2_brake, shell=True, executable='/bin/bash', timeout=2)
    except Exception:
        pass

    # 3. 徹底殺掉底盤、光達、RViz2 與所有殘留節點
    step3_kill_all = (
        "pkill -9 -f yolov2.py; "
        "pkill -9 -f playrobot_base; "
        "pkill -9 -f base_control; "
        "pkill -9 -f sllidar_node; "
        "pkill -9 -f rf2o_laser_odometry; "
        "pkill -9 -f component_container; "
        "pkill -9 -f rviz2; "
        "pkill -9 -f ros2; "
        "killall -9 _ros2_daemon 2>/dev/null"
    )
    subprocess.run(step3_kill_all, shell=True)
    active_processes.clear()
    return jsonify({"status": "WARNING", "msg": "🛑 底盤已強制煞停，鏡頭與所有背景系統皆已關閉！"})

if __name__ == '__main__':
    print("🚀 AMR 網頁中控台已啟動！請開啟 http://127.0.0.1:5000")
    app.run(host='0.0.0.0', port=5000, debug=False)