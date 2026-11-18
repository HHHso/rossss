#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
獨立式 ROS 2 Nav2 演算法比較網頁伺服器 (Dijkstra vs A*)
不修改任何既有專案程式碼，透過 /compute_path_to_pose 與動態參數服務進行實測比較。
"""

import threading
import time
import math
from flask import Flask, request, jsonify, render_template_string

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import ComputePathToPose
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue


# ==========================================
# 1. ROS 2 演算法比較專用節點
# ==========================================
class Nav2AlgoCompareNode(Node):
    def __init__(self):
        super().__init__('nav2_algo_compare_web_node')

        # 訂閱 /map (使用 Transient Local 確保能收到已經發佈的靜態地圖)
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            map_qos
        )
        self.latest_map = None

        # 建立更改 planner_server 參數的 Client
        self.param_client = self.create_client(
            SetParameters,
            '/planner_server/set_parameters'
        )

        # 建立向 Nav2 請求純路徑規劃 (不動車) 的 Action Client
        self.path_client = ActionClient(
            self,
            ComputePathToPose,
            '/compute_path_to_pose'
        )

        self.get_logger().info("✅ Nav2 演算法比較節點已啟動！正在等待 /map 資料...")

    def map_callback(self, msg: OccupancyGrid):
        self.latest_map = {
            'width': msg.info.width,
            'height': msg.info.height,
            'resolution': msg.info.resolution,
            'origin_x': msg.info.origin.position.x,
            'origin_y': msg.info.origin.position.y,
            'data': list(msg.data)
        }
        self.get_logger().info(
            f"🗺️ 已接收地圖: {msg.info.width}x{msg.info.height}, 解析度: {msg.info.resolution:.3f}m"
        )

    def set_use_astar(self, use_astar: bool, timeout_sec=3.0) -> bool:
        """動態切換 /planner_server 的 GridBased.use_astar 參數"""
        if not self.param_client.wait_for_service(timeout_sec=timeout_sec):
            self.get_logger().error("❌ 找不到 /planner_server/set_parameters 服務，請確認 Nav2 是否已啟動！")
            return False

        req = SetParameters.Request()
        param = Parameter()
        param.name = 'GridBased.use_astar'
        param.value = ParameterValue(
            type=ParameterType.PARAMETER_BOOL,
            bool_value=bool(use_astar)
        )
        req.parameters = [param]

        future = self.param_client.call_async(req)
        start_wait = time.time()
        while not future.done():
            if time.time() - start_wait > timeout_sec:
                self.get_logger().error("❌ 設定參數逾時！")
                return False
            time.sleep(0.01)

        res = future.result()
        if res and len(res.results) > 0 and res.results[0].successful:
            algo_name = "A* (啟發式搜尋)" if use_astar else "Dijkstra (廣度優先搜尋)"
            self.get_logger().info(f"🔄 已將 Nav2 演算法切換為: {algo_name}")
            return True
        else:
            self.get_logger().error("❌ 參數切換失敗，請確認 nav2_params.yaml 中插件名稱是否為 GridBased")
            return False

    def compute_path(self, start_xy, goal_xy, timeout_sec=5.0):
        """呼叫 /compute_path_to_pose 取得規劃路徑與耗時"""
        if not self.path_client.wait_for_server(timeout_sec=timeout_sec):
            return {'success': False, 'error': '找不到 /compute_path_to_pose Action Server'}

        goal_msg = ComputePathToPose.Goal()
        goal_msg.use_start = True  # 強制使用網頁指定的起點，方便任意測試
        goal_msg.planner_id = 'GridBased'

        now_msg = self.get_clock().now().to_msg()

        # 設定起點
        goal_msg.start = PoseStamped()
        goal_msg.start.header.frame_id = 'map'
        goal_msg.start.header.stamp = now_msg
        goal_msg.start.pose.position.x = float(start_xy[0])
        goal_msg.start.pose.position.y = float(start_xy[1])
        goal_msg.start.pose.orientation.w = 1.0

        # 設定終點
        goal_msg.goal = PoseStamped()
        goal_msg.goal.header.frame_id = 'map'
        goal_msg.goal.header.stamp = now_msg
        goal_msg.goal.pose.position.x = float(goal_xy[0])
        goal_msg.goal.pose.position.y = float(goal_xy[1])
        goal_msg.goal.pose.orientation.w = 1.0

        t0 = time.perf_counter()
        send_goal_future = self.path_client.send_goal_async(goal_msg)
        while not send_goal_future.done():
            if time.perf_counter() - t0 > timeout_sec:
                return {'success': False, 'error': '發送規劃請求逾時'}
            time.sleep(0.005)

        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            return {'success': False, 'error': '規劃請求被拒絕 (起終點可能在障礙物上)'}

        result_future = goal_handle.get_result_async()
        while not result_future.done():
            if time.perf_counter() - t0 > timeout_sec:
                return {'success': False, 'error': '計算路徑結果逾時'}
            time.sleep(0.005)

        total_ms = (time.perf_counter() - t0) * 1000.0
        res = result_future.result().result

        # 轉換路徑點並計算總長度 (公尺)
        pts = []
        total_length = 0.0
        poses = res.path.poses
        for i, p in enumerate(poses):
            x, y = p.pose.position.x, p.pose.position.y
            pts.append({'x': x, 'y': y})
            if i > 0:
                prev = poses[i - 1].pose.position
                total_length += math.hypot(x - prev.x, y - prev.y)

        # Nav2 內部計算時間 (planning_time)
        planning_ms = (
            res.planning_time.sec * 1000.0 + res.planning_time.nanosec / 1e6
        )
        if planning_ms <= 0.0:
            planning_ms = total_ms

        return {
            'success': len(pts) > 0,
            'points': pts,
            'num_Waypoints': len(pts),
            'length_m': round(total_length, 3),
            'planning_time_ms': round(planning_ms, 2)
        }


# ==========================================
# 2. Flask 網頁伺服器與前端 HTML
# ==========================================
app = Flask(__name__)
ros_node: Nav2AlgoCompareNode = None

HTML_PAGE = """
<!DOCTYPE html>
<html lang="zh-TW">
<head>
  <meta charset="UTF-8">
  <title>ROS 2 Nav2 演算法即時切換與比較面板</title>
  <style>
    body { font-family: 'Segoe UI', sans-serif; background: #f0f2f5; margin: 0; padding: 20px; display: flex; flex-direction: column; align-items: center; }
    h2 { color: #1a252f; margin: 0 0 12px 0; }
    .toolbar { background: white; padding: 12px 20px; border-radius: 10px; box-shadow: 0 2px 8px rgba(0,0,0,0.08); display: flex; gap: 12px; align-items: center; margin-bottom: 15px; flex-wrap: wrap; }
    button, select { padding: 8px 14px; border-radius: 6px; border: 1px solid #ccc; font-size: 14px; cursor: pointer; }
    button.primary { background: #2980b9; color: white; border: none; font-weight: bold; }
    button.primary:hover { background: #1f6391; }
    button.switch-btn { background: #27ae60; color: white; border: none; font-weight: bold; }
    .cards { display: flex; gap: 15px; width: 850px; margin-bottom: 15px; }
    .card { flex: 1; background: white; padding: 12px 18px; border-radius: 8px; box-shadow: 0 2px 6px rgba(0,0,0,0.06); border-left: 5px solid #2980b9; }
    .card.astar { border-left-color: #e74c3c; }
    .card h4 { margin: 0 0 8px 0; }
    .card p { margin: 4px 0; font-size: 14px; }
    canvas { background: #ddd; border-radius: 8px; box-shadow: 0 4px 12px rgba(0,0,0,0.15); cursor: crosshair; }
    .status { margin-top: 8px; font-size: 14px; color: #555; font-weight: bold; }
  </style>
</head>
<body>

  <h2>🤖 ROS 2 Nav2 路徑規劃比較：Dijkstra (廣度優先) vs A* (啟發式搜尋)</h2>

  <div class="toolbar">
    <label><strong>點擊地圖設定：</strong></label>
    <select id="clickMode">
      <option value="start">🟢 點擊設定「起點 (Start)」</option>
      <option value="goal">🟠 點擊設定「終點 (Goal)」</option>
    </select>

    <button class="primary" onclick="compareBoth()">⚡ 雙演算法自動切換並比較路徑</button>

    <span style="border-left: 2px solid #ddd; height: 24px;"></span>

    <label><strong>單純切換車載演算法：</strong></label>
    <button onclick="setSingleAlgo(false)">設為 Dijkstra</button>
    <button onclick="setSingleAlgo(true)">設為 A*</button>
  </div>

  <div class="cards">
    <div class="card">
      <h4>🌊 Dijkstra (use_astar = false)</h4>
      <p>規劃耗時：<strong id="dijkTime">-</strong> ms</p>
      <p>路徑總長度：<strong id="dijkLen">-</strong> m</p>
      <p>路徑點數量：<strong id="dijkPts">-</strong> 點</p>
    </div>
    <div class="card astar">
      <h4>🎯 A* 啟發式搜尋 (use_astar = true)</h4>
      <p>規劃耗時：<strong id="astarTime">-</strong> ms</p>
      <p>路徑總長度：<strong id="astarLen">-</strong> m</p>
      <p>路徑點數量：<strong id="astarPts">-</strong> 點</p>
    </div>
  </div>

  <canvas id="mapCanvas" width="850" height="550"></canvas>
  <div class="status" id="statusText">正在載入 ROS 2 /map 地圖...</div>

<script>
const canvas = document.getElementById('mapCanvas');
const ctx = canvas.getContext('2d');
let mapInfo = null;
let scale = 1.0, offsetX = 0, offsetY = 0;

let startWorld = null;
let goalWorld = null;
let dijkPath = [];
let astarPath = [];

async function fetchMap() {
  const res = await fetch('/api/map');
  const data = await res.json();
  if (!data.ready) {
    document.getElementById('statusText').innerText = '⏳ 尚未收到 /map，請確認已啟動 ROS 2 Navigation2 (或改用模擬地圖模式)...';
    setTimeout(fetchMap, 2000);
    return;
  }
  mapInfo = data.map;
  scale = Math.min(canvas.width / mapInfo.width, canvas.height / mapInfo.height);
  offsetX = (canvas.width - mapInfo.width * scale) / 2;
  offsetY = (canvas.height - mapInfo.height * scale) / 2;
  document.getElementById('statusText').innerText = '✅ 已成功同步 ROS 2 /map！請在地圖上點擊設定「起點」與「終點」。';
  render();
}

function worldToCanvas(wx, wy) {
  const gx = (wx - mapInfo.origin_x) / mapInfo.resolution;
  const gy = (wy - mapInfo.origin_y) / mapInfo.resolution;
  const cx = offsetX + gx * scale;
  const cy = offsetY + (mapInfo.height - gy) * scale;
  return { cx, cy };
}

function canvasToWorld(cx, cy) {
  const gx = (cx - offsetX) / scale;
  const gy = mapInfo.height - (cy - offsetY) / scale;
  const wx = gx * mapInfo.resolution + mapInfo.origin_x;
  const wy = gy * mapInfo.resolution + mapInfo.origin_y;
  return { wx, wy };
}

canvas.addEventListener('click', (e) => {
  if (!mapInfo) return;
  const rect = canvas.getBoundingClientRect();
  const cx = e.clientX - rect.left;
  const cy = e.clientY - rect.top;
  const { wx, wy } = canvasToWorld(cx, cy);

  const mode = document.getElementById('clickMode').value;
  if (mode === 'start') {
    startWorld = [wx, wy];
    document.getElementById('clickMode').value = 'goal'; // 自動跳到設定終點
  } else {
    goalWorld = [wx, wy];
  }
  render();
});

function render() {
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  if (!mapInfo) return;

  // 繪製 OccupancyGrid 地圖
  const imgData = ctx.createImageData(mapInfo.width, mapInfo.height);
  for (let y = 0; y < mapInfo.height; y++) {
    for (let x = 0; x < mapInfo.width; x++) {
      const val = mapInfo.data[y * mapInfo.width + x];
      const flipY = mapInfo.height - 1 - y;
      const idx = (flipY * mapInfo.width + x) * 4;
      let color = 205;
      if (val === 0) color = 254;
      else if (val === 100) color = 0;
      else if (val > 0) color = 100;
      imgData.data[idx] = color;
      imgData.data[idx+1] = color;
      imgData.data[idx+2] = color;
      imgData.data[idx+3] = 255;
    }
  }
  const tempCanvas = document.createElement('canvas');
  tempCanvas.width = mapInfo.width;
  tempCanvas.height = mapInfo.height;
  tempCanvas.getContext('2d').putImageData(imgData, 0, 0);
  ctx.imageSmoothingEnabled = false;
  ctx.drawImage(tempCanvas, offsetX, offsetY, mapInfo.width * scale, mapInfo.height * scale);

  // 繪製 Dijkstra 路徑 (藍色)
  if (dijkPath.length > 1) {
    ctx.beginPath();
    ctx.strokeStyle = '#2980b9';
    ctx.lineWidth = 4;
    dijkPath.forEach((p, i) => {
      const { cx, cy } = worldToCanvas(p.x, p.y);
      if (i === 0) ctx.moveTo(cx, cy);
      else ctx.lineTo(cx, cy);
    });
    ctx.stroke();
  }

  // 繪製 A* 路徑 (紅色虛線/疊加)
  if (astarPath.length > 1) {
    ctx.beginPath();
    ctx.strokeStyle = '#e74c3c';
    ctx.lineWidth = 3;
    astarPath.forEach((p, i) => {
      const { cx, cy } = worldToCanvas(p.x, p.y);
      if (i === 0) ctx.moveTo(cx, cy);
      else ctx.lineTo(cx, cy);
    });
    ctx.stroke();
  }

  // 繪製起點與終點
  if (startWorld) {
    const { cx, cy } = worldToCanvas(startWorld[0], startWorld[1]);
    ctx.fillStyle = '#2ecc71';
    ctx.beginPath(); ctx.arc(cx, cy, 7, 0, Math.PI*2); ctx.fill();
  }
  if (goalWorld) {
    const { cx, cy } = worldToCanvas(goalWorld[0], goalWorld[1]);
    ctx.fillStyle = '#f39c12';
    ctx.beginPath(); ctx.arc(cx, cy, 7, 0, Math.PI*2); ctx.fill();
  }
}

async function compareBoth() {
  if (!startWorld || !goalWorld) {
    alert('請先在地圖上點擊設定「起點」與「終點」！');
    return;
  }
  document.getElementById('statusText').innerText = '🔄 正在切換 Nav2 參數並計算兩組路徑...';
  const res = await fetch('/api/compare', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ start: startWorld, goal: goalWorld })
  });
  const data = await res.json();
  if (!data.success) {
    document.getElementById('statusText').innerText = '❌ 錯誤：' + data.error;
    return;
  }

  dijkPath = data.dijkstra.points || [];
  astarPath = data.astar.points || [];

  document.getElementById('dijkTime').innerText = data.dijkstra.planning_time_ms;
  document.getElementById('dijkLen').innerText = data.dijkstra.length_m;
  document.getElementById('dijkPts').innerText = data.dijkstra.num_Waypoints;

  document.getElementById('astarTime').innerText = data.astar.planning_time_ms;
  document.getElementById('astarLen').innerText = data.astar.length_m;
  document.getElementById('astarPts').innerText = data.astar.num_Waypoints;

  document.getElementById('statusText').innerText = '✅ 比較完成！藍線為 Dijkstra，紅線為 A*。';
  render();
}

async function setSingleAlgo(useAstar) {
  const res = await fetch('/api/set_algo', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ use_astar: useAstar })
  });
  const data = await res.json();
  document.getElementById('statusText').innerText = data.message;
}

fetchMap();
</script>
</body>
</html>
"""

@app.route('/')
def index():
    return render_template_string(HTML_PAGE)

@app.route('/api/map')
def api_map():
    if ros_node is None or ros_node.latest_map is None:
        return jsonify({'ready': False})
    return jsonify({'ready': True, 'map': ros_node.latest_map})

@app.route('/api/set_algo', methods=['POST'])
def api_set_algo():
    use_astar = request.json.get('use_astar', False)
    ok = ros_node.set_use_astar(use_astar)
    name = "A* (啟發式搜尋)" if use_astar else "Dijkstra (廣度優先搜尋)"
    if ok:
        return jsonify({'success': True, 'message': f'✅ 已將 Nav2 全域規劃器切換為：{name}'})
    return jsonify({'success': False, 'message': '❌ 切換失敗，請確認 Nav2 是否正在執行'})

@app.route('/api/compare', methods=['POST'])
def api_compare():
    req = request.json
    start_xy = req.get('start')
    goal_xy = req.get('goal')

    # 1. 切換為 Dijkstra (use_astar = False) 並計算路徑
    if not ros_node.set_use_astar(False):
        return jsonify({'success': False, 'error': '無法切換參數至 Dijkstra'})
    dijk_res = ros_node.compute_path(start_xy, goal_xy)
    if not dijk_res['success']:
        return jsonify({'success': False, 'error': f"Dijkstra 規劃失敗: {dijk_res.get('error')}"})

    # 2. 切換為 A* (use_astar = True) 並計算路徑
    if not ros_node.set_use_astar(True):
        return jsonify({'success': False, 'error': '無法切換參數至 A*'})
    astar_res = ros_node.compute_path(start_xy, goal_xy)
    if not astar_res['success']:
        return jsonify({'success': False, 'error': f"A* 規劃失敗: {astar_res.get('error')}"})

    return jsonify({
        'success': True,
        'dijkstra': dijk_res,
        'astar': astar_res
    })


def ros_spin_thread():
    rclpy.spin(ros_node)


if __name__ == '__main__':
    rclpy.init()
    ros_node = Nav2AlgoCompareNode()

    t = threading.Thread(target=ros_spin_thread, daemon=True)
    t.start()

    print("🚀 演算法比較網頁已啟動：請打開瀏覽器輸入 http://127.0.0.1:5005")
    app.run(host='0.0.0.0', port=5005, debug=False)