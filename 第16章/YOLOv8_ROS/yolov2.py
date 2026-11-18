#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# ROS
import rclpy
import rclpy.node as node
import cv2
import numpy as np
import sensor_msgs.msg as msg
import std_msgs.msg as std_msg
from geometry_msgs.msg import Twist
import time
import math
import signal
import sys
import requests
import threading

# YOLO
from typing import Tuple, Dict
from ultralytics.yolo.utils import ops
from ultralytics.yolo.utils.plotting import colors
import torch
from openvino.runtime import Core, Model

def letterbox(img: np.ndarray, new_shape: Tuple[int, int] = (320, 320), color: Tuple[int, int, int] = (114, 114, 114), auto: bool = False, scale_fill: bool = False, scaleup: bool = False, stride: int = 32):
    shape = img.shape[:2]
    if isinstance(new_shape, int):
        new_shape = (new_shape, new_shape)
    r = min(new_shape[0] / shape[0], new_shape[1] / shape[1])
    if not scaleup:
        r = min(r, 1.0)
    ratio = r, r
    new_unpad = int(round(shape[1] * r)), int(round(shape[0] * r))
    dw, dh = new_shape[1] - new_unpad[0], new_shape[0] - new_unpad[1]
    if auto:
        dw, dh = np.mod(dw, stride), np.mod(dh, stride)
    elif scale_fill:
        dw, dh = 0.0, 0.0
        new_unpad = (new_shape[1], new_shape[0])
        ratio = new_shape[1] / shape[1], new_shape[0] / shape[0]
    dw /= 2
    dh /= 2
    if shape[::-1] != new_unpad:
        img = cv2.resize(img, new_unpad, interpolation=cv2.INTER_LINEAR)
    top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
    left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
    img = cv2.copyMakeBorder(img, top, bottom, left, right, cv2.BORDER_CONSTANT, value=color)
    return img, ratio, (dw, dh)

def plot_one_box(box: np.ndarray, img: np.ndarray, color: Tuple[int, int, int] = None, label: str = None, line_thickness: int = 2):
    c1, c2 = (int(box[0]), int(box[1])), (int(box[2]), int(box[3]))
    cv2.rectangle(img, c1, c2, color, thickness=line_thickness)
    if label:
        tf = max(line_thickness - 1, 1)
        t_size = cv2.getTextSize(label, 0, fontScale=0.6, thickness=tf)[0]
        c2 = c1[0] + t_size[0], c1[1] - t_size[1] - 3
        cv2.rectangle(img, c1, c2, color, -1)
        cv2.putText(img, label, (c1[0], c1[1] - 2), 0, 0.6, [225, 255, 255], thickness=tf)
    return img

def draw_results(boxes: np.ndarray, source_image: np.ndarray, label_map: Dict):
    for *xyxy, conf, lbl in boxes:
        label = f'{label_map[int(lbl)]} {conf:.2f}'
        source_image = plot_one_box(xyxy, source_image, label=label, color=colors(int(lbl)), line_thickness=2)
    return source_image

def preprocess_image(img0: np.ndarray):
    img = letterbox(img0)[0]
    img = img.transpose(2, 0, 1)
    img = np.ascontiguousarray(img, dtype=np.float32)
    img /= 255.0
    return np.expand_dims(img, 0)

def detect(image: np.ndarray, model: Model):
    input_tensor = preprocess_image(image)
    result = model(input_tensor)
    boxes = result[model.output(0)]
    input_hw = input_tensor.shape[2:]
    detections = postprocess(pred_boxes=boxes, input_hw=input_hw, orig_img=image)
    return detections

def postprocess(
    pred_boxes: np.ndarray, 
    input_hw: Tuple[int, int], 
    orig_img: np.ndarray, 
    min_conf_threshold: float = 0.55, 
    nms_iou_threshold: float = 0.7, 
    max_detections: int = 10
):
    preds = ops.non_max_suppression(
        torch.from_numpy(pred_boxes),
        min_conf_threshold,
        nms_iou_threshold,
        classes=[0], 
        nc=80,
        max_det=max_detections
    )
    results = []
    shape = orig_img.shape
    for pred in preds:
        if not len(pred):
            results.append({"det": []})
            continue
        pred[:, :4] = ops.scale_boxes(input_hw, pred[:, :4], shape).round()
        results.append({"det": pred[:, :6].numpy()})
    return results

core = Core()
det_ov_model = core.read_model('yolov8n-seg.xml')
det_ov_model.reshape({0: [1, 3, 320, 320]})
det_compiled_model = core.compile_model(det_ov_model, "CPU", {"PERFORMANCE_HINT": "LATENCY"})

label_map = {0: 'person'}

class TestDisplayNode(node.Node):
    def __init__(self):
        super().__init__('IProc_TestDisplayNode')
        
        self.scan_sub = self.create_subscription(msg.LaserScan, '/scan', self.scan_callback, 1)
        self.sub = self.create_subscription(msg.Image, 'RGB_raw', self.msg_callback, 1)
        self.cmd_pub = self.create_publisher(std_msg.String, '/web_cmd', 10) 
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)          
        
        self.obstacle_too_close = False
        self.is_tracking = False
        self.last_seen_time = 0.0  
        
        # 網頁可動態切換的控制開關
        self.flip_lidar = False     # 是否反轉光達 180 度
        self.motor_enabled = True   # 是否允許 YOLO 控制底盤馬達
        
        # 網頁串流與雷達數據變數
        self.latest_frame_bytes = None
        self.latest_lidar_dict = None
        self.frame_lock = threading.Lock()
        self.session = requests.Session()
        self.stream_thread = threading.Thread(target=self._stream_worker, daemon=True)
        self.stream_thread.start()

        self.prev_time = time.time()

    def stop_robot(self):
        """強制發送 0 速度，確保馬達停下"""
        try:
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            for _ in range(3):
                self.vel_pub.publish(twist)
                time.sleep(0.05)
        except Exception:
            pass

    def scan_callback(self, scan_msg: msg.LaserScan):
        """解析 360 度光達距離，分為前後左右四區 + 雷射點雲供網頁倒車雷達顯示"""
        ranges = np.array(scan_msg.ranges, dtype=np.float32)
        num_pts = len(ranges)
        if num_pts == 0:
            return

        # 建立每個點對應的角度 (轉換至 -180 ~ 180 度)
        angles = scan_msg.angle_min + np.arange(num_pts) * scan_msg.angle_increment
        if self.flip_lidar:
            angles = angles + math.pi  # 若光達反裝，旋轉 180 度
        
        # 正規化角度到 [-pi, pi]
        angles = np.arctan2(np.sin(angles), np.cos(angles))
        angles_deg = np.degrees(angles)

        # 過濾有效距離 (忽略小於 0.18m 的車體自身遮擋與無限遠)
        valid_mask = np.isfinite(ranges) & (ranges > 0.18) & (ranges < 8.0)

        def get_sector_min(deg_min, deg_max):
            if deg_min <= deg_max:
                mask = valid_mask & (angles_deg >= deg_min) & (angles_deg <= deg_max)
            else:
                # 跨越 180/-180 度邊界 (後方)
                mask = valid_mask & ((angles_deg >= deg_min) | (angles_deg <= deg_max))
            vals = ranges[mask]
            return float(np.min(vals)) if len(vals) > 0 else 9.99

        # 計算四個方向的最近距離 (公尺)
        front_dist = get_sector_min(-25, 25)    # 正前方 ±25度
        left_dist  = get_sector_min(45, 135)    # 左側
        right_dist = get_sector_min(-135, -45)  # 右側
        back_dist  = get_sector_min(150, -150)  # 正後方

        # 防撞判定：正前方小於 0.35m (35公分) 觸發煞車
        self.obstacle_too_close = (front_dist < 0.35)

        # 抽樣 72 個雷射點 (每 5 度一點) 傳給網頁畫出 360 度即時環境輪廓
        step = max(1, num_pts // 72)
        sampled_pts = []
        for i in range(0, num_pts, step):
            r = float(ranges[i])
            if 0.18 < r < 3.0 and math.isfinite(r):
                sampled_pts.append([round(float(angles[i]), 2), round(r, 2)])

        with self.frame_lock:
            self.latest_lidar_dict = {
                "front": round(front_dist, 2),
                "left": round(left_dist, 2),
                "right": round(right_dist, 2),
                "back": round(back_dist, 2),
                "danger": self.obstacle_too_close,
                "points": sampled_pts
            }

    def _stream_worker(self):
        """背景工人：獨立負責上傳 YOLO 畫面與光達雷達數據至網頁"""
        last_lidar_send = 0.0
        while rclpy.ok():
            frame_data = None
            lidar_data = None
            now = time.time()

            with self.frame_lock:
                if self.latest_frame_bytes is not None:
                    frame_data = self.latest_frame_bytes
                    self.latest_frame_bytes = None
                if self.latest_lidar_dict is not None and (now - last_lidar_send > 0.1):
                    lidar_data = self.latest_lidar_dict
                    last_lidar_send = now

            if frame_data is not None:
                try:
                    self.session.post('http://127.0.0.1:5000/upload_frame', data=frame_data, timeout=0.05)
                except Exception:
                    pass

            if lidar_data is not None:
                try:
                    resp = self.session.post('http://127.0.0.1:5000/upload_lidar', json=lidar_data, timeout=0.05)
                    if resp.status_code == 200:
                        cfg = resp.json()
                        self.flip_lidar = cfg.get("flip_lidar", False)
                        new_motor_state = cfg.get("motor_enabled", True)
                        # 如果從網頁關閉馬達，立刻送一次煞車
                        if self.motor_enabled and not new_motor_state:
                            self.stop_robot()
                        self.motor_enabled = new_motor_state
                except Exception:
                    pass

            if frame_data is None and lidar_data is None:
                time.sleep(0.01)

    def msg_callback(self, m: msg.Image):
        np_img = np.frombuffer(m.data, dtype=np.uint8).reshape((m.height, m.width, 3)).copy()
        detections = detect(np_img, det_compiled_model)[0]
        boxes = detections.get("det", [])
        
        target_found = False
        best_box = None
        max_area = 0

        for *xyxy, conf, lbl in boxes:
            x1, y1, x2, y2 = xyxy
            area = (x2 - x1) * (y2 - y1)
            if area > max_area:
                max_area = area
                best_box = xyxy
                target_found = True

        current_time = time.time()

        # 只有在 motor_enabled 為 True 時才發送馬達速度
        if self.motor_enabled:
            if target_found:
                self.last_seen_time = current_time 
                x1, y1, x2, y2 = best_box
                cx = (x1 + x2) / 2          
                img_w = np_img.shape[1]     
                center_x = img_w / 2        

                if not self.is_tracking:
                    stop_msg = std_msg.String()
                    stop_msg.data = "STOP"
                    self.cmd_pub.publish(stop_msg)
                    self.is_tracking = True
                    print("\n👀 [哨兵模式] 鎖定目標！開始全速追擊！")

                twist = Twist()
                error_x = center_x - cx     
                twist.angular.z = float(error_x) * 0.003
                
                box_height = y2 - y1
                target_height = 500.0  
                error_distance = target_height - box_height
                
                speed_x = error_distance * 0.003  
                speed_x = max(-0.15, min(0.35, speed_x)) 
                
                twist.linear.x = 0.0 if abs(error_distance) < 40 else float(speed_x)
                
                # 🛡️ 光達防撞攔截
                if self.obstacle_too_close and twist.linear.x > 0:
                    twist.linear.x = 0.0
                    print("⚠️ [光達防撞] 前方 35cm 內有障礙物！強制煞車！")

                self.vel_pub.publish(twist)

            else:
                if self.is_tracking:
                    if current_time - self.last_seen_time < 2.0:
                        twist = Twist()
                        self.vel_pub.publish(twist)
                    else:
                        self.is_tracking = False
                        print("\n❌ [哨兵模式] 目標徹底離開！啟動雷達掃描...")
                else:
                    twist = Twist()
                    twist.angular.z = 0.8  
                    self.vel_pub.publish(twist)

        # 繪製外框、FPS 與防撞狀態
        fps = 1.0 / max(current_time - self.prev_time, 0.0001)
        self.prev_time = current_time
        
        image_with_boxes = draw_results(boxes, np_img, label_map)
        cv2.putText(image_with_boxes, f"FPS: {fps:.1f}", (10, 30), 0, 0.8, (0, 255, 0), 2)
        if self.obstacle_too_close:
            cv2.putText(image_with_boxes, "LIDAR BRAKE!", (10, 65), 0, 0.9, (0, 0, 255), 3)

        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 70]
        _, buffer = cv2.imencode('.jpg', image_with_boxes, encode_param)
        
        with self.frame_lock:
            self.latest_frame_bytes = buffer.tobytes()

def main():
    rclpy.init()
    node = TestDisplayNode()

    # 捕捉關閉訊號：被關閉前強制把車子停下來
    def sig_handler(sig, frame):
        print("\n🛑 收到中斷訊號，正在停止底盤馬達...")
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, sig_handler)
    signal.signal(signal.SIGTERM, sig_handler)

    try:
        rclpy.spin(node)
    finally:
        node.stop_robot()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()