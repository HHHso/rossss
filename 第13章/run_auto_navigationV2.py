#!/usr/bin/env python3
import os
import signal
import subprocess
import time
import sys

def main():
    print("==================================================")
    print("  ROS 2 Humble - 專題：光達自走車建圖系統   ")
    print("==================================================")

    # 讓使用者選擇建圖模式
    print("請選擇建圖模式：")
    print("[1] 全自動邊界探索 (Explore Lite) ")
    print("[2] 鍵盤手動遙控建圖 (Teleop)  ")
    
    while True:
        mode = input("\n請輸入 1 或 2: ").strip()
        if mode in ['1', '2']:
            break
        print(" 輸入錯誤，請輸入 1 或 2。")

    ros_env = "source /opt/ros/humble/setup.bash && source /home/playrobot/ros2_ws/install/setup.bash"

    print("\n[1/5] 正在啟動自走車硬體與雷射里程計...")
    hardware_cmd = f"{ros_env} && ros2 launch /home/playrobot/Desktop/AMR_範例包/第13章/robot_start.launch.py"
    hardware_proc = subprocess.Popen(hardware_cmd, shell=True, executable='/bin/bash', start_new_session=True)
    time.sleep(5)

    print("[2/5] 正在啟動 SLAM 建圖系統 (slam_toolbox)...")
    slam_cmd = f"{ros_env} && ros2 launch slam_toolbox online_async_launch.py"
    slam_proc = subprocess.Popen(slam_cmd, shell=True, executable='/bin/bash', start_new_session=True)
    time.sleep(3)

    print("[3/5] 正在啟動 Nav2 導航大腦...")
    nav2_params_path = "/home/playrobot/Desktop/AMR_範例包/第14_15章（共用nav）/playrobot_robot_nav2/nav2_params.yaml"
    nav_cmd = f"{ros_env} && ros2 launch nav2_bringup navigation_launch.py params_file:={nav2_params_path}"
    nav_proc = subprocess.Popen(nav_cmd, shell=True, executable='/bin/bash', start_new_session=True)
    time.sleep(5)

    print("[4/5] 正在啟動 RViz2 視覺化監控畫面...")
    rviz_config_path = "/home/playrobot/Desktop/AMR_範例包/第14_15章（共用nav）/playrobot_robot_nav2/slam_and_nav.rviz"
    rviz_cmd = f"{ros_env} && ros2 run rviz2 rviz2 -d {rviz_config_path}"
    rviz_proc = subprocess.Popen(rviz_cmd, shell=True, executable='/bin/bash', start_new_session=True)
    time.sleep(3)

    explore_proc = None

    if mode == '1':
        print("[5/5] 啟動全自動邊界探索 (Explore Lite)...")
        explore_cmd = f"{ros_env} && ros2 launch explore_lite explore.launch.py"
        explore_proc = subprocess.Popen(explore_cmd, shell=True, executable='/bin/bash', start_new_session=True)
        print("\n[系統提示] 全自動環境已就緒！請在 RViz2 給予 2D Pose Estimate。")
    
    elif mode == '2':
        print("[5/5] 準備進入鍵盤遙控模式！")
        print("\n[系統提示] 請確保在 RViz2 給予 2D Pose Estimate 初始定位後，點擊此終端機視窗。")
        print(" 控制方式：使用鍵盤的 i (前進)、, (後退)、j (左轉)、l (右轉)、k (停止)")
        time.sleep(3)

    print("\n🚨 若地圖掃描完畢，請按下 Ctrl + C 結束並儲存。")

    try:
        if mode == '2':
            teleop_cmd = f"{ros_env} && ros2 run teleop_twist_keyboard teleop_twist_keyboard"
            subprocess.run(teleop_cmd, shell=True, executable='/bin/bash')
        else:
            while True:
                time.sleep(1)
                
    except KeyboardInterrupt:
        pass
        
    finally:
        print("\n[系統關閉] 啟動強力清除程序，確保完全停機...")

        # === 關鍵修改 1：先殺掉會發送速度的大腦 ===
        print(">> 1. 正在切斷導航大腦 (防止速度指令覆蓋)...")
        for proc in [nav_proc, explore_proc]:
            if proc:
                try:
                    # 使用 SIGTERM (15) 強制要求退出
                    os.killpg(proc.pid, signal.SIGTERM)
                except:
                    pass
        time.sleep(1.5) # 給大腦一點時間斷線

        # === 關鍵修改 2：大腦死後，再發送煞車指令 ===
        print(">> 2. 發送強力煞車指令...")
        stop_cmd = f"{ros_env} && ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{{linear: {{x: 0.0, y: 0.0, z: 0.0}}, angular: {{x: 0.0, y: 0.0, z: 0.0}}}}'"
        try:
            subprocess.run(stop_cmd, shell=True, executable='/bin/bash', timeout=2, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except:
            pass

        # === 關鍵修改 3：關閉硬體與其他介面 ===
        print(">> 3. 關閉硬體、SLAM 與 RViz2...")
        for proc in [slam_proc, hardware_proc, rviz_proc]:
            if proc:
                try:
                    os.killpg(proc.pid, signal.SIGTERM)
                except:
                    pass
        time.sleep(1.5)

        # === 關鍵修改 4：終極殺陣，保證不留活口 ===
        print(">> 4. 系統底層清理 (終極防線)...")
        os.system("pkill -9 -f ros2")
        os.system("killall -9 _ros2_daemon 2>/dev/null")

        print("\n✅ 自走車已完全強制停止，所有背景節點皆已清除！")
        print("💡 請記得使用 map_saver_cli 儲存你剛建好的地圖。")

if __name__ == '__main__':
    main()