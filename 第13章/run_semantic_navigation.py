#!/usr/bin/env python3
import os
import signal
import subprocess
import time
import sys

def main():
    print("==================================================")
    print("  ROS 2 Humble - 已知地圖導航啟動程式  ")
    print("==================================================")

    ros_env = "source /opt/ros/humble/setup.bash && source /home/playrobot/ros2_ws/install/setup.bash"

    print("[1/3] 啟動自走車硬體與雷射里程計...")
    hardware_cmd = f"{ros_env} && ros2 launch /home/playrobot/Desktop/AMR_範例包/第13章/robot_start.launch.py"
    hardware_proc = subprocess.Popen(hardware_cmd, shell=True, executable='/bin/bash', start_new_session=True)
    time.sleep(5)

    print("[2/3] 啟動導航大腦 (載入你剛建好的 lab_map)...")
    # 這裡加上了 map:=/home/playrobot/Desktop/lab_map.yaml
    nav_cmd = f"{ros_env} && ros2 launch playrobot_robot_nav2 playrobot_navigation.launch.py map:=/home/playrobot/Desktop/lab_map.yaml"
    nav_proc = subprocess.Popen(nav_cmd, shell=True, executable='/bin/bash', start_new_session=True)
    time.sleep(6)

    print("[3/3] 啟動 RViz2 視覺化監控畫面...")
    rviz_config_path = "/home/playrobot/Desktop/AMR_範例包/第14_15章（共用nav）/playrobot_robot_nav2/slam_and_nav.rviz"
    rviz_cmd = f"{ros_env} && ros2 run rviz2 rviz2 -d {rviz_config_path}"
    rviz_proc = subprocess.Popen(rviz_cmd, shell=True, executable='/bin/bash', start_new_session=True)

    print("\n✅ 系統啟動完畢！")
    print("💡 請在 RViz2 中點擊上方『2D Pose Estimate』給予車子初始定位。")
    print("若要關閉系統，請按 Ctrl+C。")

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n正在關閉系統...")
        for proc in [rviz_proc, nav_proc, hardware_proc]:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGINT)
            except ProcessLookupError:
                pass
        time.sleep(2)
        os.system("pkill -f ros2")
        print("✅ 關機完成。")

if __name__ == '__main__':
    main()