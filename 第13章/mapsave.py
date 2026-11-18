#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import subprocess

def main():
    print("==================================================")
    print("       ROS 2 專題：一鍵地圖儲存工具 (Map Saver)     ")
    print("==================================================")

    # 1. 讓使用者輸入自訂的地圖名稱 (預設為 lab_map)
    map_name = input("👉 請輸入地圖名稱 (直接按 Enter 預設為 'lab_map'): ").strip()
    if not map_name:
        map_name = "lab_map"

    # 2. 自動抓取使用者的桌面路徑
    desktop_path = os.path.join(os.path.expanduser("~"), "Desktop")
    save_path = os.path.join(desktop_path, map_name)

    print(f"\n⏳ 準備將地圖儲存至桌面：")
    print(f"   - {save_path}.yaml")
    print(f"   - {save_path}.pgm\n")

    # 3. 組合 ROS 2 儲存指令
    ros_env = "source /opt/ros/humble/setup.bash"
    save_cmd = f"{ros_env} && ros2 run nav2_map_server map_saver_cli -f {save_path}"

    # 4. 執行指令並捕捉結果
    try:
        print("正在呼叫 map_saver_cli 節點，請稍候...")
        # 執行指令，check=True 代表如果發生錯誤會直接跳到 except
        subprocess.run(save_cmd, shell=True, executable='/bin/bash', check=True)
        
        print("\n🎉 地圖儲存成功！")
        print(f"請到桌面確認是否有看到 {map_name}.yaml 與 {map_name}.pgm 這兩個檔案。")
        
    except subprocess.CalledProcessError:
        print("\n❌ 儲存失敗！")
        print("💡 錯誤排解：請確認你的 SLAM 建圖系統 (slam_toolbox 或 Gmapping) 目前正在執行中！")

if __name__ == '__main__':
    main()