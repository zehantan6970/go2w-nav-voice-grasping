#!/usr/bin/env python3
"""
地圖視覺化更新器 — 定期更新網頁前端所需的地圖資訊
由 systemd unitree-mapvis.service 在啟動時一併執行

功能：
1. 將目前的地圖 YAML + PGM 複製到 HTTP 服務目錄
2. 產生 mapinfo.json（地圖參數）
3. 產生 tasks.json（任務點）
4. 更新 robot_pose.json symlink
"""

import json
import os
import shutil
import time

MAP_DIR = "/home/unitree/lab/map"
WWW_DIR = "/home/unitree/lab/all"


def update():
    """更新所有前端地圖檔案"""
    # 找到最新的 PGM + YAML（即 DEMO 當前載入的地圖）
    # 從 mapinfo.json 的 symlink 或直接掃描
    pgm_files = [f for f in os.listdir(MAP_DIR) if f.endswith(".pgm")]
    yaml_files = [f for f in os.listdir(MAP_DIR) if f.endswith(".yaml")]

    # 找最新修改的 YAML（代表 DEMO 剛載入的地圖）
    if yaml_files:
        latest_yaml = max(yaml_files, key=lambda f: os.path.getmtime(os.path.join(MAP_DIR, f)))
        map_name = latest_yaml.replace(".yaml", "")

        # 地圖參數
        yaml_path = os.path.join(MAP_DIR, latest_yaml)
        with open(yaml_path) as f:
            yaml_lines = f.readlines()
        info = {"name": map_name, "resolution": 0.05, "origin": [0, 0, 0]}
        for line in yaml_lines:
            if ":" in line:
                k, v = line.split(":", 1)
                k = k.strip()
                v = v.strip()
                if k == "resolution":
                    info["resolution"] = float(v)
                elif k == "origin":
                    info["origin"] = [float(x) for x in v.strip("[]").split(",")]
                elif k == "image":
                    info["image"] = v

        # PGM → PNG
        pgm_path = os.path.join(MAP_DIR, f"{map_name}.pgm")
        if os.path.exists(pgm_path):
            try:
                from PIL import Image
                img = Image.open(pgm_path)
                img.save(os.path.join(WWW_DIR, "map.png"))
                info["width"], info["height"] = img.size
            except:
                pass

        # 寫入 mapinfo.json
        with open(os.path.join(WWW_DIR, "mapinfo.json"), "w") as f:
            json.dump(info, f)

        # 任務點
        task_path = os.path.join(MAP_DIR, f"{map_name}_task.json")
        if os.path.exists(task_path):
            shutil.copy2(task_path, os.path.join(WWW_DIR, "tasks.json"))
        else:
            # 清除舊的
            tp = os.path.join(WWW_DIR, "tasks.json")
            if os.path.exists(tp):
                os.remove(tp)

    # pose symlink
    pose_link = os.path.join(WWW_DIR, "robot_pose.json")
    if not os.path.exists(pose_link):
        try:
            os.symlink("/tmp/robot_pose.json", pose_link)
        except:
            pass


def main():
    update()
    # 每 3 秒檢查更新
    last_mod = 0
    while True:
        # 檢查 YAML 檔是否有變化
        for f in os.listdir(MAP_DIR):
            if f.endswith(".yaml"):
                mtime = os.path.getmtime(os.path.join(MAP_DIR, f))
                if mtime > last_mod:
                    last_mod = mtime
                    update()
        time.sleep(3)


if __name__ == "__main__":
    main()
