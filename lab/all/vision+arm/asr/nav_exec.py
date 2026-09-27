#!/usr/bin/env python3
"""
導航指令執行器 — 由 voice_controller 解析意圖後觸發
直接寫任務檔 + pexpect 控制 DEMO，不走 OpenClaw agent 子進程

輸入格式（JSON 字串）：
  {"action": "grasp|navigate", "location": "取水點|講台前", "target": "水"}
  {"action": "grasp", "location": "取水點", "target": "茶"}
  {"action": "navigate", "location": "櫃子前", "target": ""}
  {"action": "release", "location": "屏風前", "target": ""}

輸出：exit(0)=成功, exit(1)=失敗
"""

import json
import os
import subprocess
import sys
import time
from pathlib import Path

# ─── 路徑 ───
BAK_FILE = "/home/unitree/lab/map/mapmission_task.json.BAK"
TASK_FILE = "/home/unitree/lab/map/mapmission_task.json"
DEMO_BIN = "/unitree/module/unitree_slam/example/build/demowater"
ARM_HOME_PY = "/home/unitree/lab/all/vision+arm/arm_home.py"
GRASP_SCRIPT = "/home/unitree/lab/all/vision+arm/3dcatchv5.py"
HANDOVER_SCRIPT = "/home/unitree/lab/all/vision+arm/handover/handover_listener.py"
LD_PATH = "/home/unitree/cyclonedds/install/lib:/usr/local/lib"

# ─── 任務點名稱 → BAK 索引對照 ───
LOCATION_TO_INDEX = {
    "櫃子前": 0, "防靜電實驗台": 1, "講台前": 2, "取水點": 3,
    "機械臂桌前": 4, "實驗室中間點": 5, "屏風前": 6,
}

# ─── 飲料名稱 → drink id ───
DRINK_MAP = {"水": 0, "可樂": 1, "茶": 2, "王老吉": 3}


# ─── 預建任務檔對照表 ───
# 飲料+目的地 → 檔名
DELIVERY_FILES = {
    "水": {
        "講台前": "mapmission_water_to_frontdesk.json",
        "實驗室中間點": "mapmission_water_to_midlab.json",
        "屏風前": "mapmission_water_to_screen.json",
        "機械臂桌前": "mapmission_water_to_armdesk.json",
    },
    "茶": {
        "講台前": "mapmission_tea_to_frontdesk.json",
        "實驗室中間點": "mapmission_tea_to_midlab.json",
        "屏風前": "mapmission_tea_to_screen.json",
        "機械臂桌前": "mapmission_tea_to_armdesk.json",
    },
}


def select_task_file(parsed: dict):
    """根據解析意圖選擇預建任務檔，若無匹配則回傳 None"""
    action = parsed.get("action", "")
    location = parsed.get("location", "")
    target = parsed.get("target", "")

    # 複合指令：grasp|release → 從預建檔選
    if "|" in action:
        locations = location.split("|")
        if len(locations) >= 2:
            drink_name = target if target else "水"
            dest = locations[1]  # 第二個地點是送達點
            mapping = DELIVERY_FILES.get(drink_name)
            if mapping:
                fname = mapping.get(dest)
                if fname:
                    path = os.path.join(os.path.dirname(BAK_FILE), fname)
                    if os.path.exists(path):
                        print(f"[NavExec] ✅ 使用預建檔: {fname}")
                        return path
                    print(f"[NavExec] ⚠️ 預建檔不存在: {fname}")
    return None


def copy_task_file(src_path: str) -> bool:
    """複製預建任務檔到 mapmission_task.json"""
    import shutil
    try:
        shutil.copy2(src_path, TASK_FILE)
        print(f"[NavExec] ✅ 已複製 {src_path} → {TASK_FILE}")
        return True
    except Exception as e:
        print(f"[NavExec] ❌ 複製失敗: {e}")
        return False


def build_task(parsed: dict) -> bool:
    """根據解析意圖準備任務檔（優先使用預建檔）"""
    # 1. 嘗試用預建檔
    prebuilt = select_task_file(parsed)
    if prebuilt:
        return copy_task_file(prebuilt)

    # 2. 後備：動態建立
    action = parsed.get("action", "")
    location = parsed.get("location", "")
    target = parsed.get("target", "")
    drink_id = DRINK_MAP.get(target, 0)
    locations = location.split("|") if "|" in location else [location]
    actions = action.split("|") if "|" in action else [action]

    with open(BAK_FILE) as f:
        bak = json.load(f)
    bak_pts = bak["points"]
    task_pts = []

    for i, act in enumerate(actions):
        loc = locations[i] if i < len(locations) else locations[0]
        idx = LOCATION_TO_INDEX.get(loc)
        if idx is None:
            print(f"[NavExec] ❌ 未知地點: {loc}")
            return False

        pt = dict(bak_pts[idx])
        pt["mode"] = 1 if idx == 3 else pt.get("mode", 0)

        if act == "grasp":
            pt["action"] = "grasp"
            pt["drink"] = drink_id
        elif act == "release":
            pt["action"] = "release"
        else:
            pt["action"] = ""

        task_pts.append(pt)

    task = {"count": len(task_pts), "points": task_pts}
    with open(TASK_FILE, "w") as f:
        json.dump(task, f, indent=4)

    names = [p["tts_text"] for p in task_pts]
    print(f"[NavExec] ✅ 任務檔已動態建立: {' → '.join(names)}")
    return True


def run_demo():
    """啟動 DEMO + pexpect 自動化（無超時，等到結束）"""
    import pexpect

    env = os.environ.copy()
    env["LD_LIBRARY_PATH"] = LD_PATH

    # timeout=None → 永不超時
    child = pexpect.spawn(DEMO_BIN, ["eth0"], encoding="utf-8",
                          timeout=None, env=env,
                          logfile=sys.stdout,
                          codec_errors="replace")

    # 所有 ANSI 跳脫碼轉換為純文字
    import re
    def clean_ansi(text: str) -> str:
        return re.sub(r'\x1b\[[0-9;]*[a-zA-Z]', '', text)

    try:
        # 1. 選單
        child.expect("退出程序", timeout=30)
        child.sendline("a")

        # 2. 地圖選擇
        child.expect("选择地图", timeout=15)
        child.sendline("mapmission")  # 直接輸入名稱，避免索引漂移
        print("[NavExec] 等待重定位...")

        # 3. 重定位（最多重試 3 次）
        for attempt in range(3):
            idx = child.expect(["重定位成功", "重定位失败"], timeout=120)
            if idx == 0:
                print(f"[NavExec] ✅ 重定位成功 (第{attempt+1}次)")
                break
            print(f"[NavExec] 重定位失敗 (第{attempt+1}次)，重試...")
            child.sendline("2")  # 當前位姿重試
        else:
            print("[NavExec] ❌ 重定位失敗")
            child.close()
            return False

        # 4. 啟動巡航
        print("[NavExec] 啟動巡航（無超時，等他完成）...")
        child.sendline("c")

        # 5. 等待巡航結束（無超時）
        # 監控多種可能的結束文字
        idx = child.expect(["单次巡航任务结束", "循环巡航任务结束",
                            pexpect.EOF], timeout=None)

        if idx in (0, 1):
            print("[NavExec] ✅ 任務完成")
            child.close()
            return True
        else:
            print("[NavExec] DEMO 意外結束")
            child.close()
            return False

    except Exception as e:
        print(f"[NavExec] ❌ 異常: {e}")
        child.close()
        return False


def main():
    if len(sys.argv) < 2:
        print("[NavExec] 用法: python3 nav_exec.py '<action>|<location>|<target>'")
        print("  或: python3 nav_exec.py --json '{\"action\":\"...\",\"location\":\"...\",\"target\":\"...\"}'")
        sys.exit(1)

    if sys.argv[1] == "--json":
        parsed = json.loads(sys.argv[2])
    else:
        parts = sys.argv[1].split("|")
        parsed = {
            "action": parts[0] if len(parts) > 0 else "",
            "location": parts[1] if len(parts) > 1 else "",
            "target": parts[2] if len(parts) > 2 else "",
        }

    print(f"[NavExec] 執行: {parsed}")

    # 0. TTS 預生成（背景）
    precache_script = os.path.join(os.path.dirname(__file__), "tts_precache.py")
    if os.path.exists(precache_script):
        subprocess.Popen(
            ["python3", precache_script],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        )

    # 1. 機械臂歸位
    print("[NavExec] 機械臂歸位...")
    arm_home_cmd = f"LD_LIBRARY_PATH={LD_PATH} python3 {ARM_HOME_PY}"
    os.system(arm_home_cmd)

    # 2. 寫任務檔
    if not build_task(parsed):
        sys.exit(1)

    # 3. 啟動 DEMO
    success = run_demo()

    # 4. 歸位任務檔
    with open(BAK_FILE) as f:
        bak_full = json.load(f)
    with open(TASK_FILE, "w") as f:
        json.dump(bak_full, f, indent=4)
    print("[NavExec] ✅ 任務檔已恢復至 7 點")

    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
