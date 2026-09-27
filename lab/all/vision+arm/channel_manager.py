#!/usr/bin/env python3
"""
通道管理器 — 根據 input_mode.json 啟停飛書

運作: 監聽模式文件 → 修改 openclaw.json → 重啟 openclaw
"""
import json, os, time, signal, sys

MODE_PATH = os.path.expanduser("~/.openclaw/workspace/input_mode.json")
CONFIG_PATH = os.path.expanduser("~/.openclaw/openclaw.json")
PID_FILE = "/tmp/channel_mgr.pid"

def set_feishu(enabled):
    with open(CONFIG_PATH) as f:
        cfg = json.load(f)
    old = cfg.get("plugins", {}).get("entries", {}).get("feishu", {}).get("enabled", True)
    if old == enabled:
        return False  # 没变化
    if "plugins" not in cfg: cfg["plugins"] = {}
    if "entries" not in cfg["plugins"]: cfg["plugins"]["entries"] = {}
    if "feishu" not in cfg["plugins"]["entries"]: cfg["plugins"]["entries"]["feishu"] = {}
    cfg["plugins"]["entries"]["feishu"]["enabled"] = enabled
    with open(CONFIG_PATH, "w") as f:
        json.dump(cfg, f, indent=2)
    return True

def restart_openclaw():
    os.system("pkill -f openclaw 2>/dev/null")
    time.sleep(2)
    os.system("openclaw &")
    print("[管理器] openclaw 已重啟")

def write_pid():
    with open(PID_FILE, "w") as f:
        f.write(str(os.getpid()))

def cleanup(s, f):
    sys.exit(0)

if __name__ == "__main__":
    signal.signal(signal.SIGTERM, cleanup)
    signal.signal(signal.SIGINT, cleanup)
    write_pid()
    last_mode = ""
    modes = {"all":True, "feishu":True, "terminal":False, "voice":False}
    print("[管理器] 啟動，監控模式變化...")
    try:
        while True:
            time.sleep(2)
            try:
                m = json.load(open(MODE_PATH)).get("active_channel","all")
            except:
                m = "all"
            if m != last_mode:
                want = modes.get(m, True)
                print(f"[管理器] 模式: {m} → 飛書={'啟用' if want else '停用'}")
                if set_feishu(want):
                    print("[管理器] config已更新，重啟openclaw...")
                    restart_openclaw()
                last_mode = m
    except KeyboardInterrupt:
        pass
