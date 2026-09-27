#!/usr/bin/env python3
"""输入通道状态机"""
import json, os, sys

PATH = os.path.expanduser("~/.openclaw/workspace/input_mode.json")

def get_mode():
    try:
        with open(PATH) as f:
            return json.load(f).get("active_channel", "all")
    except:
        return "all"

def check_block(channel_label):
    """检查当前通道是否被屏蔽，屏蔽则返回True"""
    mode = get_mode()
    if mode == "all":
        return False
    # channel_label: webchat/terminal, feishu, voice
    ch_map = {"webchat":"terminal", "terminal":"terminal", "feishu":"feishu", "voice":"voice"}
    ch = ch_map.get(channel_label, channel_label)
    blocked = ch != mode
    if blocked:
        print(f"[mode] 屏蔽: {ch} (当前模式={mode})")
    return blocked

if __name__ == "__main__":
    if len(sys.argv) > 1:
        mode = sys.argv[1]
        modes = {"all":"全部","terminal":"终端","feishu":"飞书","voice":"语音"}
        if mode not in modes:
            print(f"可选: {list(modes.keys())}"); sys.exit(1)
        json.dump({"active_channel":mode,"channels":modes,"last_set":__import__('datetime').datetime.now().isoformat()},
                  open(PATH,"w"), indent=2)
        print(f"已切换: {mode} ({modes[mode]})")
    else:
        c = json.load(open(PATH)).get("active_channel","all")
        print(f"当前: {c}")
