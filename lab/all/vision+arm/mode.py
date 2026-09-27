#!/usr/bin/env python3
"""切换输入通道: python3 mode.py all|terminal|feishu|voice"""
import json, sys, os
p = os.path.expanduser("~/.openclaw/workspace/input_mode.json")
m = {"all":"全部","terminal":"终端","feishu":"飞书","voice":"语音"}
if len(sys.argv) < 2:
    c = json.load(open(p)).get("active_channel","all")
    print(f"当前: {c} ({m.get(c,'?')})")
else:
    mode = sys.argv[1]
    if mode not in m: print(f"可选: {list(m.keys())}"); sys.exit(1)
    json.dump({"active_channel":mode,"channels":m}, open(p,"w"))
    print(f"已切换: {mode} ({m[mode]})")
