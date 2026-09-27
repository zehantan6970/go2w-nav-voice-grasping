#!/usr/bin/env python3
"""release.py — 举起水瓶并释放（带 J1_OFFSET+验证+重试）
與 3dcatchv5.py 採用相同的 arm 控制模式

流程：
  1. 举高 + 夹紧
  2. 播放 arrive.mp3
  3. 松开夹爪（保持举高姿态）
  4. 播放「请取走水瓶」TTS
  5. 等待 2s 讓人取走水瓶
  6. 回到 HOME [90,-75,75]
"""
import subprocess, time, sys, os, json, numpy as np

BASE = "/home/unitree/D1sdk/build"
BRIDGE = f"{BASE}/python_bridge"
LD_PATH = "/home/unitree/cyclonedds/install/lib:/usr/local/lib"

J1_OFFSET = 90.0      # 固定J1偏移，send()自動加
HOME_DEG = [0.0, -75.0, 75.0, 0.0, 0.0, 0.0]  # send()會加J1_OFFSET→[90,-75,75,...]
GRIP_OPEN = 55.0
GRIP_CLOSE = -5.0
SETTLE = 6.0          # 等待關節穩定時間（大幅移動需更久，與 arm_home.py 一致）

def read_angles():
    try:
        r = subprocess.run(["timeout","2",f"{BASE}/get_arm_joint_angle"],
                          capture_output=True, text=True, timeout=3)
        for line in r.stdout.split('\n'):
            if 'armFeedback_data:' in line and '"funcode":1' in line:
                d = json.loads(line.split('armFeedback_data:')[1].strip()).get('data',{})
                if 'angle0' in d:
                    return [d[f'angle{i}'] for i in range(7)]
    except: pass
    return [0.0]*7

def send(jnt_deg, grip=0.0, use_j1_offset=True):
    """發送角度
    use_j1_offset=True → 自動加 J1_OFFSET（HOME/歸位用）
    use_j1_offset=False → 不偏移（舉高手臂時維持當前 J1）"""
    arr = list(jnt_deg[:6]) + [grip]
    if use_j1_offset:
        arr[0] += J1_OFFSET
    env = os.environ.copy()
    env["LD_LIBRARY_PATH"] = LD_PATH
    subprocess.run([BRIDGE] + [f"{v:.2f}" for v in arr], env=env,
                   capture_output=True, timeout=10)

def step_joint(jnt_deg, grip=0.0, label="", max_retry=3, use_j1_offset=True):
    cmd = np.array(jnt_deg[:6], dtype=float)
    for attempt in range(max_retry + 1):
        send(jnt_deg, grip, use_j1_offset)
        time.sleep(SETTLE)
        act = read_angles()
        act_adj = np.array(act[:6], dtype=float)
        if use_j1_offset:
            act_adj[0] -= J1_OFFSET  # 有偏移才減
        err = max(abs(cmd - act_adj))
        if err < 5.0:
            if attempt > 0:
                print(f"  ✅ {label}: 第{attempt}次重试后到位 err={err:.1f}°")
            else:
                print(f"  ✅ {label}: err={err:.1f}°")
            return True
        elif attempt < max_retry:
            print(f"  ⚠️ {label}: err={err:.1f}°，第{attempt+1}次重试...")
            for _ln in os.popen('pgrep -f get_arm_joint_angle').read().strip().split('\n'):
                if _ln.strip():
                    try: os.kill(int(_ln.strip()), 9)
                    except: pass
        else:
            print(f"  ❌ {label}: 重试{max_retry}次失败 err={err:.1f}°")
    return False

def play_wav(wav_path):
    """播放 WAV 文件"""
    if os.path.exists(wav_path):
        subprocess.run(["paplay", wav_path],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

def play_tts_cached(text):
    """播放預生成的 TTS 快取（DJB2 雜湊，與 tts_precache.py / demo.cpp 一致）"""
    h = 5381
    for c in text:
        h = ((h << 5) + h) + ord(c)
    wav = f"/tmp/tts_nav_{h & 0xFFFFFFFF}.wav"
    play_wav(wav)

print("\n=== 机械臂释放 ===")

LIFT_DEG = [0.0, 30.0, -50.0, 0.0, 0.0, 0.0]

# [1] 举高 + 夹紧
print("\n[1/4] 举高 + 夹紧")
step_joint(LIFT_DEG, GRIP_CLOSE, "举高", use_j1_offset=False)

# 播放 arrive.mp3
AUDIO_DEVICE = "plughw:0,0"
AUDIO_PATH = f"{os.path.dirname(os.path.abspath(__file__))}/music/arrive.mp3"
if os.path.exists(AUDIO_PATH):
    os.system(f"ffmpeg -i {AUDIO_PATH} -f wav - 2>/dev/null | "
              f"PULSE_SERVER=none aplay -D {AUDIO_DEVICE} 2>/dev/null")

# [2] 松开夹爪（保持举高姿态）
print("\n[2/4] 松开夹爪")
step_joint(LIFT_DEG, GRIP_OPEN, "释放", use_j1_offset=False)

# ⏳ 播放「请取走水瓶」TTS（此時手臂舉高、夾爪張開）
print("\n[3/4] 播放「请取走水瓶」")
play_tts_cached("这是你的水")
time.sleep(2.0)

# [4] 回到 HOME
print("\n[4/4] 回到 HOME")
step_joint(HOME_DEG, GRIP_OPEN, "归位")

print("\n✅ 释放完成")
