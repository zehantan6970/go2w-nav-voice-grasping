#!/usr/bin/env python3
"""
TTS 預生成腳本 — 在背景預先合成所有導航語音，避免巡航時等待 API。

用法：
  python3 tts_precache.py                    # 全部預生成
  python3 tts_precache.py --check-only       # 只檢查缺少哪些

生成檔案：/tmp/tts_nav_{index}.wav
"""

import json
import os
import subprocess
import sys
import threading
from pathlib import Path

TTS_API_KEY = None
TTS_VOICE = "Cherry"
TTS_SCRIPT = "/tmp/aliyun_tts_bridge.py"

# 所有可能的 TTS 文字與對應 index
# 固定片語（不隨任務點變動）：
FIXED_TTS = [
    (100, "開始抓取"),
    (101, "抓取完成"),
    (102, "抓取失敗"),
    (110, "請伸出你的手,讓我看到你"),
    (111, "說放手我就會把水給你"),
    (112, "這是你的水"),
    (99, "無法到達此點"),  # 備用，主程式用 i*10+6
]

# 任務點名稱（對應 mapmission 7 點）
POINT_NAMES = [
    "櫃子前", "防靜電實驗台", "講台前", "取水點",
    "機械臂桌前", "實驗室中間點", "屏風前",
]


def load_api_key():
    global TTS_API_KEY
    if TTS_API_KEY:
        return
    from dotenv import load_dotenv
    env_path = Path(__file__).resolve().parent / ".env"
    if env_path.exists():
        load_dotenv(env_path)
    TTS_API_KEY = os.getenv("ALIYUN_TTS_API_KEY", "") or os.getenv("ALIYUN_REALTIME_API_KEY", "")


def _djb2(text: str) -> int:
    """DJB2 雜湊，對 UTF-8 bytes 逐 byte 計算（與 C++ demo.cpp 一致）"""
    h = 5381
    for b in text.encode('utf-8'):
        h = ((h << 5) + h) + b
    return h & 0xffffffffffffffff

def wav_path(text: str) -> str:
    """根據文字內容產生快取路徑（DJB2 雜湊，與 C++ demo.cpp 一致）"""
    return f"/tmp/tts_nav_{_djb2(text)}.wav"


def check_cached(text: str) -> bool:
    path = wav_path(text)
    if not os.path.exists(path):
        return False
    size = os.path.getsize(path)
    return size >= 1000


def synthesize(text: str, index: int) -> bool:
    """直接透過 WebSocket 呼叫阿里雲 TTS 合成並存檔"""
    if not TTS_API_KEY:
        print(f"  ⚠️  無 API key，跳過")
        return False
    path = wav_path(text)
    # 內聯合成
    import base64, wave, json
    import websocket
    try:
        ws = websocket.create_connection(
            "wss://dashscope-intl.aliyuncs.com/api-ws/v1/realtime?model=qwen3-tts-flash-realtime",
            header=[f"Authorization: Bearer {TTS_API_KEY}"], timeout=15)
    except Exception as e:
        print(f"  ⚠️  WS連接失敗: {e}")
        return False
    chunks = []
    ws.send(json.dumps({"event_id":"evt_pre","type":"session.update","session":{"voice":TTS_VOICE,"response_format":"pcm","sample_rate":24000,"language_type":"Chinese"}}))
    ws.send(json.dumps({"event_id":"evt_pre","type":"input_text_buffer.append","text":text}))
    ws.send(json.dumps({"event_id":"evt_pre","type":"input_text_buffer.commit"}))
    done = False
    while not done:
        try:
            msg = json.loads(ws.recv())
            t = msg.get("type","")
            if t in ("response.audio.delta","output_audio.delta"):
                d = msg.get("delta") or msg.get("audio")
                if d: chunks.append(base64.b64decode(d))
            elif t in ("response.completed","response.done","session.finished"):
                done = True
        except:
            break
    ws.close()
    pcm = b"".join(chunks)
    if not pcm:
        return False
    with wave.open(path,"wb") as w:
        w.setnchannels(1); w.setsampwidth(2); w.setframerate(24000)
        w.writeframes(pcm)
    return True


def generate_one(text: str, index: int):
    """合成一個，若有快取則跳過"""
    if check_cached(text):
        return
    print(f"  ⏳ [{index:3d}] {text}")
    ok = synthesize(text, index)
    if ok:
        print(f"  ✅ [{index:3d}] {text}")
    else:
        print(f"  ❌ [{index:3d}] 合成失敗")


def generate_all():
    """生成所有 TTS 檔"""
    print("🎧 TTS 預生成開始...")

    # 固定片語
    for idx, text in FIXED_TTS:
        generate_one(text, idx)

    # 固定片語（含變數 i）
    for i in range(7):
        generate_one("接近目標", i * 10 + 3)
        generate_one("即將到達", i * 10 + 4)
        generate_one("馬上到了", i * 10 + 5)
        generate_one("無法到達此點", i * 10 + 6)

    # 任務點名稱相關（正在前往 + 已到達）
    for i, name in enumerate(POINT_NAMES, 0):
        generate_one(f"正在前往{name}", i * 10 + 1)
        generate_one(f"已到達{name}", i * 10 + 2)

    # 統計
    all_texts = [t for _, t in FIXED_TTS]
    for i in range(7):
        for s in ["接近目標", "即將到達", "馬上到了", "無法到達此點"]:
            all_texts.append(s)
    for n in POINT_NAMES:
        all_texts.append(f"正在前往{n}")
        all_texts.append(f"已到達{n}")
    total = len(all_texts)
    cached = sum(1 for t in all_texts if check_cached(t))
    print(f"\n📊 快取: {cached}/{total}")


def check_only():
    """只檢查缺少哪些"""
    missing = []
    for _, text in FIXED_TTS:
        if not check_cached(text):
            missing.append(text)
    for suffix in ["接近目標", "即將到達", "馬上到了", "無法到達此點"]:
        if not check_cached(suffix):
            missing.append(suffix)
    for name in POINT_NAMES:
        for prefix in ["正在前往", "已到達"]:
            t = f"{prefix}{name}"
            if not check_cached(t):
                missing.append(t)
    if missing:
        print(f"缺少 {len(missing)} 個檔:")
        for t in missing:
            print(f"  {t}")
    else:
        print("✅ 全部快取已就緒")
    return missing


def main():
    load_api_key()
    if len(sys.argv) > 1 and sys.argv[1] == "--check-only":
        check_only()
    else:
        generate_all()


if __name__ == "__main__":
    main()
