#!/usr/bin/env python3
"""
手部跟随监听器 (Handover Listener)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

手部跟随（HandOver）期间的语音监听器。

启动顺序:
  1. 启动 go2_gateway.py (HTTP 网关)
  2. 启动 handover_main.py (手部骨骼追踪 + 机器狗跟随)
  3. 持续语音监听 "把水给我" 等关键字
  4. 监听到 → 运行 release.py (松开机械臂)
  5. 语音播报确认，然后退出

用法:
  python3 handover_listener.py              # 标准模式
  python3 handover_listener.py --show       # 显示 CAM1 画面
  python3 handover_listener.py --no-move    # 仅检测不移动

依赖:
  websocket-client webrtcvad requests numpy python-dotenv
"""

from __future__ import annotations

import base64
import io
import json
import os
import queue
import shlex
import signal
import subprocess
import sys
import threading
import time
import uuid
import wave
from pathlib import Path

import websocket
from dotenv import load_dotenv
import webrtcvad
import numpy as np

# ─────────── 路径 ───────────
ROOT_DIR = Path(__file__).resolve().parent
ENV_PATH = ROOT_DIR.parent / "asr" / ".env"
load_dotenv(ENV_PATH)

BASE = Path(__file__).resolve().parent.parent.parent  # /home/unitree/lab/all
GATEWAY_PATH = BASE / "go2_gateway.py"
HANDOVER_PATH = BASE / "vision+arm" / "handover" / "handover_main.py"
RELEASE_PATH = BASE / "vision+arm" / "release.py"

# ─────────── 阿里云 ASR / TTS ───────────
ASR_API_KEY = os.getenv("ALIYUN_REALTIME_API_KEY", "") or os.getenv("DASHSCOPE_API_KEY", "")
TTS_API_KEY = os.getenv("ALIYUN_TTS_API_KEY", "") or ASR_API_KEY

ASR_URL = os.getenv("ALIYUN_REALTIME_BASE_URL", "wss://dashscope.aliyuncs.com/api-ws/v1/realtime")
ASR_MODEL = os.getenv("ALIYUN_REALTIME_MODEL", "qwen3-asr-flash-realtime")
TTS_URL = os.getenv("ALIYUN_TTS_BASE_URL", "wss://dashscope.aliyuncs.com/api-ws/v1/realtime")
TTS_MODEL = os.getenv("ALIYUN_TTS_MODEL", "qwen3-tts-flash-realtime")
TTS_VOICE = os.getenv("ALIYUN_TTS_VOICE", "Cherry")

# ─────────── 音讯 ───────────
RECORD_RATE = 16000
RECORD_CHANNELS = 1
RECORD_WIDTH = 2
FRAME_MS = 20
CHUNK = int(RECORD_RATE * RECORD_WIDTH * RECORD_CHANNELS * FRAME_MS / 1000)

RECORD_DEVICE = os.getenv("ALSA_RECORD_DEVICE", "plughw:0,0")
PLAYBACK_DEVICE = os.getenv("ALSA_PLAYBACK_DEVICE", "default")

# ─────────── 释放关键字 ───────────
RELEASE_KEYWORDS = ["把水给我", "給水", "把水給我", "松開", "放开", "打開",
                    "给我水", "放下", "give me", "release", "手"]

# ─────────── 工具函数 ───────────
def _eid() -> str:
    return uuid.uuid4().hex[:12]

def pkill(name: str):
    """杀死指定名称的进程"""
    subprocess.run(["pkill", "-f", name],
                   timeout=3, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def release_audio():
    """暫停 PulseAudio 以釋放 ALSA 設備 (plughw:0,0)
    用 systemctl --user stop 停掉服務+socket，避免 socket 自動重啟"""
    print("🎤 [Audio] 暫停 PulseAudio (systemctl stop)...")
    subprocess.run(["systemctl", "--user", "stop", "pulseaudio.service", "pulseaudio.socket"],
                   timeout=10, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    # 補殺殘留 process
    subprocess.run(["pulseaudio", "--kill"], timeout=3,
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(0.5)


def resume_audio():
    """重啟 PulseAudio (systemctl start)"""
    subprocess.run(["systemctl", "--user", "start", "pulseaudio.service"], timeout=10,
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(0.5)
    print("🎤 [Audio] PulseAudio 已重啟")


def _pkill_safe(pattern: str):
    """安全殺進程：用 pgrep 先找 PID，再用 kill 逐一終止
    避免 pkill -f 因匹配自己而自殺的 bug"""
    try:
        r = subprocess.run(["pgrep", "--full", pattern], capture_output=True, text=True, timeout=5)
        if r.returncode != 0 or not r.stdout.strip():
            return
        for pid_str in r.stdout.strip().split():
            try:
                pid = int(pid_str)
                subprocess.run(["kill", str(pid)], timeout=3,
                               stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            except (ValueError, subprocess.TimeoutExpired):
                pass
    except subprocess.TimeoutExpired:
        pass


def release_camera():
    """釋放 RealSense 鏡頭 (殺掉所有可能佔用鏡頭的進程)"""
    print("📷 [Camera] 釋放鏡頭...")
    _pkill_safe("video_publisher")
    _pkill_safe("cam_transfer")
    _pkill_safe("cam1_transfer")
    _pkill_safe("cam2_transfer")


def resume_camera():
    """重啟 video_publisher (ROS2 camera)"""
    script = "/home/unitree/scripts/start_camera.sh"
    if os.path.exists(script):
        subprocess.Popen([script], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        print("📷 [Camera] video_publisher 已重啟")

def start_gateway():
    """启动 go2_gateway (端口 8520)"""
    print("🔌 [Gateway] 清理旧网关...")
    pkill("go2_gateway")
    time.sleep(0.5)
    print("🔌 [Gateway] 启动新网关...")
    subprocess.Popen(
        ["python3", str(GATEWAY_PATH)],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
    )
    time.sleep(3)
    print("🔌 [Gateway] ✅ 就绪")

def start_handover(show: bool = False, no_move: bool = False):
    """启动 handover_main (背景进程)"""
    print("🖐️ [HandOver] 启动手部跟随...")
    cmd = ["python3", str(HANDOVER_PATH)]
    if show:
        cmd.append("--show")
    if no_move:
        cmd.append("--no-move")
    proc = subprocess.Popen(
        cmd,
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
    )
    print(f"🖐️ [HandOver] ✅ 已启动 (PID: {proc.pid})")
    return proc

def stop_handover(proc):
    """停止 handover"""
    if proc and proc.poll() is None:
        proc.terminate()
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()
    pkill("handover_main")

def run_release():
    """执行 release.py — 松开机械臂"""
    print("🤖 [Release] 松开机械臂...")
    try:
        r = subprocess.run(["python3", str(RELEASE_PATH)],
                           timeout=60, capture_output=True, text=True)
        if r.returncode == 0:
            print("🤖 [Release] ✅ 完成")
        else:
            print(f"🤖 [Release] ⚠️ 返回码: {r.returncode}")
            if r.stderr:
                for line in r.stderr.strip().split("\n")[-3:]:
                    print(f"  {line}")
    except subprocess.TimeoutExpired:
        print("🤖 [Release] ⏱️ 超时")
    except Exception as e:
        print(f"🤖 [Release] ❌ 出错: {e}")

# ─────────── TTS ───────────
def tts_synthesize(text: str) -> bytes:
    try:
        ws = websocket.create_connection(
            f"{TTS_URL}?model={TTS_MODEL}",
            header=[f"Authorization: Bearer {TTS_API_KEY}"],
            timeout=15,
        )
        chunks: list[bytes] = []
        ws.send(json.dumps({
            "event_id": _eid(), "type": "session.update",
            "session": {
                "voice": TTS_VOICE, "response_format": "pcm",
                "sample_rate": RECORD_RATE, "language_type": "Chinese",
            },
        }))
        ws.send(json.dumps({"event_id": _eid(), "type": "input_text_buffer.append", "text": text}))
        ws.send(json.dumps({"event_id": _eid(), "type": "input_text_buffer.commit"}))
        while True:
            msg = json.loads(ws.recv())
            t = msg.get("type", "")
            if t in ("response.audio.delta", "output_audio.delta"):
                d = msg.get("delta") or msg.get("audio")
                if d:
                    chunks.append(base64.b64decode(d))
            elif t in ("response.completed", "response.done", "session.finished"):
                break
        ws.close()
        return b"".join(chunks)
    except Exception as e:
        print(f"⚠️  TTS 合成失败: {e}")
        return b""

def play_pcm(pcm_bytes: bytes):
    if not pcm_bytes:
        return
    buf = io.BytesIO()
    with wave.open(buf, "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(RECORD_RATE)
        w.writeframes(pcm_bytes)
    tmp = f"/tmp/handover_tts_{uuid.uuid4().hex[:8]}.wav"
    with open(tmp, "wb") as f:
        f.write(buf.getvalue())
    try:
        subprocess.run(["aplay", "-D", PLAYBACK_DEVICE, tmp, "-q"],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    finally:
        try: os.remove(tmp)
        except OSError: pass

def speak(text: str):
    threading.Thread(target=lambda: play_pcm(tts_synthesize(text)), daemon=True).start()

# ─────────── ASR 客户端 ───────────
class AsrClient:
    def __init__(self):
        self.ws: websocket.WebSocket | None = None
        self._audio_queue: queue.Queue[bytes] = queue.Queue()
        self._partial = ""
        self._final = ""
        self._completed = False
        self._closed = True
        self._sender: threading.Thread | None = None
        self._receiver: threading.Thread | None = None

    def start(self) -> bool:
        self._partial = ""
        self._final = ""
        self._completed = False
        self._closed = False
        self._audio_queue = queue.Queue()
        return self._connect()

    def _connect(self) -> bool:
        try:
            self.ws = websocket.create_connection(
                f"{ASR_URL}?model={ASR_MODEL}",
                header=[f"Authorization: Bearer {ASR_API_KEY}"],
                timeout=8,
            )
            if not self._sender or not self._sender.is_alive():
                self._sender = threading.Thread(target=self._sender_loop, daemon=True)
                self._sender.start()
            if not self._receiver or not self._receiver.is_alive():
                self._receiver = threading.Thread(target=self._receiver_loop, daemon=True)
                self._receiver.start()
            return True
        except Exception as e:
            print(f"❌ ASR 连线失败: {e}")
            return False

    def _reconnect(self):
        self.close()
        time.sleep(0.3)
        self.start()

    def _sender_loop(self):
        while not self._closed:
            try:
                chunk = self._audio_queue.get(timeout=0.05)
            except queue.Empty:
                continue
            if self.ws and self.ws.connected:
                try:
                    payload = base64.b64encode(chunk).decode("utf-8")
                    self.ws.send(json.dumps({
                        "event_id": _eid(),
                        "type": "input_audio_buffer.append",
                        "audio": payload,
                    }))
                except Exception:
                    self._reconnect()
                    return

    def _receiver_loop(self):
        while not self._closed:
            if not self.ws or not self.ws.connected:
                time.sleep(0.1)
                continue
            try:
                msg = json.loads(self.ws.recv())
            except Exception:
                continue
            t = msg.get("type", "")
            if t == "conversation.item.input_audio_transcription.text":
                text = f"{msg.get('text', '')}{msg.get('stash', '')}".strip()
                if text:
                    self._partial = text
            elif t == "conversation.item.input_audio_transcription.completed":
                text = msg.get("transcript", "").strip()
                if text:
                    self._final = text
                    self._completed = True
            elif t == "error":
                print(f"⚠️  ASR error: {msg.get('code', '')} {msg.get('message', '')}")

    def send_audio(self, data: bytes):
        if not self._closed:
            self._audio_queue.put(data)

    @property
    def partial(self) -> str:
        return self._partial

    @property
    def final(self) -> str:
        return self._final

    @property
    def completed(self) -> bool:
        return self._completed

    def close(self):
        self._closed = True
        if self.ws:
            try:
                self.ws.close()
            except Exception:
                pass

# ─────────── 音频捕获 ───────────
def audio_capture(proc: subprocess.Popen, q: queue.Queue):
    while True:
        try:
            data = proc.stdout.read(CHUNK)
            if not data or len(data) != CHUNK:
                if proc.poll() is not None:
                    break
                time.sleep(0.01)
                continue
            q.put(data)
        except (OSError, BrokenPipeError, AttributeError):
            break
        except Exception:
            time.sleep(0.01)

# ─────────── 主流程 ───────────
def main():
    import argparse
    parser = argparse.ArgumentParser(description="手部跟随监听器")
    parser.add_argument("--show", action="store_true", help="显示 CAM1 画面")
    parser.add_argument("--no-move", action="store_true", help="仅检测不移动")
    args = parser.parse_args()

    print("=" * 56)
    print("🖐️  手部跟随监听器 (Handover Listener)")
    print("    语音监听中... 说「把水给我」松开机械臂")
    print("    Ctrl+C 退出")
    print("=" * 56)

    if not ASR_API_KEY:
        print("❌ 未设定 ALIYUN_REALTIME_API_KEY")
        sys.exit(1)

    # 1. 启动网关
    start_gateway()

    # 2. 启动手部跟随
    handover_proc = start_handover(args.show, args.no_move)
    time.sleep(2)

    # 2.5 釋放鏡頭（PulseAudio + video_publisher）
    release_camera()
    release_audio()
    time.sleep(0.5)

    # 3. 准备语音监听
    arecord = subprocess.Popen(
        ["arecord", "-D", RECORD_DEVICE, "-c", "1", "-r", str(RECORD_RATE),
         "-f", "S16_LE", "-t", "raw", "-q"],
        stdout=subprocess.PIPE, bufsize=CHUNK * 20,
    )
    audio_q: queue.Queue[bytes] = queue.Queue()
    threading.Thread(target=audio_capture, args=(arecord, audio_q), daemon=True).start()

    asr = AsrClient()
    asr.start()

    released = False  # 防止重复触发
    last_partial = ""

    print("👂 开始监听「把水给我」...")

    try:
        while True:
            try:
                frame = audio_q.get(timeout=1.0)
            except queue.Empty:
                continue

            asr.send_audio(frame)
            partial = asr.partial
            final = asr.final

            # 即时 partial 检测（更快响应）
            if partial and partial != last_partial:
                last_partial = partial
                for kw in RELEASE_KEYWORDS:
                    if kw in partial and not released:
                        print(f"\n🗣️  [你说]: {partial}")
                        print(f"🔍 检测到关键字「{kw}」")
                        released = True
                        speak("好的，马上松开")
                        time.sleep(1)
                        stop_handover(handover_proc)
                        run_release()
                        print("✅ 释放完成，退出")
                        return

            # final 结果检测
            if asr.completed:
                text = final.strip()
                if text:
                    for kw in RELEASE_KEYWORDS:
                        if kw in text and not released:
                            print(f"\n🗣️  [你说]: {text}")
                            print(f"🔍 检测到关键字「{kw}」")
                            released = True
                            speak("好的，马上松开")
                            time.sleep(1)
                            stop_handover(handover_proc)
                            run_release()
                            print("✅ 释放完成，退出")
                            return

                # 非关键字的语音完成→继续监听
                asr.close()
                asr = AsrClient()
                asr.start()
                continue

    except KeyboardInterrupt:
        print("\n\n🛑 用户中断")
    finally:
        stop_handover(handover_proc)
        arecord.terminate()
        arecord.wait(timeout=3)
        resume_audio()
        resume_camera()
        asr.close()
        print("👋 已停止")


if __name__ == "__main__":
    main()
