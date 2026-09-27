#!/usr/bin/env python3
"""
小創語音橋接器 (Voice Bridge)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

喚醒詞「小創小創」→ 語音辨識 → tmux 注入 OpenClaw → 技能執行

設計理念：
  - 只做音訊 I/O + ASR + TTS，不重複造 LLM 輪子
  - 語音指令透過 tmux send-keys 送入 openclaw_session
  - OpenClaw (小創) 自然用技能去執行：移動/機械臂/導航等

依賴：
  pip install websocket-client webrtcvad requests numpy python-dotenv

使用：
  cd ~/vision+arm/asr && python3 voice_bridge.py

環境變數 (.env)：
  ALIYUN_REALTIME_API_KEY=xxx
  ALIYUN_TTS_API_KEY=xxx              (可選，預設同 ALIYUN_REALTIME_API_KEY)
"""

from __future__ import annotations

import base64
import io
import json
import os
import queue
import shlex
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
import requests

# ─────────── 配置 ───────────
ROOT_DIR = Path(__file__).resolve().parent
load_dotenv(ROOT_DIR / ".env")

ASR_API_KEY = os.getenv("ALIYUN_REALTIME_API_KEY", "") or os.getenv("DASHSCOPE_API_KEY", "")
TTS_API_KEY = os.getenv("ALIYUN_TTS_API_KEY", "") or ASR_API_KEY

ASR_URL = os.getenv("ALIYUN_REALTIME_BASE_URL", "wss://dashscope.aliyuncs.com/api-ws/v1/realtime")
ASR_MODEL = os.getenv("ALIYUN_REALTIME_MODEL", "qwen3-asr-flash-realtime")
TTS_URL = os.getenv("ALIYUN_TTS_BASE_URL", "wss://dashscope.aliyuncs.com/api-ws/v1/realtime")
TTS_MODEL = os.getenv("ALIYUN_TTS_MODEL", "qwen3-tts-flash-realtime")
TTS_VOICE = os.getenv("ALIYUN_TTS_VOICE", "Cherry")

RECORD_RATE = 16000
RECORD_CHANNELS = 1
RECORD_WIDTH = 2
FRAME_MS = 20
CHUNK = int(RECORD_RATE * RECORD_WIDTH * RECORD_CHANNELS * FRAME_MS / 1000)

RECORD_DEVICE = os.getenv("ALSA_RECORD_DEVICE", "default")
PLAYBACK_DEVICE = os.getenv("ALSA_PLAYBACK_DEVICE", "default")

# 喚醒詞 / 退出詞
WAKEWORD = "你好"
EXIT_WORDS = ["再見", "拜拜", "退出", "再會", "拜", "掛斷", "下線"]

# 語音結束判定：連續靜音多少幀視為說話結束
SILENCE_FRAMES_MAX = 250  # 25 × 20ms = 5000ms 靜音
COMMAND_TIMEOUT_SEC = 30  # 單次指令最長秒數（含 OpenClaw 處理時間）

# OpenClaw 預熱
_OPENCLAW_WARMED = False
_KEEPALIVE_RUNNING = False

# 音頻搶佔協調：nav_exec 前釋放音頻，完成後重啟
_AUDIO_RELEASE_EVT = threading.Event()
_AUDIO_RESTART_EVT = threading.Event()
_AUDIO_EXIT_EVT = threading.Event()  # nav_exec 時要求主循環退出
_MAIN_ARECORD = [None]  # list wrapper for thread-safe write
_MAIN_ASR = [None]

def _start_keepalive():
    """背景 keepalive：每 8 秒發 BalanceStand，防止機器人進入鎖定姿態"""
    global _KEEPALIVE_RUNNING
    if _KEEPALIVE_RUNNING:
        return
    _KEEPALIVE_RUNNING = True

    def _pulse():
        code = """import os, time,sys
os.environ['LD_LIBRARY_PATH'] = '/home/unitree/cyclonedds/install/lib'
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.go2.sport.sport_client import SportClient
ChannelFactoryInitialize(0, 'eth0')
c = SportClient()
c.SetTimeout(3.0)
c.Init()
while True:
    try:
        c.BalanceStand()
        time.sleep(0.2)
        c.StopMove()
    except:
        pass
    time.sleep(8)
"""
        try:
            proc = subprocess.Popen(["python3", "-c", code],
                env={**os.environ, "LD_LIBRARY_PATH": "/home/unitree/cyclonedds/install/lib"},
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            print(f"🟢 [Keepalive] PID {proc.pid} 已啟動（每 8 秒 BalanceStand）")
        except Exception as e:
            print(f"🔴 [Keepalive] 啟動失敗: {e}")

    threading.Thread(target=_pulse, daemon=True).start()


def _prewarm_openclaw():
    """非同步預熱 OpenClaw session"""
    global _OPENCLAW_WARMED
    if _OPENCLAW_WARMED:
        return
    def _warm():
        try:
            subprocess.run(
                "openclaw agent --session-key agent:main:voice -m 预热 --json 2>/dev/null",
                shell=True, timeout=30
            )
            _OPENCLAW_WARMED = True
        except Exception:
            pass
    threading.Thread(target=_warm, daemon=True).start()


def _ensure_robot_standing():
    """確保機器人處於正常站立狀態，防止鎖定姿態
    透過簡短 Python 腳本呼叫 SportClient.BalanceStand()
    """
    try:
        subprocess.run(
            ["python3", "-c", """
import sys, os
os.environ['LD_LIBRARY_PATH'] = '/home/unitree/cyclonedds/install/lib'
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.go2.sport.sport_client import SportClient
ChannelFactoryInitialize(0, 'eth0')
c = SportClient()
c.SetTimeout(3.0)
c.Init()
c.StopMove()
"""],
            capture_output=True, timeout=5
        )
    except Exception:
        pass

# tmux 配置
TMUX_SESSION = os.getenv("TMUX_SESSION", "openclaw_session")
TMUX_SOCKET = os.getenv("TMUX_SOCKET", "/tmp/tmux-1000/default")

# ─────────── 音訊設備檢測 ───────────
# 啟動時提示，不阻塞
print(f"🎤 錄音設備: {RECORD_DEVICE} (16kHz, S16_LE)")
print(f"🔊 播放設備: {PLAYBACK_DEVICE}")
_prewarm_openclaw()
_start_keepalive()

# 背景預生成 TTS 快取（非阻塞）
def _precache_tts():
    tts_script = ROOT_DIR / "tts_precache.py"
    if tts_script.exists():
        subprocess.Popen(
            ["python3", str(tts_script)],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        )
threading.Thread(target=_precache_tts, daemon=True).start()

if not ASR_API_KEY:
    print("⚠️  未設定 ALIYUN_REALTIME_API_KEY，ASR 無法運作")
    sys.exit(1)

# ─────────── 實用函數 ───────────
def _eid() -> str:
    return uuid.uuid4().hex[:12]


def _ensure_tmux_session():
    """確保 openclaw_session 存在；不存在則建立並啟動 openclaw"""
    import shlex
    check = subprocess.run(
        shlex.split(f"tmux has-session -t {TMUX_SESSION} 2>/dev/null"),
        shell=False, capture_output=True,
    )
    if check.returncode == 0:
        return  # 已存在

    print(f"🆕 建立 tmux session「{TMUX_SESSION}」並啟動 OpenClaw...")
    subprocess.run(
        shlex.split(f"tmux new-session -d -s {TMUX_SESSION}"),
        shell=False, capture_output=True,
    )
    # 在 session 中啟動 openclaw
    tmux_send_cmd("openclaw")
    time.sleep(4)  # 等 OpenClaw 初始化
    print(f"✅ OpenClaw 已在 tmux「{TMUX_SESSION}」中執行")


def tmux_send_cmd(cmd: str):
    """向 openclaw_session 傳送按鍵（底層）"""
    escaped = cmd.replace("'", "'\\''")
    if os.path.exists(TMUX_SOCKET):
        full = f"tmux -S {TMUX_SOCKET} send-keys -t {TMUX_SESSION} '{escaped}' Enter"
    else:
        full = f"tmux send-keys -t {TMUX_SESSION} '{escaped}' Enter"
    subprocess.Popen(full, shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def send_and_speak(text: str):
    """使用 voice_controller 解析語音指令後執行

    流程：
      1. parse_voice_command() → 結構化意圖
      2. intent_to_command() → {"type": "direct" | "nav_exec" | "openclaw", ...}
      3. direct → 直接跑 robot_command.py
         nav_exec → 直接跑 nav_exec.py（寫任務檔 + 控制 DEMO）
         openclaw → 送 OpenClaw 處理
      4. TTS 播報結果
    """
    import re

    def _worker():
        try:
            # 1. 解析意圖
            parsed = parse_voice_command(text)
            action = parsed.get("action", "unknown")

            if action == "unknown":
                print(f"🤷 無法解析: 「{text}」")
                speak("我沒聽清楚，請再說一次")
                return

            # 2. 取得執行指令
            cmd = intent_to_command(parsed)
            if not cmd:
                print(f"⚠️  action={action} 無對應指令")
                return

            if cmd["type"] == "direct":
                # ─── 直接執行 ───
                cmd_list = cmd["cmd"]
                print(f"⚡ [direct] → {' '.join(cmd_list)}")
                subprocess.run(
                    cmd_list, capture_output=True, text=True, timeout=30,
                    env={**os.environ, "LD_LIBRARY_PATH": "/home/unitree/cyclonedds/install/lib"},
                )
                speak("好的")

            elif cmd["type"] == "nav_exec":
                # ─── 導航/抓取/釋放：直接控制 DEMO ───
                cmd_list = cmd["cmd"]
                print(f"\n🗺️ [nav_exec] → {' '.join(cmd_list)}")

                # 先啟動 nav_exec（讓它成為孤兒進程，輸出到日誌）
                log_file = open("/tmp/nav_exec.log", "w")
                nav_proc = subprocess.Popen(
                    cmd_list,
                    stdout=log_file, stderr=subprocess.STDOUT,
                    env={**os.environ, "LD_LIBRARY_PATH": "/home/unitree/cyclonedds/install/lib",
                         "PYTHONUNBUFFERED": "1"},
                )
                log_file.close()
                print(f"🔁 nav_exec PID: {nav_proc.pid}，日誌: /tmp/nav_exec.log")
                print("👋 voice_bridge 退出，nav_exec 背景執行中")
                print("   完成後可用 cat /tmp/nav_exec.log 查看結果")

                # 通知主循環退出（釋放音頻）
                _AUDIO_EXIT_EVT.set()
                time.sleep(2)

                # 主循環退出，nav_proc 變成孤兒繼續跑
                os._exit(0)

            elif cmd["type"] == "openclaw":
                # ─── 複雜指令：送 OpenClaw 處理（超時 120s） ───
                cmd_text = cmd.get("text", "")
                if not cmd_text:
                    return

                session_key = "agent:main:voice"
                oc_cmd = f"openclaw agent --session-key {session_key} -m {shlex.quote(cmd_text)}"
                print(f"\n📡 [openclaw] → 「{cmd_text}」 (等候回應最多 120 秒)")
                print("═" * 50)
                try:
                    result = subprocess.run(
                        oc_cmd, shell=True, capture_output=True, text=True, timeout=120
                    )
                except subprocess.TimeoutExpired:
                    print("⏱️  OpenClaw 120 秒未回應，可能卡住了")
                    speak("處理超時，請稍後再試")
                    return

                # 顯示完整執行過程（過濾 ANSI + 插件日誌）
                clean = re.sub(r'\x1b\[[0-9;]*m', '', result.stdout)
                all_lines = [l.strip() for l in clean.split('\n') if l.strip()]
                skip_prefixes = ("plugins.", "Set plugins.allow", "config.schema", "gateway")
                assistant_reply = None
                shown = []
                for ln in all_lines:
                    if any(ln.startswith(p) for p in skip_prefixes):
                        continue
                    if "Process exited" in ln:
                        continue
                    if not ln.startswith("[") and ln:
                        assistant_reply = ln
                    shown.append(ln)
                if shown:
                    for ln in shown:
                        print(f"  {ln}")
                print("═" * 50)
                if assistant_reply:
                    print(f"🔊 [TTS] → {assistant_reply}")
                    speak(assistant_reply)
                else:
                    speak("好的")

        except subprocess.TimeoutExpired:
            print(f"⏱️  執行超時")
            speak("處理超時，請稍後再試")
        except Exception as e:
            print(f"❌ send_and_speak 出錯: {e}")
            speak("好的")

    print(f"🗣️  [你說]: 「{text}」")
    threading.Thread(target=_worker, daemon=True).start()


def tts_synthesize(text: str) -> bytes:
    """阿里雲 TTS 合成語音"""
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
        print(f"⚠️  TTS 合成失敗: {e}")
        return b""


def play_pcm(pcm_bytes: bytes):
    """播放 PCM 音頻"""
    if not pcm_bytes:
        return
    buf = io.BytesIO()
    with wave.open(buf, "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(RECORD_RATE)
        w.writeframes(pcm_bytes)
    tmp = f"/tmp/voice_bridge_{uuid.uuid4().hex[:8]}.wav"
    with open(tmp, "wb") as f:
        f.write(buf.getvalue())
    try:
        subprocess.run(
            ["aplay", "-D", PLAYBACK_DEVICE, tmp, "-q"],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        )
    finally:
        try:
            os.remove(tmp)
        except OSError:
            pass


def speak(text: str):
    """合成並播放語音（非阻塞）"""
    threading.Thread(target=lambda: play_pcm(tts_synthesize(text)), daemon=True).start()


# ─────────── ASR 客戶端 ───────────
class AsrClient:
    """低延遲串流 ASR，支援 partial 結果"""
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
            print(f"❌ ASR 連線失敗: {e}")
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


# ─────────── 音訊捕獲 ───────────
def audio_capture(proc: subprocess.Popen, q: queue.Queue):
    """持續從 arecord 讀取音訊幀"""
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
# 匯入 voice_controller
from voice_controller import parse_voice_command, intent_to_command


def main():
    print("=" * 56)
    print("🚀  小創語音橋接器 (Voice Bridge)")
    print(f"    喚醒詞: 「{WAKEWORD}」")
    print(f"    ASR:    {ASR_MODEL}")
    print(f"    TTS:    {TTS_MODEL} / {TTS_VOICE}")
    print("=" * 56)

    # 啟動 arecord
    arecord = subprocess.Popen(
        ["arecord", "-D", RECORD_DEVICE, "-c", "1", "-r", str(RECORD_RATE),
         "-f", "S16_LE", "-t", "raw", "-q"],
        stdout=subprocess.PIPE,
        bufsize=CHUNK * 20,
    )
    audio_q: queue.Queue[bytes] = queue.Queue()
    threading.Thread(target=audio_capture, args=(arecord, audio_q), daemon=True).start()

    vad = webrtcvad.Vad(3)
    asr = AsrClient()
    asr.start()

    state = "idle"          # idle → wake_pending → listening → done
    wake_buffer = ""        # 累積 partial 辨識喚醒詞
    command_buffer: list[str] = []
    silence_count = 0
    cmd_start_time = 0.0
    last_partial = ""
    last_speech_frame_time = time.time()

    try:
        while True:
            # 檢查退出請求
            if _AUDIO_EXIT_EVT.is_set():
                print("👋 voice_bridge 退出（nav_exec 接管）")
                _AUDIO_EXIT_EVT.clear()
                try:
                    arecord.kill()
                    arecord.wait(timeout=2)
                except:
                    pass
                asr.close()
                break

            try:
                frame = audio_q.get(timeout=1.0)
            except queue.Empty:
                continue

            # 永遠送 ASR（全雙工）
            asr.send_audio(frame)
            is_speech = vad.is_speech(frame, RECORD_RATE)

            # ─── 閒置狀態：只檢測喚醒詞 ───
            if state == "idle":
                partial = asr.partial
                if partial and partial != last_partial:
                    last_partial = partial
                    # 檢查喚醒詞是否出現在 partial 結果
                    if WAKEWORD in partial:
                        print(f"\n🔊 [喚醒] 偵測到「{WAKEWORD}」！準備聽取指令...")
                        # 播放確認音效
                        speak("在呢，請說")
                        state = "listening"
                        command_buffer = []
                        silence_count = 0
                        cmd_start_time = time.time()
                        asr.close()
                        asr = AsrClient()
                        asr.start()
                        last_partial = ""
                        # 清空佇列避免殘音干擾
                        while not audio_q.empty():
                            try:
                                audio_q.get_nowait()
                            except queue.Empty:
                                break
                continue

            # ─── 聆聽狀態 ───
            if state == "listening":
                partial = asr.partial

                # 監控退出詞
                if partial and any(kw in partial for kw in EXIT_WORDS):
                    print("👋 收到退出指令，回到待機")
                    speak("好的，下次叫我")
                    state = "idle"
                    asr.close()
                    asr = AsrClient()
                    asr.start()
                    last_partial = ""
                    continue

                # 累計最後語音時間
                if is_speech:
                    last_speech_frame_time = time.time()

                # 檢查超時
                elapsed = time.time() - cmd_start_time
                if elapsed > COMMAND_TIMEOUT_SEC:
                    print(f"⏱️  指令超時 ({COMMAND_TIMEOUT_SEC}s)，回到待機")
                    state = "idle"
                    asr.close()
                    asr = AsrClient()
                    asr.start()
                    last_partial = ""
                    continue

                # ASR 完成：收到最終結果
                if asr.completed:
                    text = asr.final.strip()
                    if text:
                        print(f"🗣️  [你說]: {text}")
                        # 透過 tmux 注入 OpenClaw
                        send_and_speak(text)
                        state = "idle"
                        asr.close()
                        asr = AsrClient()
                        asr.start()
                        last_partial = ""
                        continue

                    # 空結果 → 回待機（可能是誤觸發）
                    state = "idle"
                    asr.close()
                    asr = AsrClient()
                    asr.start()
                    last_partial = ""
                    continue

                # 顯示即時辨識進度
                if partial and partial != last_partial:
                    last_partial = partial
                    print(f"  ⏳ 辨識中: {partial}", end="\r", flush=True)

            # ========== 音頻搶佔處理 ==========
            if _AUDIO_RELEASE_EVT.is_set():
                print("🎤 釋放音頻設備（給 handover 使用）...")
                try:
                    arecord.kill()
                    arecord.wait(timeout=3)
                except:
                    pass
                asr.close()
                _AUDIO_RELEASE_EVT.clear()
                # 清空音頻佇列
                while not audio_q.empty():
                    try: audio_q.get_nowait()
                    except: break
                state = "idle"
                print("🎤 音頻已釋放")



            if _AUDIO_RESTART_EVT.is_set():
                print("🎤 任務完成，重啟音頻...")
                _AUDIO_RESTART_EVT.clear()
                try:
                    arecord = subprocess.Popen(
                        ["arecord", "-D", RECORD_DEVICE, "-c", "1", "-r", str(RECORD_RATE),
                         "-f", "S16_LE", "-t", "raw", "-q"],
                        stdout=subprocess.PIPE, bufsize=CHUNK * 20,
                    )
                    audio_q = queue.Queue()
                    threading.Thread(target=audio_capture, args=(arecord, audio_q), daemon=True).start()
                    asr = AsrClient()
                    asr.start()
                    last_partial = ""
                    print("🎤 音頻已重啟")
                except Exception as e:
                    print(f"⚠️  音頻重啟失敗: {e}")
            # ==================================

    except KeyboardInterrupt:
        print("\n\n🛑 使用者中斷")
    finally:
        arecord.terminate()
        asr.close()
        print("👋 語音橋接器已停止")


if __name__ == "__main__":
    main()
