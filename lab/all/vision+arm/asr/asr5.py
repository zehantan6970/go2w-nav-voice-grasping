from __future__ import annotations

import base64
import io
import json
import os
import queue
import sys
import threading
import time
import uuid
import wave
import subprocess
import math
import struct
from pathlib import Path

import requests
import websocket
from dotenv import load_dotenv

import webrtcvad
import numpy as np  # 引入矩陣運算用於 FFT 頻域分析

ROOT_DIR = Path(__file__).resolve().parent
load_dotenv(ROOT_DIR / ".env")

# --------------------------- 雲端模型配置 ---------------------------
ASR_URL = os.getenv("ALIYUN_REALTIME_BASE_URL", "wss://dashscope.aliyuncs.com/api-ws/v1/realtime")
ASR_MODEL = os.getenv("ALIYUN_REALTIME_MODEL", "qwen3-asr-flash-realtime")
ASR_API_KEY = os.getenv("ALIYUN_REALTIME_API_KEY", "")

TTS_URL = os.getenv("ALIYUN_TTS_BASE_URL", "wss://dashscope.aliyuncs.com/api-ws/v1/realtime")
TTS_MODEL = os.getenv("ALIYUN_TTS_MODEL", "qwen3-tts-flash-realtime")
TTS_API_KEY = os.getenv("ALIYUN_TTS_API_KEY", "") or ASR_API_KEY
TTS_VOICE = os.getenv("ALIYUN_TTS_VOICE", "Cherry")
TTS_SAMPLE_RATE = 16000  
TTS_RESPONSE_FORMAT = os.getenv("ALIYUN_TTS_RESPONSE_FORMAT", "pcm")
TTS_LANGUAGE_TYPE = os.getenv("ALIYUN_TTS_LANGUAGE_TYPE", "Chinese")

LLM_BASE_URL = os.getenv("QWEN_BASE_URL", "https://dashscope.aliyuncs.com/compatible-mode/v1")
LLM_MODEL = os.getenv("QWEN_MODEL", "qwen-plus")
LLM_API_KEY = os.getenv("QWEN_API_KEY", "") or ASR_API_KEY

OPENCLAW_BASE_URL = "http://127.0.0.1:18789"

RECORD_RATE = 16000
RECORD_CHANNELS = 1
RECORD_WIDTH = 2
FRAME_DURATION_MS = 20  
RECORD_CHUNK_SIZE = int(RECORD_RATE * RECORD_WIDTH * RECORD_CHANNELS * FRAME_DURATION_MS / 1000)

WAKEWORD = "你好"
EXIT_KEYWORDS = ["再见", "拜拜", "退出", "再会", "拜", "挂断", "下线"]

current_playback_proc: subprocess.Popen | None = None
playback_lock = threading.Lock()
is_interrupted = False

def _event_id() -> str:
    return f"event_{uuid.uuid4().hex[:12]}"

# 實時分析音訊的頻域特徵向量（FFT）
def analyze_frequency_spectrum(audio_bytes: bytes) -> np.ndarray:
    if not audio_bytes: return np.zeros(32)
    count = len(audio_bytes) // 2
    shorts = struct.unpack(f"{count}h", audio_bytes)
    audio_data = np.array(shorts, dtype=np.float32)
    
    # 執行快速傅立葉變換
    fft_vals = np.abs(np.fft.rfft(audio_data))
    
    # 將頻譜壓縮為 32 個主要頻段（特徵指紋），方便快速比對
    bands = np.array_split(fft_vals, 32)
    spectrum_fingerprint = np.array([np.mean(b) for b in bands])
    return spectrum_fingerprint

class OpenClawClient:
    def __init__(self, base_url: str):
        self.base_url = base_url
        self.tmux_socket = "/tmp/tmux-1000/default"
        if not os.path.exists(self.tmux_socket): self.tmux_socket = "default"

    def _execute_tmux(self, cmd_text: str):
        try:
            if self.tmux_socket != "default":
                tmux_cmd = f"tmux -S {self.tmux_socket} send-keys -t openclaw_session '{cmd_text}' Enter"
            else:
                tmux_cmd = f"tmux send-keys -t openclaw_session '{cmd_text}' Enter"
            subprocess.Popen(tmux_cmd, shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except Exception: pass

    def send_command(self, action: str, params: dict | None = None) -> bool:
        if not action: return False
        direction_map = {
            "forward": "機器狗向前走", "backward": "機器狗向後退",
            "left": "機器狗向左橫移", "right": "機器狗向右橫移",
            "turn_left": "機器狗左轉", "turn_right": "機器狗右轉"
        }
        raw_dir = params.get("direction", "") if params else ""
        text_command = direction_map.get(raw_dir, "機器狗停止")
        if action == "stop": text_command = "機器狗停止"
        print(f"\n📡 [Tmux 注入] 發送指令: 「{text_command}」...")
        self._execute_tmux(text_command)
        return True

    def force_stop_dog(self):
        print("\n🚨 [打斷機制] 檢測到主人有效插話！強行打斷中...")
        self._execute_tmux("機器狗停止")

class AsrClient:
    def __init__(self) -> None:
        self.ws: websocket.WebSocket | None = None
        self.audio_queue: queue.Queue[bytes] = queue.Queue()
        self._final_text, self._partial_text = "", ""
        self._closed, self._is_completed = True, False
        self._sender_thread, self._receiver_thread = None, None

    def start(self) -> bool:
        self._final_text, self._partial_text = "", ""
        self._is_completed, self._closed = False, False
        self.audio_queue = queue.Queue()
        return self._connect()

    def _connect(self) -> bool:
        try:
            self.ws = websocket.create_connection(f"{ASR_URL}?model={ASR_MODEL}", header=[f"Authorization: Bearer {ASR_API_KEY}"], timeout=8)
            if self._sender_thread is None or not self._sender_thread.is_alive():
                self._sender_thread = threading.Thread(target=self._sender_loop, daemon=True)
                self._sender_thread.start()
            if self._receiver_thread is None or not self._receiver_thread.is_alive():
                self._receiver_thread = threading.Thread(target=self._receiver_loop, daemon=True)
                self._receiver_thread.start()
            return True
        except Exception: return False

    def _sender_loop(self) -> None:
        while not self._closed:
            try: chunk = self.audio_queue.get(timeout=0.05)
            except queue.Empty: continue
            if self.ws and self.ws.connected:
                try: self.ws.send(json.dumps({"event_id": _event_id(), "type": "input_audio_buffer.append", "audio": base64.b64encode(chunk).decode("utf-8")}))
                except Exception: self._connect()

    def _receiver_loop(self) -> None:
        while not self._closed:
            if not self.ws or not self.ws.connected: time.sleep(0.1); continue
            try: message = self.ws.recv()
            except Exception: continue
            if isinstance(message, bytes): continue
            try: payload = json.loads(message)
            except json.JSONDecodeError: continue
            event_type = payload.get("type", "")
            if event_type == "conversation.item.input_audio_transcription.text":
                text = f"{payload.get('text', '')}{payload.get('stash', '')}".strip()
                if text: self._partial_text = text
            elif event_type == "conversation.item.input_audio_transcription.completed":
                text = payload.get('transcript', '').strip()
                if text: self._final_text = text
                self._is_completed = True

    def send_audio(self, audio_bytes: bytes) -> None:
        if not self._closed: self.audio_queue.put(audio_bytes)
    def get_partial_text(self) -> str: return self._partial_text
    def get_final_text(self) -> str: return self._final_text
    def is_completed(self) -> bool: return self._is_completed
    def close(self) -> None:
        self._closed = True
        if self.ws:
            try: self.ws.close()
            except Exception: pass

class TtsClient:
    def synthesize(self, text: str) -> bytes:
        cleaned = (text or "").strip()
        ws = websocket.create_connection(f"{TTS_URL}?model={TTS_MODEL}", header=[f"Authorization: Bearer {TTS_API_KEY}"], timeout=15)
        audio_chunks: list[bytes] = []
        try:
            ws.send(json.dumps({"event_id": _event_id(), "type": "session.update", "session": {"voice": TTS_VOICE, "response_format": TTS_RESPONSE_FORMAT, "sample_rate": TTS_SAMPLE_RATE, "language_type": TTS_LANGUAGE_TYPE}}))
            ws.send(json.dumps({"event_id": _event_id(), "type": "input_text_buffer.append", "text": cleaned}))
            ws.send(json.dumps({"event_id": _event_id(), "type": "input_text_buffer.commit"}))
            while True:
                msg = ws.recv(); payload = json.loads(msg); event_type = payload.get("type", "")
                if event_type in {"response.audio.delta", "output_audio.delta"}:
                    delta = payload.get("delta") or payload.get("audio")
                    if delta: audio_chunks.append(base64.b64decode(delta))
                elif event_type in {"response.completed", "response.done", "session.finished"}: break
        finally:
            try: ws.close()
            except Exception: pass
        return b"".join(audio_chunks)

def chat_with_llm(user_text: str, history: list[dict]) -> tuple[str, str | None, dict | None]:
    text_clean = user_text.strip()
    print(f"🔧 [聽力矯正前]: {text_clean}")
    if any(kw in text_clean for kw in ["then to", "big step back", "step back", "backward", "back"]): text_clean = "向後退"
    elif any(kw in text_clean for kw in ["walk forward", "go forward", "forward"]): text_clean = "向前走"
    elif any(kw in text_clean for kw in ["left", "turn left"]): text_clean = "向左橫移"
    elif any(kw in text_clean for kw in ["right", "turn right"]): text_clean = "向右橫移"
    elif any(kw in text_clean for kw in ["stop", "dont move"]): text_clean = "停止"
    print(f"🔧 [聽力矯正後]: {text_clean}")
    
    system_prompt = {"role": "system", "content": "你是一個 Go2W 機器狗語音助手。請簡短回答用戶。若有移動控制意圖，在結尾附加 [COMMAND:{\"action\":\"walk\", \"params\":{\"direction\":\"forward/backward/left/right/turn_left/turn_right\", \"speed\":0.4, \"duration\":2}}]"}
    messages = [system_prompt] + history + [{"role": "user", "content": text_clean}]
    try:
        resp = requests.post(f"{LLM_BASE_URL}/chat/completions", headers={"Authorization": f"Bearer {LLM_API_KEY}", "Content-Type": "application/json"}, json={"model": LLM_MODEL, "messages": messages, "stream": False}, timeout=15)
        raw_content = resp.json()["choices"][0]["message"]["content"].strip()
        reply_text, action, params = raw_content, None, None
        if "[COMMAND:" in raw_content:
            start_idx = raw_content.find("[COMMAND:")
            end_idx = raw_content.find("]", start_idx)
            if end_idx != -1:
                cmd_data = json.loads(raw_content[start_idx + 9 : end_idx])
                action, params = cmd_data.get("action"), cmd_data.get("params", {})
                reply_text = raw_content[:start_idx].strip()
        return reply_text, action, params
    except Exception as exc: return f"（大模型对话失败: {exc}）", None, None

def play_tts_async_proc(pcm_bytes: bytes):
    global current_playback_proc, is_interrupted
    temp_file = f"/tmp/tts_play_{uuid.uuid4().hex[:8]}.wav"
    buffer = io.BytesIO()
    with wave.open(buffer, "wb") as wav_file:
        wav_file.setnchannels(1); wav_file.setsampwidth(2); wav_file.setframerate(TTS_SAMPLE_RATE)
        wav_file.writeframes(pcm_bytes)
    with open(temp_file, "wb") as f: f.write(buffer.getvalue())

    try:
        with playback_lock:
            if is_interrupted: return
            current_playback_proc = subprocess.Popen(["aplay", "-D", "default", temp_file, "-q"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        if current_playback_proc: current_playback_proc.wait()
    except Exception: pass
    finally:
        if os.path.exists(temp_file):
            try: os.remove(temp_file)
            except Exception: pass

def stop_current_tts():
    global current_playback_proc
    with playback_lock:
        if current_playback_proc and current_playback_proc.poll() is None:
            current_playback_proc.terminate(); current_playback_proc.kill()
            current_playback_proc = None

def audio_capture_thread(proc, raw_audio_queue: queue.Queue):
    while True:
        try:
            data = proc.stdout.read(RECORD_CHUNK_SIZE)
            if not data or len(data) != RECORD_CHUNK_SIZE:
                if proc.poll() is not None: break
                time.sleep(0.01)
                continue
            raw_audio_queue.put(data)
        except (OSError, BrokenPipeError, AttributeError):
            break
        except Exception:
            time.sleep(0.01)

def main() -> None:
    global is_interrupted
    history: list[dict] = []
    claw_client = OpenClawClient(base_url=OPENCLAW_BASE_URL)

    arecord_cmd = ["arecord", "-D", "plughw:1,0", "-c", "1", "-r", str(RECORD_RATE), "-f", "S16_LE", "-t", "raw", "-q", "-B", "500000"]
    proc = subprocess.Popen(arecord_cmd, stdout=subprocess.PIPE, bufsize=RECORD_CHUNK_SIZE * 20)

    raw_audio_queue: queue.Queue[bytes] = queue.Queue()
    threading.Thread(target=audio_capture_thread, args=(proc, raw_audio_queue), daemon=True).start()

    vad = webrtcvad.Vad(3)
    asr = AsrClient()
    asr.start()

    print("=" * 60)
    print(f"🚀 核心【全雙工 V7.1 高動態頻域指紋版】就緒！喚醒詞：【{WAKEWORD}】")
    print("📢 修正門檻至百萬級別，引入自適應動態防線，徹底鎖死自打斷")
    print("=" * 60)

    current_state = 0
    vad_active_count = 0
    
    dog_spectrum_profile = np.zeros(32)
    profile_frames = 0

    try:
        while True:
            try: mic_raw = raw_audio_queue.get(timeout=1.0)
            except queue.Empty: continue

            with playback_lock:
                is_dog_speaking = (current_playback_proc is not None and current_playback_proc.poll() is None)

            # 100% 全雙工流送往雲端
            asr.send_audio(mic_raw)

            is_speech = vad.is_speech(mic_raw, RECORD_RATE)

            if current_state == 1 and is_speech:
                if is_dog_speaking:
                    current_spectrum = analyze_frequency_spectrum(mic_raw)

                    # 動態建立小狗聲音頻譜基準線（前200ms）
                    if profile_frames < 10:
                        if profile_frames == 0:
                            dog_spectrum_profile = current_spectrum
                        else:
                            dog_spectrum_profile = 0.8 * dog_spectrum_profile + 0.2 * current_spectrum
                        profile_frames += 1
                        continue

                    # 計算當前錄音頻譜與小狗特徵的絕對差值
                    spectrum_diff = np.sum(np.abs(current_spectrum - dog_spectrum_profile))

                    # 🌟 【核心優化】：將打斷門檻上調到 15,000,000.0（一千五百萬級別）
                    # 只有當你的聲音疊加進去，頻譜跳變超越這個巨量門檻時，才判定為主人插話
                    DYNAMIC_THRESHOLD = 15000000.0 

                    if spectrum_diff > DYNAMIC_THRESHOLD:
                        vad_active_count += 1
                        if vad_active_count >= 4: # 持續 80ms 判定成功
                            is_interrupted = True
                            stop_current_tts()
                            claw_client.force_stop_dog()
                            print(f"✨ [系統] 偵測到主人有效插話 (頻譜特徵差值: {spectrum_diff:.1f})，打斷成功！")
                            asr.close(); asr = AsrClient(); asr.start()
                            vad_active_count = 0
                            dog_spectrum_profile = np.zeros(32)
                            profile_frames = 0
                            is_interrupted = False
                            
                            # 清空緩衝佇列防止重疊
                            while not raw_audio_queue.empty():
                                try: raw_audio_queue.get_nowait()
                                except queue.Empty: break
                            continue
                    else:
                        vad_active_count = max(0, vad_active_count - 1)
                else:
                    dog_spectrum_profile = np.zeros(32)
                    profile_frames = 0
                    vad_active_count = 0
            else:
                if not is_dog_speaking:
                    dog_spectrum_profile = np.zeros(32)
                    profile_frames = 0
                vad_active_count = max(0, vad_active_count - 1)

            if current_state == 0:
                if WAKEWORD in asr.get_partial_text():
                    print(f"\n[唤醒成功] 聽到 '{WAKEWORD}'，請下達指令...")
                    current_state = 1
                    asr.close(); asr = AsrClient(); asr.start()

            elif current_state == 1:
                if asr.is_completed():
                    user_text = asr.get_final_text()
                    print(f"\n[你说]: {user_text}")
                    if not user_text.strip():
                        asr.close(); asr = AsrClient(); asr.start(); continue
                    if any(kw in user_text for kw in EXIT_KEYWORDS):
                        print("[AI]: 下次見，拜拜！"); current_state = 0
                        asr.close(); asr = AsrClient(); asr.start(); continue

                    reply, action, params = chat_with_llm(user_text, history)
                    print(f"[AI]: {reply}")

                    try:
                        pcm_tts_bytes = TtsClient().synthesize(reply)
                        threading.Thread(target=play_tts_async_proc, args=(pcm_tts_bytes,), daemon=True).start()
                    except Exception as tts_err:
                        print(f"⚠️ TTS 失敗: {tts_err}")

                    if action:
                        threading.Thread(target=claw_client.send_command, args=(action, params), daemon=True).start()

                    history.clear()
                    asr.close(); asr = AsrClient(); asr.start()

    except KeyboardInterrupt: pass
    finally:
        stop_current_tts()
        try: proc.terminate()
        except Exception: pass
        asr.close()

if __name__ == "__main__":
    main()