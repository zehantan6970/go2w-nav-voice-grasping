"""
ASR 测试脚本 — 验证百炼 MultiModalDialog 的语音识别是否正常工作
================================================================
测试流程:
  1. 连接百炼 WebSocket
  2. 发送静音 PCM 数据 (维持连接)
  3. 验证连接建立和状态切换是否正常
  4. 可选: 如果系统有麦克风, 按回车后录制5秒音频发送

运行:
  conda activate mmdialog
  cd D:\pycode\qwen
  python test_asr.py
"""

import os
import sys
import json
import time
import uuid
import struct
import threading
import numpy as np
from pathlib import Path
from dotenv import load_dotenv

import websocket  # websocket-client

load_dotenv()

API_KEY = os.getenv("DASHSCOPE_API_KEY", "")
WORKSPACE_ID = os.getenv("WORKSPACE_ID", "")
APP_ID = os.getenv("APP_ID", "")
VOICE = os.getenv("VOICE", "longanhuan")
DOWNSTREAM_SAMPLE_RATE = int(os.getenv("DOWNSTREAM_SAMPLE_RATE", "24000"))
WS_URL = "wss://dashscope.aliyuncs.com/api-ws/v1/inference"
UPSTREAM_SAMPLE_RATE = 16000

# ─── 测试结果收集 ─────────────────────────────
class TestResult:
    def __init__(self):
        self.connected = False
        self.task_started = False
        self.dialog_id = None
        self.states = []
        self.asr_texts = []
        self.tts_chunks = 0
        self.tts_bytes = 0
        self.response_text = ""
        self.error = None
        self.events = []
        self.listening_ready = threading.Event()
        self.done = threading.Event()
        self.speech_detected = False

result = TestResult()
_task_id = uuid.uuid4().hex

# ─── WebSocket 处理 ────────────────────────────
def on_open(ws):
    print("[ASR测试] WebSocket 已连接")
    result.connected = True
    send_start(ws)

def on_message(ws, message):
    if isinstance(message, (bytes, bytearray)):
        return
    try:
        msg = json.loads(message)
    except (json.JSONDecodeError, UnicodeDecodeError):
        return
    handle_message(ws, msg)

def on_data(ws, data, data_type, continue_flag):
    if data_type == websocket.ABNF.OPCODE_BINARY and isinstance(data, bytes):
        result.tts_chunks += 1
        result.tts_bytes += len(data)

def on_error(ws, error):
    print(f"[ASR测试] ✗ WebSocket 错误: {error}")
    result.error = str(error)
    result.done.set()

def on_close(ws, close_status_code, close_msg):
    print(f"[ASR测试] WebSocket 关闭: code={close_status_code}, msg={close_msg}")
    result.connected = False
    if not result.listening_ready.is_set():
        result.done.set()

# ─── 消息处理 ──────────────────────────────────
def send_start(ws):
    msg = {
        "header": {
            "action": "run-task",
            "task_id": _task_id,
            "request_id": _task_id,
            "streaming": "duplex",
        },
        "payload": {
            "task_group": "aigc",
            "function": "generation",
            "model": "multimodal-dialog",
            "task": "multimodal-generation",
            "parameters": {
                "upstream": {
                    "type": "AudioOnly", "mode": "duplex",
                    "audio_format": "pcm", "sample_rate": UPSTREAM_SAMPLE_RATE,
                },
                "downstream": {
                    "audio_format": "pcm", "voice": VOICE,
                    "sample_rate": DOWNSTREAM_SAMPLE_RATE,
                    "intermediate_text": "transcript,dialog",
                },
                "client_info": {
                    "user_id": "test_asr_001", "sdk": "python",
                    "device": {"uuid": f"test-{int(time.time())}"},
                },
            },
            "input": {
                "workspace_id": WORKSPACE_ID,
                "app_id": APP_ID,
                "directive": "Start", "dialog_id": None,
            },
        },
    }
    ws.send(json.dumps(msg, ensure_ascii=False))
    print(f"[ASR测试] Start 消息已发送 (voice={VOICE})")

def handle_message(ws, msg):
    header = msg.get("header", {})
    payload = msg.get("payload", {})
    event = header.get("event", "")

    if event == "task-failed":
        err_code = header.get("error_code", "Unknown")
        err_msg = header.get("error_message", "Unknown error")
        print(f"[ASR测试] ✗ 任务失败: {err_code} - {err_msg}")
        result.error = f"{err_code}: {err_msg}"
        result.done.set()
        return

    if event == "task-started":
        print("[ASR测试] 任务已启动 (task-started)")
        result.task_started = True
        return

    if event != "result-generated":
        return

    output = payload.get("output", {})
    resp_event = output.get("event", "")
    result.events.append(resp_event)

    if resp_event == "Started":
        result.dialog_id = output.get("dialog_id")
        print(f"[ASR测试] 会话已创建, dialog_id={result.dialog_id}")

    elif resp_event == "DialogStateChanged":
        state = output.get("state", "")
        result.states.append(state)
        print(f"[ASR测试] 状态切换: -> {state}")
        if state == "Listening":
            result.listening_ready.set()

    elif resp_event == "SpeechContent":
        text = output.get("text", "")
        finished = output.get("finished", False)
        result.asr_texts.append(text)
        result.speech_detected = True
        if finished:
            print(f"[ASR测试] ★ ASR 最终结果: \"{text}\"")
        else:
            print(f"[ASR测试]   ASR 流式: \"{text}\"")

    elif resp_event == "SpeechEnded":
        text = output.get("text", "")
        print(f"[ASR测试] ★ 语音结束: \"{text}\"")

    elif resp_event == "RespondingContent":
        text = output.get("text", "")
        finished = output.get("finished", False)
        result.response_text += text
        if finished:
            print(f"[ASR测试] LLM 回复: \"{text}\"")

    elif resp_event == "RespondingStarted":
        print("[ASR测试] AI 开始回复")

    elif resp_event == "RespondingEnded":
        print(f"[ASR测试] AI 回复结束")
        result.done.set()

    elif resp_event == "Error":
        err_code = output.get("error_code", "")
        err_msg = output.get("error_message", "")
        err_name = output.get("error_name", "")
        print(f"[ASR测试] ✗ 服务端错误: {err_code} ({err_name}): {err_msg}")
        result.error = f"{err_code} ({err_name}): {err_msg}"

# ─── 音频工具 ──────────────────────────────────
def generate_silence(duration_s=0.1):
    """生成静音 PCM 数据 (16kHz, 16bit, mono)"""
    n_samples = int(UPSTREAM_SAMPLE_RATE * duration_s)
    return b'\x00\x00' * n_samples

def generate_tone(freq=440, duration_s=0.5, amplitude=5000):
    """生成正弦波 PCM 数据"""
    n_samples = int(UPSTREAM_SAMPLE_RATE * duration_s)
    t = np.linspace(0, duration_s, n_samples, endpoint=False)
    samples = (amplitude * np.sin(2 * np.pi * freq * t)).astype(np.int16)
    return samples.tobytes()

def record_from_mic(duration_s=5):
    """从系统麦克风录制音频"""
    try:
        import pyaudio
        p = pyaudio.PyAudio()
        chunk_size = 3200  # 100ms at 16kHz
        stream = p.open(
            format=pyaudio.paInt16,
            channels=1,
            rate=UPSTREAM_SAMPLE_RATE,
            input=True,
            frames_per_buffer=chunk_size,
        )
        print(f"[ASR测试] 正在录音 {duration_s} 秒... 请说话!")
        frames = []
        n_chunks = int(duration_s * UPSTREAM_SAMPLE_RATE / chunk_size * 2)  # chunk_size in bytes
        for i in range(n_chunks):
            data = stream.read(chunk_size, exception_on_overflow=False)
            frames.append(data)
        stream.stop_stream()
        stream.close()
        p.terminate()
        print(f"[ASR测试] 录音完成: {len(frames)} chunks")
        return b''.join(frames)
    except Exception as e:
        print(f"[ASR测试] 麦克风录音失败: {e}")
        return None

# ─── 主函数 ────────────────────────────────────
def main():
    print("=" * 60)
    print("  ASR 测试 — 验证百炼语音识别")
    print("=" * 60)
    print(f"  API Key:   {API_KEY[:12]}...{API_KEY[-8:]}")
    print(f"  Workspace: {WORKSPACE_ID}")
    print(f"  App ID:    {APP_ID}")
    print(f"  Voice:     {VOICE}")
    print("=" * 60)

    if not API_KEY or not WORKSPACE_ID or not APP_ID:
        print("✗ 请在 .env 文件中配置 DASHSCOPE_API_KEY, WORKSPACE_ID, APP_ID")
        sys.exit(1)

    print("\n[ASR测试] 正在连接百炼 WebSocket...")

    headers = {
        "Authorization": f"Bearer {API_KEY}",
        "User-Agent": "test-asr/1.0",
    }
    ws_app = websocket.WebSocketApp(
        WS_URL, header=headers,
        on_open=on_open, on_message=on_message,
        on_data=on_data, on_error=on_error, on_close=on_close,
    )
    
    ws_thread = threading.Thread(target=ws_app.run_forever, daemon=True)
    ws_thread.start()

    # 等待 Listening 状态 (最多15秒)
    print("[ASR测试] 等待 Listening 状态...")
    if not result.listening_ready.wait(timeout=15):
        print("[ASR测试] ✗ 等待 Listening 超时!")
        if result.error:
            print(f"[ASR测试] 错误: {result.error}")
        try: ws_app.close()
        except: pass
        sys.exit(1)

    print("[ASR测试] ✓ 已就绪, 可以发送音频")
    
    # 发送静音维持连接 (2秒)
    print("[ASR测试] 发送2秒静音维持连接...")
    for _ in range(20):  # 20 * 100ms = 2s
        if ws_app.sock and ws_app.sock.connected:
            ws_app.send(generate_silence(0.1), websocket.ABNF.OPCODE_BINARY)
        time.sleep(0.1)

    # 询问用户是否要录音测试
    print("\n" + "-" * 40)
    print("[ASR测试] 连接验证通过!")
    print("[ASR测试] 接下来你可以:")
    print("  1. 按回车发送一段测试音频 (440Hz正弦波)")  
    print("  2. 输入 m 回车使用麦克风录音5秒")
    print("  3. 输入 q 回车退出")
    print("-" * 40)

    try:
        choice = input("\n请选择 [回车/m/q]: ").strip().lower()
    except (EOFError, KeyboardInterrupt):
        choice = "q"

    if choice == "m":
        # 麦克风录音
        print("[ASR测试] 使用麦克风录音...")
        mic_data = record_from_mic(duration_s=5)
        if mic_data and ws_app.sock and ws_app.sock.connected:
            # 分块发送 (100ms per chunk)
            chunk_size = 3200
            for i in range(0, len(mic_data), chunk_size):
                chunk = mic_data[i:i+chunk_size]
                if ws_app.sock and ws_app.sock.connected:
                    ws_app.send(chunk, websocket.ABNF.OPCODE_BINARY)
                time.sleep(0.05)  # 50ms间隔
            print(f"[ASR测试] 音频已发送: {len(mic_data)} bytes")
            # 等待ASR结果
            print("[ASR测试] 等待ASR识别结果 (最多15秒)...")
            result.done.wait(timeout=15)

    elif choice == "q":
        pass

    else:
        # 发送测试正弦波
        print("[ASR测试] 发送440Hz正弦波 (0.5秒)...")
        tone_data = generate_tone(freq=440, duration_s=0.5, amplitude=8000)
        chunk_size = 3200
        for i in range(0, len(tone_data), chunk_size):
            chunk = tone_data[i:i+chunk_size]
            if ws_app.sock and ws_app.sock.connected:
                ws_app.send(chunk, websocket.ABNF.OPCODE_BINARY)
            time.sleep(0.05)
        # 再发一段静音
        for _ in range(20):
            if ws_app.sock and ws_app.sock.connected:
                ws_app.send(generate_silence(0.1), websocket.ABNF.OPCODE_BINARY)
            time.sleep(0.1)
        print("[ASR测试] 测试音频已发送")
        # 等待结果
        result.done.wait(timeout=15)

    # 打印测试结果汇总
    print("\n" + "=" * 60)
    print("  ASR 测试结果汇总")
    print("=" * 60)
    print(f"  WebSocket 连接:  {'✓ 成功' if result.connected else '✗ 失败'}")
    print(f"  任务启动:        {'✓ 成功' if result.task_started else '✗ 失败'}")
    print(f"  会话ID:          {result.dialog_id or '无'}")
    print(f"  状态流转:        {' -> '.join(result.states)}")
    print(f"  事件序列:        {' -> '.join(result.events[:15])}")
    
    if result.asr_texts:
        print(f"  ASR 识别结果:    {len(result.asr_texts)} 条")
        for i, t in enumerate(result.asr_texts):
            print(f"    [{i}] {t}")
    else:
        print(f"  ASR 识别结果:    (无 — 可能因为发送的是正弦波/静音, 不是人声)")
    
    if result.response_text:
        print(f"  LLM 回复:        {result.response_text[:200]}")
    
    print(f"  TTS 音频:        {result.tts_chunks} chunks / {result.tts_bytes} bytes")
    
    if result.error:
        print(f"\n  ✗ 错误: {result.error}")
        if "voice" in str(result.error).lower():
            print("  → TTS Voice 不兼容! 请在 .env 中设置 VOICE=cosyvoice-v3-flash")
        print(f"\n  [结果] ✗ ASR 测试失败")
    else:
        print(f"\n  [结果] ✓ ASR 连接测试通过! WebSocket/协议/状态机正常工作")
        if result.asr_texts:
            print(f"  [结果] ✓ 语音识别正常工作!")
        else:
            print(f"  [提示] 如需测试真人语音识别, 请选 'm' 使用麦克风录音")

    try:
        ws_app.close()
    except Exception:
        pass

    sys.exit(0 if not result.error else 1)

if __name__ == "__main__":
    main()
