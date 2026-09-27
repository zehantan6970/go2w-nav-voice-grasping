"""
TTS 测试脚本 — 验证百炼 MultiModalDialog 的 TTS 语音合成是否正常工作
=====================================================================
测试流程:
  1. 连接百炼 WebSocket
  2. 发送文本 "你好，我是一个AI助手"
  3. 接收 TTS 音频并保存到 tts_output.pcm
  4. 打印诊断信息

运行:
  conda activate mmdialog
  cd D:\pycode\qwen
  python test_tts.py
"""

import os
import sys
import json
import time
import uuid
import threading
from datetime import datetime
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
        self.tts_chunks = 0
        self.tts_bytes = 0
        self.tts_audio_data = bytearray()
        self.response_text = ""
        self.error = None
        self.asr_text = ""
        self.events = []
        self.done = threading.Event()
        self.responding_ended = threading.Event()

result = TestResult()

# ─── WebSocket 处理 ────────────────────────────
def on_open(ws):
    print("[TTS测试] WebSocket 已连接")
    result.connected = True
    send_start(ws)

def on_message(ws, message):
    # websocket-client 有时会把二进制帧也传给 on_message
    if isinstance(message, (bytes, bytearray)):
        # 二进制数据应由 on_data 处理，这里忽略
        return
    try:
        msg = json.loads(message)
    except (json.JSONDecodeError, UnicodeDecodeError):
        # 忽略无法解码的消息
        return
    handle_message(ws, msg)

def on_data(ws, data, data_type, continue_flag):
    """接收二进制 TTS 音频数据"""
    if data_type == websocket.ABNF.OPCODE_BINARY and isinstance(data, bytes):
        result.tts_chunks += 1
        result.tts_bytes += len(data)
        result.tts_audio_data.extend(data)
        if result.tts_chunks == 1:
            print(f"[TTS测试] ★ 首次收到 TTS 音频: {len(data)} bytes")
        if result.tts_chunks % 20 == 0:
            print(f"[TTS测试]   TTS 音频: {result.tts_chunks} chunks / {result.tts_bytes} bytes")

def on_error(ws, error):
    print(f"[TTS测试] ✗ WebSocket 错误: {error}")
    result.error = str(error)
    result.done.set()

def on_close(ws, close_status_code, close_msg):
    print(f"[TTS测试] WebSocket 关闭: code={close_status_code}, msg={close_msg}")
    result.connected = False
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
                    "user_id": "test_tts_001", "sdk": "python",
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
    print(f"[TTS测试] Start 消息已发送 (voice={VOICE})")

def handle_message(ws, msg):
    header = msg.get("header", {})
    payload = msg.get("payload", {})
    event = header.get("event", "")

    if event == "task-failed":
        err_code = header.get("error_code", "Unknown")
        err_msg = header.get("error_message", "Unknown error")
        print(f"[TTS测试] ✗ 任务失败: {err_code} - {err_msg}")
        result.error = f"{err_code}: {err_msg}"
        result.done.set()
        return

    if event == "task-started":
        print("[TTS测试] 任务已启动 (task-started)")
        result.task_started = True
        return

    if event != "result-generated":
        return

    output = payload.get("output", {})
    resp_event = output.get("event", "")
    result.events.append(resp_event)

    if resp_event == "Started":
        result.dialog_id = output.get("dialog_id")
        print(f"[TTS测试] 会话已创建, dialog_id={result.dialog_id}")

    elif resp_event == "DialogStateChanged":
        state = output.get("state", "")
        result.states.append(state)
        print(f"[TTS测试] 状态切换: -> {state}")
        
        if state == "Listening" and not result.error:
            # 收到 Listening 后发送文本 (仅在无错误时)
            print(f"[TTS测试] 就绪! 发送测试文本...")
            send_text(ws, "你好，请简单介绍一下你自己")

    elif resp_event == "SpeechContent":
        text = output.get("text", "")
        result.asr_text = text
        print(f"[TTS测试] ASR: {text}")

    elif resp_event == "RespondingContent":
        text = output.get("text", "")
        finished = output.get("finished", False)
        if finished:
            result.response_text = text  # 只保存最终完整文本
            print(f"[TTS测试] LLM 最终回复: \"{text}\"")
        elif text:
            # 流式中间结果仅用于观察
            preview = text[:80] + ("..." if len(text) > 80 else "")
            print(f"[TTS测试] LLM 流式: {preview}")

    elif resp_event == "RespondingStarted":
        print("[TTS测试] AI 开始回复 (TTS音频开始下发)")

    elif resp_event == "RespondingEnded":
        print(f"[TTS测试] AI 回复结束 (TTS音频下发完毕, {result.tts_chunks} chunks / {result.tts_bytes} bytes)")
        result.responding_ended.set()

    elif resp_event == "Error":
        err_code = output.get("error_code", "")
        err_msg = output.get("error_message", "")
        err_name = output.get("error_name", "")
        print(f"[TTS测试] ✗ 服务端错误:")
        print(f"  error_code:    {err_code}")
        print(f"  error_name:    {err_name}")
        print(f"  error_message: {err_msg}")
        result.error = f"{err_code} ({err_name}): {err_msg}"
        # 不再自动重试 — 设置 done 标志退出
        result.done.set()
        result.responding_ended.set()

    elif resp_event == "RequestAccepted":
        print("[TTS测试] 打断已接受")

def send_text(ws, text):
    task_id = uuid.uuid4().hex  # 注意: 实际应该用同一个 task_id
    # 使用与 start 相同的 task_id (通过闭包或全局变量)
    msg = {
        "header": {
            "action": "continue-task",
            "task_id": _task_id,
            "request_id": _task_id,
            "streaming": "duplex",
        },
        "payload": {
            "task_group": "aigc",
            "function": "generation",
            "model": "",
            "task": "multimodal-generation",
            "input": {
                "app_id": APP_ID,
                "directive": "RequestToRespond",
                "dialog_id": result.dialog_id,
                "type": "prompt", "text": text,
            },
        },
    }
    ws.send(json.dumps(msg, ensure_ascii=False))
    print(f"[TTS测试] 文本已发送: \"{text}\"")

# ─── 主函数 ────────────────────────────────────
def main():
    global _task_id
    _task_id = uuid.uuid4().hex

    print("=" * 60)
    print("  TTS 测试 — 验证百炼 TTS 语音合成")
    print("=" * 60)
    print(f"  API Key:   {API_KEY[:12]}...{API_KEY[-8:]}")
    print(f"  Workspace: {WORKSPACE_ID}")
    print(f"  App ID:    {APP_ID}")
    print(f"  Voice:     {VOICE}")
    print(f"  下行采样率: {DOWNSTREAM_SAMPLE_RATE}Hz")
    print("=" * 60)

    if not API_KEY or not WORKSPACE_ID or not APP_ID:
        print("✗ 请在 .env 文件中配置 DASHSCOPE_API_KEY, WORKSPACE_ID, APP_ID")
        sys.exit(1)

    print("\n[TTS测试] 正在连接百炼 WebSocket...")
    
    headers = {
        "Authorization": f"Bearer {API_KEY}",
        "User-Agent": "test-tts/1.0",
    }
    ws = websocket.WebSocketApp(
        WS_URL, header=headers,
        on_open=on_open, on_message=on_message,
        on_data=on_data, on_error=on_error, on_close=on_close,
    )
    
    # 在后台线程运行 WebSocket
    ws_thread = threading.Thread(target=ws.run_forever, daemon=True)
    ws_thread.start()

    # 等待测试完成 (最多 60 秒)
    print("[TTS测试] 等待 TTS 音频...")
    finished = result.responding_ended.wait(timeout=60)
    
    # 等一小段时间让最后的音频到达
    time.sleep(2)

    # 保存音频
    if result.tts_bytes > 0:
        output_file = Path(__file__).parent / "tts_output.pcm"
        output_file.write_bytes(bytes(result.tts_audio_data))
        print(f"\n[TTS测试] TTS 音频已保存: {output_file}")
        print(f"  格式: PCM {DOWNSTREAM_SAMPLE_RATE}Hz 16bit mono")
        print(f"  大小: {result.tts_bytes} bytes")
        duration = result.tts_bytes / (DOWNSTREAM_SAMPLE_RATE * 2)  # 16bit = 2 bytes
        print(f"  时长: {duration:.1f} 秒")

    # 打印测试结果汇总
    print("\n" + "=" * 60)
    print("  TTS 测试结果汇总")
    print("=" * 60)
    print(f"  WebSocket 连接:  {'✓ 成功' if result.connected else '✗ 失败'}")
    print(f"  任务启动:        {'✓ 成功' if result.task_started else '✗ 失败'}")
    print(f"  会话ID:          {result.dialog_id or '无'}")
    print(f"  状态流转:        {' -> '.join(result.states)}")
    print(f"  事件序列:        {' -> '.join(result.events[:10])}")
    print(f"  LLM 回复文本:    {result.response_text[:200] if result.response_text else '(无)'}")
    print(f"  TTS 音频 chunks: {result.tts_chunks}")
    print(f"  TTS 音频 bytes:  {result.tts_bytes}")
    
    if result.error and result.tts_chunks == 0:
        print(f"\n  ✗ 错误: {result.error}")
        print("\n  [诊断建议]")
        if "cosyvoice" in str(result.error).lower() or "voice" in str(result.error).lower():
            print("  → TTS Voice 不兼容! 请在 .env 中设置 VOICE=longanhuan")
        elif "401" in str(result.error) or "auth" in str(result.error).lower():
            print("  → API Key 无效! 请检查 DASHSCOPE_API_KEY")
        elif "403" in str(result.error):
            print("  → 权限不足! 请检查 WORKSPACE_ID 和 APP_ID")
        print(f"  [结果] ✗ TTS 测试失败")
    elif result.tts_chunks > 0:
        print(f"\n  [结果] ✓ TTS 测试通过! 语音合成正常工作")
        print(f"  → LLM 回复: {result.response_text[:100]}...")
        print(f"  → TTS 音频: {result.tts_chunks} chunks, {result.tts_bytes} bytes")
    else:
        print(f"\n  [结果] ? TTS 测试不确定 — 未收到音频数据")

    # 关闭 WebSocket
    try:
        ws.close()
    except Exception:
        pass
    
    sys.exit(0 if (result.tts_chunks > 0) else 1)

if __name__ == "__main__":
    main()
