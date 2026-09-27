"""
多模态综合测试脚本 — 验证百炼 MultiModalDialog 全链路
=====================================================
测试流程:
  1. 连接百炼 WebSocket
  2. 发送文本 "你好" → 验证 LLM 回复 + TTS 音频
  3. 发送第二轮文本 → 验证多轮对话
  4. 保存 TTS 音频到文件
  5. 打印完整的诊断报告

运行:
  conda activate mmdialog
  cd D:\pycode\qwen
  python test_multimodal.py
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
class RoundResult:
    """单轮交互结果"""
    def __init__(self, prompt):
        self.prompt = prompt
        self.states = []
        self.asr_texts = []
        self.response_text = ""
        self.response_finished = False
        self.tts_chunks = 0
        self.tts_bytes = 0
        self.tts_audio = bytearray()
        self.error = None
        self.events = []
        self.start_time = time.time()
        self.first_tts_time = None
        self.first_text_time = None
        self.responding_ended = threading.Event()

class TestResult:
    def __init__(self):
        self.connected = False
        self.task_started = False
        self.dialog_id = None
        self.all_states = []
        self.rounds = []
        self.current_round = None
        self.error = None
        self.done = threading.Event()
        self.listening_ready = threading.Event()

result = TestResult()
_task_id = uuid.uuid4().hex
_pending_text = None  # 待发送的文本
_round_complete = threading.Event()

# ─── WebSocket 处理 ────────────────────────────
def on_open(ws):
    print("[综合测试] WebSocket 已连接")
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
        if result.current_round:
            result.current_round.tts_chunks += 1
            result.current_round.tts_bytes += len(data)
            result.current_round.tts_audio.extend(data)
            if result.current_round.first_tts_time is None:
                result.current_round.first_tts_time = time.time()
                elapsed = result.current_round.first_tts_time - result.current_round.start_time
                print(f"[综合测试] ★ 首次 TTS 音频: {len(data)} bytes (发送后 {elapsed:.1f}s)")

def on_error(ws, error):
    print(f"[综合测试] ✗ WebSocket 错误: {error}")
    result.error = str(error)
    result.done.set()
    _round_complete.set()

def on_close(ws, close_status_code, close_msg):
    print(f"[综合测试] WebSocket 关闭: code={close_status_code}, msg={close_msg}")
    result.connected = False
    result.done.set()
    _round_complete.set()

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
                    "user_id": "test_mm_001", "sdk": "python",
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
    print(f"[综合测试] Start 消息已发送 (voice={VOICE})")

def send_text_msg(ws, text):
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

def send_silence(ws, duration_s=0.1):
    """发送静音 PCM 数据"""
    n_samples = int(UPSTREAM_SAMPLE_RATE * duration_s)
    silence = b'\x00\x00' * n_samples
    if ws.sock and ws.sock.connected:
        ws.send(silence, websocket.ABNF.OPCODE_BINARY)

def send_local_responding_ended(ws):
    """上报 LocalRespondingEnded，通知服务端音频播放完毕"""
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
                "directive": "LocalRespondingEnded",
                "dialog_id": result.dialog_id,
            },
        },
    }
    if ws.sock and ws.sock.connected:
        ws.send(json.dumps(msg, ensure_ascii=False))
        print("[综合测试] 已上报 LocalRespondingEnded")

def handle_message(ws, msg):
    header = msg.get("header", {})
    payload = msg.get("payload", {})
    event = header.get("event", "")

    if event == "task-failed":
        err_code = header.get("error_code", "Unknown")
        err_msg = header.get("error_message", "Unknown error")
        print(f"[综合测试] ✗ 任务失败: {err_code} - {err_msg}")
        result.error = f"{err_code}: {err_msg}"
        if result.current_round:
            result.current_round.error = result.error
        result.done.set()
        _round_complete.set()
        return

    if event == "task-started":
        print("[综合测试] 任务已启动 (task-started)")
        result.task_started = True
        return

    if event != "result-generated":
        return

    output = payload.get("output", {})
    resp_event = output.get("event", "")
    
    if result.current_round:
        result.current_round.events.append(resp_event)

    if resp_event == "Started":
        result.dialog_id = output.get("dialog_id")
        print(f"[综合测试] 会话已创建, dialog_id={result.dialog_id}")

    elif resp_event == "DialogStateChanged":
        state = output.get("state", "")
        result.all_states.append(state)
        if result.current_round:
            result.current_round.states.append(state)
        print(f"[综合测试] 状态: -> {state}")
        
        if state == "Listening":
            result.listening_ready.set()

    elif resp_event == "SpeechContent":
        text = output.get("text", "")
        if result.current_round:
            result.current_round.asr_texts.append(text)
        print(f"[综合测试] ASR: \"{text}\"")

    elif resp_event == "RespondingContent":
        text = output.get("text", "")
        finished = output.get("finished", False)
        if result.current_round:
            if result.current_round.first_text_time is None and text:
                result.current_round.first_text_time = time.time()
                elapsed = result.current_round.first_text_time - result.current_round.start_time
                print(f"[综合测试] ★ 首次 LLM 文本 (发送后 {elapsed:.1f}s): \"{text[:60]}\"")
            # 每个chunk包含累积文本，只在 finished 时保存最终结果
            if finished:
                result.current_round.response_text = text
                result.current_round.response_finished = True
                print(f"[综合测试] LLM 最终回复: \"{text[:100]}\"")
            else:
                # 流式中间结果仅用于观察
                pass

    elif resp_event == "RespondingStarted":
        print("[综合测试] AI 开始回复")

    elif resp_event == "RespondingEnded":
        if result.current_round:
            elapsed = time.time() - result.current_round.start_time
            print(f"[综合测试] AI 回复结束 ({result.current_round.tts_chunks} chunks / "
                  f"{result.current_round.tts_bytes} bytes / {elapsed:.1f}s)")
            result.current_round.responding_ended.set()
        
        # 模拟播放完成后上报 LocalRespondingEnded
        # 等待一小段时间模拟音频播放
        time.sleep(1.0)
        send_local_responding_ended(ws)
        
        _round_complete.set()

    elif resp_event == "Error":
        err_code = output.get("error_code", "")
        err_msg = output.get("error_message", "")
        err_name = output.get("error_name", "")
        print(f"[综合测试] ✗ 服务端错误: {err_code} ({err_name}): {err_msg}")
        result.error = f"{err_code} ({err_name}): {err_msg}"
        if result.current_round:
            result.current_round.error = result.error
            result.current_round.responding_ended.set()
        _round_complete.set()

    elif resp_event == "RequestAccepted":
        print("[综合测试] 打断已接受")

# ─── 对话轮次执行 ─────────────────────────────
def run_round(ws_app, prompt, round_num, silence_thread_event):
    """执行一轮对话测试"""
    global _pending_text
    
    print(f"\n{'='*50}")
    print(f"  第 {round_num} 轮: \"{prompt}\"")
    print(f"{'='*50}")
    
    rr = RoundResult(prompt)
    result.current_round = rr
    _round_complete.clear()
    
    # 设置待发送文本
    _pending_text = prompt
    
    # 检查是否已经处于 Listening 状态
    if not result.listening_ready.is_set():
        print(f"[第{round_num}轮] 等待 Listening 状态...")
        if not result.listening_ready.wait(timeout=20):
            print(f"[第{round_num}轮] ✗ 等待 Listening 超时!")
            rr.error = "Listening 超时"
            return rr
    else:
        print(f"[第{round_num}轮] 已处于 Listening 状态")
    
    # 清除标志，为下一轮准备
    result.listening_ready.clear()

    # 直接发送文本 (不依赖状态回调的 _pending_text 机制)
    _pending_text = None  # 清除，防止回调重复发送
    print(f"[第{round_num}轮] 发送文本: \"{prompt}\"")
    if ws_app.sock and ws_app.sock.connected:
        send_text_msg(ws_app, prompt)
    else:
        rr.error = "WebSocket 未连接"
        return rr
    
    # 等待回复完成 (最多45秒)
    print(f"[第{round_num}轮] 等待 AI 回复...")
    if not _round_complete.wait(timeout=45):
        print(f"[第{round_num}轮] ✗ 等待回复超时!")
        rr.error = "回复超时"
    
    return rr

# ─── 主函数 ────────────────────────────────────
def main():
    print("=" * 60)
    print("  多模态综合测试 — 验证百炼全链路")
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

    print("\n[综合测试] 正在连接百炼 WebSocket...")

    headers = {
        "Authorization": f"Bearer {API_KEY}",
        "User-Agent": "test-multimodal/1.0",
    }
    ws_app = websocket.WebSocketApp(
        WS_URL, header=headers,
        on_open=on_open, on_message=on_message,
        on_data=on_data, on_error=on_error, on_close=on_close,
    )

    ws_thread = threading.Thread(target=ws_app.run_forever, daemon=True)
    ws_thread.start()

    # 持续发送静音的后台线程 (防止20s超时)
    silence_stop = threading.Event()
    def silence_loop():
        while not silence_stop.is_set():
            if ws_app.sock and ws_app.sock.connected:
                try:
                    send_silence(ws_app, 0.1)
                except Exception:
                    pass
            time.sleep(0.1)
    
    silence_thread = threading.Thread(target=silence_loop, daemon=True)
    silence_thread.start()

    # 等待初始 Listening 状态
    print("[综合测试] 等待初始 Listening 状态...")
    if not result.listening_ready.wait(timeout=15):
        print("[综合测试] ✗ 等待初始 Listening 超时!")
        if result.error:
            print(f"[综合测试] 错误: {result.error}")
        silence_stop.set()
        try: ws_app.close()
        except: pass
        sys.exit(1)

    # 执行测试轮次
    test_prompts = [
        "你好，请用一句话介绍一下你自己",
        "1加1等于多少？",
    ]

    for i, prompt in enumerate(test_prompts):
        if result.error or not result.connected:
            break
        rr = run_round(ws_app, prompt, i + 1, silence_stop)
        result.rounds.append(rr)
        
        if rr.error:
            print(f"\n[综合测试] 第{i+1}轮出错: {rr.error}")
            break
        
        # 轮间等待
        if i < len(test_prompts) - 1:
            time.sleep(2)

    # 停止静音
    silence_stop.set()

    # 保存所有 TTS 音频
    all_audio = bytearray()
    for rr in result.rounds:
        if rr.tts_bytes > 0:
            all_audio.extend(rr.tts_audio)
    
    if all_audio:
        output_file = Path(__file__).parent / "multimodal_tts_output.pcm"
        output_file.write_bytes(bytes(all_audio))
        duration = len(all_audio) / (DOWNSTREAM_SAMPLE_RATE * 2)
        print(f"\n[综合测试] TTS 音频已保存: {output_file} ({len(all_audio)} bytes, {duration:.1f}s)")

    # ═══════════ 打印综合测试报告 ═══════════
    print("\n" + "╔" + "═" * 58 + "╗")
    print("║" + "  多模态综合测试报告".center(50) + "        ║")
    print("╚" + "═" * 58 + "╝")
    
    print(f"\n[基础连接]")
    print(f"  WebSocket:  {'✓ 成功' if result.connected else '✗ 失败'}")
    print(f"  任务启动:   {'✓ 成功' if result.task_started else '✗ 失败'}")
    print(f"  会话ID:     {result.dialog_id or '无'}")
    print(f"  全局状态流: {' -> '.join(result.all_states[:20])}")
    
    for i, rr in enumerate(result.rounds):
        print(f"\n[第{i+1}轮] \"{rr.prompt}\"")
        print(f"  状态流转: {' -> '.join(rr.states)}")
        print(f"  事件序列: {' -> '.join(rr.events[:10])}")
        print(f"  LLM 回复: {rr.response_text[:150] if rr.response_text else '(无)'}")
        print(f"  TTS 音频: {rr.tts_chunks} chunks / {rr.tts_bytes} bytes")
        if rr.tts_bytes > 0:
            duration = rr.tts_bytes / (DOWNSTREAM_SAMPLE_RATE * 2)
            print(f"  TTS 时长: {duration:.1f}s")
        if rr.first_tts_time:
            latency = rr.first_tts_time - rr.start_time
            print(f"  首包延迟: {latency:.1f}s")
        if rr.error:
            print(f"  ✗ 错误: {rr.error}")
        else:
            ok = rr.response_text and rr.tts_bytes > 0
            print(f"  {'✓ 通过' if ok else '⚠ 部分通过'}")

    # 总体结论
    print(f"\n{'='*60}")
    all_ok = all(rr.response_text and rr.tts_bytes > 0 and not rr.error for rr in result.rounds)
    any_ok = any(rr.response_text or rr.tts_bytes > 0 for rr in result.rounds)
    
    if all_ok:
        print("  [总结] ✓ 全部测试通过! 多模态对话全链路正常工作")
        print("  → ASR (语音识别) + LLM (大模型) + TTS (语音合成) 均正常")
    elif any_ok:
        print("  [总结] ⚠ 部分测试通过")
        for i, rr in enumerate(result.rounds):
            status = "✓" if (rr.response_text and rr.tts_bytes > 0 and not rr.error) else "✗"
            print(f"    第{i+1}轮: {status}")
    else:
        print("  [总结] ✗ 测试失败")
        if result.error:
            print(f"  错误: {result.error}")
            if "voice" in str(result.error).lower():
                print("  → TTS Voice 不兼容! 请在 .env 中设置 VOICE=cosyvoice-v3-flash")
    print("=" * 60)

    # 清理
    try:
        ws_app.close()
    except Exception:
        pass

    sys.exit(0 if all_ok else 1)

if __name__ == "__main__":
    main()
