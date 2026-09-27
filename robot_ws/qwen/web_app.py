"""
阿里云百炼 MultiModalDialog — Web 交互界面 v3 (官方SDK版)
==========================================================
使用 dashscope 官方 MultiModalDialog SDK, 不再手写 WebSocket 协议。

核心改进:
  1. 使用 dashscope.multimodal.MultiModalDialog 官方 SDK
  2. 下游采样率 48kHz, 与浏览器 AudioContext 匹配, 无需重采样
  3. 上游音频 16kHz/16bit, 浏览器降采样后直接传给 SDK
  4. 延迟连接 — 等浏览器麦克风就绪后才建立连接
  5. 会话级日志 — 每次启动创建 logs/YYYYMMDD_HHMMSS/ 目录
  6. VQA 图片保存 — 拍照图片保存到会话目录

运行:
    conda activate mmdialog
    cd D:\\pycode\\qwen
    python web_app.py
    浏览器打开 http://localhost:8765
"""

import os
import sys
import io
import json
import time
import uuid
import logging
import asyncio
import threading
import base64
import cv2
import subprocess
import re
from datetime import datetime
from pathlib import Path
from typing import Optional, Dict, Any
from aiortc import MediaStreamTrack, RTCPeerConnection, RTCSessionDescription
from av import VideoFrame

# 修复 Windows 控制台 cp950 编码不支持中文的问题
if sys.stdout and hasattr(sys.stdout, 'reconfigure'):
    try:
        sys.stdout.reconfigure(encoding='utf-8', errors='replace')
    except Exception:
        pass
if sys.stderr and hasattr(sys.stderr, 'reconfigure'):
    try:
        sys.stderr.reconfigure(encoding='utf-8', errors='replace')
    except Exception:
        pass

from aiohttp import web
from dotenv import load_dotenv

# 官方 SDK
from dashscope.multimodal.dialog_state import DialogState
from dashscope.multimodal.multimodal_dialog import MultiModalDialog, MultiModalCallback
from dashscope.multimodal.multimodal_request_params import (
    Upstream, Downstream, ClientInfo, RequestParameters,
    Device, RequestToRespondParameters, DialogAttributes,
)

# ─── 配置 ────────────────────────────────────────────────
load_dotenv()
API_KEY = os.getenv("DASHSCOPE_API_KEY", "")
WORKSPACE_ID = os.getenv("WORKSPACE_ID", "")
APP_ID = os.getenv("APP_ID", "")
VOICE = os.getenv("VOICE", "app_default")
SYSTEM_PROMPT = os.getenv("SYSTEM_PROMPT", "")
HOST = os.getenv("WEB_HOST", "0.0.0.0")
PORT = int(os.getenv("WEB_PORT", "8765"))
BASE_DIR = Path(__file__).parent.resolve()
audio_process = None

# 下游采样率固定 48kHz, 与浏览器 AudioContext 匹配
DOWNSTREAM_SAMPLE_RATE = 48000
# 上游 (麦克风) 采样率: 浏览器降采样到 16kHz 后发送
UPSTREAM_SAMPLE_RATE = 16000

WEBSOCKET_URL = "wss://dashscope.aliyuncs.com/api-ws/v1/inference"
MODEL_NAME = "multimodal-dialog"

VOICES = {
    "app_default": "使用UUMO应用配置 (推荐)",
    "longanhuan": "龙安欢 (女声·欢快)",
    "longanyang": "龙安阳 (男声·沉稳)",
    "longhuhu_v3": "龙呼呼 (女声·天真)",
    "longwangwang_v3": "龙汪汪 (女声·甜美)",
    "longanshuo_v3": "龙安说 (男声·自然)",
    "longfeifei_v3": "龙菲菲 (女声·清新)",
    "longshu_v3": "龙书 (男声·知性)",
    "longjing_v3": "龙静 (女声·优雅)",
}

# ─── 会话级日志 ──────────────────────────────────────────
SESSION_TS = datetime.now().strftime("%Y%m%d_%H%M%S")
SESSION_DIR = BASE_DIR / "logs" / SESSION_TS
SESSION_DIR.mkdir(parents=True, exist_ok=True)
LOG_FILE = SESSION_DIR / "session.log"

logger = logging.getLogger("mmdialog_web")
logger.setLevel(logging.DEBUG)
_fmt = logging.Formatter("%(asctime)s.%(msecs)03d [%(levelname)-5s] %(message)s", "%H:%M:%S")

_sh = logging.StreamHandler(sys.stdout)
_sh.setFormatter(_fmt)
logger.addHandler(_sh)

_fh = logging.FileHandler(str(LOG_FILE), encoding="utf-8")
_fh.setFormatter(_fmt)
logger.addHandler(_fh)

logger.info(f"[Init] 日志目录: {SESSION_DIR}")
_compat_fh = logging.FileHandler(str(BASE_DIR / "mmdialog.log"), encoding="utf-8")
_compat_fh.setFormatter(_fmt)
logger.addHandler(_compat_fh)


def detect_robot_cameras():
    found_devices = []
    for f in sorted(os.listdir('/dev')):
        if f.startswith('video'):
            path = f"/dev/{f}"
            if os.access(path, os.R_OK | os.W_OK):
                found_devices.append(path)
    return found_devices


def detect_robot_audio_devices(mode="input"):
    """
    精准检测系统中所有可用的物理音频设备
    不再硬编码特定品牌，增强通用性
    """
    devices = []

    # 1. PulseAudio 探测
    try:
        cmd = "pactl list sources short" if mode == "input" else "pactl list sinks short"
        res = subprocess.check_output(cmd, shell=True).decode('utf-8')

        for line in res.splitlines():
            parts = line.split('\t')
            if len(parts) >= 2:
                device_name = parts[1]
                if mode == "input" and ".monitor" in device_name:
                    continue

                devices.append({
                    "id": device_name,
                    "name": f"PulseAudio: {device_name}"
                })

        if devices:
            logger.info(f"[Hardware] 发现 {len(devices)} 个 PulseAudio 设备")
            return devices
    except Exception as e:
        logger.warning(f"[Hardware] PulseAudio 探测跳过: {e}")

    return devices

def start_audio_process():
    global audio_process
    cmd = ["paplay", "--raw", "--rate=48000", "--format=s16le", "--channels=1"]
    audio_process = subprocess.Popen(cmd, stdin=subprocess.PIPE)


def stop_and_reset_audio_process():
    global audio_process
    # 立即杀死当前的播放进程
    if audio_process:
        try:
            audio_process.kill()
            audio_process.wait()
            logger.info("[Audio] 已强制终止 paplay 进程以清空缓冲区")
        except Exception as e:
            logger.warning(f"[Audio] 终止音频进程失败: {e}")

    # 重新启动一个新的播放进程，准备接收下一段音频
    start_audio_process()
    logger.info("[Audio] 音频播放进程已重置")

def play_audio_chunk(chunk):
    global audio_process
    if audio_process and audio_process.poll() is None:
        try:
            audio_process.stdin.write(chunk)
            audio_process.stdin.flush()
        except Exception as e:
            print(f"播放出错: {e}")


def capture_snapshot_from_url():
    video_url = "http://localhost:5000/video_feed/cam3"
    cap = cv2.VideoCapture(video_url)

    if not cap.isOpened():
        print(f"[Error] 无法连接到视频流: {video_url}")
        return None

    ret, frame = cap.read()
    cap.release()

    if ret:
        # 可选：如果分辨率过高，可以先缩小尺寸（例如按 0.75 或 0.5 缩放）
        h, w = frame.shape[:2]
        frame = cv2.resize(frame, (int(w * 0.8), int(h * 0.8)))

        # 核心：设置 JPEG 压缩质量（默认通常是 95，这里降低到 65 左右以减小文件体积）
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 65]
        _, buffer = cv2.imencode('.jpg', frame, encode_param)
        
        print(f"[VQA] 压缩后图片大小: {len(buffer)} bytes")
        return buffer.tobytes()
    return None

# ═══════════════════════════════════════════════════════════
#  百炼 SDK 客户端 (使用官方 dashscope SDK)
# ═══════════════════════════════════════════════════════════

class _WebCallback(MultiModalCallback):
    """SDK 回调实现 — 将事件桥接到 asyncio 事件队列"""

    def __init__(self, owner: 'BailianDialog'):
        self._owner = owner

    def on_connected(self):
        logger.info("[Bailian] 已连接到服务器")
        self._owner._emit("connected")

    def on_started(self, dialog_id: str):
        self._owner.dialog_id = dialog_id
        logger.info(f"[Bailian] 会话已创建, dialog_id={dialog_id}")
        self._owner._emit("started", dialog_id)

    def on_stopped(self):
        logger.info("[Bailian] 会话已停止")
        self._owner._emit("stopped")

    def on_state_changed(self, state: DialogState):
        state_str = state.name if hasattr(state, 'name') else str(state)
        # DialogState 枚举值: IDLE, LISTENING, THINKING, RESPONDING
        state_map = {
            "IDLE": "IDLE", "LISTENING": "Listening",
            "THINKING": "Thinking", "RESPONDING": "Responding",
        }
        mapped = state_map.get(state_str, state_str)
        old = self._owner.state
        self._owner.state = mapped
        logger.info(f"[Bailian] 状态: {old} -> {mapped}")
        self._owner._emit("state_changed", mapped)
        if mapped == "Listening":
            self._owner.round_count += 1
            logger.info(f"[Bailian] 就绪, 第 {self._owner.round_count} 轮")

    def on_speech_audio_data(self, data: bytes):
        """TTS 音频数据 (PCM, sample_rate=48kHz)"""
        self._owner._emit("tts_audio", data)

    def on_speech_content(self, payload: Dict[str, Any]):
        """用户说话内容 (ASR)"""
        if payload:
            output = payload.get("output", payload)
            text = output.get("text", "")
            finished = output.get("finished", False)
            if text:
                self._owner._emit("asr_content", {"text": text, "finished": finished})
                logger.debug(f"[ASR] {'[final]' if finished else '[mid]'}: {text[:80]}")

    def on_speech_started(self):
        logger.debug("[Bailian] 用户开始说话 (SpeechStarted)")
        self._owner._emit("speech_started")

        if self._owner.state == "Responding":
            logger.info("[Bailian] 检测到用户在 AI 回复时发言，触发自动打断")
            self._owner.trigger_interruption()

    def on_speech_ended(self):
        logger.debug("[Bailian] 用户停止说话 (SpeechEnded)")
        self._owner._emit("speech_ended")

    def on_responding_started(self):
        logger.info("[Bailian] AI 开始回复 (RespondingStarted)")
        self._owner._emit("responding_started")

    def on_responding_ended(self, payload: Dict[str, Any]):
        logger.info("[Bailian] AI 回复结束 (RespondingEnded)")
        self._owner._emit("responding_ended")

    def on_responding_content(self, payload: Dict[str, Any]):
        """AI 回复内容 (LLM)"""
        if payload:
            output = payload.get("output", payload)
            text = output.get("text", "")
            finished = output.get("finished", False)
            extra_info = output.get("extra_info", {})
            if text:
                self._owner._emit("llm_content", {
                    "text": text, "finished": finished, "extra_info": extra_info,
                })
                logger.debug(f"[LLM] {'[final]' if finished else '[mid]'}: {text[:80]}")

    def on_request_accepted(self):
        logger.info("[Bailian] 打断已接受 (RequestAccepted)")
        self._owner._emit("interrupted")

    def on_error(self, error: Exception):
        logger.error(f"[Bailian] SDK 错误: {error}")
        self._owner._emit("error", str(error))

    def on_close(self, close_status_code: int, close_msg: str):
        self._owner.connected = False
        logger.info(f"[Bailian] 连接关闭: code={close_status_code}, msg={close_msg}")
        self._owner._emit("disconnected", f"code={close_status_code}")


class BailianDialog:
    """
    使用 dashscope 官方 MultiModalDialog SDK 与百炼 API 通信。
    SDK 内部自动处理 WebSocket 协议、心跳、消息编码等。
    """

    def __init__(self, api_key, workspace_id, app_id, voice, upstream_mode="duplex"):
        self.dialog = None
        self._drop_incoming_audio = False
        self.api_key = api_key
        self.workspace_id = workspace_id
        self.app_id = app_id
        self.voice = voice
        self.upstream_mode = upstream_mode  # "duplex" | "push2talk"

        self.dialog_id = None
        self.state = "IDLE"
        self.round_count = 0
        self.connected = False

        self._sdk_dialog: Optional[MultiModalDialog] = None
        self._callback: Optional[_WebCallback] = None
        self._on_event = None  # callback(event_type, data) — 桥接到 asyncio

    def start(self, on_event):
        """启动百炼连接 (SDK 内部在独立线程中运行)"""
        self._on_event = on_event
        self.dialog_id = None
        self.state = "IDLE"
        self.round_count = 0

        # 构建 SDK 请求参数
        # 使用 AudioAndVideo 类型以支持 VQA 图片问答
        # (参照官方 run_vqa.py 示例, VQA 需要 AudioAndVideo 类型)
        up_stream = Upstream(
            type="AudioAndVideo",
            mode=self.upstream_mode,
            audio_format="pcm",
        )
        # 下游参数: 48kHz 与浏览器 AudioContext 匹配
        downstream_kwargs = {"sample_rate": DOWNSTREAM_SAMPLE_RATE}
        # 仅在用户明确选择了特定声音时才传 voice 参数
        if self.voice and self.voice != "app_default":
            downstream_kwargs["voice"] = self.voice
            logger.info(f"[Bailian] 使用客户端指定声音: {self.voice}")
        else:
            logger.info("[Bailian] 使用 UUMO 应用配置的声音")

        client_info = ClientInfo(
            user_id="web_user_001",
            device=Device(uuid=f"web-{int(time.time())}")
        )
        # 如果配置了系统提示词, 通过 DialogAttributes 传给 SDK
        dialog_attrs = None
        if SYSTEM_PROMPT:
            dialog_attrs = DialogAttributes(prompt=SYSTEM_PROMPT)
            logger.info(f"[Bailian] 系统提示词: \"{SYSTEM_PROMPT[:60]}...\"")
        request_params = RequestParameters(
            upstream=up_stream,
            downstream=Downstream(**downstream_kwargs),
            client_info=client_info,
            dialog_attributes=dialog_attrs,
        )

        # 创建回调和 SDK 实例
        self._callback = _WebCallback(self)
        self._sdk_dialog = MultiModalDialog(
            app_id=self.app_id,
            workspace_id=self.workspace_id,
            url=WEBSOCKET_URL,
            request_params=request_params,
            multimodal_callback=self._callback,
            api_key=self.api_key,
            dialog_id="",
            model=MODEL_NAME,
        )

        # SDK start() 在独立线程中运行 (阻塞直到连接建立或失败)
        thread = threading.Thread(target=self._run_sdk, daemon=True, name="BailianSDK")
        thread.start()
        logger.info("[Bailian] SDK 线程已启动, 正在连接...")

    def _run_sdk(self):
        try:
            self._sdk_dialog.start("")
            logger.info("[Bailian] SDK start() 完成")
        except Exception as e:
            logger.error(f"[Bailian] SDK start() 异常: {e}")
            self._emit("error", str(e))

    def _emit(self, event_type, data=None):
        if self._on_event:
            self._on_event(event_type, data)

    # ─── 公开接口 ─────────────────────────────────────

    def send_audio(self, pcm_bytes):
        """发送音频数据到百炼 (PCM 16kHz/16bit mono)"""
        if self._sdk_dialog:
            try:
                self._sdk_dialog.send_audio_data(pcm_bytes)
            except Exception as e:
                logger.warning(f"[Bailian] 发送音频失败: {e}")

    def start_speech(self):
        """开始语音交互 (push2talk 模式需要调用)"""
        if self._sdk_dialog:
            try:
                self._sdk_dialog.start_speech()
                logger.debug("[Bailian] start_speech() 已调用")
            except Exception as e:
                logger.warning(f"[Bailian] start_speech 失败: {e}")

    def stop_speech(self):
        """结束语音交互 (push2talk 模式需要调用)"""
        if self._sdk_dialog:
            try:
                self._sdk_dialog.stop_speech()
                logger.debug("[Bailian] stop_speech() 已调用")
            except Exception as e:
                logger.warning(f"[Bailian] stop_speech 失败: {e}")

    def local_responding_started(self):
        if self._sdk_dialog:
            try:
                self._sdk_dialog.local_responding_started()
                logger.debug("[Bailian] 已上报 LocalRespondingStarted")
            except Exception as e:
                logger.warning(f"[Bailian] local_responding_started 失败: {e}")

    def local_responding_ended(self):
        if self._sdk_dialog:
            try:
                self._sdk_dialog.local_responding_ended()
                logger.debug("[Bailian] 已上报 LocalRespondingEnded")
            except Exception as e:
                logger.warning(f"[Bailian] local_responding_ended 失败: {e}")

    def interrupt(self):
        if self._sdk_dialog:
            try:
                self._sdk_dialog.interrupt()
                logger.info("[Bailian] 已发送打断请求")
            except Exception as e:
                logger.warning(f"[Bailian] 打断失败: {e}")

    def send_text(self, text, type_="prompt"):
        """发送文本消息 (通过 SDK 的 request_to_respond)"""
        if self._sdk_dialog:
            try:
                self._sdk_dialog.request_to_respond(type_, text)
                logger.info(f"[Bailian] 已发送文本 ({type_}): \"{text[:50]}\"")
            except Exception as e:
                logger.warning(f"[Bailian] 发送文本失败: {e}")

    def send_vqa_image(self, image_base64):
        """发送 VQA 图片 (通过 SDK 的 request_to_respond + images)"""
        # 保存图片到会话目录
        try:
            img_data = base64.b64decode(image_base64)
            img_path = SESSION_DIR / f"vqa_{datetime.now().strftime('%H%M%S_%f')}.jpg"
            img_path.write_bytes(img_data)
            logger.info(f"[VQA] 图片已保存: {img_path} ({len(img_data)} bytes)")
        except Exception as e:
            logger.warning(f"[VQA] 保存图片失败: {e}")

        if self._sdk_dialog:
            try:
                image = {"type": "base64", "value": image_base64}
                params = RequestToRespondParameters(images=[image])
                self._sdk_dialog.request_to_respond("prompt", "", parameters=params)
                logger.info(f"[Bailian] 已发送 VQA 图片, base64_len={len(image_base64)}")
            except Exception as e:
                logger.warning(f"[Bailian] VQA 发送失败: {e}")

    def send_heartbeat(self):
        if self._sdk_dialog:
            try:
                self._sdk_dialog.send_heart_beat()
            except Exception:
                pass

    def send_silence_audio(self):
        """发送静音 PCM 数据 (全零) 以维持百炼长连接。
        格式: 16kHz / 16bit / mono, 每次 200ms = 3200 采样 = 6400 bytes。
        参照阿里云 FunASR 实时 SDK 文档建议: 长时间无语音输入时
        持续发送静音音频防止连接超时断开。
        """
        if self._sdk_dialog and self.connected:
            try:
                silence = b'\x00' * 6400  # 200ms × 16kHz × 2bytes = 6400 bytes
                self._sdk_dialog.send_audio_data(silence)
            except Exception:
                pass

    def stop(self):
        if self._sdk_dialog:
            try:
                self._sdk_dialog.stop()
            except Exception:
                pass
            self.connected = False
            logger.info("[Bailian] SDK 已停止")

    def trigger_interruption(self):
        self._drop_incoming_audio = True  # 开启丢弃锁
        if self.dialog and self.dialog.connected:
            self.dialog.interrupt()
        stop_and_reset_audio_process()


class GlobalCameraManager:
    """全局摄像头管理器：缓存已打开的 cap 对象，避免频繁打开造成的硬件死锁"""

    def __init__(self):
        self.caps = {}  # 存储已打开的 VideoCapture 实例
        self.locks = {}  # 每个摄像头对应的互斥锁

    def _get_cap(self, cam_id):
        """内部方法：获取或懒加载打开摄像头"""
        if cam_id not in self.caps:
            # 打开摄像头并设置参数
            cap = cv2.VideoCapture(int(cam_id), cv2.CAP_V4L2)
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            self.caps[cam_id] = cap
            self.locks[cam_id] = threading.Lock()
            print(f"[Camera] 摄像头 {cam_id} 已成功打开并缓存。")
        return self.caps[cam_id]

    def safe_read_and_encode(self, cam_id):
        """读取最新的一帧并编码，不关闭摄像头句柄"""
        try:
            lock = self.locks.get(cam_id)
            if lock is None:
                # 首次调用时确保锁存在
                _ = self._get_cap(cam_id)
                lock = self.locks[cam_id]

            with lock:
                cap = self._get_cap(cam_id)
                # 清除缓冲区旧帧（防止延迟）
                for _ in range(2):
                    cap.grab()
                success, frame = cap.read()

                if success and frame is not None:
                    _, buffer = cv2.imencode('.jpg', frame)
                    return buffer.tobytes()
        except Exception as e:
            print(f"[Camera] 读取摄像头 {cam_id} 失败: {e}")
        return None

    def release_all(self):
        """在程序退出时统一关闭"""
        for cam_id, cap in self.caps.items():
            cap.release()
        self.caps.clear()


# 实例化全局管理器
cam_manager = GlobalCameraManager()

# ═══════════════════════════════════════════════════════════
#  Web 应用
# ═══════════════════════════════════════════════════════════
class WebApp:
    """aiohttp Web 服务器 — 桥接浏览器与百炼 SDK"""

    def __init__(self):
        self.dialog: Optional[BailianDialog] = None
        self.browsers = set()
        self.event_queue: Optional[asyncio.Queue] = None
        self.dialog_started = False
        self.heartbeat_task = None
        self._silence_task = None  # 静音保活任务
        self._audio_last_received_time = 0  # 最近一次收到浏览器音频的时间
        self._silence_chunks_sent = 0  # 静音保活发送统计
        self._user_disconnect = False  # 用户主动断开标志 (不触发自动重连)
        self._auto_reconnect_count = 0  # 自动重连次数统计
        self._audio_chunks_sent = 0
        self._audio_bytes_sent = 0
        self._audio_chunks_from_browser = 0
        self._audio_first_received = False
        self._audio_last_log_time = 0
        self._tts_chunks_received = 0
        self.upstream_mode = "duplex"
        self._speech_started = False  # push2talk 模式下跟踪是否已调用 start_speech
        self.selected_mic_id = None
        self.selected_speaker_id = None
        self.selected_cam_id = None
        self.audio_proc = None
        self.is_woken = False
        self._interrupt_task = None
        self._drop_incoming_audio = False

    async def start(self):
        self.event_queue = asyncio.Queue()
        start_audio_process()

        # 创建百炼对话客户端 (但不立即连接!)
        self.dialog = BailianDialog(
            api_key=API_KEY, workspace_id=WORKSPACE_ID, app_id=APP_ID,
            voice=VOICE,
        )

        # 事件处理 + 心跳 + 静音保活
        asyncio.create_task(self._event_loop())
        self.heartbeat_task = asyncio.create_task(self._heartbeat_loop())
        self._silence_task = asyncio.create_task(self._silence_loop())

        # 启动 Web 服务器
        app = web.Application()
        app.router.add_get("/", self._handle_index)
        app.router.add_get("/ws", self._handle_browser_ws)
        app.router.add_get('/video_feed', self._handle_video_feed)
        runner = web.AppRunner(app)
        await runner.setup()
        site = web.TCPSite(runner, HOST, PORT)
        await site.start()

        logger.info("=" * 60)
        logger.info(f"[Web] 服务器已启动: http://localhost:{PORT}")
        logger.info(f"[Web] 请在浏览器中打开 http://localhost:{PORT}")
        logger.info(f"[Web] 日志目录: {SESSION_DIR}")
        logger.info("[Web] 等待浏览器连接后, 将自动连接百炼服务器...")
        logger.info("=" * 60)

        try:
            while True:
                await asyncio.sleep(3600)
        except asyncio.CancelledError:
            pass

    async def _delayed_interrupt(self, delay: float):
        """延迟执行打断，用于过滤短促杂音"""
        try:
            await asyncio.sleep(delay)
            # 0.5秒后如果 AI 依然在回复，说明用户在持续说话，执行打断
            if self.dialog and self.dialog.state == "Responding":
                logger.info(f"[Bailian] 用户持续说话超过 {delay} 秒，触发语音打断 (Barge-in)")
                self.trigger_interruption()
        except asyncio.CancelledError:
            # 0.5秒内被取消（说明用户很快闭嘴了）
            pass

    def trigger_interruption(self):
        self._drop_incoming_audio = True  # 开启丢弃锁
        if self.dialog and self.dialog.connected:
            self.dialog.interrupt()
        stop_and_reset_audio_process()

    # ─── 百炼连接管理 ─────────────────────────────────
    def _start_bailian(self):
        if self.dialog_started and self.dialog.connected:
            logger.info("[Bailian] 已在连接中, 跳过重复启动")
            return
        if self.dialog_started:
            logger.info("[Bailian] 上次连接已断开, 重新创建...")
            self.dialog.stop()

        self.dialog_started = True
        self._audio_last_received_time = time.time()  # 重置音频时间戳
        self._silence_chunks_sent = 0  # 重置静音计数
        loop = asyncio.get_event_loop()
        self.dialog.start(lambda evt, data: loop.call_soon_threadsafe(
            self.event_queue.put_nowait, (evt, data)))
        logger.info("[Bailian] 正在通过 SDK 建立连接...")

    def _stop_bailian(self):
        self._user_disconnect = True  # 标记为用户主动断开, 防止触发自动重连
        if self.dialog:
            self.dialog.stop()
        self.dialog_started = False
        logger.info("[Bailian] 已停止")

    # ─── 事件处理 (百炼 → 浏览器) ────────────────────
    async def _event_loop(self):
        while True:
            try:
                event_type, data = await self.event_queue.get()
                await self._handle_bailian_event(event_type, data)
            except asyncio.CancelledError:
                break
            except Exception as e:
                logger.error(f"[EventLoop] 异常: {e}")

    async def _handle_bailian_event(self, event_type, data):
        if event_type == "connected":
            self.dialog.connected = True
            self._auto_reconnect_count = 0  # 连接成功后重置重连计数

        if event_type == "error":
            logger.error(f"[Bailian] SDK 错误: {data}")
            await self._broadcast_json("log", f"⚠ 百炼服务器错误: {data}")

        if event_type == "disconnected":
            self.dialog_started = False
            self.dialog.connected = False
            self._speech_started = False
            logger.warning(f"[Bailian] 连接断开: {data}")
            # ─── 自动重连: 非用户主动断开且有浏览器在线时, 3秒后自动重连 ───
            MAX_RECONNECT = 10
            if not self._user_disconnect and self.browsers:
                self._auto_reconnect_count += 1
                if self._auto_reconnect_count > MAX_RECONNECT:
                    logger.error(
                        f"[AutoReconnect] 已达最大重连次数 ({MAX_RECONNECT}), 停止自动重连"
                    )
                    await self._broadcast_json("log",
                                               f"自动重连已达上限 ({MAX_RECONNECT}次), 请手动点击\"重连\"按钮")
                else:
                    delay = min(3 + self._auto_reconnect_count, 15)  # 退避: 3s, 4s, 5s...最大15s
                    logger.info(
                        f"[AutoReconnect] 百炼意外断开, 第 {self._auto_reconnect_count} 次自动重连 ({delay}秒后)..."
                    )
                    await self._broadcast_json("log",
                                               f"百炼连接意外断开 (第{self._auto_reconnect_count}次), {delay}秒后自动重连...")
                    await asyncio.sleep(delay)
                    # 重连前再次确认浏览器仍在线且非用户主动断开
                    if self.browsers and not self._user_disconnect:
                        self._start_bailian()
                        await self._broadcast_json("log", "正在自动重连百炼服务器...")
                    else:
                        logger.info("[AutoReconnect] 浏览器已断开或用户主动断开, 取消重连")
            # 重置标志
            self._user_disconnect = False

        if event_type == "state_changed":
            # 每次进入 Listening 状态, 重置 push2talk 的 speech 状态
            if data == "Listening":
                self._speech_started = False
            # 当 AI 开始新一轮正式回复时，解除音频丢弃锁 ───
            elif data == "Responding":
                self._drop_incoming_audio = False

        elif event_type == "speech_started":
            logger.debug("[Bailian] 用户开始说话 (SpeechStarted)")
            # 如果 AI 正在回复，启动 0.5 秒延迟打断计时器
            if self.dialog and self.dialog.state == "Responding":
                if self._interrupt_task and not self._interrupt_task.done():
                    self._interrupt_task.cancel()
                self._interrupt_task = asyncio.create_task(self._delayed_interrupt(0.5))

        elif event_type == "speech_ended":
            logger.debug("[Bailian] 用户停止说话 (SpeechEnded)")
            # 如果用户说话很快就结束了（小于 0.5 秒），取消打断任务
            if self._interrupt_task and not self._interrupt_task.done():
                self._interrupt_task.cancel()
                self._interrupt_task = None
                logger.info("[Bailian] 用户说话时间小于 0.5 秒（判定为误触/短语），取消打断")

        if event_type == "tts_audio":
            # 如果处于打断丢弃期，直接拦截并丢弃残余切片 ───
            if getattr(self, "_drop_incoming_audio", False):
                logger.debug("[Audio] 丢弃被打断后的残余音频切片")
                return
            # SDK 返回 48kHz PCM
            # 1. 本地播放 (paplay, 机器人喇叭)
            play_audio_chunk(data)
            # 2. 同时广播给所有连接的浏览器 (用于独立聊天页 8766)
            await self._broadcast_audio(data)
        else:
            await self._broadcast_json(event_type, data)

    async def _broadcast_json(self, event_type, data=None):
        msg = json.dumps({"type": event_type, "data": data}, ensure_ascii=False)
        dead = set()
        for ws in self.browsers:
            try:
                await ws.send_str(msg)
            except Exception:
                dead.add(ws)
        self.browsers -= dead

    async def _broadcast_audio(self, pcm_bytes):
        """TTS 音频广播: SDK 已返回 48kHz PCM, 直接转发"""
        self._tts_chunks_received += 1
        dead = set()
        for ws in self.browsers:
            try:
                await ws.send_bytes(pcm_bytes)
            except Exception:
                dead.add(ws)
        self.browsers -= dead

    async def _heartbeat_loop(self):
        while True:
            await asyncio.sleep(45)
            if self.dialog and self.dialog.connected and self.dialog.dialog_id:
                self.dialog.send_heartbeat()
                logger.debug("[Heartbeat] 心跳已发送")

    async def _silence_loop(self):
        """静音保活: 当浏览器未发送音频时, 持续发送静音 PCM 给百炼 SDK。

        参照阿里云 FunASR 实时 SDK 建议:
          - 静音格式: 16kHz / 16bit / mono / 全零 PCM
          - 检测间隔: 1000ms (每秒检查一次)
          - 触发阈值: 浏览器 20 秒内未发送音频时启动
          - 收到浏览器音频后自动停止

        实测 10~20 秒无语音后说话仍可正常工作, 20 秒阈值
        远小于百炼超时 (约 10 分钟), 安全且低开销。
        静音音频不触发 VAD/ASR, 不产生额外 token 费用。
        """
        SILENCE_GAP = 20.0  # 浏览器超过 20 秒没发音频 → 开始发静音
        while True:
            await asyncio.sleep(1.0)  # 1000ms 间隔检查
            if not (self.dialog and self.dialog.connected and self.dialog.dialog_id):
                continue
            # 检查浏览器最近是否发送过音频
            elapsed = time.time() - self._audio_last_received_time
            if elapsed > SILENCE_GAP:
                self.dialog.send_silence_audio()
                self._silence_chunks_sent += 1
                # 每 30 个静音块 (约30秒) 打印一次日志, 避免刷屏
                if self._silence_chunks_sent % 30 == 1:
                    logger.info(
                        f"[Silence] 静音保活中: 已发送 {self._silence_chunks_sent} 块 "
                        f"(浏览器已 {elapsed:.0f}s 未发送音频)"
                    )

    # ─── HTTP 处理 ────────────────────────────────────
    async def _handle_index(self, request):
        html_path = BASE_DIR / "templates" / "index.html"
        resp = web.FileResponse(str(html_path))
        # 禁止浏览器缓存 HTML, 确保每次加载最新代码 (唤醒词等功能更新后立即生效)
        resp.headers['Cache-Control'] = 'no-store, no-cache, must-revalidate, max-age=0'
        resp.headers['Pragma'] = 'no-cache'
        resp.headers['Expires'] = '0'
        return resp

    # ─── 新增：機器狗本地視訊流處理 ────────────────────────
    async def _handle_video_feed(self, request):
        try:
            cam_id = int(request.query.get('cam', 0))
        except ValueError:
            cam_id = 0

        response = web.StreamResponse(
            status=200, reason='OK',
            headers={'Content-Type': 'multipart/x-mixed-replace; boundary=frame'}
        )
        await response.prepare(request)

        try:
            while True:
                frame_bytes = await asyncio.get_event_loop().run_in_executor(None, cam_manager.safe_read_and_encode, cam_id)

                if frame_bytes is None:
                    await asyncio.sleep(0.1)
                    continue

                await response.write(
                    b'--frame\r\n'
                    b'Content-Type: image/jpeg\r\n'
                    b'Content-Length: ' + str(len(frame_bytes)).encode() + b'\r\n\r\n' +
                    frame_bytes + b'\r\n'
                )
                await asyncio.sleep(1 / 15)
        except asyncio.CancelledError:
            pass
        return response

    # ─── 浏览器 WebSocket 处理 ────────────────────────
    async def _handle_browser_ws(self, request):
        ws = web.WebSocketResponse(max_msg_size=4 * 1024 * 1024)
        await ws.prepare(request)
        self.browsers.add(ws)
        logger.info(f"[Browser] 新连接, 当前 {len(self.browsers)} 个浏览器")

        # 发送初始状态
        await ws.send_str(json.dumps({
            "type": "init",
            "data": {
                "state": self.dialog.state if self.dialog else "IDLE",
                "connected": self.dialog.connected if self.dialog else False,
                "dialog_started": self.dialog_started,
                "dialog_id": self.dialog.dialog_id if self.dialog else None,
                "round_count": self.dialog.round_count if self.dialog else 0,
                "downstream_sample_rate": DOWNSTREAM_SAMPLE_RATE,
                "session_dir": str(SESSION_DIR.name),
            }
        }))

        # 自动检测并推送狗子相机列表（原逻辑保持不变，确保 4 索引默认首选）
        try:
            robot_cams = detect_robot_cameras()
            await ws.send_str(json.dumps({
                "type": "camera_list",
                "devices": robot_cams
            }))
        except Exception as e:
            logger.error(f"[Hardware] 检测或推送相机列表失败: {e}")

        # 新增：自动检测并推送狗子本地的麦克风输入硬件
        try:
            input_devs = detect_robot_audio_devices("input")
            logger.info(f"[Hardware] 初始自动推送狗子麦克风列表: {input_devs}")
            await ws.send_str(json.dumps({
                "type": "mic_list",
                "devices": input_devs
            }, ensure_ascii=False))
        except Exception as e:
            logger.error(f"[Hardware] 初始推送麦克风列表失败: {e}")

        # 新增：自动检测并推送狗子本地的喇叭输出硬件
        try:
            output_devs = detect_robot_audio_devices("output")
            logger.info(f"[Hardware] 初始自动推送狗子喇叭列表: {output_devs}")
            await ws.send_str(json.dumps({
                "type": "speaker_list",
                "devices": output_devs
            }, ensure_ascii=False))
        except Exception as e:
            logger.error(f"[Hardware] 初始推送喇叭列表失败: {e}")

        if self.selected_mic_id:
            logger.info(f"[Hardware] 恢复上次选择的麦克风: {self.selected_mic_id}")
            await ws.send_str(json.dumps({
                "type": "restore_device_state",
                "mic_id": self.selected_mic_id
            }))

        if self.selected_speaker_id:
            logger.info(f"[Hardware] 恢复上次选择的喇叭: {self.selected_speaker_id}")
            await ws.send_str(json.dumps({
                "type": "restore_device_state",
                "spk_id": self.selected_speaker_id
            }))

        # 保持连接并处理前后端后续的数据通信
        try:
            async for msg in ws:
                if msg.type == web.WSMsgType.BINARY:
                    self._handle_browser_audio(msg.data)
                elif msg.type == web.WSMsgType.TEXT:
                    await self._handle_browser_command(msg.data, ws)
                elif msg.type == web.WSMsgType.ERROR:
                    logger.error(f"[Browser] WS 错误: {ws.exception()}")
        except Exception as e:
            logger.error(f"[Browser] 连接异常: {e}")
        finally:
            self.browsers.discard(ws)
            logger.info(f"[Browser] 断开, 剩余 {len(self.browsers)} 个")
            if len(self.browsers) == 0 and self.dialog_started:
                logger.info("[Bailian] 所有浏览器已断开, 停止百炼连接...")
                self._stop_bailian()

        return ws

    def _handle_browser_audio(self, audio_bytes):
        if not self.is_woken:
            return
        """浏览器音频 → 直接发送给百炼 SDK (浏览器已降采样到16kHz)"""
        self._audio_chunks_from_browser += 1
        self._audio_last_received_time = time.time()  # 记录最后收到音频的时间
        if not self.dialog or not self.dialog.connected:
            if self._audio_chunks_from_browser <= 3:
                logger.warning(f"[Audio] 收到浏览器音频 (#{self._audio_chunks_from_browser}, "
                               f"{len(audio_bytes)} bytes) 但百炼未连接")
            return

        # push2talk 模式: 首次收到音频时自动调用 start_speech()
        # 参照官方示例 run.py 中的 start_speech_interaction() 行为
        if self.upstream_mode == "push2talk" and not self._speech_started:
            self._speech_started = True
            self.dialog.start_speech()
            logger.info("[Audio] push2talk 模式: 已自动调用 start_speech()")

        # 首次收到音频
        if not self._audio_first_received:
            self._audio_first_received = True
            logger.info(f"[Audio] ★ 首次收到浏览器音频: {len(audio_bytes)} bytes "
                        f"(16kHz/16bit, 直接转发给SDK)")

        # 浏览器已将音频降采样到 16kHz, 直接发送给 SDK
        self.dialog.send_audio(audio_bytes)
        self._audio_chunks_sent += 1
        self._audio_bytes_sent += len(audio_bytes)

        # 每5秒打印一次音频统计
        now = time.time()
        if now - self._audio_last_log_time >= 5.0:
            self._audio_last_log_time = now
            logger.info(f"[Audio] 5s统计: 浏览器→服务器 {self._audio_chunks_sent} chunks / "
                        f"{self._audio_bytes_sent} bytes / "
                        f"百炼状态={self.dialog.state}")

    async def _run_audio_capture(self, device_id):
        cmd = ["parec", "-d", device_id, "--format=s16le", "--rate=16000", "--channels=1"]
        logger.info(f"[Hardware] 启动 arecord 进程: {' '.join(cmd)}")

        self.audio_proc = await asyncio.create_subprocess_exec(
            *cmd,
            stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.DEVNULL
        )

        try:
            # 每次读取 3200 字节 (对应 16kHz/16bit 下的 100ms 音频数据)
            chunk_size = 3200
            while self.audio_proc.returncode is None:
                data = await self.audio_proc.stdout.read(chunk_size)
                if not data:
                    break
                # 推送给百炼 SDK
                if self.dialog and self.dialog.connected:
                    self.dialog.send_audio(data)
        except Exception as e:
            logger.error(f"[Hardware] 音频采集异常: {e}")
        finally:
            await self._stop_robot_audio_process()

    async def _stop_robot_audio_process(self):
        if self.audio_proc:
            try:
                self.audio_proc.terminate()
                await self.audio_proc.wait()
            except Exception:
                pass
            self.audio_proc = None
            logger.info("[Hardware] 录音进程已终止")

    async def _handle_browser_command(self, text, ws):
        """处理浏览器命令"""
        try:
            cmd = json.loads(text)
        except json.JSONDecodeError:
            return

        cmd_type = cmd.get("type", cmd.get("action", ""))

        if cmd_type == "start":
            browser_mode = cmd.get("mode", "duplex")
            if browser_mode in ("duplex", "push2talk"):
                self.upstream_mode = browser_mode
                if self.dialog:
                    self.dialog.upstream_mode = browser_mode
                logger.info(
                    f"[Browser] 对话模式: {browser_mode} ({'全双工' if browser_mode == 'duplex' else '半双工'})")

            browser_voice = cmd.get("voice", "")
            if browser_voice and browser_voice in VOICES:
                if self.dialog:
                    self.dialog.voice = browser_voice
                logger.info(f"[Browser] 声音: {browser_voice} ({VOICES[browser_voice]})")

            if not self.dialog_started:
                logger.info("[Browser] 麦克风已就绪, 启动百炼连接...")
                self._start_bailian()
                voice_info = ""
                if browser_voice and browser_voice in VOICES:
                    voice_info = f", 声音: {VOICES[browser_voice]}"
                await ws.send_str(json.dumps({
                    "type": "log",
                    "data": f"正在连接百炼服务器 (模式: {'全双工' if self.upstream_mode == 'duplex' else '半双工'}{voice_info})..."
                }))
            elif self.dialog and self.dialog.connected:
                await ws.send_str(json.dumps({
                    "type": "log", "data": f"百炼已连接, 状态: {self.dialog.state}"
                }))
            else:
                logger.info("[Browser] 百炼已断开, 正在重连...")
                self._start_bailian()
                await ws.send_str(json.dumps({
                    "type": "log", "data": "正在重连百炼服务器..."
                }))

        elif cmd_type == "stop":
            self._stop_bailian()
            await self._broadcast_json("log", "百炼连接已停止")

        elif cmd_type == "stop_speech":
            # push2talk 模式: 用户松开按钮, 调用 stop_speech()
            if self.dialog and self.dialog.connected and self._speech_started:
                self.dialog.stop_speech()
                self._speech_started = False
                logger.info("[Browser] push2talk: stop_speech() 已调用")

        elif cmd_type == "text":
            text_content = cmd.get("text", "")
            if text_content and self.dialog and self.dialog.connected:
                self.dialog.send_text(text_content)
            else:
                await ws.send_str(json.dumps({
                    "type": "log", "data": "百炼未连接, 无法发送文本"
                }))

        elif cmd_type == "interrupt":
            if self.dialog and self.dialog.connected:
                self.trigger_interruption()
                logger.info("[Control] 已触发打断：SDK 已停止，音频已清空")

        elif cmd_type == "vqa":
            logger.info("[VQA] 收到前端请求，正在抓取固定摄像头流 cam3...")
            buffer_bytes = await asyncio.get_event_loop().run_in_executor(None, capture_snapshot_from_url)

            if buffer_bytes is not None:
                img_base64 = base64.b64encode(buffer_bytes).decode('utf-8')
                if self.dialog and self.dialog.connected:
                    logger.info("[VQA] 正在将 cam3 的截图提交给百炼大模型...")
                    self.dialog.send_vqa_image(img_base64)

                else:
                    await ws.send_str(json.dumps({
                        "type": "log", "data": "百炼未连接, 无法发送 VQA"
                    }))

            else:
                logger.error("[VQA] 无法连接到 cam3 视频流或抓图失败！")
                await ws.send_str(json.dumps({
                    "type": "log", "data": "错误: 无法获取 cam3 摄像头画面，请检查流服务是否运行"
                }))

        elif cmd_type in ["get_robot_audio_devices", "get_mic_list", "get_speaker_list"]:
            if cmd_type == "get_mic_list":
                mode = "input"
            elif cmd_type == "get_speaker_list":
                mode = "output"
            else:
                mode = cmd.get("mode", "input")

            audio_devs = detect_robot_audio_devices(mode)
            logger.info(f"[Hardware] 收到前端请求({cmd_type})，成功同步机器狗音频设备({mode}): {audio_devs}")
            target_type = "mic_list" if mode == "input" else "speaker_list"
            await ws.send_str(json.dumps({
                "type": target_type,
                "mode": mode,
                "devices": audio_devs
            }, ensure_ascii=False))

        elif cmd_type == "get_camera_list":
            robot_cams = detect_robot_cameras()
            logger.info(f"[Hardware] 收到前端请求，成功同步机器狗摄像头列表: {robot_cams}")
            await ws.send_str(json.dumps({
                "type": "camera_list",
                "devices": robot_cams
            }))

        elif cmd_type == "_local_responding_started":
            if self.dialog and self.dialog.connected:
                self.dialog.local_responding_started()

        elif cmd_type == "_local_responding_ended":
            if self.dialog and self.dialog.connected:
                self.dialog.local_responding_ended()

        elif cmd_type == "status":
            await self._broadcast_json("status", {
                "state": self.dialog.state if self.dialog else "IDLE",
                "connected": self.dialog.connected if self.dialog else False,
                "dialog_started": self.dialog_started,
                "dialog_id": self.dialog.dialog_id if self.dialog else None,
                "round_count": self.dialog.round_count if self.dialog else 0,
                "audio_chunks_from_browser": self._audio_chunks_from_browser,
                "audio_chunks_sent": self._audio_chunks_sent,
                "audio_bytes_sent": self._audio_bytes_sent,
                "tts_chunks_received": self._tts_chunks_received,
                "session_dir": str(SESSION_DIR),
            })

        elif cmd_type == "voices":
            await ws.send_str(json.dumps({
                "type": "voices",
                "data": VOICES,
            }, ensure_ascii=False))

        elif cmd_type == "shutdown":
            logger.info("[Browser] 用户请求关闭服务器...")
            await self._broadcast_json("log", "服务器正在关闭...")
            asyncio.get_event_loop().call_later(0.5, lambda: sys.exit(0))

        elif cmd_type == "test_bailian":
            await self._run_bailian_test(ws)

        elif cmd_type == "test_device":
            device_type = cmd.get("device_type")  # mic, speaker, cam
            device_id = cmd.get("device_id")
            logger.info(f"[Hardware] 收到测试请求: {device_type} -> {device_id}")

            # 使用 create_task 异步运行，避免阻塞 WebSocket 通信循环
            asyncio.create_task(self._run_hardware_test(ws, device_type, device_id))

        elif cmd_type == "start_robot_audio":
            device_id = cmd.get("deviceId")
            if self.selected_mic_id == device_id and self.audio_proc is not None:
                logger.info(f"[Hardware] 设备 {device_id} 已在运行中，跳过重复启动请求。")
                return

            self.selected_mic_id = device_id
            logger.info(f"[Hardware] 收到指令，启动机器狗录音，设备: {device_id}")
            await self._stop_robot_audio_process()
            asyncio.create_task(self._run_audio_capture(device_id))
            await ws.send_str(json.dumps({"type": "log", "data": f"已启动机器狗硬件录音: {device_id}"}))

        elif cmd_type == "stop_robot_audio":
            logger.info("[Hardware] 收到指令，停止机器狗录音")
            await self._stop_robot_audio_process()
            await ws.send_str(json.dumps({"type": "log", "data": "已停止机器狗硬件录音"}))

        elif cmd_type == "set_woken":
            self.is_woken = cmd.get("woken", False)
            logger.info(f"[State] 唤醒状态更新: {'已唤醒' if self.is_woken else '待唤醒'}")

    async def _run_hardware_test(self, ws, d_type, d_id):
        """后端底层硬件测试实现"""
        try:
            result = {"type": f"{d_type}_test_result", "success": False, "msg": ""}

            if d_type == "mic":
                # 录音 2 秒进行测试
                cmd = ["parecord", "--device=" + d_id, "--format=s16le", "--rate=16000", "--channels=1", "/tmp/test.wav"]
                proc = await asyncio.create_subprocess_exec(*cmd)
                await proc.wait()
                result.update({"success": True, "msg": f"麦克风 {d_id} 录音测试成功 (已保存至 /tmp/test.wav)"})

            elif d_type == "speaker":
                # 播放一段声音
                cmd = ["paplay", "/usr/share/sounds/alsa/Front_Center.wav"]
                proc = await asyncio.create_subprocess_exec(*cmd)
                await proc.wait()
                result.update({"success": True, "msg": f"喇叭 {d_id} 播放测试完成"})

            elif d_type == "cam":
                # 直接通过 cv2 检查设备是否可打开
                cap = cv2.VideoCapture(d_id)
                if cap.isOpened():
                    result.update({"success": True, "msg": f"摄像头 {d_id} 硬件可用"})
                    cap.release()
                else:
                    result.update({"success": False, "msg": f"摄像头 {d_id} 无法打开"})

            await ws.send_str(json.dumps(result, ensure_ascii=False))

        except Exception as e:
            logger.error(f"[Hardware] 测试异常: {e}")
            await ws.send_str(json.dumps({"type": "error", "msg": str(e)}))

    async def _run_bailian_test(self, ws):
        """测试百炼 API 连接: 发送文本 → 接收 TTS 音频"""
        test_text = "请说1、2、3、4、5"
        logger.info(f"[Test] 开始百炼连接测试, 文本: {test_text}")
        await ws.send_str(json.dumps({"type": "test_status", "data": "正在连接百炼服务器..."}))

        # 创建临时百炼对话
        test_voice = VOICE
        dialog = BailianDialog(
            api_key=API_KEY, workspace_id=WORKSPACE_ID, app_id=APP_ID,
            voice=test_voice,
        )

        result = {"connected": False, "got_tts": False,
                  "llm_text": "", "tts_chunks": [], "error": ""}
        done_event = asyncio.Event()
        loop = asyncio.get_event_loop()

        def on_event(evt, data):
            if evt == "connected":
                result["connected"] = True
                dialog.connected = True
            elif evt == "started":
                logger.info(f"[Test] 会话已创建: {data}")
            elif evt == "state_changed":
                logger.info(f"[Test] 状态: {data}")
                if data == "Listening":
                    dialog.send_text(test_text)
            elif evt == "llm_content":
                if isinstance(data, dict) and data.get("finished"):
                    result["llm_text"] = data.get("text", "")
                    logger.info(f"[Test] LLM 回复: {result['llm_text'][:100]}")
            elif evt == "tts_audio":
                result["tts_chunks"].append(data)
                result["got_tts"] = True
            elif evt == "responding_ended":
                logger.info("[Test] AI 回复结束, 测试完成")
                loop.call_soon_threadsafe(done_event.set)
            elif evt == "error":
                result["error"] = str(data)
                logger.error(f"[Test] 错误: {data}")
                loop.call_soon_threadsafe(done_event.set)
            elif evt == "disconnected":
                if not result["got_tts"]:
                    result["error"] = f"连接断开: {data}"
                loop.call_soon_threadsafe(done_event.set)

        dialog.start(on_event)

        try:
            await asyncio.wait_for(done_event.wait(), timeout=20)
        except asyncio.TimeoutError:
            result["error"] = "测试超时 (20秒)"
            logger.warning("[Test] 测试超时")

        dialog.stop()

        if result["error"]:
            await ws.send_str(json.dumps({"type": "test_status", "data": f"测试失败: {result['error']}"}, ensure_ascii=False))
            return

        if not result["connected"]:
            await ws.send_str(json.dumps({"type": "test_status", "data": "测试失败: 无法连接百炼服务器"}))
            return

        # SDK 返回的 TTS 已经是 48kHz, 直接发送
        tts_chunks = result["tts_chunks"]
        if tts_chunks:
            logger.info(f"[Test] 发送 {len(tts_chunks)} 个 TTS 音频块到浏览器")
            await ws.send_str(json.dumps({"type": "test_status", "data": f"百炼连接正常! 正在播放 TTS ({len(tts_chunks)} 块)..."}))
            await ws.send_str(json.dumps({"type": "reset_audio"}))
            await asyncio.sleep(0.1)
            for i, chunk in enumerate(tts_chunks):
                try:
                    await ws.send_bytes(chunk)
                    if (i + 1) % 6 == 0:
                        await asyncio.sleep(0.05)
                except Exception:
                    break
            await ws.send_str(json.dumps({
                "type": "test_result",
                "data": {
                    "ok": True,
                    "llm_text": result["llm_text"],
                    "voice": test_voice,
                    "tts_chunks": len(tts_chunks),
                }
            }, ensure_ascii=False))
        else:
            await ws.send_str(json.dumps({
                "type": "test_result",
                "data": {"ok": True, "llm_text": result["llm_text"], "voice": test_voice, "tts_chunks": 0,
                         "warning": "未收到 TTS 音频, 但 API 连接正常"}
            }, ensure_ascii=False))

        logger.info(f"[Test] 测试完成: LLM=\"{result['llm_text'][:50]}\", TTS={len(tts_chunks)}块")


class RobotVideoTrack(MediaStreamTrack):
    kind = "video"

    def __init__(self):
        super().__init__()
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    async def recv(self):
        ret, frame = self.cap.read()
        if not ret:
            raise Exception("无法读取摄像头")
            
        frame = VideoFrame.from_ndarray(frame, format="bgr24")
        frame.pts = 0
        frame.time_base = 0
        return frame

# 2. 在 web_app.py 中增加处理逻辑
async def offer_handler(request):
    params = await request.json()
    offer = RTCSessionDescription(sdp=params["sdp"], type=params["type"])
    
    pc = RTCPeerConnection()
    pc.addTrack(RobotVideoTrack())
    
    await pc.setRemoteDescription(offer)
    answer = await pc.createAnswer()
    await pc.setLocalDescription(answer)
    
    return web.json_response({
        "sdp": pc.localDescription.sdp,
        "type": pc.localDescription.type
    })

# ═══════════════════════════════════════════════════════════
#  入口
# ═══════════════════════════════════════════════════════════
def main():
    print(r"""
╔══════════════════════════════════════════════════════════╗
║   阿里云百炼 MultiModalDialog — Web 交互界面 v3 (SDK)    ║
║   ─────────────────────────────────────────────────────  ║
║   功能: 全双工语音 | 打断 | RAG知识库 | VQA | 文本输入   ║
║   SDK:  dashscope.multimodal.MultiModalDialog            ║
╚══════════════════════════════════════════════════════════╝
    """)

    if not API_KEY:
        logger.error("请在 .env 文件中设置 DASHSCOPE_API_KEY"); sys.exit(1)
    if not WORKSPACE_ID:
        logger.error("请在 .env 文件中设置 WORKSPACE_ID"); sys.exit(1)
    if not APP_ID:
        logger.error("请在 .env 文件中设置 APP_ID"); sys.exit(1)

    logger.info(f"[Init] API Key:  {API_KEY[:12]}...{API_KEY[-8:]}")
    logger.info(f"[Init] Workspace: {WORKSPACE_ID}")
    logger.info(f"[Init] App ID:   {APP_ID}")
    logger.info(f"[Init] Voice:    {VOICE}, Downstream: {DOWNSTREAM_SAMPLE_RATE}Hz")
    logger.info(f"[Init] SDK:      dashscope.multimodal.MultiModalDialog")
    if SYSTEM_PROMPT:
        logger.info(f"[Init] Prompt:   \"{SYSTEM_PROMPT[:80]}{'...' if len(SYSTEM_PROMPT)>80 else ''}\"")
    else:
        logger.info("[Init] Prompt:   (使用 UUMO 控制台配置)")
    logger.info(f"[Init] Log:      {LOG_FILE}")

    app = WebApp()
    try:
        asyncio.run(app.start())
    except KeyboardInterrupt:
        logger.info("[Main] Ctrl+C 退出")
    finally:
        if app.dialog:
            app.dialog.stop()
        logger.info("[Main] 已退出")


if __name__ == "__main__":
    main()
