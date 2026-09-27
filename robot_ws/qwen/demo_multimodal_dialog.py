"""
阿里云百炼 MultiModalDialog 实时语音交互 Demo
=============================================
参考协议: https://help.aliyun.com/zh/model-studio/multimodal-interaction-protocol

功能:
  1. 全双工语音对话 (duplex) — 麦克风实时采集，扬声器播放AI回复
  2. 语音打断 — AI说话时用户插话，自动停止播报并切换为输入
  3. RAG知识库问答 — 通过百炼控制台给 app_id 挂载知识库，对话时自动检索回答
  4. 开图说话 (VQA) — 摄像头拍照发给大模型，基于图片内容问答

关键协议流程:
  Start -> Started -> DialogStateChanged(Listening) -> 开始发送音频
  -> SpeechStarted -> SpeechContent(ASR) -> SpeechEnded -> DialogStateChanged(Thinking)
  -> DialogStateChanged(Responding) -> RespondingStarted -> 下发音频
  -> RespondingEnded -> LocalRespondingEnded -> DialogStateChanged(Listening)
  duplex模式: 用户说话自动打断; push2talk/tap2talk: RequestToSpeak打断

前提:
  - 在 https://bailian.console.aliyun.com/ 开通百炼服务
  - 创建"多模态交互应用"获取 WORKSPACE_ID 和 APP_ID
  - 如需 RAG: 在应用配置中挂载知识库
  - 复制 .env.example 为 .env 并填入真实值

用法:
  conda activate mmdialog
  cd D:\\pycode\\WorkSpace\\qwen
  python demo_multimodal_dialog.py
"""

import os
import sys
import time
import json
import queue
import base64
import logging
import threading
import numpy as np
from datetime import datetime
from dotenv import load_dotenv

import pyaudio
import sounddevice as sd

# OpenCV 可选 — 仅 VQA/开图说话功能需要
try:
    import cv2
    HAS_CV2 = True
except ImportError:
    HAS_CV2 = False

from dashscope.multimodal import (
    MultiModalDialog,
    MultiModalCallback,
    RequestParameters,
    Upstream,
    Downstream,
    ClientInfo,
    Device,
    BizParams,
    RequestToRespondParameters,
    DialogState,
)

load_dotenv()

# ─── 配置 ────────────────────────────────────────────────
API_KEY = os.getenv("DASHSCOPE_API_KEY", "")
WORKSPACE_ID = os.getenv("WORKSPACE_ID", "")
APP_ID = os.getenv("APP_ID", "")
VOICE = os.getenv("VOICE", "longanhuan")
DOWNSTREAM_SAMPLE_RATE = int(os.getenv("DOWNSTREAM_SAMPLE_RATE", "24000"))

# WebSocket 地址 (国内)
WS_URL = "wss://dashscope.aliyuncs.com/api-ws/v1/inference"

# ─── 音频参数 ────────────────────────────────────────────
MIC_SAMPLE_RATE = 16000       # 麦克风采样率 (上行固定16kHz, 16bit, mono, little-endian PCM)
MIC_CHANNELS = 1              # 单声道
MIC_CHUNK_MS = 100            # 每次读取 100ms (协议推荐)
MIC_CHUNK_SIZE = MIC_SAMPLE_RATE * (16 // 8) * MIC_CHUNK_MS // 1000  # = 3200 bytes
PLAYBACK_SAMPLE_RATE = DOWNSTREAM_SAMPLE_RATE  # 下行采样率

# ─── 心跳间隔 (协议要求60s无消息断连, 建议50s发一次) ────
HEARTBEAT_INTERVAL = 45  # 秒

# ─── 日志配置 ────────────────────────────────────────────
LOG_FILE = os.path.join(os.path.dirname(__file__), "mmdialog.log")

logging.basicConfig(
    level=logging.DEBUG,
    format="%(asctime)s.%(msecs)03d [%(levelname)-5s] %(message)s",
    datefmt="%H:%M:%S",
    handlers=[
        logging.StreamHandler(sys.stdout),
        logging.FileHandler(LOG_FILE, encoding="utf-8"),
    ],
)
logger = logging.getLogger("mmdialog")


def _ts() -> str:
    """当前时间戳字符串, 用于日志"""
    return datetime.now().strftime("%H:%M:%S.%f")[:-3]


# ═══════════════════════════════════════════════════════════
#  音频播放器 — 线程安全的 PCM 音频流播放
# ═══════════════════════════════════════════════════════════
class AudioPlayer:
    """
    将服务端下发的 PCM 音频数据放入队列，由独立线程实时播放到扬声器。
    支持打断: 调用 interrupt() 可立即清空队列并停止播放。
    播放完成通知: 通过 on_playback_done 回调通知上层。
    """

    def __init__(self, sample_rate: int = PLAYBACK_SAMPLE_RATE):
        self.sample_rate = sample_rate
        self._queue: queue.Queue = queue.Queue()
        self._playing = False
        self._is_playing_audio = False  # 当前是否正在播放音频
        self._thread: threading.Thread | None = None
        self._stop_event = threading.Event()
        self._total_chunks = 0
        self._total_bytes = 0
        self._on_playback_done = None  # 播放完成回调

    def start(self, on_playback_done=None):
        """启动播放线程"""
        self._on_playback_done = on_playback_done
        self._stop_event.clear()
        self._playing = True
        self._thread = threading.Thread(target=self._play_loop, daemon=True, name="AudioPlayer")
        self._thread.start()
        logger.debug("[AudioPlayer] 播放线程已启动")

    def interrupt(self) -> int:
        """立即停止播放并清空队列 (用于打断), 返回清空的chunk数"""
        dropped = 0
        while not self._queue.empty():
            try:
                self._queue.get_nowait()
                dropped += 1
            except queue.Empty:
                break
        self._is_playing_audio = False
        sd.stop()
        if dropped > 0:
            logger.info(f"[AudioPlayer] 打断: 丢弃 {dropped} 个待播放chunk")
        return dropped

    def shutdown(self):
        """关闭播放器"""
        self._stop_event.set()
        self._playing = False
        sd.stop()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=3)
        logger.debug(f"[AudioPlayer] 已关闭, 累计播放 {self._total_chunks} chunks / {self._total_bytes} bytes")

    def add_audio(self, pcm_data: bytes):
        """添加 PCM 音频数据到播放队列"""
        if self._playing:
            self._queue.put(pcm_data)

    def queue_size(self) -> int:
        return self._queue.qsize()

    @property
    def is_playing(self) -> bool:
        return self._is_playing_audio

    def _play_loop(self):
        """播放循环: 从队列取数据并播放"""
        logger.debug("[AudioPlayer] 播放循环开始")
        while not self._stop_event.is_set():
            try:
                pcm_data = self._queue.get(timeout=0.1)
            except queue.Empty:
                # 如果之前正在播放但队列空了，说明播放完成
                if self._is_playing_audio:
                    self._is_playing_audio = False
                    logger.debug(f"[AudioPlayer] 播放队列耗尽, 累计 {self._total_chunks} chunks")
                    if self._on_playback_done:
                        self._on_playback_done()
                continue

            if not self._playing:
                continue

            try:
                audio_array = np.frombuffer(pcm_data, dtype=np.int16)
                if len(audio_array) > 0:
                    self._is_playing_audio = True
                    self._total_chunks += 1
                    self._total_bytes += len(pcm_data)
                    sd.play(audio_array, samplerate=self.sample_rate)
                    sd.wait()
            except Exception as e:
                logger.warning(f"[AudioPlayer] 播放异常: {e}")
                self._is_playing_audio = False


# ═══════════════════════════════════════════════════════════
#  SDK 回调处理
# ═══════════════════════════════════════════════════════════
class DialogCallback(MultiModalCallback):
    """
    处理 MultiModalDialog SDK 的各种回调事件。

    协议状态流转 (duplex 模式):
      Started -> DialogStateChanged(Listening) -> [可以发送音频]
      -> SpeechStarted -> SpeechContent(ASR流式) -> SpeechEnded
      -> DialogStateChanged(Thinking) -> DialogStateChanged(Responding)
      -> RespondingStarted -> [下发音频+RespondingContent]
      -> RespondingEnded -> [客户端上报LocalRespondingEnded]
      -> DialogStateChanged(Listening) -> [循环]

    duplex 打断:
      Responding 状态下用户说话 -> SpeechStarted -> RequestAccepted -> 停止下发
    """

    def __init__(self, audio_player: AudioPlayer, dialog_ref: list):
        super().__init__()
        self.audio_player = audio_player
        self.dialog_ref = dialog_ref
        self.dialog_id: str | None = None
        self._current_state: str = "UNKNOWN"
        self._ready_for_audio = False  # 收到 Listening 后才允许发送音频
        self._round_count = 0  # 交互轮次计数

        # 统计
        self._asr_chunks = 0
        self._tts_audio_chunks = 0

    # ─── 连接与生命周期 ─────────────────────────────────
    def on_connected(self) -> None:
        logger.info("[WS] ====== WebSocket 连接已建立 ======")

    def on_close(self, close_status_code, close_msg):
        logger.info(f"[WS] ====== 连接关闭: code={close_status_code}, msg={close_msg} ======")

    # ─── 对话启动 (Started 事件) ────────────────────────
    def on_started(self, dialog_id: str) -> None:
        self.dialog_id = dialog_id
        logger.info(f"[Started] 会话创建成功, dialog_id={dialog_id}")
        logger.info("[Started] 等待 DialogStateChanged(Listening) 后才开始发送音频...")
        # 注意: 协议明确要求 Started 后不能立即发送音频, 必须等 Listening 状态

    def on_stopped(self) -> None:
        logger.info("[Stopped] 会话已停止")

    # ─── 状态切换 (DialogStateChanged 事件) ─────────────
    def on_state_changed(self, state) -> None:
        state_str = str(state).upper() if state else "UNKNOWN"
        old_state = self._current_state
        self._current_state = state_str
        logger.info(f"[State] {old_state} -> {state_str}")

        if state_str == "LISTENING":
            # 关键: 只有收到 Listening 后才能发送音频 (协议要求)
            self._ready_for_audio = True
            self._round_count += 1
            logger.info(f"[State] 已就绪, 可以发送音频 (第 {self._round_count} 轮)")
            logger.info("-" * 50)

            # 通知 SDK 开始发送音频 (duplex 模式需持续发送)
            dialog = self.dialog_ref[0]
            if dialog:
                try:
                    dialog.start_speech()
                    logger.debug("[State] start_speech() 已调用")
                except Exception as e:
                    logger.debug(f"[State] start_speech() 异常(可能已在speech状态): {e}")

        elif state_str == "THINKING":
            logger.info("[State] AI 正在思考中...")

        elif state_str == "RESPONDING":
            logger.info("[State] AI 正在回复中...")

    # ─── 用户语音识别 (SpeechContent 事件) ──────────────
    def on_speech_content(self, payload) -> None:
        """
        用户说的话被 ASR 识别为文字。
        协议: {"text": "...", "finished": bool}
        流式返回, finished=True 时是最终结果。
        """
        if not payload:
            return
        self._asr_chunks += 1

        text = ""
        finished = False
        if isinstance(payload, dict):
            text = payload.get("text", "")
            finished = payload.get("finished", False)
        elif isinstance(payload, str):
            text = payload

        if finished:
            logger.info(f"[ASR 最终] >>> 你说: \"{text}\"")
        else:
            logger.debug(f"[ASR 流式] {text}  (chunk#{self._asr_chunks})")

    # ─── AI 回复文本 (RespondingContent 事件) ───────────
    def on_responding_content(self, payload) -> None:
        """
        大模型回复的文字内容。
        协议: {"text": "...", "spoken": "...", "finished": bool,
               "extra_info": {"commands": "...", "tool_calls": [...]}}
        """
        if not payload:
            return

        text = ""
        finished = False
        extra_info = {}

        if isinstance(payload, dict):
            text = payload.get("text", "")
            finished = payload.get("finished", False)
            extra_info = payload.get("extra_info", {})
        elif isinstance(payload, str):
            text = payload
            finished = True

        # 检查是否有 VQA (拍照识别) 意图
        commands_str = extra_info.get("commands", "")
        if commands_str and "visual_qa" in str(commands_str):
            logger.info(f"[VQA] 检测到 visual_qa 意图, commands={commands_str}")
            threading.Thread(target=self._handle_vqa_request, daemon=True).start()
            return

        # 检查 tool_calls (插件调用)
        tool_calls = extra_info.get("tool_calls", [])
        if tool_calls:
            for tc in tool_calls:
                func = tc.get("function", {})
                logger.info(f"[Tool] 调用插件: {func.get('name', 'unknown')}, "
                            f"args={func.get('arguments', '')}")

        if finished:
            logger.info(f"[LLM 最终] <<< AI说: \"{text}\"")
        else:
            if text:
                logger.debug(f"[LLM 流式] {text[:80]}{'...' if len(text)>80 else ''}")

    # ─── AI 回复音频 (RespondingStarted/Ended 事件) ─────
    def on_responding_started(self):
        """
        服务端开始下发 TTS 音频。
        协议: 客户端需上报 LocalRespondingStarted。
        """
        self._tts_audio_chunks = 0
        logger.info("[TTS Start] AI 开始说话 (RespondingStarted)")

        # 通知 SDK 客户端开始播放
        dialog = self.dialog_ref[0]
        if dialog:
            try:
                dialog.local_responding_started()
                logger.debug("[TTS Start] 已上报 LocalRespondingStarted")
            except Exception as e:
                logger.warning(f"[TTS Start] 上报 LocalRespondingStarted 失败: {e}")

    def on_responding_ended(self):
        """
        服务端 TTS 音频下发完毕。
        协议: 客户端播放完成后需上报 LocalRespondingEnded。
        """
        logger.info(f"[TTS End] AI 音频下发完毕 (RespondingEnded), "
                    f"本次下发 {self._tts_audio_chunks} chunks")

        # 等待播放队列清空后再上报 LocalRespondingEnded
        # 注意: 不能立即上报, 因为播放队列中可能还有未播放完的音频
        dialog = self.dialog_ref[0]
        if dialog:
            # 等待播放器队列清空 (最多等5秒)
            wait_start = time.time()
            while self.audio_player.queue_size() > 0 and (time.time() - wait_start) < 5:
                time.sleep(0.1)
            # 再等一下确保最后一块播放完
            time.sleep(0.3)

            try:
                dialog.local_responding_ended()
                logger.info("[TTS End] 已上报 LocalRespondingEnded, 等待服务端切换到 Listening...")
            except Exception as e:
                logger.warning(f"[TTS End] 上报 LocalRespondingEnded 失败: {e}")

    def on_speech_audio_data(self, data: bytes) -> None:
        """接收 AI 回复的 TTS 音频数据 -> 放入播放队列"""
        if data:
            self._tts_audio_chunks += 1
            self.audio_player.add_audio(data)

    # ─── 打断 (RequestAccepted 事件) ────────────────────
    def on_request_accepted(self):
        """
        打断请求被接受。
        duplex 模式下, 服务端检测到用户说话会自动打断。
        也可通过 interrupt() 手动发送 RequestToSpeak 触发。
        """
        logger.info("[Interrupt] ====== 打断已生效 (RequestAccepted) ======")
        dropped = self.audio_player.interrupt()
        logger.info(f"[Interrupt] 播放已停止, 丢弃 {dropped} 个待播放chunk")

    # ─── 错误 ───────────────────────────────────────────
    def on_error(self, error) -> None:
        logger.error(f"[Error] ====== 服务端错误: {error} ======")

    # ─── VQA 拍照处理 ───────────────────────────────────
    def _handle_vqa_request(self):
        """捕获摄像头画面并发送给大模型"""
        logger.info("[VQA] === 开始拍照识别流程 ===")

        if not HAS_CV2:
            logger.warning("[VQA] OpenCV 未安装, 无法拍照。请运行: pip install opencv-python")
            return

        try:
            t0 = time.time()
            cap = cv2.VideoCapture(0)
            if not cap.isOpened():
                logger.error("[VQA] 无法打开摄像头 (cv2.VideoCapture)")
                return

            ret, frame = cap.read()
            cap.release()
            logger.debug(f"[VQA] 摄像头读取耗时: {(time.time()-t0)*1000:.0f}ms")

            if not ret or frame is None:
                logger.error("[VQA] 拍照失败: cv2.VideoCapture.read() 返回 False")
                return

            # 压缩图片到 <180KB (协议要求)
            quality = 70
            _, buffer = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), quality])
            while len(buffer) > 180 * 1024 and quality > 20:
                quality -= 10
                _, buffer = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), quality])

            img_base64 = base64.b64encode(buffer).decode('utf-8')
            logger.info(f"[VQA] 拍照成功: {frame.shape[1]}x{frame.shape[0]}, "
                        f"quality={quality}, size={len(buffer)/1024:.1f}KB, "
                        f"base64_len={len(img_base64)}, "
                        f"耗时={(time.time()-t0)*1000:.0f}ms")

            dialog = self.dialog_ref[0]
            if dialog:
                images = [{"type": "base64", "value": img_base64}]
                params = RequestToRespondParameters(images=images)

                # 协议要求: RequestToRespond 需在 Listening 状态下发送
                state = self._current_state
                if state != "LISTENING":
                    logger.info(f"[VQA] 当前状态={state}, 需要先打断再发送...")
                    dialog.interrupt()
                    # 等待切换到 LISTENING
                    for _ in range(50):
                        if self._current_state == "LISTENING":
                            break
                        time.sleep(0.1)
                    else:
                        logger.warning("[VQA] 等待 LISTENING 超时, 仍尝试发送")

                dialog.request_to_respond("prompt", "请描述你看到的画面", parameters=params)
                logger.info(f"[VQA] 图片已发送, 等待AI视觉分析... (总耗时 {(time.time()-t0)*1000:.0f}ms)")

        except Exception as e:
            logger.error(f"[VQA] 拍照失败: {e}", exc_info=True)


# ═══════════════════════════════════════════════════════════
#  主程序
# ═══════════════════════════════════════════════════════════
class MultimodalDialogDemo:
    """
    阿里云百炼 MultiModalDialog 实时语音交互 Demo

    功能:
      - 全双工语音对话 (duplex 模式, 服务端VAD自动检测)
      - 自动语音打断 (duplex下服务端检测用户说话自动打断)
      - RAG 知识库问答 (通过 app_id 在控制台配置, SDK自动处理)
      - 开图说话 / VQA (摄像头拍照 + 视觉问答)
    """

    def __init__(self):
        self._check_config()

        # 音频播放器
        self.audio_player = AudioPlayer(sample_rate=PLAYBACK_SAMPLE_RATE)

        # SDK 引用 (用列表包装以便回调中访问)
        self.dialog_ref: list = [None]
        self.callback = DialogCallback(self.audio_player, self.dialog_ref)

        # 麦克风状态
        self._mic_running = False
        self._mic_thread: threading.Thread | None = None
        self._mic_chunks_sent = 0
        self._mic_bytes_sent = 0

        # 心跳
        self._heartbeat_thread: threading.Thread | None = None
        self._heartbeat_running = False

        # 构建请求参数 (参照协议文档 Start 消息结构)
        upstream = Upstream(
            type="AudioOnly",    # 纯语音模式 (如需视频通话改为 "AudioAndVideo")
            mode="duplex",       # 全双工 — 服务端VAD, 支持语音打断
            audio_format="pcm",  # PCM: 16kHz/16bit/mono/little-endian
        )
        downstream = Downstream(
            voice=VOICE,
            sample_rate=DOWNSTREAM_SAMPLE_RATE,
            audio_format="pcm",
            intermediate_text="transcript,dialog",  # 返回ASR识别结果+LLM回复中间文本
        )
        client_info = ClientInfo(
            user_id="demo_user_001",
            device=Device(uuid=f"pc-demo-{int(time.time())}"),
        )
        self.request_params = RequestParameters(
            upstream=upstream,
            downstream=downstream,
            client_info=client_info,
        )

        logger.info(f"[Init] 配置参数:")
        logger.info(f"  upstream:  AudioOnly / duplex / pcm")
        logger.info(f"  downstream: voice={VOICE}, sample_rate={DOWNSTREAM_SAMPLE_RATE}, pcm")
        logger.info(f"  mic_chunk: {MIC_CHUNK_SIZE} bytes ({MIC_CHUNK_MS}ms @ {MIC_SAMPLE_RATE}Hz/16bit)")
        logger.info(f"  heartbeat: {HEARTBEAT_INTERVAL}s")

    def _check_config(self):
        """检查必要配置"""
        if not API_KEY:
            logger.error("请在 .env 文件中设置 DASHSCOPE_API_KEY")
            sys.exit(1)
        if not WORKSPACE_ID:
            logger.error("请在 .env 文件中设置 WORKSPACE_ID")
            sys.exit(1)
        if not APP_ID:
            logger.error("请在 .env 文件中设置 APP_ID")
            sys.exit(1)
        logger.info(f"[Init] 配置加载成功: workspace={WORKSPACE_ID[:8]}..., "
                    f"app={APP_ID[:8]}..., api_key={API_KEY[:8]}...")

    def start(self):
        """启动对话 — 建立 WebSocket 连接并发送 Start 消息"""
        logger.info("[Start] === 正在创建 MultiModalDialog 并连接百炼服务器 ===")
        logger.info(f"[Start] WS URL: {WS_URL}")
        t0 = time.time()

        # 创建 MultiModalDialog 实例 (内部会建立 WebSocket 连接)
        dialog = MultiModalDialog(
            workspace_id=WORKSPACE_ID,
            app_id=APP_ID,
            request_params=self.request_params,
            multimodal_callback=self.callback,
            url=WS_URL,
            api_key=API_KEY,
            model="multimodal-dialog",
        )
        self.dialog_ref[0] = dialog

        # 启动音频播放器
        self.audio_player.start()

        # 发送 Start 消息
        dialog.start(dialog_id=None)
        logger.info(f"[Start] Start 消息已发送, 等待 Started + Listening 事件... "
                    f"(耗时 {(time.time()-t0)*1000:.0f}ms)")

        # 等待 on_started 回调 (最多等 15 秒)
        for i in range(150):
            if self.callback.dialog_id:
                break
            time.sleep(0.1)
        else:
            logger.error("[Start] 等待 Started 事件超时(15s), 请检查: "
                        "1) API_KEY/WORKSPACE_ID/APP_ID 是否正确  2) 网络是否正常")
            return False

        # 等待 Listening 状态 (最多再等 10 秒)
        for i in range(100):
            if self.callback._ready_for_audio:
                break
            time.sleep(0.1)
        else:
            logger.warning("[Start] 等待 Listening 状态超时(10s), 仍尝试启动麦克风")

        logger.info(f"[Start] === 会话建立成功, 总耗时 {(time.time()-t0)*1000:.0f}ms ===")

        # 启动麦克风采集 (必须在 Listening 之后)
        self._start_microphone()

        # 启动心跳线程
        self._start_heartbeat()

        return True

    def _start_microphone(self):
        """启动麦克风采集线程"""
        self._mic_running = True
        self._mic_chunks_sent = 0
        self._mic_bytes_sent = 0
        self._mic_thread = threading.Thread(target=self._mic_loop, daemon=True, name="Microphone")
        self._mic_thread.start()
        logger.info(f"[Mic] 麦克风采集已启动: {MIC_SAMPLE_RATE}Hz/16bit/mono, "
                    f"chunk={MIC_CHUNK_SIZE}bytes/{MIC_CHUNK_MS}ms")

    def _mic_loop(self):
        """
        麦克风采集循环 — 持续录音并发送给 SDK。

        协议要求 (duplex 模式):
          - 持续上传音频, 服务端自动检测语音活动
          - 建议每100ms上传一次数据
          - 超过20秒不上传音频会报错
          - 即使在 AI 播报(Responding)时也不能停止发送
        """
        p = pyaudio.PyAudio()
        try:
            stream = p.open(
                format=pyaudio.paInt16,
                channels=MIC_CHANNELS,
                rate=MIC_SAMPLE_RATE,
                input=True,
                frames_per_buffer=MIC_CHUNK_SIZE,
            )
            logger.info("[Mic] PyAudio 流已打开, 开始录音...")
            log_interval = 50  # 每50个chunk(5秒)打一次统计日志
            last_log_time = time.time()

            while self._mic_running:
                # 只有在 Listening 状态后才发送音频 (协议要求)
                if not self.callback._ready_for_audio:
                    time.sleep(0.05)
                    continue

                try:
                    data = stream.read(MIC_CHUNK_SIZE, exception_on_overflow=False)
                    dialog = self.dialog_ref[0]
                    if dialog and data:
                        try:
                            dialog.send_audio_data(data)
                            self._mic_chunks_sent += 1
                            self._mic_bytes_sent += len(data)
                        except Exception as e:
                            logger.warning(f"[Mic] send_audio_data 失败: {e}")

                    # 定期打印统计
                    now = time.time()
                    if now - last_log_time >= 5.0:
                        state = self.callback._current_state
                        qsize = self.audio_player.queue_size()
                        logger.debug(
                            f"[Mic] 5s统计: chunks={self._mic_chunks_sent}, "
                            f"bytes={self._mic_bytes_sent}, "
                            f"state={state}, "
                            f"playback_queue={qsize}"
                        )
                        last_log_time = now

                except IOError as e:
                    logger.warning(f"[Mic] 麦克风读取异常: {e}")
                    time.sleep(0.1)

            stream.stop_stream()
            stream.close()
            logger.info(f"[Mic] 录音停止, 累计发送 {self._mic_chunks_sent} chunks / "
                        f"{self._mic_bytes_sent} bytes / "
                        f"{self._mic_chunks_sent * MIC_CHUNK_MS / 1000:.1f}s")
        except Exception as e:
            logger.error(f"[Mic] 麦克风初始化失败: {e}", exc_info=True)
        finally:
            p.terminate()

    def _start_heartbeat(self):
        """启动心跳线程 — 防止60秒超时断连"""
        self._heartbeat_running = True
        self._heartbeat_thread = threading.Thread(target=self._heartbeat_loop, daemon=True, name="Heartbeat")
        self._heartbeat_thread.start()
        logger.debug("[Heartbeat] 心跳线程已启动")

    def _heartbeat_loop(self):
        """心跳循环 — 定期发送 HeartBeat 消息"""
        while self._heartbeat_running:
            time.sleep(HEARTBEAT_INTERVAL)
            if not self._heartbeat_running:
                break
            dialog = self.dialog_ref[0]
            if dialog and self.callback.dialog_id:
                try:
                    # SDK 没有直接的心跳方法, 但持续发送音频本身就能保活
                    # 这里打日志提醒
                    logger.debug("[Heartbeat] 连接保活中...")
                except Exception as e:
                    logger.warning(f"[Heartbeat] 异常: {e}")

    def _stop_microphone(self):
        """停止麦克风"""
        self._mic_running = False
        if self._mic_thread and self._mic_thread.is_alive():
            self._mic_thread.join(timeout=3)
            logger.info("[Mic] 麦克风已停止")

    def _stop_heartbeat(self):
        """停止心跳"""
        self._heartbeat_running = False

    def send_vqa(self):
        """手动触发拍照识别 (VQA)"""
        if not HAS_CV2:
            logger.warning("[VQA] 需要安装 opencv-python: pip install opencv-python")
            return
        threading.Thread(target=self.callback._handle_vqa_request, daemon=True).start()

    def manual_interrupt(self):
        """手动打断 AI 说话 (发送 RequestToSpeak)"""
        dialog = self.dialog_ref[0]
        if dialog:
            logger.info("[Interrupt] 手动发送打断请求 (interrupt/RequestToSpeak)...")
            dialog.interrupt()
        else:
            logger.warning("[Interrupt] dialog 未初始化")

    def send_text(self, text: str):
        """发送文本消息给 AI (RequestToRespond type=prompt)"""
        dialog = self.dialog_ref[0]
        if not dialog:
            logger.warning("[Text] dialog 未初始化")
            return

        state = self.callback._current_state
        if state != "LISTENING":
            logger.info(f"[Text] 当前状态={state}, 先打断...")
            dialog.interrupt()
            for _ in range(50):
                if self.callback._current_state == "LISTENING":
                    break
                time.sleep(0.1)

        logger.info(f"[Text] 发送文本: \"{text}\"")
        dialog.request_to_respond("prompt", text, parameters=None)

    def run(self):
        """主循环 — 等待用户命令"""
        try:
            if not self.start():
                return

            logger.info("=" * 60)
            logger.info("  就绪! 直接对着麦克风说话即可 (全双工 duplex 模式)")
            logger.info("  键盘命令: v=拍照识别 | i=打断 | t=文本输入 | q=退出")
            logger.info("  日志文件: " + LOG_FILE)
            logger.info("=" * 60)

            while True:
                try:
                    cmd = input("\n>>> ").strip().lower()
                except EOFError:
                    break

                if not cmd:
                    continue
                elif cmd in ("q", "quit", "exit"):
                    logger.info("[Main] 用户请求退出")
                    break
                elif cmd in ("v", "video", "vqa"):
                    self.send_vqa()
                elif cmd in ("i", "interrupt"):
                    self.manual_interrupt()
                elif cmd.startswith("t ") or cmd == "t":
                    text = cmd[2:] if cmd.startswith("t ") else ""
                    if not text:
                        text = input("请输入文本: ").strip()
                    if text:
                        self.send_text(text)
                elif cmd in ("s", "status"):
                    self._print_status()
                elif cmd in ("h", "help"):
                    self._print_help()
                else:
                    print(f"未知命令: {cmd}  (输入 h 查看帮助)")

        except KeyboardInterrupt:
            logger.info("\n[Main] Ctrl+C 退出")
        finally:
            self.shutdown()

    def _print_status(self):
        """打印当前状态"""
        logger.info("=" * 40)
        logger.info(f"  状态:     {self.callback._current_state}")
        logger.info(f"  dialog_id: {self.callback.dialog_id}")
        logger.info(f"  交互轮次: {self.callback._round_count}")
        logger.info(f"  麦克风:   {self._mic_chunks_sent} chunks / {self._mic_bytes_sent} bytes")
        logger.info(f"  播放队列: {self.audio_player.queue_size()} chunks")
        logger.info(f"  正在播放: {self.audio_player.is_playing}")
        logger.info("=" * 40)

    def _print_help(self):
        """打印帮助"""
        print("""
╔══════════════════════════════════════════════════╗
║  命令列表                                        ║
╠══════════════════════════════════════════════════╣
║  (直接说话)  对着麦克风说话, duplex自动交互       ║
║  v           拍照识别 — 摄像头拍照让AI描述画面    ║
║  i           打断 — 手动打断AI说话               ║
║  t <文本>    文本输入 — 打字发给AI (调试用)       ║
║  s           状态 — 查看当前对话状态和统计信息    ║
║  h           帮助 — 显示此命令列表                ║
║  q           退出程序                             ║
╚══════════════════════════════════════════════════╝
""")

    def shutdown(self):
        """清理所有资源"""
        logger.info("[Shutdown] 正在清理资源...")
        self._stop_heartbeat()
        self._stop_microphone()
        self.audio_player.shutdown()

        dialog = self.dialog_ref[0]
        if dialog:
            try:
                logger.info("[Shutdown] 发送 Stop 消息...")
                dialog.stop()
            except Exception as e:
                logger.warning(f"[Shutdown] Stop 异常: {e}")

        logger.info(f"[Shutdown] === 已退出, 共完成 {self.callback._round_count} 轮交互 ===")


# ═══════════════════════════════════════════════════════════
#  入口
# ═══════════════════════════════════════════════════════════
def main():
    print(r"""
╔══════════════════════════════════════════════════════════╗
║   阿里云百炼 MultiModalDialog 实时语音交互 Demo          ║
║   ─────────────────────────────────────────────────────  ║
║   功能: 全双工语音对话 | 语音打断 | RAG知识库 | 开图说话  ║
║   协议: WebSocket duplex (wss://dashscope.aliyuncs.com)  ║
╚══════════════════════════════════════════════════════════╝
    """)

    demo = MultimodalDialogDemo()
    demo.run()


if __name__ == "__main__":
    main()
