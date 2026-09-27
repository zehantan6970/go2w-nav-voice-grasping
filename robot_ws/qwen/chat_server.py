"""
AI 语音助手独立服务器 — 端口 8766
====================================
轻量 aiohttp 服务器，只服务于语音对话页面 (chat.html)。
浏览器通过 WebSocket 直接连接到主服务器 (端口 8765) 进行实时通信。

运行:
    conda activate mmdialog
    python chat_server.py
    浏览器打开 http://localhost:8766
"""

import os
import sys
import logging
from pathlib import Path

from aiohttp import web
from dotenv import load_dotenv

load_dotenv()
HOST = os.getenv("CHAT_HOST", "0.0.0.0")
PORT = int(os.getenv("CHAT_PORT", "8766"))
BASE_DIR = Path(__file__).parent.resolve()

logger = logging.getLogger("chat_server")
logger.setLevel(logging.INFO)
_fmt = logging.Formatter("%(asctime)s [%(levelname)-5s] %(message)s", "%H:%M:%S")
_sh = logging.StreamHandler(sys.stdout)
_sh.setFormatter(_fmt)
logger.addHandler(_sh)


async def handle_index(request):
    """服务 chat.html 页面"""
    html_path = BASE_DIR / "templates" / "chat.html"
    if not html_path.exists():
        logger.error(f"chat.html 不存在于: {html_path}")
        return web.Response(text="chat.html not found", status=404)
    resp = web.FileResponse(str(html_path))
    resp.headers['Cache-Control'] = 'no-store, no-cache, must-revalidate, max-age=0'
    resp.headers['Pragma'] = 'no-cache'
    resp.headers['Expires'] = '0'
    return resp


async def handle_status(request):
    """健康检查端点"""
    return web.json_response({"status": "ok", "service": "chat_server", "port": PORT})


def main():
    print(r"""
╔══════════════════════════════════════════════════════╗
║   AI 语音助手 — 独立对话页面                         ║
║   ──────────────────────────────────────             ║
║   端口 8766 — 连接到主服务器 (端口 8765) 的 WebSocket ║
║   主画面请访问 http://localhost:8765                 ║
║   语音对话请访问 http://localhost:8766               ║
╚══════════════════════════════════════════════════════╝
    """)

    app = web.Application()
    app.router.add_get("/", handle_index)
    app.router.add_get("/status", handle_status)

    runner = web.AppRunner(app)
    try:
        import asyncio
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        loop.run_until_complete(runner.setup())
        site = web.TCPSite(runner, HOST, PORT)
        loop.run_until_complete(site.start())

        logger.info(f"Chat server started: http://localhost:{PORT}")
        logger.info(f"Chat page will connect to main server WebSocket at ws://localhost:8765/ws")
        logger.info("Press Ctrl+C to stop.")

        loop.run_forever()
    except KeyboardInterrupt:
        logger.info("Chat server stopping (Ctrl+C)...")
    finally:
        loop.run_until_complete(runner.cleanup())


if __name__ == "__main__":
    main()
