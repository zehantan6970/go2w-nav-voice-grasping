#!/usr/bin/env python3
"""
小創 Python 導航 v1 (Python SLAM v1)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

基於 unitree_sdk2py RPC 客戶端，完整對應 keyDemo.cpp 功能。

按鍵：
  q: 開始建圖          w: 結束建圖 & 保存
  a: 載入地圖重定位     s: 記錄當前位址到任務列表
  c: 單次巡航           d: 循環巡航
  l: 查看任務列表       f: 清空任務列表
  z: 暫停巡航           x: 恢復巡航
  r: 移除最後一個點
  其他鍵: 停止 SLAM

使用：
  python3 python_slamv1.py eth0
"""

import json
import os
import sys
import threading
import time
import math
from pathlib import Path

os.environ.setdefault("LD_LIBRARY_PATH", "/home/unitree/cyclonedds/install/lib")
os.environ.setdefault("CYCLONEDDS_HOME", "/home/unitree/cyclonedds/install")

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.rpc.client import Client
from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_
from unitree_sdk2py.idl.default import std_msgs_msg_dds__String_

# ─── TTS ───
import base64, io, wave, websocket
from dotenv import load_dotenv

_ENV = Path("/home/unitree/lab/all/vision+arm/asr/.env")
if _ENV.exists(): load_dotenv(_ENV)
TTS_API_KEY = os.getenv("ALIYUN_TTS_API_KEY", "") or os.getenv("ALIYUN_REALTIME_API_KEY", "")
TTS_VOICE = os.getenv("ALIYUN_TTS_VOICE", "Cherry")

# ─── RPC API ID ───
ROBOT_API_ID_STOP_NODE = 1901
ROBOT_API_ID_START_MAPPING_PL = 1801
ROBOT_API_ID_END_MAPPING_PL = 1802
ROBOT_API_ID_START_RELOCATION_PL = 1804
ROBOT_API_ID_POSE_NAV_PL = 1102
ROBOT_API_ID_PAUSE_NAV = 1201
ROBOT_API_ID_RESUME_NAV = 1202

SLAM_SERVICE = "slam_operate"
SLAM_INFO_TOPIC = "rt/slam_info"
SLAM_KEY_INFO_TOPIC = "rt/slam_key_info"

# ─── ANSI ───
G = "\033[32m"; Y = "\033[33m"; C = "\033[36m"; R = "\033[31m"; B = "\033[1m"; N = "\033[0m"

# ─── Globals ───
slam_client = None
current_pose = {"x":0,"y":0,"z":0,"q_x":0,"q_y":0,"q_z":0,"q_w":1}
is_arrived = False
nav_running = False
_pose_lock = threading.Lock()
_nav_thread = None


# ═══════════════════════════════════════════════
#  SLAM Client
# ═══════════════════════════════════════════════
class SlamClient(Client):
    def __init__(self):
        super().__init__(SLAM_SERVICE, False)
        self.pose_list = []
        self.map_name = "?"

    def Init(self):
        self._SetApiVerson("1.0.0.1")
        for a in [ROBOT_API_ID_POSE_NAV_PL, ROBOT_API_ID_PAUSE_NAV,
                  ROBOT_API_ID_RESUME_NAV, ROBOT_API_ID_STOP_NODE,
                  ROBOT_API_ID_START_MAPPING_PL, ROBOT_API_ID_END_MAPPING_PL,
                  ROBOT_API_ID_START_RELOCATION_PL]:
            self._RegistApi(a, 0)
        self.SetTimeout(10.0)

    def _rpc(self, api_id, params=None):
        p = json.dumps(params or {"data": {}})
        return self._Call(api_id, p)

    def stop_node(self):          return self._rpc(ROBOT_API_ID_STOP_NODE)
    def start_mapping(self):      return self._rpc(ROBOT_API_ID_START_MAPPING_PL, {"data":{"slam_type":"indoor"}})
    def end_mapping(self, addr):  return self._rpc(ROBOT_API_ID_END_MAPPING_PL, {"data":{"address":addr}})
    def relocate(self,x=0,y=0,z=0,qx=0,qy=0,qz=0,qw=1,addr="/home/unitree/test.pcd"):
        return self._rpc(ROBOT_API_ID_START_RELOCATION_PL,
                         {"data":{"x":x,"y":y,"z":z,"q_x":qx,"q_y":qy,"q_z":qz,"q_w":qw,"address":addr}})
    def pause_nav(self):          return self._rpc(ROBOT_API_ID_PAUSE_NAV)
    def resume_nav(self):         return self._rpc(ROBOT_API_ID_RESUME_NAV)

    def send_target(self, pose, mode=1, speed=0.6):
        return self._rpc(ROBOT_API_ID_POSE_NAV_PL, {
            "data": {
                "targetPose": {"x":pose["x"],"y":pose["y"],"z":pose["z"],
                               "q_x":pose["q_x"],"q_y":pose["q_y"],"q_z":pose["q_z"],"q_w":pose["q_w"]},
                "mode":mode,"speed":speed,
            }
        })


# ═══════════════════════════════════════════════
#  DDS Handlers
# ═══════════════════════════════════════════════
def on_slam_info(msg):
    global current_pose
    try:
        d = json.loads(msg.data)
        if d.get("errorCode") != 0 or d.get("type") != "pos_info": return
        p = d["data"]["currentPose"]
        with _pose_lock:
            for k in ("x","y","z","q_x","q_y","q_z","q_w"):
                current_pose[k] = float(p.get(k, current_pose.get(k, 0)))
        try:
            with open("/tmp/robot_pose.json","w") as f: json.dump(current_pose, f)
        except: pass
    except: pass

def on_slam_key(msg):
    global is_arrived
    try:
        d = json.loads(msg.data)
        if d.get("errorCode") != 0: return
        if d.get("type") == "task_result":
            arrived = d["data"].get("is_arrived", False)
            target = d["data"].get("targetNodeName", "?")
            is_arrived = arrived
            if arrived:
                print(f"\n{G}✅ 已到達 {target}{N}")
                play_tts(f"已到達{target}")
            else:
                print(f"\n{Y}❌ 未到達 {target}{N}")
    except: pass


# ═══════════════════════════════════════════════
#  TTS
# ═══════════════════════════════════════════════
def tts_synth(text):
    if not TTS_API_KEY: return b""
    try:
        ws = websocket.create_connection(
            "wss://dashscope-intl.aliyuncs.com/api-ws/v1/realtime?model=qwen3-tts-flash-realtime",
            header=[f"Authorization: Bearer {TTS_API_KEY}"], timeout=10)
        chunks = []
        ws.send(json.dumps({"event_id":"t","type":"session.update",
                            "session":{"voice":TTS_VOICE,"response_format":"pcm","sample_rate":24000,"language_type":"Chinese"}}))
        ws.send(json.dumps({"event_id":"t","type":"input_text_buffer.append","text":text}))
        ws.send(json.dumps({"event_id":"t","type":"input_text_buffer.commit"}))
        while True:
            m = json.loads(ws.recv())
            t = m.get("type","")
            if t in ("response.audio.delta","output_audio.delta"):
                d = m.get("delta") or m.get("audio")
                if d: chunks.append(base64.b64decode(d))
            elif t in ("response.completed","response.done","session.finished"): break
        ws.close()
        return b"".join(chunks)
    except:
        return b""

def tts_play(pcm):
    if not pcm: return
    tmp = f"/tmp/tts_py_{os.urandom(4).hex()}.wav"
    try:
        with wave.open(tmp,"wb") as w: w.setnchannels(1); w.setsampwidth(2); w.setframerate(24000); w.writeframes(pcm)
        os.system(f"paplay {tmp} 2>/dev/null || aplay -D default -q {tmp} 2>/dev/null")
    finally:
        try: os.remove(tmp)
        except: pass

def play_tts(text):
    if TTS_API_KEY:
        threading.Thread(target=lambda: tts_play(tts_synth(text)), daemon=True).start()


# ═══════════════════════════════════════════════
#  Navigation Worker
# ═══════════════════════════════════════════════
def nav_worker(poses, loop_mode):
    global is_arrived, nav_running
    nav_running = True
    n = len(poses)
    mode = "循環模式" if loop_mode else "單次模式"
    print(f"\n{'─'*50}")
    print(f"  {B}🚀 巡航开始{N}  {n} 個點 [{mode}]")
    print(f"{'─'*50}")

    i = 0
    while nav_running and i < n:
        pose = poses[i]
        is_arrived = False

        name = pose.get("tts_text", pose.get("name", f"点{i}"))
        out_name = f"「{name}」" if name else f"[{i}]"
        print(f"\n{C}[{i}]{N} 前往 {out_name}  ({pose['x']:.2f}, {pose['y']:.2f})")
        play_tts(f"正在前往{name}" if name else f"正在前往第{i}点")

        code, _ = slam_client.send_target(pose, pose.get("mode", 1), pose.get("speed", 0.6))
        if code != 0:
            print(f"  {R}⚠️ 發送失敗 (code={code}){N}")
            i += 1; continue

        tick = 1200
        while tick > 0 and nav_running:
            if is_arrived: break
            time.sleep(0.1); tick -= 1

        if not is_arrived:
            print(f"  {Y}⏱ 超时{N}")
            play_tts("無法到達此點")

        i += 1
        if loop_mode and i >= n:
            poses.reverse()
            i = 0
            print(f"\n{C}↔ 反轉路徑繼續巡航{N}\n")

    nav_running = False
    print(f"\n{G}{'─'*50}{N}")
    print(f"  ✅ 巡航{'结束' if not nav_running else '被中斷'}")
    print(f"{G}{'─'*50}{N}\n")


def start_nav(loop_mode=False):
    global _nav_thread, nav_running
    if nav_running: print(f"  {Y}⚠ 巡航已在執行中{N}"); return
    if not slam_client or not slam_client.pose_list: print(f"  {Y}⚠ 任務列表為空{N}"); return
    _nav_thread = threading.Thread(target=nav_worker, args=(slam_client.pose_list[:], loop_mode), daemon=True)
    _nav_thread.start()


# ═══════════════════════════════════════════════
#  Key Loop
# ═══════════════════════════════════════════════
def show_help():
    print(f"\n{B}────────────────────────────────────────────────────{N}")
    print(f"  🐕 小創 Python 導航 v1")
    print(f"  ─────────────────────────────")
    print(f"  {G}q{N} 开始建图     {G}s{N} 记录点位     {G}l{N} 查看列表")
    print(f"  {G}w{N} 结束建图保存 {G}c{N} 单次巡航     {G}f{N} 清空列表")
    print(f"  {G}a{N} 加载地图重定位 {G}d{N} 循环巡航     {G}r{N} 移除末点")
    print(f"  {G}z{N} 暂停巡航    {G}x{N} 恢复巡航")
    print(f"  其他键 停止 SLAM")
    print(f"{B}────────────────────────────────────────────────────{N}")

def run():
    global slam_client, nav_running
    import tty, termios, select

    show_help()

    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        while True:
            if select.select([sys.stdin],[],[],0.2)[0]:
                ch = sys.stdin.read(1)
            else:
                continue

            if ch == 'q':
                code, data = slam_client.start_mapping()
                print(f"\n  {C}[建图]{N} code={code} data={data}")

            elif ch == 'w':
                ts = int(time.time())
                addr = f"/home/unitree/lab/map/mymap_{ts}.pcd"
                code, data = slam_client.end_mapping(addr)
                print(f"\n  {C}[保存]{N} {addr}")
                print(f"  code={code} data={data}")

            elif ch == 'a':
                code, data = slam_client.relocate()
                print(f"\n  {C}[重定位]{N} code={code}")
                if code == 0:
                    print(f"  {G}✅ 重定位请求已发送{N}")
                else:
                    print(f"  {R}❌ 失敗 (code={code}){N}")
                    print(f"  data={data}")

            elif ch == 's':
                # 暫停 raw mode，讓使用者輸入名稱
                termios.tcsetattr(fd, termios.TCSANOW, old)
                with _pose_lock:
                    p = dict(current_pose)
                tts_text = ""
                try:
                    print(f"\n📝 輸入此點的名稱/TTS文字（直接 Enter 跳過）: ", end="", flush=True)
                    tts_text = sys.stdin.readline().strip()
                except:
                    pass
                # 恢復 raw mode
                tty.setraw(fd)

                name = tts_text if tts_text else f"P{len(slam_client.pose_list)}"
                p["name"] = name
                p["tts_text"] = name
                p["mode"] = 1
                p["speed"] = 0.6
                slam_client.pose_list.append(p)
                n = len(slam_client.pose_list)
                yaw = math.atan2(2*(p['q_w']*p['q_z']), 1-2*p['q_z']**2)*180/math.pi
                print(f"\n  {G}✅ 记录第 {n} 点: {name}{N}")
                print(f"    位置: ({p['x']:.2f}, {p['y']:.2f})  朝向: {yaw:.0f}°")

            elif ch == 'l':
                pl = slam_client.pose_list
                if not pl:
                    print(f"\n  {Y}📭 任务列表为空{N}")
                else:
                    print(f"\n  {B}📋 任务列表 ({len(pl)} 个点){N}")
                    print(f"  {'─'*45}")
                    for i, p in enumerate(pl):
                        yaw = math.atan2(2*(p['q_w']*p['q_z']), 1-2*p['q_z']**2)*180/math.pi
                        name = p.get("name","?")
                        print(f"  [{i}] {name}  x={p['x']:.2f} y={p['y']:.2f}  {yaw:.0f}°")
                    print(f"  {'─'*45}")

            elif ch == 'c':
                print(f"\n  {C}[巡航] 单次模式{N}")
                start_nav(False)

            elif ch == 'd':
                print(f"\n  {C}[巡航] 循环模式{N}")
                start_nav(True)

            elif ch == 'f':
                slam_client.pose_list.clear()
                print(f"\n  {Y}🗑 任务列表已清空{N}")

            elif ch == 'z':
                code, data = slam_client.pause_nav()
                print(f"\n  {Y}[暂停]{N} code={code}")

            elif ch == 'x':
                code, data = slam_client.resume_nav()
                print(f"\n  {G}[恢复]{N} code={code}")

            elif ch == 'r':
                if slam_client.pose_list:
                    rm = slam_client.pose_list.pop()
                    print(f"\n  {Y}🗑 移除末点 ({rm.get('name','?')}) 剩余 {len(slam_client.pose_list)} 个{N}")
                else:
                    print(f"\n  {Y}📭 列表已空{N}")

            else:
                print(f"\n{R}🛑 停止 SLAM{N}")
                nav_running = False
                code, data = slam_client.stop_node()
                print(f"  code={code}")
                break

    except Exception as e:
        print(f"\n{R}❌ {e}{N}")
    finally:
        termios.tcsetattr(fd, termios.TCSANOW, old)


# ═══════════════════════════════════════════════
#  Main
# ═══════════════════════════════════════════════
def main():
    global slam_client
    if len(sys.argv) < 2:
        print(f"用法: {sys.argv[0]} <网口名>"); sys.exit(1)

    net = sys.argv[1]
    print(f"{C}🔄 初始化 DDS domain=0 iface={net}{N}")
    ChannelFactoryInitialize(0, net)

    print(f"{C}🔄 连接 SLAM 服务...{N}")
    slam_client = SlamClient()
    slam_client.Init()

    print(f"{C}🔄 订阅位姿 Topic...{N}")
    ChannelSubscriber(SLAM_INFO_TOPIC, String_).Init(on_slam_info)
    ChannelSubscriber(SLAM_KEY_INFO_TOPIC, String_).Init(on_slam_key)

    print(f"{G}✅ 就绪！按 a 加载地图 → s 记录 → c/d 开始巡航{N}")

    try:
        run()
    except KeyboardInterrupt:
        print(f"\n{R}🛑 用户中断{N}")
    finally:
        nav_running = False
        if slam_client:
            try: slam_client.stop_node()
            except: pass
        print(f"{C}👋 再见{N}")

if __name__ == "__main__":
    main()
