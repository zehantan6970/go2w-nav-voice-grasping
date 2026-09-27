#!/usr/bin/env python3
"""
小創語音控制器 (Voice Controller)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

將 ASR 語音文字解析為結構化意圖，對應小創的實際技能。

技能列表：
  - navigate  : 導航到任務點
  - grasp     : 視覺抓取飲料（0=水, 1=可樂, 2=茶, 3=王老吉）
  - release   : 語音釋放水瓶（handover）
  - stop/sit/stand/hello/heart/dance/arm_home  : 運動控制
  - forward/backward/left/right       : 基本移動

使用方式：
  from voice_controller import parse_voice_command, intent_to_command
  parsed = parse_voice_command("去取水點拿水")
  cmd = intent_to_command(parsed)
"""

import json
import os
from typing import Optional
import requests
from dotenv import load_dotenv
from pathlib import Path

# ─── 環境變數 ───
ROOT_DIR = Path(__file__).resolve().parent
load_dotenv(ROOT_DIR / ".env")

LLM_API_KEY = os.getenv("QWEN_API_KEY", "")
LLM_BASE_URL = os.getenv("QWEN_BASE_URL", "https://dashscope.aliyuncs.com/compatible-mode/v1")
LLM_MODEL = os.getenv("QWEN_MODEL", "qwen-plus")

# ─── 快車道規則 ───
STOP_WORDS = ["停", "stop", "別動", "站住", "停下來", "緊急停止"]
SIT_WORDS = ["坐", "坐下", "sit"]
STAND_WORDS = ["站", "站起", "站起來", "stand"]
HELLO_WORDS = ["你好", "哈囉", "打招呼", "hello", "hi"]
HEART_WORDS = ["比心", "愛心", "heart"]
DANCE_WORDS = ["跳舞", "dance"]
ARM_HOME_WORDS = ["回家", "歸位", "回歸", "初始", "機械臂回家", "機械臂歸位", "arm home"]
DIRECTION_WORDS = {
    "forward": ["前進", "向前", "往前走"],
    "backward": ["後退", "向後", "退後"],
    "left": ["左轉", "向左"],
    "right": ["右轉", "向右"],
}
SHORT_CMD_THRESHOLD = 8

INTENT_SYSTEM_PROMPT = """你是一個語音指令解析器，用於控制機器狗「小創」。

請將使用者的語音文字解析為結構化 JSON，格式如下：
{"action": "...", "location": "...", "target": "..."}

可用的 action 值（嚴格使用下列之一）：
- "navigate": 導航到某個地點（location 填入地點名稱）
- "grasp": 抓取飲料（target 填入飲料類型）
- "release": 釋放/交給人
- "stop": 緊急停止
- "sit": 坐下
- "stand": 站起
- "hello": 打招呼
- "heart": 比心
- "dance": 跳舞
- "arm_home": 機械臂歸位
- "forward": 前進
- "backward": 後退
- "left": 左轉
- "right": 右轉
- "unknown": 無法理解

可用的地點名稱（location 用）：
- 櫃子前, 防靜電實驗台, 講台前, 取水點, 機械臂桌前, 實驗室中間點, 屏風前
注意：ASR 可能把「講台前」聽成「講堂前」「獎台前」等，請糾正為「講台前」

可用的飲料類型（target 用）：
- 水, 可樂, 茶, 王老吉

重要：複合指令規則
1. 如果指令包含「去取水點拿X到Y點」或類似結構，代表先去取水點抓取，再去Y點送達
   輸出時 action=grasp, location=取水點, target=飲料, 並在 action 中用 "|" 分隔：先做 action=grasp，再做 action=navigate
   例如輸入：「去取水點拿水到講台前」
   輸出：{"action": "grasp|navigate", "location": "取水點|講台前", "target": "水"}
2. 如果只說「去取水點拿X」，代表只取水
3. 如果只說「拿X到Y點」，代表先取水再送到Y

範例對照表：
輸入：去取水點拿水
輸出：{"action": "grasp", "location": "取水點", "target": "水"}

輸入：拿水到講台前
輸出：{"action": "grasp|navigate", "location": "取水點|講台前", "target": "水"}

輸入：去拿水到講台前給我
輸出：{"action": "grasp|release", "location": "取水點|講台前", "target": "水"}

輸入：去取水點拿茶到實驗室中間點
輸出：{"action": "grasp|navigate", "location": "取水點|實驗室中間點", "target": "茶"}

輸入：把水拿到實驗室中間點給我
輸出：{"action": "grasp|release", "location": "取水點|實驗室中間點", "target": "水"}

輸入：去櫃子前
輸出：{"action": "navigate", "location": "櫃子前", "target": ""}

輸入：停下來
輸出：{"action": "stop", "location": "", "target": ""}

輸入：坐
輸出：{"action": "sit", "location": "", "target": ""}

輸入：拿茶
輸出：{"action": "grasp", "location": "取水點", "target": "茶"}

輸入：去講台前釋放
輸出：{"action": "release", "location": "講台前", "target": ""}

輸入：把水給屏風前
輸出：{"action": "release", "location": "屏風前", "target": ""}

輸入：阿巴阿巴測試
輸出：{"action": "unknown", "location": "", "target": ""}

如果無法理解，輸出：
{"action": "unknown", "location": "", "target": ""}

只輸出 JSON，不要其他文字。"""


def fast_lane_match(asr_text: str) -> Optional[dict]:
    """快車道：本地規則匹配，0ms 延遲"""
    if not asr_text:
        return {"action": "unknown", "location": "", "target": ""}

    text = asr_text.strip()

    if any(w in text for w in STOP_WORDS):
        return {"action": "stop", "location": "", "target": ""}
    if any(w in text for w in SIT_WORDS):
        return {"action": "sit", "location": "", "target": ""}
    if any(w in text for w in STAND_WORDS):
        return {"action": "stand", "location": "", "target": ""}
    if any(w in text for w in HELLO_WORDS):
        return {"action": "hello", "location": "", "target": ""}
    if any(w in text for w in HEART_WORDS):
        return {"action": "heart", "location": "", "target": ""}
    if any(w in text for w in DANCE_WORDS):
        return {"action": "dance", "location": "", "target": ""}
    if any(w in text for w in ARM_HOME_WORDS):
        return {"action": "arm_home", "location": "", "target": ""}
    # 方向指令：只在文字簡短時匹配，避免與地點名稱衝突
    if len(text) <= SHORT_CMD_THRESHOLD:
        for action, keywords in DIRECTION_WORDS.items():
            if any(w in text for w in keywords):
                return {"action": action, "location": "", "target": ""}

    return None


def llm_parse(asr_text: str) -> dict:
    """慢車道：呼叫 LLM（Qwen）解析模糊/複雜指令"""
    try:
        headers = {
            "Authorization": f"Bearer {LLM_API_KEY}",
            "Content-Type": "application/json",
        }
        data = {
            "model": LLM_MODEL,
            "messages": [
                {"role": "system", "content": INTENT_SYSTEM_PROMPT},
                {"role": "user", "content": asr_text},
            ],
            "temperature": 0.1,
            "max_tokens": 150,
            "response_format": {"type": "json_object"},
        }
        resp = requests.post(
            f"{LLM_BASE_URL}/chat/completions",
            headers=headers, json=data, timeout=15,
        )
        resp.raise_for_status()
        body = resp.json()
        content = body["choices"][0]["message"]["content"]
        parsed = json.loads(content)
        return {
            "action": parsed.get("action", "unknown"),
            "location": parsed.get("location", ""),
            "target": parsed.get("target", ""),
        }
    except Exception as e:
        print(f"[LLM] 錯誤: {e}")
        return {"action": "unknown", "location": "", "target": ""}


def parse_voice_command(asr_text: str) -> dict:
    """入口函數：快車道 → 慢車道（同步，非阻塞）"""
    if not asr_text or not asr_text.strip():
        return {"action": "unknown", "location": "", "target": ""}

    fast_result = fast_lane_match(asr_text)
    if fast_result is not None:
        return fast_result

    return llm_parse(asr_text)


def intent_to_command(parsed: dict) -> dict:
    """將解析後的意圖轉換為執行指令

    Returns:
        {"type": "nav_exec", "parsed": {...}}  → 經由 nav_exec.py 直接控制 DEMO
        {"type": "direct", "cmd": [...]}        → 直接執行 robot_command.py
    """
    action = parsed.get("action", "unknown")
    location = parsed.get("location", "")
    target = parsed.get("target", "")

    robot_script = "/home/unitree/lab/all/vision+arm/asr/robot_command.py"

    # ─── 基本動作：直接走 SDK ───
    if action in ("forward", "backward", "left", "right", "stop",
                  "sit", "stand", "hello", "heart", "dance",
                  "arm_home"):
        return {
            "type": "direct",
            "cmd": ["python3", robot_script, action],
        }

    # ─── 導航/抓取/釋放：經由 nav_exec.py 控制 DEMO ───
    nav_exec_script = "/home/unitree/lab/all/vision+arm/asr/nav_exec.py"
    if action in ("navigate", "grasp", "release") or "|" in action:
        return {
            "type": "nav_exec",
            "parsed": parsed,
            "cmd": ["python3", nav_exec_script, "--json", json.dumps(parsed)],
        }

    return {"type": "unknown", "text": ""}


# ─── 直接測試 ───
if __name__ == "__main__":
    tests = [
        "去取水點拿水",
        "拿水到講台前",
        "去取水點拿茶到實驗室中間點",
        "去講台前釋放",
        "停下來", "坐", "站起來",
        "你好", "比心", "跳舞",
        "前進", "後退",
        "機械臂回家",
        "去櫃子前",
        "阿巴阿巴測試測試",
    ]
    for t in tests:
        p = parse_voice_command(t)
        c = intent_to_command(p)
        print(f"  「{t}」\n    → {p}\n    → {c}\n")
