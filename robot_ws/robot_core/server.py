# -*- coding: utf-8 -*-
import multiprocessing

# 必須放在最前面
multiprocessing.set_start_method('spawn', force=True)

import eventlet

eventlet.monkey_patch()

import os
import pty
import select
import socketio
import bridge
import subprocess
import re
import gc
import json
from flask import Flask, jsonify, send_from_directory, Response
from flask_cors import CORS

# --- 全局變數 ---
flask_app = Flask(__name__)
CORS(flask_app)
sio = socketio.Server(async_mode='eventlet', cors_allowed_origins='*')
app = socketio.WSGIApp(sio, flask_app)

shared_frames = None
shared_telemetry = None
terminal_procs = {}
MAP_PATH = "/home/unitree/lab/map/"
TASK_PATH = "/home/unitree/lab/map/"
LAB_PATH = "/home/unitree/lab/"
START_NAV_SCRIPT = "/home/unitree/Downloads/start_nav.sh"
START_STREAM_SCRIPT = "/home/unitree/Downloads/start_stream.sh"
OPENCLAW_PATH = "/home/unitree/openclaw/go2-openclaw-skill/"
terminal_buffers = {'ubuntu': '', 'openclaw': ''}
MAX_BUFFER_SIZE = 5000


# --- 函數邏輯 ---
def read_terminal_output(fd, id_name):
    global terminal_buffers
    while True:
        try:
            r, _, _ = select.select([fd], [], [], 0.2)
            if r:
                data = os.read(fd, 4096)
                if data:
                    output_str = data.decode('utf-8', 'ignore')

                    # 存入缓冲区
                    terminal_buffers[id_name] += output_str
                    if len(terminal_buffers[id_name]) > MAX_BUFFER_SIZE:
                        terminal_buffers[id_name] = terminal_buffers[id_name][-MAX_BUFFER_SIZE:]

                    sio.emit('output', {'id': id_name, 'data': output_str})
        except Exception:
            break


def start_terminal(id_name, command, init_commands=None):
    pid, fd = pty.fork()
    if pid == 0:
        # 子进程：执行终端命令
        os.execvp(command[0], command)
    else:
        # 父进程：管理输入输出
        terminal_procs[id_name] = fd
        eventlet.spawn(read_terminal_output, fd, id_name)

        if init_commands:
            def auto_input():
                # 假设 init_commands 是一个列表: [("cmd1", 1.0), ("cmd2", 2.0)]
                # 每个元组包含：命令字符串 和 等待时间(秒)
                for cmd, delay in init_commands:
                    eventlet.sleep(delay)
                    try:
                        # 确保使用 \r 作为回车，而不是 \n
                        full_cmd = f"{cmd}\r"
                        os.write(fd, full_cmd.encode('utf-8'))
                        print(f"[{id_name}] 已执行: {cmd}")
                    except Exception as e:
                        print(f"[{id_name}] 指令发送失败: {e}")

            eventlet.spawn(auto_input)


def telemetry_broadcaster(telemetry_data):
    last_data = None
    while True:
        if telemetry_data:
            try:
                current_data = dict(telemetry_data)
                if current_data != last_data:
                    sio.emit('telemetry_data', current_data)
                    last_data = current_data
            except Exception as e:
                print(f"遥测广播异常: {e}")
        eventlet.sleep(0.4)


def get_audio_status():
    """解析 amixer 输出，获取音量和静音状态"""
    try:
        output = subprocess.check_output(['amixer', '-c', '1', 'get', 'PCM']).decode('utf-8')
        vol_match = re.search(r'\[(\d+)%\]', output)
        volume = int(vol_match.group(1)) if vol_match else 50
        is_muted = '[off]' in output
        return volume, is_muted
    except Exception as e:
        print(f"查询音量失败: {e}")
        return 50, False


def run_task_sequence(sid, file_id, map_name):
    try:
        print(f"[任务编排] 第一步：执行 start_nav.sh start")
        subprocess.Popen(['bash', START_NAV_SCRIPT, 'start'])
        eventlet.sleep(10)

        print(f"[任务编排] 第二步：执行 start_nav.sh a {map_name}")
        subprocess.Popen(['bash', START_NAV_SCRIPT, 'a', map_name])
        eventlet.sleep(5)

        #print(f"[任务编排] 第三步：执行 start_stream.sh")
        #subprocess.Popen(['bash', START_STREAM_SCRIPT])
        #eventlet.sleep(5)

        print(f"[任务编排] 第四步：执行 start_nav.sh c")
        subprocess.Popen(['bash', START_NAV_SCRIPT, 'c'])
        eventlet.sleep(1)

        print(f"[任务编排] 任务自然结束，正在通知前端重置状态...")
        sio.emit('task_finished', {"fileId": file_id}, to=sid)

    except Exception as e:
        print(f"[错误] 任务编排执行异常: {e}")
        sio.emit('task_status', {"status": "error", "message": f"执行失败: {e}"}, to=sid)


def generate_selection_instruction(map_name, task_points):
    """
    转换函数：将任务点数组转换为AI Agent可理解的格式
    task_points: [{'index': 0, 'action': '...', 'tts_content': '...'}, ...]
    """
    path_segments = []

    for item in task_points:
        # 索引转字母: 0->A, 1->B, 2->C...
        point_label = chr(65 + item['index'])

        action = item.get('action', 'none')
        if action == '广播':
            mode_str = f"tts: {item.get('tts_content', '无内容')}"
        elif action == '抓取':
            mode_str = "抓取"
        elif action == '拍照':
            mode_str = "拍照"
        else:
            mode_str = "巡逻"

        path_segments.append(f"{point_label}({mode_str})")

    path_string = " -> ".join(path_segments)
    instruction = f"请在地图 {map_name} 上执行任务，请按照以下路径行动：{path_string}。"
    return instruction

# --- 路由 ---
@flask_app.route('/video_feed/<key>')
def video_feed(key):
    def generate():
        last_frame_bytes = None
        last_mtime = 0
        try:
            while True:
                # 文件方式：cam1=Go2前摄像头, cam2=3dcatch/RealSense
                file_paths = {'cam1': '/tmp/cam1_feed.jpg', 'cam2': '/tmp/cam2_feed.jpg'}
                if key in file_paths:
                    fpath = file_paths[key]
                    try:
                        mtime = os.path.getmtime(fpath)
                        if mtime > last_mtime:
                            with open(fpath, 'rb') as f:
                                frame = f.read()
                            last_mtime = mtime
                            yield (b'--frame\r\n'
                                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
                        else:
                            eventlet.sleep(0.07)
                    except:
                        eventlet.sleep(0.5)
                else:
                    frame = shared_frames.get(key) if shared_frames else None
                    if frame:
                        if frame == last_frame_bytes:
                            eventlet.sleep(0.07)
                            continue
                        last_frame_bytes = frame
                        yield (b'--frame\r\n'
                               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
                    else:
                        eventlet.sleep(0.1)
                eventlet.sleep(0.04)
        except (GeneratorExit, BrokenPipeError, ConnectionResetError):
            print(f"[Stream] 客户端已断开 {key} 的视频流连接")
        except Exception as e:
            print(f"[Stream] 视频流发生错误: {e}")
        finally:
            last_frame_bytes = None
            frame = None
            gc.collect()

    return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')


@flask_app.route('/files', methods=['GET'])
def list_files():
    if not os.path.exists(MAP_PATH):
        print(f"警告: 地圖路徑不存在: {MAP_PATH}")
        return jsonify([])
    try:
        files = [f for f in os.listdir(MAP_PATH) if f.endswith('.pcd')]
        return jsonify(files)
    except Exception as e:
        print(f"讀取文件路徑失敗: {e}")
        return jsonify([])

@flask_app.route('/map/<path:filename>')
def serve_map(filename):
    try:
        return send_from_directory(MAP_PATH, filename)
    except Exception as e:
        print(f"[DEBUG] 地图文件加载失败: {e}")
        return jsonify({"error": "Map file not found"}), 404

@flask_app.route('/openclaw/<path:filename>')
def serve_openclaw(filename):
    return send_from_directory(OPENCLAW_PATH, filename)


@flask_app.route('/api/get_all_tasks', methods=['GET'])
def get_all_tasks():
    tasks = []
    # 使用字典记录每个 file_id 对应的最佳文件路径
    # 结构: { file_id: filename }
    best_files = {}

    for filename in os.listdir(TASK_PATH):
        # 匹配备份文件
        if filename.endswith('_task.json.bak'):
            file_id = filename.replace('_task.json.bak', '')
            best_files[file_id] = filename

        # 匹配常规文件
        elif filename.endswith('_task.json'):
            file_id = filename.replace('_task.json', '')
            # 只有当还没有发现对应的 .bak 文件时，才将其放入，确保 .bak 优先级更高
            if file_id not in best_files:
                best_files[file_id] = filename

    # 解析找到的最佳文件列表
    for file_id, filename in best_files.items():
        try:
            with open(os.path.join(TASK_PATH, filename), 'r', encoding='utf-8') as f:
                data = json.load(f)
                tasks.append({
                    "file_id": file_id,
                    "name": data.get('name', file_id),
                    "map_name": data.get('map_name', f"{file_id}.pcd")
                })
        except Exception as e:
            print(f"解析错误 ({filename}): {e}")

    return jsonify(tasks)


@flask_app.route('/api/get_task/<file_id>', methods=['GET'])
def get_task(file_id):
    try:
        # 定义可能的路径
        bak_filename = f"{file_id}_task.json.bak"
        json_filename = f"{file_id}_task.json"

        bak_path = os.path.join(TASK_PATH, bak_filename)
        json_path = os.path.join(TASK_PATH, json_filename)

        # 优先级判断：优先选 bak，否则选 json
        if os.path.exists(bak_path):
            target_path = bak_path
            print(f"正在读取备份任务文件: {target_path}")
        elif os.path.exists(json_path):
            target_path = json_path
            print(f"正在读取常规任务文件: {target_path}")
        else:
            return jsonify({"error": "File not found"}), 404

        with open(target_path, 'r', encoding='utf-8') as f:
            data = json.load(f)
            return jsonify(data)

    except Exception as e:
        print(f"读取任务出错: {e}")
        return jsonify({"error": str(e)}), 500

@flask_app.route('/lab/<path:filename>')
def serve_lab(filename):
    try:
        return send_from_directory(LAB_PATH, filename)
    except Exception as e:
        print(f"[DEBUG] lab 文件加载失败: {e}")
        return jsonify({"error": "File not found"}), 404

# --- 終端輸入監聽 ---
@sio.event
def input(sid, data):
    id_name = data.get('id')
    command = data.get('command')
    if id_name in terminal_procs:
        fd = terminal_procs[id_name]
        try:
            os.write(fd, command.encode('utf-8'))
        except Exception as e:
            print(f"寫入終端錯誤: {e}")


# --- 音量与静音控制 ---
@sio.event
def volume_control(sid, data):
    try:
        vol = int(data.get('volume', 50))
        subprocess.run(['amixer', '-c', '1', 'sset', 'PCM', f'{vol}%'], check=True)
        subprocess.run(['amixer', '-c', '1', 'sset', 'PCM', 'unmute'], check=True)
        print(f"[Audio] 设置音量为: {vol}%")
    except Exception as e:
        print(f"[Audio] 设置音量失败: {e}")


@sio.event
def mute_toggle(sid, data):
    try:
        is_muted = data.get('muted')
        state = 'mute' if is_muted else 'unmute'
        subprocess.run(['amixer', '-c', '1', 'sset', 'PCM', state], check=True)
        print(f"[Audio] 静音状态已切换: {state}")
    except Exception as e:
        print(f"[Audio] 切换静音失败: {e}")


@sio.event
def request_history(sid, data):
    id_name = data.get('id')
    if id_name in terminal_buffers:
        sio.emit('output', {
            'id': id_name,
            'data': f"\r\n--- 历史记录恢复 ---\r\n{terminal_buffers[id_name]}"
        })

@sio.event
def request_audio_status(sid, data):
    volume, is_muted = get_audio_status()
    sio.emit('audio_status_update', {'volume': volume, 'muted': is_muted})


@sio.on('save_task_by_map')
def handle_save_task(sid, data):
    map_name = data.get('map_name')
    task_name = data.get('name', 'unnamed')
    mode = data.get('mode', 'mark')

    if not map_name:
        sio.emit('save_status', {"status": "error", "message": "缺失地图名称"}, to=sid)
        return

    # 如果是选点模式，注入自然语言指令
    if mode == 'selection':
        points = data.get('points_config')
        data['agent_instruction'] = generate_selection_instruction(map_name, points)
        target_filename = f"{task_name}_selection_task.json"
    else:
        target_filename = f"{task_name}_task.json"

    target_path = os.path.join(TASK_PATH, target_filename)

    try:
        if not os.path.exists(TASK_PATH):
            os.makedirs(TASK_PATH)

        with open(target_path, 'w', encoding='utf-8') as f:
            json.dump(data, f, ensure_ascii=False, indent=4)

        sio.emit('save_status', {"status": "success", "file": target_filename}, to=sid)
        print(f"任务已保存 ({'选点' if mode == 'selection' else '标点'}模式): {target_path}")
        if mode == 'selection':
            print(f"生成的 Agent 指令: {data['agent_instruction']}")

    except Exception as e:
        print(f"保存任务时出错: {str(e)}")
        sio.emit('save_status', {"status": "error", "message": str(e)}, to=sid)


@sio.on('delete_task')
def handle_delete_task(sid, data):
    task_name = data.get('name', 'unnamed')
    target_filename = f"{task_name}_task.json"
    target_filename2 = f"{task_name}_selection_task.json"
    target_path = os.path.join(TASK_PATH, target_filename)
    target_path2 = os.path.join(TASK_PATH, target_filename2)

    if os.path.exists(target_path):
        os.remove(target_path)
        print(f"文件已删除: {target_path}")
        sio.emit('delete_status', {"status": "success"}, to=sid)
    else:
        sio.emit('delete_status', {"status": "error", "message": "文件不存在"}, to=sid)

    if os.path.exists(target_path2):
        os.remove(target_path2)
        print(f"文件已删除: {target_path2}")
        sio.emit('delete_status', {"status": "success"}, to=sid)
    else:
        sio.emit('delete_status', {"status": "error", "message": "文件不存在"}, to=sid)


@sio.on('switch_mode')
def handle_switch_mode(sid, data):
    mode = data.get('mode')

    try:
        cmd = f"python3 /home/unitree/vision+arm/mode.py {mode}"
        result = subprocess.check_output(cmd, shell=True, text=True)

        print(f"模式切换成功: {result}")
        sio.emit('mode_status', {"status": "success", "message": result}, to=sid)
    except Exception as e:
        sio.emit('mode_status', {"status": "error", "message": str(e)}, to=sid)


@sio.on('task_control')
def handle_task_control(sid, data):
    action = data.get('action')
    file_id = data.get('fileId')
    map_name = data.get('mapName')
    mode = data.get('mode', 'mark')

    if mode == 'mark':
        try:
            if action == 'start_task':
                print(f"[任务控制] 收到启动指令，地图: {map_name}")
                eventlet.spawn(run_task_sequence, sid, file_id, map_name)
                sio.emit('task_status', {"status": "success", "message": "正在依次启动脚本..."}, to=sid)

            elif action == 'stop_task':
                print("[任务控制] 准备停止导航任务")
                subprocess.Popen(['bash', START_NAV_SCRIPT, 'stop'])
                sio.emit('task_status', {"status": "success", "message": "任务已停止"}, to=sid)

            elif action == 'pause_task':
                print(f"[{sid}] 收到暂停指令")
                subprocess.Popen([START_NAV_SCRIPT, 'z'])
                sio.emit('task_status', {"status": "success", "message": "任务已暂停"}, to=sid)

            elif action == 'resume_task':
                print(f"[{sid}] 收到恢复指令")
                subprocess.Popen([START_NAV_SCRIPT, 'x'])
                sio.emit('task_status', {"status": "success", "message": "任务已恢复"}, to=sid)

        except Exception as e:
            print(f"[错误] 任务控制出错: {e}")
            sio.emit('task_status', {"status": "error", "message": str(e)}, to=sid)
    else:
        if action == 'start_task':
            json_filename = f"{map_name}_selection_task.json"
            file_path = os.path.join(TASK_PATH, json_filename)

            try:
                if not os.path.exists(file_path):
                    raise FileNotFoundError(f"找不到任务文件: {json_filename}")

                with open(file_path, 'r', encoding='utf-8') as f:
                    task_data = json.load(f)
                    instruction = task_data.get("agent_instruction", "").strip()

                if not instruction:
                    raise ValueError("任务文件中 agent_instruction 为空")

                fd = terminal_procs.get('openclaw')
                if fd is not None:
                    os.write(fd, (instruction + "\r").encode('utf-8'))
                    sio.emit('task_status', {"status": "success", "message": "选点模式任务已启动"}, to=sid)
                else:
                    sio.emit('task_status', {"status": "error", "message": "openclaw 终端未初始化"}, to=sid)

            except Exception as e:
                print(f"[Error] 选点模式处理失败: {e}")
                sio.emit('task_status', {"status": "error", "message": str(e)}, to=sid)


if __name__ == '__main__':
    manager = multiprocessing.Manager()
    shared_frames = manager.dict({'cam1': None, 'cam2': None, 'cam3': None})
    shared_telemetry = manager.dict()

    # cam1由go2_cam.py写入/tmp/cam1_feed.jpg提供
    # multiprocessing.Process(target=bridge.ros_bridge_raw,
    #                         args=(shared_frames, 0, '/camera/camera/color/image_raw', 'cam1'), daemon=True).start()
    # cam2由3dcatchv5.py写入/tmp/cam2_feed.jpg提供，注释掉ROS版
    # multiprocessing.Process(target=bridge.ros_bridge_raw,
    #                         args=(shared_frames, 0, '/camera/camera/color/image_raw', 'cam2'), daemon=True).start()
    multiprocessing.Process(target=bridge.ros_bridge_compressed,
                            args=(shared_frames, 1, '/camera/image/compressed', 'cam3'), daemon=True).start()
    multiprocessing.Process(target=bridge.ros_bridge_telemetry, args=(shared_telemetry,), daemon=True).start()

    ubuntu_sequence = [("1\n", 1.0)]
    openclaw_sequence = [
        ("1\nopenclaw", 10.0),  # 执行 openclaw，等待 10 秒
        ("talk to agent", 3.0)  # 再执行 talk to agent，等待 3 秒
    ]

    start_terminal('ubuntu', ['/bin/bash', '-i'], init_commands=ubuntu_sequence)
    start_terminal('openclaw', ['/bin/bash', '-i'], init_commands=openclaw_sequence)
    eventlet.spawn(telemetry_broadcaster, shared_telemetry)

    print("服务器运行在 http://0.0.0.0:5000")
    eventlet.wsgi.server(eventlet.listen(('0.0.0.0', 5000)), app)