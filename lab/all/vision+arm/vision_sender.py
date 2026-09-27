import socket
import json
import time

def start_sender():
    # 1. 在迴圈外建立連線
    client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        client.connect(('localhost', 9999))
        print("[視覺] 已成功連線至控制端")
    except ConnectionRefusedError:
        print("[錯誤] 無法連接至 main_control.py，請確認控制程式已啟動。")
        return

    # 2. 持久發送數據
    try:
        while True:
            target = [0.4, 0.0, 0.2]
            client.send(json.dumps(target).encode())
            print(f"[視覺] 已發送目標: {target}")
            time.sleep(1) # 每秒發送一次
    except Exception as e:
        print(f"[視覺] 連線中斷: {e}")
    finally:
        client.close()

if __name__ == "__main__":
    start_sender()
