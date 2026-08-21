# -*- coding: utf-8 -*-
"""
OptiTrack / NatNet 剛體位移量量測程式

功能：
1. 接收 Motive NatNet 串流
2. 追蹤指定剛體，例如 RigidBody001
3. 按 q 第一次：記錄起點
4. 按 q 第二次：記錄終點並計算位移
5. 可重複 q 開始 / 停止量測
6. 按 ESC 離開

執行範例：
python measure_rigidbody_displacement.py 192.168.250.20 192.168.250.100 Unicast

或本機測試：
python measure_rigidbody_displacement.py 127.0.0.1 127.0.0.1 Multicast
"""

import sys
import time
import math
import threading
from NatNetClient import NatNetClient


# ============================================================
# 使用者設定
# ============================================================

# Motive 裡 RigidBody001 通常是 ID=1
# 如果你 Motive 裡 RigidBody001 的 ID 不是 1，請改這裡
TARGET_RIGID_BODY_ID = 1

# NatNet position 通常是 meter。
# 若你想輸出 mm，設成 1000.0。
# 若你的資料本來就是 mm，改成 1.0。
POSITION_SCALE = 1000.0

# 單位名稱
UNIT_NAME = "mm"

# 顯示目前位置的間隔秒數
PRINT_CURRENT_POSITION_INTERVAL = 0.5


# ============================================================
# 全域狀態
# ============================================================

latest_position = None
latest_rotation = None
latest_time = None
latest_frame_id = None

recording = False
start_position = None
start_time = None

lock = threading.Lock()
program_running = True


# ============================================================
# NatNet callback
# ============================================================

def receive_new_frame(data_dict):
    """
    每一個 mocap frame 進來時會呼叫。
    這裡只拿 frameNumber 存起來。
    """
    global latest_frame_id

    try:
        frame_id = data_dict.get("frameNumber", None)
    except Exception:
        frame_id = None

    with lock:
        latest_frame_id = frame_id


def receive_rigid_body_frame(new_id, position, rotation):
    """
    每一個剛體每一幀會呼叫一次。

    new_id   : rigid body ID
    position : (x, y, z)
    rotation : quaternion
    """
    global latest_position, latest_rotation, latest_time

    if int(new_id) != int(TARGET_RIGID_BODY_ID):
        return

    pos = (
        float(position[0]) * POSITION_SCALE,
        float(position[1]) * POSITION_SCALE,
        float(position[2]) * POSITION_SCALE,
    )

    with lock:
        latest_position = pos
        latest_rotation = rotation
        latest_time = time.time()


# ============================================================
# 工具函式
# ============================================================

def calc_displacement(p_start, p_end):
    dx = p_end[0] - p_start[0]
    dy = p_end[1] - p_start[1]
    dz = p_end[2] - p_start[2]
    dist = math.sqrt(dx * dx + dy * dy + dz * dz)
    return dx, dy, dz, dist


def print_position(label, p):
    print(
        f"{label}: "
        f"X={p[0]:.6f} {UNIT_NAME}, "
        f"Y={p[1]:.6f} {UNIT_NAME}, "
        f"Z={p[2]:.6f} {UNIT_NAME}"
    )


def get_latest_position_safe():
    with lock:
        if latest_position is None:
            return None, None, None
        return latest_position, latest_time, latest_frame_id


def handle_q_key():
    """
    q 鍵：
    第一次按下：開始記錄
    第二次按下：停止記錄並計算位移
    """
    global recording, start_position, start_time

    pos, pos_time, frame_id = get_latest_position_safe()

    if pos is None:
        print("[WARN] 尚未收到指定剛體資料，無法記錄。")
        print(f"       請確認 Motive 有開啟 Streaming，且 RigidBody ID = {TARGET_RIGID_BODY_ID}")
        return

    if not recording:
        recording = True
        start_position = pos
        start_time = time.time()

        print("\n========== 開始記錄 ==========")
        print(f"RigidBody ID: {TARGET_RIGID_BODY_ID}")
        if frame_id is not None:
            print(f"Frame ID: {frame_id}")
        print_position("Start position", start_position)
        print("再次按 q 停止並計算位移。")

    else:
        end_position = pos
        end_time = time.time()
        duration = end_time - start_time

        dx, dy, dz, dist = calc_displacement(start_position, end_position)

        print("\n========== 停止記錄 ==========")
        if frame_id is not None:
            print(f"Frame ID: {frame_id}")

        print_position("Start position", start_position)
        print_position("End position  ", end_position)

        print("\n---------- 位移量 ----------")
        print(f"dX = {dx:.6f} {UNIT_NAME}")
        print(f"dY = {dy:.6f} {UNIT_NAME}")
        print(f"dZ = {dz:.6f} {UNIT_NAME}")
        print(f"3D displacement = {dist:.6f} {UNIT_NAME}")
        print(f"Elapsed time = {duration:.3f} s")

        if duration > 0:
            speed = dist / duration
            print(f"Average speed = {speed:.6f} {UNIT_NAME}/s")

        print("----------------------------")
        print("再次按 q 可重新開始下一次量測。")

        recording = False
        start_position = None
        start_time = None


# ============================================================
# 跨平台鍵盤偵測
# ============================================================

def keyboard_loop_windows():
    """
    Windows 用 msvcrt 非阻塞讀鍵。
    """
    global program_running

    import msvcrt

    print("\n按 q 開始/停止量測，按 ESC 離開。")

    while program_running:
        if msvcrt.kbhit():
            ch = msvcrt.getch()

            if ch in [b"q", b"Q"]:
                handle_q_key()

            elif ch == b"\x1b":  # ESC
                program_running = False
                break

        time.sleep(0.02)


def keyboard_loop_linux():
    """
    Linux / WSL / Ubuntu 用 termios 非阻塞讀鍵。
    """
    global program_running

    import tty
    import termios
    import select

    print("\n按 q 開始/停止量測，按 ESC 離開。")

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)

    try:
        tty.setcbreak(fd)

        while program_running:
            dr, _, _ = select.select([sys.stdin], [], [], 0.02)

            if dr:
                ch = sys.stdin.read(1)

                if ch.lower() == "q":
                    handle_q_key()

                elif ch == "\x1b":  # ESC
                    program_running = False
                    break

            time.sleep(0.02)

    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


def keyboard_loop():
    if sys.platform.startswith("win"):
        keyboard_loop_windows()
    else:
        keyboard_loop_linux()


# ============================================================
# NatNet 連線設定
# ============================================================

def my_parse_args(arg_list, args_dict):
    """
    用法：
    python script.py serverIP clientIP Multicast/Unicast

    例如：
    python measure_rigidbody_displacement.py 192.168.250.20 192.168.250.100 Unicast
    """
    arg_list_len = len(arg_list)

    if arg_list_len > 1:
        args_dict["serverAddress"] = arg_list[1]

    if arg_list_len > 2:
        args_dict["clientAddress"] = arg_list[2]

    if arg_list_len > 3:
        if len(arg_list[3]) > 0:
            args_dict["use_multicast"] = True
            if arg_list[3][0].upper() == "U":
                args_dict["use_multicast"] = False

    return args_dict


def print_configuration(natnet_client):
    natnet_client.refresh_configuration()

    print("\n========== NatNet Configuration ==========")
    print(f"Client IP      : {natnet_client.local_ip_address}")
    print(f"Server IP      : {natnet_client.server_ip_address}")
    print(f"Command Port   : {natnet_client.command_port}")
    print(f"Data Port      : {natnet_client.data_port}")

    if natnet_client.use_multicast:
        print("Mode           : Multicast")
        print(f"Multicast Group: {natnet_client.multicast_address}")
    else:
        print("Mode           : Unicast")

    print(f"Target RigidBody ID : {TARGET_RIGID_BODY_ID}")
    print(f"Position scale      : {POSITION_SCALE}")
    print(f"Output unit         : {UNIT_NAME}")
    print("=========================================\n")


# ============================================================
# 目前位置顯示 thread
# ============================================================

def monitor_loop():
    """
    定期顯示目前剛體位置，方便確認有收到資料。
    """
    global program_running

    last_print_time = 0.0

    while program_running:
        now = time.time()

        if now - last_print_time >= PRINT_CURRENT_POSITION_INTERVAL:
            pos, pos_time, frame_id = get_latest_position_safe()

            if pos is not None:
                status = "RECORDING" if recording else "IDLE"
                frame_text = f"Frame={frame_id}" if frame_id is not None else "Frame=N/A"

                print(
                    f"[{status}] {frame_text} | "
                    f"RigidBody{TARGET_RIGID_BODY_ID:03d} "
                    f"X={pos[0]:.3f}, "
                    f"Y={pos[1]:.3f}, "
                    f"Z={pos[2]:.3f} {UNIT_NAME}"
                )
            else:
                print(f"[WAIT] 尚未收到 RigidBody ID={TARGET_RIGID_BODY_ID} 的資料...")

            last_print_time = now

        time.sleep(0.05)


# ============================================================
# 主程式
# ============================================================

def main():
    global program_running

    options_dict = {
        "clientAddress": "127.0.0.1",
        "serverAddress": "127.0.0.1",
        "use_multicast": True,
    }

    options_dict = my_parse_args(sys.argv, options_dict)

    streaming_client = NatNetClient()
    streaming_client.set_client_address(options_dict["clientAddress"])
    streaming_client.set_server_address(options_dict["serverAddress"])
    streaming_client.set_use_multicast(options_dict["use_multicast"])

    streaming_client.new_frame_listener = receive_new_frame
    streaming_client.rigid_body_listener = receive_rigid_body_frame

    # 先關閉 NatNetClient 原本大量 MoCap Frame 輸出
    streaming_client.set_print_level(0)

    is_running = streaming_client.run()

    if not is_running:
        print("ERROR: Could not start NatNet streaming client.")
        sys.exit(1)

    # run() 後再補一次，避免部分版本 run 後重設 print_level
    streaming_client.set_print_level(0)

    time.sleep(1.0)
    if streaming_client.connected() is False:
        print("ERROR: Could not connect properly.")
        print("請確認：")
        print("1. Motive 已開啟")
        print("2. Motive Streaming 已啟用")
        print("3. Server IP / Client IP 正確")
        print("4. Multicast / Unicast 設定正確")
        streaming_client.shutdown()
        sys.exit(2)

    print_configuration(streaming_client)

    monitor_thread = threading.Thread(target=monitor_loop, daemon=True)
    monitor_thread.start()

    try:
        keyboard_loop()

    except KeyboardInterrupt:
        print("\n收到 Ctrl+C，準備離開。")

    finally:
        program_running = False
        time.sleep(0.2)
        streaming_client.shutdown()
        print("NatNet client shutdown.")
        print("exiting")


if __name__ == "__main__":
    main()