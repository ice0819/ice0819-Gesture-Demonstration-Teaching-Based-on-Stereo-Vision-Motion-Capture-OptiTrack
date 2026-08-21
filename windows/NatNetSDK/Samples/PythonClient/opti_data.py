#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os, sys, socket, struct, time, threading
import cv2
import numpy as np
from collections import deque
import mediapipe as mp

# ===================== 共同參數（依你環境修改） =====================
# ---- NatNet / Motive 側 ----
RIGID_BODY_ID = 1
SERVER_IP = "127.0.0.1"
CLIENT_IP = "127.0.0.1"
MCAST_ADDR    = "239.255.42.99"
CMD_PORT      = 1510
DATA_PORT     = 1511

# ---- UDP 送往接收端 ----
DEST_IP       = "192.168.250.100"
DEST_PORT     = 5005
SEND_CMD_TO_SAME_PORT = False
DEST_CMD_PORT = 5006

# ---- OptiTrack 尺度修正 ----
# 目前 Motive / OptiTrack 量測距離會固定放大約 1.54 倍。
# 送出 UDP 前先將 x/y/z 除以此倍率，接收端收到後紀錄的剛體位移才會回到正確尺度。
#
# 設定方式：
#   實際移動 300 mm，但 OptiTrack 量到 462 mm：
#   OPTITRACK_DISTANCE_SCALE = 462 / 300 = 1.54
#
# 注意：只修正位置 x/y/z，不修正 quaternion 姿態。
ENABLE_OPTITRACK_SCALE_CORRECTION = True
OPTITRACK_DISTANCE_SCALE = 1.54
POSITION_SEND_SCALE = (1.0 / OPTITRACK_DISTANCE_SCALE) if ENABLE_OPTITRACK_SCALE_CORRECTION else 1.0

# ---- MediaPipe 偵測與判斷 ----
CONFIDENCE_DET   = 0.6
CONFIDENCE_TRACK = 0.6
SMOOTH_ALPHA     = 0.25
ENABLE_FLICKER_SMOOTH = True
FLICKER_MEDIAN_WIN    = 3

# 開關判斷門檻（搭配 hysteresis）
OPEN_THR         = 0.205
CLOSE_THR        = 0.205
STABILITY_FRAMES = 10

DRAW_SKELETON    = True
USE_FLIP         = True
CONTROL_HAND     = 'Right'   # 只用右手控制

PRINT_STATUS_EVERY_SEC = True

# ---- CLOSE 指令抑制/冷卻（僅限制 close；座標不停送）----
CLOSE_COOLDOWN_SEC       = 3.0
HAND_ENTRY_SUPPRESS_SEC  = 4.0
SEND_CLOSE_AS_TEXT       = True

# ===================== NatNet SDK 載入路徑 =====================
CANDIDATES = [
    os.path.dirname(__file__),
    r"C:\Users\Public\NatNetSDK\Samples\PythonClient",
]
for p in CANDIDATES:
    if os.path.isdir(p) and p not in sys.path:
        sys.path.append(p)

import NatNetClient as NatNet  # 官方 Sample 的 Python 客戶端

# ===================== UDP socket =====================
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# ★★★ 右手存在旗標：現在僅用於 UI 顯示與 Close 指令觸發，不阻擋座標發送 ★★★
right_hand_present = threading.Event()
right_hand_present.clear()

PKT_FMT = "<7f"  # x,y,z,qx,qy,qz,qw（小端序）

def send_xyz_quat(pos, quat):
    # ★★★ 修改處：原本會檢查右手是否存在，現在註解掉，讓資料持續傳送 ★★★
    # if not right_hand_present.is_set():
    #     return

    # OptiTrack 尺度修正：只縮放位置，不縮放姿態 quaternion。
    # NatNet 的 pos 通常是 meter；這裡維持 meter 單位送出，只先做倍率補償。
    x = float(pos[0]) * POSITION_SEND_SCALE
    y = float(pos[1]) * POSITION_SEND_SCALE
    z = float(pos[2]) * POSITION_SEND_SCALE

    pkt = struct.pack(PKT_FMT,
                      x, y, z,
                      float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3]))
    sock.sendto(pkt, (DEST_IP, DEST_PORT))

def send_close_command():
    """送出一個 close 指令（文字或簡單 JSON），到同埠或獨立埠。"""
    # Close 指令通常還是需要手部觸發，所以這裡保留檢查，或是依需求也可拿掉
    if not right_hand_present.is_set():
        return
    payload = b"close" if SEND_CLOSE_AS_TEXT else b'{"cmd":"close"}'
    if SEND_CMD_TO_SAME_PORT:
        sock.sendto(payload, (DEST_IP, DEST_PORT))
    else:
        sock.sendto(payload, (DEST_IP, DEST_CMD_PORT))

# ===================== NatNet 客戶端（背景執行） =====================
_last_rb_send_ok = 0.0  # 僅供觀察

def _silence_natnet(client):
    for attr, val in [("set_print_level", 0), ("set_verbose", False),
                      ("verbose", False), ("print_level", 0)]:
        try:
            fn = getattr(client, attr)
            if callable(fn): fn(val)
            else: setattr(client, attr, val)
        except Exception:
            pass
    for target in [NatNet, getattr(NatNet, "NatNetClient", None)]:
        if not target: continue
        for name, val in [("VERBOSE", False), ("DEBUG", False), ("verbose", False)]:
            try: setattr(target, name, val)
            except Exception: pass
        for name in ["logger", "log", "print_fn", "printer"]:
            try:
                if hasattr(target, name):
                    setattr(target, name, lambda *a, **k: None)
            except Exception:
                pass

def _handle_rb(rigid_body_id, pos, rot):
    global _last_rb_send_ok
    if (RIGID_BODY_ID >= 0) and (int(rigid_body_id) != int(RIGID_BODY_ID)):
        return
    send_xyz_quat(pos, rot)
    _last_rb_send_ok = time.time()

def rigid_body_listener_compat(*args, **kwargs):
    # NatNet 4.x：第一參數是剛體列表
    if len(args) >= 1 and isinstance(args[0], (list, tuple)):
        rbs = args[0]
        if not rbs:
            return
        for rb in rbs:
            rid  = getattr(rb, "id", -1)
            ok   = getattr(rb, "tracking_valid", True)
            if not ok:
                continue
            pos  = getattr(rb, "position", (0.0, 0.0, 0.0))
            quat = getattr(rb, "orientation", (0.0, 0.0, 0.0, 1.0))
            _handle_rb(rid, pos, quat)
        return
    # NatNet 3.x：三參數
    if len(args) == 3:
        rid, pos, rot = args
        _handle_rb(rid, pos, rot)

def natnet_thread(stop_event: threading.Event):
    client = NatNet.NatNetClient()
    # 設定本機/遠端位址
    if hasattr(client, "set_client_address"):
        client.set_client_address(CLIENT_IP)
    elif hasattr(client, "set_local_address"):
        client.set_local_address(CLIENT_IP)
    else:
        print("❌ NatNetClient 缺少 set_client_address / set_local_address")
        return

    if hasattr(client, "set_server_address"):
        client.set_server_address(SERVER_IP)
    else:
        print("❌ NatNetClient 缺少 set_server_address")
        return

    if hasattr(client, "set_use_multicast"):
        client.set_use_multicast(True)
    if hasattr(client, "set_multicast_address"):
        client.set_multicast_address(MCAST_ADDR)
    if hasattr(client, "set_command_port"):
        client.set_command_port(CMD_PORT)
    if hasattr(client, "set_data_port"):
        client.set_data_port(DATA_PORT)

    client.rigid_body_listener = rigid_body_listener_compat
    for nm in ["new_frame_listener", "skeleton_listener", "labeled_marker_listener",
               "marker_set_listener", "rigid_body_list_listener", "asset_listener",
               "unlabeled_marker_listener", "force_plate_listener", "device_listener"]:
        if hasattr(client, nm):
            try: setattr(client, nm, None)
            except Exception: pass
    _silence_natnet(client)

    ok = False
    try:
        ok = client.run(SERVER_CMD_PORT=CMD_PORT, SERVER_DATA_PORT=DATA_PORT,
                        MULTICAST_ADDR=MCAST_ADDR, VERBOSE=False)
    except TypeError:
        try: ok = client.run()
        except Exception: ok = False

    if not ok:
        print("❌ NatNet 連線失敗")
        return
    print("✅ NatNet 已連線")
    print(f"📏 OptiTrack position send scale = {POSITION_SEND_SCALE:.6f} "
          f"(ENABLE={ENABLE_OPTITRACK_SCALE_CORRECTION}, distance_scale={OPTITRACK_DISTANCE_SCALE})")

    try:
        while not stop_event.is_set():
            time.sleep(0.1)
    finally:
        try: client.shutdown()
        except Exception: pass
        print("⏹ NatNet 已中止")

# ===================== MediaPipe 手部偵測 =====================
FINGER_TIPS = [4, 8, 12, 16, 20]
FINGER_PIPS = {4: 2, 8: 6, 12: 10, 16: 14, 20: 18}
mp_hands = mp.solutions.hands
mp_draw  = mp.solutions.drawing_utils
mp_style = mp.solutions.drawing_styles

def ema(prev, x, alpha): return alpha * x + (1 - alpha) * prev
def norm(v): return np.linalg.norm(v)
def clamp(x, lo, hi): return max(lo, min(hi, x))
def median_of(deq):
    if not deq: return 0.0
    arr = np.fromiter(deq, dtype=np.float32)
    return float(np.median(arr))

def openness_score(world_landmarks):
    WRIST = 0
    if world_landmarks is None or len(world_landmarks) != 21:
        return 0.0
    pts = np.array([[lm.x, lm.y, lm.z] for lm in world_landmarks], dtype=np.float32)
    per_finger = []
    for tip in FINGER_TIPS:
        pip = FINGER_PIPS[tip]
        d_tip = norm(pts[tip] - pts[WRIST]) + 1e-9
        d_pip = norm(pts[pip] - pts[WRIST]) + 1e-9
        ratio = d_tip / d_pip
        s = (ratio - 1.0) / 0.5
        per_finger.append(clamp(s, 0.0, 1.0))
    return float(np.mean(per_finger)) if per_finger else 0.0

class HysteresisState:
    def __init__(self, low, high, need_frames):
        self.low = float(low); self.high = float(high)
        self.need_frames = int(need_frames)
        self.state_open = False
        self.buf_open  = deque(maxlen=self.need_frames)
        self.buf_close = deque(maxlen=self.need_frames)
    def update(self, value: float) -> bool:
        if value > self.high:
            self.buf_open.append(True); self.buf_close.clear()
        elif value < self.low:
            self.buf_close.append(True); self.buf_open.clear()
        if len(self.buf_open) == self.need_frames:
            self.state_open = True; self.buf_open.clear()
        elif len(self.buf_close) == self.need_frames:
            self.state_open = False; self.buf_close.clear()
        return self.state_open

# ===== 新增：右上角綠色數值條 =====
def draw_meter(img, x, y, w, h, value, label, color_fill=(80, 200, 80)):
    """
    在 (x,y) 畫一個 0~1 的長條 meter（像第二個程式那樣）。
    """
    # 外框
    cv2.rectangle(img, (x, y), (x + w, y + h), (60, 60, 60), 2)
    # 填滿長度
    v = clamp(value, 0.0, 1.0)
    filled = int(w * v)
    cv2.rectangle(img, (x, y), (x + filled, y + h), color_fill, -1)
    # 顯示文字
    cv2.putText(img, f"{label}: {value:.2f}",
                (x, y - 6),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                (220, 220, 220), 2, cv2.LINE_AA)

def draw_top_right_info(frame, lines, margin=12, line_h=22, font_scale=0.6, thickness=2):
    h, w = frame.shape[:2]
    max_line = max(lines, key=len) if lines else ""
    (tw, th), _ = cv2.getTextSize(max_line, cv2.FONT_HERSHEY_SIMPLEX, font_scale, thickness)
    box_w = tw + 2*margin
    box_h = line_h*len(lines) + margin

    # 原本：y1 = margin  → 文字框會跟上面的綠色條疊在一起
    # 改成：往下移 60 pixels，空出一排給綠色條
    x1 = w - box_w - margin
    y1 = margin + 60          # <<< 這行改這樣
    x2 = w - margin
    y2 = y1 + box_h

    overlay = frame.copy()
    cv2.rectangle(overlay, (x1, y1), (x2, y2), (0,0,0), -1)
    cv2.addWeighted(overlay, 0.35, frame, 0.65, 0, frame)
    y = y1 + margin + int(line_h*0.8)
    for line in lines:
        cv2.putText(frame, line, (x1 + margin//2, y),
                    cv2.FONT_HERSHEY_SIMPLEX, font_scale, (240,240,240), thickness, cv2.LINE_AA)
        y += line_h


def hand_thread(stop_event: threading.Event):
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("❌ 無法開啟攝影機")
        return
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    prev_time = time.time()
    last_print_ts = 0.0

    ema_values = {}
    states     = {}
    raw_hist   = { 'Left': deque(maxlen=FLICKER_MEDIAN_WIN),
                   'Right': deque(maxlen=FLICKER_MEDIAN_WIN) }

    control_state = None
    prev_control_state = None

    last_close_sent_ts = 0.0

    control_present_prev = False
    suppress_until_ts = 0.0

    with mp_hands.Hands(
        model_complexity=1, max_num_hands=2,
        min_detection_confidence=CONFIDENCE_DET,
        min_tracking_confidence=CONFIDENCE_TRACK
    ) as hands:
        while not stop_event.is_set():
            ok, frame = cap.read()
            if not ok: break
            if USE_FLIP: frame = cv2.flip(frame, 1)

            img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            res = hands.process(img_rgb)

            now = time.time()
            fps = 1.0 / (now - prev_time + 1e-9)
            prev_time = now

            updated_this_frame = False
            saw_control = False

            if res.multi_hand_landmarks and res.multi_handedness:
                for lm_world, lm_img, handed in zip(
                    getattr(res, "multi_hand_world_landmarks", res.multi_hand_landmarks),
                    res.multi_hand_landmarks,
                    res.multi_handedness
                ):
                    label = handed.classification[0].label  # 'Left' or 'Right'
                    raw_score = openness_score(
                        lm_world.landmark if hasattr(lm_world, "landmark") else lm_world
                    )
                    # 輕微平滑：median + EMA
                    if ENABLE_FLICKER_SMOOTH:
                        raw_hist[label].append(raw_score)
                        smooth_input = median_of(raw_hist[label])
                    else:
                        smooth_input = raw_score

                    if label not in ema_values: ema_values[label] = smooth_input
                    else: ema_values[label] = ema(ema_values[label], smooth_input, SMOOTH_ALPHA)

                    if DRAW_SKELETON:
                        mp_draw.draw_landmarks(
                            frame, lm_img, mp_hands.HAND_CONNECTIONS,
                            mp_style.get_default_hand_landmarks_style(),
                            mp_style.get_default_hand_connections_style()
                        )

                    # 只讓 CONTROL_HAND 控制開關判定與 close 觸發
                    if label == CONTROL_HAND:
                        saw_control = True
                        if label not in states:
                            states[label] = HysteresisState(low=CLOSE_THR, high=OPEN_THR, need_frames=STABILITY_FRAMES)
                        is_open = states[label].update(ema_values[label])
                        control_state = is_open
                        updated_this_frame = True

                        # === 右上角綠色數值條 ===
                        h, w = frame.shape[:2]
                        meter_w, meter_h = 220, 20
                        meter_x = w - meter_w - 20
                        meter_y = 30
                        draw_meter(
                            frame,
                            meter_x,
                            meter_y,
                            meter_w,
                            meter_h,
                            ema_values[label],
                            f"{label} Open",
                            color_fill=(80, 200, 80)
                        )
                        # 顯示目前 OPEN / CLOSE
                        state_text = "OPEN" if is_open else "CLOSE"
                        color_txt = (0, 230, 0) if is_open else (0, 165, 255)
                        cv2.putText(frame,
                                    f"{label} [{state_text}]",
                                    (meter_x, meter_y + meter_h + 24),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                                    color_txt, 2, cv2.LINE_AA)

                        # 右手剛出現 → 開始 Entry Suppress
                        if not control_present_prev:
                            suppress_until_ts = now + HAND_ENTRY_SUPPRESS_SEC

                        # 邊緣觸發 CLOSE + 冷卻 + 進場抑制（右手存在才允許）
                        if (control_state is False) and (prev_control_state is not False):
                            cooldown_ok = (now - last_close_sent_ts) >= CLOSE_COOLDOWN_SEC
                            entry_ok    = now >= suppress_until_ts
                            if cooldown_ok and entry_ok and saw_control:
                                send_close_command()
                                last_close_sent_ts = now
                                cv2.putText(frame, "SENT: close", (20, 80),
                                            cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 180, 255), 2, cv2.LINE_AA)

                    else:
                        # 非控制手，如果想看數值也可以畫灰色條（可留可刪）
                        h, w = frame.shape[:2]
                        meter_w, meter_h = 220, 20
                        meter_x = 20
                        meter_y = 30
                        draw_meter(
                            frame,
                            meter_x,
                            meter_y,
                            meter_w,
                            meter_h,
                            ema_values[label],
                            f"{label} Open",
                            color_fill=(120, 120, 120)
                        )
                        cv2.putText(frame,
                                    f"{label}: Ignored",
                                    (meter_x, meter_y + meter_h + 24),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                                    (160, 160, 160), 2, cv2.LINE_AA)

            # 更新右手存在旗標
            if saw_control:
                right_hand_present.set()
            else:
                right_hand_present.clear()

            # 狀態維持
            state_source = "detect" if updated_this_frame else ("hold" if control_state is not None else "EMPTY")
            if control_state is True:
                status = "OPEN"
            elif control_state is False:
                status = "CLOSE"
            else:
                status = "EMPTY"

            suppress_left = max(0.0, suppress_until_ts - now)
            cooldown_left = max(0.0, CLOSE_COOLDOWN_SEC - (now - last_close_sent_ts))
            hud_lines = [
                f"Hand: {CONTROL_HAND} | Present: {'YES' if saw_control else 'NO'}",
                f"State: {status} ({state_source})",
                f"Entry Suppress: {suppress_left:.1f}s",
                f"Close Cooldown: {cooldown_left:.1f}s",
                f"FPS: {fps:.1f}",
            ]
            # 右上角已經有 meter，這裡黑底文字會稍微往內縮一點，但還是 OK
            draw_top_right_info(frame, hud_lines)

            suffix = "" if saw_control else " | NO CONTROL HAND"
            cv2.putText(frame,
                        f"CONTROL={CONTROL_HAND} | STATE={status} ({state_source}) | FPS: {fps:.1f}{suffix}",
                        (20, frame.shape[0] - 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 200, 200), 2, cv2.LINE_AA)

            if PRINT_STATUS_EVERY_SEC and (now - last_print_ts) >= 1.0:
                print(f"[{time.strftime('%H:%M:%S')}] STATE={status} SOURCE={state_source} "
                      f"| suppress_left={suppress_left:.1f}s | cooldown_left={cooldown_left:.1f}s | present={saw_control}")
                last_print_ts = now

            cv2.imshow("Right-hand Close -> send 'close' (entry-suppress + cooldown)", frame)

            control_present_prev = saw_control
            prev_control_state = control_state

            key = cv2.waitKey(1) & 0xFF
            if key in (ord('q'), 27): break

    cap.release()
    cv2.destroyAllWindows()
    print("⏹ 手部偵測已中止")

# ===================== 主程式：並行執行 =====================
def main():
    stop_event = threading.Event()
    # 背景啟動 NatNet 串流（會依 right_hand_present 決定是否傳）
    t_natnet = threading.Thread(target=natnet_thread, args=(stop_event,), daemon=True)
    t_natnet.start()

    # 前景跑手勢
    try:
        hand_thread(stop_event)
    finally:
        stop_event.set()
        t_natnet.join(timeout=2.0)
        try: sock.close()
        except Exception: pass
        print("✅ 程式結束")

if __name__ == "__main__":
    main()