#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
import time
import numpy as np
from collections import deque
import mediapipe as mp

# ===================== 可調參數 =====================
CONFIDENCE_DET   = 0.6
CONFIDENCE_TRACK = 0.6

# EMA 平滑（數值越小越平滑）
SMOOTH_ALPHA     = 0.25

# 輕微平滑：短窗中位數（抗閃爍）
ENABLE_FLICKER_SMOOTH = True
FLICKER_MEDIAN_WIN    = 3   # 建議 3 或 5

OPEN_THR         = 0.205
CLOSE_THR        = 0.205
STABILITY_FRAMES = 10
DRAW_SKELETON    = True
USE_FLIP         = True

# 只允許哪一隻手控制（'Right' 或 'Left'）
CONTROL_HAND     = 'Right'

# === 新增：每秒列印狀態的開關 ===
PRINT_STATUS_EVERY_SEC = True

FINGER_TIPS = [4, 8, 12, 16, 20]
FINGER_PIPS = {4: 2, 8: 6, 12: 10, 16: 14, 20: 18}

mp_hands = mp.solutions.hands
mp_draw  = mp.solutions.drawing_utils
mp_style = mp.solutions.drawing_styles

def ema(prev, x, alpha):
    return alpha * x + (1 - alpha) * prev

def norm(v):
    return np.linalg.norm(v)

def clamp(x, lo, hi):
    return max(lo, min(hi, x))

def median_of(deq):
    if not deq:
        return 0.0
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
    def __init__(self, low=CLOSE_THR, high=OPEN_THR, need_frames=STABILITY_FRAMES):
        self.low = low
        self.high = high
        self.need_frames = need_frames
        self.state_open = False
        self.buf_open = deque(maxlen=need_frames)
        self.buf_close = deque(maxlen=need_frames)

    def update(self, value):
        if value > self.high:
            self.buf_open.append(True)
            self.buf_close.clear()
        elif value < self.low:
            self.buf_close.append(True)
            self.buf_open.clear()
        # 中間區域維持
        if len(self.buf_open) == self.need_frames:
            self.state_open = True
            self.buf_open.clear()
        elif len(self.buf_close) == self.need_frames:
            self.state_open = False
            self.buf_close.clear()
        return self.state_open

def draw_meter(img, x, y, w, h, value, label, color_fill=(80,200,80)):
    cv2.rectangle(img, (x, y), (x + w, y + h), (60, 60, 60), 2)
    filled = int(w * clamp(value, 0, 1))
    cv2.rectangle(img, (x, y), (x + filled, y + h), color_fill, -1)
    cv2.putText(img, f"{label}: {value:.2f}", (x, y - 6),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (220, 220, 220), 2, cv2.LINE_AA)

def main():
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("無法開啟攝影機")
        return
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    prev_time = time.time()
    last_print_ts = 0.0

    ema_values = {}          # 'Left'/'Right' -> float
    states = {}              # 'Left'/'Right' -> HysteresisState

    # 控制狀態：True=open, False=close, None=EMPTY
    control_state = None

    # 顯示狀態來源（detect=本幀偵測更新；hold=沿用上一幀）
    state_source = "EMPTY"

    # 短窗原始分數序列：用於輕微平滑
    raw_score_hist = { 'Left': deque(maxlen=FLICKER_MEDIAN_WIN),
                       'Right': deque(maxlen=FLICKER_MEDIAN_WIN) }

    with mp_hands.Hands(
        model_complexity=1,
        max_num_hands=2,
        min_detection_confidence=CONFIDENCE_DET,
        min_tracking_confidence=CONFIDENCE_TRACK
    ) as hands:

        while True:
            ok, frame = cap.read()
            if not ok:
                break
            if USE_FLIP:
                frame = cv2.flip(frame, 1)

            img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            res = hands.process(img_rgb)

            now = time.time()
            fps = 1.0 / (now - prev_time + 1e-9)
            prev_time = now

            saw_control_hand = False
            updated_this_frame = False

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

                    # 輕微平滑：短窗中位數 → EMA
                    if ENABLE_FLICKER_SMOOTH:
                        raw_score_hist[label].append(raw_score)
                        med_score = median_of(raw_score_hist[label])
                        smooth_input = med_score
                    else:
                        smooth_input = raw_score

                    if label not in ema_values:
                        ema_values[label] = smooth_input
                    else:
                        ema_values[label] = ema(ema_values[label], smooth_input, SMOOTH_ALPHA)

                    # 畫骨架
                    if DRAW_SKELETON:
                        mp_draw.draw_landmarks(
                            frame, lm_img, mp_hands.HAND_CONNECTIONS,
                            mp_style.get_default_hand_landmarks_style(),
                            mp_style.get_default_hand_connections_style()
                        )

                    # UI 位置
                    h, w = frame.shape[:2]
                    meter_x = 20 if label == 'Left' else (w - 220)
                    meter_y = 30

                    # 只有 CONTROL_HAND 參與判定；其他手顯示為 Ignored
                    if label == CONTROL_HAND:
                        saw_control_hand = True
                        if label not in states:
                            states[label] = HysteresisState()
                        is_open = states[label].update(ema_values[label])
                        control_state = is_open         # 直接更新目前狀態
                        updated_this_frame = True

                        draw_meter(frame, meter_x, meter_y, 200, 18, ema_values[label],
                                   f"{label} Open")
                        state_text = "開 (OPEN)" if is_open else "合 (CLOSE)"
                        color = (0, 225, 0) if is_open else (0, 140, 255)
                        cv2.putText(frame, f"{label} [CONTROL]: {state_text}",
                                    (meter_x, meter_y + 48),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2, cv2.LINE_AA)
                    else:
                        # 非控制手：只顯示分數 + Ignored
                        draw_meter(frame, meter_x, meter_y, 200, 18, ema_values[label],
                                   f"{label} Open", color_fill=(120,120,120))
                        cv2.putText(frame, f"{label}: Ignored",
                                    (meter_x, meter_y + 48),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (160,160,160), 2, cv2.LINE_AA)

            # ===== 狀態維持策略 =====
            # 若本幀沒有偵測到控制手，則「沿用上一個狀態」；
            # 除非之前完全沒有狀態（None），那就維持 EMPTY。
            if updated_this_frame:
                state_source = "detect"
            else:
                state_source = "hold" if control_state is not None else "EMPTY"

            # 底部狀態列
            if control_state is True:
                status = "OPEN"
            elif control_state is False:
                status = "CLOSE"
            else:
                status = "EMPTY"

            suffix = "" if saw_control_hand else " | NO CONTROL HAND"
            cv2.putText(frame,
                        f"CONTROL={CONTROL_HAND} | STATE={status} ({state_source}) | FPS: {fps:.1f}{suffix}",
                        (20, frame.shape[0] - 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 200, 200), 2, cv2.LINE_AA)

            # === 每秒列印一次狀態（可選） ===
            if PRINT_STATUS_EVERY_SEC and (now - last_print_ts) >= 1.0:
                print(f"[{time.strftime('%H:%M:%S')}] STATE={status} SOURCE={state_source}")
                last_print_ts = now

            cv2.imshow("Hand Open/Close (Right-hand control only)", frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q') or key == 27:
                break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()


