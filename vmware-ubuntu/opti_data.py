#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
TM + OptiTrack rigid-body teleoperation + Robotiq gripper toggle + 30 Hz Excel logger.

功能：
1. 接收剛體 UDP pose：<7f = x,y,z,qx,qy,qz,qw。
2. 接收 close 指令：b"close" 或 JSON 內含 close。
3. 第一次收到 close：
   - 只啟用教導 / 剛體同步控制手臂。
   - 啟動 Excel 資料紀錄，手臂與剛體同一時間建立 0 點。
   - 不動作夾爪。
4. 第二次收到 close：夾爪用位置控制直接關閉。
5. 第三次收到 close：夾爪開啟。
6. 後續 close：關閉/開啟交替切換。
7. 資料以 30 Hz 寫入 Excel：
   - 手臂末端「實際 feedback TCP pose」的絕對值與相對初始值。
   - OptiTrack 剛體 pose 的絕對值與相對初始值。
   - 夾爪命令位置與開關狀態。

注意：
- 本程式已移除力感測與力量控制。
- 手臂末端資料改用 /feedback_states 內的實際 TCP pose 欄位。
  若你的 tm_msgs/FeedbackState 欄位名稱不是 tool_pose 或 tcp_pose，請修改 FEEDBACK_POSE_FIELD_CANDIDATES。
"""

import math
import os
import socket
import struct
import sys
import threading
import time
from collections import deque
from datetime import datetime

# 修復 Qt Wayland 錯誤；本程式不使用 OpenCV，但保留無害
os.environ.setdefault("QT_QPA_PLATFORM", "xcb")

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from tm_msgs.msg import FeedbackState
from tm_msgs.srv import SendScript

import serial
import minimalmodbus
import ROS2_gripper as rq

try:
    from openpyxl import Workbook
except Exception as e:
    Workbook = None
    print("[WARN] openpyxl not available. 請安裝：pip install openpyxl")
    print("       import error:", e)


# ============================================================
# 使用者設定
# ============================================================

# ---- Robotiq / RS485 ----
PORT = "/dev/ttyUSB0"
PORT_FALLBACK = "/dev/ttyUSB0"
SLAVE_ID = 9
GRIPPER_SPEED = 150       # 0..255
GRIPPER_FORCE = 200       # 0..255
POS_MIN_MM = 0.0
POS_MAX_MM = 85.0

# 啟動時開口與 close toggle 的開/關位置
GRIPPER_OPEN_MM = 40.0    # 收到第二次 close 時開到這個位置；要全開可改 85.0
GRIPPER_CLOSE_MM = 0.0    # 收到第一次 close 時關到這個位置

# ---- TM arm ----
INITIAL_ARM_POSE = [500.0, 200.0, 420.0, -180.0, 0.0, 90.0]  # x,y,z,rx,ry,rz; mm, deg
INITIAL_ARM_CMD = 'PTP("CPP",500.0, 200.0, 420.0, -180.0, 0.0, 90.0,100,100,100,false,0,2,4)'

# 手→機械手臂比例。ratio=10 表示剛體移動 10 mm，手臂移動 1 mm。
MOVE_RATIO = 1
MIN_SEND_INTERVAL_SEC = 0.1

# ---- TM feedback TCP pose ----
# 會從 /feedback_states 裡依序嘗試讀取這些欄位。
# 常見可能是 tool_pose 或 tcp_pose，格式預期為 [x,y,z,rx,ry,rz]。
FEEDBACK_POSE_FIELD_CANDIDATES = ("tool_pose", "tcp_pose")

# 若不確定單位請保持 auto：
# - xyz 小於 10 時自動視為 m 並轉 mm；否則視為 mm。
# - rpy 絕對值小於約 2π 時自動視為 rad 並轉 deg；否則視為 deg。
FEEDBACK_POSITION_UNITS = "auto"   # "auto", "m", "mm"
FEEDBACK_RPY_UNITS = "auto"        # "auto", "rad", "deg"

# 手臂 Joint 狀態 UDP 廣播，可給其他程式看。不要用可保留不影響。
UDP_IP = "192.168.250.40"
UDP_PORT = 8888

# ---- 剛體 UDP ----
RB_PACKET_FMT = "<7f"
RB_INPUT_UNITS = "m"      # 傳輸端若送 m，這裡自動 *1000 變 mm；若已是 mm 改成 "mm"
RB_BIND_IP = "0.0.0.0"
RB_BIND_PORT = 5005
CMD_BIND_PORT = 5006

# ---- logger ----
RECORD_ROOT = "records"
LOG_HZ = 30.0
COOLDOWN_SEC = 1.0        # close 指令最小間隔，避免同一手勢連續觸發


# ============================================================
# 數學工具
# ============================================================

def quat_to_euler_sxyz(qx, qy, qz, qw):
    norm = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
    if norm == 0:
        return 0.0, 0.0, 0.0
    qx, qy, qz, qw = qx/norm, qy/norm, qz/norm, qw/norm

    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (qw * qy - qz * qx)
    if sinp >= 1.0:
        pitch = math.pi / 2.0
    elif sinp <= -1.0:
        pitch = -math.pi / 2.0
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def quat_to_rpy_deg(quat):
    qx, qy, qz, qw = quat
    rr, rp, ry = quat_to_euler_sxyz(qx, qy, qz, qw)
    return math.degrees(rr), math.degrees(rp), math.degrees(ry)


def shortest_diff_deg(current_deg, target_deg):
    diff = current_deg - target_deg
    return (diff + 180.0) % 360.0 - 180.0


def pose_rel_deg_abs(pose, base_pose):
    """pose/base: [x,y,z,rx,ry,rz] -> relative [dx,dy,dz,drx,dry,drz]."""
    return [
        pose[0] - base_pose[0],
        pose[1] - base_pose[1],
        pose[2] - base_pose[2],
        shortest_diff_deg(pose[3], base_pose[3]),
        shortest_diff_deg(pose[4], base_pose[4]),
        shortest_diff_deg(pose[5], base_pose[5]),
    ]


def normalize_feedback_pose6(raw_pose6):
    """把 FeedbackState 的 TCP pose 轉成 [x,y,z,rx,ry,rz]，單位固定為 mm / deg。"""
    if raw_pose6 is None or len(raw_pose6) < 6:
        return None

    vals = [float(v) for v in raw_pose6[:6]]
    xyz = vals[:3]
    rpy = vals[3:6]

    pos_units = FEEDBACK_POSITION_UNITS.lower()
    if pos_units == "m":
        xyz = [v * 1000.0 for v in xyz]
    elif pos_units == "mm":
        pass
    else:
        # auto：機械手臂工作空間若以 m 表示通常小於 10；若已是 mm 通常數百。
        if max(abs(v) for v in xyz) < 10.0:
            xyz = [v * 1000.0 for v in xyz]

    rpy_units = FEEDBACK_RPY_UNITS.lower()
    if rpy_units == "rad":
        rpy = [math.degrees(v) for v in rpy]
    elif rpy_units == "deg":
        pass
    else:
        # auto：角度若用 rad 通常不超過 2π；若用 deg 可能接近 90/180。
        if max(abs(v) for v in rpy) <= (2.0 * math.pi + 0.2):
            rpy = [math.degrees(v) for v in rpy]

    return xyz + rpy


# ============================================================
# Gripper
# ============================================================

def resolve_serial_port():
    if os.path.exists(PORT):
        return PORT
    if os.path.exists(PORT_FALLBACK):
        print(f"[gripper] PORT not found: {PORT}")
        print(f"[gripper] fallback to: {PORT_FALLBACK}")
        return PORT_FALLBACK
    return PORT


def ensure_serial_ok(port):
    s = serial.Serial(port, 115200, timeout=1)
    s.close()


def setup_minimalmodbus(port):
    ins = minimalmodbus.Instrument(port, SLAVE_ID, debug=False)
    ins.mode = minimalmodbus.MODE_RTU
    ins.serial.baudrate = 115200
    ins.serial.bytesize = 8
    ins.serial.parity = serial.PARITY_NONE
    ins.serial.stopbits = 1
    ins.serial.timeout = 0.2
    ins.clear_buffers_before_each_transaction = True
    ins.close_port_after_each_call = True
    return ins


def setup_gripper(port):
    g = rq.RobotiqGripper(portname=port, slaveaddress=SLAVE_ID)
    g.resetActivate()
    time.sleep(0.3)
    if hasattr(g, "setCalibration"):
        g.setCalibration(openmm=85.0, closemm=0.0, openbit=0, closebit=255)
    try:
        g.goTomm(GRIPPER_OPEN_MM, GRIPPER_SPEED, GRIPPER_FORCE)
        print(f"[gripper] Activation completed @ open {GRIPPER_OPEN_MM:.1f} mm")
    except Exception as e:
        print("[gripper] initial goTomm error:", e)
    return g


class GripperToggle:
    def __init__(self, gripper):
        self.g = gripper
        self.closed = False
        self.cmd_mm = GRIPPER_OPEN_MM
        self._lock = threading.Lock()

    def toggle(self):
        with self._lock:
            if self.closed:
                target = GRIPPER_OPEN_MM
                state = "OPEN"
                self.closed = False
            else:
                target = GRIPPER_CLOSE_MM
                state = "CLOSE"
                self.closed = True

            try:
                self.g.goTomm(target, GRIPPER_SPEED, GRIPPER_FORCE)
                self.cmd_mm = float(target)
                print(f"[gripper] {state}: goTomm({target:.1f} mm)")
            except Exception as e:
                print(f"[gripper] {state} command error:", e)
            return state, self.cmd_mm

    def snapshot(self):
        with self._lock:
            return {
                "gripper_state": "CLOSE" if self.closed else "OPEN",
                "gripper_cmd_mm": float(self.cmd_mm),
            }


# ============================================================
# 剛體 Socket 接收器
# ============================================================

class UdpRigidBodyReceiver:
    def __init__(self):
        self._scale_pos = 1000.0 if RB_INPUT_UNITS.lower() == "m" else 1.0
        self._pos_mm = None
        self._quat = None
        self._last_pose_ts = None
        self._lock = threading.Lock()

        self._stop_evt = threading.Event()
        self._th_pose = threading.Thread(target=self._loop_pose_and_cmd_on_5005, daemon=True)
        self._th_cmd = threading.Thread(target=self._loop_cmd_5006, daemon=True)
        self._sock_pose = None
        self._sock_cmd = None
        self._pkt_size = struct.calcsize(RB_PACKET_FMT)

        self._close_events = deque()
        self._close_lock = threading.Lock()

    def start(self):
        self._setup_pose_socket()
        self._setup_cmd_socket()
        self._stop_evt.clear()
        self._th_pose.start()
        self._th_cmd.start()

    def stop(self):
        self._stop_evt.set()
        for s in [self._sock_pose, self._sock_cmd]:
            try:
                if s:
                    s.close()
            except Exception:
                pass
        for t in [self._th_pose, self._th_cmd]:
            if t:
                t.join(timeout=1.0)

    def get_pose(self):
        with self._lock:
            if self._pos_mm is None or self._quat is None:
                return None, None, None
            return tuple(self._pos_mm), tuple(self._quat), self._last_pose_ts

    def pop_close(self):
        with self._close_lock:
            if self._close_events:
                return self._close_events.popleft()
            return None

    def _emit_close(self, src):
        with self._close_lock:
            self._close_events.append({"src": src, "ts": time.time()})

    def _setup_pose_socket(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.settimeout(1.0)
        s.bind((RB_BIND_IP, RB_BIND_PORT))
        self._sock_pose = s
        print(f"[RB] listen pose/cmd on {RB_BIND_IP}:{RB_BIND_PORT}")

    def _setup_cmd_socket(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.settimeout(1.0)
        try:
            s.bind((RB_BIND_IP, CMD_BIND_PORT))
            self._sock_cmd = s
            print(f"[CMD] listen close on {RB_BIND_IP}:{CMD_BIND_PORT}")
        except Exception as e:
            print(f"[CMD] bind {CMD_BIND_PORT} failed, maybe unused:", e)
            self._sock_cmd = None

    def _handle_packet(self, data, src_label):
        # pose packet
        if len(data) >= self._pkt_size:
            try:
                x, y, z, qx, qy, qz, qw = struct.unpack(RB_PACKET_FMT, data[:self._pkt_size])
                px, py, pz = x * self._scale_pos, y * self._scale_pos, z * self._scale_pos
                with self._lock:
                    self._pos_mm = (px, py, pz)
                    self._quat = (qx, qy, qz, qw)
                    self._last_pose_ts = time.time()
                return
            except Exception:
                pass

        # close command
        dlow = data.strip().lower()
        if dlow == b"close" or (b'"cmd"' in dlow and b"close" in dlow):
            self._emit_close(src_label)

    def _loop_pose_and_cmd_on_5005(self):
        while not self._stop_evt.is_set():
            try:
                data, _ = self._sock_pose.recvfrom(4096)
                self._handle_packet(data, src_label="5005")
            except socket.timeout:
                continue
            except Exception as e:
                if not self._stop_evt.is_set():
                    print("[RB] error:", e)
                break

    def _loop_cmd_5006(self):
        if not self._sock_cmd:
            return
        while not self._stop_evt.is_set():
            try:
                data, _ = self._sock_cmd.recvfrom(4096)
                self._handle_packet(data, src_label="5006")
            except socket.timeout:
                continue
            except Exception as e:
                if not self._stop_evt.is_set():
                    print("[CMD] error:", e)
                break


# ============================================================
# ROS2：SendScript + Arm 控制
# ============================================================

class SendScriptClient(Node):
    def __init__(self):
        super().__init__("send_script_client")
        self.cli = self.create_client(SendScript, "send_script")
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("等待 send_script 服務...")
        self.req = SendScript.Request()

    def send_cmd(self, cmd: str):
        self.req.id = "demo"
        self.req.script = cmd
        return self.cli.call_async(self.req)


class JointStateBroadcaster:
    def __init__(self, ip=UDP_IP, port=UDP_PORT):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.target_ip = ip
        self.port = port

    def send(self, joint_angles):
        try:
            data = struct.pack("6f", *joint_angles[:6])
            self.sock.sendto(data, (self.target_ip, self.port))
        except Exception:
            pass


class ArmController(Node):
    def __init__(self, client: SendScriptClient, rb_receiver: UdpRigidBodyReceiver):
        super().__init__("arm_controller_with_feedback")
        self.client = client
        self.rb_rx = rb_receiver
        self.joint_sender = JointStateBroadcaster()
        self.last_udp_send_time = 0.0

        # command pose：用於產生下一個 PTP 指令
        self.initial_arm_pose = list(INITIAL_ARM_POSE)
        self.current_arm_pose = list(INITIAL_ARM_POSE)

        # feedback pose：用於 Excel 紀錄與誤差比較，來源是 /feedback_states 的實際 TCP pose
        self.feedback_arm_pose = None
        self.feedback_pose_stamp = None
        self._warned_no_feedback_pose = False
        self._pose_lock = threading.Lock()

        self.initial_rbody_pos_mm = None
        self.initial_rbody_rpy_deg = None
        self.current_rbody_pos_mm = None
        self.current_rbody_ori = None

        self.last_future = None
        self.ready_to_move = False
        self.listening_enabled = False
        self.ratio = float(MOVE_RATIO)

        self.create_subscription(FeedbackState, "/feedback_states", self.feedback_callback, 15)

        self.last_future = self.client.send_cmd(INITIAL_ARM_CMD)
        self.get_logger().info(f"已傳送初始命令: {INITIAL_ARM_CMD}")

        self.last_send_time = 0.0
        self.min_send_interval = MIN_SEND_INTERVAL_SEC
        self.create_timer(0.1, self.poll_rigidbody_once)

    def set_ratio(self, ratio: float):
        self.ratio = float(ratio)
        self.get_logger().info(f"[ArmController] 控制比例 ratio = {self.ratio:.3f}")

    def set_listening(self, enabled: bool):
        self.listening_enabled = bool(enabled)
        if enabled:
            self.rebase_reference()

    def rebase_reference(self):
        pos_mm, quat, _ = self.rb_rx.get_pose()
        if pos_mm is not None and quat is not None:
            self.initial_rbody_pos_mm = tuple(pos_mm)
            self.initial_rbody_rpy_deg = quat_to_rpy_deg(quat)
        fb_pose, _ = self.get_feedback_arm_pose()
        with self._pose_lock:
            # 控制基準優先採用實際 feedback TCP pose；若尚未收到 feedback，才退回 commanded pose。
            self.initial_arm_pose = list(fb_pose) if fb_pose is not None else list(self.current_arm_pose)
            self.current_arm_pose = list(self.initial_arm_pose)
        self.get_logger().info(
            f"[ArmController] rebase DONE | rb={self.initial_rbody_pos_mm}, "
            f"rb_rpy={self.initial_rbody_rpy_deg}, arm_base={self.initial_arm_pose}, "
            f"arm_source={'feedback' if fb_pose is not None else 'commanded'}"
        )

    def get_current_arm_pose(self):
        """回傳最後一次送出的 commanded pose；控制用，不作為正式紀錄。"""
        with self._pose_lock:
            return list(self.current_arm_pose)

    def get_feedback_arm_pose(self):
        """回傳 /feedback_states 實際 TCP pose，單位 mm / deg。"""
        with self._pose_lock:
            if self.feedback_arm_pose is None:
                return None, None
            return list(self.feedback_arm_pose), self.feedback_pose_stamp

    def _extract_feedback_pose6(self, msg: FeedbackState):
        for field_name in FEEDBACK_POSE_FIELD_CANDIDATES:
            if hasattr(msg, field_name):
                raw = getattr(msg, field_name)
                try:
                    if raw is not None and len(raw) >= 6:
                        return normalize_feedback_pose6(raw), field_name
                except Exception:
                    pass
        return None, None

    def poll_rigidbody_once(self):
        pos_mm, quat, _ = self.rb_rx.get_pose()
        if pos_mm is None:
            return
        self.current_rbody_pos_mm = pos_mm
        self.current_rbody_ori = quat
        if self.ready_to_move and self.listening_enabled:
            self._send_periodic_move()

    def feedback_callback(self, msg: FeedbackState):
        now = time.time()

        fb_pose, field_name = self._extract_feedback_pose6(msg)
        if fb_pose is not None:
            with self._pose_lock:
                self.feedback_arm_pose = fb_pose
                self.feedback_pose_stamp = now
        elif not self._warned_no_feedback_pose:
            self._warned_no_feedback_pose = True
            try:
                fields = list(getattr(msg, "__slots__", []))
            except Exception:
                fields = []
            self.get_logger().warn(
                "沒有在 /feedback_states 找到 TCP pose 欄位。"
                f"目前嘗試欄位={FEEDBACK_POSE_FIELD_CANDIDATES}，msg fields={fields}。"
                " Excel 的手臂 feedback 欄位會是 NaN，請確認 FeedbackState 欄位名稱。"
            )

        if len(msg.joint_pos) >= 6 and (now - self.last_udp_send_time) > 0.1:
            self.joint_sender.send(msg.joint_pos)
            self.last_udp_send_time = now

        moving_joints = any(abs(v) > 1000 for v in msg.joint_vel)
        tcp_lin = msg.tcp_speed[:3]
        tcp_ang = msg.tcp_speed[3:]
        moving_tcp = any(abs(v) > 5e-1 for v in (tcp_lin + tcp_ang))
        status = "moving" if (moving_joints or moving_tcp) else ("project_idle" if msg.project_run else "stopped")
        self.ready_to_move = (status == "project_idle") and self.listening_enabled

    def _send_periodic_move(self):
        now = time.time()
        if now - self.last_send_time < self.min_send_interval:
            return
        if not (self.initial_rbody_pos_mm and self.current_rbody_pos_mm and self.current_rbody_ori):
            return

        dx_mm = self.current_rbody_pos_mm[0] - self.initial_rbody_pos_mm[0]
        dy_mm = self.current_rbody_pos_mm[1] - self.initial_rbody_pos_mm[1]
        dz_mm = self.current_rbody_pos_mm[2] - self.initial_rbody_pos_mm[2]

        r = self.ratio if abs(self.ratio) > 1e-6 else 1.0
        tx = self.initial_arm_pose[0] + (dx_mm / r)
        ty = self.initial_arm_pose[1] + (dy_mm / r)
        tz = self.initial_arm_pose[2] + (dz_mm / r)

        cur_r_deg, cur_p_deg, cur_y_deg = quat_to_rpy_deg(self.current_rbody_ori)
        if self.initial_rbody_rpy_deg is None:
            init_r_deg, init_p_deg, init_y_deg = cur_r_deg, cur_p_deg, cur_y_deg
        else:
            init_r_deg, init_p_deg, init_y_deg = self.initial_rbody_rpy_deg

        d_roll = shortest_diff_deg(cur_r_deg, init_r_deg) / r
        d_pitch = shortest_diff_deg(cur_p_deg, init_p_deg) / r
        d_yaw = shortest_diff_deg(cur_y_deg, init_y_deg) / r

        roll_deg = self.initial_arm_pose[3] + d_roll
        pitch_deg = self.initial_arm_pose[4] + d_pitch
        yaw_deg = self.initial_arm_pose[5] + d_yaw

        with self._pose_lock:
            self.current_arm_pose = [tx, ty, tz, roll_deg, pitch_deg, yaw_deg]

        cmd = (
            f'PTP("CPP",{tx:.2f},{ty:.2f},{tz:.2f},'
            f'{roll_deg:.2f},{pitch_deg:.2f},{yaw_deg:.2f},'
            '100,100,100,false,0,2,4)'
        )

        if self.last_future and not self.last_future.done():
            self.last_future.cancel()
        self.last_future = self.client.send_cmd(cmd)
        self.last_send_time = now
        self.ready_to_move = False


# ============================================================
# 30 Hz Excel Logger
# ============================================================

class ExcelPoseLogger:
    def __init__(self, arm: ArmController, rb_rx: UdpRigidBodyReceiver, gripper_toggle: GripperToggle):
        self.arm = arm
        self.rb_rx = rb_rx
        self.gripper = gripper_toggle
        self.recording = False
        self.stop_evt = threading.Event()
        self.thread = None

        self.t0 = None
        self.out_dir = None
        self.xlsx_path = None
        self.wb = None
        self.ws = None

        self.arm_base_pose = None
        self.rb_base_pos = None
        self.rb_base_rpy = None

    def _make_output(self):
        if Workbook is None:
            raise RuntimeError("openpyxl 未安裝，無法輸出 Excel。請執行：pip install openpyxl")
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.out_dir = os.path.join(RECORD_ROOT, f"pose_compare_{ts}")
        os.makedirs(self.out_dir, exist_ok=True)
        self.xlsx_path = os.path.join(self.out_dir, "arm_rigidbody_pose_30hz.xlsx")
        self.wb = Workbook(write_only=True)
        self.ws = self.wb.create_sheet("pose_30hz")
        self.ws.append([
            "sample_index",
            "wall_time_iso",
            "t_since_start_s",
            "logger_hz",
            "arm_fb_abs_x_mm", "arm_fb_abs_y_mm", "arm_fb_abs_z_mm", "arm_fb_abs_rx_deg", "arm_fb_abs_ry_deg", "arm_fb_abs_rz_deg",
            "arm_fb_rel_x_mm", "arm_fb_rel_y_mm", "arm_fb_rel_z_mm", "arm_fb_rel_rx_deg", "arm_fb_rel_ry_deg", "arm_fb_rel_rz_deg",
            "rb_abs_x_mm", "rb_abs_y_mm", "rb_abs_z_mm", "rb_abs_roll_deg", "rb_abs_pitch_deg", "rb_abs_yaw_deg",
            "rb_rel_x_mm", "rb_rel_y_mm", "rb_rel_z_mm", "rb_rel_roll_deg", "rb_rel_pitch_deg", "rb_rel_yaw_deg",
            "gripper_state", "gripper_cmd_mm",
            "arm_feedback_age_s",
            "rb_pose_age_s",
        ])

    def start(self):
        if self.recording:
            return
        pos_mm, quat, pose_ts = self.rb_rx.get_pose()
        if pos_mm is None or quat is None:
            print("[logger] 尚未收到剛體資料，無法開始紀錄。")
            return

        fb_pose, fb_ts = self.arm.get_feedback_arm_pose()
        if fb_pose is None:
            print("[logger] 尚未收到手臂 /feedback_states 的實際 TCP pose，無法開始紀錄。")
            print("[logger] 請確認 /feedback_states 內是否有 tool_pose 或 tcp_pose 欄位。")
            return

        self._make_output()
        self.t0 = time.time()
        self.arm_base_pose = list(fb_pose)
        self.rb_base_pos = tuple(pos_mm)
        self.rb_base_rpy = quat_to_rpy_deg(quat)

        self.stop_evt.clear()
        self.recording = True
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

        print("[logger] START 30 Hz Excel recording")
        print(f"[logger] file: {self.xlsx_path}")
        print(f"[logger] arm base: {self.arm_base_pose}")
        print(f"[logger] rb base pos: {self.rb_base_pos}")
        print(f"[logger] rb base rpy: {self.rb_base_rpy}")

    def stop(self):
        if not self.recording and self.wb is None:
            return
        self.stop_evt.set()
        if self.thread:
            self.thread.join(timeout=2.0)
        self.recording = False
        try:
            if self.wb is not None and self.xlsx_path is not None:
                self.wb.save(self.xlsx_path)
                print(f"[logger] SAVED: {self.xlsx_path}")
        except Exception as e:
            print("[logger] save xlsx error:", e)
        self.thread = None
        self.wb = None
        self.ws = None

    def _append_row(self, sample_index):
        now = time.time()
        t_rel = now - self.t0
        wall = datetime.now().isoformat(timespec="milliseconds")

        arm_pose, arm_ts = self.arm.get_feedback_arm_pose()
        if arm_pose is None:
            arm_pose = [float("nan")] * 6
            arm_rel = [float("nan")] * 6
            arm_age = float("nan")
        else:
            arm_rel = pose_rel_deg_abs(arm_pose, self.arm_base_pose)
            arm_age = now - arm_ts if arm_ts is not None else float("nan")

        rb_pos, rb_quat, rb_ts = self.rb_rx.get_pose()
        if rb_pos is None or rb_quat is None:
            rb_abs = [float("nan")] * 6
            rb_rel = [float("nan")] * 6
            rb_age = float("nan")
        else:
            rb_rpy = quat_to_rpy_deg(rb_quat)
            rb_abs = [rb_pos[0], rb_pos[1], rb_pos[2], rb_rpy[0], rb_rpy[1], rb_rpy[2]]
            rb_rel = [
                rb_pos[0] - self.rb_base_pos[0],
                rb_pos[1] - self.rb_base_pos[1],
                rb_pos[2] - self.rb_base_pos[2],
                shortest_diff_deg(rb_rpy[0], self.rb_base_rpy[0]),
                shortest_diff_deg(rb_rpy[1], self.rb_base_rpy[1]),
                shortest_diff_deg(rb_rpy[2], self.rb_base_rpy[2]),
            ]
            rb_age = now - rb_ts if rb_ts is not None else float("nan")

        grip = self.gripper.snapshot()
        row = [
            int(sample_index),
            wall,
            round(t_rel, 6),
            float(LOG_HZ),
            *[round(float(v), 6) for v in arm_pose],
            *[round(float(v), 6) for v in arm_rel],
            *[round(float(v), 6) for v in rb_abs],
            *[round(float(v), 6) for v in rb_rel],
            grip["gripper_state"],
            round(float(grip["gripper_cmd_mm"]), 6),
            round(float(arm_age), 6) if not math.isnan(arm_age) else "nan",
            round(float(rb_age), 6) if not math.isnan(rb_age) else "nan",
        ]
        self.ws.append(row)

    def _loop(self):
        dt = 1.0 / LOG_HZ
        next_t = time.time()
        sample_index = 0
        while not self.stop_evt.is_set():
            try:
                self._append_row(sample_index)
                sample_index += 1
            except Exception as e:
                print("[logger] row error:", e)
            next_t += dt
            sleep_t = next_t - time.time()
            if sleep_t > 0:
                time.sleep(sleep_t)
            else:
                next_t = time.time()


# ============================================================
# Stage：收到 close 後只做夾爪位置開/關切換
# ============================================================

class StageController:
    def __init__(self, rb_rx, arm_node, gripper_toggle, logger, stop_event):
        self.rb_rx = rb_rx
        self.arm = arm_node
        self.gripper = gripper_toggle
        self.logger = logger
        self.stop_event = stop_event
        self.started = False
        self.last_close_ts = 0.0

    def _cooldown_ok(self):
        return (time.time() - self.last_close_ts) >= COOLDOWN_SEC

    def handle_close(self, evt):
        if not self._cooldown_ok():
            remain = COOLDOWN_SEC - (time.time() - self.last_close_ts)
            print(f"[Stage] 忽略 close，冷卻中，剩 {remain:.2f}s")
            return
        self.last_close_ts = time.time()

        if not self.started:
            # 第一個 close 只用來啟動教導 / 建立 0 點 / 開始紀錄，不動作夾爪。
            self.arm.set_ratio(MOVE_RATIO)
            self.arm.set_listening(True)
            self.logger.start()
            if not self.logger.recording:
                print("[Stage] 首次 close：紀錄器未啟動，請確認剛體資料與手臂 feedback TCP pose 都已收到。")
                return
            self.started = True
            print("[Stage] 首次 close：已啟動教導、手臂剛體同步與 30Hz Excel 紀錄；夾爪不動作。")
            return

        state, cmd_mm = self.gripper.toggle()
        print(f"[Stage] close event from {evt.get('src')} -> gripper {state}, cmd={cmd_mm:.1f} mm")

    def loop(self):
        print("[Stage] 等待 close 訊號。第1次 close 只啟動教導/紀錄；第2次 close 關爪；第3次 close 開爪；之後開/關交替。Ctrl+C 結束並儲存 Excel。")
        while not self.stop_event.is_set():
            evt = self.rb_rx.pop_close()
            if evt is not None:
                self.handle_close(evt)
            time.sleep(0.02)


# ============================================================
# ROS2 spin thread
# ============================================================

def ros_spin_thread(executor: MultiThreadedExecutor, stop_event: threading.Event):
    while not stop_event.is_set():
        executor.spin_once(timeout_sec=0.1)


# ============================================================
# main
# ============================================================

def main():
    serial_port = resolve_serial_port()
    try:
        ensure_serial_ok(serial_port)
    except Exception as e:
        print(f"[preflight] serial error: {e}")
        print("[preflight] 請確認 RS485 是否存在：ls -l /dev/serial/by-id/  或  ls /dev/ttyUSB*")
        sys.exit(1)

    _ = setup_minimalmodbus(serial_port)
    gripper = setup_gripper(serial_port)
    gripper_toggle = GripperToggle(gripper)

    rb_rx = UdpRigidBodyReceiver()
    rb_rx.start()

    rclpy.init(args=None)
    send_script_client = SendScriptClient()
    arm_node = ArmController(send_script_client, rb_receiver=rb_rx)

    executor = MultiThreadedExecutor()
    executor.add_node(send_script_client)
    executor.add_node(arm_node)

    stop_event = threading.Event()
    t_ros = threading.Thread(target=ros_spin_thread, args=(executor, stop_event), daemon=True)
    t_ros.start()

    logger = ExcelPoseLogger(arm_node, rb_rx, gripper_toggle)
    stage = StageController(rb_rx, arm_node, gripper_toggle, logger, stop_event)

    try:
        stage.loop()
    except KeyboardInterrupt:
        print("\n[main] 收到 Ctrl+C，準備結束並儲存資料。")
    finally:
        stop_event.set()
        try:
            logger.stop()
        except Exception as e:
            print("[main] logger stop error:", e)
        try:
            rb_rx.stop()
        except Exception:
            pass
        try:
            executor.shutdown()
        except Exception:
            pass
        try:
            send_script_client.destroy_node()
            arm_node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass
        print("Bye.")


if __name__ == "__main__":
    main()
