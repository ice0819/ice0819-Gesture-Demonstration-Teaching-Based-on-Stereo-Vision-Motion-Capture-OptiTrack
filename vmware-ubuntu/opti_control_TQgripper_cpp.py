#!/usr/bin/env python3
import math
import socket
import time
import struct
import threading
import sys
import os
import curses

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import PoseStamped
from tm_msgs.msg import FeedbackState
from tm_msgs.srv import SendScript

# ---- Gripper deps ----
import serial
import minimalmodbus
import ROS2_gripper as rq
 
# ==============================
# 通用設定
# ==============================
# --- Gripper 固定步進 ---
GRIPPER_STEP = 10           # 固定 30（不允許用 +/- 改）
GRIPPER_SPEED = 128         # 0..255
GRIPPER_FORCE = 170         # 0..255
POS_MIN, POS_MAX = 0, 255   # Robotiq: 0=open, 255=closed
PORT = '/dev/ttyUSB0'
SLAVE_ID = 9

# --- UDP 目標（Windows 機器）---
UDP_IP = '192.168.250.20'
UDP_PORT = 8888

# ==============================
# 工具：四元數 → Euler(sxyz: roll,pitch,yaw)
# ==============================
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

# ==============================
# Gripper（序列埠 + Robotiq 包裝）
# ==============================
def ensure_serial_ok():
    s = serial.Serial(PORT, 115200, timeout=1)
    s.close()

def setup_minimalmodbus():
    ins = minimalmodbus.Instrument(PORT, SLAVE_ID, debug=False)
    ins.mode = minimalmodbus.MODE_RTU
    ins.serial.baudrate = 115200
    ins.serial.bytesize = 8
    ins.serial.parity   = serial.PARITY_NONE
    ins.serial.stopbits = 1
    ins.serial.timeout  = 0.2
    ins.clear_buffers_before_each_transaction = True
    ins.close_port_after_each_call = True
    return ins

def setup_gripper():
    g = rq.RobotiqGripper(portname=PORT, slaveaddress=SLAVE_ID)
    # 安全起見：先 Reset + Activate
    g.resetActivate()
    time.sleep(0.3)
    return g

def send_gripper_position(g, pos):
    pos = max(POS_MIN, min(POS_MAX, int(pos)))
    if hasattr(g, 'goTomm'):
        g.goTomm(pos, GRIPPER_SPEED, GRIPPER_FORCE)
    elif hasattr(g, 'goto'):
        g.goto(pos, GRIPPER_SPEED, GRIPPER_FORCE)
    elif hasattr(g, 'goTo'):
        g.goTo(pos, GRIPPER_SPEED, GRIPPER_FORCE)
    else:
        raise AttributeError("RobotiqGripper has no goTomm/goto/goTo method")
    return pos

# ==============================
# ROS2：SendScript 客戶端 + Arm 控制
# ==============================
class SendScriptClient(Node):
    def __init__(self):
        super().__init__('send_script_client')
        self.cli = self.create_client(SendScript, 'send_script')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待 send_script 服務...')
        self.req = SendScript.Request()

    def send_cmd(self, cmd: str):
        self.req.id = 'demo'
        self.req.script = cmd
        return self.cli.call_async(self.req)

class JointStateBroadcaster:
    def __init__(self, ip=UDP_IP, port=UDP_PORT):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.target_ip = ip
        self.port = port

    def send(self, joint_angles):
        data = struct.pack('6f', *joint_angles[:6])
        self.sock.sendto(data, (self.target_ip, self.port))

class ArmController(Node):
    def __init__(self, client: SendScriptClient):
        super().__init__('arm_controller_with_feedback')
        self.client = client
        self.joint_sender = JointStateBroadcaster()
        self.last_udp_send_time = 0.0

        # 初始參考位置
        self.initial_arm_pose = [400, 200, 450, -180, 0, 90]
        self.initial_rbody_pos = None
        self.current_rbody_pos = None
        self.current_rbody_ori = None
        self.last_future = None
        self.ready_to_move = False

        # 是否啟用「聽取點位」（由 UI 空白鍵切換）
        self.listening_enabled = False
        self.initial_recorded = False

        # 訂閱 RigidBody 位姿與 FeedbackState
        self.create_subscription(PoseStamped, '/RigidBody001/pose', self.pose_callback, 15)
        self.create_subscription(FeedbackState, '/feedback_states', self.feedback_callback, 15)

        # 送出第一個 PTP 命令
        init_cmd = 'PTP("CPP",400,200,450,-180,0,90,50,200,0,false,0,2,4)'
        self.last_future = self.client.send_cmd(init_cmd)
        self.get_logger().info(f'已傳送初始命令: {init_cmd}')

        self.last_send_time = 0.0
        self.min_send_interval = 0.25
        self.create_timer(0.1, lambda: None)

    # 由 UI 呼叫：切換聽取
    def set_listening(self, enabled: bool):
        self.listening_enabled = enabled
        if enabled and not self.initial_recorded and self.current_rbody_pos:
            self.initial_rbody_pos = self.current_rbody_pos
            self.initial_recorded = True
            self.get_logger().info(f'[聽取啟用] 設定初始剛體位置: {self.initial_rbody_pos}')

    def pose_callback(self, msg: PoseStamped):
        self.current_rbody_pos = (msg.pose.position.x,
                                  msg.pose.position.y,
                                  msg.pose.position.z)
        self.current_rbody_ori = (msg.pose.orientation.x,
                                  msg.pose.orientation.y,
                                  msg.pose.orientation.z,
                                  msg.pose.orientation.w)

    def feedback_callback(self, msg: FeedbackState):
        now = time.time()
        if len(msg.joint_pos) >= 6 and (now - self.last_udp_send_time) > 0.5:
            self.joint_sender.send(msg.joint_pos)
            self.last_udp_send_time = now

        moving_joints = any(abs(v) > 1000 for v in msg.joint_vel)
        tcp_lin = msg.tcp_speed[:3]
        tcp_ang = msg.tcp_speed[3:]
        moving_tcp = any(abs(v) > 1e-2 for v in (tcp_lin + tcp_ang))

        status = 'moving' if (moving_joints or moving_tcp) else ('project_idle' if msg.project_run else 'stopped')
        if status == 'project_idle' and self.listening_enabled:
            self.ready_to_move = True
            self._send_periodic_move()

    def _send_periodic_move(self):
        now = time.time()
        if now - self.last_send_time < self.min_send_interval:
            return
        if not (self.ready_to_move and self.initial_rbody_pos and self.current_rbody_pos and self.current_rbody_ori):
            return
 
        dx = self.current_rbody_pos[0] - self.initial_rbody_pos[0]
        dy = self.current_rbody_pos[1] - self.initial_rbody_pos[1]
        dz = self.current_rbody_pos[2] - self.initial_rbody_pos[2]
        ratio = 1.5
        tx = self.initial_arm_pose[0] + math.floor(dx/ratio)
        ty = self.initial_arm_pose[1] + math.floor(dy/ratio)
        tz = self.initial_arm_pose[2] + math.floor(dz/ratio)

        qx, qy, qz, qw = self.current_rbody_ori
        roll, pitch, yaw = quat_to_euler_sxyz(qx, qy, qz, qw)

        cmd = (f'PTP("CPP",{tx},{ty},{tz},'
               f'{math.degrees(roll):.1f},{math.degrees(pitch):.1f},{math.degrees(yaw):.1f},'
               '100,100,100,true,0,2,4)')

        if self.last_future and not self.last_future.done():
            self.last_future.cancel()

        self.last_future = self.client.send_cmd(cmd)
        # self.get_logger().info(f'送出命令: {cmd}')
        self.ready_to_move = False
        self.last_send_time = now

# ==============================
# ROS2 執行緒
# ==============================
def ros_spin_thread(executor: MultiThreadedExecutor, stop_event: threading.Event):
    # 簡易輪詢，支援外部 stop_event
    while not stop_event.is_set():
        executor.spin_once(timeout_sec=0.1)

# ==============================
# curses UI
# ==============================
def ui_loop(stdscr, arm_node: ArmController, gripper):
    curses.curs_set(0)
    stdscr.nodelay(True)
    stdscr.timeout(100)

    # 夾爪初始位置：先全開
    pos = send_gripper_position(gripper, POS_MIN)
    listening = False

    last_msg = "ready"
    while True:
        stdscr.erase()
        stdscr.addstr(0, 0, "Arm + Robotiq Controller (q=quit, SPACE=toggle listen, Up/Down=gripper)")
        stdscr.addstr(1, 0, f"UDP: {UDP_IP}:{UDP_PORT}")
        stdscr.addstr(2, 0, f"Serial: {PORT}  (slave {SLAVE_ID})")
        stdscr.addstr(3, 0, f"Gripper step: {GRIPPER_STEP}  Speed: {GRIPPER_SPEED}  Force: {GRIPPER_FORCE}")
        stdscr.addstr(5, 0, "SPACE : start/stop listening rigid-body to drive the arm")
        stdscr.addstr(6, 0, "Up    : gripper close (pos + 30)")
        stdscr.addstr(7, 0, "Down  : gripper open  (pos - 30)")
        stdscr.addstr(8, 0, "q     : quit ALL")
        stdscr.addstr(10, 0, f"Gripper pos: {pos:3d}   Listening: {'ON' if listening else 'OFF'}")
        stdscr.addstr(12, 0, f"Status: {last_msg}        ")
        stdscr.refresh()

        key = stdscr.getch()
        if key == -1:
            continue

        if key in (ord('q'), ord('Q')):
            last_msg = "quit requested"
            stdscr.addstr(14, 0, "Exiting...")
            stdscr.refresh()
            time.sleep(0.05)
            break
        elif key == ord(' '):  # toggle listening
            listening = not listening
            arm_node.set_listening(listening)
            last_msg = f"listening -> {'ON' if listening else 'OFF'}"
        elif key == curses.KEY_UP:
            new_pos = min(POS_MAX, pos + GRIPPER_STEP)
            if new_pos != pos:
                pos = send_gripper_position(gripper, new_pos)
                last_msg = f"gripper close -> {pos}"
        elif key == curses.KEY_DOWN:
            new_pos = max(POS_MIN, pos - GRIPPER_STEP)
            if new_pos != pos:
                pos = send_gripper_position(gripper, new_pos)
                last_msg = f"gripper open -> {pos}"
        else:
            pass

# ==============================
# main
# ==============================
def main():
    # --- Gripper preflight ---
    try:
        ensure_serial_ok()
    except Exception as e:
        print(f"[preflight] cannot open serial port: {e}")
        sys.exit(1)
    _ = setup_minimalmodbus()
    gripper = setup_gripper()

    # --- ROS2 啟動 ---
    rclpy.init(args=None)
    send_script_client = SendScriptClient()
    arm_node = ArmController(send_script_client)

    executor = MultiThreadedExecutor()
    executor.add_node(send_script_client)
    executor.add_node(arm_node)

    stop_event = threading.Event()
    t = threading.Thread(target=ros_spin_thread, args=(executor, stop_event), daemon=True)
    t.start()

    # --- curses UI（阻塞，直到按 q）---
    try:
        curses.wrapper(ui_loop, arm_node, gripper)
    except KeyboardInterrupt:
        pass
    finally:
        # 結束所有程式
        stop_event.set()
        # 稍微等一下 spin_once loop 收尾
        time.sleep(0.1)
        try:
            executor.shutdown()
        except Exception:
            pass
        try:
            send_script_client.destroy_node()
            arm_node.destroy_node()
        except Exception:
            pass
        rclpy.shutdown()
        print("Bye.")

if __name__ == '__main__':
    main()
