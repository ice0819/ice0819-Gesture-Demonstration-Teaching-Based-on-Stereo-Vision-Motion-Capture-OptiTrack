#!/usr/bin/env python3
import time
import minimalmodbus, serial
import ROS2_gripper as rq

PORT = '/dev/ttyUSB0'
SLAVE_ID = 9

# 先用 pyserial 確認可開
serial.Serial(PORT, 115200, timeout=1).close()

# MinimalModbus（若 ROS2_gripper 內部也用，不衝突）
ins = minimalmodbus.Instrument(PORT, SLAVE_ID, debug=False)
ins.mode = minimalmodbus.MODE_RTU
ins.serial.baudrate = 115200
ins.serial.bytesize = 8
ins.serial.parity   = serial.PARITY_NONE
ins.serial.stopbits = 1
ins.serial.timeout  = 0.2
ins.clear_buffers_before_each_transaction = True
ins.close_port_after_each_call = True

# Robotiq 物件
g = rq.RobotiqGripper(portname=PORT, slaveaddress=SLAVE_ID)

print("Initializing gripper...")
g.resetActivate()        # 啟動完成即可

# ===== 沒有 calibrate，就直接動作 =====
time.sleep(0.3)

# 方案 A：如果有 goTomm（你剛剛用過，可能存在）
if hasattr(g, 'goTomm'):
    # 例：行程=「完全閉合」(255), 速度=128, 力量=255
    g.goTomm(255, 128, 255)
    time.sleep(1.0)
    g.goTomm(0,   128, 128)   # 完全打開
else:
    # 方案 B：常見命名 goto / goTo（擇一）
    pos_close, pos_open = 255, 0      # 0=全開, 255=全閉（Robotiq 通常是這樣）
    sp, fr = 128, 128
    if hasattr(g, 'goto'):
        g.goto(pos_close, sp, fr); time.sleep(1.0)
        g.goto(pos_open,  sp, fr)
    elif hasattr(g, 'goTo'):
        g.goTo(pos_close, sp, fr); time.sleep(1.0)
        g.goTo(pos_open,  sp, fr)
    else:
        print("找不到 goTomm/goto/goTo，請看下方『如何查可用方法』")
print("Done.")
