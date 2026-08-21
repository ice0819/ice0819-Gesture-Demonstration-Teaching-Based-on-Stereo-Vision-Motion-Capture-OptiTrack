#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
robotiq_test.py
互動式測試腳本 ─ 2F/Hand-E 位置夾爪 & EPick/AirPick 真空吸嘴
--------------------------------------------------------------
依賴:
    • minimalmodbus
    • ROS2_gripper.py   ← 上一步驟整合好的驅動檔

使用方式:
    $ python3 robotiq_test.py
"""

import time
import minimalmodbus            # 僅確認連線是否正常
import ROS2_gripper as rq       # ← 若檔名不同請自行修改

# ======= 使用者可修改的設定 =====================================
DEVICE_PATH   = "/dev/ttyUSB1"  # USB-RS485 轉接器裝置檔
SLAVE_ID      = 9               # 出廠預設 9
VAC_P_MAX     = -78             # 真空吸取目標最大真空 (kPa, 負值)
VAC_P_MIN     = -20             # 真空吸取目標最小真空 (kPa, 負值)
VAC_TIMEOUT   = 8               # 真空達壓逾時 (秒)
# ==============================================================

def main():
    # ── 基本連線測試 (非必要，但有助於早期偵錯) ───────────────
    try:
        test_inst = minimalmodbus.Instrument(DEVICE_PATH, SLAVE_ID)
        test_inst.serial.baudrate = 115200
        test_inst.serial.parity   = minimalmodbus.serial.PARITY_NONE
        test_inst.serial.stopbits = 1
        test_inst.serial.timeout  = 0.2
        _ = test_inst.read_register(2000)      # 嘗試讀取狀態
        print("✅  Modbus 連線檢查 OK")
    except Exception as e:
        print("❌  無法與裝置通訊，請確認連線：", e)
        return
    # -----------------------------------------------------------

    # ── 建立 Robotiq 物件 ───────────────────────────────────────
    gripper = rq.RobotiqGripper(DEVICE_PATH, SLAVE_ID)
    print("Initializing / Reset-Activate...")
    gripper.resetActivate()

    # ── 進入互動迴圈 ───────────────────────────────────────────
    print(
        "\n指令：\n"
        "  [o] Open 夾爪\n"
        "  [c] Close 夾爪\n"
        "  [g] Grip  真空吸取\n"
        "  [r] Release 放開／洩壓\n"
        "  [q] Quit  離開\n"
    )

    while True:
        cmd = input("請輸入指令 o/c/g/r/q → ").strip().lower()

        if cmd == "o":
            print("→ 開夾中...")
            gripper.openGripper(speed=128, force=255)
            time.sleep(1)
            gripper.printInfo()

        elif cmd == "c":
            print("→ 關夾中...")
            gripper.closeGripper(speed=128, force=255)
            time.sleep(1)
            gripper.printInfo()

        elif cmd == "g":
            print(f"→ 真空吸取 (Pmax={VAC_P_MAX} kPa, Pmin={VAC_P_MIN} kPa)")
            gripper.vacuum_grip(
                p_max=VAC_P_MAX,
                p_min=VAC_P_MIN,
                timeout=VAC_TIMEOUT
            )
            time.sleep(1)
            gripper.printInfo()

        elif cmd == "r":
            print("→ 放開／洩壓...")
            gripper.vacuum_release()
            time.sleep(1)
            gripper.printInfo()

        elif cmd == "q":
            print("Bye!")
            break

        else:
            print("⚠️  指令無效，請重新輸入。")

if __name__ == "__main__":
    main()
