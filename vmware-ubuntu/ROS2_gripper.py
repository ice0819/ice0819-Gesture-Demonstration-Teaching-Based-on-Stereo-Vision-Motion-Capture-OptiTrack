# #!/usr/bin/env python3
# # -*- coding: utf-8 -*-
# """
# Robotiq gripper (2F, Hand-E, EPick/AirPick) Modbus-RTU driver

# 原作者: Benoit CASTETS
# 修訂者: ChatGPT 2025-07-10  (加入 EPick 支援)
# 依賴:
#     • minimalmodbus  https://pypi.org/project/MinimalModbus/
# """
# # ───────────────────────────────────────────────────────────────
# import minimalmodbus as mm
# import time

# # 全域通訊設定 ---------------------------------------------------
# mm.BAUDRATE = 115200
# mm.BYTESIZE = 8
# mm.PARITY   = "N"
# mm.STOPBITS = 1
# mm.TIMEOUT  = 0.2
# # ───────────────────────────────────────────────────────────────

# class RobotiqGripper(mm.Instrument):
#     """
#     通用 Robotiq 夾爪 / 真空吸嘴驅動 (Modbus-RTU, USB-to-RS485)
#     預設從站位址(slave address) = 9
#     """

#     def __init__(self, portname:str, slaveaddress:int=9):
#         super().__init__(portname, slaveaddress)
#         self.debug      = True
#         self.mode       = mm.MODE_RTU
#         self.processing = False
#         self.timeOut    = 10          # 動作逾時(秒)

#         self.registerDic = {}
#         self._buildRegisterDic()

#         self.paramDic = {}
#         self.readAll()

#         # 校正用變數
#         self.closemm  = None
#         self.closebit = None
#         self.openmm   = None
#         self.openbit  = None
#         self._aCoef   = None
#         self._bCoef   = None

#     # ══════════════════════════════════════════════════════════
#     #  1. 建立暫存器說明表
#     # ══════════════════════════════════════════════════════════
#     def _buildRegisterDic(self):
#         """產生暫存器對照表 (dict) 供 printInfo() 使用"""
#         # ---------- 輸入位元組 (Status, 2000~2005) ----------
#         self.registerDic.update({
#             "gOBJ": {}, "gSTA": {}, "gGTO": {}, "gACT": {}, "gMOD": {},  # ← NEW
#             "kFLT": {}, "gFLT": {},
#             "gPR": {}, "gPO": {}, "gCU": {}
#         })

#         # gOBJ
#         obj = self.registerDic["gOBJ"]
#         obj[0] = "移動中，尚未偵測到物件"
#         obj[1] = "開 → 遇到物件"
#         obj[2] = "關 → 遇到物件 / 已吸住"
#         obj[3] = "到達目標位置 / 物件脫落"

#         # gSTA
#         sta = self.registerDic["gSTA"]
#         sta[0] = "復位 / 自動釋放中"
#         sta[1] = "啟動進行中"
#         sta[3] = "啟動完成"

#         # gGTO
#         gto = self.registerDic["gGTO"]
#         gto[0] = "停止 / 復位中"
#         gto[1] = "執行 GoTo"

#         # gACT
#         act = self.registerDic["gACT"]
#         act[0] = "尚未啟動"
#         act[1] = "已啟動"

#         # gMOD  (EPick 專用)
#         gmod = self.registerDic["gMOD"]
#         gmod[0] = "自動模式"
#         gmod[1] = "手動模式"

#         # kFLT & gFLT
#         kflt = self.registerDic["kFLT"]
#         gflt = self.registerDic["gFLT"]
#         for i in range(256):
#             kflt[i] = i
#             gflt[i] = i
#         gflt.update({
#             0 : "無錯誤 (藍燈)",
#             5 : "優先級錯誤，須重新啟動",
#             7 : "等待啟動",
#             8 : "溫度過高",
#             9 : "通訊中斷 (>1 s)",
#             10: "欠壓，需復位",
#             11: "自動釋放中",
#             12: "內部錯誤",
#             13: "啟動失敗",
#             14: "過電流",
#             15: "自動釋放完成"
#         })

#         # gPR / gPO / gCU
#         for name in ("gPR", "gPO"):
#             d = self.registerDic[name]
#             for i in range(256):
#                 d[i] = f"{name} = {i}/255"
#         cu = self.registerDic["gCU"]
#         for i in range(256):
#             cu[i] = f"電流 ≈ {i*10} mA"

#         # ---------- 輸出位元組 (Action, 1000~1005) ----------
#         self.registerDic.update({
#             "rARD": {}, "rATR": {}, "rGTO": {}, "rACT": {},
#             "rMOD": {},              # ← NEW
#             "rPR" : {}, "rFR" : {}, "rSP" : {}
#         })
#         self.registerDic["rMOD"][0] = "自動模式(Auto)"
#         self.registerDic["rMOD"][1] = "手動模式(Manual)"
#     # -----------------------------------------------------------------

#     @staticmethod
#     def _intToHex(integer:int, digits:int=2) -> str:
#         """整數轉十六進位字串 (左補零)"""
#         h = hex(integer)[2:]
#         return "0"*(digits-len(h)) + h

#     # ══════════════════════════════════════════════════════════
#     #  2. 狀態讀取
#     # ══════════════════════════════════════════════════════════
#     def readAll(self):
#         """讀取 2000~2005，結果存入 self.paramDic (int)"""
#         self.paramDic.clear()
#         regs = self.read_registers(2000, 6)

#         # ── 2000
#         b0 = bin(regs[0])[2:].zfill(16)[:8]   # 只要低位元組
#         self.paramDic["gOBJ"] = b0[0:2]
#         self.paramDic["gSTA"] = b0[2:4]
#         self.paramDic["gGTO"] = b0[4:6]
#         self.paramDic["gMOD"] = b0[6]         # ← NEW
#         self.paramDic["gACT"] = b0[7]

#         # ── 2002
#         b2 = bin(regs[2])[2:].zfill(16)[:8]
#         self.paramDic["kFLT"] = b2[0:4]
#         self.paramDic["gFLT"] = b2[4:]

#         # ── 2003
#         b3 = bin(regs[3])[2:].zfill(8)
#         self.paramDic["gPR"] = b3

#         # ── 2004
#         b4 = bin(regs[4])[2:].zfill(16)[:8]
#         self.paramDic["gPO"] = b4

#         # ── 2005
#         b5 = bin(regs[5])[2:].zfill(16)[:8]
#         self.paramDic["gCU"] = b5

#         # 轉成 int
#         for k, v in self.paramDic.items():
#             self.paramDic[k] = int(v, 2)

#     # ══════════════════════════════════════════════════════════
#     #  3. 基本控制 (2F/Hand-E, 舊版保持不變)
#     # ══════════════════════════════════════════════════════════
#     def reset(self):
#         """rACT = 0，復位"""
#         self.write_registers(1000, [0, 0, 0])

#     def activate(self):
#         """rACT = 1，啟動夾爪 / 吸嘴"""
#         self.processing = True
#         self.write_registers(1000, [256, 0, 0])  # 0x0100
#         t0 = time.time()
#         while time.time()-t0 < self.timeOut:
#             self.readAll()
#             if self.paramDic["gSTA"] == 3:   # activation completed
#                 print("Activation completed")
#                 break
#             time.sleep(0.05)
#         else:
#             print("Activation timeout")
#         self.processing = False

#     def resetActivate(self):
#         self.reset()
#         self.activate()

#     # ---------- 位置型夾爪 -------------------------------------------------
#     def goTo(self, position:int, speed:int=255, force:int=255):
#         """2F/Hand-E 位置模式"""
#         if not 0 <= position <= 255:
#             print("position 必須 0~255")
#             return
#         cmd = int("00001001"+"00000000", 2)      # rGTO=1,rACT=1
#         self.write_registers(
#             1000,
#             [cmd, position,
#              int(self._intToHex(speed)+self._intToHex(force), 16)]
#         )

#     def closeGripper(self, speed=255, force=255):
#         self.goTo(255, speed, force)

#     def openGripper(self, speed=255, force=255):
#         self.goTo(0, speed, force)
#     # ---------------------------------------------------------------------

#     # ══════════════════════════════════════════════════════════
#     #  4. EPick / AirPick 真空控制 (NEW)
#     # ══════════════════════════════════════════════════════════
#     def vacuum_grip(self, p_max:int=-80, p_min:int=-25, timeout:int=10):
#         """
#         EPick 吸取
#         p_max: 目標最大真空 (負 kPa, 如 -80)
#         p_min: 目標最小真空 (負 kPa, 如 -25)
#         timeout: 動作逾時 (秒)
#         """
#         rMOD = 1                               # 手動模式
#         rPR  = 100 + abs(int(p_max))           # 目標最大真空
#         rFR  = 100 + abs(int(p_min))           # 目標最小真空
#         rSP  = int(timeout * 10)               # 0.1 s/bit
#         cmd  = int("00011001", 2)              # rGTO=1,rACT=1

#         self.write_registers(1000,
#             [cmd, 0, 0, rPR, rSP, rFR])

#     def vacuum_release(self, timeout:int=5):
#         """EPick 釋放 / 回到大氣"""
#         rMOD = 0             # 自動模式
#         rPR  = 0x64          # 100 (= 大氣)
#         cmd  = int("00001001", 2)  # rGTO=1,rACT=1
#         self.write_registers(1000,
#             [cmd, 0, 0, rPR, 0, 0])
#     # ---------------------------------------------------------------------

#     # ══════════════════════════════════════════════════════════
#     #  5. 其他輔助函式
#     # ══════════════════════════════════════════════════════════
#     def printInfo(self):
#         """印出目前暫存器狀態 (文字說明)"""
#         self.readAll()
#         for k, v in self.paramDic.items():
#             if k in self.registerDic and v in self.registerDic[k]:
#                 print(f"{k:4s}: {v}  → {self.registerDic[k][v]}")
#             else:
#                 print(f"{k:4s}: {v}")

# # ───────────────────────────────────────────────────────────────
# #  範例 ------------------------------------------------------------------
# if __name__ == "__main__":
#     gp = RobotiqGripper("/dev/ttyUSB0", 9)

#     # 若尚未啟動
#     gp.resetActivate()

#     # ---------- 位置型夾爪 ----------
#     # gp.closeGripper()
#     # time.sleep(2)
#     # gp.openGripper()

#     # ---------- EPick 真空 ----------
#     gp.vacuum_grip(p_max=-78, p_min=-20, timeout=8)
#     time.sleep(2)
#     gp.readAll()
#     if gp.paramDic["gOBJ"] in (1, 2):
#         print("已吸住！")

#     gp.vacuum_release()
#     gp.printInfo()
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Robotiq gripper (2F, Hand-E, EPick/AirPick) Modbus-RTU driver

原作者: Benoit CASTETS
修訂者: ChatGPT 2025-10-16  (穩定化初始化、加入 mm 相容層)
依賴:
    • minimalmodbus  https://pypi.org/project/MinimalModbus/
"""
# ───────────────────────────────────────────────────────────────
import minimalmodbus as mm
import time

# 全域通訊設定 ---------------------------------------------------
mm.BAUDRATE = 115200
mm.BYTESIZE = 8
mm.PARITY   = "N"
mm.STOPBITS = 1
mm.TIMEOUT  = 0.8   # 放寬，避免剛上電/轉換器延遲造成讀不到
# mm.CLOSE_PORT_AFTER_EACH_CALL = True  # 若介面卡緩衝有問題，可開這行
# ───────────────────────────────────────────────────────────────

class RobotiqGripper(mm.Instrument):
    """
    通用 Robotiq 夾爪 / 真空吸嘴驅動 (Modbus-RTU, USB-to-RS485)
    預設從站位址(slave address) = 9
    """

    # mm ↔ bit 線性映射預設（2F/Hand-E：0=開, 255=關）
    _DEFAULT_OPEN_MM   = 85.0
    _DEFAULT_CLOSE_MM  = 0.0
    _DEFAULT_OPEN_BIT  = 0
    _DEFAULT_CLOSE_BIT = 255

    def __init__(self, portname:str, slaveaddress:int=9):
        super().__init__(portname, slaveaddress)
        self.debug      = False                 # 開著方便除錯
        self.mode       = mm.MODE_RTU
        self.processing = False
        self.timeOut    = 10                   # 動作逾時(秒)

        # 先把 serial 屬性設好（比照全域設定，但再保險一次）
        try:
            self.serial.baudrate = mm.BAUDRATE
            self.serial.bytesize = mm.BYTESIZE
            # 某些 minimalmodbus 版本用 pyserial 常數，也可直接填 "N"
            self.serial.parity   = "N"
            self.serial.stopbits = mm.STOPBITS
            self.serial.timeout  = mm.TIMEOUT
        except Exception:
            pass

        self.registerDic = {}
        self._buildRegisterDic()

        self.paramDic = {}

        # 校正用變數（提供 mm 介面）
        self.openmm   = self._DEFAULT_OPEN_MM
        self.closemm  = self._DEFAULT_CLOSE_MM
        self.openbit  = self._DEFAULT_OPEN_BIT
        self.closebit = self._DEFAULT_CLOSE_BIT
        self._aCoef, self._bCoef = self._compute_ab_from_bounds()

        # ❌ 不要在 __init__ 內就讀暫存器（尚未 activate / serial 也可能未穩）
        # self.readAll()

    # ══════════════════════════════════════════════════════════
    #  1. 建立暫存器說明表
    # ══════════════════════════════════════════════════════════
    def _buildRegisterDic(self):
        """產生暫存器對照表 (dict) 供 printInfo() 使用"""
        # ---------- 輸入位元組 (Status, 2000~2005) ----------
        self.registerDic.update({
            "gOBJ": {}, "gSTA": {}, "gGTO": {}, "gACT": {}, "gMOD": {},
            "kFLT": {}, "gFLT": {},
            "gPR": {}, "gPO": {}, "gCU": {}
        })

        # gOBJ
        obj = self.registerDic["gOBJ"]
        obj[0] = "移動中，尚未偵測到物件"
        obj[1] = "開 → 遇到物件"
        obj[2] = "關 → 遇到物件 / 已吸住"
        obj[3] = "到達目標位置 / 物件脫落"

        # gSTA
        sta = self.registerDic["gSTA"]
        sta[0] = "復位 / 自動釋放中"
        sta[1] = "啟動進行中"
        sta[3] = "啟動完成"

        # gGTO
        gto = self.registerDic["gGTO"]
        gto[0] = "停止 / 復位中"
        gto[1] = "執行 GoTo"

        # gACT
        act = self.registerDic["gACT"]
        act[0] = "尚未啟動"
        act[1] = "已啟動"

        # gMOD  (EPick 專用)
        gmod = self.registerDic["gMOD"]
        gmod[0] = "自動模式"
        gmod[1] = "手動模式"

        # kFLT & gFLT
        kflt = self.registerDic["kFLT"]
        gflt = self.registerDic["gFLT"]
        for i in range(256):
            kflt[i] = i
            gflt[i] = i
        gflt.update({
            0 : "無錯誤 (藍燈)",
            5 : "優先級錯誤，須重新啟動",
            7 : "等待啟動",
            8 : "溫度過高",
            9 : "通訊中斷 (>1 s)",
            10: "欠壓，需復位",
            11: "自動釋放中",
            12: "內部錯誤",
            13: "啟動失敗",
            14: "過電流",
            15: "自動釋放完成"
        })

        # gPR / gPO / gCU
        for name in ("gPR", "gPO"):
            d = self.registerDic[name]
            for i in range(256):
                d[i] = f"{name} = {i}/255"
        cu = self.registerDic["gCU"]
        for i in range(256):
            cu[i] = f"電流 ≈ {i*10} mA"

        # ---------- 輸出位元組 (Action, 1000~1005) ----------
        self.registerDic.update({
            "rARD": {}, "rATR": {}, "rGTO": {}, "rACT": {},
            "rMOD": {},
            "rPR" : {}, "rFR" : {}, "rSP" : {}
        })
        self.registerDic["rMOD"][0] = "自動模式(Auto)"
        self.registerDic["rMOD"][1] = "手動模式(Manual)"
    # -----------------------------------------------------------------

    @staticmethod
    def _intToHex(integer:int, digits:int=2) -> str:
        """整數轉十六進位字串 (左補零)"""
        h = hex(integer)[2:]
        return "0"*(digits-len(h)) + h

    # ══════════════════════════════════════════════════════════
    #  2. 狀態讀取
    # ══════════════════════════════════════════════════════════
    def readAll(self):
        """讀取 2000~2005，結果存入 self.paramDic (int)"""
        self.paramDic.clear()
        regs = self.read_registers(2000, 6)

        # ── 2000
        b0 = bin(regs[0])[2:].zfill(16)[:8]   # 只要低位元組
        self.paramDic["gOBJ"] = int(b0[0:2], 2)
        self.paramDic["gSTA"] = int(b0[2:4], 2)
        self.paramDic["gGTO"] = int(b0[4:6], 2)
        self.paramDic["gMOD"] = int(b0[6], 2)
        self.paramDic["gACT"] = int(b0[7], 2)

        # ── 2002
        b2 = bin(regs[2])[2:].zfill(16)[:8]
        self.paramDic["kFLT"] = int(b2[0:4], 2)
        self.paramDic["gFLT"] = int(b2[4:], 2)

        # ── 2003, 2004, 2005
        self.paramDic["gPR"] = int(bin(regs[3])[2:].zfill(8), 2)
        self.paramDic["gPO"] = int(bin(regs[4])[2:].zfill(16)[:8], 2)
        self.paramDic["gCU"] = int(bin(regs[5])[2:].zfill(16)[:8], 2)

    # ══════════════════════════════════════════════════════════
    #  3. 基本控制 (2F/Hand-E)
    # ══════════════════════════════════════════════════════════
    def reset(self):
        """rACT = 0，復位"""
        self.write_registers(1000, [0, 0, 0])

    def activate(self):
        """rACT = 1，啟動夾爪 / 吸嘴"""
        self.processing = True
        self.write_registers(1000, [256, 0, 0])  # 0x0100
        t0 = time.time()
        while time.time()-t0 < self.timeOut:
            try:
                self.readAll()
                if self.paramDic.get("gSTA") == 3:   # activation completed
                    print("Activation completed")
                    break
            except Exception:
                pass
            time.sleep(0.05)
        else:
            print("Activation timeout")
        self.processing = False

    def resetActivate(self):
        self.reset()
        self.activate()

    # ---------- 位置型夾爪 -------------------------------------------------
    def goTo(self, position:int, speed:int=255, force:int=255):
        """2F/Hand-E 位置模式（bit 0~255; 0=開, 255=關）"""
        if not 0 <= position <= 255:
            print("position 必須 0~255")
            return
        cmd = int("00001001"+"00000000", 2)      # rGTO=1,rACT=1
        self.write_registers(
            1000,
            [cmd, int(position),
             int(self._intToHex(speed)+self._intToHex(force), 16)]
        )

    def closeGripper(self, speed=255, force=255):
        self.goTo(self._DEFAULT_CLOSE_BIT, speed, force)

    def openGripper(self, speed=255, force=255):
        self.goTo(self._DEFAULT_OPEN_BIT, speed, force)
    # ---------------------------------------------------------------------

    # ══════════════════════════════════════════════════════════
    #  4. EPick / AirPick 真空控制 (NEW)
    # ══════════════════════════════════════════════════════════
    def vacuum_grip(self, p_max:int=-80, p_min:int=-25, timeout:int=10):
        """
        EPick 吸取
        p_max: 目標最大真空 (負 kPa, 如 -80)
        p_min: 目標最小真空 (負 kPa, 如 -25)
        timeout: 動作逾時 (秒)
        """
        rPR  = 100 + abs(int(p_max))           # 目標最大真空
        rFR  = 100 + abs(int(p_min))           # 目標最小真空
        rSP  = int(timeout * 10)               # 0.1 s/bit
        cmd  = int("00011001", 2)              # rGTO=1,rACT=1
        self.write_registers(1000, [cmd, 0, 0, rPR, rSP, rFR])

    def vacuum_release(self, timeout:int=5):
        """EPick 釋放 / 回到大氣"""
        rPR  = 0x64          # 100 (= 大氣)
        cmd  = int("00001001", 2)  # rGTO=1,rACT=1
        self.write_registers(1000, [cmd, 0, 0, rPR, 0, 0])
    # ---------------------------------------------------------------------

    # ══════════════════════════════════════════════════════════
    #  5. mm 相容層（配合你的第一個程式）
    # ══════════════════════════════════════════════════════════
    def setCalibration(self, openmm:float, closemm:float,
                       openbit:int=0, closebit:int=255):
        """設定行程端點與對應 bit，並重算線性係數"""
        self.openmm, self.closemm = float(openmm), float(closemm)
        self.openbit, self.closebit = int(openbit), int(closebit)
        self._aCoef, self._bCoef = self._compute_ab_from_bounds()

    def _compute_ab_from_bounds(self):
        """
        從 (openmm,openbit) 與 (closemm,closebit) 計算線性映射
        bit = a*mm + b
        """
        x1, y1 = self.openmm,  self.openbit
        x2, y2 = self.closemm, self.closebit
        if abs(x2-x1) < 1e-6:
            a = -3.0
            b = self.openbit - a*self.openmm
        else:
            a = (y2 - y1) / (x2 - x1)
            b = y1 - a * x1
        return float(a), float(b)

    def _mmToBit(self, mm_val:float) -> int:
        if self._aCoef is None or self._bCoef is None:
            self._aCoef, self._bCoef = self._compute_ab_from_bounds()
        bit = self._aCoef * float(mm_val) + self._bCoef
        return int(max(0, min(255, round(bit))))

    def _bitToMm(self, bit_val:int) -> float:
        if self._aCoef is None or abs(self._aCoef) < 1e-9:
            self._aCoef, self._bCoef = self._compute_ab_from_bounds()
        return float((int(bit_val) - self._bCoef) / self._aCoef)

    def goTomm(self, mm_val:float, speed:int=255, force:int=255):
        """以 mm 下達位置命令，內部轉成 0~255 bit"""
        mm_val = max(min(mm_val, self.openmm), self.closemm)
        self.goTo(self._mmToBit(mm_val), speed, force)

    def getPositionmm(self) -> float:
        """讀回 gPO (bit) 並換算成 mm；若讀不到回 NaN"""
        try:
            self.readAll()
            bit = int(self.paramDic.get("gPO", 0))
            return self._bitToMm(bit)
        except Exception:
            try:
                # 退而求其次，用 gPR（命令位置）估計
                self.readAll()
                bit = int(self.paramDic.get("gPR", 0))
                return self._bitToMm(bit)
            except Exception:
                return float("nan")

    # ══════════════════════════════════════════════════════════
    #  6. 其他輔助函式
    # ══════════════════════════════════════════════════════════
    def printInfo(self):
        """印出目前暫存器狀態 (文字說明)"""
        self.readAll()
        for k, v in self.paramDic.items():
            if k in self.registerDic and v in self.registerDic[k]:
                print(f"{k:4s}: {v}  → {self.registerDic[k][v]}")
            else:
                print(f"{k:4s}: {v}")

    def ping(self, retries:int=3, delay:float=0.2) -> bool:
        """簡單讀 2000..2005 作為連線測試"""
        for _ in range(retries):
            try:
                _ = self.read_registers(2000, 6)
                return True
            except Exception:
                time.sleep(delay)
        return False

# ───────────────────────────────────────────────────────────────
#  範例 ------------------------------------------------------------------
if __name__ == "__main__":
    gp = RobotiqGripper("/dev/ttyUSB0", 9)
    gp.resetActivate()
    gp.setCalibration(openmm=85, closemm=0, openbit=0, closebit=255)
    gp.goTomm(20, 255, 100)
    time.sleep(1.0)
    print("POS(mm)=", gp.getPositionmm())
    # gp.printInfo()
