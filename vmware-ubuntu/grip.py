#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import minimalmodbus, serial, time

def main():
    inst = minimalmodbus.Instrument("/dev/ttyUSB0", 9)
    inst.serial.baudrate = 115200
    inst.serial.bytesize = 8
    inst.serial.parity   = serial.PARITY_NONE
    inst.serial.stopbits = 1
    inst.serial.timeout  = 0.5
    inst.mode            = minimalmodbus.MODE_RTU
    inst.debug           = False

    # 1) Activate → 0x0100
    inst.write_registers(1000, [0x0100, 0x0000, 0x0000])
    print("Gripper activated")
    time.sleep(1.0)

    speed = 200
    force = 150
    sf    = (speed << 8) | force  # high byte: speed, low byte: force

    while True:
        cmd = input("Enter (o=open, c=close, q=quit): ").strip().lower()
        if cmd == 'o':
            # Open → GoTo(0) with Control=0x0900
            inst.write_registers(1000, [0x0900, 0x0000, sf])
            print("Gripper opening...")
        elif cmd == 'c':
            # Close → GoTo(255) with Control=0x0900
            inst.write_registers(1000, [0x0900, 0x00FF, sf])
            print("Gripper closing...")
        elif cmd == 'q':
            print("Bye")
            break
        else:
            print("Unknown, please enter o, c, or q.")

if __name__ == "__main__":
    main()
