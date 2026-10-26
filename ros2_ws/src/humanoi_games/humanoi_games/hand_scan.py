#!/usr/bin/env python3
"""hand_scan — เช็กบอร์ดมือทุกตัวที่เสียบอยู่: servo ID ไหนตอบ ที่ baud ไหน ไฟเท่าไหร่ (อ่านอย่างเดียว ไม่ขยับ)

  arm hand-scan        (บน Pi — ปิด arm player / arm rps / hand-teach / hand-calibrate ก่อน: ใช้พอร์ตมือเดียวกัน)

แทน tools\\scan_servos.py บน Windows · บอร์ด = Waveshare Bus Servo Adapter (CH343 1a86:55d3)
"""
import glob
import os
import sys

from humanoi_games.amazinghand_node import find_ports, scalar

IDS = list(range(1, 9)) + list(range(11, 19))  # ชุดมือขวา 1-8 · เผื่อชุดที่ตั้งเป็น 11-18
BAUDS = (1_000_000, 500_000, 250_000, 115_200)
VOLT_MIN, VOLT_MAX = 4.8, 7.4


def main():
    from rustypot import Scs0009PyController
    from serial.tools import list_ports
    serials = {p.device: p.serial_number for p in list_ports.comports()}
    ports = find_ports()
    if not ports:
        print("✗ ไม่เจอบอร์ดมือเลย (Waveshare CH343 1a86:55d3) — เสียบ USB ของบอร์ดมือ")
        return 1
    ok_any = False
    for port in ports:
        byid = [os.path.basename(p) for p in glob.glob("/dev/serial/by-id/*") if os.path.realpath(p) == port]
        print(f"บอร์ด {port}  serial {serials.get(port, '?')}  {byid[0] if byid else ''}")
        try:
            c = Scs0009PyController(serial_port=port, baudrate=BAUDS[0], timeout=0.02)
        except Exception as exc:
            print(f"  ✗ เปิดพอร์ตไม่ได้ ({exc}) — มีโปรแกรมอื่นใช้อยู่? ปิด arm player / hand-teach ก่อน")
            continue
        found_any = False
        for baud in BAUDS:
            c.set_baudrate(baud)
            found = []
            for sid in IDS:
                try:
                    volts = scalar(c.read_present_voltage(sid)) / 10
                except Exception:
                    continue
                flag = "" if VOLT_MIN <= volts <= VOLT_MAX else " ⚠"
                found.append(f"ID{sid} {volts:.1f}V{flag}")
            if found:
                found_any = ok_any = True
                print(f"  ✓ {baud} baud: {len(found)} ตัว — " + ", ".join(found))
        if not found_any:
            print("  ✗ ไม่มีเซอร์โวตอบ — ไฟมือปิด / สายเซอร์โวไม่ได้ต่อ (ช่อง D V G) / jumper ไม่อยู่ที่ B (USB)")
        del c
    print(f"ไฟที่ปลอดภัย {VOLT_MIN}-{VOLT_MAX} V (⚠ = นอกช่วง ห้ามสั่งขยับ) · มือที่ใช้จริง: โปรแกรมเลือกบอร์ดที่มีเซอร์โวตอบเอง")
    return 0 if ok_any else 1


if __name__ == "__main__":
    sys.exit(main())
