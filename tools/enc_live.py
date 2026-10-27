"""ดูค่า encoder 4 ล้อสดๆ จาก Mega (เฟิร์มแวร์ humanoi_base_mega) บน PC — ไม่ต้องมี ROS

  python tools\\enc_live.py COM8          หมุนล้อด้วยมือ ดูว่า E ไหนนับ (Ctrl+C ออก)
  พิมพ์ทุก 0.25 วิ: ค่านับสะสม / rpm (คิด 1440 ต่อรอบ) / นับพลาด ของ E1-E4 + โหมดของ Mega
"""
import sys
import time

import serial

CPR = 1440


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else "COM8"
    ser = serial.Serial(port, 115200, timeout=0.1)  # เปิดพอร์ต = Mega รีเซ็ต (relay ปิดตอนบูต)
    print(f"{port}: หมุนล้อด้วยมือ ดูว่า encoder ตัวไหนนับ · Ctrl+C ออก")
    mode = "?"
    hist = []  # (t_ms, counts)
    last_print = 0.0
    try:
        while True:
            line = ser.readline().decode(errors="replace").strip()
            if line.startswith("READY"):
                print(f"\n{line}")
                hist.clear()
            elif line.startswith("MODE:"):
                mode = line[5:]
            elif line.startswith("ENC "):
                p = line.split()
                if len(p) != 10:
                    continue
                v = [int(x) for x in p[1:]]
                hist.append((v[0], v[1:5], v[5:9]))
                hist = [h for h in hist if h[0] >= v[0] - 1000]  # rpm จาก 1 วิล่าสุด
            now = time.monotonic()
            if hist and now - last_print >= 0.25:
                last_print = now
                (t0, c0, _), (t1, c1, e1) = hist[0], hist[-1]
                dt = (t1 - t0) / 1000.0
                cells = []
                for i in range(4):
                    rpm = (c1[i] - c0[i]) / CPR * 60.0 / dt if dt > 0 else 0.0
                    cells.append(f"E{i + 1} {c1[i]:+7d} {rpm:+6.1f}rpm" + (f" พลาด{e1[i]}" if e1[i] else ""))
                print("\r" + "  ".join(cells) + f"  [{mode}]   ", end="", flush=True)
    except KeyboardInterrupt:
        print()
    finally:
        ser.close()


if __name__ == "__main__":
    main()
