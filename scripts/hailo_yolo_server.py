#!/usr/bin/env python3
"""ตัวคิด YOLO บนการ์ด Hailo-10H (AI HAT) — รันบนเครื่อง Pi ตรงๆ นอก container

  python3 ~/Humanoi/scripts/hailo_yolo_server.py
  python3 ~/Humanoi/scripts/hailo_yolo_server.py --hef /usr/share/hailo-models/yolov8m_h10.hef

แล้วใน container รัน  ros2 launch humanoi_vision yolo.launch.py  (yolo_node ต่อเข้ามาที่ 127.0.0.1:5555)

ทำไมรันนอก container: HailoRT ต้องเวอร์ชันตรงกับเฟิร์มแวร์ของการ์ด (5.1.1) ซึ่งลงไว้บน Raspberry Pi OS แล้ว
(apt: h10-hailort, python3-h10-hailort) ส่วน container เป็น Ubuntu 24.04 ไม่มีแพ็กเกจนี้ — container ใช้
network แบบ host อยู่แล้ว จึงต่อ 127.0.0.1 ถึงกันตรงๆ ใช้ /usr/bin/python3 ของระบบ ไม่ต้องมี venv

ถือการ์ดเฉพาะตอนมี client ต่ออยู่ (เปิดตอนต่อเข้ามา คืนตอนตัด) — ระหว่างไม่มี yolo_node
โปรแกรม Hailo อื่น (hailo-apps ฯลฯ) ใช้การ์ดได้ตามปกติ เพราะ HailoRT ให้เปิดการ์ดได้ทีละโปรแกรม

โปรโตคอล (TCP, รับทีละ client):
  1) client ต่อเข้ามา -> server เปิดการ์ดแล้วส่ง JSON 1 บรรทัด {"input": [h, w, c], "classes": 80, "hef": "..."}
     ถ้าเปิดการ์ดไม่ได้ (โปรแกรมอื่นถืออยู่) ส่ง {"error": "..."} แล้วตัดการเชื่อมต่อ
  2) client ส่งภาพ RGB uint8 ขนาด h*w*c ไบต์พอดี (ย่อ + เติมขอบ letterbox มาแล้ว)
  3) server ตอบ JSON 1 บรรทัด {"dets": [[class_id, score, ymin, xmin, ymax, xmax], ...], "ms": 26.3}
     พิกัด 0-1 เทียบกับภาพที่ส่งมา — การแปลงกลับเป็นพิกเซลของกล้องเป็นหน้าที่ของ client
  วนข้อ 2-3 ไปเรื่อยๆ จน client ตัดการเชื่อมต่อ

ใช้ได้กับ HEF ตรวจจับวัตถุที่ทำ NMS บนชิป (output แบบ HAILO_NMS_BY_CLASS) เช่น yolov11m_h10 / yolov8m_h10
ใน /usr/share/hailo-models — yolov11m คิด ~26 ms/ภาพ (~38 fps) บน Hailo-10H
"""
import argparse
import json
import signal
import socket
import sys
import time

import numpy as np
from hailo_platform import HEF, FormatType, HailoSchedulingAlgorithm, VDevice

DEFAULT_HEF = "/usr/share/hailo-models/yolov11m_h10.hef"


def recv_exact_into(conn, arr):
    """อ่านจาก socket ให้เต็ม arr พอดี — คืน False ถ้า client ปิดก่อนครบ"""
    view = memoryview(arr).cast("B")
    got = 0
    while got < len(view):
        n = conn.recv_into(view[got:])
        if n == 0:
            return False
        got += n
    return True


def send_line(conn, obj):
    conn.sendall((json.dumps(obj) + "\n").encode())


def serve_client(conn, hef_path, hello):
    """เปิดการ์ด -> คิดภาพของ client นี้จนเขาตัด -> คืนการ์ด

    ถือการ์ดเฉพาะตอนมี yolo_node ต่ออยู่: HailoRT ให้โปรแกรมเดียวเปิดการ์ดได้ (โปรแกรมที่สองได้
    HAILO_OUT_OF_PHYSICAL_DEVICES) ถ้าถือค้างตั้งแต่บูต hailo-apps / งาน Hailo อื่นบน Pi จะใช้การ์ดไม่ได้เลย
    """
    t = time.monotonic()
    params = VDevice.create_params()
    params.scheduling_algorithm = HailoSchedulingAlgorithm.ROUND_ROBIN
    try:
        vdevice = VDevice(params)
    except Exception as e:
        reason = f"เปิดการ์ด Hailo ไม่ได้ — มีโปรแกรมอื่นใช้การ์ดอยู่หรือเปล่า? ({type(e).__name__})"
        print(reason, flush=True)
        send_line(conn, {"error": reason})
        return
    with vdevice:
        model = vdevice.create_infer_model(hef_path)
        model.set_batch_size(1)
        model.input().set_format_type(FormatType.UINT8)
        model.output().set_format_type(FormatType.FLOAT32)
        with model.configure() as configured:
            out_buf = np.empty(model.output().shape, np.float32)
            bindings = configured.create_bindings(output_buffers={model.output().name: out_buf})
            frame = np.empty(model.input().shape, np.uint8)
            print(f"เปิดการ์ดแล้ว ({time.monotonic() - t:.1f} s)", flush=True)
            send_line(conn, hello)

            n, busy, t0 = 0, 0.0, time.monotonic()
            while recv_exact_into(conn, frame):
                t = time.monotonic()
                bindings.input().set_buffer(frame)
                configured.run([bindings], 10000)
                dets = [[cls, round(float(score), 4), round(float(y1), 5), round(float(x1), 5),
                         round(float(y2), 5), round(float(x2), 5)]
                        for cls, boxes in enumerate(bindings.output().get_buffer())
                        for y1, x1, y2, x2, score in boxes]
                ms = 1000 * (time.monotonic() - t)
                send_line(conn, {"dets": dets, "ms": round(ms, 1)})
                n, busy = n + 1, busy + ms
                if time.monotonic() - t0 >= 10.0:
                    print(f"{n / (time.monotonic() - t0):.1f} ภาพ/วินาที (Hailo {busy / n:.1f} ms/ภาพ)", flush=True)
                    n, busy, t0 = 0, 0.0, time.monotonic()


def main():
    ap = argparse.ArgumentParser(description="YOLO บน Hailo-10H ให้ yolo_node ใน container เรียกใช้")
    ap.add_argument("--hef", default=DEFAULT_HEF)
    ap.add_argument("--host", default="127.0.0.1", help="127.0.0.1 = รับเฉพาะในเครื่อง Pi เอง")
    ap.add_argument("--port", type=int, default=5555)
    args = ap.parse_args()

    # ตรวจโมเดลจากไฟล์ HEF ตอนเริ่ม (ไม่ต้องเปิดการ์ด)
    hef = HEF(args.hef)
    out_infos = hef.get_output_vstream_infos()
    if len(out_infos) != 1 or not str(out_infos[0].format.order).endswith("HAILO_NMS_BY_CLASS"):
        sys.exit(f"{args.hef}: ต้องเป็นโมเดลตรวจจับที่มี NMS บนชิป (HAILO_NMS_BY_CLASS) "
                 f"แต่ได้ {[str(o.format.order) for o in out_infos]}")
    h, w, c = hef.get_input_vstream_infos()[0].shape
    hello = {"input": [h, w, c], "classes": out_infos[0].shape[0], "hef": args.hef}

    srv = socket.create_server((args.host, args.port))
    print(f"พร้อม: {args.hef} (input {w}x{h}, {hello['classes']} คลาส) รอที่ {args.host}:{args.port} "
          "— เปิดการ์ดเฉพาะตอนมี client", flush=True)
    while True:
        conn, addr = srv.accept()
        conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        print(f"client ต่อเข้ามา {addr}", flush=True)
        try:
            serve_client(conn, args.hef, hello)
        except OSError as e:
            print(f"client {addr} หลุด: {e}", flush=True)
        except Exception as e:  # error จาก HailoRT ระหว่างคิด — ปิด client นี้ รอตัวใหม่ (การ์ดคืนแล้วจาก with)
            print(f"client {addr}: error จาก Hailo {type(e).__name__}: {e}", flush=True)
        finally:
            conn.close()
        print(f"client {addr} ออกแล้ว — คืนการ์ดแล้ว รอ client ใหม่", flush=True)


def _raise_interrupt(signum, frame):
    raise KeyboardInterrupt


if __name__ == "__main__":
    # ปิดให้เรียบร้อยเสมอ (ออกจาก with ของ VDevice = คืนการ์ดอย่างถูกต้อง):
    # - SIGINT: shell สั่งงานเบื้องหลัง (nohup ... &) จะส่งต่อสถานะ "ignore SIGINT" มา Python เลยไม่ตั้ง
    #   KeyboardInterrupt ให้ — ตั้งเองเพื่อให้ kill -INT / Ctrl+C หยุดได้ทุกกรณี
    # - SIGTERM (kill / systemctl stop): ค่าเริ่มต้นคือตายทันทีโดยไม่คืนการ์ด — แปลงเป็น KeyboardInterrupt แทน
    signal.signal(signal.SIGINT, signal.default_int_handler)
    signal.signal(signal.SIGTERM, _raise_interrupt)
    try:
        main()
    except KeyboardInterrupt:
        print("หยุด server แล้ว", flush=True)
