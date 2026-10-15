#!/usr/bin/env bash
# arm — คำสั่งลัดคุมแขน AR4 บน Raspberry Pi (รันนอก container ได้เลย ไม่ต้อง docker exec เอง)
# ติดตั้งเป็นคำสั่ง `arm` ด้วย alias ใน ~/.bashrc:  alias arm='bash ~/Humanoi/scripts/arm.sh'
# ดูคำสั่งทั้งหมด:  arm help      คู่มือเต็ม: docs/AR4_ARM_COMMANDS.md
set -uo pipefail

C=humanoi_ros
PORT=/dev/serial/by-id/usb-Teensyduino_USB_Serial_13173040-if00
MEGA=/dev/serial/by-id/usb-Arduino__www.arduino.cc__0042_858303033393517150C0-if00

ros() { docker exec "$C" /entrypoint.sh "$@"; }          # คำสั่งสั้น ไม่ต้องโต้ตอบ
ros_tty() { docker exec -it "$C" /entrypoint.sh "$@"; }  # คำสั่งที่เปิดค้าง กด Ctrl+C ได้
is_running() { docker exec "$C" pgrep -f "$1" >/dev/null 2>&1; }

need_container() {
  if ! docker ps --format '{{.Names}}' | grep -qx "$C"; then
    echo "✗ container $C ไม่ได้เปิดอยู่ — สั่ง: cd ~/Humanoi && docker compose up -d"
    exit 1
  fi
}

need_free_arm() {  # กันเปิดระบบแขนซ้อนสองชุด (เคยทำให้ข้อความกับ Teensy ปนกันจนแขนขยับมั่ว)
  if is_running ros2_control_node; then
    echo "✗ ระบบแขนเปิดอยู่แล้ว — ห้ามเปิดซ้อน ปิดหน้าต่างเดิมด้วย Ctrl+C หรือสั่ง: arm off"
    exit 1
  fi
  if [ ! -e "$PORT" ]; then
    echo "✗ ไม่เจอ Teensy ($PORT) — เสียบ USB ของแขนเข้า Pi และเปิดไฟแขนก่อน"
    exit 1
  fi
}

need_player() {
  if ! is_running "lib/ar_program/arm_program"; then
    echo "✗ ตัวเล่นโปรแกรมยังไม่เปิด — เปิดอีกหน้าต่างแล้วสั่ง: arm player"
    exit 1
  fi
}

show() { if is_running "$1"; then echo "  ✓ $2"; else echo "  ✗ $2"; fi; }

cmd="${1:-help}"
shift || true
need_container

case "$cmd" in
  status)
    echo "สถานะ:"
    if [ -e "$PORT" ]; then echo "  ✓ Teensy เสียบอยู่"; else echo "  ✗ Teensy ไม่ได้เสียบ"; fi
    show ros2_control_node "ระบบแขน          (arm start / arm calibrate)"
    show "lib/ar_program/arm_program" "ตัวเล่นโปรแกรม     (arm player)"
    show move_group "MoveIt + RViz      (arm moveit)"
    show "lib/humanoi_base/rc_arm_trigger" "สวิตช์รีโมท        (arm remote)"
    show "lib/humanoi_base/relay_bridge" "relay_bridge (Mega)"
    if is_running ros2_control_node; then
      echo "ตัวควบคุม:"
      ros timeout 10 ros2 control list_controllers 2>/dev/null | sed 's/^/  /'
    fi
    ;;

  calibrate)
    need_free_arm
    cat <<'EOF'
=== CALIBRATE — แขนจะหมุนทุกแกนไปชนสวิตช์ แล้วกลับท่าพัก (ราว 2 นาที) ===
 • รอบแขนต้องโล่ง สาย USB หย่อนพ้นรัศมีแขน (ฐาน J1 หมุนไปถึง −170°)
 • log จะเงียบ หรือขึ้น error/Failed to acquire lock ก็ได้ — ช่างมัน
 • ห้ามกด Ctrl+C จนกว่าแขนจะ "หยุดนิ่งที่ท่าพัก"
 • แขนนิ่งแล้ว: กด Ctrl+C แล้วสั่ง  arm start
EOF
    read -r -p "พร้อมแล้วกด Enter (ยกเลิกกด Ctrl+C) " _
    ros_tty ros2 launch ar_hardware_interface ar_hardware.launch.py serial_port:="$PORT" calibrate:=True
    echo
    echo "ถ้าแขนนิ่งที่ท่าพักแล้ว สั่งต่อ:  arm start"
    ;;

  start)
    need_free_arm
    echo "เปิดระบบแขน (ไม่ calibrate) — ปล่อยหน้าต่างนี้เปิดไว้ ปิดด้วย Ctrl+C"
    ros_tty ros2 launch ar_hardware_interface ar_hardware.launch.py serial_port:="$PORT" calibrate:=False
    ;;

  player)
    if is_running "lib/ar_program/arm_program"; then
      echo "✗ ตัวเล่นโปรแกรมเปิดอยู่แล้ว"; exit 1
    fi
    speed="${1:-0.19}"
    echo "เปิดตัวเล่นโปรแกรม speed=$speed rad/s — ปล่อยหน้าต่างนี้เปิดไว้ ปิดด้วย Ctrl+C"
    ros_tty ros2 launch ar_program arm_program.launch.py speed:="$speed"
    ;;

  moveit)
    if ! is_running ros2_control_node; then
      echo "✗ เปิดระบบแขนก่อน (arm start)"; exit 1
    fi
    exec bash "$HOME/Humanoi/scripts/arm_moveit.sh"
    ;;

  remote)
    # SWB โยกลง = เล่นโปรแกรม, SWD โยกลง = หยุดแขน (ดู humanoi_base/rc_arm_trigger.py)
    prog="${1:-1}"
    if is_running "lib/humanoi_base/rc_arm_trigger"; then
      echo "✗ ตัวรับสวิตช์รีโมทเปิดอยู่แล้ว"; exit 1
    fi
    if ! is_running "lib/ar_program/arm_program"; then
      echo "! ตัวเล่นโปรแกรมยังไม่เปิด — โยกสวิตช์แล้วจะเห็นผลบนจอ แต่แขนยังไม่ขยับ (เปิดด้วย arm player)"
    fi
    echo "SWB โยกลง = เล่นโปรแกรม '$prog' · SWD โยกลง = หยุดแขน · ปิดด้วย Ctrl+C"
    if is_running "lib/humanoi_base/relay_bridge"; then
      ros_tty ros2 run humanoi_base rc_arm_trigger --ros-args -p program_swb:="$prog"
    else
      [ -e "$MEGA" ] || { echo "✗ ไม่เจอ Mega ($MEGA) — เสียบ USB ของ Mega เข้า Pi"; exit 1; }
      # เปิด relay_bridge (คุยกับ Mega) คู่กันไปด้วย Ctrl+C ปิดทั้งคู่
      ros_tty bash -c "ros2 run humanoi_base relay_bridge --ros-args -p port:=$MEGA & \
        ros2 run humanoi_base rc_arm_trigger --ros-args -p program_swb:=$prog; kill %1 2>/dev/null"
    fi
    ;;

  run|goto)
    if [ "$cmd" = goto ]; then
      [ $# -ge 1 ] || { echo "ใช้: arm goto <ชื่อท่า>   (พาแขนไปท่าเดียว ดูชื่อด้วย arm list)"; exit 1; }
      target="@$1"; what="ไปท่า '$1'"
    else
      [ $# -ge 1 ] || { echo "ใช้: arm run <ชื่อโปรแกรม>   (ดูชื่อด้วย arm list)"; exit 1; }
      target="$1"; what="เล่นโปรแกรม '$1'"
    fi
    need_player
    echo "$what ... (หยุดฉุกเฉิน: arm stop หรือปิดสวิตช์ไฟแขน)"
    ros bash -c "timeout 600 ros2 topic echo /arm/done --once --field data 2>/dev/null & E=\$!; sleep 1.5; \
      ros2 topic pub --once /arm/run std_msgs/msg/String \"{data: '$target'}\" >/dev/null; wait \$E" \
      | sed -n 's/^\(.*:ok\)$/  ✓ \1/p; s/^\(.*:\(error\|stopped\|rejected\).*\)$/  ✗ \1/p' \
      | grep . || echo "  ? ไม่ได้รับผลภายใน 10 นาที — ดูหน้าต่าง arm player"
    ;;

  stop)
    ros ros2 topic pub --once /arm/stop std_msgs/msg/Empty "{}" >/dev/null && echo "ส่งคำสั่งหยุดแล้ว"
    ;;

  list)
    ros ros2 run ar_program teach --list
    ;;

  teach)
    [ $# -ge 1 ] || { echo "ใช้: arm teach <ชื่อท่า>   (บันทึกท่าปัจจุบันของแขน)"; exit 1; }
    ros ros2 run ar_program teach "$1"
    ;;

  program)
    [ $# -ge 2 ] || { echo "ใช้: arm program <ชื่อโปรแกรม> <ท่า1> <ท่า2> wait=1.0 <ท่า3> ..."; exit 1; }
    ros ros2 run ar_program teach --program "$@"
    ;;

  delete)
    [ $# -ge 1 ] || { echo "ใช้: arm delete <ชื่อท่า>"; exit 1; }
    ros ros2 run ar_program teach --delete "$1"
    ;;

  delete-program)
    [ $# -ge 1 ] || { echo "ใช้: arm delete-program <ชื่อโปรแกรม>"; exit 1; }
    ros ros2 run ar_program teach --delete-program "$1"
    ;;

  off)
    echo "ปิดทุกอย่างของแขน (แขนค้างที่ท่าปัจจุบัน)..."
    for p in "ros2 launch ar_program" "ros2 launch ar_gz_bringup" "ros2 launch ar_hardware_interface" \
             "lib/humanoi_base/rc_arm_trigger"; do
      docker exec "$C" pkill -INT -f "$p" 2>/dev/null
    done
    for _ in $(seq 1 15); do
      is_running ros2_control_node || is_running move_group || is_running "lib/ar_program/arm_program" || break
      sleep 1
    done
    for p in ros2_control_node move_group lib/ar_program/arm_program spawner rviz2; do
      docker exec "$C" pkill -9 -f "$p" 2>/dev/null
    done
    bash "$0" status
    ;;

  help|*)
    cat <<'EOF'
arm — คำสั่งลัดแขน AR4 (รันบน Pi นอก container)

 เริ่มใช้งาน (คำสั่งที่เปิดค้าง ให้ใช้หน้าต่างละคำสั่ง)
   arm status               ดูว่าอะไรเปิดอยู่บ้าง
   arm calibrate            calibrate (ต้องทำหลังปิดไฟแขน/ถอด USB/แขนชน/สายพานรูด)
   arm start                เปิดระบบแขน (หน้าต่าง 1 — เปิดค้าง)
   arm player [speed]       เปิดตัวเล่นโปรแกรม (หน้าต่าง 2 — เปิดค้าง) ค่าเริ่ม 0.19
   arm moveit               เปิด MoveIt + RViz บนจอ Pi (รันในเทอร์มินัลบนเดสก์ท็อป Pi)
   arm remote [โปรแกรม]      สั่งจากรีโมท: SWB โยกลง = เล่นโปรแกรม (ค่าเริ่ม 1), SWD โยกลง = หยุด (เปิดค้าง)

 ใช้งาน (หน้าต่างไหนก็ได้)
   arm run <โปรแกรม>         เล่นโปรแกรม รอจนจบแล้วบอกผล
   arm stop                 หยุดโปรแกรมกลางคัน
   arm list                 ดูท่าและโปรแกรมทั้งหมด
   arm teach <ท่า>          บันทึกท่าปัจจุบันของแขน
   arm program <ชื่อ> <ท่า...>  สร้างโปรแกรม เช่น arm program pick rest above wait=1.0 rest
   arm goto <ท่า>           พาแขนไปท่าเดียว (ใช้ตอนแก้ท่า: goto -> ปรับใน MoveIt -> teach ชื่อเดิม)
   arm delete <ท่า>         ลบท่า (ต้องไม่มีโปรแกรมไหนใช้อยู่)
   arm delete-program <ชื่อ>  ลบโปรแกรม (ท่าในโปรแกรมยังอยู่)

 ปิด
   arm off                  ปิดทุกอย่างของแขน
EOF
    ;;
esac
