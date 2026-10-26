#!/usr/bin/env bash
# arm — คำสั่งลัดคุมแขน AR4 สองตัว (ซ้าย = แขน 1, ขวา = แขน 2) บน Raspberry Pi (รันนอก container ได้เลย)
# ติดตั้งเป็นคำสั่ง `arm` ด้วย alias ใน ~/.bashrc:  alias arm='bash ~/Humanoi/scripts/arm.sh'
# ดูคำสั่งทั้งหมด:  arm help      คู่มือเต็ม: docs/AR4_ARM_COMMANDS.md
# ระบบสองแขน = แพ็กเกจ ar_dual (โมเดลรวมสองแขน + เสา, ros2_control Teensy สองตัว, MoveIt เช็กการชน)
set -uo pipefail

C=humanoi_ros
# พอร์ต Teensy: หาเองทุกครั้งโดยถามชื่อแขนที่เก็บไว้ในบอร์ด (เฟิร์มแวร์ AR4_ROS2_seq v2 + ar_dual/scripts/ar_ports.py)
# — ย้ายสาย USB ไปพอร์ตไหนของ Pi ก็ได้ (ปิดไฟแขนก่อนถอด/เสียบเสมอ) · ตั้งชื่อครั้งแรก/หลังเปลี่ยนบอร์ด: arm identify
# (Teensy สองตัวรายงานหมายเลข USB ซ้ำกัน 13173040 /dev/serial/by-id จึงแยกไม่ได้ ส่วน by-path ผูกกับพอร์ต)
# บังคับพอร์ตเองได้: LEFT_TEENSY=/dev/serial/by-path/... RIGHT_TEENSY=... arm start
# Nano (CH340) ก็ไม่มีหมายเลขเฉพาะ — ยังไม่ได้ใช้ใน ROS (มือจับ) ยังไม่ได้ทำให้หาเอง
LEFT_TEENSY="${LEFT_TEENSY:-}"
RIGHT_TEENSY="${RIGHT_TEENSY:-}"
MEGA=/dev/serial/by-id/usb-Arduino__www.arduino.cc__0042_858303033393517150C0-if00
# ลำดับข้อต่อตอน calibrate (เฟิร์มแวร์ AR4_ROS2_seq เลือกข้อได้) — ว่าง = ต้องใส่เองทุกครั้ง
# ตั้งหลังไล่ลองทีละข้อแล้วว่ามือที่ติดปลายแขนไม่ชน เช่น "6 5 4 1 2 3" (เลขติดกัน เช่น "456" = พร้อมกัน)
LEFT_CAL_ORDER=""
RIGHT_CAL_ORDER=""

ros() { docker exec "$C" /entrypoint.sh "$@"; }          # คำสั่งสั้น ไม่ต้องโต้ตอบ
ros_tty() { docker exec -it "$C" /entrypoint.sh "$@"; }  # คำสั่งที่เปิดค้าง กด Ctrl+C ได้
is_running() { docker exec "$C" pgrep -f "$1" >/dev/null 2>&1; }

need_container() {
  if ! docker ps --format '{{.Names}}' | grep -qx "$C"; then
    echo "✗ container $C ไม่ได้เปิดอยู่ — สั่ง: cd ~/Humanoi && docker compose up -d"
    exit 1
  fi
}

need_no_system() {  # กันเปิดระบบแขนซ้อนสองชุด / calibrate ขณะระบบแขนถือพอร์ตอยู่ (เคยทำให้แขนขยับมั่ว)
  if is_running ros2_control_node; then
    echo "✗ ระบบแขนเปิดอยู่แล้ว — ห้ามเปิดซ้อน ปิดหน้าต่างเดิมด้วย Ctrl+C หรือสั่ง: arm off"
    exit 1
  fi
}

resolve_ports() {  # เติม LEFT_TEENSY / RIGHT_TEENSY จากชื่อแขนในบอร์ด (ระบบแขนต้องปิดอยู่ บอร์ดถึงตอบ)
  [ -n "$LEFT_TEENSY" ] && [ -n "$RIGHT_TEENSY" ] && return
  local out
  out=$(ros ros2 run ar_dual ar_ports.py) || true
  [ -n "$LEFT_TEENSY" ] || LEFT_TEENSY=$(sed -n 's/.*left=\([^ ]*\).*/\1/p' <<< "$out")
  [ -n "$RIGHT_TEENSY" ] || RIGHT_TEENSY=$(sed -n 's/.*right=\([^ ]*\).*/\1/p' <<< "$out")
}

need_port() {  # need_port <พอร์ต> <ชื่อแขน>
  if [ -z "$1" ] || [ ! -e "$1" ]; then
    echo "✗ หาบอร์ด Teensy แขน$2 ไม่เจอ — เสียบ USB + เปิดไฟแขน แล้วดู: arm ports"
    echo "  (บอร์ดที่ยังไม่ตั้งชื่อ: arm identify · บอร์ดที่ค้างใน calibrate ต้องปิด-เปิดไฟแขนก่อน)"
    exit 1
  fi
}

arm_name() {  # left|1|ซ้าย -> left · right|2|ขวา -> right
  case "${1:-}" in
    left|l|1|ซ้าย) echo left ;;
    right|r|2|ขวา) echo right ;;
    *) echo "" ;;
  esac
}

need_player() {
  if ! is_running "lib/ar_program/arm_program"; then
    echo "✗ ตัวเล่นโปรแกรมยังไม่เปิด — เปิดอีกหน้าต่างแล้วสั่ง: arm player"
    exit 1
  fi
}

show() { if is_running "$1"; then echo "  ✓ $2"; else echo "  ✗ $2"; fi; }

rps_result() {  # อ่าน "<หุ่น>:<ผู้เล่น>:<ผล>" จาก /rps/result แล้วพิมพ์เป็นภาษาไทย
  local line robot player result
  read -r line || { echo "  ? ไม่ได้ผลภายใน 3 นาที — ดูหน้าต่าง arm rps"; return; }
  IFS=: read -r robot player result <<< "$line"
  declare -A th=([rock]=ค้อน [paper]=กระดาษ [scissors]=กรรไกร [none]="ไม่เห็นมือ")
  case "$result" in
    player_win) result="ผู้เล่นชนะ (หุ่นชูนิ้วโป้ง)" ;;
    robot_win) result="หุ่นชนะ (หุ่นชู 2 นิ้ว)" ;;
    draw) result="เสมอ (เป่าใหม่ครบแล้ว)" ;;
    no_player) result="ไม่เห็นมือผู้เล่น — ยื่นมือให้กล้องเห็นตอน ฉุบ" ;;
    stopped) result="หยุดกลางคัน" ;;
  esac
  echo "  หุ่น: ${th[$robot]:-$robot} · ผู้เล่น: ${th[$player]:-$player} → $result"
}

cmd="${1:-help}"
shift || true
need_container

case "$cmd" in
  status)
    echo "สถานะ:"
    if is_running ros2_control_node; then
      echo "  (ระบบแขนใช้พอร์ต Teensy อยู่ — ดูบอร์ดได้หลัง arm off ด้วย arm ports)"
    else
      ros ros2 run ar_dual ar_ports.py --list | sed 's/^/  /'
    fi
    show "ar_dual.*dual_hardware" "ระบบสองแขน      (arm start)"
    show "ar_hardware_interface ar_hardware.launch" "ระบบแขนเดี่ยวแบบเก่า (ควรปิด: arm off)"
    show "lib/ar_program/arm_program" "ตัวเล่นโปรแกรม     (arm player)"
    show move_group "MoveIt + RViz      (arm moveit)"
    show "lib/humanoi_base/rc_arm_trigger" "สวิตช์รีโมท        (arm remote)"
    show "lib/humanoi_games/rps_game" "เกมเป่ายิ้งฉุบ      (arm rps)"
    show "lib/humanoi_games/amazinghand_node" "ตัวคุมมือ           (arm player / arm rps)"
    show "lib/humanoi_base/relay_bridge" "relay_bridge (Mega)"
    if is_running ros2_control_node; then
      echo "ตัวควบคุม:"
      ros timeout 10 ros2 control list_controllers 2>/dev/null | sed 's/^/  /'
    fi
    ;;

  ports)
    need_no_system
    ros ros2 run ar_dual ar_ports.py --list
    ;;

  name)
    arm=$(arm_name "${2:-}")
    [ $# -ge 2 ] && [ -n "$arm" ] || { echo "ใช้: arm name <พอร์ต จาก arm ports> <left|right>   (ตั้งชื่อแขนเก็บในบอร์ด ทำครั้งเดียว)"; exit 1; }
    need_no_system
    ros ros2 run ar_dual ar_ports.py --name "$1" "$arm"
    ;;

  identify)  # ตั้งชื่อบอร์ดโดยดูว่าแขนไหนขยับ — แทนการถอดสาย USB (ห้ามถอด/เสียบขณะไฟแขนเปิด)
    need_no_system
    if [ $# -ge 1 ]; then
      ros_tty ros2 run ar_dual ar_ports.py --wiggle "$1"
    else
      ros_tty ros2 run ar_dual ar_ports.py --identify
    fi
    ;;

  dir)
    arm=$(arm_name "${1:-}")
    [ -n "$arm" ] && [ $# -ge 2 ] || { cat <<'EOF'
ใช้: arm dir <left|right> <6 หลัก J1..J6>   ตั้งข้อที่มอเตอร์หมุนกลับด้าน (1 = กลับ) เก็บในบอร์ด
  เช่น  arm dir right 000010   = J5 แขนขวากลับทิศ · arm dir right 000000 = ปกติทุกข้อ
  ใช้เมื่อ calibrate แล้วข้อนั้นหมุนหนีสวิตช์ (ขึ้น FAIL) — ตั้งแล้วต้อง calibrate ข้อนั้นใหม่
EOF
      exit 1; }
    need_no_system
    resolve_ports
    port=$LEFT_TEENSY; [ "$arm" = right ] && port=$RIGHT_TEENSY
    need_port "$port" "$arm"
    ros ros2 run ar_dual ar_ports.py --dir "$port" "$2"
    ;;

  zero)  # เขียนค่าชดเชยศูนย์ข้อต่อ (ar_dual/config/zero_trims.yaml) ลงบอร์ด — เฟิร์มแวร์ AR4_ROS2_seq v3
    arm=$(arm_name "${1:-}")
    [ -n "$arm" ] || { echo "ใช้: arm zero <left|right>   เขียนค่าชดเชยศูนย์จาก ar_dual/config/zero_trims.yaml ลงบอร์ด"; exit 1; }
    need_no_system
    resolve_ports
    port=$LEFT_TEENSY; [ "$arm" = right ] && port=$RIGHT_TEENSY
    need_port "$port" "$arm"
    ros ros2 run ar_dual ar_zero.py "$port" "$arm" --apply
    ;;

  calibrate)
    arm=$(arm_name "${1:-}")
    [ -n "$arm" ] || { echo "ใช้: arm calibrate <left|right> [ลำดับข้อ เช่น 6 5 4 1 2 3 | all]   (ทีละแขน)"; exit 1; }
    shift
    need_no_system
    resolve_ports
    port=$LEFT_TEENSY; [ "$arm" = right ] && port=$RIGHT_TEENSY
    th=ซ้าย; [ "$arm" = right ] && th=ขวา
    order=("$@")
    if [ ${#order[@]} -eq 0 ]; then  # ลำดับที่ใช้ได้แล้วของแต่ละแขน (ตั้งหลังไล่ลองทีละข้อว่ามือไม่ชน)
      default=$LEFT_CAL_ORDER; [ "$arm" = right ] && default=$RIGHT_CAL_ORDER
      if [ -z "$default" ]; then
        cat <<EOF
ยังไม่ได้ตั้งลำดับ calibrate ของแขน$th — ใส่ลำดับข้อเอง เช่น
  arm calibrate $arm 5            ข้อเดียว (ไล่ลองทีละข้อว่าข้อไหนพามือไปชน)
  arm calibrate $arm 6 5 4 1 2 3  ทีละข้อตามลำดับ
  arm calibrate $arm all          6 ข้อพร้อมกันแบบต้นฉบับ (มือที่ติดปลายแขนอาจชน)
ได้ลำดับที่ปลอดภัยแล้ว ตั้งเป็นค่าเริ่มได้ที่ LEFT_CAL_ORDER / RIGHT_CAL_ORDER ใน scripts/arm.sh
EOF
        exit 1
      fi
      read -r -a order <<< "$default"
    fi
    need_port "$port" "$th"
    cat <<EOF
=== CALIBRATE แขน$th — ลำดับ: ${order[*]} ===
 • แต่ละข้อหมุนไปชนสวิตช์ ถอยออก แล้วกลับท่าพัก ก่อนไปข้อถัดไป (ข้อที่ไม่ได้ทำไม่ขยับ)
 • ดูให้แน่ใจว่าแขนที่ขยับคือแขน$th (ครั้งแรกหลังย้ายสาย) — ไม่ใช่ หรือมือจะชน ให้ปิดไฟแขนทันที
 • รอบแขนต้องโล่ง สาย USB หย่อนพ้นรัศมีแขน (J1 หมุนไปถึง −170°) อีกแขนต้องไม่อยู่ในทาง
 • ถ้า USB หลุดกลางทางไม่เป็นไร — แขนทำต่อเอง โปรแกรมจะรอจนเสร็จ
 • เสร็จแล้วข้อที่ตั้งไว้ใน ar_dual/config/zero_trims.yaml (after_calibrate) หมุนต่อไปท่านั้น เช่นข้อ 5 ตรง
EOF
    read -r -p "พร้อมแล้วกด Enter (ยกเลิกกด Ctrl+C) " _
    ros_tty ros2 run ar_dual ar_calibrate.py "$port" --arm "$arm" "${order[@]}" \
      && echo "เสร็จแล้ว — calibrate ข้ออื่น/อีกแขน หรือเปิดระบบ: arm start"
    ;;

  start)
    need_no_system
    left_hw=real; right_hw=real
    case "${1:-}" in
      "") ;;
      left|l|1|ซ้าย) right_hw=mock ;;
      right|r|2|ขวา) left_hw=mock ;;
      *) echo "ใช้: arm start [left|right]   (ไม่ใส่ = สองแขน, left/right = แขนเดียว อีกแขนเป็นแบบจำลอง)"; exit 1 ;;
    esac
    resolve_ports
    [ "$left_hw" = real ] && need_port "$LEFT_TEENSY" ซ้าย
    [ "$right_hw" = real ] && need_port "$RIGHT_TEENSY" ขวา
    # ค่าชดเชยศูนย์ในบอร์ดต้องตรงกับไฟล์ ไม่งั้นท่าที่สอนไว้จะไปผิดที่ (เช่นแฟลชบอร์ดใหม่แล้วลืม arm zero)
    [ "$left_hw" = real ] && { ros ros2 run ar_dual ar_zero.py "$LEFT_TEENSY" left || exit 1; }
    [ "$right_hw" = real ] && { ros ros2 run ar_dual ar_zero.py "$RIGHT_TEENSY" right || exit 1; }
    echo "เปิดระบบแขน (ซ้าย=$left_hw ${LEFT_TEENSY:+@ $LEFT_TEENSY} · ขวา=$right_hw ${RIGHT_TEENSY:+@ $RIGHT_TEENSY})"
    echo "ปล่อยหน้าต่างนี้เปิดไว้ ปิดด้วย Ctrl+C · แขนต้อง calibrate แล้ว ไม่งั้นตำแหน่งที่ระบบเห็นจะผิด"
    ros_tty ros2 launch ar_dual dual_hardware.launch.py left_port:="${LEFT_TEENSY:-none}" \
      right_port:="${RIGHT_TEENSY:-none}" left_hw:=$left_hw right_hw:=$right_hw
    ;;

  player)
    if is_running "lib/ar_program/arm_program"; then
      echo "✗ ตัวเล่นโปรแกรมเปิดอยู่แล้ว"; exit 1
    fi
    if is_running "lib/humanoi_games/rps_game"; then
      echo "✗ เกมเปิดอยู่ (สั่งแขนเดียวกัน + ใช้พอร์ตมือ) — ปิดหน้าต่าง arm rps ก่อน"; exit 1
    fi
    speed=0.19; hand=true
    for a in "$@"; do case "$a" in nohand) hand=false ;; *) speed=$a ;; esac; done
    if [ "$hand" = true ] && { is_running "lib/humanoi_games/hand_calibrate" || is_running "lib/humanoi_games/hand_teach"; }; then
      echo "✗ arm hand-calibrate / hand-teach เปิดอยู่ (ใช้พอร์ตมือ) — ปิดก่อน หรือ arm player $speed nohand"; exit 1
    fi
    echo "เปิดตัวเล่นโปรแกรม speed=$speed rad/s + ตัวคุมมือ (มือจริง: $hand) — ปล่อยหน้าต่างนี้เปิดไว้ ปิดด้วย Ctrl+C"
    ros_tty ros2 launch humanoi_games player.launch.py speed:="$speed" hand:="$hand"
    ;;

  hand)  # ลองท่ามือตรงๆ — ต้องมีตัวคุมมือเปิดอยู่ (arm player หรือ arm rps)
    names=$(ros ros2 run ar_program teach --hand-names 2>/dev/null | tr '\n' ' ')  # ท่าในตัว + ท่าที่สอนเอง
    g="${1:-}"; arm=$(arm_name "${2:-left}")
    if [ -z "$g" ] || [[ " $names " != *" $g "* ]] || [ -z "$arm" ]; then
      echo "ใช้: arm hand <ท่า> [left|right]   ท่า: $names"; exit 1
    fi
    topic=/hand/gesture; th=ซ้าย
    [ "$arm" = right ] && topic=/right_hand/gesture && th=ขวา
    # มีตัวคุมมือฟังอยู่ไหม (มือบนแขนขวายังไม่มี — หัวข้อนี้เผื่อมือที่สอง)
    if ! ros ros2 topic info "$topic" 2>/dev/null | grep -q "Subscription count: [1-9]"; then
      echo "✗ ไม่มีตัวคุมมือบนแขน$th ($topic) — เปิด arm player ก่อน"; exit 1
    fi
    ros timeout 10 ros2 topic pub --once -w 1 "$topic" std_msgs/msg/String "{data: $g}" >/dev/null 2>&1 \
      && echo "มือ$th -> $g" || echo "✗ ส่งไม่ถึงตัวคุมมือ ($topic)"
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
      arm=$(arm_name "${1:-}")
      [ -n "$arm" ] && [ $# -ge 2 ] || { echo "ใช้: arm goto <left|right> <ชื่อท่า>   (ดูชื่อด้วย arm list)"; exit 1; }
      target="@$arm:$2"; what="แขน $arm ไปท่า '$2'"
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

  rps)  # เกมเป่ายิ้งฉุบ (ros2_ws/src/humanoi_games) — คู่มือ docs/RPS_GAME.md
    case "${1:-}" in
      play)
        is_running "lib/humanoi_games/rps_game" || { echo "✗ เปิดเกมก่อน (อีกหน้าต่าง): arm rps"; exit 1; }
        echo "เป่า-ยิ้ง-ฉุบ! ... (หยุด: arm rps stop หรือปิดสวิตช์ไฟแขน)"
        ros bash -c "timeout 180 ros2 topic echo /rps/result --once --field data 2>/dev/null & E=\$!; sleep 1.5; \
          ros2 topic pub --once /rps/play std_msgs/msg/Empty '{}' >/dev/null; wait \$E" | rps_result
        ;;
      stop)
        ros ros2 topic pub --once /rps/stop std_msgs/msg/Empty "{}" >/dev/null && echo "ส่งคำสั่งหยุดเกมแล้ว"
        ;;
      ""|camera|random|topic|nohand|*:=*)
        player=camera; hand=true; extra=()
        for a in "$@"; do
          case "$a" in
            camera|random|topic) player=$a ;;
            nohand) hand=false ;;
            *:=*) extra+=("$a") ;;  # ส่งต่อให้ rps.launch.py เช่น pump_deg:=4
            *) echo "✗ ไม่รู้จัก '$a' — ดู: arm rps help"; exit 1 ;;
          esac
        done
        is_running ros2_control_node || { echo "✗ เปิดระบบแขนก่อน (arm start left)"; exit 1; }
        if is_running "lib/ar_program/arm_program"; then
          echo "✗ ตัวเล่นโปรแกรมเปิดอยู่ (สั่งแขนเดียวกัน + ใช้พอร์ตมือ) — ปิดหน้าต่าง arm player ก่อน"; exit 1
        fi
        if is_running "lib/humanoi_games/rps_game"; then echo "✗ เกมเปิดอยู่แล้ว — เล่น: arm rps play"; exit 1; fi
        camera=true; is_running astra_camera && camera=false  # กล้องเปิดอยู่แล้ว (เช่นจาก yolo) ไม่ต้องเปิดซ้ำ
        echo "เปิดเกมเป่ายิ้งฉุบ (ผู้เล่น: $player · มือ: $hand ${extra[*]}) — ปล่อยหน้าต่างนี้เปิดไว้ ปิดด้วย Ctrl+C"
        echo "เล่น 1 ตา: arm rps play (อีกหน้าต่าง) · ต้องสอนท่าตั้งก่อน: arm teach left rps_ready"
        ros_tty ros2 launch humanoi_games rps.launch.py player:="$player" hand:="$hand" camera:="$camera" "${extra[@]}"
        ;;
      *)
        cat <<'EOF'
ใช้: arm rps [camera|random|topic] [nohand] [ชื่อ:=ค่า ...]   เปิดเกม (เปิดค้าง)
       camera = ดูมือผู้เล่นจากกล้อง (ค่าเริ่ม) · random = สุ่มแทนผู้เล่น (ทดสอบ) · nohand = ไม่ต่อมือจริง
       ปรับได้ เช่น pump_deg:=4 (ระยะง้างข้อ 1: 5° = 8.6 วินาที, 4° = 7.7, 3° = 6.7) · robot_choice:=rock
     arm rps play                           เล่น 1 ตา รอจนจบแล้วบอกผล
     arm rps stop                           หยุดเกมกลางคัน
EOF
        exit 1 ;;
    esac
    ;;

  tray)  # กล้อง + หาตำแหน่งกล่องจากป้าย ArUco id 1 (humanoi_vision tray.launch.py) — ดูภาพ /tray/image/compressed
    if is_running "lib/humanoi_vision/tray_detector"; then
      echo "✗ ตัวหากล่องเปิดอยู่แล้ว"; exit 1
    fi
    if is_running "lib/astra_camera/astra_camera_node"; then  # กล้องเปิดได้ทีละโปรแกรม
      echo "✗ กล้องถูกเปิดอยู่โดยโปรแกรมอื่น (hand_eye_calibrate / yolo / arm rps) — ปิดหน้าต่างนั้นก่อน"
      echo "  หรือใช้กล้องตัวที่เปิดอยู่: arm tray camera:=false"; exit 1
    fi
    ros_tty ros2 launch humanoi_vision tray.launch.py "$@"
    ;;

  tray-teach)  # สอนหยิบกล่องแบบทางลัด: ท่าแตะหาทิศกล้อง + ท่าจับจริง (humanoi_vision tray_teach)
    if ! is_running "lib/humanoi_vision/tray_detector"; then
      echo "✗ เปิดตัวหากล่องก่อน (อีกหน้าต่าง): arm tray"; exit 1
    fi
    arm=$(arm_name "${1:-left}")
    [ -n "$arm" ] || { echo "ใช้: arm tray-teach [left|right]"; exit 1; }
    ros_tty ros2 run humanoi_vision tray_teach --ros-args -p arm:="$arm"
    ;;

  hand-side)  # ชุดมือที่ใส่อยู่ (right = ชุดมือขวาตัวแรก, left = ชุดมือซ้าย) — ID 1-8 เหมือนกัน แยกเองไม่ได้
    f=/humanoi_ws/src/humanoi_games/config/hand_side
    case "${1:-}" in
      "")
        side=$(ros cat "$f" 2>/dev/null || echo right)
        echo "ชุดมือตอนนี้: $side   (เปลี่ยน: arm hand-side left|right)" ;;
      left|right)
        if is_running "lib/humanoi_games/amazinghand_node" || is_running "lib/humanoi_games/hand_teach" \
          || is_running "lib/humanoi_games/hand_calibrate"; then
          echo "✗ มีโปรแกรมใช้มืออยู่ (arm player / arm rps / hand-teach / hand-calibrate) — ปิดก่อนแล้วค่อยเปลี่ยน"; exit 1
        fi
        ros sh -c "mkdir -p ${f%/*} && echo $1 > $f" && echo "ตั้งเป็นชุดมือ$([ "$1" = left ] && echo ซ้าย || echo ขวา) แล้ว" \
          "— ค่าคาลิเบรตและท่าที่สอนแยกตามชุดมือ" ;;
      *) echo "ใช้: arm hand-side [left|right]"; exit 1 ;;
    esac
    ;;

  hand-scan)  # เช็กบอร์ดมือทุกตัว: servo ID ไหนตอบ baud ไหน ไฟเท่าไหร่ (อ่านอย่างเดียว) — humanoi_games/hand_scan.py
    if is_running "lib/humanoi_games/amazinghand_node" || is_running "lib/humanoi_games/hand_teach"       || is_running "lib/humanoi_games/hand_calibrate"; then
      echo "✗ มีโปรแกรมใช้พอร์ตมืออยู่ (arm player / arm rps / hand-teach / hand-calibrate) — ปิดก่อน"; exit 1
    fi
    ros_tty ros2 run humanoi_games hand_scan
    ;;

  hand-teach)  # จัดท่ามือเอง (จ็อกทีละนิ้ว / ปล่อยให้ดัดด้วยมือ) แล้วบันทึก — humanoi_games/hand_teach.py
    if is_running "lib/humanoi_games/amazinghand_node" || is_running "lib/humanoi_games/hand_calibrate"; then
      echo "✗ มีโปรแกรมใช้พอร์ตมืออยู่ (arm player / arm rps / arm hand-calibrate) — ปิดก่อน"; exit 1
    fi
    ros_tty ros2 run humanoi_games hand_teach
    ;;

  hand-calibrate)  # ตั้งค่ากลางนิ้ว (MiddlePos) ของมือ AmazingHand ด้วยตา — humanoi_games/hand_calibrate.py
    if is_running "lib/humanoi_games/amazinghand_node" || is_running "lib/humanoi_games/hand_teach"; then
      echo "✗ มีโปรแกรมใช้พอร์ตมืออยู่ (arm player / arm rps / arm hand-teach) — ปิดก่อน"; exit 1
    fi
    ros_tty ros2 run humanoi_games hand_calibrate "$@"
    ;;

  stop)
    ros ros2 topic pub --once /arm/stop std_msgs/msg/Empty "{}" >/dev/null && echo "ส่งคำสั่งหยุดแล้ว (ทุกแขน)"
    ;;

  list)
    ros ros2 run ar_program teach --list
    ;;

  show)
    [ $# -ge 1 ] || { echo "ใช้: arm show <ชื่อโปรแกรม>   (ดูเป็นตาราง ขั้น | แขนซ้าย | แขนขวา)"; exit 1; }
    ros ros2 run ar_program teach --show "$1"
    ;;

  teach)
    arm=$(arm_name "${1:-}")
    [ -n "$arm" ] && [ $# -ge 2 ] || { echo "ใช้: arm teach <left|right> <ชื่อท่า>   (บันทึกท่าปัจจุบันของแขนนั้น)"; exit 1; }
    ros ros2 run ar_program teach "$2" --arm "$arm"
    ;;

  program)
    [ $# -ge 2 ] || { cat <<'EOF'
ใช้: arm program <ชื่อโปรแกรม> <ขั้น1> <ขั้น2> ...
  แต่ละขั้น = <แขนซ้าย>/<แขนขวา>[/<มือซ้าย>[/<มือขวา>]]   ใช้ - ถ้าไม่ขยับ   wait=วินาที = รอ
  ท่ามือ: rock paper scissors point like windup victory nonono relax
  แขนกับมือในขั้นเดียวกันขยับพร้อมกัน — ให้แขนถึงก่อนแล้วค่อยกำ: แยกขั้น act1/- แล้ว -/-/rock
  เช่น  arm program pick2 act1/act1 act2/- act3/act2 -/act3 wait=1.0 rest/rest
        arm program grab act1/- -/-/rock act2/-/paper
EOF
      exit 1; }
    ros ros2 run ar_program teach --program "$@"
    ;;

  delete)
    arm=$(arm_name "${1:-}")
    [ -n "$arm" ] && [ $# -ge 2 ] || { echo "ใช้: arm delete <left|right> <ชื่อท่า>"; exit 1; }
    ros ros2 run ar_program teach --delete "$2" --arm "$arm"
    ;;

  delete-program)
    [ $# -ge 1 ] || { echo "ใช้: arm delete-program <ชื่อโปรแกรม>"; exit 1; }
    ros ros2 run ar_program teach --delete-program "$1"
    ;;

  off)
    echo "ปิดทุกอย่างของแขน (แขนค้างที่ท่าปัจจุบัน)..."
    for p in "ros2 launch ar_program" "ros2 launch ar_dual" "ros2 launch ar_gz_bringup" \
             "ros2 launch ar_hardware_interface" "lib/humanoi_base/rc_arm_trigger" "ar_calibrate.py" \
             "ros2 launch humanoi_games"; do
      docker exec "$C" pkill -INT -f "$p" 2>/dev/null
    done
    for _ in $(seq 1 15); do
      is_running ros2_control_node || is_running move_group || is_running "lib/ar_program/arm_program" \
        || is_running "lib/humanoi_games" || break
      sleep 1
    done
    for p in ros2_control_node move_group lib/ar_program/arm_program spawner rviz2 lib/humanoi_games; do
      docker exec "$C" pkill -9 -f "$p" 2>/dev/null
    done
    bash "$0" status
    ;;

  help|*)
    cat <<'EOF'
arm — คำสั่งลัดแขน AR4 สองตัว (รันบน Pi นอก container) · left = แขน 1 (ซ้ายเมื่อมองจากตัวหุ่น), right = แขน 2

 บอร์ดแขน (หาพอร์ตเองจากชื่อที่เก็บในบอร์ด — ย้ายสาย USB ได้)
   arm ports                   ดูบอร์ด Teensy ทุกตัว: พอร์ต ชื่อแขน ข้อที่ตั้งกลับทิศ
   arm identify                ตั้งชื่อบอร์ด (ครั้งแรก / หลังเปลี่ยนบอร์ด): ข้อ 6 หมุนไป-กลับ 10° ทีละบอร์ด
                               ดูว่าแขนไหนขยับแล้วตอบ l = ซ้าย / r = ขวา — ไม่ต้องถอดสาย
   arm identify <left|right>   หมุนข้อ 6 ของแขนชื่อนั้น เช็กว่าชื่อตรงกับแขนจริง
   arm name <พอร์ต> <left|right>  ตั้งชื่อแขนให้บอร์ดตรงๆ (ถ้ารู้พอร์ตอยู่แล้ว)
   arm dir <left|right> <J1..J6>   ตั้งข้อที่มอเตอร์หมุนกลับด้าน เช่น arm dir right 000010
   arm zero <left|right>           เขียนค่าชดเชยศูนย์ข้อต่อ (ar_dual/config/zero_trims.yaml) ลงบอร์ด
   ห้ามถอด/เสียบ USB ของแขนขณะไฟแขนเปิด — ปิดไฟแขนก่อนเสมอ (Pi 5 เคยเสียจากเรื่องนี้)

 เริ่มใช้งาน (คำสั่งที่เปิดค้าง ให้ใช้หน้าต่างละคำสั่ง)
   arm status                  ดูว่าอะไรเปิดอยู่บ้าง
   arm calibrate <left|right> [ข้อ...]  calibrate ทีละแขน ทีละข้อตามลำดับ เช่น 6 5 4 1 2 3 (หรือ all = พร้อมกัน)
                               ต้องทำหลังปิดไฟแขน/ถอด USB/แขนชน/สายพานรูด
   arm start [left|right]      เปิดระบบแขน (หน้าต่าง 1 — เปิดค้าง) ไม่ใส่ = สองแขน
   arm player [speed] [nohand] เปิดตัวเล่นโปรแกรม + ตัวคุมมือ (หน้าต่าง 2 — เปิดค้าง) ค่าเริ่ม 0.19
                               nohand = ไม่ต่อมือจริง · ห้ามเปิดพร้อม arm rps
   arm moveit                  MoveIt + RViz สองแขน บนจอ Pi — Planning Group เลือก left_arm/right_arm/both_arms
   arm remote [โปรแกรม]         สั่งจากรีโมท: SWB โยกลง = เล่นโปรแกรม (ค่าเริ่ม 1), SWD โยกลง = หยุด (เปิดค้าง)

 สอนท่า / สร้างโปรแกรม (หน้าต่างไหนก็ได้)
   arm teach <left|right> <ท่า>       บันทึกท่าปัจจุบันของแขนนั้น (ชื่อท่าซ้ำกันข้ามแขนได้)
   arm program <ชื่อ> <ขั้น...>        แต่ละขั้น <แขนซ้าย>/<แขนขวา>[/<มือซ้าย>[/<มือขวา>]] - = ไม่ขยับ, wait=วินาที
                                      ท่ามือ: rock paper scissors point like windup victory nonono relax
                                      เช่น arm program pick2 act1/act1 act2/- act3/act2 -/act3
   arm show <ชื่อ>                    ดูโปรแกรมเป็นตาราง ขั้น | แขนซ้าย | แขนขวา
   arm list                           ดูท่าทุกแขนและโปรแกรมทั้งหมด
   arm delete <left|right> <ท่า>      ลบท่า (ต้องไม่มีโปรแกรมไหนใช้อยู่)
   arm delete-program <ชื่อ>           ลบโปรแกรม (ท่าในโปรแกรมยังอยู่)

 เล่น
   arm run <โปรแกรม>                  เล่นโปรแกรม รอจนจบแล้วบอกผล (ทุกขั้น แขนที่มีท่าเริ่มและถึงพร้อมกัน)
   arm goto <left|right> <ท่า>        พาแขนเดียวไปท่าเดียว (ใช้ตอนแก้ท่า: goto -> ปรับใน MoveIt -> teach ชื่อเดิม)
   arm stop                           หยุดโปรแกรมกลางคัน (ทุกแขน)
   arm hand <ท่า> [left|right]        ลองท่ามือตรงๆ (ต้องเปิด arm player อยู่) เช่น arm hand rock
   arm tray [marker_size:=0.12]       กล้อง + หาตำแหน่งกล่อง (ป้าย id 1 บนพื้นกล่อง) ดูภาพ /tray/image/compressed
   arm tray-teach [left|right]        สอนหยิบกล่อง: แตะ 4-6 ครั้งข้อมือมุมเดิม + จับจริง g (เปิด arm start + arm tray ก่อน)
   arm hand-side [left|right]         ชุดมือที่ใส่อยู่ (เปลี่ยนมือแล้วตั้งก่อน — คาลิเบรต/ท่าที่สอนแยกตามชุดมือ)
   arm hand-scan                      เช็กบอร์ดมือ: เซอร์โวตอบไหม baud ไหน ไฟเท่าไหร่ (ไม่ขยับ)
   arm hand-teach                     จัดท่ามือเอง (1-4 เลือกนิ้ว +/- กำแบ l/r เอียง free ดัดด้วยมือ) save <ชื่อ>
                                      ชื่อซ้ำท่าในตัว (rock paper scissors ...) = ใช้แทน · ปิด arm player ก่อน

 เกมเป่ายิ้งฉุบ (คู่มือ docs/RPS_GAME.md — สอนท่าตั้งก่อน: arm teach left rps_ready)
   arm rps [camera|random] [nohand]   เปิดเกม (เปิดค้าง แทน arm player) ดูมือผู้เล่นจากกล้อง / สุ่มแทน
                                      ปรับระยะง้าง: arm rps camera pump_deg:=4 (ค่าเริ่ม 5°)
   arm rps play                       เล่นตาแรก: J1 ขึ้นลง 3 ครั้ง → มือออกท่า → แขนไปท่าโชว์ → หุ่นแพ้ชูโป้ง ชนะชู 2 นิ้ว
                                      ตาต่อไป: ผู้เล่นชูนิ้วชี้ค้างไว้ = เล่นใหม่ (วนได้เรื่อยๆ)
                                      ท่าโชว์ (ไม่บังคับ): arm teach left victory_pose / arm teach left lose_pose
   arm rps stop                       หยุดเกมกลางคัน
   arm hand-calibrate                 ตั้งค่ากลางนิ้วของมือทีละนิ้ว (ปิดเกมก่อน) — เกมใช้ค่าใหม่ตอนเปิดครั้งถัดไป

 ปิด
   arm off                            ปิดทุกอย่างของแขน (รวมเกม)

 ไม่มีการเช็คการชนตอน arm run/goto (รวมสองแขนชนกันเอง) — ทดสอบโปรแกรมใหม่ด้วย speed ต่ำก่อนเสมอ
EOF
    ;;
esac
