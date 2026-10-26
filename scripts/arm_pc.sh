#!/usr/bin/env bash
# arm (PC) — คำสั่งลัดแขน AR4 สองตัวแบบเดียวกับ scripts/arm.sh บน Pi แต่รันใน WSL (Ubuntu-24.04) บน PC
# ติดตั้งเป็นคำสั่ง `arm` ใน WSL:  alias arm='bash /mnt/c/Users/User/Desktop/Humanoi/scripts/arm_pc.sh'
# ดูคำสั่งทั้งหมด:  arm help      คู่มือเต็ม: docs/AR4_ARM_COMMANDS.md (หัวข้อ "ใช้บน PC")
#
# ต่างจาก Pi สองเรื่อง:
#  1) ROS รันใน WSL ตรงๆ (ไม่มี docker) — ใช้ ar4_ws ที่ build ไว้ใน WSL (ถัดจาก Humanoi: ../ar4/ar4_ws)
#  2) Teensy เป็นพอร์ต COM ของ Windows — WSL มองไม่เห็น
#     · งานที่คุยกับบอร์ดตรงๆ (ports/name/dir/identify/calibrate) เรียก Python ฝั่ง Windows ให้ (python.exe + pyserial)
#     · arm start: Windows เปิด COM แล้วส่งผ่าน TCP (ar4/tools/serial_bridge_win.py) socat ใน WSL ทำเป็น
#       /tmp/ttyV_left /tmp/ttyV_right ให้ ros2_control (ไม่ใช้ usbip — ช้าและหลุด ดูหัวไฟล์สะพาน)
#       สะพานฟังเฉพาะ IP ฝั่ง Windows ของ WSL (ไม่เปิดให้เครื่องอื่นในวงแลนสั่งแขนได้) ปิดเองตอนปิด arm start
set -o pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
AR4_DIR="$(realpath -m "${AR4_DIR:-$HERE/../../ar4}")"
AR4_WS="$AR4_DIR/ar4_ws"
if [ ! -f "$AR4_WS/install/setup.bash" ]; then
  echo "✗ ไม่เจอ $AR4_WS/install — build ใน WSL ก่อน: cd $AR4_WS && colcon build --symlink-install"
  exit 1
fi
# ROS setup ใช้ตัวแปรที่ไม่ได้ตั้ง — source ก่อน set -u
source /opt/ros/jazzy/setup.bash
source "$AR4_WS/install/setup.bash"
# แพ็กเกจของ Humanoi (เกม humanoi_games) — build แยกใน Humanoi/ros2_ws
[ -f "$HERE/../ros2_ws/install/local_setup.bash" ] && source "$HERE/../ros2_ws/install/local_setup.bash"
set -u

# พอร์ต COM ของ Teensy: หาเองจากชื่อแขนที่เก็บในบอร์ด (เหมือน Pi) · บังคับเองได้: LEFT_TEENSY=COM9 arm start
LEFT_TEENSY="${LEFT_TEENSY:-}"
RIGHT_TEENSY="${RIGHT_TEENSY:-}"
# ลำดับ calibrate ใช้ค่าเดียวกับ Pi — ตั้งที่ scripts/arm.sh ที่เดียว (LEFT_CAL_ORDER / RIGHT_CAL_ORDER)
LEFT_CAL_ORDER=$(sed -n 's/^LEFT_CAL_ORDER="\(.*\)".*/\1/p' "$HERE/arm.sh")
RIGHT_CAL_ORDER=$(sed -n 's/^RIGHT_CAL_ORDER="\(.*\)".*/\1/p' "$HERE/arm.sh")
BRIDGE_TCP_LEFT=5555
BRIDGE_TCP_RIGHT=5556

# ---- ฝั่ง Windows ----
if ! command -v python.exe >/dev/null 2>&1; then
  echo "✗ เรียก python.exe ของ Windows จาก WSL ไม่ได้ — ติดตั้ง Python บน Windows (+ pip install pyserial)"
  exit 1
fi
# Python บน Windows เขียนออก pipe ด้วย code page ของเครื่อง — บังคับ UTF-8 ไม่งั้นข้อความไทยพัง
export PYTHONUTF8=1 PYTHONIOENCODING=utf-8
export WSLENV="${WSLENV:+$WSLENV:}PYTHONUTF8/w:PYTHONIOENCODING/w"
PORTS_PY=$(wslpath -w "$AR4_WS/src/ar_dual/scripts/ar_ports.py")
CAL_PY=$(wslpath -w "$AR4_WS/src/ar_dual/scripts/ar_calibrate.py")
BRIDGE_PY=$(wslpath -w "$AR4_DIR/tools/serial_bridge_win.py")
WIN_OURS='serial_bridge_win\.py|ar_calibrate\.py|ar_ports\.py'  # python ฝั่ง Windows ที่ถือพอร์ต COM ของแขน

winpy() { python.exe -u "$@"; }

win_ps() {  # รัน PowerShell ฝั่ง Windows (ส่งเป็น base64 — ไม่ต้องห่วงเรื่อง quote ข้าม WSL)
  powershell.exe -NoProfile -NonInteractive -EncodedCommand \
    "$(printf '%s' "$1" | iconv -f UTF-8 -t UTF-16LE | base64 -w0)" 2>/dev/null | tr -d '\r'
}

win_procs() {  # win_procs <regex> — "PID บรรทัดคำสั่ง" ของ python.exe ฝั่ง Windows ที่ตรง regex
  win_ps "Get-CimInstance Win32_Process | Where-Object { \$_.Name -eq 'python.exe' -and \$_.CommandLine -match '$1' } |
          ForEach-Object { '{0} {1}' -f \$_.ProcessId, \$_.CommandLine }"
}

win_kill() {  # win_kill <regex>
  win_ps "Get-CimInstance Win32_Process | Where-Object { \$_.Name -eq 'python.exe' -and \$_.CommandLine -match '$1' } |
          ForEach-Object { Stop-Process -Id \$_.ProcessId -Force }"
}

# ---- ฝั่ง WSL ----
is_running() { pgrep -f "$1" >/dev/null 2>&1; }

need_no_system() {  # กันเปิดระบบแขนซ้อนสองชุด
  if is_running ros2_control_node; then
    echo "✗ ระบบแขนเปิดอยู่แล้ว — ห้ามเปิดซ้อน ปิดหน้าต่างเดิมด้วย Ctrl+C หรือสั่ง: arm off"
    exit 1
  fi
  need_ports_free
}

need_ports_free() {  # กัน calibrate / ถามบอร์ด ขณะมีโปรแกรมฝั่ง Windows ถือพอร์ต COM อยู่
  # (ระบบแขนจริงบน PC ถือพอร์ตผ่านสะพานฝั่ง Windows เสมอ — ระบบจำลอง arm start sim ไม่ถือ จึงไม่ต้องปิด)
  local busy
  busy=$(win_procs "$WIN_OURS")
  if [ -n "$busy" ]; then
    echo "✗ มีโปรแกรมฝั่ง Windows ใช้พอร์ตแขนอยู่ (calibrate / สะพาน serial ค้าง):"
    sed 's/^/    /' <<< "$busy"
    echo "  ถ้าไม่ได้เปิดค้างไว้ที่หน้าต่างอื่น สั่ง: arm off"
    exit 1
  fi
}

resolve_ports() {  # เติม LEFT_TEENSY / RIGHT_TEENSY (COMx) จากชื่อแขนในบอร์ด
  [ -n "$LEFT_TEENSY" ] && [ -n "$RIGHT_TEENSY" ] && return
  local out
  # รวม stderr มาด้วยแล้วพิมพ์เอง — interop ส่ง stderr ของ python.exe มาช้ากว่า แล้วไปแทรกกลางข้อความถัดไป
  out=$(winpy "$PORTS_PY" 2>&1 | tr -d '\r') || true
  grep '^!' <<< "$out" >&2
  [ -n "$LEFT_TEENSY" ] || LEFT_TEENSY=$(sed -n 's/.*left=\([^ ]*\).*/\1/p' <<< "$out")
  [ -n "$RIGHT_TEENSY" ] || RIGHT_TEENSY=$(sed -n 's/.*right=\([^ ]*\).*/\1/p' <<< "$out")
}

need_port() {  # need_port <COMx> <ชื่อแขน>
  if [ -z "$1" ]; then
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
    no_player) result="ไม่เห็นมือผู้เล่น" ;;
    stopped) result="หยุดกลางคัน" ;;
  esac
  echo "  หุ่น: ${th[$robot]:-$robot} · ผู้เล่น: ${th[$player]:-$player} → $result"
}

# ---- สะพาน serial (arm start) ----
HOST_IP=127.0.0.1  # networkingMode=mirrored: Windows คือ localhost
if [ "$(wslinfo --networking-mode 2>/dev/null)" != mirrored ]; then
  HOST_IP=$(ip route show default | awk '{print $3; exit}')  # NAT: Windows คือ default gateway ของ WSL
fi
BRIDGE_PIDS=()

start_bridge() {  # start_bridge <left|right> <COMx> <tcp port> -> /tmp/ttyV_<arm>
  local link="/tmp/ttyV_$1" log="/tmp/arm_bridge_$1.log" ok=""
  rm -f "$link"
  # setsid: Ctrl+C ในหน้าต่างไปถึง ros2 launch อย่างเดียว สะพานอยู่จนระบบแขนปิดเสร็จ แล้ว cleanup ปิดเอง
  setsid python.exe -u "$BRIDGE_PY" --port "$2" --tcp-port "$3" --listen "$HOST_IP" </dev/null >"$log" 2>&1 &
  BRIDGE_PIDS+=($!)
  for _ in $(seq 1 20); do  # รอ Windows เปิด COM แล้วฟังพอร์ต (เปิด COM ไม่ได้ = วนลองเอง ไม่ฟัง)
    if timeout 1 bash -c "</dev/tcp/$HOST_IP/$3" 2>/dev/null; then ok=1; break; fi
    sleep 0.5
  done
  if [ -z "$ok" ]; then
    echo "✗ สะพาน serial แขน$1 ($2) ไม่ขึ้น:"
    tr -d '\r' < "$log" | tail -3 | sed 's/^/    /'
    exit 1
  fi
  # ตัวเลือก socat เดียวกับ ar4/tools/serial_bridge_wsl.sh (คำอธิบายอยู่ในไฟล์นั้น)
  setsid socat -d "pty,link=$link,raw,echo=0,perm=0666,b115200" \
    "tcp:$HOST_IP:$3,nodelay,keepalive,keepidle=10,keepintvl=5,keepcnt=3" </dev/null >>"$log" 2>&1 &
  BRIDGE_PIDS+=($!)
  for _ in $(seq 1 20); do [ -e "$link" ] && return; sleep 0.25; done
  echo "✗ socat สร้าง $link ไม่ได้ — ดู $log"
  exit 1
}

stop_bridges() {
  [ ${#BRIDGE_PIDS[@]} -gt 0 ] && kill "${BRIDGE_PIDS[@]}" 2>/dev/null
  win_kill "serial_bridge_win\.py.*--tcp-port ($BRIDGE_TCP_LEFT|$BRIDGE_TCP_RIGHT)"
  rm -f /tmp/ttyV_left /tmp/ttyV_right
}

cmd="${1:-help}"
shift || true

case "$cmd" in
  status)
    echo "สถานะ (PC):"
    win=$(win_procs "$WIN_OURS")
    if [ -n "$win" ]; then
      echo "  (พอร์ตแขนถูกใช้อยู่ — ดูบอร์ดได้หลัง arm off ด้วย arm ports) โปรแกรมฝั่ง Windows ที่ถือพอร์ต:"
      sed 's/^/    /' <<< "$win"
    else
      winpy "$PORTS_PY" --list 2>&1 | tr -d '\r' | sed 's/^/  /'
    fi
    if is_running "ar_dual.*dual_hardware"; then
      hw=$(pgrep -af "ros2 launch ar_dual dual_hardware" | grep -o '[a-z]*_hw:=[a-z]*' | tr '\n' ' ')
      echo "  ✓ ระบบสองแขน      (arm start) ${hw:-left_hw:=real right_hw:=real}"
    else
      echo "  ✗ ระบบสองแขน      (arm start)"
    fi
    show "lib/ar_program/arm_program" "ตัวเล่นโปรแกรม     (arm player)"
    show move_group "MoveIt + RViz      (arm moveit)"
    show "lib/humanoi_games/rps_game" "เกมเป่ายิ้งฉุบ      (arm rps)"
    show "lib/humanoi_games/amazinghand_node" "ตัวคุมมือ (จำลอง)   (arm player / arm rps)"
    if is_running ros2_control_node; then
      echo "ตัวควบคุม:"
      timeout 10 ros2 control list_controllers 2>/dev/null | sed 's/^/  /'
    fi
    ;;

  ports)
    need_ports_free
    winpy "$PORTS_PY" --list
    ;;

  name)
    arm=$(arm_name "${2:-}")
    [ $# -ge 2 ] && [ -n "$arm" ] || { echo "ใช้: arm name <COMx จาก arm ports> <left|right>   (ตั้งชื่อแขนเก็บในบอร์ด ทำครั้งเดียว)"; exit 1; }
    need_ports_free
    winpy "$PORTS_PY" --name "$1" "$arm"
    ;;

  identify)  # ตั้งชื่อบอร์ดโดยดูว่าแขนไหนขยับ — แทนการถอดสาย USB (ห้ามถอด/เสียบขณะไฟแขนเปิด)
    need_ports_free
    if [ $# -ge 1 ]; then
      winpy "$PORTS_PY" --wiggle "$1"
    else
      winpy "$PORTS_PY" --identify
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
    need_ports_free
    resolve_ports
    port=$LEFT_TEENSY; [ "$arm" = right ] && port=$RIGHT_TEENSY
    need_port "$port" "$arm"
    winpy "$PORTS_PY" --dir "$port" "$2"
    ;;

  calibrate)
    arm=$(arm_name "${1:-}")
    [ -n "$arm" ] || { echo "ใช้: arm calibrate <left|right> [ลำดับข้อ เช่น 6 5 4 1 2 3 | all]   (ทีละแขน)"; exit 1; }
    shift
    need_ports_free
    resolve_ports
    port=$LEFT_TEENSY; [ "$arm" = right ] && port=$RIGHT_TEENSY
    th=ซ้าย; [ "$arm" = right ] && th=ขวา
    order=("$@")
    if [ ${#order[@]} -eq 0 ]; then
      default=$LEFT_CAL_ORDER; [ "$arm" = right ] && default=$RIGHT_CAL_ORDER
      if [ -z "$default" ]; then
        cat <<EOF
ยังไม่ได้ตั้งลำดับ calibrate ของแขน$th — ใส่ลำดับข้อเอง เช่น
  arm calibrate $arm 5            ข้อเดียว (ไล่ลองทีละข้อว่าข้อไหนพามือไปชน)
  arm calibrate $arm 6 5 4 1 2 3  ทีละข้อตามลำดับ
  arm calibrate $arm all          6 ข้อพร้อมกันแบบต้นฉบับ (มือที่ติดปลายแขนอาจชน)
ได้ลำดับที่ปลอดภัยแล้ว ตั้งเป็นค่าเริ่มได้ที่ LEFT_CAL_ORDER / RIGHT_CAL_ORDER ใน scripts/arm.sh (ใช้ทั้ง Pi และ PC)
EOF
        exit 1
      fi
      read -r -a order <<< "$default"
    fi
    need_port "$port" "$th"
    cat <<EOF
=== CALIBRATE แขน$th ($port) — ลำดับ: ${order[*]} ===
 • แต่ละข้อหมุนไปชนสวิตช์ ถอยออก แล้วกลับท่าพัก ก่อนไปข้อถัดไป (ข้อที่ไม่ได้ทำไม่ขยับ)
 • ดูให้แน่ใจว่าแขนที่ขยับคือแขน$th (ครั้งแรกหลังย้ายสาย) — ไม่ใช่ หรือมือจะชน ให้ปิดไฟแขนทันที
 • รอบแขนต้องโล่ง สาย USB หย่อนพ้นรัศมีแขน (J1 หมุนไปถึง −170°) อีกแขนต้องไม่อยู่ในทาง
 • ถ้า USB หลุดกลางทางไม่เป็นไร — แขนทำต่อเอง โปรแกรมจะรอจนเสร็จ
EOF
    read -r -p "พร้อมแล้วกด Enter (ยกเลิกกด Ctrl+C) " _
    winpy "$CAL_PY" "$port" "${order[@]}" \
      && echo "เสร็จแล้ว — calibrate ข้ออื่น/อีกแขน หรือเปิดระบบ: arm start"
    ;;

  start)
    need_no_system
    left_hw=real; right_hw=real
    case "${1:-}" in
      "") ;;
      left|l|1|ซ้าย) right_hw=mock ;;
      right|r|2|ขวา) left_hw=mock ;;
      sim|mock) left_hw=mock; right_hw=mock ;;
      *) echo "ใช้: arm start [left|right|sim]   (ไม่ใส่ = สองแขน, left/right = แขนเดียว อีกแขนจำลอง, sim = จำลองทั้งคู่)"; exit 1 ;;
    esac
    if [ "$left_hw" = real ] || [ "$right_hw" = real ]; then
      resolve_ports
      [ "$left_hw" = real ] && need_port "$LEFT_TEENSY" ซ้าย
      [ "$right_hw" = real ] && need_port "$RIGHT_TEENSY" ขวา
    fi
    trap stop_bridges EXIT
    trap 'exit 129' HUP TERM   # ปิดหน้าต่างทิ้ง -> ยังปิดสะพานให้
    trap ':' INT               # Ctrl+C ไปที่ ros2 launch — สคริปต์รอให้ระบบแขนปิดเสร็จก่อนค่อยปิดสะพาน
    left_port=none; right_port=none
    if [ "$left_hw" = real ]; then
      start_bridge left "$LEFT_TEENSY" $BRIDGE_TCP_LEFT; left_port=/tmp/ttyV_left
    fi
    if [ "$right_hw" = real ]; then
      start_bridge right "$RIGHT_TEENSY" $BRIDGE_TCP_RIGHT; right_port=/tmp/ttyV_right
    fi
    echo "เปิดระบบแขน (ซ้าย=$left_hw ${LEFT_TEENSY:+@ $LEFT_TEENSY} · ขวา=$right_hw ${RIGHT_TEENSY:+@ $RIGHT_TEENSY})"
    echo "ปล่อยหน้าต่างนี้เปิดไว้ ปิดด้วย Ctrl+C · แขนจริงต้อง calibrate แล้ว ไม่งั้นตำแหน่งที่ระบบเห็นจะผิด"
    ros2 launch ar_dual dual_hardware.launch.py left_port:="$left_port" right_port:="$right_port" \
      left_hw:=$left_hw right_hw:=$right_hw
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
    hand=false  # PC: มือจริงต่อฝั่ง Windows WSL มองไม่เห็น — ตัวคุมมือแค่ log ท่า
    echo "เปิดตัวเล่นโปรแกรม speed=$speed rad/s + ตัวคุมมือ (มือจริง: $hand) — ปล่อยหน้าต่างนี้เปิดไว้ ปิดด้วย Ctrl+C"
    ros2 launch humanoi_games player.launch.py speed:="$speed" hand:=false
    ;;

  hand)  # ลองท่ามือตรงๆ — ต้องมีตัวคุมมือเปิดอยู่ (arm player หรือ arm rps)
    names=$(ros2 run ar_program teach --hand-names 2>/dev/null | tr '\n' ' ')  # ท่าในตัว + ท่าที่สอนเอง
    g="${1:-}"; arm=$(arm_name "${2:-left}")
    if [ -z "$g" ] || [[ " $names " != *" $g "* ]] || [ -z "$arm" ]; then
      echo "ใช้: arm hand <ท่า> [left|right]   ท่า: $names"; exit 1
    fi
    topic=/hand/gesture; th=ซ้าย
    [ "$arm" = right ] && topic=/right_hand/gesture && th=ขวา
    # มีตัวคุมมือฟังอยู่ไหม (มือบนแขนขวายังไม่มี — หัวข้อนี้เผื่อมือที่สอง)
    if ! ros2 topic info "$topic" 2>/dev/null | grep -q "Subscription count: [1-9]"; then
      echo "✗ ไม่มีตัวคุมมือบนแขน$th ($topic) — เปิด arm player ก่อน"; exit 1
    fi
    timeout 10 ros2 topic pub --once -w 1 "$topic" std_msgs/msg/String "{data: $g}" >/dev/null 2>&1 \
      && echo "มือ$th -> $g" || echo "✗ ส่งไม่ถึงตัวคุมมือ ($topic)"
    ;;

  moveit)
    if ! is_running ros2_control_node; then
      echo "✗ เปิดระบบแขนก่อน (arm start หรือ arm start sim)"; exit 1
    fi
    if [ "${1:-}" = full ]; then
      ros2 launch ar_dual dual_moveit.launch.py
    else
      cat <<EOF
เปิด MoveIt + RViz (RViz เปิดว่างโดยตั้งใจ — โหลด config ด้วย -d บน WSLg แล้วเมาส์คลิกไม่ได้)
  ใน RViz: File > Open Config > $AR4_WS/install/ar_dual/share/ar_dual/config/dual_moveit.rviz
  หน้าต่าง RViz ย่อแล้วกดเปิดไม่ขึ้น / [WARN:COPY MODE]: ปิดทุกหน้าต่าง WSL แล้วสั่ง wsl --shutdown ใน PowerShell
EOF
      ros2 launch ar_dual dual_moveit.launch.py rviz_config:=none
    fi
    ;;

  hand-teach|hand-calibrate|hand-side|hand-scan|tray|tray-teach)
    echo "✗ arm $cmd ใช้บน Pi (มือต่ออยู่กับ Pi — บน PC มือเป็นแบบจำลอง)"
    exit 1
    ;;

  remote)
    echo "✗ arm remote ใช้ได้บน Pi เท่านั้น (Mega + ตัวรับรีโมทต่ออยู่กับ Pi)"
    exit 1
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
    bash -c "timeout 600 ros2 topic echo /arm/done --once --field data 2>/dev/null & E=\$!; sleep 1.5; \
      ros2 topic pub --once /arm/run std_msgs/msg/String \"{data: '$target'}\" >/dev/null; wait \$E" \
      | sed -n 's/^\(.*:ok\)$/  ✓ \1/p; s/^\(.*:\(error\|stopped\|rejected\).*\)$/  ✗ \1/p' \
      | grep . || echo "  ? ไม่ได้รับผลภายใน 10 นาที — ดูหน้าต่าง arm player"
    ;;

  rps)  # เกมเป่ายิ้งฉุบ (Humanoi/ros2_ws/src/humanoi_games) — คู่มือ docs/RPS_GAME.md
    case "${1:-}" in
      play)
        is_running "lib/humanoi_games/rps_game" || { echo "✗ เปิดเกมก่อน (อีกหน้าต่าง): arm rps"; exit 1; }
        echo "เป่า-ยิ้ง-ฉุบ! ... (หยุด: arm rps stop)"
        bash -c "timeout 180 ros2 topic echo /rps/result --once --field data 2>/dev/null & E=\$!; sleep 1.5; \
          ros2 topic pub --once /rps/play std_msgs/msg/Empty '{}' >/dev/null; wait \$E" | rps_result
        ;;
      stop)
        ros2 topic pub --once /rps/stop std_msgs/msg/Empty "{}" >/dev/null && echo "ส่งคำสั่งหยุดเกมแล้ว"
        ;;
      ""|random|topic|camera|*:=*)
        player=random; extra=()
        for a in "$@"; do
          case "$a" in
            random|topic|camera) player=$a ;;
            *:=*) extra+=("$a") ;;  # ส่งต่อให้ rps.launch.py เช่น pump_deg:=4
            *) echo "✗ ไม่รู้จัก '$a' — ดู: arm rps help"; exit 1 ;;
          esac
        done
        if [ "$player" = camera ]; then
          echo "✗ บน PC กล้องกับมือจริงอยู่ฝั่ง Windows — WSL มองไม่เห็น ใช้ทดสอบ: arm rps random / arm rps topic"
          exit 1
        fi
        if ! ros2 pkg prefix humanoi_games >/dev/null 2>&1; then
          echo "✗ ยังไม่ได้ build เกมใน WSL: cd $(realpath -m "$HERE/../ros2_ws") && colcon build --symlink-install --packages-select humanoi_games"
          exit 1
        fi
        is_running ros2_control_node || { echo "✗ เปิดระบบแขนก่อน (arm start sim หรือ arm start)"; exit 1; }
        if is_running "lib/ar_program/arm_program"; then
          echo "✗ ตัวเล่นโปรแกรมเปิดอยู่ (สั่งแขนเดียวกัน) — ปิดหน้าต่าง arm player ก่อน"; exit 1
        fi
        if is_running "lib/humanoi_games/rps_game"; then echo "✗ เกมเปิดอยู่แล้ว — เล่น: arm rps play"; exit 1; fi
        echo "เปิดเกมเป่ายิ้งฉุบ (ผู้เล่น: $player · มือจำลอง — log ท่าแทน ${extra[*]}) — ปล่อยหน้าต่างนี้เปิดไว้ ปิดด้วย Ctrl+C"
        echo "เล่น 1 ตา: arm rps play (อีกหน้าต่าง) · ต้องสอนท่าตั้งก่อน: arm teach left rps_ready"
        [ "$player" = topic ] && echo "ส่งท่าผู้เล่นเอง: ros2 topic pub -r 10 /rps/player_gesture std_msgs/msg/String \"{data: paper}\""
        ros2 launch humanoi_games rps.launch.py player:="$player" hand:=false "${extra[@]}"
        ;;
      *)
        cat <<'EOF'
ใช้: arm rps [random|topic] [ชื่อ:=ค่า ...]   เปิดเกมแบบทดสอบบน PC (เปิดค้าง) — random = สุ่มแทนผู้เล่น,
                             topic = ส่งท่าผู้เล่นเอง · ปรับได้ เช่น pump_deg:=4
     arm rps play             เล่น 1 ตา รอจนจบแล้วบอกผล
     arm rps stop             หยุดเกมกลางคัน
EOF
        exit 1 ;;
    esac
    ;;

  stop)
    ros2 topic pub --once /arm/stop std_msgs/msg/Empty "{}" >/dev/null && echo "ส่งคำสั่งหยุดแล้ว (ทุกแขน)"
    ;;

  list)
    ros2 run ar_program teach --list
    ;;

  show)
    [ $# -ge 1 ] || { echo "ใช้: arm show <ชื่อโปรแกรม>   (ดูเป็นตาราง ขั้น | แขนซ้าย | แขนขวา)"; exit 1; }
    ros2 run ar_program teach --show "$1"
    ;;

  teach)
    arm=$(arm_name "${1:-}")
    [ -n "$arm" ] && [ $# -ge 2 ] || { echo "ใช้: arm teach <left|right> <ชื่อท่า>   (บันทึกท่าปัจจุบันของแขนนั้น)"; exit 1; }
    ros2 run ar_program teach "$2" --arm "$arm"
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
    ros2 run ar_program teach --program "$@"
    ;;

  delete)
    arm=$(arm_name "${1:-}")
    [ -n "$arm" ] && [ $# -ge 2 ] || { echo "ใช้: arm delete <left|right> <ชื่อท่า>"; exit 1; }
    ros2 run ar_program teach --delete "$2" --arm "$arm"
    ;;

  delete-program)
    [ $# -ge 1 ] || { echo "ใช้: arm delete-program <ชื่อโปรแกรม>"; exit 1; }
    ros2 run ar_program teach --delete-program "$1"
    ;;

  off)
    echo "ปิดทุกอย่างของแขน (แขนค้างที่ท่าปัจจุบัน)..."
    for p in "ros2 launch ar_program" "ros2 launch ar_dual" "ros2 launch ar_gz_bringup" "ros2 launch humanoi_games"; do
      pkill -INT -f "$p" 2>/dev/null
    done
    for _ in $(seq 1 15); do
      is_running ros2_control_node || is_running move_group || is_running "lib/ar_program/arm_program" \
        || is_running "lib/humanoi_games" || break
      sleep 1
    done
    for p in ros2_control_node move_group lib/ar_program/arm_program spawner rviz2 lib/humanoi_games; do
      pkill -9 -f "$p" 2>/dev/null
    done
    pkill -f "socat .*pty,link=/tmp/ttyV_(left|right)" 2>/dev/null
    win_kill "$WIN_OURS"   # สะพาน serial / calibrate / ar_ports ฝั่ง Windows ที่ค้าง
    rm -f /tmp/ttyV_left /tmp/ttyV_right
    bash "$0" status
    ;;

  help|*)
    cat <<'EOF'
arm — คำสั่งลัดแขน AR4 สองตัว บน PC (รันใน WSL Ubuntu-24.04) · ชุดคำสั่งเดียวกับบน Pi
left = แขน 1 (ซ้ายเมื่อมองจากตัวหุ่น), right = แขน 2 · พอร์ตเป็น COM ของ Windows (หาเองจากชื่อแขนในบอร์ด)

 บอร์ดแขน
   arm ports                   ดูบอร์ด Teensy ทุกตัว: พอร์ต COM ชื่อแขน ข้อที่ตั้งกลับทิศ
   arm identify                ตั้งชื่อบอร์ด: ข้อ 6 หมุนไป-กลับ 10° ทีละบอร์ด ดูว่าแขนไหนขยับแล้วตอบ l / r
   arm identify <left|right>   หมุนข้อ 6 ของแขนชื่อนั้น เช็กว่าชื่อตรงกับแขนจริง
   arm name <COMx> <left|right>    ตั้งชื่อแขนให้บอร์ดตรงๆ
   arm dir <left|right> <J1..J6>   ตั้งข้อที่มอเตอร์หมุนกลับด้าน เช่น arm dir right 000010
   ห้ามถอด/เสียบ USB ของแขนขณะไฟแขนเปิด — ปิดไฟแขนก่อนเสมอ

 เริ่มใช้งาน (คำสั่งที่เปิดค้าง ให้ใช้หน้าต่างละคำสั่ง)
   arm status                  ดูว่าอะไรเปิดอยู่บ้าง
   arm calibrate <left|right> [ข้อ...]  calibrate ทีละแขน ทีละข้อตามลำดับ เช่น 6 5 4 1 2 3 (หรือ all)
   arm start [left|right|sim]  เปิดระบบแขน (หน้าต่าง 1 — เปิดค้าง) ไม่ใส่ = สองแขนจริง, sim = จำลองทั้งคู่
   arm player [speed]          เปิดตัวเล่นโปรแกรม + ตัวคุมมือจำลอง (หน้าต่าง 2 — เปิดค้าง) ค่าเริ่ม 0.19
   arm moveit                  MoveIt + RViz (RViz เปิดว่าง แล้ว File > Open Config ตามที่ขึ้นบอก)
   arm moveit full             เปิดพร้อม config เลย (บน WSLg เมาส์อาจคลิกในหน้าต่างไม่ได้)

 สอนท่า / สร้างโปรแกรม (หน้าต่างไหนก็ได้) — เก็บใน programs.yaml บน PC (คนละไฟล์กับบน Pi)
   arm teach <left|right> <ท่า>       บันทึกท่าปัจจุบันของแขนนั้น
   arm program <ชื่อ> <ขั้น...>        แต่ละขั้น <แขนซ้าย>/<แขนขวา>[/<มือซ้าย>[/<มือขวา>]] - = ไม่ขยับ, wait=วินาที
                                      ท่ามือ: rock paper scissors point like windup victory nonono relax
   arm show <ชื่อ> · arm list · arm delete <left|right> <ท่า> · arm delete-program <ชื่อ>

 เล่น
   arm run <โปรแกรม> · arm goto <left|right> <ท่า> · arm stop
   arm hand <ท่า> [left|right]        ลองท่ามือ (ต้องเปิด arm player อยู่ — บน PC มือแค่ log ท่า)

 เกมเป่ายิ้งฉุบ (ทดสอบบน PC: มือจำลอง ไม่มีกล้อง — คู่มือ docs/RPS_GAME.md)
   arm rps [random|topic]             เปิดเกม (เปิดค้าง) · arm rps play = เล่น 1 ตา · arm rps stop = หยุด

 ปิด
   arm off                            ปิดทุกอย่างของแขน รวมสะพาน serial ฝั่ง Windows

 ไม่มีบน PC: arm remote (รีโมท FlySky ต่อกับ Mega บน Pi)
 ไม่มีการเช็คการชนตอน arm run/goto (รวมสองแขนชนกันเอง) — ทดสอบโปรแกรมใหม่ด้วย speed ต่ำก่อนเสมอ
EOF
    ;;
esac
