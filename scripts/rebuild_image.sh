#!/usr/bin/env bash
# rebuild_image.sh — สร้าง Docker image humanoi-ros:jazzy ใหม่จาก docker/Dockerfile (รันบน Pi นอก container)
# ทำให้ของที่เคยติดตั้งเข้า container ตอนรัน (MoveIt/RViz, ไลบรารีมือ/เกม, teensy-loader) ติดถาวร
# — ตอนนี้ถ้า container ถูกสร้างใหม่ (docker compose up หลังแก้ compose, Pi ล้าง container) ของพวกนั้นหายหมด
#
#   bash ~/Humanoi/scripts/rebuild_image.sh check     เช็กอย่างเดียว: ดิสก์ / อะไรรันอยู่ / Dockerfile ตรงกับ PC
#   bash ~/Humanoi/scripts/rebuild_image.sh build     สร้าง image ใหม่ (container เดิมรันต่อได้ ไม่กระทบระบบ)
#                                                     แล้วเทียบแพ็กเกจ: มีใน container ตอนนี้ แต่ image ใหม่ไม่มี
#   bash ~/Humanoi/scripts/rebuild_image.sh switch    สลับ container ไป image ใหม่ (ระบบแขน/ฐานต้องปิด: arm off)
#   bash ~/Humanoi/scripts/rebuild_image.sh rollback  กลับไป image เดิม (เก็บไว้ชื่อ humanoi-ros:jazzy-prev)
#
# ros2_ws (โค้ด + ผล build) อยู่บน Pi ผ่าน volume — สลับ container แล้วไม่ต้อง build ใหม่
set -uo pipefail
cd ~/Humanoi || exit 1

C=humanoi_ros
IMG=humanoi-ros:jazzy
PREV=humanoi-ros:jazzy-prev
NEED_GB=4      # ชั้นที่สร้างใหม่ + เผื่อ (image เดิม 3.6 GB ส่วนใหญ่ใช้ cache ร่วม)
LOG=~/Humanoi/docker_build.log

if docker compose version >/dev/null 2>&1; then DC="docker compose"; else DC="docker-compose"; fi

busy() {  # โปรแกรม ROS ที่รันอยู่ใน container (ไม่นับ ros2 daemon)
  docker exec "$C" pgrep -af "ros2 launch|ros2 run|lib/[a-z_]+/" 2>/dev/null | grep -v "ros2cli.daemon\|pgrep" || true
}

packages() {  # packages <container|image> <ชื่อ> — รายชื่อ apt + pip (เรียงแล้ว)
  local q='dpkg-query -W -f "apt \${Package}\n"; pip3 list --format=freeze 2>/dev/null | sed "s/==.*//; s/^/pip /"'
  if [ "$1" = container ]; then
    docker exec "$2" sh -c "$q" | sort -u
  else
    docker run --rm --entrypoint sh "$2" -c "$q" | sort -u
  fi
}

check() {
  local ok=0 free
  free=$(df -BG --output=avail / | tail -1 | tr -dc 0-9)
  if [ "$free" -lt "$NEED_GB" ]; then
    echo "✗ ดิสก์เหลือ ${free} GB (ต้อง ≥ ${NEED_GB} GB) — ลบของไม่ใช้ เช่น docker builder prune / core dump / log"; ok=1
  else
    echo "✓ ดิสก์เหลือ ${free} GB"
  fi
  if grep -q "rustypot" docker/Dockerfile; then
    echo "✓ Dockerfile มีชั้นไลบรารีมือ/เกม (ตรงกับ PC)"
  else
    echo "✗ Dockerfile บน Pi ยังเป็นรุ่นเก่า (ไม่มีชั้น rustypot/mediapipe) — ส่ง docker/Dockerfile จาก PC มาก่อน"; ok=1
  fi
  local b; b=$(busy)
  if [ -n "$b" ]; then echo "! มีโปรแกรมรันอยู่ (build ได้ แต่ switch ต้องปิดก่อน):"; echo "$b" | cut -c1-100 | sed 's/^/    /'
  else echo "✓ ไม่มีโปรแกรม ROS รันอยู่"; fi
  return $ok
}

case "${1:-check}" in
  check)
    check
    ;;

  build)
    check || exit 1
    if docker image inspect "$IMG" >/dev/null 2>&1; then
      docker tag "$IMG" "$PREV" && echo "เก็บ image เดิมไว้เป็น $PREV (rollback ได้)"
    fi
    echo "สร้าง image ใหม่ (ชั้นที่ไม่เปลี่ยนใช้ cache) — log: $LOG"
    if ! $DC build > "$LOG" 2>&1; then
      echo "✗ build ไม่ผ่าน — ดูท้าย $LOG:"; tail -15 "$LOG"; exit 1
    fi
    echo "✓ สร้างแล้ว: $(docker image inspect "$IMG" --format '{{.Id}}' | cut -c8-19)"
    echo "เทียบแพ็กเกจ: มีใน container ตอนนี้ แต่ image ใหม่ไม่มี (= เคยติดตั้งตอนรัน และ Dockerfile ยังไม่ครอบคลุม)"
    missing=$(comm -23 <(packages container "$C") <(packages image "$IMG"))
    if [ -n "$missing" ]; then
      echo "$missing" | sed 's/^/    /'
      echo "! ถ้าในนี้มีของที่ใช้อยู่ ให้เพิ่มใน Dockerfile แล้ว build ใหม่ก่อน switch"
    else
      echo "✓ ไม่มีอะไรหาย — switch ได้ (ตอนระบบปิด)"
    fi
    ;;

  switch|rollback)
    b=$(busy)
    if [ -n "$b" ]; then echo "✗ มีโปรแกรมรันอยู่ — arm off ก่อน:"; echo "$b" | cut -c1-100; exit 1; fi
    if [ "$1" = rollback ]; then
      docker image inspect "$PREV" >/dev/null 2>&1 || { echo "✗ ไม่มี $PREV"; exit 1; }
      docker tag "$PREV" "$IMG"
    fi
    $DC up -d --force-recreate --no-build || exit 1
    sleep 3
    echo "ตรวจหลังสลับ:"
    docker exec "$C" /entrypoint.sh bash -c '
      for p in moveit_ros_move_group rviz2 controller_manager astra_camera; do
        ros2 pkg prefix $p >/dev/null 2>&1 && echo "  ✓ $p" || echo "  ✗ $p"; done
      python3 -c "import rustypot, mediapipe, matplotlib" 2>/dev/null && echo "  ✓ rustypot mediapipe matplotlib" || echo "  ✗ python libs"
      command -v teensy_loader_cli >/dev/null && echo "  ✓ teensy_loader_cli" || echo "  ✗ teensy_loader_cli"
      ros2 run humanoi_mission mission < /dev/null >/dev/null 2>&1 && echo "  ✓ workspace (humanoi_mission)" || echo "  ✗ workspace"'
    ;;

  *)
    sed -n 2,12p "$0"; exit 1 ;;
esac
