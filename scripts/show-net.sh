#!/bin/bash
# show-net — เตรียมเครือข่ายของ Pi สำหรับงานโชว์ (เน็ตในงานล่ม / Pi Connect ใช้ไม่ได้ ก็ยังเข้า Pi ได้)
# รันบน Pi นอก container (ใช้ sudo — ระบบถามรหัสผ่านเอง) · รายละเอียด: docs/SHOW.md หัวข้อเครือข่าย
#
#   bash ~/Humanoi/scripts/show-net.sh status                 ดูว่าต่ออะไรอยู่ IP อะไร
#   bash ~/Humanoi/scripts/show-net.sh router "<SSID>"        จำเราเตอร์พกพา (2.4 GHz) ให้ต่อก่อน WiFi อื่น
#   bash ~/Humanoi/scripts/show-net.sh hotspot-setup          สร้าง hotspot ของ Pi เอง (Humanoi-Show, 2.4 GHz)
#   bash ~/Humanoi/scripts/show-net.sh hotspot on|off         เปิด/ปิด hotspot ตอนนี้ (เปิด = หลุดจาก WiFi อื่น)
#   bash ~/Humanoi/scripts/show-net.sh fallback-install       เปิดเครื่องแล้ว 90 วิ ยังไม่ได้ WiFi = เปิด hotspot เอง
#   bash ~/Humanoi/scripts/show-net.sh fallback-remove        เลิกใช้ข้อบน
#   bash ~/Humanoi/scripts/show-net.sh direct-setup           สาย LAN ตรงโน้ตบุ๊ก ↔ Pi: Pi = 169.254.77.1 เสมอ
#   bash ~/Humanoi/scripts/show-net.sh direct-remove          เลิกใช้ข้อบน (สาย LAN กลับไปขอ IP จากเราเตอร์)
#
# รหัส WiFi พิมพ์ตอนสคริปต์ถาม (ไม่ขึ้นจอ ไม่อยู่ในประวัติคำสั่ง) เก็บใน /etc/NetworkManager/system-connections (root อ่านได้คนเดียว)
set -euo pipefail

NM_DIR=/etc/NetworkManager/system-connections
ROUTER=humanoi-router
HOTSPOT=humanoi-hotspot
HOTSPOT_SSID=Humanoi-Show
DIRECT=humanoi-direct
DIRECT_IP=169.254.77.1
FALLBACK=humanoi-net-fallback

uuid() { cat /proc/sys/kernel/random/uuid; }

ask_psk() {  # ask_psk <ข้อความ> -> ตัวแปร PSK
  local a b
  while true; do
    read -r -s -p "$1 (8 ตัวขึ้นไป): " a; echo
    read -r -s -p "พิมพ์ซ้ำ: " b; echo
    [ "$a" = "$b" ] || { echo "✗ ไม่ตรงกัน ลองใหม่"; continue; }
    [ ${#a} -ge 8 ] || { echo "✗ สั้นกว่า 8 ตัว"; continue; }
    PSK=$(printf '%s' "$a" | sed 's/\\/\\\\/g'); return  # keyfile อ่าน \ เป็นตัวหนี -> เขียนเป็น \\
  done
}

write_profile() {  # write_profile <ชื่อ> < เนื้อไฟล์ keyfile
  local f="$NM_DIR/$1.nmconnection"
  sudo -v  # ถามรหัส sudo ก่อน (stdin เป็นเนื้อไฟล์)
  sudo tee "$f" >/dev/null
  sudo chmod 600 "$f"
  sudo nmcli connection reload
  echo "✓ บันทึก $1 ($f)"
}

case "${1:-status}" in
  status)
    echo "การเชื่อมต่อที่ใช้อยู่:"
    nmcli -t -f NAME,DEVICE connection show --active | grep -v -e '^lo:' -e docker | sed 's/:/  @ /; s/^/  /'
    echo "IP:"
    ip -4 -o addr show | awk '$2 != "lo" && $2 !~ /^docker|^veth|^br-/ {print "  " $2 "  " $4}'
    echo "ชื่อเครื่อง: $(hostname).local"
    for p in "$ROUTER" "$HOTSPOT" "$DIRECT"; do
      nmcli -t -f NAME connection show | grep -qx "$p" && echo "  มีโปรไฟล์ $p" || echo "  - ยังไม่มี $p"
    done
    systemctl is-enabled "$FALLBACK" >/dev/null 2>&1 && echo "  fallback hotspot ตอนเปิดเครื่อง: เปิดใช้" \
      || echo "  - fallback hotspot ตอนเปิดเครื่อง: ไม่ได้ใช้"
    ;;

  router)
    ssid="${2:-}"
    [ -n "$ssid" ] || { echo "ใช้: show-net.sh router \"<ชื่อ WiFi ของเราเตอร์พกพา>\""; exit 1; }
    ask_psk "รหัส WiFi ของ '$ssid'"
    write_profile "$ROUTER" <<EOF
[connection]
id=$ROUTER
uuid=$(uuid)
type=wifi
autoconnect=true
autoconnect-priority=100

[wifi]
mode=infrastructure
ssid=$ssid

[wifi-security]
key-mgmt=wpa-psk
psk=$PSK

[ipv4]
method=auto

[ipv6]
method=auto
EOF
    unset PSK
    echo "  เจอ '$ssid' เมื่อไหร่ Pi ต่อตัวนี้ก่อน WiFi อื่น (priority 100) · ต่อเลยตอนนี้: sudo nmcli connection up $ROUTER"
    ;;

  hotspot-setup)
    ask_psk "ตั้งรหัส hotspot '$HOTSPOT_SSID'"
    write_profile "$HOTSPOT" <<EOF
[connection]
id=$HOTSPOT
uuid=$(uuid)
type=wifi
autoconnect=false

[wifi]
mode=ap
band=bg
channel=6
ssid=$HOTSPOT_SSID

[wifi-security]
key-mgmt=wpa-psk
proto=rsn
pairwise=ccmp
group=ccmp
psk=$PSK

[ipv4]
method=shared
address1=10.42.0.1/24

[ipv6]
method=disabled
EOF
    unset PSK
    echo "  เปิด: show-net.sh hotspot on · โน้ตบุ๊กต่อ WiFi '$HOTSPOT_SSID' แล้ว ssh $USER@10.42.0.1"
    ;;

  hotspot)
    case "${2:-}" in
      on)
        echo "! เปิด hotspot = wlan0 หลุดจาก WiFi ที่ต่ออยู่ — ถ้าเข้ามาทาง WiFi นี้ หน้าต่างนี้จะหลุด ต่อใหม่ที่ $HOTSPOT_SSID"
        sudo nmcli connection up "$HOTSPOT" ;;
      off)
        sudo nmcli connection down "$HOTSPOT" && echo "✓ ปิด hotspot — Pi กลับไปต่อ WiFi ที่จำไว้เอง" ;;
      *) echo "ใช้: show-net.sh hotspot on|off"; exit 1 ;;
    esac
    ;;

  fallback-install)
    nmcli -t -f NAME connection show | grep -qx "$HOTSPOT" || { echo "✗ สร้าง hotspot ก่อน: show-net.sh hotspot-setup"; exit 1; }
    sudo -v
    sudo tee "/usr/local/bin/$FALLBACK" >/dev/null <<EOF
#!/bin/sh
# Humanoi: เปิดเครื่องแล้ว 90 วิ wlan0 ยังไม่ได้ต่อ WiFi ไหน -> เปิด hotspot $HOTSPOT_SSID (scripts/show-net.sh)
sleep 90
nmcli -t -f DEVICE,STATE device | grep -q '^wlan0:connected' && exit 0
logger -t $FALLBACK "no WiFi after boot - starting hotspot $HOTSPOT_SSID"
nmcli connection up $HOTSPOT
EOF
    sudo chmod 755 "/usr/local/bin/$FALLBACK"
    sudo tee "/etc/systemd/system/$FALLBACK.service" >/dev/null <<EOF
[Unit]
Description=Humanoi: no WiFi 90 s after boot -> Pi hotspot $HOTSPOT_SSID
After=NetworkManager.service
Wants=NetworkManager.service

[Service]
Type=simple
ExecStart=/usr/local/bin/$FALLBACK

[Install]
WantedBy=multi-user.target
EOF
    sudo systemctl daemon-reload
    sudo systemctl enable "$FALLBACK"
    echo "✓ ทุกครั้งที่เปิดเครื่อง: 90 วิแล้วไม่ได้ WiFi = เปิด hotspot '$HOTSPOT_SSID' (10.42.0.1)"
    ;;

  fallback-remove)
    sudo systemctl disable --now "$FALLBACK" 2>/dev/null || true
    sudo rm -f "/etc/systemd/system/$FALLBACK.service" "/usr/local/bin/$FALLBACK"
    sudo systemctl daemon-reload
    echo "✓ เลิก fallback hotspot แล้ว"
    ;;

  direct-setup)
    write_profile "$DIRECT" <<EOF
[connection]
id=$DIRECT
uuid=$(uuid)
type=ethernet
interface-name=eth0
autoconnect=true
autoconnect-priority=10

[ethernet]

[ipv4]
method=manual
address1=$DIRECT_IP/16
never-default=true

[ipv6]
method=link-local
EOF
    echo "  เสียบสาย LAN ตรงโน้ตบุ๊ก (Windows ได้ IP 169.254.x.x เองใน ~1 นาที) แล้ว ssh $USER@$DIRECT_IP"
    echo "  ! สาย LAN เข้าเราเตอร์จะไม่ได้ IP จากเราเตอร์ — ใช้ WiFi แทน หรือ show-net.sh direct-remove"
    ;;

  direct-remove)
    sudo nmcli connection delete "$DIRECT" && echo "✓ ลบ $DIRECT — สาย LAN กลับไปขอ IP จากเราเตอร์ (Wired connection 1)"
    ;;

  *)
    sed -n '2,15p' "$0" | sed 's/^# \{0,1\}//'
    exit 1
    ;;
esac
