"""ประกอบคู่มือ Humanoi เป็น HTML ไฟล์เดียว (A4 พิมพ์ได้) -> docs/manual/HUMANOI_MANUAL.html

  python docs/manual/src/build_manual.py

ต้นฉบับแต่ละบทอยู่ในโฟลเดอร์นี้ (part_*.html) แก้ที่นี่แล้วรันใหม่ — อย่าแก้ HUMANOI_MANUAL.html ตรงๆ
ภาคผนวกคำสั่ง (arm help) อ่านจาก scripts/arm.sh ทุกครั้งที่รัน · รูป: วางใน docs/manual/img/ ไม่ต้องรันใหม่
"""
import html
import os
import re
import sys

S = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(S, "..", "..", ".."))
OUT = os.path.join(REPO, "docs", "manual", "HUMANOI_MANUAL.html")
VERSION = "ฉบับร่าง · 29 ตุลาคม 2026"


def read(name):
    with open(os.path.join(S, name), encoding="utf-8") as f:
        return f.read()


def split_trouble(text):
    if "<!-- TROUBLE -->" in text:
        body, rows = text.split("<!-- TROUBLE -->", 1)
        return body, rows.strip()
    return text, ""


main_a, _ = split_trouble(read("part_main_a.html"))
main_b, rows_main = split_trouble(read("part_main_b.html"))
arm, rows_arm = split_trouble(read("part_arm.html"))
misc, rows_misc = split_trouble(read("part_misc.html"))


def chapters(text):
    """{id: section html}"""
    return {m.group(1): m.group(0) for m in re.finditer(r'<section class="chapter" id="([^"]+)">.*?</section>', text, re.S)}


ch = {}
for t in (main_a, main_b, arm, misc):
    ch.update(chapters(t))
ORDER = ["ch-overview", "ch-safety", "ch-pi", "ch-startup", "ch-arm", "ch-teach", "ch-hand", "ch-tray",
         "ch-base", "ch-lidar", "ch-guard", "ch-mission", "ch-rps"]
missing = [c for c in ORDER if c not in ch]
extra = [c for c in ch if c not in ORDER]
if missing or extra:
    sys.exit(f"missing {missing} extra {extra}")

# ---------------------------------------------------------------- แก้ปัญหา
trouble = f"""<section class="chapter" id="ch-trouble"><h1>แก้ปัญหา</h1>
<p>อาการที่เคยเจอ เรียงตามส่วนของหุ่น ถ้าไม่เจอในนี้ ดูข้อความในหน้าต่างของคำสั่งนั้นก่อน — ส่วนใหญ่บอกสาเหตุและวิธีแก้ไว้แล้ว</p>
<h2>ฐานล้อ รีโมท lidar</h2>
<table class="trouble"><thead><tr><th>อาการ</th><th>สาเหตุที่น่าจะเป็น</th><th>ทำอะไร</th></tr></thead><tbody>
{rows_main}
</tbody></table>
<h2>แขน และโปรแกรม</h2>
<table class="trouble"><thead><tr><th>อาการ</th><th>สาเหตุที่น่าจะเป็น</th><th>ทำอะไร</th></tr></thead><tbody>
{rows_arm}
</tbody></table>
<h2>Pi มือ กล้อง เกม</h2>
<table class="trouble"><thead><tr><th>อาการ</th><th>สาเหตุที่น่าจะเป็น</th><th>ทำอะไร</th></tr></thead><tbody>
{rows_misc}
</tbody></table>
</section>"""

# ---------------------------------------------------------------- ภาคผนวก: คำสั่งทั้งหมด (จาก arm help)
arm_sh = open(os.path.join(REPO, "scripts", "arm.sh"), encoding="utf-8").read()
help_txt = re.search(r"help_text\(\) \{\n  cat <<'EOF'\n(.*?)\nEOF\n\}", arm_sh, re.S).group(1)
parts = []
rows = []


def flush():
    if rows:
        parts.append("<table class='cmdref'><tbody>" + "".join(rows) + "</tbody></table>")
        rows.clear()


for ln in help_txt.splitlines():
    if ln.startswith("## "):
        flush()
        key, title = ln[3:].split(" | ", 1)
        parts.append(f"<h2>{html.escape(title)} <span class='key'>arm help {html.escape(key.split()[0])}</span></h2>")
    elif ln.startswith("   arm "):
        m = re.match(r"^   (arm .*?\S)\s{2,}(\S.*)$", ln)
        cmd, desc = (m.group(1), m.group(2)) if m else (ln.strip(), "")
        rows.append(f"<tr><td><code>{html.escape(cmd)}</code></td><td>{html.escape(desc)}</td></tr>")
    elif ln.startswith("   ! "):
        rows.append(f"<tr class='w'><td colspan='2'>⚠ {html.escape(ln[5:])}</td></tr>")
    elif ln.strip() and rows:  # บรรทัดต่อของคำอธิบาย
        rows[-1] = rows[-1].replace("</td></tr>", "<br>" + html.escape(ln.strip()) + "</td></tr>")
flush()
cmdref = ('<section class="chapter appendix" id="ap-commands"><h1>คำสั่ง arm ทั้งหมด</h1>'
          "<p>ตารางนี้สร้างจาก <code>arm help</code> ตอนทำคู่มือ ถ้าไม่ตรงกับเครื่อง ให้เชื่อ <code>arm help</code> บน Pi</p>"
          + "".join(parts) + "</section>")

files = """<section class="chapter appendix" id="ap-files"><h1>ไฟล์ตั้งค่าและตำแหน่งไฟล์</h1>
<p>บน Pi โฟลเดอร์ <code>~/Humanoi</code> (ไม่ใช่ git) — ใน container เห็น <code>ros2_ws</code> เป็น <code>/humanoi_ws</code>
แก้โค้ดที่ PC แล้วส่งขึ้น Pi ส่วนไฟล์ที่โปรแกรมบนหุ่นเขียนเอง (ท่าที่สอน ค่าที่บันทึก) ต้องคัดลอกกลับมาเก็บที่ PC ก่อน commit</p>
<table><thead><tr><th>ไฟล์</th><th>เก็บอะไร</th><th>ใครเขียน</th></tr></thead><tbody>
<tr><td><code>ros2_ws/src/humanoi_base/config/base.yaml</code></td><td>ฐานล้อ: ขนาดล้อ encoder_map กันชน (guard.*)</td><td><code>arm wheel-check</code> · แก้มือ</td></tr>
<tr><td><code>ros2_ws/src/humanoi_base/config/lidar_mount.yaml</code></td><td>ตำแหน่ง lidar บนฐาน (x y z yaw_deg)</td><td><code>arm lidar-check</code> (yaw) · แก้มือ</td></tr>
<tr><td><code>ros2_ws/src/humanoi_mission/config/missions.yaml</code></td><td>ภารกิจ + ท่าพับแขน (stow)</td><td>แก้มือ (ไม่ต้อง build)</td></tr>
<tr><td><code>ros2_ws/src/ar_program/config/programs.yaml</code></td><td>ท่าแขน ท่ามือ และโปรแกรม (repo ar4: <code>ar4_ws/src/ar_program/config/</code>)</td><td><code>arm teach</code> · <code>arm program</code> · <code>arm hand-teach</code></td></tr>
<tr><td><code>ar4_ws/src/ar_dual/config/zero_trims.yaml</code></td><td>ค่าชดเชยศูนย์ข้อต่อของแต่ละแขน</td><td>แก้มือ แล้ว <code>arm zero</code></td></tr>
<tr><td><code>ros2_ws/src/humanoi_games/config/hand_boards.yaml</code></td><td>บอร์ดมือไหนเป็นมือซ้าย/ขวา</td><td><code>arm hand-side</code></td></tr>
<tr><td><code>ros2_ws/src/humanoi_games/config/middle_pos*.json</code></td><td>ค่ากลางนิ้วของมือแต่ละข้าง</td><td><code>arm hand-calibrate</code></td></tr>
<tr><td><code>firmware/humanoi_base_mega/</code></td><td>เฟิร์มแวร์ Arduino Mega (+ <code>guard.h</code>)</td><td>แฟลชจาก PC</td></tr>
<tr><td><code>tools/mega_sim/run_tests.sh</code></td><td>ทดสอบเฟิร์มแวร์ Mega + กันชน โดยไม่ใช้ฮาร์ดแวร์</td><td>—</td></tr>
<tr><td><code>scripts/arm.sh</code></td><td>คำสั่ง <code>arm</code> (alias ใน <code>~/.bashrc</code> ของ Pi)</td><td>—</td></tr>
<tr><td><code>docs/</code></td><td>เอกสารแยกเรื่อง (คู่มือเล่มนี้รวมมาจากที่นี่)</td><td>—</td></tr>
</tbody></table>
<h2>ข้อความระหว่าง Pi กับ Mega</h2>
<p>บรรทัดละคำสั่ง ผ่าน USB 115200 — ดูสดได้จาก topic <code>/motor_state</code> หรือ Serial Monitor (ตอน Mega ต่อ PC)</p>
<table><thead><tr><th>ทาง</th><th>ข้อความ</th><th>ความหมาย</th></tr></thead><tbody>
<tr><td>Pi → Mega</td><td><code>W F B S F</code></td><td>ทิศ 4 ล้อ FL FR RL RR (F หน้า B ถอย S หยุด) ส่งซ้ำ 20 ครั้ง/วิ</td></tr>
<tr><td>Pi → Mega</td><td><code>G 5</code> · <code>G OFF</code></td><td>กันชน: บิตทิศที่ห้าม 1 หน้า 2 หลัง 4 ซ้าย 8 ขวา 16 หมุน · ปิดกันชน</td></tr>
<tr><td>Pi → Mega</td><td><code>S</code> · <code>P</code> · <code>Z</code></td><td>หยุดทุกล้อ · ping (ตอบ OK) · ล้างค่านับ encoder</td></tr>
<tr><td>Mega → Pi</td><td><code>READY humanoi_base_mega enc=a2</code></td><td>บูตเสร็จ + วิธีนับ encoder</td></tr>
<tr><td>Mega → Pi</td><td><code>ENC t c1 c2 c3 c4 e1 e2 e3 e4</code></td><td>ค่านับ encoder ทุก 50 ms</td></tr>
<tr><td>Mega → Pi</td><td><code>MODE:ROS</code> · <code>STATE:FFFF</code></td><td>ใครคุมล้อ · ทิศล้อตอนนี้ (เมื่อเปลี่ยน)</td></tr>
<tr><td>Mega → Pi</td><td><code>GUARD:ON F---T</code> · <code>GUARD:HOLD …</code> · <code>GUARD:LOST</code></td><td>สถานะกันชน</td></tr>
<tr><td>Mega → Pi</td><td><code>SW:BUDD</code></td><td>ตำแหน่ง SWB / SWD (U ขึ้น D ลง)</td></tr>
<tr><td>Mega → Pi</td><td><code>RC CH1 1500 … | ROS | FL S … G:-----</code></td><td>บรรทัดสถานะทุก 0.5 วิ</td></tr>
</tbody></table>
</section>"""

# ---------------------------------------------------------------- ประกอบ + เลขรูป
body_chapters = [ch[c] for c in ORDER] + [trouble]
figs = []
for n, sec in enumerate(body_chapters, 1):
    title = re.search(r"<h1>(.*?)</h1>", sec).group(1)
    k = 0
    for m in re.finditer(r'<figure class="ph" data-kind="(\w+)">(.*?)</figure>', sec, re.S):
        k += 1
        inner = m.group(2)
        src = re.search(r'<img src="([^"]+)"', inner)
        cap = re.search(r"<figcaption>(.*?)</figcaption>", inner, re.S)
        note = re.search(r'<div class="ph-note">(.*?)</div>', inner, re.S)
        if src and src.group(1).startswith("img/"):
            figs.append((f"{n}.{k}", src.group(1), m.group(1), re.sub("<[^>]+>", "", cap.group(1)) if cap else "",
                         re.sub("<[^>]+>", "", note.group(1)) if note else "", title))

KIND = {"photo": "รูปถ่าย", "screenshot": "ภาพหน้าจอ", "diagram": "แผนผัง"}
fig_rows = "".join(
    f"<tr><td>{num}</td><td><code>{html.escape(src)}</code></td><td>{KIND.get(kind, kind)}</td>"
    f"<td>{cap}</td><td>{note}</td></tr>" for num, src, kind, cap, note, _ in figs)
figlist = f"""<section class="chapter appendix" id="ap-figures"><h1>รายการรูปที่ต้องเตรียม</h1>
<p>วางไฟล์รูปไว้ในโฟลเดอร์ <code>docs/manual/img/</code> ตามชื่อในตาราง (ใช้ <code>.jpg</code> หรือ <code>.png</code> ก็ได้)
แล้วเปิดคู่มือใหม่ — รูปจะขึ้นแทนช่องเส้นประเอง ไม่ต้องแก้ไฟล์คู่มือ · ทั้งหมด {len(figs)} รูป</p>
<table class="figlist"><thead><tr><th>รูป</th><th>ไฟล์</th><th>ชนิด</th><th>คำบรรยาย</th><th>ถ่าย/วาดอะไร</th></tr></thead><tbody>
{fig_rows}</tbody></table></section>"""

appendices = [cmdref, files, figlist]

# สารบัญ
toc_items = []
for n, sec in enumerate(body_chapters, 1):
    cid = re.search(r'id="([^"]+)"', sec).group(1)
    title = re.search(r"<h1>(.*?)</h1>", sec).group(1)
    subs = re.findall(r"<h2>(.*?)</h2>", sec)
    sub_html = "".join(f"<li>{re.sub('<[^>]+>', '', s)}</li>" for s in subs)
    toc_items.append(f'<li><a href="#{cid}"><span class="tn">{n}</span>{title}</a><ul>{sub_html}</ul></li>')
for i, sec in enumerate(appendices):
    cid = re.search(r'id="([^"]+)"', sec).group(1)
    title = re.search(r"<h1>(.*?)</h1>", sec).group(1)
    toc_items.append(f'<li class="ap"><a href="#{cid}"><span class="tn">{"ABC"[i]}</span>{title}</a></li>')
toc = '<section class="toc"><h1>สารบัญ</h1><ol>' + "".join(toc_items) + "</ol></section>"

todo_count = sum(s.count('class="todo"') for s in body_chapters)

CSS = r"""
:root{--ink:#1a1d1c;--muted:#5b6662;--accent:#0f766e;--accent-soft:#e6f4f1;--line:#d4dbd9;--paper:#fff;--bg:#e7ebea;}
*{box-sizing:border-box}
html{background:var(--bg)}
body{margin:0;font-family:"Sarabun","IBM Plex Sans Thai","Leelawadee UI","Tahoma",sans-serif;font-size:11pt;line-height:1.65;color:var(--ink);
  counter-reset:chapter appendix}
.book{max-width:210mm;margin:16px auto;background:var(--paper);box-shadow:0 1px 6px rgba(0,0,0,.12)}
.page,.chapter,.toc{padding:16mm 16mm 14mm}
code,kbd,pre{font-family:"IBM Plex Mono","Consolas","Menlo",monospace}
code{font-size:.9em;background:#f1f4f3;border-radius:3px;padding:0 .25em}
kbd{font-size:.82em;border:1px solid #b9c2bf;border-bottom-width:2px;border-radius:4px;padding:0 .35em;background:#fafbfb}
a{color:var(--accent);text-decoration:none}
h1,h2,h3{line-height:1.3;font-weight:600}
h2{font-size:15pt;color:var(--accent);border-bottom:1.5px solid var(--accent-soft);padding-bottom:3px;margin:22px 0 8px;break-after:avoid}
h3{font-size:12pt;margin:16px 0 6px;break-after:avoid}
p{margin:6px 0 8px}
ul,ol{margin:6px 0 10px;padding-left:22px}
li{margin:3px 0}
/* ปก */
.cover{min-height:277mm;display:flex;flex-direction:column;justify-content:space-between;padding:22mm 18mm 18mm;
  background:linear-gradient(#0f766e,#0f766e) top/100% 9mm no-repeat}
.cover .eyebrow{font-size:11pt;letter-spacing:.08em;color:var(--accent);font-weight:600;margin-top:14mm}
.cover h1{font-size:34pt;margin:6px 0 4px;line-height:1.15}
.cover .sub{font-size:13pt;color:var(--muted)}
.cover figure{margin:10mm 0}
.cover figure img{max-height:120mm}
.cover .meta{font-size:10pt;color:var(--muted);border-top:1px solid var(--line);padding-top:6px}
/* สารบัญ */
.toc h1{font-size:22pt;margin:0 0 8px}
.toc>ol>li>a{font-size:10.5pt}
.toc>ol{list-style:none;padding:0;columns:2;column-gap:10mm}
.toc>ol>li{break-inside:avoid;margin:0 0 5px}
.toc>ol>li>a{font-weight:600;color:var(--ink);display:flex;gap:8px}
.toc .tn{display:inline-block;min-width:22px;color:var(--accent)}
.toc ul{list-style:none;padding-left:30px;margin:1px 0 0;font-size:8.3pt;line-height:1.3;color:var(--muted)}
.toc ul li{margin:0}
/* บท */
.chapter{counter-increment:chapter;counter-reset:fig}
.chapter>h1{font-size:24pt;margin:0 0 12px;padding-bottom:8px;border-bottom:3px solid var(--accent)}
.chapter>h1::before{content:"บทที่ " counter(chapter);display:block;font-size:11pt;color:var(--accent);letter-spacing:.06em;font-weight:600}
.chapter.appendix{counter-increment:appendix}
.chapter.appendix>h1::before{content:"ภาคผนวก " counter(appendix,upper-alpha)}
/* คำสั่ง */
pre{white-space:pre-wrap;word-break:break-word;margin:6px 0 10px}
pre.cmd{background:#f3f6f5;border-left:3px solid var(--accent);padding:6px 10px;font-size:9.6pt;border-radius:0 4px 4px 0;break-inside:avoid}
pre.cmd::before{content:"$ ";color:var(--accent);font-weight:600}
pre.code{background:#f7f8f8;border:1px solid var(--line);padding:6px 10px;font-size:9pt;border-radius:4px}
td pre.cmd{margin:0}
/* กล่องหมายเหตุ */
.note,.tip,.warn{border-left:4px solid;padding:6px 10px;margin:8px 0;border-radius:0 4px 4px 0;break-inside:avoid;font-size:10.3pt}
.note{border-color:#3b82f6;background:#eff5ff}.tip{border-color:#16a34a;background:#effaf2}.warn{border-color:#dc2626;background:#fef2f2}
.note>b:first-child,.tip>b:first-child,.warn>b:first-child{margin-right:6px}
.warn>b:first-child{color:#b91c1c}.note>b:first-child{color:#1d4ed8}.tip>b:first-child{color:#15803d}
ol.steps{counter-reset:st;list-style:none;padding-left:0}
ol.steps>li{counter-increment:st;position:relative;padding-left:30px;margin:6px 0}
ol.steps>li::before{content:counter(st);position:absolute;left:0;top:1px;width:21px;height:21px;border-radius:50%;background:var(--accent);
  color:#fff;font-size:9.5pt;font-weight:600;text-align:center;line-height:21px}
.todo{background:#fef9c3;border:1px dashed #ca8a04;border-radius:3px;font-size:9pt;padding:0 4px;color:#854d0e}
/* ตาราง */
table{width:100%;border-collapse:collapse;margin:8px 0 12px;font-size:10pt}
thead{display:table-header-group}
th,td{border:1px solid var(--line);padding:4px 7px;vertical-align:top;text-align:left}
th{background:#eef3f2;font-weight:600}
tr{break-inside:avoid}
table.cmdref td:first-child{width:40%;white-space:nowrap}
table.cmdref tr.w td{background:#fff7ed;color:#9a3412}
table.figlist{font-size:9pt}table.figlist td:nth-child(2){white-space:nowrap}
h2 .key{font-size:9pt;font-weight:400;color:var(--muted);margin-left:8px;font-family:"IBM Plex Mono",monospace}
/* รูป */
figure.ph{margin:12px 0;text-align:center;break-inside:avoid;counter-increment:fig}
figure.ph img{max-width:100%;max-height:95mm;border-radius:4px}
figure.ph svg.dg{width:100%;max-height:110mm;font-family:inherit}
figure.ph figcaption{font-size:9.5pt;color:var(--muted);margin-top:4px}
.chapter:not(.appendix) figure.ph figcaption::before{content:"รูป " counter(chapter) "." counter(fig) " — ";font-weight:600;color:var(--ink)}
.ph-note{display:none}
.ph-box{display:none}
figure.ph.missing img{display:none}
figure.ph.missing .ph-box{display:flex;flex-direction:column;justify-content:center;gap:4px;min-height:42mm;border:1.5px dashed #9aa6a2;border-radius:6px;
  background:repeating-linear-gradient(135deg,#fafbfb 0 10px,#f3f5f5 10px 20px);padding:10px 14px;font-size:9.5pt;color:var(--muted)}
.ph-box .k{font-weight:600;color:var(--accent)}
.ph-box .f{font-family:"IBM Plex Mono",monospace;font-size:9pt;color:var(--ink)}
body.hide-missing figure.ph.missing{display:none}
body.hide-todo .todo{display:none}
/* svg */
svg.dg text{font-size:13px;fill:#1f2937}
svg.dg .ts{font-size:12px;fill:#374151}
svg.dg .th{font-weight:600}
svg.dg .tc{text-anchor:middle}svg.dg .te{text-anchor:end}
svg.dg .b-gray{fill:#f1f5f4;stroke:#6b7a76;stroke-width:1}
svg.dg .b-blue{fill:#dbeafe;stroke:#2563eb;stroke-width:1}
svg.dg .b-teal{fill:#d5f2ec;stroke:#0f766e;stroke-width:1}
svg.dg .b-amber{fill:#fdf0d5;stroke:#b45309;stroke-width:1}
svg.dg .wheel{fill:#374151}
svg.dg .pole{fill:#9ca3af}
svg.dg .zone{fill:#fde2e2;stroke:#dc2626;stroke-width:1}
svg.dg .ln{stroke:#4b5563;stroke-width:1.2;fill:none}
svg.dg .dim{stroke:#374151;stroke-width:1;fill:none}
svg.dg .lead{stroke:#9ca3af;stroke-width:.8;stroke-dasharray:3 3}
/* แถบเครื่องมือ (จอเท่านั้น) */
.toolbar{position:fixed;top:12px;right:12px;background:#fff;border:1px solid var(--line);border-radius:8px;padding:8px 10px;font-size:10pt;
  box-shadow:0 2px 8px rgba(0,0,0,.1);display:flex;flex-direction:column;gap:6px;z-index:10;max-width:230px}
.toolbar button{font:inherit;background:var(--accent);color:#fff;border:0;border-radius:5px;padding:5px 10px;cursor:pointer}
.toolbar label{display:flex;gap:6px;align-items:center;cursor:pointer}
.toolbar .stat{color:var(--muted);font-size:9pt}
@media (max-width:900px){.toolbar{position:static;margin:8px auto;max-width:210mm;flex-direction:row;flex-wrap:wrap}}
@media screen{.chapter,.toc,.page{border-bottom:10px solid var(--bg)}}
@page{size:A4;margin:14mm 0 16mm;@bottom-center{content:counter(page);font-size:9pt;color:#6b7280}}
@page:first{margin:0;@bottom-center{content:none}}
@media print{
  html,body{background:#fff}
  .book{max-width:none;margin:0;box-shadow:none}
  .toolbar{display:none}
  .page,.chapter,.toc{padding:0 16mm}
  .cover{padding:22mm 18mm 18mm;min-height:297mm;break-after:page}
  .toc{break-after:page}
  .chapter{break-before:page}
  a{color:inherit}
}
"""

JS = r"""
(function(){
  const KIND={photo:"รูปถ่าย",screenshot:"ภาพหน้าจอ",diagram:"แผนผัง"};
  let missing=0;
  function stat(){const s=document.getElementById("stat");
    s.textContent="ยังไม่มีรูป "+missing+" ช่อง · ต้องเช็ก "+document.querySelectorAll(".todo").length+" จุด";}
  document.querySelectorAll("figure.ph img").forEach(img=>{
    const fig=img.closest("figure");
    const note=fig.querySelector(".ph-note");
    const box=document.createElement("div");box.className="ph-box";
    box.innerHTML='<span class="k">'+(KIND[fig.dataset.kind]||"รูป")+'</span><span class="f">docs/manual/'+img.getAttribute("src")+'</span>'+
      (note?'<span>'+note.innerHTML+'</span>':'');
    img.after(box);
    const fail=()=>{const src=img.getAttribute("src");
      if(/\.jpg$/.test(src)&&!img.dataset.tried){img.dataset.tried=1;img.src=src.replace(/\.jpg$/,".png");return;}
      if(!fig.classList.contains("missing")){fig.classList.add("missing");missing++;stat();}};
    img.addEventListener("error",fail);
    if(img.complete&&img.naturalWidth===0)fail();
  });
  stat();
  document.getElementById("t-missing").addEventListener("change",e=>document.body.classList.toggle("hide-missing",!e.target.checked));
  document.getElementById("t-todo").addEventListener("change",e=>document.body.classList.toggle("hide-todo",!e.target.checked));
})();
"""

cover = f"""<section class="page cover">
<div><div class="eyebrow">HUMANOI · คู่มือการใช้งาน</div>
<h1>คู่มือหุ่นยนต์ Humanoi</h1>
<div class="sub">ฐานล้อ mecanum · แขนกล AR4 สองแขน · มือ AmazingHand<br>ROS 2 Jazzy บน Raspberry Pi 5</div></div>
<figure class="ph" data-kind="photo"><img src="../images/dual_arms_tray.jpg" alt=""><figcaption>Humanoi</figcaption></figure>
<div class="meta">{VERSION} · ใช้กับคำสั่ง <code>arm</code> และเฟิร์มแวร์ฐานรุ่นที่มีกันชน lidar<br>
เอกสารต้นทางอยู่ใน <code>docs/</code> ของโปรเจ็กต์ · ช่องรูปเส้นประ = รูปที่ยังไม่ได้ใส่ (ภาคผนวก C)</div>
</section>"""

doc = f"""<!doctype html>
<html lang="th"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1">
<title>คู่มือหุ่นยนต์ Humanoi</title>
<link rel="preconnect" href="https://fonts.googleapis.com"><link rel="preconnect" href="https://fonts.gstatic.com" crossorigin>
<link rel="stylesheet" href="https://fonts.googleapis.com/css2?family=Sarabun:wght@400;600&family=IBM+Plex+Mono:wght@400;500&display=swap">
<style>{CSS}</style></head>
<body>
<div class="toolbar"><button onclick="window.print()">พิมพ์ / บันทึก PDF</button>
<label><input type="checkbox" id="t-missing" checked> แสดงช่องรูปที่ยังไม่มี</label>
<label><input type="checkbox" id="t-todo" checked> แสดงจุด "ต้องเช็ก"</label>
<span class="stat" id="stat"></span>
<span class="stat">พิมพ์: Chrome · A4 · ขอบ "ค่าเริ่มต้น" · ติ๊ก "กราฟิกพื้นหลัง"</span></div>
<main class="book">
{cover}
{toc}
{"".join(body_chapters)}
{"".join(appendices)}
</main>
<script>{JS}</script>
</body></html>
"""
os.makedirs(os.path.join(REPO, "docs", "manual", "img"), exist_ok=True)
with open(OUT, "w", encoding="utf-8", newline="\n") as f:
    f.write(doc)

readme = ["รูปสำหรับคู่มือ docs/manual/HUMANOI_MANUAL.html — วางไฟล์ชื่อตามนี้ (.jpg หรือ .png) แล้วเปิดคู่มือใหม่", ""]
for num, src, kind, cap, note, chap in figs:
    readme.append(f"รูป {num:<6} {src[4:]:<40} [{KIND.get(kind, kind)}] {cap}")
    readme.append(f"         {note}")
with open(os.path.join(REPO, "docs", "manual", "img", "README.txt"), "w", encoding="utf-8", newline="\n") as f:
    f.write("\n".join(readme) + "\n")
sys.stdout.reconfigure(encoding="utf-8")
print(f"ok: {OUT} · {len(body_chapters)} บท + {len(appendices)} ภาคผนวก · รูปที่ต้องเตรียม {len(figs)} · ต้องเช็ก {todo_count}")
