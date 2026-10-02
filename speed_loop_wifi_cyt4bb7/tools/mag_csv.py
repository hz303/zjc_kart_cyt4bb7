"""把编码器的三个通道录成 CSV：角度变化 / 角度 raw / 编码器角度。

用法：双击 mag_csv.bat
      mag_csv.bat 30     —— 录 30 秒
      mag_csv.bat 0      —— 一直录，按 Ctrl+C 停（会正常收尾并落盘）

导出的列（mag_csv.csv）：
    host_s      本机墙钟（从采集开始算的秒数）
    car_ts_ms   车端时间戳（板子的毫秒，跨机比时间用这个）
    mag_raw     通道 12：原始 16 bit 回读值
    mag_angle   通道 13：按当前协议取有效位后的绝对角度（raw）
    mag_deg     通道 14：编码器角度（°，含零点/方向）
    mag_delta   通道 15：与上一次读数的差（raw）
    moving      1 = 这一帧在运动（用最近 0.5s 的跨度判），仅供切段参考

注意：KartHost 在 UDP 模式下独占 8086，跑这个之前请先在上位机点「断开」。
"""
import os
import socket
import struct
import sys
import time

DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 60.0
# 等端口的最长秒数（双击用时不用管；我在这边后台录的时候会传个大值）
WAIT_PORT = float(sys.argv[2]) if len(sys.argv) > 2 else 60.0
PORT = 8086
HERE = os.path.dirname(os.path.abspath(__file__))
CSV = os.path.join(HERE, "mag_csv.csv")

MAGIC_DN, MAGIC_UP, TAIL = b"\x5a\xa5", b"\xa5\x5a", b"\x00\x00\x80\x7f"
FULL = 16384.0          # 协议 3 的满量程
CH_COUNT = 16


def crc16(buf, n):
    crc = 0xFFFF
    for i in range(n):
        crc ^= buf[i]
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc & 0xFFFF


def build_cmd(cmd, pid, val, token):
    pl = struct.pack("<BBfH", cmd, pid, val, token)
    body = bytes([0x01]) + struct.pack("<H", len(pl)) + pl
    return MAGIC_DN + body + struct.pack("<H", crc16(body, len(body)))


s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
print("=" * 72)
print(" 需要独占 UDP 8086。")
print(" ★ 请先在上位机（KartHost）点一下「断开」—— 它占着端口时谁也收不到。")
print("   点完不用管，这个脚本会自己接上来（最多等 60 秒）。")
print("=" * 72)
for k in range(int(WAIT_PORT / 2)):
    try:
        s.bind(("0.0.0.0", PORT))
        break
    except OSError:
        print(f"   ...还在等端口空出来（{k*2}s / {WAIT_PORT:.0f}s）")
        time.sleep(2.0)
else:
    print("[ERR] 端口一直没空出来。")
    print("      → 请确认上位机已「断开」；另外逐飞助手 / 其它抓包脚本也会占这个端口。")
    sys.exit(1)
print("[OK] 已占住 8086\n")
s.settimeout(0.4)

# 唤醒：这块模块跑一阵子会进入"只收不发"的僵死态（上行静默、下行也不投递），
# 而板子自己 tx 照涨、link_err 照旧是 0 —— 从 PC 敲一个入站包能把上行唤回来。
s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
for _k in range(5):
    for dst in ("192.168.50.255", "192.168.50.238"):
        try:
            s.sendto(build_cmd(0x01, 0, 0, 9000 + _k), (dst, 6666))
        except OSError:
            pass
    time.sleep(0.1)

peer = None
rows = []          # (host_t, car_ts, raw, angle, deg, delta)
parse_fail = 0


def pump(sec):
    """收一收包，塞进 rows。返回收到的合规帧数。"""
    global peer, parse_fail
    got = 0
    t0 = time.time()
    while time.time() - t0 < sec:
        try:
            data, addr = s.recvfrom(65535)
        except socket.timeout:
            continue
        if len(data) < 11 or data[:2] != MAGIC_UP or data[-4:] != TAIL:
            continue
        ftype, plen = data[2], struct.unpack_from("<H", data, 3)[0]
        if 5 + plen + 2 > len(data) or ftype != 0x01:
            continue
        if struct.unpack_from("<H", data, 5 + plen)[0] != crc16(data[2:], 3 + plen):
            parse_fail += 1
            continue
        peer = addr
        body = data[5:5 + plen]
        if struct.unpack_from("<H", body, 8)[0] < CH_COUNT:
            continue
        base = 10
        ts = struct.unpack_from("<I", body, 2)[0]
        ch = struct.unpack_from("<16f", body, base)
        rows.append((time.time(), ts, int(ch[12]), int(ch[13]), ch[14], ch[15]))
        got += 1
    return got


pump(3.0)
if peer is None:
    print("[ERR] 收不到车端遥测 —— 板子在发吗？（串口看 tx 有没有在涨）")
    s.close()
    sys.exit(2)
print(f"[OK] 车端 {peer[0]}:{peer[1]}")

# 确认编码器协议 = 3（0x3FFF / 16384），否则 mag_deg 的刻度不对
s.sendto(build_cmd(0x03, 6, 3.0, 1), peer)
pump(0.6)
print("   （已把「编码器协议」确认为 3）")
print()

if DUR > 0:
    print("=" * 72)
    print(f" 接下来录 {DUR:.0f} 秒。现在就开始转转向（推到两端、来回几次）。")
    print(" 想停就按 Ctrl+C，数据照样会存下来。")
    print("=" * 72)
else:
    print("=" * 72)
    print(" 开始连续录制，按 Ctrl+C 停止（数据会存下来）。现在可以转转向了。")
    print("=" * 72)
for i in (3, 2, 1):
    print(f"   {i} ...")
    time.sleep(1.0)
print("   开始！\n")
sys.stdout.flush()

rows.clear()
t0 = time.time()
last_tick = t0
try:
    while True:
        pump(0.3)
        now = time.time()
        if now - last_tick >= 2.0:
            last_tick = now
            print(f"   [{now-t0:5.1f}s] 已录 {len(rows)} 帧")
            sys.stdout.flush()
        if DUR > 0 and now - t0 >= DUR:
            break
except KeyboardInterrupt:
    print("\n   （收到 Ctrl+C，收尾中…）")
wall = time.time() - t0

# ---------------- 落盘 ----------------
with open(CSV, "w", encoding="utf-8") as f:
    f.write("host_s,car_ts_ms,mag_raw,mag_angle,mag_deg,mag_delta,moving\n")
    # 运动判定：用最近约 0.5s（25 帧）的 mag_angle 跨度
    for i, (ht, ts, raw, ang, deg, dlt) in enumerate(rows):
        lo = max(0, i - 25)
        win = [r[3] for r in rows[lo:i + 1]]
        moving = 1 if (max(win) - min(win)) >= 60 else 0
        f.write(f"{ht-t0:.3f},{ts},{raw},{ang},{deg:.3f},{dlt:.0f},{moving}\n")

# ---------------- 小结 ----------------
print()
print("=" * 72)
if not rows:
    print("[结论] 一帧都没收到 —— 检查板子有没有在跑、目标 IP 对不对。")
else:
    ts0, ts1 = rows[0][1], rows[-1][1]
    ang = [r[3] for r in rows]
    deg = [r[4] for r in rows]
    dlt = [r[5] for r in rows]
    mov = sum(1 for i, r in enumerate(rows)
              if max([x[3] for x in rows[max(0, i - 25):i + 1]]) -
                 min([x[3] for x in rows[max(0, i - 25):i + 1]]) >= 60)
    print(f"采集 {wall:.1f}s（墙钟） / 车端 {ts1-ts0} ms，共 {len(rows)} 帧"
          f"，{len(rows)/max(wall,0.001):.1f} 帧/秒")
    if parse_fail:
        print(f"  （另有 {parse_fail} 帧 CRC/长度不合规，已丢弃）")
    print(f"mag_angle : {min(ang)} ~ {max(ang)}   跨度 {max(ang)-min(ang)} 计数"
          f" = {(max(ang)-min(ang))/FULL*360:.1f}°")
    print(f"mag_deg   : {min(deg):.2f} ~ {max(deg):.2f}   跨度 {max(deg)-min(deg):.2f}°")
    print(f"mag_delta : {min(dlt):.0f} ~ {max(dlt):.0f}"
          f"   （非零帧 {sum(1 for d in dlt if abs(d) >= 1)}/{len(dlt)}）")
    print(f"运动帧    : {mov}/{len(rows)} （{mov/max(len(rows),1)*100:.1f}%）")
    if mov == 0:
        print("  ⚠ 全程没检出运动 —— 这次多半没在转（或读数没动），数据完整性有限。")
print()
print(f"[CSV] {CSV}")
print("=" * 72)
s.close()
