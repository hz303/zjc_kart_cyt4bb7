"""舵机行程标定 / 重复性 + 回差测量。

用法：双击 mag_steps.bat（或 python mag_steps.py [每位姿采样秒数]）

它按固定节奏提示你把转向摆到 5 个位姿，来回扫 3 遍：
    第1遍 左→右   第2遍 右→左（测回差）   第3遍 左→右（测同向重复性）
最后给：
    · 位姿 × 遍数 的读数矩阵
    · 同向重复性（第1遍 vs 第3遍）
    · 回差（第1遍 vs 第2遍，即"从左过来"和"从右过来"差多少）
    · 刻度（每个位姿差多少计数）与线性度
    · 直接给出 mag_zero / mag_dir 的建议值
并写 mag_steps.csv。

为什么这是关键实验：机械限位是"硬限位"，如果磁铁跟轴刚性连接，
同一位姿的读数**必须**一模一样。差出来的量就是回差（机构旷量）或磁铁打滑 ——
这个数决定了这套编码器能不能当舵机反馈用。
"""
import os
import socket
import struct
import sys
import time

DWELL = float(sys.argv[1]) if len(sys.argv) > 1 else 3.0
PORT = 8086
HERE = os.path.dirname(os.path.abspath(__file__))
CSV = os.path.join(HERE, "mag_steps.csv")

MAGIC_DN, MAGIC_UP, TAIL = b"\x5a\xa5", b"\xa5\x5a", b"\x00\x00\x80\x7f"
POS = ["最左", "左中", "正中", "右中", "最右"]


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
print(" 等 8086 …… 一直等不到就在上位机点一下「断开」（KartHost 占着这个端口）。")
print("=" * 72)
for k in range(90):
    try:
        s.bind(("0.0.0.0", PORT)); break
    except OSError:
        if k % 5 == 0:
            print(f"   ...等（{k*2}s）")
        time.sleep(2.0)
else:
    print("[ERR] 端口没空出来。"); sys.exit(1)
print("[OK] 已占住 8086\n")
s.settimeout(0.4)

s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
# ★ 唤醒：这块模块跑一阵子会进入"只收不发"的僵死态（上行静默、下行也不投递），
#   而板子自己 tx 照涨、link_err 照旧是 0 —— 从 PC 敲一个入站包能把上行唤回来。
for _k in range(5):
    try:
        s.sendto(build_cmd(0x01, 0, 0, 9000 + _k), ("192.168.50.255", 6666))
        s.sendto(build_cmd(0x01, 0, 0, 9100 + _k), ("192.168.50.238", 6666))
    except OSError:
        pass
    time.sleep(0.1)
print("   （已朝模块敲了几帧唤醒包）")

peer, tok = None, 0
buf = []


def pump(sec):
    global peer, tok
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
            continue
        peer = addr
        body = data[5:5 + plen]
        if struct.unpack_from("<H", body, 8)[0] >= 16:
            buf.append((struct.unpack_from("<I", body, 2)[0],
                        int(struct.unpack_from("<f", body, 10 + 12 * 4)[0]) & 0x3FFF))


pump(3.0)
if peer is None:
    print("[ERR] 收不到车端遥测。"); sys.exit(2)
print(f"[OK] 车端 {peer[0]}:{peer[1]}")
tok += 1
s.sendto(build_cmd(0x03, 6, 3.0, tok), peer)      # 协议 3
pump(1.0)

print("""
========================================================================
 接下来按提示摆位姿。每个位姿**保持不动**，脚本会自己采 3 秒。
 一共 3 遍（左→右 / 右→左 / 左→右），约 1 分钟。
 ★ 关键：每一遍都要推到**机械硬限位**为止，别怕顶到头。
========================================================================
""")
for i in (3, 2, 1):
    print(f"   {i} ..."); time.sleep(1.0)
print()

PASSES = [("左→右", POS), ("右→左", list(reversed(POS))), ("左→右", POS)]
data = {}
for pi, (pname, order) in enumerate(PASSES):
    print(f"--- 第 {pi+1} 遍（{pname}）---")
    for pos in order:
        print(f"   → 请把转向摆到【{pos}】，保持不动 ……", end="")
        sys.stdout.flush()
        buf.clear()
        pump(DWELL)
        v = [x[1] for x in buf[-20:]]
        if len(v) < 5:
            print(" （没收到数据）")
            continue
        m = sum(v) / len(v)
        sp = max(v) - min(v)
        data[(pi, pos)] = (m, sp)
        flag = "" if sp <= 6 else f"  ⚠ 没停稳（跨度 {sp}）"
        print(f" {m:8.1f} ({m/16384*360:6.2f}°){flag}")
    print()

with open(CSV, "w", encoding="utf-8") as f:
    f.write("pass,position,mean,spread,deg\n")
    for (pi, pos), (m, sp) in sorted(data.items()):
        f.write(f"{pi+1},{pos},{m:.1f},{sp},{m/16384*360:.2f}\n")

print("=" * 72)
print("读数矩阵（计数 / 度）")
print(f"   {'位姿':<6}", end="")
for pi, (pname, _) in enumerate(PASSES):
    print(f"{'第'+str(pi+1)+'遍':>18}", end="")
print()
for pos in POS:
    print(f"   {pos:<6}", end="")
    for pi in range(3):
        d = data.get((pi, pos))
        print(f"{d[0]:11.1f} ({d[0]/16384*360:5.1f}°)" if d else f"{'--':>18}", end="")
    print()

# ---------- 重复性 / 回差 ----------
print()
print("位姿     同向重复(|1遍-3遍|)      回差(|1遍-2遍|)")
rep_max = hys_max = 0
for pos in POS:
    d0, d1, d2 = data.get((0, pos)), data.get((1, pos)), data.get((2, pos))
    if not (d0 and d1 and d2):
        continue
    rep = abs(d0[0] - d2[0]); hys = abs(d0[0] - d1[0])
    rep_max = max(rep_max, rep); hys_max = max(hys_max, hys)
    print(f"   {pos:<6} {rep:8.1f} 计数 = {rep/16384*360:5.2f}°      "
          f"{hys:8.1f} 计数 = {hys/16384*360:5.2f}°")

# ---------- 刻度 / 线性度 ----------
xs = []
for pos in POS:
    if (0, pos) in data:
        xs.append(data[(0, pos)][0])
if len(xs) == 5:
    steps = [xs[i + 1] - xs[i] for i in range(4)]
    print()
    print(f"相邻位姿读数差：{['%.0f' % s for s in steps]} 计数  "
          f"（±6° 的 1/4 行程 ≈ {60/4/360*16384:.0f} 计数，若明显偏小就是磁场被压住了）")
    mono = all(s > 0 for s in steps) or all(s < 0 for s in steps)
    print(f"单调性：{'✅ 单调' if mono else '⚠ 不单调（中间有反转 → 磁场畸变）'}")

# ---------- 标定建议 ----------
print()
print("=" * 72)
if (0, "正中") in data and (0, "最左") in data and (0, "最右") in data:
    c = data[(0, "正中")][0]
    l = data[(0, "最左")][0]
    r = data[(0, "最右")][0]
    print(f"标定建议（以第1遍为准，raw 值）：")
    print(f"   mag_zero = {c:.0f}      （把正中对到 0°，mag_deg 就会以正中为零点）")
    print(f"   mag_dir  = {'1' if r > l else '0'}      "
          f"（{'向右读数增大 → 用 1' if r > l else '向右读数减小 → 用 0'}）")
    print(f"   mag_deg 行程约 {(max(l,r)-min(l,r))/16384*360:.1f}°，"
          f"左右各约 {(r-l)/2/16384*360:+.1f}°")
print()
print(f"★ 结论：同向重复性最差 {rep_max:.0f} 计数 = {rep_max/16384*360:.2f}°；"
      f"回差最差 {hys_max:.0f} 计数 = {hys_max/16384*360:.2f}°")
if max(rep_max, hys_max) < 150:
    print("   → **可以当舵机反馈用**（误差 <3.3°）。下一步：把 mag_zero/mag_dir 设成上面的值。")
elif max(rep_max, hys_max) < 400:
    print("   → 勉强可用（3~9°）：做位置闭环时要有余量，别用高增益。")
else:
    print("   → 不可用（>9°）：同一位姿两次读数差太多。先查机构旷量/磁铁固定，再查磁场。")
print("=" * 72)
s.close()
