"""慢速连续扫描：把"转向角度 → 读数"这条映射的形状打出来。

用法：双击 mag_trace.bat（或 python mag_trace.py [秒数]）

为什么要做这个：
    分点停位姿只能看出"点"，看不出"线"。如果读数真的是连续角度，
    那么**匀速推到底**时读数应该是**平滑单调**地爬升/下降；
    如果它是在两个电平之间跳（双稳/旷量/磁场翻转），就会看到台阶。
    这决定了这套编码器能不能做**连续**位置反馈（而不仅是两端判定）。

输出：
    · 每 200ms 一行的 ASCII 轨迹（值 → 条形）
    · 每段扫描的起止值、是否单调、最大单步跳变
    · 结论：平滑连续 / 台阶跳变
并写 mag_trace.csv。
"""
import os
import socket
import struct
import sys
import time

SECS = float(sys.argv[1]) if len(sys.argv) > 1 else 30.0
PORT = 8086
HERE = os.path.dirname(os.path.abspath(__file__))
CSV = os.path.join(HERE, "mag_trace.csv")

MAGIC_DN, MAGIC_UP, TAIL = b"\x5a\xa5", b"\xa5\x5a", b"\x00\x00\x80\x7f"


def crc16(b, n):
    c = 0xFFFF
    for i in range(n):
        c ^= b[i]
        for _ in range(8):
            c = (c >> 1) ^ 0xA001 if c & 1 else c >> 1
    return c & 0xFFFF


def cmd(c, pid=0, val=0.0, tok=1):
    pl = struct.pack("<BBfH", c, pid, val, tok)
    body = bytes([0x01]) + struct.pack("<H", len(pl)) + pl
    return MAGIC_DN + body + struct.pack("<H", crc16(body, len(body)))


s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
print("=" * 72)
print(" 等 8086 …… 一直等不到就在上位机点「断开」。")
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
s.settimeout(0.4)

s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
for _k in range(5):
    try:
        s.sendto(cmd(0x01, 0, 0, 9000 + _k), ("192.168.50.255", 6666))
        s.sendto(cmd(0x01, 0, 0, 9100 + _k), ("192.168.50.238", 6666))
    except OSError:
        pass
    time.sleep(0.1)
print("   （已朝模块敲了几帧唤醒包）")

peer, tok, rows = None, 0, []


def pump(sec):
    global peer
    t0 = time.time()
    while time.time() - t0 < sec:
        try:
            d, a = s.recvfrom(2048)
        except socket.timeout:
            continue
        if len(d) < 11 or d[:2] != MAGIC_UP or d[-4:] != TAIL:
            continue
        ftype, plen = d[2], struct.unpack_from("<H", d, 3)[0]
        if 5 + plen + 2 > len(d) or ftype != 0x01:
            continue
        if struct.unpack_from("<H", d, 5 + plen)[0] != crc16(d[2:], 3 + plen):
            continue
        peer = a
        body = d[5:5 + plen]
        if struct.unpack_from("<H", body, 8)[0] >= 16:
            rows.append((struct.unpack_from("<I", body, 2)[0],
                         int(struct.unpack_from("<f", body, 10 + 12 * 4)[0]) & 0x3FFF))


pump(3.0)
if peer is None:
    print("[ERR] 收不到车端遥测（板子在发吗？）"); sys.exit(2)
print(f"[OK] 车端 {peer[0]}:{peer[1]}")
tok += 1
s.sendto(cmd(0x03, 6, 3.0, tok), peer)
pump(1.0)

print("""
========================================================================
 ★ 请做 **2 个完整来回**，每个方向大约 **5 秒**：
     从最左 → 最右（慢慢推，匀速，**中途不要停**，一直顶到限位）
     再从最右 → 最左（同样慢慢推，顶到限位）
   整个过程约 20 秒，然后把手拿开。
   ★ 关键是"**连续、匀速**"——不要一格一格跳着推。
========================================================================
""")
for i in (3, 2, 1):
    print(f"   {i} ..."); time.sleep(1.0)
print("   开始！\n"); sys.stdout.flush()
rows.clear()

t0 = time.time()
nxt = 0.2
while time.time() - t0 < SECS:
    pump(0.1)
    if time.time() - t0 >= nxt and rows:
        # 取最近 10 帧（约 200ms）的中位
        v = sorted(x[1] for x in rows[-10:])[len(rows[-10:]) // 2]
        lo = min(x[1] for x in rows); hi = max(x[1] for x in rows)
        span = max(400, hi - lo)
        pos = int((v - lo) / span * 44)
        bar = " " * max(0, pos) + "#"
        print(f"   {time.time()-t0:5.1f}s  {v:6d}  |{bar}")
        sys.stdout.flush()
        nxt += 0.2

with open(CSV, "w", encoding="utf-8") as f:
    f.write("car_ts_ms,angle14,deg\n")
    for ts, v in rows:
        f.write(f"{ts},{v},{v/16384*360:.2f}\n")

a = [r[1] for r in rows]
n = len(a)
print(f"\n采集 {n} 帧，值域 {min(a)} ~ {max(a)}（跨度 {max(a)-min(a)} = {(max(a)-min(a))/16384*360:.1f}°）")

# ---- 台阶检测：把"单步跳变超过阈值"的地方挑出来 ----
TH = 150
jumps = [(i, a[i - 1], a[i]) for i in range(1, n) if abs(a[i] - a[i - 1]) > TH]
merged = []
for i, p, q in jumps:
    if merged and i - merged[-1][-1][0] <= 3:
        merged[-1].append((i, p, q))
    else:
        merged.append([(i, p, q)])
print()
print(f"单步跳变 >{TH} 计数的地方：{len(merged)} 处")
for g in merged[:12]:
    i0, p0, _ = g[0]
    _, _, q1 = g[-1]
    print(f"   ts={rows[i0][0]}ms  {p0} → {q1}   跳 {q1-p0:+d} ({(q1-p0)/16384*360:+5.1f}°)")

# ---- 单调段 ----
dirs = []
for i in range(1, n):
    d = a[i] - a[i - 1]
    if abs(d) > 3:
        dirs.append(1 if d > 0 else -1)
rev = sum(1 for i in range(1, len(dirs)) if dirs[i] != dirs[i - 1])
print()
print(f"运动方向反转次数：{rev}（人手来回 2 趟 → 期望约 3~4 次；远多于这个数就是在抖/跳）")
print()
print("=" * 72)
span_all = max(a) - min(a)
if span_all < 300:
    print(f"[结论] ⚠ 这一段几乎没动（整段跨度只有 {span_all} 计数 = {span_all/16384*360:.1f}°）")
    print("       → 无法判断映射形状。请重跑，并在脚本说「开始」之后**真的匀速推到底**。")
elif len(merged) <= 3 and rev <= 8:
    print("[结论] ✅ 映射是**连续平滑**的：慢推时读数单调爬升，几乎没有台阶。")
    print("       → 编码器可以直接做连续位置反馈。中间位姿读数不稳，是**手感放不准**，不是传感器的问题。")
elif len(merged) >= 8:
    print(f"[结论] ⚠ 出现 {len(merged)} 处台阶（单步跳过 {TH} 计数 = {TH/16384*360:.1f}° 以上）。")
    print("       → 读数不是连续角度，而是**在两个/几个电平之间跳**。按可能性：")
    print("         ① 机械旷量/回差太大（转向与磁铁之间有几十度的空行程）")
    print("         ② 磁铁没固定死，会打滑/被吸到某个位置")
    print("         ③ 芯片处的场被一个强静态分量主导，导致角度输出在某个方向附近翻转")
    print(f"       （第1步先看上面那张轨迹图：是「斜坡」还是「方波」）")
else:
    print(f"[结论] 介于两者之间（{len(merged)} 处台阶，{rev} 次方向反转）——看上面轨迹图判断。")
print("=" * 72)
s.close()
