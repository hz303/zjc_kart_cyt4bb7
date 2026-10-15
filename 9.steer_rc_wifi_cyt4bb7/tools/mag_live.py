"""实时监听转向 → 编码器读数的关系（不用管时机，脚本自己等运动）。

用法：双击 mag_live.bat

流程：
  1. 等 8086 空出来（被上位机占着就每 2 秒重试 + 提示你点「断开」）
  2. 给你 3 秒准备，然后**一直等到你转够 4 次**（最长 120 秒），
     每检测到一次运动就在屏幕上打一行，你能看到它在跟
  3. 末尾三张表 + 结论，原始数据写 mag_live.csv

三张表怎么读：
  ① 静止段表 —— 同一个机械位置停下时读到的角度。
        值**只在两组之间跳**（左端一组、右端一组）→ 传感器正常，是机构回差
        值**一路单调游走**（每次循环偏一点）        → 磁铁在打滑
        值**几乎不变**（虽然你在转）                → 传感头看不到磁铁在转
  ② 运动段表 —— 每次运动的净变化 Δ，看它跟"往左/往右"有没有稳定对应
  ③ 标志位   —— 运动中 bit15/bit14 是否恒为 0/1（变了就说明帧本身有问题）
"""
import collections
import os
import socket
import struct
import sys
import time

MAX_SECS = float(sys.argv[1]) if len(sys.argv) > 1 else 120.0
WANT_BURSTS = 4
PORT = 8086
HERE = os.path.dirname(os.path.abspath(__file__))
CSV = os.path.join(HERE, "mag_live.csv")

MAGIC_DN, MAGIC_UP, TAIL = b"\x5a\xa5", b"\xa5\x5a", b"\x00\x00\x80\x7f"


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
print(" 等 8086 端口…… 如果一直等不到，请在上位机点一下「断开」。")
print("=" * 72)
for k in range(60):
    try:
        s.bind(("0.0.0.0", PORT)); break
    except OSError:
        print(f"   ...还在等（{k*2}s）")
        time.sleep(2.0)
else:
    print("[ERR] 端口一直没空出来。"); sys.exit(1)
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
rows = []          # (car_ts, raw, angle14)


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
            ts = struct.unpack_from("<I", body, 2)[0]
            raw = int(struct.unpack_from("<f", body, 10 + 12 * 4)[0])
            rows.append((ts, raw, raw & 0x3FFF))


pump(3.0)
if peer is None:
    print("[ERR] 收不到车端遥测 —— 板子在发吗？"); sys.exit(2)
print(f"[OK] 车端 {peer[0]}:{peer[1]}  （编码器协议已设为 3）")
tok += 1
s.sendto(build_cmd(0x03, 6, 3.0, tok), peer)
pump(1.0)
print()
print("=" * 72)
print(" ★ 做两件事（脚本最多等 120 秒，够做两个）：")
print()
print("  [1] 把转向**反复从最左推到最右、再推回最左**，推 5~6 个来回。")
print("      每次都尽量推到**同样的两个端点**（这样才看得出值可不可重复）。")
print()
print("  [2] 如果上面一直没出现「√ 第 N 次运动」，就**捏一块小磁铁**，")
print("      在编码器芯片**正上方**慢慢转一整圈。")
print("      ↑ 这一下能把「芯片/读取」和「装配」一刀切开：")
print("         转磁铁有反应 → 芯片和读取都是好的，问题 100% 在装配上")
print("         转磁铁也没反应 → 传感头根本没在工作")
print()
print("  脚本会自己认出你在动；出现「√ 第 N 次运动」就说明它在跟。")
print("=" * 72)
for i in (3, 2, 1):
    print(f"   {i} ..."); time.sleep(1.0)
print("   开始！\n"); sys.stdout.flush()
rows.clear()

# ---------- 采集：检测到足够运动就提前结束 ----------
t0 = time.time()
bursts = []
cur_start = None
last_span_t = time.time()
n_burst = 0
while time.time() - t0 < MAX_SECS:
    pump(0.3)
    if len(rows) < 20:
        continue
    recent = rows[-25:]                       # 最近约 500 ms
    v = [r[2] for r in recent]
    span = max(v) - min(v)
    if span >= 60:                            # 0.5 s 内跳 60 个计数以上 = 在动
        if cur_start is None:
            cur_start = rows[-25]
        last_span_t = time.time()
    else:
        if cur_start is not None and time.time() - last_span_t > 0.5:
            a0, a1 = cur_start[2], rows[-1][2]
            ex = a1 - a0
            n_burst += 1
            print(f"   √ 第 {n_burst} 次运动：{a0:6d} → {a1:6d}  净 {ex:+7d}"
                  f" ({(ex)/16384*360:+7.2f}°)")
            sys.stdout.flush()
            bursts.append((cur_start, rows[-1]))
            cur_start = None
            if n_burst >= WANT_BURSTS:
                break
    if time.time() - last_span_t > 5.0 and n_burst == 0:
        last_span_t = time.time()             # 只重置计时，继续等
print(f"\n采集结束：{time.time()-t0:.1f}s，{len(rows)} 帧，识别到 {len(bursts)} 次运动")

with open(CSV, "w", encoding="utf-8") as f:
    f.write("car_ts_ms,raw,angle14,deg\n")
    for ts, raw, a in rows:
        f.write(f"{ts},{raw},{a},{a/16384*360:.2f}\n")
print(f"[CSV] {CSV}\n")

# ---------- ① 静止段 ----------
segs, cur = [], []
for r in rows:
    if not cur or max(x[2] for x in cur) - min(x[2] for x in cur) <= 3:
        cur.append(r)
    else:
        if len(cur) >= 15:
            segs.append(cur)
        cur = [r]
if len(cur) >= 15:
    segs.append(cur)

print("=" * 72)
print("① 静止段表（停下时读到的角度）")
print("   序号    时刻     角度        与上一段之差")
prev = None
vals = []
for i, sg in enumerate(segs):
    a = sum(x[2] for x in sg) / len(sg)
    vals.append(a)
    d = "" if prev is None else f"{a-prev:+8.1f} 计数 = {(a-prev)/16384*360:+7.2f}°"
    print(f"   {i:3d}  {sg[0][0]/1000:7.1f}s  {a:8.1f} ({a/16384*360:7.2f}°)   {d}")
    prev = a

# ---------- ② 运动段 ----------
print()
print("② 运动段表")
for i, (b0, b1) in enumerate(bursts):
    print(f"   {i:3d}  {b0[0]/1000:7.1f}s  起 {b0[2]:6d} → 止 {b1[2]:6d}"
          f"   净 {b1[2]-b0[2]:+7d} ({(b1[2]-b0[2])/16384*360:+7.2f}°)")
if not bursts:
    print("   （一次运动都没识别到 —— 说明你在转的时候读数没动）")

# ---------- ③ 标志位 ----------
if rows:
    b15 = sum((r[1] >> 15) & 1 for r in rows)
    b14 = sum((r[1] >> 14) & 1 for r in rows)
    print()
    print(f"③ 标志位：bit15=1 占 {b15/len(rows)*100:.1f}%   bit14=1 占 {b14/len(rows)*100:.1f}%"
          f"  （应恒为 0% / 100%）")

# ---------- 结论 ----------
print()
print("=" * 72)
if not bursts:
    print("[结论] ★ 你在转，但**读数没有跟着动**（一次运动都没识别到）。")
    print("       → 传感头看到的磁场不随轴旋转。按可能性排：")
    print("         ① 磁铁没跟着轴转（没固定死、装在了外壳侧、或轴与磁铁之间打滑）")
    print("         ② 磁铁离芯片太远/太近（大多数要求轴向 0.5~3 mm）")
    print("         ③ 磁铁充磁方向不对（轴端编码器必须用**径向/对径充磁**的圆片；")
    print("            轴向充磁的磁铁在芯片处的面内分量几乎不转，角度就会卡住）")
    print("         ④ 芯片没对准磁铁中心（偏心 >0.5 mm 就开始烂）")
    print("       最快的判定：**拿一块小磁铁捏在手里，在芯片正上方慢慢转一圈**。")
    print("       若这时读数平滑 0→360 循环 → 芯片和读取都是好的，问题 100% 在装配上。")
elif len(vals) <= 1:
    print("[结论] 转到了，但中间没停够 —— 重跑一次，每个端点多停 1 秒。")
else:
    vmin, vmax = min(vals), max(vals)
    print(f"[结论] 静止值范围 {vmin:.0f} ~ {vmax:.0f}（跨度 {vmax-vmin:.0f} 计数 = "
          f"{(vmax-vmin)/16384*360:.1f}°），共 {len(vals)} 个静止段。")
    if vmax - vmin < 30:
        print("       → 各个位置停下时读数几乎一样 → 传感器基本没看到转动 → 装配问题（见上）。")
    else:
        print("       → 看上面 ① 表的差值列：")
        print("          · 若差值是 +x / −x 交替、且同一端点能回到同一值 → 机构回差（可接受，需标定）")
        print("          · 若差值一路同号累积（棘轮）→ 磁铁/联轴器打滑")
print("=" * 72)
s.close()
