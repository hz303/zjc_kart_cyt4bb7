"""磁编码器数据体检：静止时逐位分析 + 转动时覆盖范围测量。

两种模式：
    python mag_sweep.py rest  [秒数]      # 静止分析（默认 8 秒），全自动
    python mag_sweep.py sweep [秒数]      # 带提示的转动采集（默认 25 秒），给人看的

为什么做逐位分析：
    "静止时读数与机械位置无关、只在一个窄区间里随机"这种症状，
    要么是传感器没看到磁铁转（磁铁磁化方向/气隙/偏心），
    要么是读回来的帧本身不是角度（标志位/位错位）。
    逐位统计能直接把这两种分开：
      · 有若干位**恒定**（尤其高位/低位）= 那不是角度数据，是标志位或位错位 → 先修读取
      · 所有 15 位都在动、只是幅度小 = 帧没问题，是磁场的事 → 查磁铁/机械

输出同时写一份 CSV，方便事后细看。
"""
import collections
import os
import socket
import struct
import sys
import time

MODE = sys.argv[1] if len(sys.argv) > 1 else "rest"
SECS = float(sys.argv[2]) if len(sys.argv) > 2 else (8.0 if MODE == "rest" else 25.0)

PORT = 8086
HERE = os.path.dirname(os.path.abspath(__file__))
CSV = os.path.join(HERE, "mag_sweep.csv")

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
try:
    s.bind(("0.0.0.0", PORT))
except OSError as e:
    print(f"[ERR] 绑定 :{PORT} 失败：{e}")
    print("      → 上位机（KartHost）正开着的话点一下「断开」，或先关掉。")
    sys.exit(1)
s.settimeout(0.5)

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

peer = None
tok = 0
rows = []          # (墙钟秒, 车端ts, raw, angle, deg, delta)


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
        if 5 + plen + 2 > len(data):
            continue
        if struct.unpack_from("<H", data, 5 + plen)[0] != crc16(data[2:], 3 + plen):
            continue
        peer = addr
        if ftype == 0x01 and plen >= 10:
            body = data[5:5 + plen]
            if struct.unpack_from("<H", body, 8)[0] >= 16:
                ts = struct.unpack_from("<I", body, 2)[0]
                ch = struct.unpack_from("<4f", body, 10 + 12 * 4)
                rows.append((time.time() - t0, ts, int(ch[0]), int(ch[1]), ch[2], ch[3]))


pump(3.0)
if peer is None:
    print("[ERR] 收不到车端遥测 —— 先确认板子在发、且 MOTOR_HOST_IP 指到这台机器。")
    sys.exit(2)
tok += 1
s.sendto(build_cmd(0x03, 6, 0.0, tok), peer)      # mag_proto = 0
pump(0.6)
print(f"[OK] 车端 {peer[0]}:{peer[1]}，已把「编码器协议」设为 0\n")

if MODE == "sweep":
    print("=" * 70)
    print(" 接下来 25 秒，请按这个节奏操作（不用掐秒表，大致就行）：")
    print("   ① 前 5 秒：**完全别碰**，让转向停住")
    print("   ② 第 5~15 秒：把转向从最左**缓慢匀速**推到最右，再推回来")
    print("   ③ 最后 10 秒：再停住别碰")
    print("=" * 70)
    print()
    for i in (5, 4, 3, 2, 1):
        print(f"   {i} ...")
        time.sleep(1.0)
    print("   开始！")
    sys.stdout.flush()
    # ★ 倒计时这 5 秒没读 socket，帧堆在内核缓冲区里；不清掉的话
    #   它们会在下面被一次性读出，把"静止"和"转动"混在一起（踩过）。
    #   另外下面一律用**车端 ts** 分桶，因为它才是单调的真实时间轴。
    rows.clear()

t0 = time.time()
pump(SECS)
dur = time.time() - t0

raws = [r[2] for r in rows]
if not raws:
    print("[ERR] 这一段没收到遥测帧")
    sys.exit(3)

n = len(raws)
uniq = sorted(set(raws))
print(f"共 {n} 帧 / {dur:.1f} s（{n/dur:.1f} 帧/秒）")
print(f"取值种类 {len(uniq)}  最小 {min(raws)} (0x{min(raws):04X})  最大 {max(raws)} (0x{max(raws):04X})"
      f"  极差 {max(raws) - min(raws)}")
th = [x / 32768 * 360 for x in raws]
print(f"折算角度 {min(th):.1f}° ~ {max(th):.1f}°（跨度 {max(th)-min(th):.1f}°）")

# ---------- 逐位统计 ----------
print()
print("--- 逐位统计（1 的出现比例）---")
always1, always0, moving = [], [], []
for b in range(15, -1, -1):
    c = sum((v >> b) & 1 for v in raws)
    r = c / n
    tag = "恒1" if c == n else ("恒0" if c == 0 else "")
    if c == n:
        always1.append(b)
    elif c == 0:
        always0.append(b)
    else:
        moving.append(b)
    print(f"   bit{b:2d}  {r*100:6.1f}%  {tag}")
print()
print(f"  恒为 1 的位: {always1 or '无'}")
print(f"  恒为 0 的位: {always0 or '无'}")
print(f"  在变化的位: {moving}")

# ---------- 逐秒跨度（一眼看出"哪几秒在动、动多大"）----------
print()
print("--- 逐秒跨度（raw 的最小/最大/跨度；跨度大 = 那一秒在转）---")
buckets = collections.defaultdict(list)
for r in rows:
    buckets[r[1] // 1000].append(r[2])
for sec in sorted(buckets):
    v = buckets[sec]
    lo, hi = min(v), max(v)
    bar = "#" * min(60, int((hi - lo) / 32768 * 60))
    print(f"   {sec:3d}s  min {lo:6d}  max {hi:6d}  跨度 {hi-lo:6d}  {bar}")

# ---------- 跳变幅度 ----------
d = [abs(raws[i] - raws[i - 1]) for i in range(1, n)]
d.sort()
print()
print(f"--- 相邻帧跳变（50 Hz 采样，20ms 一步）---")
print(f"  中位 {d[n//2] if False else d[len(d)//2]}  90% {d[int(len(d)*0.9)]}  最大 {d[-1]}"
      f"   （满量程 32768，{max(d)/32768*100:.1f}%）")

# ---------- 写 CSV ----------
with open(CSV, "w", encoding="utf-8") as f:
    f.write("t_s,car_ts_ms,raw,angle,deg,delta\n")
    for r in rows:
        f.write(f"{r[0]:.3f},{r[1]},{r[2]},{r[3]},{r[4]:.2f},{r[5]:.0f}\n")
print()
print(f"[CSV] {CSV}")

# ---------- 结论 ----------
print()
print("=" * 70)
if len(always1) + len(always0) >= 1:
    print(f"[结论] ⚠ 有 {len(always1)+len(always0)} 个位**恒定不变化**（恒1={always1}，恒0={always0}）。")
    print("       一个真实的角度字段不可能整位不动 → **读回来的帧不是纯角度**（含标志位），")
    print("       或位对齐错了。先解决读取，再谈磁铁。")
elif max(raws) - min(raws) > 20000:
    print("[结论] ✅ 取值几乎覆盖满量程 → 帧是像样的角度数据，位对齐没问题。")
    print("       接下来只看：同一个机械位置、两次测量是否**可重复**（不可重复 = 磁铁打滑/装配松动）。")
elif max(raws) - min(raws) < 3000:
    print(f"[结论] ⚠ 全部取值挤在 {max(raws)-min(raws)} 个计数内（≈{(max(raws)-min(raws))/32768*360:.1f}°）。")
    print("       也就是说**转动没有引起角度变化** → 传感器看不到磁铁在转。按可能性排：")
    print("       ① 磁铁的充磁方向不对：轴上编码器必须用**径向（对径）充磁**的磁铁，")
    print("          轴向充磁的磁铁在芯片处的面内分量几乎不旋转 → 角度就卡住不动")
    print("       ② 磁铁没对准芯片中心 / 气隙不合适（多数要求轴向 0.5~3 mm，偏心 >0.5 mm 就开始烂）")
    print("       ③ 磁铁**没跟着轴转**（打滑、胶没粘住、联轴器松）")
    print("       ④ 电机铁磁外壳 + 电流磁场把磁铁的场压掉了（编码器贴电机装时很常见）")
else:
    print(f"[结论] 取值跨度 {max(raws)-min(raws)} 个计数（{max(raws)-min(raws):.1f}/32768），介于两者之间。")
    print("       先看是否为「部分覆盖 + 跳变很大」——那更像磁场被干扰/磁铁偏心，而不是没信号。")
print("=" * 70)
s.close()
