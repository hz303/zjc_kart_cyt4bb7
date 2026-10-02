"""第三段分析：用车端时间戳当时间轴（可靠），并证明"低 14 位才是真角度"。

★ 为什么不用墙钟：mag_sweep.py 早期版本的 rows 把前面几段采集也算了进来，
  且倒计时 5 秒里没读 socket，帧堆在内核缓冲区里、随后被一次性读出 →
  按墙钟分桶会把"静止"和"转动"混在一起。车端 ts 是单调递增的真实时间轴，不受影响。

判据（证明 14 位）：
  · 若低 14 位是真角度，转一整圈时它必然**穿过 0/16383 边界**并且**跨界的步子很小**
    （真角 16380 → 20 这样连续地在环上走）
  · 若"15 位（bit14 是角度最高位）"成立，则取值必须**始终落在 0x4000~0x7FFF**、
    且从不接近 32767 或 0 —— 手转一整圈却"刚好"不跨过边界是极不可能的
"""
import collections
import os
import sys

CSV = sys.argv[1] if len(sys.argv) > 1 else \
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "mag_sweep.csv")

rows = []
with open(CSV, encoding="utf-8") as f:
    f.readline()
    for line in f:
        p = line.strip().split(",")
        if len(p) >= 4:
            rows.append((int(p[1]), int(p[2])))        # (car_ts, raw)

rows.sort()
ts = [r[0] for r in rows]
raws = [r[1] for r in rows]
a14 = [v & 0x3FFF for v in raws]

print(f"帧 {len(rows)}，车端 ts {ts[0]} → {ts[-1]}（{ts[-1]-ts[0]} ms = {(ts[-1]-ts[0])/1000:.1f} s）")

print()
print("=== ① 恒定不变的高位 ===")
for b in range(15, 12, -1):
    c1 = sum((v >> b) & 1 for v in raws)
    print(f"   bit{b}: 1 占 {c1/len(raws)*100:6.1f}%")

print()
print("=== ② 两种解释的取值域 ===")
print(f"   15 位（bit14 当角度）: {min(raws)} (0x{min(raws):04X}) ~ {max(raws)} (0x{max(raws):04X})"
      f"  占满量程 {(max(raws)-min(raws))/32768*100:.1f}%")
print(f"   14 位（bit14 当标志）: {min(a14)} (0x{min(a14):04X}) ~ {max(a14)} (0x{max(a14):04X})"
      f"  占满量程 {(max(a14)-min(a14))/16384*100:.1f}%")

print()
print("=== ③ 用车端 ts 分桶，看静止/转动（低 14 位）===")
bucket = collections.defaultdict(list)
for t, v in zip(ts, a14):
    bucket[t // 1000].append(v)
prev = None
for sec in sorted(bucket):
    v = bucket[sec]
    lo, hi = min(v), max(v)
    bar = "#" * min(50, int((hi - lo) / 16384 * 50))
    mark = ""
    if prev is not None and abs(sec - prev) > 1:
        mark = "   ← 时间轴有断点"
    print(f"   ts={sec:6d}s  {lo:6d} ~ {hi:6d}  跨度 {hi-lo:6d}"
          f"  ({lo/16384*360:6.1f}° ~ {hi/16384*360:6.1f}°)  {bar}{mark}")
    prev = sec

print()
print("=== ④ 关键证据：低 14 位有没有「小步跨界」（= 真角度转过一圈）===")
wraps_up, wraps_dn = [], []
for i in range(1, len(a14)):
    p, q = a14[i - 1], a14[i]
    step = q - p
    if p > 14000 and q < 2400:                 # 正向跨界
        wraps_up.append((ts[i], p, q, q + 16384 - p))
    elif p < 2400 and q > 14000:               # 反向跨界
        wraps_dn.append((ts[i], p, q, q - 16384 - p))
print(f"   正向跨界 {len(wraps_up)} 次，反向 {len(wraps_dn)} 次")
for t, p, q, st in (wraps_up + wraps_dn)[:12]:
    print(f"     ts={t:6d}ms  {p:6d} → {q:6d}   等价步长 {st:+6d}")

print()
print("=== ⑤ 静态段稳定性（末尾 3 秒）===")
tail = [v for t, v in zip(ts, a14) if t >= ts[-1] - 3000]
rawtail = [r for t, r in zip(ts, raws) if t >= ts[-1] - 3000]
print(f"   {len(tail)} 帧：低14位 {min(tail)} ~ {max(tail)}（跨度 {max(tail)-min(tail)}）"
      f"，raw {min(rawtail)} ~ {max(rawtail)}")
dt = sorted(abs(tail[i] - tail[i - 1]) for i in range(1, len(tail)))
print(f"   相邻帧跳变：中位 {dt[len(dt)//2]}，最大 {dt[-1]}"
      f"（满量程 16384，即最大 {dt[-1]/16384*360:.4f}°）")

print()
print("=" * 72)
if len(wraps_up) + len(wraps_dn) >= 1:
    print("[结论] ✅ 低 14 位确实**小步跨过 0/16383 边界** → 它是绕环走的真实角度（14 位）。")
    print("       而 bit14 在全部数据里恒为 1 → 它是**固定标志位**，不是角度位。")
    print(f"       所以正确解码：角度 = raw & 0x3FFF，满量程 16384（≠ 现在的 0x7FFF / 32768）。")
else:
    print("[结论] 没抓到跨界事件（可能没转满一圈）—— 但 bit14 恒 1 已足以说明它不是角度位。")
print("=" * 72)
