"""从 mag_sweep.csv 反推"哪个位段才是真正的角度"。

思路：真实角度在同一方向转动时**相邻帧变化很小**；而错误位段会把高位掺进来，
造成大跳变。所以对所有 (位移, 位宽) 组合算"覆盖范围 + 相邻跳变"，找既能覆盖大范围、
跳变又最小的那一段。

用法：python mag_analyze.py [csv路径]
"""
import collections
import os
import sys

CSV = sys.argv[1] if len(sys.argv) > 1 else \
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "mag_sweep.csv")

rows = []
with open(CSV, encoding="utf-8") as f:
    head = f.readline()
    for line in f:
        p = line.strip().split(",")
        if len(p) < 4:
            continue
        rows.append((float(p[0]), int(p[1]), int(p[2])))     # t_s, car_ts, raw

n = len(rows)
print(f"帧数 {n}   时长 {rows[-1][0]:.2f} s   平均 {n / max(rows[-1][0], 1e-9):.1f} 帧/秒")

# ---- 车端时间戳是否在递增（分辨重复帧 / 假帧率）----
ts = [r[1] for r in rows]
dup = sum(1 for i in range(1, n) if ts[i] == ts[i - 1])
back = sum(1 for i in range(1, n) if ts[i] < ts[i - 1])
mono = sum(1 for i in range(1, n) if ts[i] > ts[i - 1])
print(f"车端 ts：递减 {back} 次，重复 {dup} 次，递增 {mono} 次"
      f"   → 真实上报周期约 {1000.0 / max(1, mono / (rows[-1][0] - rows[0][0])):.1f} ms")

raws = [r[2] for r in rows]

# ---- 恒定位 ----
const1 = [b for b in range(16) if all((v >> b) & 1 for v in raws)]
const0 = [b for b in range(16) if not any((v >> b) & 1 for v in raws)]
print(f"恒定位：恒1={const1}  恒0={const0}")

# ---- 逐个候选位段打分 ----
print()
print("--- 候选位段（跳变按「车端 ts 前进 20ms」的相邻帧统计）---")
print("   位移 位宽   覆盖范围   中位跳变   90%跳变   最大跳变   评价")
best = []
for shift in range(0, 5):
    for width in (12, 13, 14, 15, 16):
        if shift + width > 16:
            continue
        mask = (1 << width) - 1
        a = [(v >> shift) & mask for v in raws]
        rng = max(a) - min(a)
        d = [abs(a[i] - a[i - 1]) for i in range(1, n) if ts[i] - ts[i - 1] in (1, 2, 15, 20, 21, 40)]
        if not d:
            continue
        d.sort()
        med, p90, mx = d[len(d) // 2], d[int(len(d) * 0.9)], d[-1]
        # 质量分：覆盖越宽越好、跳变越小越好
        score = rng / (p90 + 1)
        best.append((score, shift, width, rng, med, p90, mx))
        print(f"   {shift:4d} {width:4d} {rng:10d} {med:10d} {p90:9d} {mx:10d}"
              f"   rng/p90={score:7.1f}")

best.sort(reverse=True)
print()
score, shift, width, rng, med, p90, mx = best[0]
mask = (1 << width) - 1
print(f"[最佳] 位移 {shift}、位宽 {width} → mask = 0x{mask:04X}，满量程 {1 << width}")
print(f"       覆盖 {rng} / {1 << width}（{rng / (1 << width) * 100:.1f}%），"
      f"90% 跳变 {p90}（占满量程 {p90 / (1 << width) * 100:.2f}%），最大跳变 {mx}")

# ---- 最佳位段的逐秒跨度 ----
a = [(v >> shift) & mask for v in raws]
bucket = collections.defaultdict(list)
for (t, _, _), av in zip(rows, a):
    bucket[int(t)].append(av)
print()
print(f"--- 用 0x{mask:04X} 解码后逐秒跨度 ---")
full = 1 << width
for sec in sorted(bucket):
    v = bucket[sec]
    lo, hi = min(v), max(v)
    bar = "#" * min(60, int((hi - lo) / full * 60))
    print(f"   {sec:3d}s  {lo:7d} ~ {hi:7d}  跨度 {hi - lo:7d}"
          f"  ({lo / full * 360:6.1f}° ~ {hi / full * 360:6.1f}°)  {bar}")

# ---- 静止段的稳定性（这是传感器质量的硬指标）----
quiet = [av for (t, _, _), av in zip(rows, a) if t >= 13]
if quiet:
    print()
    print(f"--- 末段静止（{len(quiet)} 帧）稳定性 ---")
    print(f"   {min(quiet)} ~ {max(quiet)}，跨度 {max(quiet) - min(quiet)} 个计数"
          f"（满量程 {full}，即 ±{(max(quiet) - min(quiet)) / 2 / full * 360:.4f}°）")
    dq = sorted(abs(quiet[i] - quiet[i - 1]) for i in range(1, len(quiet)))
    print(f"   相邻帧跳变：中位 {dq[len(dq)//2]}，最大 {dq[-1]}")
