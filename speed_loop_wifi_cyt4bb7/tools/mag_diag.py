"""分析 mag_csv.csv：mag_delta 为什么这么剧烈。

用法：
    python mag_diag.py            # 读 tools/mag_csv.csv
    python mag_diag.py 某文件.csv
"""
import collections
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
PATH = sys.argv[1] if len(sys.argv) > 1 else os.path.join(HERE, "mag_csv.csv")

rows = []
with open(PATH, encoding="utf-8") as f:
    hdr = f.readline().strip().split(",")
    for line in f:
        p = line.strip().split(",")
        if len(p) < 7:
            continue
        rows.append([int(p[1]), int(p[2]), int(p[3]), float(p[4]), int(p[5]), int(p[6])])

print("=" * 74)
print(f"文件 {os.path.basename(PATH)}   共 {len(rows)} 帧")
print("=" * 74)

d = [r[4] for r in rows]
a = [r[2] for r in rows]          # mag_angle
raw = [r[1] for r in rows]        # mag_raw

print(f"mag_delta 范围 {min(d)} ~ {max(d)}     |delta|>500 的帧 {sum(1 for x in d if abs(x)>500)}"
      f"  ({sum(1 for x in d if abs(x)>500)/len(d)*100:.1f}%)")
print(f"mag_angle 范围 {min(a)} ~ {max(a)}   （14 位满量程 16384）")
print()

# ---------- ① delta 的取值分布 ----------
print("① |delta| 分档")
bins = [0, 1, 5, 20, 100, 500, 1000, 2000, 4000, 8000, 20000]
for i in range(len(bins) - 1):
    n = sum(1 for x in d if bins[i] <= abs(x) < bins[i + 1])
    bar = "#" * min(50, int(n / max(1, len(d)) * 200))
    print(f"   {bins[i]:6d} ~ {bins[i+1]:6d} : {n:6d}  {bar}")
print()

print("② 出现最多的 delta 值（前 15）")
for v, n in collections.Counter(d).most_common(15):
    print(f"   {v:8d}  × {n}")
print()

# ---------- ③ 大跳变是"单帧毛刺"还是"台阶" ----------
print("③ 大跳变（|delta|>500）之后，下一帧有没有跳回来？")
glitch, step = [], []
i = 1
while i < len(rows):
    if abs(d[i]) > 500:
        back = abs(a[i + 1] - a[i - 1]) < 60 if i + 1 < len(rows) else False
        (glitch if back else step).append(i)
    i += 1
print(f"   单帧毛刺（跳出去又跳回来）：{len(glitch)} 次")
print(f"   台阶式变化（跳出去就留在那儿）：{len(step)} 次")
print()

print("④ 前 8 个大跳变的现场（mag_raw / mag_angle）")
shown = 0
for i in glitch + step:
    if shown >= 8:
        break
    lo, hi = max(0, i - 1), min(len(rows), i + 3)
    seq = "  ".join(f"{a[j]}" for j in range(lo, hi))
    tag = "毛刺" if i in glitch else "台阶"
    print(f"   [{tag}] 帧{i}: ... {seq} ...   (raw {raw[i-1]} → {raw[i]} → {raw[i+1] if i+1<len(rows) else '-'})")
    shown += 1
print()

# ---------- ⑤ 大跳变时 mag_angle 是不是"均匀乱" ----------
big = [a[i] for i in range(len(rows)) if abs(d[i]) > 500]
if big:
    print(f"⑤ 大跳变那 292 帧里 mag_angle 的分布（共 {len(big)} 帧）")
    for lo in range(0, 16384, 2048):
        n = sum(1 for v in big if lo <= v < lo + 2048)
        print(f"   {lo:6d}~{lo+2048:6d} : {n:5d}  {'#' * int(n/max(1,len(big))*60)}")
    print("   → 若各段都有、且大致平均 = 读数在这时是**乱的**（不是真角度）")
print()

# ---------- ⑥ 静止帧的稳定性 ----------
print("⑥ 静止帧（|delta|<=2 的那些）的 mag_angle 都落在哪儿（前 12 个簇）")
still = collections.Counter(a[i] for i in range(len(rows)) if abs(d[i]) <= 2)
tot = sum(still.values())
print(f"   静止帧共 {tot} 帧")
GROUP = 64
g = collections.Counter()
for v, n in still.items():
    g[v // GROUP * GROUP] += n
for v, n in g.most_common(12):
    print(f"   raw {v:6d} ~ {v+GROUP:6d} : {n:5d} 帧  {'#' * int(n/max(1,tot)*60)}")
print()

# ---------- 结论 ----------
n_big = sum(1 for x in d if abs(x) > 500)
print("=" * 74)
print(f"|delta| 中位 {sorted(abs(x) for x in d)[len(d)//2]}，最大 {max(abs(x) for x in d)}")
print(f"毛刺 {len(glitch)} 次 / 台阶 {len(step)} 次  →  毛刺占比 "
      f"{len(glitch)/max(1,len(glitch)+len(step))*100:.0f}%")
print("=" * 74)
