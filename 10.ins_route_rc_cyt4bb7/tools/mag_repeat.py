"""把 mag_live.csv 切成"平台段"（停住不动的那些段），看端点的重复性。

为什么不用 mag_live.py 里那张静止段表：它的判据（连续 15 帧跨度 ≤3）太脆，
一转就断、又会把相邻的两段并起来。这里改成"滚动窗口找最长平台"，并且**按值聚类**成
"低端 / 高端"两组 —— 组内离散度就是**重复性**，这才是舵机反馈真正要的那个数。

用法：python mag_repeat.py [csv路径] [平台判据：允许跨度, 默认4]
"""
import os
import sys

CSV = sys.argv[1] if len(sys.argv) > 1 else \
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "mag_live.csv")
TOL = int(sys.argv[2]) if len(sys.argv) > 2 else 4
MIN_MS = 500                      # 平台至少持续这么久（车端 ms）

rows = []
with open(CSV, encoding="utf-8") as f:
    f.readline()
    for line in f:
        p = line.strip().split(",")
        if len(p) >= 3:
            rows.append((int(p[0]), int(p[2])))
rows.sort()
ts = [r[0] for r in rows]
a = [r[1] for r in rows]
n = len(rows)
print(f"帧 {n}，车端 ts {ts[0]} → {ts[-1]}（{(ts[-1]-ts[0])/1000:.1f} s）")

# ---------- 平台检测 ----------
plats = []
i = 0
while i < n:
    j = i
    lo = hi = a[i]
    while j + 1 < n:
        nl, nh = min(lo, a[j + 1]), max(hi, a[j + 1])
        if nh - nl <= TOL:
            lo, hi = nl, nh
            j += 1
        else:
            break
    dur = ts[j] - ts[i]
    if dur >= MIN_MS:
        plats.append((ts[i], ts[j], dur, sum(a[i:j + 1]) / (j - i + 1), hi - lo))
        i = j + 1
    else:
        i += 1

print(f"\n=== 平台段（跨度 ≤{TOL} 个计数、持续 ≥{MIN_MS}ms）===")
print("   序号   起(s)    时长(ms)   均值        跨度   折算")
for k, (t0, t1, d, m, sp) in enumerate(plats):
    print(f"   {k:3d}  {t0/1000:7.1f}  {d:7d}  {m:9.1f}  ({m/16384*360:7.2f}°)  {sp}")

if len(plats) < 2:
    print("\n[结论] 只有不到两段停顿 —— 请在每个端点多停 1~2 秒再重跑。")
    sys.exit(0)

# ---------- 按值聚类成低端/高端 ----------
vals = sorted(p[3] for p in plats)
mid = (vals[0] + vals[-1]) / 2
low = [p for p in plats if p[3] < mid]
high = [p for p in plats if p[3] >= mid]


def rep(name, g):
    if not g:
        return None
    v = [p[3] for p in g]
    print(f"\n{name}：{len(g)} 段   均值 {sum(v)/len(v):9.1f} ({sum(v)/len(v)/16384*360:6.2f}°)")
    print(f"   最小 {min(v):9.1f}  最大 {max(v):9.1f}  "
          f"离散 {max(v)-min(v):6.1f} 计数 = {(max(v)-min(v))/16384*360:6.2f}°")
    return sum(v) / len(v)


print("\n" + "=" * 72)
lo_m = rep("低端（一端机械限位）", low)
hi_m = rep("高端（另一端机械限位）", high)
print("=" * 72)

if lo_m is not None and hi_m is not None:
    swing = hi_m - lo_m
    print(f"\n行程：{lo_m:.0f} → {hi_m:.0f}  跨度 {swing:.0f} 计数 = {swing/16384*360:.1f}°")
    print(f"       （机械行程 ±30° ≈ 60°；若读数接近 60° 就是对的，明显小就是磁场被压）")
    spread = max(max(p[3] for p in low) - min(p[3] for p in low),
                 max(p[3] for p in high) - min(p[3] for p in high))
    print(f"\n★ 端点重复性：{spread:.0f} 计数 = {spread/16384*360:.2f}°")
    if spread < 100:
        print("   → **很好**，同一个机械限位每次都回到同一个读数（<2°）→ 直接可以标定使用。")
    elif spread < 400:
        print("   → 一般（2~9°）：可能是机构回差/装配旷量，也可能是磁场不稳。做舵机够用但要留余量。")
    else:
        print("   → **差**（>9°）：同一位姿两次读数差这么多 → 先解决重复性再谈标定。")
