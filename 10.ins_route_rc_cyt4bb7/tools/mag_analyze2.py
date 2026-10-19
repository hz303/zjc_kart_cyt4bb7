"""第二段分析：① 车端时间戳的真实步长 ② 用「卷绕感知的相邻差分」定出正确的位宽。

判据：真实角度在慢转时相邻帧差值很小且**方向一致**；
若把一个恒定的高位当成角度的一部分，那么每转一圈就会多出一次**假的大跳变**
（真角 16383→0 时，带偏移的 15 位值会 0x7FFF→0x4000，跳 -16383）。
所以看"大跳变次数"，越少越对。
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
            rows.append((float(p[0]), int(p[1]), int(p[2])))

ts = [r[1] for r in rows]
raws = [r[2] for r in rows]
n = len(rows)

# ---------- ① ts 步长 ----------
print("=== ① 车端时间戳（=motor_tick_ms，单位应为 ms）步长 ===")
d = [ts[i] - ts[i - 1] for i in range(1, n)]
print("   前 20 个步长:", d[:20])
h = collections.Counter(d)
print("   步长分布（前 8 种）:", h.most_common(8))
print(f"   平均 {sum(d)/len(d):.2f} ms，最小 {min(d)}，最大 {max(d)}")
print(f"   帧率 {len(d)/(rows[-1][0]-rows[0][0]):.1f} FPS")
nominal = [x for x in d if 0 < x < 100]
if nominal:
    print(f"   去掉异常后平均 {sum(nominal)/len(nominal):.2f} ms")
print("   → 若步长不是 5 的整数倍，说明 motor_tick_ms 不是只由 5ms PIT 累加，")
print("     或 PIT 实际周期 ≠ 5ms（会让 ramp / 失联判定全部偏掉）")

# ---------- ② 卷绕感知差分，比较各种位宽 ----------
print()
print("=== ② 卷绕感知差分（判正确的角度位段）===")
print("   位宽  满量程   中位|Δ|   p90|Δ|   大跳变次数(>10%满量程)  占比")


def wrap_delta(a, b, mod):
    x = (b - a) % mod
    if x > mod // 2:
        x -= mod
    return x


for width in (11, 12, 13, 14, 15, 16):
    mod = 1 << width
    mask = mod - 1
    a = [v & mask for v in raws]        # shift = 0：从最低位起取 width 位
    ds = [wrap_delta(a[i - 1], a[i], mod) for i in range(1, n)]
    ad = sorted(abs(x) for x in ds)
    big = sum(1 for x in ad if x > mod * 0.1)
    print(f"   {width:4d}  {mod:7d}  {ad[len(ad)//2]:8d}  {ad[int(len(ad)*0.9)]:8d}"
          f"   {big:8d}   {big/len(ad)*100:5.1f}%")

# 也试一下"跳过 k 个低位/高位"的组合
print()
print("   位移 位宽  满量程   中位|Δ|   p90|Δ|   大跳变次数  占比")
for shift in (1, 2):
    for width in (12, 13, 14):
        mod = 1 << width
        mask = mod - 1
        a = [(v >> shift) & mask for v in raws]
        ds = [wrap_delta(a[i - 1], a[i], mod) for i in range(1, n)]
        ad = sorted(abs(x) for x in ds)
        big = sum(1 for x in ad if x > mod * 0.1)
        print(f"   {shift:4d} {width:4d}  {mod:7d}  {ad[len(ad)//2]:8d}  {ad[int(len(ad)*0.9)]:8d}"
              f"   {big:8d}   {big/len(ad)*100:5.1f}%")

# ---------- ③ 取最佳位宽，看方向一致性 ----------
print()
print("=== ③ 14 位（0x3FFF）下的运动方向一致性 ===")
mod, mask = 1 << 14, 0x3FFF
a = [v & mask for v in raws]
ds = [wrap_delta(a[i - 1], a[i], mod) for i in range(1, n)]
moving = [x for x in ds if abs(x) > 5]
pos = sum(1 for x in moving if x > 0)
print(f"   有效运动帧 {len(moving)}，其中正向 {pos}（{pos/max(len(moving),1)*100:.1f}%）、"
      f"反向 {len(moving)-pos}")
print(f"   |Δ| 中位 {sorted(abs(x) for x in moving)[len(moving)//2] if moving else 0}"
      f"，最大 {max(abs(x) for x in moving) if moving else 0}")
print("   （方向来回摆动属正常：人手来回转；关键是**没有异常大的单步跳变堆在一起**）")
