"""把 WinForms 界面里的字面量尺寸统一包上 Dpi 换算层。

规则（只动"整数字面量"，不碰变量/表达式）：
  new Point(a, b)        -> new Point(Dpi.Px(a), Dpi.Px(b))
  new Size(a, b)         -> Dpi.Sz(a, b)
  Width = n / Height = n -> Dpi.Px(n)
  new Padding(a,b,c,d)   -> Dpi.Pad(a,b,c,d)

为什么要做：字号是点值（GDI+ 会按 DPI 放大），而像素尺寸不放大，
150% 缩放下就会"字大了框没大" → 文字被吃掉。
"""
import re
import sys

FILES = [
    r"D:/jisuyueye9car/KartHost/Ui/MainForm.cs",
    r"D:/jisuyueye9car/KartHost/Ui/ParamPanel.cs",
    r"D:/jisuyueye9car/KartHost/Ui/LampControl.cs",
    r"D:/jisuyueye9car/KartHost/Ui/WavePanel.cs",
]

NUM = r"-?\d+"
RULES = [
    ("Point", re.compile(r"new Point\(\s*(" + NUM + r")\s*,\s*(" + NUM + r")\s*\)"),
     lambda m: f"new Point(Dpi.Px({m.group(1)}), Dpi.Px({m.group(2)}))"),
    ("Size", re.compile(r"new Size\(\s*(" + NUM + r")\s*,\s*(" + NUM + r")\s*\)"),
     lambda m: f"Dpi.Sz({m.group(1)}, {m.group(2)})"),
    ("Wh", re.compile(r"\b(Width|Height)\s*=\s*(" + NUM + r")\s*;"),
     lambda m: f"{m.group(1)} = Dpi.Px({m.group(2)});"),
    ("Padding", re.compile(r"new Padding\(\s*(" + NUM + r")\s*,\s*(" + NUM + r")\s*,\s*(" + NUM + r")\s*,\s*(" + NUM + r")\s*\)"),
     lambda m: f"Dpi.Pad({m.group(1)}, {m.group(2)}, {m.group(3)}, {m.group(4)})"),
]


def main():
    total = {}
    for path in FILES:
        try:
            src = open(path, encoding="utf-8-sig").read()
        except FileNotFoundError:
            print(f"[跳过] {path} 不存在")
            continue
        counts = {}
        for name, rx, rep in RULES:
            src, n = rx.subn(rep, src)
            if n:
                counts[name] = n
        if counts:
            open(path, "w", encoding="utf-8").write(src)
        print(f"{path.split('/')[-1]:16s} {counts}")
        for k, v in counts.items():
            total[k] = total.get(k, 0) + v
    print("\n合计:", total)


if __name__ == "__main__":
    sys.exit(main())
