"""修波形工具条 5 个按钮的剪字（差 2px 高）+ 把工具条加高。

体检判据：Button 的 PreferredSize 高 32，实际只有 Dpi.Px(24)=30（实测 DPI=120 → 1.25x）。
把按钮 24 → 26、工具条 30 → 34、y 偏移 3 → 4，两个缩放下都留余量。
"""
import os

p = r"D:/jisuyueye9car/KartHost/Ui/MainForm.cs"
raw = open(p, "rb").read()
t = raw.decode("utf-8-sig")
eol = "\r\n" if "\r\n" in t else "\n"
t = t.replace("\r\n", "\n")

pairs = [
    # 工具条加高
    ('var waveBar = new Panel { Dock = DockStyle.Top, Height = Dpi.Px(30), BackColor = Theme.Panel };',
     'var waveBar = new Panel { Dock = DockStyle.Top, Height = Dpi.Px(34), BackColor = Theme.Panel };'),
    # 5 个按钮：高度 24 → 26，y 3 → 4
    ('var b1 = new Button { Text = "1×1", Location = new Point(Dpi.Px(44), Dpi.Px(3)), Width = Dpi.Px(46), Height = Dpi.Px(24) };',
     'var b1 = new Button { Text = "1×1", Location = new Point(Dpi.Px(44), Dpi.Px(4)), Width = Dpi.Px(46), Height = Dpi.Px(26) };'),
    ('var b2 = new Button { Text = "2×1", Location = new Point(Dpi.Px(94), Dpi.Px(3)), Width = Dpi.Px(46), Height = Dpi.Px(24) };',
     'var b2 = new Button { Text = "2×1", Location = new Point(Dpi.Px(94), Dpi.Px(4)), Width = Dpi.Px(46), Height = Dpi.Px(26) };'),
    ('var b4 = new Button { Text = "2×2", Location = new Point(Dpi.Px(144), Dpi.Px(3)), Width = Dpi.Px(46), Height = Dpi.Px(24) };',
     'var b4 = new Button { Text = "2×2", Location = new Point(Dpi.Px(144), Dpi.Px(4)), Width = Dpi.Px(46), Height = Dpi.Px(26) };'),
    ('_btnPause = new Button { Text = "暂停", Location = new Point(Dpi.Px(428), Dpi.Px(3)), Width = Dpi.Px(58), Height = Dpi.Px(24) };',
     '_btnPause = new Button { Text = "暂停", Location = new Point(Dpi.Px(428), Dpi.Px(4)), Width = Dpi.Px(58), Height = Dpi.Px(26) };'),
    ('var btnPreset = new Button { Text = "套用预设分图", Location = new Point(Dpi.Px(494), Dpi.Px(3)), Width = Dpi.Px(108), Height = Dpi.Px(24) };',
     'var btnPreset = new Button { Text = "套用预设分图", Location = new Point(Dpi.Px(494), Dpi.Px(4)), Width = Dpi.Px(108), Height = Dpi.Px(26) };'),
]

for old, new in pairs:
    n = t.count(old)
    assert n == 1, ("没命中或重复：%s（%d 次）" % (old[:60], n))
    t = t.replace(old, new, 1)
    print("  ✔", old[:52])

if eol == "\r\n":
    t = t.replace("\n", "\r\n")
open(p, "wb").write(t.encode("utf-8-sig"))
print("MainForm.cs 已写回（%s）" % eol.strip())
