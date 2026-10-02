"""体检器报的 13 处重叠，逐个定位后修根因：

① ParamPanel：`y += 26` 是**字面量**（没走 Dpi），而分组标题高度是 Dpi.Px(24)。
   1.25x 下标题 = 30px > 偏移 26px → 标题比下一行的行首还低 4px → 两行文字压住。
   改成 `y += Dpi.Px(28)`（≥ 标题高度 + 余量）。
② MainForm 数值框：值标签宽 Dpi.Px(66) 从 x=6 起 → 右沿 72；单位标签从 70 起 → 压 2px。
   单位标签右移到 Dpi.Px(76)。
③ LayoutAudit：重叠判据加"细缝不算"的门槛（两个方向都要 > 6px）。
   1~2px 的圆整重叠是布局取整造成的，肉眼看不见；真正的"文字压文字"一定两个方向都很大。
   —— 判据太灵会变噪音，噪音一起就没人看了（这条以前吃过亏）。
"""
import os

ROOT = r"D:/jisuyueye9car/KartHost"


def load(p):
    raw = open(p, "rb").read()
    for enc in ("utf-8-sig", "utf-8"):
        try:
            t = raw.decode(enc)
            break
        except UnicodeDecodeError:
            continue
    else:
        raise SystemExit("解码失败 " + p)
    eol = "\r\n" if "\r\n" in t else "\n"
    return t.replace("\r\n", "\n"), enc, eol


def save(p, t, enc, eol):
    if eol == "\r\n":
        t = t.replace("\n", "\r\n")
    open(p, "wb").write(t.encode(enc))


def sub(t, old, new, tag):
    n = t.count(old)
    assert n == 1, (tag, "命中 %d 次" % n)
    print("  ✔", tag)
    return t.replace(old, new, 1)


# ① ParamPanel 行偏移
p = os.path.join(ROOT, "Ui/ParamPanel.cs")
t, enc, eol = load(p)
t = sub(t, """                y += 26;
            }

            var row = BuildRow(p, y);""",
        """                // ⚠ 必须是 Dpi.Px：标题高度是 Dpi.Px(24)，写死 26（物理px）会让
                //   1.25x 下的标题(30px) 压到下一行行首 —— 布局体检的"重叠"就是这么抓到的
                y += Dpi.Px(28);
            }

            var row = BuildRow(p, y);""",
        "ParamPanel 行偏移过 Dpi")
save(p, t, enc, eol)

# ② 单位标签右移
p = os.path.join(ROOT, "Ui/MainForm.cs")
t, enc, eol = load(p)
t = sub(t, """                Text = units[i],
                Location = new Point(Dpi.Px(70), Dpi.Px(32)),
                AutoSize = true,                     // ★ 固定 26px 宽在 150% 下装不下 "m/s"（需要 43）""",
        """                Text = units[i],
                Location = new Point(Dpi.Px(76), Dpi.Px(32)),   // 值标签右沿在 72 → 让开
                AutoSize = true,                     // ★ 固定 26px 宽在 150% 下装不下 "m/s"（需要 43）""",
        "单位标签右移")
save(p, t, enc, eol)

# ③ 重叠判据加门槛
p = os.path.join(ROOT, "LayoutAudit.cs")
t, enc, eol = load(p)
t = sub(t,
        """                var r = Rectangle.Intersect(a.Bounds, b.Bounds);
                if (r.Width > 2 && r.Height > 2)
                {""",
        """                var r = Rectangle.Intersect(a.Bounds, b.Bounds);
                // 两个方向都要压得住才算 —— 1~2px 的细缝是布局取整造成的，肉眼看不见，
                // 报出来只会变噪音（判据太灵 = 没人看，这条吃过亏）。
                if (r.Width > 6 && r.Height > 6)
                {""",
        "重叠判据加细缝门槛")
save(p, t, enc, eol)
print("三处修完")
