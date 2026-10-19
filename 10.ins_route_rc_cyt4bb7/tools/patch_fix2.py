"""修两处：
① TravelCalPanel 的按钮 MinimumSize 被二次缩放（我传了已缩放的 Dpi.Px()，MkBtn 里又缩一次）
② SelfTest 的"滑条 ↔ 数字框"断言把标定面板的"机械行程"数字框也算进去了 → 改成只数参数面板内部
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


# ---------------- ① 按钮宽度：传逻辑值 ----------------
p = os.path.join(ROOT, "Ui/TravelCalPanel.cs")
t, enc, eol = load(p)

t = sub(t, """            _btnLeft  = MkBtn("记左限位",     Dpi.Px(100), Dpi.Px(6),   Dpi.Px(26));
            _btnRight = MkBtn("记右限位",     Dpi.Px(100), Dpi.Px(122), Dpi.Px(26));
            _btnClear = MkBtn("清空",         Dpi.Px(70),  Dpi.Px(6),   Dpi.Px(58));
            _btnZero  = MkBtn("当前位置→0°",  Dpi.Px(120), Dpi.Px(92),  Dpi.Px(58));""",
        """            // ⚠ 第 2 个参数是**逻辑**宽度（MkBtn 里会过 Dpi.Sz），第 3/4 个是**物理**坐标（已过 Dpi.Px）。
            //   一开始我两个都传了 Dpi.Px()，结果宽度被缩放两次 → 按钮右沿越出侧栏。
            _btnLeft  = MkBtn("记左限位",    96,  Dpi.Px(6),   Dpi.Px(26));
            _btnRight = MkBtn("记右限位",    96,  Dpi.Px(112), Dpi.Px(26));
            _btnClear = MkBtn("清空",        66,  Dpi.Px(6),   Dpi.Px(58));
            _btnZero  = MkBtn("当前位置→0°", 130, Dpi.Px(84),  Dpi.Px(58));""",
        "按钮宽度改逻辑值")

t = sub(t, """        private static Button MkBtn(string text, int minW, int x, int y) => new Button
        {
            Text = text,
            AutoSize = true,
            AutoSizeMode = AutoSizeMode.GrowAndShrink,
            MinimumSize = Dpi.Sz(minW, 26),
            Location = new Point(x, y),
        };""",
        """        /// <param name="logicalMinW">最小宽度，**逻辑值**（内部会过 Dpi）</param>
        /// <param name="x">已经过 Dpi.Px 的物理 x</param>
        /// <param name="y">已经过 Dpi.Px 的物理 y</param>
        private static Button MkBtn(string text, int logicalMinW, int x, int y) => new Button
        {
            Text = text,
            AutoSize = true,
            AutoSizeMode = AutoSizeMode.GrowAndShrink,
            MinimumSize = Dpi.Sz(logicalMinW, 26),
            Location = new Point(x, y),
        };""",
        "MkBtn 参数语义写清楚")
save(p, t, enc, eol)

# ---------------- ② SelfTest：滑条统计限定在参数面板内 ----------------
p = os.path.join(ROOT, "SelfTest.cs")
t, enc, eol = load(p)
t = sub(t, """            // 数一下参数滑条（应从参数表自动生成）
            int bars = CountType<TrackBar>(f);
            int nums = CountType<NumericUpDown>(f);""",
        """            // 数一下参数滑条（应从参数表自动生成）
            // ⚠ 只数**参数面板内部**的 —— 之前是全窗体统计，行车标定面板加了个
            //   "机械行程"数字框，这条就误报了（26 vs 25）。断言该贴着的对象数，别数全局。
            var pps = new List<Ui.ParamPanel>();
            CollectType(f, pps);
            Check(pps.Count == 1, "找得到参数面板", "实际 " + pps.Count);
            int bars = pps.Count > 0 ? CountType<TrackBar>(pps[0]) : 0;
            int nums = pps.Count > 0 ? CountType<NumericUpDown>(pps[0]) : 0;""",
        "滑条统计限定在参数面板")
save(p, t, enc, eol)
print("两处修完")
