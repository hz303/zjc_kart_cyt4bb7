"""修编译错误 + 给「行程标定」补自检断言。"""
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


# ---------------- ① TravelCalPanel：改颜色 + 删未用字段 ----------------
p = os.path.join(ROOT, "Ui/TravelCalPanel.cs")
t, enc, eol = load(p)
t = sub(t, "                ForeColor = Theme.TextInfo,", "                ForeColor = Theme.Accent,", "Theme.Accent")
t = sub(t, "        private readonly ComboBox _cbSource;      // 用哪一对通道（单电机档案=速度积分）\n",
        "", "删掉未用的 _cbSource")
save(p, t, enc, eol)

# ---------------- ② SelfTest：新增行程标定自检 ----------------
p = os.path.join(ROOT, "SelfTest.cs")
t, enc, eol = load(p)

t = sub(t, "        TestMotorProfile();          // 单电机验证档案（跑完会切回卡丁车档案）",
        "        TestMotorProfile();          // 单电机验证档案（跑完会切回卡丁车档案）\n"
        "        TestTravelCal();             // 行程标定（左右限位 → mrad/° 映射）",
        "挂到 Run()")

t = sub(t, "    /// <summary>静态统计控件树大小</summary>",
        '''    // ==================================================================
    // 行程标定（速度积分 → 机械角）
    // ==================================================================
    private static void TestTravelCal()
    {
        Section("行程标定（速度积分 → 机械角）");

        var cal = new Ui.TravelCal();
        Check(!cal.Complete, "只记一边时不算标定完成");
        Check(double.IsNaN(cal.AngleDeg(123)), "没标定时 AngleDeg 返回 NaN（不许瞎给数）");

        // 左 −2400 / 右 +2400 mrad，机械行程 60° → 每度 80 mrad
        cal.Capture(true, -2400, 10.0);
        cal.Capture(false, 2400, 40.0);
        Check(cal.Complete, "左右都记完 → 标定完成");
        Check(Math.Abs(cal.SpanSum - 4800) < 1e-6, "跨度 = 4800 mrad", cal.SpanSum.ToString("0"));
        Check(Math.Abs(cal.MradPerDeg - 80.0) < 1e-6, "映射 = 80 mrad/°（4800 ÷ 60°）", cal.MradPerDeg.ToString("0.00"));
        Check(Math.Abs(cal.MidSum) < 1e-6, "中点 = 0 mrad", cal.MidSum.ToString("0"));
        Check(Math.Abs(cal.AngleDeg(2400) - 30.0) < 1e-6, "右限位处 → +30°", cal.AngleDeg(2400).ToString("0.0"));
        Check(Math.Abs(cal.AngleDeg(-2400) + 30.0) < 1e-6, "左限位处 → -30°", cal.AngleDeg(-2400).ToString("0.0"));
        Check(Math.Abs(cal.AngleDeg(0)) < 1e-9, "中点处 → 0°");
        Check(Math.Abs(cal.Turns - 4800.0 / (2000.0 * Math.PI)) < 1e-9, "跨度折算 = 0.764 圈");
        Check(Math.Abs(cal.HalfSpanDeg - 30.0) < 1e-9, "限位 = ±30.0°", cal.HalfSpanDeg.ToString("0.0"));

        // ★ 反向对照：跨度太小必须判无效，否则"每度多少"会被放大成垃圾
        var bad = new Ui.TravelCal();
        bad.Capture(true, 0, 0);
        bad.Capture(false, 50, 1);
        Check(!bad.SpanOk, "跨度只有 50 mrad → 判定标定无效");
        Check(double.IsNaN(bad.AngleDeg(50)), "标定无效时 AngleDeg 仍返回 NaN");
        Check(double.IsNaN(bad.MradPerDeg), "标定无效时映射也是 NaN");

        // 中点不在 0、机械行程换成 90° 也要对
        var off = new Ui.TravelCal { MechSpanDeg = 90.0 };
        off.Capture(true, 1000, 0);
        off.Capture(false, 10000, 45);
        Check(Math.Abs(off.MradPerDeg - 100.0) < 1e-6, "9000 mrad ÷ 90° = 100 mrad/°", off.MradPerDeg.ToString("0.0"));
        Check(Math.Abs(off.AngleDeg(5500)) < 1e-9, "中点 5500 mrad 处 → 0°");

        // 清空
        cal.Clear();
        Check(!cal.Complete && double.IsNaN(cal.SpanSum), "清空后回到未标定状态");

        // ---- 界面：档案决定它可不可用 ----
        Defaults.SetProfile(HostProfile.Motor);
        var panel = new Ui.TravelCalPanel(new ChannelStore(), v => { }, s => { });
        var btns = new List<Button>();
        CollectType(panel, btns);
        Check(btns.Count == 4, "标定面板有 4 个按钮", "实际 " + btns.Count);
        Check(btns.TrueForAll(b => b.Enabled), "单电机档案：找得到 mag_sum → 按钮可用");

        Defaults.SetProfile(HostProfile.Kart);
        var panel2 = new Ui.TravelCalPanel(new ChannelStore(), v => { }, s => { });
        var btns2 = new List<Button>();
        CollectType(panel2, btns2);
        Check(btns2.TrueForAll(b => !b.Enabled), "卡丁车档案：没有 mag_sum → 按钮自动禁用（不会指错通道）");

        panel.Dispose();
        panel2.Dispose();
    }

    /// <summary>静态统计控件树大小</summary>''',
        "插入 TestTravelCal")
save(p, t, enc, eol)
print("SelfTest.cs ✔")
