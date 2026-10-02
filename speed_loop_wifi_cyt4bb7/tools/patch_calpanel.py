"""把「行程标定」面板接进 MainForm。"""
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


# ---------------- 面板方法改 public ----------------
p = os.path.join(ROOT, "Ui/TravelCalPanel.cs")
t, enc, eol = load(p)
t = sub(t, "        /// <summary>单电机档案：积分角 = mag_sum；换档案后这里会变成 -1，面板自动禁用。</summary>\n        private void ResolveChannels()",
        "        /// <summary>单电机档案：积分角 = mag_sum；换档案后这里会变成 -1，面板自动禁用。</summary>\n        public void ResolveChannels()",
        "ResolveChannels → public")
save(p, t, enc, eol)

# ---------------- MainForm ----------------
p = os.path.join(ROOT, "Ui/MainForm.cs")
t, enc, eol = load(p)

t = sub(t, "    private Panel _valHost;                  // 切档案时要整体重建",
        "    private Panel _valHost;                  // 切档案时要整体重建\n"
        "    private TravelCalPanel _calPanel;         // 行程标定（左右限位 → mrad/° 映射）",
        "字段 _calPanel")

t = sub(t, """        BuildValueBoxes(_valHost);
        BuildChannelChecks();
        _store.Reset();""",
        """        BuildValueBoxes(_valHost);
        BuildChannelChecks();
        _calPanel?.ResolveChannels();            // 换档案后重新找通道（找不到会自动禁用）
        _store.Reset();""",
        "档案重建时重新解析通道")

t = sub(t, """        _valHost = new Panel { Dock = DockStyle.Top, Height = Dpi.Px(200), BackColor = Theme.Panel };
        BuildValueBoxes(_valHost);""",
        """        _valHost = new Panel { Dock = DockStyle.Top, Height = Dpi.Px(200), BackColor = Theme.Panel };
        BuildValueBoxes(_valHost);

        // ★ 行程标定：把转向推到左右机械硬限位各记一次"速度积分"，
        //   得到 (角度 ↔ 速度积分) 的定量映射。夹在"实时值"和"通道显隐"之间。
        _calPanel = new TravelCalPanel(_store, OnSetMagZero, LogLine);""",
        "创建行程标定面板")

t = sub(t, """        sideHost.Controls.Add(_chkHost);
        sideHost.Controls.Add(_valHost);
        sideHost.Controls.Add(sideHead);""",
        """        sideHost.Controls.Add(_chkHost);
        sideHost.Controls.Add(_valHost);
        sideHost.Controls.Add(_calPanel);
        sideHost.Controls.Add(sideHead);""",
        "挂到右侧栏")

t = sub(t, """            foreach (var w in _waves) if (w != null) w.Tick();
            UpdateValueLabels();""",
        """            foreach (var w in _waves) if (w != null) w.Tick();
            UpdateValueLabels();
            _calPanel?.RefreshValues();""",
        "UI 定时器刷新")

t = sub(t, """    private void UpdateStats()""",
        """    /// <summary>按英文名找参数 ID —— 车端的表可能和本地默认表不同，所以按名字查而不是写死 ID。</summary>
    private int ParamIdByName(string name)
    {
        foreach (var p in Defaults.BuildDefaultParams())
            if (p.Name == name) return p.Id;
        return -1;
    }

    /// <summary>「当前位置→0°」：把当前角度寄存器的原始值写进 mag_zero（车端算 deg 时要减它）。</summary>
    private void OnSetMagZero(float rawAngle)
    {
        int id = ParamIdByName("mag_zero");
        if (id < 0) { LogLine("参数表里没有 mag_zero —— 车端固件版本可能不匹配"); return; }
        SendCmd(Cmd.SetParam, (byte)id, rawAngle);
    }

    private void UpdateStats()""",
        "加 ParamIdByName / OnSetMagZero")
save(p, t, enc, eol)
print("MainForm.cs ✔")
