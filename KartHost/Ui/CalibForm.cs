using System.Globalization;
using System.Text;

namespace KartHost.Ui;

/// <summary>
/// 标定窗口要用到的"宿主能力"。用接口而不是直接传 MainForm：
/// 这样这个窗口**只依赖三样东西**（数据仓库 / 参数表 / 写参数），不碰主界面的其余部分。
/// </summary>
public interface ICalibHost
{
    ChannelStore Store { get; }
    bool LinkOpen { get; }
    IReadOnlyList<ParamMeta> Params { get; }
    /// <summary>写一个参数（内部会做 ±27° 夹紧 → 下发 → 打日志）</summary>
    void WriteParam(ParamMeta p, float v);
    void Log(string s);
}

/// <summary>
/// ★ 实车标定窗口（2026-10-13 新增）。
///
/// 它解决的是两个具体的现场问题（见 待办-实车标定.md）：
///   ① **速度环**现在的参数是空转标定的，落地带载后起跑瞬间超调 ⇒ 要单独整定"带载"一组。
///      车上接不了串口 ⇒ 不能用 VOFA+ ⇒ 只能靠 WiFi 遥测 + 本地落盘。
///   ② **舵机角度环**是纯 P + 摩擦前馈，指令到跟随有明显时延 ⇒ 先测准真实滞后/时间常数，再决定
///      是改结构还是重调参（照 Polaris 的七步工序）。
///
/// 所以这个窗口同时提供三件事，**一个界面下做完**：
///   · 速度环的**开环占空比阶跃**（车端 ol_en / ol_duty_* 参数）
///   · 舵机角度环的**角度阶跃**（车端 steer_tgt / steer_en / steer_slew 参数）
///   · **CSV 录制**：把两条环路的响应按时间戳落盘，离线分析
///
/// 三条设计纪律：
///   1. **只发参数、不发新命令**：全部走已有的 SET_PARAM 通道 —— 协议一个字节都没改，
///      车端也不需要为它新增任何命令（这正是"沿用现有架构与协议"的做法）。
///   2. 阶跃一律**带基线段和回落段**（0 → A → 0），不是"打一下就不管" ——
///      这样一张图里同时有静差、上升段和回落段，离线一眼能读。
///   3. 每一步都在 CSV 里打 note（`note` 列），离线时按 note 对齐时间轴，不靠肉眼看拐点。
/// </summary>
public sealed class CalibForm : Form
{
    private readonly ICalibHost _host;
    private readonly CsvRecorder _csv;

    // ---- 速度环开环阶跃 ----
    private NumericUpDown _numOlAmp, _numOlHold;
    private Button _btnOlPos, _btnOlNeg, _btnOlZero, _btnOlEnable;
    private Label _lbOlState;

    // ---- 舵机角度阶跃 ----
    private NumericUpDown _numStAmp, _numStHold;
    private Button _btnStPos, _btnStNeg, _btnStMid, _btnStEnable;
    private Label _lbStState;

    // ---- CSV ----
    private NumericUpDown _numPeriod;
    private TextBox _txtPath;
    private Button _btnBrowse, _btnOpenDir, _btnRecStart, _btnRecStop;
    private Label _lbRecState;

    // ---- 实时值 ----
    private TextBox _txtLive;

    // ---- 顶栏 ----
    private Button _btnTelSel, _btnDutyFix;
    private Label _lbTop;

    private System.Windows.Forms.Timer _tick;      // 10ms：跑阶跃序列 + 喂 CSV
    private int _uiDiv;

    // 阶跃序列状态机
    private int _seq = 0;              // 0=空闲 1=速度环 2=舵机
    private int _seqPhase = 0;         // 0=基线 1=施加 2=回落 3=收尾
    private double _seqT;
    private float _seqAmp;
    private int _seqBaseMs = 300, _seqHoldMs = 600;
    // 阶跃前 steer_slew 的值，收尾时还回去。
    // ⚠ 2026-10-14 起车端默认就是 0（关）—— 照队友实车结论，纯 P+前馈环里目标斜坡只贡献纯滞后。
    //   这里"临时置 0"的逻辑因此通常是个 no-op；保留它是因为**用户可能自己把斜坡拖大了**，
    //   而"阶跃测试"要的必须是真阶跃。
    private float _restoreSlew = 0f;

    private const int TelFull = 0;
    private const int TelCalib = 1;

    public CalibForm(ICalibHost host)
    {
        _host = host;
        _csv = new CsvRecorder(host.Store);

        AutoScaleMode = AutoScaleMode.None;
        _ = Handle;
        Dpi.Init(DeviceDpi);              // ★ 与 MainForm 同一套路：先建句柄、再按本机真实 DPI 初始化

        Text = "实车标定 · 速度环开环阶跃 / 舵机角度阶跃 / CSV 记录";
        ClientSize = Dpi.Sz(760, 760);
        StartPosition = FormStartPosition.CenterParent;
        BackColor = Theme.Bg;
        ForeColor = Theme.Text;
        Font = Theme.F(9f);
        MinimumSize = Dpi.Sz(700, 620);

        BuildUi();

        _tick = new System.Windows.Forms.Timer { Interval = 10 };
        _tick.Tick += (_, __) => OnTick();
        _tick.Start();

        UpdateTopLabel();
        UpdateLive();

        Theme.Apply(this);

        _host.Log("标定窗口已打开：只走参数下发，不发新命令。");
    }

    protected override void OnFormClosed(FormClosedEventArgs e)
    {
        if (_csv.IsRecording)
        {
            long n = _csv.Stop();
            _host.Log($"标定窗口关闭，CSV 自动停止（{n} 行）→ {_csv.Path}");
        }
        _tick?.Stop();
        base.OnFormClosed(e);
    }

    // ==================================================================
    // UI
    // ==================================================================
    private void BuildUi()
    {
        var root = new Panel { Dock = DockStyle.Fill, AutoScroll = true, BackColor = Theme.Bg, Padding = Dpi.Pad(10, 8, 10, 10) };

        var flow = new FlowLayoutPanel
        {
            Dock = DockStyle.Top,
            FlowDirection = FlowDirection.TopDown,
            WrapContents = false,
            AutoSize = true,
            AutoSizeMode = AutoSizeMode.GrowAndShrink,
            BackColor = Theme.Bg,
        };

        int W = 716;                        // 逻辑宽（留出滚动条）

        // ---------------- 顶栏 ----------------
        var top = new Panel { Width = Dpi.Px(W), Height = Dpi.Px(64), BackColor = Theme.Panel };

        _lbTop = new Label
        {
            AutoSize = true,
            Location = new Point(Dpi.Px(10), Dpi.Px(8)),
            ForeColor = Theme.TextDim,
        };

        _btnTelSel = new Button { Text = "车端遥测：全量", Location = new Point(Dpi.Px(10), Dpi.Px(30)), Width = Dpi.Px(150), Height = Dpi.Px(28) };
        _btnTelSel.Click += (_, __) => OnToggleTel();

        _btnDutyFix = new Button { Text = "限幅升到 60%", Location = new Point(Dpi.Px(168), Dpi.Px(30)), Width = Dpi.Px(120), Height = Dpi.Px(28) };
        _btnDutyFix.Click += (_, __) =>
        {
            var p = FindParam("duty_limit");
            if (p == null) { _host.Log("找不到参数 duty_limit"); return; }
            _host.WriteParam(p, 60f);
            _host.Log("已把「限幅|占空比上限」写到 60%（开环阶跃要够大才测得动）");
        };

        var lbTopHint = new Label
        {
            AutoSize = true,
            Location = new Point(Dpi.Px(300), Dpi.Px(36)),
            ForeColor = Theme.TextDim,
            Text = "开关量/极性/限位都在主界面的参数面板里",
        };

        TopGroup(top, _lbTop, _btnTelSel, _btnDutyFix, lbTopHint);
        flow.Controls.Add(top);

        // ---------------- 速度环开环阶跃 ----------------
        var g1 = MakeGroup("速度环 · 开环占空比阶跃", W, 132);
        var c1 = g1.Controls;

        c1.Add(LabelAt("阶跃幅度", 12, 26));
        _numOlAmp = NumAt(90, 22, 0, 60, 15, 1, "%");
        c1.Add(_numOlAmp);

        c1.Add(LabelAt("持续时间", 210, 26));
        _numOlHold = NumAt(288, 22, 100, 5000, 600, 50, "ms");
        c1.Add(_numOlHold);

        c1.Add(LabelAt("基线", 400, 26));
        var lbBase = new Label { AutoSize = true, Location = new Point(Dpi.Px(436), Dpi.Px(26)), ForeColor = Theme.TextDim, Text = "300 ms（固定）" };
        c1.Add(lbBase);

        _btnOlEnable = new Button { Text = "开环关", Location = new Point(Dpi.Px(556), Dpi.Px(20)), Width = Dpi.Px(140), Height = Dpi.Px(28) };
        _btnOlEnable.Click += (_, __) => OnToggleOpenLoop();
        c1.Add(_btnOlEnable);

        _btnOlPos = new Button { Text = "0 → +A（前进阶跃）", Location = new Point(Dpi.Px(12), Dpi.Px(60)), Width = Dpi.Px(180), Height = Dpi.Px(32) };
        _btnOlNeg = new Button { Text = "0 → −A（后退阶跃）", Location = new Point(Dpi.Px(200), Dpi.Px(60)), Width = Dpi.Px(180), Height = Dpi.Px(32) };
        _btnOlZero = new Button { Text = "归零", Location = new Point(Dpi.Px(388), Dpi.Px(60)), Width = Dpi.Px(90), Height = Dpi.Px(32) };
        _btnOlPos.Click += (_, __) => StartSeq(1, _numOlAmp.Value, (int)_numOlHold.Value);
        _btnOlNeg.Click += (_, __) => StartSeq(1, -_numOlAmp.Value, (int)_numOlHold.Value);
        _btnOlZero.Click += (_, __) => { StopSeq(); WritePair("ol_duty_l", "ol_duty_r", 0f); _host.Log("开环 duty 已归零"); };
        c1.Add(_btnOlPos); c1.Add(_btnOlNeg); c1.Add(_btnOlZero);

        _lbOlState = new Label { AutoSize = true, Location = new Point(Dpi.Px(492), Dpi.Px(68)), ForeColor = Theme.TextDim, Text = "空闲" };
        c1.Add(_lbOlState);

        flow.Controls.Add(g1);

        // ---------------- 舵机角度阶跃 ----------------
        var g2 = MakeGroup("舵机角度环 · 角度阶跃（限位 ±27°）", W, 132);
        var c2 = g2.Controls;

        c2.Add(LabelAt("阶跃角", 12, 26));
        _numStAmp = NumAt(78, 22, 1, (decimal)Defaults.SteerLimitDeg, 10, 1, "°");
        c2.Add(_numStAmp);

        c2.Add(LabelAt("持续时间", 190, 26));
        _numStHold = NumAt(268, 22, 100, 5000, 600, 50, "ms");
        c2.Add(_numStHold);

        // ⚠ 这里刻意只留一句**短**提示：早先写成"±27°（硬限位，禁止 30/32）"，
        //   长度会顶到右边的「内环关」按钮上（布局体检查出 2 处重叠）。限位含义写在标题与文档里。
        var lbLim = new Label
        {
            AutoSize = true,
            Location = new Point(Dpi.Px(380), Dpi.Px(26)),
            ForeColor = Theme.Warn,
            Text = $"硬限位 ±{Defaults.SteerLimitDeg:0.#}°",
        };
        c2.Add(lbLim);

        _btnStEnable = new Button { Text = "内环关", Location = new Point(Dpi.Px(556), Dpi.Px(20)), Width = Dpi.Px(140), Height = Dpi.Px(28) };
        _btnStEnable.Click += (_, __) => OnToggleSteer();
        c2.Add(_btnStEnable);

        _btnStPos = new Button { Text = "0 → +θ（左阶跃）", Location = new Point(Dpi.Px(12), Dpi.Px(60)), Width = Dpi.Px(180), Height = Dpi.Px(32) };
        _btnStNeg = new Button { Text = "0 → −θ（右阶跃）", Location = new Point(Dpi.Px(200), Dpi.Px(60)), Width = Dpi.Px(180), Height = Dpi.Px(32) };
        _btnStMid = new Button { Text = "回中", Location = new Point(Dpi.Px(388), Dpi.Px(60)), Width = Dpi.Px(90), Height = Dpi.Px(32) };
        _btnStPos.Click += (_, __) => StartSeq(2, _numStAmp.Value, (int)_numStHold.Value);
        _btnStNeg.Click += (_, __) => StartSeq(2, -_numStAmp.Value, (int)_numStHold.Value);
        _btnStMid.Click += (_, __) => { StopSeq(); WriteParam("steer_tgt", 0f); _host.Log("转向目标已回中"); };
        c2.Add(_btnStPos); c2.Add(_btnStNeg); c2.Add(_btnStMid);

        _lbStState = new Label { AutoSize = true, Location = new Point(Dpi.Px(492), Dpi.Px(68)), ForeColor = Theme.TextDim, Text = "空闲" };
        c2.Add(_lbStState);

        flow.Controls.Add(g2);

        // ---------------- CSV ----------------
        var g3 = MakeGroup("CSV 记录 / 导出（替代串口 VOFA+）", W, 132);
        var c3 = g3.Controls;

        c3.Add(LabelAt("采样周期", 12, 26));
        _numPeriod = NumAt(78, 22, 5, 1000, 20, 5, "ms");
        c3.Add(_numPeriod);

        c3.Add(LabelAt("保存路径", 12, 62));
        _txtPath = new TextBox
        {
            Location = new Point(Dpi.Px(78), Dpi.Px(58)),
            Width = Dpi.Px(430),
            Text = DefaultCsvPath(),
        };
        c3.Add(_txtPath);

        _btnBrowse = new Button { Text = "浏览…", Location = new Point(Dpi.Px(516), Dpi.Px(56)), Width = Dpi.Px(84), Height = Dpi.Px(28) };
        _btnBrowse.Click += (_, __) => OnBrowse();
        c3.Add(_btnBrowse);

        _btnOpenDir = new Button { Text = "打开目录", Location = new Point(Dpi.Px(606), Dpi.Px(56)), Width = Dpi.Px(90), Height = Dpi.Px(28) };
        _btnOpenDir.Click += (_, __) => OnOpenDir();
        c3.Add(_btnOpenDir);

        _btnRecStart = new Button { Text = "开始记录", Location = new Point(Dpi.Px(12), Dpi.Px(94)), Width = Dpi.Px(140), Height = Dpi.Px(30) };
        _btnRecStart.Click += (_, __) => OnRecStart();
        c3.Add(_btnRecStart);

        _btnRecStop = new Button { Text = "停止并保存", Location = new Point(Dpi.Px(160), Dpi.Px(94)), Width = Dpi.Px(140), Height = Dpi.Px(30) };
        _btnRecStop.Click += (_, __) => OnRecStop();
        c3.Add(_btnRecStop);

        _lbRecState = new Label { AutoSize = true, Location = new Point(Dpi.Px(314), Dpi.Px(100)), ForeColor = Theme.TextDim, Text = "未记录" };
        c3.Add(_lbRecState);

        flow.Controls.Add(g3);

        // ---------------- 实时值 ----------------
        var g4 = MakeGroup("实时值（跟随当前档案的通道表）", W, 168);
        _txtLive = new TextBox
        {
            Multiline = true,
            ReadOnly = true,
            ScrollBars = ScrollBars.Vertical,
            WordWrap = false,
            Font = Theme.FM(9f),
            Location = new Point(Dpi.Px(12), Dpi.Px(22)),
            Size = Dpi.Sz(W - 34, 132),
            BackColor = Theme.Canvas,
            ForeColor = Theme.Text,
            BorderStyle = BorderStyle.FixedSingle,
        };
        g4.Controls.Add(_txtLive);
        flow.Controls.Add(g4);

        root.Controls.Add(flow);
        Controls.Add(root);
    }

    private static void TopGroup(Control parent, params Control[] kids)
    {
        // 顶栏是普通 Panel，不是 GroupBox：只是把"建了控件必须挂进父容器"这件事收口到一处
        parent.Controls.AddRange(kids);
    }

    private Label LabelAt(string text, int x, int y)
        => new() { AutoSize = true, Location = new Point(Dpi.Px(x), Dpi.Px(y)), Text = text, ForeColor = Theme.Text };

    private NumericUpDown NumAt(int x, int y, decimal lo, decimal hi, decimal val, decimal step, string suffix)
    {
        return new NumericUpDown
        {
            Location = new Point(Dpi.Px(x), Dpi.Px(y)),
            Width = Dpi.Px(96),
            Minimum = lo,
            Maximum = hi,
            Value = val,
            Increment = step,
            DecimalPlaces = 0,
            Tag = suffix,
            BackColor = Theme.PanelAlt,
            ForeColor = Theme.Text,
        };
    }

    private GroupBox MakeGroup(string title, int w, int h)
        => new() { Text = title, Width = Dpi.Px(w), Height = Dpi.Px(h), ForeColor = Theme.TextDim, BackColor = Theme.Panel };

    private static string DefaultCsvPath()
    {
        string dir = System.IO.Path.Combine(AppContext.BaseDirectory, "logs");
        return System.IO.Path.Combine(dir, "calib_" + DateTime.Now.ToString("yyyyMMdd_HHmmss") + ".csv");
    }

    // ==================================================================
    // 参数查找 / 写入
    // ==================================================================
    private ParamMeta FindParam(string name)
        => _host.Params?.FirstOrDefault(p => p.Name == name);

    private float GetParam(string name, float fallback = 0f)
    {
        var p = FindParam(name);
        return p == null ? fallback : p.Value;
    }

    private void WriteParam(string name, float v)
    {
        var p = FindParam(name);
        if (p == null) { _host.Log("找不到参数：" + name); return; }
        _host.WriteParam(p, v);
    }

    private void WritePair(string n1, string n2, float v)
    {
        WriteParam(n1, v);
        WriteParam(n2, v);
    }

    private bool ParamOn(string name) => GetParam(name) > 0.5f;

    // ==================================================================
    // 顶栏：车端遥测档案
    // ==================================================================
    private void OnToggleTel()
    {
        // 车端的 tel_sel：0 = 全量 39 通道，1 = 标定精简 18 通道。
        // ⚠ 通道数一变，帧头 COUNT 也变 —— **本机档案下拉框必须同步切**，否则通道名错位。
        bool toCalib = !ParamOn("tel_sel");
        WriteParam("tel_sel", toCalib ? 1f : 0f);
        UpdateTopLabel();

        _host.Log(toCalib
            ? "已让车端切换到【标定·精简】档案（18 通道）—— 请把本机「档案」下拉框也切到「标定·精简」。"
            : "已让车端切回【全量】档案（39 通道）—— 请把本机「档案」下拉框也切到「卡丁·遥控」。");
    }

    private void UpdateTopLabel()
    {
        bool open = _host.LinkOpen;
        bool calib = ParamOn("tel_sel");

        _btnTelSel.Text = calib ? "车端遥测：精简" : "车端遥测：全量";
        _lbTop.Text = $"链路 {(open ? "已连接" : "未连接")} ｜ 本机档案：{Defaults.ProfileDisplayName}"
                    + $"（{Defaults.ChannelNames.Length} 通道）｜ 车端遥测：{(calib ? "精简 18" : "全量 39")}";
        _lbTop.ForeColor = open ? Theme.Ok : Theme.Err;
    }

    // ==================================================================
    // 开环 / 内环 开关
    // ==================================================================
    private void OnToggleOpenLoop()
    {
        bool on = !ParamOn("ol_en");
        if (on)
        {
            // 先确保两路 duty = 0，再打开开环开关 —— 顺序反了会有一拍"上一轮的残留占空比"冲出去
            WritePair("ol_duty_l", "ol_duty_r", 0f);
            WriteParam("ol_en", 1f);
            WriteParam("steer_en", 0f);              // 角度环让位：只测速度环
            _host.Log("★ 已进入开环模式：两路直接吃 ol_duty_*，速度环整套被绕过。");
        }
        else
        {
            WritePair("ol_duty_l", "ol_duty_r", 0f);
            WriteParam("ol_en", 0f);
            _host.Log("已退出开环模式，交回速度环。");
        }
        UpdateSwitchLabels();
    }

    private void OnToggleSteer()
    {
        bool on = !ParamOn("steer_en");
        if (on)
        {
            WritePair("ol_duty_l", "ol_duty_r", 0f);
            WriteParam("ol_en", 0f);                 // 速度环让位
            // 记下 slew 原值：阶跃要的是"真阶跃"，所以临时关斜坡，收尾还回去
            _restoreSlew = GetParam("steer_slew", 0f);
            WriteParam("steer_slew", 0f);
            WriteParam("steer_tgt", 0f);
            WriteParam("steer_en", 1f);
            _host.Log("★ 舵机内环已使能（阶跃期间目标斜坡已置 0 = 关，收尾会还回 " + _restoreSlew.ToString("0.#") + "）。");
        }
        else
        {
            WriteParam("steer_en", 0f);
            WriteParam("steer_tgt", 0f);
            WriteParam("steer_slew", _restoreSlew);
            _host.Log("舵机内环已关闭，目标斜坡已还回 " + _restoreSlew.ToString("0.#"));
        }
        UpdateSwitchLabels();
    }

    private void UpdateSwitchLabels()
    {
        _btnOlEnable.Text = ParamOn("ol_en") ? "开环开" : "开环关";
        _btnOlEnable.ForeColor = ParamOn("ol_en") ? Theme.Warn : Theme.Text;
        _btnStEnable.Text = ParamOn("steer_en") ? "内环开" : "内环关";
        _btnStEnable.ForeColor = ParamOn("steer_en") ? Theme.Ok : Theme.Text;
    }

    // ==================================================================
    // CSV
    // ==================================================================
    private void OnBrowse()
    {
        using var dlg = new SaveFileDialog
        {
            Filter = "CSV 文件 (*.csv)|*.csv|所有文件 (*.*)|*.*",
            FileName = System.IO.Path.GetFileName(_txtPath.Text),
            InitialDirectory = System.IO.Path.GetDirectoryName(_txtPath.Text),
        };
        if (dlg.ShowDialog(this) == DialogResult.OK) _txtPath.Text = dlg.FileName;
    }

    private void OnOpenDir()
    {
        try
        {
            string dir = System.IO.Path.GetDirectoryName(_txtPath.Text);
            if (!string.IsNullOrEmpty(dir) && Directory.Exists(dir)) System.Diagnostics.Process.Start("explorer.exe", dir);
            else _host.Log("目录还不存在：" + dir);
        }
        catch (Exception ex) { _host.Log("打开目录失败：" + ex.Message); }
    }

    private void OnRecStart()
    {
        if (_csv.IsRecording) { _host.Log("已经在录制中"); return; }
        if (!_host.LinkOpen) { _host.Log("未连接，先点主界面的「连接」再录"); return; }

        var chans = Enumerable.Range(0, Defaults.ChannelNames.Length).ToList();
        bool ok = _csv.Start(_txtPath.Text, (int)_numPeriod.Value, chans, NowMs(), out string err);
        if (!ok) { _host.Log("开始录制失败：" + err); return; }

        _host.Log($"★ 开始记录 CSV（{chans.Count} 通道 / 每 {_numPeriod.Value:0} ms）→ {_txtPath.Text}");
        UpdateRecState();
    }

    private void OnRecStop()
    {
        if (!_csv.IsRecording) { _host.Log("当前没有在录制"); return; }
        long n = _csv.Stop();
        _host.Log($"CSV 已停止并保存：{n} 行 → {_csv.Path}");
        _txtPath.Text = DefaultCsvPath();     // 下次默认换一个新文件名，避免手滑覆盖
        UpdateRecState();
    }

    private void UpdateRecState()
    {
        _lbRecState.Text = _csv.IsRecording
            ? $"● 记录中  {_csv.Rows} 行"
            : "未记录";
        _lbRecState.ForeColor = _csv.IsRecording ? Theme.Err : Theme.TextDim;
    }

    // ==================================================================
    // 阶跃序列（0 → A → 0）
    // ==================================================================
    private double NowMs() => Environment.TickCount64;   // 与界面同一时钟，够用（毫秒级）

    private void StartSeq(int kind, decimal amp, int holdMs)
    {
        if (!_host.LinkOpen) { _host.Log("未连接，阶跃没发出去"); return; }
        if (_seq != 0) { _host.Log("上一组阶跃还没跑完"); return; }

        _seq = kind;
        _seqPhase = 0;
        _seqT = 0;
        _seqAmp = (float)amp;
        _seqHoldMs = Math.Max(100, holdMs);

        if (kind == 1)
        {
            // 速度环开环：自动把限幅提到够大，否则阶跃会被 duty_limit 削平（测出来是假的）
            var dl = FindParam("duty_limit");
            if (dl != null && dl.Value < Math.Abs(_seqAmp) + 1f)
            {
                float want = Math.Min(60f, Math.Abs(_seqAmp) + 5f);
                _host.WriteParam(dl, want);
                _host.Log($"开环阶跃：占空比限幅自动提到 {want:0}%（原 {dl.Value:0}%，否则阶跃会被削平）");
            }

            WriteParam("steer_en", 0f);
            WriteParam("ol_duty_l", 0f);
            WriteParam("ol_duty_r", 0f);
            WriteParam("ol_en", 1f);
            _csv.Note(NowMs(), $"step_OL_begin A={_seqAmp:0.#}% base={_seqBaseMs}ms hold={_seqHoldMs}ms");
            _host.Log($"★ 开环阶跃开始：0 → {_seqAmp:0.#}%（基线 {_seqBaseMs} ms，保持 {_seqHoldMs} ms）");
        }
        else
        {
            WriteParam("ol_en", 0f);
            WritePair("ol_duty_l", "ol_duty_r", 0f);
            _restoreSlew = GetParam("steer_slew", 0f);
            WriteParam("steer_slew", 0f);          // 关斜坡 = 真阶跃
            WriteParam("steer_tgt", 0f);
            WriteParam("steer_en", 1f);
            _csv.Note(NowMs(), $"step_STEER_begin A={_seqAmp:0.#}deg base={_seqBaseMs}ms hold={_seqHoldMs}ms");
            _host.Log($"★ 角度阶跃开始：0 → {_seqAmp:0.#}°（基线 {_seqBaseMs} ms，保持 {_seqHoldMs} ms；目标斜坡已临时置 0）");
        }

        UpdateSwitchLabels();
        UpdateSeqLabel();
    }

    private void StopSeq()
    {
        if (_seq != 0)
        {
            _host.Log("阶跃已中止");
        }
        _seq = 0;
        _seqPhase = 0;
        UpdateSeqLabel();
    }

    private void OnTick()
    {
        // ---- 阶跃序列（10ms 一拍）----
        if (_seq != 0)
        {
            _seqT += 10;

            if (_seqPhase == 0 && _seqT >= _seqBaseMs)
            {
                _csv.Note(NowMs(), $"step_apply A={_seqAmp:0.###}");
                if (_seq == 1) WritePair("ol_duty_l", "ol_duty_r", _seqAmp);
                else WriteParam("steer_tgt", Defaults.ClampSteerDeg(_seqAmp));
                _seqPhase = 1;
                _seqT = 0;
            }
            else if (_seqPhase == 1 && _seqT >= _seqHoldMs)
            {
                _csv.Note(NowMs(), "step_release");
                if (_seq == 1) WritePair("ol_duty_l", "ol_duty_r", 0f);
                else WriteParam("steer_tgt", 0f);
                _seqPhase = 2;
                _seqT = 0;
            }
            else if (_seqPhase == 2 && _seqT >= _seqBaseMs * 2)
            {
                _csv.Note(NowMs(), "step_end");
                _host.Log("阶跃序列结束（回落段已跑完）");
                _seq = 0;
                _seqPhase = 0;
            }

            UpdateSeqLabel();
        }

        // ---- CSV 抽稀落盘 ----
        _csv.Tick(NowMs());

        // ---- 界面刷新（约 10 Hz）----
        if (++_uiDiv >= 10)
        {
            _uiDiv = 0;
            UpdateLive();
            UpdateRecState();
            UpdateSwitchLabels();
            UpdateTopLabel();
        }
    }

    private void UpdateSeqLabel()
    {
        string s = _seq switch
        {
            0 => "空闲",
            1 => new[] { "基线", "施加", "回落" }[Math.Min(_seqPhase, 2)] + "…（速度环）",
            _ => new[] { "基线", "施加", "回落" }[Math.Min(_seqPhase, 2)] + "…（舵机）",
        };
        _lbOlState.Text = _seq == 1 ? s : "空闲";
        _lbStState.Text = _seq == 2 ? s : "空闲";
        _lbOlState.ForeColor = _seq == 1 ? Theme.Warn : Theme.TextDim;
        _lbStState.ForeColor = _seq == 2 ? Theme.Warn : Theme.TextDim;
    }

    // ==================================================================
    // 实时值
    // ==================================================================
    private void UpdateLive()
    {
        var sb = new StringBuilder();
        int n = Defaults.ChannelNames.Length;

        for (int i = 0; i < n; i++)
        {
            float v = _host.Store.Latest(i);
            string unit = Defaults.ChannelUnit(i);
            string val = float.IsNaN(v) ? "  --  " : v.ToString("0.###", CultureInfo.InvariantCulture);

            sb.Append(Defaults.ChannelName(i).PadRight(14));
            sb.Append(' ');
            sb.Append(val.PadLeft(11));
            sb.Append(' ');
            sb.Append((unit ?? "").PadRight(5));      // 单位列定宽，右边才不会和下一列黏在一起
            sb.Append('\n');
        }

        // 只在内容真的变了才写回，避免每 100ms 触发一次重绘/滚动跳动
        string txt = sb.ToString();
        if (_txtLive.Text != txt) _txtLive.Text = txt;
    }
}
