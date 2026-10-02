using System.Diagnostics;

namespace KartHost.Ui;

/// <summary>
/// 主窗体：连接栏 / 波形区 / 参数面板 / 数值面板 / 使能区 / 状态栏。
/// 架构要点：网络线程只往 ChannelStore 写；本窗体的 30 FPS 定时器负责取快照重绘。
/// 两侧完全解耦 —— 这是能带得动 40 通道 50 Hz 的前提。
/// </summary>
public sealed class MainForm : Form
{
    // ---- 数据与链路 ----
    private readonly ChannelStore _store = new();
    private readonly Link _link = new();
    private Simulator _sim = new();          // 切档案时会重建（模拟器要跟着发另一套通道）

    private readonly List<ParamMeta> _tableBuf = new();
    private System.Windows.Forms.Timer _tableTimer;
    private System.Windows.Forms.Timer _uiTimer;
    private System.Windows.Forms.Timer _hbTimer;    // 心跳：告诉车端"上位机还活着"
    private System.Windows.Forms.Timer _statTimer;

    // ---- 顶部连接栏 ----
    private ComboBox _cbProto;
    private Label _lbIp;                     // UDP 模式下语义变成"本地端口"
    private ComboBox _cbProfile;             // 档案选择：全车 / 单电机验证 / 双电机速度环
    private TextBox _txtIp, _txtPort;
    private Button _btnConnect, _btnSim;
    private LampControl _lampLink;

    // ---- 波形 ----
    private Panel _waveHost;
    private WavePanel[] _waves;
    private int _layout = 4;                  // 1 / 2 / 4
    private ComboBox _cbWindow;
    private Button _btnPause;
    private CheckBox _ckAutoY;

    // ---- 参数 ----
    private ParamPanel _paramPanel;
    private Label _lblDirty;

    // ---- 通道勾选 ----
    private FlowLayoutPanel _chkHost;
    private readonly Dictionary<int, CheckBox> _chkBoxes = new();

    // ---- 数值面板 ----
    private readonly Dictionary<int, Label> _valLabels = new();
    private Panel _valHost;                  // 切档案时要整体重建
    private TravelCalPanel _calPanel;         // 行程标定（左右限位 → mrad/° 映射）
    private bool _inProfileSwitch;           // 防止 SelectedIndex 回写时递归触发

    // ---- 使能 ----
    private LampControl _lampSteer, _lampMotor, _lampMaster;
    private Button _btnSteer, _btnMotor, _btnMaster, _btnEStop;
    private EnableBits _enable = EnableBits.None;

    // ---- 状态栏 ----
    private StatusStrip _status;
    private ToolStripStatusLabel _stLink, _stRate, _stLoss, _stBw, _stCrc, _stDirty;

    // ---- 统计 ----
    private long _lastFrames, _lastBytes;
    private int _lastSeq = -1;
    private long _lost, _total;
    private double _lastStatT;
    private readonly Stopwatch _sw = Stopwatch.StartNew();
    private ushort _token;
    private readonly Dictionary<ushort, double> _pendingCmd = new();

    /// <summary>给自检用的：窗体字体一旦被系统偷改，这里立刻能看出来</summary>
    public float TestFontSizePt => Font.SizeInPoints;

    public MainForm()
    {
        // ★★ DPI 三件套，**顺序是硬要求**（理由见 Ui/Dpi.cs 顶部注释）：
        //   1) 先关自动缩放：默认的 Font 模式会跟 Dpi 层的换算叠加 → 字号被缩两遍
        //   2) 再建句柄：DeviceDpi 这时才是**这台显示器**的真实值（不是被虚拟化的 96）
        //   3) 最后设字体
        AutoScaleMode = AutoScaleMode.None;
        _ = Handle;
        Dpi.Init(DeviceDpi);

        Text = "卡丁快跑 · 上位机  (KartHost)";
        // 窗口尺寸也不许超出工作区（150% 缩放下 1440 逻辑像素 = 2160 物理像素，会顶出版面）
        var wa = Screen.FromControl(this).WorkingArea;
        Width = Math.Max(Dpi.Px(900), Math.Min(Dpi.Px(1440), wa.Width - Dpi.Px(24)));
        Height = Math.Max(Dpi.Px(600), Math.Min(Dpi.Px(900), wa.Height - Dpi.Px(24)));
        MinimumSize = Dpi.Sz(900, 560);
        StartPosition = FormStartPosition.CenterScreen;
        BackColor = Theme.Bg;
        ForeColor = Theme.Text;
        Font = new Font("微软雅黑", 9f);

        // ★ Dock 顺序有讲究：WinForms 按 Z 序「最后添加的最先布局」，
        //   所以 Fill 的必须最先 Add，边缘 Dock 的后加，否则 Fill 会吃掉全部空间。
        BuildBody();          // Fill
        BuildTopBar();        // Top
        BuildLog();           // Bottom
        BuildEnableBar();     // Bottom
        BuildStatusBar();     // Bottom（最后添加 → 最先布局 → 贴最底）

        HookLink();
        BuildTimers();

        _paramPanel.SetParams(Defaults.BuildDefaultParams());
        ApplyLayout(4);
        RadioDefaultChannels();

        Theme.Apply(this);
        LoadConfig();

        _uiTimer.Start();
        _statTimer.Start();

        LogLine("就绪。可先点「模拟车端」再点「连接」，无需硬件即可看到完整效果。");
    }

    // ==================================================================
    // UI 构建
    // ==================================================================
    private void BuildTopBar()
    {
        var bar = new Panel { Dock = DockStyle.Top, Height = Dpi.Px(46), BackColor = Theme.Panel, Padding = Dpi.Pad(8, 6, 8, 6) };

        _lampLink = new LampControl { Caption = "链路", Location = new Point(Dpi.Px(8), Dpi.Px(8)), Size = Dpi.Sz(92, 30) };

        _cbProto = new ComboBox { DropDownStyle = ComboBoxStyle.DropDownList, Width = Dpi.Px(78), Location = new Point(Dpi.Px(104), Dpi.Px(11)) };
        _cbProto.Items.AddRange(new object[] { "TCP", "UDP" });
        _cbProto.SelectedIndex = 0;
        _cbProto.SelectedIndexChanged += (_, __) => SyncProtoUi();

        _lbIp = new Label { Text = "IP", AutoSize = true, Location = new Point(Dpi.Px(194), Dpi.Px(15)), ForeColor = Theme.TextDim };
        _txtIp = new TextBox { Text = "192.168.137.1", Width = Dpi.Px(116), Location = new Point(Dpi.Px(216), Dpi.Px(11)) };

        var lbPort = new Label { Text = "端口", AutoSize = true, Location = new Point(Dpi.Px(342), Dpi.Px(15)), ForeColor = Theme.TextDim };
        _txtPort = new TextBox { Text = "8086", Width = Dpi.Px(62), Location = new Point(Dpi.Px(376), Dpi.Px(11)) };

        _btnConnect = new Button { Text = "连接", Width = Dpi.Px(72), Height = Dpi.Px(26), Location = new Point(Dpi.Px(450), Dpi.Px(10)) };
        _btnConnect.Click += (_, __) => ToggleConnect();

        _btnSim = new Button { Text = "模拟车端", Width = Dpi.Px(86), Height = Dpi.Px(26), Location = new Point(Dpi.Px(530), Dpi.Px(10)) };
        _btnSim.Click += (_, __) => ToggleSim();

        // ---- 档案选择：只改"通道怎么显示"，不改协议 ----
        var lbPf = new Label { Text = "档案", AutoSize = true, Location = new Point(Dpi.Px(628), Dpi.Px(15)), ForeColor = Theme.TextDim };
        _cbProfile = new ComboBox
        {
            DropDownStyle = ComboBoxStyle.DropDownList,
            Width = Dpi.Px(118),
            Location = new Point(Dpi.Px(664), Dpi.Px(11)),
        };
        _cbProfile.Items.AddRange(new object[] { "卡丁车（32 通道）", "单电机验证（12 通道）", "双电机速度环（20 通道）" });
        _cbProfile.SelectedIndex = 0;
        _cbProfile.SelectedIndexChanged += (_, __) => OnProfileChanged();

        var lbHint = new Label
        {
            Text = "提示：模拟车端在 127.0.0.1:8086，把 IP 改成 127.0.0.1 再连接即可",
            AutoSize = true,
            Location = new Point(Dpi.Px(796), Dpi.Px(15)),
            ForeColor = Theme.TextDim,
        };

        bar.Controls.AddRange(new Control[] { _lampLink, _cbProto, _lbIp, _txtIp, lbPort, _txtPort, _btnConnect, _btnSim, lbPf, _cbProfile, lbHint });
        Controls.Add(bar);
        SyncProtoUi();
    }

    /// <summary>
    /// TCP 与 UDP 的输入语义不一样：
    ///   TCP —— 我们去连车端（车端当服务端），IP 必填；
    ///   UDP —— 车端往我们这里发（我们当接收端），只需本地端口，车端 IP 收到包后自动学。
    /// </summary>
    private void SyncProtoUi()
    {
        bool udp = _cbProto.SelectedIndex == 1;
        _lbIp.Text = udp ? "本地" : "IP";
        _txtIp.Enabled = !udp;
        _txtIp.BackColor = udp ? Theme.PanelAlt : Theme.Panel;
        _txtPort.Text = udp
            ? (int.TryParse(_txtPort.Text, out int p) && p > 0 ? _txtPort.Text : "8086")
            : _txtPort.Text;
    }

    /// <summary>
    /// 切档案＝换一套通道表。协议不变，所以**车端不用改任何东西**：
    /// 全车档案显示 32 通道，单电机档案显示 motor_link.h 里那 12 通道。
    /// ⚠ 必须在断开状态下切（连着的时候切会让 ChannelStore 里的老数据错位）。
    /// </summary>
    private void OnProfileChanged()
    {
        HostProfile want = _cbProfile.SelectedIndex switch
        {
            1 => HostProfile.Motor,
            2 => HostProfile.DualMotor,
            _ => HostProfile.Kart,
        };
        if (want == Defaults.Current || _inProfileSwitch) return;

        if (_sim.IsRunning || _link.IsOpen)
        {
            _inProfileSwitch = true;
            _cbProfile.SelectedIndex = Defaults.ProfileIndex;      // 改回去（会再次触发本函数，靠 _inProfileSwitch 挡住）
            _inProfileSwitch = false;
            LogLine("请先断开（并停止模拟车端）再切换档案 —— 切换会重建通道表。");
            return;
        }

        Defaults.SetProfile(want);

        // 模拟器是按 ChannelNames.Length 建缓冲的，必须重建
        _sim.Dispose();
        _sim = new Simulator();
        HookSimLog();

        // 界面里的通道勾选框 / 数值框是按下标硬建的，一并重建
        foreach (var ck in _chkBoxes.Values) ck.Dispose();
        _chkBoxes.Clear();
        _chkHost.Controls.Clear();

        foreach (var lb in _valLabels.Values) lb.Parent?.Dispose();
        _valLabels.Clear();
        _valHost.Controls.Clear();

        BuildValueBoxes(_valHost);
        BuildChannelChecks();
        _calPanel?.ResolveChannels();            // 换档案后重新找通道（找不到会自动禁用）
        _store.Reset();
        ApplyLayout(_layout);
        RadioDefaultChannels();
        _paramPanel.SetParams(Defaults.BuildDefaultParams());

        LogLine($"档案已切换：{Defaults.ProfileDisplayName}，共 {Defaults.ChannelNames.Length} 个遥测通道。");
    }

    private void BuildLog()
    {
        var host = new Panel { Dock = DockStyle.Bottom, Height = Dpi.Px(108), BackColor = Theme.Panel };
        var head = new Panel { Dock = DockStyle.Top, Height = Dpi.Px(22), BackColor = Theme.PanelAlt };
        head.Controls.Add(new Label
        {
            Text = "  日志（连接 / 参数写入 / CRC / 错误）",
            Dock = DockStyle.Fill,
            TextAlign = ContentAlignment.MiddleLeft,
            ForeColor = Theme.TextDim,
        });

        _log = new RichTextBox
        {
            Dock = DockStyle.Fill,
            BackColor = Theme.Bg,
            ForeColor = Theme.TextDim,
            BorderStyle = BorderStyle.None,
            ReadOnly = true,
            WordWrap = false,
            Font = new Font("Consolas", 8.5f),
            ScrollBars = RichTextBoxScrollBars.Vertical,
        };

        host.Controls.Add(_log);    // Fill 先加
        host.Controls.Add(head);    // Top 后加
        Controls.Add(host);
    }

    /// <summary>跨线程安全地回到 UI 线程</summary>
    private void UI(Action a)
    {
        if (!IsHandleCreated) return;
        try { BeginInvoke(a); } catch (ObjectDisposedException) { }
    }

    private void BuildStatusBar()
    {
        _status = new StatusStrip { SizingGrip = false, BackColor = Theme.Panel };
        _stLink = new ToolStripStatusLabel("未连接") { ForeColor = Theme.TextDim };
        _stRate = new ToolStripStatusLabel("帧率 --") { ForeColor = Theme.TextDim };
        _stLoss = new ToolStripStatusLabel("丢包 --") { ForeColor = Theme.TextDim };
        _stBw = new ToolStripStatusLabel("带宽 --") { ForeColor = Theme.TextDim };
        _stCrc = new ToolStripStatusLabel("CRC 错 0") { ForeColor = Theme.TextDim };
        _stDirty = new ToolStripStatusLabel("参数改动 0") { ForeColor = Theme.Warn };
        _status.Items.AddRange(new ToolStripItem[]
        {
            _stLink, new ToolStripSeparator(), _stRate, new ToolStripSeparator(),
            _stLoss, new ToolStripSeparator(), _stBw, new ToolStripSeparator(),
            _stCrc, new ToolStripSeparator(), _stDirty,
        });
        Controls.Add(_status);
    }

    private void BuildEnableBar()
    {
        var bar = new Panel { Dock = DockStyle.Bottom, Height = Dpi.Px(62), BackColor = Theme.Panel, Padding = Dpi.Pad(10, 8, 10, 8) };

        _lampSteer = new LampControl { Caption = "舵机", Location = new Point(Dpi.Px(12), Dpi.Px(4)), Size = Dpi.Sz(104, 26) };
        _lampMotor = new LampControl { Caption = "电机", Location = new Point(Dpi.Px(12), Dpi.Px(32)), Size = Dpi.Sz(104, 26) };
        _lampMaster = new LampControl { Caption = "整体", Location = new Point(Dpi.Px(128), Dpi.Px(4)), Size = Dpi.Sz(104, 26) };

        _btnSteer = new Button { Text = "舵机使能", Location = new Point(Dpi.Px(240), Dpi.Px(8)), Width = Dpi.Px(88), Height = Dpi.Px(34) };
        _btnMotor = new Button { Text = "电机使能", Location = new Point(Dpi.Px(336), Dpi.Px(8)), Width = Dpi.Px(88), Height = Dpi.Px(34) };
        _btnMaster = new Button { Text = "整体使能", Location = new Point(Dpi.Px(432), Dpi.Px(8)), Width = Dpi.Px(88), Height = Dpi.Px(34) };
        _btnEStop = new Button { Text = "急停 (Space)", Location = new Point(Dpi.Px(540), Dpi.Px(6)), Width = Dpi.Px(130), Height = Dpi.Px(38) };

        _btnEStop.ForeColor = Theme.Err;
        _btnEStop.Font = new Font("微软雅黑", 10.5f, FontStyle.Bold);

        _btnSteer.Click += (_, __) => ToggleBit(EnableBits.Steer);
        _btnMotor.Click += (_, __) => ToggleBit(EnableBits.Motor);
        _btnMaster.Click += (_, __) => ToggleBit(EnableBits.Master);
        _btnEStop.Click += (_, __) => EmergencyStop();

        var lb = new Label
        {
            Text = "整体使能为最高优先级：它断开时，舵机/电机位无论怎么点都不生效。\r\n" +
                   "链路中断超过 500 ms，车端应自行停机（车端责任，上位机只负责发指令）。",
            Location = new Point(Dpi.Px(690), Dpi.Px(8)),
            Size = Dpi.Sz(620, 44),
            ForeColor = Theme.TextDim,
        };

        bar.Controls.AddRange(new Control[] { _lampSteer, _lampMotor, _lampMaster, _btnSteer, _btnMotor, _btnMaster, _btnEStop, lb });
        Controls.Add(bar);
    }

    private void BuildBody()
    {
        var main = new SplitContainer
        {
            Dock = DockStyle.Fill,
            Orientation = Orientation.Vertical,
            SplitterWidth = 4,
            BackColor = Theme.Border,
        };

        // ---- 左：参数 ----
        var leftHost = new Panel { Dock = DockStyle.Fill, BackColor = Theme.Panel };
        var paramHead = new Panel { Dock = DockStyle.Top, Height = Dpi.Px(28), BackColor = Theme.PanelAlt };
        paramHead.Controls.Add(new Label
        {
            Text = "  参数（滑条实时下发 / 数字框精确输入）",
            Dock = DockStyle.Fill,
            TextAlign = ContentAlignment.MiddleLeft,
            ForeColor = Theme.Text,
        });

        var paramFoot = new Panel { Dock = DockStyle.Bottom, Height = Dpi.Px(62), BackColor = Theme.Panel };

        _lblDirty = new Label
        {
            Text = "改动 0 项未保存",
            Location = new Point(Dpi.Px(10), Dpi.Px(4)),
            Size = Dpi.Sz(260, 18),
            ForeColor = Theme.Warn,
        };

        var btnRefresh = new Button { Text = "刷新参数表", Location = new Point(Dpi.Px(10), Dpi.Px(26)), Width = Dpi.Px(96), Height = Dpi.Px(28) };
        var btnSave = new Button { Text = "存 Flash", Location = new Point(Dpi.Px(112), Dpi.Px(26)), Width = Dpi.Px(84), Height = Dpi.Px(28) };
        var btnDef = new Button { Text = "恢复默认", Location = new Point(Dpi.Px(202), Dpi.Px(26)), Width = Dpi.Px(84), Height = Dpi.Px(28) };

        btnRefresh.Click += (_, __) => SendCmd(Cmd.GetTable);
        btnSave.Click += (_, __) =>
        {
            SendCmd(Cmd.SaveParams);
            _paramPanel.ClearDirty();
            UpdateDirtyLabel();
        };
        btnDef.Click += (_, __) =>
        {
            if (MessageBox.Show("把面板上的值恢复为默认？\n（车端也会收到恢复命令）", "确认",
                    MessageBoxButtons.OKCancel, MessageBoxIcon.Question) != DialogResult.OK) return;
            SendCmd(Cmd.LoadParams);
            _paramPanel.RestoreDefaults();
        };

        paramFoot.Controls.AddRange(new Control[] { _lblDirty, btnRefresh, btnSave, btnDef });

        _paramPanel = new ParamPanel { Dock = DockStyle.Fill };
        _paramPanel.ParamChanged += OnParamChanged;

        leftHost.Controls.Add(_paramPanel);
        leftHost.Controls.Add(paramFoot);
        leftHost.Controls.Add(paramHead);

        // ---- 右：波形 + 右侧栏 ----
        var right = new SplitContainer
        {
            Dock = DockStyle.Fill,
            Orientation = Orientation.Vertical,
            SplitterWidth = 4,
            BackColor = Theme.Border,
        };

        // 波形区
        var waveHost = new Panel { Dock = DockStyle.Fill, BackColor = Theme.Bg };
        var waveBar = new Panel { Dock = DockStyle.Top, Height = Dpi.Px(34), BackColor = Theme.Panel };

        var lbLayout = new Label { Text = "布局", AutoSize = true, Location = new Point(Dpi.Px(8), Dpi.Px(8)), ForeColor = Theme.TextDim };
        var b1 = new Button { Text = "1×1", Location = new Point(Dpi.Px(44), Dpi.Px(4)), Width = Dpi.Px(46), Height = Dpi.Px(26) };
        var b2 = new Button { Text = "2×1", Location = new Point(Dpi.Px(94), Dpi.Px(4)), Width = Dpi.Px(46), Height = Dpi.Px(26) };
        var b4 = new Button { Text = "2×2", Location = new Point(Dpi.Px(144), Dpi.Px(4)), Width = Dpi.Px(46), Height = Dpi.Px(26) };
        b1.Click += (_, __) => ApplyLayout(1);
        b2.Click += (_, __) => ApplyLayout(2);
        b4.Click += (_, __) => ApplyLayout(4);

        var lbWin = new Label { Text = "时间窗", AutoSize = true, Location = new Point(Dpi.Px(206), Dpi.Px(8)), ForeColor = Theme.TextDim };
        _cbWindow = new ComboBox { DropDownStyle = ComboBoxStyle.DropDownList, Width = Dpi.Px(68), Location = new Point(Dpi.Px(254), Dpi.Px(4)) };
        _cbWindow.Items.AddRange(new object[] { "5 s", "10 s", "15 s", "30 s", "60 s" });
        _cbWindow.SelectedIndex = 2;
        _cbWindow.SelectedIndexChanged += (_, __) =>
        {
            double[] w = { 5, 10, 15, 30, 60 };
            foreach (var wv in _waves) if (wv != null) wv.TimeWindow = w[_cbWindow.SelectedIndex];
        };

        _ckAutoY = new CheckBox { Text = "自动量程", Checked = true, Location = new Point(Dpi.Px(334), Dpi.Px(6)), AutoSize = true, ForeColor = Theme.Text };
        _ckAutoY.CheckedChanged += (_, __) => { foreach (var wv in _waves) if (wv != null) wv.AutoScale = _ckAutoY.Checked; };

        _btnPause = new Button { Text = "暂停", Location = new Point(Dpi.Px(428), Dpi.Px(4)), Width = Dpi.Px(58), Height = Dpi.Px(26) };
        _btnPause.Click += (_, __) =>
        {
            bool p = !_waves[0].Paused;
            foreach (var wv in _waves) if (wv != null) wv.Paused = p;
            _btnPause.Text = p ? "继续" : "暂停";
        };

        var btnPreset = new Button { Text = "套用预设分图", Location = new Point(Dpi.Px(494), Dpi.Px(4)), Width = Dpi.Px(108), Height = Dpi.Px(26) };
        btnPreset.Click += (_, __) => ApplyPlotPresets();

        waveBar.Controls.AddRange(new Control[] { lbLayout, b1, b2, b4, lbWin, _cbWindow, _ckAutoY, _btnPause, btnPreset });

        _waveHost = new Panel { Dock = DockStyle.Fill, BackColor = Theme.Bg };

        right.Panel1.Controls.Add(_waveHost);
        right.Panel1.Controls.Add(waveBar);

        // 右侧栏：数值 + 通道勾选
        var sideHost = new Panel { Dock = DockStyle.Fill, BackColor = Theme.Panel, AutoScroll = true };
        var sideHead = new Panel { Dock = DockStyle.Top, Height = Dpi.Px(28), BackColor = Theme.PanelAlt };
        sideHead.Controls.Add(new Label
        {
            Text = "  实时值 / 通道显隐",
            Dock = DockStyle.Fill,
            TextAlign = ContentAlignment.MiddleLeft,
            ForeColor = Theme.Text,
        });

        _valHost = new Panel { Dock = DockStyle.Top, Height = Dpi.Px(200), BackColor = Theme.Panel };
        BuildValueBoxes(_valHost);

        // ★ 行程标定：把转向推到左右机械硬限位各记一次"速度积分"，
        //   得到 (角度 ↔ 速度积分) 的定量映射。夹在"实时值"和"通道显隐"之间。
        _calPanel = new TravelCalPanel(_store, OnSetMagZero, LogLine);

        _chkHost = new FlowLayoutPanel
        {
            Dock = DockStyle.Fill,       // 先加 → 最后布局 → 填剩余空间
            FlowDirection = FlowDirection.TopDown,
            WrapContents = false,
            AutoScroll = true,
            BackColor = Theme.Panel,
            Padding = Dpi.Pad(8, 4, 4, 6),
        };
        BuildChannelChecks();

        sideHost.Controls.Add(_chkHost);
        sideHost.Controls.Add(_valHost);
        sideHost.Controls.Add(_calPanel);
        sideHost.Controls.Add(sideHead);
        right.Panel2.Controls.Add(sideHost);

        main.Panel1.Controls.Add(leftHost);
        main.Panel2.Controls.Add(right);
        Controls.Add(main);

        _mainSplit = main;
        _rightSplit = right;

        Load += (_, __) => ApplyRuntimeLayout();

        void ApplyRuntimeLayout()
        {
            try
            {
                main.SplitterDistance = Dpi.Px(320);
                right.SplitterDistance = Math.Max(Dpi.Px(400), right.Width - Dpi.Px(240));
            }
            catch { }
        }
        _applyRuntimeLayout = ApplyRuntimeLayout;
    }

    private SplitContainer _mainSplit, _rightSplit;
    private Action _applyRuntimeLayout;

    /// <summary>给自检用：把窗口布局推到"Load 之后"的状态（否则量到的是默认分隔位置）</summary>
    public void ApplyRuntimeLayoutForTest()
    {
        _applyRuntimeLayout?.Invoke();
        PerformLayout();
    }

    private void BuildValueBoxes(Panel host)
    {
        // ★ 各档案的关注点完全不同：全车看速度/航向，单电机看占空比/方向/链路，
        //   双电机速度环看左右轮的目标 vs 实测 + 编码器计数。
        //   下标必须与该档案的通道表一致，所以这里按档案取。
        int[] keys;
        string[] caps;
        string[] units;

        if (Defaults.IsMotor)
        {
            keys = new[] { 1, 0, 14, 13, 15, 6 };
            caps = new[] { "实际占空比", "目标占空比", "编码器角度", "角度 raw", "角度变化", "收包间隔" };
            units = new[] { "%", "%", "°", "raw", "raw", "ms" };
        }
        else if (Defaults.IsDualMotor)
        {
            // 速度环看的是"目标 vs 实测"，而 pos/dlt 这类大整数放进波形会被压成一条平线 → 走数值预览
            keys = new[] { 1, 4, 0, 3, 10, 11 };
            caps = new[] { "左轮实测", "右轮实测", "左轮目标", "右轮目标", "左轮计数", "右轮计数" };
            units = new[] { "rps", "rps", "rps", "rps", "cnt", "cnt" };
        }
        else
        {
            keys = new[] { 15, 20, 22, 13, 0, 30 };
            caps = new[] { "速度", "航向", "航向偏差", "转向角", "yaw", "控制周期" };
            units = new[] { "m/s", "°", "°", "°", "°", "µs" };
        }

        for (int i = 0; i < keys.Length; i++)
        {
            int col = i % 2, row = i / 2;
            var box = new Panel
            {
                Location = new Point(Dpi.Px(8 + col * 104), Dpi.Px(6 + row * 64)),
                Size = Dpi.Sz(98, 58),
                BackColor = Theme.PanelAlt,
            };
            box.Paint += (s, e) =>
            {
                using var pen = new Pen(Theme.Border);
                e.Graphics.DrawRectangle(pen, 0, 0, ((Panel)s).Width - 1, ((Panel)s).Height - 1);
            };

            box.Controls.Add(new Label
            {
                Text = caps[i],
                Location = new Point(Dpi.Px(6), Dpi.Px(4)),
                Size = Dpi.Sz(86, 16),
                ForeColor = Theme.TextDim,
            });

            var val = new Label
            {
                Text = "--",
                Location = new Point(Dpi.Px(6), Dpi.Px(20)),
                Size = Dpi.Sz(62, 30),               // ⚠ 别加宽：6+62=68，单位标签从 70 起，
                                                     //   加宽就会压上去（布局体检的"重叠"抓的就是这个）
                Font = new Font("Consolas", 14f, FontStyle.Bold),
                ForeColor = Theme.CurveOf(keys[i]),
            };
            box.Controls.Add(val);
            box.Controls.Add(new Label
            {
                Text = units[i],
                Location = new Point(Dpi.Px(70), Dpi.Px(32)),   // 值标签右沿在 68 → 让开了
                AutoSize = true,                     // ★ 固定 26px 宽在 150% 下装不下 "m/s"（需要 43）
                ForeColor = Theme.TextDim,
            });

            _valLabels[keys[i]] = val;
            host.Controls.Add(box);
        }
    }

    private void BuildChannelChecks()
    {
        for (int i = 0; i < Defaults.ChannelNames.Length; i++)
        {
            int ch = i;
            var ck = new CheckBox
            {
                Text = $"{i,2}  {Defaults.ChannelName(i)}",
                Width = Dpi.Px(176),
                Height = Dpi.Px(20),
                ForeColor = Theme.CurveOf(i),
                BackColor = Theme.Panel,
                Font = new Font("Consolas", 8.5f),
            };
            ck.CheckedChanged += (_, __) => OnChannelToggled(ch, ck.Checked);
            _chkBoxes[i] = ck;
            _chkHost.Controls.Add(ck);
        }
    }

    private void RadioDefaultChannels()
    {
        foreach (int c in Defaults.DefaultVisible)
            if (_chkBoxes.TryGetValue(c, out var ck)) ck.Checked = true;
    }

    // ------------------------------------------------------------------
    // 字体自愈守卫
    //
    // 实测（同一份代码连跑 5 次自检，有 2 次中招）：句柄创建 / 首次布局时，
    // 窗体字体偶尔会被系统改回默认字号（9pt → 15pt）。之后所有"字被吃掉"的
    // 布局断言都会亮红，而根因根本不在布局上 —— 这里幂等地兜一下。
    // ------------------------------------------------------------------
    protected override void OnHandleCreated(EventArgs e)
    {
        base.OnHandleCreated(e);
        EnsureBaseFont();
    }

    protected override void OnShown(EventArgs e)
    {
        base.OnShown(e);
        EnsureBaseFont();
    }

    private void EnsureBaseFont()
    {
        if (Math.Abs(Font.SizeInPoints - 9f) > 0.01f)
        {
            Font = new Font("微软雅黑", 9f);
        }
    }

    // ==================================================================
    // 定时器
    // ==================================================================
    private void BuildTimers()
    {
        // ★ 固定 30 FPS 重绘，与接收解耦 —— 这是本方案对参考工程的核心改进
        _uiTimer = new System.Windows.Forms.Timer { Interval = 33 };
        _uiTimer.Tick += (_, __) =>
        {
            foreach (var w in _waves) if (w != null) w.Tick();
            UpdateValueLabels();
            _calPanel?.RefreshValues();
        };

        _statTimer = new System.Windows.Forms.Timer { Interval = 500 };
        _statTimer.Tick += (_, __) => UpdateStats();

        // ★ 心跳：车端用"距上次下行帧 > lost_ms（默认 600ms）"判失联，失联即清零输出。
        //   而上位机只在用户操作时发命令 → 不补心跳就会"松手 0.6 秒电机自己停"，且完全静默。
        //   周期见 Heartbeat.IntervalMs 的注释（为什么是 100ms）。
        _hbTimer = new System.Windows.Forms.Timer { Interval = Heartbeat.IntervalMs };
        _hbTimer.Tick += (_, __) => Heartbeat.Beat(_link, NextToken);
        _hbTimer.Start();

        // 参数表分批合并：收到第一条后起 300 ms 定时器，到点整体重建一次界面
        _tableTimer = new System.Windows.Forms.Timer { Interval = 300 };
        _tableTimer.Tick += (_, __) =>
        {
            _tableTimer.Stop();
            if (_tableBuf.Count > 0)
            {
                _paramPanel.MergeTable(new List<ParamMeta>(_tableBuf));
                LogLine($"参数表已更新（{_tableBuf.Count} 项）");
                _tableBuf.Clear();
            }
        };
    }

    // ==================================================================
    // 链路
    // ==================================================================
    private void HookSimLog()
    {
        _sim.Log += m => UI(() => LogLine("[模拟] " + m));
    }

    private void HookLink()
    {
        _link.FrameReceived += OnFrame;
        _link.Log += m => UI(() => LogLine(m));
        _link.StateChanged += up => UI(() =>
        {
            _lampLink.State = up ? LampControl.LampState.On : LampControl.LampState.Off;
            _btnConnect.Text = up ? "断开" : "连接";
            _paramPanel.ReadOnly = !up;
            _stLink.Text = up ? "已连接 " + _link.RemoteDesc : "未连接";
        });

        HookSimLog();
    }

    private void ToggleConnect()
    {
        if (_link.IsOpen) { _link.Close(); return; }

        string ip = _txtIp.Text.Trim();
        if (!int.TryParse(_txtPort.Text.Trim(), out int port)) { Warn("端口号不合法"); return; }

        try
        {
            if (_cbProto.SelectedIndex == 0)
            {
                _link.ConnectTcp(ip, port);
            }
            else
            {
                // UDP：车端主动往这个端口上报，所以这里只需要"绑本地端口"，车端 IP 收到包后自动学
                _link.ConnectUdp(port);
            }

            SaveConfig();
            SendCmd(Cmd.GetTable);      // 连上就拉参数表
            SendCmd(Cmd.Ping);
        }
        catch (Exception ex)
        {
            Warn("连接失败：" + ex.Message + "\n\n若只是想看效果，先点「模拟车端」，再把 IP 改成 127.0.0.1。");
            _lampLink.State = LampControl.LampState.Fault;
        }
    }

    private void ToggleSim()
    {
        if (_sim.IsRunning) { _sim.Stop(); _btnSim.Text = "模拟车端"; return; }
        try
        {
            _sim.Port = int.TryParse(_txtPort.Text, out int p) ? p : 8086;
            _sim.Start();
            _btnSim.Text = "停止模拟";
            _txtIp.Text = "127.0.0.1";
            _cbProto.SelectedIndex = 0;
        }
        catch (Exception ex) { Warn("模拟启动失败：" + ex.Message); }
    }

    // ==================================================================
    // 帧分发
    // ==================================================================
    private void OnFrame(byte type, byte[] buf, int off)
    {
        switch ((FrameType)type)
        {
            case FrameType.Telemetry: ParseTelemetry(buf, off); break;
            case FrameType.ParamMeta: ParseParamMeta(buf, off); break;
            case FrameType.ParamValue:
                {
                    byte id = buf[off];
                    float v = Proto.GetF32(buf, off + 1);
                    UI(() => { _paramPanel.UpdateFromCar(id, v); UpdateDirtyLabel(); });
                    break;
                }
            case FrameType.Ack:
                {
                    ushort token = Proto.GetU16(buf, off);
                    double rtt = 0;
                    lock (_pendingCmd)
                    {
                        if (_pendingCmd.TryGetValue(token, out double t0))
                        {
                            rtt = _sw.Elapsed.TotalSeconds - t0;
                            _pendingCmd.Remove(token);
                        }
                    }
                    _ = rtt;
                    break;
                }
            case FrameType.Nak:
                {
                    ushort token = Proto.GetU16(buf, off);
                    byte cmd = buf[off + 2];
                    byte err = buf[off + 3];
                    UI(() => LogLine($"车端拒绝命令 0x{cmd:X2}（错误码 {err}）"));
                    break;
                }
            case FrameType.Event:
                {
                    int n = 0;
                    while (off + 1 + n < buf.Length && buf[off + 1 + n] != 0) n++;
                    string msg = System.Text.Encoding.UTF8.GetString(buf, off + 1, n);
                    UI(() => LogLine("[车] " + msg));
                    break;
                }
        }
    }

    private void ParseTelemetry(byte[] b, int o)
    {
        // 载荷：SEQ(2) TS(4) MASK(2) COUNT(2) DATA(4N)
        int seq = Proto.GetU16(b, o);
        uint ts = Proto.GetU32(b, o + 2);
        ushort mask = Proto.GetU16(b, o + 6);
        int cnt = Proto.GetU16(b, o + 8);

        if (cnt <= 0 || cnt > Proto.MAX_CHANNELS) return;

        var ch = new float[cnt];
        for (int i = 0; i < cnt; i++) ch[i] = Proto.GetF32(b, o + 10 + i * 4);

        // 丢包统计（按帧内 SEQ 连续性）
        _total++;
        if (_lastSeq >= 0)
        {
            int expect = (_lastSeq + 1) & 0xFFFF;
            if (seq != expect)
            {
                int gap = (seq - expect) & 0xFFFF;
                if (gap < 1000) _lost += gap;
            }
        }
        _lastSeq = seq;

        _store.Ensure(cnt);
        // 用 PC 本地时间做横轴基准（★ 不要用"第几个包"，否则丢包会让波形压缩/拉伸）
        _store.Append(ch, cnt, _sw.Elapsed.TotalSeconds, mask);
        _ = ts;
    }

    private void ParseParamMeta(byte[] b, int o)
    {
        var p = new ParamMeta
        {
            Id = b[o],
            Lo = Proto.GetF32(b, o + 1),
            Hi = Proto.GetF32(b, o + 5),
            Def = Proto.GetF32(b, o + 9),
            Step = Proto.GetF32(b, o + 13),
            Unit = Proto.GetStr(b, o + 17, 8),
            Name = Proto.GetStr(b, o + 25, 16),
        };
        string grpLabel = Proto.GetStr(b, o + 41, 32);
        int bar = grpLabel.IndexOf('|');
        if (bar > 0) { p.Group = grpLabel.Substring(0, bar); p.Label = grpLabel.Substring(bar + 1); }
        else { p.Group = "其他"; p.Label = grpLabel; }
        p.Value = p.Def;

        _tableBuf.Add(p);
        UI(() => { _tableTimer.Stop(); _tableTimer.Start(); });
    }

    // ==================================================================
    // 用户操作
    // ==================================================================
    private void OnParamChanged(ParamMeta p, float v)
    {
        if (!_link.IsOpen) { LogLine("未连接，参数未下发：" + p.Label); return; }
        SendCmd(Cmd.SetParam, p.Id, v);
        UpdateDirtyLabel();
    }

    private void OnChannelToggled(int ch, bool on)
    {
        foreach (var w in _waves)
        {
            if (w == null) continue;
            if (on) w.Shown.Add(ch); else w.Shown.Remove(ch);
        }
        // 单图模式下所有勾选都进 1 号图；4 图模式下按预设分配见 ApplyPlotPresets
    }

    private void ToggleBit(EnableBits bit)
    {
        _enable ^= bit;
        // 整体使能断开时，两位子使能一并清掉
        if ((_enable & EnableBits.Master) == 0) _enable &= ~(EnableBits.Steer | EnableBits.Motor);
        SendEnable();
    }

    private void EmergencyStop()
    {
        _enable = EnableBits.None;
        SendEnable();
        LogLine("★ 急停：已发送使能位 = 0");
    }

    private void SendEnable()
    {
        UpdateEnableLamps();
        if (!_link.IsOpen) { LogLine("未连接，使能指令未下发"); return; }
        var f = Proto.BuildSetEnable(_enable | EnableBits.Telemetry, NextToken());
        _link.Send(f);
        LogLine("下发使能 = 0x" + ((ushort)_enable).ToString("X2"));
    }

    private void UpdateEnableLamps()
    {
        bool master = (_enable & EnableBits.Master) != 0;
        bool anyLive = _link.IsOpen;

        _lampMaster.State = !anyLive ? LampControl.LampState.Fault
                         : master ? LampControl.LampState.On : LampControl.LampState.Off;
        _lampSteer.State = master && (_enable & EnableBits.Steer) != 0 ? LampControl.LampState.On : LampControl.LampState.Off;
        _lampMotor.State = master && (_enable & EnableBits.Motor) != 0 ? LampControl.LampState.On : LampControl.LampState.Off;

        _btnSteer.Text = (_enable & EnableBits.Steer) != 0 ? "舵机 已使能" : "舵机使能";
        _btnMotor.Text = (_enable & EnableBits.Motor) != 0 ? "电机 已使能" : "电机使能";
        _btnMaster.Text = master ? "整体 已使能" : "整体使能";
    }

    private ushort NextToken()
    {
        _token++;
        if (_token == 0) _token = 1;
        lock (_pendingCmd)
        {
            _pendingCmd[_token] = _sw.Elapsed.TotalSeconds;
            if (_pendingCmd.Count > 256) _pendingCmd.Clear();   // 防止泄漏
        }
        return _token;
    }

    private void SendCmd(Cmd cmd, byte pid = 0, float val = 0f)
    {
        if (!_link.IsOpen) return;
        _link.Send(Proto.BuildCommand(cmd, pid, val, NextToken()));
    }

    // ==================================================================
    // 布局
    // ==================================================================
    private void ApplyLayout(int n)
    {
        _layout = n;
        _waveHost.SuspendLayout();
        _waveHost.Controls.Clear();
        foreach (var w in _waves ?? Array.Empty<WavePanel>()) w?.Dispose();

        _waves = new WavePanel[n];
        var grid = new TableLayoutPanel
        {
            Dock = DockStyle.Fill,
            ColumnCount = n == 1 ? 1 : 2,
            RowCount = n == 1 ? 1 : n == 2 ? 1 : 2,
            BackColor = Theme.Bg,
            Padding = new Padding(2),
        };
        for (int i = 0; i < grid.ColumnCount; i++) grid.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 100f / grid.ColumnCount));
        for (int i = 0; i < grid.RowCount; i++) grid.RowStyles.Add(new RowStyle(SizeType.Percent, 100f / grid.RowCount));

        for (int i = 0; i < n; i++)
        {
            var wp = new WavePanel
            {
                Store = _store,
                Dock = DockStyle.Fill,
                Margin = new Padding(2),
                TimeWindow = _cbWindow.SelectedIndex >= 0 ? new[] { 5.0, 10, 15, 30, 60 }[_cbWindow.SelectedIndex] : 15,
                AutoScale = _ckAutoY?.Checked ?? true,
                Paused = _btnPause?.Text == "继续",
            };
            _waves[i] = wp;
            grid.Controls.Add(wp, grid.ColumnCount == 1 ? 0 : i % 2, grid.RowCount == 1 ? 0 : i / 2);
        }

        _waveHost.Controls.Add(grid);
        _waveHost.ResumeLayout();

        // 把当前勾选的通道分配到各子图
        foreach (var kv in _chkBoxes) OnChannelToggled(kv.Key, kv.Value.Checked);
        if (_layout > 1) ApplyPlotPresets();
        UpdateWaveTitles();
    }

    private void ApplyPlotPresets()
    {
        if (_waves == null || _waves.Length <= 1) return;
        for (int i = 0; i < _waves.Length; i++)
        {
            _waves[i].Shown.Clear();
            if (i < Defaults.PlotPresets.Length)
                foreach (int c in Defaults.PlotPresets[i]) _waves[i].Shown.Add(c);
        }
        UpdateWaveTitles();
    }

    private void UpdateWaveTitles()
    {
        if (_waves == null) return;
        for (int i = 0; i < _waves.Length; i++)
        {
            if (_waves.Length == 1) _waves[i].Title = "全部勾选通道";
            else if (i < Defaults.PlotPresetNames.Length) _waves[i].Title = Defaults.PlotPresetNames[i];
            else _waves[i].Title = "图 " + (i + 1);
        }
    }

    // ==================================================================
    // 刷新与统计
    // ==================================================================
    private void UpdateValueLabels()
    {
        foreach (var kv in _valLabels)
        {
            float v = _store.Latest(kv.Key);
            kv.Value.Text = float.IsNaN(v) ? "--"
                          : Math.Abs(v) >= 1000 ? v.ToString("0")
                          : Math.Abs(v) >= 100 ? v.ToString("0.0")
                          : v.ToString("0.00");
        }
    }

    /// <summary>按英文名找参数 ID —— 车端的表可能和本地默认表不同，所以按名字查而不是写死 ID。</summary>
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

    private void UpdateStats()
    {
        double now = _sw.Elapsed.TotalSeconds;
        double dt = now - _lastStatT;
        if (dt < 0.2) return;
        _lastStatT = now;

        long f = _link.FramesOk, b = _link.BytesIn;
        double rate = (f - _lastFrames) / dt;
        double bw = (b - _lastBytes) / dt;
        _lastFrames = f; _lastBytes = b;

        double loss = _total > 0 ? 100.0 * _lost / _total : 0;

        _stRate.Text = $"帧率 {rate:0.0} Hz";
        _stLoss.Text = $"丢包 {loss:0.000}%";
        _stBw.Text = $"带宽 {bw / 1024.0:0.0} KB/s";
        _stCrc.Text = $"CRC 错 {_link.FramesBadCrc}";

        if (_link.FramesBadCrc > 0) _stCrc.ForeColor = Theme.Err;
        else _stCrc.ForeColor = Theme.TextDim;

        if (loss > 1) _stLoss.ForeColor = Theme.Warn;
        else _stLoss.ForeColor = Theme.TextDim;

        UpdateDirtyLabel();
        UpdateEnableLamps();
    }

    private void UpdateDirtyLabel()
    {
        int d = _paramPanel.DirtyCount;
        _lblDirty.Text = d == 0 ? "无未保存改动" : $"改动 {d} 项未保存";
        _lblDirty.ForeColor = d == 0 ? Theme.TextDim : Theme.Warn;
        _stDirty.Text = "参数改动 " + d;
    }

    private void LogLine(string s)
    {
        if (_log == null) return;
        _log.AppendText($"[{DateTime.Now:HH:mm:ss.fff}] {s}\r\n");
        if (_log.Lines.Length > 500)
            _log.Text = string.Join("\r\n", _log.Lines.Skip(_log.Lines.Length - 300));
        _log.SelectionStart = _log.TextLength;
        _log.ScrollToCaret();
    }

    private RichTextBox _log;

    private void Warn(string s) => MessageBox.Show(s, "提示", MessageBoxButtons.OK, MessageBoxIcon.Warning);

    // ==================================================================
    // 配置持久化
    // ==================================================================
    private static string CfgPath =>
        Path.Combine(AppContext.BaseDirectory, "karthost.cfg");

    private void LoadConfig()
    {
        try
        {
            if (!File.Exists(CfgPath)) return;
            foreach (var line in File.ReadAllLines(CfgPath))
            {
                var kv = line.Split('=', 2);
                if (kv.Length != 2) continue;
                switch (kv[0])
                {
                    case "ip": _txtIp.Text = kv[1]; break;
                    case "port": _txtPort.Text = kv[1]; break;
                    case "proto": _cbProto.SelectedIndex = kv[1] == "UDP" ? 1 : 0; break;
                    case "window": _cbWindow.SelectedIndex = Math.Clamp(int.Parse(kv[1]), 0, 4); break;
                }
            }
        }
        catch { }
    }

    private void SaveConfig()
    {
        try
        {
            File.WriteAllLines(CfgPath, new[]
            {
                "ip=" + _txtIp.Text.Trim(),
                "port=" + _txtPort.Text.Trim(),
                "proto=" + (_cbProto.SelectedIndex == 0 ? "TCP" : "UDP"),
                "window=" + _cbWindow.SelectedIndex,
            });
        }
        catch { }
    }

    // ==================================================================
    protected override void OnKeyDown(KeyEventArgs e)
    {
        if (e.KeyCode == Keys.Space) { EmergencyStop(); e.Handled = true; }
        base.OnKeyDown(e);
    }

    protected override void OnFormClosing(FormClosingEventArgs e)
    {
        EmergencyStop();          // 关窗先停机，避免车带着使能状态跑
        try { _uiTimer?.Stop(); _statTimer?.Stop(); _tableTimer?.Stop(); _hbTimer?.Stop(); } catch { }
        _sim.Stop();
        _link.Close();
        SaveConfig();
        base.OnFormClosing(e);
    }
}
