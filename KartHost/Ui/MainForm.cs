using System.Diagnostics;
using System.Text;

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
    private ComboBox _cbProfile;             // 档案选择：全车 / 单电机 / 双电机速度环 / 转向闭环 / 惯导+路表
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

    // ---- 二维轨迹（移植自 3.1imu_trails_mode_pc 的 BoardView） ----
    private SplitContainer _waveSplit;        // 上=波形，下=轨迹
    private BoardView _board;

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
    private Button _btnRoute;                 // 「读路表」：让车端 dump 整张路表
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

    // ---- ★ 路表 dump 会话（点「读路表」→ 车端分帧发 → 全部收齐后画到轨迹面板 + 存 CSV）----
    private readonly SortedDictionary<int, RouteProto.Rec> _rtRecs = new();
    private RouteProto.Head _rtHead;
    private bool _rtHaveHead;
    private int _rtDeclared = -1;            // 表头声明的点数
    private int _rtChunkFrames;              // 收到多少个数据帧
    private double _rtLastRxMs;              // 最近收到路表帧的时刻（ms，_sw 计时）
    // 上一次收到表头的「身份」（点数 + 圈长）：用来判断这次读的是不是同一张表。
    // 同一张 ⇒ **保留已收到的点、把洞补上**（一次 dump 上百帧，UDP 丢一帧很常见）；
    // 不同 ⇒ 清空重来。
    private int _rtPrevCount = -1;
    private double _rtPrevLapM = double.NaN;
    private bool _rtActive;

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
        _cbProfile.Items.AddRange(new object[] { "卡丁车（32 通道）", "单电机验证（12 通道）", "双电机速度环（20 通道）", "转向闭环（26 通道）", "惯导+路表（32 通道）", "卡丁·遥控（39 通道）", "标定·精简（18 通道）" });
        // ★ 用 Defaults.ProfileIndex 而不是硬写 0 —— 这样下拉框显示的永远是"实际生效的档案"。
        //   （--shot-form steer 这类"构造前就换好档案"的场景下，硬写 0 会让下拉框标签和真实档案对不上；
        //     此处事件还没挂，不会触发 OnProfileChanged。）
        _cbProfile.SelectedIndex = Defaults.ProfileIndex;
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
    ///
    /// ★ 2026-10-14 放宽：**已连接时也允许切**。原先禁止的理由是"连着切会让 ChannelStore
    ///   里的老数据错位"，但 `SwitchProfileTo()` 下面本来就会 `_store.Reset()`，
    ///   老数据已经清干净了 —— 禁止只会造成一个死结：
    ///   「通道名错位」这件事**只有在连着的时候才看得见**，却要求"先断开再切"，
    ///   于是现场就卡在"看图知道错了、但改不了"（2026-10-14 实测 `kz.csv` 就是这么来的）。
    ///   仍然要求**停掉模拟车端**：模拟器是按通道数建缓冲的，重建会打断正在跑的仿真。
    /// </summary>
    private void OnProfileChanged()
    {
        HostProfile want = _cbProfile.SelectedIndex switch
        {
            1 => HostProfile.Motor,
            2 => HostProfile.DualMotor,
            3 => HostProfile.SteerLoop,
            4 => HostProfile.InsRoute,
            5 => HostProfile.KartRC,
            6 => HostProfile.Calib,
            _ => HostProfile.Kart,
        };
        SwitchProfileTo(want, "下拉框");
    }

    /// <summary>
    /// 真正执行一次档案切换。<paramref name="by"/> 只用于日志（"下拉框" / "点了波形上的红条"）。
    /// 返回是否真的切了。
    /// </summary>
    private bool SwitchProfileTo(HostProfile want, string by)
    {
        if (want == Defaults.Current || _inProfileSwitch) return false;

        if (_sim.IsRunning)
        {
            _inProfileSwitch = true;
            _cbProfile.SelectedIndex = Defaults.ProfileIndex;      // 改回去（会再次触发本函数，靠 _inProfileSwitch 挡住）
            _inProfileSwitch = false;
            LogLine("请先停止「模拟车端」再切换档案 —— 切换会按新通道数重建模拟器。");
            return false;
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
        _calPanel?.ResolveChannels();            // 换档案后重新找通道（找不到会**连面板一起隐藏**）
        if (_calPanel != null && !_calPanel.Visible)
            LogLine("「行程标定」面板已隐藏：本档案没有速度积分(mag_sum)通道 —— 它是「单电机验证」专用的；"
                  + "闭环档案的标定请用底部「实车标定」窗口。");
        _store.Reset();
        if (_board != null)
        {
            // ★ 26/27/28 只在「惯导+路表」档案里是位姿；别的档案里是毫不相干的通道，
            //   不挡住就会画出一条完全意义的"乱轨迹"，而且不会报错。
            _board.PoseAvailable = Defaults.IsInsRoute;
            _board.ClearTrail();
        }
        ApplyLayout(_layout);
        RadioDefaultChannels();
        _paramPanel.SetParams(Defaults.BuildDefaultParams());

        // 下拉框跟着走 —— 一键切换（点波形红条）不走下拉框，不显式同步的话标签会留在旧档案上
        if (_cbProfile.SelectedIndex != Defaults.ProfileIndex)
        {
            _inProfileSwitch = true;
            _cbProfile.SelectedIndex = Defaults.ProfileIndex;
            _inProfileSwitch = false;
        }

        UpdateChannelMatch();               // 帧 COUNT 没变，但"当前档案"变了 ⇒ 红条可能该出现/消失

        LogLine($"档案已切换：{Defaults.ProfileDisplayName}，共 {Defaults.ChannelNames.Length} 个遥测通道"
              + $"（{by}）。");
        return true;
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

        // ★ 实车标定窗口：两组开环/角度阶跃 + CSV 记录（工程 9 的落地标定用）。
        //   只发参数（SET_PARAM），不改协议、不新增命令。
        var btnCalib = new Button { Text = "实车标定", Location = new Point(Dpi.Px(678), Dpi.Px(6)), Width = Dpi.Px(122), Height = Dpi.Px(38) };
        btnCalib.Click += (_, __) =>
        {
            using var f = new CalibForm(new CalibHostAdapter(this));
            f.ShowDialog(this);
        };

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
            Location = new Point(Dpi.Px(812), Dpi.Px(8)),
            Size = Dpi.Sz(470, 44),
            ForeColor = Theme.TextDim,
        };

        bar.Controls.AddRange(new Control[] { _lampSteer, _lampMotor, _lampMaster, _btnSteer, _btnMotor, _btnMaster, _btnEStop, btnCalib, lb });
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

        // ★ 导出 CSV：把**当前勾选的那些通道**在缓冲里的整段倒出来。
        //   存在的理由很直白 —— 屏幕上那条波形是画出来的（自动量程 + 像素 + 抗锯齿），
        //   要拿它算超调/时间常数就是猜；通道值本来就在缓冲里，直接倒成 CSV 才是数。
        var btnCsv = new Button { Text = "导出 CSV", Location = new Point(Dpi.Px(610), Dpi.Px(4)), Width = Dpi.Px(96), Height = Dpi.Px(26) };
        btnCsv.Click += (_, __) => OnExportCsv();

        waveBar.Controls.AddRange(new Control[] { lbLayout, b1, b2, b4, lbWin, _cbWindow, _ckAutoY, _btnPause, btnPreset, btnCsv });

        _waveHost = new Panel { Dock = DockStyle.Fill, BackColor = Theme.Bg };

        // ---- 下：二维实时轨迹（上=波形 / 下=轨迹，分隔条可拖） ----
        var splitWB = new SplitContainer
        {
            Dock = DockStyle.Fill,
            Orientation = Orientation.Horizontal,   // 水平分隔条 => 上下两个窗格
            SplitterWidth = 4,
            BackColor = Theme.Border,
            // ⚠ 不要在这里设 Panel1MinSize / Panel2MinSize：此时容器还是默认小尺寸，
            //   设一个比它当前高度还大的最小值会**当场抛 InvalidOperationException**
            //   （"SplitterDistance 必须在 Panel1MinSize 和 Width - Panel2MinSize 之间"）。
            //   最小值统一留在 ApplyRuntimeLayout 里按真实高度夹一次。
        };

        var boardBar = new Panel { Dock = DockStyle.Top, Height = Dpi.Px(30), BackColor = Theme.Panel };
        var lbTrack = new Label { Text = "实时轨迹", AutoSize = true, Location = new Point(Dpi.Px(8), Dpi.Px(7)), ForeColor = Theme.TextDim };
        var btnFit = new Button { Text = "自动适应", Location = new Point(Dpi.Px(74), Dpi.Px(2)), Width = Dpi.Px(84), Height = Dpi.Px(26) };
        var btnClr = new Button { Text = "清空轨迹", Location = new Point(Dpi.Px(162), Dpi.Px(2)), Width = Dpi.Px(84), Height = Dpi.Px(26) };
        var btnZero = new Button { Text = "车端归零", Location = new Point(Dpi.Px(250), Dpi.Px(2)), Width = Dpi.Px(84), Height = Dpi.Px(26) };
        var btnRoute = new Button { Text = "读路表", Location = new Point(Dpi.Px(338), Dpi.Px(2)), Width = Dpi.Px(84), Height = Dpi.Px(26) };
        _btnRoute = btnRoute;
        // ★ 路表管理：串口（DAP 虚拟串口）那一侧的"看 / 上传 / 下载 / 出图"。
        //   与左边那个「读路表」（走 WiFi 只把车端**当前**表 dump 上来）不是一条路：
        //   那条只管当前表；这条管 Flash 里的**全部槽**，而且能往车里灌、能选当前槽。
        var btnRouteMgr = new Button { Text = "路表管理", Location = new Point(Dpi.Px(426), Dpi.Px(2)), Width = Dpi.Px(84), Height = Dpi.Px(26) };
        btnRouteMgr.Click += (_, __) => { using var f = new RouteManagerForm(); f.ShowDialog(this); };
        var lbTrackHint = new Label
        {
            // ⚠ 这行是 AutoSize 的，太长会顶出父容器（LayoutAudit 会报「横向越界」）。
            //   栏内可用宽 ≈ 1294 − 518 = 776 px（150% 缩放下），所以控制在 ~30 个汉字以内。
            Text = "滚轮缩放 / 拖动平移 / 双击自适应（青虚线 = 车端路表）",
            AutoSize = true,
            Location = new Point(Dpi.Px(518), Dpi.Px(7)),
            ForeColor = Theme.TextDim,
        };

        btnFit.Click += (_, __) => _board.AutoFit = true;
        btnClr.Click += (_, __) => { _board.ClearTrail(); LogLine("轨迹已清空（车端位姿不受影响）"); };
        btnZero.Click += (_, __) => OnTrackZero();
        btnRoute.Click += (_, __) => OnReadRoute();

        // ⚠⚠ 建了控件**必须挂进父容器**，否则它只是个没人持有的对象 —— 不报错、不显示。
        //     「路表管理」第一版就漏在这里：btnRouteMgr 建好、事件也接好，就是没进 AddRange。
        boardBar.Controls.AddRange(new Control[] { lbTrack, btnFit, btnClr, btnZero, btnRoute, btnRouteMgr, lbTrackHint });

        _board = new BoardView { Dock = DockStyle.Fill, Store = _store, PoseAvailable = Defaults.IsInsRoute };

        splitWB.Panel2.Controls.Add(_board);      // Fill 先加
        splitWB.Panel2.Controls.Add(boardBar);    // Top 后加
        splitWB.Panel1.Controls.Add(_waveHost);

        right.Panel1.Controls.Add(splitWB);       // Fill 先加
        right.Panel1.Controls.Add(waveBar);       // Top 后加

        _waveSplit = splitWB;

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

            // 波形 : 轨迹 ≈ 6 : 4（轨迹太矮就看不出形状了）。
            // ⚠ SplitterDistance 只能在控件已有实际高度之后设，而且要**自己夹一次**：
            //   WinForms 的合法区间是 [Panel1MinSize, Height - Panel2MinSize - SplitterWidth]，
            //   越界就抛 InvalidOperationException。这里给个小的最小值兜底，再 Clamp。
            try
            {
                int h = splitWB.Height;
                int lo = Dpi.Px(90), hi = h - Dpi.Px(90) - splitWB.SplitterWidth;
                if (hi > lo)
                {
                    splitWB.Panel1MinSize = lo;
                    splitWB.Panel2MinSize = lo;
                    splitWB.SplitterDistance = Math.Clamp((int)(h * 0.62), lo, hi);
                }
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
        // ★ 各档案的关注点完全不同 ⇒ 取数方案按档案走，统一放在 Defaults.ValueBoxes()。
        //   ⚠⚠ 这里原来是一串 if/else，以"卡丁车"那份老下标收尾；2026-10-13 加了
        //   「卡丁·遥控」「标定·精简」两档却没跟着加分支，两档全落到老下标上 ——
        //   遥控档案的"速度"格读的是 ch15 = flip_r（换向次数，现场读到 21.0）、
        //   "控制周期"格读的是 ch30 = rc_scale（油门行程 ‰，读到 200.0）。
        //   不越界、不报错、数字还挺像，所以必须靠"每档案一张表 + 自检断言"来防。
        var vb = Defaults.ValueBoxes();
        int[] keys = vb.Keys;
        string[] caps = vb.Caps;
        string[] units = vb.Units;

        if (caps.Length != 6 || units.Length != 6)
            throw new InvalidOperationException("ValueBoxes() 必须刚好给 6 格（下标/标签/单位三个数组等长）");

        // ⚠ 单位字符串宽度受数值框限制（约 147 逻辑像素 @1.5x）：
        //   实测 "deg" 会把右边界顶到 149 > 147（比例字体里 d/e/g 比 c/n/t 宽）⇒ 布局体检亮红；
        //   改用度数符号 "°" / "-" —— 更窄也更规范。完整的 "deg"/"duty" 在波形图例里能看到
        //   （那份用的是 Defaults.ChannelUnits）。
        //   ⚠ 别为了"显示更多字"去加宽数值标签：值标签右沿在 68、单位从 70 起，
        //     加宽就压上去（布局体检的"重叠"抓的就是这个）。

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
            _board?.Tick();                      // 二维轨迹与波形同频（30 FPS），共用一次快照拉取
            UpdateValueLabels();
            _calPanel?.RefreshValues();
            RouteTimeoutCheck();                 // ★ 路表 dump 的收尾超时（车端若半路丢帧，不能永远等）
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
            case FrameType.RouteHead: OnRouteHead(buf, off); break;
            case FrameType.RouteData: OnRouteData(buf, off); break;
            case FrameType.RouteEnd: OnRouteEnd(buf, off); break;
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

    // ==================================================================
    // ★ 路表 dump（车端 → 上位机）
    //
    //  时序：本机发 Cmd.RouteDump → 车端 ACK → 每 10ms 一帧
    //        RouteHead(32B) → RouteData×(first,n,records) → RouteEnd(count,lap)
    //
    //  ⚠ 走 UDP 时**允许丢帧**：这里把每条记录按"全局下标"存进 SortedDictionary，
    //    所以丢哪一块都能看出来（缺号 + declared 对不上），而不是"默默拼错"。
    // ==================================================================
    private void OnReadRoute()
    {
        if (_link == null || !_link.IsOpen)
        {
            Warn("先连接车端（或开「模拟车端」）再读路表");
            return;
        }

        // ★ 不在这里 Clear：一次 dump 上百帧（车端 16 条/帧时 1728 点 = 108 帧），
        //   丢一帧就留个洞。保留已收到的点，再点一次「读路表」正好把洞补上
        //   （OnRouteHead 里按"表头身份"判断是不是同一张表，不是才清）。
        _rtHaveHead = false;
        _rtDeclared = -1;
        _rtChunkFrames = 0;
        _rtActive = true;
        _rtLastRxMs = _sw.Elapsed.TotalMilliseconds;

        btnRouteText("读路表…");
        SendCmd(Cmd.RouteDump);
        LogLine($"已发「读路表」(0x08)：等车端分帧发回（现已收到 {_rtRecs.Count} 点，缺号会补）");
    }

    private void btnRouteText(string s)
    {
        if (_btnRoute != null) { _btnRoute.Text = s; _btnRoute.Enabled = s == "读路表"; }
    }

    private void OnRouteHead(byte[] b, int o)
    {
        var h = RouteProto.ReadHead(b, o);
        _rtHead = h;
        _rtLastRxMs = _sw.Elapsed.TotalMilliseconds;

        if (h.Magic2 != RouteProto.Magic || h.Ver2 != RouteProto.Ver || h.RecLen != RouteProto.RecSize)
        {
            FinishRoute(false, $"表头不匹配 magic=0x{h.Magic2:X8} ver={h.Ver2} rec_len={h.RecLen}");
            return;
        }

        // ★★ 是同一张表就**不清空**：把这次读到的点并进上次的结果里，正好补上丢帧留下的洞。
        //    只有"点数/圈长变了"才说明车上换了另一张表，那时才重头来。
        bool sameTable = (_rtPrevCount == (int)h.Count)
                         && !double.IsNaN(_rtPrevLapM)
                         && Math.Abs(_rtPrevLapM - h.LapM) < 0.001;   // 同一张表：圈长精确到 mm 都一样
        if (!sameTable && _rtRecs.Count > 0)
        {
            _rtRecs.Clear();
            LogLine($"（车上换了一张表：{_rtPrevCount} 点 → {h.Count} 点，清空上次的部分数据）");
        }
        _rtPrevCount = (int)h.Count;
        _rtPrevLapM = h.LapM;

        _rtHaveHead = true;
        _rtDeclared = (int)h.Count;
        LogLine($"路表表头：{h.Count} 点，间距 {h.DsMm} mm，圈长 {h.LapM:F2} m，" +
                $"起点位姿 ({h.OriginX:F3},{h.OriginY:F3}) yaw={h.OriginYawDeg:F1}°" +
                (h.Full ? "  ⚠ 车端标记「录满」" : ""));
    }

    private void OnRouteData(byte[] b, int o)
    {
        if (!_rtHaveHead) return;                    // 表头丢了就整次作废（FinishRoute 会报）
        _rtLastRxMs = _sw.Elapsed.TotalMilliseconds;

        int first = (int)Proto.GetU32(b, o);
        int n = Proto.GetU16(b, o + 4);
        if (n <= 0) return;

        _rtChunkFrames++;
        for (int i = 0; i < n; i++)
        {
            int idx = first + i;
            _rtRecs[idx] = RouteProto.ReadRec(b, o + 6 + i * RouteProto.RecSize);
        }
    }

    private void OnRouteEnd(byte[] b, int o)
    {
        int count = (int)Proto.GetU32(b, o);
        int lapMm = (int)Proto.GetU32(b, o + 4);
        FinishRoute(true, $"车端结束帧：{count} 点 / {lapMm / 1000f:F2} m");
    }

    /// <summary>车端半路停了（丢帧/掉线）—— 不能永远等下去</summary>
    private void RouteTimeoutCheck()
    {
        if (!_rtActive) return;
        if (_sw.Elapsed.TotalMilliseconds - _rtLastRxMs < 3000) return;
        // ★ 把"收到多少"一起报出来（2026-10-11）。
        //   只报"没收到"的话，三种完全不同的原因会长得一模一样：
        //     · 车端一帧都没发出去（单帧太大被模块退回）→ _rtChunkFrames == 0
        //     · 发了但一路丢包                              → _rtChunkFrames > 0 但 _rtRecs 少
        //     · 车端根本没在 dump（表空/状态机没起）        → 连表头都没有
        //   有这三个数，下次一眼就知道往哪儿查。
        FinishRoute(false, $"3 秒没收到新的路表帧 —— 已收 {_rtRecs.Count}/" +
                           $"{(_rtDeclared > 0 ? _rtDeclared : -1)} 点、{_rtChunkFrames} 个数据帧" +
                           (_rtChunkFrames == 0
                               ? "（★ 一个数据帧都没到：多半是单帧太大被模块退回，车端会自动把分块减半，再点一次「读路表」）"
                               : "（帧到了但点不全：丢包，再点一次「读路表」会补缺号）"));
    }

    private void FinishRoute(bool ok, string why)
    {
        if (!_rtActive) return;
        _rtActive = false;
        btnRouteText("读路表");

        if (!ok || !_rtHaveHead)
        {
            LogLine("读路表失败：" + why);
            return;
        }

        // 连续性检查：缺号就是丢帧（UDP 下很正常），报出来而不是装作没发生
        var miss = new List<int>();
        for (int i = 0; i < _rtDeclared; i++) if (!_rtRecs.ContainsKey(i)) miss.Add(i);

        var pts = new List<(float x, float y, float yaw)>(_rtRecs.Count);
        int prevIdx = -1;
        foreach (var kv in _rtRecs)
        {
            if (kv.Key >= _rtDeclared) continue;      // 多余的点不要
            _ = prevIdx; prevIdx = kv.Key;
            pts.Add((kv.Value.X, kv.Value.Y, kv.Value.YawDeg));
        }

        LogLine(why);
        if (miss.Count > 0)
        {
            LogLine($"⚠ 缺 {miss.Count} 点（首缺 #{miss[0]}）—— UDP 丢包。点「读路表」可再读一次；" +
                    "轨迹面板画的是收到的那部分，形状仍可判读。");
        }

        if (pts.Count < 2)
        {
            LogLine("收到点数不足 2，不画。车端路表可能是空的（先在菜单里 RecTbl 录一圈，或 RtLoadF 从 Flash 载入）");
            return;
        }

        // 闭环判定：首尾点相距 < 0.5 m 就当闭环（车的往返轨迹不会这么近地回到起点）
        float cdx = pts[^1].x - pts[0].x, cdy = pts[^1].y - pts[0].y;
        bool closed = (cdx * cdx + cdy * cdy) < 0.25f;

        _board.SetRoute(pts, closed);
        LogLine($"路表已画到轨迹面板：{pts.Count} 点 / 声明 {_rtDeclared} 点 / {_rtChunkFrames} 个数据帧；" +
                $"起点对齐平移 ({_board.RouteOffset.x * 1000:F0},{_board.RouteOffset.y * 1000:F0}) mm" +
                (closed ? "；首尾闭合" : ""));

        string csv = SaveRouteCsv(pts);
        if (csv != null) LogLine("已存 CSV：" + csv);
    }

    /// <summary>把收到的路表写成 CSV（含表头注释行），便于用 Python/Excel 复核与和地图中线对比</summary>
    private string SaveRouteCsv(List<(float x, float y, float yaw)> pts)
    {
        try
        {
            string dir = AppContext.BaseDirectory;
            string path = Path.Combine(dir, "route_dump.csv");
            var sb = new StringBuilder();
            sb.AppendLine("# KartHost route table dump");
            sb.AppendLine($"# magic=0x{RouteProto.Magic:X8} ver={RouteProto.Ver} declared={_rtDeclared} got={pts.Count}");
            sb.AppendLine($"# ds_mm={_rtHead.DsMm} lap_mm={_rtHead.LapMm} flags={_rtHead.Flags}");
            sb.AppendLine($"# origin_mm=({_rtHead.OriginXmm},{_rtHead.OriginYmm}) origin_yaw_cdeg={_rtHead.OriginYawCdeg}");
            sb.AppendLine("# 坐标是【录制起点车体系】：x 向右、y 向前，单位米；yaw 是相对录制起点的度");
            sb.AppendLine("idx,x_m,y_m,yaw_deg");
            for (int i = 0; i < pts.Count; i++)
                sb.AppendLine($"{i},{pts[i].x:F4},{pts[i].y:F4},{pts[i].yaw:F2}");
            File.WriteAllText(path, sb.ToString(), Encoding.UTF8);
            return path;
        }
        catch (Exception ex)
        {
            LogLine("存 CSV 失败：" + ex.Message);
            return null;
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

        // ★★ 帧里的通道数与当前档案不符 ⇒ **通道名整体错位**（最阴的一类：不越界、不报错、数字还挺像）。
        //    2026-10-13 实测：车端 tel_sel=0（全量）而上位机档案是「标定·精简」——
        //    速度环那 6 列恰好对得上（两种布局前 6 个相同），但状态列全错位：
        //    表头写 `enable` 的列其实是 `pos_l`（冒出 −14 万那种数）、`ol_en` 其实是 `flip_l`。
        //    只在"不符的通道数变了"时说一次，并写清怎么改。
        //
        //    ★ 2026-10-14 现场第二例（更隐蔽）：还是"档案停在「标定·精简」、车端发全量"，
        //    但这次错位落在**增益回读**上 —— 表头 `k_v` 实为 ch16=`online`（恒 1）、
        //    `k_z` 实为 ch17=`rx_age`（毫秒计数器，被 100 ms 心跳清零 ⇒ 一条锯齿）。
        //    用户看着"k_z 一直变"来问，说明**只写日志根本不够** ⇒ 加波形上的红条（见 UpdateChannelMatch）。
        if (cnt != Defaults.ChannelNames.Length && cnt != _warnedCnt)
        {
            _warnedCnt = cnt;
            LogLine($"⚠⚠ 帧里 {cnt} 个通道，而当前档案「{Defaults.ProfileDisplayName}」是 "
                  + $"{Defaults.ChannelNames.Length} 个 —— **通道名会整体错位**（数据不越界，但含义全错）。");
            LogLine("   改法：① 让车端 `tel_sel` 与档案一致（「标定·精简」= 1 发 18 通道；其余 = 0 发全量）；"
                  + "② 或把「档案」下拉框切到与帧一致的那档。"
                  + "③ 两边都升到 2026-10-14 之后的固件/上位机：全量 39、精简 18，"
                  + "老固件发 37/16 时这里会一直报。");
            LogCandidateProfiles(cnt);
        }
        else if (cnt == Defaults.ChannelNames.Length)
        {
            _warnedCnt = -1;                 // 对上了就复位，"下次再错"还能再提示
        }

        // 用 PC 本地时间做横轴基准（★ 不要用"第几个包"，否则丢包会让波形压缩/拉伸）
        _store.Append(ch, cnt, _sw.Elapsed.TotalSeconds, mask);
        _ = ts;

        if (cnt != _frameChCount) { _frameChCount = cnt; UpdateChannelMatch(); }
    }

    private int _warnedCnt = -1;

    /// <summary>最近一帧里的通道数（-1 = 还没收到过）。</summary>
    private int _frameChCount = -1;

    /// <summary>
    /// 2026-10-14 之前，工程 9「全量档」是 **37** 通道（加增益回读之后才是 39）。
    /// 上位机侧从来没有"37 通道"这个档案 ⇒ 拿旧固件跑时**一个档案都对不上**。
    /// 这个常量只为了把那句话说得具体（"重刷车端"），不是为了给旧固件配一个档案。
    /// </summary>
    private const int LegacyFullChCount = 37;

    /// <summary>日志里把"这个 COUNT 到底对得上哪个档案"说清楚（唯一命中就点名建议）。</summary>
    private void LogCandidateProfiles(int cnt)
    {
        var c = Defaults.ProfilesWithChannelCount(cnt);
        if (c.Count == 0)
        {
            // ★ 一个都对不上：最可能是**车端固件太旧** —— 2026-10-14 之前的全量档是 **37** 通道
            //   （加增益回读之后才变 39），而上位机侧从来就没有"37 通道"这个档案。
            LogLine(cnt == LegacyFullChCount
                ? $"   ⇒ 没有任何档案是 {cnt} 通道 —— **车端还是 2026-10-14 之前的旧固件**（全量档那时是 37，"
                + "加增益回读之后才是 39）。要这块功能必须先把车端两个核都重刷。"
                : $"   ⇒ 没有任何档案是 {cnt} 通道，先确认车端固件版本与档案表是否配套。");
            return;
        }
        string names = string.Join(" / ", c.Select(ProfName));
        LogLine(c.Count == 1
            ? $"   ⇒ 帧里 {cnt} 通道唯一对应「{names}」：把「档案」下拉切到它即可（或直接点波形上的红条）。"
            : $"   ⇒ 帧里 {cnt} 通道对应 {c.Count} 个档案（{names}）—— **不能替你猜**，请手动选。");
    }

    private static string ProfName(HostProfile p) => p switch
    {
        HostProfile.Motor => "单电机验证",
        HostProfile.DualMotor => "双电机速度环",
        HostProfile.SteerLoop => "转向闭环",
        HostProfile.InsRoute => "惯导+路表",
        HostProfile.KartRC => "卡丁·遥控",
        HostProfile.Calib => "标定·精简",
        _ => "卡丁车",
    };

    /// <summary>
    /// 把"帧 COUNT 与档案是否一致"这个结论刷到**每个波形**上（红条 + 一键切换回调）。
    /// 帧数、档案数、当前档案三样任一变了都要调它。
    /// </summary>
    private void UpdateChannelMatch()
    {
        string warn = null;
        var cand = _frameChCount > 0 ? Defaults.ProfilesWithChannelCount(_frameChCount) : new List<HostProfile>();
        bool canFix = cand.Count == 1 && cand[0] != Defaults.Current;

        if (_frameChCount > 0 && _frameChCount != Defaults.ChannelNames.Length)
        {
            // ⚠ 文案要短：2×2 分图下面板只有 ~280 逻辑像素宽，长句子会被折成三行把图压没。
            warn = $"⚠ 帧里 {_frameChCount} 通道 ≠ 本档案 {Defaults.ChannelNames.Length} 通道 —— 通道名整体错位";
            if (canFix) warn += $"｜点此切到「{ProfName(cand[0])}」";
            else if (cand.Count == 0)
                warn += _frameChCount == LegacyFullChCount
                      ? "｜**车端是旧固件**（37 通道），要增益回读得先把两个核重刷"
                      : "｜没有档案是这个通道数 —— 先核对车端固件版本";
            else warn += "｜请手动选「档案」";
        }

        foreach (var w in _waves ?? Array.Empty<WavePanel>())
        {
            if (w == null) continue;
            w.WarnText = warn;
            w.WarnClick = canFix ? (Action)OnWarnClick : null;
        }
    }

    /// <summary>点波形上的红条：切到与帧 COUNT 唯一匹配的那个档案。</summary>
    private void OnWarnClick()
    {
        var cand = Defaults.ProfilesWithChannelCount(_frameChCount);
        if (cand.Count != 1) { LogLine("帧里的通道数对应多个档案，无法自动切换，请手动选。"); return; }
        SwitchProfileTo(cand[0], "点了波形上的红条");
    }

    // ---- 自检用的两个口子（与 ApplyRuntimeLayoutForTest / BoardForTest 同一套路） ----
    /// <summary>注入"帧里的通道数"并刷新红条（不经过链路，纯粹为了能测这条 UI 通路）。</summary>
    internal void SetFrameCountForTest(int cnt) { _frameChCount = cnt; UpdateChannelMatch(); }

    /// <summary>当前波形面板（红条挂在它们身上）。</summary>
    internal WavePanel[] WavesForTest => _waves;

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
        // ★★ 角度类参数统一夹在 ±27°（与车端 steer_abs.h 的 STEER_ABS_LIMIT_DEG_HARD 一致）。
        //    车端 `steer_ctrl_eff_limit()` 还会再夹一次，这里夹是为了"根本不要发出不会生效的指令"，
        //    以及万一连的是一张旧参数表（上限还是 30/32）时不会把 30 度发出去。
        if (Defaults.IsSteerAngleParam(p.Name))
        {
            float cv = Defaults.ClampSteerDeg(v);
            if (Math.Abs(cv - v) > 1e-4f)
            {
                LogLine($"角度按 ±{Defaults.SteerLimitDeg:0.#}° 夹紧：{p.Label}  {v:0.##} -> {cv:0.##}");
                v = cv;
            }
        }

        if (!_link.IsOpen) { LogLine("未连接，参数未下发：" + p.Label); return; }
        SendCmd(Cmd.SetParam, p.Id, v);

        // ★★ 「转向|内环使能」从 0 打到 1 = 车端清转向保护锁存（`steer_ctrl_set_angle_enable`
        //    的 0→1 上升沿是**唯一**复位途径）。但车端在那一拍会用**当时的实测角**覆写目标转角
        //    （防超调），于是滑条上显示的 10.5° 与车上的目标就对不上了 ——
        //    现场表现就是"我按提示关了再开，还是不动"。所以这里**立刻补发一次当前目标转角**。
        if (p.Name == "steer_en" && v > 0.5f)
        {
            ParamMeta tgt = null;
            foreach (var q in _paramPanel.Params) { if (q.Name == "steer_tgt") { tgt = q; break; } }
            if (tgt != null)
            {
                SendCmd(Cmd.SetParam, tgt.Id, tgt.Value);
                LogLine($"内环使能已打开（车端清锁存）⇒ 目标转角已补发一次：{tgt.Value:0.##}°");
            }
        }
        UpdateDirtyLabel();
    }

    /// <summary>
    /// 把主界面的三样能力交给标定窗口（数据仓库 / 参数表 / 写参数），不暴露别的。
    /// 单独一个适配器而不是让 MainForm 直接实现接口：标定窗口只需要这三样，
    /// 边界清楚，也不会因为主界面加了个同名成员就悄悄改掉行为。
    /// </summary>
    private sealed class CalibHostAdapter : ICalibHost
    {
        private readonly MainForm _f;
        public CalibHostAdapter(MainForm f) { _f = f; }

        public ChannelStore Store => _f._store;
        public bool LinkOpen => _f._link.IsOpen;
        public IReadOnlyList<ParamMeta> Params => _f._paramPanel.Params;

        public void WriteParam(ParamMeta p, float v)
        {
            if (p == null) return;
            p.Value = v;
            _f._paramPanel.UpdateFromCar(p.Id, v);   // 让面板上的数字与滑条跟上（否则界面与实际不一致）
            _f.OnParamChanged(p, v);
        }

        public void Log(string s) => _f.LogLine(s);
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

        // 波形是新建的 ⇒ 红条（通道名可能错位）必须重新贴上去，否则改一下分图它就没了
        UpdateChannelMatch();
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
        CheckSteerFault();

        foreach (var kv in _valLabels)
        {
            float v = _store.Latest(kv.Key);
            // ★ 数值框只有 62 逻辑像素（Consolas 14pt Bold）—— 150% 缩放下**最多装 5 个字符**。
            //   所以格式必须按"字符数"封顶，而不是按量级：
            //     "0.00" 用 |v|<10  → 最长 "-9.99"（5 字符）✓
            //     "0.0"  用 <100    → 最长 "-99.9"（5 字符）✓
            //   原来 |v|∈[1,100) 一律 "0.00"，于是 "-16.00"（6 字符）会被剪掉尾巴，
            //   而这件事只在"刚好有 10~100 量级的负数"时才现形 —— 正是布局体检抓出来的那类静默问题。
            kv.Value.Text = float.IsNaN(v) ? "--"
                          : Math.Abs(v) >= 1000 ? v.ToString("0")
                          : Math.Abs(v) >= 100 ? v.ToString("0.0")
                          : Math.Abs(v) >= 10 ? v.ToString("0.0")
                          : v.ToString("0.00");
        }
    }

    /// <summary>
    /// 转向保护锁存的"翻译器"。
    ///
    /// 车端 `steer_status` = **bit0 自检通过 | (fault &lt;&lt; 1)**，fault: `0x01` 反馈失效 / `0x02` 顶死或反转。
    /// 锁存后车端**把转向输出钉 0**，现场看到的就是"上位机给了指令、前电机一动不动" ——
    /// 而数值框里只有一个 "5.00"，没人能想到是这个原因（2026-10-13 现场就是这么卡住的）。
    /// 这里把它翻成人话、把那一格标红，并直接给出复位步骤。
    ///
    /// ⚠ 只在"故障值发生变化"时打日志（每 33 ms 一条会把日志刷爆）。
    /// </summary>
    private void CheckSteerFault ()
    {
        int st = ChannelIndex("steer_status");
        if (st < 0) st = ChannelIndex("steer_st");          // 标定·精简档案里的名字
        if (st < 0) return;

        float v = _store.Latest(st);
        if (float.IsNaN(v)) return;

        int flt = ((int)v >> 1) & 0x03;
        if (flt == _steerFaultShown) return;
        _steerFaultShown = flt;

        if (_valLabels.TryGetValue(st, out var lb))
            lb.ForeColor = (flt != 0) ? Theme.Err : Theme.CurveOf(st);

        if (flt == 0) { LogLine("转向状态回到正常。"); return; }

        // ★ bit1（车端 fault 0x01）= 锁存故障 ⇒ 输出已被钉 0；
        //   bit2（车端 fault 0x02）= 顶死/无进展 ⇒ **只是告警，车端没有停输出**（2026-10-13 改）。
        //   两者对现场的处置完全不同，日志必须分开说，否则又会把人带偏。
        if (0 != (flt & 1))
        {
            LogLine($"⚠ 转向**锁存故障**：{SteerFaultText((int)v)}（转向状态={v:0}）");
            LogLine("   ⇒ 车端把转向输出钉 0，这就是\"给指令但前电机不动\"的原因，指令本身是到了的。");
            LogLine("   复位：把参数「转向|内环使能」关到 0 再打开（唯一途径）。⚠ 复位那一拍车端会用**当时的实测角**");
            LogLine("   覆写目标转角，所以本机在打开内环使能时会自动补发一次「目标转角」；用别的方式复位请手动再动一下它。");
        }
        else
        {
            // 只有顶死告警：把三个数一起打出来 —— 下一次就不用猜"到底是谁没跟"
            LogLine($"⚠ 转向顶死告警：输出顶到限幅、而角度 0.6 s 不跟（转向状态={v:0}）"
                  + $" —— 目标 {Val("steer_tgt")} / 实测 {Val("steer_delta")} / 输出 {Val("steer_u")}。"
                  + "【只是告警：车端没有停输出，不会失能】");
            LogLine("   常见原因：机械卡住（轮子顶墙 / 连杆到底）、kp 与 u0 太小推不动、或供电带载塌陷。");
        }
    }

    /// <summary>取某通道最新值并格式化成短文本（找不到 / 没收过数据返回 "--"）。</summary>
    private string Val (string chName)
    {
        int i = ChannelIndex(chName);
        if (i < 0) return "--";
        float x = _store.Latest(i);
        return float.IsNaN(x) ? "--" : x.ToString("0.##");
    }

    /// <summary>
    /// 把 `steer_status` 的原始值翻成故障原因；**没有故障/告警返回空串**。
    /// ⚠ 位的顺序是最容易搞反的地方，所以单独抽成纯函数，自检直接断言
    /// （车端：`steer_status = self_check_ok | (fault &lt;&lt; 1)`；
    ///   fault `0x01` = 反馈失效【**锁存**，输出钉 0】、`0x02` = 顶死/无进展【**只告警**，不停输出】）。
    /// </summary>
    internal static string SteerFaultText (int bits)
    {
        int flt = (bits >> 1) & 0x03;

        if (flt == 0) return "";
        if (flt == 1) return "反馈失效 —— 绝对编码器读数无效（锁存故障，输出已钉 0）";
        if (flt == 2) return "顶死 / 无进展 —— 0.6 s 内 |err| 不收敛（只是告警，车端没有停输出）";
        return "反馈失效（锁存，输出钉 0） + 顶死告警";
    }

    private int _steerFaultShown = -1;

    /// <summary>按英文名找当前档案的遥测通道下标（档案不同名字也不同，所以按名字查）。</summary>
    private static int ChannelIndex (string name)
    {
        for (int i = 0; i < Defaults.ChannelNames.Length; i++)
            if (Defaults.ChannelName(i) == name) return i;
        return -1;
    }

    /// <summary>按英文名找参数 ID —— 车端的表可能和本地默认表不同，所以按名字查而不是写死 ID。</summary>
    private int ParamIdByName(string name)
    {
        foreach (var p in Defaults.BuildDefaultParams())
            if (p.Name == name) return p.Id;
        return -1;
    }

    // ---- 自检 / 出图用入口（internal：同程序集可见，不对外暴露）----
    internal ChannelStore StoreForTest => _store;
    internal BoardView BoardForTest => _board;

    /// <summary>「当前位置→0°」：把当前角度寄存器的原始值写进 mag_zero（车端算 deg 时要减它）。</summary>
    private void OnSetMagZero(float rawAngle)
    {
        int id = ParamIdByName("mag_zero");
        if (id < 0) { LogLine("参数表里没有 mag_zero —— 车端固件版本可能不匹配"); return; }
        SendCmd(Cmd.SetParam, (byte)id, rawAngle);
    }

    /// <summary>
    /// 「车端归零」：① 给车端发 ins_zero 边沿（x/y/yaw/里程一起归零）；② 本地轨迹基准与缓冲一起归零。
    ///
    /// ★ 两边必须一起做：只归车端的话，新点会从 (0,0) 重新开始，和旧轨迹之间被拉出一条贯穿全屏的假直线。
    /// ★ ins_zero 是"0→1 边沿"触发的（车端 main 里按边沿处理），所以**不能 1、0 连发两帧** ——
    ///   车端主循环可能在同一圈里把两帧都读完，只看到最终值 0，边沿就丢了、什么都不会发生。
    ///   这里隔 200ms 再拖回 0，保证边沿被看到，也让下一次点击还能再触发。
    /// </summary>
    private void OnTrackZero()
    {
        int id = ParamIdByName("ins_zero");
        if (id < 0)
        {
            LogLine("参数表里没有 ins_zero —— 车端固件版本可能不匹配（本次只归零本地轨迹）");
        }
        else
        {
            SendCmd(Cmd.SetParam, (byte)id, 1f);
            var t = new System.Windows.Forms.Timer { Interval = 200 };
            t.Tick += (_, __) =>
            {
                t.Stop(); t.Dispose();
                SendCmd(Cmd.SetParam, (byte)id, 0f);          // 拖回 0，便于下次再触发
            };
            t.Start();
            LogLine("已发 ins_zero（车端 x/y/航向/里程归零）");
        }
        _board.ResetReference();
        _board.ClearTrail();
    }

    /// <summary>
    /// 「导出 CSV」：把**所有波形上勾选的通道**在环形缓冲里的整段倒成 CSV。
    ///
    /// 为什么是"勾选的"而不是"全部"：勾选就是"我现在关心的这几路"，
    /// 导出来的表正好是屏幕上那张图的数据版本（同一个起点、同一段时间跨度）。
    /// 要换通道就改勾选再导一次，不做"全通道大宽表"（那种表列太多、看不动）。
    ///
    /// 通道集合取的是 <see cref="WavePanel.Shown"/>（面板真正在画的那一组）的并集，
    /// 而不是复读勾选框 —— 因为「套用预设分图」会直接改 Shown，
    /// 这时屏幕上显示的和勾选框上的已经不一样了，**以屏幕上显示为准**。
    /// </summary>
    private void OnExportCsv()
    {
        var set = new SortedSet<int>();
        if (_waves != null)
            foreach (var wv in _waves)
                if (wv != null)
                    foreach (int c in wv.Shown) set.Add(c);

        if (set.Count == 0) { Warn("波形里一个通道都没勾选，没东西可导。"); return; }
        if (_store.Count == 0) { Warn("缓冲里还没有数据 —— 先连上车端（或开「模拟车端」）再导。"); return; }

        using var dlg = new SaveFileDialog
        {
            Title = "导出波形数据（当前勾选的通道）",
            Filter = "CSV 文件|*.csv",
            FileName = System.IO.Path.GetFileName(CsvExport.DefaultPath()),
            InitialDirectory = System.IO.Path.GetDirectoryName(CsvExport.DefaultPath()),
        };
        if (dlg.ShowDialog(this) != DialogResult.OK) return;

        try
        {
            int rows = CsvExport.Dump(dlg.FileName, _store, set, out string info);
            LogLine($"已导出 {set.Count} 个通道 / {rows} 点 → {dlg.FileName}");
            LogLine("   通道：" + string.Join(", ", set.Select(c => Defaults.ChannelName(c) + "(" + c + ")")));
            if (!string.IsNullOrEmpty(info)) LogLine("   " + info);
        }
        catch (Exception ex)
        {
            Warn("导出失败：" + ex.Message);
        }
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
