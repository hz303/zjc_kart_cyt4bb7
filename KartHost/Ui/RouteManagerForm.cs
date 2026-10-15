using System.Drawing.Drawing2D;

namespace KartHost.Ui;

/// <summary>
/// 路表管理窗口 —— 一个窗口管完"看 / 上传 / 下载 / 出图"。
///
/// <para>通道走 <see cref="RouteLink"/>（串口 = DAP 虚拟串口，车端 UART_0）。
/// 与 WiFi 那条遥测链路**完全独立**：这里不发命令、不碰使能，只读写 Flash 里的路表槽。</para>
///
/// <para><b>槽的纪律</b>：槽 0 = 手推录制（车端 <c>RtSaveF</c> 只写它）；槽 1..N = 串口灌表。
/// 本窗口的"下载"只允许选 1..N-1 ⇒ 不可能覆盖手推的那张表。</para>
///
/// ⚠ 界面全部按 <see cref="Dpi.Px"/> 缩放（本机 150% ⇒ 不缩放会出现"按钮上的字被吃掉"），
///   并且 <c>AutoScaleMode = None</c>（默认的 Font 模式会缩两遍）。
/// </summary>
public sealed class RouteManagerForm : Form
{
    private readonly RouteLink _link = new();
    private ComboBox _cbPort, _cbBaud;
    private Button _btnOpen, _btnRefreshList, _btnUpload, _btnDownload, _btnSetActive, _btnLoadToCar;
    private Label _lbState, _lbDetail;
    private DataGridView _dgv;
    private TextBox _log;
    private RoutePlot _plot;
    private Panel _pLeft, _pRight;
    private Button _btnFit, _btnSaveBin, _btnCsv;
    private Button _btnStep, _btnLock;      // 内环辨识（阶跃测试 / 满舵半径）
    private CheckBox _cbAutoRead;

    private byte[] _image;                 // 最近一次拿到/准备发的那张镜像
    private RouteProto.Head _head;
    private List<RouteProto.Rec> _recs = new();
    private bool _busy;
    private int _shownSlot = -1;           // 图上画的这张表属于哪一槽（-1 = 还没画）
    private int _autoReadDone = -1;        // ★ 已经"自动读过"的槽（不论成败）—— 防失败后无限重试

    public RouteManagerForm()
    {
        // ★★ DPI 三件套，**顺序是硬要求**（同 MainForm，理由见 Ui/Dpi.cs 顶部注释）：
        //   ① 先关自动缩放：默认的 Font 模式会跟 Dpi 层的换算叠加 ⇒ 字号被缩两遍
        //   ② 再建句柄：DeviceDpi 这时才是**这台显示器**的真实值（不是被虚拟化的 96）
        //   ③ 之后才能用 Dpi.Px 换算尺寸
        // ⚠ 漏了这三步的症状（LayoutAudit 直接会报）：窗口按 96 DPI 建出来 ⇒
        //   150% 机器上按钮框比字小 ⇒「按钮上的字被吃掉」，而且开发机上是好的。
        AutoScaleMode = AutoScaleMode.None;
        _ = Handle;
        Dpi.Init(DeviceDpi);

        Text = "路表管理（串口 = DAP 虚拟串口，车端 UART_0）";
        StartPosition = FormStartPosition.CenterParent;
        Size = Dpi.Sz(1040, 700);
        MinimumSize = Dpi.Sz(900, 580);
        BackColor = Theme.Bg;
        ForeColor = Theme.Text;
        Font = new Font("Microsoft YaHei UI", 9f);

        BuildBody();
        BuildLog();
        // ⚠ 顺序有讲究：WinForms 的 Dock 是**后加的控件先占外边缘**。
        //   所以顶栏必须最后加，否则它会被挤到"左边栏+日志"之外的那块区域里去。
        BuildTop();

        Theme.Apply(this);
        RefreshPorts();
        Shown += (_, __) => LogLine("提示：车要先上电；DAP 插好；别的程序别占着这个 COM 口。");
        FormClosed += (_, __) => _link.Close();
    }

    // ================================================================ 界面
    private void BuildTop()
    {
        var bar = new Panel { Dock = DockStyle.Top, Height = Dpi.Px(46), BackColor = Theme.Panel, Padding = Dpi.Pad(8, 6, 8, 6) };

        var lb1 = new Label { Text = "串口", AutoSize = true, Location = new Point(Dpi.Px(8), Dpi.Px(15)), ForeColor = Theme.TextDim };
        _cbPort = new ComboBox { DropDownStyle = ComboBoxStyle.DropDown, Width = Dpi.Px(104), Location = new Point(Dpi.Px(44), Dpi.Px(11)) };
        _cbPort.Text = "COM33";
        var btnRp = new Button { Text = "刷新", Width = Dpi.Px(56), Height = Dpi.Px(26), Location = new Point(Dpi.Px(154), Dpi.Px(10)) };
        btnRp.Click += (_, __) => RefreshPorts();

        var lb2 = new Label { Text = "波特率", AutoSize = true, Location = new Point(Dpi.Px(222), Dpi.Px(15)), ForeColor = Theme.TextDim };
        _cbBaud = new ComboBox { DropDownStyle = ComboBoxStyle.DropDownList, Width = Dpi.Px(92), Location = new Point(Dpi.Px(272), Dpi.Px(11)) };
        _cbBaud.Items.AddRange(new object[] { "115200", "230400", "460800", "921600" });
        _cbBaud.SelectedIndex = 0;

        _btnOpen = new Button { Text = "打开", Width = Dpi.Px(72), Height = Dpi.Px(26), Location = new Point(Dpi.Px(374), Dpi.Px(10)) };
        _btnOpen.Click += (_, __) => ToggleOpen();

        _lbState = new Label { Text = "未打开", AutoSize = true, Location = new Point(Dpi.Px(460), Dpi.Px(15)), ForeColor = Theme.TextDim };

        // ★ 「选中即读」：点/选到哪一槽，就自动把那槽读回来画图。
        //   起因：第一版只画"上传过的"那张，而列表里明明显示槽 0 有 1742 点，
        //   图上却是"（还没有表）" ⇒ 用户以为坏了。列表里的元数据是**摘要**（来自 DIR），
        //   真正的点要再发一轮 SLOT_READ 才拿得到 —— 这一步不该让用户猜。
        // ⚠ 用 Dock.Right 而不是写死 x：`_lbState` 是 AutoSize 的，出图模式下它的文案更长
        //   （"（出图模式：演示数据，未连串口）"）⇒ 写死坐标会撞上，LayoutAudit 立刻报重叠。
        var autoPanel = new Panel { Dock = DockStyle.Right, Width = Dpi.Px(230), BackColor = Theme.Panel };
        _cbAutoRead = new CheckBox
        {
            Text = "选中即读（点哪一槽就出图）",
            AutoSize = true,
            Checked = true,
            Location = new Point(0, Dpi.Px(9)),
            ForeColor = Theme.TextDim,
        };
        _cbAutoRead.CheckedChanged += (_, __) => MaybeAutoRead();
        autoPanel.Controls.Add(_cbAutoRead);

        bar.Controls.AddRange(new Control[] { lb1, _cbPort, btnRp, lb2, _cbBaud, _btnOpen, _lbState, autoPanel });
        Controls.Add(bar);
    }

    private void BuildBody()
    {
        // 右侧：图
        _pRight = new Panel { Dock = DockStyle.Fill, BackColor = Theme.Bg };
        var plotBar = new Panel { Dock = DockStyle.Top, Height = Dpi.Px(30), BackColor = Theme.Panel };
        var lbPlot = new Label { Text = "路表图像", AutoSize = true, Location = new Point(Dpi.Px(8), Dpi.Px(7)), ForeColor = Theme.TextDim };
        _btnFit = new Button { Text = "自动适应", Location = new Point(Dpi.Px(90), Dpi.Px(2)), Width = Dpi.Px(92), Height = Dpi.Px(26) };
        _btnSaveBin = new Button { Text = "另存 .bin", Location = new Point(Dpi.Px(190), Dpi.Px(2)), Width = Dpi.Px(92), Height = Dpi.Px(26) };
        _btnCsv = new Button { Text = "导出 CSV", Location = new Point(Dpi.Px(290), Dpi.Px(2)), Width = Dpi.Px(92), Height = Dpi.Px(26) };
        _btnFit.Click += (_, __) => { _plot.AutoFit = true; _plot.Invalidate(); };
        _btnSaveBin.Click += (_, __) => SaveBin();
        _btnCsv.Click += (_, __) => SaveCsv();
        plotBar.Controls.AddRange(new Control[] { lbPlot, _btnFit, _btnSaveBin, _btnCsv });

        _plot = new RoutePlot { Dock = DockStyle.Fill };
        _pRight.Controls.Add(_plot);
        _pRight.Controls.Add(plotBar);

        // 左侧：槽列表 + 按钮 + 详情
        _pLeft = new Panel { Dock = DockStyle.Left, Width = Dpi.Px(566), BackColor = Theme.Panel, Padding = Dpi.Pad(8, 8, 8, 8) };

        _dgv = new DataGridView
        {
            Dock = DockStyle.Top,
            Height = Dpi.Px(214),
            AllowUserToAddRows = false,
            AllowUserToResizeRows = false,
            ReadOnly = true,
            MultiSelect = false,
            SelectionMode = DataGridViewSelectionMode.FullRowSelect,
            RowHeadersVisible = false,
            BackgroundColor = Theme.PanelAlt,
            BorderStyle = BorderStyle.None,
            AutoSizeColumnsMode = DataGridViewAutoSizeColumnsMode.None,
            EnableHeadersVisualStyles = false,
            ColumnHeadersHeightSizeMode = DataGridViewColumnHeadersHeightSizeMode.DisableResizing,
        };
        _dgv.ColumnHeadersHeight = Dpi.Px(26);
        _dgv.RowTemplate.Height = Dpi.Px(24);
        _dgv.ColumnHeadersDefaultCellStyle.BackColor = Theme.PanelAlt;
        _dgv.ColumnHeadersDefaultCellStyle.ForeColor = Theme.TextDim;
        _dgv.ColumnHeadersDefaultCellStyle.SelectionBackColor = Theme.PanelAlt;
        _dgv.DefaultCellStyle.BackColor = Theme.Panel;
        _dgv.DefaultCellStyle.ForeColor = Theme.Text;
        _dgv.DefaultCellStyle.SelectionBackColor = Theme.Accent;
        _dgv.DefaultCellStyle.SelectionForeColor = Color.White;
        _dgv.GridColor = Theme.Border;
        foreach (var (name, w, fill) in new[]
        {
            ("槽", 40, 0f), ("有效", 48, 0f), ("点数", 70, 0f), ("圈长 (m)", 84, 0f),
            ("点距", 60, 0f), ("CRC(记录)", 92, 0f), ("当前", 70, 1f),
        })
        {
            _dgv.Columns.Add(new DataGridViewTextBoxColumn
            {
                HeaderText = name,
                Width = Dpi.Px(w),
                FillWeight = fill == 0f ? 1f : fill,
                AutoSizeMode = fill == 0f
                    ? DataGridViewAutoSizeColumnMode.None
                    : DataGridViewAutoSizeColumnMode.Fill,
            });
        }

        var btns = new Panel { Dock = DockStyle.Top, Height = Dpi.Px(110), BackColor = Theme.Panel, Padding = Dpi.Pad(0, 8, 0, 0) };
        _btnRefreshList = MkBtn("刷新列表", 0, 0); _btnRefreshList.Click += (_, __) => RunAsync(DoList);
        _btnUpload = MkBtn("上传 车→PC", 1, 0); _btnUpload.Click += (_, __) => RunAsync(DoUpload);
        _btnDownload = MkBtn("下载 PC→车", 2, 0); _btnDownload.Click += (_, __) => OnDownloadClick();
        _btnSetActive = MkBtn("选为当前表", 0, 1); _btnSetActive.Click += (_, __) => RunAsync(DoSetActive);
        _btnLoadToCar = MkBtn("载入到车 RAM", 1, 1); _btnLoadToCar.Click += (_, __) => RunAsync(DoLoadToCar);
        var btnShow = MkBtn("显示图像", 2, 1); btnShow.Click += (_, __) => { _plot.AutoFit = true; _plot.Invalidate(); };
        // ★ 内环辨识（Polaris 七步工序的第 ① ② 步）。两枚按钮 = 两组固定预设，
        //   避免为三个参数再加一排 NumericUpDown（那会多出三个 DPI 归一化点）。
        _btnStep = MkBtn("阶跃测试(内环)", 0, 2);
        _btnStep.Click += (_, __) => OnStepTest(0.30, 400, 8.0);
        _btnLock = MkBtn("满舵半径测试", 1, 2);
        _btnLock.Click += (_, __) => OnStepTest(1.00, 3000, 6.0);
        var lbStepHint = new Label
        {
            Text = "← 车会真的动（先架空）",
            AutoSize = true,
            Location = new Point(Dpi.Px(324), Dpi.Px(76)),
            ForeColor = Theme.TextDim,
        };
        btns.Controls.AddRange(new Control[] { _btnRefreshList, _btnUpload, _btnDownload,
                                               _btnSetActive, _btnLoadToCar, btnShow,
                                               _btnStep, _btnLock, lbStepHint });

        _lbDetail = new Label
        {
            Dock = DockStyle.Fill,
            ForeColor = Theme.TextDim,
            Padding = Dpi.Pad(2, 10, 2, 2),
            Text = "还没读到表。\n"
                 + "① 打开串口 → ② 刷新列表 → ③ 选中某一槽\n"
                 + "　· 上传：把车里的表读回 PC（出图 / 存 .bin / 导 CSV）\n"
                 + "　· 下载：把 PC 的 route_car.bin 灌进车\n\n"
                 + "槽 0 = 手推录制（车端 RtSaveF 只写它）\n"
                 + "槽 1..6 = 串口灌表 ⇒ 下载写不到槽 0，不会覆盖手推表。",
        };

        _pLeft.Controls.Add(_lbDetail);
        _pLeft.Controls.Add(btns);
        _pLeft.Controls.Add(_dgv);

        Controls.Add(_pRight);
        Controls.Add(_pLeft);

        FillPlaceholderRows();

        // 选中哪一行就（可选地）把那槽读回来 —— 挂在这里而不是构造函数里，
        // 是为了让占位行的批量 Rows.Add 不触发一堆无意义的自动读。
        _dgv.SelectionChanged += (_, __) => MaybeAutoRead();
    }

    /// <summary>
    /// 「选中即读」：当前选中的槽**有表**、且**不是**图上正在画的那张时，自动拉回来画。
    /// <para>为什么需要它：列表里的点数/圈长是车端 DIR 回的**摘要**，点必须再发一轮
    /// SLOT_READ 才拿得到 ⇒ 不自动读的话，"有 1742 点"和"图上是空的"会同时出现在屏幕上。</para>
    /// <para>幂等：读完把 <see cref="_shownSlot"/> 设上 ⇒ 完成回调里再进来一次就直接返回，不会自激。</para>
    /// </summary>
    private void MaybeAutoRead()
    {
        if (_cbAutoRead == null || !_cbAutoRead.Checked) return;
        if (_busy || !_link.IsOpen) return;
        // ★ 车端不支持读槽就别反复试了 —— 否则每选中一行就刷一条"状态不允许"，
        //   把真正有用的日志淹掉。OPEN 时已经报过一次原因了。
        if (!_link.CanSlotRead) return;
        if (_dgv.SelectedRows.Count == 0) return;

        int slot = SelectedSlot();
        if (slot < 0 || slot == _shownSlot || slot == _autoReadDone) return;

        var row = _dgv.SelectedRows[0];
        if (row.Cells[1].Value?.ToString() != "有") return;   // 空槽没什么可读的

        // ★★ 先把"这一槽自动读过了"记上，**再去读**。
        //   为什么必须在读之前记：读失败时 `_shownSlot` 永远不会被设上（它在成功后的 Post 里），
        //   于是 `RunAsync` 的 finally → MaybeAutoRead() 会**再读一遍同一个槽** ⇒
        //   校验失败 → 重试 → 再失败 …… **死循环刷屏**（现场就是这样"卡在反复上传"的）。
        //   ⇒ 失败也是一次"读过了"；要重试就点「上传」（手动通道永远可用）或「刷新列表」（会清掉这个标记）。
        _autoReadDone = slot;
        RunAsync(DoUpload);
    }

    /// <summary>
    /// 还没连车时先把 7 行占位画出来 —— 否则空表格看着像"坏了"，而且现场一眼看不出槽 0 是干嘛的。
    /// 连上之后「刷新列表」会用真实内容整表替换。
    /// </summary>
    private void FillPlaceholderRows()
    {
        _dgv.Rows.Clear();
        for (int k = 0; k < _link.SlotCount; k++)
            _dgv.Rows.Add(k, "—", "", "", "", "", k == 0 ? "手推录制" : "灌表位");
    }

    private Button MkBtn(string text, int col, int row = 0)
        => new()
        {
            Text = text,
            // 3 列 × 2 行：列距 158 / 按钮宽 150 ⇒ 最右 8 + 2×158 + 150 = 474，
            // 左栏可用宽 = 566 − 16 = 550 ⇒ 不越界。（越界会被 LayoutAudit 立刻抓出来）
            Location = new Point(Dpi.Px(8 + col * 158), Dpi.Px(2 + row * 34)),
            Width = Dpi.Px(150),
            Height = Dpi.Px(28),
        };

    private void BuildLog()
    {
        _log = new TextBox
        {
            Dock = DockStyle.Bottom,
            Height = Dpi.Px(122),
            Multiline = true,
            ReadOnly = true,
            ScrollBars = ScrollBars.Vertical,
            BackColor = Theme.PanelAlt,
            ForeColor = Theme.TextDim,
            BorderStyle = BorderStyle.None,
            Font = new Font("Consolas", 8.5f),
        };
        Controls.Add(_log);
    }

    private void RefreshPorts()
    {
        var names = RouteLink.ListPorts();
        _cbPort.Items.Clear();
        _cbPort.Items.AddRange(names);
        if (names.Length > 0 && !names.Contains(_cbPort.Text)) _cbPort.Text = names.Contains("COM33") ? "COM33" : names[0];
        LogLine("可用串口：" + (names.Length == 0 ? "（一个都没有）" : string.Join(", ", names)));
    }

    private void ToggleOpen()
    {
        try
        {
            if (_link.IsOpen) { _link.Close(); SetBusy(false); }
            else
            {
                int baud = int.TryParse(_cbBaud.Text, out var b) ? b : 115200;
                _link.Open(_cbPort.Text.Trim(), baud);
                LogLine("已打开 " + _link.Describe);
                LogLine(_link.Ping());

                // ★ 车端能力位对不上就直接说清，别让用户去猜"为什么一点上传就状态不允许"。
                if (!_link.CanSlotRead)
                {
                    LogLine("!! 车端不支持「读槽」⇒ 「上传 车→PC」用不了（会自动跳过，不再反复重试）。");
                    LogLine("!! " + RouteLink.StaleFirmwareHint);
                    _cbAutoRead.Checked = false;
                    _cbAutoRead.Enabled = false;
                }
                else
                {
                    // 只撤销"我们自己关掉的那一次"，不覆盖用户主动取消勾选
                    if (!_cbAutoRead.Enabled) { _cbAutoRead.Checked = true; }
                    _cbAutoRead.Enabled = true;
                }
            }
        }
        catch (Exception ex) { LogLine("!! " + ex.Message); _link.Close(); }
        finally
        {
            _btnOpen.Text = _link.IsOpen ? "关闭" : "打开";
            _lbState.Text = _link.IsOpen ? _link.Describe : "未打开";
            _lbState.ForeColor = _link.IsOpen ? Theme.Ok : Theme.TextDim;
        }
    }

    private void SetBusy(bool busy)
    {
        _busy = busy;
        foreach (var b in new[] { _btnOpen, _btnRefreshList, _btnUpload, _btnDownload, _btnSetActive, _btnLoadToCar, _btnFit, _btnSaveBin, _btnCsv })
            if (b != null) b.Enabled = !busy;
        Cursor = busy ? Cursors.WaitCursor : Cursors.Default;
    }

    private void RunAsync(Action job)
    {
        if (_busy) return;
        if (!_link.IsOpen) { LogLine("先打开串口。"); return; }
        SetBusy(true);
        Task.Run(() =>
        {
            try { job(); }
            catch (Exception ex) { Post(() => LogLine("!! " + ex.Message)); }
            // ★ 干完活再补一次 MaybeAutoRead：刷新列表 / 选为当前表 都会在 _busy 期间
            //   重选行，那一次 SelectionChanged 被 _busy 挡掉了，所以必须在这里补。
            finally { Post(() => { SetBusy(false); MaybeAutoRead(); }); }
        });
    }

    private void Post(Action a) { if (IsHandleCreated) BeginInvoke(a); }
    private void LogLine(string s) { _log.AppendText(s + Environment.NewLine); }

    /// <summary>
    /// 只给"出图 / 自检"用：塞一张现造的镜像进来直接显示，**不碰串口**。
    /// （`--shot-route` 靠它出图 ⇒ 截图不依赖车、不依赖串口、不依赖任何外部文件。）
    /// </summary>
    public void LoadDemoImage(byte[] img)
    {
        LoadImage(img);
        _lbState.Text = "（出图模式：演示数据，未连串口）";
        LogLine("出图模式：已灌入一张现造的圆环表，仅用于看界面长相。");
    }

    private int SelectedSlot()
    {
        if (_dgv.SelectedRows.Count == 0) return -1;
        var v = _dgv.SelectedRows[0].Cells[0].Value;
        return v == null ? -1 : Convert.ToInt32(v);
    }

    // ================================================================ 动作
    private void DoList()
    {
        var list = _link.Dir(out int active, out int slotCount);
        Post(() =>
        {
            _dgv.Rows.Clear();
            foreach (var s in list)
            {
                _dgv.Rows.Add(s.Slot, s.Valid ? "有" : "—",
                              s.Valid ? s.Count.ToString() : "", s.Valid ? (s.LapMm / 1000.0).ToString("F2") : "",
                              s.Valid ? s.DsMm.ToString() : "", s.Valid ? "0x" + s.CrcR.ToString("X4") : "",
                              s.Active ? "★ 当前" : "");
            }
            if (_dgv.Rows.Count > 0)
            {
                int pick = Math.Max(0, Math.Min(_dgv.Rows.Count - 1, active));
                _dgv.Rows[pick].Selected = true;
                _dgv.CurrentCell = _dgv.Rows[pick].Cells[0];
            }
            LogLine($"槽位一览：{slotCount} 槽，当前 = 槽 {active}。槽 0 是手推录制，1..{slotCount - 1} 是灌表位。");

            // 刷新完把"已画的是哪一槽"作废 ⇒ 紧接着的 MaybeAutoRead 会重读选中的那槽。
            // 这样「刷新列表」一步就能把摘要 + 图一起给到，不用再单独点一次「上传」。
            // ★ 同时清掉自动读的"读过了"标记：刷新是用户明确要求重来一次的动作。
            _shownSlot    = -1;
            _autoReadDone = -1;
        });
    }

    private void DoUpload()
    {
        int slot = SelectedSlot();
        if (slot < 0) { Post(() => LogLine("先在上面选一行。")); return; }

        LogLine($"开始上传（车 → PC）：槽 {slot} …");
        var img = _link.ReadSlot(slot, (f, t) => Post(() => _lbState.Text = $"上传 {f * 100:F0}%  {t}"));
        string why = RouteLink.VerifyImage(img);
        if (why != null) throw new Exception($"从车里读回来的镜像校验不过：{why}（镜像 {img.Length} B）");

        Post(() =>
        {
            LoadImage(img);
            _shownSlot = slot;
            LogLine($"上传完成：{img.Length} B，{_recs.Count} 点，{_head.LapM:F2} m。表头 CRC 与记录区 CRC 都对上了。");
            _lbState.Text = _link.Describe;
        });
    }

    private void DoDownload()
    {
        int slot = SelectedSlot();
        if (slot < 0) { Post(() => LogLine("先在上面选一行（要写到哪一槽）。")); return; }
        if (slot == 0) { Post(() => LogLine("槽 0 是手推录制的家，下载不能写它 —— 选 1..6。")); return; }

        var img = File.ReadAllBytes(_dlgBin);
        string why = RouteLink.VerifyImage(img);
        if (why != null) throw new Exception($"这个文件不是合格的车端镜像：{why}");

        LogLine($"开始下载（PC → 车）：{Path.GetFileName(_dlgBin)} → 槽 {slot} …");
        var msg = _link.Download(slot, img, (f, t) => Post(() => _lbState.Text = $"下载 {f * 100:F0}%  {t}"));
        Post(() =>
        {
            LoadImage(img);
            _shownSlot = slot;
            LogLine(msg);
            _lbState.Text = _link.Describe;
            DoList();
        });
    }

    /// <summary>选文件必须在 UI 线程做（OpenFileDialog 是模态的），选完再扔给后台线程传。</summary>
    private void OnDownloadClick()
    {
        if (_busy) return;
        if (!_link.IsOpen) { LogLine("先打开串口。"); return; }
        int slot = SelectedSlot();
        if (slot < 0) { LogLine("先在上面选一行（要写到哪一槽）。"); return; }
        if (slot == 0) { LogLine("槽 0 是手推录制的家，下载不能写它 —— 选 1..6。"); return; }

        using var dlg = new OpenFileDialog
        {
            Title = $"选择要灌进槽 {slot} 的车端镜像（route_car.bin）",
            Filter = "车端路表镜像 (*.bin)|*.bin|所有文件 (*.*)|*.*",
        };
        if (dlg.ShowDialog(this) != DialogResult.OK) return;
        _dlgBin = dlg.FileName;
        RunAsync(DoDownload);
    }

    private string _dlgBin;

    private void DoSetActive()
    {
        int slot = SelectedSlot();
        if (slot < 0) { Post(() => LogLine("先选一行。")); return; }
        _link.SlotSelect(slot, persist: true, load: true);
        Post(() => { LogLine($"已把槽 {slot} 选为当前表（落盘 + 载入 RAM）。菜单 RtLoadF / Cruise 就是它。"); DoList(); });
    }

    private void DoLoadToCar()
    {
        int slot = SelectedSlot();
        if (slot < 0) { Post(() => LogLine("先选一行。")); return; }
        _link.SlotLoad(slot);
        Post(() => LogLine($"已把槽 {slot} 载入车端 RAM（不改「当前槽」的持久记录）。"));
    }

    private void LoadImage(byte[] img)
    {
        _image = img;
        _head = RouteProto.ReadHead(img, 0);
        int n = (int)_head.Count;
        _recs = new List<RouteProto.Rec>(n);
        for (int i = 0; i < n; i++) _recs.Add(RouteProto.ReadRec(img, 36 + i * 12));

        double cx = 0, cy = 0, maxR = 0;
        foreach (var r in _recs) { cx += r.X; cy += r.Y; }
        if (n > 0) { cx /= n; cy /= n; }
        foreach (var r in _recs) maxR = Math.Max(maxR, Math.Sqrt((r.X - cx) * (r.X - cx) + (r.Y - cy) * (r.Y - cy)));

        if (_head.IsStep)
        {
            // ★ 转向阶跃测试记录：12 B 的字段含义**全变了**（见 Protocol.cs 的 Head.IsStep），
            //   画轨迹没有意义 ⇒ 只报统计，图区给一句说明。分析在 PC 侧交给
            //   tools/steer_step_analysis.py。
            _lbDetail.Text =
                $"镜像 {img.Length} B（= 36 + {n}×12）\n" +
                $"★ 这是**转向阶跃测试**记录（内环辨识），不是路表\n" +
                $"记录 {n} 条    采样周期 {_head.DsMm} ms    总时长 {_head.LapMm / 1000.0:F1} s" +
                $"{((_head.Flags & 1) != 0 ? "    ⚠ 记满了（后面的没记上）" : "")}\n" +
                $"表头 CRC 0x{RouteLink.Crc16(0xFFFF, img, 0, 34):X4}（存 0x{((img[35] << 8) | img[34]):X4}）\n" +
                $"记录区 CRC 0x{RouteLink.Crc16(0xFFFF, img, 36, n * 12):X4}（存 0x{((img[33] << 8) | img[32]):X4}）\n" +
                $"字段：t_ms | cmd(舵量×1000) | duty | gz(°/s×100) | v(cm/s)\n" +
                $"⇒ 导出 CSV 存档；分析用 tools/steer_step_analysis.py 直接读下面的 .bin";

            _plot.SetData(new List<RouteProto.Rec>(),
                $"阶跃测试记录 · {n} 条 · {_head.DsMm} ms/条（不是轨迹，不画图）");
        }
        else if (_head.IsLog)
        {
            // ★ 巡航记录：语义与路表不同（见 Protocol.cs 的 Head.IsLog）
            _lbDetail.Text =
                $"镜像 {img.Length} B（= 36 + {n}×12）\n" +
                $"★ 这是**巡航记录**（Cruise 实际轨迹），不是路表\n" +
                $"点数 {n}    采样周期 {_head.DsMm} ms    总时长 {_head.LapMm / 1000.0:F1} s" +
                $"{((_head.Flags & 1) != 0 ? "    ⚠ 记满了（后面的没记上）" : "")}\n" +
                $"表头 CRC 0x{RouteLink.Crc16(0xFFFF, img, 0, 34):X4}（存 0x{((img[35] << 8) | img[34]):X4}）\n" +
                $"记录区 CRC 0x{RouteLink.Crc16(0xFFFF, img, 36, n * 12):X4}（存 0x{((img[33] << 8) | img[32]):X4}）\n" +
                $"第 4 个字段 = **舵机 PWM duty**（2420 中位 / 2160 右满 / 2660 左满）\n" +
                $"yaw 是**绝对航向**；等时间采样 ⇒ 车速可反推 v ≈ Δpos / {_head.DsMm} ms\n" +
                $"几何外接半径 {maxR:F3} m\n" +
                $"（先「导出 CSV」，再用 tools/cruise_log_analysis.py 看舵机抖动）";

            _plot.SetData(_recs, $"巡航记录 · {n} 点 · {_head.DsMm} ms/点 · {_head.LapMm / 1000.0:F1} s");
        }
        else
        {
            _lbDetail.Text =
                $"镜像 {img.Length} B（= 36 + {n}×12）\n" +
                $"点数 {n}    点距 {_head.DsMm} mm    圈长 {_head.LapM:F3} m\n" +
                $"表头 CRC 0x{RouteLink.Crc16(0xFFFF, img, 0, 34):X4}（存 0x{((img[35] << 8) | img[34]):X4}）\n" +
                $"记录区 CRC 0x{RouteLink.Crc16(0xFFFF, img, 36, n * 12):X4}（存 0x{((img[33] << 8) | img[32]):X4}）\n" +
                $"起点 ({_head.OriginX:F2}, {_head.OriginY:F2}) m   起点航向 {_head.OriginYawDeg:F2}°\n" +
                $"几何外接半径 {maxR:F3} m   圈长/周长比 {(maxR > 1e-6 ? _head.LapM / (2 * Math.PI * maxR) : 0):F3}\n" +
                $"（圈长/周长比 ≈ 1.00 ⇒ 这张表基本就是个圆；明显 ≠ 1 说明表里有直道）";

            _plot.SetData(_recs, $"槽内路表 · {n} 点 · {_head.LapM:F2} m");
        }

        _plot.AutoFit = true;
        _plot.Invalidate();
    }

    private void SaveBin()
    {
        if (_image == null) { LogLine("先上传或下载一张表。"); return; }
        using var dlg = new SaveFileDialog { Title = "另存车端镜像", FileName = "route_car.bin", Filter = "车端路表镜像 (*.bin)|*.bin" };
        if (dlg.ShowDialog(this) != DialogResult.OK) return;
        File.WriteAllBytes(dlg.FileName, _image);
        LogLine("已存 " + dlg.FileName);
    }

    /// <summary>
    /// 转向阶跃测试（内环辨识）—— Polaris 七步工序的第 ① 步，顺带能出第 ② 步的满舵半径。
    /// <para>流程：STEP_GO(0x0A) → 等 STEP_DONE(0x88)（车端跑完 + 记录写进 Flash）→ 读回槽 4。</para>
    /// <para>⚠ 车会真的动（舵机按 amp 打、电机按 rps 转）：先架空前轮，或留足直线跑道。</para>
    /// </summary>
    private void OnStepTest(double amp, int holdMs, double rps)
    {
        if (!_link.IsOpen) { LogLine("先打开串口。"); return; }
        if (!_link.CanStep)
        {
            LogLine("!! 车端不支持「转向阶跃测试」(0x0A) ⇒ 需重刷固件（两个核）。");
            LogLine("   " + RouteLink.StaleFirmwareHint);
            return;
        }

        int totalMs = RouteLink.StepBaseMs + 3 * holdMs;

        RunAsync(() =>
        {
            Post(() => LogLine($"⚠ 车会真的动：舵量 amp={amp:F2}，目标 {rps:F1} rps；"
                             + $"时序 300ms 基线 → +A → -A → 回中，每段 {holdMs} ms（共 {totalMs / 1000.0:F1} s）"));
            _link.StepGo(amp, holdMs, rps);

            int n = _link.WaitStepDone(totalMs + 20000);
            if (n < 0) throw new Exception("没等到 STEP_DONE（车端可能还在写盘，或链路掉了）");
            Post(() => LogLine($"测试跑完：记录 {n} 条，已写进槽 {RouteLink.StepSlot}"));

            var img = _link.ReadSlot(RouteLink.StepSlot,
                (f, t) => Post(() => _lbState.Text = $"读阶跃记录 {f * 100:F0}%  {t}"));
            Post(() =>
            {
                LoadImage(img);
                LogLine("已读回槽 " + RouteLink.StepSlot + "。要定量分析就点「另存 .bin」，");
                LogLine("然后：python tools/steer_step_analysis.py <那个 .bin> --png step.png");
                _lbState.Text = _link.Describe;
                DoList();
            });
        });
    }

    private void SaveCsv()
    {
        if (_recs.Count == 0) { LogLine("先上传或下载一张表。"); return; }
        using var dlg = new SaveFileDialog { Title = "导出 CSV", FileName = "route_dump.csv", Filter = "CSV (*.csv)|*.csv" };
        if (dlg.ShowDialog(this) != DialogResult.OK) return;

        // ★ 表头按 KartHost「读路表」存出来的那份**逐字对齐**，
        //   这样 tools/route_align_icp.py / route_vs_actual.py 直接就能吃。
        var sb = new System.Text.StringBuilder();
        if (_head.IsStep)
        {
            // ★ 阶跃测试：12 B 的字段全变了（t_ms / cmd / duty / gz / v），
            //   所以列名跟路表、巡航记录都不一样。分析交给 tools/steer_step_analysis.py
            //   （它读的是 .bin —— 用工具栏的「另存 .bin」存一份即可）。
            sb.AppendLine("# KartHost steer step test dump (from slot)");
            sb.AppendLine($"# magic=0x{0x4B525443:X8} ver=1 declared={_recs.Count} got={_recs.Count}");
            sb.AppendLine($"# step=1 sample_ms={_head.DsMm} total_ms={_head.LapMm} flags={_head.Flags}");
            sb.AppendLine("# 时序：300ms 基线(u=0) → +A → -A → 回中，每段各自等长");
            sb.AppendLine("# t_ms = 毫秒时刻；cmd = 已施加的归一化舵量×1000（正=左打）");
            sb.AppendLine("# duty: 2420 = 中位 ｜ 2160 = 右满舵 ｜ 2660 = 左满舵");
            sb.AppendLine("# gz_dps = 陀螺 z（度/秒）；v_mps = 车速（米/秒）");
            sb.AppendLine("idx,t_ms,cmd,duty,gz_dps,v_mps");
            for (int i = 0; i < _recs.Count; i++)
            {
                // ⚠ 车端那份 12 B 在这里是 i32|i16|i16|i16|i16，而 C# 的 Rec 是按
                //   i32|i32|i16|i16 读的（它只为路表服务）⇒ 必须**从原始字节手工拆**，
                //   不能直接用 Rec 的 X/Y 派生属性（它们会算出无意义的坐标）。
                int tms = _recs[i].Xmm;
                short cmd = (short)(_recs[i].Ymm & 0xFFFF);
                short dut = (short)((_recs[i].Ymm >> 16) & 0xFFFF);
                short gz100 = _recs[i].YawCdeg;
                short vcm = _recs[i].VCmps;
                sb.AppendLine($"{i},{tms},{cmd},{dut},{gz100 / 100.0:F2},{vcm / 100.0:F2}");
            }
        }
        else if (_head.IsLog)
        {
            // ★ 巡航记录：第 4 列是**舵机 duty**（不是车速），yaw 是**绝对航向**（不是相对起点）。
            //   采样是**等时间间隔**（sample_ms）⇒ 车速可以自己反推：v ≈ Δpos / Δt。
            sb.AppendLine("# KartHost cruise log dump (from slot)");
            sb.AppendLine($"# magic=0x{0x4B525443:X8} ver=1 declared={_recs.Count} got={_recs.Count}");
            sb.AppendLine($"# log=1 sample_ms={_head.DsMm} total_ms={_head.LapMm} flags={_head.Flags}");
            sb.AppendLine("# 坐标是【里程计世界系】：x、y 单位米");
            sb.AppendLine("# yaw_deg 是**绝对航向**（度）；servo_pwm 是车端算出的舵机指令 duty");
            sb.AppendLine("# duty: 2420 = 中位 ｜ 2160 = 右满舵 ｜ 2660 = 左满舵");
            sb.AppendLine("# 等时间采样 ⇒ 车速可反推：v ≈ Δpos / (sample_ms/1000)");
            sb.AppendLine("idx,x_m,y_m,yaw_deg,servo_pwm");
            for (int i = 0; i < _recs.Count; i++)
                sb.AppendLine($"{i},{_recs[i].X:F4},{_recs[i].Y:F4},{_recs[i].YawDeg:F2},{_recs[i].VCmps}");
        }
        else
        {
            // ★ 表头按 KartHost「读路表」存出来的那份**逐字对齐**，
            //   这样 tools/route_align_icp.py / route_vs_actual.py 直接就能吃。
            sb.AppendLine("# KartHost route table dump (from slot)");
            sb.AppendLine($"# magic=0x{0x4B525443:X8} ver=1 declared={_recs.Count} got={_recs.Count}");
            sb.AppendLine($"# ds_mm={_head.DsMm} lap_mm={_head.LapMm} flags={_head.Flags}");
            sb.AppendLine($"# origin_mm=({_head.OriginXmm},{_head.OriginYmm}) origin_yaw_cdeg={_head.OriginYawCdeg}");
            sb.AppendLine("# 坐标是【录制起点车体系】：x 向右、y 向前，单位米；yaw 是相对录制起点的度");
            sb.AppendLine("idx,x_m,y_m,yaw_deg");
            for (int i = 0; i < _recs.Count; i++)
                sb.AppendLine($"{i},{_recs[i].X:F4},{_recs[i].Y:F4},{_recs[i].YawDeg:F2}");
        }
        File.WriteAllText(dlg.FileName, sb.ToString(), new System.Text.UTF8Encoding(true));
        LogLine("已导出 " + dlg.FileName
                + (_head.IsStep ? "（阶跃测试：t_ms/cmd/duty/gz/v 五列）"
                                : (_head.IsLog ? "（巡航记录：第 5 列 = 舵机 duty）" : "")));
    }
}

/// <summary>路表的俯视图像：坐标轴刻度 + 中线折线 + 起点/终点标记 + 每 N 点朝向箭头。</summary>
public sealed class RoutePlot : Control
{
    private List<RouteProto.Rec> _recs = new();
    private string _title = "（还没有表）";
    public bool AutoFit = true;

    private float _sc = 1f, _ox, _oy;
    private PointF _dragFrom;
    private bool _dragging;

    public RoutePlot()
    {
        DoubleBuffered = true;
        BackColor = Theme.Canvas;
        ForeColor = Theme.Text;
        SetStyle(ControlStyles.AllPaintingInWmPaint | ControlStyles.UserPaint | ControlStyles.OptimizedDoubleBuffer, true);
    }

    public void SetData(List<RouteProto.Rec> recs, string title)
    {
        _recs = recs ?? new List<RouteProto.Rec>();
        _title = title;
        AutoFit = true;
    }

    protected override void OnMouseEnter(EventArgs e) { Focus(); base.OnMouseEnter(e); }

    protected override void OnMouseWheel(MouseEventArgs e)
    {
        float k = e.Delta > 0 ? 1.15f : 1f / 1.15f;
        _sc *= k;
        _ox = e.X - (e.X - _ox) * k;
        _oy = e.Y - (e.Y - _oy) * k;
        Invalidate();
    }

    protected override void OnMouseDown(MouseEventArgs e) { _dragging = true; _dragFrom = e.Location; }
    protected override void OnMouseUp(MouseEventArgs e) { _dragging = false; }
    protected override void OnMouseMove(MouseEventArgs e)
    {
        if (!_dragging) return;
        _ox += e.X - _dragFrom.X;
        _oy += e.Y - _dragFrom.Y;
        _dragFrom = e.Location;
        Invalidate();
    }

    private void Fit()
    {
        if (_recs.Count == 0) return;
        float x0 = float.MaxValue, x1 = float.MinValue, y0 = float.MaxValue, y1 = float.MinValue;
        foreach (var r in _recs)
        {
            x0 = Math.Min(x0, r.X); x1 = Math.Max(x1, r.X);
            y0 = Math.Min(y0, r.Y); y1 = Math.Max(y1, r.Y);
        }
        float w = Math.Max(x1 - x0, 1e-3f), h = Math.Max(y1 - y0, 1e-3f);
        float pad = Dpi.Pxf(36);
        float sc = Math.Min((Width - pad * 2) / w, (Height - pad * 2) / h);
        _sc = sc;
        // 车体系：x 向右、y 向前 ⇒ 屏幕 y 要**翻过来**（不然图是上下镜像的）
        _ox = Width * 0.5f - (x0 + x1) * 0.5f * sc;
        _oy = Height * 0.5f + (y0 + y1) * 0.5f * sc;
        AutoFit = false;
    }

    private PointF P(float x, float y) => new(_ox + x * _sc, _oy - y * _sc);

    protected override void OnPaint(PaintEventArgs e)
    {
        var g = e.Graphics;
        g.SmoothingMode = SmoothingMode.AntiAlias;
        g.Clear(Theme.Canvas);

        using (var tb = new SolidBrush(Theme.Text))
        using (var f = new Font("Microsoft YaHei UI", 9f))
            g.DrawString(_title, f, tb, Dpi.Pxf(10), Dpi.Pxf(8));

        if (_recs.Count == 0)
        {
            using var fb = new SolidBrush(Theme.TextDim);
            using var f2 = new Font("Microsoft YaHei UI", 10f);
            g.DrawString("没有数据 —— 先「上传」把车里的表读回来，或「下载」灌一张进去。", f2, fb, Dpi.Pxf(10), Dpi.Pxf(34));
            return;
        }

        if (AutoFit) Fit();

        // 网格（每 0.1 m）
        using (var pen = new Pen(Theme.GridDash, 1f) { DashStyle = DashStyle.Dot })
        {
            float step = 0.1f;
            while (step * _sc < 26f) step *= 2f;
            for (float x = -100f; x <= 100f; x += step)
            {
                var a = P(x, -100f); var b = P(x, 100f);
                if (a.X >= 0 && a.X <= Width) g.DrawLine(pen, a, b);
            }
            for (float y = -100f; y <= 100f; y += step)
            {
                var a = P(-100f, y); var b = P(100f, y);
                if (a.Y >= 0 && a.Y <= Height) g.DrawLine(pen, a, b);
            }
        }

        // 过原点的两条轴（= 车的起点位姿：+y 前 / +x 右）
        using (var pen = new Pen(Theme.Axis, 1.4f))
        {
            var o = P(0, 0);
            g.DrawLine(pen, P(-100f, 0), P(100f, 0));
            g.DrawLine(pen, P(0, -100f), P(0, 100f));
            using var fb = new SolidBrush(Theme.TextDim);
            using var f = new Font("Microsoft YaHei UI", 8f);
            g.DrawString("+y 前", f, fb, o.X + Dpi.Pxf(4), o.Y - Dpi.Pxf(16));
            g.DrawString("+x 右", f, fb, o.X + Dpi.Pxf(4), o.Y + Dpi.Pxf(3));
        }

        // 中线折线
        using (var pen = new Pen(Theme.Accent, Dpi.Pxf(1.6f)) { LineJoin = LineJoin.Round })
        {
            var pts = new PointF[_recs.Count];
            for (int i = 0; i < _recs.Count; i++) pts[i] = P(_recs[i].X, _recs[i].Y);
            if (pts.Length >= 2) g.DrawLines(pen, pts);
        }

        // 每 10 点的朝向箭头（看走行方向）
        using (var pen = new Pen(Theme.CurveOf(3), Dpi.Pxf(1.2f)))
        {
            for (int i = 0; i < _recs.Count; i += 10)
            {
                var r = _recs[i];
                double a = r.YawDeg * Math.PI / 180.0;      // 固件系：yaw=0 朝 +y
                float dx = (float)(-Math.Sin(a)) * Dpi.Pxf(9);
                float dy = (float)(-Math.Cos(a)) * Dpi.Pxf(9);   // 屏幕 y 向下 ⇒ 取负
                var p0 = P(r.X, r.Y);
                g.DrawLine(pen, p0.X, p0.Y, p0.X + dx, p0.Y + dy);
            }
        }

        // 起 / 终
        using (var bStart = new SolidBrush(Theme.Ok))
        using (var bEnd = new SolidBrush(Theme.Err))
        {
            var p0 = P(_recs[0].X, _recs[0].Y);
            g.FillEllipse(bStart, p0.X - Dpi.Pxf(4), p0.Y - Dpi.Pxf(4), Dpi.Pxf(8), Dpi.Pxf(8));
            var p1 = P(_recs[_recs.Count - 1].X, _recs[_recs.Count - 1].Y);
            g.FillEllipse(bEnd, p1.X - Dpi.Pxf(4), p1.Y - Dpi.Pxf(4), Dpi.Pxf(8), Dpi.Pxf(8));
            using var fb = new SolidBrush(Theme.TextDim);
            using var f = new Font("Microsoft YaHei UI", 8f);
            g.DrawString("起点", f, fb, p0.X + Dpi.Pxf(5), p0.Y + Dpi.Pxf(2));
            g.DrawString("终点", f, fb, p1.X + Dpi.Pxf(5), p1.Y + Dpi.Pxf(2));
        }

        // 比例尺
        {
            float bar = 0.5f;                       // 0.5 m
            float px = bar * _sc;
            var y = Height - Dpi.Pxf(16);
            using var pen = new Pen(Theme.TextDim, Dpi.Pxf(1.4f));
            g.DrawLine(pen, Dpi.Pxf(12), y, Dpi.Pxf(12) + px, y);
            g.DrawLine(pen, Dpi.Pxf(12), y - Dpi.Pxf(4), Dpi.Pxf(12), y + Dpi.Pxf(4));
            g.DrawLine(pen, Dpi.Pxf(12) + px, y - Dpi.Pxf(4), Dpi.Pxf(12) + px, y + Dpi.Pxf(4));
            using var fb = new SolidBrush(Theme.TextDim);
            using var f = new Font("Microsoft YaHei UI", 8f);
            g.DrawString("0.5 m", f, fb, Dpi.Pxf(12) + px / 2 - Dpi.Pxf(14), y - Dpi.Pxf(16));
        }
    }
}
