namespace KartHost.Ui;

/// <summary>
/// 参数面板：从参数表**自动生成**滑条 + 数字框（不是硬编码 50 个控件）。
///
/// 两个必须做对的地方（参考工程都做错了）：
///   1) 双向同步 —— 拖滑条要更新数字框，数字框打字也要回写滑条；用一个 _syncing
///      标志防止两者互相触发成死循环（Qt-Serial-PID-Host 只做了单向）；
///   2) 30 ms 防抖 —— 拖动过程中不发帧，只发最后一次。不加防抖一次拖动能刷出
///      几百帧，把车端解析缓冲冲爆。
/// </summary>
public sealed class ParamPanel : Panel
{
    public event Action<ParamMeta, float> ParamChanged;   // 防抖后触发

    private readonly List<ParamMeta> _list = new();
    private readonly List<Row> _rows = new();
    private readonly System.Windows.Forms.Timer _debounce = new() { Interval = 30 };
    private ParamMeta _pending;
    private float _pendingValue;
    private bool _syncing;
    private bool _readOnly;

    private sealed class Row
    {
        public ParamMeta P;
        public TrackBar Bar;
        public NumericUpDown Num;
        public Label Name;
        public int Y;
    }

    // ★ 这些是 96 DPI 下的逻辑值，实际像素由 Dpi 层换算（不能写成 const，运行期才知道缩放）
    private static int RowH => Dpi.Px(46);
    private static int NumW => Dpi.Px(68);
    private static int Pad  => Dpi.Px(10);

    public ParamPanel()
    {
        AutoScroll = true;
        BackColor = Theme.Panel;
        _debounce.Tick += (_, __) => Flush();
    }

    public IReadOnlyList<ParamMeta> Params => _list;

    /// <summary>未连接时禁止改参数，避免"改了没反应"的困惑</summary>
    public bool ReadOnly
    {
        get => _readOnly;
        set
        {
            _readOnly = value;
            foreach (var r in _rows) { r.Bar.Enabled = !value; r.Num.Enabled = !value; }
        }
    }

    public int DirtyCount
    {
        get { int n = 0; foreach (var p in _list) if (p.Dirty) n++; return n; }
    }

    public void ClearDirty() { foreach (var p in _list) p.Dirty = false; }

    // ------------------------------------------------------------------
    public void SetParams(List<ParamMeta> list)
    {
        _list.Clear();
        if (list != null) _list.AddRange(list);
        Rebuild();
    }

    /// <summary>连接后用车端下发的元数据覆盖/追加（这就是"上位机自动长出界面"）</summary>
    public void MergeTable(List<ParamMeta> incoming)
    {
        if (incoming == null || incoming.Count == 0) return;
        foreach (var inc in incoming)
        {
            var cur = _list.Find(x => x.Id == inc.Id);
            if (cur == null)
            {
                inc.Value = inc.Def;
                _list.Add(inc);
            }
            else
            {
                cur.Name = inc.Name; cur.Label = inc.Label; cur.Group = inc.Group;
                cur.Lo = inc.Lo; cur.Hi = inc.Hi; cur.Def = inc.Def;
                cur.Step = inc.Step; cur.Unit = inc.Unit;
            }
        }
        _list.Sort((a, b) => a.Id.CompareTo(b.Id));
        Rebuild();
    }

    // ------------------------------------------------------------------
    private void Rebuild()
    {
        SuspendLayout();
        Controls.Clear();
        _rows.Clear();

        int y = 0;
        string lastGroup = null;

        foreach (var p in _list)
        {
            string grp = string.IsNullOrEmpty(p.Group) ? "其他" : p.Group;
            if (grp != lastGroup)
            {
                lastGroup = grp;
                Controls.Add(new Label
                {
                    Text = "▎" + grp,
                    Font = new Font("微软雅黑", 9.5f, FontStyle.Bold),
                    ForeColor = Theme.Accent,
                    BackColor = Theme.Panel,
                    AutoSize = false,
                    Height = Dpi.Px(24),
                    TextAlign = ContentAlignment.MiddleLeft,
                    Location = new Point(Pad - 4, y),
                });
                // ⚠ 必须是 Dpi.Px：标题高度是 Dpi.Px(24)，写死 26（物理px）会让
                //   1.25x 下的标题(30px) 压到下一行行首 —— 布局体检的"重叠"就是这么抓到的
                y += Dpi.Px(28);
            }

            var row = BuildRow(p, y);
            _rows.Add(row);
            y += RowH;
        }

        // 分组标题也要跟着宽度走（统一处理）
        int hdrIdx = 0;
        foreach (Control c in Controls)
            if (c is Label l && l.Text.StartsWith("▎")) { l.Width = ClientSize.Width - Pad; hdrIdx++; }

        _contentHeight = y + Dpi.Px(10);
        ResumeLayout();
        ApplyLayout();
    }

    private int _contentHeight;

    private Row BuildRow(ParamMeta p, int y)
    {
        var row = new Row { P = p, Y = y };

        row.Name = new Label
        {
            Text = string.IsNullOrEmpty(p.Unit) ? p.Label : p.Label + "  [" + p.Unit + "]",
            ForeColor = Theme.Text,
            BackColor = Theme.Panel,
            AutoSize = false,
            Height = Dpi.Px(18),
            TextAlign = ContentAlignment.MiddleLeft,
        };

        int dec = p.Step >= 1f ? 0 : p.Step >= 0.1f ? 1 : p.Step >= 0.01f ? 2 : 3;
        decimal lo = (decimal)Math.Min(p.Lo, p.Hi);
        decimal hi = (decimal)Math.Max(p.Lo, p.Hi);
        decimal inc = (decimal)Math.Max(p.Step, (float)Math.Pow(10, -dec));

        row.Num = new NumericUpDown
        {
            DecimalPlaces = dec,
            Increment = inc,
            Minimum = lo,
            Maximum = hi,
            Value = ClampDec((decimal)p.Value, lo, hi),
            BackColor = Theme.PanelAlt,
            ForeColor = Theme.Text,
            BorderStyle = BorderStyle.FixedSingle,
            TextAlign = HorizontalAlignment.Right,
        };
        row.Num.ValueChanged += (_, __) =>
        {
            if (_syncing) return;
            _syncing = true;
            float v = (float)row.Num.Value;
            row.Bar.Value = p.ValueToSlider(v);
            _syncing = false;
            Queue(p, v);
        };

        row.Bar = new TrackBar
        {
            Minimum = 0,
            Maximum = 1000,
            TickStyle = TickStyle.None,
            SmallChange = 1,
            LargeChange = 20,
            Value = p.ValueToSlider(p.Value),
            BackColor = Theme.Panel,
            // ★ 原生 TrackBar 在非 100% DPI 下会无视 Height 自高到 ~69 物理像素，
            //   把下一行的 Label 整个涂掉（先加的控件后画，在上层）。三条一起上才压得住：
            AutoSize = false,                                       // ① 必须写在 Height 之前
            Height = Dpi.Px(26),
        };
        row.Bar.AutoSize = false;                                   // ② 建好之后再显式关一次
        row.Bar.MaximumSize = new Size(Dpi.Px(4000), Dpi.Px(26));    // ③ 用 MaximumSize 锁死高度
        row.Bar.Height = Dpi.Px(26);                                 //    ★ Width 千万别写 0！
        row.Bar.ValueChanged += (_, __) =>
        {
            if (_syncing) return;
            _syncing = true;
            int s = row.Bar.Value;
            float v = p.Snap(p.SliderToValue(s));
            row.Num.Value = ClampDec((decimal)v, lo, hi);
            _syncing = false;
            Queue(p, v);
        };

        Controls.Add(row.Name);
        Controls.Add(row.Num);
        Controls.Add(row.Bar);
        return row;
    }

    private static decimal ClampDec(decimal v, decimal lo, decimal hi)
        => v < lo ? lo : v > hi ? hi : v;

    protected override void OnClientSizeChanged(EventArgs e)
    {
        base.OnClientSizeChanged(e);
        ApplyLayout();
    }

    private void ApplyLayout()
    {
        int cw = ClientSize.Width - Pad * 2;
        if (cw < 80) return;

        SuspendLayout();

        // ★ 分组标题的宽度只在重建时算过一次，面板变窄后必须跟着重算 ——
        //   否则它按旧的（更宽的）ClientSize 撑着，Right 会越出面板（高 DPI 下更明显）
        foreach (Control c in Controls)
        {
            if (c is Label l && l.Text.StartsWith("▎"))
            {
                l.Width = Math.Max(Dpi.Px(60), ClientSize.Width - Pad);
            }
        }

        foreach (var r in _rows)
        {
            r.Name.Location = new Point(Pad, r.Y);
            r.Name.Width = cw - NumW - Dpi.Px(6);

            r.Num.Location = new Point(Pad + cw - NumW, r.Y);
            r.Num.Width = NumW;
            r.Num.Height = Dpi.Px(20);

            r.Bar.Location = new Point(Pad - Dpi.Px(6), r.Y + Dpi.Px(21));
            r.Bar.Width = cw + 10;
            r.Bar.Height = Dpi.Px(26);
        }
        ResumeLayout();
    }

    // ------------------------------------------------------------------
    private void Queue(ParamMeta p, float v)
    {
        if (_readOnly) return;
        p.Value = v;
        p.Dirty = true;
        _pending = p;
        _pendingValue = v;
        _debounce.Stop();      // 重置计时：拖动过程中不发
        _debounce.Start();
    }

    private void Flush()
    {
        _debounce.Stop();
        var p = _pending;
        _pending = null;
        if (p != null) ParamChanged?.Invoke(p, _pendingValue);
    }

    /// <summary>车端回读 → 同步 UI（不再触发下发）</summary>
    public void UpdateFromCar(byte id, float v)
    {
        var row = _rows.Find(r => r.P.Id == id);
        if (row == null) return;
        var p = row.P;
        p.Value = v;
        p.Dirty = false;

        _syncing = true;
        row.Bar.Value = p.ValueToSlider(v);
        row.Num.Value = ClampDec((decimal)v, row.Num.Minimum, row.Num.Maximum);
        _syncing = false;
    }

    /// <summary>本地恢复默认值（不发送；要生效需再点「保存到 Flash」或逐项下发）</summary>
    public void RestoreDefaults()
    {
        foreach (var p in _list)
        {
            p.Value = p.Def;
            UpdateFromCar(p.Id, p.Def);
        }
    }

    /// <summary>把刚改过的参数逐个重发（"写回"按钮用）</summary>
    public List<ParamMeta> DirtyList()
    {
        var r = new List<ParamMeta>();
        foreach (var p in _list) if (p.Dirty) r.Add(p);
        return r;
    }

    protected override void Dispose(bool disposing)
    {
        if (disposing) _debounce.Dispose();
        base.Dispose(disposing);
    }
}
