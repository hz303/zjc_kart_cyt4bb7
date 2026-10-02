using System.Drawing.Drawing2D;

namespace KartHost.Ui;

/// <summary>
/// 多通道波形控件（GDI+ 自绘，零第三方依赖）。
///
/// 三条性能纪律（参考工程全都没做，所以它们跑不了 40 通道）：
///   1) 重绘由外部固定 30 FPS 触发，与接收完全解耦 —— 本控件只负责画；
///   2) 点数多于像素宽时做 min/max 降采样 —— 保留尖峰包络，绘制量成倍下降；
///   3) 隐藏的通道直接跳过，不参与绘制也不参与自动量程。
/// </summary>
public sealed class WavePanel : Control
{
    public ChannelStore Store { get; set; }

    /// <summary>本图要显示的通道号（由外部勾选控制）</summary>
    public readonly HashSet<int> Shown = new();

    public double TimeWindow { get; set; } = 15.0;
    public bool AutoScale { get; set; } = true;
    public bool Paused { get; set; }
    public string Title { get; set; } = "";
    public double YMin { get; set; } = -100;
    public double YMax { get; set; } = 100;

    private readonly float[] _v = new float[ChannelStore.Capacity];
    private readonly double[] _t = new double[ChannelStore.Capacity];
    private readonly PointF[] _pts = new PointF[ChannelStore.Capacity + 64];

    private double _frozenRight;
    private double _yLo, _yHi;

    private bool _hover;
    private int _mouseX;

    private const int PadLeft = 54, PadRight = 12, PadTop = 24, PadBottom = 18;

    public WavePanel()
    {
        SetStyle(ControlStyles.AllPaintingInWmPaint | ControlStyles.OptimizedDoubleBuffer
               | ControlStyles.UserPaint | ControlStyles.ResizeRedraw, true);
        BackColor = Theme.Bg;
    }

    /// <summary>由外部 30 FPS 定时器调用</summary>
    public void Tick()
    {
        if (!Paused && Store != null && Store.Count > 0) _frozenRight = Store.LatestTime();
        Invalidate();
    }

    protected override void OnPaint(PaintEventArgs e)
    {
        var g = e.Graphics;
        g.SmoothingMode = SmoothingMode.None;      // 折线用 None 更快，肉眼几乎无差
        g.Clear(Theme.Bg);

        int pw = Width - PadLeft - PadRight, ph = Height - PadTop - PadBottom;
        if (pw < 20 || ph < 20 || Store == null || Store.Count == 0 || Shown.Count == 0)
        {
            Placeholder(g);
            return;
        }

        double tRight = Paused ? _frozenRight : Store.LatestTime();
        double tLeft = tRight - TimeWindow;
        if (TimeWindow <= 0) TimeWindow = 1;

        AutoRange(tLeft);

        double ySpan = _yHi - _yLo;
        if (Math.Abs(ySpan) < 1e-9) ySpan = 1;

        DrawGrid(g, pw, ph, tLeft, tRight, ySpan);
        DrawCurves(g, pw, ph, tLeft, tRight, ySpan);

        using (var pb = new Pen(Theme.Border, 1))
            g.DrawRectangle(pb, PadLeft, PadTop, pw, ph);

        DrawTitle(g);
        DrawLegend(g);
        DrawCursor(g, tLeft, tRight, pw, ph);
    }

    // ------------------------------------------------------------------
    private void AutoRange(double tLeft)
    {
        if (!AutoScale) { _yLo = YMin; _yHi = YMax; return; }

        double lo = double.MaxValue, hi = double.MinValue;
        foreach (int ch in Shown)
        {
            int n = Store.Snapshot(ch, _v, _t, ChannelStore.Capacity);
            for (int k = 0; k < n; k++)
            {
                if (_t[k] < tLeft) continue;
                float v = _v[k];
                if (float.IsNaN(v) || float.IsInfinity(v)) continue;
                if (v < lo) lo = v;
                if (v > hi) hi = v;
            }
        }
        if (lo > hi) { lo = -1; hi = 1; }
        if (Math.Abs(hi - lo) < 1e-6) { hi += 1; lo -= 1; }
        double pad = (hi - lo) * 0.10;
        _yLo = lo - pad;
        _yHi = hi + pad;
    }

    private void DrawGrid(Graphics g, int pw, int ph, double tLeft, double tRight, double ySpan)
    {
        using var penMinor = new Pen(Theme.Grid, 1);
        using var penMajor = new Pen(Theme.GridMajor, 1);
        using var fAxis = new Font("Consolas", 7.5f);
        using var bAxis = new SolidBrush(Theme.TextDim);

        // 时间轴
        double tStep = NiceStep((tRight - tLeft) / 6.0);
        double t0 = Math.Ceiling(tLeft / tStep) * tStep;
        for (double tt = t0; tt <= tRight; tt += tStep)
        {
            float x = PadLeft + (float)((tt - tLeft) / (tRight - tLeft) * pw);
            g.DrawLine(penMinor, x, PadTop, x, PadTop + ph);
            string s = tt.ToString("0.#");
            var sz = g.MeasureString(s, fAxis);
            g.DrawString(s, fAxis, bAxis, x - sz.Width / 2, PadTop + ph + 2);
        }

        // 值轴
        double vStep = NiceStep(ySpan / 5.0);
        double v0 = Math.Ceiling(_yLo / vStep) * vStep;
        for (double vv = v0; vv <= _yHi + vStep * 0.01; vv += vStep)
        {
            float y = PadTop + ph - (float)((vv - _yLo) / ySpan * ph);
            g.DrawLine(penMinor, PadLeft, y, PadLeft + pw, y);
            string s = Math.Abs(vv) >= 10000 ? vv.ToString("0")
                     : Math.Abs(vv) >= 100 ? vv.ToString("0")
                     : vv.ToString("0.##");
            var sz = g.MeasureString(s, fAxis);
            g.DrawString(s, fAxis, bAxis, PadLeft - 5 - sz.Width, y - sz.Height / 2);
        }
    }

    private void DrawCurves(Graphics g, int pw, int ph, double tLeft, double tRight, double ySpan)
    {
        double tSpan = tRight - tLeft;
        if (tSpan <= 0) return;

        // 目标每像素 2 个点 → 超过就 min/max 降采样
        int targetMax = pw * 2;
        int stride = Math.Max(1, Store.Count / Math.Max(1, targetMax));

        float yScale = (float)(ph / ySpan);

        foreach (int ch in Shown)
        {
            int n = Store.Snapshot(ch, _v, _t, ChannelStore.Capacity);
            if (n < 2) continue;

            int np = BuildPoints(n, tLeft, tSpan, pw, ph, yScale, stride);
            if (np < 2) continue;

            using var pen = new Pen(Theme.CurveOf(ch), 1.4f) { LineJoin = LineJoin.Round };
            g.DrawLines(pen, _pts.AsSpan(0, np));          // ★ 零分配重载
        }
    }

    /// <summary>构造屏幕点序列。stride&gt;1 时对每段取 min/max 两点，保留包络。</summary>
    private int BuildPoints(int n, double tLeft, double tSpan, int pw, int ph, float yScale, int stride)
    {
        int np = 0;
        int i = 0;
        while (i < n && _t[i] < tLeft) i++;
        if (i >= n) i = n - 1;
        if (i > 0) i--;                                   // 回退一点，保证曲线贴住左边界

        int cap = _pts.Length - 1;

        while (i < n && np < cap)
        {
            int end = Math.Min(i + stride, n);

            float vMin = float.MaxValue, vMax = float.MinValue;
            int iMin = i, iMax = i;
            bool any = false;
            for (int k = i; k < end; k++)
            {
                float v = _v[k];
                if (float.IsNaN(v) || float.IsInfinity(v)) continue;
                any = true;
                if (v < vMin) { vMin = v; iMin = k; }
                if (v > vMax) { vMax = v; iMax = k; }
            }

            if (any)
            {
                if (stride == 1 || iMin == iMax)
                {
                    _pts[np++] = ToPoint(_t[iMin], vMin, tLeft, tSpan, pw, ph, yScale);
                }
                else if (iMin < iMax)
                {
                    _pts[np++] = ToPoint(_t[iMin], vMin, tLeft, tSpan, pw, ph, yScale);
                    _pts[np++] = ToPoint(_t[iMax], vMax, tLeft, tSpan, pw, ph, yScale);
                }
                else
                {
                    _pts[np++] = ToPoint(_t[iMax], vMax, tLeft, tSpan, pw, ph, yScale);
                    _pts[np++] = ToPoint(_t[iMin], vMin, tLeft, tSpan, pw, ph, yScale);
                }
            }
            i = end;
        }
        return np;
    }

    private PointF ToPoint(double t, float v, double tLeft, double tSpan,
                           int pw, int ph, float yScale)
    {
        float x = PadLeft + (float)((t - tLeft) / tSpan * pw);
        float y = PadTop + ph - (v - (float)_yLo) * yScale;
        // 允许略微越界，钳到画布外一点，避免长竖线
        if (y < PadTop - 5000) y = PadTop - 5000;
        if (y > PadTop + ph + 5000) y = PadTop + ph + 5000;
        return new PointF(x, y);
    }

    private void Placeholder(Graphics g)
    {
        using var f = new Font("微软雅黑", 10f);
        using var b = new SolidBrush(Theme.TextDim);
        string msg = Store == null || Store.Count == 0
            ? "等待数据…（点「模拟车端」可立刻看到波形）"
            : "本图未勾选任何通道（右侧勾选框里选几个）";
        var sz = g.MeasureString(msg, f);
        g.DrawString(msg, f, b, (Width - sz.Width) / 2, (Height - sz.Height) / 2);
    }

    private void DrawTitle(Graphics g)
    {
        if (string.IsNullOrEmpty(Title)) return;
        using var f = new Font("微软雅黑", 9f, FontStyle.Bold);
        using var b = new SolidBrush(Theme.Text);
        g.DrawString(Title, f, b, PadLeft, 5);
    }

    private void DrawLegend(Graphics g)
    {
        using var f = new Font("Consolas", 8f);
        using var b = new SolidBrush(Theme.TextDim);
        int y = PadTop + 4;
        int rightX = PadLeft + (Width - PadLeft - PadRight) - 8;
        foreach (int ch in Ordered)
        {
            float val = Store.Latest(ch);
            string vs = float.IsNaN(val) ? "  --  " : val.ToString("0.##");
            string txt = $"{Defaults.ChannelName(ch)}  {vs}{Defaults.ChannelUnit(ch)}";
            var sz = g.MeasureString(txt, f);
            float x = rightX - sz.Width;
            using (var pen = new Pen(Theme.CurveOf(ch), 2.5f))
                g.DrawLine(pen, x - 14, y + sz.Height / 2, x - 3, y + sz.Height / 2);
            g.DrawString(txt, f, b, x, y);
            y += (int)sz.Height + 1;
            if (y > Height - 24) break;
        }
    }

    private readonly List<int> _ordered = new();
    /// <summary>稳定顺序的可见通道（避免 HashSet 枚举顺序抖动导致图例跳来跳去）</summary>
    private IReadOnlyList<int> Ordered
    {
        get
        {
            _ordered.Clear();
            foreach (int c in Shown) _ordered.Add(c);
            _ordered.Sort();
            return _ordered;
        }
    }

    private void DrawCursor(Graphics g, double tLeft, double tRight, int pw, int ph)
    {
        if (!_hover || _mouseX < PadLeft || _mouseX > PadLeft + pw) return;
        using (var pen = new Pen(Color.FromArgb(130, Theme.Accent), 1) { DashStyle = DashStyle.Dash })
            g.DrawLine(pen, _mouseX, PadTop, _mouseX, PadTop + ph);

        using var f = new Font("Consolas", 8f);
        using var b = new SolidBrush(Theme.Text);
        double tAt = tLeft + (_mouseX - PadLeft) / (double)pw * (tRight - tLeft);
        g.DrawString($"{tAt - tRight:0.00} s", f, b, _mouseX + 4, PadTop + ph - 14);
    }

    protected override void OnMouseEnter(EventArgs e) { _hover = true; Invalidate(); base.OnMouseEnter(e); }
    protected override void OnMouseLeave(EventArgs e) { _hover = false; Invalidate(); base.OnMouseLeave(e); }
    protected override void OnMouseMove(MouseEventArgs e)
    {
        _hover = true; _mouseX = e.X; Invalidate(); base.OnMouseMove(e);
    }

    /// <summary>把任意步长吸附到 1 / 2 / 5 × 10^n（刻度才会是整数）</summary>
    private static double NiceStep(double raw)
    {
        if (raw <= 0 || double.IsNaN(raw) || double.IsInfinity(raw)) return 1;
        double e = Math.Pow(10, Math.Floor(Math.Log10(raw)));
        double m = raw / e;
        double f = m <= 1 ? 1 : m <= 2 ? 2 : m <= 5 ? 5 : 10;
        return f * e;
    }
}
