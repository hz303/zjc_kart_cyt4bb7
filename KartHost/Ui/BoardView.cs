using System.Drawing.Drawing2D;

namespace KartHost.Ui;

/// <summary>
/// 实时二维轨迹 / 地图控件（移植自 `A_zjc_project/3.1imu_trails_mode_pc` 的 `Ui/BoardView.cs`）。
///
/// ---- 坐标约定（★ 与车端 `ins_odom.c` 完全一致，移植时不需要任何旋转/镜像）----------
///   车端：`dx = -sin(yaw)·ds`、`dy = +cos(yaw)·ds`  ⇒ yaw=0 沿 +y 前进，yaw=+90° 朝 -x
///   这里：+x（东）= 屏幕右，+y（北）= 屏幕上，**y 轴翻转**
///   ⇒ 车头方向 = `(-sin yaw, -cos yaw)`；yaw = 0/90/180/270 分别朝 上/左/下/右
///   车体用 `TranslateTransform(位置) + RotateTransform(-yaw)`，变换后局部"上"= 车头。
///
/// ---- 数据来源与"为什么要有本地轨迹缓冲" -----------------------------------------
///   直接读 <see cref="ChannelStore"/> 的 26/27/28（x_m / y_m / yaw_deg，见 `Defaults.InsRoute*`）。
///   <see cref="Tick"/> 由 30 FPS 的 UI 定时器调用：**增量**拉取比上次更新的样本，追加进本地缓冲。
///   不直接拿 Store 快照当轨迹，有两个硬理由：
///     ① 「清空轨迹」只能清轨迹、不能连波形一起清（`Store.Reset()` 是所有通道共用的）；
///     ② 需要 2cm 抽稀 —— 否则静止时点会一直涨、AutoFit 会被噪声撑大、画面抖动。
///   ⚠ 本控件**只读** Store，不从接收线程收到任何回调（与 WavePanel 同一套纪律）。
/// </summary>
public sealed class BoardView : Control
{
    // ---- 通道号（= 车端 motor_link.h 第四节；InsRoute 档案） ----
    public int ChX = 26, ChY = 27, ChYaw = 28, ChDist = 29, ChFlags = 30;

    /// <summary>数据仓库（外部注入，本控件不拥有）</summary>
    public ChannelStore Store { get; set; }

    /// <summary>本档案是否有位姿通道。非「惯导+路表」档案置 false：只提示，不画（否则会把别的档案的 26/27 当坐标画成乱线）。</summary>
    public bool PoseAvailable { get; set; } = true;

    // ---- 轨迹点 ----
    private struct Pt { public float X, Y, T; }          // X/Y = 车端原始米；T = 相对轨迹起点的秒
    private readonly List<Pt> _trail = new();
    private const int MaxPts = 100000;                   // 50Hz × 2cm 抽稀，够很长的赛道
    private readonly float[] _sx = new float[ChannelStore.Capacity];
    private readonly float[] _sy = new float[ChannelStore.Capacity];
    private readonly double[] _st = new double[ChannelStore.Capacity];
    private double _lastT = double.NegativeInfinity;      // 已消费到的 Store 时间戳
    private double _t0;                                   // 轨迹起点时间（时间着色用）
    private float _tSpan = 1f;                            // 轨迹总时长（秒），重算在 Tick

    // ---- 最新位姿与运动量（画 HUD / 速度矢量 / 旋转弧用） ----
    private bool _hasLatest;
    private float _lx, _ly, _lyaw;                        // 当前位置与航向（原始米 / 度）
    private float _vMps;                                  // 地速（由相邻两点算）
    private float _wDps;                                  // 航向角速度（度/秒）
    private double _prevT = double.NegativeInfinity;
    private float _prevX, _prevY, _prevYaw;

    // ---- 视图 ----
    private float _pxPerM = 80f;                          // 像素/米
    private float _cx, _cy;                               // 画布中心对应的世界坐标（米，已含基准）
    private float _refX, _refY;                           // 显示基准：显示坐标 = 原始 - 基准
    private bool _autoFit = true;

    private bool _drag;
    private Point _dragPt;

    private const int GridTargetPx = 56;                  // 希望每格约 56 物理像素（面板不高时也能有像样的网格）
    private static readonly float[] Steps = { 0.02f, 0.05f, 0.1f, 0.2f, 0.5f, 1f, 2f, 5f, 10f, 20f, 50f };

    public bool AutoFit
    {
        get => _autoFit;
        set { _autoFit = value; Invalidate(); }
    }

    public int PointCount => _trail.Count;

    public BoardView()
    {
        SetStyle(ControlStyles.AllPaintingInWmPaint | ControlStyles.OptimizedDoubleBuffer
               | ControlStyles.UserPaint | ControlStyles.ResizeRedraw | ControlStyles.Selectable, true);
        BackColor = Theme.Canvas;
        TabStop = true;
        Cursor = Cursors.Cross;
    }

    // ==================================================================
    // 数据：由 UI 定时器驱动的增量拉取
    // ==================================================================

    /// <summary>由外部 30 FPS 定时器调用</summary>
    public void Tick()
    {
        PullNew();
        Invalidate();
    }

    private void PullNew()
    {
        var s = Store;
        if (s == null || !PoseAvailable) return;

        int n = s.Snapshot(ChX, _sx, _st, _sx.Length);
        if (n <= 0) return;
        int m = s.Snapshot(ChY, _sy, _st, _sy.Length);
        if (m < n) n = m;
        if (n <= 0) return;

        _hasLatest = true;

        for (int i = 0; i < n; i++)
        {
            double t = _st[i];
            if (!(t > _lastT)) continue;                   // 只消费新样本（_lastT 初值 -∞，避免 NaN 比较坑）
            _lastT = t;

            float x = _sx[i], y = _sy[i];
            if (float.IsNaN(x) || float.IsNaN(y)) continue;

            if (_trail.Count > 0)
            {
                var last = _trail[^1];
                float dx = x - last.X, dy = y - last.Y;
                if (dx * dx + dy * dy < 0.0004f) continue;  // < 2cm 不记（0.02²）
            }
            else
            {
                _t0 = t;
            }

            if (_trail.Count >= MaxPts) _trail.RemoveRange(0, MaxPts / 4);
            _trail.Add(new Pt { X = x, Y = y, T = (float)(t - _t0) });
        }

        // 最新位姿 + 运动量（HUD / 速度矢量 / 旋转弧）
        if (_trail.Count > 0)
        {
            var last = _trail[^1];
            _lx = last.X; _ly = last.Y; _tSpan = Math.Max(last.T, 1f);
        }
        float yaw = s.Latest(ChYaw);
        if (!float.IsNaN(yaw)) _lyaw = yaw;

        double nowT = s.LatestTime();
        if (_prevT > double.NegativeInfinity && nowT > _prevT)
        {
            double dt = nowT - _prevT;
            if (dt > 1e-3)
            {
                float gx = _lx - _prevX, gy = _ly - _prevY;
                _vMps = (float)(Math.Sqrt(gx * gx + gy * gy) / dt);
                float dyaw = _lyaw - _prevYaw;
                while (dyaw > 180f) dyaw -= 360f;
                while (dyaw < -180f) dyaw += 360f;
                _wDps = (float)(dyaw / dt);
            }
        }
        _prevT = nowT; _prevX = _lx; _prevY = _ly; _prevYaw = _lyaw;
    }

    /// <summary>清空轨迹（不影响波形）。新数据到来后会自动从当前点重新开始记。</summary>
    public void ClearTrail()
    {
        _trail.Clear();
        _t0 = 0;
        _tSpan = 1f;
        _lastT = double.NegativeInfinity;
        _hasLatest = false;                  // 车体/HUD 也一起收起来，避免停在旧位置上骗人
        _prevT = double.NegativeInfinity;
        _vMps = 0f;
        _wDps = 0f;
        _autoFit = true;
        Invalidate();
    }

    /// <summary>把"当前位置"当成显示原点（轨迹与车体会整体平移，形状不变）。</summary>
    public void ZeroHere()
    {
        _refX = _lx;
        _refY = _ly;
        _autoFit = true;
        Invalidate();
    }

    /// <summary>显示基准回到车端原始坐标（车端 `ins_zero` 之后的等价视图）。</summary>
    public void ResetReference()
    {
        _refX = 0f; _refY = 0f;
        _autoFit = true;
        Invalidate();
    }

    private PointF Disp(float rawX, float rawY) => new(rawX - _refX, rawY - _refY);

    /// <summary>
    /// 航向(度) → 世界系单位方向矢量。**这是全控件唯一的"约定点"**，与车端 `ins_odom.c` 一致：
    ///   `dx = -sin(yaw)·ds`、`dy = +cos(yaw)·ds` ⇒  yaw=0 朝北(+y)、yaw=+90° 朝西(-x)、yaw=180° 朝南、
    ///   yaw=270° 朝东。因为 yaw 的定义是"从正北起、俯视逆时针为正"。
    /// ★ 单独抽成函数是为了**可被自检数值验证**：这个符号一旦有人改反，轨迹会整体镜像，
    ///   而画面上"看起来也挺像一条轨迹"，只有拿基准数才能抓出来。
    /// </summary>
    public static (float dx, float dy) Heading(float yawDeg)
    {
        float r = yawDeg * MathF.PI / 180f;
        return (-MathF.Sin(r), MathF.Cos(r));
    }

    // ---- 路表（车端 dump 上来的整张表；与实时轨迹叠画，用于判"跟没跟上"）----
    private struct RtPt { public float X, Y, Yaw; }
    private readonly List<RtPt> _route = new();
    private bool _routeClosed;

    /// <summary>路表点数（0 = 没读表）。</summary>
    public int RouteCount => _route.Count;

    private PointF WorldToScreen(float wx, float wy)
        => new(Width / 2f + (wx - _cx) * _pxPerM, Height / 2f - (wy - _cy) * _pxPerM);

    private PointF ScreenToWorld(float sx, float sy)
        => new(_cx + (sx - Width / 2f) / _pxPerM, _cy - (sy - Height / 2f) / _pxPerM);

    // ---- 路表接口（KartHost 收到车端 dump 后调用）----
    /// <summary>
    /// 设置路表。点是**录制起点车体系**的米（x 右 / y 前），与实时轨迹同一套约定，
    /// 所以**直接叠画、零旋转**（这正是两份数据都用"相对量"的好处）。
    /// <paramref name="alignToTrail"/> = true 时把表的第 0 点平移到本地轨迹的第 0 点
    ///   —— 两份数据的原点分别是"录制起点"和"轨迹起点"，实际是同一个物理位置，
    ///   但上电/归零时机不同会有固定偏差。对齐后**形状与横向误差**的比较才成立。
    /// </summary>
    public void SetRoute(IReadOnlyList<(float x, float y, float yaw)> pts, bool closed = false,
                         bool alignToTrail = true)
    {
        _route.Clear();
        RouteOffset = (0f, 0f);

        if (pts != null && pts.Count > 0)
        {
            if (alignToTrail && _trail.Count > 0)
            {
                RouteOffset = (_trail[0].X - pts[0].x, _trail[0].Y - pts[0].y);
            }

            foreach (var p in pts)
                _route.Add(new RtPt { X = p.x + RouteOffset.x, Y = p.y + RouteOffset.y, Yaw = p.yaw });
        }

        _routeClosed = closed;
        AutoFit = true;                 // 表一进来就重新取景，否则可能整条表都在画布外
        Invalidate();
    }

    /// <summary>上一次 SetRoute 施加的平移量（米）—— 写日志/自检用</summary>
    public (float x, float y) RouteOffset { get; private set; }

    public void ClearRoute()
    {
        _route.Clear();
        _routeClosed = false;
        Invalidate();
    }

    /// <summary>自检用：路表点的显示坐标包围盒</summary>
    internal (float minX, float minY, float maxX, float maxY) RouteBoundsForTest()
    {
        float mnx = float.MaxValue, mny = float.MaxValue, mxx = float.MinValue, mxy = float.MinValue;
        foreach (var p in _route)
        {
            float x = p.X - _refX, y = p.Y - _refY;
            if (x < mnx) mnx = x;
            if (x > mxx) mxx = x;
            if (y < mny) mny = y;
            if (y > mxy) mxy = y;
        }
        return (mnx, mny, mxx, mxy);
    }

    // ---- 自检入口（internal：同程序集可见，不对外暴露为 API）----
    internal void SetViewForTest(float cx, float cy, float pxPerM)
    { _cx = cx; _cy = cy; _pxPerM = pxPerM; _autoFit = false; }

    /// <summary>强制跑一次 AutoFit 并回报结果（自检用来把"缩放到多大/居中到哪"钉成数字）。</summary>
    internal (float pxPerM, float cx, float cy) AutoFitForTest()
    {
        _autoFit = true;
        DoAutoFit();
        return (_pxPerM, _cx, _cy);
    }

    internal float PxPerMForTest => _pxPerM;

    /// <summary>轨迹的显示坐标包围盒（自检用：把"AutoFit 看的是什么范围"也变成可断言的数）。</summary>
    internal (float minX, float minY, float maxX, float maxY) TrailBoundsForTest()
    {
        float mnx = float.MaxValue, mny = float.MaxValue, mxx = float.MinValue, mxy = float.MinValue;
        foreach (var p in _trail)
        {
            float x = p.X - _refX, y = p.Y - _refY;
            if (x < mnx) mnx = x;
            if (x > mxx) mxx = x;
            if (y < mny) mny = y;
            if (y > mxy) mxy = y;
        }
        return (mnx, mny, mxx, mxy);
    }

    internal PointF ToScreenForTest(float wx, float wy) => WorldToScreen(wx, wy);

    // ==================================================================
    // 绘制
    // ==================================================================

    protected override void OnPaint(PaintEventArgs e)
    {
        var g = e.Graphics;
        g.SmoothingMode = SmoothingMode.AntiAlias;
        g.TextRenderingHint = System.Drawing.Text.TextRenderingHint.ClearTypeGridFit;
        g.Clear(Theme.Canvas);

        if (!PoseAvailable)
        {
            DrawCentered(g, "当前档案没有位姿通道 —— 把档案切到「惯导+路表」再看轨迹");
            return;
        }
        if (_trail.Count == 0 && !_hasLatest && _route.Count == 0)
        {
            DrawCentered(g, "等待位姿…   连接车端，或点「模拟车端」拖速度滑条");
            return;
        }

        if (_autoFit) DoAutoFit();

        DrawGrid(g);
        DrawRoute(g);
        DrawTrail(g);
        DrawBoard(g);
        DrawCompass(g);
        DrawLegend(g);
        DrawHud(g);
    }

    private void DrawCentered(Graphics g, string msg)
    {
        using var f = Theme.F(11f);
        using var b = new SolidBrush(Theme.TextDim);
        var sz = g.MeasureString(msg, f);
        g.DrawString(msg, f, b, (Width - sz.Width) / 2f, (Height - sz.Height) / 2f);
    }

    /// <summary>自动适应：包围盒中心作视图中心，pxPerM 取能装下的最大值（四周留 55 逻辑像素内边距）。</summary>
    private void DoAutoFit()
    {
        if (_trail.Count == 0 && _route.Count == 0)
        {
            _cx = _refX; _cy = _refY;
            _pxPerM = 80f;
            return;
        }

        float minX = float.MaxValue, minY = float.MaxValue, maxX = float.MinValue, maxY = float.MinValue;
        int n = _trail.Count;
        int stride = Math.Max(1, n / 4000);
        for (int i = 0; i < n; i += stride) Accum(_trail[i].X, _trail[i].Y);
        if (n > 0) Accum(_trail[^1].X, _trail[^1].Y);

        // ★ 路表也要计入取景范围：否则"表比轨迹大"时，表会有一半在画布外 ——
        //   而"表和轨迹的差"正是这块面板要看的东西。
        foreach (var rp in _route) Accum(rp.X, rp.Y);

        void Accum(float rx, float ry)
        {
            float x = rx - _refX, y = ry - _refY;
            if (x < minX) minX = x;
            if (x > maxX) maxX = x;
            if (y < minY) minY = y;
            if (y > maxY) maxY = y;
        }

        float w = Math.Max(maxX - minX, 0.40f);            // 太小会抖（除零 + 噪声放大）
        float h = Math.Max(maxY - minY, 0.40f);
        _cx = (minX + maxX) * 0.5f;
        _cy = (minY + maxY) * 0.5f;

        float availW = Math.Max(Width - Dpi.Pxf(110), 60f);
        float availH = Math.Max(Height - Dpi.Pxf(110), 60f);
        _pxPerM = Math.Clamp(Math.Min(availW / w, availH / h), 1f, 20000f);
    }

    private void DrawGrid(Graphics g)
    {
        float target = GridTargetPx / _pxPerM;
        float step = Steps[^1];
        foreach (float s in Steps) if (s >= target) { step = s; break; }

        // ⚠ 用 GridMajor 而不是 Grid：Canvas(16,19,24) 上 (46,50,58) 几乎看不见，
        //   而网格看不见 = 无法判断"轨迹有多大"，这块面板就白画了。
        using var pen = new Pen(Theme.GridMajor, 1f);
        using var penAxis = new Pen(Theme.Axis, 1.2f);

        var bl = ScreenToWorld(0, Height);
        var tr = ScreenToWorld(Width, 0);

        for (float x = MathF.Floor(bl.X / step) * step; x <= tr.X; x += step)
        {
            float sx = WorldToScreen(x, 0).X;
            g.DrawLine(Math.Abs(x) < step * 0.01f ? penAxis : pen, sx, 0, sx, Height);
        }
        for (float y = MathF.Floor(bl.Y / step) * step; y <= tr.Y; y += step)
        {
            float sy = WorldToScreen(0, y).Y;
            g.DrawLine(Math.Abs(y) < step * 0.01f ? penAxis : pen, 0, sy, Width, sy);
        }

        // 右下角比例尺（米/厘米随格距自适应）
        int barPx = (int)(step * _pxPerM);
        if (barPx >= Dpi.Px(24) && barPx <= Width / 2)
        {
            int bx = Width - barPx - Dpi.Px(24);
            int by = Height - Dpi.Px(26);
            using var bp = new Pen(Theme.TextDim, 2f);
            g.DrawLine(bp, bx, by, bx + barPx, by);
            g.DrawLine(bp, bx, by - Dpi.Px(5), bx, by + Dpi.Px(5));
            g.DrawLine(bp, bx + barPx, by - Dpi.Px(5), bx + barPx, by + Dpi.Px(5));
            using var f = Theme.F(8.5f);
            using var b = new SolidBrush(Theme.TextDim);
            string lbl = step >= 1f ? $"{step:0.#} m" : $"{step * 100:0} cm";
            var sz = g.MeasureString(lbl, f);
            g.DrawString(lbl, f, b, bx + barPx / 2f - sz.Width / 2f, by - sz.Height - Dpi.Px(2));
        }
    }

    /// <summary>
    /// 画路表（车端 dump 上来的整张表）。
    /// ★ 与实时轨迹**同一套坐标约定**（x 右 / y 前），所以直接叠画、零旋转 ——
    ///   两条线之间的横向差就是"跟踪误差"，这是这块面板最有用的一个读数。
    /// 画法：整条线一条 Pen（不做时间着色，避免和轨迹混淆），末端画一个小方块表示"表的终点"。
    /// </summary>
    private void DrawRoute(Graphics g)
    {
        int n = _route.Count;
        if (n < 2) return;

        using var pen = new Pen(Theme.Route, 1.6f) { DashStyle = DashStyle.Dash };
        using var penC = new Pen(Theme.Route, 1.6f);

        var p0 = WorldToScreen(Disp(_route[0].X, _route[0].Y).X, Disp(_route[0].X, _route[0].Y).Y);
        var prev = p0;
        int stride = Math.Max(1, n / 3000);

        for (int i = stride; i < n; i += stride)
        {
            var d = Disp(_route[i].X, _route[i].Y);
            var p1 = WorldToScreen(d.X, d.Y);
            g.DrawLine(pen, prev, p1);
            prev = p1;
        }

        // 尾巴必须连到最后一个点（stride 会跳过去）
        {
            var d = Disp(_route[^1].X, _route[^1].Y);
            g.DrawLine(pen, prev, WorldToScreen(d.X, d.Y));
        }

        if (_routeClosed)
        {
            var d = Disp(_route[^1].X, _route[^1].Y);
            var dl = WorldToScreen(d.X, d.Y);
            var df = Disp(_route[0].X, _route[0].Y);
            g.DrawLine(pen, dl, WorldToScreen(df.X, df.Y));
        }

        // 起点（空心方块）/ 终点（实心方块）
        using var bf = new SolidBrush(Theme.Route);
        using var b0 = new SolidBrush(Theme.Canvas);
        int s = Dpi.Px(6);
        g.FillRectangle(b0, p0.X - s / 2f, p0.Y - s / 2f, s, s);
        g.DrawRectangle(penC, p0.X - s / 2f, p0.Y - s / 2f, s, s);

        var dEnd = Disp(_route[^1].X, _route[^1].Y);
        var pEnd = WorldToScreen(dEnd.X, dEnd.Y);
        g.FillRectangle(bf, pEnd.X - s / 2f, pEnd.Y - s / 2f, s, s);
    }

    /// <summary>按时间着色逐段画轨迹；分段数封顶，避免几万个 GDI 对象。</summary>
    private void DrawTrail(Graphics g)
    {
        int n = _trail.Count;
        if (n < 2) return;

        int stride = Math.Max(1, n / 2500);
        using var pen = new Pen(Theme.Accent, 1.8f) { LineJoin = LineJoin.Round };

        var d0 = Disp(_trail[0].X, _trail[0].Y);
        var start = WorldToScreen(d0.X, d0.Y);
        var last = start;

        for (int i = stride; i < n; i += stride)
        {
            var d = Disp(_trail[i].X, _trail[i].Y);
            var p1 = WorldToScreen(d.X, d.Y);
            pen.Color = Theme.TimeColor(_trail[i].T / _tSpan);
            g.DrawLine(pen, last, p1);
            last = p1;
        }
        // 尾巴一定要连到最新点（stride 会跳过去）
        var dl = Disp(_trail[^1].X, _trail[^1].Y);
        var pl = WorldToScreen(dl.X, dl.Y);
        pen.Color = Theme.TimeColor(1f);
        g.DrawLine(pen, last, pl);

        // 起点标记（"从哪儿出发"）
        using var sb = new SolidBrush(Theme.Ok);
        g.FillEllipse(sb, start.X - Dpi.Px(4), start.Y - Dpi.Px(4), Dpi.Px(8), Dpi.Px(8));
        using var se = new Pen(Theme.Canvas, 1.5f);
        g.DrawEllipse(se, start.X - Dpi.Px(4), start.Y - Dpi.Px(4), Dpi.Px(8), Dpi.Px(8));
    }

    /// <summary>车体：圆角矩形 + 车头三角 + 指向箭头 + 旋转方向弧 + 速度矢量。</summary>
    private void DrawBoard(Graphics g)
    {
        var d = Disp(_lx, _ly);
        var p = WorldToScreen(d.X, d.Y);

        float L = Math.Clamp(0.16f * _pxPerM, Dpi.Pxf(26), Dpi.Pxf(150));   // 车长（前后）
        float W = L * 0.52f;                                                // 车宽（左右）

        // 速度矢量（画在世界系，不随车体旋转）：从车心出发，长度 = v * 0.25s * pxPerM
        if (_vMps > 0.02f)
        {
            var (hx, hy) = Heading(_lyaw);                    // 与车端同一套约定，别再手写 sin/cos
            float vx = hx * _vMps * 0.25f;
            float vy = hy * _vMps * 0.25f;
            var q = WorldToScreen(d.X + vx, d.Y + vy);
            using var vp = new Pen(Theme.Vel, 2f);
            vp.CustomEndCap = new AdjustableArrowCap(3f, 3f, true);
            g.DrawLine(vp, p.X, p.Y, q.X, q.Y);
        }

        var st = g.Save();
        g.TranslateTransform(p.X, p.Y);
        g.RotateTransform(-_lyaw);                           // 与约定一致：局部"上" = 车头

        // 旋转方向弧（在车体坐标系里画在车身外圈）
        DrawRotationArc(g, _wDps, L * 0.78f);

        using (var path = Theme.RoundRect(
                   new Rectangle((int)(-W / 2f), (int)(-L / 2f), (int)W, (int)L),
                   Math.Max(3, (int)(W * 0.16f))))
        {
            using var fill = new SolidBrush(Theme.BoardFill);
            using var edge = new Pen(Theme.Board, 2f);
            g.FillPath(fill, path);
            g.DrawPath(edge, path);
        }

        using (var nose = new SolidBrush(Theme.Nose))
        {
            g.FillPolygon(nose, new[]
            {
                new PointF(0, -L / 2f - L * 0.16f),
                new PointF(-W * 0.34f, -L / 2f + L * 0.06f),
                new PointF(W * 0.34f, -L / 2f + L * 0.06f),
            });
        }

        using (var arow = new Pen(Theme.Board, Math.Max(1.6f, L * 0.05f)))
        {
            arow.CustomEndCap = new AdjustableArrowCap(3f, 3f, true);
            g.DrawLine(arow, 0, L * 0.24f, 0, -L * 0.12f);
        }

        g.Restore(st);
    }

    /// <summary>绕车体的一小段带箭头的弧：方向与真实旋转方向一致（左转绿 / 右转橙）。</summary>
    private static void DrawRotationArc(Graphics g, float wDps, float r)
    {
        if (Math.Abs(wDps) < 2f) return;                     // 太小说明是噪声，不画

        float a0 = wDps > 0 ? 26f : -26f;
        float sweep = wDps > 0 ? -52f : 52f;                 // 负 = 视觉逆时针
        var rect = new RectangleF(-r * 0.62f, -r * 0.62f, r * 1.24f, r * 1.24f);
        float alpha = Math.Clamp(Math.Abs(wDps) / 60f, 0.35f, 1f);
        var c = wDps > 0 ? Theme.Ccw : Theme.Cw;
        using var pen = new Pen(Color.FromArgb((int)(255 * alpha), c), Math.Max(2f, r * 0.10f));
        g.DrawArc(pen, rect, a0, sweep);

        float ae = a0 + sweep;
        float rad = rect.Width / 2f;
        float ex = rect.X + rect.Width / 2f + rad * MathF.Cos(ae * MathF.PI / 180f);
        float ey = rect.Y + rect.Height / 2f + rad * MathF.Sin(ae * MathF.PI / 180f);
        float tx = MathF.Sin(ae * MathF.PI / 180f) * (sweep < 0 ? 1f : -1f);
        float ty = -MathF.Cos(ae * MathF.PI / 180f) * (sweep < 0 ? 1f : -1f);
        float hl = Math.Max(5f, r * 0.22f);
        using var br = new SolidBrush(Color.FromArgb((int)(255 * alpha), c));
        g.FillPolygon(br, new[]
        {
            new PointF(ex + tx * hl, ey + ty * hl),
            new PointF(ex - ty * hl * 0.45f, ey + tx * hl * 0.45f),
            new PointF(ex + ty * hl * 0.45f, ey - tx * hl * 0.45f),
        });
    }

    /// <summary>右上角罗盘：指针指向当前航向（0~360°，从正北起逆时针为正）。</summary>
    private void DrawCompass(Graphics g)
    {
        int R = Dpi.Px(46);
        int cx = Width - R - Dpi.Px(26);
        int cy = R + Dpi.Px(20);

        using (var bg = new SolidBrush(Color.FromArgb(180, 16, 21, 28)))
        using (var edge = new Pen(Theme.Border, 1.4f))
        {
            g.FillEllipse(bg, cx - R, cy - R, R * 2, R * 2);
            g.DrawEllipse(edge, cx - R, cy - R, R * 2, R * 2);
        }

        using (var fp = new Pen(Theme.GridDash, 1f) { DashStyle = DashStyle.Dot })
        {
            g.DrawLine(fp, cx - R + Dpi.Px(4), cy, cx + R - Dpi.Px(4), cy);
            g.DrawLine(fp, cx, cy - R + Dpi.Px(4), cx, cy + R - Dpi.Px(4));
        }

        using (var f = Theme.F(8f, FontStyle.Bold))
        using (var b = new SolidBrush(Theme.TextDim))
        {
            g.DrawString("N", f, b, cx - Dpi.Px(5), cy - R + Dpi.Px(3));
            g.DrawString("S", f, b, cx - Dpi.Px(5), cy + R - Dpi.Px(16));
            g.DrawString("E", f, b, cx + R - Dpi.Px(15), cy - Dpi.Px(7));
            g.DrawString("W", f, b, cx - R + Dpi.Px(4), cy - Dpi.Px(7));
        }

        if (_hasLatest)
        {
            float yaw360 = _lyaw % 360f;
            if (yaw360 < 0) yaw360 += 360f;

            // yaw 从北起、逆时针为正 → 屏幕角（顺时针为正）= -yaw，且"上"是 -90°
            float a = (-yaw360 - 90f) * MathF.PI / 180f;
            float len = R - Dpi.Pxf(12);
            var tip = new PointF(cx + len * MathF.Cos(a), cy + len * MathF.Sin(a));

            using var np = new Pen(Theme.Nose, 2.6f);
            np.CustomEndCap = new AdjustableArrowCap(3f, 3f, true);
            g.DrawLine(np, cx, cy, tip.X, tip.Y);

            using var dot = new SolidBrush(Theme.Board);
            g.FillEllipse(dot, cx - 2.5f, cy - 2.5f, 5f, 5f);

            using var f2 = Theme.FM(10f, FontStyle.Bold);
            using var b2 = new SolidBrush(Theme.Text);
            string t = $"{yaw360:0.0}°";
            var sz = g.MeasureString(t, f2);
            g.DrawString(t, f2, b2, cx - sz.Width / 2f, cy + R + Dpi.Px(4));
        }
    }

    /// <summary>左下角图例：时间着色渐变条 + 左/右转颜色。</summary>
    private void DrawLegend(Graphics g)
    {
        int x = Dpi.Px(16);
        int y = Height - Dpi.Px(74);

        using var f = Theme.F(8.5f);
        using var b = new SolidBrush(Theme.TextDim);

        // 时间渐变条（早 -> 晚），逐段上色，20 段足够
        int barW = Dpi.Px(150), barH = Dpi.Px(7);
        for (int i = 0; i < 20; i++)
        {
            using var sb = new SolidBrush(Theme.TimeColor(i / 19f));
            g.FillRectangle(sb, x + barW * i / 20, y, barW / 20 + 1, barH);
        }
        g.DrawString("轨迹起点", f, b, x, y - Dpi.Px(13));
        var szE = g.MeasureString("轨迹末端", f);
        g.DrawString("轨迹末端", f, b, x + barW - szE.Width, y - Dpi.Px(13));

        int yy = y + Dpi.Px(15);
        using (var pc = new Pen(Theme.Ccw, 3f)) g.DrawLine(pc, x, yy + Dpi.Px(6), x + Dpi.Px(18), yy + Dpi.Px(6));
        g.DrawString("左转（逆时针）", f, b, x + Dpi.Px(24), yy);
        yy += Dpi.Px(17);
        using (var pw2 = new Pen(Theme.Cw, 3f)) g.DrawLine(pw2, x, yy + Dpi.Px(6), x + Dpi.Px(18), yy + Dpi.Px(6));
        g.DrawString("右转（顺时针）", f, b, x + Dpi.Px(24), yy);
    }

    /// <summary>左上角 HUD：位姿/里程/速度/状态位（状态位与车端 main_cm7_0.c 的打包逐位对应）。</summary>
    private void DrawHud(Graphics g)
    {
        if (!_hasLatest) return;

        int flags = (int)(Store?.Latest(ChFlags) ?? float.NaN);
        if (float.IsNaN(Store?.Latest(ChFlags) ?? float.NaN)) flags = 0;

        float dist = Store?.Latest(ChDist) ?? float.NaN;

        float yaw360 = _lyaw % 360f;
        if (yaw360 < 0) yaw360 += 360f;

        var lines = new List<string>
        {
            $"位姿  x {_lx,7:0.00} m    y {_ly,7:0.00} m",
            $"航向  {yaw360,6:0.0}°   ω {_wDps,7:0.0} °/s",
        };
        if (!float.IsNaN(dist)) lines.Add($"里程  {dist,7:0.00} m   点 {_trail.Count}");
        lines.Add($"速度  {_vMps,7:0.00} m/s   " + FlagText(flags));

        using var f = Theme.FM(9f);
        using var b = new SolidBrush(Theme.Text);
        using var bg = new SolidBrush(Color.FromArgb(170, 16, 21, 28));

        float wMax = 0, hSum = 0;
        foreach (var s in lines) { var sz = g.MeasureString(s, f); if (sz.Width > wMax) wMax = sz.Width; hSum += sz.Height + 1; }

        int px = Dpi.Px(10), py = Dpi.Px(10);
        g.FillRectangle(bg, px - Dpi.Px(6), py - Dpi.Px(4), wMax + Dpi.Px(12), hSum + Dpi.Px(8));
        float yy = py;
        foreach (var s in lines)
        {
            g.DrawString(s, f, b, px, yy);
            yy += g.MeasureString(s, f).Height + 1;
        }
    }

    private static string FlagText(int f)
    {
        var sb = new System.Text.StringBuilder();
        if ((f & 0x01) != 0) sb.Append("就绪 ");
        if ((f & 0x02) != 0) sb.Append("磁标定 ");
        if ((f & 0x04) != 0) sb.Append("静止 ");
        if ((f & 0x08) != 0) sb.Append("打滑 ");
        if ((f & 0x10) != 0) sb.Append("录路表 ");
        if ((f & 0x20) != 0) sb.Append("路表满 ");
        if ((f & 0x40) != 0) sb.Append("IMU断 ");
        if ((f & 0x80) != 0) sb.Append("告警 ");
        return sb.Length == 0 ? "—" : sb.ToString().TrimEnd();
    }

    // ==================================================================
    // 交互：滚轮缩放（以光标为锚）/ 拖动平移 / 双击复位 / F 自适应
    // ==================================================================

    protected override void OnMouseWheel(MouseEventArgs e)
    {
        base.OnMouseWheel(e);
        if (!PoseAvailable) return;

        var w0 = ScreenToWorld(e.X, e.Y);
        float k = e.Delta > 0 ? 1.15f : (1f / 1.15f);
        _pxPerM = Math.Clamp(_pxPerM * k, 1f, 20000f);
        var w1 = ScreenToWorld(e.X, e.Y);
        _cx += w0.X - w1.X;
        _cy += w0.Y - w1.Y;
        _autoFit = false;
        Invalidate();
    }

    protected override void OnMouseDown(MouseEventArgs e)
    {
        base.OnMouseDown(e);
        if (e.Button == MouseButtons.Left) { _drag = true; _dragPt = e.Location; Focus(); }
    }

    protected override void OnMouseMove(MouseEventArgs e)
    {
        base.OnMouseMove(e);
        if (!_drag) return;
        float dx = e.X - _dragPt.X, dy = e.Y - _dragPt.Y;
        _dragPt = e.Location;
        _cx -= dx / _pxPerM;
        _cy += dy / _pxPerM;                 // 屏幕 y 向下、世界 y 向上
        _autoFit = false;
        Invalidate();
    }

    protected override void OnMouseUp(MouseEventArgs e)
    {
        base.OnMouseUp(e);
        _drag = false;
    }

    protected override void OnMouseDoubleClick(MouseEventArgs e)
    {
        base.OnMouseDoubleClick(e);
        _autoFit = true; _pxPerM = 80f;
        Invalidate();
    }

    protected override bool IsInputKey(Keys keyData) => true;

    protected override void OnKeyDown(KeyEventArgs e)
    {
        base.OnKeyDown(e);
        switch (e.KeyCode)
        {
            case Keys.F: _autoFit = true; Invalidate(); break;
            case Keys.Add or Keys.Oemplus:
                _pxPerM = Math.Clamp(_pxPerM * 1.2f, 1f, 20000f); _autoFit = false; Invalidate(); break;
            case Keys.Subtract or Keys.OemMinus:
                _pxPerM = Math.Clamp(_pxPerM / 1.2f, 1f, 20000f); _autoFit = false; Invalidate(); break;
        }
    }
}
