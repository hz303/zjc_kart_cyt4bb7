using System.Drawing.Drawing2D;

namespace KartHost.Ui;

/// <summary>
/// 自绘指示灯。用于「舵机使能 / 电机使能 / 整体使能 / 链路状态」。
/// 画法借鉴 qcw_indicatorlamp：外层线性渐变做立体边框，内层径向渐变做灯芯。
/// </summary>
public sealed class LampControl : Control
{
    public enum LampState { Off, On, Fault }

    private LampState _state = LampState.Off;
    private bool _blinkPhase;
    private readonly System.Windows.Forms.Timer _blink = new() { Interval = 400 };

    /// <summary>Fault 时是否闪烁</summary>
    public bool Blink { get; set; } = true;

    /// <summary>灯旁边的文字（画在控件内，左灯右字）</summary>
    public string Caption { get; set; } = "";

    public LampState State
    {
        get => _state;
        set
        {
            if (_state == value) return;
            _state = value;
            _blink.Enabled = Blink && value == LampState.Fault;
            Invalidate();
        }
    }

    public LampControl()
    {
        // ★ 必须带 SupportsTransparentBackColor，否则 BackColor = Transparent 会抛
        //   ArgumentException("控件不支持透明的背景色")。指示灯要贴在深色面板上，需要透明底。
        SetStyle(ControlStyles.AllPaintingInWmPaint | ControlStyles.OptimizedDoubleBuffer
               | ControlStyles.UserPaint | ControlStyles.ResizeRedraw
               | ControlStyles.SupportsTransparentBackColor, true);
        BackColor = Color.Transparent;
        Size = Dpi.Sz(120, 34);
        _blink.Tick += (_, __) => { _blinkPhase = !_blinkPhase; Invalidate(); };
    }

    protected override void OnPaint(PaintEventArgs e)
    {
        var g = e.Graphics;
        g.SmoothingMode = SmoothingMode.AntiAlias;

        int d = Math.Min(Height - 4, 26);
        float cx = 3 + d / 2f, cy = Height / 2f;
        float r = d / 2f;

        bool dim = _state == LampState.Fault && Blink && _blinkPhase;
        Color core = _state switch
        {
            LampState.On => dim ? Color.FromArgb(60, 110, 70) : Color.FromArgb(60, 230, 130),
            LampState.Fault => dim ? Color.FromArgb(110, 45, 45) : Color.FromArgb(240, 80, 80),
            _ => Color.FromArgb(42, 70, 50),
        };

        // 外层边框（立体感）
        using (var lg = new LinearGradientBrush(
            new PointF(cx, cy - r), new PointF(cx, cy + r),
            Color.FromArgb(120, 126, 138), Color.FromArgb(40, 44, 50)))
        {
            g.FillEllipse(lg, cx - r, cy - r, d, d);
        }

        // 灯芯（径向渐变）
        float ir = r - 3f;
        if (ir > 1)
        {
            using var rg = new GraphicsPath();
            rg.AddEllipse(cx - ir, cy - ir, ir * 2, ir * 2);
            using var pgb = new PathGradientBrush(rg)
            {
                CenterColor = Color.FromArgb(255, core),
                SurroundColors = new[] { Color.FromArgb(255, Darken(core, 0.45f)) },
            };
            g.FillEllipse(pgb, cx - ir, cy - ir, ir * 2, ir * 2);
        }

        // 高光
        using (var hl = new SolidBrush(Color.FromArgb(_state == LampState.Off ? 18 : 60, 255, 255, 255)))
        {
            g.FillEllipse(hl, cx - ir * 0.55f, cy - ir * 0.72f, ir * 0.62f, ir * 0.5f);
        }

        // 文字
        if (!string.IsNullOrEmpty(Caption))
        {
            using var f = new Font("微软雅黑", 9.5f, FontStyle.Regular);
            using var b = new SolidBrush(_state == LampState.Off ? Theme.TextDim : Theme.Text);
            var sz = g.MeasureString(Caption, f);
            g.DrawString(Caption, f, b, cx + r + 6, cy - sz.Height / 2);
        }
    }

    private static Color Darken(Color c, float k)
        => Color.FromArgb(c.A, (int)(c.R * k), (int)(c.G * k), (int)(c.B * k));

    protected override void Dispose(bool disposing)
    {
        if (disposing) _blink.Dispose();
        base.Dispose(disposing);
    }
}
