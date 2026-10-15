using System.Drawing.Drawing2D;

namespace KartHost.Ui;

/// <summary>深色主题配色（参考 VOFA+ 的观感）</summary>
public static class Theme
{
    public static readonly Color Bg = Color.FromArgb(22, 24, 28);
    public static readonly Color Panel = Color.FromArgb(30, 33, 38);
    public static readonly Color PanelAlt = Color.FromArgb(38, 42, 48);
    public static readonly Color Border = Color.FromArgb(56, 61, 70);
    public static readonly Color Text = Color.FromArgb(226, 230, 236);
    public static readonly Color TextDim = Color.FromArgb(148, 156, 168);
    public static readonly Color Accent = Color.FromArgb(78, 154, 241);
    public static readonly Color Ok = Color.FromArgb(56, 200, 122);
    public static readonly Color Warn = Color.FromArgb(240, 180, 60);
    public static readonly Color Err = Color.FromArgb(232, 82, 82);
    public static readonly Color Grid = Color.FromArgb(46, 50, 58);
    public static readonly Color GridMajor = Color.FromArgb(62, 68, 78);

    /// <summary>40 色曲线调色板（HSV 均匀分布，保证相邻通道可区分）</summary>
    public static readonly Color[] Curve = BuildPalette(48);

    private static Color[] BuildPalette(int n)
    {
        var a = new Color[n];
        for (int i = 0; i < n; i++)
        {
            float h = (i * 0.618034f) % 1.0f;          // 黄金角，避免相邻色相近
            float s = 0.62f + 0.28f * ((i % 3) / 2f);
            float v = 0.95f - 0.18f * ((i % 4) / 3f);
            a[i] = FromHsv(h, s, v);
        }
        return a;
    }

    public static Color CurveOf(int i) => Curve[((i % Curve.Length) + Curve.Length) % Curve.Length];

    public static Color FromHsv(float h, float s, float v)
    {
        h = (h % 1f + 1f) % 1f;
        int i = (int)(h * 6f);
        float f = h * 6f - i;
        float p = v * (1 - s), q = v * (1 - f * s), t = v * (1 - (1 - f) * s);
        float r, g, b;
        switch (i % 6)
        {
            case 0: r = v; g = t; b = p; break;
            case 1: r = q; g = v; b = p; break;
            case 2: r = p; g = v; b = t; break;
            case 3: r = p; g = q; b = v; break;
            case 4: r = t; g = p; b = v; break;
            default: r = v; g = p; b = q; break;
        }
        return Color.FromArgb((int)(r * 255), (int)(g * 255), (int)(b * 255));
    }

    /// <summary>把控件树整体刷成深色（WinForms 原生控件默认是浅色）</summary>
    public static void Apply(Control root)
    {
        switch (root)
        {
            case Form f:
                f.BackColor = Bg; f.ForeColor = Text;
                break;
            case Button b:
                b.FlatStyle = FlatStyle.Flat;
                b.FlatAppearance.BorderColor = Border;
                b.FlatAppearance.MouseOverBackColor = PanelAlt;
                b.BackColor = PanelAlt; b.ForeColor = Text;
                b.UseVisualStyleBackColor = false;
                break;
            case TextBox t:
                t.BackColor = PanelAlt; t.ForeColor = Text; t.BorderStyle = BorderStyle.FixedSingle;
                break;
            case NumericUpDown n:
                n.BackColor = PanelAlt; n.ForeColor = Text; n.BorderStyle = BorderStyle.FixedSingle;
                break;
            case ComboBox c:
                c.BackColor = PanelAlt; c.ForeColor = Text; c.FlatStyle = FlatStyle.Flat;
                break;
            case Label l:
                l.BackColor = Color.Transparent; l.ForeColor = Text;
                break;
            case CheckBox ck:
                // ButtonBase 系默认不带 SupportsTransparentBackColor，
                // 设 Transparent 会抛 "控件不支持透明的背景色" → 用面板色
                ck.BackColor = Panel; ck.ForeColor = Text;
                break;
            case GroupBox g:
                g.BackColor = Panel; g.ForeColor = TextDim;
                break;
            case Panel p:
                p.BackColor = p.BackColor == Color.Transparent ? Panel : p.BackColor;
                break;
            case TrackBar tb:
                tb.BackColor = Panel;
                break;
            case StatusStrip ss:
                ss.BackColor = Panel; ss.ForeColor = Text;
                break;
            case SplitContainer sc:
                sc.BackColor = Border;
                break;
        }
        foreach (Control c in root.Controls) Apply(c);
    }

    // ===================== 二维轨迹画布（BoardView）用色 =====================
    // 从 A_zjc_project/3.1imu_trails_mode_pc 的 Ui/Theme.cs 移植，配色改成与本站深色主题一致。
    public static readonly Color Canvas    = Color.FromArgb(16, 19, 24);     // 画布底（比 Bg 更深，突出轨迹）
    public static readonly Color Axis      = Color.FromArgb(74, 84, 100);    // 过原点的两条轴
    public static readonly Color GridDash  = Color.FromArgb(52, 62, 76);    // 罗盘虚线/辅助线
    public static readonly Color Ccw       = Color.FromArgb(108, 227, 168);  // 逆时针（左转）
    public static readonly Color Cw        = Color.FromArgb(255, 160, 90);   // 顺时针（右转）
    public static readonly Color Board     = Color.FromArgb(155, 201, 255);  // 车体轮廓
    public static readonly Color BoardFill = Color.FromArgb(36, 59, 88);     // 车体填充
    public static readonly Color Nose      = Color.FromArgb(255, 209, 84);   // 车头三角 / 罗盘指针
    public static readonly Color Vel       = Color.FromArgb(141, 224, 255);  // 速度矢量
    public static readonly Color Route     = Color.FromArgb(120, 230, 255);  // 已录路表（预留）

    /// <summary>
    /// 轨迹按时间着色：早=冷（蓝→青）、晚=暖（橙→红）。
    /// 用途与参考工程一致 —— 一眼看出"轨迹是从什么时候开始漂的"（冷色段是刚起步，暖色段是末端）。
    /// </summary>
    public static Color TimeColor(float f)
    {
        f = Math.Clamp(f, 0f, 1f);
        if (f < 0.45f) return FromHsv(0.58f - 0.13f * (f / 0.45f), 0.80f, 1.00f);          // 蓝 -> 青
        if (f < 0.75f) return FromHsv(0.45f - 0.37f * ((f - 0.45f) / 0.30f), 0.85f, 1.00f); // 青 -> 橙
        return FromHsv(0.08f * (1f - (f - 0.75f) / 0.25f), 0.90f, 1.00f);                    // 橙 -> 红
    }

    /// <summary>界面字体工厂（点值，**不要**再乘 DPI；GDI+ 会自己按缩放放大）</summary>
    public static Font F(float pt, FontStyle st = FontStyle.Regular) => new("Segoe UI", pt, st);

    /// <summary>等宽字体（数字对齐用）</summary>
    public static Font FM(float pt, FontStyle st = FontStyle.Regular) => new("Consolas", pt, st);

    /// <summary>圆角矩形路径（画车体用；用完要 Dispose）</summary>
    public static GraphicsPath RoundRect(Rectangle r, int radius)
    {
        var p = new GraphicsPath();
        int d = Math.Max(1, Math.Min(radius * 2, Math.Min(r.Width, r.Height)));
        p.AddArc(r.X, r.Y, d, d, 180, 90);
        p.AddArc(r.Right - d, r.Y, d, d, 270, 90);
        p.AddArc(r.Right - d, r.Bottom - d, d, d, 0, 90);
        p.AddArc(r.X, r.Bottom - d, d, d, 90, 90);
        p.CloseFigure();
        return p;
    }
}
