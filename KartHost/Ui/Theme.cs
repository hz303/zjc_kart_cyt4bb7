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
}
