namespace KartHost.Ui;

/// <summary>
/// DPI 换算层：**所有界面尺寸都写 96 DPI 下的逻辑像素，再由这里乘上实际缩放**。
///
/// 为什么必须有这一层（血泪教训，别再问"这行是不是多余的"）：
///   字号写的是**点值**（`new Font("微软雅黑", 9f)`），GDI+ 会按当前 DPI 自动放大
///   （150% 下 9pt 中文约 18×24 物理像素）；而 `Width = 78` 是**物理像素**，不放大。
///   于是"字大了、框没大" —— 按钮只剩得下两三个字、输入框吃掉后半截。
///
///   症状有个恶心的特点：编译 0 警告、自检全绿、在 100% 缩放的开发机上完全正常，
///   只在 125%/150% 的机器上现形。所以必须把它变成"机器可判"的 —— 见 SelfTest 里的布局体检。
///
/// 用法：`Left = Dpi.Px(8)`、`Size = Dpi.Sz(120, 26)`、`Padding = Dpi.Pad(4,0,4,0)`。
/// 注意：**字号不要手动乘**（点值是物理量，乘了就缩两遍）。
/// </summary>
public static class Dpi
{
    public static double Scale { get; private set; } = 1.0;

    public static void Init(int deviceDpi) => Scale = (deviceDpi > 0 ? deviceDpi : 96) / 96.0;

    /// <summary>离屏出图用：让结果与机器无关、可复现。
    /// ⚠ Reset() 之后**不许**跑布局体检 —— GDI+ 仍按真实屏幕 DPI 渲染字体，
    ///   框按 96 算、字按 144 画，体检会满屏假阳性。</summary>
    public static void Reset() => Init(96);

    public static int Px(double v) => (int)Math.Round(v * Scale, MidpointRounding.AwayFromZero);

    public static float Pxf(double v) => (float)(v * Scale);

    public static Size Sz(double w, double h) => new(Px(w), Px(h));

    public static Padding Pad(double l, double t, double r, double b) => new(Px(l), Px(t), Px(r), Px(b));
}
