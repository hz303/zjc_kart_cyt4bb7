namespace KartHost;

internal static class Program
{
    [STAThread]
    private static int Main(string[] args)
    {
        // 无界面自检：KartHost.exe --selftest [结果文件]
        if (args.Length > 0 && args[0] == "--selftest")
        {
            ApplicationConfiguration.Initialize();   // 冒烟测试要构造窗体，必须先初始化
            string outPath = args.Length > 1
                ? args[1]
                : Path.Combine(AppContext.BaseDirectory, "selftest.txt");
            return SelfTest.Run(outPath);
        }

        // 整窗出图 + 布局体检：KartHost.exe --shot-form 输出.png
        // ★ 刻意**不做** Dpi.Reset() —— 它要看的恰恰是"当前这台机器 DPI 下的真实长相"
        if (args.Length > 0 && args[0] == "--shot-form")
        {
            ApplicationConfiguration.Initialize();
            string png = args.Length > 1 ? args[1] : Path.Combine(AppContext.BaseDirectory, "form.png");
            return SelfTest.ShotForm(png);
        }

        // ApplicationConfiguration.Initialize() 内部已包含
        // EnableVisualStyles + SetCompatibleTextRenderingDefault(false) + 高 DPI 设置，
        // 不要再重复调用 SetCompatibleTextRenderingDefault（会抛异常）。
        ApplicationConfiguration.Initialize();

        try
        {
            Application.Run(new Ui.MainForm());
        }
        catch (Exception ex)
        {
            MessageBox.Show(ex.ToString(), "KartHost 启动失败",
                MessageBoxButtons.OK, MessageBoxIcon.Error);
            return 1;
        }
        return 0;
    }
}
