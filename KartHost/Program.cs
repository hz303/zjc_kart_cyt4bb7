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

        // 整窗出图 + 布局体检：KartHost.exe --shot-form 输出.png [kart|motor|dual|steer|ins|kartrc|calib] [帧通道数]
        // ★ 刻意**不做** Dpi.Reset() —— 它要看的恰恰是"当前这台机器 DPI 下的真实长相"
        if (args.Length > 0 && args[0] == "--shot-form")
        {
            ApplicationConfiguration.Initialize();

            // 可选的第 3 个参数指定档案（默认卡丁车）—— 这样每个档案都能单独出图体检，
            // 不用为了截一张图去改 Defaults.Current。
            if (args.Length > 2)
            {
                switch (args[2].ToLowerInvariant())
                {
                    case "motor": Defaults.SetProfile(HostProfile.Motor);     break;
                    case "dual":  Defaults.SetProfile(HostProfile.DualMotor); break;
                    case "steer": Defaults.SetProfile(HostProfile.SteerLoop); break;
                    case "ins":   Defaults.SetProfile(HostProfile.InsRoute);  break;
                    case "kartrc":Defaults.SetProfile(HostProfile.KartRC);    break;
                    case "calib": Defaults.SetProfile(HostProfile.Calib);     break;
                    default:      Defaults.SetProfile(HostProfile.Kart);      break;
                }
            }

            // 可选的第 4 个参数：假装"帧里有 N 个通道"，用来给出「通道名整体错位」那条红条出图。
            // 例：--shot-form warn.png calib 39   （档案=标定·精简，帧=39 通道 ⇒ 明显错位）
            if (args.Length > 3 && int.TryParse(args[3], out int shotFc)) SelfTest.ShotFrameCount = shotFc;

            string png = args.Length > 1 ? args[1] : Path.Combine(AppContext.BaseDirectory, "form.png");
            return SelfTest.ShotForm(png);
        }

        // 路表管理窗口出图 + 布局体检：KartHost.exe --shot-route 输出.png
        //   与 --shot-form 同一套路（**不做** Dpi.Reset：要看的就是这台机器真实缩放下的长相）。
        //   里面灌一张现造的圆环表 ⇒ 不需要车、不需要串口、不需要任何外部文件。
        if (args.Length > 0 && args[0] == "--shot-route")
        {
            ApplicationConfiguration.Initialize();
            string png = args.Length > 1 ? args[1] : Path.Combine(AppContext.BaseDirectory, "route_mgr.png");
            return SelfTest.ShotRoute(png);
        }

        // 实车标定窗口出图 + 布局体检：KartHost.exe --shot-calib 输出.png
        //   窗口只依赖 ICalibHost，内部用假宿主 ⇒ 同样不需要车、不需要链路。
        if (args.Length > 0 && args[0] == "--shot-calib")
        {
            ApplicationConfiguration.Initialize();
            string png = args.Length > 1 ? args[1] : Path.Combine(AppContext.BaseDirectory, "calib.png");
            return SelfTest.ShotCalib(png);
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
