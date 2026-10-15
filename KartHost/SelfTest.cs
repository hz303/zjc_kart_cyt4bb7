using System.Text;

namespace KartHost;

/// <summary>
/// 无界面自检：验证协议编解码、帧同步状态机、以及「模拟车端 → 网络 → 解析」整条链路。
/// 用法：KartHost.exe --selftest 结果文件路径
/// </summary>
public static class SelfTest
{
    private static readonly StringBuilder Log = new();
    private static int _pass, _fail;

    private static void Check(bool ok, string name, string detail = "")
    {
        if (ok) { _pass++; Log.AppendLine("  [PASS] " + name); }
        else { _fail++; Log.AppendLine("  [FAIL] " + name + (detail.Length > 0 ? "  -> " + detail : "")); }
    }

    private static void Section(string t) => Log.AppendLine("\n== " + t + " ==");

    public static int Run(string outPath)
    {
        Log.AppendLine("KartHost 自检报告  " + DateTime.Now.ToString("yyyy-MM-dd HH:mm:ss"));
        // ⚠ 这里不能打印 Ui.Dpi.Scale —— 它是在"建窗之前"读的，必然是 1x（会骗人）。
        //   真实缩放见下面「高 DPI 布局体检」那一段：那里先 new MainForm()，Dpi 才是真值。
        Log.AppendLine($"进程 DPI 感知 = {ApplicationHighDpiMode()}"
                       + "（Ui.Dpi.Scale 见下方「高 DPI 布局体检」段，那里才是真值）");

        TestCrc();
        TestCommandFrame();
        TestRouteLinkCaps();         // 路表串口协议：能力位 / 旧固件识别
        TestFraming();
        TestEndToEnd();
        TestUdpLink();               // UDP：车端主动上报的路径（曾经绑错端口）
        TestMotorProfile();          // 单电机验证档案（跑完会切回卡丁车档案）
        TestDualMotorProfile();      // 双电机速度环档案（跑完会切回卡丁车档案）
        TestSteerLoopProfile();      // 转向闭环档案（跑完会切回卡丁车档案）
        TestInsRouteProfile();       // 惯导+路表档案（跑完会切回卡丁车档案）
        TestKartRCProfile();         // 卡丁·遥控档案（工程 9 全量，39 通道）
        TestCalibProfile();          // 标定·精简档案（工程 9 的 tel_sel=1，18 通道）
        TestFrameProfileMatch();     // ★ 帧 COUNT ↔ 档案 反查 + 波形上的"通道名错位"红条
        TestSimulatorFillsAllProfiles();  // 模拟车端：每个档案都必须有专用造数分支
        TestValueBoxes();            // 「实时值」数值框：每档案一张取数表（跑完会切回卡丁车档案）
        TestSteerFaultDecode();      // 转向保护锁存位的解读（bit0 自检 / bit1 反馈 / bit2 顶死）
        TestSteerLimitClamp();       // 舵机限位 ±27°（上位机侧）
        TestCsvRecorder();           // CSV 记录 / 导出模块（跑完会切回卡丁车档案）
        TestCsvExport();             // 波形快照导出：勾选的通道 → CSV（跑完会切回卡丁车档案）
        TestBoardView();             // 二维轨迹图的坐标约定 / 抽稀 / 渲染
        TestTravelCal();             // 行程标定（左右限位 → mrad/° 映射）
        TestLayout();                // 高 DPI 布局体检（含反向对照）
        TestUi();

        Log.AppendLine();
        Log.AppendLine($"结果：{_pass} 项通过，_fail 项失败".Replace("_fail", _fail.ToString()));

        try { File.WriteAllText(outPath, Log.ToString(), Encoding.UTF8); } catch { }
        Console.WriteLine(Log.ToString());
        return _fail == 0 ? 0 : 1;
    }

    // ==================================================================
    // UDP 链路
    //
    // 这条路径曾经是坏的：ConnectUdp 以前绑随机本地端口，而车端固定发往 8086，
    // 结果"车端在发、上位机一片空白"。下面的断言就是钉死这件事。
    // ==================================================================
    private static byte[] BuildUpFrame(FrameType type, byte[] payload)
    {
        int pl = payload?.Length ?? 0;
        var f = new byte[Proto.UP_OVERHEAD + pl];
        f[0] = Proto.MAGIC_UP_0; f[1] = Proto.MAGIC_UP_1;
        f[2] = (byte)type;
        Proto.PutU16(f, 3, (ushort)pl);
        if (pl > 0) Buffer.BlockCopy(payload, 0, f, 5, pl);
        Proto.PutU16(f, 5 + pl, Proto.Crc16(f, 2, 3 + pl));
        Array.Copy(Proto.TAIL, 0, f, 7 + pl, 4);
        return f;
    }

    private static void TestUdpLink()
    {
        Section("UDP 链路（车端主动上报 → 绑本地端口 → 解析 → 回发）");

        const int port = 18525;
        var link = new Link();
        int frames = 0;
        link.FrameReceived += (t, b, o) => { if (t == (byte)FrameType.Telemetry) frames++; };

        var sender = new System.Net.Sockets.UdpClient(0);
        var dst = new System.Net.IPEndPoint(System.Net.IPAddress.Loopback, port);

        try { link.ConnectUdp(port); Check(true, $"绑定本地端口 :{port}（不是随机端口）"); }
        catch (Exception ex) { Check(false, "绑定本地端口", ex.Message); }

        // 按车端的格式造 3 帧遥测
        var pl = new byte[Proto.TELEM_OVERHEAD + 4 * 4];
        Proto.PutU16(pl, 0, 7);
        Proto.PutU32(pl, 2, 1234);
        Proto.PutU16(pl, 6, 0x000F);
        Proto.PutU16(pl, 8, 4);
        for (int i = 0; i < 4; i++) Proto.PutF32(pl, 10 + i * 4, i * 1.5f);
        var frame = BuildUpFrame(FrameType.Telemetry, pl);

        for (int i = 0; i < 3; i++) { sender.Send(frame, frame.Length, dst); Thread.Sleep(30); }
        Thread.Sleep(400);
        Check(frames >= 3, "UDP 收到 ≥3 帧遥测", "实际 " + frames);

        // 下行：Link 应当已经从收到的包里学到车端地址，能把命令发回去
        bool back = false;
        try
        {
            link.Send(Proto.BuildCommand(Cmd.Ping, 0, 0f, 77));
            sender.Client.ReceiveTimeout = 1000;
            var any = new System.Net.IPEndPoint(System.Net.IPAddress.Any, 0);
            var b = sender.Receive(ref any);
            back = b != null && b.Length == 15 && b[5] == (byte)Cmd.Ping;
        }
        catch (Exception ex) { Check(false, "UDP 回发命令", ex.Message); }
        Check(back, "UDP 回发：自动学到车端地址，下行命令发得回去");

        link.Close();
        sender.Close();
    }

    // ==================================================================
    // 单电机验证档案
    //
    // 这里不只断言"没崩"，还断言**画出来的数对不对**：
    //   使能不全 → 必须真的停在 0；给正/负占空比 → 方向位和输出必须跟着走；
    //   参数表的量程/默认值必须和 firmware 的 MOTOR_PARAM_TABLE 一致。
    // ==================================================================
    private static void TestMotorProfile()
    {
        Section("单电机验证档案（12 通道 + 电机参数表）");

        // ---- 1) 通道表 ----
        Defaults.SetProfile(HostProfile.Motor);
        string[] want =
        {
            "duty_cmd", "duty_act", "dir", "pwm_raw", "enable", "online",
            "rx_age", "duty_limit", "ramp_ms", "tx_count", "uptime", "init_err",
            "mag_raw", "mag_angle", "mag_deg", "mag_delta",
            "mag_speed", "mag_sum", "mag_delta_w", "mag_glitch",
        };
        Check(Defaults.ChannelNames.Length == 20, "通道数 = 20", "实际 " + Defaults.ChannelNames.Length);
        bool namesOk = true;
        string nameBad = "";
        for (int i = 0; i < want.Length; i++)
        {
            if (Defaults.ChannelName(i) != want[i]) { namesOk = false; nameBad = $"ch{i} = {Defaults.ChannelName(i)}"; break; }
        }
        Check(namesOk, "通道名与 firmware motor_link.h 的 CH_* 一一对应", nameBad);

        // ---- 2) 参数表（必须与 firmware 的 MOTOR_PARAM_TABLE 完全一致）----
        var ps = Defaults.BuildDefaultParams();
        (string name, float lo, float hi, float def, float step)[] wantP =
        {
            ("duty_cmd",   -100f,  100f,   0f,  0.5f),
            ("duty_limit",    0f,  100f,  60f,  1.0f),
            ("ramp_ms",       0f, 3000f, 400f, 10.0f),
            ("deadzone",      0f,   20f,   0f,  0.1f),
            ("lost_ms",     100f, 3000f, 600f, 10.0f),
            ("auto_stop",     0f,    1f,   1f,  1.0f),
            ("mag_proto",     0f,    3f,   3f,  1.0f),
            ("mag_dir",       0f,    1f,   1f,  1.0f),
            ("mag_zero",      0f,65535f,   0f,  1.0f),
            ("sum_clr",       0f,    1f,   0f,  1.0f),
            ("spi_mode",      0f,    3f,   2f,  1.0f),
            ("spi_mhz",       1f,   40f,   8f,  1.0f),
            ("k_speed",     -10f,   10f, 1.917476f, 0.001f),
            ("glitch_th",     0f, 8192f, 512f,  1.0f),
        };
        Check(ps.Count == wantP.Length, $"参数数量 = {wantP.Length}", "实际 " + ps.Count);
        bool pOk = true;
        string pBad = "";
        for (int i = 0; i < Math.Min(ps.Count, wantP.Length); i++)
        {
            var p = ps[i];
            var w = wantP[i];
            if (p.Id != i || p.Name != w.name || Math.Abs(p.Lo - w.lo) > 1e-3
                || Math.Abs(p.Hi - w.hi) > 1e-3 || Math.Abs(p.Def - w.def) > 1e-3
                || Math.Abs(p.Step - w.step) > 1e-3)
            {
                pOk = false;
                pBad = $"#{i}: got {p.Name}[{p.Lo},{p.Hi}]def={p.Def}step={p.Step}";
                break;
            }
        }
        Check(pOk, "参数表的量程/默认/步长与 firmware 一致", pBad);

        // ---- 3) 端到端：模拟车端（单电机档案）→ TCP → 解析 → 通道值 ----
        var sim = new Simulator { Port = 18522 };
        var link = new Link();
        var store = new ChannelStore();
        long frames = 0;
        long afterStart = 0;

        link.FrameReceived += (t, b, o) =>
        {
            if (t != (byte)FrameType.Telemetry) return;
            int cnt = Proto.GetU16(b, o + 8);
            var ch = new float[cnt];
            for (int i = 0; i < cnt; i++) ch[i] = Proto.GetF32(b, o + 10 + i * 4);
            store.Ensure(cnt);
            store.Append(ch, cnt, 1.0, Proto.GetU16(b, o + 6));
            frames++;
        };

        bool up = false;
        sim.Start();
        Thread.Sleep(150);
        try { link.ConnectTcp("127.0.0.1", sim.Port); up = true; }
        catch (Exception ex) { Check(false, "连接模拟车端（单电机）", ex.Message); }

        if (up)
        {
            WaitWithHeartbeat(link, 0.6);
            afterStart = frames;
            Check(afterStart >= 10, "收到 ≥10 帧遥测", "实际 " + afterStart);

            // ★ 关键断言 1：使能不全时，哪怕 duty_cmd 给了值，实际输出也必须是 0
            link.Send(Proto.BuildCommand(Cmd.SetParam, 0, 40f, 1));
            WaitWithHeartbeat(link, 0.4);
            Check(Math.Abs(store.Latest(1)) < 0.5f,
                  "未使能时 duty_act 恒为 0", "实际 " + store.Latest(1).ToString("F2"));

            // ★ 关键断言 2：使能后正占空比 → 输出跟上，方向位 = 1
            link.Send(Proto.BuildCommand(Cmd.SetEnable, 0, (ushort)(EnableBits.Master | EnableBits.Motor), 2));
            WaitWithHeartbeat(link, 1.5);
            float act = store.Latest(1);
            Check(Math.Abs(act - 40f) < 2.0f, "使能后 duty_act 收敛到 +40%", "实际 " + act.ToString("F2"));
            Check(store.Latest(2) > 0.5f, "占空比为正 → dir 电平 = 1", "实际 " + store.Latest(2).ToString("F2"));
            Check(Math.Abs(store.Latest(3) - 4000f) < 80f,
                  "pwm_raw ≈ 40% × 100 = 4000", "实际 " + store.Latest(3).ToString("F0"));

            // ★ 关键断言 3：负占空比 → 大小不变、方向位翻转
            link.Send(Proto.BuildCommand(Cmd.SetParam, 0, -40f, 3));
            WaitWithHeartbeat(link, 2.5);
            float actN = store.Latest(1);
            Check(Math.Abs(actN + 40f) < 2.0f, "反向后 duty_act 收敛到 -40%", "实际 " + actN.ToString("F2"));
            Check(store.Latest(2) < 0.5f, "占空比为负 → dir 电平 = 0", "实际 " + store.Latest(2).ToString("F2"));
            Check(Math.Abs(store.Latest(3) - 4000f) < 80f, "反向后 pwm_raw 大小不变", "实际 " + store.Latest(3).ToString("F0"));

            // ★ 关键断言 4：撤掉使能 → 立刻回 0
            link.Send(Proto.BuildCommand(Cmd.SetEnable, 0, (ushort)EnableBits.None, 4));
            WaitWithHeartbeat(link, 1.5);
            Check(Math.Abs(store.Latest(1)) < 1.0f, "撤使能后 duty_act 回到 0", "实际 " + store.Latest(1).ToString("F2"));

            // ★ 关键断言 5：磁编码器那 4 个通道确实映射到了（模拟器给的是"匀速转动"，
            //   所以度数必须落在 0~360，每帧变化量应当恒定 —— 只断言"通道数=16"是套套逻辑）
            float magDeg = store.Latest(14);
            float magDelta = store.Latest(15);
            Check(magDeg >= 0f && magDeg < 360f, "mag_deg 落在 0~360°", "实际 " + magDeg.ToString("F1"));
            Check(Math.Abs(magDelta - 60f) < 5f, "mag_delta ≈ 60（3000 计数/s × 20ms）", "实际 " + magDelta.ToString("F1"));

            // ★ 关键断言 6：新加的 4 个通道（角速度 / 积分 / 圆上最短差 / 毛刺）也确实映射到了
            float magDW = store.Latest(18);
            Check(Math.Abs(magDW - 60f) < 5f, "mag_delta_w ≈ 60（模拟器无跨零，应与 mag_delta 一致）",
                  "实际 " + magDW.ToString("F1"));
            float magSpd = store.Latest(16);
            Check(Math.Abs(magSpd - 1.15f) < 0.1f, "mag_speed ≈ 1.15 rad/s（与模拟器 3000 计数/s 自洽）",
                  "实际 " + magSpd.ToString("F2"));
            Check(store.Latest(17) > 0f, "mag_sum 在累加（>0）", "实际 " + store.Latest(17).ToString("F1"));
            Check(store.Latest(19) == 0f, "mag_glitch = 0（模拟器无噪声）", "实际 " + store.Latest(19).ToString("F0"));

            // ★ 关键断言 6：**心跳一停，车端就该判失联并清零输出**。
            //   以前这里是空的：模拟器把 online 写死成 1，上位机又没有心跳，
            //   于是真机上"拖到 40% 转一下、松手 0.6 秒自己停"照样发生，自检全绿。
            link.Send(Proto.BuildCommand(Cmd.SetEnable, 0,
                                         (ushort)(EnableBits.Master | EnableBits.Motor), 400));
            link.Send(Proto.BuildCommand(Cmd.SetParam, 0, 40f, 401));
            WaitWithHeartbeat(link, 1.2);
            float hbAct = store.Latest(1);
            Check(Math.Abs(hbAct - 40f) < 2.0f, "（前置）有心跳时输出稳定在 +40%",
                  "实际 " + hbAct.ToString("F2"));

            Thread.Sleep(1000);                     // 故意不发心跳，超过 lost_ms=600
            Check(store.Latest(5) < 0.5f, "停心跳 1s 后 online 位变 0",
                  "实际 " + store.Latest(5).ToString("F2"));
            Check(Math.Abs(store.Latest(1)) < 1.0f, "停心跳后 duty_act 自动归 0（失联保护生效）",
                  "实际 " + store.Latest(1).ToString("F2"));
            Check(Math.Abs(store.Latest(0) - 40f) < 1.0f, "（对照）duty_cmd 参数本身没被改",
                  "实际 " + store.Latest(0).ToString("F2"));

            WaitWithHeartbeat(link, 1.6);            // 心跳恢复 —— 走的就是 App 用的那条路
            Check(store.Latest(5) > 0.5f, "心跳恢复后 online 位回到 1",
                  "实际 " + store.Latest(5).ToString("F2"));
            Check(Math.Abs(store.Latest(1) - 40f) < 2.0f, "恢复心跳后输出重新跟上 +40%",
                  "实际 " + store.Latest(1).ToString("F2"));

            // ★ 关键断言 7：心跳参数本身要合规（周期必须明显小于车端 lost_ms）
            Check(Heartbeat.IntervalMs > 0 && Heartbeat.IntervalMs <= 200,
                  "心跳周期 <= 200ms（车端 lost_ms 默认 600、最小 100）",
                  "实际 " + Heartbeat.IntervalMs + "ms");
            Check(!Heartbeat.Beat(new Link(), () => 1), "链路没开时 Beat 返回 false 且不抛异常");

            link.Close();
        }

        sim.Stop();
        sim.Dispose();

        // ---- 4) 切回全车档案，别把后面的界面测试带歪 ----
        Defaults.SetProfile(HostProfile.Kart);
        Check(Defaults.ChannelNames.Length == 32, "切回卡丁车档案 32 通道", "实际 " + Defaults.ChannelNames.Length);
    }

    // ==================================================================
    // 双电机速度环档案
    //
    // 光断言"通道数 = 20"是套套逻辑 —— 这里把通道名/单位、参数表、以及
    // "未使能 → v 恒 0、使能后 → v 收敛到目标" 这三件事都钉死。
    // ==================================================================
    private static void TestDualMotorProfile()
    {
        Section("双电机速度环档案（20 通道 + 速度环参数表）");

        Defaults.SetProfile(HostProfile.DualMotor);

        string[] wantName =
        {
            "target_l", "v_l",      "u_l",      "target_r", "v_r",      "u_r",
            "uff_l",    "uff_r",    "z_l",      "z_r",
            "pos_l",    "pos_r",    "dlt_l",    "dlt_r",
            "flip_l",   "flip_r",   "online",   "rx_age",   "enable",   "safety",
        };
        string[] wantUnit =
        {
            "rps", "rps", "%",     "rps",   "rps", "%",
            "%",   "%",   "rps·s", "rps·s",
            "cnt", "cnt", "cnt",   "cnt",
            "-",   "-",   "-",     "ms",    "-",   "-",
        };

        Check(Defaults.ChannelNames.Length == 20, "通道数 = 20", "实际 " + Defaults.ChannelNames.Length);
        Check(Defaults.ChannelUnits.Length == 20, "单位表长度 = 20", "实际 " + Defaults.ChannelUnits.Length);

        bool nameOk = true, unitOk = true; string bad = "";
        for (int i = 0; i < wantName.Length; i++)
        {
            if (Defaults.ChannelName(i) != wantName[i]) { nameOk = false; bad = $"ch{i} 名 = {Defaults.ChannelName(i)}"; break; }
            if (Defaults.ChannelUnit(i) != wantUnit[i]) { unitOk = false; bad = $"ch{i} 单位 = {Defaults.ChannelUnit(i)}"; break; }
        }
        Check(nameOk, "通道名与车端 CH_* 逐项对应", bad);
        Check(unitOk, "通道单位与上表逐项一致", bad);

        Check(Defaults.ProfileDisplayName == "双电机速度环", "档案显示名 = 双电机速度环", Defaults.ProfileDisplayName);
        Check(Defaults.ProfileIndex == 2, "下拉框下标 = 2", "实际 " + Defaults.ProfileIndex);
        Check(Defaults.IsDualMotor && !Defaults.IsMotor, "IsDualMotor 与 IsMotor 互斥");

        bool presetIdxOk = true;
        foreach (var preset in Defaults.PlotPresets)
            foreach (int c in preset) if (c < 0 || c >= Defaults.ChannelNames.Length) presetIdxOk = false;
        Check(presetIdxOk, "分图预设下标都落在 0~19");
        Check(Defaults.PlotPresets.Length == Defaults.PlotPresetNames.Length,
              "分图预设与名字数量一致", $"{Defaults.PlotPresets.Length}/{Defaults.PlotPresetNames.Length}");

        bool visOk = true;
        foreach (int c in Defaults.DefaultVisible) if (c < 0 || c >= Defaults.ChannelNames.Length) visOk = false;
        Check(visOk, "默认可见通道下标都落在 0~19");

        // ---- 参数表：目标转速必须能从滑条下发 ----
        var ps = Defaults.BuildDefaultParams();
        var tl = ps.Find(p => p.Name == "target_l");
        var tr = ps.Find(p => p.Name == "target_r");
        Check(tl != null && tr != null, "参数表含 target_l / target_r");
        Check(tl != null && tl.Unit == "rps" && Math.Abs(tl.Def) < 1e-6 && tl.Hi > tl.Lo,
              "target_l 单位 rps、默认 0、量程有效");
        Check(ps.Exists(p => p.Name == "k_v") && ps.Exists(p => p.Name == "k_z") && ps.Exists(p => p.Name == "duty_limit"),
              "参数表含速度环增益 k_v / k_z / duty_limit");

        // ---- 端到端：模拟车端（双电机档案）→ TCP → 解析 ----
        var sim = new Simulator { Port = 18523 };
        var link = new Link();
        var store = new ChannelStore();
        long frames = 0;

        link.FrameReceived += (t, b, o) =>
        {
            if (t != (byte)FrameType.Telemetry) return;
            int cnt = Proto.GetU16(b, o + 8);
            var ch = new float[cnt];
            for (int i = 0; i < cnt; i++) ch[i] = Proto.GetF32(b, o + 10 + i * 4);
            store.Ensure(cnt);
            store.Append(ch, cnt, 1.0, Proto.GetU16(b, o + 6));
            frames++;
        };

        bool up = false;
        sim.Start();
        Thread.Sleep(150);
        try { link.ConnectTcp("127.0.0.1", sim.Port); up = true; }
        catch (Exception ex) { Check(false, "连接模拟车端（双电机）", ex.Message); }

        if (up)
        {
            WaitWithHeartbeat(link, 0.6);
            Check(frames >= 10, "收到 ≥10 帧遥测", "实际 " + frames);

            // 目标经滑条下发（param 0 = target_l，param 1 = target_r）
            link.Send(Proto.BuildCommand(Cmd.SetParam, 0, 1.0f, 1));
            link.Send(Proto.BuildCommand(Cmd.SetParam, 1, 1.0f, 2));
            WaitWithHeartbeat(link, 0.6);

            // ★ 关键断言 1：目标参数已写进去，但未使能 → 实测转速/输出必须还是 0
            Check(Math.Abs(store.Latest(0) - 1.0f) < 0.05f, "target_l 参数已下发 = 1.0",
                  "实际 " + store.Latest(0).ToString("F2"));
            Check(Math.Abs(store.Latest(1)) < 0.5f, "未使能时 v_l 恒为 0",
                  "实际 " + store.Latest(1).ToString("F2"));
            Check(Math.Abs(store.Latest(2)) < 0.5f, "未使能时 u_l 恒为 0",
                  "实际 " + store.Latest(2).ToString("F2"));

            // ★ 关键断言 2：使能后左右轮实测转速都收敛到目标 40 rps
            link.Send(Proto.BuildCommand(Cmd.SetEnable, 0,
                                         (ushort)(EnableBits.Master | EnableBits.Motor), 3));
            WaitWithHeartbeat(link, 1.5);
            Check(Math.Abs(store.Latest(1) - 1.0f) < 0.15f, "使能后 v_l 收敛到 1.0 rps",
                  "实际 " + store.Latest(1).ToString("F2"));
            Check(Math.Abs(store.Latest(4) - 1.0f) < 0.15f, "使能后 v_r 收敛到 1.0 rps",
                  "实际 " + store.Latest(4).ToString("F2"));
            Check(Math.Abs(store.Latest(2)) > 1f, "使能后 u_l 有实际输出",
                  "实际 " + store.Latest(2).ToString("F2"));
            Check(store.Latest(10) > 0f, "pos_l 在累计（>0）", "实际 " + store.Latest(10).ToString("F0"));
            Check(store.Latest(16) > 0.5f, "online 位 = 1", "实际 " + store.Latest(16).ToString("F0"));
            Check(store.Latest(18) > 0f, "enable 位图随使能变化", "实际 " + store.Latest(18).ToString("F0"));

            // ★ 关键断言 3：撤使能 → 输出立刻归 0
            link.Send(Proto.BuildCommand(Cmd.SetEnable, 0, (ushort)EnableBits.None, 4));
            WaitWithHeartbeat(link, 1.0);
            Check(Math.Abs(store.Latest(2)) < 0.5f, "撤使能后 u_l 归 0",
                  "实际 " + store.Latest(2).ToString("F2"));

            link.Close();
        }

        sim.Stop();
        sim.Dispose();

        // ---- 界面：新档案在真实 DPI 下也不能剪字 / 越界 / 互相压 ----
        Ui.MainForm f = null;
        try
        {
            f = new Ui.MainForm();                 // 构造时 Current = DualMotor → 建的就是新档案的界面
            f.CreateControl();
            _ = f.Handle;
            f.PerformLayout();
            f.ApplyRuntimeLayoutForTest();

            var issues = LayoutAudit.Walk(f);
            var clipped = issues.Where(i => i.Kind == "剪字").ToList();
            var overflow = issues.Where(i => i.Kind.EndsWith("越界")).ToList();
            var overlap = issues.Where(i => i.Kind == "重叠").ToList();
            Check(clipped.Count == 0, "双电机档案界面无剪字",
                  clipped.Count + " 处：" + string.Join(" ; ", clipped.Take(3).Select(i => i.Detail)));
            Check(overflow.Count == 0, "双电机档案界面无越界",
                  overflow.Count + " 处：" + string.Join(" ; ", overflow.Take(3).Select(i => i.Detail)));
            Check(overlap.Count == 0, "双电机档案界面无控件互相压",
                  overlap.Count + " 处：" + string.Join(" ; ", overlap.Take(3).Select(i => i.Detail)));
        }
        catch (Exception ex) { Check(false, "双电机档案界面构建 / 布局体检", ex.GetType().Name + ": " + ex.Message); }
        finally { try { f?.Dispose(); } catch { } }

        // ---- 切回全车档案，别把后面的界面测试带歪 ----
        Defaults.SetProfile(HostProfile.Kart);
        Check(Defaults.ChannelNames.Length == 32, "切回卡丁车档案 32 通道", "实际 " + Defaults.ChannelNames.Length);
    }

    // ==================================================================
    // 转向闭环档案（工程 7 = 双电机速度环 + 转向位置环）
    //
    // 除了通道 / 参数表，这里还钉死一条**最容易踩的固件行为**：
    // 内环使能的 0→1 上升沿会用"当前实测角"重置目标 ⇒ **先给目标再使能 = 目标被吞掉**。
    // 不知道这条的话，现场会以为"使能了但方向盘不动"是坏了。
    // ==================================================================
    private static void TestSteerLoopProfile()
    {
        Section("转向闭环档案（26 通道 + 25 项参数表）");

        Defaults.SetProfile(HostProfile.SteerLoop);

        string[] wantName =
        {
            "target_l", "v_l",      "u_l",      "target_r", "v_r",      "u_r",
            "uff_l",    "uff_r",    "z_l",      "z_r",
            "pos_l",    "pos_r",    "dlt_l",    "dlt_r",
            "flip_l",   "flip_r",   "online",   "rx_age",   "enable",   "safety",
            "steer_tgt", "steer_delta", "steer_u", "steer_raw", "steer_en", "steer_status",
        };
        string[] wantUnit =
        {
            "rps",   "rps",  "%",     "rps",   "rps",   "%",
            "%",     "%",    "rps·s", "rps·s",
            "cnt",   "cnt",  "cnt",   "cnt",
            "-",     "-",    "-",     "ms",    "-",     "-",
            "deg",   "deg",  "duty",  "raw",   "-",     "-",
        };

        Check(Defaults.ChannelNames.Length == 26, "通道数 = 26", "实际 " + Defaults.ChannelNames.Length);
        Check(Defaults.ChannelUnits.Length == 26, "单位表长度 = 26", "实际 " + Defaults.ChannelUnits.Length);

        bool nameOk = true, unitOk = true; string bad = "";
        for (int i = 0; i < wantName.Length; i++)
        {
            if (Defaults.ChannelName(i) != wantName[i]) { nameOk = false; bad = $"ch{i} 名 = {Defaults.ChannelName(i)}"; break; }
            if (Defaults.ChannelUnit(i) != wantUnit[i]) { unitOk = false; bad = $"ch{i} 单位 = {Defaults.ChannelUnit(i)}"; break; }
        }
        Check(nameOk, "通道名与车端 CH_* 逐项对应", bad);
        Check(unitOk, "通道单位与上表逐项一致", bad);

        // ★ 前 20 个下标必须与「双电机速度环」一字不差 —— 工程 7 = 工程 6 + 转向。
        //   老下标动了就是改协议，两个档案的前 20 列解读不再通用。
        string[] dual20 =
        {
            "target_l", "v_l", "u_l", "target_r", "v_r", "u_r",
            "uff_l", "uff_r", "z_l", "z_r",
            "pos_l", "pos_r", "dlt_l", "dlt_r",
            "flip_l", "flip_r", "online", "rx_age", "enable", "safety",
        };
        bool prefixOk = true;
        for (int i = 0; i < 20 && prefixOk; i++)
            if (Defaults.ChannelName(i) != dual20[i]) prefixOk = false;
        Check(prefixOk, "前 20 个通道与双电机档案完全一致（老下标没动）");

        Check(Defaults.ProfileDisplayName == "转向闭环", "档案显示名 = 转向闭环", Defaults.ProfileDisplayName);
        Check(Defaults.ProfileIndex == 3, "下拉框下标 = 3", "实际 " + Defaults.ProfileIndex);
        Check(Defaults.IsSteerLoop && !Defaults.IsDualMotor && !Defaults.IsMotor,
              "IsSteerLoop 与另两个闭环档案互斥");
        Check(Defaults.IsClosedLoop, "归入闭环类档案（50 Hz 固定遥测）");

        bool presetIdxOk = true;
        foreach (var preset in Defaults.PlotPresets)
            foreach (int c in preset) if (c < 0 || c >= Defaults.ChannelNames.Length) presetIdxOk = false;
        Check(presetIdxOk, "分图预设下标都落在 0~25");
        Check(Defaults.PlotPresets.Length == Defaults.PlotPresetNames.Length,
              "分图预设与名字数量一致", $"{Defaults.PlotPresets.Length}/{Defaults.PlotPresetNames.Length}");

        bool visOk = true;
        foreach (int c in Defaults.DefaultVisible) if (c < 0 || c >= Defaults.ChannelNames.Length) visOk = false;
        Check(visOk, "默认可见通道下标都落在 0~25");

        // ---- 参数表：与车端 ID 逐项对齐（错位 = 拖错滑条，把限幅写成 0.02% 那种）----
        var ps = Defaults.BuildDefaultParams();
        Check(ps.Count == 27, "参数表 27 项（与车端 MOTOR_PARAM_TABLE 同数）", "实际 " + ps.Count);
        bool idOk = true;
        for (int i = 0; i < ps.Count; i++) if (ps[i].Id != i) { idOk = false; bad = $"第 {i} 项 Id = {ps[i].Id}"; break; }
        Check(idOk, "参数 Id 与下标一一对应（不会错位）", bad);

        Check(ps.Exists(p => p.Name == "target_l") && ps.Exists(p => p.Name == "duty_limit")
              && ps.Exists(p => p.Name == "k_v") && ps.Exists(p => p.Name == "k_z"),
              "含后轮速度环参数（target_l / duty_limit / k_v / k_z）");
        Check(ps.Exists(p => p.Name == "steer_tgt") && ps.Exists(p => p.Name == "steer_kp")
              && ps.Exists(p => p.Name == "steer_en") && ps.Exists(p => p.Name == "minv_s"),
              "含转向参数（steer_tgt / steer_kp / steer_en / minv_s）");
        Check(ps.Exists(p => p.Name == "steer_u0") && ps.Exists(p => p.Name == "steer_db")
              && ps.Exists(p => p.Name == "steer_slew") && ps.Exists(p => p.Name == "steer_lim"),
              "含新控制律参数（steer_u0 / steer_db / steer_slew / steer_lim）");

        // ★★ 2026-10-14：ID25「目标斜坡」默认必须 = 0（关）—— 照队友实车结论
        //   （YINGYYUN/Karting-Project commit 4ce6c93「删除了舵机的目标斜坡」）：
        //   本环是纯 P + 静摩擦前馈、**没有积分器**，阶跃不需要防裹死；误差已被 ±27 软限位卡住，
        //   Kp·err 直接顶到 u_max 才是"最快到位"。留着斜坡只剩**纯滞后** ——
        //   我们这条还是 40 度/秒（0→27° 要 675 ms）。这条断言防止哪天被"顺手改回 40"。
        var slewP = ps.Find(p => p.Name == "steer_slew");
        Check(slewP != null && Math.Abs(slewP.Def) < 1e-6f,
              "ID25 steer_slew 默认 = 0（关）—— 照队友实测：纯 P+前馈环里斜坡只贡献纯滞后",
              slewP == null ? "找不到" : $"实际默认 {slewP.Def}");
        Check(slewP != null && slewP.Hi > 0f,
              "…但量程还在（>0）⇒ 想研究斜坡仍能在线拖回来，不用重烧",
              slewP == null ? "找不到" : $"Hi = {slewP.Hi}");

        // ★★ 2026-10-15：ID22「输出上限」默认 3500 → 9000（用户拍板）。
        //   这条断言的目的**不是**"钉住 9000 这个数"，而是钉住"这个决定的含义别被当成随手调参"：
        //     Kp=300 / u0=1600 ⇒ 饱和门槛 = (u_max − u0) / Kp
        //        3500 ⇒ |err| ≈ 6.3°   大阶跃一半路程恒出 35%，被限幅压着
        //        9000 ⇒ |err| ≈ 24.7°  几乎全行程按 Kp·err 比例给力（前段更快、也更冲）
        //   要换值就顺手改这条，但**别在没想清楚饱和门槛的前提下改回去**。
        var umaxP = ps.Find(p => p.Name == "steer_umax");
        Check(umaxP != null && Math.Abs(umaxP.Def - 9000f) < 1f,
              "ID22 steer_umax 默认 = 9000(90%) ⇒ 饱和门槛 |err| 抬到 ≈24.7°（几乎全行程按误差比例给力）",
              umaxP == null ? "找不到" : $"实际默认 {umaxP.Def}");
        Check(umaxP != null && umaxP.Hi >= 9000f,
              "…且量程容得下（≥9000）⇒ 9000 不会被上位机静默夹小",
              umaxP == null ? "找不到" : $"Hi = {umaxP.Hi}");
        var u0P = ps.Find(p => p.Name == "steer_u0");
        Check(u0P != null && u0P.Def < umaxP?.Def,
              "…U0 仍远小于 u_max（否则前馈一项就把输出钉在限幅上，Kp 就没作用了）",
              $"u0 = {u0P?.Def} / u_max = {umaxP?.Def}");

        // ★ 把三个关键 ID 的位置钉死 —— 写这段自检时就因为把 steer_en 当成 24 而挂过
        Check(ps[18].Name == "steer_tgt" && ps[23].Name == "steer_en" && ps[24].Name == "minv_s",
              "转向参数的 ID 位置与车端一致（18 / 23 / 24）",
              $"18={ps[18].Name} 23={ps[23].Name} 24={ps[24].Name}");
        var st = ps.Find(p => p.Name == "steer_tgt");
        Check(st != null && st.Unit == "deg" && st.Lo < 0 && st.Hi > 0, "steer_tgt 单位 deg、量程双向有效");

        // ---- 端到端：模拟车端（转向档案）→ TCP → 解析 ----
        var sim = new Simulator { Port = 18527 };
        var link = new Link();
        var store = new ChannelStore();
        long frames = 0;

        link.FrameReceived += (t, b, o) =>
        {
            if (t != (byte)FrameType.Telemetry) return;
            int cnt = Proto.GetU16(b, o + 8);
            var ch = new float[cnt];
            for (int i = 0; i < cnt; i++) ch[i] = Proto.GetF32(b, o + 10 + i * 4);
            store.Ensure(cnt);
            store.Append(ch, cnt, 1.0, Proto.GetU16(b, o + 6));
            frames++;
        };

        bool up = false;
        sim.Start();
        Thread.Sleep(150);
        try { link.ConnectTcp("127.0.0.1", sim.Port); up = true; }
        catch (Exception ex) { Check(false, "连接模拟车端（转向）", ex.Message); }

        if (up)
        {
            WaitWithHeartbeat(link, 0.6);
            Check(frames >= 10, "收到 ≥10 帧遥测", "实际 " + frames);
            Check(store.ChannelCount == 26, "帧内通道数 = 26", "实际 " + store.ChannelCount);

            // ① 未使能：目标可写，但输出恒 0、转角不动
            link.Send(Proto.BuildCommand(Cmd.SetParam, 18, 20f, 1));       // steer_tgt = 20 度
            WaitWithHeartbeat(link, 0.6);
            Check(Math.Abs(store.Latest(20) - 20f) < 2f, "未使能时目标可写入（steer_tgt = 20 度）",
                  "实际 " + store.Latest(20).ToString("F1"));
            Check(Math.Abs(store.Latest(22)) < 0.5f, "未使能时转向输出恒 0",
                  "实际 " + store.Latest(22).ToString("F1"));
            Check(Math.Abs(store.Latest(21)) < 2f, "未使能时转角不动",
                  "实际 " + store.Latest(21).ToString("F1"));

            // ② ★ 关键：先给目标、再使能 → 上升沿用"当前实测角"重置目标，旧目标被吞掉
            link.Send(Proto.BuildCommand(Cmd.SetParam, 23, 1f, 2));        // steer_en = 1（ID 23）
            WaitWithHeartbeat(link, 0.8);
            Check(Math.Abs(store.Latest(20)) < 2f, "使能上升沿吞掉旧目标（防大跳变）",
                  "实际 " + store.Latest(20).ToString("F1"));
            Check(store.Latest(24) > 0.5f, "steer_en 通道 = 1", "实际 " + store.Latest(24).ToString("F0"));
            Check(Math.Abs(store.Latest(21)) < 2f, "所以方向盘没有跳变",
                  "实际 " + store.Latest(21).ToString("F1"));

            // ③ 给目标，看它转到位。
            //    Kp/U0 的默认值现在是队友实车验证过的 300 / 1600，所以不必再"补增益"；
            //    这里仍显式写一次 kp，是为了让这条用例不依赖"默认值恰好是多少"。
            //    模拟器不含静摩擦模型，纯 P 就能到位，不必给 u0。
            link.Send(Proto.BuildCommand(Cmd.SetParam, 19, 200f, 3));      // steer_kp = 200
            link.Send(Proto.BuildCommand(Cmd.SetParam, 18, 20f,  4));      // steer_tgt = 20 度
            WaitWithHeartbeat(link, 0.4);
            Check(Math.Abs(store.Latest(21)) > 1f, "转向开始运动",
                  "实际 " + store.Latest(21).ToString("F1"));
            Check(Math.Abs(store.Latest(22)) > 1f, "转向输出非零",
                  "实际 " + store.Latest(22).ToString("F0"));

            WaitWithHeartbeat(link, 1.6);
            Check(Math.Abs(store.Latest(21) - 20f) < 3f, "转角收敛到目标 20 度",
                  "实际 " + store.Latest(21).ToString("F2"));
            // raw = 2157 - 20 * 50.08 ≈ 1155
            Check(Math.Abs(store.Latest(23) - 1155f) < 80f, "原始角 ≈ 2157 - 20*50.08",
                  "实际 " + store.Latest(23).ToString("F0"));
            Check(store.Latest(25) > 0.5f, "steer_status 自检位 = 1",
                  "实际 " + store.Latest(25).ToString("F0"));

            // ④ 关内环 → 输出归 0
            link.Send(Proto.BuildCommand(Cmd.SetParam, 23, 0f, 4));        // steer_en = 0（ID 23）
            WaitWithHeartbeat(link, 0.6);
            Check(Math.Abs(store.Latest(22)) < 0.5f, "关内环后转向输出归 0",
                  "实际 " + store.Latest(22).ToString("F1"));

            link.Close();
        }

        sim.Stop();
        sim.Dispose();

        // ---- 界面：新档案在真实 DPI 下也不能剪字 / 越界 / 互相压 ----
        Ui.MainForm f = null;
        try
        {
            f = new Ui.MainForm();                 // 构造时 Current = SteerLoop → 建的就是新档案的界面
            f.CreateControl();
            _ = f.Handle;
            f.PerformLayout();
            f.ApplyRuntimeLayoutForTest();

            var issues = LayoutAudit.Walk(f);
            var clipped = issues.Where(i => i.Kind == "剪字").ToList();
            var overflow = issues.Where(i => i.Kind.EndsWith("越界")).ToList();
            var overlap = issues.Where(i => i.Kind == "重叠").ToList();
            Check(clipped.Count == 0, "转向档案界面无剪字",
                  clipped.Count + " 处：" + string.Join(" ; ", clipped.Take(3).Select(i => i.Detail)));
            Check(overflow.Count == 0, "转向档案界面无越界",
                  overflow.Count + " 处：" + string.Join(" ; ", overflow.Take(3).Select(i => i.Detail)));
            Check(overlap.Count == 0, "转向档案界面无控件互相压",
                  overlap.Count + " 处：" + string.Join(" ; ", overlap.Take(3).Select(i => i.Detail)));
        }
        catch (Exception ex) { Check(false, "转向档案界面构建 / 布局体检", ex.GetType().Name + ": " + ex.Message); }
        finally { try { f?.Dispose(); } catch { } }

        // ---- 切回全车档案，别把后面的界面测试带歪 ----
        Defaults.SetProfile(HostProfile.Kart);
        Check(Defaults.ChannelNames.Length == 32, "切回卡丁车档案 32 通道", "实际 " + Defaults.ChannelNames.Length);
    }

    // ==================================================================
    // 惯导 + 路表档案（工程 8.ins_route_cyt4bb7，C 车）
    //
    // 除了通道 / 参数表对齐，这里钉死三件**只有跑起来才看得见**的事：
    //   ① 位姿是**真的在积分**（拖速度滑条 x/y/dist 会动，静止时不动）；
    //   ② 差速 ⇒ yaw 真的在转（不是画一条假曲线）；
    //   ③ 三个"边沿命令"（归零 / 录路表 / 磁标定）只在 0→1 那一拍动手。
    // ==================================================================
    private static void TestInsRouteProfile()
    {
        Section("惯导+路表档案（32 通道 + 34 项参数表）");

        Defaults.SetProfile(HostProfile.InsRoute);

        string[] wantName =
        {
            "target_l", "v_l",      "u_l",      "target_r", "v_r",      "u_r",
            "uff_l",    "uff_r",    "z_l",      "z_r",
            "pos_l",    "pos_r",    "dlt_l",    "dlt_r",
            "glitch_l", "glitch_r", "online",   "rx_age",   "enable",   "safety",
            "srv_norm", "srv_duty", "srv_en",
            "ins_roll", "ins_pitch","ins_gz",
            "ins_x",    "ins_y",    "ins_yaw",  "ins_dist", "ins_flags","ins_seq",
        };
        string[] wantUnit =
        {
            "rps",   "rps",  "%",     "rps",   "rps",   "%",
            "%",     "%",    "rps·s", "rps·s",
            "cnt",   "cnt",  "cnt",   "cnt",
            "-",     "-",    "-",     "ms",    "-",     "-",
            "-",     "duty", "-",
            "°",     "°",    "°/s",
            "m",     "m",    "°",     "m",     "-",     "-",
        };

        Check(Defaults.ChannelNames.Length == 32, "通道数 = 32", "实际 " + Defaults.ChannelNames.Length);
        Check(Defaults.ChannelUnits.Length == 32, "单位表长度 = 32", "实际 " + Defaults.ChannelUnits.Length);

        bool nameOk = true, unitOk = true; string bad = "";
        for (int i = 0; i < wantName.Length; i++)
        {
            if (Defaults.ChannelName(i) != wantName[i]) { nameOk = false; bad = $"ch{i} 名 = {Defaults.ChannelName(i)}"; break; }
            if (Defaults.ChannelUnit(i) != wantUnit[i]) { unitOk = false; bad = $"ch{i} 单位 = {Defaults.ChannelUnit(i)}"; break; }
        }
        Check(nameOk, "通道名与车端 CH_* 逐项对应", bad);
        Check(unitOk, "通道单位与上表逐项一致", bad);

        // ★ 前 20 个下标必须与「双电机速度环」一字不差（三个工程共用同一段速度环通道）。
        //   ⚠ 但 20~25 与「转向闭环」**语义不同**（那边是转向位置环，这边是舵机 + 惯导）——
        //     这是有意为之，不是错位；下面单独断言一次，防止以后有人"顺手对齐"。
        string[] dual20 =
        {
            "target_l", "v_l", "u_l", "target_r", "v_r", "u_r",
            "uff_l", "uff_r", "z_l", "z_r",
            "pos_l", "pos_r", "dlt_l", "dlt_r",
            "glitch_l", "glitch_r", "online", "rx_age", "enable", "safety",
        };
        bool prefixOk = true;
        for (int i = 0; i < 20 && prefixOk; i++)
            if (Defaults.ChannelName(i) != dual20[i]) prefixOk = false;
        Check(prefixOk, "前 20 个通道与双电机档案完全一致（老下标没动）");
        Check(Defaults.ChannelName(20) == "srv_norm" && Defaults.ChannelName(26) == "ins_x",
              "20~31 用的是舵机+惯导语义（与转向闭环档案不同，属有意）");

        Check(Defaults.ProfileDisplayName == "惯导+路表", "档案显示名 = 惯导+路表", Defaults.ProfileDisplayName);
        Check(Defaults.ProfileIndex == 4, "下拉框下标 = 4", "实际 " + Defaults.ProfileIndex);
        Check(Defaults.IsInsRoute && !Defaults.IsSteerLoop && !Defaults.IsDualMotor && !Defaults.IsMotor,
              "IsInsRoute 与其余三个闭环档案互斥");
        Check(Defaults.IsClosedLoop, "归入闭环类档案（50 Hz 固定遥测）");

        bool presetIdxOk = true;
        foreach (var preset in Defaults.PlotPresets)
            foreach (int c in preset) if (c < 0 || c >= Defaults.ChannelNames.Length) presetIdxOk = false;
        Check(presetIdxOk, "分图预设下标都落在 0~31");
        Check(Defaults.PlotPresets.Length == Defaults.PlotPresetNames.Length,
              "分图预设与名字数量一致", $"{Defaults.PlotPresets.Length}/{Defaults.PlotPresetNames.Length}");

        bool visOk = true;
        foreach (int c in Defaults.DefaultVisible) if (c < 0 || c >= Defaults.ChannelNames.Length) visOk = false;
        Check(visOk, "默认可见通道下标都落在 0~31");

        // ---- 参数表：与车端 ID 逐项对齐（错位 = 拖错滑条）----
        var ps = Defaults.BuildDefaultParams();
        Check(ps.Count == 39, "参数表 39 项（ID 0~38；34~37 舵机工作点、38 记录采样周期）",
              "实际 " + ps.Count);

        bool idSeq = true;
        for (int i = 0; i < ps.Count; i++) if (ps[i].Id != i) idSeq = false;
        Check(idSeq, "参数 ID 从 0 连续递增（错位会让滑条写错参数）");

        string[] wantParamName =
        {
            "target_l","target_r","duty_limit","target_max",
            "k_v","k_z","inv_K","u0",
            "cpr_l","cpr_r","inv_l","inv_r","minv_l","minv_r",
            "lost_ms","auto_stop","slew","enc_win","u_lim","use_kf",
            "srv_en","srv_norm",
            "ins_mpm","ins_odom_en","ins_slip","ins_route","ins_zero","ins_magcal",
            "crs_kp","crs_kp2","crs_kd","crs_ufull","crs_speed","crs_sign",
        };
        bool pNameOk = true; string pBad = "";
        for (int i = 0; i < wantParamName.Length; i++)
            if (ps[i].Name != wantParamName[i]) { pNameOk = false; pBad = $"ID{i} = {ps[i].Name}"; break; }
        Check(pNameOk, "参数名与车端 MOTOR_PARAM_TABLE 逐项对应", pBad);

        // 默认值必须与车端一致（★ 这几个是"差一点就不对"的关键值）
        void ChkDef(int id, float want, string what)
        {
            float got = ps[id].Def;
            Check(Math.Abs(got - want) < Math.Max(1e-6f, Math.Abs(want) * 1e-4f),
                  $"默认值 {what} = {want}", "实际 " + got);
        }
        ChkDef(4,  0.014f,    "k_v（C 车 20.zf_clean）");
        ChkDef(5,  0.070f,    "k_z（C 车）");
        ChkDef(6,  0.004651f, "inv_K（1/215，C 车）");
        ChkDef(7,  0.053f,    "u0（C 车）");
        ChkDef(9,  1024f,     "cpr_r（★ CYT4BB7 正交是 X1，不是 MM32 的 4096）");
        ChkDef(20, 0f,        "srv_en（默认 0 = 舵机锁中位）");
        ChkDef(22, 0.0895f,   "ins_mpm（2026-10-11 实测定稿 EncCal 12 次合并均值 0.0895）");

        // 上下限也必须装得下默认值，否则滑条一碰就把值夹到边界
        bool rangeOk = true;
        foreach (var p in ps) if (p.Def < p.Lo || p.Def > p.Hi) rangeOk = false;
        Check(rangeOk, "每项默认值都落在 [下限, 上限] 内");

        // ---- 端到端：跑模拟车端，验"位姿真在动、静止时不动、边沿命令只动一次" ----
        var store = new ChannelStore();
        int frames = 0;

        using var sim = new Simulator();
        using var link = new Link();

        link.FrameReceived += (t, b, o) =>
        {
            if (t != (byte)FrameType.Telemetry) return;
            int cnt = Proto.GetU16(b, o + 8);
            var ch = new float[cnt];
            for (int i = 0; i < cnt; i++) ch[i] = Proto.GetF32(b, o + 10 + i * 4);
            store.Ensure(cnt);
            store.Append(ch, cnt, 1.0, Proto.GetU16(b, o + 6));
            frames++;
        };

        bool up = false;
        sim.Start();
        Thread.Sleep(150);
        try { link.ConnectTcp("127.0.0.1", sim.Port); up = true; }
        catch (Exception ex) { Check(false, "连接模拟车端（惯导）", ex.Message); }

        if (up)
        {
            WaitWithHeartbeat(link, 0.6);
            Check(frames >= 10, "收到 ≥10 帧遥测", "实际 " + frames);
            Check(store.ChannelCount == 32, "帧内通道数 = 32", "实际 " + store.ChannelCount);

            // ① 未使能：位姿不该动（否则就是模拟器在拿随机数糊曲线）
            float x0 = store.Latest(26), y0 = store.Latest(27), d0 = store.Latest(29);
            WaitWithHeartbeat(link, 0.6);
            Check(Math.Abs(store.Latest(26) - x0) < 1e-3f &&
                  Math.Abs(store.Latest(27) - y0) < 1e-3f &&
                  Math.Abs(store.Latest(29) - d0) < 1e-3f,
                  "未使能时位姿与里程都不动",
                  $"Δx={store.Latest(26) - x0:F4} Δy={store.Latest(27) - y0:F4} Δd={store.Latest(29) - d0:F4}");

            // ② ins_seq 应该匀速增长（丢帧会在这里露出来）
            float seqA = store.Latest(31);
            WaitWithHeartbeat(link, 0.6);
            Check(store.Latest(31) > seqA, "ins_seq 在增长", $"{seqA:F0} -> {store.Latest(31):F0}");

            // ③ 使能 + 给"等速直行"→ 里程/y 增长、yaw 基本不变
            //    ★ 基线必须在 wait **之前**取：位姿是每拍在涨的，wait 之后再取基线等于跟自己比。
            float dBefore   = store.Latest(29);
            float yBefore   = store.Latest(27);
            float xBefore   = store.Latest(26);
            float yawBefore = store.Latest(28);

            link.Send(Proto.BuildCommand(Cmd.SetParam, 22, 0.2f, 100));   // ins_mpm = 0.2 m/rev
            link.Send(Proto.BuildCommand(Cmd.SetParam, 0,  6f,   101));   // target_l = 6 rps
            link.Send(Proto.BuildCommand(Cmd.SetParam, 1,  6f,   102));   // target_r = 6 rps
            link.Send(Proto.BuildCommand(Cmd.SetEnable, 0,
                                         (ushort)(EnableBits.Master | EnableBits.Motor), 103));
            WaitWithHeartbeat(link, 1.2);

            Check(store.Latest(29) > dBefore + 0.05f, "直行：里程在增长",
                  $"{dBefore:F3} -> {store.Latest(29):F3} m");
            Check(store.Latest(27) > yBefore + 0.05f, "直行：y（向前）在增长",
                  $"{yBefore:F3} -> {store.Latest(27):F3} m");
            Check(Math.Abs(store.Latest(28) - yawBefore) < 8f, "直行：yaw 基本不变",
                  $"实际 Δyaw = {store.Latest(28) - yawBefore:F2} 度");
            Check(Math.Abs(store.Latest(26) - xBefore) < 0.05f, "直行：x（横向）没跑偏",
                  $"实际 Δx = {store.Latest(26) - xBefore:F4} m");

            // ④ 差速 ⇒ yaw 真的转起来
            link.Send(Proto.BuildCommand(Cmd.SetParam, 0, 5f, 110));      // target_l = 5
            link.Send(Proto.BuildCommand(Cmd.SetParam, 1, 7f, 111));      // target_r = 7
            WaitWithHeartbeat(link, 0.8);
            Check(Math.Abs(store.Latest(28) - yawBefore) > 5f, "差速：yaw 明显变化",
                  $"Δyaw = {store.Latest(28) - yawBefore:F2} 度");
            Check(Math.Abs(store.Latest(25)) > 1f, "差速：陀螺 Z 通道非零",
                  $"实际 gz = {store.Latest(25):F2} °/s");

            // ⑤ 舵机：使能 + 给舵量 → duty 偏离中位 2420，符号正确（正 = 左 = 增大）
            link.Send(Proto.BuildCommand(Cmd.SetParam, 20, 1f,   120));   // srv_en = 1
            link.Send(Proto.BuildCommand(Cmd.SetParam, 21, 0.5f, 121));   // srv_norm = +0.5
            WaitWithHeartbeat(link, 0.5);
            Check(store.Latest(22) > 0.5f, "srv_en 通道 = 1", "实际 " + store.Latest(22).ToString("F0"));
            Check(Math.Abs(store.Latest(21) - 2540f) < 20f, "舵机 duty = 2420 + 0.5×240 = 2540",
                  "实际 " + store.Latest(21).ToString("F0"));
            link.Send(Proto.BuildCommand(Cmd.SetParam, 21, -0.5f, 122));  // srv_norm = -0.5
            WaitWithHeartbeat(link, 0.4);
            Check(Math.Abs(store.Latest(21) - 2290f) < 20f, "右打 duty = 2420 − 0.5×260 = 2290",
                  "实际 " + store.Latest(21).ToString("F0"));
            link.Send(Proto.BuildCommand(Cmd.SetParam, 20, 0f, 123));     // srv_en = 0
            WaitWithHeartbeat(link, 0.4);
            Check(Math.Abs(store.Latest(21) - 2420f) < 5f, "关使能后回中位 2420",
                  "实际 " + store.Latest(21).ToString("F0"));

            // ⑥ 路表录制：0→1 开始（bit4 亮），1→0 停止
            link.Send(Proto.BuildCommand(Cmd.SetParam, 25, 1f, 130));     // ins_route = 1
            WaitWithHeartbeat(link, 0.8);
            Check(((int)store.Latest(30) & 0x10) != 0, "录制中：flags bit4 = 1",
                  "flags = 0x" + ((int)store.Latest(30)).ToString("X2"));
            Check(sim.InsRouteRunning && sim.InsRoutePointCount > 1,
                  "模拟车端真在录路表（点数 > 1）",
                  $"点数 = {sim.InsRoutePointCount}, running = {sim.InsRouteRunning}");
            link.Send(Proto.BuildCommand(Cmd.SetParam, 25, 0f, 131));     // ins_route = 0
            WaitWithHeartbeat(link, 0.5);
            Check(((int)store.Latest(30) & 0x10) == 0, "停录后 flags bit4 = 0",
                  "flags = 0x" + ((int)store.Latest(30)).ToString("X2"));

            // ⑥ ★ 读路表（Cmd.RouteDump）：车端分帧 dump → 上位机按固定偏移解析。
            //    这条用例锁的是【跨语言字节布局】：车端是 C 的 packed 结构体，
            //    上位机是 C# 按偏移读 —— 两边对不上时【不会报错，只会画出一条乱线】。
            {
                var recs = new SortedDictionary<int, RouteProto.Rec>();
                RouteProto.Head hd = default;
                bool haveHead = false;
                int endCount = -1;
                int nBad = 0;

                void OnRt(byte t, byte[] bb, int oo)
                {
                    if (t == (byte)FrameType.RouteHead) { hd = RouteProto.ReadHead(bb, oo); haveHead = true; }
                    else if (t == (byte)FrameType.RouteData)
                    {
                        int first = (int)Proto.GetU32(bb, oo);
                        int n = Proto.GetU16(bb, oo + 4);
                        for (int i = 0; i < n; i++)
                            recs[first + i] = RouteProto.ReadRec(bb, oo + 6 + i * RouteProto.RecSize);
                    }
                    else if (t == (byte)FrameType.RouteEnd) { endCount = (int)Proto.GetU32(bb, oo); }
                    else nBad++;
                }

                link.FrameReceived += OnRt;
                link.Send(Proto.BuildCommand(Cmd.RouteDump, 0, 0f, 160));
                WaitWithHeartbeat(link, 0.8);
                link.FrameReceived -= OnRt;

                int want = sim.InsRoutePointCount;
                Check(want >= 2, "模拟车端确实录到了路表（前置条件）", "点数 " + want);
                Check(haveHead, "收到路表表头帧 RouteHead(0x10)");
                Check(hd.Magic2 == RouteProto.Magic, $"magic = 0x{RouteProto.Magic:X8}",
                      $"实际 0x{hd.Magic2:X8}");
                Check(hd.Ver2 == RouteProto.Ver && hd.RecLen == RouteProto.RecSize,
                      $"ver/rec_len = {RouteProto.Ver}/{RouteProto.RecSize}", $"实际 {hd.Ver2}/{hd.RecLen}");
                Check((int)hd.Count == want, "表头声明点数 = 车端点数", $"声明 {hd.Count} vs 车端 {want}");
                Check(recs.Count == want, "收齐了全部点（无缺号）", $"收到 {recs.Count} / 应有 {want}");
                Check(endCount == want, "结束帧点数一致", $"实际 {endCount}");

                bool idxOk = true;
                for (int i = 0; i < want; i++) if (!recs.ContainsKey(i)) { idxOk = false; break; }
                Check(idxOk, "下标 0..N-1 连续无缺");

                bool valOk = true; string vBad = "";
                foreach (var kv in recs)
                {
                    if (Math.Abs(kv.Value.Xmm) > 32000 || Math.Abs(kv.Value.Ymm) > 32000)
                    { valOk = false; vBad = $"#{kv.Key} = ({kv.Value.Xmm},{kv.Value.Ymm}) mm"; break; }
                }
                Check(valOk, "坐标量级合理（|mm| ≤ 32000，说明偏移没错位）", vBad);

                // 首点必须在原点附近（车端第 0 点就是起点）
                Check(recs.ContainsKey(0) && Math.Abs(recs[0].Xmm) + Math.Abs(recs[0].Ymm) < 50,
                      "第 0 点在起点附近（说明 x/y 没被读错位）",
                      recs.ContainsKey(0) ? $"({recs[0].Xmm},{recs[0].Ymm}) mm" : "缺第 0 点");
            }

            // ⑦ 位姿归零：0→1 只动一次。
            //    ★ 必须先**关使能把车停住**再归零 —— 否则归零那一拍之后车还在跑，
            //      0.5s 后读到的当然不是 0（这不是固件的 bug，是用例写错了）。
            link.Send(Proto.BuildCommand(Cmd.SetEnable, 0, (ushort)EnableBits.None, 138));
            WaitWithHeartbeat(link, 1.0);                                  // 等它滑停

            Check(Math.Abs(store.Latest(26)) + Math.Abs(store.Latest(27)) > 0.05f,
                  "归零前位姿确实非零（否则这条用例没意义）",
                  $"x={store.Latest(26):F3} y={store.Latest(27):F3}");

            link.Send(Proto.BuildCommand(Cmd.SetParam, 26, 1f, 140));     // ins_zero = 1
            WaitWithHeartbeat(link, 0.5);
            Check(Math.Abs(store.Latest(26)) < 0.05f && Math.Abs(store.Latest(27)) < 0.05f,
                  "归零后 x/y 回到 0",
                  $"x={store.Latest(26):F4} y={store.Latest(27):F4}");
            Check(store.Latest(29) < 0.05f, "归零后里程也回 0",
                  "实际 " + store.Latest(29).ToString("F4"));
            link.Send(Proto.BuildCommand(Cmd.SetParam, 26, 0f, 141));     // 拖回 0，便于下次再触发

            // ⑧ 磁标定：0→1 开始、1→0 结算 → flags bit1 亮
            link.Send(Proto.BuildCommand(Cmd.SetParam, 27, 1f, 150));     // ins_magcal = 1
            WaitWithHeartbeat(link, 0.4);
            link.Send(Proto.BuildCommand(Cmd.SetParam, 27, 0f, 151));     // 结算
            WaitWithHeartbeat(link, 0.5);
            Check(((int)store.Latest(30) & 0x02) != 0, "磁标定结算后 flags bit1 = 1",
                  "flags = 0x" + ((int)store.Latest(30)).ToString("X2"));

            link.Close();
        }

        sim.Stop();
        sim.Dispose();

        // ---- 界面：新档案在真实 DPI 下也不能剪字 / 越界 / 互相压 ----
        Ui.MainForm f = null;
        try
        {
            f = new Ui.MainForm();                 // 构造时 Current = InsRoute → 建的就是新档案的界面
            f.CreateControl();
            _ = f.Handle;
            f.PerformLayout();
            f.ApplyRuntimeLayoutForTest();

            var issues = LayoutAudit.Walk(f);
            var clipped = issues.Where(i => i.Kind == "剪字").ToList();
            var overflow = issues.Where(i => i.Kind.EndsWith("越界")).ToList();
            var overlap = issues.Where(i => i.Kind == "重叠").ToList();
            Check(clipped.Count == 0, "惯导档案界面无剪字",
                  clipped.Count + " 处：" + string.Join(" ; ", clipped.Take(3).Select(i => i.Detail)));
            Check(overflow.Count == 0, "惯导档案界面无越界",
                  overflow.Count + " 处：" + string.Join(" ; ", overflow.Take(3).Select(i => i.Detail)));
            Check(overlap.Count == 0, "惯导档案界面无控件互相压",
                  overlap.Count + " 处：" + string.Join(" ; ", overlap.Take(3).Select(i => i.Detail)));
        }
        catch (Exception ex) { Check(false, "惯导档案界面构建 / 布局体检", ex.GetType().Name + ": " + ex.Message); }
        finally { try { f?.Dispose(); } catch { } }

        // ---- 切回全车档案，别把后面的界面测试带歪 ----
        Defaults.SetProfile(HostProfile.Kart);
        Check(Defaults.ChannelNames.Length == 32, "切回卡丁车档案 32 通道", "实际 " + Defaults.ChannelNames.Length);
    }

    /// <summary>
    /// 一边等一边按真实上位机那样发心跳 —— **走的就是 App 里 Heartbeat.Beat 那条路**。
    /// ★ 必须这么等：车端/模拟车端都用 lost_ms（默认 600ms）判失联，干 sleep 会让仿真掉线、
    ///   输出清零，断言就变成在测"失联保护"而不是在测目标行为。
    /// </summary>
    private static void WaitWithHeartbeat(Link link, double seconds, int hbMs = 100)
    {
        var sw = System.Diagnostics.Stopwatch.StartNew();
        ushort tok = 4000;
        while (sw.Elapsed.TotalSeconds < seconds)
        {
            Heartbeat.Beat(link, () => tok++);
            Thread.Sleep(hbMs);
        }
    }

    // ==================================================================
    // 二维轨迹图
    // ==================================================================

    /// <summary>
    /// 二维轨迹图（<see cref="Ui.BoardView"/>）：
    ///   ① **坐标约定数值验证** —— 最值钱的一条：符号反了画面"看起来也是一条轨迹"，只有基准数能抓出来；
    ///   ② 从 ChannelStore 增量拉点 + 2cm 抽稀；③ 清空；④ 真渲染一遍不抛异常。
    /// </summary>
    // ==================================================================
    // 工程 9（卡丁 · 遥控）新增：遥控档案 / 标定精简档案 / 舵机限位 / CSV 记录
    // ==================================================================
    private static void TestKartRCProfile()
    {
        Section("卡丁·遥控档案（39 通道 + 34 项参数表）");

        Defaults.SetProfile(HostProfile.KartRC);

        int n = Defaults.ChannelNames.Length;
        Check(n == 39, "通道数 = 39", "实际 " + n);
        Check(Defaults.ChannelUnits.Length == n, "单位表长度 = " + n, "实际 " + Defaults.ChannelUnits.Length);

        // ★ 0~25 必须与「转向闭环」一字不差 —— 工程 9 = 工程 7 + 遥控，老下标一个都没动。
        string[] head26 =
        {
            "target_l", "v_l",      "u_l",      "target_r", "v_r",      "u_r",
            "uff_l",    "uff_r",    "z_l",      "z_r",
            "pos_l",    "pos_r",    "dlt_l",    "dlt_r",
            "flip_l",   "flip_r",   "online",   "rx_age",   "enable",   "safety",
            "steer_tgt", "steer_delta", "steer_u", "steer_raw", "steer_en", "steer_status",
        };
        bool headOk = true; string bad = "";
        for (int i = 0; i < head26.Length && i < n; i++)
            if (Defaults.ChannelName(i) != head26[i]) { headOk = false; bad = $"ch{i} = {Defaults.ChannelName(i)}"; break; }
        Check(headOk, "0~25 与「转向闭环」一字不差", bad);

        // 26~36 遥控组，逐项点名（顺序 = 车端 motor_link.h 的 CH_RC_*）
        string[] rc =
        {
            "rc_eng",  "rc_estop", "rc_steer_deg", "rc_thr", "rc_scale", "rc_tgt_rps",
            "rc_ch1",  "rc_ch2",   "rc_ch5",       "rc_ch6", "rc_frames",
        };
        bool rcOk = true;
        for (int i = 0; i < rc.Length; i++)
            if (n < 26 + rc.Length || Defaults.ChannelName(26 + i) != rc[i]) { rcOk = false; bad = $"ch{26 + i} = {Defaults.ChannelName(26 + i)}"; break; }
        Check(rcOk, "26~36 遥控组与车端 CH_RC_* 逐项对应", bad);

        // 37~38 = 增益回读（车端 CH_K_V / CH_K_Z）。★ 这两条必须真的在通道表里：
        //   它们就是"车上真正生效的 k_v / k_z"进 CSV 的唯一通路，掉了就查不出
        //   "滑条写 0.164、车上跑 0.165"这类事，而且掉了**不报任何错**（只是少两列）。
        Check(Defaults.ChannelName(37) == "k_v" && Defaults.ChannelName(38) == "k_z",
              "37~38 = 增益回读 k_v / k_z",
              $"ch37 = {Defaults.ChannelName(37)}, ch38 = {Defaults.ChannelName(38)}");
        Check(Defaults.PlotPresetNames.Contains("增益"),
              "有一个叫「增益」的分图预设（k_v/k_z 能一键上波形）");
        Check(Defaults.DefaultVisible.Contains(37) && Defaults.DefaultVisible.Contains(38),
              "k_v / k_z 默认就勾上（一开界面就在波形里）");

        var ps = Defaults.BuildDefaultParams();
        Check(ps.Count == 34, "参数表 = 34 项", "实际 " + ps.Count);
        if (ps.Count == 34)
        {
            Check(ps[26].Name == "steer_lim" && Math.Abs(ps[26].Hi - Defaults.SteerLimitDeg) < 1e-6f,
                  "ID26 软限位上限 = ±27", $"{ps[26].Name} hi={ps[26].Hi}");
            Check(ps[27].Name == "rc_arm_en", "ID27 = rc_arm_en", ps[27].Name);
            Check(ps[28].Name == "rc_thr_max", "ID28 = rc_thr_max", ps[28].Name);
            Check(ps[29].Name == "ol_en" && ps[30].Name == "ol_duty_l" && ps[31].Name == "ol_duty_r",
                  "ID29~31 = 开环 duty 组");
            Check(ps[32].Name == "tel_sel" && ps[33].Name == "tel_period", "ID32~33 = 遥测档案/周期");
        }

        Check(IndexesInRange(n), "分图预设 / 默认可见下标都在范围内");
    }

    private static void TestCalibProfile()
    {
        Section("标定·精简档案（18 通道，车端 tel_sel = 1）");

        Defaults.SetProfile(HostProfile.Calib);

        int n = Defaults.ChannelNames.Length;
        Check(n == 18, "通道数 = 18", "实际 " + n);
        Check(Defaults.ChannelUnits.Length == n, "单位表长度 = " + n, "实际 " + Defaults.ChannelUnits.Length);

        // 通道规划：速度环(左/右 各 目标/实际/PWM) + 舵机角度环(目标/实际/PWM) + 状态 + 预留 + 增益回读
        string[] want =
        {
            "target_l", "v_l", "u_l",
            "target_r", "v_r", "u_r",
            "steer_tgt", "steer_delta", "steer_u",
            "online", "enable", "safety",
            "steer_en", "steer_st", "ol_en", "resv0",
            "k_v", "k_z",
        };
        bool ok = true; string bad = "";
        for (int i = 0; i < want.Length; i++)
            if (Defaults.ChannelName(i) != want[i]) { ok = false; bad = $"ch{i} = {Defaults.ChannelName(i)}"; break; }
        Check(ok, "通道布局与车端 CT_* 逐项对应", bad);

        Check(Defaults.IsSteerAngleParam("steer_tgt"), "steer_tgt 仍算角度类（会被夹 ±27）");

        // 三路一组的分图预设：速度环L / 速度环R / 舵机角度环 —— 阶跃响应就靠这三张图读
        Check(Defaults.PlotPresets.Length >= 3
              && Defaults.PlotPresets[0].Length == 3
              && Defaults.PlotPresets[0][0] == 0 && Defaults.PlotPresets[0][1] == 1 && Defaults.PlotPresets[0][2] == 2,
              "分图预设[0] = 速度环L（目标/实际/PWM）");
        Check(Defaults.PlotPresets[2][0] == 6 && Defaults.PlotPresets[2][2] == 8,
              "分图预设[2] = 舵机角度环（目标/实际/PWM）");
        Check(Defaults.PlotPresets.Length >= 5
              && Defaults.PlotPresets[4].Length == 2
              && Defaults.PlotPresets[4][0] == 16 && Defaults.PlotPresets[4][1] == 17,
              "分图预设[4] = 增益（车上真正生效的 k_v / k_z）");

        Check(IndexesInRange(n), "分图预设 / 默认可见下标都在范围内");

        // 两个新档案共用同一张参数表（参数表与遥测档案无关）
        var pCalib = Defaults.BuildDefaultParams();
        Defaults.SetProfile(HostProfile.KartRC);
        var pKart = Defaults.BuildDefaultParams();
        Check(pCalib.Count == pKart.Count, "标定档案与遥控档案参数表项数一致",
              $"{pCalib.Count} vs {pKart.Count}");
    }

    /// <summary>
    /// ★★ 帧 COUNT ↔ 档案 的反查（2026-10-14 现场踩坑，用户拿 `kz.csv` 来问的那次）。
    ///
    /// 现场：车端在发**全量 39 通道**，本机档案停在**「标定·精简」(18)**。
    /// 后果不是报错，而是**通道名整体错位 21 个下标**：
    ///   图例写着 `k_v` 的那条线 = ch16 = `online`（**恒 1**）；
    ///   写着 `k_z` 的那条线 = ch17 = `rx_age`（**毫秒计数器**，被上位机 100 ms 心跳周期性
    ///   清零 ⇒ 画出来是一条 0→100 的锯齿，逐行 +20）。
    /// ⇒ 用户看到"两个增益都在剧烈跳动"，而**日志里那条警告根本没被读**。
    ///
    /// 这里钉三样：
    ///   ① `ChannelCountOf()` 必须等于各档案通道表的真实长度（防两处数组各写一份而漂移）；
    ///   ② 反查表：**唯一命中**才允许"一键切过去"；32 / 20 命中两个 ⇒ 只能提示，不许猜；
    ///   ③ 用"同名不同义"的断言把那个 CSV 的真身记在代码里（下面的断言就是说明书）。
    /// </summary>
    private static void TestFrameProfileMatch()
    {
        Section("帧 COUNT ↔ 档案 反查（防「通道名整体错位」）");

        foreach (HostProfile p in Enum.GetValues<HostProfile>())
        {
            Defaults.SetProfile(p);
            int real = Defaults.ChannelNames.Length;
            Check(Defaults.ChannelCountOf(p) == real,
                  $"{Defaults.ProfileDisplayName}：ChannelCountOf = 通道表长度",
                  $"{Defaults.ChannelCountOf(p)} vs {real}");
        }

        // 唯一命中 ⇒ 可以一键切
        foreach (var pair in new[] { (39, HostProfile.KartRC), (18, HostProfile.Calib), (26, HostProfile.SteerLoop) })
        {
            var c = Defaults.ProfilesWithChannelCount(pair.Item1);
            Check(c.Count == 1 && c[0] == pair.Item2,
                  $"{pair.Item1} 通道唯一命中「{pair.Item2}」⇒ 允许一键切换",
                  c.Count == 0 ? "一个都没命中" : string.Join("/", c));
        }

        // 有歧义 ⇒ 绝不自动切
        foreach (int cnt in new[] { 32, 20 })
        {
            var c = Defaults.ProfilesWithChannelCount(cnt);
            Check(c.Count == 2, $"{cnt} 通道命中 2 个档案 ⇒ 只能提示、不能自动切",
                  $"实际 {c.Count} 个：" + string.Join("/", c));
        }

        Check(Defaults.ProfilesWithChannelCount(37).Count == 0
              && Defaults.ProfilesWithChannelCount(999).Count == 0,
              "未知通道数（37 = 老固件 / 999）命中 0 个 ⇒ 只报错不猜");

        // ★★ 同名不同义：kz.csv 里那两列的真身
        Defaults.SetProfile(HostProfile.Calib);
        string c16 = Defaults.ChannelName(16), c17 = Defaults.ChannelName(17);
        Defaults.SetProfile(HostProfile.KartRC);
        Check(c16 == "k_v" && c17 == "k_z", "「标定·精简」的 16/17 = k_v / k_z", $"{c16} / {c17}");
        Check(Defaults.ChannelName(16) == "online" && Defaults.ChannelName(17) == "rx_age",
              "★ 同样是下标 16/17，「卡丁·遥控」里是 online / rx_age —— kz.csv 的「k_v 恒 1、k_z 一路 +20」正是它们",
              $"{Defaults.ChannelName(16)} / {Defaults.ChannelName(17)}");

        // 波形上的红条：**真的画出来 + 真的是红的**才算数（不是"设了个属性就完事"）
        var store = new ChannelStore();
        var wp = new Ui.WavePanel { Store = store, Width = 900, Height = 240 };
        Bitmap bmp = null;
        try
        {
            bmp = new Bitmap(wp.Width, wp.Height);
            wp.DrawToBitmap(bmp, new Rectangle(0, 0, wp.Width, wp.Height));
            Check(wp.WarnRectForTest.IsEmpty, "WarnText 为空时不画警示条（命中矩形为空）",
                  wp.WarnRectForTest.ToString());

            wp.WarnText = "⚠ 帧里 39 通道 ≠ 本档案「标定·精简」18 通道 —— 通道名整体错位｜点此切到「卡丁·遥控」";
            wp.DrawToBitmap(bmp, new Rectangle(0, 0, wp.Width, wp.Height));

            var r = wp.WarnRectForTest;
            Check(!r.IsEmpty, "WarnText 非空时真画出了警示条（命中矩形非空）", r.ToString());

            int red = 0;
            if (!r.IsEmpty)
                for (int y = r.Top; y < r.Bottom && y < bmp.Height; y += 2)
                    for (int x = r.Left; x < r.Right && x < bmp.Width; x += 2)
                    {
                        var c = bmp.GetPixel(x, y);
                        if (c.R > 180 && c.G < 80 && c.B < 80) red++;    // Theme.Err = (232,82,82)
                    }
            Check(red >= 50, "警示条真的是红的（按 Theme.Err 逐像素数）", $"命中 {red} 像素");
        }
        catch (Exception ex) { Check(false, "波形警示条渲染", ex.GetType().Name + ": " + ex.Message); }
        finally { try { bmp?.Dispose(); } catch { } try { wp.Dispose(); } catch { } }

        Defaults.SetProfile(HostProfile.Kart);
    }

    /// <summary>
    /// 模拟车端：**每个档案都必须有自己的造数分支**。
    /// 落错分支的后果是**不对称**的：图例/数值框全错位（不抛异常，最难查），
    /// 或者对只有 18 个通道的标定档案来说直接越界（FillKart 会写到 ch[30]）。
    /// 所以这里把枚举里的每个档案都真跑一遍填充，只断言"不抛异常 + 长度对得上"。
    /// </summary>
    private static void TestSimulatorFillsAllProfiles()
    {
        Section("模拟车端：每个档案都有专用造数分支");

        foreach (HostProfile p in Enum.GetValues<HostProfile>())
        {
            Defaults.SetProfile(p);
            int n = Defaults.ChannelNames.Length;

            // ⚠⚠ 模拟器必须**按当前档案现建一个**：它的参数表 `_params` 是在 `new Simulator()`
            //    那一刻按 `Defaults.Current` 定下来的（App 里 `SwitchProfileTo()` 也是
            //    `_sim.Dispose(); _sim = new Simulator();`，同一套规矩）。
            //    拿"早先建好的那个 sim"跨档案跑，它带的还是**旧档案的参数表** ——
            //    2026-10-14 就踩到了：前一个测试把档案停在「卡丁车」，于是 sim 里压根没有
            //    `k_v` / `k_z`，`Prm("k_v")` 返回 0，「增益回读非 0」两条断言红了（**代码是对的**）。
            //    ⇒ 老老实实一个档案建一个，顺便把"换档案必须重建模拟器"这条也测了。
            var sim = new Simulator();
            try
            {
                var buf = sim.TestFillOnce();
                Check(buf.Length == n, $"{Defaults.ProfileDisplayName}：造出 {n} 个通道", "实际 " + buf.Length);
            }
            catch (Exception ex)
            {
                Check(false, $"{Defaults.ProfileDisplayName}：造数不抛异常", ex.GetType().Name + ": " + ex.Message);
            }
            finally { try { sim.Dispose(); } catch { } }
        }

        // 卡丁·遥控：26~36 的遥控组必须是 0 —— 模拟器不模拟接收机，
        // 编一组"看起来在接管"的假值比不填更容易误导现场。
        // ⚠ 只查 26~36：37/38 是**增益回读**，不是遥控组，它们必须非 0（否则这条断言
        //   会把"回读挂了"跟"遥控没插"混成一件事）。
        Defaults.SetProfile(HostProfile.KartRC);
        var simRc = new Simulator();
        var kartRc = simRc.TestFillOnce();
        Check(kartRc.Skip(26).Take(11).All(v => v == 0f), "遥控组 26~36 全 0（模拟器没有接收机）");
        Check(kartRc[37] > 0f && kartRc[38] > 0f,
              "增益回读 37/38 非 0（模拟器回读的是参数面板里的 k_v / k_z）",
              $"k_v = {kartRc[37]}, k_z = {kartRc[38]}");
        simRc.Dispose();

        // 标定档案也要有增益回读 —— 落地阶跃标定录的就是这一档，缺了它算出来的 k_v 没法对账。
        Defaults.SetProfile(HostProfile.Calib);
        var simCal = new Simulator();
        var calibBuf = simCal.TestFillOnce();
        Check(calibBuf.Length >= 18 && calibBuf[16] > 0f && calibBuf[17] > 0f,
              "标定档案 16/17 = 增益回读且非 0",
              $"k_v = {(calibBuf.Length > 16 ? calibBuf[16] : float.NaN)}, "
            + $"k_z = {(calibBuf.Length > 17 ? calibBuf[17] : float.NaN)}");
        simCal.Dispose();

        Defaults.SetProfile(HostProfile.Kart);
    }

    /// <summary>
    /// 「实时值」那 6 格数值框：**每个档案一张取数表**（<see cref="Defaults.ValueBoxes"/>）。
    ///
    /// 为什么单钉这一段 —— 2026-10-13 现场抓到的错位：
    /// 「卡丁·遥控」档案落到了"卡丁车"那份老下标 {15,20,22,13,0,30} 上，于是
    /// "速度"格显示 21.0 m/s（其实是 ch15 = flip_r，右后轮**换向次数**）、
    /// "控制周期"格显示 200.0 µs（其实是 ch30 = rc_scale，遥控油门行程 200‰）。
    /// **下标不越界、单位也对得上，只有含义是错的** —— 不测就查不出来。
    /// </summary>
    private static void TestValueBoxes()
    {
        Section("「实时值」数值框：每档案一张取数表");

        foreach (HostProfile p in Enum.GetValues<HostProfile>())
        {
            Defaults.SetProfile(p);
            int n = Defaults.ChannelNames.Length;
            var vb = Defaults.ValueBoxes();
            string who = Defaults.ProfileDisplayName;

            Check(vb.Keys.Length == 6 && vb.Caps.Length == 6 && vb.Units.Length == 6,
                  $"{who}：下标/标签/单位三个数组都是 6 项",
                  $"{vb.Keys.Length}/{vb.Caps.Length}/{vb.Units.Length}");
            Check(vb.Keys.All(k => k >= 0 && k < n), $"{who}：6 个下标都在本档案通道表内",
                  string.Join("/", vb.Keys));
            Check(vb.Caps.All(c => !string.IsNullOrWhiteSpace(c)), $"{who}：标签都不为空");

            // 除"卡丁车"（全车关注点：速度/航向/位姿误差）与"单电机"（看占空比/编码器）外，
            // 第 1 格必须是**真的实测转速/位姿**，不能是标志位或计数器 ——
            // 这一条正是"错位"最典型的形态：格子里跑的是别的档案的下标。
            if (p is HostProfile.DualMotor or HostProfile.SteerLoop or HostProfile.InsRoute
                  or HostProfile.KartRC or HostProfile.Calib)
            {
                string first = Defaults.ChannelName(vb.Keys[0]);
                Check(first is "v_l" or "ins_x",
                      $"{who}：第 1 格是实测转速/位姿，不是标志位或计数器",
                      $"{vb.Caps[0]} → ch{vb.Keys[0]} = {first} [{vb.Units[0]}]");
            }
        }

        // 标定·精简：取数与分图预设同口径（左轮 实测/目标 → 右轮 → 舵机）
        Defaults.SetProfile(HostProfile.Calib);
        var names = Defaults.ValueBoxes().Keys.Select(Defaults.ChannelName).ToArray();
        string[] want = { "v_l", "target_l", "v_r", "target_r", "steer_delta", "steer_tgt" };
        Check(names.SequenceEqual(want), "标定档案取数 = 三路一组（左 / 右 / 舵机）",
              string.Join(",", names));

        Defaults.SetProfile(HostProfile.Kart);
    }

    /// <summary>
    /// 转向保护锁存位的解读。
    ///
    /// 2026-10-13 现场：上位机给 10.5° 指令，前电机一动不动，而车端遥测 `steer_status = 5`
    /// —— 5 = 0b101 = **bit0 自检通过 + fault 0x02 顶死/反转锁存**，车端已经把输出钉 0。
    /// 这种"数字看着正常、含义是故障"的东西必须由上位机翻成人话，位序更不能搞反
    /// （车端是 `self_check_ok | (fault &lt;&lt; 1)`，把 fault 当成 bit0 就会把"5"读成"两个故障"）。
    /// </summary>
    private static void TestSteerFaultDecode()
    {
        Section("转向保护锁存位的解读（steer_status）");

        Check(Ui.MainForm.SteerFaultText(0) == "", "steer_status=0（自检没过但无锁存）→ 不报",
              Ui.MainForm.SteerFaultText(0));
        Check(Ui.MainForm.SteerFaultText(1) == "", "steer_status=1（bit0 自检通过）→ 正常，不报",
              Ui.MainForm.SteerFaultText(1));
        Check(Ui.MainForm.SteerFaultText(3).Contains("锁存") && Ui.MainForm.SteerFaultText(3).Contains("钉 0"),
              "steer_status=3 → fault 0x01 反馈失效：**锁存**且输出已钉 0",
              Ui.MainForm.SteerFaultText(3));
        Check(Ui.MainForm.SteerFaultText(5).Contains("顶死") && Ui.MainForm.SteerFaultText(5).Contains("告警")
              && !Ui.MainForm.SteerFaultText(5).Contains("钉 0"),
              "steer_status=5 → fault 0x02 顶死：**只告警、不停输出**（2026-10-13 改，别再写成锁存）",
              Ui.MainForm.SteerFaultText(5));
        Check(Ui.MainForm.SteerFaultText(7).Contains("锁存") && Ui.MainForm.SteerFaultText(7).Contains("顶死"),
              "steer_status=7 → 锁存故障 + 顶死告警都在", Ui.MainForm.SteerFaultText(7));
    }

    private static bool IndexesInRange(int n)
    {
        foreach (var g in Defaults.PlotPresets)
            foreach (int c in g) if (c < 0 || c >= n) return false;
        foreach (int c in Defaults.DefaultVisible) if (c < 0 || c >= n) return false;
        return true;
    }

    private static void TestSteerLimitClamp()
    {
        Section("舵机限位 ±27°（上位机侧夹紧，车端再夹一次）");

        Check(Math.Abs(Defaults.SteerLimitDeg - 27f) < 1e-6f, "限位常量 = 27",
              Defaults.SteerLimitDeg.ToString("0.##"));
        Check(Math.Abs(Defaults.ClampSteerDeg(30f) - 27f) < 1e-6f, "30° -> 27°",
              Defaults.ClampSteerDeg(30f).ToString("0.##"));
        Check(Math.Abs(Defaults.ClampSteerDeg(-32f) + 27f) < 1e-6f, "-32° -> -27°",
              Defaults.ClampSteerDeg(-32f).ToString("0.##"));
        Check(Math.Abs(Defaults.ClampSteerDeg(12.5f) - 12.5f) < 1e-6f, "12.5° 原样通过");
        Check(Defaults.IsSteerAngleParam("steer_tgt") && Defaults.IsSteerAngleParam("steer_lim"),
              "steer_tgt / steer_lim 属于角度类");
        Check(!Defaults.IsSteerAngleParam("steer_kp"), "steer_kp 不属于角度类");

        Defaults.SetProfile(HostProfile.KartRC);
        var ps = Defaults.BuildDefaultParams();
        foreach (var p in ps)
        {
            if (p.Name == "steer_tgt") Check(Math.Abs(p.Hi - 27f) < 1e-6f, "参数表 steer_tgt 上限 = 27", p.Hi.ToString("0.##"));
            if (p.Name == "steer_lim") Check(Math.Abs(p.Hi - 27f) < 1e-6f, "参数表 steer_lim 上限 = 27", p.Hi.ToString("0.##"));
        }
    }

    private static void TestCsvRecorder()
    {
        Section("CSV 记录模块（时间戳 / 表头 / 抽稀 / 标记 / 收尾）");

        Defaults.SetProfile(HostProfile.Calib);

        var store = new ChannelStore();
        store.Ensure(Defaults.ChannelNames.Length);
        var ch = new float[Defaults.ChannelNames.Length];
        for (int i = 0; i < ch.Length; i++) ch[i] = i + 0.5f;
        store.Append(ch, ch.Length, 0.0, 0xFFFF);

        string path = Path.Combine(Path.GetTempPath(), "karthost_csv_test.csv");
        try { File.Delete(path); } catch { }

        var rec = new CsvRecorder(store);
        bool started = rec.Start(path, 20, Enumerable.Range(0, 4).ToList(), 1000.0, out string err);
        Check(started, "Start 成功", err);
        Check(rec.IsRecording, "IsRecording = true");

        rec.Tick(1000.0);        // 第一行
        rec.Tick(1005.0);        // 不足一个周期 -> 不落盘
        rec.Tick(1030.0);        // 第二行
        Check(rec.Rows == 2, "按周期抽稀（5ms 那次不该落盘）", "实际 " + rec.Rows);

        rec.Note(1040.0, "step_apply A=15");
        long total = rec.Stop();
        Check(total == 3, "Stop 返回 3 行", "实际 " + total);
        Check(!rec.IsRecording, "Stop 后 IsRecording = false");

        var lines = File.ReadAllLines(path);
        Check(lines.Length == 4, "文件 = 表头 + 3 行", "实际 " + lines.Length);
        Check(lines.Length > 0 && lines[0].StartsWith("t_ms,note,"), "表头以 t_ms,note 开头", lines.Length > 0 ? lines[0] : "");
        Check(lines.Length > 0 && lines[0].Contains("v_l[rps]"), "表头带「通道名[单位]」");
        Check(lines.Length > 1 && lines[1].Split(',').Length == 6, "数据列数 = 2 + 选择记录的几个通道",
              lines.Length > 1 ? lines[1].Split(',').Length.ToString() : "");
        Check(lines.Length > 1 && lines[1].Contains("0.5"), "第一行含 ch0 的值 0.5");
        Check(Array.Exists(lines, l => l.Contains("step_apply A=15")), "note 标记落进 CSV");

        try { File.Delete(path); } catch { }

        // 收尾：恢复默认档案，别影响后面的用例
        Defaults.SetProfile(HostProfile.Kart);
    }

    /// <summary>
    /// 波形快照导出：**屏幕上勾了什么，导出来的就是什么**。
    /// 这条用例存在的意义 = 把"看截图读波形"这条路彻底堵死：
    /// 以后要分析就导 CSV，数字从表里来。
    /// </summary>
    private static void TestCsvExport()
    {
        Section("波形快照导出（勾选通道 → CSV：表头 / 行数 / 时间轴 / 空值）");

        Defaults.SetProfile(HostProfile.Calib);

        var store = new ChannelStore();
        store.Ensure(8);
        var ch = new float[8];
        const int N = 25;
        for (int k = 0; k < N; k++)
        {
            for (int c = 0; c < 8; c++) ch[c] = c * 100 + k;   // 每列一条线性斜坡，值一眼能认
            store.Append(ch, 8, 10.0 + k * 0.02, 0xFFFF);      // 20 ms 一点
        }

        string path = Path.Combine(Path.GetTempPath(), "karthost_csv_export_test.csv");
        try { File.Delete(path); } catch { }

        int rows = CsvExport.Dump(path, store, new[] { 3, 1 }, out string info);
        Check(rows == N, "行数 = 缓冲里的点数（两个通道不重复计）", "实际 " + rows);
        Check(info.Contains("25 点"), "info 报了点数与跨度", info);

        var raw = File.ReadAllBytes(path);
        Check(raw.Length > 3 && raw[0] == 0xEF && raw[1] == 0xBB && raw[2] == 0xBF,
              "带 UTF-8 BOM（Excel 打开中文表头不乱码）");

        var lines = File.ReadAllLines(path);
        Check(lines.Length == N + 1, "文件 = 表头 + 25 行", "实际 " + lines.Length);

        string want = "t_ms,note," + Defaults.ChannelName(1) + "[" + Defaults.ChannelUnit(1) + "],"
                                     + Defaults.ChannelName(3) + "[" + Defaults.ChannelUnit(3) + "]";
        Check(lines[0] == want, "表头与 CsvRecorder 同格式，且**按通道号升序**（不管传进来的顺序）", lines[0]);

        var f0 = lines[1].Split(',');
        Check(f0.Length == 4, "数据列数 = 2 + 勾选的 2 个通道", f0.Length.ToString());
        Check(f0[0] == "0", "第一行 t_ms = 0（相对第一点）", f0[0]);
        Check(lines[N].Split(',')[0] == "480", "最后一行 t_ms = 24 × 20 = 480", lines[N].Split(',')[0]);
        Check(f0[1] == "", "快照的 note 列恒为空", "'" + f0[1] + "'");
        Check(f0[2] == "100" && f0[3] == "300", "第一行 = ch1/ch3 的起点值", lines[1]);
        Check(lines[N].Split(',')[2] == "124" && lines[N].Split(',')[3] == "324",
              "最后一行 = ch1/ch3 的终点值", lines[N]);

        // NaN → 空单元格（写 "NaN" 会被 Excel 当文本，后面的列全乱）
        var store2 = new ChannelStore();
        store2.Ensure(4);
        store2.Append(new[] { 1f, float.NaN, 3f, 4f }, 4, 0.0, 0xFFFF);
        store2.Append(new[] { 2f, 5f, 6f, 7f }, 4, 0.02, 0xFFFF);
        string p2 = Path.Combine(Path.GetTempPath(), "karthost_csv_export_nan.csv");
        CsvExport.Dump(p2, store2, new[] { 1 }, out _);
        var l2 = File.ReadAllLines(p2);
        Check(l2.Length == 3 && l2[1].Split(',')[2] == "" && l2[2].Split(',')[2] == "5",
              "NaN 写成空单元格", l2.Length > 2 ? l2[1] + " | " + l2[2] : "");

        // 缓冲为空：给一句人话，别写出半个文件
        var empty = new ChannelStore();
        empty.Ensure(4);
        string p3 = Path.Combine(Path.GetTempPath(), "karthost_csv_export_empty.csv");
        Check(CsvExport.Dump(p3, empty, new[] { 0 }, out string e2) == 0 && e2.Contains("还没有数据"),
              "缓冲为空 → 返回 0 + 一句人话", e2);
        Check(!File.Exists(p3), "缓冲为空时不产生空文件");

        try { File.Delete(path); } catch { }
        try { File.Delete(p2); } catch { }

        Defaults.SetProfile(HostProfile.Kart);
    }

    private static void TestBoardView()
    {
        Section("二维轨迹图 BoardView（坐标约定 / 增量拉点 / 抽稀 / 渲染）");

        // ---- ① 航向约定：与车端 ins_odom.c 的 dx=-sin(yaw), dy=+cos(yaw) 逐值对齐 ----
        var h0 = Ui.BoardView.Heading(0f);
        var h90 = Ui.BoardView.Heading(90f);
        var h180 = Ui.BoardView.Heading(180f);
        var h270 = Ui.BoardView.Heading(270f);
        Check(Math.Abs(h0.dx) < 1e-5f && Math.Abs(h0.dy - 1f) < 1e-5f,
              "Heading(0°)   = (0,+1) 朝北", $"实际 ({h0.dx:0.#####},{h0.dy:0.#####})");
        Check(Math.Abs(h90.dx + 1f) < 1e-5f && Math.Abs(h90.dy) < 1e-5f,
              "Heading(90°)  = (-1,0) 朝西", $"实际 ({h90.dx:0.#####},{h90.dy:0.#####})");
        Check(Math.Abs(h180.dx) < 1e-5f && Math.Abs(h180.dy + 1f) < 1e-5f,
              "Heading(180°) = (0,-1) 朝南", $"实际 ({h180.dx:0.#####},{h180.dy:0.#####})");
        Check(Math.Abs(h270.dx - 1f) < 1e-5f && Math.Abs(h270.dy) < 1e-5f,
              "Heading(270°) = (+1,0) 朝东", $"实际 ({h270.dx:0.#####},{h270.dy:0.#####})");

        // ---- ② 世界(米) → 屏幕(像素)：+x 向右、+y 向上（y 轴必须翻转）----
        var v = new Ui.BoardView { Width = 400, Height = 300 };
        v.SetViewForTest(0f, 0f, 100f);                       // 100 px/m，视图中心 = 世界原点
        var o = v.ToScreenForTest(0f, 0f);
        var sx = v.ToScreenForTest(1f, 0f);
        var sy = v.ToScreenForTest(0f, 1f);
        Check(Math.Abs(o.X - 200f) < 0.5f && Math.Abs(o.Y - 150f) < 0.5f,
              "世界原点映射到画布中心", $"实际 ({o.X:0.0},{o.Y:0.0})");
        Check(sx.X > o.X + 50f && Math.Abs(sx.Y - o.Y) < 0.5f,
              "世界 +x 朝屏幕右（1m = 100px）", $"实际 ({sx.X:0.0},{sx.Y:0.0})");
        Check(sy.Y < o.Y - 50f, "世界 +y 朝屏幕上（y 轴翻转）", $"实际 ({sy.X:0.0},{sy.Y:0.0})");

        // ---- ③ 增量拉点 + 2cm 抽稀 + 清空 ----
        var store = new ChannelStore();
        store.Ensure(32);
        var ch = new float[32];
        var view = new Ui.BoardView { Width = 400, Height = 300, Store = store };

        // 直行 1 m：每 2ms 一个样本、每个样本前进 0.4mm ⇒ 共 2500 个样本
        double t = 0;
        for (int i = 0; i < 2500; i++)
        {
            ch[26] = 0f; ch[27] = i * 0.0004f; ch[28] = 0f;
            t += 0.002;
            store.Append(ch, 32, t, 0xFFFF);
        }
        view.Tick();
        int n1 = view.PointCount;
        Check(n1 >= 45 && n1 <= 55, "直行 1m 按 2cm 抽稀 ⇒ 约 51 点", $"实际 {n1} 点");

        // 原地不动 200 帧：不应该再涨点（否则静止噪声会把 AutoFit 越撑越大、画面一直抖）
        for (int i = 0; i < 200; i++) { t += 0.02; store.Append(ch, 32, t, 0xFFFF); }
        view.Tick();
        Check(view.PointCount == n1, "静止不再涨点", $"{n1} -> {view.PointCount}");

        // 继续走 0.2 m ⇒ 再涨约 10 点
        for (int i = 1; i <= 500; i++)
        {
            ch[27] = 1f + i * 0.0004f;
            t += 0.002;
            store.Append(ch, 32, t, 0xFFFF);
        }
        view.Tick();
        Check(view.PointCount > n1, "继续走会接着记点", $"{n1} -> {view.PointCount}");

        view.ClearTrail();
        Check(view.PointCount == 0, "清空轨迹后点数为 0", "实际 " + view.PointCount);

        // ---- ③b AutoFit 的"缩放到多大 / 居中到哪"钉成数字 ----
        // 只看画面"像不像"是不够的：比例算错会表现为"轨迹缩在角落"，但那也可能被当成"车走得少"。
        var store2 = new ChannelStore();
        store2.Ensure(32);
        var view2 = new Ui.BoardView { Width = 400, Height = 300, Store = store2 };
        double t2 = 0;
        for (int i = 0; i <= 250; i++)          // 沿 +y 直行 1.00 m（每 4mm 一点）
        {
            ch[26] = 0f; ch[27] = i * 0.004f; ch[28] = 0f;
            t2 += 0.02;
            store2.Append(ch, 32, t2, 0xFFFF);
        }
        view2.Tick();
        var bb = view2.TrailBoundsForTest();
        // ★ maxY 是 0.988 而不是 1.000：**尾部不足 2cm 的那一截不会被记点** —— 这是"按距离阈值抽稀"的固有行为
        //   （参考工程也一样，只是没断言）。视觉上没影响：最后一点离车心 ≤2cm，而车体本身有 26px 以上，被车盖住了。
        //   所以这里断言的是"接近 1"，不是"等于 1"。
        Check(Math.Abs(bb.minX) < 1e-4f && Math.Abs(bb.maxX) < 1e-4f &&
              bb.minY < 1e-4f && bb.maxY > 0.97f && bb.maxY <= 1.0001f,
              "直行 1m 的包围盒 = x[0,0] y[0, ≈1]（尾部 <2cm 不记点）",
              $"实际 x[{bb.minX:0.###},{bb.maxX:0.###}] y[{bb.minY:0.###},{bb.maxY:0.###}]");

        var fit = view2.AutoFitForTest();
        // 解析解：可用区 = (宽-110逻辑px, 高-110逻辑px)；宽取 max(spanX,0.40) 兜底
        float avW = view2.Width - Ui.Dpi.Pxf(110);
        float avH = view2.Height - Ui.Dpi.Pxf(110);
        float spanX = Math.Max(bb.maxX - bb.minX, 0.40f);
        float spanY = Math.Max(bb.maxY - bb.minY, 0.40f);
        float expectPx = Math.Min(avW / spanX, avH / spanY);
        Check(Math.Abs(fit.pxPerM - expectPx) < 0.01f * expectPx,
              $"AutoFit 比例 = {expectPx:0.0} px/m（由包围盒解析）", $"实际 {fit.pxPerM:0.0} px/m");
        Check(Math.Abs(fit.cx - (bb.minX + bb.maxX) * 0.5f) < 0.01f &&
              Math.Abs(fit.cy - (bb.minY + bb.maxY) * 0.5f) < 0.01f,
              "AutoFit 中心 = 包围盒中心",
              $"实际 ({fit.cx:0.00},{fit.cy:0.00}) 期望 ({(bb.minX + bb.maxX) * 0.5f:0.00},{(bb.minY + bb.maxY) * 0.5f:0.00})");
        // 端点必须落在画布里、且"北"在上
        var top = view2.ToScreenForTest(0f, 1f);
        var bot = view2.ToScreenForTest(0f, 0f);
        Check(top.Y > 30f && bot.Y < 270f && bot.Y > top.Y,
              "轨迹两端都落在画布内（上端在上）", $"y 上 {top.Y:0.0} / 下 {bot.Y:0.0}");
        view2.Dispose();

        // ---- ④ 真渲染（离屏位图）：不抛异常，且画面上确实有东西（不是纯背景）----
        try
        {
            using var host = new Form { ClientSize = new Size(400, 300) };
            view.Dock = DockStyle.Fill;
            host.Controls.Add(view);
            host.CreateControl();
            _ = host.Handle;
            host.PerformLayout();                              // 触发一次真实布局


            // ★★ 路表：不仅算数量，还要数“青色虚线”的像素
            //   —— 因为“方法被调用了”和“真的画出去了”是两件事：
            //   DrawRoute 里一旦因为数据异常提前 return，数量断言依然全绿。
            view.SetRoute(new (float x, float y, float yaw)[]
                          { (0f, 0f, 0f), (1f, 0f, 0f), (1f, 1f, 0f), (0f, 1f, 0f) },
                          closed: false, alignToTrail: false);
            Check(view.RouteCount == 4, "路表点数 = 4", "实际 " + view.RouteCount);
            var rb = view.RouteBoundsForTest();
            Check(Math.Abs(rb.minX) < 1e-4f && Math.Abs(rb.minY) < 1e-4f &&
                  Math.Abs(rb.maxX - 1f) < 1e-4f && Math.Abs(rb.maxY - 1f) < 1e-4f,
                  "路表包围盒 = 1m×1m 方框（alignToTrail=false 时不平移）",
                  $"实际 x[{rb.minX:0.###},{rb.maxX:0.###}] y[{rb.minY:0.###},{rb.maxY:0.###}]");
            view.Tick();                                       // 重新拉一次数据
            using var bmp = new Bitmap(Math.Max(view.Width, 1), Math.Max(view.Height, 1));
            view.DrawToBitmap(bmp, new Rectangle(0, 0, bmp.Width, bmp.Height));

            int lit = 0;
            for (int y = 0; y < bmp.Height; y += 3)
                for (int x = 0; x < bmp.Width; x += 3)
                {
                    var c = bmp.GetPixel(x, y);
                    if (c.R + c.G + c.B > 90) lit++;           // 画布底 (16,19,24) 只和 59 ⇒ 不算"画了东西"
                }
            Check(lit >= 5, "渲染出非空画面（网格/轨迹/车体/HUD）", $"亮点 {lit} 个");
            // ★ “青色虚线真的在画布上”的硬证据：按 Theme.Route 逐像素数。
            //   ❗ 门槛不能定高：虚线 + 抗锯齿 ⇒ 只有“笔画芯”像素能精确匹配，
            //      一个 190px 方框只命中几十个。关键是“≠ 0”：0 就是压根没画。
            int rtHit = 0;
            for (int y = 0; y < bmp.Height; y++)
                for (int x = 0; x < bmp.Width; x++)
                {
                    var c = bmp.GetPixel(x, y);
                    if (Math.Abs(c.R - Ui.Theme.Route.R) < 26 &&
                        Math.Abs(c.G - Ui.Theme.Route.G) < 26 &&
                        Math.Abs(c.B - Ui.Theme.Route.B) < 26) rtHit++;
                }
            Check(rtHit >= 30, "路表按青色虚线真的画出去了", $"命中 {rtHit} 个像素");
        }
        catch (Exception ex) { Check(false, "BoardView 渲染", ex.GetType().Name + ": " + ex.Message); }
        finally { try { view.Dispose(); } catch { } }
    }

    // ==================================================================
    // 高 DPI 布局体检
    // ==================================================================
    private static string ApplicationHighDpiMode()
    {
        using var f = new Form();
        _ = f.Handle;
        return f.DeviceDpi.ToString();       // 报告里带上实测 DPI
    }

    private static void CollectType<T>(Control root, List<T> sink) where T : Control
    {
        if (root is T t) sink.Add(t);
        foreach (Control c in root.Controls) CollectType(c, sink);
    }

    private static void TestLayout()
    {
        Section("高 DPI 布局体检（剪字 / 越界 / 滑条高度）");

        Ui.MainForm f = null;
        try
        {
            f = new Ui.MainForm();
            f.CreateControl();
            _ = f.Handle;
            f.PerformLayout();
            f.ApplyRuntimeLayoutForTest();      // ★ 必须先推到位再量，否则量的是默认分隔位置

            Log.AppendLine($"  实测 DPI = {f.DeviceDpi}（Ui.Dpi.Scale = {Ui.Dpi.Scale:0.###}x），窗体 {f.Width}x{f.Height} 物理像素");

            // 字体被偷改会让所有剪字断言集体亮红，根因却不在布局 —— 单钉一条
            Check(Math.Abs(f.TestFontSizePt - 9f) < 0.01f, "窗体字体没被偷改（仍是 9pt）", $"实际 {f.TestFontSizePt:0.##}pt");

            var bars = new List<TrackBar>();
            CollectType(f, bars);
            int want = Ui.Dpi.Px(26);
            bool hOk = bars.Count > 0 && bars.TrueForAll(b => b.Height == want);
            Check(hOk, $"每条滑条高度都 == Dpi.Px(26) = {want}",
                  "实际高度：" + string.Join("/", bars.Select(b => b.Height).Distinct()));
            Check(bars.TrueForAll(b => b.Width > Ui.Dpi.Px(50)),
                  "滑条没被压成 0 宽（MaximumSize.Width 写 0 会静默消失）",
                  string.Join("/", bars.Select(b => b.Width).Distinct()));

            var issues = LayoutAudit.Walk(f);
            var clipped = issues.Where(i => i.Kind == "剪字").ToList();
            var overflow = issues.Where(i => i.Kind.EndsWith("越界")).ToList();
            Check(clipped.Count == 0, "无「字被吃掉」（剪字）",
                  clipped.Count + " 处：" + string.Join(" ; ", clipped.Take(3).Select(i => i.Detail)));
            Check(overflow.Count == 0, "无控件越出父容器",
                  overflow.Count + " 处：" + string.Join(" ; ", overflow.Take(3).Select(i => i.Detail)));

            // ★ 反向对照：必须能抓到故意写窄的按钮，且不许误伤 AutoSize 的
            using var probe = new Panel { Size = Ui.Dpi.Sz(600, 200) };
            var bad = new Button { Text = "打开 CSV…", Width = Ui.Dpi.Px(30), Height = Ui.Dpi.Px(26) };
            var good = new Button
            {
                Text = "打开 CSV…", AutoSize = true, AutoSizeMode = AutoSizeMode.GrowAndShrink,
                MinimumSize = Ui.Dpi.Sz(96, 26),
            };
            probe.Controls.Add(bad);
            probe.Controls.Add(good);
            var probeIssues = LayoutAudit.Walk(probe).Where(i => i.Kind == "剪字").ToList();
            Check(probeIssues.Any(i => i.C == bad), "反向对照：写死 30px 的窄按钮必须被点名");
            Check(!probeIssues.Any(i => i.C == good), "反向对照：AutoSize 按钮不许被点名");

            // ---- 「实时值」数值框（Consolas 14pt Bold、62 逻辑像素宽）能装下最坏文本吗 ----
            // 这是 150% 缩放下最容易剪字的地方：字体是点值（会随 DPI 放大），框是物理像素（不放大）。
            // ⚠ 别只测 "0.00" —— 真正会出事的是 "-99.9" 这种 5 字符的负数/三位数。
            using (var f14 = new Font("Consolas", 14f, FontStyle.Bold))
            {
                int boxW = Ui.Dpi.Px(62);
                foreach (var s in new[] { "-9.99", "-99.9", "999.9", "99999", "-1000", "--" })
                {
                    var sz = TextRenderer.MeasureText(s, f14);
                    Check(sz.Width <= boxW, $"实时值框装得下 \"{s}\"",
                          $"需要 {sz.Width}px，只有 {boxW}px");
                }
            }

            f.Close();
        }
        catch (Exception ex)
        {
            Check(false, "布局体检执行", ex.GetType().Name + ": " + ex.Message);
        }
        finally
        {
            try { f?.Dispose(); } catch { }
        }
    }

    /// <summary>
    /// `--shot-form` 的第 4 个参数：注入"帧里有 N 个通道"。
    /// 用途只有一个 —— **给「通道名整体错位」那条红条出图**（≥1 时生效，默认 -1 不注入）。
    /// 例：`KartHost.exe --shot-form warn.png calib 39` ⇒ 打出"档案=标定·精简、帧=39 通道"的红条图。
    /// </summary>
    public static int ShotFrameCount = -1;

    /// <summary>
    /// 出图专用：往 ChannelStore 灌一条合成赛道（直道 → 左弯 → 直道 → 右弯 → 直道），
    /// 让「实时轨迹」面板在截图里能看出形状。只在「惯导+路表」档案下生效。
    /// ★ 用与车端 `ins_odom.c` **完全相同**的积分式 `x += -sin(yaw)·ds; y += +cos(yaw)·ds` 生成 ——
    ///   灌进去的就是"真轨迹该长的样子"，不是为了好看随手画的曲线。
    ///   （如果哪天有人把上位机的坐标约定改反了，这张图的形状会立刻不对。）
    /// </summary>
    private static void FillDemoTrack(Ui.MainForm f)
    {
        if (!Defaults.IsInsRoute) return;

        int cn = Defaults.ChannelNames.Length;
        var store = f.StoreForTest;
        store.Ensure(cn);
        var ch = new float[Math.Max(cn, 32)];

        double x = 0, y = 0, yawDeg = 0, dist = 0, t = 0;
        const double dt = 0.02;                                  // 50 Hz，与真实遥测一致

        foreach (var seg in DemoSegments())
        {
            for (int k = 0; k < 50; k++)                         // 每段 1 秒 = 50 拍
            {
                double v = seg.Item2;
                yawDeg += seg.Item1 * dt;
                double rad = yawDeg * Math.PI / 180.0;
                double ds = v * dt;
                x += -Math.Sin(rad) * ds;
                y += Math.Cos(rad) * ds;
                dist += ds;
                t += dt;

                ch[26] = (float)x;
                ch[27] = (float)y;
                ch[28] = (float)yawDeg;
                ch[29] = (float)dist;
                ch[30] = 0x01;                                   // flags: bit0 就绪
                ch[31] = (float)(t * 50);                        // seq
                ch[1] = (float)(v / 0.5);                        // 左轮实测（rps，占位比例）
                ch[4] = ch[1];                                   // 右轮实测
                store.Append(ch, cn, t, 0xFFFF);
            }
        }

        // ★ 顺便造一张“路表”叠在轨迹上（比实时轨迹略大一圈）——
        //   出图时能一眼看出「青色虚线 = 路表」这层语义是对的，而不是只有一条轨迹。
        var rp = new List<(float x, float y, float yaw)>();
        double rx = 0, ry = 0, ryaw = 0;
        foreach (var seg in DemoSegments())
        {
            for (int k = 0; k < 50; k++)
            {
                ryaw += seg.Item1 * 1.15 * dt;
                double rad = ryaw * Math.PI / 180.0;
                double ds = seg.Item2 * 1.10 * dt;
                rx += -Math.Sin(rad) * ds;
                ry += Math.Cos(rad) * ds;
                rp.Add(((float)rx, (float)ry, (float)ryaw));
            }
        }
        f.BoardForTest.SetRoute(rp, false, alignToTrail: false);
    }
    /// <summary>演示赛道分段：(角速度 °/s, 速度 m/s)，每段 1 秒</summary>
    private static IEnumerable<Tuple<double, double>> DemoSegments()
    {
        for (int i = 0; i < 2; i++) yield return Tuple.Create(0.0, 1.20);    // 直道
        for (int i = 0; i < 4; i++) yield return Tuple.Create(22.0, 0.90);   // 左弯（yaw 增加 = 逆时针）
        for (int i = 0; i < 2; i++) yield return Tuple.Create(0.0, 1.10);    // 直道
        for (int i = 0; i < 4; i++) yield return Tuple.Create(-26.0, 0.85);  // 右弯
        for (int i = 0; i < 2; i++) yield return Tuple.Create(0.0, 1.00);    // 直道收尾
    }

    /// <summary>整窗出图 + 布局报告（--shot-form 用）</summary>
    public static int ShotForm(string png)
    {
        using var f = new Ui.MainForm
        {
            ShowInTaskbar = false,
            Opacity = 0.01,                                  // ★ 必须 Show()：只有真成为可见窗口，布局才结算完
            StartPosition = FormStartPosition.CenterScreen,
        };
        f.Show();
        Application.DoEvents();

        // 灌一条演示轨迹并让轨迹面板消费掉 —— 否则截图里永远是"等待位姿…"
        try { FillDemoTrack(f); f.BoardForTest?.Tick(); } catch { }
        Application.DoEvents();

        // 注入"帧里有 N 个通道" ⇒ 出那张「通道名整体错位」红条的图（见 ShotFrameCount）
        if (ShotFrameCount > 0) { f.SetFrameCountForTest(ShotFrameCount); Application.DoEvents(); }

        f.PerformLayout();
        using var bmp = new Bitmap(f.Width, f.Height);
        f.DrawToBitmap(bmp, new Rectangle(0, 0, f.Width, f.Height));
        bmp.Save(png, System.Drawing.Imaging.ImageFormat.Png);

        string report = LayoutAudit.Report(f, "主窗体", f.DeviceDpi);
        File.WriteAllText(Path.ChangeExtension(png, ".layout.txt"), report, Encoding.UTF8);
        Console.WriteLine(report);
        f.Close();
        return 0;
    }

    /// <summary>
    /// 路表管理窗口出图 + 布局体检：<c>KartHost.exe --shot-route 输出.png</c>
    /// 与 --shot-form 同一套路（**不 Reset DPI**：要看的就是这台机器真实缩放下的长相）。
    /// 灌一张**现造的圆环表**进去 ⇒ 不需要车、不需要串口、不需要任何外部文件。
    /// </summary>
    public static int ShotRoute(string png)
    {
        using var f = new Ui.RouteManagerForm
        {
            ShowInTaskbar = false,
            Opacity = 0.01,
            StartPosition = FormStartPosition.CenterScreen,
        };
        f.Show();
        Application.DoEvents();

        try { f.LoadDemoImage(MakeDemoRouteImage()); }
        catch (Exception ex) { Console.WriteLine("演示数据失败：" + ex.Message); }
        Application.DoEvents();
        f.PerformLayout();

        using var bmp = new Bitmap(f.Width, f.Height);
        f.DrawToBitmap(bmp, new Rectangle(0, 0, f.Width, f.Height));
        bmp.Save(png, System.Drawing.Imaging.ImageFormat.Png);

        string report = LayoutAudit.Report(f, "路表管理窗口", f.DeviceDpi);
        File.WriteAllText(Path.ChangeExtension(png, ".layout.txt"), report, Encoding.UTF8);
        Console.WriteLine(report);
        f.Close();
        return 0;
    }

    /// <summary>
    /// 标定窗口出图 + 布局体检：KartHost.exe --shot-calib 输出.png
    /// 这个窗口只依赖 ICalibHost（数据仓库 / 参数表 / 写参数），所以给一个纯内存的假宿主就能跑 ——
    /// 不需要车、不需要链路、不需要串口、不需要任何外部文件。
    /// </summary>
    public static int ShotCalib(string png)
    {
        Defaults.SetProfile(HostProfile.Calib);      // 出图用「标定·精简」，这才是这个窗口的日常档案

        var store = new ChannelStore();
        store.Ensure(Defaults.ChannelNames.Length);

        var host = new FakeCalibHost(store);

        using var f = new Ui.CalibForm(host)
        {
            ShowInTaskbar = false,
            Opacity = 0.01,
            StartPosition = FormStartPosition.CenterScreen,
        };
        f.Show();
        Application.DoEvents();
        f.PerformLayout();

        using var bmp = new Bitmap(f.Width, f.Height);
        f.DrawToBitmap(bmp, new Rectangle(0, 0, f.Width, f.Height));
        bmp.Save(png, System.Drawing.Imaging.ImageFormat.Png);

        string report = LayoutAudit.Report(f, "标定窗口", f.DeviceDpi);
        File.WriteAllText(Path.ChangeExtension(png, ".layout.txt"), report, Encoding.UTF8);
        Console.WriteLine(report);
        f.Close();
        return 0;
    }

    /// <summary>标定窗口的假宿主：不连车也能把"界面构建 / 出图 / 布局体检"跑完。</summary>
    private sealed class FakeCalibHost : Ui.ICalibHost
    {
        public FakeCalibHost(ChannelStore s)
        {
            Store = s;
            Params = Defaults.BuildDefaultParams();
        }

        public ChannelStore Store { get; }
        public bool LinkOpen => true;
        public IReadOnlyList<ParamMeta> Params { get; }
        public void WriteParam(ParamMeta p, float v) { if (p != null) p.Value = v; }
        public void Log(string s) { }
    }

    /// <summary>现造一张 R=500 mm 的圆环路表镜像（36 B 头 + 157×12 B 记录），两个 CRC 都算对。</summary>
    private static byte[] MakeDemoRouteImage()
    {
        const int n = 157;
        const double R = 500.0;                       // mm
        double ds = 2 * Math.PI * R / n;

        var img = new byte[36 + n * 12];
        for (int i = 0; i < n; i++)
        {
            double th = i * ds / R;
            int x = (int)Math.Round(R * Math.Sin(th));
            int y = (int)Math.Round(R * Math.Cos(th));
            double yaw = th * 180.0 / Math.PI;         // 相对第 0 点
            while (yaw > 180.0) yaw -= 360.0;
            while (yaw < -180.0) yaw += 360.0;

            int o = 36 + i * 12;
            BitConverter.GetBytes(x).CopyTo(img, o);
            BitConverter.GetBytes(y).CopyTo(img, o + 4);
            BitConverter.GetBytes((short)Math.Round(yaw * 100)).CopyTo(img, o + 8);
            BitConverter.GetBytes((short)80).CopyTo(img, o + 10);      // 0.80 m/s
        }

        BitConverter.GetBytes(0x4B525443u).CopyTo(img, 0);            // 'CTRK'
        BitConverter.GetBytes((ushort)1).CopyTo(img, 4);              // ver
        BitConverter.GetBytes((ushort)12).CopyTo(img, 6);             // rec_len
        BitConverter.GetBytes((uint)n).CopyTo(img, 8);                // count
        BitConverter.GetBytes((uint)20).CopyTo(img, 12);              // ds_mm
        BitConverter.GetBytes((uint)(n * 20)).CopyTo(img, 16);        // lap_mm
        BitConverter.GetBytes(0).CopyTo(img, 20);                     // origin x
        BitConverter.GetBytes(0).CopyTo(img, 24);                     // origin y
        BitConverter.GetBytes((ushort)0).CopyTo(img, 28);             // origin yaw
        BitConverter.GetBytes((ushort)0).CopyTo(img, 30);             // flags

        BitConverter.GetBytes(RouteLink.Crc16(0xFFFF, img, 36, n * 12)).CopyTo(img, 32);
        BitConverter.GetBytes(RouteLink.Crc16(0xFFFF, img, 0, 34)).CopyTo(img, 34);
        return img;
    }

    // ==================================================================
    // 行程标定（速度积分 → 机械角）
    // ==================================================================
    private static void TestTravelCal()
    {
        Section("行程标定（速度积分 → 机械角）");

        var cal = new Ui.TravelCal();
        Check(!cal.Complete, "只记一边时不算标定完成");
        Check(double.IsNaN(cal.AngleDeg(123)), "没标定时 AngleDeg 返回 NaN（不许瞎给数）");

        // 左 −2400 / 右 +2400 mrad，机械行程 60° → 每度 80 mrad
        cal.Capture(true, -2400, 10.0);
        cal.Capture(false, 2400, 40.0);
        Check(cal.Complete, "左右都记完 → 标定完成");
        Check(Math.Abs(cal.SpanSum - 4800) < 1e-6, "跨度 = 4800 mrad", cal.SpanSum.ToString("0"));
        Check(Math.Abs(cal.MradPerDeg - 80.0) < 1e-6, "映射 = 80 mrad/°（4800 ÷ 60°）", cal.MradPerDeg.ToString("0.00"));
        Check(Math.Abs(cal.MidSum) < 1e-6, "中点 = 0 mrad", cal.MidSum.ToString("0"));
        Check(Math.Abs(cal.AngleDeg(2400) - 30.0) < 1e-6, "右限位处 → +30°", cal.AngleDeg(2400).ToString("0.0"));
        Check(Math.Abs(cal.AngleDeg(-2400) + 30.0) < 1e-6, "左限位处 → -30°", cal.AngleDeg(-2400).ToString("0.0"));
        Check(Math.Abs(cal.AngleDeg(0)) < 1e-9, "中点处 → 0°");
        Check(Math.Abs(cal.Turns - 4800.0 / (2000.0 * Math.PI)) < 1e-9, "跨度折算 = 0.764 圈");
        Check(Math.Abs(cal.HalfSpanDeg - 30.0) < 1e-9, "限位 = ±30.0°", cal.HalfSpanDeg.ToString("0.0"));

        // ★ 反向对照：跨度太小必须判无效，否则"每度多少"会被放大成垃圾
        var bad = new Ui.TravelCal();
        bad.Capture(true, 0, 0);
        bad.Capture(false, 50, 1);
        Check(!bad.SpanOk, "跨度只有 50 mrad → 判定标定无效");
        Check(double.IsNaN(bad.AngleDeg(50)), "标定无效时 AngleDeg 仍返回 NaN");
        Check(double.IsNaN(bad.MradPerDeg), "标定无效时映射也是 NaN");

        // 中点不在 0、机械行程换成 90° 也要对
        var off = new Ui.TravelCal { MechSpanDeg = 90.0 };
        off.Capture(true, 1000, 0);
        off.Capture(false, 10000, 45);
        Check(Math.Abs(off.MradPerDeg - 100.0) < 1e-6, "9000 mrad ÷ 90° = 100 mrad/°", off.MradPerDeg.ToString("0.0"));
        Check(Math.Abs(off.AngleDeg(5500)) < 1e-9, "中点 5500 mrad 处 → 0°");

        // 清空
        cal.Clear();
        Check(!cal.Complete && double.IsNaN(cal.SpanSum), "清空后回到未标定状态");

        // ---- 界面：档案决定它可不可用 ----
        Defaults.SetProfile(HostProfile.Motor);
        var panel = new Ui.TravelCalPanel(new ChannelStore(), v => { }, s => { });
        var btns = new List<Button>();
        CollectType(panel, btns);
        Check(btns.Count == 4, "标定面板有 4 个按钮", "实际 " + btns.Count);
        Check(btns.TrueForAll(b => b.Enabled), "单电机档案：找得到 mag_sum → 按钮可用");

        Defaults.SetProfile(HostProfile.Kart);
        var panel2 = new Ui.TravelCalPanel(new ChannelStore(), v => { }, s => { });
        var btns2 = new List<Button>();
        CollectType(panel2, btns2);
        // 「清空」不依赖通道，永远可用；另外 3 个必须禁用
        Check(btns2.Count(b => b.Enabled) == 1, "卡丁车档案：只留「清空」可用（其余 3 个自动禁用）",
              "可用 " + btns2.Count(b => b.Enabled) + " 个");
        Check(btns2.Count(b => !b.Enabled) == 3, "禁用 3 个（记左/记右/置零）", "实际 " + btns2.Count(b => !b.Enabled));

        panel.Dispose();
        panel2.Dispose();
    }

    /// <summary>静态统计控件树大小</summary>
    private static int CountControls(Control root)
    {
        int n = 1;
        foreach (Control c in root.Controls) n += CountControls(c);
        return n;
    }

    /// <summary>
    /// 界面冒烟测试：完整构造 MainForm、强制创建句柄触发一次真实布局、
    /// 再切换分图布局、再关闭。全程不进入消息循环。
    /// </summary>
    private static void TestUi()
    {
        Section("界面构建（无消息循环冒烟测试）");
        Ui.MainForm f = null;
        try
        {
            f = new Ui.MainForm();
            f.CreateControl();
            _ = f.Handle;                    // 触发布局与 OnHandleCreated
            f.PerformLayout();

            int total = CountControls(f);
            Check(total > 150, "控件树已构建", "共 " + total + " 个控件");
            Check(f.Width >= 1120 && f.Height >= 700, "窗体尺寸合理",
                  f.Width + "x" + f.Height);

            // 数一下参数滑条（应从参数表自动生成）
            // ⚠ 只数**参数面板内部**的 —— 之前是全窗体统计，行车标定面板加了个
            //   "机械行程"数字框，这条就误报了（26 vs 25）。断言该贴着的对象数，别数全局。
            var pps = new List<Ui.ParamPanel>();
            CollectType(f, pps);
            Check(pps.Count == 1, "找得到参数面板", "实际 " + pps.Count);
            int bars = pps.Count > 0 ? CountType<TrackBar>(pps[0]) : 0;
            int nums = pps.Count > 0 ? CountType<NumericUpDown>(pps[0]) : 0;
            int lamps = CountType<Ui.LampControl>(f);
            int waves = CountType<Ui.WavePanel>(f);
            Check(bars >= 20, "参数滑条已自动生成", "TrackBar " + bars + " 个");
            Check(nums == bars, "滑条与数字框一一对应", $"NumericUpDown {nums} / TrackBar {bars}");
            Check(lamps == 4, "指示灯 4 盏（链路+舵机+电机+整体）", "实际 " + lamps);
            Check(waves == 4, "默认 2×2 分图（4 个波形）", "实际 " + waves);

            // ★ 按文案点名检查关键按钮**是否真的挂在控件树里**。
            //   起因：「路表管理」第一版建了控件、也接了事件，就是忘了 AddRange 进父容器 ⇒
            //   界面上根本看不到，而且**不报错**。LayoutAudit 也查不出来 —— 孤儿控件不在树里，
            //   任何"遍历控件树"的体检都看不见它。只有按名字点名才拦得住。
            var btns = new List<Button>();
            CollectType(f, btns);
            var btnTexts = new HashSet<string>(btns.Select(b => b.Text));
            string[] mustHave =
            {
                "自动适应", "清空轨迹", "车端归零", "读路表", "路表管理",
                "刷新参数表", "存 Flash", "恢复默认", "暂停", "套用预设分图",
                "舵机使能", "电机使能", "整体使能", "1×1", "2×1", "2×2",
            };
            var missing = mustHave.Where(t => !btnTexts.Contains(t)).ToList();
            Check(missing.Count == 0, "关键按钮全部在位（防「建了没挂」）",
                  missing.Count == 0 ? $"{mustHave.Length} 个都在"
                                     : "缺：" + string.Join(" / ", missing));

            // 切布局再切回来，验证不会抛异常
            var m = typeof(Ui.MainForm).GetMethod("ApplyLayout",
                System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);
            Check(m != null, "ApplyLayout 可调用");
            if (m != null)
            {
                m.Invoke(f, new object[] { 1 });
                Check(CountType<Ui.WavePanel>(f) == 1, "切到 1×1 后只剩 1 个波形");
                m.Invoke(f, new object[] { 2 });
                Check(CountType<Ui.WavePanel>(f) == 2, "切到 2×1 后为 2 个波形");
                m.Invoke(f, new object[] { 4 });
                Check(CountType<Ui.WavePanel>(f) == 4, "切回 2×2 后为 4 个波形");
            }

            // ★★ 「帧 COUNT ≠ 档案」的红条必须真的挂到波形上，而且**点一下能修** ——
            //   这就是 2026-10-14 那次"k_z 一直变"（其实是 rx_age）的完整闭环。
            //   注入 39（唯一命中「卡丁·遥控」）；当前档案是「卡丁车」(32) ⇒ 属于不匹配。
            f.SetFrameCountForTest(39);
            var w0 = f.WavesForTest?[0];
            Check(w0 != null && !string.IsNullOrEmpty(w0.WarnText),
                  "帧 39 通道而档案 32 通道 ⇒ 波形上出现红条", w0?.WarnText ?? "（没有波形）");
            Check(w0 != null && w0.WarnClick != null, "红条可点（唯一命中 ⇒ 给了「一键切」回调）");

            w0?.WarnClick?.Invoke();
            Check(Defaults.Current == HostProfile.KartRC,
                  "点红条后档案切到「卡丁·遥控」", Defaults.Current.ToString());
            var w1 = f.WavesForTest?[0];
            Check(w1 != null && string.IsNullOrEmpty(w1?.WarnText),
                  "切完红条自己消失（39 对 39）", w1?.WarnText ?? "（没有波形）");

            // 有歧义的通道数不许自动切：注入 32 ⇒ 命中「卡丁车 / 惯导+路表」两个
            f.SetFrameCountForTest(32);
            var w2 = f.WavesForTest?[0];
            Check(w2 != null && !string.IsNullOrEmpty(w2.WarnText) && w2.WarnClick == null,
                  "32 通道（命中 2 个档案）⇒ 只提示、红条不可点", w2?.WarnText ?? "（没有波形）");

            // 旧固件（37 = 2026-10-14 之前的全量档）**一个档案都对不上** ⇒ 必须点名"要重刷车端"，
            // 否则用户会以为"换个档案就好了"，然后一个个试过去（都不对）。
            f.SetFrameCountForTest(37);
            var w3 = f.WavesForTest?[0];
            Check(w3 != null && w3.WarnClick == null && (w3.WarnText ?? "").Contains("旧固件"),
                  "37 通道（老固件）⇒ 红条点名「车端是旧固件，要重刷」", w3?.WarnText ?? "（没有波形）");

            Defaults.SetProfile(HostProfile.Kart);      // 收尾，别把后面的测试带歪

            f.Close();
            Check(true, "窗体关闭流程无异常");
        }
        catch (Exception ex)
        {
            Check(false, "MainForm 构造 / 布局 / 关闭", ex.GetType().Name + ": " + ex.Message);
        }
        finally
        {
            try { f?.Dispose(); } catch { }
        }
    }

    private static int CountType<T>(Control root) where T : Control
    {
        int n = root is T ? 1 : 0;
        foreach (Control c in root.Controls) n += CountType<T>(c);
        return n;
    }

    // ------------------------------------------------------------------
    private static void TestCrc()
    {
        Section("CRC16-MODBUS");
        // 标准测试向量：CRC16/MODBUS("123456789") = 0x4B37
        var b = Encoding.ASCII.GetBytes("123456789");
        ushort crc = Proto.Crc16(b, 0, b.Length);
        Check(crc == 0x4B37, "标准向量 \"123456789\" -> 0x4B37",
              "实际 0x" + crc.ToString("X4"));

        // 改一个字节，CRC 必须变
        var c = Encoding.ASCII.GetBytes("123456788");
        ushort crc2 = Proto.Crc16(c, 0, c.Length);
        Check(crc2 != crc, "单字节改动使 CRC 变化");
    }

    private static void TestCommandFrame()
    {
        Section("下行命令帧编解码");

        var f = Proto.BuildCommand(Cmd.SetParam, 7, 1.5f, 0x1234);
        Check(f.Length == 15, "总长 = 15", "实际 " + f.Length);
        Check(f[0] == 0x5A && f[1] == 0xA5, "MAGIC = 5A A5");
        Check(f[2] == (byte)FrameType.Command, "TYPE = COMMAND");
        Check(Proto.GetU16(f, 3) == 8, "LEN = 8");
        Check(f[5] == (byte)Cmd.SetParam, "CMD 正确");
        Check(f[6] == 7, "PARAM_ID 正确");
        Check(Math.Abs(Proto.GetF32(f, 7) - 1.5f) < 1e-6, "VALUE 往返一致");
        Check(Proto.GetU16(f, 11) == 0x1234, "TOKEN 正确");
        Check(Proto.Crc16(f, 2, 11) == Proto.GetU16(f, 13), "CRC 自校验通过");

        // 使能帧
        var e = Proto.BuildSetEnable(EnableBits.Master | EnableBits.Motor, 1);
        Check(Proto.GetF32(e, 7) == (float)((ushort)(EnableBits.Master | EnableBits.Motor)),
              "使能位图按 ushort 装进 VALUE");
    }

    /// <summary>构造一个合法的上行遥测帧（模拟车端用法）</summary>
    private static byte[] MakeTelemetry(ushort seq, int cnt, float baseVal)
    {
        int pl = Proto.TELEM_OVERHEAD + cnt * 4;
        var f = new byte[Proto.UP_OVERHEAD + pl];
        f[0] = Proto.MAGIC_UP_0; f[1] = Proto.MAGIC_UP_1;
        f[2] = (byte)FrameType.Telemetry;
        Proto.PutU16(f, 3, (ushort)pl);
        Proto.PutU16(f, 5, seq);
        Proto.PutU32(f, 7, 12345);
        Proto.PutU16(f, 11, 0xFFFF);
        Proto.PutU16(f, 13, (ushort)cnt);
        for (int i = 0; i < cnt; i++) Proto.PutF32(f, 15 + i * 4, baseVal + i);
        Proto.PutU16(f, 5 + pl, Proto.Crc16(f, 2, 3 + pl));
        Array.Copy(Proto.TAIL, 0, f, 7 + pl, 4);
        return f;
    }

    /// <summary>
    /// 路表串口链路（RouteLink）的**纯函数**部分：PONG 能力位解析。
    /// <para>为什么要单独测：这条链最阴的失败是「上位机发了、车端不认识」——
    /// 车端只回一个笼统的 ERR_STATE，现场看不出是固件旧。能力位就是为它加的，
    /// 而"v1 老 PONG（8 B）要按推断补齐能力"这条最容易写错（返回 0 会误判成"什么都不会"）。</para>
    /// </summary>
    private static void TestRouteLinkCaps()
    {
        Section("路表串口协议：PONG 能力位");

        // ---- 1) v1 老固件：8 B PONG，没有能力位 ----
        var v1 = new byte[8];
        v1[0] = 1; v1[1] = 7; v1[2] = 13; v1[3] = 12;

        Check(!RouteLink.PongHasCaps(v1), "v1 PONG(8 B) 判为「没有能力位」");

        uint c1 = RouteLink.ParseCaps(v1);
        Check(c1 == (RouteLink.CapDir | RouteLink.CapLoad | RouteLink.CapSlotSel),
              "v1 推断能力 = DIR|LOAD|SEL",
              "实际 0x" + c1.ToString("X"));
        Check((c1 & RouteLink.CapSlotRead) == 0,
              "★ v1 推断「不支持读槽」⇒ 上传能提前拦住，而不是等一句笼统的状态不允许");
        Check((c1 & RouteLink.CapStep) == 0,
              "★ v1 推断「不支持阶跃测试」(0x0A) ⇒ 老固件上那两枚按钮也能被提前拦住");

        // ---- 2) v2 新固件：12 B PONG，能力位在偏移 8 ----
        var v2 = new byte[12];
        Array.Copy(v1, v2, 8);
        uint all = RouteLink.CapDir | RouteLink.CapLoad | RouteLink.CapSlotSel
                 | RouteLink.CapSlotRead | RouteLink.CapStep;
        v2[8] = (byte)(all & 0xFF); v2[9] = (byte)((all >> 8) & 0xFF);
        v2[10] = (byte)((all >> 16) & 0xFF); v2[11] = (byte)((all >> 24) & 0xFF);

        Check(RouteLink.PongHasCaps(v2), "v2 PONG(12 B) 判为「有能力位」");
        Check(RouteLink.ParseCaps(v2) == all, "v2 能力位按原样读出",
              "实际 0x" + RouteLink.ParseCaps(v2).ToString("X"));
        Check((RouteLink.ParseCaps(v2) & RouteLink.CapStep) != 0, "v2 含 STEP ⇒ 阶跃测试可用");

        // ---- 3) v2 但车端没开读槽（比如以后按需裁剪） ----
        var v2b = (byte[])v2.Clone();
        uint noRead = all & ~RouteLink.CapSlotRead;
        v2b[8] = (byte)(noRead & 0xFF); v2b[9] = (byte)((noRead >> 8) & 0xFF);
        v2b[10] = 0; v2b[11] = 0;
        Check((RouteLink.ParseCaps(v2b) & RouteLink.CapSlotRead) == 0, "v2 缺 SLOT_READ 时判为不支持");

        // ---- 4) 空/异常输入不许抛（按 v1 兜底） ----
        uint cn = RouteLink.ParseCaps(null);
        Check(!RouteLink.PongHasCaps(null) && cn == (RouteLink.CapDir | RouteLink.CapLoad | RouteLink.CapSlotSel),
              "null PONG 不抛异常，且按 v1 兜底", "实际 0x" + cn.ToString("X"));
    }

    private static void TestFraming()
    {
        Section("帧同步状态机（TCP 粘包 / 半包 / 错帧重同步）");

        // --- 1) 粘包：两帧一次喂 ---
        {
            var p = new FrameParser();
            int got = 0; ushort lastSeq = 0; int lastCnt = 0;
            p.OnFrame = (t, b, o) =>
            {
                if (t != (byte)FrameType.Telemetry) return;
                got++; lastSeq = Proto.GetU16(b, o); lastCnt = Proto.GetU16(b, o + 8);
            };
            var a = MakeTelemetry(100, 8, 1.0f);
            var b2 = MakeTelemetry(101, 8, 2.0f);
            var both = new byte[a.Length + b2.Length];
            Array.Copy(a, 0, both, 0, a.Length);
            Array.Copy(b2, 0, both, a.Length, b2.Length);
            p.Feed(both, both.Length);
            Check(got == 2, "粘包（两帧一次喂）→ 解析出 2 帧", "实际 " + got);
            Check(lastSeq == 101, "最后一帧 SEQ = 101");
            Check(lastCnt == 8, "COUNT = 8");
        }

        // --- 2) 半包：一个字节一个字节喂 ---
        {
            var p = new FrameParser();
            int got = 0; float v0 = 0;
            p.OnFrame = (t, b, o) => { if (t == (byte)FrameType.Telemetry) { got++; v0 = Proto.GetF32(b, o + 10); } };
            var a = MakeTelemetry(7, 4, 3.25f);
            for (int i = 0; i < a.Length; i++) p.Feed(new[] { a[i] }, 1);
            Check(got == 1, "半包（逐字节喂）→ 恰好解析出 1 帧", "实际 " + got);
            Check(Math.Abs(v0 - 3.25f) < 1e-6, "首通道值无损", "实际 " + v0);
            Check(p.PendingBytes == 0, "喂完无残留字节", "残留 " + p.PendingBytes);
        }

        // --- 3) 前导垃圾 + CRC 损坏帧，必须能重新同步 ---
        {
            var p = new FrameParser();
            var seqs = new List<ushort>();
            p.OnFrame = (t, b, o) => { if (t == (byte)FrameType.Telemetry) seqs.Add(Proto.GetU16(b, o)); };

            var junk = new byte[] { 0x11, 0x22, 0xA5, 0x00, 0x33 };
            p.Feed(junk, junk.Length);

            var bad = MakeTelemetry(50, 4, 1f);
            bad[20] ^= 0xFF;                       // 破坏数据 → CRC 必错
            p.Feed(bad, bad.Length);

            var good = MakeTelemetry(51, 4, 1f);
            p.Feed(good, good.Length);

            Check(seqs.Count == 1 && seqs[0] == 51,
                  "跳过垃圾与坏帧，仍能解析出后面那帧 (SEQ=51)",
                  "解析到 " + string.Join(",", seqs));
            Check(p.FramesBadCrc >= 1, "坏帧被记为 CRC 错误", "计数 " + p.FramesBadCrc);
        }

        // --- 4) 帧头恰好在缓冲末尾断开（跨包） ---
        {
            var p = new FrameParser();
            int got = 0;
            p.OnFrame = (t, b, o) => { if (t == (byte)FrameType.Telemetry) got++; };
            var a = MakeTelemetry(1, 2, 0f);
            p.Feed(new[] { a[0] }, 1);             // 只喂 MAGIC 第 1 字节
            p.Feed(a, a.Length);                   // 再喂整帧（含重复的 MAGIC）
            // 第二个 0xA5 才是真帧头，应解析出 1 帧
            Check(got == 1, "帧头跨包断开仍能正确识别", "实际 " + got);
        }
    }

    private static void TestEndToEnd()
    {
        Section("端到端：模拟车端 → TCP → Link → 解析 → 数据仓库");

        var sim = new Simulator { Port = 18521 };
        var logs = new List<string>();
        sim.Log += m => { lock (logs) logs.Add(m); };
        sim.Start();

        var link = new Link();
        var store = new ChannelStore();
        long frames = 0;
        int lastCnt = 0;
        var seqs = new List<int>();
        var evt = new ManualResetEventSlim(false);

        link.FrameReceived += (t, b, o) =>
        {
            if (t != (byte)FrameType.Telemetry) return;
            int cnt = Proto.GetU16(b, o + 8);
            var ch = new float[cnt];
            for (int i = 0; i < cnt; i++) ch[i] = Proto.GetF32(b, o + 10 + i * 4);
            store.Ensure(cnt);
            store.Append(ch, cnt, 1.0, Proto.GetU16(b, o + 6));
            lastCnt = cnt;
            frames++;
            lock (seqs) seqs.Add(Proto.GetU16(b, o));
            if (frames >= 30) evt.Set();
        };

        bool connected = false;
        try
        {
            link.ConnectTcp("127.0.0.1", sim.Port);
            connected = true;
        }
        catch (Exception ex)
        {
            Check(false, "连接模拟车端", ex.Message);
        }

        if (connected)
        {
            Check(true, "TCP 连接建立");
            evt.Wait(4000);
            Thread.Sleep(200);

            Check(frames >= 30, "收到 ≥30 帧遥测", "实际 " + frames);
            Check(link.FramesOk >= 30, "解析器 FramesOk ≥ 30", "实际 " + link.FramesOk);
            Check(link.FramesBadCrc == 0, "无 CRC 错误", "实际 " + link.FramesBadCrc);
            Check(lastCnt == Defaults.ChannelNames.Length,
                  $"COUNT = {Defaults.ChannelNames.Length}", "实际 " + lastCnt);
            Check(store.Count > 0, "数据仓库已写入", "实际 " + store.Count);

            // SEQ 应该连续
            bool seqOk = true;
            lock (seqs)
            {
                for (int i = 1; i < seqs.Count; i++)
                    if (seqs[i] != ((seqs[i - 1] + 1) & 0xFFFF)) { seqOk = false; break; }
            }
            Check(seqOk, "SEQ 连续（无丢包）");

            // ---- 命令下行 + ACK ----
            long before = sim.CommandsGot;
            for (int i = 0; i < 5; i++)
            {
                link.Send(Proto.BuildCommand(Cmd.SetParam, 4, 3.0f + i * 0.1f, (ushort)(100 + i)));
                Thread.Sleep(30);
            }
            Thread.Sleep(300);
            Check(sim.CommandsGot >= before + 5,
                  "车端收到 5 条参数写入", $"收到 {sim.CommandsGot - before} 条");

            // ---- 参数表下发 ----
            int meta = 0;
            link.FrameReceived += (t, b, o) => { if (t == (byte)FrameType.ParamMeta) meta++; };
            link.Send(Proto.BuildSimple(Cmd.GetTable, 999));
            Thread.Sleep(500);
            Check(meta >= 20, "收到 ≥20 条参数表条目", "实际 " + meta);

            link.Close();
        }

        Check(store.Latest(15) != 0 || store.Latest(0) != 0, "通道有非零实时值（模拟数据在动）");
        sim.Stop();
    }
}
