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
        TestFraming();
        TestEndToEnd();
        TestUdpLink();               // UDP：车端主动上报的路径（曾经绑错端口）
        TestMotorProfile();          // 单电机验证档案（跑完会切回卡丁车档案）
        TestDualMotorProfile();      // 双电机速度环档案（跑完会切回卡丁车档案）
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
