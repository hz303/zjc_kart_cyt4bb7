namespace KartHost;

/// <summary>遥测通道按档案切换：全车（Kart）、单电机验证（Motor）、双电机速度环（DualMotor）。</summary>
public enum HostProfile
{
    Kart,
    Motor,
    DualMotor,
}

/// <summary>
/// 本地默认通道表与参数表。
/// 真机上这两张表应当由车端下发（PARAM_META 帧）覆盖；
/// 这里内置一份，好处是**没有车也能立刻把上位机跑起来看效果**。
///
/// 通道名/单位/预览/分图预设都是**按档案切换**的静态数组，
/// 其余模块（DataStore / UI / 模拟器）统一通过 ChannelName(i) 等只读入口访问，
/// 因此切换档案后只要重建界面即可，不需要动任何逻辑代码。
/// </summary>
public static class Defaults
{
    public static HostProfile Current { get; private set; } = HostProfile.Kart;

    public static bool IsMotor => Current == HostProfile.Motor;
    public static bool IsDualMotor => Current == HostProfile.DualMotor;

    // ===================== 全车档案（原有 32 通道） =====================
    private static readonly string[] KartChannelNames =
    {
        "yaw",        "gyro_z",     "gyro_bias",  "gps_fix",
        "gps_dx",     "gps_dy",     "enc_L",      "enc_R",
        "mL_tgt",     "mL_act",     "mR_tgt",     "mR_act",
        "steer_tgt",  "steer_act",
        "v_tgt",      "v_act",
        "dist",       "pos_x",      "pos_y",
        "hdg_tgt",    "hdg_act",    "hdg_err",    "lat_err",
        "pp_curv",    "pid_sp",     "pid_si",     "pid_sd",
        "eskf_x",     "eskf_y",     "eskf_yaw",   "loop_us",
        "resv",
    };

    private static readonly string[] KartChannelUnits =
    {
        "°",   "°/s", "°/s", "-",
        "m",   "m",   "cm/s", "cm/s",
        "%",   "%",   "%",    "%",
        "°",   "°",
        "m/s", "m/s",
        "m",   "m",   "m",
        "°",   "°",   "°",    "m",
        "1/m", "-",   "-",    "-",
        "m",   "m",   "°",    "µs",
        "-",
    };

    private static readonly int[] KartDefaultVisible = { 15, 14, 13, 12, 20, 19 };

    private static readonly int[][] KartPlotPresets =
    {
        new[] { 15, 14 },        // 速度：目标 vs 实际
        new[] { 20, 19, 22 },    // 航向：目标 vs 实际 vs 横向偏差
        new[] { 13, 12 },        // 转向：目标 vs 实际
        new[] { 0, 1, 3 },       // 惯导：yaw / 角速度 / GPS 状态
    };

    private static readonly string[] KartPlotPresetNames = { "速度环", "方向环", "转向内环", "惯导" };

    // ===================== 单电机验证档案（12 通道） =====================
    // 下标必须与 firmware motor_link.h 的 CH_* 宏一一对应
    private static readonly string[] MotorChannelNames =
    {
        "duty_cmd",   "duty_act",   "dir",        "pwm_raw",
        "enable",     "online",     "rx_age",     "duty_limit",
        "ramp_ms",    "tx_count",   "uptime",     "init_err",
        "mag_raw",    "mag_angle",  "mag_deg",    "mag_delta",
        "mag_speed",  "mag_sum",    "mag_delta_w","mag_glitch",
    };

    private static readonly string[] MotorChannelUnits =
    {
        "%",   "%",   "",     "",
        "",    "",    "ms",   "%",
        "ms",  "",    "s",    "",
        "raw", "raw", "°",    "raw",
        "rad/s", "mrad", "cnt", "cnt",
    };

    private static readonly int[] MotorDefaultVisible = { 1, 0, 14, 15 };

    private static readonly int[][] MotorPlotPresets =
    {
        new[] { 0, 1 },          // 占空比：目标 vs 实际
        new[] { 2, 3 },          // 方向电平 + PWM 比较值
        new[] { 5, 6 },          // 链路：在线 + 收包间隔
        new[] { 9, 10 },         // 帧计数 + 上电时间（看是不是匀速涨）
    };

    private static readonly string[] MotorPlotPresetNames = { "占空比", "方向/PWM", "链路", "心跳" };

    /// <summary>磁编码器专用分图（单电机档案里另外那 4 通道）</summary>
    private static readonly int[][] MotorEncPresets =
    {
        new[] { 13, 14 },        // 角度：raw 计数 vs 换算后的度数
        new[] { 15, 18 },        // 变化量：老口径 vs 圆上最短差（并排看跨零有没有翻号）
        new[] { 16 },            // 角速度 rad/s（自带符号）
        new[] { 17, 13 },        // 积分角 mrad vs 绝对角 —— 两者是否一致就靠这张图
        new[] { 19 },            // 单帧毛刺累计（SPI 时序余量够不够的客观指标）
        new[] { 10, 5 },         // 上电时间 + 在线
    };

    // ===================== 双电机速度环档案（20 通道） =====================
    // 下标必须与 firmware 双电机工程（CYT4BB7 双电机闭环测速 / WiFi-SPI）的 CH_* 一一对应。
    // 协议一个字节都不改：这里只是换一套"通道怎么显示"，车端不用动。
    private static readonly string[] DualMotorChannelNames =
    {
        "target_l", "v_l",      "u_l",      "target_r", "v_r",      "u_r",
        "uff_l",    "uff_r",    "z_l",      "z_r",
        "pos_l",    "pos_r",    "dlt_l",    "dlt_r",
        "flip_l",   "flip_r",   "online",   "rx_age",   "enable",   "safety",
    };

    private static readonly string[] DualMotorChannelUnits =
    {
        "rps",   "rps",  "%",     "rps",   "rps",   "%",
        "%",     "%",    "rps·s", "rps·s",
        "cnt",   "cnt",  "cnt",   "cnt",
        "-",     "-",    "-",     "ms",    "-",     "-",
    };

    private static readonly int[] DualMotorDefaultVisible = { 1, 0, 4, 3 };

    private static readonly int[][] DualMotorPlotPresets =
    {
        new[] { 0, 1 },          // 左轮速度环：目标 vs 实际
        new[] { 3, 4 },          // 右轮速度环：目标 vs 实际
        new[] { 2, 5, 6, 7 },    // 输出占空比 + 前馈分量（同为 %）
        new[] { 16, 18, 19 },    // 链路：在线 + 使能位图 + 保护位图
    };

    private static readonly string[] DualMotorPlotPresetNames = { "左轮速度环", "右轮速度环", "输出/前馈", "链路/保护" };

    // ===================== 当前生效的表 =====================
    public static string[] ChannelNames = KartChannelNames;
    public static string[] ChannelUnits = KartChannelUnits;
    public static int[] DefaultVisible = KartDefaultVisible;
    public static int[][] PlotPresets = KartPlotPresets;
    public static string[] PlotPresetNames = KartPlotPresetNames;

    /// <summary>档案在 UI 下拉框里的下标（与 MainForm 的 _cbProfile 项一一对应）。</summary>
    public static int ProfileIndex => Current switch
    {
        HostProfile.Motor => 1,
        HostProfile.DualMotor => 2,
        _ => 0,
    };

    /// <summary>档案的中文显示名（日志 / 下拉框用）。</summary>
    public static string ProfileDisplayName => Current switch
    {
        HostProfile.Motor => "单电机验证",
        HostProfile.DualMotor => "双电机速度环",
        _ => "卡丁车",
    };

    /// <summary>切换档案。调用方负责在此之后重建界面（渠道勾选框 / 数值框 / 模拟器）。</summary>
    public static void SetProfile(HostProfile p)
    {
        Current = p;
        switch (p)
        {
            case HostProfile.Motor:
                ChannelNames = MotorChannelNames;
                ChannelUnits = MotorChannelUnits;
                DefaultVisible = MotorDefaultVisible;
                PlotPresets = MotorPlotPresets;
                PlotPresetNames = MotorPlotPresetNames;
                break;
            case HostProfile.DualMotor:
                ChannelNames = DualMotorChannelNames;
                ChannelUnits = DualMotorChannelUnits;
                DefaultVisible = DualMotorDefaultVisible;
                PlotPresets = DualMotorPlotPresets;
                PlotPresetNames = DualMotorPlotPresetNames;
                break;
            default:
                ChannelNames = KartChannelNames;
                ChannelUnits = KartChannelUnits;
                DefaultVisible = KartDefaultVisible;
                PlotPresets = KartPlotPresets;
                PlotPresetNames = KartPlotPresetNames;
                break;
        }
    }

    public static string ChannelName(int i)
        => i >= 0 && i < ChannelNames.Length ? ChannelNames[i] : ("ch" + i);

    public static string ChannelUnit(int i)
        => i >= 0 && i < ChannelUnits.Length ? ChannelUnits[i] : "";

    /// <summary>默认参数表（分组 / 英文名 / 中文标签 / 下限 / 上限 / 默认 / 步长 / 单位）</summary>
    public static List<ParamMeta> BuildDefaultParams() => Current switch
    {
        HostProfile.Motor => BuildMotorParams(),
        HostProfile.DualMotor => BuildDualMotorParams(),
        _ => BuildKartParams(),
    };

    private static List<ParamMeta> BuildKartParams()
    {
        var list = new List<ParamMeta>();
        byte id = 0;

        void P(string grp, string name, string label,
               float lo, float hi, float def, float step, string unit)
        {
            list.Add(new ParamMeta
            {
                Id = id++,
                Group = grp,
                Name = name,
                Label = label,
                Lo = lo,
                Hi = hi,
                Def = def,
                Step = step,
                Unit = unit,
                Value = def,
            });
        }

        // ---- 转向环 ----
        P("转向环", "steer_kp", "转向 Kp", 0f, 100f, 1.50f, 0.05f, "-");
        P("转向环", "steer_ki", "转向 Ki", 0f, 10f, 0.02f, 0.005f, "-");
        P("转向环", "steer_kd", "转向 Kd", 0f, 10f, 0.00f, 0.005f, "-");
        P("转向环", "steer_out_lim", "转向限幅", 0f, 60f, 25.0f, 0.5f, "°");

        // ---- 速度环 ----
        P("速度环", "speed_kp", "速度 Kp", 0f, 200f, 30.0f, 0.5f, "-");
        P("速度环", "speed_ki", "速度 Ki", 0f, 50f, 1.20f, 0.05f, "-");
        P("速度环", "speed_kd", "速度 Kd", 0f, 50f, 0.00f, 0.05f, "-");
        P("速度环", "speed_out_lim", "速度限幅", 0f, 100f, 80.0f, 1.0f, "%");

        // ---- 惯导 ----
        P("惯导", "gyro_bias_x", "陀螺零偏 X", -1f, 1f, 0.000f, 0.001f, "°/s");
        P("惯导", "gyro_bias_y", "陀螺零偏 Y", -1f, 1f, 0.000f, 0.001f, "°/s");
        P("惯导", "gyro_bias_z", "陀螺零偏 Z", -1f, 1f, 0.012f, 0.001f, "°/s");
        P("惯导", "gyro_lpf_hz", "陀螺低通", 0f, 200f, 80.0f, 1.0f, "Hz");
        P("惯导", "zupt_thr", "ZUPT 阈值", 0f, 5f, 0.30f, 0.01f, "cm/s");

        // ---- 里程 ----
        P("里程", "enc_scale_L", "左轮刻度", 0.90f, 1.10f, 1.000f, 0.001f, "-");
        P("里程", "enc_scale_R", "右轮刻度", 0.90f, 1.10f, 1.000f, 0.001f, "-");
        P("里程", "wheel_dia", "轮径", 0.10f, 0.40f, 0.250f, 0.001f, "m");

        // ---- 路径 ----
        P("路径", "look_ahead_base", "前视基准", 0.30f, 1.50f, 0.500f, 0.01f, "m");
        P("路径", "look_ahead_gain", "前视增益", 0f, 0.10f, 0.020f, 0.001f, "-");
        P("路径", "v_max", "最高速度", 0f, 8f, 3.00f, 0.05f, "m/s");
        P("路径", "a_lat_max", "侧向加速度", 0f, 10f, 4.00f, 0.10f, "m/s²");
        P("路径", "a_brake", "制动减速度", 0f, 10f, 2.50f, 0.10f, "m/s²");

        // ---- 控制 ----
        P("控制", "ctrl_period_ms", "控制周期", 1f, 20f, 5.0f, 0.5f, "ms");
        P("控制", "telemetry_hz", "遥测频率", 10f, 200f, 50.0f, 5.0f, "Hz");

        // ---- 调试 ----
        P("调试", "enable_mask", "使能位图", 0f, 255f, 0.0f, 1.0f, "-");
        P("调试", "dbg_ch_mask", "调试通道掩码", 0f, 65535f, 0.0f, 1.0f, "-");

        return list;
    }

    /// <summary>单电机验证档案的参数表 —— 下标必须与 firmware 的 MOTOR_PARAM_TABLE ID 一致。</summary>
    private static List<ParamMeta> BuildMotorParams()
    {
        var list = new List<ParamMeta>();
        byte id = 0;

        void P(string grp, string name, string label,
               float lo, float hi, float def, float step, string unit)
        {
            list.Add(new ParamMeta
            {
                Id = id++,
                Group = grp,
                Name = name,
                Label = label,
                Lo = lo,
                Hi = hi,
                Def = def,
                Step = step,
                Unit = unit,
                Value = def,
            });
        }

        P("电机",   "duty_cmd",   "目标占空比",     -100f, 100f,   0.0f, 0.5f,  "%");
        P("电机",   "duty_limit", "占空比限幅",        0f, 100f,  60.0f, 1.0f,  "%");
        P("电机",   "ramp_ms",    "加减速时间",        0f, 3000f, 400.0f, 10f,  "ms");
        P("电机",   "deadzone",   "启动死区补偿",      0f,  20f,   0.0f, 0.1f,  "%");
        P("保护",   "lost_ms",    "失联停车延时",    100f, 3000f, 600.0f, 10f,  "ms");
        P("保护",   "auto_stop",  "失联自停",          0f,   1f,   1.0f, 1.0f,  "-");
        // 3 = 命令 0x8021 + 低 14 位（实测：bit15=0/bit14=1 是标志位）—— 本机的正确解，设成默认
        P("编码器", "mag_proto",  "编码器协议",        0f,   3f,   3.0f, 1.0f,  "-");   // 0=MENC15A 1=AS5047 2=RAW 3=实测14位
        P("编码器", "mag_dir",    "编码器方向",        0f,   1f,   1.0f, 1.0f,  "-");
        P("编码器", "mag_zero",   "编码器零点",        0f, 65535f, 0.0f, 1.0f, "raw");
        P("编码器", "sum_clr",    "积分清零",          0f,   1f,    0.0f, 1.0f,  "-");      // 拖到 1 清零积分
        P("编码器", "spi_mode",   "SPI 模式",          0f,   3f,    2.0f, 1.0f,  "-");      // TLE5012B 的 SSC 用 2
        P("编码器", "spi_mhz",    "SPI 速率",          1f,  40f,    8.0f, 1.0f,  "MHz");    // 手册推荐 <=8
        P("编码器", "k_speed",    "速度系数",        -10f,  10f, 1.917476f,0.001f, "-");     // rad/s per LSB
        P("编码器", "glitch_th",  "毛刺阈值",          0f,8192f,  512.0f, 1.0f,  "cnt");    // 单帧毛刺判据

        return list;
    }

    /// <summary>
    /// 双电机速度环档案的参数表 —— 目标转速就是通过这里的 target_l / target_r 滑条下发的。
    /// ⚠ 真机连上后会被车端 PARAM_META 下发的表覆盖（ID 以车端为准），这份只是"没车也能跑"的本地回退。
    /// </summary>
    /// <summary>
    /// 双电机速度环（工程 6.speed_loop_wifi_cyt4bb7）的**本地回退**参数表。
    ///
    /// !!! 这张表必须与车端 code/motor_link.h 的 MOTOR_PARAM_TABLE **逐项对齐（ID / 名字 / 量程 / 默认值）** !!!
    /// 理由：车端连上后会用 PARAM_META 覆盖这张表，但**覆盖是按 Id 逐项做的**
    /// （见 ParamPanel.MergeTable：同 Id 更新元数据、新 Id 追加）。
    /// 如果这里的 Id 含义与车端不同，一旦某几帧参数表丢包，
    /// 界面就会留下"命名来自本地、含义来自车端"的错位滑条 ——
    /// 用户照着名字拖一下，就把值写进了完全不相干的参数（最狠的是把占空比限幅写成 0.02%，
    /// 现象是"上位机一切正常但电机不动"）。2026-10-01 踩过。
    /// </summary>
    private static List<ParamMeta> BuildDualMotorParams()
    {
        var list = new List<ParamMeta>();
        byte id = 0;

        void P(string grp, string name, string label,
               float lo, float hi, float def, float step, string unit)
        {
            list.Add(new ParamMeta
            {
                Id = id++,
                Group = grp,
                Name = name,
                Label = label,
                Lo = lo,
                Hi = hi,
                Def = def,
                Step = step,
                Unit = unit,
                Value = def,
            });
        }

        // ---- 与车端 MOTOR_PARAM_TABLE 一一对应（顺序都不能变）----
        P("目标",   "target_l",   "左轮转速",        -8f,      8f,      0f,      0.05f,  "rps");
        P("目标",   "target_r",   "右轮转速",        -8f,      8f,      0f,      0.05f,  "rps");
        P("限幅",   "duty_limit", "占空比上限",       0f,    100f,     30f,     1f,     "%");
        P("限幅",   "target_max", "目标转速上限",     0f,     20f,      3f,     0.1f,   "rps");
        P("增益",   "k_v",        "比例 k_v",         0f,      2f,      0.164f, 0.005f, "-");
        P("增益",   "k_z",        "积分 k_z",         0f,     20f,      2.32f,  0.05f,  "-");
        P("前馈",   "inv_K",      "1/K",              0f,      0.5f,    0.1548f,0.001f, "-");
        P("前馈",   "u0",         "摩擦截距",         0f,      0.3f,    0.033f, 0.001f, "-");
        P("编码器", "cpr_l",      "每转计数L",        1f,  65535f,   1858f,   1f,     "cnt");
        P("编码器", "cpr_r",      "每转计数R",        1f,  65535f,   1858f,   1f,     "cnt");
        P("编码器", "inv_l",      "测速取反L",        0f,      1f,      0f,     1f,     "-");
        P("编码器", "inv_r",      "测速取反R",        0f,      1f,      0f,     1f,     "-");
        P("电机",   "minv_l",     "极性翻转L",        0f,      1f,      0f,     1f,     "-");
        P("电机",   "minv_r",     "极性翻转R",        0f,      1f,      0f,     1f,     "-");
        P("保护",   "lost_ms",    "失联停车延时",   100f,   3000f,    600f,    10f,    "ms");
        P("保护",   "auto_stop",  "失联自停",         0f,      1f,      1f,     1f,     "-");

        return list;
    }
}
