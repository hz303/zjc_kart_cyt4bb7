namespace KartHost;

/// <summary>
/// 遥测通道按档案切换：全车（Kart）、单电机验证（Motor）、双电机速度环（DualMotor）、
/// 转向闭环（SteerLoop，= 双电机速度环 + 转向位置环）、
/// **惯导 + 路表（InsRoute，= 双电机速度环 + 舵机 + 惯导位姿，工程 8）**。
/// ⚠ 枚举值一旦定下就**只能往后加** —— 下拉框下标与本地存档都按序号走。
/// </summary>
public enum HostProfile
{
    Kart,
    Motor,
    DualMotor,
    SteerLoop,
    InsRoute,
    /// <summary>卡丁 · 遥控（工程 9 全量档案，39 通道）：转向闭环 + 遥控 11 个通道 + 增益回读 2 个。</summary>
    KartRC,
    /// <summary>标定 · 精简（工程 9 的 tel_sel=1 档案，18 通道）：只保留两条环路的核心量 + 增益回读。</summary>
    Calib,
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

    /// <summary>全车档案（32 通道，最早的"卡丁车"，带 GPS/航向/位姿误差那一套）。</summary>
    public static bool IsKart => Current == HostProfile.Kart;

    /// <summary>转向闭环档案：后轮速度环 + 转向位置环（工程 7）。</summary>
    public static bool IsSteerLoop => Current == HostProfile.SteerLoop;

    /// <summary>惯导 + 路表档案：后轮速度环 + 舵机 + 惯导位姿（工程 8，C 车）。</summary>
    public static bool IsInsRoute => Current == HostProfile.InsRoute;

    /// <summary>是否属于"车端闭环类"档案（50 Hz 固定遥测、不走 kart 那套参数）。</summary>
    public static bool IsClosedLoop => IsMotor || IsDualMotor || IsSteerLoop || IsInsRoute || IsKartRC || IsCalib;

    /// <summary>卡丁 · 遥控（工程 9 全量档案，39 通道）。</summary>
    public static bool IsKartRC => Current == HostProfile.KartRC;

    /// <summary>标定 · 精简（工程 9 的 tel_sel=1 档案，18 通道）。</summary>
    public static bool IsCalib => Current == HostProfile.Calib;

    /// <summary>该档案是否吃"实车标定"那套界面（两组阶跃 + CSV 录制）。</summary>
    public static bool SupportsCalibPanel => IsKartRC || IsCalib;

    /// <summary>
    /// ★★ 舵机硬限位(°)。**必须与车端 `steer_abs.h` 的 `STEER_ABS_LIMIT_DEG_HARD` 一致。**
    /// 无论遥控还是上位机 s/p 指令，左右极值统一 ±27°，**禁止使用机械极限 30 / 32** ——
    /// 目的是不让位置环主动去顶机械限位（那会让前轮电机堵转、烧 DRV8701）。
    /// 车端还有一道 `steer_ctrl_eff_limit()` 兜底，上位机这里夹是为了"别发出根本不会生效的指令"。
    /// </summary>
    public const float SteerLimitDeg = 27f;

    /// <summary>把角度夹进 ±SteerLimitDeg。</summary>
    public static float ClampSteerDeg(float v)
    {
        if (v > SteerLimitDeg) return SteerLimitDeg;
        if (v < -SteerLimitDeg) return -SteerLimitDeg;
        return v;
    }

    /// <summary>参数是否属于"角度类"，需要按 ±27° 夹紧（steer_tgt 目标角 / steer_lim 软限位）。</summary>
    public static bool IsSteerAngleParam(string name)
        => name == "steer_tgt" || name == "steer_lim";

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

    // ===================== 转向闭环档案（26 通道） =====================
    // 下标必须与 firmware 工程 7（7.steer_loop_wifi_cyt4bb7）motor_link.h 的 CH_* 一一对应。
    // 前 20 个下标与「双电机速度环」**完全一致**（工程 7 = 工程 6 + 转向），
    // 20~25 是新增的转向组 —— 老通道一个没动，所以两个档案可以共用前 20 列的解读。
    private static readonly string[] SteerLoopChannelNames =
    {
        "target_l", "v_l",      "u_l",      "target_r", "v_r",      "u_r",
        "uff_l",    "uff_r",    "z_l",      "z_r",
        "pos_l",    "pos_r",    "dlt_l",    "dlt_r",
        "flip_l",   "flip_r",   "online",   "rx_age",   "enable",   "safety",
        "steer_tgt", "steer_delta", "steer_u", "steer_raw", "steer_en", "steer_status",
    };

    private static readonly string[] SteerLoopChannelUnits =
    {
        "rps",   "rps",  "%",     "rps",   "rps",   "%",
        "%",     "%",    "rps·s", "rps·s",
        "cnt",   "cnt",  "cnt",   "cnt",
        "-",     "-",    "-",     "ms",    "-",     "-",
        // 20~25 转向组：角度是"方向盘度"(deg)、输出是 duty。
        // 2026-10-07 控制律从"编码器计数 + PID"换成"方向盘度 + 纯P/摩擦前馈"后，这三项单位跟着变。
        "deg",   "deg",  "duty",  "raw",   "-",     "-",
    };

    private static readonly int[] SteerLoopDefaultVisible = { 1, 4, 0, 3, 21, 20 };

    private static readonly int[][] SteerLoopPlotPresets =
    {
        new[] { 0, 1 },              // 左轮速度环：目标 vs 实际
        new[] { 3, 4 },              // 右轮速度环：目标 vs 实际
        new[] { 20, 21, 22 },        // 转向位置环：目标转角 vs 实测转角 vs 输出
        new[] { 23, 25 },            // 转向原始角 + 状态位（自检过没过看这里）
        new[] { 16, 18, 19, 24 },    // 链路：在线 + 使能 + 保护 + 转向内环使能
    };

    private static readonly string[] SteerLoopPlotPresetNames = { "左轮速度环", "右轮速度环", "转向位置环", "转向原始/状态", "链路/保护" };

    // ===================== 惯导 + 路表档案（32 通道） =====================
    // 下标必须与 firmware 工程 8（8.ins_route_cyt4bb7）的 code/motor_link.h 的 CH_* 一一对应。
    // ★★ 0~19 与「双电机速度环 / 转向闭环」**完全一致**（三个工程共用同一段速度环通道）；
    //    20~22 是**舵机**（C 车是舵机，不是卡丁那种"直流电机+绝对编码器"，所以 20~25 的语义与
    //    「转向闭环」档案**不同** —— 别把两个档案的 20~25 混着看）；
    //    23~31 是惯导位姿与路表状态（本工程新增）。
    private static readonly string[] InsRouteChannelNames =
    {
        "target_l", "v_l",      "u_l",      "target_r", "v_r",      "u_r",
        "uff_l",    "uff_r",    "z_l",      "z_r",
        "pos_l",    "pos_r",    "dlt_l",    "dlt_r",
        "glitch_l", "glitch_r", "online",   "rx_age",   "enable",   "safety",
        "srv_norm", "srv_duty", "srv_en",
        "ins_roll", "ins_pitch","ins_gz",
        "ins_x",    "ins_y",    "ins_yaw",  "ins_dist", "ins_flags","ins_seq",
    };

    private static readonly string[] InsRouteChannelUnits =
    {
        "rps",   "rps",  "%",     "rps",   "rps",   "%",
        "%",     "%",    "rps·s", "rps·s",
        "cnt",   "cnt",  "cnt",   "cnt",
        "-",     "-",    "-",     "ms",    "-",     "-",
        "-",     "duty", "-",
        "°",     "°",    "°/s",
        "m",     "m",    "°",     "m",     "-",     "-",
    };

    /// <summary>默认可见（数值框）：位姿四件套 + 左右轮实测转速 + 舵量。</summary>
    private static readonly int[] InsRouteDefaultVisible = { 26, 27, 28, 29, 1, 4 };

    private static readonly int[][] InsRoutePlotPresets =
    {
        new[] { 0, 1 },              // 左轮速度环：目标 vs 实际
        new[] { 3, 4 },              // 右轮速度环：目标 vs 实际
        new[] { 26, 27 },            // 位姿：x / y（看轨迹有没有闭合）
        new[] { 28, 25 },            // 航向 yaw vs 陀螺 Z（看航向漂移与真实转动是否一致）
        new[] { 29, 21 },            // 里程 vs 舵机 duty
        new[] { 16, 18, 19, 22 },    // 链路：在线 + 使能 + 保护 + 舵机使能
    };

    private static readonly string[] InsRoutePlotPresetNames = { "左轮速度环", "右轮速度环", "位姿 x/y", "航向/角速度", "里程/舵机", "链路/保护" };

    // ===================== 卡丁 · 遥控档案（39 通道） =====================
    // 下标必须与 firmware 工程 9（9.steer_rc_wifi_cyt4bb7）code/motor_link.h 的 CH_* 一一对应。
    // ★★ 0~25 与「转向闭环」档案**完全一致**（工程 9 = 工程 7 + 遥控），
    //    26~36 是遥控新增的 11 个通道（急停 / 使能 / 指令 / 原始通道值 / 收帧数），
    //    37~38 是 **2026-10-14 新增的"本拍实际生效增益"**（k_v / k_z）。
    // ⚠ 这是**全量**档案（车端 tel_sel = 0）。要看"标定精简"就切到 Calib 档案、并把车端 tel_sel 写 1。
    private static readonly string[] KartRCChannelNames =
    {
        "target_l", "v_l",      "u_l",      "target_r", "v_r",      "u_r",
        "uff_l",    "uff_r",    "z_l",      "z_r",
        "pos_l",    "pos_r",    "dlt_l",    "dlt_r",
        "flip_l",   "flip_r",   "online",   "rx_age",   "enable",   "safety",
        "steer_tgt", "steer_delta", "steer_u", "steer_raw", "steer_en", "steer_status",
        "rc_eng",   "rc_estop", "rc_steer_deg", "rc_thr", "rc_scale", "rc_tgt_rps",
        "rc_ch1",   "rc_ch2",   "rc_ch5",   "rc_ch6",   "rc_frames",
        "k_v",      "k_z",
    };

    private static readonly string[] KartRCChannelUnits =
    {
        "rps",   "rps",  "%",     "rps",   "rps",   "%",
        "%",     "%",    "rps·s", "rps·s",
        "cnt",   "cnt",  "cnt",   "cnt",
        "-",     "-",    "-",     "ms",    "-",     "-",
        "deg",   "deg",  "duty",  "raw",   "-",     "-",
        "-",     "-",    "deg",   "‰",     "‰",     "rps",
        "raw",   "raw",  "raw",   "raw",   "-",
        "-",     "-",
    };

    // ★ 37/38（k_v / k_z）默认就勾上：它们是"车上真正生效的增益"，
    //   跟滑条上显示的值可能不是一回事，所以每张图里都该有一条。
    private static readonly int[] KartRCDefaultVisible = { 1, 4, 21, 20, 27, 31, 37, 38 };

    private static readonly int[][] KartRCPlotPresets =
    {
        new[] { 0, 1 },                      // 左轮速度环：目标 vs 实际
        new[] { 3, 4 },                      // 右轮速度环：目标 vs 实际
        new[] { 20, 21, 22 },                // 转向位置环：目标 / 实测 / 输出
        new[] { 27, 28, 31 },                // 遥控：急停 / 指令角 / 后轮目标
        new[] { 16, 18, 19, 26 },            // 链路：在线 / 使能 / 保护 / 接管状态
        new[] { 37, 38 },                    // ★ 增益回读：车上真正生效的 k_v / k_z
    };

    private static readonly string[] KartRCPlotPresetNames = { "左轮速度环", "右轮速度环", "转向位置环", "遥控", "链路/保护", "增益" };

    // ===================== 标定 · 精简档案（18 通道） =====================
    // 车端 `tel_sel = 1` 时发的就是这一套（见 firmware motor_link.h 第五节 CT_*）。
    // 为什么另起一套：落地标定要长时间高密度采数据，18 通道一帧 93 字节（全量 39 通道 177 字节），
    // 同样的链路能把遥测周期压到 5~10 ms —— 阶跃响应的时间分辨率就是这个周期。
    // 通道顺序刻意排成"三路一组"：速度环左 / 速度环右 / 舵机角度环，
    // 这样分图预设直接三通道一图，阶跃响应一眼就能读。
    // ★ 2026-10-14 尾部加 16/17 = **车上真正生效的 k_v / k_z**（增益回读，16 通道 -> 18 通道）。
    private static readonly string[] CalibChannelNames =
    {
        "target_l", "v_l",      "u_l",
        "target_r", "v_r",      "u_r",
        "steer_tgt", "steer_delta", "steer_u",
        "online",   "enable",   "safety",
        "steer_en", "steer_st", "ol_en",   "resv0",
        "k_v",      "k_z",
    };

    private static readonly string[] CalibChannelUnits =
    {
        "rps",  "rps",  "%",
        "rps",  "rps",  "%",
        "deg",  "deg",  "duty",
        "-",    "-",    "-",
        "-",    "-",    "-",    "-",
        "-",    "-",
    };

    private static readonly int[] CalibDefaultVisible = { 1, 0, 4, 3, 7, 6, 16, 17 };

    private static readonly int[][] CalibPlotPresets =
    {
        new[] { 0, 1, 2 },                   // 速度环(左)：目标 / 实际 / PWM  ← 开环阶跃看这张
        new[] { 3, 4, 5 },                   // 速度环(右)
        new[] { 6, 7, 8 },                   // 舵机角度环：目标 / 实际 / PWM ← 角度阶跃看这张
        new[] { 9, 10, 11, 12, 13, 14 },     // 状态：链路 / 使能 / 保护 / 舵机使能 / 舵机状态 / 开环
        new[] { 16, 17 },                    // ★ 增益回读：车上真正生效的 k_v / k_z
    };

    private static readonly string[] CalibPlotPresetNames = { "速度环L", "速度环R", "舵机角度环", "状态", "增益" };

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
        HostProfile.SteerLoop => 3,
        HostProfile.InsRoute => 4,
        HostProfile.KartRC => 5,
        HostProfile.Calib => 6,
        _ => 0,
    };

    /// <summary>档案的中文显示名（日志 / 下拉框用）。</summary>
    public static string ProfileDisplayName => Current switch
    {
        HostProfile.Motor => "单电机验证",
        HostProfile.DualMotor => "双电机速度环",
        HostProfile.SteerLoop => "转向闭环",
        HostProfile.InsRoute => "惯导+路表",
        HostProfile.KartRC => "卡丁·遥控",
        HostProfile.Calib => "标定·精简",
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
            case HostProfile.SteerLoop:
                ChannelNames = SteerLoopChannelNames;
                ChannelUnits = SteerLoopChannelUnits;
                DefaultVisible = SteerLoopDefaultVisible;
                PlotPresets = SteerLoopPlotPresets;
                PlotPresetNames = SteerLoopPlotPresetNames;
                break;
            case HostProfile.InsRoute:
                ChannelNames = InsRouteChannelNames;
                ChannelUnits = InsRouteChannelUnits;
                DefaultVisible = InsRouteDefaultVisible;
                PlotPresets = InsRoutePlotPresets;
                PlotPresetNames = InsRoutePlotPresetNames;
                break;
            case HostProfile.KartRC:
                ChannelNames = KartRCChannelNames;
                ChannelUnits = KartRCChannelUnits;
                DefaultVisible = KartRCDefaultVisible;
                PlotPresets = KartRCPlotPresets;
                PlotPresetNames = KartRCPlotPresetNames;
                break;
            case HostProfile.Calib:
                ChannelNames = CalibChannelNames;
                ChannelUnits = CalibChannelUnits;
                DefaultVisible = CalibDefaultVisible;
                PlotPresets = CalibPlotPresets;
                PlotPresetNames = CalibPlotPresetNames;
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

    // ==================================================================
    //  ★★ 「帧里的通道数」→「该用哪个档案」的反查（2026-10-14 新增）
    //
    //  起因（现场踩到的）：车端在发**全量 39 通道**，而本机档案停在**「标定·精简」(18)**。
    //  后果不是"报错"，而是**通道名整体错位 21 个下标** —— 于是看图的人看到的是：
    //      · 表头写着 `k_v` 的那一列，其实是 ch16 = `online`（恒 1）
    //      · 表头写着 `k_z` 的那一列，其实是 ch17 = `rx_age`（**毫秒计数器**，
    //        被上位机 100 ms 心跳周期性清零 ⇒ 画出来是一条 0→100 的**锯齿**）
    //  两个"增益"都在剧烈跳动、看着还挺像回事，**没有任何报错**。
    //  ⇒ 所以这里必须有一个**不依赖 UI 状态**的纯查询：给定帧里的 COUNT，
    //    告诉我"这可能是哪个档案"。唯一命中就允许一键切过去；命中两个（32 / 20）就只提示。
    //
    //  ⚠ 注意 `ChannelCountOf` **不许改 Current** —— 它会被 Paint 路径调用。
    // ==================================================================
    public static int ChannelCountOf(HostProfile p) => p switch
    {
        HostProfile.Motor => MotorChannelNames.Length,
        HostProfile.DualMotor => DualMotorChannelNames.Length,
        HostProfile.SteerLoop => SteerLoopChannelNames.Length,
        HostProfile.InsRoute => InsRouteChannelNames.Length,
        HostProfile.KartRC => KartRCChannelNames.Length,
        HostProfile.Calib => CalibChannelNames.Length,
        _ => KartChannelNames.Length,
    };

    /// <summary>通道数恰好等于 <paramref name="n"/> 的所有档案。
    /// ★ 唯一命中（26 / 39 / 18 …）⇒ 可以安全地"一键切过去"；
    ///   命中两个（**32 = 卡丁车/惯导+路表、20 = 单电机/双电机**）⇒ 只能提示，别猜。</summary>
    public static List<HostProfile> ProfilesWithChannelCount(int n)
        => Enum.GetValues<HostProfile>().Where(p => ChannelCountOf(p) == n).ToList();

    /// <summary>
    /// ★★ 右侧栏「实时值」那 6 格数值框的**取数方案 —— 每个档案一套**。
    ///
    /// ⚠⚠ 这里是按**各档案自己的下标**硬取的，所以每加一个档案就必须加一支，
    /// 否则会落到最后那支"卡丁车"的老下标 {15,20,22,13,0,30} 上，而新档案里
    /// 这些下标指向毫不相干的通道。2026-10-13 现场就是这么被抓住的：
    /// 「卡丁·遥控」档案里 ch15 = flip_r（右后轮**换向次数**）、ch30 = rc_scale
    /// （遥控油门行程 ‰），于是"速度"格显示 21.0 m/s、"控制周期"格显示 200.0 µs ——
    /// **下标不越界、数字看着还挺像、单位也对得上，就是含义全错**，不报任何错。
    /// ⇒ SelfTest 里有断言：每个档案的 6 个下标都必须落在自己的通道表里，
    ///   且第 1 格必须是真正的转速通道（v_l）。
    /// </summary>
    public static (int[] Keys, string[] Caps, string[] Units) ValueBoxes() => Current switch
    {
        HostProfile.Motor => (new[] { 1, 0, 14, 13, 15, 6 },
                              new[] { "实际占空比", "目标占空比", "编码器角度", "角度 raw", "角度变化", "收包间隔" },
                              new[] { "%", "%", "°", "raw", "raw", "ms" }),

        HostProfile.DualMotor => (new[] { 1, 4, 0, 3, 10, 11 },
                                  new[] { "左轮实测", "右轮实测", "左轮目标", "右轮目标", "左轮计数", "右轮计数" },
                                  new[] { "rps", "rps", "rps", "rps", "cnt", "cnt" }),

        HostProfile.SteerLoop => (new[] { 1, 4, 21, 20, 22, 25 },
                                  new[] { "左轮实测", "右轮实测", "实测转角", "目标转角", "转向输出", "转向状态" },
                                  new[] { "rps", "rps", "°", "°", "-", "-" }),

        HostProfile.InsRoute => (new[] { 26, 27, 28, 29, 1, 4 },
                                 new[] { "位姿 X", "位姿 Y", "航向 yaw", "累计里程", "左轮实测", "右轮实测" },
                                 new[] { "m", "m", "°", "m", "rps", "rps" }),

        // 卡丁·遥控（工程 9 全量 39 通道）：先看左右轮转速，再看转向位置环三件套，
        // 最后是"遥控接管状态"（0 OFF / 1 STANDBY / 2 ON / 3 FAILSAFE）。
        // （k_v / k_z 回读不占这 6 格 —— 它们看波形与 CSV，见「增益」分图预设）
        HostProfile.KartRC => (new[] { 1, 4, 21, 20, 22, 26 },
                               new[] { "左轮实测", "右轮实测", "实测转角", "目标转角", "转向输出", "接管状态" },
                               new[] { "rps", "rps", "°", "°", "-", "-" }),

        // 标定·精简（18 通道）：与分图预设同一套"三路一组"——
        // 左轮 目标/实测、右轮 目标/实测、舵机 实测/目标（阶跃响应盯这 6 个数就够）。
        HostProfile.Calib => (new[] { 1, 0, 4, 3, 7, 6 },
                              new[] { "左轮实测", "左轮目标", "右轮实测", "右轮目标", "实测转角", "目标转角" },
                              new[] { "rps", "rps", "rps", "rps", "°", "°" }),

        _ => (new[] { 15, 20, 22, 13, 0, 30 },
              new[] { "速度", "航向", "航向偏差", "转向角", "yaw", "控制周期" },
              new[] { "m/s", "°", "°", "°", "°", "µs" }),
    };

    /// <summary>默认参数表（分组 / 英文名 / 中文标签 / 下限 / 上限 / 默认 / 步长 / 单位）</summary>
    public static List<ParamMeta> BuildDefaultParams() => Current switch
    {
        HostProfile.Motor => BuildMotorParams(),
        HostProfile.DualMotor => BuildDualMotorParams(),
        HostProfile.SteerLoop => BuildSteerLoopParams(),
        HostProfile.InsRoute => BuildInsRouteParams(),
        // 工程 9 的两个档案共用同一张参数表（车端参数表与遥测档案无关，都是 34 项）
        HostProfile.KartRC => BuildKartRCParams(),
        HostProfile.Calib => BuildKartRCParams(),
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
        P("编码器", "inv_r",      "测速取反R",        0f,      1f,      1f,     1f,     "-");
        P("电机",   "minv_l",     "极性翻转L",        0f,      1f,      1f,     1f,     "-");
        P("电机",   "minv_r",     "极性翻转R",        0f,      1f,      0f,     1f,     "-");
        P("保护",   "lost_ms",    "失联停车延时",   100f,   3000f,    600f,    10f,    "ms");
        P("保护",   "auto_stop",  "失联自停",         0f,      1f,      1f,     1f,     "-");

        return list;
    }

    /// <summary>
    /// 转向闭环（工程 7.steer_loop_wifi_cyt4bb7）的**本地回退**参数表。
    ///
    /// !!! 与 BuildDualMotorParams 同样：必须与车端 code/motor_link.h 的 MOTOR_PARAM_TABLE
    /// **逐项对齐（ID / 名字 / 量程 / 默认值）** !!!
    /// 前 18 项与「双电机速度环」一字不差，18~26 是转向新增的（ID 只能往后加）。
    ///
    /// ⚠ 转向的标定顺序：① 手推方向盘到左右极限，核 `steer_abs.h` 里中位/左右限位三个 raw，
    ///   并确认 `get_deg()` 的符号（约定：左 ≈ +30 度、右 ≈ -30 度；反了改 DIR_SIGN）；
    ///   ② 开 `steer_en`，先只加 `steer_u0` 到"刚好能把方向盘推动"（队友那台约 1300）；
    ///   ③ 再加 `steer_kp` 到阶跃响应不振荡；④ 方向不对改 `minv_s`（在线可改）。
    /// ⚠ 后轮四个极性开关已按 **2026-10-07 晚实测**填好（引脚对调后重标）：
    ///   `minv_l = 1 / minv_r = 0 / inv_l = 0 / inv_r = 1`。换接线或换电机后这四个要重标。
    /// </summary>
    private static List<ParamMeta> BuildSteerLoopParams()
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

        // ---- 后轮速度环（与车端 MOTOR_PARAM_TABLE ID 0~17 一一对应，顺序不能变）----
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
        P("编码器", "inv_r",      "测速取反R",        0f,      1f,      1f,     1f,     "-");
        P("电机",   "minv_l",     "极性翻转L",        0f,      1f,      1f,     1f,     "-");
        P("电机",   "minv_r",     "极性翻转R",        0f,      1f,      0f,     1f,     "-");
        P("保护",   "lost_ms",    "失联停车延时",   100f,   3000f,    600f,    10f,    "ms");
        P("保护",   "auto_stop",  "失联自停",         0f,      1f,      1f,     1f,     "-");
        P("限幅",   "slew",       "目标斜坡",         0f,    200f,     20f,     1f,     "rps/s");
        P("编码器", "enc_win",    "滤波窗口",         1f,     16f,      4f,     1f,     "-");

        // ---- 转向位置环（ID 18~26，工程 7）----
        // ★ 2026-10-07: 控制律由 PI 换成"纯 P + 静摩擦前馈 u=Kp*err+u0*sign(err)"（抄队友已验证的方案），
        //   单位由"编码器计数"换成"方向盘度"，故 ID 20/21 的语义变了（Ki/Kd -> u0/死区），25/26 是新增的斜坡与软限位。
        //   旧存档里的值会对不上，用之前请点一次"恢复默认"。
        // ★ Kp=300 / U0=1600 / minv_s=1 是**队友在实车上验证过的值**（两边同一套硬件：同引脚、同 dir 编码器），
        //   所以上电就是可用状态；若实测方向反了，先把 minv_s 拖回 0。
        P("转向",   "steer_tgt",  "目标转角",      -32f,     32f,      0f,    0.5f,   "deg");
        P("转向",   "steer_kp",   "比例 Kp",          0f,    500f,    300f,    1f,     "-");
        P("转向",   "steer_u0",   "摩擦前馈 U0",      0f,   3500f,   1600f,   10f,     "duty");
        P("转向",   "steer_db",   "误差死区",         0f,      5f,     0.2f,  0.05f,  "deg");
        // ★★ 2026-10-15：输出上限默认 3500 → **9000(90%)**（用户拍板）。
        //   影响不是"力气大一点"，而是**饱和门槛从 |err| ≈ 6.3° 抬到 ≈ 24.7°**
        //   ⇒ 从"超过 6.3° 就恒出 35%"变成"几乎全行程按 Kp·err 比例给力"：
        //     前段明显更快、也明显更冲。风险与撤回见车端 steer_ctrl.h 的 STEER_U_MAX_DEFAULT 注解。
        P("转向",   "steer_umax", "输出上限",         0f,  10000f,   9000f,  100f,    "duty");
        P("转向",   "steer_en",   "内环使能",         0f,      1f,      0f,    1f,     "-");
        P("转向",   "minv_s",     "电机极性",         0f,      1f,      1f,    1f,     "-");
        // ★★ 2026-10-14：目标斜坡默认由 40 改成 **0（关）** —— 照队友实车结论
        //   （YINGYYUN/Karting-Project commit 4ce6c93「删除了舵机的目标斜坡」）：
        //   本环是纯 P + 静摩擦前馈、**没有积分器**，阶跃不需要防裹死；误差已被 ±27 软限位卡住，
        //   Kp·err 直接顶到 u_max 才是"最快到位"。留着斜坡只剩纯滞后
        //   （我们这条 40 度/秒 ⇒ 0→27° 要 675 ms，队友那条 120 度/秒 也要 225 ms）。
        //   ⚠ 滑条还在 0~200，想研究斜坡随时拖回去；默认上车就是"没有斜坡"。
        P("转向",   "steer_slew", "目标斜坡(0=关)",   0f,    200f,     0f,    1f,     "deg/s");
        P("转向",   "steer_lim",  "软限位",           0f,     30f,     28f,    0.5f,   "deg");

        return list;
    }

    /// <summary>
    /// 惯导 + 路表（工程 8.ins_route_cyt4bb7，C 车）的**本地回退**参数表。
    ///
    /// <summary>
    /// 工程 9（卡丁 · 遥控）的参数表 —— 34 项。
    /// 与车端 code/motor_link.h 的 MOTOR_PARAM_TABLE **逐条对应**（ID 顺序不能变）。
    /// ⚠ 与「转向闭环」只差三处：ID 26 的软限位上限定成 ±27（不许 30）、
    ///   ID 27/28 遥控、ID 29~33 标定（开环 duty / 遥测档案 / 遥测周期）。
    /// </summary>
    private static List<ParamMeta> BuildKartRCParams()
    {
        var list = BuildSteerLoopParams();

        // ID 18 目标转角 与 ID 26 软限位：上限都收成 ±27（与 steer_abs.h 的硬限位一致，机械极限 30/32 不许用）
        var tgt = list.First(p => p.Id == 18);
        tgt.Lo = -SteerLimitDeg; tgt.Hi = SteerLimitDeg; tgt.Def = 0f; tgt.Value = 0f;

        var lim = list.First(p => p.Id == 26);
        lim.Lo = 0f; lim.Hi = SteerLimitDeg; lim.Def = SteerLimitDeg; lim.Value = SteerLimitDeg;

        byte id = 27;
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

        // ---- 遥控（ID 27~28）----
        P("遥控",   "rc_arm_en",   "允许遥控接管",   0f,      1f,     1f,     1f,     "-");
        P("遥控",   "rc_thr_max",  "满杆转速",       0f,     20f,     2f,     0.1f,   "rps");

        // ---- 标定（ID 29~33）----
        // ol_en=1 时两路都走开环：直接把 ol_duty_* 当占空比给出去（绕过速度环），
        // 落地做"开环阶跃"就用这两个滑条 —— 见 待办-实车标定.md 与 spd_ctrl.h。
        P("标定",   "ol_en",       "开环模式",       0f,      1f,     0f,     1f,     "-");
        P("标定",   "ol_duty_l",   "左轮开环duty", -100f,   100f,     0f,     1f,     "%");
        P("标定",   "ol_duty_r",   "右轮开环duty", -100f,   100f,     0f,     1f,     "%");
        // tel_sel=1 -> 车端改发「标定·精简」18 通道；切完**本机档案下拉框也要切到「标定·精简」**。
        P("标定",   "tel_sel",     "遥测档案0全1精", 0f,      1f,     0f,     1f,     "-");
        P("标定",   "tel_period",  "遥测周期",       5f,    200f,    20f,     1f,     "ms");

        return list;
    }

    /// !!! 与 BuildDualMotorParams / BuildSteerLoopParams 同理：必须与车端 code/motor_link.h 的
    /// MOTOR_PARAM_TABLE **逐项对齐（ID / 名字 / 量程 / 默认值）** !!!
    /// 覆盖是按 Id 逐项做的（ParamPanel.MergeTable），错位会让用户照着名字拖一下，
    /// 把值写进完全不相干的参数。
    ///
    /// ★★ 前 18 项与「双电机速度环 / 转向闭环」一字不差（都用 20.zf_clean 那套 K≈215 的速度环）；
    ///    18/19 是本档案新增的输出上限与卡尔曼开关（原「转向闭环」的 18/19 是 enc_win 之后的转向组，
    ///    两个档案 ID 18 之后的语义**不同** —— 所以本档案的参数表必须整份替换，不能只补尾巴）。
    ///
    /// ★ 标定顺序（不能颠倒）：
    ///   ① 电机方向 minv_l/minv_r → ② 计数正负 inv_l/inv_r → ③ **每转计数 cpr（默认 1024，
    ///      因为 CYT4BB7 库的正交是 X1 倍频，MM32 那边才是 X4=4096）** →
    ///   ④ **比例尺 ins_mpm（2026-10-11 实测定稿 0.0895）** ——
    ///      1 m ×8 + 2 m ×4 共 12 次合并（均值 0.089483、中位数 0.089700、95% CI ±0.73%）；
    ///      等效轮周长 0.2028 m（Ø64.6 mm）。⚠ 手推标定的 σ 地板 ≈0.66%（打滑），
    ///      加长基线 1→3 m 也只从 1.27% 到 0.75% ⇒ 再想更准只能"电机驱动标定"或"整圈+地图真值"。
    ///   ⑤ yaw 符号（编译期 INS_YAW_SIGN）→ ⑥ 磁力计 ins_magcal（打 1 → 绕轴各转一圈 → 打 0 结算）。
    ///
    /// ⚠⚠ **四个极性开关的默认值已按 C 车 2026-10-09 实测写入**：`inv_r = 1 / minv_r = 1`，
    ///    `inv_l = 0 / minv_l = 0`。**与「卡丁车」档案（minv_l=1/minv_r=0）不同，别互相抄**：
    ///    卡丁与 C 车是两套电机/接线；而且本工程的右轮 PWM/DIR 已从 P05_2/P05_3 挪到 P17_4/P17_3。
    ///    车端对应三处必须一致：`motor_link.h` 参数表默认值 + `spd_enc.h` / `motor_drv.h` 的默认宏。
    /// </summary>
    private static List<ParamMeta> BuildInsRouteParams()
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

        // ---- 速度环（与车端 MOTOR_PARAM_TABLE ID 0~17 一一对应，顺序不能变）----
        // 参数值来自 C 车现役工程 20.zf_clean（**不是卡丁的**：卡丁 K≈6.46 / k_v=0.164 / k_z=2.32）。
        P("目标",   "target_l",   "左轮转速",       -30f,     30f,      0f,      0.5f,   "rps");
        P("目标",   "target_r",   "右轮转速",       -30f,     30f,      0f,      0.5f,   "rps");
        P("限幅",   "duty_limit", "占空比上限",       0f,    100f,     30f,     1f,     "%");
        P("限幅",   "target_max", "目标转速上限",     0f,     60f,     20f,     1f,     "rps");
        P("增益",   "k_v",        "比例 k_v",         0f,      1f,      0.014f, 0.001f, "-");
        P("增益",   "k_z",        "积分 k_z",         0f,      5f,      0.070f, 0.005f, "-");
        P("前馈",   "inv_K",      "1/K（1/215）",     0f,      0.05f,   0.004651f,0.0001f,"-");
        P("前馈",   "u0",         "摩擦截距",         0f,      0.3f,    0.053f, 0.001f, "-");
        P("编码器", "cpr_l",      "每转计数L",        1f,  65535f,   1024f,   1f,     "cnt");
        P("编码器", "cpr_r",      "每转计数R",        1f,  65535f,   1024f,   1f,     "cnt");
        P("编码器", "inv_l",      "计数取反L",        0f,      1f,      0f,     1f,     "-");
        P("编码器", "inv_r",      "计数取反R",        0f,      1f,      1f,     1f,     "-");
        P("电机",   "minv_l",     "极性翻转L",        0f,      1f,      0f,     1f,     "-");
        P("电机",   "minv_r",     "极性翻转R",        0f,      1f,      1f,     1f,     "-");
        P("保护",   "lost_ms",    "失联停车延时",   100f,   3000f,    600f,    10f,    "ms");
        P("保护",   "auto_stop",  "失联自停",         0f,      1f,      1f,     1f,     "-");
        P("限幅",   "slew",       "目标斜坡",         0f,    500f,     50f,     1f,     "rps/s");
        P("编码器", "enc_win",    "滤波窗口",         1f,     16f,      4f,     1f,     "-");
        P("增益",   "u_lim",      "输出上限",         0f,      1f,      0.8f,   0.01f,  "-");
        P("增益",   "use_kf",     "用卡尔曼",         0f,      1f,      1f,     1f,     "-");

        // ---- 舵机（ID 20/21；C 车是舵机，不是卡丁的转向位置环）----
        //   duty 口径：中位 2420、右极限 2160、左极限 2660（**左右不对称**：260 / 240）
        P("舵机",   "srv_en",     "舵机使能",         0f,      1f,      0f,     1f,     "-");
        P("舵机",   "srv_norm",   "归一化舵量",      -1f,      1f,      0f,     0.01f,  "-");

        // ---- 惯导 + 路表（ID 22~27）----
        //   ⚠ ins_mpm 默认 0.0895 是 **2026-10-11 实测值**（12 次合并，CI ±0.73%），见方法注释 ④
        //   ⚠ ins_route / ins_zero / ins_magcal 是**边沿命令**：车端按 0→1（ins_route 按 0→1 开、1→0 停）
        //     处理一次，拖到 1 之后要拖回 0 才能再次触发。
        P("惯导",   "ins_mpm",    "米每转（比例尺）", 0f,      5f,      0.0895f, 0.005f, "m/rev");
        P("惯导",   "ins_odom_en","航位推算使能",     0f,      1f,      1f,     1f,     "-");
        P("惯导",   "ins_slip",   "打滑门限",         0f,    100f,     30f,     1f,     "%");
        P("路表",   "ins_route",  "录制开关(0→1开,1→0停)", 0f, 1f,     0f,     1f,     "-");
        P("惯导",   "ins_zero",   "位姿归零(0→1)",    0f,      1f,      0f,     1f,     "-");
        P("惯导",   "ins_magcal", "磁标定(0→1开,1→0结算)", 0f, 1f,     0f,     1f,     "-");

        // ---- ID 28~33：循迹（route_cruise）。★ 与车端 motor_link.h 同名同序，**只能往后加** ----
        //    ⚠ 这几个原来只能在车端的屏幕上改；加进参数表后上位机也能调，而且能被「保存参数」落 Flash。
        P("循迹",   "crs_kp",     "比例 kp",           0f,     5f,     0.80f,  0.01f,  "-");
        P("循迹",   "crs_kp2",    "非线性 kp2",        0f,     1f,     0.020f, 0.001f, "-");
        P("循迹",   "crs_kd",     "微分 kd",           0f,     5f,     0.40f,  0.01f,  "-");
        P("循迹",   "crs_ufull",  "满舵误差角",        1f,   180f,    30f,     1f,     "deg");
        P("循迹",   "crs_speed",  "巡航基速",          0f,    30f,     8f,     0.5f,   "rps");
        // ⚠ 符号项：下限 -1 / 上限 +1 / 步长 1 ⇒ 只能取 ±1。
        //   车端吸附已按"四舍五入到最近步长"实现（负数也吸得对）——
        //   老实现 `(int32)(v/step + 0.5f)` 会把 -1 吸成 0，现象是"舵向反了却怎么改都改不过来"。
        P("循迹",   "crs_sign",   "舵向符号 ±1",      -1f,     1f,     1f,     1f,     "-");

        // ---- 舵机「工作点」（车端 ID 34~37，2026-10-12）----
        //   为什么单独给四项：1210/1080/1330 µs 与 200 Hz 是从 MM32 工程抄来的**声明值**，
        //   不是在这台车上实测的。现场「舵机嗡嗡响 + 不受控」通常就是频率不对
        //   （有些模拟舵机只认 50~60 Hz）或脉宽已经顶到机械限位 ⇒ 必须能在线试。
        //   ⚠ 改 srv_freq 之后由车端主循环的 steer_srv_poll() 重开 PWM（不在 5ms 中断里）。
        P("舵机",   "srv_freq",   "PWM 频率",         20f,   400f,   200f,   10f,    "Hz");
        P("舵机",   "srv_usmid",  "中位脉宽",        200f,  2500f,  1210f,    5f,    "us");
        P("舵机",   "srv_usmin",  "右极限脉宽",      200f,  2500f,  1080f,    5f,    "us");
        P("舵机",   "srv_usmax",  "左极限脉宽",      200f,  2500f,  1330f,    5f,    "us");

        // ---- 巡航记录采样周期（车端 ID 38，2026-10-12）----
        //   Cruise 跑完会把"实际轨迹 + 舵机 duty"写进一个槽（车端 INS_ROUTE_LOG_SLOT）。
        //   采样是**等时间**的 ⇒ 周期决定一次能记多久：20 ms(50 Hz) ⇒ 2040 条 = 40.8 s。
        //   ⚠ 抖动是时域现象，周期调大就看不清了 —— 只在"圈太长记不下"时才调大。
        P("记录",   "log_ms",     "巡航记录采样周期",   5f,   200f,    20f,    5f,    "ms");

        return list;
    }
}
