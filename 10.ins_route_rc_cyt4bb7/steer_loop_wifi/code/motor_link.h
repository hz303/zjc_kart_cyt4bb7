/* 转向位置环 + 双电机速度环(WiFi) - 车端链路(WiFi-SPI)
 * 作用: 把本工程的实时数据发给上位机 KartHost, 并接收上位机下发的参数 / 使能命令。
 * 本工程**没有开环通道**: 上位机直接下发"目标转速 rps", 闭环在车端跑。
 * 协议(与上位机 KartHost 完全对齐, 不要改):
 * 上行  MAGIC(2) + TYPE(1) + LEN(2) + PAYLOAD + CRC16(2) + TAIL(4)   总长 11+LEN
 * 下行  MAGIC(2) + TYPE(1) + LEN(2) + PAYLOAD + CRC16(2)             总长  7+LEN
 * MAGIC: 上行 A5 5A / 下行 5A A5; 全部小端
 * CRC16-MODBUS(0xA001) 覆盖 TYPE~PAYLOAD; CRC 不过只前移 1 字节(不能跳整帧)
 * TAIL = 00 00 80 7F(与 VOFA+ JustFloat 一致, 必要时可先用 VOFA+ 验证链路通不通)
 * 硬件占用: SPI_0 @10MHz(SCK P02_2 / MOSI P02_1 / MISO P02_0 / CS P02_3)
 * INT P02_4(握手, 低=模块忙)、RST P23_0、5V 供电
 * (定义在 libraries/zf_device/zf_device_wifi_spi.h)
 * 使用三步:
 * 1) 改下面第一节的 WiFi 名/密码/上位机 IP
 * 2) main 里初始化完外设之后调 motor_link_init()
 * 3) while(1) 里调 motor_link_task(); 并在 5ms 的 PIT 中断里调 motor_link_tick_5ms()
 */

#ifndef _MOTOR_LINK_H
#define _MOTOR_LINK_H

#include "zf_common_headfile.h"

/* 参数表要取这些变量的地址 —— 它们分散在各自的模块里(编码器/控制器/保护) */
#include "spd_enc.h"
#include "spd_ctrl.h"
#include "spd_safety.h"
#include "motor_drv.h"
#include "steer_ctrl.h"          /* 转向位置环(参数表要取它的成员地址) */
#include "steer_abs.h"           /* 转向绝对编码器 */
#include "remote_ctrl.h"
#include "ins_odom.h"
#include "ins_route.h"
#include "route_app.h"
#include "route_pb.h"

/* spd_tel_sel    0 = **全量档案**（36 通道，日常/整机用）
 * 1 = **标定精简档案**（18 通道，见本文件第五节 CT_*）
 * 39 通道一帧 177 B、@20 ms；18 通道只有 93 B ⇒ 同样带宽能把周期压到 5~10 ms，
 * 或者反过来在 20 ms 下把丢包率压到接近 0。**换档案只改这一个参数，不用重烧。**
 * 否则通道名对不上（数还是那些数，只是名字错位）。
 */
extern volatile float   spd_tel_sel;        /* 0 = 全量(36), 1 = 标定精简(18) */
extern volatile float   spd_tel_period;     /* 遥测周期 ms(5~200) */

/* 一、用户必改: WiFi 与上位机 */
/* MOTOR_NET_PROFILE = 0  ->  手机热点 XIOQ-3695   （手机跑 KartPhone App）
 * MOTOR_NET_PROFILE = 1  ->  实验室 IoT 网        （PC 跑 KartHost）
 * 只会往这一个地址发包。**写错了两边都不报错、一个包都收不到** —— 这是"没反应"最高频的原因。
 * · 这台手机实测是 **192.168.254.177**（254 段，而且不是 .1）；
 * · 换手机/换 ROM 后别猜 —— 打开 KartPhone，连接页「本机 IP」只显示热点接口那一个地址，
 * 备注里带接口名（ap0 / swlan0 / wlan1）。`ccmni*` 是移动数据、`wlan0` 是 STA 上网，都不是。
 * （如 192.168.50.255），这样 PC 端 KartHost 照样能收，且不怕 IP 变。
 */
#define MOTOR_NET_PROFILE       (1)

#if   (MOTOR_NET_PROFILE == 0)
  #define MOTOR_WIFI_SSID       "XIOQ-3695"             /* 手机热点名 */
  #define MOTOR_WIFI_PASSWORD   "123456890"             /* 热点密码(无密码填 NULL) */
  #define MOTOR_HOST_IP         "192.168.254.177"       /* 手机 ap0 地址(实测) */
#elif (MOTOR_NET_PROFILE == 1)
  #define MOTOR_WIFI_SSID       "ASC_Circuit_IoT"       /* 实验室 IoT 网 */
  #define MOTOR_WIFI_PASSWORD   "1145141919810"
  #define MOTOR_HOST_IP         "192.168.50.176"        /* 那台 PC */
#else
  #error "MOTOR_NET_PROFILE must be 0 (phone hotspot) or 1 (lab IoT)"
#endif

#define MOTOR_HOST_PORT         "8086"                  /* 与上位机/App 里的端口一致 */
#define MOTOR_LOCAL_PORT        "6666"                  /* 本机端口, 一般不用改 */

/* TCP / UDP 二选一。**改了这里, 上位机左上角的下拉也要选同一个**, 否则收不到数据。
 * 怎么选: 想要"丢帧也不拖累主循环、上位机随时开关" -> UDP;
 * 想要"每帧都到、由模块负责重连"            -> TCP(默认)。
 */
#define MOTOR_SOCK_TYPE         "UDP"

/* 二、可调项 */
#define MOTOR_TX_PERIOD_MS      (20)        /* 遥测周期 ms: 20 -> 50Hz */
#define MOTOR_RX_PERIOD_MS      (10)        /* 收命令周期 ms: 10 -> 100Hz */
#define MOTOR_CH_COUNT          (36)        /* 全量档案的遥测通道数(见下面第四节)。 每多 1 通道 = 每帧多 4 B、50 Hz 下多 200 B/s。 砍掉的 12 个都是"中间量/一次性标定量"： dlt_l/dlt_r（本拍增量，pos 差分即得） flip_l/flip_r（换向丢拍，老问题已消） steer_raw（只在中位标定时看） rc_estop/rc_steer_deg/rc_thr/rc_scale/rc_tgt_rps rc_ch5/rc_ch6（急停/使能极性一次性核对） 唯一权威量（控制权仲裁全靠它），后两者是核摇杆标定用的。 ≤48 就不用动数组；但**上位机档案必须同步**，否则一条红条 "帧量 36 ≠ 本档案 N、通道名整体错位"。 当前 36ch 帧 = 21 + 36×4 = **165 B**（原 48ch = 213 B）。 */
#define MOTOR_CH_COUNT_MAX      (48)        /* 缓冲/协议上限（数组按它开，防越界） */
#define MOTOR_RX_TIMEOUT_MS     (600)       /* 兜底值; 实际判定用可调的 spd_lost_ms */

/* 三、参数表(X-Macro: 一份定义, 两处展开)
 * 上位机通过 GET_TABLE 拉取元数据后自动长出滑条 —— 加一个参数只要加一行,
 * 上位机不用改代码、不用重编译。
 * 字段: ID, 名字, "组名|中文标签", 下限, 上限, 默认值, 单位, 滑条步长, 变量地址
 * 滑条步长只供上位机交互使用; 车端接收有限浮点值后仅做量程钳位, 不按步长取整。
 * 上位机连上后按表显示"默认 2.32"，谁点一次「恢复默认值」积分就放大 4.5 倍（= 猛超调）。
 * 现在两处同源，改一处就行。
 * 注意: ID 一旦定下就不要插队(新参数用新 ID), 否则上位机本地存档里保存的值会错位。
 * 本表就是**唯一权威**: 上位机连上后会被这张表覆盖。
 * 标定顺序建议: 后轮 先 minv_l/minv_r(电机方向) -> 再 inv_l/inv_r(测速正负) -> 再 cpr_l/cpr_r;
 * 转向 ① 手推方向盘到左右极限, 核 steer_abs.h 里中位/左右限位三个 raw, 并确认
 * get_deg() 的符号(约定: 左限位 ≈ +30 度, 右限位 ≈ -30 度; 反了改 DIR_SIGN);
 * ⑤ 方向不对就改 minv_s(在线可改), 不要动 steer_abs.h。
 * P+摩擦前馈), 25/26 是新增的斜坡与软限位。上位机档案要一起换, 别拿旧存档连。
 * 删掉反而响应更好且不震荡。滑条还在(0~200 度/秒)，想研究斜坡照样能拖，
 * 但**默认上车就是"没有斜坡"**。详见 steer_ctrl.h 的 STEER_SLEW_DEG_S_DEFAULT 注解。
 * 饱和门槛因此从 |err| ≈ 6.3° 抬到 **≈ 24.7°**，等于"从限速巡航变成全程按误差给力"。
 * 风险与撤回方式见 steer_ctrl.h 的 STEER_U_MAX_DEFAULT 注解（**别只当它是"调大力气"**）。
 */
#define MOTOR_PARAM_TABLE(X)                                                                                     \
    X( 0, "target_l",   "目标|左轮转速",     -8.000f,    8.000f,    0.000f, "rps",   0.050f, &spd_target_l        ) \
    X( 1, "target_r",   "目标|右轮转速",     -8.000f,    8.000f,    0.000f, "rps",   0.050f, &spd_target_r        ) \
    X( 2, "duty_limit", "限幅|占空比上限",    0.000f,  100.000f, SPD_DUTY_DEFAULT, "%", 1.000f, &spd_duty_limit_pct  ) \
    X( 3, "target_max", "限幅|目标转速上限",  0.000f,   20.000f,    3.000f, "rps",   0.100f, &spd_target_max      ) \
    X( 4, "k_v",        "增益|比例 k_v",      0.000f,    2.000f, SPD_K_V_DEFAULT, "-",    0.005f, &spd_g.k_v           ) \
    X( 5, "k_z",        "增益|积分 k_z",      0.000f,   20.000f, SPD_K_Z_DEFAULT, "-",    0.050f, &spd_g.k_z           ) \
    X( 6, "inv_K",      "前馈|1/K",           0.000f,    0.500f, SPD_INV_K_DEFAULT,"-",   0.001f, &spd_g.inv_K         ) \
    X( 7, "u0",         "前馈|摩擦截距",      0.000f,    0.300f, SPD_U0_DEFAULT, "-",     0.001f, &spd_g.u0            ) \
    X( 8, "cpr_l",      "编码器|每转计数L",   1.000f,65535.000f, 1858.000f, "cnt",   1.000f, &enc_l.cpr           ) \
    X( 9, "cpr_r",      "编码器|每转计数R",   1.000f,65535.000f, 1858.000f, "cnt",   1.000f, &enc_r.cpr           ) \
    X(10, "inv_l",      "编码器|测速取反L",   0.000f,    1.000f,    0.000f, "-",     1.000f, &enc_l.invert        ) \
    X(11, "inv_r",      "编码器|测速取反R",   0.000f,    1.000f,    1.000f, "-",     1.000f, &enc_r.invert        ) \
    X(12, "minv_l",     "电机|极性翻转L",     0.000f,    1.000f,    1.000f, "-",     1.000f, &mot_inv_l           ) \
    X(13, "minv_r",     "电机|极性翻转R",     0.000f,    1.000f,    0.000f, "-",     1.000f, &mot_inv_r           ) \
    X(14, "lost_ms",    "保护|失联停车延时",100.000f, 3000.000f,  600.000f, "ms",   10.000f, &spd_lost_ms         ) \
    X(15, "auto_stop",  "保护|失联自停",      0.000f,    1.000f,    1.000f, "-",     1.000f, &spd_auto_stop       )        \
    X(16, "slew",      "限幅|目标斜坡",      0.000f,  200.000f,   20.000f, "rps/s", 1.000f, &spd_slew_rps_s     )        \
    X(17, "enc_win",   "编码器|滤波窗口",    1.000f,   16.000f,    4.000f, "-",     1.000f, &spd_enc_win        ) \
    X(18, "steer_tgt",  "转向|目标转角",      -27.000f,   27.000f,   0.000f, "deg",   0.500f, &steer.target_delta  ) \
    X(19, "steer_kp",   "转向|比例 Kp",         0.000f,  500.000f,   STEER_KP_DEFAULT,   "-",    1.000f, &steer.kp      ) \
    X(20, "steer_u0",   "转向|摩擦前馈 U0",     0.000f, 3500.000f,   STEER_U0_DEFAULT, "duty", 10.000f, &steer.u0      ) \
    X(21, "steer_db",   "转向|误差死区",        0.000f,    5.000f, STEER_DEADBAND_DEG_DEFAULT, "deg", 0.050f, &steer.deadband_deg  ) \
    X(22, "steer_umax", "转向|输出上限",        0.000f,10000.000f, STEER_U_MAX_DEFAULT,"duty",100.000f, &steer.u_max         ) \
    X(23, "steer_en",   "转向|内环使能",        0.000f,    1.000f,   0.000f,  "-",    1.000f, &steer_en_cmd        ) \
    X(24, "minv_s",     "转向|电机极性",        0.000f,    1.000f,   1.000f,  "-",    1.000f, &mot_inv_s           ) \
    X(25, "steer_slew", "转向|目标斜坡",        0.000f,  200.000f,   0.000f, "deg/s", 1.000f, &steer.slew_deg_s    ) \
    X(26, "steer_lim",  "转向|软限位",          0.000f,   27.000f,  27.000f, "deg",   0.500f, &steer.limit_deg     ) \
    X(27, "rc_arm_en",  "遥控|允许接管",        0.000f,    1.000f,   1.000f, "-",     1.000f, &rc_arm_en           ) \
    X(28, "rc_thr_max", "遥控|满杆转速",        0.000f,   20.000f,   2.000f, "rps",   0.100f, &rc_thr_max          ) \
    X(29, "ol_en",      "标定|开环模式",        0.000f,    1.000f,   0.000f, "-",     1.000f, &spd_ol_en           ) \
    X(30, "ol_duty_l",  "标定|左轮开环duty",  -100.000f,  100.000f,   0.000f, "%",     1.000f, &spd_ol_duty_l       ) \
    X(31, "ol_duty_r",  "标定|右轮开环duty",  -100.000f,  100.000f,   0.000f, "%",     1.000f, &spd_ol_duty_r       ) \
    X(32, "tel_sel",    "标定|遥测档案(0全/1精)",0.000f,   1.000f,   0.000f, "-",     1.000f, &spd_tel_sel         ) \
    X(33, "tel_period", "标定|遥测周期",        5.000f,  200.000f,  20.000f, "ms",    1.000f, &spd_tel_period      ) \
    X(34, "ins_mpm",    "惯导|米每编码器转",    0.050f,    5.000f,   0.8193f,"m/rev",0.001f, &ins_m_per_rev       ) \
    X(35, "ins_odom_en","惯导|航位推算使能",    0.000f,    1.000f,   1.000f, "-",     1.000f, &ins_odom_enable     ) \
    X(36, "ins_slip",   "惯导|打滑门限",        0.000f,  100.000f,  30.000f, "%",     1.000f, &ins_slip_pct        ) \
    X(37, "ins_zero",   "惯导|位姿归零(0>1)",   0.000f,    1.000f,   0.000f, "-",     1.000f, &ins_zero_cmd        ) \
    X(38, "rt_rec",     "路表|录制(0>1开/1>0停)",0.000f,   1.000f,   0.000f, "-",     1.000f, &rt_rec_cmd          ) \
    X(39, "rt_ds",      "路表|采样点距",       0.020f,    0.200f,   0.050f, "m",     0.005f, &ins_route_ds        ) \
    X(40, "rt_slot",    "路表|当前槽(0~4)",    0.000f,    4.000f,   0.000f, "-",     1.000f, &rt_slot_cmd         ) \
    X(41, "rt_load",    "路表|重载槽(0>1)",    0.000f,    1.000f,   0.000f, "-",     1.000f, &rt_load_cmd         ) \
    X(42, "rt_save",    "路表|存到槽(0>1)",    0.000f,    1.000f,   0.000f, "-",     1.000f, &rt_save_cmd         ) \
    X(43, "pb_en",      "回放|使能",          0.000f,    1.000f,   0.000f, "-",     1.000f, &pb_en               ) \
    X(44, "pb_mode",    "回放|1只转向/2带速度",1.000f,    2.000f,   1.000f, "-",     1.000f, &pb_mode             ) \
    X(45, "pb_vmax",    "回放|巡航车速",      0.100f,    4.000f,   1.500f, "m/s",   0.050f, &pb_vmax             ) \
    X(46, "pb_alat",    "回放|横向加速度",    0.100f,   10.000f,   1.200f, "m/s2",  0.050f, &pb_alat             ) \
    X(47, "pb_abrk",    "回放|刹车减速度",    0.100f,   10.000f,   0.800f, "m/s2",  0.050f, &pb_abrk             ) \
    X(48, "pb_l",       "回放|有效轴距",      0.300f,    1.500f,   PB_L_DEFAULT, "m", 0.005f, &pb_l                ) \
    X(49, "pb_ld_a",    "回放|前视基准",      0.100f,    2.000f,   0.350f, "m",     0.010f, &pb_ld_a             ) \
    X(50, "pb_ld_b",    "回放|前视随速",      0.000f,    1.000f,   0.200f, "m/(m/s)",0.005f,&pb_ld_b            ) \
    X(51, "pb_ld_min",  "回放|前视下限",      0.100f,    2.000f,   0.300f, "m",     0.010f, &pb_ld_min           ) \
    X(52, "pb_ld_max",  "回放|前视上限",      0.300f,    5.000f,   1.500f, "m",     0.010f, &pb_ld_max           ) \
    X(53, "rt_mark",    "路表|标记关键点(0>1)",0.000f,    1.000f,   0.000f, "-",     1.000f, &rt_mark_cmd         ) \
    X(54, "pb_ff_en",   "回放|打角前馈使能",  0.000f,    1.000f,   0.000f, "-",     1.000f, &pb_ff_en            ) \
    X(55, "pb_ff",      "回放|前馈增益",      0.000f,    2.000f,   1.000f, "-",     0.050f, &pb_ff_gain          ) \
    X(56, "df_en",      "差速|电子差速使能",  0.000f,    1.000f,   1.000f, "-",     1.000f, &spd_diff_en         ) \
    X(57, "df_w",       "差速|轮距(轮心)",    0.300f,    1.000f,   SPD_DIFF_W_DEFAULT, "m", 0.005f, &spd_diff_w     ) \
    X(58, "df_max",     "差速|单侧偏差上限",  0.100f,    0.950f,   SPD_DIFF_MAX_DEFAULT, "-", 0.010f, &spd_diff_max ) \
    X(59, "steer_ki",   "转向|积分 Ki",        0.000f,  200.000f,   STEER_KI_DEFAULT,   "-",     1.000f, &steer.ki      ) \
    X(60, "steer_imax", "转向|积分上限",       0.000f, 4000.000f,   STEER_IMAX_DEFAULT, "duty", 50.000f, &steer.i_max   ) \
    X(61, "steer_brk",  "转向|前馈开门门限",    0.000f,    5.000f,   STEER_BRK_DEG_DEFAULT,   "deg",  0.050f, &steer.brk_deg   ) \
    X(62, "steer_izone","转向|积分清零门限",   0.000f,   20.000f,   STEER_IZONE_DEG_DEFAULT, "deg",  0.500f, &steer.izone_deg ) \
    X(63, "steer_i_db", "转向|积分累加下界",   0.000f,    2.000f,   STEER_I_DB_DEG_DEFAULT,  "deg",  0.010f, &steer.i_db_deg  )

/* 四、通道下标(motor_link_ch_set 用的就是这些值) */
#define CH_TARGET_L     (0)     /* 目标转速 左轮 rps */
#define CH_V_L          (1)     /* 实测转速 左轮 rps(滤波后) */
#define CH_U_L          (2)     /* 实际输出占空比 左轮 %(带符号) */
#define CH_TARGET_R     (3)     /* 目标转速 右轮 rps */
#define CH_V_R          (4)     /* 实测转速 右轮 rps */
#define CH_U_R          (5)     /* 实际输出占空比 右轮 % */
#define CH_UFF_L        (6)     /* 前馈分量 左轮 % */
#define CH_UFF_R        (7)     /* 前馈分量 右轮 % */
#define CH_Z_L          (8)     /* 积分状态 左轮 rps*s */
#define CH_Z_R          (9)     /* 积分状态 右轮 rps*s */
#define CH_POS_L        (10)    /* 带符号累计计数 左轮 cnt(标 cpr 用) */
#define CH_POS_R        (11)    /* 带符号累计计数 右轮 cnt */
#define CH_ONLINE       (12)    /* 1 = 链路活着 */
#define CH_RX_AGE_MS    (13)    /* 距上次收到下行帧的时间 ms */
#define CH_ENABLE_BITS  (14)    /* 使能位图 bit1=电机 bit2=总使能 */
#define CH_SAFETY       (15)    /* 保护状态位图(0 = 一切正常) */

/* --- 转向(16~20) --- */
#define CH_STEER_TARGET (16)    /* = 软限位夹紧 + (可选)斜坡之后 —— 即转向环**真正的输入**。 发这个的理由与增益回读一样：斜坡开着时若发命令， `steer_slew = 0`(默认) 时两者相等。 */
#define CH_STEER_DELTA  (17)    /* 实测转角 deg(已减中位、已解环绕) */
#define CH_STEER_U      (18)    /* 转向电机输出 duty(-10000~10000) */
#define CH_STEER_EN     (19)    /* 内环使能 0/1 */
#define CH_STEER_STATUS (20)    /* 转向状态 bit0=自检通过, bit1=反馈失效, bit2=顶死/反转 */

/* --- 遥控(21~24): 只留"判断谁在开车"和"核摇杆标定"必须的 4 个 --- */
#define CH_RC_ENGAGE    (21)
#define CH_RC_CH1       (22)    /* CH1 原始值（核摇杆标定用; 回中应≈992） */
#define CH_RC_CH2       (23)    /* CH2 原始值（核油门标定用） */
#define CH_RC_FRAMES    (24)    /* 累计 SBUS 帧数（不涨 = 接收机没在发） */

/* --- 增益回读(25~26) --- */
#define CH_K_V          (25)    /* 本拍实际生效的比例增益 k_v (u/rps) */
#define CH_K_Z          (26)    /* 本拍实际生效的积分增益 k_z (u/(rps*s)) */

/* --- 惯导(27~32) --- */
#define CH_INS_YAW      (27)    /* 累积航向 度(可超 ±360; 已乘 INS_YAW_SIGN) */
#define CH_INS_GZ       (28)    /* 陀螺 Z 角速度 度/秒(去零偏后; 静止看它=残余零偏) */
#define CH_INS_X        (29)    /* 航位推算 x 米(向右为正) */
#define CH_INS_Y        (30)    /* 航位推算 y 米(向前为正) */
#define CH_INS_DIST     (31)    /* 累计里程 米(只累加绝对值, 倒车也增) */
#define CH_INS_FLAGS    (32)    /* 惯导状态位图, 见 ins_flags 的自检打印 */

/* `rt_st` 是一个**位图 + 槽号**的混合值：低 8 位 = 状态位（见 route_app.h 的 RT_ST_*），
 * bit8..11 = 当前槽号。上位机按整数显示即可（要看位就自己 &）。
 * 但**加通道必须同步改上位机档案**（`Defaults.cs` 的 KartInsChannelNames），否则通道名整体错位。
 */
#define CH_RT_CNT       (33)    /* 路表点数(当前 RAM 里的表) */
#define CH_RT_ST        (34)    /* 路表状态位 | (槽号 << 8) */
#define CH_PB_ELAT      (35)    /* P2 回放横向误差 米(正 = 车在参考线右侧) */

/* 39 通道一帧 177 B 把带宽吃满了；18 通道只有 93 B ⇒ 同样的链路能把周期压到 5~10 ms。
 * 而且标定时真正要看的就这 9 个量 + 增益 + 几个状态位，别的通道只会把图画乱。
 * 切完 **上位机的档案下拉框也要跟着切**，否则通道名错位。
 */
#define MOTOR_CH_COUNT_CALIB    (18)

#define CT_TARGET_L     (0)     /* 速度环目标转速 左轮 rps */
#define CT_V_L          (1)     /* 速度环实际转速 左轮 rps */
#define CT_U_L          (2)     /* 左电机实际 PWM %（带符号；开环时=给定） */
#define CT_TARGET_R     (3)     /* 速度环目标转速 右轮 rps */
#define CT_V_R          (4)     /* 速度环实际转速 右轮 rps */
#define CT_U_R          (5)     /* 右电机实际 PWM % */
#define CT_STEER_TGT    (6)     /* —— 与全量档 CH_STEER_TARGET 同一个口径，不是上位机写的命令 */
#define CT_STEER_DELTA  (7)     /* 舵机角度环实际角 deg */
#define CT_STEER_U      (8)     /* 舵机 PWM（duty，-10000~10000） */
#define CT_ONLINE       (9)     /* 链路在线 0/1 */
#define CT_ENABLE       (10)    /* 使能位图 */
#define CT_SAFETY       (11)    /* 保护位图（0x40 = 本地(遥控)接管中） */
#define CT_STEER_EN     (12)    /* 转向内环使能 0/1 */
#define CT_STEER_ST     (13)    /* 转向状态：bit0 自检通过 / bit1 反馈失效 / bit2 顶死或反转 */
#define CT_OL_EN        (14)    /* 开环模式 0/1（一眼看出现在走的是开环还是闭环） */
#define CT_RESV0        (15)    /* 预留位（扩展用，别占用） */
#define CT_K_V          (16)    /* 本拍实际生效的比例增益 k_v */
#define CT_K_Z          (17)    /* 本拍实际生效的积分增益 k_z */

/* 五、对外接口 */

/* 初始化: 连 WiFi + 建 TCP/UDP + 上报参数表
 * 注意: 会阻塞(WiFi 最长等约 10 秒、Socket 最长等数十秒), 只能在初始化阶段调用。
 * 返回 0=成功, 1=WiFi 连接失败, 2=Socket 连接失败
 */
uint8   motor_link_init          (void);

/* 时基: 必须在 5ms 的 PIT 中断里调用, 且要在速度环之前(保护层要用它的时间戳) */
void    motor_link_tick_5ms      (void);

/* 主循环任务: 发遥测 + 收命令。放在 while(1) 里, 尽量每圈都调到 */
void    motor_link_task          (void);
void    motor_route_dump_poll    (void);

/* 状态查询 */
uint8   motor_link_online        (void);     /* 1 = 最近收到过下行帧(链路活着) */
uint16  motor_link_enable_bits   (void);     /* bit1 电机 / bit2 总使能 */
uint32  motor_link_rx_age_ms     (void);     /* 距上次收到下行帧 ms */
uint32  motor_link_tx_count      (void);     /* 已发出的遥测帧数 */
uint32  motor_link_now_ms        (void);     /* 上电时间 ms(来自 5ms 时基) */
uint8   motor_link_init_err      (void);     /* motor_link_init 的返回值 */
uint8   motor_link_ever_rx       (void);     /* 1 = 上电以来至少收到过一帧下行 */

/* 遥测通道读写: 由 main 负责填值, 本模块只负责按周期打包发送 */
void    motor_link_ch_set        (uint8 idx, float v);

/* 切换后帧头的 COUNT 立刻跟着变，所以上位机的档案下拉框必须同步切（见 motor_link.h 顶部说明）。
 * n 会被夹到 [1, MOTOR_CH_COUNT_MAX]；ms 会被夹到 [MOTOR_TX_PERIOD_MIN, 1000]。
 */
void    motor_link_set_ch_count     (uint16 n);
uint16  motor_link_ch_count         (void);
void    motor_link_set_tx_period_ms (uint16 ms);
uint16  motor_link_tx_period_ms     (void);

#define MOTOR_TX_PERIOD_MIN     (5)         /* 遥测周期下限 ms(再小主循环扛不住) */

#endif
