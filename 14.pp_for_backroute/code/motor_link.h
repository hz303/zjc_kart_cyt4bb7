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

/* 它内部包含各模块头文件 —— 原先由本文件转出去的那一串声明仍然可用，不必再加 include。
 * [!] 写注释时不要出现 "星号 + 斜杠" 那个终止符（哪怕是在举例）：
 * 它会把块注释提前闭合，后面半句变成代码，报一串 "identifier xxx is undefined"。
 */
#include "param_table.h"

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
#define MOTOR_NET_PROFILE       (0)

#if   (MOTOR_NET_PROFILE == 0)
  #define MOTOR_WIFI_SSID       "XIOQ-3695"             /* 手机热点名 */
  #define MOTOR_WIFI_PASSWORD   "123456890"             /* 热点密码(无密码填 NULL) */
  #define MOTOR_HOST_IP         "192.168.244.69"       /* 手机 ap0 地址(实测) */
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
#define MOTOR_CH_COUNT          (41)        /* 全量档案的遥测通道数(见下面第四节)。 tick_ms(38) / ins_frz(39) / ins_static(40)。 用途只有一个：判"yaw 转一圈缺一块是 dt、刻度还是 ZUPT 冻结"。 每多 1 通道 = 每帧多 4 B、50 Hz 下多 200 B/s。 砍掉的 12 个都是"中间量/一次性标定量"： dlt_l/dlt_r（本拍增量，pos 差分即得） flip_l/flip_r（换向丢拍，老问题已消） steer_raw（只在中位标定时看） rc_estop/rc_steer_deg/rc_thr/rc_scale/rc_tgt_rps rc_ch5/rc_ch6（急停/使能极性一次性核对） 唯一权威量（控制权仲裁全靠它），后两者是核摇杆标定用的。 ≤48 就不用动数组；但**上位机档案必须同步**，否则一条红条 "帧量 41 ≠ 本档案 N、通道名整体错位"。 当前 41ch 帧 = 21 + 41×4 = **185 B**（38ch 时 173 B）。 */
#define MOTOR_CH_COUNT_MAX      (48)        /* 缓冲/协议上限（数组按它开，防越界） */
#define MOTOR_RX_TIMEOUT_MS     (600)       /* 兜底值; 实际判定用可调的 spd_lost_ms */

/* 三、参数表：见 `param_table.h`（**唯一源**）
 * （motor_link.c 取元数据、app_params.c 存取 Flash、ui_menu.c 取指针/量程），
 * 加一个字段要改三处宏，漏一处不报错、只在现场表现成"改了没用"。
 * 现在表在 param_table.h，且**只在 proto.c 里展开一次**，对外只暴露元数据接口：
 * proto_param_count() / proto_param_at() / proto_param_by_id()
 * 仍然适用的纪律（原注释保留在这里，免得去别的文件里找）：
 * · ID 一旦定下不要插队（新参数用新 ID），否则上位机本地存档会错位；
 * · 滑条步长只供上位机交互；车端收到浮点值后**只做量程钳位**，不按步长取整；
 * · 每行新增的 `PERM`（P_RO/P_RW_LIVE/P_RW_IDLE/P_CMD）决定"谁能写、什么时候能写"。
 * `ap_is_cmd()` 那个手工黑名单已经删除，改由 `PERM == P_CMD` 表达。
 * 标定顺序建议: 后轮 先 minv_l/minv_r -> 再 inv_l/inv_r -> 再 cpr_l/cpr_r;
 * 转向 ① 手推方向盘到左右极限, 核 steer_abs.h 里中位/左右限位三个 raw;
 * ② 先只加 steer_u0, 加到"方向盘刚好能被推动";
 * ④ 最后按需调 steer_db / steer_slew。
 */

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

/* --- 航向测试(36~37): 工程 11 的 App 测试页要用 --- */
#define CH_TAR_YAW      (36)    /* "舵机外环"的输入与输出，台阶测试全靠它们量上升时间。 */
#define CH_HT_SEG       (37)    /* 航向测试段号：0=没跑，1..n=第几段，254=跑完 */

/* · 时基只能车端自己数（`motor_link_now_ms()`，5 ms 中断累加）；
 * · ZUPT 冻结丢了多少度只有车端知道（`ins_imu_zupt_frz_deg()`）；
 * · "被判静止"用的是 **IMU 内部**的 s_is_static，**不是** odom 的 ODOM_ST_STILL
 * （后者 = "编码器 50 ms 没动"，抬着车原地转时它照样判静止 —— 两个别混）。
 * 否则通道名整体错位（不报错、图看着还挺像 —— 见 kz.csv 那次）。
 */
#define CH_TICK_MS      (38)    /* 车端上电毫秒数(5ms 时基累加)。掐 60 s 秒表核时基就看它 */
#define CH_INS_FRZ      (39)    /* ZUPT 冻结期间**被丢弃的转角** 度(累计, 只读不清)。 */
#define CH_INS_STATIC   (40)    /* 1 = 本拍被判"静止"(航向正在被冻结)。与 CH_INS_FRZ 配对看 */

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
/* 菜单的 9.Reset 也要用同一条路径，否则"上位机点的恢复默认"和"车上按的恢复默认" 会走出两套行为 —— 这类分叉最后一定会在现场表现成"两个地方的结果不一样"。 */
void    motor_param_load_defaults(void);
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
