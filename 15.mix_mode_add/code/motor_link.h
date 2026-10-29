/*********************************************************************************************************************
 * 转向位置环 + 双电机速度环(WiFi) - 车端链路(WiFi-SPI)
 *
 * 上行 MAGIC(2)+TYPE(1)+LEN(2)+PAYLOAD+CRC16(2)+TAIL(4) = 11+LEN, 下行同构无 TAIL = 7+LEN
 * MAGIC 上行 A5 5A / 下行 5A A5; 全部小端; CRC16-MODBUS(0xA001) 覆盖 TYPE~PAYLOAD
 * CRC 不过只前移 1 字节(不能跳整帧); TAIL = 00 00 80 7F, 与 VOFA+ JustFloat 一致
 * 硬件: SPI_0 @10 MHz, SCK P02_2 / MOSI P02_1 / MISO P02_0 / CS P02_3, INT P02_4 低 = 模块忙, RST P23_0
 * 上位机只下发目标值, 闭环在车端运行。初始化后每圈调 motor_link_task(),
 * 5 ms PIT 中断调 motor_link_tick_5ms()。
 ********************************************************************************************************************/

#ifndef _MOTOR_LINK_H
#define _MOTOR_LINK_H

#include "zf_common_headfile.h"

#include "param_table.h"

extern volatile float   spd_tel_sel;        /* 遥测档案: 0 = 全量 / 1 = 标定精简(18) */
extern volatile float   spd_tel_period;     /* 遥测周期 ms(5~200)             */

/* ---- 一、用户必改: WiFi 与上位机 ---- */
#define MOTOR_NET_PROFILE       (0)                     /* 0 = 手机热点 / 1 = 实验室内网 */

#if   (MOTOR_NET_PROFILE == 0)
  #define MOTOR_WIFI_SSID       "XIOQ-3695"             /* 手机热点名                          */
  #define MOTOR_WIFI_PASSWORD   "123456890"             /* 热点密码(无密码填 NULL)             */
  #define MOTOR_HOST_IP         "192.168.244.69"       /* 手机热点接口 ap0 的地址, 不是 .1    */
#elif (MOTOR_NET_PROFILE == 1)
  #define MOTOR_WIFI_SSID       "ASC_Circuit_IoT"       /* 实验室 IoT 网                       */
  #define MOTOR_WIFI_PASSWORD   "1145141919810"
  #define MOTOR_HOST_IP         "192.168.50.176"        /* 那台 PC 的地址                      */
#else
  #error "MOTOR_NET_PROFILE must be 0 (phone hotspot) or 1 (lab IoT)"
#endif

#define MOTOR_HOST_PORT         "8086"                  /* 与上位机/App 里的端口一致           */
#define MOTOR_LOCAL_PORT        "6666"

/* 车是 socket client, 只往 MOTOR_HOST_IP 发包, 地址写错时两边都不报错也收不到包;
   MOTOR_SOCK_TYPE 必须与上位机下拉同一项: UDP 丢帧不拖累主循环, TCP 由模块负责重连。 */
#define MOTOR_SOCK_TYPE         "UDP"

/* ---- 二、可调项 ---- */
#define MOTOR_TX_PERIOD_MS      (20)        /* 遥测周期 ms: 20 -> 50Hz                      */
#define MOTOR_RX_PERIOD_MS      (10)        /* 收命令周期 ms: 10 -> 100Hz                   */
#define MOTOR_CH_COUNT          (45)        /* 全量档案通道数; 帧长 = 11 + 10 + N*4         */
#define MOTOR_CH_COUNT_MAX      (48)        /* 缓冲/协议上限(数组按它开, 防越界)            */
#define MOTOR_RX_TIMEOUT_MS     (600)       /* 兜底值; 实际判定用可调的 spd_lost_ms         */

/* ---- 三、参数表 ---- 表在 param_table.h (唯一源), 元数据接口 proto_param_count/at/by_id
   ID 定下后不插队; 默认值引用模块宏; 浮点值只做量程钳位; PERM 决定谁能写、何时能写 */

/* ---- 四、通道下标(motor_link_ch_set 用的就是这些值) ---- 顺序即协议, 与上位机档案一一对应 */
#define CH_TARGET_L     (0)     /* 目标转速 左轮 rps                  */
#define CH_V_L          (1)     /* 实测转速 左轮 rps(滤波后)          */
#define CH_U_L          (2)     /* 实际输出占空比 左轮 %(带符号)      */
#define CH_TARGET_R     (3)     /* 目标转速 右轮 rps                  */
#define CH_V_R          (4)     /* 实测转速 右轮 rps                  */
#define CH_U_R          (5)     /* 实际输出占空比 右轮 %              */
#define CH_UFF_L        (6)     /* 前馈分量 左轮 %                    */
#define CH_UFF_R        (7)     /* 前馈分量 右轮 %                    */
#define CH_Z_L          (8)     /* 积分状态 左轮 rps*s                */
#define CH_Z_R          (9)     /* 积分状态 右轮 rps*s                */
#define CH_POS_L        (10)    /* 带符号累计计数 左轮 cnt(标 cpr 用) */
#define CH_POS_R        (11)    /* 带符号累计计数 右轮 cnt            */
#define CH_ONLINE       (12)    /* 1 = 链路活着                       */
#define CH_RX_AGE_MS    (13)    /* 距上次收到下行帧的时间 ms          */
#define CH_ENABLE_BITS  (14)    /* 使能位图 bit1=电机 bit2=总使能     */
#define CH_SAFETY       (15)    /* 保护状态位图(0 = 一切正常)         */

/* --- 转向(16~20) --- */
#define CH_STEER_TARGET (16)    /* 实际施加的目标转角 deg(向右为正), 已过软限位与斜坡 */
#define CH_STEER_DELTA  (17)    /* 实测转角 deg(已减中位、已解环绕)   */
#define CH_STEER_U      (18)    /* 转向电机输出 duty(-10000~10000)    */
#define CH_STEER_EN     (19)    /* 内环使能 0/1                       */
#define CH_STEER_STATUS (20)    /* 转向状态 bit0=自检通过, bit1=反馈失效, bit2=顶死/反转 */

/* --- 遥控(21~24) --- */
#define CH_RC_ENGAGE    (21)    /* 接管状态 0 OFF/1 STANDBY/2 ON/3 FAILSAFE, 控制权仲裁依据 */
#define CH_RC_CH1       (22)    /* CH1 原始值(核摇杆标定; 回中应≈992)        */
#define CH_RC_CH2       (23)    /* CH2 原始值(核油门标定)                    */
#define CH_RC_FRAMES    (24)    /* 累计 SBUS 帧数(不涨 = 接收机没在发)       */

/* --- 增益回读(25~26) --- */
#define CH_K_V          (25)    /* 本拍实际生效的比例增益 k_v (u/rps) */
#define CH_K_Z          (26)    /* 本拍实际生效的积分增益 k_z (u/(rps*s)) */

/* --- 惯导(27~32) --- */
#define CH_INS_YAW      (27)    /* 累积航向 度(可超 ±360; 已乘 INS_YAW_SIGN) */
#define CH_INS_GZ       (28)    /* 陀螺 Z 角速度 度/秒(去零偏后; 静止看它=残余零偏) */
#define CH_INS_X        (29)    /* 航位推算 x 米(向右为正)                   */
#define CH_INS_Y        (30)    /* 航位推算 y 米(向前为正)                   */
#define CH_INS_DIST     (31)    /* 累计里程 米(只累加绝对值, 倒车也增)        */
#define CH_INS_FLAGS    (32)    /* 惯导状态位图, 见 ins_flags 的自检打印      */

/* --- 路表/回放(33~35) --- CH_RT_ST = 状态位(低 8 位, 见 route_app.h 的 RT_ST_*) | (槽号 << 8) */
#define CH_RT_CNT       (33)    /* 路表点数(当前 RAM 里的表)                  */
#define CH_RT_ST        (34)    /* 路表状态位 | (槽号 << 8)                   */
#define CH_PB_ELAT      (35)    /* 回放横向误差 米(正 = 车在参考线右侧)       */

/* --- 航向测试(36~37) --- */
#define CH_TAR_YAW      (36)    /* 本拍目标航向 deg(航向外环输入), 与 CH_INS_YAW 成对看 */
#define CH_HT_SEG       (37)    /* 航向测试段号: 0=没跑, 1..n=第几段, 254=跑完 */

/* --- 诊断通道(38~40) --- 时基 / ZUPT 冻结丢弃角 / 本拍是否判静止 */
#define CH_TICK_MS      (38)    /* 上电毫秒数(5ms 时基累加), 核时基就看它       */
#define CH_INS_FRZ      (39)    /* ZUPT 冻结期间被丢弃的转角 度(累计, 只读不清) */
#define CH_PB_BEST      (41)    /* 回放: 最近点索引                            */
#define CH_PB_FI        (42)    /* 回放: 前视点索引                            */
#define CH_PB_KREF      (43)    /* 回放: 参考曲率 1/m                          */
#define CH_PB_WHY       (44)    /* 回放: 结束原因码(见 pb_set_why)             */
#define CH_INS_STATIC   (40)    /* 1 = 本拍被判静止(航向正在被冻结), 与 CH_INS_FRZ 配对看 */

/* ---- 五、标定精简档案(18 通道) ---- 参数 tel_sel = 1 启用; 下标即协议, 上位机档案必须同步切 */
#define MOTOR_CH_COUNT_CALIB    (18)

#define CT_TARGET_L     (0)     /* 速度环目标转速 左轮 rps                  */
#define CT_V_L          (1)     /* 速度环实际转速 左轮 rps                  */
#define CT_U_L          (2)     /* 左电机实际 PWM %(带符号; 开环时 = 给定)  */
#define CT_TARGET_R     (3)     /* 速度环目标转速 右轮 rps                  */
#define CT_V_R          (4)     /* 速度环实际转速 右轮 rps                  */
#define CT_U_R          (5)     /* 右电机实际 PWM %                         */
#define CT_STEER_TGT    (6)     /* 转向环实际施加的目标角 deg(软限位与斜坡之后) */
#define CT_STEER_DELTA  (7)     /* 转向环实际角 deg                         */
#define CT_STEER_U      (8)     /* 转向 PWM(duty, -10000~10000)             */
#define CT_ONLINE       (9)     /* 链路在线 0/1                             */
#define CT_ENABLE       (10)    /* 使能位图                                 */
#define CT_SAFETY       (11)    /* 保护位图(0x40 = 本地遥控接管中)          */
#define CT_STEER_EN     (12)    /* 转向内环使能 0/1                         */
#define CT_STEER_ST     (13)    /* 转向状态 bit0 自检通过 / bit1 反馈失效 / bit2 顶死或反转 */
#define CT_OL_EN        (14)    /* 开环模式 0/1                             */
#define CT_RESV0        (15)    /* 预留位(扩展用, 别占用)                   */
#define CT_K_V          (16)    /* 本拍实际生效的比例增益 k_v, 与 CH_K_V 同口径 */
#define CT_K_Z          (17)    /* 本拍实际生效的积分增益 k_z              */

/* ---- 六、对外接口 ---- */

/* 初始化: 连 WiFi + 建 TCP/UDP + 上报参数表; 会阻塞(WiFi 约 10 s), 只能在初始化阶段调用
   返回 0=成功, 1=WiFi 连接失败, 2=Socket 连接失败 */
uint8   motor_link_init          (void);

/* 时基: 必须在 5ms 的 PIT 中断里调用, 且要在速度环之前(保护层要用它的时间戳) */
void    motor_link_tick_5ms      (void);

/* 主循环任务: 发遥测 + 收命令, 放在 while(1) 里每圈调用 */
void    motor_link_task          (void);
/* 路表 dump 分步发送(0x08 读路表), 由 motor_link_task() 每圈调用, 空闲即返回 */
void    motor_route_dump_poll    (void);

uint8   motor_link_online        (void);     /* 1 = 最近收到过下行帧         */
uint16  motor_link_enable_bits   (void);     /* bit1 电机 / bit2 总使能      */
uint32  motor_link_rx_age_ms     (void);     /* 距上次收到下行帧 ms          */
uint32  motor_link_tx_count      (void);     /* 已发出的遥测帧数             */
uint32  motor_link_now_ms        (void);     /* 上电时间 ms                  */
/* 恢复默认值, 与菜单 Reset 走同一条路径 */
void    motor_param_load_defaults(void);
uint8   motor_link_init_err      (void);
uint8   motor_link_ever_rx       (void);

/* 遥测通道读写: 由 main 填值, 本模块按周期打包发送 */
void    motor_link_ch_set        (uint8 idx, float v);

/* 运行时切换遥测通道数与周期; 帧头 COUNT 立刻变化, 上位机档案必须同步切
   n 夹到 [1, MOTOR_CH_COUNT_MAX], ms 夹到 [MOTOR_TX_PERIOD_MIN, 1000] */
void    motor_link_set_ch_count     (uint16 n);
uint16  motor_link_ch_count         (void);
void    motor_link_set_tx_period_ms (uint16 ms);
uint16  motor_link_tx_period_ms     (void);

#define MOTOR_TX_PERIOD_MIN     (5)         /* 遥测周期下限 ms(再小主循环扛不住) */

#endif
