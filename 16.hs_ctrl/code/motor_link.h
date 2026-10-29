/*********************************************************************************************************************
 * 车端 WiFi-SPI 链路: 上行遥测, 下行参数与使能命令; 全部小端
 *
 * 上行  A5 5A | TYPE(1) | LEN(2) | PAYLOAD | CRC16(2) | TAIL(4)   总长 11+LEN
 * 下行  5A A5 | TYPE(1) | LEN(2) | PAYLOAD | CRC16(2)             总长  7+LEN
 * CRC16-MODBUS(0xA001) 覆盖 TYPE~PAYLOAD; 校验失败只前移 1 字节
 * TAIL = 00 00 80 7F
 * SPI_0 @10MHz: SCK P02_2 / MOSI P02_1 / MISO P02_0 / CS P02_3
 *               INT P02_4(低=模块忙) / RST P23_0 / 5V
 * 用法: 填下方 WiFi 与 IP 宏 -> 初始化后调 motor_link_init() -> 主循环调 motor_link_task()
 *       5ms PIT 中断调 motor_link_tick_5ms()
 ********************************************************************************************************************/

#ifndef _MOTOR_LINK_H
#define _MOTOR_LINK_H

#include "zf_common_headfile.h"

/* 参数表在 param_table.h(全工程唯一源), 本文件不再转出各模块声明 */
#include "param_table.h"

/* 遥测档案: 0=巡线(22 通道, 默认) / 1=标定(18) / 2=全量(45) / 3=采集(16)
 * 上位机档案下拉框必须切到同一档, 否则通道名整体错位 */
extern volatile float   spd_tel_sel;        /* 0=巡线(22) / 1=标定(18) / 2=全量(45) / 3=采集(16) */
extern volatile float   spd_tel_period;     /* 遥测周期 ms(5~200)             */

/* ---- 一、用户必改: WiFi 与上位机 ---- */
/* 网络配置: 0 = 手机热点(手机跑 KartPhone) / 1 = 实验室 IoT 网(PC 跑 KartHost)
 * 车是 socket client, 只往 MOTOR_HOST_IP 发包; 写错则两边都不报错且收不到包
 * 档 0 的 MOTOR_HOST_IP 取手机热点接口(ap0)自己的地址, 不是网关地址
 * 档 1 的 PC 若用 DHCP 取址, 可改成该网段广播地址 */
#define MOTOR_NET_PROFILE       (0)                     /* 0 = 手机热点 / 1 = 实验室 IoT 网  */

#if   (MOTOR_NET_PROFILE == 0)
  #define MOTOR_WIFI_SSID       "XIOQ-3695"             /* 手机热点名                          */
  #define MOTOR_WIFI_PASSWORD   "123456890"             /* 热点密码(无密码填 NULL)             */
  #define MOTOR_HOST_IP         "192.168.244.69"       /* 手机热点接口地址                    */
#elif (MOTOR_NET_PROFILE == 1)
  #define MOTOR_WIFI_SSID       "ASC_Circuit_IoT"       /* 实验室 IoT 网                       */
  #define MOTOR_WIFI_PASSWORD   "1145141919810"
  #define MOTOR_HOST_IP         "192.168.50.176"        /* 上位机 PC 地址                      */
#else
  #error "MOTOR_NET_PROFILE must be 0 (phone hotspot) or 1 (lab IoT)"
#endif

#define MOTOR_HOST_PORT         "8086"                  /* 与上位机/App 里的端口一致           */
#define MOTOR_LOCAL_PORT        "6666"                  /* 本机端口, 一般不用改                */

/* TCP / UDP 二选一, 须与上位机下拉框一致
   UDP: 丢帧不拖主循环, 上位机可随时开关; TCP: 每帧送达, 由模块负责重连 */
#define MOTOR_SOCK_TYPE         "UDP"

/* ---- 二、可调项 ---- */
#define MOTOR_TX_PERIOD_MS      (20)        /* 遥测周期 ms: 20 -> 50Hz                      */
#define MOTOR_RX_PERIOD_MS      (10)        /* 收命令周期 ms: 10 -> 100Hz                   */
#define MOTOR_RX_TIMEOUT_MS     (600)       /* 兜底值; 实际判定用可调的 spd_lost_ms          */

/* ---- 三、参数表 ----
 * 表在 param_table.h, 只在 proto.c 展开一次, 对外是
 * proto_param_count() / proto_param_at() / proto_param_by_id()
 * ID 定下后不插队; 默认值引用模块宏; 车端只做量程钳位, 不按步长取整
 * 标定顺序: 后轮 minv_l/minv_r -> cpr_l/cpr_r -> 转向 steer_u0 -> steer_kp -> steer_db/slew */

/* ---- 四、通道下标(motor_link_ch_set 用的就是这些值) ----
 * 第 N 个 #define 的值 = 本档第 N 号通道 = 上位机同档通道表的第 N 项
 * 改通道: 编辑 telemetry/tel_channels.def 后重跑 telemetry/gen_tel.py, 三端同时更新
 * 三档由 spd_tel_sel 运行时切换, 缓冲上限 MOTOR_CH_COUNT_MAX = 48
 * 跨档取数走 KartState.resolve(), 不要硬写某一档的下标
 * CH_STEER_TARGET / CT_STEER_TGT 是软限位夹紧与斜坡之后实际施加的目标角, 不是上位机下发值 */
/* ---- 通道表: 由 telemetry/gen_tel.py 生成, 勿手改 ---- */
// >>> GEN:TEL_CH <<<

#define MOTOR_TEL_PROFILE_ROUTE      0   /* 巡线档（默认） */
#define MOTOR_TEL_PROFILE_CALIB      1   /* 标定档 */
#define MOTOR_TEL_PROFILE_FULL       2   /* 全量档 */
#define MOTOR_TEL_PROFILE_TRAIN      3   /* 采集档 */

#define MOTOR_CH_COUNT_MAX         48  /* 存储按上限开，通道数运行时可切 */
#define MOTOR_CH_COUNT_ROUTE   22  /* 巡线档（默认）：109 B/帧 */
#define MOTOR_CH_COUNT_CALIB   18  /* 标定档：93 B/帧 */
#define MOTOR_CH_COUNT_FULL    45  /* 全量档：201 B/帧 */
#define MOTOR_CH_COUNT_TRAIN   16  /* 采集档：85 B/帧 */
#define MOTOR_CH_COUNT             MOTOR_CH_COUNT_FULL  /* 数组/上限按全量档 */

/* ---- 全量档 full：45 通道 ---- */
#define CH_TARGET_L            0  /* 目标速度·左轮 [rps] */
#define CH_V_L                 1  /* 实测速度·左轮 [rps] */
#define CH_U_L                 2  /* 输出占空比·左轮 [%] */
#define CH_TARGET_R            3  /* 目标速度·右轮 [rps] */
#define CH_V_R                 4  /* 实测速度·右轮 [rps] */
#define CH_U_R                 5  /* 输出占空比·右轮 [%] */
#define CH_UFF_L               6  /* 前馈分量·左轮 [%] */
#define CH_UFF_R               7  /* 前馈分量·右轮 [%] */
#define CH_Z_L                 8  /* 积分状态·左轮 [rps*s] */
#define CH_Z_R                 9  /* 积分状态·右轮 [rps*s] */
#define CH_POS_L               10 /* 带符号累计计数·左轮 [cnt] */
#define CH_POS_R               11 /* 带符号累计计数·右轮 [cnt] */
#define CH_ONLINE              12 /* 链路在线 */
#define CH_RX_AGE_MS           13 /* 下行静默时长 [ms] */
#define CH_ENABLE_BITS         14 /* 使能位图 */
#define CH_SAFETY              15 /* 保护状态位图 */
#define CH_STEER_TARGET        16 /* 目标转角(已施加) [deg] */
#define CH_STEER_DELTA         17 /* 实测转角 [deg] */
#define CH_STEER_U             18 /* 转向输出 [duty] */
#define CH_STEER_EN            19 /* 转向内环使能 */
#define CH_STEER_STATUS        20 /* 转向状态位图 */
#define CH_RC_ENGAGE           21 /* 接管状态 */
#define CH_RC_CH1              22 /* 摇杆 CH1 原始值 [raw] */
#define CH_RC_CH2              23 /* 摇杆 CH2 原始值 [raw] */
#define CH_RC_FRAMES           24 /* SBUS 累计帧数 */
#define CH_K_V                 25 /* 比例增益 k_v */
#define CH_K_Z                 26 /* 积分增益 k_z */
#define CH_INS_YAW             27 /* 累积航向 [deg] */
#define CH_INS_GZ              28 /* 陀螺 Z 角速度 [deg/s] */
#define CH_INS_X               29 /* 航位推算 X [m] */
#define CH_INS_Y               30 /* 航位推算 Y [m] */
#define CH_INS_DIST            31 /* 累计里程 [m] */
#define CH_INS_FLAGS           32 /* 惯导状态位图 */
#define CH_RT_CNT              33 /* 路表点数 [pt] */
#define CH_RT_ST               34 /* 路表状态(位/槽号) */
#define CH_PB_ELAT             35 /* 回放横向误差 [m] */
#define CH_TAR_YAW             36 /* 目标航向 [deg] */
#define CH_HT_SEG              37 /* 航向测试段号 */
#define CH_TICK_MS             38 /* 上电毫秒数 [ms] */
#define CH_INS_FRZ             39 /* 冻结丢弃的转角 [deg] */
#define CH_INS_STATIC          40 /* 本拍判静止 */
#define CH_PB_BEST             41 /* 最近点索引 [idx] */
#define CH_PB_FI               42 /* 前视点索引 [idx] */
#define CH_PB_KREF             43 /* 参考曲率 [1/m] */
#define CH_PB_WHY              44 /* 收车原因码 */

/* ---- 巡线档（默认） route：22 通道 ---- */
#define RCH_U_L                0  /* 输出占空比·左轮 [%] */
#define RCH_V_L                1  /* 实测速度·左轮 [rps] */
#define RCH_TARGET_L           2  /* 目标速度·左轮 [rps] */
#define RCH_U_R                3  /* 输出占空比·右轮 [%] */
#define RCH_V_R                4  /* 实测速度·右轮 [rps] */
#define RCH_TARGET_R           5  /* 目标速度·右轮 [rps] */
#define RCH_STEER_TARGET       6  /* 目标转角(已施加) [deg] */
#define RCH_STEER_DELTA        7  /* 实测转角 [deg] */
#define RCH_STEER_U            8  /* 转向输出 [duty] */
#define RCH_K_V                9  /* 比例增益 k_v */
#define RCH_K_Z                10 /* 积分增益 k_z */
#define RCH_INS_YAW            11 /* 累积航向 [deg] */
#define RCH_TAR_YAW            12 /* 目标航向 [deg] */
#define RCH_INS_GZ             13 /* 陀螺 Z 角速度 [deg/s] */
#define RCH_PB_ELAT            14 /* 回放横向误差 [m] */
#define RCH_RC_ENGAGE          15 /* 接管状态 */
#define RCH_STEER_EN           16 /* 转向内环使能 */
#define RCH_STEER_STATUS       17 /* 转向状态位图 */
#define RCH_RT_CNT             18 /* 路表点数 [pt] */
#define RCH_RT_ST              19 /* 路表状态(位/槽号) */
#define RCH_TICK_MS            20 /* 上电毫秒数 [ms] */
#define RCH_INS_STATIC         21 /* 本拍判静止 */

/* ---- 标定档 calib：18 通道 ---- */
#define CT_TARGET_L            0  /* 目标速度·左轮 [rps] */
#define CT_V_L                 1  /* 实测速度·左轮 [rps] */
#define CT_U_L                 2  /* 输出占空比·左轮 [%] */
#define CT_TARGET_R            3  /* 目标速度·右轮 [rps] */
#define CT_V_R                 4  /* 实测速度·右轮 [rps] */
#define CT_U_R                 5  /* 输出占空比·右轮 [%] */
#define CT_STEER_TGT           6  /* 目标转角(已施加) [deg] */
#define CT_STEER_DELTA         7  /* 实测转角 [deg] */
#define CT_STEER_U             8  /* 转向输出 [duty] */
#define CT_ONLINE              9  /* 链路在线 */
#define CT_ENABLE              10 /* 使能位图 */
#define CT_SAFETY              11 /* 保护状态位图 */
#define CT_STEER_EN            12 /* 转向内环使能 */
#define CT_STEER_ST            13 /* 转向状态位图 */
#define CT_OL_EN               14 /* 开环模式 */
#define CT_RESV0               15 /* 预留位 */
#define CT_K_V                 16 /* 比例增益 k_v */
#define CT_K_Z                 17 /* 积分增益 k_z */

/* ---- 采集档 train：16 通道 ---- */
#define TRCH_STEER_TARGET      0  /* 目标转角(已施加) [deg] */
#define TRCH_STEER_DELTA       1  /* 实测转角 [deg] */
#define TRCH_STEER_U           2  /* 转向输出 [duty] */
#define TRCH_STEER_STATUS      3  /* 转向状态位图 */
#define TRCH_TARGET_L          4  /* 目标速度·左轮 [rps] */
#define TRCH_V_L               5  /* 实测速度·左轮 [rps] */
#define TRCH_U_L               6  /* 输出占空比·左轮 [%] */
#define TRCH_TARGET_R          7  /* 目标速度·右轮 [rps] */
#define TRCH_V_R               8  /* 实测速度·右轮 [rps] */
#define TRCH_U_R               9  /* 输出占空比·右轮 [%] */
#define TRCH_INS_GZ            10 /* 陀螺 Z 角速度 [deg/s] */
#define TRCH_INS_STATIC        11 /* 本拍判静止 */
#define TRCH_ENABLE_BITS       12 /* 使能位图 */
#define TRCH_SAFETY            13 /* 保护状态位图 */
#define TRCH_RC_ENGAGE         14 /* 接管状态 */
#define TRCH_TICK_MS           15 /* 上电毫秒数 [ms] */
// <<< GEN:TEL_CH <<<

/* ---- 五、对外接口 ---- */

/* 初始化: 连 WiFi + 建 TCP/UDP + 上报参数表; 阻塞, 只能在初始化阶段调用
   返回 0=成功 1=WiFi 连接失败 2=Socket 连接失败 */
uint8   motor_link_init          (void);

/* 时基: 必须在 5ms 的 PIT 中断里调用, 且在速度环之前(保护层要用它的时间戳) */
void    motor_link_tick_5ms      (void);

/* 主循环任务: 发遥测 + 收命令, 放在 while(1) 里, 尽量每圈都调到 */
void    motor_link_task          (void);
/* 路表 dump 的分步发送(0x08 读路表), 由 motor_link_task() 每圈调用, 空闲即返回 */
void    motor_route_dump_poll    (void);

/* 状态查询 */
uint8   motor_link_online        (void);     /* 1 = 最近收到过下行帧(链路活着) */
uint16  motor_link_enable_bits   (void);     /* bit1 电机 / bit2 总使能        */
uint32  motor_link_rx_age_ms     (void);     /* 距上次收到下行帧 ms            */
uint32  motor_link_tx_count      (void);     /* 已发出的遥测帧数               */
uint32  motor_link_now_ms        (void);     /* 上电时间 ms(来自 5ms 时基)     */
/* 恢复默认值: 与上位机 LOAD 命令走同一条路径 */
void    motor_param_load_defaults(void);
uint8   motor_link_init_err      (void);     /* motor_link_init 的返回值       */
uint8   motor_link_ever_rx       (void);     /* 1 = 上电以来至少收到过一帧下行 */

/* 遥测通道读写: 由 main 负责填值, 本模块只负责按周期打包发送 */
void    motor_link_ch_set        (uint8 idx, float v);

/* 运行时切换遥测档案(通道数)与遥测周期, 由 main 每圈按 tel_sel / tel_period 推
   n 夹到 [1, MOTOR_CH_COUNT_MAX]; ms 夹到 [MOTOR_TX_PERIOD_MIN, 1000] */
void    motor_link_set_ch_count     (uint16 n);
uint16  motor_link_ch_count         (void);
void    motor_link_set_tx_period_ms (uint16 ms);
uint16  motor_link_tx_period_ms     (void);

#define MOTOR_TX_PERIOD_MIN     (5)         /* 遥测周期下限 ms(再小主循环扛不住) */

#endif
