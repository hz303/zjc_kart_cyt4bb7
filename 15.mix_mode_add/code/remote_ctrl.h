/*********************************************************************************************************************
 * 遥控：HotRc 枪式遥控器 + 逐飞 UART 接收机（SBUS 帧）
 * 数据源 uart_receiver 快照，UART_4 / P14_0，SBUS 100 kbps，16 通道；量程 192 ~ 1792，中位 992
 * 接管时以 spd_safety_set_local(1) 替换上位机在场的三道门；主循环心跳与占空比限幅始终生效
 ********************************************************************************************************************/

#ifndef _REMOTE_CTRL_H
#define _REMOTE_CTRL_H

#include "zf_common_headfile.h"

/* ---- 一、通道分配 ---- */
#define RC_CH_STEER             (0)             /* CH1 摇杆   → 方向盘；CH3 为低时改控目标航向 */
#define RC_CH_THROTTLE          (1)             /* CH2 扳机   → 前后油门   */
#define RC_CH_STEER_TRAVEL      (2)             /* CH3 二值开关 → 低 = 航向模式 / 高 = 舵机角模式 */
#define RC_CH_THR_TRAVEL        (3)             /* CH4 三档开关 → 空置（行程在发射机侧缩放）*/
#define RC_CH_ESTOP             (4)             /* CH5 二值   → 急停       */
#define RC_CH_ARM               (5)             /* CH6 二值   → 使能/接管  */

/* 开关档位原始值：192 = 低档，992 = 中位，1792 = 高档 */
#define RC_ESTOP_ACTIVE_HIGH    (1)             /* 1 = CH5 高档急停生效；0 = 低档生效 */
#define RC_ARM_ACTIVE_HIGH      (1)             /* 1 = CH6 高档允许接管；0 = 低档     */

/* ---- 二、标定值 ----
 * 中位 992，单侧满量程 800；重新微调后中位需重标 */
#define RC_STEER_MID            (992)
#define RC_STEER_MIN            (192)
#define RC_STEER_MAX            (1792)

#define RC_THR_MID              (992)
#define RC_THR_MIN              (192)
#define RC_THR_MAX              (1792)

/* 开关档位原始值；CH3/CH4 为旋钮，按上述极值做满幅换算 */
#define RC_SW_VAL_LOW           (192)
#define RC_SW_VAL_MID           (992)
#define RC_SW_VAL_HIGH          (1792)

/* 方向盘极性：+1 = 原始值增大为向右；方向相反改 -1 */
#define RC_STEER_DIR_SIGN       (+1)

/* 中位死区（原始计数）：|CH - 992| 不超过它按 0 处理
 * ±8 约为单侧满量程的 1%（方向盘 ±0.27°，油门 ±0.02 rps）；区内归零，不重映射 */
#define RC_STEER_DEADBAND       (8)
#define RC_THR_DEADBAND         (8)

/* 中位学习：开机捕获 + 慢跟踪，运行期值为 RC_Thr_Mid / RC_Steer_Mid
 * 只改中位，量程仍 ±800，lo/hi 随中位平移，满杆仍为 ±1.0 */
#define RC_MID_CAP_BAND         (80)        /* 开机捕获：|raw - 标称中位| 允许范围 */
#define RC_MID_TRACK_BAND       (100)       /* 慢跟踪：|raw - mid| 小于此值才挪动；须大于 CAP_BAND */
#define RC_MID_TRACK_STEP       (1)         /* 慢跟踪：每次最多挪动计数 */
#define RC_MID_TRACK_MS         (100u)      /* 慢跟踪：挪动周期 ms */

/* 油门满杆对应的后轮目标转速 rps（参数表 rc_thr_max 可在线改） */
#define RC_THR_MAX_DEFAULT      (2.0f)

/* ---- 三、油门行程缩放 ----
 * 行程在发射机侧缩放，CH2 已缩过，代码不再乘；CH4 空置，RC_Scale 恒 1.0
 * RC_SCALE_MIN/MAX 仅为预留，当前路径不使用 */
#define RC_SCALE_MIN            (0.20f)
#define RC_SCALE_MAX            (1.00f)

/* ---- 三·五、CH3 二值开关 + CH1 航向微调 ----
 * CH1 在死区内 = 以当前航向为基准并保持；推左/右 = 基准 ± 偏移（≤ ±20°）
 * 松手时重取基准，摇杆相当于相对当前航向的微调；步长 0.5° */
#define RC_HDG_MAX_DEG          (20.0f)     /* CH1 打满对应的航向偏移上限（±） */
#define RC_HDG_KP               (2.20f)     /* 航向环增益 deg/deg（纯追踪 ld = 0.6 m 时取 2L/ld） */
#define RC_HDG_STEP             (0.5f)      /* 偏移量化步长 deg（20/0.5 = 40 格） */

/* 通道有效性下限：raw 小于它按该通道无信号处理
 * 接收机取不到通道时值为 0，落在 192 ~ 1792 之外，归一化后会被夹成满舵/满倒车 */
#define RC_CH_VALID_MIN         (120)

/* CH3 阈值运行时自适应：记 min/max，见过高位（>1200）且跨度 ≥400 才标定，阈值取中点
 * 未标定时保持舵机角模式 */
#define RC_HDG_HYST             (60u)       /* 阈值滞环（计数）：防临界点抖动时模式来回翻 */

/* 合闸门限：油门归一化绝对值不超过它才允许从 STANDBY 升到 ON */
#define RC_ARM_THR_NEUTRAL      (0.05f)

/* ---- 四、接管状态机 ---- */
#define RC_ENGAGE_OFF           (0)     /* 全部失能、目标 0（开关收回 / 未授权）        */
#define RC_ENGAGE_STANDBY       (1)     /* 双环使能、目标 0（刚合闸，等油门回中）        */
#define RC_ENGAGE_ON            (2)     /* 双环使能 + 遥控设置目标（真正接管）          */
#define RC_ENGAGE_FAILSAFE      (3)     /* 失控挂起：转向失能；后轮目标 0（锁存）        */
#define RC_ENGAGE_ESTOP         (4)     /* 急停：全局硬停机（锁存，CH5 回位才解除）      */

/* ---- 五、对外数据（由 remote_ctrl_update() / remote_ctrl_apply() 刷新） ---- */
extern volatile uint16  RC_Raw_Ch1;         /* CH1 原始值（标定/显示用） */
extern volatile uint16  RC_Raw_Ch2;         /* CH2 原始值                 */
extern volatile uint16  RC_Raw_Ch3;         /* CH3 原始值（模式开关）     */
extern volatile uint16  RC_Raw_Ch4;         /* CH4 原始值（空置）         */
extern volatile uint16  RC_Raw_Ch5;         /* CH5 原始值（急停开关）      */
extern volatile uint16  RC_Raw_Ch6;         /* CH6 原始值（使能开关）      */
extern volatile float   RC_Steer_Deg;       /* 方向盘指令角（度，向右为正；已夹在 ±27 内）*/
extern volatile float   RC_Steer_Norm;      /* CH1 归一化 -1~+1（已过中位死区） */
extern volatile uint8   RC_Ch3_Pct;         /* CH3 二值：0 = 低(航向) / 100 = 高(舵机) */
extern volatile uint8   RC_Stick_Bad;       /* 通道掉线位图 bit0=CH1 bit1=CH2 bit2=CH3（raw 过低）*/
extern volatile uint16  RC_Thr_Mid;         /* 学习到的油门中位（开机捕获 + 慢跟踪）*/
extern volatile uint16  RC_Steer_Mid;       /* 学习到的方向盘中位 */
extern volatile uint8   RC_Hdg_Mode;        /* 1 = CH1 正在控制航向；0 = 控制舵机角 */
extern volatile uint16  RC_Hdg_Thr;         /* CH3 判高/低的当前阈值（运行时自适应）*/
extern volatile uint8   RC_Hdg_Cal;         /* 1 = CH3 已标定（见过开关两端） */
extern volatile float   RC_Hdg_Step;        /* 步长 deg（现固定 0.5） */
extern volatile float   RC_Hdg_Off;         /* 当前航向偏移 deg（已量化、已夹 ±20） */
extern volatile float   RC_Cmd_Yaw;         /* 航向模式下的目标航向 deg（= 基准 + 偏移）*/
extern volatile float   RC_Throttle;        /* 归一化油门 -1.0 ~ +1.0（前进为正）*/
extern volatile float   RC_Scale;           /* CH4 旋钮算出的速度缩放 0~1  */
extern volatile uint8   RC_Estop;           /* 1 = 急停开关生效           */
extern volatile uint8   RC_Arm;             /* 1 = 使能开关合闸           */
extern volatile uint8   RC_Online;          /* 1 = 接收机报告状态正常（未失控） */
extern volatile uint8   RC_Engage;          /* 接管状态，取 RC_ENGAGE_* */
extern volatile uint32  RC_Frames;          /* 累计收到的完整 SBUS 帧数；不涨 = 接收机/接线问题 */
extern volatile float   RC_Target_Rps;      /* 本拍算出的后轮目标 rps（带缩放） */

/* 遥控总开关：1 = 允许遥控接管（上位机可一键禁掉，避免和调参抢车） */
extern volatile float   rc_arm_en;
/* 油门满杆目标 rps（在线可改） */
extern volatile float   rc_thr_max;

/* ---- 六、接口 ---- */
/* 初始化接收机（UART_4 / P14.0，SBUS 100 kbps）并清状态
 * 必须在 5 ms PIT 中断使能前调用（帧间隔计时用 TC_TIME2_CH2） */
void    remote_ctrl_init    (void);

/* 主循环每圈调一次：读 uart_receiver 快照，换算成有物理含义的指令量 */
void    remote_ctrl_update  (void);

/* 主循环每圈调一次（在 update 之后）：跑接管状态机、写目标、推急停与本地接管档位 */
void    remote_ctrl_apply   (void);

/* 立刻解除接管：两环失能、目标清零、急停与本地档位都放开。上电/停机路径调用 */
void    remote_ctrl_release (void);

/* 急停是否生效（供 1Hz 打印/遥测；输出已由保护层钉住，这里只是回读） */
uint8   remote_ctrl_estop   (void);

#endif
