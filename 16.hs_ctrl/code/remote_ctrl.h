/* 遥控（HotRc 枪式遥控器 / 逐飞 UART 接收机 + SBUS 帧）
 * 接收机：UART_4 = SCB2，RX = P14.0，SBUS 100 kbps，帧间隔计时用 TC_TIME2_CH2
 * 内容：通道分配、中位与量程、死区、中位学习、接管状态机、急停与本地授权
 */

#ifndef _REMOTE_CTRL_H
#define _REMOTE_CTRL_H

#include "zf_common_headfile.h"

/* ---- 一、通道分配 ----
 * CH1 摇杆 方向盘   CH2 扳机 油门     CH3 旋钮 空置
 * CH4 三档 空置     CH5 二值 急停     CH6 二值 使能/接管合闸 */
#define RC_CH_STEER             (0)             /* CH1 摇杆 -> 方向盘          */
#define RC_CH_THROTTLE          (1)             /* CH2 扳机 -> 前后油门        */
#define RC_CH_STEER_TRAVEL      (2)             /* CH3 空置, 仅显示原始值      */
#define RC_CH_THR_TRAVEL        (3)             /* CH4 空置, 行程在发射机侧缩放 */
#define RC_CH_ESTOP             (4)             /* CH5 二值 -> 急停            */
#define RC_CH_ARM               (5)             /* CH6 二值 -> 使能/接管       */

/* 开关极性：192 = 低档, 992 = 中位, 1792 = 高档（全文同此约定） */
#define RC_ESTOP_ACTIVE_HIGH    (1)             /* 1 = CH5 高档急停生效；0 = 低档生效 */
#define RC_ARM_ACTIVE_HIGH      (1)             /* 1 = CH6 高档允许接管；0 = 低档     */

/* ---- 二、中位与量程 ----
 * 摇杆中位 992，量程 192~1792（单侧 800 计数）；换遥控器或重新微调后需重标中位 */
#define RC_STEER_MID            (992)
#define RC_STEER_MIN            (192)
#define RC_STEER_MAX            (1792)

#define RC_THR_MID              (992)
#define RC_THR_MIN              (192)
#define RC_THR_MAX              (1792)

/* 开关档位原始值（CH5/CH6 两态；CH3/CH4 用三个极值做满幅换算） */
#define RC_SW_VAL_LOW           (192)
#define RC_SW_VAL_MID           (992)
#define RC_SW_VAL_HIGH          (1792)

/* 方向盘极性：+1 = 原始值增大为向右；方向相反改成 -1 */
#define RC_STEER_DIR_SIGN       (+1)

/* 中位死区（原始计数）：|CH - 中位| 不超过它按 0 处理，区内归零不做重映射
 * ±8 约为单侧量程 800 的 1%：方向盘 ±0.27°（满 27°），油门 ±0.02 rps（满杆 2 rps） */
#define RC_STEER_DEADBAND       (8)
#define RC_THR_DEADBAND         (8)

/* 中位学习：中位是运行期值，对外用 RC_Thr_Mid / RC_Steer_Mid 读取
 * ① 开机捕获：信号首次有效起的第 3~20 帧内，raw 落在标称中位 ±RC_MID_CAP_BAND 内即锁为静止位
 * ② 慢跟踪：每 RC_MID_TRACK_MS 最多挪 RC_MID_TRACK_STEP 个计数，仅当 |raw - mid| 在跟踪带内
 * 只调中位不动量程 ±800，lo/hi 随 mid 平移，满杆仍为 ±1.0 */
#define RC_MID_CAP_BAND         (80)        /* 开机捕获：|raw - 标称中位| 允许多大 */
#define RC_MID_TRACK_BAND       (100)       /* 慢跟踪带, 必须 > CAP_BAND, 否则捕获偏差落在带外无法修正 */
#define RC_MID_TRACK_STEP       (1)         /* 慢跟踪：每次最多挪几个计数 */
#define RC_MID_TRACK_MS         (100u)      /* 慢跟踪：多久挪一次（1 计数 / 100 ms）*/

/* 油门满杆对应的目标转速 rps（在线可改, 参数表 rc_thr_max） */
#define RC_THR_MAX_DEFAULT      (2.0f)

/* ---- 三、油门行程缩放 ----
 * 行程在发射机侧缩放, CH4 空置, 代码内恒乘 1.0；调快慢改 rc_thr_max 或发射机旋钮 */
#define RC_SCALE_MIN            (0.20f)
#define RC_SCALE_MAX            (1.00f)

/* 通道有效下限：raw 低于它按通道无信号处理
 * 接收机取不到通道时 channel[] 为 0, 0 在量程 192~1792 之外, 归一化得 -1.24 夹到 -1 即满舵/满倒车
 * 低于下限按摇杆回中处理, 并记入 RC_Stick_Bad 位图 */
#define RC_CH_VALID_MIN         (120)

/* 合闸判定：油门归一化值不超过它才允许从 STANDBY 升到 ON */
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
extern volatile uint16  RC_Raw_Ch3;         /* CH3 原始值（已空置，仅显示/查线用）*/
extern volatile uint16  RC_Raw_Ch4;         /* CH4 原始值（已空置）       */
extern volatile uint16  RC_Raw_Ch5;         /* CH5 原始值（急停开关）      */
extern volatile uint16  RC_Raw_Ch6;         /* CH6 原始值（使能开关）      */
extern volatile float   RC_Steer_Deg;       /* 方向盘指令角（度，向右为正；已夹在 ±27 内）*/
extern volatile float   RC_Steer_Norm;      /* CH1 归一化 -1~+1（已过中位死区） */

extern volatile uint8   RC_Stick_Bad;       /* 通道掉线位图 bit0=CH1 bit1=CH2（raw 低于下限）*/
extern volatile uint16  RC_Thr_Mid;         /* 学习到的油门中位（非固定 992）*/
extern volatile uint16  RC_Steer_Mid;       /* 学习到的方向盘中位 */

extern volatile float   RC_Throttle;        /* 归一化油门 -1.0 ~ +1.0（前进为正）*/
extern volatile float   RC_Scale;           /* 速度缩放, 恒 RC_SCALE_MAX  */
extern volatile uint8   RC_Estop;           /* 1 = 急停开关生效           */
extern volatile uint8   RC_Arm;             /* 1 = 使能开关合闸           */
extern volatile uint8   RC_Online;          /* 1 = 接收机报告状态正常（未失控） */
extern volatile uint8   RC_Engage;          /* 接管状态，取 RC_ENGAGE_* */
extern volatile uint32  RC_Frames;          /* 累计收到的完整 SBUS 帧数（不涨 = 接收机/接线问题）*/
extern volatile float   RC_Target_Rps;      /* 本拍算出来的后轮目标 rps（带缩放） */

/* 遥控总开关：1 = 允许遥控接管（上位机可一键禁掉，避免和调参抢车） */
extern volatile float   rc_arm_en;
/* 油门满杆目标 rps（在线可改） */
extern volatile float   rc_thr_max;

/* ---- 六、接口 ---- */
/* 初始化：初始化接收机（UART_4 / P14.0，SBUS 100 kbps）+ 清状态。
   必须在 5 ms PIT 中断起来之前调用（接收机的帧间隔计时用 TC_TIME2_CH2）。 */
void    remote_ctrl_init    (void);

/* 主循环每圈调一次：读 uart_receiver 快照 -> 换算成有物理含义的指令量 */
void    remote_ctrl_update  (void);

/* 主循环每圈调一次（在 update 之后）：跑接管状态机 + 写目标 + 推急停/本地接管档位 */
void    remote_ctrl_apply   (void);

/* 立刻解除接管（两环失能、目标清零、急停与本地档位都放开）。上电/停机路径调用。 */
void    remote_ctrl_release (void);

/* 急停是否生效（供 1 Hz 打印/遥测；真实输出已被保护层钉住，这里只是回读） */
uint8   remote_ctrl_estop   (void);

#endif
