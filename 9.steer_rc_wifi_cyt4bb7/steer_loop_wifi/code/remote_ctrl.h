/* 遥控（HotRc 枪式遥控器 / 逐飞 UART 接收机 + SBUS 帧）
 * 本文件把那套"标定值 + 接管状态机"照搬过来，**换成走本工程的接口**：
 * 数据源            uart_receiver 快照(UART_4/P14_0, SBUS)   同一个（驱动逐飞库里就有）
 * 方向盘            Servo_Target_Deg                         steer.target_delta / steer_en_cmd
 * 后轮              Motor_LR_Crtl.target / Motor_RR_Crtl.target   spd_target_l / spd_target_r
 * 使能              Servo_Crtl_Enable / Motor_Crtl_Enable    steer_en_cmd / spd_safety_set_local()
 * 页面              Remote_Crtl_Page()（IPS200 菜单里的一项）  **没有页面** —— 见下
 * CH1  摇杆    方向盘（左右）                  -> RC_Steer_Deg
 * CH2  扳机    油门（前后）                    -> RC_Throttle
 * CH3  旋钮    方向行程                        -> **暂不做，留空**（只读出来备将来用）
 * CH4  旋钮    油门行程 = **速度缩放**          -> RC_Scale
 * CH5  二值    急停                            -> RC_Estop（锁存，见下）
 * CH6  二值    使能 / 接管                     -> 接管状态机的合闸开关
 * 开关极性（192 = 低档 / 1792 = 高档，实测标定值）：
 * CH5 急停   ：`RC_ESTOP_ACTIVE_HIGH = 1` -> **打到高档 = 急停生效**
 * CH6 使能   ：`RC_ARM_ACTIVE_HIGH   = 1` -> **打到高档 = 允许接管**
 * 改用**发射机上的两个开关**：
 * 合闸条件（三条同时满足）：CH6 打到高档  且  接收机在线(ST=1)  且  参数 `rc_arm_en` = 1
 * 合闸后：先进 STANDBY（双环使能、目标 0，让人看清方向盘回中）-> 油门回中才升 ON
 * CH6 收回 / 上位机把 `rc_arm_en` 写 0 -> 立刻回到 OFF（全部失能、目标 0）
 * `RC_ESTOP_ACTIVE_HIGH = 1` 时，CH5 打到高档即急停生效。生效后：
 * ① 后轮：`spd_safety_set_estop(1)` -> 保护层把输出**无条件钉 0**（SAFE_BIT_ESTOP，**不可被本地接管替换**）；
 * ② 转向：`steer_ctrl_set_estop(1)` -> 内环强制失能、输出钉 0；
 * ③ 遥控自己的目标全部清零、本地接管档位放掉。
 * ⇒ **就算此刻是 WiFi 上位机在开车，拨一下 CH5 也能把车停住。** 这是刻意做的：
 * "至少要有一种手段能控制住车辆"，遥控不该只在"它自己接管时"才有用。
 * 保护层有三道"上位机在场"的门（没收到过下行帧 / 没有使能位 / 链路离线），
 * SPI 没插的时候它们会把电机**永久锁死且不报错**。所以：
 * 遥控进入 ON / STANDBY 时 -> `spd_safety_set_local(1)`，把这三道门换成"本地授权"；
 * 退出 / 失控 / 急停          -> `spd_safety_set_local(0)`，完整五道门原样恢复。
 * 两条路合起来正好是「**WiFi 在场 OR 遥控在场**」，都不会出力的情形只有"两个都不在"。
 * ⇒ **喂心跳是硬约束**：谁能把后轮闭环转起来，谁就得每圈喂 `spd_safety_heartbeat()`。
 * `RC_THR_TARGET_FWD = 10.0` 换算过来只有约 **0.54 rps**（cpr=1858）。
 * 本工程 `spd_ctrl` 的目标量纲是 **rps**，所以这里用 `rc_thr_max`（默认 2.0 rps）表达满杆，
 * 再乘 CH4 旋钮的缩放，最后夹一次 `spd_target_max`。
 */

#ifndef _REMOTE_CTRL_H
#define _REMOTE_CTRL_H

#include "zf_common_headfile.h"

/* 一、通道分配 */
#define RC_CH_STEER             (0)             /* CH1 摇杆 → 方向盘指令 */
#define RC_CH_THROTTLE          (1)             /* CH2 扳机 → 前后油门 */
#define RC_CH_STEER_TRAVEL      (2)             /* CH3 旋钮 → 方向行程（**暂不做，留空**） */
#define RC_CH_THR_TRAVEL        (3)             /* CH4 旋钮 → 油门行程 = 速度缩放 */
#define RC_CH_ESTOP             (4)             /* CH5 二值 → 急停 */
#define RC_CH_ARM               (5)             /* CH6 二值 → 使能/接管 */

/* 开关极性：192 = 低档, 992 = 中位, 1792 = 高档（下同）。 */
#define RC_ESTOP_ACTIVE_HIGH    (1)             /* 1 = CH5 高档急停生效；0 = 低档生效 */
#define RC_ARM_ACTIVE_HIGH      (1)             /* 1 = CH6 高档允许接管；0 = 低档 */

#define RC_STEER_MID            (992)
#define RC_STEER_MIN            (192)
#define RC_STEER_MAX            (1792)

#define RC_THR_MID              (992)
#define RC_THR_MIN              (192)
#define RC_THR_MAX              (1792)

/* 开关档位原始值（CH5/CH6 两态；CH3/CH4 是旋钮，用上面三个极值做满幅换算） */
#define RC_SW_VAL_LOW           (192)
#define RC_SW_VAL_MID           (992)
#define RC_SW_VAL_HIGH          (1792)

/* 方向盘极性：+1 = 原始值增大 = "向右"。实测方向相反就改 -1。 */
#define RC_STEER_DIR_SIGN       (+1)

/* 中位死区（原始计数）：|CH - 992| 不超过它就算 0。
 * 方向盘用它把"没碰杆子时的零点漂移"直接压成 0 度。
 * （换算：方向盘 ±0.27°（满 27°）；油门 ±0.02 rps（满杆默认 2 rps））。
 */
#define RC_STEER_DEADBAND       (8)
#define RC_THR_DEADBAND         (8)

/* 油门满杆对应的目标转速 rps（在线可改，见参数表 rc_thr_max）。 第一次上车请按 01-上电标定卡.md 从小往大加。 */
#define RC_THR_MAX_DEFAULT      (2.0f)

/* 三、CH4 油门行程旋钮 -> 速度缩放 旋钮在低位 = 慢（缩放到 RC_SCALE_MIN），旋到高位 = 快（缩放到 RC_SCALE_MAX）。 */
#define RC_SCALE_MIN            (0.20f)
#define RC_SCALE_MAX            (1.00f)

/* 合闸判定：油门归一化值不超过它才允许从 STANDBY 升到 ON（防止合闸瞬间全油门窜车） */
#define RC_ARM_THR_NEUTRAL      (0.05f)

/* 四、接管状态机 */
#define RC_ENGAGE_OFF           (0)     /* 全部失能、目标 0（开关收回 / 未授权） */
#define RC_ENGAGE_STANDBY       (1)     /* 双环使能、目标 0（刚合闸，等油门回中） */
#define RC_ENGAGE_ON            (2)     /* 双环使能 + 遥控设置目标（真正接管） */
#define RC_ENGAGE_FAILSAFE      (3)     /* 失控挂起：转向失能；后轮目标 0（锁存） */
#define RC_ENGAGE_ESTOP         (4)

/* 五、对外数据（都由 remote_ctrl_update() / remote_ctrl_apply() 刷新） */
extern volatile uint16  RC_Raw_Ch1;         /* CH1 原始值（标定/显示用） */
extern volatile uint16  RC_Raw_Ch2;         /* CH2 原始值 */
extern volatile uint16  RC_Raw_Ch4;         /* CH4 原始值（油门行程旋钮） */
extern volatile uint16  RC_Raw_Ch5;         /* CH5 原始值（急停开关） */
extern volatile uint16  RC_Raw_Ch6;         /* CH6 原始值（使能开关） */
extern volatile float   RC_Steer_Deg;       /* 方向盘指令角（度，向右为正；已夹在 ±27 内） */
extern volatile float   RC_Throttle;        /* 归一化油门 -1.0 ~ +1.0（前进为正） */
extern volatile float   RC_Scale;           /* CH4 旋钮算出的速度缩放 0~1 */
extern volatile uint8   RC_Estop;           /* 1 = 急停开关生效 */
extern volatile uint8   RC_Arm;             /* 1 = 使能开关合闸 */
extern volatile uint8   RC_Online;          /* 1 = 接收机报告状态正常（未失控） */
extern volatile uint8   RC_Engage;          /* 接管状态，取 RC_ENGAGE_* */
extern volatile uint32  RC_Frames;          /* 累计收到的完整 SBUS 帧数（不涨 = 接收机/接线问题） */
extern volatile float   RC_Target_Rps;      /* 本拍算出来的后轮目标 rps（带缩放） */

/* 遥控总开关：1 = 允许遥控接管（上位机可一键禁掉，避免和调参抢车） */
extern volatile float   rc_arm_en;
/* 油门满杆目标 rps（在线可改） */
extern volatile float   rc_thr_max;

/* 六、接口 */
/* 初始化：初始化接收机（UART_4 / P14.0，SBUS 100 kbps）+ 清状态。
 * 注意：必须在 5ms PIT 中断起来之前调用（接收机的帧间隔计时用 TC_TIME2_CH2）。
 */
void    remote_ctrl_init    (void);

/* 主循环每圈调一次：读 uart_receiver 快照 -> 换算成有物理含义的指令量（纯算术，可忽略开销） */
void    remote_ctrl_update  (void);

/* 主循环每圈调一次（在 update 之后）：跑接管状态机 + 写目标 + 推急停/本地接管档位 */
void    remote_ctrl_apply   (void);

/* 立刻解除接管（把两环失能、目标清零、急停与本地档位都放开）。上电/停机路径调用。 */
void    remote_ctrl_release (void);

/* 急停是否生效（供 1Hz 打印/遥测；真实输出已经被保护层钉住，这里只是回读） */
uint8   remote_ctrl_estop   (void);

#endif
