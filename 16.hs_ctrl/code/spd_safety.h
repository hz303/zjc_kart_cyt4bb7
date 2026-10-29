/***********************************************************************************************************************
 * 双电机速度环(WiFi) - 保护与门控: 决定本拍允不允许电机出力, 并把输出上限压给控制器
 * ---- 五道门(全部满足才允许输出) ----
 *   1) 收到过下行帧(motor_link_ever_rx)   2) 使能位 bit1(电机) 且 bit2(总)
 *   3) 未失联: auto_stop=1 时超 lost_ms 没收到下行 -> 目标归零
 *   4) 主循环心跳: 超 200 ms 没调 spd_safety_heartbeat() -> 直接停机
 *   5) 占空比限幅: duty_limit% 压成 u_max; motor_drv 里还有一道 |u| <= 1
 * 配置自检(通道数/周期/PWM 频率)放运行期: #if 的浮点常量 IAR 不认。
 **********************************************************************************************************************/

#ifndef _SPD_SAFETY_H
#define _SPD_SAFETY_H

#include "zf_common_headfile.h"

/* 保护状态位图(遥测 CH_SAFETY, 0 = 一切正常) */
#define SAFE_BIT_NEVER_RX   (0x0001u)       /* 上电以来还没收到过下行帧        */
#define SAFE_BIT_NOT_EN     (0x0002u)       /* 使能位缺失(bit1 电机 / bit2 总) */
#define SAFE_BIT_LOST       (0x0004u)       /* 失联(超 lost_ms)                */
#define SAFE_BIT_NO_HB      (0x0008u)       /* 主循环心跳丢失(卡死)            */
#define SAFE_BIT_CFG_BAD    (0x0010u)       /* 配置自检不过                    */
#define SAFE_BIT_OVER_LIM   (0x0020u)       /* 本拍被占空比限幅夹过            */
#define SAFE_BIT_LOCAL      (0x0040u)       /* 本地接管中(信息位, 不阻塞)       */
#define SAFE_BIT_ESTOP      (0x0080u)       /* 遥控急停(CH5): 全局硬停机, 本地接管替不掉 */

/* 本地接管: 按来源屏蔽 1)/2)/3) 三道上位机门, 改亮 SAFE_BIT_LOCAL(信息位, 不阻塞)。
 *   4) 心跳与 5) 限幅永远生效; 档位由 remote_ctrl.c 推: 遥控接管 ON 置位, 退出/失控清 0。 */
#define SPD_HB_TIMEOUT_MS_PUB   (200u)      /* 主循环心跳超时(与 .c 里一致, 便于外部判读) */

/* 占空比限幅 %: 硬顶 SPD_DUTY_HARD_MAX, 上电默认 SPD_DUTY_DEFAULT。
 *   80% 是唯一的输出硬约束, 热与电流都靠它兜, 长时间顶到 80% 会热(DRV8701E + 电机)。
 *   v[rps] = 0.069*(u% - 5.68), 3.2 rps 约需 52%; 本宏是唯一权威, .c 初值与参数表 ID2 引用它。 */
#define SPD_DUTY_HARD_MAX   (80.0f)
#define SPD_DUTY_DEFAULT    (80.0f)

extern volatile float   spd_duty_limit_pct;     /* 占空比限幅 %(上位机可改)  */
extern volatile float   spd_lost_ms;            /* 失联判定延时 ms           */
extern volatile float   spd_auto_stop;          /* 失联自停 1/0              */

void    spd_safety_init         (void);
void    spd_safety_tick         (void);         /* 5ms: 更新保护状态(由 spd_ctrl_tick 调) */
void    spd_safety_heartbeat    (void);         /* 主循环每圈调一次                       */

/* 本地授权按来源 OR 点灯: 每个来源只管自己那一位, 灭别人的灯会互相覆盖 */
#define SPD_LOCAL_SRC_RC        (0x01u)     /* 遥控（ON / STANDBY 时点） */
#define SPD_LOCAL_SRC_PB        (0x02u)     /* 路表回放 PWR */
#define SPD_LOCAL_SRC_BENCH     (0x04u)     /* 保留：跑班（目前走遥控/上位机使能） */
void    spd_safety_set_local_src(uint8 src, uint8 on);
void    spd_safety_set_local    (uint8 on);      /* 兼容旧调用, 只动遥控那一位 */
uint8   spd_safety_local        (void);         /* 当前档位(回读, 诊断用)                 */

/* 遥控急停(CH5): on=1 亮 SAFE_BIT_ESTOP, 输出无条件钉 0, 本地接管替不掉, 与谁在开车无关 */
void    spd_safety_set_estop    (uint8 on);
uint8   spd_safety_estop        (void);

float   spd_safety_limit_u      (void);         /* 本拍允许的 |u| 上限(归一化)            */
uint8   spd_safety_allow        (uint8 ch);     /* 该路是否允许闭环输出 1/0               */
uint32  spd_safety_flags        (void);         /* 当前保护状态位图(遥测)                 */
uint32  spd_safety_trip_take    (void);         /* 取走并清除新发生的停机事件             */
uint8   spd_safety_cfg_bad      (void);

/* 本拍输出被 u_max 夹掉时调一次, 点亮 SAFE_BIT_OVER_LIM(每拍刷新, 下拍自动熄) */
void    spd_safety_note_clip    (uint8 ch);

#endif
