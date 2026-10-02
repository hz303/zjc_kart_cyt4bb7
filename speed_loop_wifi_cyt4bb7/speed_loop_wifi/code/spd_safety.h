/*********************************************************************************************************************
 * 双电机速度环(WiFi) - 保护与门控(唯一收口)
 *
 * 这一层的唯一职责:**决定"这一拍到底允不允许电机出力"**, 并把上限压给控制器。
 *
 * ---- 五道门(全部满足才允许闭环输出) ------------------------------------------------
 *   1) 上位机曾经说过话      : 上电以来至少收到过一帧下行(motor_link_ever_rx)
 *                            -> 没连过上位机就绝不自转
 *   2) 使能位图              : bit1(电机) 且 bit2(总使能) 都置位
 *   3) 失联自停              : auto_stop=1 且链路超过 lost_ms 没收到下行 -> 目标归零
 *   4) 主循环心跳            : 主循环每圈调 spd_safety_heartbeat();
 *                            超过 200ms 没心跳(主循环卡死/被遥控阻塞) -> 直接停机
 *   5) 占空比限幅            : duty_limit% 压给控制器的 u_max(每拍下压, 抗饱和才真实)
 *                            并且 motor_drv 里还有一道物理收口(|u|<=1)
 *
 * ---- 配置合法性自检 -----------------------------------------------------------------
 *   通道数、周期、PWM 频率这些"写死在宏里、错了就整台车不对"的东西, 上电时查一遍。
 *   注意: 检查必须放运行期 -- 在 #if 里写浮点常量 IAR 会报
 *         "floating constant in preprocessor expression"(这个坑本项目踩过两次)。
 ********************************************************************************************************************/

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

/* 占空比限幅上限(%): 硬顶 SPD_DUTY_HARD_MAX, 默认 30 */
#define SPD_DUTY_HARD_MAX   (60.0f)
#define SPD_DUTY_DEFAULT    (30.0f)

extern volatile float   spd_duty_limit_pct;     /* 占空比限幅 %(上位机可改)  */
extern volatile float   spd_lost_ms;            /* 失联判定延时 ms           */
extern volatile float   spd_auto_stop;          /* 失联自停 1/0              */

void    spd_safety_init         (void);
void    spd_safety_tick         (void);         /* 5ms: 更新保护状态(由 spd_ctrl_tick 调) */
void    spd_safety_heartbeat    (void);         /* 主循环每圈调一次                       */

float   spd_safety_limit_u      (void);         /* 本拍允许的 |u| 上限(归一化)            */
uint8   spd_safety_allow        (uint8 ch);     /* 该路是否允许闭环输出 1/0               */
uint32  spd_safety_flags        (void);         /* 当前保护状态位图(遥测)                 */
uint32  spd_safety_trip_take    (void);         /* 取走并清除"新发生的"停机事件           */
uint8   spd_safety_cfg_bad      (void);         /* 配置自检结果位图(0 = 正常)             */

/* 控制器发现"本拍输出被 u_max 夹掉了"时调一次, 用来点亮 SAFE_BIT_OVER_LIM。
   注意这是"每拍刷新"的瞬时位: 本拍没被夹, 下拍就会自动熄掉。 */
void    spd_safety_note_clip    (uint8 ch);

#endif
