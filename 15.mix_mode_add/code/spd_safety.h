/* 双电机速度环 - 保护与门控: 决定本拍是否允许电机出力, 并把输出上限压给控制器
 *
 *  五道门(全满足才允许闭环输出): 收到过下行帧; 使能位 bit1 与 bit2; 未失联;
 *  主循环心跳未超时; 占空比限幅 duty_limit% 压给 u_max(另有 motor_drv 的 |u|<=1 收口)
 *  配置自检(通道数/周期/PWM 频率)在运行期做, 不写进 #if */

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
#define SAFE_BIT_LOCAL      (0x0040u)       /* 本地接管中(信息位, 不阻塞)      */
#define SAFE_BIT_ESTOP      (0x0080u)       /* 遥控急停 CH5: 全局硬停机, 本地接管不可替换 */

#define SPD_HB_TIMEOUT_MS_PUB   (200u)      /* 主循环心跳超时 ms, 与 .c 内一致 */

/* 占空比限幅 %: 硬顶 SPD_DUTY_HARD_MAX, 上电默认 SPD_DUTY_DEFAULT;
   本宏为唯一来源, motor_link.h 参数表 ID2 的默认值也引用它 */
#define SPD_DUTY_HARD_MAX   (80.0f)
#define SPD_DUTY_DEFAULT    (80.0f)

extern volatile float   spd_duty_limit_pct;     /* 占空比限幅 %(上位机可改)  */
extern volatile float   spd_lost_ms;            /* 失联判定延时 ms           */
extern volatile float   spd_auto_stop;          /* 失联自停 1/0              */

void    spd_safety_init         (void);
void    spd_safety_tick         (void);         /* 5 ms 更新保护状态(由 spd_ctrl_tick 调) */
void    spd_safety_heartbeat    (void);         /* 主循环每圈调一次                       */

/* 本地接管: 任一来源位置位即跳过上位机三道门并亮 SAFE_BIT_LOCAL, 心跳与限幅仍生效 */
#define SPD_LOCAL_SRC_RC        (0x01u)     /* 遥控(ON / STANDBY 时置位) */
#define SPD_LOCAL_SRC_PB        (0x02u)     /* 路表回放 PWR */
#define SPD_LOCAL_SRC_BENCH     (0x04u)     /* 跑班(保留) */
void    spd_safety_set_local_src(uint8 src, uint8 on);
void    spd_safety_set_local    (uint8 on);      /* 只动遥控那一位 */
uint8   spd_safety_local        (void);         /* 当前档位(回读, 诊断用)                 */

/* 遥控急停 CH5: 全局硬停机, 与谁在开车无关; on=1 亮 SAFE_BIT_ESTOP, 输出无条件钉 0 */
void    spd_safety_set_estop    (uint8 on);
uint8   spd_safety_estop        (void);

float   spd_safety_limit_u      (void);         /* 本拍允许的 |u| 上限(归一化)            */
uint8   spd_safety_allow        (uint8 ch);     /* 该路是否允许闭环输出 1/0               */
uint32  spd_safety_flags        (void);         /* 当前保护状态位图(遥测)                 */
uint32  spd_safety_trip_take    (void);         /* 取走并清除新发生的停机事件            */
uint8   spd_safety_cfg_bad      (void);         /* 配置自检结果位图(0 = 正常)             */

/* 控制器发现本拍输出被 u_max 夹掉时调一次, 点亮 SAFE_BIT_OVER_LIM;
   该位每拍刷新, 本拍未被夹则下一拍自动熄灭 */
void    spd_safety_note_clip    (uint8 ch);

#endif
