/*********************************************************************************************************************
 * 三路 DRV8701E 驱动输出：转向位置环 + 双后轮速度环，PWM + DIR
 *
 *   通道  用途       PWM 引脚(通道)              DIR 引脚
 *   ch0   左后轮     P09_0 (TCPWM_CH24)          P09_1
 *   ch1   右后轮     P10_2 (TCPWM_CH30)          P10_3
 *   ch2   转向       P05_0 (TCPWM_CH09)          P05_1
 *
 *   PWM 17 kHz，比较值满量程 PWM_DUTY_MAX = 10000
 *   逐飞 PWM 按通道号寻址，一个通道有多个候选引脚；三路占 CH24/CH30/CH09，互不重叠
 *   同通道两脚不能同时初始化为 PWM，且不会报错
 *   P09_1 / P10_3 / P05_1 只作普通 GPIO 用
 *   DIR 约定：u >= 0 -> DIR 高；u < 0 -> DIR 低
 *   u = 0 -> PWM 0%(自由滑行, 非刹车)；|u| > 1 夹到 1
 ********************************************************************************************************************/

#ifndef _MOTOR_DRV_H
#define _MOTOR_DRV_H

#include "zf_common_headfile.h"

#define MOTOR_CH_COUNT_DRV  (3)         /* 0=左后轮  1=右后轮  2=转向 */

#define MOTOR_PWM_FREQ_HZ   (17000)

#define MOTOR_L_PWM_PIN     (TCPWM_CH24_P09_0)
#define MOTOR_L_DIR_PIN     (P09_1)

#define MOTOR_R_PWM_PIN     (TCPWM_CH30_P10_2)
#define MOTOR_R_DIR_PIN     (P10_3)

/* ch2 转向: 直流电机 + 绝对编码器做位置环(不是 RC 舵机), 见 steer_ctrl.h */
#define MOTOR_S_PWM_PIN     (TCPWM_CH09_P05_0)
#define MOTOR_S_DIR_PIN     (P05_1)

/* 电机极性: > 0.5 表示把该路的 u 整体取反(等价于电机两根线对调, 二者只能选一个)
   核对方法: 给正 u -> 电机朝车前进方向转, 且同侧编码器读数为正; 上位机可在线改
   转向极性给反会使位置环正反馈并堵转, 必须单独核对 */
extern float mot_inv_l;
extern float mot_inv_r;
extern float mot_inv_s;         /* 转向电机极性 */

void    motor_drv_init      (void);                     /* 初始化三路 PWM/DIR, 立即输出 0 */
void    motor_drv_set       (uint8 ch, float u);        /* ch: 0=左 1=右 2=转向; u 归一化带符号 */
void    motor_drv_stop_all  (void);                     /* 三路立即归零(不经过任何斜坡)   */

float   motor_drv_out       (uint8 ch);                 /* 本拍实际写下去的值(已含极性)   */
int     motor_drv_dir_level (uint8 ch);                 /* DIR 引脚当前电平 0/1           */
uint32  motor_drv_pwm_raw   (uint8 ch);                 /* 写进 PWM 的比较值              */

#endif
