/*********************************************************************************************************************
 * 转向位置环 + 双后轮速度环 + 遥控(WiFi/RC) - 三路 DRV8701E 驱动输出(PWM + DIR)
 *
 * 硬件连接 (17 kHz, PWM_DUTY_MAX = 10000 = 100%, DIR 约定 u >= 0 高 / u < 0 低)
 *   ch0 左后轮  PWM P09_0 (TCPWM_CH24)   DIR P09_1
 *   ch1 右后轮  PWM P10_2 (TCPWM_CH30)   DIR P10_3
 *   ch2 转向    PWM P05_0 (TCPWM_CH09)   DIR P05_1
 *   CH11 (P05_2/P05_3) 预留不用; P09_1 / P10_3 / P05_1 只作普通 GPIO
 *
 * u 为归一化带符号输出, 符号即方向; |u| > 1 夹到 1, 物理限幅在本模块收口:
 *   u = 0 -> PWM 0% (自由滑行, 不是刹车);  u = 0.3 / -0.3 -> DIR 高/低, PWM 30%
 *
 * PWM 时钟: 所有通道共用分频器 0 (80 MHz / 10 = 8 MHz), 频率 = 8 MHz / clockPrescaler / period;
 *   同一通道的两个候选引脚不能同时使用且不报错。
 ********************************************************************************************************************/

#ifndef _MOTOR_DRV_H
#define _MOTOR_DRV_H

#include "zf_common_headfile.h"

#define MOTOR_CH_COUNT_DRV  (3)         /* 0=左后轮  1=右后轮  2=转向 */

#define MOTOR_PWM_FREQ_HZ   (17000)     /* PWM 频率 17 kHz */

/* ch0 左后轮: 引脚改动后 mot_inv_l 必须重核 */
#define MOTOR_L_PWM_PIN     (TCPWM_CH24_P09_0)
#define MOTOR_L_DIR_PIN     (P09_1)

/* ch1 右后轮: 引脚改动后 mot_inv_r 必须重核 */
#define MOTOR_R_PWM_PIN     (TCPWM_CH30_P10_2)
#define MOTOR_R_DIR_PIN     (P10_3)

/* ch2 转向: 直流电机 + 绝对编码器做位置环(不是 RC 舵机), 见 steer_ctrl.h */
#define MOTOR_S_PWM_PIN     (TCPWM_CH09_P05_0)
#define MOTOR_S_DIR_PIN     (P05_1)

/* 电机极性: >0.5 表示把该路 u 整体取反(等价于电机两根线对调, 二者只能选一个)。
   核对方法: 给正 u -> 电机朝车前进方向转, 且同侧编码器读数为正。上位机可在线改。
   默认值 左轮取反(1) / 右轮不取反(0) / 转向取反(1); 换引脚后三项都必须重核。
   后轮给反只是前进变后退; 转向给反 = 位置环正反馈 = 全速顶机械限位。 */
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
