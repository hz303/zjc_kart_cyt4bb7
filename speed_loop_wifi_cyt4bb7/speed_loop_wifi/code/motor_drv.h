/*********************************************************************************************************************
 * 双电机速度环(WiFi) - 两路 DRV8701E 驱动输出(PWM + DIR)
 *
 * 硬件(逐飞 CYT4BB7 主板):
 *      左电机:  PWM -> P10_2 (TCPWM_CH30)      DIR -> P10_3
 *      右电机:  PWM -> P05_2 (TCPWM_CH11)      DIR -> P05_3
 *      频率 17 kHz, 周期的比较值满量程 PWM_DUTY_MAX = 10000
 *      DIR 约定: 高 = 正转, 低 = 反转
 *
 * 对外只有一个概念: **归一化的带符号输出 u**, 符号即方向:
 *      u = 0      -> PWM 0%(自由滑行, 不是刹车)
 *      u = +0.3   -> DIR 高, PWM 30%
 *      u = -0.3   -> DIR 低, PWM 30%
 *      |u| > 1    -> 夹到 1
 *
 * 本模块是"最后一道收口": 控制器算出来的任何值都要过 convert+写寄存器这一关,
 * 所以限幅的最终保证在这里。占空比限幅(% -> 归一化)由 spd_safety 每拍下压给控制器,
 * 这里只保证物理上不越界。
 *
 * 注意: 两路 PWM 都在 TCPWM0 的 GRP0 里, 该组的时钟分频是共享的
 *       -> 同组内所有通道必须同频(本工程两路都是 17 kHz, 没问题)。
 *       将来若要在同一组上接 50Hz 舵机, 必须实测确认, 别想当然。
 ********************************************************************************************************************/

#ifndef _MOTOR_DRV_H
#define _MOTOR_DRV_H

#include "zf_common_headfile.h"

#define MOTOR_CH_COUNT_DRV  (2)

#define MOTOR_PWM_FREQ_HZ   (17000)

#define MOTOR_L_PWM_PIN     (TCPWM_CH30_P10_2)
#define MOTOR_L_DIR_PIN     (P10_3)

#define MOTOR_R_PWM_PIN     (TCPWM_CH11_P05_2)
#define MOTOR_R_DIR_PIN     (P05_3)

/* 电机极性: >0.5 表示把该路的 u 整体取反(等价于把电机两根线对调, 二者只能选一个)。
   标定判据: 给正 u -> 电机朝"车前进"方向转, 且同侧编码器读数为正。上位机可在线改。 */
extern float mot_inv_l;
extern float mot_inv_r;

void    motor_drv_init      (void);                     /* 初始化两路 PWM/DIR, 立即输出 0 */
void    motor_drv_set       (uint8 ch, float u);        /* ch: 0=左 1=右; u 归一化带符号 */
void    motor_drv_stop_all  (void);                     /* 两路立即归零(不经过任何斜坡)   */

float   motor_drv_out       (uint8 ch);                 /* 本拍实际写下去的值(已含极性)   */
int     motor_drv_dir_level (uint8 ch);                 /* DIR 引脚当前电平 0/1           */
uint32  motor_drv_pwm_raw   (uint8 ch);                 /* 写进 PWM 的比较值              */

#endif
