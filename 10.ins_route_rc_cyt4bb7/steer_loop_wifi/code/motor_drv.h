/*********************************************************************************************************************
 * 转向位置环 + 双后轮速度环 + 遥控(WiFi/RC) - 三路 DRV8701E 驱动输出(PWM + DIR)
 *
 * ★★ 引脚按【队友 YINGYYUN/Karting-Project **最新**(commit ca55f2b "引脚对应至新版")】的整车线束表
 *    —— 这是**第二次**整体换脚。工程 7 跟的是 commit 20d0f98 那一版, 已经过期。
 *
 *      队友 Motor.h (最新):                     本工程通道            引脚
 *      ─────────────────────────────────────────────────────────────────────────────
 *      MOTOR_1 左后轮: PWM CH24_P09_0 / DIR P09_1   -> ch0 左后轮   P09_0(PWM) / P09_1(DIR)
 *      MOTOR_2 右后轮: PWM CH30_P10_2 / DIR P10_3   -> ch1 右后轮   P10_2(PWM) / P10_3(DIR)
 *      MOTOR_3 转向  : PWM CH09_P05_0 / DIR P05_1   -> ch2 转向     P05_0(PWM) / P05_1(DIR)
 *      MOTOR_4 预留  : PWM CH11_P05_2 / DIR P05_3   -> 本工程不用
 *
 *    ⚠⚠ 引脚变动史(三版, 别照抄旧文档):
 *      ① 逐飞主板默认  : 左 P05_0(PWM CH09)/P05_1(DIR); 右 P05_2(CH11)/P05_3; 转向 P10_2(CH30)/P10_3
 *      ② teammate 20d0f98 (2026-10-07, = 工程 7):
 *                       左 P05_2(CH11)/P05_3; 右 P05_1(CH10)/P05_0; 转向 P10_2(CH30)/P10_3
 *      ③ teammate ca55f2b (2026-10-10, = **本工程**):
 *                       左 P09_0(CH24)/P09_1; 右 P10_2(CH30)/P10_3; 转向 P05_0(CH09)/P05_1
 *      ②→③ 是一次"整体轮转": 新的 M1=旧的 M4、新的 M2=旧的 M3、新的 M3=旧的 M2、新的 M4=旧的 M1。
 *      **旧文档里凡是写着 P05_2/P05_3 是左轮、P10_2/P10_3 是转向的, 全部作废。**
 *
 *    ⚠ 逐飞 PWM 是**按通道号寻址**(`get_pwm_ch()` -> `TCPWM0->GRP[0].CNT[ch]`), 每个通道有两个候选引脚
 *      (CH11 = P05_2/P01_1, CH12 = P05_3/P01_0)。本工程三路占 **CH24 / CH30 / CH09**, 互不重叠;
 *      CH09/CH10/CH11 都在 P05 口, 若以后再加 PWM 务必先查通道号, **同通道两脚不能同时用且不报错**。
 *    ⚠ `P09_1` 本身也是 `TCPWM_CH25_P09_1` 的候选脚, 这里**只当普通 GPIO 用**;
 *      只要别处不再 `pwm_init(TCPWM_CH25_P09_1, ...)` 就不会打架。
 *    ⚠ `P10_3` / `P05_1` 同理, 只作 GPIO。
 *
 *      频率 17 kHz(与上一版一致, 未随队友的 10 kHz 改), 比较值满量程 PWM_DUTY_MAX = 10000
 *      DIR 约定: u >= 0 -> DIR 高; u < 0 -> DIR 低
 *
 * 对外只有一个概念: **归一化的带符号输出 u**, 符号即方向:
 *      u = 0      -> PWM 0%(自由滑行, 不是刹车)
 *      u = +0.3   -> DIR 高, PWM 30%
 *      u = -0.3   -> DIR 低, PWM 30%
 *      |u| > 1    -> 夹到 1
 *
 * 本模块是"最后一道收口": 控制器算出来的任何值都要过 convert+写寄存器这一关,
 * 所以限幅的最终保证在这里。占空比限幅(% -> 归一化)由 spd_safety / steer_ctrl 那边给,
 * 这里只保证物理上不越界。
 *
 * ⚠ PWM 时钟(逐飞 zf_driver_pwm.c 的做法, 已查证): 所有通道都挂到【同一个 16 位分频器 0】
 *   (写死 0ul, 80MHz / 10 = 8MHz), 频率 = 8MHz / clockPrescaler / period, 后两者是每通道独立的。
 *   ⇒ 同一条 TCPWM0 上跑不同频率是可行的, 但那一个分频器是共享的, 别处改了会连带影响这里。
 *   本工程三路同为 17 kHz。
 ********************************************************************************************************************/

#ifndef _MOTOR_DRV_H
#define _MOTOR_DRV_H

#include "zf_common_headfile.h"

#define MOTOR_CH_COUNT_DRV  (3)         /* 0=左后轮  1=右后轮  2=转向 */

#define MOTOR_PWM_FREQ_HZ   (17000)

/* ch0 左后轮  (2026-10-13 随队友 ca55f2b: 旧 P05_2(CH11,PWM)/P05_3(DIR) -> 现 P09_0/P09_1) */
#define MOTOR_L_PWM_PIN     (TCPWM_CH24_P09_0)
#define MOTOR_L_DIR_PIN     (P09_1)

/* ch1 右后轮  (2026-10-13 随队友 ca55f2b: 旧 P05_1(CH10,PWM)/P05_0(DIR) -> 现 P10_2/P10_3) */
#define MOTOR_R_PWM_PIN     (TCPWM_CH30_P10_2)
#define MOTOR_R_DIR_PIN     (P10_3)

/* ch2 转向: 直流电机 + 绝对编码器做位置环(不是 RC 舵机), 见 steer_ctrl.h
   (2026-10-13 随队友 ca55f2b: 旧 P10_2(CH30,PWM)/P10_3(DIR) -> 现 P05_0/P05_1) */
#define MOTOR_S_PWM_PIN     (TCPWM_CH09_P05_0)
#define MOTOR_S_DIR_PIN     (P05_1)

/* 电机极性: >0.5 表示把该路的 u 整体取反(等价于把电机两根线对调, 二者只能选一个)。
   标定判据: 给正 u -> 电机朝"车前进方向"转, 且同侧编码器读数为正。上位机可在线改。

   ★★ 2026-10-13 换脚后的"待复核"说明(换引脚 != 换电机接线, 极性必须重核):
      · 队友在 Motor_Set() 里对 **MOTOR_1(左后轮)** 做了 `duty = -duty`, M2/M3 不取反。
        本工程用 mot_inv_l/mot_inv_r/mot_inv_s 表达同一件事 ⇒ **默认值对齐队友**:
            左轮取反(1) / 右轮不取反(0)  —— 与工程 7 的实测值也一致(左1右0)。
      · ⚠ 但引脚从 P05_x 挪到 P09_x/P10_x 是**物理换线**, 上面这条只是"照队友的符号约定",
        必须按 01-上电标定卡.md 的步骤**在台架上复核一次**(判据见下)。
      · 后轮极性给反的后果只是"前进变后退/原地打转", 不烧东西;
        **转向(mot_inv_s)给反 = 位置环正反馈 = 全速顶机械限位 = 堵转烧驱动**(已烧过一个)。
        转向这条务必按 02-转向标定卡.md 单独验, 且 steer_ctrl.c 里已加"堵死/反转"主动保护。 */
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
