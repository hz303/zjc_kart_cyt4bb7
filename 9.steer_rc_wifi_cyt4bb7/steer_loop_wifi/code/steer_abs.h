#ifndef _STEER_ABS_H
#define _STEER_ABS_H

#include "zf_common_headfile.h"

/* 转向绝对角度编码器(12 位, SPI) —— 全车【唯一】的转向角来源。
 * 它自带 SPI 裸读写 + 上电自检 + 过零环绕处理, 不依赖逐飞的 zf_device_absolute_encoder。
 * 两者协议相同, 换成库版本只是少维护一份代码, 行为不变。)
 * SPI_4 (SCB5):  CLK P07_2 / MOSI P07_1 / MISO P07_0 / CS P07_3(软件控制)
 * 读寄存器: 先发 (reg|0x40), 再发 0x00; CS 抬起延时 1us; 再拉低读 2 字节
 * 写寄存器: 先发 (reg|0x80), 再发 data
 * 角度帧  : 16 位, 角度 = frame >> 4, 取低 12 位 -> 0~4095(一圈)
 * 自检    : 往 1~6 号寄存器写 {0,0,0,0xC0,0xFF,0x1C}, 直到 6 号读回 0x1C
 * 左限位 654.5 / 中位 2157 / 右限位 3659.3  (15 次满幅摆动的统计均值)
 * get_frame()         SPI 读回的整 16 位帧; 查 SPI 通不通看这个 —— 恒 0 或恒 0xFFFF 就是没通
 * get_raw()           12 位原始计数 0~4095, 没减中位
 * get_center_delta()  已减中位、已解 4096 环绕, 单位【计数】
 * 推到最左, get_deg() 应读到 +30 附近; 推到最右, 应读到 -30 附近。
 * 若符号相反, 把 STEER_ABS_DIR_SIGN 改成 +1。
 */

#define STEER_ABS_SPI_INDEX         (SPI_4)
#define STEER_ABS_SPI_MODE          (SPI_MODE0)
/* 1 MHz: 飞线后信号质量不如原板, 先保守; 读一次 2 字节也只要 16us, 不影响 5ms 控制。 */
#define STEER_ABS_SPI_BAUD          (1000000)
#define STEER_ABS_SPI_SCK_PIN       (SPI4_CLK_P07_2)
#define STEER_ABS_SPI_MOSI_PIN      (SPI4_MOSI_P07_1)
#define STEER_ABS_SPI_MISO_PIN      (SPI4_MISO_P07_0)
#define STEER_ABS_SPI_HW_CS_PIN     (SPI_CS_NULL)   /* 硬件 CS 不用, 走下面这根普通 GPIO */
#define STEER_ABS_CS_GPIO_PIN       (P07_3)

#define STEER_ABS_RAW_SHIFT         (4)             /* frame >> 4 得到 12 位角度 */

/* 原始读数(未减中位):  左限位 654.5   中位 2157   右限位 3659.3
 * 行程 3004.8 计数 = 方向盘左右各 30 度  =>  50.08 计数/度
 * 分辨率: 1 计数 ≈ 0.020 度
 */
#define STEER_ABS_CALIBRATED        (1)             /* 1 = 用上面的实测值; 换车/重装编码器后按新实测改这三个 raw */
#define STEER_ABS_MID_RAW           (2157)          /* 0 度(方向盘正中)对应的原始读数 */
#define STEER_ABS_LEFT_RAW          (654)           /* 左硬限位原始读数 */
#define STEER_ABS_RIGHT_RAW         (3660)          /* 右硬限位原始读数 */
#define STEER_ABS_CNT_PER_DEG       (50.08f)        /* 每 1 度方向盘对应的编码器计数 */
#define STEER_ABS_RAW_FULL          (4096)          /* 12 位一圈 */

/* 实测方向相反就改 +1。 */
#define STEER_ABS_DIR_SIGN          (-1)

/* 机械物理极限是 ±30（读数 654 / 3660，行程 3004.8 计数），但**一律不许用满**：
 * ② 留的余量要大于限位读数的重复性波动（实测 ±1.2 度）。
 * 落地点是 `steer_ctrl.c` 的 `steer_eff_limit()`：它同时夹"参数软限位"和"目标角"。
 * 也就是说上位机就算把「转向|软限位」拖到 30，实际生效的仍然是 27。
 */
#define STEER_ABS_LIMIT_DEG_HARD    (27.0f)

/* 兼容旧名（同一个数；新代码请用 STEER_ABS_LIMIT_DEG_HARD） */
#define STEER_ABS_SOFT_LIMIT_DEG    (STEER_ABS_LIMIT_DEG_HARD)
#define STEER_ABS_MECH_LIMIT_DEG    (30.0f)     /* 机械极限，仅存档/判读用，禁止当限位使 */

/* ============================== 接口 ============================== */
void    steer_abs_init          (void);
void    steer_abs_update        (void);     /* 每 5ms 调一次(在 pit0_ch2 中断里), 必须排在 steer_ctrl_update 之前 */

uint16  steer_abs_get_frame     (void);
uint16  steer_abs_get_raw       (void);
int16   steer_abs_get_center_delta (void);  /* 单位: 计数(已解环绕) —— 只在需要原始分辨率时用 */
float   steer_abs_get_deg       (void);     /* 单位: 方向盘度(向右为正) —— 控制环用这个 */
int16   steer_abs_get_deg_x100  (void);     /* 同上, 放大 100 倍取整, 方便定点打印/遥测 */

int     steer_abs_self_check_ok (void);     /* 上电自检是否通过(0/1), 自检失败时角度不可信 */

#endif
