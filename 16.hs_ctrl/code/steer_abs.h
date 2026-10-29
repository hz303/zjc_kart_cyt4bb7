#ifndef _STEER_ABS_H
#define _STEER_ABS_H

#include "zf_common_headfile.h"

/*---------------------------------------------------------------------------------------------------------------------
 * 转向绝对角度编码器(12 位, SPI) —— 全车唯一的转向角来源
 * 引脚: SPI_4(SCB5)  CLK P07_2 / MOSI P07_1 / MISO P07_0 / CS P07_3(软件控制)
 *       SPI_4 由 zf_driver_spi.c/h 扩展, 引脚仅 P07_0~P07_4, 不占 P17/P19
 * 协议: 读 (reg|0x40) 后发 0x00, CS 抬起 1 us 再拉低读 2 字节; 写 (reg|0x80) 后发 data
 *       角度帧 16 位, 角度 = frame >> 4, 取低 12 位, 0~4095 为一圈
 *       自检 往 1~6 号寄存器写 {0,0,0,0xC0,0xFF,0x1C}, 直到 6 号读回 0x1C
 * 标定: 左 654.5 / 中 2157 / 右 3659.3(原始读数), 三值不跨零, 中位也不贴 0
 * 量纲: get_frame 整 16 位帧(恒 0 或 0xFFFF 即 SPI 不通); get_raw 12 位计数, 未减中位
 *       get_center_delta 计数, 已减中位并解环绕; get_deg 方向盘度, 控制环使用
 * 未标定时 get_deg 不可信; 上电推方向盘核对: 最左约 +30, 最右约 -30, 相反改 DIR_SIGN = +1
 ---------------------------------------------------------------------------------------------------------------------*/

/* ---- SPI 与引脚 ---- */
#define STEER_ABS_SPI_INDEX         (SPI_4)
#define STEER_ABS_SPI_MODE          (SPI_MODE0)
/* 1 MHz: 飞线后信号质量不如原板, 读 2 字节仅 16 us, 不影响 5 ms 控制; 波形干净可上调 */
#define STEER_ABS_SPI_BAUD          (1000000)
#define STEER_ABS_SPI_SCK_PIN       (SPI4_CLK_P07_2)
#define STEER_ABS_SPI_MOSI_PIN      (SPI4_MOSI_P07_1)
#define STEER_ABS_SPI_MISO_PIN      (SPI4_MISO_P07_0)
#define STEER_ABS_SPI_HW_CS_PIN     (SPI_CS_NULL)   /* 硬件 CS 不用, 走下面这根普通 GPIO */
#define STEER_ABS_CS_GPIO_PIN       (P07_3)

#define STEER_ABS_RAW_SHIFT         (4)             /* frame >> 4 得到 12 位角度 */

/* ---- 标定 ----
 * 原始读数: 左限位 654.5 / 中位 2157 / 右限位 3659.3
 * 行程 3004.8 计数 = 左右各 30 度 -> 50.08 计数/度; 1 计数约 0.020 度 */
#define STEER_ABS_CALIBRATED        (1)             /* 1 = 使用下列标定值; 换车或重装编码器后按新实测改 raw */
#define STEER_ABS_MID_RAW           (2157)          /* 0 度(方向盘正中)对应的原始读数 */
#define STEER_ABS_LEFT_RAW          (654)           /* 左硬限位原始读数 */
#define STEER_ABS_RIGHT_RAW         (3660)          /* 右硬限位原始读数 */
#define STEER_ABS_CNT_PER_DEG       (50.08f)        /* 每 1 度方向盘对应的编码器计数 */
#define STEER_ABS_RAW_FULL          (4096)          /* 12 位一圈 */

/* 角度极性: 读数增大 = 方向盘向左, 取 -1 使 get_deg() 向右为正; 方向相反改 +1 */
#define STEER_ABS_DIR_SIGN          (-1)

/* 硬限位 度, 全系统唯一权威, 改限位只改这一个数。
 * 机械极限 ±30(读数 654 / 3660), 不许用满: 顶住即堵转满流烧驱动; 余量需大于读数波动 ±1.2 度, 故取 27。
 * 遥控与上位机指令最终都由 steer_ctrl_eff_limit() 夹到此值。 */
#define STEER_ABS_LIMIT_DEG_HARD    (27.0f)

#define STEER_ABS_SOFT_LIMIT_DEG    (STEER_ABS_LIMIT_DEG_HARD)  /* 兼容旧名, 同一数值 */
#define STEER_ABS_MECH_LIMIT_DEG    (30.0f)     /* 机械极限 度, 仅存档判读用, 禁止当限位 */

/* ---- 读数防抖 ----
 * 电机大电流干扰编码器飞线(P07_0~P07_3)时 SPI 读回乱值, 乱值落在 0~4095 内越界判据抓不到,
 * 直接进环会把方向盘顶到硬限位。本层保证单拍跳变不直接进环:
 *   孤立乱值丢弃; 连续 CONFIRM_N 拍为新值则接受并限速逼近(每拍 <= MAX_STEP_CNT);
 *   乱值持续则 rejecting 保持, 上层漏水积分 200 ms 内锁存并钉 0。被拒样计入 steer_abs_bad_cnt()。 */
#define STEER_ABS_MAX_STEP_CNT      (400)       /* 单拍最大计数变化: 400 cnt 约 8.0 度约 1600 度/秒 */
#define STEER_ABS_CONFIRM_N         (2u)        /* 超限跳变需连续这么多拍才接受 */
#define STEER_ABS_GOOD_FOR_OK       (4u)        /* 连续这么多拍干净认为链路恢复(20 ms) */

/* ---- 接口 ---- */
void    steer_abs_init          (void);
void    steer_abs_update        (void);     /* 每 5 ms 调一次(pit0_ch2 中断), 必须排在 steer_ctrl_update 之前 */

uint16  steer_abs_get_frame     (void);
uint16  steer_abs_get_raw       (void);     /* 滤波之后的原始计数, 环内实际使用 */
uint16  steer_abs_get_raw_live  (void);     /* 本拍刚读到的原始计数, 未滤波, 判链路死活 */
int16   steer_abs_get_center_delta (void);  /* 单位计数, 已解环绕, 需要原始分辨率时用 */
float   steer_abs_get_deg       (void);     /* 单位方向盘度, 向右为正, 控制环使用 */
int16   steer_abs_get_deg_x100  (void);     /* 同上放大 100 倍取整, 便于定点打印/遥测 */

int     steer_abs_self_check_ok (void);     /* 上电自检结果 0/1, 失败时角度不可信 */

uint16  steer_abs_bad_cnt       (void);     /* 累计被拒样数, 只增, 上电清零; 供 steer_ctrl 判失效与 1 Hz 打印 */
uint8   steer_abs_link_ok       (void);     /* 0 = 最近仍在丢样; 连续 GOOD_FOR_OK 拍干净才回 1 */
uint8   steer_abs_rejecting     (void);     /* 1 = 本拍该样被拒(瞬态, 供状态位) */

#endif
