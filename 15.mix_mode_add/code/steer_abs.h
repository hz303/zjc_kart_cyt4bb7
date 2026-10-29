#ifndef _STEER_ABS_H
#define _STEER_ABS_H

#include "zf_common_headfile.h"

/*********************************************************************************************************************
 * 转向绝对角度编码器(12 位, SPI) —— 全车唯一的转向角来源。
 * 引脚: SPI_4(SCB5) CLK P07_2 / MOSI P07_1 / MISO P07_0 / CS P07_3(软件片选, 硬件 CS 不用)
 * 协议: 读 = 发 (reg|0x40) 再发 0x00, CS 抬起延时 1us 后读 2 字节; 写 = 发 (reg|0x80) 再发 data
 * 自检: 往 1~6 号寄存器写 {0,0,0,0xC0,0xFF,0x1C}, 直到 6 号读回 0x1C
 ********************************************************************************************************************/

/* ---- 量纲(别混): get_frame 整 16 位帧, 判 SPI 通不通 看它(恒 0 / 恒 0xFFFF = 没通);
 *      get_raw 12 位计数 0~4095; get_center_delta 计数(已减中位、解环绕); get_deg 方向盘度, 控制环只用它
 * 未标定也能出数但角度错: 最左应读 +30、最右 -30, 符号相反则改 STEER_ABS_DIR_SIGN。 */

/* ---- SPI 与引脚 ---- */
#define STEER_ABS_SPI_INDEX         (SPI_4)
#define STEER_ABS_SPI_MODE          (SPI_MODE0)
/* 1 MHz: 飞线信号质量保守取值, 读一次 2 字节仅 16us, 不影响 5ms 控制; 波形干净后可提频 */
#define STEER_ABS_SPI_BAUD          (1000000)
#define STEER_ABS_SPI_SCK_PIN       (SPI4_CLK_P07_2)
#define STEER_ABS_SPI_MOSI_PIN      (SPI4_MOSI_P07_1)
#define STEER_ABS_SPI_MISO_PIN      (SPI4_MISO_P07_0)
#define STEER_ABS_SPI_HW_CS_PIN     (SPI_CS_NULL)   /* 硬件 CS 不用, 走下面这根普通 GPIO */
#define STEER_ABS_CS_GPIO_PIN       (P07_3)

#define STEER_ABS_RAW_SHIFT         (4)             /* frame >> 4 得到 12 位角度 */

/* ---- 标定(原始读数, 未减中位): 左 654.5 / 中 2157 / 右 3659.3 ----
 *   行程 3004.8 计数 = 左右各 30 度 => 50.08 计数/度, 1 计数 = 0.020 度 */
#define STEER_ABS_CALIBRATED        (1)             /* 1 = 用本文件的值; 换车/重装编码器后按新实测改这三个 raw */
#define STEER_ABS_MID_RAW           (2157)          /* 0 度(方向盘正中)对应的原始读数 */
#define STEER_ABS_LEFT_RAW          (654)           /* 左硬限位原始读数 */
#define STEER_ABS_RIGHT_RAW         (3660)          /* 右硬限位原始读数 */
#define STEER_ABS_CNT_PER_DEG       (50.08f)        /* 每 1 度方向盘对应的编码器计数 */
#define STEER_ABS_RAW_FULL          (4096)          /* 12 位一圈 */

/* 角度极性: 读数增大 = 方向盘向左, 取 -1 使 get_deg() 向右为正; 方向相反则改 +1 */
#define STEER_ABS_DIR_SIGN          (-1)

/* ---- 硬限位 度, 全系统唯一定值来源: 改限位只改这一个数 ----
 * 机械极限 ±30 一律不许用满(顶到即堵转满流), 故取 左右各 27; 遥控与上位机路径最终都由 steer_ctrl_eff_limit() 夹到这里。 */
#define STEER_ABS_LIMIT_DEG_HARD    (27.0f)

#define STEER_ABS_SOFT_LIMIT_DEG    (STEER_ABS_LIMIT_DEG_HARD)
#define STEER_ABS_MECH_LIMIT_DEG    (30.0f)     /* 机械极限, 仅存档/判读, 禁止当限位使 */

/* ---- 读数防抖: 飞线受电机干扰, 单拍乱值可能落在 0~4095 内, 故任何 单拍跳变 都不直接进环 ----
 *   ① 孤立乱值 本拍屏蔽, 环里保持上一个好值; ② 连续 CONFIRM_N 拍判为真动作, 按 MAX_STEP_CNT 限速逼近;
 *   ③ 乱值持续 -> rejecting 常亮 -> 上层漏水积分 200ms 内锁存并钉零输出; MAX_STEP_CNT 须高于机械最大角速度(满舵约 100~300 度/秒)。 */
#define STEER_ABS_MAX_STEP_CNT      (400)       /* 单拍(5ms)允许的最大原始计数变化, 400 cnt ≈ 8.0 度 ≈ 1600 度/秒 */
#define STEER_ABS_CONFIRM_N         (2u)        /* 超限跳变需连续出现这么多拍才接受 */
#define STEER_ABS_GOOD_FOR_OK       (4u)        /* 连续这么多拍干净 => 认为链路恢复(20ms) */

/* ---- 接口 ---- */
void    steer_abs_init          (void);
void    steer_abs_update        (void);     /* 每 5ms 一次(pit0_ch2 中断), 必须排在 steer_ctrl_update 之前 */

uint16  steer_abs_get_frame     (void);
uint16  steer_abs_get_raw       (void);     /* 滤波后 的原始计数, 即环里真正用的那个 */
uint16  steer_abs_get_raw_live  (void);     /* 本拍刚读到的原始计数, 未滤波, 判链路死活看它 */
int16   steer_abs_get_center_delta (void);  /* 单位计数(已解环绕), 只在需要原始分辨率时用 */
float   steer_abs_get_deg       (void);     /* 单位方向盘度(向右为正), 控制环用这个 */
int16   steer_abs_get_deg_x100  (void);     /* 同上放大 100 倍取整, 供定点打印/遥测 */

int     steer_abs_self_check_ok (void);     /* 上电自检结果(0/1), 失败时角度不可信 */

/* 防抖统计, 供 steer_ctrl 判失效与 1Hz 打印/遥测 */
uint16  steer_abs_bad_cnt       (void);     /* 累计被拒样数, 只增, 上电清零 */
uint8   steer_abs_link_ok       (void);     /* 1 = 链路正常; 连续 GOOD_FOR_OK 拍干净才回到 1 */
uint8   steer_abs_rejecting     (void);     /* 1 = 本拍这条样被拒(瞬态, 供状态位) */

#endif
