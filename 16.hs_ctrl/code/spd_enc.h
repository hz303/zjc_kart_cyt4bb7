/*********************************************************************************************************************
 * 双电机速度环(WiFi) - 增量式编码器测速(脉冲 + 方向), 双通道
 * 5ms 定周期把脉冲/方向式编码器换算成转速 rps, 左右各一个实例。接线与计数如下:
 *      左后轮:  PULSE -> P17_3 (TC_CH58 的 CH1)     DIR -> P17_4 (TC_CH58 的 CH2)
 *      右后轮:  PULSE -> P19_2 (TC_CH27 的 CH1)     DIR -> P19_3 (TC_CH27 的 CH2)
 *      GND -> GND, 电源按模块丝印; 解码用 encoder_dir_init() 脉冲 + 方向, 不用正交
 * ---- 计数与取符号 ----
 *  1024 线(只数上升沿)、电机轴 49 齿带 27 齿(增速 49/27)、轮子与电机同轴:
 *      cpr = 1024 * 49/27 = 1858 计数/转; 标定: 清 pos, 手转电机轴整 10 圈, cpr = pos/10
 *  库按每条脉冲 +1 计数, 方向取自 encoder_get_count() 读的那一拍 DIR 电平;
 *  采样间换向会使差分出错, 故检出换向时本拍速度记 0 并计 flip, 代价丢 1 拍(5ms)。
 ********************************************************************************************************************/

#ifndef _SPD_ENC_H
#define _SPD_ENC_H

#include "zf_common_headfile.h"

/* ---- 一、接线与滤波参数 ---- */
#define ENC_L_INDEX         (TC_CH58_ENCODER)
#define ENC_L_PULSE_PIN     (TC_CH58_ENCODER_CH1_P17_3)
#define ENC_L_DIR_PIN       (TC_CH58_ENCODER_CH2_P17_4)
#define ENC_L_DIR_GPIO      (P17_4)             /* 同一根 DIR 线对应的 GPIO, 用来直接读电平 */

#define ENC_R_INDEX         (TC_CH27_ENCODER)
#define ENC_R_PULSE_PIN     (TC_CH27_ENCODER_CH1_P19_2)
#define ENC_R_DIR_PIN       (TC_CH27_ENCODER_CH2_P19_3)
#define ENC_R_DIR_GPIO      (P19_3)             /* 同一根 DIR 线对应的 GPIO */

#define ENC_PERIOD_MS       (5)                 /* 采样周期, 必须与控制环一致(5ms -> 200Hz) */
#define ENC_T_S             (0.005f)
#define ENC_WIN             (4)                 /* 滑动平均窗口默认值(1 = 不滤波), 4 -> 等效 20ms 均值 */
#define ENC_WIN_MAX         (16)                /* 窗口上限, 缓冲按它开; 实际长度由 spd_enc_win 决定 */

/* ENC_DIR_DEBOUNCE: 新电平连续 N 拍一致才认换向, 之前维持旧方向; 不丢拍, 晚 2 拍(10ms)。
   ENC_DLT_GLITCH_MAX: 单拍增量超它即判毛刺并丢该拍; 400 cnt/5ms = 43 rps, 不误杀。 */
#define ENC_DIR_DEBOUNCE        (2)
#define ENC_DLT_GLITCH_MAX      (400)

#define ENC_CPR_DEFAULT     (1858.0f)      /* 每转计数 cpr 默认值, 上位机可在线改 */

/* 测速取反: 给正目标转速时该侧 rps 应为正, 为负才取反; 车端 SAVE 不写 Flash, 结果须回填此处 */
#define ENC_INVERT_L_DEFAULT    (0.0f)
#define ENC_INVERT_R_DEFAULT    (1.0f)

/* ---- 二、一路编码器的实例(左右各一个) ---- */
#define ENC_DELTA_MAX           (40)        /* 单拍计数上限: 4.3 rps * 1858 cpr * 5 ms = 40, 超出即毛刺 */
typedef struct
{
    encoder_index_enum      idx;            /* 定时器通道                       */
    encoder_channel1_enum   pulse_pin;
    encoder_channel2_enum   dir_pin;
    gpio_pin_enum           dir_gpio;       /* 方向脚对应的 GPIO 号(读电平用)   */

    volatile float  cpr;                    /* 每转计数(可在线改)               */
    volatile float  invert;                 /* >0.5 = 测速取反(可在线改)        */

    volatile int16  last_raw;               /* 上次的原始计数(未加符号)         */
    volatile uint8  last_dir;               /* 去抖之后的 DIR 电平              */
    volatile uint8  dir_pend;               /* 待确认的新 DIR 电平              */
    volatile uint8  dir_pend_cnt;           /* 新电平已连续出现几拍             */
    volatile int16  delta;                  /* 本拍计数增量(带符号)             */
    volatile int32  pos;                    /* 累计计数(带符号)                 */
    volatile float  raw_rps;                /* 本拍单点转速(未滤波)             */
    volatile float  rps;                    /* 滤波后转速 rps                   */
    volatile uint32 flip;                   /* 真换向次数(去抖确认后)           */
    volatile uint32 glitch;                 /* 读数毛刺丢弃次数                 */
    volatile uint32 samples;                /* 总采样拍数(应匀速增长)           */

    uint8           win_n_used;             /* 上次实际用的窗口长度(变了就重建缓冲) */

    int16           win[ENC_WIN_MAX];       /* 滑动平均缓冲(按上限开)            */
    uint32          win_cnt;
    int32           win_sum;
} spd_enc_t;

extern spd_enc_t enc_l;
extern spd_enc_t enc_r;

/* 测速滤波窗口长度(在线可调, 1~ENC_WIN_MAX), 两路共用; 等效延迟 = (win_n-1)/2 * 5ms。
   噪声按 1/sqrt(win_n) 降; 窗口加长后 k_v 要相应调小, 否则会振。 */
extern volatile float spd_enc_win;

/* ---- 三、接口 ---- */
void    spd_enc_init        (void);                 /* 两路一起初始化, 并预读初值 */
void    spd_enc_update      (spd_enc_t *e);         /* 5ms 调一次(放 PIT 中断里)  */
void    spd_enc_reset_pos   (spd_enc_t *e);         /* 清累计计数(标定 cpr 前用)  */

#endif
