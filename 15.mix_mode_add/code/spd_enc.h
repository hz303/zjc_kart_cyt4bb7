/* 双电机速度环 - 增量式编码器测速(脉冲 + 方向), 双通道
 *
 *  接线: 左 PULSE P17_3 / DIR P17_4 (TC_CH58); 右 PULSE P19_2 / DIR P19_3 (TC_CH27)
 *  解码: 单路计数 + DIR 电平定符号; 每转计数 cpr = 1024 线 x 49/27 = 1858
 *  标定: 清 pos 后单方向手转电机轴整 10 圈, cpr = pos 增量 / 10 */

#ifndef _SPD_ENC_H
#define _SPD_ENC_H

#include "zf_common_headfile.h"

/* ---- 一、接线与滤波参数 ---- */
/* --- 左后轮(P17_3/P17_4, TC_CH58) --- */
#define ENC_L_INDEX         (TC_CH58_ENCODER)
#define ENC_L_PULSE_PIN     (TC_CH58_ENCODER_CH1_P17_3)
#define ENC_L_DIR_PIN       (TC_CH58_ENCODER_CH2_P17_4)
#define ENC_L_DIR_GPIO      (P17_4)             /* 同一根 DIR 线对应的 GPIO, 读电平时用 */

/* --- 右后轮(P19_2/P19_3, TC_CH27) --- */
#define ENC_R_INDEX         (TC_CH27_ENCODER)
#define ENC_R_PULSE_PIN     (TC_CH27_ENCODER_CH1_P19_2)
#define ENC_R_DIR_PIN       (TC_CH27_ENCODER_CH2_P19_3)
#define ENC_R_DIR_GPIO      (P19_3)

#define ENC_PERIOD_MS       (5)                 /* 采样周期 ms, 与控制环一致(5 ms -> 200 Hz) */
#define ENC_T_S             (0.005f)
#define ENC_WIN             (4)                 /* 滑动平均窗口默认值, 1 = 不滤波; 4 -> 等效 20 ms 均值 */
#define ENC_WIN_MAX         (16)                /* 窗口上限, 缓冲按它开 */

/* DIR 去抖: 新电平连续 ENC_DIR_DEBOUNCE 拍一致才认换向, 认之前保持旧方向;
   换向那一拍速度记 0 并计入 flip。ENC_DLT_GLITCH_MAX: 单拍计数增量超过它判为毛刺并丢弃 */
#define ENC_DIR_DEBOUNCE        (2)
#define ENC_DLT_GLITCH_MAX      (400)

/* 每电机转计数默认值, 上位机可在线改 */
#define ENC_CPR_DEFAULT     (1858.0f)

/* 测速取反默认值: 给正目标转速时该侧 rps 应为正, 读到负则置 1 */
#define ENC_INVERT_L_DEFAULT    (0.0f)
#define ENC_INVERT_R_DEFAULT    (1.0f)

/* ---- 二、一路编码器的实例(左右各一个) ---- */
/* 单拍计数增值物理上限: 4.3 rps x 1858 cpr x 5 ms 约 40, 超过视为毛刺 */
#define ENC_DELTA_MAX           (40)
typedef struct
{
    encoder_index_enum      idx;            /* 定时器通道                       */
    encoder_channel1_enum   pulse_pin;      /* 脉冲脚(配置成计数输入)           */
    encoder_channel2_enum   dir_pin;        /* 方向脚(库会配成 GPIO 输入)       */
    gpio_pin_enum           dir_gpio;       /* 方向脚对应的 GPIO 号(读电平用)   */

    volatile float  cpr;                    /* 每转计数(可在线改)               */
    volatile float  invert;                 /* >0.5 = 测速取反(可在线改)        */

    volatile int16  last_raw;               /* 上次原始计数(未加符号)           */
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

/* 测速滤波窗口长度, 1~ENC_WIN_MAX, 两路共用; 等效延迟 (win_n-1)/2*5 ms, 噪声按 1/sqrt(win_n) 降 */
extern volatile float spd_enc_win;

/* ---- 三、接口 ---- */
void    spd_enc_init        (void);                 /* 两路一起初始化, 并预读初值 */
void    spd_enc_update      (spd_enc_t *e);         /* 每 5 ms 调一次(PIT 中断内) */
void    spd_enc_reset_pos   (spd_enc_t *e);         /* 清累计计数(标定 cpr 前用)  */

#endif
