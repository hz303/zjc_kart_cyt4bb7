/**********************************************************************************************************************
 * 双电机速度环(WiFi) - 增量式编码器测速实现: 5ms 取原始计数, 差分后换算成 rps
 *********************************************************************************************************************/

#include "spd_enc.h"

/* ---- 两个实例: 上电初值, cpr / invert 之后可由上位机在线改 ---- */
volatile float spd_enc_win = (float)ENC_WIN;

/* 用指定初始化: 未列出的成员为 0, 运行时状态由 spd_enc_init_one() 统一清 */
spd_enc_t enc_l =
{
    .idx       = ENC_L_INDEX,
    .pulse_pin = ENC_L_PULSE_PIN,
    .dir_pin   = ENC_L_DIR_PIN,
    .dir_gpio  = ENC_L_DIR_GPIO,
    .cpr       = ENC_CPR_DEFAULT,
    .invert    = ENC_INVERT_L_DEFAULT,
};

spd_enc_t enc_r =
{
    .idx       = ENC_R_INDEX,
    .pulse_pin = ENC_R_PULSE_PIN,
    .dir_pin   = ENC_R_DIR_PIN,
    .dir_gpio  = ENC_R_DIR_GPIO,
    .cpr       = ENC_CPR_DEFAULT,
    .invert    = ENC_INVERT_R_DEFAULT,
};

/* ---- 一路初始化 ---- */
static void spd_enc_init_one (spd_enc_t *e)
{
    uint8 i;

    encoder_dir_init(e->idx, e->pulse_pin, e->dir_pin);

    /* 记录初值, 否则第一拍 raw - 0 是巨大假增量; 预读按原始计数口径, 与 spd_enc_update 第 1 步一致 */
    e->last_dir = (uint8)gpio_get_level(e->dir_gpio);
    {
        int16 c0 = encoder_get_count(e->idx);
        e->last_raw = (0 != e->last_dir) ? c0 : (int16)(-c0);
    }
    e->dir_pend     = e->last_dir;
    e->dir_pend_cnt = 0;
    e->glitch       = 0;

    if ((spd_enc_win < 1.0f) || (spd_enc_win > (float)ENC_WIN_MAX))
    {
        spd_enc_win = (float)ENC_WIN;
    }

    for (i = 0; i < ENC_WIN_MAX; i++)
    {
        e->win[i] = 0;
    }
    e->win_n_used = 0;                                          /* 下一拍按 spd_enc_win 重建 */
    e->win_cnt = 0;
    e->win_sum = 0;
    e->delta   = 0;
    e->pos     = 0;
    e->raw_rps = 0.0f;
    e->rps     = 0.0f;
    e->flip    = 0;
    e->samples = 0;
}

void spd_enc_init (void)
{
    spd_enc_init_one(&enc_l);
    spd_enc_init_one(&enc_r);
}

/* ---- 一路 5ms 更新 ---- */
void spd_enc_update (spd_enc_t *e)
{
    int16 c;
    int16 raw;
    int16 d_raw;
    uint8 d;
    int16 delta;
    float cpr;

    c = encoder_get_count(e->idx);              /* 库内部按它读到的 DIR 定符号 */
    d = (uint8)gpio_get_level(e->dir_gpio);

        /* 1) 还原原始计数: 库返回的 ±CNT 符号只取决于 DIR, 按本拍 DIR 反推即可去符号 */
    raw = (int16)((0 != d) ? c : (int16)(-c));

    /* ---- 2) 原始计数的单拍增量 + 毛刺门限 ---- */
    d_raw = (int16)(raw - e->last_raw);
    if ((d_raw > ENC_DLT_GLITCH_MAX) || (d_raw < -ENC_DLT_GLITCH_MAX))
    {
        d_raw = 0;
        e->glitch++;
    }
    e->last_raw = raw;

    /* ---- 3) DIR 去抖, 门限见 spd_enc.h ---- */
    if (d == e->last_dir)
    {
        e->dir_pend     = d;
        e->dir_pend_cnt = 0;
    }
    else
    {
        if (d == e->dir_pend)
        {
            if (e->dir_pend_cnt < 250u) { e->dir_pend_cnt++; }
        }
        else
        {
            e->dir_pend     = d;
            e->dir_pend_cnt = 1;
        }

        if ((uint16)e->dir_pend_cnt >= (uint16)ENC_DIR_DEBOUNCE)
        {
            e->last_dir     = d;                /* 真换向: 之后按新方向定符号 */
            e->dir_pend_cnt = 0;
            e->flip++;
        }
    }

        /* 4) 用去抖后的方向给原始增量定符号; 毛刺拍不丢, 计数照旧按旧方向计入 */
    delta = (int16)((0 != e->last_dir) ? d_raw : (int16)(-d_raw));

    if (e->invert > 0.5f)
    {
        delta = (int16)(-delta);
    }

        /* 单拍计数上限 40 = 4.3 rps * 1858 cpr * 5 ms, 超出即毛刺: 丢弃本拍、不累加位置 */
    if ((delta > ENC_DELTA_MAX) || (delta < -ENC_DELTA_MAX))
    {
        delta = 0;
    }

    e->delta = delta;
    e->pos  += delta;
    e->samples++;

    cpr = e->cpr;
    if (cpr < 1.0f)
    {
        cpr = 1.0f;                                 /* 防除零: 宁可刻度错也不许崩 */
    }

    e->raw_rps = (float)delta / (cpr * ENC_T_S);

        /* 滑动平均: 上电后前 win_n 拍是窗口填充, 读到偏小; 此时速度环还没使能 */
    {
        uint32 n = (uint32)spd_enc_win;

        if (n < 1u)          { n = 1u; }
        if (n > ENC_WIN_MAX) { n = ENC_WIN_MAX; }

        /* 窗口长度变了 -> 把缓冲整体预填成当前 delta, 避免旧残留造成一次假跳变 */
        if ((uint8)n != e->win_n_used)
        {
            uint32 k;
            for (k = 0; k < ENC_WIN_MAX; k ++) { e->win[k] = delta; }
            e->win_sum    = (int32)delta * (int32)n;
            e->win_cnt    = 0;
            e->win_n_used = (uint8)n;
        }

        e->win_sum += (int32)delta - (int32)e->win[e->win_cnt];
        e->win[e->win_cnt] = delta;
        e->win_cnt++;
        if (e->win_cnt >= n)
        {
            e->win_cnt = 0;
        }

        e->rps = (float)e->win_sum / (cpr * ENC_T_S * (float)n);
    }
}

void spd_enc_reset_pos (spd_enc_t *e)
{
    e->pos = 0;
}
