/*********************************************************************************************************************
 * 双电机速度环(WiFi) - 增量式编码器测速实现
 * 接口与设计说明见 spd_enc.h
 ********************************************************************************************************************/

#include "spd_enc.h"

/*===================================================================================================================
 *  两个实例(上电初值; cpr / invert 之后可由上位机在线改)
 *=================================================================================================================*/
/* 测速滤波窗口(在线可改, 见 spd_enc.h) */
volatile float spd_enc_win = (float)ENC_WIN;

spd_enc_t enc_l =
{
    ENC_L_INDEX, ENC_L_PULSE_PIN, ENC_L_DIR_PIN, ENC_L_DIR_GPIO,
    ENC_CPR_DEFAULT, 0.0f,
    0, 0, 0, 0, 0, 0, 0.0f, 0.0f, 0, 0, 0,
    0, {0}, 0, 0
};

spd_enc_t enc_r =
{
    ENC_R_INDEX, ENC_R_PULSE_PIN, ENC_R_DIR_PIN, ENC_R_DIR_GPIO,
    ENC_CPR_DEFAULT, 0.0f,
    0, 0, 0, 0, 0, 0, 0.0f, 0.0f, 0, 0, 0,
    0, {0}, 0, 0
};

/*-------------------------------------------------------------------------------------------------------------------
 *  一路初始化
 *-----------------------------------------------------------------------------------------------------------------*/
static void spd_enc_init_one (spd_enc_t *e)
{
    uint8 i;

    encoder_dir_init(e->idx, e->pulse_pin, e->dir_pin);

    /* 记录初值: 否则第一拍的 "raw - 0" 会变成一个巨大的假增量, 直接冲进速度环。
       ★ 预读必须用【原始计数】口径(见 spd_enc_update 第 1 步), 与后续算法一致:
         库的 encoder_get_count() 返回 ±CNT, 按我们自己读到的 DIR 反推即可去掉符号。 */
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
        spd_enc_win = (float)ENC_WIN;                           /* 上电夹一次 */
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

/*-------------------------------------------------------------------------------------------------------------------
 *  一路 5ms 更新
 *-----------------------------------------------------------------------------------------------------------------*/
void spd_enc_update (spd_enc_t *e)
{
    int16 c;
    int16 raw;
    int16 d_raw;
    uint8 d;
    int16 delta;
    float cpr;

    c = encoder_get_count(e->idx);              /* 库内部按它读到的 DIR 定符号 */
    d = (uint8)gpio_get_level(e->dir_gpio);     /* 我们紧随其后再读一次        */

    /* ---- 1) 还原【原始计数】(去掉库加的符号) ----
       库返回的 ±CNT 里, 符号只取决于 DIR, 与计数本身无关,
       所以按我们自己读到的 DIR 反推, 就能把原始计数拿回来。 */
    raw = (int16)((0 != d) ? c : (int16)(-c));

    /* ---- 2) 原始计数的单拍增量 + 毛刺门限 ---- */
    d_raw = (int16)(raw - e->last_raw);
    if ((d_raw > ENC_DLT_GLITCH_MAX) || (d_raw < -ENC_DLT_GLITCH_MAX))
    {
        d_raw = 0;
        e->glitch++;
    }
    e->last_raw = raw;

    /* ---- 3) DIR 去抖 (见 spd_enc.h 的 ENC_DIR_DEBOUNCE 注解) ---- */
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

    /* ---- 4) 用【去抖后的方向】给原始增量定符号 ----
       注意这里【不再丢整拍】: 毛刺只是被忽略, 那几拍的计数照样按旧方向计入。 */
    delta = (int16)((0 != e->last_dir) ? d_raw : (int16)(-d_raw));

    if (e->invert > 0.5f)
    {
        delta = (int16)(-delta);
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

    /* 滑动平均(窗口长度在线可调)。上电后前 win_n 拍是"窗口填充"过程,
       读出来的速度只有真值的一部分 -- 此时速度环还没使能, 无影响。 */
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
