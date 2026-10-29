/*********************************************************************************************************************
 * 转向绝对角度编码器(SPI) 实现 —— 协议、量纲、标定见 steer_abs.h
 *********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "steer_abs.h"

#define STEER_ABS_SPI_W             (0x80)
#define STEER_ABS_SPI_R             (0x40)
#define STEER_ABS_ZERO_L_REG        (0x00)
#define STEER_ABS_ZERO_H_REG        (0x01)
#define STEER_ABS_DIR_REG           (0x09)
#define STEER_ABS_STATUS_REG        (0x06)
#define STEER_ABS_TIMEOUT_COUNT     (100)

/* 初值取中位: 未读到有效帧前视为居中(0 度) */
static uint16   s_raw           = (uint16)STEER_ABS_MID_RAW;  /* 环内使用的值, 防抖之后 */
static uint16   s_raw_live      = 0;        /* 本拍原始计数, 未防抖, 判链路死活 */
static uint16   s_frame         = 0;
static int16    s_center_delta  = 0;
static float    s_deg           = 0.0f;
static int16    s_deg_x100      = 0;
static uint8    s_self_ok       = 0;

static uint16   s_bad_cnt       = 0;        /* 累计被拒样数, 只增 */
static uint8    s_good_run      = 0;
static uint8    s_link_ok       = 1;        /* 1 = 链路正常 */
static uint8    s_rejecting     = 0;        /* 1 = 本拍被拒 */
static uint16   s_pend_raw      = 0;        /* 存疑值及其连续出现次数 */
static uint8    s_pend_n        = 0;
static uint8    s_first         = 1;        /* 上电第一条样无条件接受 */

/* raw - center 折回 ±2048: 编码器为环形量, 机构装偏跨零时不折会读出约 ±4000 的假打角; 2048/4096 由 12 位宽决定 */
static int16 steer_abs_wrap_delta (uint16 raw, uint16 center)
{
    int16 delta = (int16)raw - (int16)center;

    if (delta > 2048)
    {
        delta -= 4096;
    }
    else if (delta < -2048)
    {
        delta += 4096;
    }

    return delta;
}

static void steer_abs_cs (uint8 state)
{
    if (state)
    {
        gpio_high(STEER_ABS_CS_GPIO_PIN);
    }
    else
    {
        gpio_low(STEER_ABS_CS_GPIO_PIN);
    }
}

static void steer_abs_write_register (uint8 reg, uint8 data)
{
    steer_abs_cs(0);
    spi_write_8bit(STEER_ABS_SPI_INDEX, (uint8)(reg | STEER_ABS_SPI_W));
    spi_write_8bit(STEER_ABS_SPI_INDEX, data);
    steer_abs_cs(1);
    system_delay_us(1);

    steer_abs_cs(0);
    spi_read_8bit(STEER_ABS_SPI_INDEX);         /* 回读两字节并丢弃 */
    spi_read_8bit(STEER_ABS_SPI_INDEX);
    steer_abs_cs(1);
}

static uint8 steer_abs_read_register (uint8 reg)
{
    uint8 data;

    steer_abs_cs(0);
    spi_write_8bit(STEER_ABS_SPI_INDEX, (uint8)(reg | STEER_ABS_SPI_R));
    spi_write_8bit(STEER_ABS_SPI_INDEX, 0x00);
    steer_abs_cs(1);
    system_delay_us(1);

    steer_abs_cs(0);
    data = spi_read_8bit(STEER_ABS_SPI_INDEX);
    spi_read_8bit(STEER_ABS_SPI_INDEX);
    steer_abs_cs(1);

    return data;
}

/* 上电自检: 反复写 6 个配置寄存器直到状态寄存器读回 0x1C, 超 100 轮判失败。
 * 自检失败不代表 SPI 断线(主机自产时钟, 从机不在也会完成传输), 判 SPI 死活看 get_frame()。 */
static uint8 steer_abs_self_check (void)
{
    uint8           i;
    uint16          time_count = 0;
    const uint8     dat[6] = {0, 0, 0, 0xC0, 0xFF, 0x1C};

    while (0x1C != steer_abs_read_register(STEER_ABS_STATUS_REG))
    {
        for (i = 0; i < 6; i ++)
        {
            steer_abs_write_register((uint8)(i + 1), dat[i]);
            system_delay_ms(1);
        }

        if (STEER_ABS_TIMEOUT_COUNT < time_count ++)
        {
            return 1;                           /* 1 = 失败 */
        }
    }

    return 0;
}

static uint16 steer_abs_read_frame (void)
{
    uint16 data;

    steer_abs_cs(0);
    data   = ((uint16)spi_read_8bit(STEER_ABS_SPI_INDEX) & 0x00FF) << 8;
    data  |= spi_read_8bit(STEER_ABS_SPI_INDEX);
    steer_abs_cs(1);

    return data;
}

void steer_abs_init (void)
{
    gpio_init(STEER_ABS_CS_GPIO_PIN, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    spi_init(STEER_ABS_SPI_INDEX,
             STEER_ABS_SPI_MODE,
             STEER_ABS_SPI_BAUD,
             STEER_ABS_SPI_SCK_PIN,
             STEER_ABS_SPI_MOSI_PIN,
             STEER_ABS_SPI_MISO_PIN,
             STEER_ABS_SPI_HW_CS_PIN);

    s_self_ok = (uint8)(0 == steer_abs_self_check());

    if (s_self_ok)
    {
        /* 芯片零位写 0, 中位由软件 STEER_ABS_MID_RAW 提供, 重标只改一个宏 */
        steer_abs_write_register(STEER_ABS_DIR_REG,    0x00);
        steer_abs_write_register(STEER_ABS_ZERO_L_REG, 0x00);
        steer_abs_write_register(STEER_ABS_ZERO_H_REG, 0x00);
    }

    steer_abs_update();
}

/* 由 s_raw 派生 center_delta / deg / deg_x100, 公式只在此一处 */
static void steer_abs_derive (void)
{
    s_center_delta = steer_abs_wrap_delta(s_raw, (uint16)STEER_ABS_MID_RAW);

    /* 方向盘度: 用已解环绕的 center_delta 折算再乘极性, 跨零那一拍才不出错 */
    s_deg      = (float)STEER_ABS_DIR_SIGN * ((float)s_center_delta / STEER_ABS_CNT_PER_DEG);
    s_deg_x100 = (int16)(s_deg * 100.0f);
}

void steer_abs_update (void)
{
    uint16 raw_new;
    int16  step;

    s_frame    = steer_abs_read_frame();
    raw_new    = (uint16)((s_frame >> STEER_ABS_RAW_SHIFT) & 0x0FFF);
    s_raw_live = raw_new;

    /* 上电第一条样无条件接受 */
    if (0 != s_first)
    {
        s_first     = 0u;
        s_rejecting = 0u;
    }
    else
    {
        /* 单拍跳变合理性判定, 差值按 12 位环绕差计算 */
        step = (int16)raw_new - (int16)s_raw;
        if (step >  2048) { step -= 4096; }
        if (step < -2048) { step += 4096; }

        if ((step > STEER_ABS_MAX_STEP_CNT) || (step < -STEER_ABS_MAX_STEP_CNT))
        {
            if (raw_new == s_pend_raw) { s_pend_n ++; }
            else                       { s_pend_raw = raw_new; s_pend_n = 1u; }

            s_bad_cnt ++;
            s_good_run  = 0u;
            s_link_ok   = 0u;
            s_rejecting = 1u;

            if (s_pend_n < STEER_ABS_CONFIRM_N)
            {
                /* 孤立乱值: 本拍不动, 环内保持上一个好值 */
                steer_abs_derive();
                return;
            }

            /* 新值已连续 CONFIRM_N 拍, 可能是真动作, 但仍按 MAX_STEP_CNT 限速逼近。
             * 真摆动几拍内跟上; 持续乱值则 rejecting 保持, 上层漏水积分 200 ms 内锁存并钉 0。
             * 任何情况下环内单拍角变化都不超过 MAX_STEP_CNT。 */
            if (step > 0)
            {
                s_raw = (uint16)((int16)s_raw + STEER_ABS_MAX_STEP_CNT);
            }
            else
            {
                s_raw = (uint16)((int16)s_raw - STEER_ABS_MAX_STEP_CNT);
            }
            steer_abs_derive();
            return;
        }

        s_rejecting = 0u;
        s_pend_n    = 0u;
    }

    s_raw = raw_new;

    if (s_good_run < 250u) { s_good_run ++; }
    if (s_good_run >= STEER_ABS_GOOD_FOR_OK) { s_link_ok = 1u; }

    steer_abs_derive();
}

uint16 steer_abs_get_frame       (void) { return s_frame; }
uint16 steer_abs_get_raw         (void) { return s_raw; }
uint16 steer_abs_get_raw_live    (void) { return s_raw_live; }
int16  steer_abs_get_center_delta(void) { return s_center_delta; }
float  steer_abs_get_deg         (void) { return s_deg; }
int16  steer_abs_get_deg_x100    (void) { return s_deg_x100; }
int    steer_abs_self_check_ok   (void) { return (int)s_self_ok; }
uint16 steer_abs_bad_cnt         (void) { return s_bad_cnt; }
uint8  steer_abs_link_ok         (void) { return s_link_ok; }
uint8  steer_abs_rejecting       (void) { return s_rejecting; }
