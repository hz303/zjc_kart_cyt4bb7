/* 转向绝对角度编码器(SPI) 实现 —— 协议、量纲、标定说明见 steer_abs.h */

#include "zf_common_headfile.h"
#include "steer_abs.h"

#define STEER_ABS_SPI_W             (0x80)
#define STEER_ABS_SPI_R             (0x40)
#define STEER_ABS_ZERO_L_REG        (0x00)
#define STEER_ABS_ZERO_H_REG        (0x01)
#define STEER_ABS_DIR_REG           (0x09)
#define STEER_ABS_STATUS_REG        (0x06)
#define STEER_ABS_TIMEOUT_COUNT     (100)

static uint16   s_raw           = 0;        /* 环里用的值 = 防抖**之后** */
static uint16   s_raw_live      = 0;        /* 本拍刚读到的原始计数（没防抖，判链路死活看它） */
static uint16   s_frame         = 0;
static int16    s_center_delta  = 0;
static float    s_deg           = 0.0f;
static int16    s_deg_x100      = 0;
static uint8    s_self_ok       = 0;

static uint16   s_bad_cnt       = 0;        /* 累计被拒样数（铁证：只增） */
static uint8    s_good_run      = 0;        /* 连续干净拍数 */
static uint8    s_link_ok       = 1;        /* 1 = 链路正常 */
static uint8    s_rejecting     = 0;        /* 1 = **本拍**被拒 */
static uint16   s_pend_raw      = 0;        /* 存疑值（等待确认） */
static uint8    s_pend_n        = 0;        /* 存疑值连续出现次数 */
static uint8    s_first         = 1;        /* 上电第一条样无条件接受（否则要空等 2 拍） */

/* raw - center 折回 ±2048。
 * 单圈内不会跨零; 但只要机构再装偏一点就会跨, 跨了不折会读出 ±4000 的假打角 —— 而软限位
 * 是拿这个值比的, 那一下就是打死。折一次只花两个比较, 别省。
 */
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
    spi_read_8bit(STEER_ABS_SPI_INDEX);         /* 回读两字节, 丢弃 */
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

/* 上电自检: 把 6 个配置寄存器反复写一遍, 直到状态寄存器读回 0x1C 才算好; 超过 100 轮放弃。
 * 所以判 SPI 死活要看 get_frame() 是否恒 0 或恒 0xFFFF。自检失败更可能是芯片型号不对/寄存器表不同。
 */
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
        /* 芯片零位写 0: 不把零点烧进编码器, 中位放在软件里(STEER_ABS_MID_RAW)。 这样换齿轮/重标只改一个宏, 不用重新配芯片。 */
        steer_abs_write_register(STEER_ABS_DIR_REG,    0x00);
        steer_abs_write_register(STEER_ABS_ZERO_L_REG, 0x00);
        steer_abs_write_register(STEER_ABS_ZERO_H_REG, 0x00);
    }

    steer_abs_update();
}

/* 由 s_raw 派生 center_delta / deg / deg_x100（只算这一个地方，避免两处公式漂移） */
static void steer_abs_derive (void)
{
    s_center_delta = steer_abs_wrap_delta(s_raw, (uint16)STEER_ABS_MID_RAW);

    /* 方向盘度: 先按实测的 50.08 计数/度折算, 再乘极性。 用 center_delta(已解环绕)而不是 (raw - mid) —— 跨零那一拍两者才会分道扬镳。 */
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

    /* ---- 上电第一条样：无条件接受（否则要空等 2 拍，s_deg 先是垃圾） ---- */
    if (0 != s_first)
    {
        s_first     = 0u;
        s_rejecting = 0u;
    }
    else
    {
        /* 差值必须按**12 位环绕差**算（环形量，别在 0/4096 交界处误判）。 */
        step = (int16)raw_new - (int16)s_raw;
        if (step >  2048) { step -= 4096; }
        if (step < -2048) { step += 4096; }

        if ((step > STEER_ABS_MAX_STEP_CNT) || (step < -STEER_ABS_MAX_STEP_CNT))
        {
            /* 同一个"存疑值"连续出现几次了？ */
            if (raw_new == s_pend_raw) { s_pend_n ++; }
            else                       { s_pend_raw = raw_new; s_pend_n = 1u; }

            s_bad_cnt ++;
            s_good_run  = 0u;
            s_link_ok   = 0u;
            s_rejecting = 1u;

            if (s_pend_n < STEER_ABS_CONFIRM_N)
            {
                /* ① 孤立乱值：下一拍就变回去了 ⇒ **完全不动**，环里连一丝扰动都没有。 （"晃一下"就堵在这里 —— 环里始终是上一个好值。） */
                steer_abs_derive();
                return;
            }

            /* ② 这个新值已经连续存在 STEER_ABS_CONFIRM_N 拍了 ⇒ 倾向于"是真动作"，
             * 但也**不许一跳到位**，只按 STEER_ABS_MAX_STEP_CNT **限速逼近**：
             * · 真·快速摆动：几拍内跟上，代价 5~15ms 滞后；
             * · 持续存在的乱值：`rejecting` 会一直亮 ⇒ 上层 enc_bad_ms 漏水积分持续累加
             * ⇒ 200ms 内锁存、输出钉 0（这才是它该有的下场，而不是让环路追着它跑）。
             * 所以永远不会出现"一拍之内甩到硬限位"。
             */
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
