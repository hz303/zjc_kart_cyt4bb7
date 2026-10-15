/*********************************************************************************************************************
 * 转向绝对角度编码器(SPI) 实现 —— 协议、量纲、标定说明见 steer_abs.h
 * 主体移植自 北极星队 Polaris-Kart-TC387 的 code/Kart_Driver/kart_steer_abs.c
 * 标定值换成 队友 YINGYYUN 的实测值, 并新增"方向盘度"输出(见 steer_abs.h)。
 ********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "steer_abs.h"

#define STEER_ABS_SPI_W             (0x80)
#define STEER_ABS_SPI_R             (0x40)
#define STEER_ABS_ZERO_L_REG        (0x00)
#define STEER_ABS_ZERO_H_REG        (0x01)
#define STEER_ABS_DIR_REG           (0x09)
#define STEER_ABS_STATUS_REG        (0x06)
#define STEER_ABS_TIMEOUT_COUNT     (100)

static uint16   s_raw           = 0;
static uint16   s_frame         = 0;
static int16    s_center_delta  = 0;
static float    s_deg           = 0.0f;
static int16    s_deg_x100      = 0;
static uint8    s_self_ok       = 0;

/*-------------------------------------------------------------------------------------------------------------------
 *  raw - center 折回 ±2048。
 *  为什么要折: 编码器是 0~4095 的环形量, 中位若靠边(本车 2157, 离两端各约 2157/1939),
 *  单圈内不会跨零; 但只要机构再装偏一点就会跨, 跨了不折会读出 ±4000 的假打角 —— 而软限位
 *  是拿这个值比的, 那一下就是打死。折一次只花两个比较, 别省。
 *  ⚠ 2048 / 4096 是写死的(12 位宽决定), 换成不是 12 位的编码器时这两个数要跟着改。
 *-----------------------------------------------------------------------------------------------------------------*/
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

/*-------------------------------------------------------------------------------------------------------------------
 *  上电自检: 把 6 个配置寄存器反复写一遍, 直到状态寄存器读回 0x1C 才算好; 超过 100 轮放弃。
 *  ⚠ 自检失败【不代表 SPI 断了】: SPI 主机自产时钟, 从机不在时传输照样"完成", 只是读回全 0/全 1。
 *    所以判 SPI 死活要看 get_frame() 是否恒 0 或恒 0xFFFF。自检失败更可能是芯片型号不对/寄存器表不同。
 *-----------------------------------------------------------------------------------------------------------------*/
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
        /* 芯片零位写 0: 不把零点烧进编码器, 中位放在软件里(STEER_ABS_MID_RAW)。
           这样换齿轮/重标只改一个宏, 不用重新配芯片。 */
        steer_abs_write_register(STEER_ABS_DIR_REG,    0x00);
        steer_abs_write_register(STEER_ABS_ZERO_L_REG, 0x00);
        steer_abs_write_register(STEER_ABS_ZERO_H_REG, 0x00);
    }

    steer_abs_update();
}

void steer_abs_update (void)
{
    s_frame        = steer_abs_read_frame();
    s_raw          = (uint16)((s_frame >> STEER_ABS_RAW_SHIFT) & 0x0FFF);
    s_center_delta = steer_abs_wrap_delta(s_raw, (uint16)STEER_ABS_MID_RAW);

    /* 方向盘度: 先按实测的 50.08 计数/度折算, 再乘极性。
       用 center_delta(已解环绕)而不是 (raw - mid) —— 跨零那一拍两者才会分道扬镳。 */
    s_deg      = (float)STEER_ABS_DIR_SIGN * ((float)s_center_delta / STEER_ABS_CNT_PER_DEG);
    s_deg_x100 = (int16)(s_deg * 100.0f);
}

uint16 steer_abs_get_frame       (void) { return s_frame; }
uint16 steer_abs_get_raw         (void) { return s_raw; }
int16  steer_abs_get_center_delta(void) { return s_center_delta; }
float  steer_abs_get_deg         (void) { return s_deg; }
int16  steer_abs_get_deg_x100    (void) { return s_deg_x100; }
int    steer_abs_self_check_ok   (void) { return (int)s_self_ok; }
