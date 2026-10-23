/* 转向位置环 + 双后轮速度环 + 遥控(WiFi/RC) - 三路 DRV8701E 驱动输出实现 接口与硬件说明(含引脚表)见 motor_drv.h */

#include "zf_common_headfile.h"
#include "motor_drv.h"

/* 左轮极性翻转 = 1、右轮 = 0。
 * 判据: 给一个小的正目标 -> 该侧轮子朝"车前进"方向转, 且同侧 rps 为正。
 */
float mot_inv_l = 1.0f;
float mot_inv_r = 0.0f;

/* 转向极性**必须**按 02-转向标定卡.md 在台架上单独验一次(车架空, 先只给 u0 看能不能推动,
 * 再给一个小目标角, 看方向盘是"朝目标去"还是"背着目标跑"); 反了就在上位机把
 * 「转向|电机极性」拖成 0(在线可改, 不必重烧)。
 * steer_ctrl.c 里另有一道**主动保护**(输出饱和且误差不收敛 -> 锁存停机)兜底, 但那是兜底不是许可。
 */
float mot_inv_s = 1.0f;

static float    s_out[MOTOR_CH_COUNT_DRV]     = {0.0f, 0.0f, 0.0f};    /* 实际写下去的(已含极性) */
static int      s_dir[MOTOR_CH_COUNT_DRV]     = {1, 1, 1};             /* DIR 引脚电平 */
static uint32   s_pwm_raw[MOTOR_CH_COUNT_DRV] = {0, 0, 0};             /* PWM 比较值 */

/* 通道 -> 引脚 的对照表(改引脚只改 motor_drv.h 的宏) */
static const pwm_channel_enum    s_pwm_pin[MOTOR_CH_COUNT_DRV] =
{
    MOTOR_L_PWM_PIN, MOTOR_R_PWM_PIN, MOTOR_S_PWM_PIN
};
static const gpio_pin_enum       s_dir_pin[MOTOR_CH_COUNT_DRV] =
{
    MOTOR_L_DIR_PIN, MOTOR_R_DIR_PIN, MOTOR_S_DIR_PIN
};

/* 一路写硬件: u 为归一化带符号输出 */
static void motor_drv_write (uint8 ch, float u)
{
    float   a;
    uint32  raw;

    if (u >= 0.0f)
    {
        gpio_set_level(s_dir_pin[ch], GPIO_HIGH);
        s_dir[ch] = 1;
        a = u;
    }
    else
    {
        gpio_set_level(s_dir_pin[ch], GPIO_LOW);
        s_dir[ch] = 0;
        a = -u;
    }

    if (a > 1.0f)
    {
        a = 1.0f;                               /* 物理上界, 最后一道收口 */
    }

    raw = (uint32)(a * (float)PWM_DUTY_MAX);
    pwm_set_duty(s_pwm_pin[ch], raw);

    s_pwm_raw[ch] = raw;
    s_out[ch]     = u;
}

void motor_drv_init (void)
{
    uint8 i;

    for (i = 0; i < MOTOR_CH_COUNT_DRV; i ++)
    {
        gpio_init(s_dir_pin[i], GPO, GPIO_HIGH, GPO_PUSH_PULL);
        pwm_init(s_pwm_pin[i], MOTOR_PWM_FREQ_HZ, 0);
        pwm_set_duty(s_pwm_pin[i], 0);

        s_out[i]     = 0.0f;
        s_dir[i]     = 1;
        s_pwm_raw[i] = 0;
    }
}

void motor_drv_set (uint8 ch, float u)
{
    if (ch >= MOTOR_CH_COUNT_DRV)
    {
        return;
    }

    /* 极性修正放在"写硬件之前"这一层: 控制器和遥测看到的都是未翻转的 u, 翻转只影响真正出去的 DIR/PWM, 这样极性标定不会污染控制律。 */
    if ((0 == ch) && (mot_inv_l > 0.5f)) { u = -u; }
    if ((1 == ch) && (mot_inv_r > 0.5f)) { u = -u; }
    if ((2 == ch) && (mot_inv_s > 0.5f)) { u = -u; }

    motor_drv_write(ch, u);
}

void motor_drv_stop_all (void)
{
    uint8 i;

    for (i = 0; i < MOTOR_CH_COUNT_DRV; i ++)
    {
        motor_drv_write(i, 0.0f);
    }
}

float motor_drv_out (uint8 ch)
{
    return (ch >= MOTOR_CH_COUNT_DRV) ? 0.0f : s_out[ch];
}

int motor_drv_dir_level (uint8 ch)
{
    return (ch >= MOTOR_CH_COUNT_DRV) ? 0 : s_dir[ch];
}

uint32 motor_drv_pwm_raw (uint8 ch)
{
    return (ch >= MOTOR_CH_COUNT_DRV) ? 0u : s_pwm_raw[ch];
}
