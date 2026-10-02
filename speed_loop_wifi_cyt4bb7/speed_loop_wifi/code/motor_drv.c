/*********************************************************************************************************************
 * 双电机速度环(WiFi) - 两路 DRV8701E 驱动输出实现
 * 接口与硬件说明见 motor_drv.h
 ********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "motor_drv.h"

float mot_inv_l = 0.0f;
float mot_inv_r = 0.0f;

static float    s_out[2]     = {0.0f, 0.0f};    /* 实际写下去的(已含极性) */
static int      s_dir[2]     = {1, 1};          /* DIR 引脚电平           */
static uint32   s_pwm_raw[2] = {0, 0};          /* PWM 比较值             */

/*-------------------------------------------------------------------------------------------------------------------
 *  一路写硬件: u 为归一化带符号输出
 *-----------------------------------------------------------------------------------------------------------------*/
static void motor_drv_write (uint8 ch, float u)
{
    float   a;
    uint32  raw;

    if (u >= 0.0f)
    {
        gpio_set_level((0 == ch) ? MOTOR_L_DIR_PIN : MOTOR_R_DIR_PIN, GPIO_HIGH);
        s_dir[ch] = 1;
        a = u;
    }
    else
    {
        gpio_set_level((0 == ch) ? MOTOR_L_DIR_PIN : MOTOR_R_DIR_PIN, GPIO_LOW);
        s_dir[ch] = 0;
        a = -u;
    }

    if (a > 1.0f)
    {
        a = 1.0f;                               /* 物理上界, 最后一道收口 */
    }

    raw = (uint32)(a * (float)PWM_DUTY_MAX);
    pwm_set_duty((0 == ch) ? MOTOR_L_PWM_PIN : MOTOR_R_PWM_PIN, raw);

    s_pwm_raw[ch] = raw;
    s_out[ch]     = u;
}

void motor_drv_init (void)
{
    gpio_init(MOTOR_L_DIR_PIN, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    gpio_init(MOTOR_R_DIR_PIN, GPO, GPIO_HIGH, GPO_PUSH_PULL);

    pwm_init(MOTOR_L_PWM_PIN, MOTOR_PWM_FREQ_HZ, 0);
    pwm_init(MOTOR_R_PWM_PIN, MOTOR_PWM_FREQ_HZ, 0);

    pwm_set_duty(MOTOR_L_PWM_PIN, 0);
    pwm_set_duty(MOTOR_R_PWM_PIN, 0);

    s_out[0] = 0.0f; s_out[1] = 0.0f;
    s_dir[0] = 1;    s_dir[1] = 1;
    s_pwm_raw[0] = 0; s_pwm_raw[1] = 0;
}

void motor_drv_set (uint8 ch, float u)
{
    if (ch > 1)
    {
        return;
    }

    /* 极性修正放在"写硬件之前"这一层: 控制器和遥测看到的都是未翻转的 u,
       翻转只影响真正出去的 DIR/PWM, 这样极性标定不会污染控制律。 */
    if ((0 == ch) && (mot_inv_l > 0.5f)) { u = -u; }
    if ((1 == ch) && (mot_inv_r > 0.5f)) { u = -u; }

    motor_drv_write(ch, u);
}

void motor_drv_stop_all (void)
{
    motor_drv_write(0, 0.0f);
    motor_drv_write(1, 0.0f);
}

float motor_drv_out (uint8 ch)
{
    return (ch > 1) ? 0.0f : s_out[ch];
}

int motor_drv_dir_level (uint8 ch)
{
    return (ch > 1) ? 0 : s_dir[ch];
}

uint32 motor_drv_pwm_raw (uint8 ch)
{
    return (ch > 1) ? 0u : s_pwm_raw[ch];
}
