/*********************************************************************************************************************
 * 惯导 - IMU 姿态与航向实现（IMU963RA + Mahony 九轴 AHRS）
 * 硬件: SPI_2, SCK P15_2 / MOSI P15_1 / MISO P15_0 / CS P15_3
 * 节拍: ins_imu_update() 在 5 ms PIT 中断内, 200 Hz; 配置与接口见 ins_imu.h
 ********************************************************************************************************************/

#include "zf_common_headfile.h"
#include <math.h>
#include "ins_imu.h"
#include "ins_odom.h"

#define DEG2RAD_F   (0.017453292519943295f)
#define RAD2DEG_F   (57.29577951308232f)

/* 临界区: 直接存取 PRIMASK, 可任意嵌套; 逐飞 interrupt_global_* 为计数器语义, 不能配对使用 */
static uint32 ins_crit_enter (void) { uint32 pm = __get_PRIMASK(); __disable_irq(); return pm; }
static void   ins_crit_exit  (uint32 pm) { __set_PRIMASK(pm); }

/* ---- 内部状态 ---- */
/* 陀螺零偏, raw LSB; 去零偏与死区均在 LSB 域 */
static float    s_bias_x = 0.0f;
static float    s_bias_y = 0.0f;
static float    s_bias_z = 0.0f;

/* 磁力计 Min-Max 标定 */
static volatile uint8  s_mag_cal_state = 0;         /* 0=IDLE 1=RUNNING 2=DONE */
static float    s_mag_off[3]   = {0.0f, 0.0f, 0.0f};
static float    s_mag_scale[3] = {1.0f, 1.0f, 1.0f};
static int16    s_mag_min[3]   = { 32767,  32767,  32767};
static int16    s_mag_max[3]   = {-32768, -32768, -32768};

/* 姿态四元数 */
static float    s_q[4]   = {1.0f, 0.0f, 0.0f, 0.0f};
static float    s_exInt  = 0.0f, s_eyInt = 0.0f, s_ezInt = 0.0f;
static uint16   s_quick_cnt = 0;

/* 输出 */
static volatile float s_yaw_acc   = 0.0f;           /* 累积航向, 度 */
static volatile uint16 s_yaw_rej  = 0u;             /* 被单拍合理性闸门丢弃的拍数, 诊断 */
static volatile float s_yaw_raw   = 0.0f;           /* Mahony 的 -180~180 */
static volatile float s_roll      = 0.0f;
static volatile float s_pitch     = 0.0f;
static volatile float s_gz_dps    = 0.0f;           /* 去零偏、过死区后的 Z 角速度 */
static volatile float s_dt_ms     = INS_T_S * 1000.0f;
static volatile float s_acc_norm  = 1.0f;
static volatile uint16 s_flags    = 0;
static volatile uint16 s_seq      = 0;

/* 最近一次有效磁力计, 拍间复用 */
static float    s_mag_g[3] = {0.0f, 0.0f, 0.0f};

/* ---- ZUPT 与在线零偏估计状态 ---- */
static volatile uint8  s_is_static   = 1;               /* 1 = 判为静止, 上电初值为 1 */
static uint16   s_zupt_quiet  = INS_ZUPT_ENTER_MS;      /* 静止条件连续满足时长(ms) */
static uint16   s_zupt_noisy  = 0;                      /* 不满足的连续时长(ms)     */
static uint16   s_horiz_run   = 0;                      /* 水平加速度条件连续满足时长(ms) */
static volatile uint8  s_bias_trk    = 0;               /* 本拍在跟踪零偏, 诊断     */
static volatile float  s_horiz_acc   = 0.0f;            /* 去重力后的水平加速度(g)  */

/* 冻结计数器: ticks 为冻结拍数(5 ms/拍), deg 为冻结期间 |gyro| 积分(度) */
static volatile uint32 s_frz_ticks = 0;                 /* 冻结拍数 */
static volatile float  s_frz_deg   = 0.0f;              /* 冻结期间被丢弃的转角, 度 */

/* DWT 实测 dt */
static uint8    s_dwt_ok   = 0;
static uint32   s_dwt_last = 0;

/* ---- DWT 周期计数器, Cortex-M7 自带, 用于实测 dt ---- */
static void ins_dwt_init (void)
{
    uint32 a, b;

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;

    a = DWT->CYCCNT;
    {
        volatile uint32 spin;
        for (spin = 0; spin < 2000; spin ++) { ; }
    }
    b = DWT->CYCCNT;

    s_dwt_ok   = ((b - a) > 100u) ? 1u : 0u;
    if (!s_dwt_ok)
    {
        s_flags |= INS_ST_DT_NODWT;         /* 如实上报: dt 用标称周期 */
    }
    s_dwt_last = DWT->CYCCNT;
}

/* 返回本拍实测 dt, 秒; 异常值钳回标称周期并置 INS_ST_DT_BAD */
static float ins_dt_seconds (void)
{
    float dt;

    if (!s_dwt_ok)
    {
        /* 未测到 dt 只置 NODWT, 不置 DT_BAD: 5 ms 为 PIT 硬周期, 标称值本身即准 */
        s_flags |= INS_ST_DT_NODWT;
        return INS_T_S;
    }

    {
        uint32 now = DWT->CYCCNT;
        uint32 cyc = now - s_dwt_last;                  /* uint32 差值天然处理回绕 */
        s_dwt_last = now;

        /* 用运行期主频换算, 不写死 250 MHz; SystemCoreClock 由 SystemCoreClockUpdate() 给出 */
        {
            float f_cpu = (float)SystemCoreClock;

            if (f_cpu < 1.0e6f) { f_cpu = 250000000.0f; }   /* 未初始化时退回标称值, 避免除 0 */
            dt = (float)cyc / f_cpu;
        }

        /* 合理区间 [T/2, 4T]; 越界说明丢拍或抖动, 钳回标称并置位 */
        if ((dt < (INS_T_S * 0.5f)) || (dt > (INS_T_S * 4.0f)))
        {
            s_flags |= INS_ST_DT_BAD;
            dt = INS_T_S;
        }
        else
        {
            /* 恢复正常即清标志, 只反映当前状态 */
            s_flags &= (uint16)~INS_ST_DT_BAD;
        }
    }

    return dt;
}

/* ---- IMU963RA 读取 ----
 * 哑字节、寄存器顺序与量程换算均由逐飞 imu963ra_* 库处理, 不自行拼寄存器 */
static void ins_read_raw (void)
{
    imu963ra_get_acc();
    imu963ra_get_gyro();
}

static void ins_read_mag (void)
{
    imu963ra_get_mag();
}

/* ---- 陀螺零偏标定, 要求静止 ----
 * 采样期内任一轴极差超过 INS_CALIB_MAX_SPAN_LSB 则重来, 最多 INS_CALIB_MAX_RETRY 次 */
static uint8 ins_calib_gyro_once (void)
{
    int32   sum_x = 0, sum_y = 0, sum_z = 0;
    int16   min_x =  32767, min_y =  32767, min_z =  32767;
    int16   max_x = -32768, max_y = -32768, max_z = -32768;
    uint16  i;
    uint32  primask;

    for (i = 0; i < INS_CALIB_SAMPLES; i ++)
    {
        int16 gx, gy, gz;

        ins_read_raw();
        gx = imu963ra_gyro_x;
        gy = imu963ra_gyro_y;
        gz = imu963ra_gyro_z;

        sum_x += gx; sum_y += gy; sum_z += gz;

        if (gx < min_x) min_x = gx;   if (gx > max_x) max_x = gx;
        if (gy < min_y) min_y = gy;   if (gy > max_y) max_y = gy;
        if (gz < min_z) min_z = gz;   if (gz > max_z) max_z = gz;

        system_delay_us(5000);                          /* 5ms 一拍，与运行期采样率一致 */
    }

    if (((int32)max_x - (int32)min_x > INS_CALIB_MAX_SPAN_LSB) ||
        ((int32)max_y - (int32)min_y > INS_CALIB_MAX_SPAN_LSB) ||
        ((int32)max_z - (int32)min_z > INS_CALIB_MAX_SPAN_LSB))
    {
        return 1;                                       /* 被碰过，重来 */
    }

    primask = ins_crit_enter();
    s_bias_x = (float)sum_x / (float)INS_CALIB_SAMPLES;
    s_bias_y = (float)sum_y / (float)INS_CALIB_SAMPLES;
    s_bias_z = (float)sum_z / (float)INS_CALIB_SAMPLES;
    ins_crit_exit(primask);

    if ((fabsf(s_bias_x) > INS_BIAS_LIMIT_LSB) ||
        (fabsf(s_bias_y) > INS_BIAS_LIMIT_LSB) ||
        (fabsf(s_bias_z) > INS_BIAS_LIMIT_LSB))
    {
        return 2;                                       /* 结果超限 = 传感器异常 */
    }

    return 0;
}

uint8 ins_imu_calib_gyro (void)
{
    uint8 try_n;
    uint8 ret = 2;

    for (try_n = 0; try_n < INS_CALIB_MAX_RETRY; try_n ++)
    {
        /* 每次尝试打印一行, 标定阻塞期间可见进度 */
        printf("[##] IMU gyro calib try %u/%u (%u samples, keep still)...\r\n",
               (unsigned)(try_n + 1), (unsigned)INS_CALIB_MAX_RETRY,
               (unsigned)INS_CALIB_SAMPLES);

        ret = ins_calib_gyro_once();

        if (0 == ret)
        {
            /* 不用 %f, IAR 默认库不一定支持浮点格式化, 一律按整数打印 */
            printf("[##] IMU gyro calib OK  bias=(%d, %d, %d) LSB\r\n",
                   (int)s_bias_x, (int)s_bias_y, (int)s_bias_z);
            break;
        }
        else if (1 == ret)
        {
            printf("[##] IMU gyro calib retry: 车被晃动了（单轴极差 > %d LSB）\r\n",
                   (int)INS_CALIB_MAX_SPAN_LSB);
        }
        else
        {
            printf("[##] IMU gyro calib FAIL: 零偏超限（|bias| > %d LSB）—— 检查 IMU 供电/SPI 接线\r\n",
                   (int)INS_BIAS_LIMIT_LSB);
        }
    }

    if (0 == ret)
    {
        s_flags &= (uint16)~INS_ST_WARN;
        s_flags |= INS_ST_READY;
    }
    else
    {
        s_flags |= INS_ST_WARN;
    }

    return ret;
}

/* ---- 磁力计 Min-Max 标定 ---- */
void ins_imu_mag_calib_start (void)
{
    s_mag_min[0] = s_mag_min[1] = s_mag_min[2] =  32767;
    s_mag_max[0] = s_mag_max[1] = s_mag_max[2] = -32768;
    s_mag_cal_state = 1;
    s_flags |=  INS_ST_MAG_CAL_RUN;
    s_flags &= (uint16)~INS_ST_MAG_CAL_DONE;
}

void ins_imu_mag_calib_stop (void)
{
    uint8 i;

    if (1 != s_mag_cal_state)
    {
        return;
    }

    for (i = 0; i < 3; i ++)
    {
        float span = (float)s_mag_max[i] - (float)s_mag_min[i];

        if (span < 100.0f)                              /* 这一轴基本没转动过 -> 不标 */
        {
            s_mag_cal_state = 0;
            s_flags |=  INS_ST_WARN;
            s_flags &= (uint16)~INS_ST_MAG_CAL_RUN;
            return;
        }

        s_mag_off[i]   = ((float)s_mag_max[i] + (float)s_mag_min[i]) * 0.5f;
        s_mag_scale[i] = 2.0f / span;                   /* 归一化到 ±1（只影响尺度，不影响方向） */
    }

    s_mag_cal_state = 2;
    s_flags &= (uint16)~INS_ST_MAG_CAL_RUN;
    s_flags |=  INS_ST_MAG_CAL_DONE;
}

/* ---- ZUPT 静止检测, 返回 1 = 本拍判为静止 ----
 * 三轴角速度最大值 < INS_ZUPT_GYRO_LIMIT_DPS 且 | |a| - 1g | < INS_ZUPT_ACC_TOL_G
 * 且水平加速度 < INS_ZUPT_HORIZ_TOL_G, 连续满足 ENTER_MS 判静止, 连续不满足 EXIT_MS 判运动 */
static uint8 ins_zupt_step (const float g_dps[3], const float a_g[3], float dt,
                            float vgx, float vgy, float vgz)
{
    float  gmax, an, dot, horiz;
    uint16 dt_ms;
    uint8  cond, horiz_ok;

    gmax = fabsf(g_dps[0]);
    {
        float t = fabsf(g_dps[1]); if (t > gmax) { gmax = t; }
        t = fabsf(g_dps[2]);       if (t > gmax) { gmax = t; }
    }

    an = sqrtf(a_g[0] * a_g[0] + a_g[1] * a_g[1] + a_g[2] * a_g[2]);

    dt_ms = (uint16)(dt * 1000.0f + 0.5f);
    if (0u == dt_ms) { dt_ms = 1u; }

    /* 水平加速度 = |sin(单位化加计与预测重力的夹角)| = sqrt(1 - dot^2) */
    if (an > 1e-3f)
    {
        dot = (a_g[0] * vgx + a_g[1] * vgy + a_g[2] * vgz) / an;
        if (dot >  1.0f) { dot =  1.0f; }
        if (dot < -1.0f) { dot = -1.0f; }
        horiz = sqrtf(1.0f - dot * dot);
    }
    else
    {
        horiz = 1.0f;                       /* 加计异常, 不判静止 */
    }
    s_horiz_acc = horiz;

    cond = (uint8)((gmax < INS_ZUPT_GYRO_LIMIT_DPS) &&
                   (fabsf(an - 1.0f) < INS_ZUPT_ACC_TOL_G));

    /* 轮速互斥: 惯性判据会把匀速低速直线行驶判成静止; 轮速证明在动时不判静止
       odom 未开时 v 恒为 0, 自动退回纯惯性判据 */
    if (fabsf(ins_odom_get_v_mps()) > INS_ZUPT_WHEEL_EPS)
    {
        cond = 0u;
    }

    if (0u != cond)
    {
        s_zupt_quiet = (s_zupt_quiet < 60000u) ? (uint16)(s_zupt_quiet + dt_ms) : 60000u;
        s_zupt_noisy = 0u;
    }
    else
    {
        s_zupt_noisy = (s_zupt_noisy < 60000u) ? (uint16)(s_zupt_noisy + dt_ms) : 60000u;
        s_zupt_quiet = 0u;
    }

    if ((0u == s_is_static) && (s_zupt_quiet >= INS_ZUPT_ENTER_MS)) { s_is_static = 1u; }
    if ((1u == s_is_static) && (s_zupt_noisy >= INS_ZUPT_EXIT_MS))  { s_is_static = 0u; }

    horiz_ok    = (uint8)(horiz < INS_ZUPT_HORIZ_TOL_G);
    s_horiz_run = horiz_ok ? ((s_horiz_run < 60000u) ? (uint16)(s_horiz_run + dt_ms) : 60000u) : 0u;

    /* 零偏门控用瞬时条件连续满足, 不用去抖后的 s_is_static */
    s_bias_trk = (uint8)((s_zupt_quiet >= INS_BIAS_GATE_MS) && (s_horiz_run >= INS_BIAS_GATE_MS));

    return s_is_static;
}

/* ---- Mahony 九轴 AHRS 一次更新 ---- */
static void ins_mahony_update (float dt)
{
    float gx, gy, gz;                                   /* rad/s */
    float ax, ay, az;                                   /* g     */
    float mx, my, mz;                                   /* 归一化前, 尺度无关 */
    float acc_sum, mag_sum;
    float recip;
    float q0, q1, q2, q3;
    float vx, vy, vz;
    float hx, hy, bx, bz;
    float wx, wy, wz;
    float ex, ey, ez;
    float halfT, kp, ki;
    int16 g_raw[3], a_raw[3];
    float gb[3];                                        /* 去零偏 + 过死区后的 raw LSB */
    float gd_dps[3];                                    /* 同上, °/s, ZUPT 判据用 */
    float a_g[3];                                       /* 加计, g, 未归一化 */
    float vgx, vgy, vgz;                                /* 预测重力方向（机体系，单位） */
    float db = INS_GYRO_DEADBAND_LSB;

    if (dt <= 1e-6f) { dt = INS_T_S; }

    g_raw[0] = imu963ra_gyro_x;  g_raw[1] = imu963ra_gyro_y;  g_raw[2] = imu963ra_gyro_z;
    a_raw[0] = imu963ra_acc_x;   a_raw[1] = imu963ra_acc_y;   a_raw[2] = imu963ra_acc_z;

    /* --- 陀螺: 去零偏(LSB) -> 死区(LSB) -> dps -> rad/s ---
       死区 0.5 LSB ≈ 0.035 dps, 只削量化底噪 */
    {
        gb[0] = (float)g_raw[0] - s_bias_x;
        gb[1] = (float)g_raw[1] - s_bias_y;
        gb[2] = (float)g_raw[2] - s_bias_z;

        if ((gb[0] > -db) && (gb[0] < db)) { gb[0] = 0.0f; }
        if ((gb[1] > -db) && (gb[1] < db)) { gb[1] = 0.0f; }
        if ((gb[2] > -db) && (gb[2] < db)) { gb[2] = 0.0f; }

        /* 饱和检测: ±2000 dps 量程, 接近满量程判异常 */
        if ((g_raw[0] > 32000) || (g_raw[0] < -32000) ||
            (g_raw[1] > 32000) || (g_raw[1] < -32000) ||
            (g_raw[2] > 32000) || (g_raw[2] < -32000))
        {
            s_flags |= INS_ST_GYRO_SAT;
        }

        gx = imu963ra_gyro_transition(gb[0]) * DEG2RAD_F;
        gy = imu963ra_gyro_transition(gb[1]) * DEG2RAD_F;
        gz = imu963ra_gyro_transition(gb[2]) * DEG2RAD_F;

        s_gz_dps = imu963ra_gyro_transition(gb[2]);
    }

    /* --- 加计: LSB -> g --- */
    ax = imu963ra_acc_transition(a_raw[0]);
    ay = imu963ra_acc_transition(a_raw[1]);
    az = imu963ra_acc_transition(a_raw[2]);
    a_g[0] = ax; a_g[1] = ay; a_g[2] = az;

    acc_sum = ax * ax + ay * ay + az * az;
    s_acc_norm = sqrtf(acc_sum);
    if (acc_sum < 1e-6f)
    {
        return;                                         /* 小向量保护 */
    }
    recip = 1.0f / sqrtf(acc_sum);
    ax *= recip; ay *= recip; az *= recip;

    /* --- 磁力计: Min-Max 校正 + 归一化, 必须用 float 中间量 --- */
    if (2 == s_mag_cal_state)
    {
        mx = ((float)imu963ra_mag_x - s_mag_off[0]) * s_mag_scale[0];
        my = ((float)imu963ra_mag_y - s_mag_off[1]) * s_mag_scale[1];
        mz = ((float)imu963ra_mag_z - s_mag_off[2]) * s_mag_scale[2];
        s_mag_g[0] = mx; s_mag_g[1] = my; s_mag_g[2] = mz;
    }
    else
    {
        mx = s_mag_g[0]; my = s_mag_g[1]; mz = s_mag_g[2];
    }

    mag_sum = mx * mx + my * my + mz * mz;
    if (mag_sum > 1e-6f)
    {
        recip = 1.0f / sqrtf(mag_sum);
        mx *= recip; my *= recip; mz *= recip;
    }
    else
    {
        mx = 0.0f; my = 0.0f; mz = 0.0f;
    }

    q0 = s_q[0]; q1 = s_q[1]; q2 = s_q[2]; q3 = s_q[3];

    /* --- 重力预测方向（四元数第三行） --- */
    {
        float q0q0 = q0*q0, q0q1 = q0*q1, q0q2 = q0*q2, q0q3 = q0*q3;
        float q1q1 = q1*q1, q1q2 = q1*q2, q1q3 = q1*q3;
        float q2q2 = q2*q2, q2q3 = q2*q3, q3q3 = q3*q3;

        vx = 2.0f * (q1q3 - q0q2);
        vy = 2.0f * (q0q1 + q2q3);
        vz = q0q0 - q1q1 - q2q2 + q3q3;

        /* --- 磁力计参考方向 --- */
        hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
        hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
        bx = sqrtf(hx * hx + hy * hy);
        bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

        wx = 2.0f * bx * (0.5f - q2q2 - q3q3) + 2.0f * bz * (q1q3 - q0q2);
        wy = 2.0f * bx * (q1q2 - q0q3) + 2.0f * bz * (q0q1 + q2q3);
        wz = 2.0f * bx * (q0q2 + q1q3) + 2.0f * bz * (0.5f - q1q1 - q2q2);
    }

    /* ZUPT 需在重力方向算出之后、误差与增益之前调用 */
    vgx = vx; vgy = vy; vgz = vz;
    /* 复用已算出的 rad/s 乘回得到 °/s, 省 3 次浮点除法; 与 transition() 相差 < 1e-7 */
    gd_dps[0] = gx * RAD2DEG_F;
    gd_dps[1] = gy * RAD2DEG_F;
    gd_dps[2] = s_gz_dps;

    (void)ins_zupt_step(gd_dps, a_g, dt, vgx, vgy, vgz);

    if (s_bias_trk)
    {
        /* 一阶跟踪: k = dt_ms / INS_BIAS_TAU_MS, 在 raw LSB 域跟踪 */
        float k = ((float)((uint16)(dt * 1000.0f + 0.5f))) / INS_BIAS_TAU_MS;

        if (k > 1.0f) { k = 1.0f; }
        s_bias_x += k * ((float)g_raw[0] - s_bias_x);
        s_bias_y += k * ((float)g_raw[1] - s_bias_y);
        s_bias_z += k * ((float)g_raw[2] - s_bias_z);

        s_flags |=  INS_ST_BIAS_TRK;
    }
    else
    {
        s_flags &= (uint16)~INS_ST_BIAS_TRK;
    }

    if (0u != s_is_static) { s_flags |=  INS_ST_STATIC; }
    else                   { s_flags &= (uint16)~INS_ST_STATIC; }

    /* --- 误差: 重力 + 磁(仅磁力计已标定时) ---
       未标定时跳过磁修正, 防止未校准磁数据造成偏航正反馈发散 */
    if (2 == s_mag_cal_state)
    {
        ex = (ay * vz - az * vy) + (my * wz - mz * wy);
        ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
        ez = (ax * vy - ay * vx) + (mx * wy - my * wx);
    }
    else
    {
        ex = (ay * vz - az * vy);
        ey = (az * vx - ax * vz);
        ez = (ax * vy - ay * vx);
    }

    kp = (s_quick_cnt < INS_MAHONY_QUICK_TICKS) ? INS_MAHONY_QUICK_KP : INS_MAHONY_KP;
    ki = (s_quick_cnt < INS_MAHONY_QUICK_TICKS) ? INS_MAHONY_QUICK_KI : INS_MAHONY_KI;
    if (s_quick_cnt < INS_MAHONY_QUICK_TICKS) { s_quick_cnt ++; }

    if (ki > 0.0f)
    {
        s_exInt += ex * ki * dt;
        s_eyInt += ey * ki * dt;

        /* 六轴下 ez 与 yaw 无关, 积分器无反馈通道会保持恒定 yaw 速率; 仅磁力计已标定时累积 */
        if (2 == s_mag_cal_state)
        {
            s_ezInt += ez * ki * dt;
        }
        else
        {
            s_ezInt = 0.0f;
        }

        /* 积分限幅 ±0.5, 防止急加速时段积分攒成静态偏置 */
        if (s_exInt >  0.5f) { s_exInt =  0.5f; }  if (s_exInt < -0.5f) { s_exInt = -0.5f; }
        if (s_eyInt >  0.5f) { s_eyInt =  0.5f; }  if (s_eyInt < -0.5f) { s_eyInt = -0.5f; }
        if (s_ezInt >  0.5f) { s_ezInt =  0.5f; }  if (s_ezInt < -0.5f) { s_ezInt = -0.5f; }
    }
    else
    {
        s_exInt = s_eyInt = s_ezInt = 0.0f;
    }

    gx += kp * ex + s_exInt;
    gy += kp * ey + s_eyInt;

    /* --- 航向: 冻结时 Z 轴角速度置 0, 否则 陀螺 + 修正; X/Y 轴始终做重力修正 --- */
#if INS_ZUPT_FREEZE_YAW
    if (0u != s_is_static)
    {
        /* 记录冻结期间本应转过的角度 */
        s_frz_ticks ++;
        s_frz_deg += fabsf(s_gz_dps) * dt;

        gz = 0.0f;
    }
    else
    {
        gz += kp * ez + s_ezInt;
    }
#else
    gz += kp * ez + s_ezInt;
#endif

    /* --- 四元数积分 + 归一化 ---
       先用旧 q 一次算完 4 个微分再一起累加, 避免各分量使用不同时间戳 */
    halfT = 0.5f * dt;
    {
        float d0 = (-q1 * gx - q2 * gy - q3 * gz) * halfT;
        float d1 = ( q0 * gx + q2 * gz - q3 * gy) * halfT;
        float d2 = ( q0 * gy - q1 * gz + q3 * gx) * halfT;
        float d3 = ( q0 * gz + q1 * gy - q2 * gx) * halfT;

        q0 += d0; q1 += d1; q2 += d2; q3 += d3;
    }

    mag_sum = q0*q0 + q1*q1 + q2*q2 + q3*q3;
    if ((mag_sum < 1e-6f) || (mag_sum > 1e6f) || (mag_sum != mag_sum))   /* NaN/Inf 保护 */
    {
        s_q[0] = 1.0f; s_q[1] = 0.0f; s_q[2] = 0.0f; s_q[3] = 0.0f;
        s_exInt = s_eyInt = s_ezInt = 0.0f;
        return;
    }
    recip = 1.0f / sqrtf(mag_sum);
    q0 *= recip; q1 *= recip; q2 *= recip; q3 *= recip;

    /* --- 累积航向: 取四元数的 ZYX 欧拉 yaw, 即车体 x 轴在世界水平面内的方位角 ---
       dyaw = 本拍 yaw 与上拍之差, 解绕后逐拍累加, 永不卷绕 */
    {
        float yaw_w = atan2f(2.0f * (q1 * q2 + q0 * q3),
                             q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * RAD2DEG_F;
        float dyaw  = yaw_w - s_yaw_raw;

        /* 解绕: s_yaw_raw 恒为 -180~180, 跨 ±180 时差分会跳 360° */
        while (dyaw >  180.0f) { dyaw -= 360.0f; }
        while (dyaw < -180.0f) { dyaw += 360.0f; }

        /* 单拍合理性闸门: 按本拍 dt 折算上限 3000 °/s, 钳在 10~180° */
        {
            float lim = 3000.0f * dt;

            if (lim <  10.0f) { lim =  10.0f; }
            if (lim > 180.0f) { lim = 180.0f; }

            if ((dyaw > lim) || (dyaw < -lim))
            {
                s_yaw_rej ++;
            }
            else
            {
                s_yaw_acc += dyaw;
            }
        }
    }

    s_q[0] = q0; s_q[1] = q1; s_q[2] = q2; s_q[3] = q3;

    /* --- 欧拉角（诊断用） --- */
    s_yaw_raw = atan2f(2.0f * (q1 * q2 + q0 * q3),
                       q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * RAD2DEG_F;
    s_roll    = atan2f(2.0f * (q0 * q1 + q2 * q3),
                       q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3) * RAD2DEG_F;
    {
        float sa = 2.0f * (q0 * q2 - q1 * q3);
        if (sa >  1.0f) { sa =  1.0f; }
        if (sa < -1.0f) { sa = -1.0f; }
        s_pitch = asinf(sa) * RAD2DEG_F;
    }
}

/* ---- 5 ms 每拍 ---- */
void ins_imu_update (void)
{
    static uint8 mag_div = 0;
    float dt;

    /* 链路坏时完全不碰 SPI: SPI 读寄存器为忙等, 照读会拖长 5 ms 中断 */
    if (s_flags & INS_ST_IMU_LINK_BAD)
    {
        s_dt_ms = INS_T_S * 1000.0f;
        s_acc_norm = 0.0f;                          /* 表示无有效数据 */
        s_seq ++;                                   /* 序号仍递增, 表示中断在运行 */
        return;
    }

    dt = ins_dt_seconds();

    ins_read_raw();

    if (s_mag_cal_state == 1)                           /* 标定中: 只采极值, 不跑姿态 */
    {
        if (imu963ra_mag_x < s_mag_min[0]) { s_mag_min[0] = imu963ra_mag_x; }
        if (imu963ra_mag_x > s_mag_max[0]) { s_mag_max[0] = imu963ra_mag_x; }
        if (imu963ra_mag_y < s_mag_min[1]) { s_mag_min[1] = imu963ra_mag_y; }
        if (imu963ra_mag_y > s_mag_max[1]) { s_mag_max[1] = imu963ra_mag_y; }
        if (imu963ra_mag_z < s_mag_min[2]) { s_mag_min[2] = imu963ra_mag_z; }
        if (imu963ra_mag_z > s_mag_max[2]) { s_mag_max[2] = imu963ra_mag_z; }
    }

    /* 磁力计降频读, sensor hub 为慢变量 */
    if (++ mag_div >= INS_MAG_DIV)
    {
        mag_div = 0;
        ins_read_mag();
    }

    ins_mahony_update(dt);

    s_dt_ms  = dt * 1000.0f;
    s_seq ++;
}

/* ---- 器件识别, 诊断用 ----
 * 963RA(LSM6DSL) WHO_AM_I @0x0F 期望 0x6A; 660RA CHIP_ID @0x00; 660RC CHIP_ID @0x0F
 * 裸读须在 imu963ra_init() 之后, SPI 与 CS 的 gpio 在其中初始化 */
static uint8 s_id_0f = 0u;          /* 0x0F 读回值 */
static uint8 s_id_00 = 0u;          /* 0x00 读回值 */
static uint8 s_id_valid = 0u;       /* 1 = 上面两个值有效 */

#if (1 == INS_IMU_ENABLE)
static uint8 ins_imu_raw_reg (uint8 reg)
{
    uint8 v;

    IMU963RA_CS(0);
    v = spi_read_8bit_register(IMU963RA_SPI, (uint8)(reg | IMU963RA_SPI_R));
    IMU963RA_CS(1);

    return v;
}
#endif  /* 关闭 IMU 时该函数无引用 */

/* 供 1 Hz 打印用: 取两个 ID 字节, 取后 valid=1 */
void ins_imu_dev_id_get (uint8 *id_0f, uint8 *id_00, uint8 *valid)
{
    if (id_0f) { *id_0f = s_id_0f; }
    if (id_00) { *id_00 = s_id_00; }
    if (valid) { *valid = s_id_valid; }
}

/* ---- 初始化 ---- */
uint8 ins_imu_init (void)
{
    s_flags    = 0;
    s_seq      = 0;
    s_yaw_acc  = 0.0f;
    s_yaw_raw  = 0.0f;
    s_roll     = 0.0f;
    s_pitch    = 0.0f;
    s_gz_dps   = 0.0f;
    s_quick_cnt = 0;
    s_exInt = s_eyInt = s_ezInt = 0.0f;
    s_q[0] = 1.0f; s_q[1] = s_q[2] = s_q[3] = 0.0f;
    s_mag_cal_state = 0;
    s_bias_x = s_bias_y = s_bias_z = 0.0f;
    s_mag_g[0] = s_mag_g[1] = s_mag_g[2] = 0.0f;

    /* ZUPT 上电初判静止, 标定结束后即可跟踪零偏; 车一动 exit 30 ms 内翻转 */
    s_is_static  = 1u;
    s_zupt_quiet = INS_ZUPT_ENTER_MS;
    s_zupt_noisy = 0u;
    s_horiz_run  = 0u;
    s_bias_trk   = 0u;
    s_horiz_acc  = 0.0f;

    /* 冻结计数器清零 */
    s_frz_ticks  = 0u;
    s_frz_deg    = 0.0f;

    ins_dwt_init();

    /* ---- 器件自动探测 + WHO_AM_I 打印 ----
     * 三个驱动宏引脚相同(SPI_2), 故须读 ID 判定板载型号:
     * 963RA(LSM6DSL) WHO_AM_I @0x0F 期望 0x6A; 660RA CHIP_ID @0x00; 660RC CHIP_ID @0x0F
     * 本工程仅支持 963RA, ID 不符时只如实上报, 不按 963RA 解算(量程系数不同) */
#if (0 == INS_IMU_ENABLE)
    /* INS_IMU_ENABLE = 0 时连 imu963ra_init() 都不调, SPI_2 完全不初始化, 等同 IMU 未接 */
    s_id_valid = 0u;
    s_flags   |= (INS_ST_WARN | INS_ST_IMU_LINK_BAD);

    printf("[##] INS    IMU DISABLED (INS_IMU_ENABLE=0) - SPI_2/SCB9 NOT initialized [A/B]\r\n");

    return 1;
#else
    uint8 ret;                          /* 仅在启用 IMU 的分支中使用 */

    if (0 != imu963ra_init())
    {
        s_id_0f    = ins_imu_raw_reg(0x0F);
        s_id_00    = ins_imu_raw_reg(0x00);
        s_id_valid = 1u;
        s_flags |= (INS_ST_WARN | INS_ST_IMU_LINK_BAD);
        return 1;
    }
    s_id_0f    = 0x6Au;                /* 963RA = LSM6DSL 系，WHO_AM_I(0x0F) 期望 0x6A */
    s_id_00    = 0u;
    s_id_valid = 1u;

    /* 上电陀螺零偏标定 */
    ret = ins_imu_calib_gyro();

    /* 姿态复位: 累积航向从 0 起, 车头方向记为起点朝向, 路表只用相对量 */
    s_yaw_acc = 0.0f;

    return ret;
#endif                                  /* INS_IMU_ENABLE */
}

/* ---- 查询 / 命令 ---- */
uint8  ins_imu_ready (void)         { return (s_flags & INS_ST_READY) ? 1u : 0u; }
uint16 ins_imu_flags (void)         { return s_flags; }

uint8  ins_imu_link_ok (void)
{
    return (s_flags & INS_ST_IMU_LINK_BAD) ? 0u : 1u;
}

uint8  ins_imu_try_recover (void)
{
#if (0 == INS_IMU_ENABLE)
    /* 禁用 IMU 时堵住此路径, 否则 imu963ra_init() 仍会初始化 SPI_2 */
    return 0u;
#else
    uint32 primask;

    if (0 == (s_flags & INS_ST_IMU_LINK_BAD))
    {
        return 0u;
    }

    /* 只在链路坏时走到这里 ⇒ 中断里的 ins_imu_update() 不碰 SPI_2 ⇒ 此处重入安全 */
    if (0 != imu963ra_init())
    {
        /* 仍不通时重读 ID, 供诊断区分无芯片与接线问题 */
        s_id_0f    = ins_imu_raw_reg(0x0F);
        s_id_00    = ins_imu_raw_reg(0x00);
        s_id_valid = 1u;
        return 0u;                                  /* 下一轮再试 */
    }
    s_id_0f    = 0x6Au;
    s_id_00    = 0u;
    s_id_valid = 1u;

    primask = ins_crit_enter();
    s_flags &= (uint16)~INS_ST_IMU_LINK_BAD;
    s_flags &= (uint16)~INS_ST_READY;               /* 尚未标定零偏, 不算就绪 */
    s_flags |=  INS_ST_WARN;                        /* 提醒: 零偏可能为旧值或 0 */
    ins_crit_exit(primask);

    return 1u;
#endif
}

float  ins_imu_get_yaw (void)       { return INS_YAW_SIGN * s_yaw_acc; }
uint16 ins_imu_yaw_rej (void)       { return s_yaw_rej; }
float  ins_imu_get_roll (void)      { return s_roll; }
float  ins_imu_get_pitch (void)     { return s_pitch; }
float  ins_imu_get_gyro_z_dps (void){ return s_gz_dps; }

/* ---- 冻结计数器, 取走即清 ----
 * 返回本窗口内的冻结量; deg 为被丢弃的转角, 接近 0 表示只是停着 */
void ins_imu_zupt_frz_get (uint32 *ticks, float *deg)
{
    uint32 primask = ins_crit_enter();

    if (ticks) { *ticks = s_frz_ticks; }
    if (deg)   { *deg   = s_frz_deg;   }
    s_frz_ticks = 0u;
    s_frz_deg   = 0.0f;

    ins_crit_exit(primask);
}

/* 只读、不清 */
uint32 ins_imu_zupt_frz_ticks (void) { return s_frz_ticks; }
float  ins_imu_zupt_frz_deg   (void) { return s_frz_deg;   }

void ins_imu_zupt_frz_reset (void)
{
    uint32 primask = ins_crit_enter();

    s_frz_ticks = 0u;
    s_frz_deg   = 0.0f;

    ins_crit_exit(primask);
}

void ins_imu_zero_yaw (void)
{
    s_yaw_acc = 0.0f;
}

void ins_imu_set_yaw (float deg)
{
    s_yaw_acc = deg / INS_YAW_SIGN;
}

/* ---- 诊断 ---- */
volatile uint32 ins_diag_isr_ticks = 0;

/* 5 ms 中断实际间隔的 min/max, DWT 实测, µs; 稳定 ≈5000 µs 为正常, 偶发 6000~9000 表示中断被拖 */
volatile uint32 ins_diag_tick_us_max = 0;
volatile uint32 ins_diag_tick_us_min = 0xFFFFFFFFu;
volatile uint32 ins_diag_tick_long_cnt = 0;   /* 间隔 > 7.5 ms 的拍数 */
static   uint32 ins_diag_tick_last_cyc = 0;
static   uint8  ins_diag_tick_first    = 1;

/* 主循环每秒取一次, 取走即清, 统计本窗口内的极值; long_cnt 只增不清 */
void ins_diag_tick_take (uint32 *min_us, uint32 *max_us)
{
    if (min_us) { *min_us = ins_diag_tick_us_min; }
    if (max_us) { *max_us = ins_diag_tick_us_max; }
    ins_diag_tick_us_min = 0xFFFFFFFFu;
    ins_diag_tick_us_max = 0;
}

/* 5 ms 中断体耗时的 min/max, µs, DWT 实测; 常态 150~250 µs, 偶发 > 1000 µs 表示 ISR 内有阻塞 */
volatile uint32 ins_diag_isr_us_max = 0;
volatile uint32 ins_diag_isr_us_min = 0xFFFFFFFFu;
static   uint32 ins_diag_isr_t0_cyc = 0;

void ins_diag_isr_leave (void)
{
    if (0 != s_dwt_ok)
    {
        uint32 d  = DWT->CYCCNT - ins_diag_isr_t0_cyc;
        uint32 us = (uint32)(((uint64_t)d * 1000000ull) / 250000000ull);

        if (us > ins_diag_isr_us_max) { ins_diag_isr_us_max = us; }
        if (us < ins_diag_isr_us_min) { ins_diag_isr_us_min = us; }
    }
}

void ins_diag_isr_cost_take (uint32 *min_us, uint32 *max_us)
{
    if (min_us) { *min_us = ins_diag_isr_us_min; }
    if (max_us) { *max_us = ins_diag_isr_us_max; }
    ins_diag_isr_us_min = 0xFFFFFFFFu;
    ins_diag_isr_us_max = 0;
}

void ins_diag_isr_enter (void)
{
    ins_diag_isr_ticks ++;

    if (0 != s_dwt_ok)
    {
        uint32 c = DWT->CYCCNT;

        if (0 == ins_diag_tick_first)
        {
            uint32 d = c - ins_diag_tick_last_cyc;
            /* 250 MHz ⇒ 1 µs = 250 个周期。用 64 位中间量避免溢出。 */
            uint32 us = (uint32)(((uint64_t)d * 1000000ull) / 250000000ull);

            if (us > ins_diag_tick_us_max) { ins_diag_tick_us_max = us; }
            if (us < ins_diag_tick_us_min) { ins_diag_tick_us_min = us; }
            if (us > 7500u)                { ins_diag_tick_long_cnt ++; }
        }
        ins_diag_tick_last_cyc = c;
        ins_diag_tick_first    = 0;

        ins_diag_isr_t0_cyc = c;            /* 中断体计时起点 */
    }
}

void ins_diag_get_imu_raw (int16 *ax, int16 *ay, int16 *az,
                           int16 *gx, int16 *gy, int16 *gz,
                           int16 *mx, int16 *my, int16 *mz)
{
    uint32 primask = ins_crit_enter();

    if (ax) { *ax = imu963ra_acc_x; }   if (ay) { *ay = imu963ra_acc_y; }   if (az) { *az = imu963ra_acc_z; }
    if (gx) { *gx = imu963ra_gyro_x; }  if (gy) { *gy = imu963ra_gyro_y; }  if (gz) { *gz = imu963ra_gyro_z; }
    if (mx) { *mx = imu963ra_mag_x; }   if (my) { *my = imu963ra_mag_y; }   if (mz) { *mz = imu963ra_mag_z; }

    ins_crit_exit(primask);
}

void ins_diag_get_bias (float *bx, float *by, float *bz)
{
    uint32 primask = ins_crit_enter();     /* 零偏每拍可能被在线估计改写, 需原子读取 */

    if (bx) { *bx = s_bias_x; }
    if (by) { *by = s_bias_y; }
    if (bz) { *bz = s_bias_z; }

    ins_crit_exit(primask);
}

void ins_imu_snapshot (ins_imu_snap_t *out)
{
    uint32 primask;

    if (NULL == out) { return; }

    primask = ins_crit_enter();
    out->yaw_acc     = INS_YAW_SIGN * s_yaw_acc;
    out->yaw_raw     = s_yaw_raw;
    out->roll        = s_roll;
    out->pitch       = s_pitch;
    out->gyro_z_dps  = s_gz_dps;
    out->dt_ms       = s_dt_ms;
    out->acc_norm_g  = s_acc_norm;
    out->horiz_acc_g = s_horiz_acc;
    out->is_static   = s_is_static;
    out->bias_trk    = s_bias_trk;
    out->flags       = s_flags;
    out->seq         = s_seq;
    ins_crit_exit(primask);
}
