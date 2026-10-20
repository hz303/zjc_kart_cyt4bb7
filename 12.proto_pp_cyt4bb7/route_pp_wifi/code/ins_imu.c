/*********************************************************************************************************************
 * 工程 10（卡丁）惯导 - IMU 姿态与航向实现（IMU963RA + Mahony 九轴）
 *
 * ★★ 本文件由【工程 8（C 车）】的 `ins_imu.c` **整体移植**（2026-10-15），算法一行未改。
 *    移植适配只有三处，都在本文件内部（搜 "工程 10 适配"）：
 *      ① 顶部这份说明
 *      ② `ins_imu_init()` 的**器件自动探测 + WHO_AM_I 打印**（卡丁从没初始化过 IMU，不能赌）
 *      ③ 上电标定的提示语改成卡丁的实操话术
 *    ⚠ 不要"顺手优化"这里的 ZUPT / 在线零偏 —— 它比 Polaris 卡丁那套强，
 *       是 C 车实测过的（开 ZUPT 静止 20 s 残差 0.0000° vs 关 2.098°）。
 *
 * 原设计意图、三处修正、节拍约束见 ins_imu.h
 * 算法谱系: 20.zf_clean/code/IMU_Analysis.c（Mahony 九轴 AHRS，已在 C 车上跑过）
 *           + 工程 1 的"实测 dt / 零偏标定"做法
 *           + 工程 2 的"累积角逐拍增量、不用 atan2"做法
 *
 * ★ 硬件事实（工程 10 已核实）：逐飞 CYT4BB7 主板的 IMU660RA / IMU660RC / **IMU963RA**
 *   三个驱动宏**引脚完全相同** = `SPI_2`：SCK **P15_2** / MOSI **P15_1** / MISO **P15_0** / CS **P15_3**。
 *   工程 9（卡丁）**P15 一根都没占用** ⇒ 不冲突。
 ********************************************************************************************************************/

#include "zf_common_headfile.h"
#include <math.h>
#include "ins_imu.h"
#include "ins_odom.h"

#define DEG2RAD_F   (0.017453292519943295f)
#define RAD2DEG_F   (57.29577951308232f)

/*===================================================================================================================
 *  ★★ 临界区：**绝不要用逐飞的 `interrupt_global_disable()` / `interrupt_global_enable(pm)` 配对**
 *
 *  原因（见 libraries/zf_common/zf_common_interrupt.c）—— 那一对不是"保存/恢复 PRIMASK"，
 *  而是一个自增自减的计数器，且 `enable` 只在参数为 0 时才真的开中断：
 *
 *      void interrupt_global_enable (uint32 primask)
 *      {
 *          if(primask == 0) { __enable_irq(); }    // ★ 只有传 0 才开中断
 *          interrupt_global_flag ++;
 *      }
 *      uint32 interrupt_global_disable (void)
 *      {
 *          __disable_irq();
 *          if(interrupt_global_flag > 0) interrupt_global_flag --;
 *          return interrupt_global_flag;           // ★ 返回计数器，不是 PRIMASK
 *      }
 *
 *  ⇒ `clock_init()` 里调过一次 `interrupt_global_enable(0)`，`main()` 里又调一次，
 *    计数器已经 ≥ 2。此后写 `pm = disable(); ...; enable(pm);`，`pm` 拿到的是 1（非 0）
 *    ⇒ **`__enable_irq()` 不会执行 ⇒ 中断被永久关死，且零报错**。
 *    症状：主循环照跑，但 5ms 中断再不进入 ⇒ 速度环/编码器/IMU 全停、`motor_tick_ms` 不涨、
 *    WiFi 一帧不发（`motor_link_task()` 连 if 体都不进）。
 *
 *  ⇒ 正确做法就是下面两行：直接存取 PRIMASK（CMSIS 标准做法，可任意嵌套）。
 *=================================================================================================================*/
static uint32 ins_crit_enter (void) { uint32 pm = __get_PRIMASK(); __disable_irq(); return pm; }
static void   ins_crit_exit  (uint32 pm) { __set_PRIMASK(pm); }

/*===================================================================================================================
 *  内部状态
 *=================================================================================================================*/
/* 陀螺零偏（raw LSB，去零偏在 LSB 域做，死区也在 LSB 域做） */
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
static volatile float s_yaw_acc   = 0.0f;           /* ★ 累积航向（度） */
static volatile uint16 s_yaw_rej  = 0u;             /* 被"单拍合理性闸门"丢掉的拍数（诊断）*/
static volatile float s_yaw_raw   = 0.0f;           /* Mahony 的 -180~180 */
static volatile float s_roll      = 0.0f;
static volatile float s_pitch     = 0.0f;
static volatile float s_gz_dps    = 0.0f;           /* 去零偏/过死区后 Z 角速度 */
static volatile float s_dt_ms     = INS_T_S * 1000.0f;
static volatile float s_acc_norm  = 1.0f;
static volatile uint16 s_flags    = 0;
static volatile uint16 s_seq      = 0;

/* 最近一次有效磁力计（float，sensor hub 低频更新，拍间复用） */
static float    s_mag_g[3] = {0.0f, 0.0f, 0.0f};

/*===================================================================================================================
 *  ★★ ZUPT / 在线零偏估计状态（2026-10-11 新增，见 ins_imu.h 一·b）
 *  为什么需要：上电 5 s 标定完零偏就**冻结**了，芯片升温会让零偏走掉 1~2 LSB，
 *  而六轴下 yaw 不可观测 ⇒ 静止时 yaw 匀速单调上升。ZUPT 是唯一能观测到零偏的窗口。
 *=================================================================================================================*/
static volatile uint8  s_is_static   = 1;               /* 上电先判"静止"（MM32 同做法：可立刻开始攒零偏） */
static uint16   s_zupt_quiet  = INS_ZUPT_ENTER_MS;      /* 静止条件**连续**满足的时长(ms) */
static uint16   s_zupt_noisy  = 0;                      /* 不满足的连续时长(ms)           */
static uint16   s_horiz_run   = 0;                      /* 水平加速度条件连续满足时长(ms) */
static volatile uint8  s_bias_trk    = 0;               /* 本拍是否在跟踪零偏（诊断）      */
static volatile float  s_horiz_acc   = 0.0f;            /* 去重力后的水平加速度(g)        */

/*===================================================================================================================
 *  ★★ 冻结计数器（2026-10-12 新增）—— 回答"ZUPT 到底吃掉了多少转角"
 *
 *  背景：实验 B 里"极慢原地转 90° 只读出 8.6°"。机理是 `gz = 0` 把整段低速转动丢掉了，
 *  但**丢掉多少只有实测才知道**。这两个计数器就是那把尺子：
 *      s_frz_ticks —— 本段内有几拍 `s_is_static == 1`（5 ms/拍 ⇒ ×5 = 毫秒）
 *      s_frz_deg   —— 冻结期间被丢弃的 |gyro| 积分（度）= Σ |s_gz_dps| · dt
 *
 *  ★ 判读：
 *      fz 大、fzd ≈ 0   ⇒ 只是"停着不动时被冻"（正确行为，没丢东西）
 *      fzd 明显 > 0     ⇒ 冻结期间**确实在转** ⇒ 这些转角被吃掉了 ⇒ 必须查为什么低速被判静止
 *  起点：`ins_imu_zupt_frz_reset()`；录表开始时由 `ins_route_start()` 自动清零。
 *=================================================================================================================*/
static volatile uint32 s_frz_ticks = 0;                 /* 冻结拍数                        */
static volatile float  s_frz_deg   = 0.0f;              /* 冻结期间被丢弃的转角（度）      */

/* DWT 实测 dt */
static uint8    s_dwt_ok   = 0;
static uint32   s_dwt_last = 0;

/*===================================================================================================================
 *  DWT 周期计数器（Cortex-M7 自带，用于实测 dt）
 *=================================================================================================================*/
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
        s_flags |= INS_ST_DT_NODWT;         /* 如实上报：dt 将用标称周期 */
    }
    s_dwt_last = DWT->CYCCNT;
}

/* 返回本拍实测 dt（秒）。异常值钳回标称周期，并置位 INS_ST_DT_BAD。 */
static float ins_dt_seconds (void)
{
    float dt;

    if (!s_dwt_ok)
    {
        /* ★ 用 NODWT 而不是 DT_BAD：这只表示"没测到"，不是"测到异常"。
           5ms 是 PIT 硬周期，标称值本身就是准的 ⇒ 不该每次上电都挂告警。 */
        s_flags |= INS_ST_DT_NODWT;
        return INS_T_S;
    }

    {
        uint32 now = DWT->CYCCNT;
        uint32 cyc = now - s_dwt_last;                  /* uint32 差值天然处理回绕 */
        s_dwt_last = now;

        /* ★★ 2026-10-20：**用运行期主频，不写死 250 MHz**。
           原来这里硬编码 250e6：一旦 M0 那边把核心配成 280/350 MHz（或以后换板子），
           dt 就会系统性偏小/偏大 ⇒ 四元数按错的时间积分 ⇒ **航向按比例少转/多转**。
           这个错很像"陀螺刻度差 10%"，但根子在时钟：症状就是"转一圈 yaw 只有 320°"。
           SystemCoreClock 由 SystemCoreClockUpdate() 启动时从寄存器算出。 */
        {
            float f_cpu = (float)SystemCoreClock;

            if (f_cpu < 1.0e6f) { f_cpu = 250000000.0f; }   /* 没初始化就退回标称，别除 0 */
            dt = (float)cyc / f_cpu;
        }

        /* 合理区间 [T/2, 4T]；越界说明丢拍/抖动，钳回标称并置位 */
        if ((dt < (INS_T_S * 0.5f)) || (dt > (INS_T_S * 4.0f)))
        {
            s_flags |= INS_ST_DT_BAD;
            dt = INS_T_S;
        }
        else
        {
            /* ★ 不粘滞：恢复正常就把标志清掉。
               （原来只置位不清，于是"WiFi 建连那几秒把中断拖了一次"这种一次性事件
                 会让上电后的整个运行期都挂着 DT_BAD —— 那不是"当前状态"，是历史。） */
            s_flags &= (uint16)~INS_ST_DT_BAD;
        }
    }

    return dt;
}

/*===================================================================================================================
 *  IMU963RA 读取
 *  ⚠ 963RA 与 660RA 的三处差异（**都不报错**，见 ins_imu.h 与方案文档 §3.2）:
 *     ① 不需要哑字节 ② 数据寄存器陀螺在前（0x22）且不连续 ③ 换算 ÷14.3/÷4098/÷3000
 *  这三条逐飞库的 imu963ra_* 已经处理好了 ⇒ **只用库函数，不要自己拼寄存器**。
 *=================================================================================================================*/
static void ins_read_raw (void)
{
    imu963ra_get_acc();
    imu963ra_get_gyro();
}

static void ins_read_mag (void)
{
    imu963ra_get_mag();
}

/*===================================================================================================================
 *  陀螺零偏标定（上电，要求静止）
 *  带"被碰检测": 采样期内任一轴极差超过门槛就重来（最多 INS_CALIB_MAX_RETRY 次）。
 *  这条是 20.zf_clean 没有、而工程 1 有的 —— 防"标定时有人碰车"这个致命场景。
 *=================================================================================================================*/
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
        /* ★ 每次尝试都打印一行：标定是阻塞的（每轮 INS_CALIB_SAMPLES×5ms），
           不打印的话失败重试会让现场看起来像"卡住了/什么都没发生"。 */
        printf("[##] IMU gyro calib try %u/%u (%u samples, keep still)...\r\n",
               (unsigned)(try_n + 1), (unsigned)INS_CALIB_MAX_RETRY,
               (unsigned)INS_CALIB_SAMPLES);

        ret = ins_calib_gyro_once();

        if (0 == ret)
        {
            /* ⚠ 不用 %f：IAR 默认库不一定支持浮点格式化（这个坑本工作区踩过），一律打整数 */
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

/*===================================================================================================================
 *  磁力计 Min-Max 标定
 *=================================================================================================================*/
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

/*===================================================================================================================
 *  ZUPT 静止检测（2026-10-11 新增）—— 返回 1 = 本拍判定为静止
 *
 *  三条纪律（全部来自 MM32 的 yaw_filter.c / trail_att.c，别改）：
 *   ① **三轴一起判**（取三轴角速度最大值）。只看 Z 轴会把"绕水平轴晃动"误判成静止。
 *   ② **加计判据用 | |a| − 1g | **（抓线加速度）+ **去重力后的水平加速度**（抓水平加速：
 *      1 m/s² 只让 |a| 变 0.005 g，而噪声就有 0.002 g —— 光靠模长抓不住）。
 *   ③ **enter 150 ms / exit 30 ms**：进得慢（防抖），出得快（保证响应灵敏）。
 *
 *  ⚠ `horiz` 用"把单位化加计投到预测重力法线上取垂足"算，等价于 MM32 的
 *    "把 ac 转到世界系后取 x/y 模长"，但少构造一次旋转矩阵。
 *=================================================================================================================*/
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

    /* 去重力后的水平加速度 ||a_hat − (a_hat·v̂)v̂|| = sqrt(1 − cos²) = |sin(夹角)| */
    if (an > 1e-3f)
    {
        dot = (a_g[0] * vgx + a_g[1] * vgy + a_g[2] * vgz) / an;
        if (dot >  1.0f) { dot =  1.0f; }
        if (dot < -1.0f) { dot = -1.0f; }
        horiz = sqrtf(1.0f - dot * dot);
    }
    else
    {
        horiz = 1.0f;                       /* 加计异常 ⇒ 不敢判静止 */
    }
    s_horiz_acc = horiz;

    cond = (uint8)((gmax < INS_ZUPT_GYRO_LIMIT_DPS) &&
                   (fabsf(an - 1.0f) < INS_ZUPT_ACC_TOL_G));

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

    /* ★★ 零偏门控 = 瞬时条件连续满足，**不是**去抖后的 s_is_static（见 ins_imu.h 的警告） */
    s_bias_trk = (uint8)((s_zupt_quiet >= INS_BIAS_GATE_MS) && (s_horiz_run >= INS_BIAS_GATE_MS));

    return s_is_static;
}

/*===================================================================================================================
 *  Mahony 九轴 AHRS 一次更新
 *  参考 20.zf_clean/IMU_Analysis.c 的 Mahony_AHRS_Update，三处改动见 ins_imu.h
 *=================================================================================================================*/
static void ins_mahony_update (float dt)
{
    float gx, gy, gz;                                   /* rad/s */
    float ax, ay, az;                                   /* g     */
    float mx, my, mz;                                   /* 归一化前（尺度不重要）*/
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
    float gd_dps[3];                                    /* 同上换算成 °/s（ZUPT 判据用）*/
    float a_g[3];                                       /* 加计（g，未归一化）         */
    float vgx, vgy, vgz;                                /* 预测重力方向（机体系，单位） */
    float db = INS_GYRO_DEADBAND_LSB;

    if (dt <= 1e-6f) { dt = INS_T_S; }

    g_raw[0] = imu963ra_gyro_x;  g_raw[1] = imu963ra_gyro_y;  g_raw[2] = imu963ra_gyro_z;
    a_raw[0] = imu963ra_acc_x;   a_raw[1] = imu963ra_acc_y;   a_raw[2] = imu963ra_acc_z;

    /* --- 陀螺: 去零偏(LSB) -> 死区(LSB) -> dps -> rad/s ---
       ★【改 1】死区从 7 LSB 收到 0.5 LSB。真实角速度只有 0.5~2 dps，大死区会把它吃掉。
       ⚠ 副作用：大死区当年恰好把"零偏慢漂"一起盖住了 ⇒ 改小之后静止漂移露出来。
         正确解法是下面补的 ZUPT，**不是把死区改回去**。 */
    {
        gb[0] = (float)g_raw[0] - s_bias_x;
        gb[1] = (float)g_raw[1] - s_bias_y;
        gb[2] = (float)g_raw[2] - s_bias_z;

        if ((gb[0] > -db) && (gb[0] < db)) { gb[0] = 0.0f; }
        if ((gb[1] > -db) && (gb[1] < db)) { gb[1] = 0.0f; }
        if ((gb[2] > -db) && (gb[2] < db)) { gb[2] = 0.0f; }

        /* 饱和检测（±2000dps 量程 16 位，接近满量程判异常） */
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
        return;                                         /* 小向量保护（比判 recip 更稳） */
    }
    recip = 1.0f / sqrtf(acc_sum);
    ax *= recip; ay *= recip; az *= recip;

    /* --- 磁力计: Min-Max 校正 + 归一化（**必须用 float 中间量**，int16 会被截断成 0）--- */
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

    /*===============================================================================================================
     *  ★★ ZUPT 静止检测 + 在线零偏估计（2026-10-11 新增）
     *  位置很讲究：必须在**重力方向 vx/vy/vz 算出来之后**调用（水平加速度判据要用它），
     *  又必须在**误差/增益之前**（冻结开关要影响本拍的 gz）。
     *  `is_static` 上电默认 1 ⇒ 上电就能立刻开始攒零偏，一旦数据表明在动，exit 30 ms 就翻过来。
     *=============================================================================================================*/
    vgx = vx; vgy = vy; vgz = vz;
    /* 复用上面已经算好的 rad/s，乘回来就得到 °/s —— **省 3 次浮点除法**（ISR 里值得）。
       与 imu963ra_gyro_transition(gb[i]) 只差一个 ~1e-7 的浮点往返，对 ZUPT 判据无关。 */
    gd_dps[0] = gx * RAD2DEG_F;
    gd_dps[1] = gy * RAD2DEG_F;
    gd_dps[2] = s_gz_dps;

    (void)ins_zupt_step(gd_dps, a_g, dt, vgx, vgy, vgz);

    if (s_bias_trk)
    {
        /* 一阶跟踪：k = dt_ms / τ_ms（MM32 的 yaw_bias_track 就是这个式子）。
           ⚠ 这里在 **raw LSB 域**跟踪（不动 s_bias 的语义）⇒ 去零偏/死区/换算三处都不用改。 */
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

    /* --- 误差: 重力 + （仅在磁力计已标定时）磁 ---
       未标定时跳过磁修正 —— 否则未校准的磁数据会造成偏航正反馈发散（20.zf_clean 的原始保护）。 */
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

        /* ★★ 六轴下 `ez = ax*vy - ay*vx` 与 yaw **无关** ⇒ 它的积分器没有反馈通道：
           启动瞬态把它推到某个值之后，即使 ez 回到 0 也**不会衰减**（积分器零输入保持），
           于是变成一个恒定 yaw 速率 ⇒ 静止恒速漂。
           MM32 的 `trail_att.c` 用 Ki=0 把这条整个回避掉了；本工程只在**磁力计已标定**
           （yaw 真有观测）时才累积，否则强制清零。 */
        if (2 == s_mag_cal_state)
        {
            s_ezInt += ez * ki * dt;
        }
        else
        {
            s_ezInt = 0.0f;
        }

        /* 积分限幅（20.zf_clean 没做，这里补上：急加速时段积分会攒成静态偏置） */
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

    /* --- 航向：冻结时整条 Z 轴分支置 0；否则 陀螺 + 修正 ---
       MM32 `trail_att.c` 第 6 步原话："静止时把 Z 轴角速度强制为 0 = 冻结航向
       （剩下的都是零偏残差）；X/Y 轴仍然做重力修正，因为静止窗口正是把 roll/pitch
       拉平的最好时机。" 本工程一致。 */
#if INS_ZUPT_FREEZE_YAW
    if (0u != s_is_static)
    {
        /* ★ 冻结计数器：这一拍躺平了，把"本来会转多少"记下来（见 s_frz_* 的注释）。
           |s_gz_dps| 已经去过零偏、过过死区 ⇒ 纯静止时它 ≈ 0，不会虚增。 */
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
       ★★ 2026-10-20：改成**先用旧 q 一次算完 4 个微分，再一起加**。
          原来是原地更新（q1 那一行已经用上刚改过的 q0），等于每个分量吃到的时间戳
          都不一样；单拍误差 O(dt²) 很小，但它**随转速单调累积**——转一整圈就能看出
          "航向少转了几度"，和陀螺刻度错的表现几乎一样，很难区分。 */
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

    /* --- ★【改 3】累积航向：相邻四元数的相对旋转 Z 分量，逐拍累加，永不卷绕 ---
       rel = conj(q_prev) ⊗ q_new ;
         rel.w = pw*w + px*x + py*y + pz*z
         rel.z = pw*z - px*y + py*x - pz*w
       dyaw  = 2*atan2(rel.z, rel.w)
       （用 atan2(abs(q0,q3)±...) 那种做法在 ±180° 边界会丢一整圈 —— 工程 2 数值复现过。） */
    {
        float pw = s_q[0], px = s_q[1], py = s_q[2], pz = s_q[3];
        float rel_w = pw * q0 + px * q1 + py * q2 + pz * q3;
        float rel_z = pw * q3 - px * q2 + py * q1 - pz * q0;
        float dyaw  = 2.0f * atan2f(rel_z, rel_w) * RAD2DEG_F;

        /* ★ 单拍合理性闸门（2026-10-20）：按**本拍真实 dt** 折算角速度上限，
           而不是写死 90°。写死的话，一旦主循环/中断被拖住（WiFi 收发、菜单刷屏），
           那一拍的真转角可能就超过 90° —— **闸门自己会把真实转动丢掉**，
           正好又制造出"转一圈角度缺一块"。这里用 3000 °/s 作上限：
           本车满舵也就几百 °/s，单拍 5ms 约 15°，真毛刺挡得住，真转动漏不掉。 */
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

/*===================================================================================================================
 *  5ms 每拍
 *=================================================================================================================*/
void ins_imu_update (void)
{
    static uint8 mag_div = 0;
    float dt;

    /* ★★ 器件没通就**一步都不碰 SPI**。
       这是本模块最重要的一条硬约束：`ins_imu_update()` 跑在 5ms 中断里，
       而 SPI 读寄存器是"发一个字节等接收 FIFO"的忙等（虽然有超时兜底，仍会占时间）。
       IMU 没接好时如果照读，每一次中断都要白等超时 —— 中断被拖长、主循环被饿死，
       现象是"编码器/IMU 都读不到"甚至"WiFi 一帧都发不出"，而根因其实只是 IMU 没插好。
       链接状态由 ins_imu_init()/ins_imu_try_recover() 维护。 */
    if (s_flags & INS_ST_IMU_LINK_BAD)
    {
        s_dt_ms = INS_T_S * 1000.0f;
        s_acc_norm = 0.0f;                          /* 明确表达"没有有效数据" */
        s_seq ++;                                   /* 序号照涨 ⇒ 上位机能看出中断还活着 */
        return;
    }

    dt = ins_dt_seconds();

    ins_read_raw();

    if (s_mag_cal_state == 1)                           /* 标定中：只采极值，不跑姿态 */
    {
        if (imu963ra_mag_x < s_mag_min[0]) { s_mag_min[0] = imu963ra_mag_x; }
        if (imu963ra_mag_x > s_mag_max[0]) { s_mag_max[0] = imu963ra_mag_x; }
        if (imu963ra_mag_y < s_mag_min[1]) { s_mag_min[1] = imu963ra_mag_y; }
        if (imu963ra_mag_y > s_mag_max[1]) { s_mag_max[1] = imu963ra_mag_y; }
        if (imu963ra_mag_z < s_mag_min[2]) { s_mag_min[2] = imu963ra_mag_z; }
        if (imu963ra_mag_z > s_mag_max[2]) { s_mag_max[2] = imu963ra_mag_z; }
    }

    /* 磁力计降频读（sensor hub 是慢变量；省下的时间留给控制环） */
    if (++ mag_div >= INS_MAG_DIV)
    {
        mag_div = 0;
        ins_read_mag();
    }

    ins_mahony_update(dt);

    s_dt_ms  = dt * 1000.0f;
    s_seq ++;
}

/*===================================================================================================================
 *  ★ 工程 10 适配②：器件识别（诊断用）
 *
 *  卡丁从没初始化过 IMU，"板上贴的是哪颗"只能靠读 ID 回答。三个候选的 ID 寄存器不是同一个：
 *      963RA(LSM6DSL) WHO_AM_I @0x0F（期望 0x6A）｜660RA CHIP_ID @0x00 ｜660RC CHIP_ID @0x0F
 *  ⇒ 读这两个地址就够分辨。
 *  ⚠ 裸读**必须在 `imu963ra_init()` 之后**：SPI 与 CS 的 gpio 都在它里面初始化。
 * =================================================================================================================*/
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
#endif  /* 关掉 IMU 时这个函数没人用 ⇒ 一起收起来，否则 Pe177 未引用告警（本工程要求 0/0） */

/* 供 1 Hz 打印用：拿到"读到的两个 ID 字节"。返回时置 valid=1。 */
void ins_imu_dev_id_get (uint8 *id_0f, uint8 *id_00, uint8 *valid)
{
    if (id_0f) { *id_0f = s_id_0f; }
    if (id_00) { *id_00 = s_id_00; }
    if (valid) { *valid = s_id_valid; }
}

/*===================================================================================================================
 *  初始化
 *=================================================================================================================*/
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

    /* ZUPT：上电**先判"静止"**（MM32 同做法）⇒ 标定一结束就能立刻开始跟踪零偏；
       若数据表明车在动，exit（30 ms）会很快把它翻过来。 */
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

    /* ---- ★★ 工程 10 适配②：器件自动探测 + WHO_AM_I 打印 ----
     *
     * 为什么必须探测：卡丁（工程 9）**从来没有初始化过 IMU**，谁也不能打包票板上贴的是哪颗。
     * 逐飞 CYT4BB7 主板的 IMU660RA / IMU660RC / IMU963RA 三个驱动**宏引脚完全相同**
     * （`SPI_2`：P15_2/P15_1/P15_0/CS P15_3），所以"插上就能读"，但**读到的 ID 才作数**。
     *
     * ⚠ 本工程按用户选型**只支持 IMU963RA**（→ 选它就是因为它带磁力计、且工程 8 已在同款板上跑过）。
     *   若 963RA 自检不过，**不要把 660RA/RC 硬当 963RA 用** —— 两者的量程换算系数不同，
     *   硬用会得到"数字在动但量级全错"的假象。这里只**如实报出读到的 ID**，让下一步有依据。
     *   寄存器地址（三个都读一遍才够定位）：
     *      963RA(LSM6DSL 系) WHO_AM_I @ 0x0F（期望 0x6A）
     *      660RA            CHIP_ID  @ 0x00
     *      660RC            CHIP_ID  @ 0x0F
     *   ⚠ SPI 与 CS 的 `gpio_init` 都在 `imu963ra_init()` 里做 ⇒ **裸读必须放在它之后**。 */
#if (0 == INS_IMU_ENABLE)
    /* ★★ A/B 实验：**连 imu963ra_init() 都不调** ⇒ SPI_2(SCB9) 一个寄存器都不碰。
       目的就是回答"转向打死是不是 SPI_2 与 SPI_4 同核冲突造成的"：
       置 0 后故障消失 ⇒ 冲突坐实；故障照旧 ⇒ 与 SPI_2 无关，去查编码器线路。
       语义等同于"IMU 没接"（→ update() 走链路坏分支，绝不碰 SPI），所以能长期这么跑。 */
    s_id_valid = 0u;
    s_flags   |= (INS_ST_WARN | INS_ST_IMU_LINK_BAD);

    printf("[##] INS    IMU DISABLED (INS_IMU_ENABLE=0) - SPI_2/SCB9 NOT initialized [A/B]\r\n");

    return 1;
#else
    uint8 ret;                          /* 只在启用 IMU 的那一支用得到（否则 Pe177 未引用告警） */

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

    /* 姿态复位：累积航向从 0 起（= 车头方向记为"起点朝向"，
       这正是 Polaris 的"起点位姿记录"纪律 —— 路表只用相对量）。 */
    s_yaw_acc = 0.0f;

    return ret;
#endif                                  /* INS_IMU_ENABLE */
}

/*===================================================================================================================
 *  查询 / 命令
 *=================================================================================================================*/
uint8  ins_imu_ready (void)         { return (s_flags & INS_ST_READY) ? 1u : 0u; }
uint16 ins_imu_flags (void)         { return s_flags; }

uint8  ins_imu_link_ok (void)
{
    return (s_flags & INS_ST_IMU_LINK_BAD) ? 0u : 1u;
}

uint8  ins_imu_try_recover (void)
{
#if (0 == INS_IMU_ENABLE)
    /* ★★ A/B 实验期间**必须整只堵住这里**：主循环每秒会调一次 try_recover，
       而它内部会调 imu963ra_init() ⇒ 一旦走到这里，SPI_2/SCB9 还是会被初始化，
       A/B 就白做了（"禁用 IMU"必须严格等于"SPI_2 一个寄存器都不碰"）。 */
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
        /* 工程 10：仍不通 ⇒ 顺手把 ID 再读一次，1 Hz 行里能看到"是不是板子上压根没这颗芯片" */
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
    s_flags &= (uint16)~INS_ST_READY;               /* 还没标定零偏，不算就绪 */
    s_flags |=  INS_ST_WARN;                        /* 提醒：零偏可能是旧值/0 */
    ins_crit_exit(primask);

    return 1u;
#endif
}

float  ins_imu_get_yaw (void)       { return INS_YAW_SIGN * s_yaw_acc; }
uint16 ins_imu_yaw_rej (void)       { return s_yaw_rej; }
float  ins_imu_get_roll (void)      { return s_roll; }
float  ins_imu_get_pitch (void)     { return s_pitch; }
float  ins_imu_get_gyro_z_dps (void){ return s_gz_dps; }

/* ---- 冻结计数器（见 s_frz_* 的注释）------------------------------------------------
   取走即清（和 ins_diag_*_take 一个风格）：1 Hz 行打印的是"这一个窗口内"的冻结量。
   `deg` 是**被丢弃的转角**：判据就是它 —— 接近 0 说明只是停着，明显 > 0 说明真在转被冻了。 */
void ins_imu_zupt_frz_get (uint32 *ticks, float *deg)
{
    uint32 primask = ins_crit_enter();

    if (ticks) { *ticks = s_frz_ticks; }
    if (deg)   { *deg   = s_frz_deg;   }
    s_frz_ticks = 0u;
    s_frz_deg   = 0.0f;

    ins_crit_exit(primask);
}

/* 只读、不清（录表期间想反复看同一次累计值时用） */
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

/*===================================================================================================================
 *  诊断（专治"读不到"）
 *=================================================================================================================*/
volatile uint32 ins_diag_isr_ticks = 0;

/* ★ 2026-10-09 新增：**5ms 中断实际间隔**的 min/max（DWT 实测，单位 µs）。
 *
 *  为什么需要它：编码器测速是 `Δcnt / (cpr × 5ms)` —— **它假设每拍真的是 5.00 ms**。
 *  一旦中断被拖（例如某拍花了 6.5 ms），这一拍的 Δ 会偏大、下一拍偏小，
 *  测速就会"假跌"，速度环立刻补一脚 ⇒ 表现成**周期性的占空比尖峰 + 电机一卡一卡**。
 *  所以先量这个数，再谈是"算法问题"还是"电气/机械问题"：
 *      dtmax 稳定在 ≈5000 µs  ⇒ 中断没抖 ⇒ 测速跌是**真的掉速**（电源/机械/保护）
 *      dtmax 偶发 6000~9000  ⇒ 中断被拖住 ⇒ 测速是**假的**，去查 ISR 里谁在等（SPI 超时？）
 *  （本函数放在 ins_diag_isr_enter() 里，ISR 第一句就调用，量的是"进中断的时刻"间隔。） */
volatile uint32 ins_diag_tick_us_max = 0;
volatile uint32 ins_diag_tick_us_min = 0xFFFFFFFFu;
volatile uint32 ins_diag_tick_long_cnt = 0;   /* 间隔 > 1.5×标称(7.5ms) 的拍数 —— ISR 被拖过的直接证据 */
static   uint32 ins_diag_tick_last_cyc = 0;
static   uint8  ins_diag_tick_first    = 1;

/* 主循环每秒取一次（取走即清，所以它统计的是"这一个 1 秒窗口"内的极值）。
   `long_cnt` 是**累计**（只增不清），它涨得快就说明中断在被反复拖过。 */
void ins_diag_tick_take (uint32 *min_us, uint32 *max_us)
{
    if (min_us) { *min_us = ins_diag_tick_us_min; }
    if (max_us) { *max_us = ins_diag_tick_us_max; }
    ins_diag_tick_us_min = 0xFFFFFFFFu;
    ins_diag_tick_us_max = 0;
}

/* ★ 2026-10-09 再新增：**5ms 中断"体"耗时**的 min/max（µs，DWT 实测）。
 *
 *  为什么还要量这个：上面量的是"**进中断的时刻**间隔"（= 周期是否准），
 *  这里量的是"**这一次中断本身跑了多久**"（= 体有多重）。
 *  两者合起来才能回答"原工程（MM20.zf_clean）为什么没有尖峰"：
 *      · MM32 那边 5ms ISR **只有 `speed_ctrl_update()`**（纯计算），IMU 的 SPI 读在主循环；
 *      · 本工程 ISR 里多了 舵机 + **IMU 的 2~3 次 SPI 事务**（acc+gyro，每 4 拍再 +mag）。
 *  判据：
 *      isr 耗时常态 ≈150~250 µs  ⇒ 轻，只是周期偶尔不准（看 tick 那组）
 *      isr 耗时偶发 > 1000 µs    ⇒ **ISR 里有阻塞**（SPI 忙等超时 `SPI_BUSY_TIMEOUT`？）
 *                                  它同时会把"这一拍变长"直接坐实
 *  ⚠ 只统计中断体的耗时，不含"被更高优先级中断抢占"的时间。 */
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

        ins_diag_isr_t0_cyc = c;            /* ★ 中断体计时的起点 */
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
    uint32 primask = ins_crit_enter();     /* ★ 零偏现在**每拍都会被在线估计改写** ⇒ 必须原子取 */

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
