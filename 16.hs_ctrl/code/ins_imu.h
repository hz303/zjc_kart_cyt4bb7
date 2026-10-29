/*********************************************************************************************************************
 * 惯导 + 路表 - IMU 姿态与航向（IMU963RA + Mahony 九轴 AHRS）
 *
 * 传感器: IMU963RA(加计 + 陀螺 + 磁力计), 磁力计使 yaw 可观测。
 * 节拍: ins_imu_update() 在 5 ms PIT 中断内调用(200 Hz), 全工程只有它访问 IMU 的 SPI;
 *       主循环取数据走 ins_imu_snapshot()。航向取 ZYX 欧拉 yaw(度), 累积可超 ±360。
 ********************************************************************************************************************/

#ifndef _INS_IMU_H
#define _INS_IMU_H

#include "zf_common_headfile.h"

/* ---- 一、配置 ---- */
/* A/B 实验开关: 0 = 完全不初始化 IMU(imu963ra_init() 都不调, SPI_2/SCB9 一个寄存器都不碰,
   语义等同 IMU 没接); 1 = 正常。用于判定 SPI_2 与 SPI_4 同核共存是否冲突。 */
#define INS_IMU_ENABLE              (1)             /* 0 = 不初始化 IMU（A/B 实验用） */

#define INS_SAMPLE_HZ               (200)           /* 与 5 ms PIT 一致 */
#define INS_T_S                     (0.005f)        /* 标称采样周期, 秒 */

/* 陀螺死区, LSB: 1 LSB = 0.0699 dps(±2000 dps 量程), 0.5 LSB ≈ 0.035 dps, 只削量化底噪 */
#define INS_GYRO_DEADBAND_LSB       (0.5f)

/* 磁力计读取分频: 每 N 拍读一次(200/N Hz); 走 sensor hub, 慢变量, 50 Hz 足够 */
#define INS_MAG_DIV                 (4)

/* yaw 符号: 车头向左转(俯视逆时针)时 yaw 增大取 +1; 装反或芯片朝下取 -1 */
#define INS_YAW_SIGN                (+1.0f)

/* 上电陀螺标定 */
#define INS_CALIB_SAMPLES           (1000)          /* 5 ms 一拍, 1000 点 = 5 s */
#define INS_CALIB_MIN_SAMPLES       (300)
#define INS_CALIB_MAX_RETRY         (5)
/* 采样期内单轴极差(raw LSB)超过它判为被碰动, 重来; 300 LSB ≈ 21 dps 峰峰, 正常静止远小于它 */
#define INS_CALIB_MAX_SPAN_LSB      (300)
/* 标定结果超限判为异常(传感器坏/线松) */
#define INS_BIAS_LIMIT_LSB          (400.0f)

/* Mahony 增益: Kp 比例项(收敛速度), Ki 积分项(消除稳态误差); quick_* 是前 200 拍的快收敛档 */
#define INS_MAHONY_KP               (10.0f)
#define INS_MAHONY_KI               (0.005f)
#define INS_MAHONY_QUICK_KP         (30.0f)
#define INS_MAHONY_QUICK_KI         (0.005f)
#define INS_MAHONY_QUICK_TICKS      (200)

/* ---- ZUPT 静止检测 + 在线零偏估计: 六轴下 yaw 不可观测, 零偏随温度漂移, 只能在静止时观测 ----
 *   判据① 三轴角速度最大值 < INS_ZUPT_GYRO_LIMIT_DPS
 *   判据② | |a| - 1g | < INS_ZUPT_ACC_TOL_G
 *   判据③ 去重力后水平加速度 < INS_ZUPT_HORIZ_TOL_G */
#define INS_ZUPT_GYRO_LIMIT_DPS     (2.5f)      /* 单轴角速度上限(三轴取最大) */
#define INS_ZUPT_ACC_TOL_G          (0.08f)     /* | |a| - 1g | 上限(0.08 g ≈ 0.78 m/s²) */
#define INS_ZUPT_HORIZ_TOL_G        (0.06f)     /* 去重力后水平加速度上限(0.06 g ≈ 0.59 m/s²) */
#define INS_ZUPT_WHEEL_EPS          (0.08f)    /* 轮速互斥门限 m/s(约 0.1 rps), 超过它不判静止 */
#define INS_ZUPT_ENTER_MS           (150u)      /* 连续满足多久判静止(防抖) */
#define INS_ZUPT_EXIT_MS            (30u)       /* 连续不满足多久判运动(比进入短, 响应灵敏) */

/* 静止时是否冻结航向(Z 轴角速度置 0): 1 = 冻结(默认), 静止时角速度只剩零偏残差;
   0 = 只靠零偏估计修正。判据取三轴最大值, 绕水平轴晃动不会误判成静止。 */
#define INS_ZUPT_FREEZE_YAW         (1)

/* ---- 在线零偏估计: 只在确实静止的窗口里开 ----
 * 门控用瞬时条件连续满足 INS_BIAS_GATE_MS, 不用去抖后的 static 值(其退出有 30 ms 滞后, 会吃掉真实转速) */
#define INS_BIAS_GATE_MS            (50u)       /* 瞬时条件连续满足这么久才允许动零偏 */
#define INS_BIAS_TAU_MS             (2000.0f)   /* 一阶时间常数：静止 2 s 衰减到 37%，6 s 到 5% */

/* ---- 二、状态位(遥测 ins_flags 的高 8 位, 低 8 位留给航位推算) ---- */
#define INS_ST_READY            (0x0001u)   /* 已上电标定完成并在运行   */
#define INS_ST_WARN             (0x0002u)   /* 有告警（见下面 detail）  */
#define INS_ST_MAG_CAL_DONE     (0x0004u)   /* 磁力计已标定（磁修正生效）*/
#define INS_ST_MAG_CAL_RUN      (0x0008u)   /* 磁力计标定进行中         */
#define INS_ST_IMU_LINK_BAD     (0x0010u)   /* IMU 通信异常             */
#define INS_ST_GYRO_SAT         (0x0020u)   /* 陀螺饱和                 */
#define INS_ST_DT_BAD           (0x0040u)   /* dt 异常（被钳过）        */
/* 与 DT_BAD 分开: 这只表示 DWT 没起来、dt 用标称值, 不是测到异常 */
#define INS_ST_DT_NODWT         (0x0080u)   /* DWT 不可用，dt = 标称周期 */

/* ZUPT 状态: 信息位, 不是告警, 不参与 INS_ST_WARN 判据 */
#define INS_ST_STATIC           (0x0100u)   /* 当前判定为静止（ZUPT 通过）      */
#define INS_ST_BIAS_TRK         (0x0200u)   /* 本拍在跟踪零偏（门控通过）        */

/* ---- 三、快照(主循环用; 中断里更新, 一次取全, 避免撕裂) ---- */
typedef struct
{
    float   yaw_acc;        /* 累积航向, 度, 可超 ±360 */
    float   yaw_raw;        /* Mahony 输出, -180~180, 诊断用 */
    float   roll;           /* 横滚, 度 */
    float   pitch;          /* 俯仰, 度 */
    float   gyro_z_dps;     /* 去零偏、过死区后的 Z 轴角速度, °/s */
    float   dt_ms;          /* 本拍实测 dt, ms */
    float   acc_norm_g;     /* |a|, g, 静止时约 1 */
    float   horiz_acc_g;    /* 去重力后水平加速度, g, 静止时约 0 */
    uint8   is_static;      /* ZUPT 结果: 1 = 判为静止(yaw 已冻结) */
    uint8   bias_trk;       /* 本拍是否在跟踪零偏 */
    uint16  flags;          /* INS_ST_* */
    uint16  seq;            /* 采样序号, 应匀速增长 */
} ins_imu_snap_t;

/* ---- 四、接口 ---- */
/* 上电初始化: DWT 时间源 + IMU963RA + 陀螺零偏标定(阻塞, 要求车静止) + 姿态复位;
   返回 0 = 成功, 1 = 器件初始化失败, 2 = 标定结果超限 */
uint8   ins_imu_init            (void);

/* 5 ms 每拍: 读 IMU -> 去零偏/死区 -> Mahony 九轴 -> 更新累积 yaw */
void    ins_imu_update          (void);

uint8   ins_imu_ready           (void);
uint16  ins_imu_flags           (void);

/* 器件链路健康, 1 = 通信正常; 为 0 时 ins_imu_update() 完全不碰 SPI_2 */
uint8   ins_imu_link_ok         (void);

/* 链路坏时从主循环调(约 1 Hz): 重试一次器件初始化, 返回 1 = 刚恢复; 此刻中断不碰 SPI_2, 无竞争 */
uint8   ins_imu_try_recover     (void);

/* 器件识别结果(诊断用, 给 1 Hz 打印): 963RA WHO_AM_I @0x0F(期望 0x6A), 660RA CHIP_ID @0x00,
   660RC CHIP_ID @0x0F; valid = 0 表示还没探测过 */
void    ins_imu_dev_id_get      (uint8 *id_0f, uint8 *id_00, uint8 *valid);

/* 累积航向, 度, 可超 ±360 */
float   ins_imu_get_yaw         (void);
uint16  ins_imu_yaw_rej         (void);   /* 被单拍合理性闸门丢掉的拍数, 非 0 说明航向有毛刺 */
float   ins_imu_get_roll        (void);
float   ins_imu_get_pitch       (void);
float   ins_imu_get_gyro_z_dps  (void);

/* 把累积 yaw 归零(不改姿态四元数) / 直接设定累积 yaw */
void    ins_imu_zero_yaw        (void);
void    ins_imu_set_yaw         (float deg);

/* 重做陀螺零偏标定(要求静止) */
uint8   ins_imu_calib_gyro      (void);

/* ---- 冻结计数器: 统计 ZUPT 冻结期间被丢弃的转角 ----
 *   ticks = 冻结拍数(×5 ms = 冻结时长), deg = 冻结期间被丢弃的 |gyro| 积分(度);
 *   deg 明显大于 0 说明冻结时确实在转。_get 取走即清, _ticks/_deg 只读不清 */
void    ins_imu_zupt_frz_get    (uint32 *ticks, float *deg);
uint32  ins_imu_zupt_frz_ticks  (void);
float   ins_imu_zupt_frz_deg    (void);
void    ins_imu_zupt_frz_reset  (void);

/* 磁力计 Min-Max 标定: start 后手动绕各轴各转一圈, 再 stop 结算; 标定后磁修正才生效 */
void    ins_imu_mag_calib_start (void);
void    ins_imu_mag_calib_stop  (void);

/* 取一份一致的快照(内部关中断复制, 避免主循环读到半拍数据) */
void    ins_imu_snapshot        (ins_imu_snap_t *out);

/* ---- 五、诊断(上电横幅与 1 Hz 诊断行用) ---- */
/* 每进一次 5 ms 中断就 +1, 用于判断中断有没有跑: 不涨则编码器/IMU 都读不到, 问题在 PIT 或中断使能 */
extern volatile uint32 ins_diag_isr_ticks;
void    ins_diag_isr_enter     (void);

/* 5 ms 中断实际间隔的 min/max(µs, DWT 实测, 每秒取走一次并清零): 稳定约 5000 µs 为正常, 偶发 6000~9000 = 被拖住 */
void    ins_diag_tick_take     (uint32 *min_us, uint32 *max_us);

/* 间隔 > 7.5 ms 的累计拍数(只增不清), 持续增长说明中断被反复拖过 */
extern volatile uint32 ins_diag_tick_long_cnt;

/* 5 ms 中断体耗时的 min/max(µs, DWT 实测): 常态约 150~250 µs, 偶发 > 1000 µs 说明 ISR 里有阻塞;
   在 cm7_0_isr.c 的 pit0_ch2_isr() 最后一句调用 */
void    ins_diag_isr_leave     (void);
void    ins_diag_isr_cost_take (uint32 *min_us, uint32 *max_us);

/* 最近一拍的 IMU 原始值(LSB): 全是 0 或全是 ±32767/±1 说明 IMU 没通(SPI/CS/供电) */
void    ins_diag_get_imu_raw   (int16 *ax, int16 *ay, int16 *az,
                                int16 *gx, int16 *gy, int16 *gz,
                                int16 *mx, int16 *my, int16 *mz);

/* 陀螺零偏标定结果(LSB), 诊断用 */
void    ins_diag_get_bias      (float *bx, float *by, float *bz);

#endif
