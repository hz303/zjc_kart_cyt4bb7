/*********************************************************************************************************************
 * 惯导 - IMU 姿态与航向（IMU963RA + Mahony 九轴 AHRS）
 * 采样 dt 由 DWT 实测, 异常值钳回标称周期 INS_T_S; 累积航向 yaw_acc 逐拍累加, 可超 ±360°
 * 节拍: ins_imu_update() 在 5 ms PIT 中断, 200 Hz
 * 全工程只有该函数访问 IMU 的 SPI, 总线不可重入; 主循环一律走 ins_imu_snapshot()
 ********************************************************************************************************************/

#ifndef _INS_IMU_H
#define _INS_IMU_H

#include "zf_common_headfile.h"

/* ---- 一、配置 ---- */
/* ---- A/B 实验开关 ----
 * 0 = 完全不初始化 IMU (imu963ra_init() 不调, SPI_2 一个寄存器都不碰), 1 = 正常
 * 用于判定转向故障是否来自 SPI_2 与 SPI_4 同核共存; 置 0 时 update() 走链路坏分支 */
#define INS_IMU_ENABLE              (1)             /* 0 = 不初始化 IMU */

#define INS_SAMPLE_HZ               (200)           /* 与 5ms PIT 一致 */
#define INS_T_S                     (0.005f)        /* 采样周期, s */

/* 陀螺死区, LSB; 默认 0.5 LSB ≈ 0.035 dps, 只削量化底噪(1 LSB = 0.0699 dps) */
#define INS_GYRO_DEADBAND_LSB       (0.5f)

/* 磁力计读取分频: 每 N 拍读一次(200/N Hz), sensor hub 为慢变量 */
#define INS_MAG_DIV                 (4)

/* yaw 符号: 车头左转(俯视逆时针)时 yaw 增大取 +1; 装反或芯片朝下取 -1 */
#define INS_YAW_SIGN                (+1.0f)

/* 上电陀螺标定 */
#define INS_CALIB_SAMPLES           (1000)          /* 5ms 一拍 ⇒ 1000 点 = 5 s */
#define INS_CALIB_MIN_SAMPLES       (300)
#define INS_CALIB_MAX_RETRY         (5)
/* 采样期内单轴极差(raw LSB)上限, 超过判被碰过并重来; 300 LSB ≈ 21 dps 峰峰 */
#define INS_CALIB_MAX_SPAN_LSB      (300)
/* 标定结果超限判为异常(传感器坏或线松) */
#define INS_BIAS_LIMIT_LSB          (400.0f)

/* Mahony 增益; quick_* 为启动期快收敛档, 前 INS_MAHONY_QUICK_TICKS 拍使用 */
#define INS_MAHONY_KP               (10.0f)         /* 比例增益: 加计/磁误差对陀螺的修正权重 */
#define INS_MAHONY_KI               (0.005f)        /* 积分增益: 累积误差用于估计陀螺零偏 */
#define INS_MAHONY_QUICK_KP         (30.0f)
#define INS_MAHONY_QUICK_KI         (0.005f)
#define INS_MAHONY_QUICK_TICKS      (200)

/* ---- 一·b、ZUPT 静止检测 + 在线零偏估计: 静止窗口是唯一能观测零偏的时机 ---- */
/* ---- 静止判据：三轴陀螺 + 加计模长 + 去重力后的水平加速度，三者同时满足 ---- */
#define INS_ZUPT_GYRO_LIMIT_DPS     (2.5f)      /* 单轴角速度上限（三轴取最大） */
#define INS_ZUPT_ACC_TOL_G          (0.08f)     /* | |a| - 1g | 上限（0.08 g ≈ 0.78 m/s²） */
#define INS_ZUPT_HORIZ_TOL_G        (0.06f)     /* 去重力后水平加速度上限（0.06 g ≈ 0.59 m/s²） */
#define INS_ZUPT_WHEEL_EPS          (0.08f)    /* 轮速互斥门限 m/s（≈0.1 rps）：超过它就不许判静止 */
#define INS_ZUPT_ENTER_MS           (150u)      /* 连续满足多久判静止（防抖） */
#define INS_ZUPT_EXIT_MS            (30u)       /* 连续不满足多久判运动（比进短得多 ⇒ 响应灵敏） */

/* 静止时是否冻结航向: 1 = 冻结(默认), Z 轴角速度置 0; 0 = 只靠零偏估计修正 */
#define INS_ZUPT_FREEZE_YAW         (1)

/* ---- 在线零偏估计, 只在确认静止的窗口内开启 ----
 * 门控必须用瞬时条件连续满足 INS_BIAS_GATE_MS, 不能用去抖后的 static 值:
 * static 退出有 30 ms 滞后, 用它当门控会把真实转速当成零偏吃进去 */
#define INS_BIAS_GATE_MS            (50u)       /* 瞬时条件连续满足这么久才允许动零偏 */
#define INS_BIAS_TAU_MS             (2000.0f)   /* 一阶时间常数：静止 2 s 衰减到 37%，6 s 到 5% */

/* ---- 二、状态位, 遥测 ins_flags 高 8 位; 低 8 位留给航位推算 ---- */
#define INS_ST_READY            (0x0001u)   /* 已上电标定完成并在运行   */
#define INS_ST_WARN             (0x0002u)   /* 有告警（见下面 detail）  */
#define INS_ST_MAG_CAL_DONE     (0x0004u)   /* 磁力计已标定（磁修正生效）*/
#define INS_ST_MAG_CAL_RUN      (0x0008u)   /* 磁力计标定进行中         */
#define INS_ST_IMU_LINK_BAD     (0x0010u)   /* IMU 通信异常             */
#define INS_ST_GYRO_SAT         (0x0020u)   /* 陀螺饱和                 */
#define INS_ST_DT_BAD           (0x0040u)   /* dt 异常（被钳过）        */
/* 与 DT_BAD 区分: 仅表示 DWT 不可用、dt 用标称周期, 不是测到异常 */
#define INS_ST_DT_NODWT         (0x0080u)   /* DWT 不可用，dt = 标称周期 */

/* ZUPT 状态位, 信息位, 不参与 INS_ST_WARN 判据 */
#define INS_ST_STATIC           (0x0100u)   /* 当前判定为静止（ZUPT 通过）      */
#define INS_ST_BIAS_TRK         (0x0200u)   /* 本拍在跟踪零偏（门控通过）        */

/* ---- 三、快照, 中断里更新, 主循环一次取全 ---- */
typedef struct
{
    float   yaw_acc;        /* 累积航向, 度, 可超 ±360 */
    float   yaw_raw;        /* Mahony 输出的 -180~180, 诊断用 */
    float   roll;           /* 横滚, 度 */
    float   pitch;          /* 俯仰, 度 */
    float   gyro_z_dps;     /* 去零偏、过死区后的 Z 轴角速度, °/s */
    float   dt_ms;          /* 本拍实测 dt, ms */
    float   acc_norm_g;     /* |a|, g, 静止时应 ≈1 */
    float   horiz_acc_g;    /* 去重力后水平加速度, g, 静止时应 ≈0 */
    uint8   is_static;      /* ZUPT 结果, 1 = 判为静止(yaw 已冻结) */
    uint8   bias_trk;       /* 本拍在跟踪零偏 */
    uint16  flags;          /* INS_ST_* */
    uint16  seq;            /* 采样序号, 应匀速增长 */
} ins_imu_snap_t;

/* ---- 四、接口 ---- */
/* 上电初始化: DWT 时间源 + IMU963RA + 陀螺零偏标定 + 姿态复位; 标定阻塞, 要求车静止
   返回 0 = 成功, 1 = IMU 器件初始化失败, 2 = 标定失败(结果超限) */
uint8   ins_imu_init            (void);

/* 5 ms 每拍: 读 IMU → 去零偏/死区 → Mahony 九轴 → 更新累积 yaw */
void    ins_imu_update          (void);

uint8   ins_imu_ready           (void);
uint16  ins_imu_flags           (void);

/* 器件链路健康, 1 = 通信正常; 为 0 时 ins_imu_update() 完全不碰 SPI_2 */
uint8   ins_imu_link_ok         (void);

/* 链路坏时从主循环调用(约 1 Hz), 重试一次器件初始化, 返回 1 = 刚恢复 */
uint8   ins_imu_try_recover     (void);

/* 器件识别结果, 诊断用; 963RA(LSM6DSL) WHO_AM_I @0x0F 期望 0x6A,
 * 660RA CHIP_ID @0x00, 660RC CHIP_ID @0x0F; valid 为 0 表示尚未探测 */
void    ins_imu_dev_id_get      (uint8 *id_0f, uint8 *id_00, uint8 *valid);

/* 累积航向, 度, 可超 ±360; 惯导唯一需要的 IMU 输出 */
float   ins_imu_get_yaw         (void);
uint16  ins_imu_yaw_rej         (void);   /* 被单拍合理性闸门丢弃的拍数, 非 0 表示航向有毛刺 */
float   ins_imu_get_roll        (void);
float   ins_imu_get_pitch       (void);
float   ins_imu_get_gyro_z_dps  (void);

/* 把累积 yaw 归零（不改姿态四元数） / 直接设定累积 yaw */
void    ins_imu_zero_yaw        (void);
void    ins_imu_set_yaw         (float deg);

/* 重做陀螺零偏标定（要求静止） */
uint8   ins_imu_calib_gyro      (void);

/* ---- 冻结计数器: 统计 ZUPT 冻结期间丢掉的转角 ----
 * ticks 冻结拍数(×5 ms = 冻结时长), deg 冻结期间 |gyro| 积分(度)
 * deg 明显 > 0 表示真在转却被冻结; _get 取走即清, _ticks/_deg 只读不清 */
void    ins_imu_zupt_frz_get    (uint32 *ticks, float *deg);
uint32  ins_imu_zupt_frz_ticks  (void);
float   ins_imu_zupt_frz_deg    (void);
void    ins_imu_zupt_frz_reset  (void);

/* 磁力计 Min-Max 标定: start 后手动绕各轴各转一圈, 再 stop 结算; 未标定时跳过磁修正 */
void    ins_imu_mag_calib_start (void);
void    ins_imu_mag_calib_stop  (void);

/* 取一份一致的快照, 内部关中断复制, 避免主循环读到半拍数据 */
void    ins_imu_snapshot        (ins_imu_snap_t *out);

/* ---- 五、诊断 ---- */
/* 每进一次 5 ms 中断 +1, 用于判断中断是否在运行; 不增长则问题在 PIT 或中断使能 */
extern volatile uint32 ins_diag_isr_ticks;
void    ins_diag_isr_enter     (void);

/* 5 ms 中断实际间隔的 min/max, µs, DWT 实测, 主循环每秒取走并清零
   稳定 ≈5000 µs 为正常; 偶发 6000~9000 表示中断被拖, 此时编码器测速偏小 */
void    ins_diag_tick_take     (uint32 *min_us, uint32 *max_us);

/* 间隔 > 7.5 ms 的累计拍数, 只增不清; 持续增长表示中断被反复拖过 */
extern volatile uint32 ins_diag_tick_long_cnt;

/* 5 ms 中断体耗时的 min/max, µs, DWT 实测; 常态 150~250 µs, 偶发 > 1000 µs 表示 ISR 内有阻塞
   在 pit0_ch2_isr() 最后一句调用 ins_diag_isr_leave() */
void    ins_diag_isr_leave     (void);
void    ins_diag_isr_cost_take (uint32 *min_us, uint32 *max_us);

/* 最近一拍的 IMU 原始值, LSB; 全为 0 或 ±32767 表示 IMU 未通 */
void    ins_diag_get_imu_raw   (int16 *ax, int16 *ay, int16 *az,
                                int16 *gx, int16 *gy, int16 *gz,
                                int16 *mx, int16 *my, int16 *mz);

/* 陀螺零偏标定结果, LSB, 诊断用 */
void    ins_diag_get_bias      (float *bx, float *by, float *bz);

#endif
