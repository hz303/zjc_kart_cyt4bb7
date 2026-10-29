/*********************************************************************************************************************
 * 航位推算: IMU 航向 × 编码器里程 → 米制位姿
 *   yaw <- IMU 累积航向(度);  ds <- 左右编码器各自换算成米后取车体中心
 *   x += -sin(yaw) * ds (x 向右);  y += cos(yaw) * ds (y 向前, 即录制起点朝向)
 * 坐标约定: yaw = 0 时车头朝 +Y, yaw 增大(左转)朝 -X 转; 上位机 screen_x ∝ x, screen_y ∝ -y
 * 距离: k_m = m_per_rev / cpr; ds 带符号, 总路程另用 fabsf 累加, 两者不可混用
 ********************************************************************************************************************/

#ifndef _INS_ODOM_H
#define _INS_ODOM_H

#include "zf_common_headfile.h"

/* ---- 参数, 参数表按地址引用, 见 motor_link.h ---- */
/* 米/转, 唯一的比例尺参数; 真实值 = 2π × 轮半径 ÷ 减速比
   标定: 清零后沿直线推车, m_per_rev = 实测距离 × cpr / 累计计数
   前提 cpr = 1858; cpr_l/cpr_r 改动后本值须同比例跟改, 不变量为 mpm/cpr
   取值只影响米这个单位, 尺度错了轨迹形状与航向仍对 */
#define INS_MPM_DEFAULT     (0.8193f)
extern volatile float    ins_m_per_rev;

/* 航位推算使能, 0 = 冻结积分只跟踪基准, 标定或静止时用 */
extern volatile float    ins_odom_enable;

/* 打滑判据: 左右计数增量之差超过该比例且航向几乎没变, 置打滑标志, 只报警不改数据 */
extern volatile float    ins_slip_pct;

/* 位姿归零命令, 0->1 边沿触发一次: ins_odom_reset() + ins_imu_zero_yaw()
   位姿与航向必须一起归零; 边沿处理放主循环, 不放 5 ms 中断 */
extern volatile float    ins_zero_cmd;

/* ---- 状态位, 遥测 ins_flags 低 8 位 ---- */
#define ODOM_ST_ACTIVE      (0x01u)     /* 正在积分                       */
#define ODOM_ST_STILL       (0x02u)     /* 连续 N 拍两轮都没动（静止）     */
#define ODOM_ST_SLIP        (0x04u)     /* 本拍判定打滑                   */

/* ---- 接口 ---- */
void    ins_odom_init       (void);
void    ins_odom_update     (void);         /* 5 ms 每拍: 编码器 Δcnt + yaw → (x,y) */
void    ins_odom_reset      (void);         /* 位置/里程清零并刷新基准 */
void    ins_odom_set_origin (float x, float y, float yaw_deg);  /* 设当前点为指定位姿 */

float   ins_odom_get_x      (void);
float   ins_odom_get_y      (void);
float   ins_odom_get_yaw    (void);
float   ins_odom_get_dist   (void);         /* 累计路程, 标量, m */
float   ins_odom_get_v_mps  (void);         /* 本拍车速, m/s, 带符号 */
float   ins_odom_get_km     (void);         /* 当前生效的米/计数, 诊断用 */
uint8   ins_odom_flags      (void);

/* 一致位姿快照, 原子复制, 避免主循环读到半拍数据 */
typedef struct
{
    float x;            /* 米, 向右 */
    float y;            /* 米, 向前 */
    float yaw;          /* 度, 相对起点, 累积可超 ±360 */
    float dist_abs;     /* 累计路程, 标量, m */
    float v_mps;        /* 本拍车速, m/s, 带符号 */
    uint8 flags;        /* ODOM_ST_* */
} ins_odom_snap_t;

void    ins_odom_snapshot   (ins_odom_snap_t *out);

#endif
