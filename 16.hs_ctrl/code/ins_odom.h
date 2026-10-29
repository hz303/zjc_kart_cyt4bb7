/*********************************************************************************************************************
 * 航位推算: IMU 航向 × 编码器里程 -> 米制位姿
 *
 *   yaw <- IMU 累积航向(度);  ds <- 左右编码器各自换算成米, 取车体中心
 *   x += -sin(yaw) * ds;  y += cos(yaw) * ds
 * 坐标约定: yaw = 0 时车头朝 +Y, yaw 增大(左转)朝 -X 转; x = 车头朝前时的右手方向, y = 向前。
 *           上位机 screen_x 与 x 同号, screen_y 与 y 反号, 改一处必须同步改上位机。
 * 比例尺: k_m = m_per_rev / cpr; 位置积分用带符号 ds, 总路程另用 dist_abs 累加 |ds|。
 ********************************************************************************************************************/

#ifndef _INS_ODOM_H
#define _INS_ODOM_H

#include "zf_common_headfile.h"

/* ---- 参数(参数表取地址, 见 motor_link.h) ---- */
/* 米/转, 唯一的比例尺参数: 轮子与电机同轴时等于后轮周长, 默认 0.8193 对应轮径 Ø260.8 mm。
   标定: 清零 pos_l/pos_r, 沿直线推车走 2.0~3.0 m, 取计数平均 N, m_per_rev = cpr * 距离 / N。
   cpr_l/cpr_r 改动后本值须按同比例改, 不变量是 m_per_rev / cpr。 */
#define INS_MPM_DEFAULT     (0.8193f)
extern volatile float    ins_m_per_rev;

/* 航位推算使能, 0 = 冻结积分, 只跟踪基准(标定/静止时用) */
extern volatile float    ins_odom_enable;

/* 打滑判据: 左右计数增量之差超过该比例且航向几乎没变, 置打滑标志, 只报警不改数据 */
extern volatile float    ins_slip_pct;

/* 位姿归零命令(参数表 ID 37), 0->1 边沿触发一次, 边沿处理放主循环 */
extern volatile float    ins_zero_cmd;

/* ---- 状态位(遥测 ins_flags 的低 8 位) ---- */
#define ODOM_ST_ACTIVE      (0x01u)     /* 正在积分                       */
#define ODOM_ST_STILL       (0x02u)     /* 连续 N 拍两轮都没动（静止）     */
#define ODOM_ST_SLIP        (0x04u)     /* 本拍判定打滑                   */

/*===================================================================================================================
 *  接口
 *=================================================================================================================*/
void    ins_odom_init       (void);
void    ins_odom_update     (void);         /* 5 ms 每拍: 编码器 Δcnt + yaw -> (x, y) */
void    ins_odom_reset      (void);         /* 位置/里程清零并刷新基准 */
void    ins_odom_set_origin (float x, float y, float yaw_deg);  /* 设当前点为指定位姿 */

float   ins_odom_get_x      (void);
float   ins_odom_get_y      (void);
float   ins_odom_get_yaw    (void);
float   ins_odom_get_dist   (void);         /* 累计路程, 标量, m */
float   ins_odom_get_v_mps  (void);         /* 本拍车速, m/s, 带符号 */
float   ins_odom_get_km     (void);         /* 当前生效的米/计数, 诊断用 */
uint8   ins_odom_flags      (void);

/* 一致位姿快照: 原子取, 避免拿到旧 x + 新 yaw 的撕裂值 */
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
