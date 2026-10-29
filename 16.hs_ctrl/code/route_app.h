/* 路表应用层：上电读槽索引 -> 补空槽内建表 -> 载入 active 槽；主循环处理参数边沿；对外暴露位图状态
 * 模块分工：ins_route.c 录/存/载（Flash 多槽，16 B/条）；route_link.c 串口灌表/拉表；ins_odom.c 航位推算
 * 两种路表来源：手工（rt_rec 录制 + rt_mark 打点，停止后自动落槽 0）、电子制图（PC 规划后经串口灌入槽 1/2）*/

#ifndef _ROUTE_APP_H
#define _ROUTE_APP_H

#include "zf_common_headfile.h"

#define RT_MODE_ZZ       (0u)
#define RT_MODE_ZF       (1u)
#define RT_MODE_FZ       (2u)
#define RT_MODE_MIX_S    (3u)
#define RT_MODE_MIX_E    (4u)
#define RT_MODE_COUNT    (5u)
extern volatile float rt_route_mode;    /* ID88, 负值保留给旧上位机会话 */

extern volatile float rt_rec_cmd;       /* ID38 0->1 开始录制 / 1->0 停止并落槽 0 */
extern volatile float rt_slot_cmd;      /* ID40 目标槽 0..4；改了就切并载入      */
extern volatile float rt_load_cmd;      /* ID41 0->1 重新载入当前槽              */
extern volatile float rt_save_cmd;      /* ID42 0->1 把 RAM 里的表存进当前槽      */
extern volatile float rt_mark_cmd;      /* ID53 0->1 给最后一个已录点打关键点     */

/* 下面这些是设置项（非命令，会持久化）；命令量不恢复，否则下次上电会自行触发 */
extern volatile float rt_revrec;        /* ID76 0/1 倒车录表：录制时 v 存 |v|（保留 REV 标记）*/
extern volatile float rt_revplay;       /* ID77 0/1 倒序回放：回放时把点序倒过来       */
extern volatile float pb_drev;          /* ID85 行驶方向 0=正车 1=倒车 */
extern volatile float pb_rvk;           /* ID86 倒车开环航向 Kp */
extern volatile float pb_ctrlmode;      /* ID78 回放控制律 0=无 PP / 1=纯追踪 / 2..5=LQR 变体（基础, 前馈, +ESO, +监督器）*/
extern volatile float pb_cauto;         /* ID87 自动选控制律 0=手动 1=自动 */

/* LQR 可调参数：ID79 横向误差权重 | ID80 航向误差权重 | ID81 打角权重 | ID82 曲率前馈预览 s
 * ID83 / ID84 已废弃，无人读取 */
extern volatile float ctrl_qe;
extern volatile float ctrl_qth;
extern volatile float ctrl_r;
extern volatile float ctrl_tlead;
extern volatile float ctrl_kusff;
extern volatile float ctrl_esow;

/* ---- 高速 LQR（HS）结构（ID89~93）：ctrl_hs 为位掩码，0 = 完全走原 LQR
 * bit0(1) 带宽归一化 Ke=(wn/v)², Kth=2ζ·wn/v | bit1(2) 弧长域延迟预测
 * bit2(4) 连续曲率 κ(s) 插值 | bit3(8) 预览距离取实测车速 | bit4(16) 用上一拍指令曲率 */
extern volatile float ctrl_hs;      /* ID89 结构掩码, 默认 15.0 */
extern volatile float ctrl_hs_wn;   /* ID90 闭环自然频率 rad/s */
extern volatile float ctrl_hs_zeta; /* ID91 闭环阻尼比 */
extern volatile float ctrl_hs_lag;  /* ID92 执行器等效延迟 s */
extern volatile float ctrl_hs_tp;   /* ID93 总预览时间 s（含延迟）*/

/* ---- 状态位（遥测 CH46 rt_st 的低 8 位, bit8..11 = 当前槽号）
 * 0x01 录制中 | 0x02 录满 | 0x04 有表 | 0x08 写盘中 | 0x10 写盘失败 | 0x20 回放中 | 0x40 上电载入 | 0x80 灌表/拉表 */
#define RT_ST_REC       (0x01u)
#define RT_ST_FULL      (0x02u)
#define RT_ST_HAVE      (0x04u)
#define RT_ST_SAVING    (0x08u)
#define RT_ST_SAVEFAIL  (0x10u)
#define RT_ST_PLAY      (0x20u)
#define RT_ST_FROMFLSH  (0x40u)
#define RT_ST_LINK      (0x80u)

#define RT_MIN_SAVE_POINTS  (20u)

void        route_app_init  (void);     /* 上电一次，必须在 5 ms 中断打开之前 */
void        route_app_poll  (void);     /* 主循环每圈一次（幂等）             */
uint16      route_app_count (void);
uint16      route_app_status(void);
const char *route_app_str   (void);     /* 一行摘要（屏幕/日志）             */
uint8       route_app_slot_next (void); /* 切到下一槽并载入（返回新槽号）      */

#endif
