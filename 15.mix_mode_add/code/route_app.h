/*********************************************************************************************************************
 * 路表应用层：把录制/存取模块与上层动作粘起来
 * ins_route.c 录/存/载（Flash 多槽，16 B/条）；route_link.c 串口灌表/拉表；ins_odom.c 航位推算
 * 职责：上电读槽索引并载入 active 槽；主循环处理命令边沿并驱动各模块 poll；对遥测/菜单暴露位图状态
 * 两种来源：手工录制（rt_rec 0->1 开始，rt_mark 打点并记舵角，1->0 停止并自动落槽 0）
 *           电子制图（PC 生成镜像经串口灌入槽 1/2；协议层禁止写槽 0）
 ********************************************************************************************************************/

#ifndef _ROUTE_APP_H
#define _ROUTE_APP_H

#include "zf_common_headfile.h"

#define RT_MODE_ZZ       (0u)
#define RT_MODE_ZF       (1u)
#define RT_MODE_FZ       (2u)
#define RT_MODE_MIX_S    (3u)
#define RT_MODE_MIX_E    (4u)
#define RT_MODE_COUNT    (5u)
/* ID88 路线模式 RT_MODE_*；负值保留 */
extern volatile float rt_route_mode;

/* ---- 命令量（参数表 ID 38/40/41/42/53）；均为 0->1 边沿，不参与参数恢复 ---- */
extern volatile float rt_rec_cmd;       /* 0->1 开始录制 / 1->0 停止并落槽 0    */
extern volatile float rt_slot_cmd;      /* 目标槽 0..4；改了就切换并载入        */
extern volatile float rt_load_cmd;      /* 0->1 重新载入当前槽                  */
extern volatile float rt_save_cmd;      /* 0->1 把 RAM 里的表存进当前槽         */
extern volatile float rt_mark_cmd;      /* 0->1 给最后一个已录点打关键点        */

/* ---- 二值开关（设置项，会持久化）---- */
extern volatile float rt_revrec;        /* ID76 0/1 倒车录表：录制时 v 存 |v|（保留 REV 标记）*/
extern volatile float rt_revplay;       /* ID77 0/1 倒序回放：回放时把点序倒过来       */
extern volatile float pb_drev;        /* ID85 行驶方向 0=正车 1=倒车 */
extern volatile float pb_rvk;         /* ID86 倒车开环航向 Kp */
extern volatile float pb_ctrlmode;     /* ID78 回放控制律选择 0..5（菜单 2.CtrlMode）*/
extern volatile float pb_cauto;       /* ID87 回放自动选控制律 0=手动 1=自动 */
/* pb_ctrlmode：0 = 无 PP（录制打角 + 航向 P）/ 1 = 纯追踪（默认）/ 2~5 = LQR 变体（基础/前馈/+ESO/监督器）*/

/* ---- LQR 族可调参数（ID79~84）---- */
extern volatile float ctrl_qe;      /* 横向误差权重   */
extern volatile float ctrl_qth;     /* 航向误差权重   */
extern volatile float ctrl_r;       /* 打角权重       */
extern volatile float ctrl_tlead;   /* 曲率前馈预览 s */
extern volatile float ctrl_kusff;   /* 保留，无人读取 */
extern volatile float ctrl_esow;    /* 保留，无人读取 */

/* 状态位（遥测 CH46 rt_st 的低 8 位；bit8..11 = 当前槽号）
 * 0x01 录制中 / 0x02 录满（点距过大或赛道过长）/ 0x04 RAM 里有表 / 0x08 正在分步写 Flash
 * 0x10 上次写盘失败 / 0x20 正在回放 / 0x40 当前表上电从 Flash 载入 / 0x80 串口正在灌表 */
#define RT_ST_REC       (0x01u)
#define RT_ST_FULL      (0x02u)
#define RT_ST_HAVE      (0x04u)
#define RT_ST_SAVING    (0x08u)
#define RT_ST_SAVEFAIL  (0x10u)
#define RT_ST_PLAY      (0x20u)
#define RT_ST_FROMFLSH  (0x40u)
#define RT_ST_LINK      (0x80u)

/* 录制点数少于它不落盘，防止误按把槽 0 覆盖成垃圾 */
#define RT_MIN_SAVE_POINTS  (20u)

void        route_app_init  (void);     /* 上电一次，必须在 5 ms 中断打开之前    */
void        route_app_poll  (void);     /* 主循环每圈一次（幂等，空闲开销近零）  */
uint16      route_app_count (void);     /* 当前 RAM 表点数                       */
uint16      route_app_status(void);     /* 位图 | (槽号 << 8)                    */
const char *route_app_str   (void);     /* 一行摘要（屏幕/日志）                 */
uint8       route_app_slot_next (void); /* 菜单用：切到下一槽并载入（返回新槽号） */

#endif
