/* 工程 10（卡丁）· 路表应用层（P1）
 * 作用：把"模块"和"现场动作"粘起来。三块东西各自独立：
 * `ins_route.c`   —— 录 / 存 / 载（Flash 多槽，**16 B/条**）+ 内建表注册表
 * `route_link.c`  —— PC 经 DAP 虚拟串口灌表 / 拉表（"电子制图"那条路）
 * `ins_odom.c`    —— 航位推算（ins_route 的输入）
 * 本文件只做三件事：
 * ① 上电：读槽索引 → 补空槽的内建表 → 载入 active 槽
 * ② 主循环：参数边沿（录制/选槽/载入/存盘/打点）→ 调三块模块的 poll
 * ③ 对遥测/菜单暴露一个**位图状态**
 * `rt_mark` 打关键点（**逐点同时记下舵机打角**）→ 1→0 停止并**自动落槽 0**。
 * 回放时 `pb_ff_en=1` ⇒ 用表里的打角当前馈，PP 只做修正。
 * 或 ASC 图那条 `asc_to_route.py → route_reroot.py → route_to_carbin.py`，
 * 然后用 `route_send.py`（或上位机「路表管理」）经串口灌进槽 1/2。
 * `rt_rec` / `rt_load` / `rt_save` / `rt_mark` 都是 0→1 边沿命令；持久化它们会在
 * **下次上电自己触发一次**。见 `app_params.c` 的 `ap_is_cmd()`（ID 38/41/42/53 已登记）。
 */

#ifndef _ROUTE_APP_H
#define _ROUTE_APP_H

#include "zf_common_headfile.h"

/* ---- 命令量（参数表 ID 38/40/41/42/53）---- */
extern volatile float rt_rec_cmd;       /* 0→1 开始录制 / 1→0 停止并落槽 0 */
extern volatile float rt_slot_cmd;      /* 目标槽 0..4；改了就切并载入 */
extern volatile float rt_load_cmd;      /* 0→1 重新载入当前槽 */
extern volatile float rt_save_cmd;      /* 0→1 把 RAM 里的表存进当前槽 */
extern volatile float rt_mark_cmd;      /* 0→1 给"最后一个已录点"打关键点 */

extern volatile float rt_revrec;        /* ID76 0/1 倒车录表：录制时 v 存 |v|（保留 REV 标记） */
extern volatile float rt_revplay;       /* ID77 0/1 倒序回放：回放时把点序倒过来 */

/* ---- 状态位（遥测 CH46 `rt_st` 的**低 8 位**；bit8..11 = 当前槽号）---- */
#define RT_ST_REC       (0x01u)     /* 正在录制 */
#define RT_ST_FULL      (0x02u)     /* 录满了（点距太大 / 赛道太长） */
#define RT_ST_HAVE      (0x04u)     /* RAM 里有表（count > 0） */
#define RT_ST_SAVING    (0x08u)     /* 正在分步写 Flash */
#define RT_ST_SAVEFAIL  (0x10u)     /* 上次写盘失败（页写错） */
#define RT_ST_PLAY      (0x20u)     /* 正在回放（P2，见 route_pb.h） */
#define RT_ST_FROMFLSH  (0x40u)     /* 当前表是上电从 Flash 载入的 */
#define RT_ST_LINK      (0x80u)     /* 串口正在灌表/拉表 */

/* 录制少于这么多点就不落盘（防止误按把槽 0 覆盖成垃圾） */
#define RT_MIN_SAVE_POINTS  (20u)

void        route_app_init  (void);     /* 上电一次（**必须在 5 ms 中断打开之前**） */
void        route_app_poll  (void);     /* 主循环每圈一次（幂等、空闲几乎不要钱） */
uint16      route_app_count (void);     /* 当前 RAM 表点数 */
uint16      route_app_status(void);     /* 位图 | (槽号 << 8) */
const char *route_app_str   (void);     /* 一行摘要（屏幕/日志） */
uint8       route_app_slot_next (void); /* 菜单用：切到下一槽并载入（返回新槽号） */

#endif
