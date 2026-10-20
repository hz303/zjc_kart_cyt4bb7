/*********************************************************************************************************************
 * 工程 11（卡丁）· 路表回放 —— Pure Pursuit 外环（照 Polaris 的权威结构重建）
 *
 * ---- 与工程 10 的关系 -------------------------------------------------------------------
 *   工程 10 的 `route_pb.c` 已经能跑（`δ = atan(L·κ)` 直给内环），但它是"一个函数干完所有事"，
 *   而且把 Polaris 里几条**判据**省掉了。工程 11 做的是**结构对齐**，不是重写算法：
 *
 *     ① 拆成 Polaris 的五段：位姿投影 → 最近点(单调) → 完成/中止判据 → 速度决策 → 转向前视
 *     ② **补回"完成/中止"两条判据**（工程 10 只有"到表尾就停"，把"脱线跑到末点"也当成正常结束）
 *     ③ **补回"最近点给速度、前视点给转向"**（工程 10 用前视点的限速，等于提前把弯里的低速
 *        套到直道末段，长时间偏慢；Polaris 明确写"速度跟当前弧段，弯道不提前拉满"）
 *     ④ **转向多一条通道**：`pb_smode=0` 走 Polaris 的**航向外环**（目标航向 → 内环），
 *        `pb_smode=1` 走工程 10 的**直接给 δ**。两条**数学等价**（下面有推导），
 *        留两条是为了 A/B —— 航向通道能把"目标航向"这一路单独发遥测，调外环时看得见。
 *     ⑤ 曲率窗口、最低速度、终点半径都提成参数（工程 10 是写死的 ±2 点）。
 *
 * ---- 为什么"航向外环"和"直接给 δ"是等价的 ----------------------------------------------
 *   Pure Pursuit 的几何量：前视点在前方 ld、与车头夹角 α。则
 *        κ = 2·sin(α) / ld,        δ = atan(L·κ)
 *   而"目标航向 − 当前航向"恰好就是 α（前视点的方位角减车头朝向）。
 *   所以航向环只要取 **k = 2L/ld**（无量纲，deg/deg），
 *        δ = k·(yaw_tgt − yaw) ≈ 2L·α/ld ≈ atan(L·κ)
 *   就是同一个式子（小角近似差 sinα≈α，α<30° 时差 <5%）。
 *   ⇒ `pb_kh=0` 时**每拍按 ld 自动算增益**，不需要人去调 —— 这一点是工程 10 没做的
 *     （它那个 `head_kp=15` 是手填的，与 ld 无关，所以"谁开谁振"）。
 *
 * ---- 速度剖面（与 Polaris `kart_playback_build_profile` 同构）--------------------------
 *   ① 曲率限速：`v_i = min(vmax, sqrt(alat/|κ_i|))`，κ 用**中心差分半窗 pb_kwin 点**
 *      （窗口太小会被航向量化噪声打出一串假减速点）；
 *   ② **反向传刹车**：`v_{i-1} ≤ sqrt(v_i² + 2·abrk·ds)` —— 进弯前就减到位；
 *   ③ 最低速度 `pb_vmin` 兜底防爬行；末段 `PB_END_SLOW_M` 内线性收油，防冲过终点。
 *
 * ---- 完成 / 中止（Polaris 的两条判据，工程 10 缺）--------------------------------------
 *   · `dist(当前, 末点) < pb_fin` ⇒ **COMPLETED**（正常结束，收油停车）
 *   · 索引已经到末点、但离末点还很远 ⇒ **ABORTED**（脱线/跳点，不是跑完了）
 *   这两种在遥测 `pb_state` 上分得开 —— 工程 10 里它们都表现成"到表尾，pb_en 归 0"，
 *   现场没法区分"跑完"和"跑飞了"。
 *
 * ---- 模式 -------------------------------------------------------------------------------
 *   `pb_mode` 1 = **只给转向**（手推验收：看方向盘方向对不对）
 *             2 = 转向 + 速度（真跑）
 *   `pb_smode` 0 = 航向外环（Polaris 通道，默认）1 = 直接给 δ（工程 10 通道）
 *
 * ---- 安全门（一条不减）------------------------------------------------------------------
 *   · `pb_en=0` ⇒ 退出，并把转角目标对齐实测角（不甩头）
 *   · 遥控接管 / FAILSAFE / 急停 ⇒ 让位
 *   · 表点太少 / 航位推算没开 ⇒ 不启动；启动失败把 `pb_en` 退回 0
 ********************************************************************************************************************/

#ifndef _ROUTE_PB_H
#define _ROUTE_PB_H

#include "zf_common_headfile.h"

/* ---- 参数（ID 43~55 + 64~68）----------------------------------------------------------- */
extern volatile float pb_en;        /* 43 回放使能 0/1                                */
extern volatile float pb_mode;      /* 44 1 = 只给转向；2 = 转向+速度                 */
extern volatile float pb_vmax;      /* 45 巡航最高车速 m/s                            */
extern volatile float pb_alat;      /* 46 允许的横向加速度 m/s^2                      */
extern volatile float pb_abrk;      /* 47 刹车减速度 m/s^2（反向传一遍）              */
extern volatile float pb_l;         /* 48 有效轴距 m                                  */
extern volatile float pb_ld_a;      /* 49 前视 = a + b·|v|                            */
extern volatile float pb_ld_b;      /* 50                                             */
extern volatile float pb_ld_min;    /* 51                                             */
extern volatile float pb_ld_max;    /* 52                                             */
extern volatile float pb_ff_en;     /* 54 打角前馈使能（手工方案）                    */
extern volatile float pb_ff_gain;   /* 55 融合权重 w                                  */
extern volatile float pb_vmin;      /* 64 ★ 最低速度 m/s（防爬行）                    */
extern volatile float pb_kwin;      /* 65 ★ 曲率中心差分半窗（点）                    */
extern volatile float pb_smode;     /* 66 ★ 0 = 航向外环 / 1 = 直接给 δ               */
extern volatile float pb_kh;        /* 67 ★ 航向环增益；0 = 自动 2L/ld                */
extern volatile float pb_fin;       /* 68 ★ 终点判定半径 m                            */

/* ★ 有效轴距默认值（**唯一权威**：route_pb.c 的初值与参数表 ID48 的默认值都引用它）
 *
 *   ---- 2026-10-17 满舵转圈实测（手推自由滚动，|δ|=26.76°）----------------------------
 *       κ(IMU Δψ ÷ 里程) ⇒ R = 1.305 m ｜ R(左右计数 + 几何 W) ⇒ 1.298 m ⇒ **L_eff = 0.66**
 *       几何卷尺 0.63；差 4.3% ＝「27° 读数 ≈ 25.9° 真实轮角」，数学等价，别填 0.63。
 *   ⚠ 电机驱动段**不能**用来标定：内轮被拖 ⇒ 里程虚高 22% ⇒ R 虚高 33%。
 */
#define PB_L_DEFAULT        (0.66f)

/* 点距与窗口的换算：`pb_kwin=6` × 0.05 m ⇒ 差分跨度 0.6 m。
 * Polaris 取 8 点 × 0.05 m = 0.8 m；我们的航向存的是 float（0.01° 级），噪声比它小，
 * 所以取 6 就够，弯道形状失真也更小。 */
#define PB_KWIN_DEFAULT     (6.0f)

#define PB_MIN_POINTS       (10u)
#define PB_END_SLOW_M       (1.0f)          /* 末段收油距离 m                        */
#define PB_NEAR_WIN         (40u)           /* 最近点前向搜索窗（点）= 2 m           */

/* ---- 状态（遥测用；`pb_state` 上能区分"跑完"和"跑飞"）--------------------------------- */
#define PB_ST_IDLE          (0u)
#define PB_ST_RUN           (1u)
#define PB_ST_DONE          (2u)
#define PB_ST_ABORT         (3u)

void    route_pb_init   (void);
void    route_pb_poll   (void);         /* 主循环每圈一次 */
/* ★★ 2026-10-20：自由轮从状态机里挪出来，每 5 ms 按"用户意图"重算一遍。
   为什么：`route_pb_poll()` 在主循环，前面有一串安全门（没表 / 航位没开 / 遥控 FAILSAFE）。
   任一道门没过就提前 return，根本走不到"PUSH 分支设 free-wheel"那几行，而速度环仍在以
   target=0 闭环 ⇒ 人推不动。调用点：5 ms 中断里、`spd_ctrl_tick()` **之前**。 */
void    route_pb_sync_coast (void);

/* 起跑失败（没表/没航位/没令牌）后允许重试一次（菜单写 Follow 时调） */
void    route_pb_rearm  (void);

/* 最近一次"为什么没跑/为什么结束"（给屏幕看；串口也会打一遍） */
const char * route_pb_reason (void);

void    route_pb_stop   (const char *why);

float   route_pb_elat   (void);         /* 横向误差 m（正 = 车在参考线右侧）      */
uint8   route_pb_active (void);         /* 1 = 正在回放                          */
uint8   route_pb_state  (void);         /* PB_ST_*                               */
float   route_pb_ld     (void);         /* 本拍前视距离 m                        */
float   route_pb_tv     (void);         /* 本拍目标车速 m/s                      */
float   route_pb_yaw_tgt(void);         /* 本拍目标航向 deg（航向通道才有意义）   */
uint16  route_pb_aim    (void);         /* 本拍前视点索引                        */
uint16  route_pb_index  (void);         /* 本拍最近点索引                        */

#endif
