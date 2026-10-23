/* ---- 与工程 10 的关系 -------------------------------------------------------------------
 * 工程 10 的 `route_pb.c` 已经能跑（`δ = atan(L·κ)` 直给内环），但它是"一个函数干完所有事"，
 * ② **补回"完成/中止"两条判据**（工程 10 只有"到表尾就停"，把"脱线跑到末点"也当成正常结束）
 * ③ **补回"最近点给速度、前视点给转向"**（工程 10 用前视点的限速，等于提前把弯里的低速
 * 留两条是为了 A/B —— 航向通道能把"目标航向"这一路单独发遥测，调外环时看得见。
 * ⑤ 曲率窗口、最低速度、终点半径都提成参数（工程 10 是写死的 ±2 点）。
 * Pure Pursuit 的几何量：前视点在前方 ld、与车头夹角 α。则
 * κ = 2·sin(α) / ld,        δ = atan(L·κ)
 * 而"目标航向 − 当前航向"恰好就是 α（前视点的方位角减车头朝向）。
 * 所以航向环只要取 **k = 2L/ld**（无量纲，deg/deg），
 * δ = k·(yaw_tgt − yaw) ≈ 2L·α/ld ≈ atan(L·κ)
 * 就是同一个式子（小角近似差 sinα≈α，α<30° 时差 <5%）。
 * ⇒ `pb_kh=0` 时**每拍按 ld 自动算增益**，不需要人去调 —— 这一点是工程 10 没做的
 * （它那个 `head_kp=15` 是手填的，与 ld 无关，所以"谁开谁振"）。
 * ① 曲率限速：`v_i = min(vmax, sqrt(alat/|κ_i|))`，κ 用**中心差分半窗 pb_kwin 点**
 * （窗口太小会被航向量化噪声打出一串假减速点）；
 * ② **反向传刹车**：`v_{i-1} ≤ sqrt(v_i² + 2·abrk·ds)` —— 进弯前就减到位；
 * ③ 最低速度 `pb_vmin` 兜底防爬行；**末点锚 `PB_END_CREEP`（爬行速度）**，
 * 减速曲线由 ② 的反向刹车自动生成（减速度恒 ≤ `pb_abrk`，连续无台阶）。
 * · `dist(当前, 末点) < pb_fin` ⇒ **COMPLETED**（正常结束，收油停车）
 * · 索引已经到末点、但离末点还很远 ⇒ **ABORTED**（脱线/跳点，不是跑完了）
 * 这两种在遥测 `pb_state` 上分得开 —— 工程 10 里它们都表现成"到表尾，pb_en 归 0"，
 * 现场没法区分"跑完"和"跑飞了"。
 * ---- 模式 -------------------------------------------------------------------------------
 * `pb_mode` 1 = **只给转向**（手推验收：看方向盘方向对不对）
 * 2 = 转向 + 速度（真跑）
 * ---- 安全门（一条不减）------------------------------------------------------------------
 * · `pb_en=0` ⇒ 退出，并把转角目标对齐实测角（不甩头）
 * · 遥控接管 / FAILSAFE / 急停 ⇒ 让位
 * · 表点太少 / 航位推算没开 ⇒ 不启动；启动失败把 `pb_en` 退回 0
 */

#ifndef _ROUTE_PB_H
#define _ROUTE_PB_H

#include "zf_common_headfile.h"

/* ---- 参数（ID 43~55 + 64~68）----------------------------------------------------------- */
extern volatile float pb_en;        /* 43 回放使能 0/1 */
extern volatile float pb_mode;      /* 44 1 = 只给转向；2 = 转向+速度 */
extern volatile float pb_vmax;      /* 45 巡航最高车速 m/s */
extern volatile float pb_alat;      /* 46 允许的横向加速度 m/s^2 */
extern volatile float pb_abrk;      /* 47 刹车减速度 m/s^2（反向传一遍） */
extern volatile float pb_l;         /* 48 有效轴距 m */
extern volatile float pb_ld_a;      /* 49 前视 = a + b·|v| */
extern volatile float pb_ld_b;
extern volatile float pb_ld_min;
extern volatile float pb_ld_max;
extern volatile float pb_ff_en;     /* 54 打角前馈使能（手工方案） */
extern volatile float pb_ff_gain;   /* 55 融合权重 w */
extern volatile float pb_vmin;
extern volatile float pb_kwin;
extern volatile float pb_smode;
extern volatile float pb_kh;
extern volatile float pb_fin;

/* κ(IMU Δψ ÷ 里程) ⇒ R = 1.305 m ｜ R(左右计数 + 几何 W) ⇒ 1.298 m ⇒ **L_eff = 0.66**
 * 几何卷尺 0.63；差 4.3% ＝「27° 读数 ≈ 25.9° 真实轮角」，数学等价，别填 0.63。
 */
#define PB_L_DEFAULT        (0.66f)

/* 点距与窗口的换算：`pb_kwin=6` × 0.05 m ⇒ 差分跨度 0.6 m。 所以取 6 就够，弯道形状失真也更小。 */
#define PB_KWIN_DEFAULT     (6.0f)

#define PB_MIN_POINTS       (10u)
/* 现场症状：快到终点时**突然顿一下**，然后以很慢的速度爬到终点再停。
 * `for (k = n-1; k > 0; k--) { r = 1 - back/PB_END_SLOW_M; v[k] = vmin + (v[k]-vmin)*r; back += ds; }`
 * `back` 从**末点往前**递增 ⇒ `r` 从 1 递减 ⇒ **越靠近末点速度越高、越往前越低**。
 * 结果是：末点保持全速(≈2.0 rps)，末点前 1 m 处被压到 vmin(0.24 rps)，而那 1 m
 * 之外又跳回 1.7 rps —— 在 s = L−1.0 m 处形成一个 **13.3 m/s² 的减速度尖峰**
 * （pb_abrk 只有 0.8，超 16 倍）= 现场那"一顿"。逐点复算见 `end_decel_old_vs_new.png`。
 * 现在的做法：**末点速度锚到 PB_END_CREEP**，减速曲线交给剖面 ② 的反向刹车
 * `v[i-1] ≤ sqrt(v[i]² + 2·abrk·ds)`
 * 它是"用给定减速度能刹住的最快速度"⇒ 天然连续、天然不超过 pb_abrk。
 * 2.0 rps → 0.61 rps 需要 (1.97² − 0.5²)/(2×0.8) ≈ 2.3 m，也就是从终点前 2.3 m 开始匀着收。
 * 0.5 m/s（0.61 rps）比 pb_vmin(0.24 rps) 高，避开电机死区，爬行段不会一走一停。
 */
#define PB_END_CREEP        (0.50f)         /* 末点爬行速度 m/s（0.61 rps，高于死区） */

/* （占空比被 80% 限幅钳住）⇒ 电流冲击 + 车头点一下。这就是用户说的
 * "不能直接给 0 rps，2 rps 一步到 0 瞬时电流太大"。
 * 周期 = 1 ms + 各任务耗时（菜单刷屏最差 40 ms，`ui_menu.c` 自己的注释就写着
 * "route_pb_poll 也跟着抖"）⇒ 同样"80 拍"在不同时刻是 0.16 s 还是 3 s。
 * 用 `motor_link_now_ms()`（5 ms 中断累加）才与周期无关。
 * 350 ms 从 0.61 rps 降下来 ≈ 1.75 rps/s ≈ 1.4 m/s²；此时车速只有 0.5 m/s、动能很小。
 */
#define PB_STOP_MS          (350u)
#define PB_NEAR_WIN         (40u)           /* 最近点前向搜索窗（点）= 2 m */
/* 而 odom 相对上电 —— 中间断过电或按过 ins_zero 就对不上，车会被指到几十米外。 ⇒ 按 origin 换算出来的位置离表起点超过这个距离，就退回"车现在就在第 0 点"。 */
#define PB_ALIGN_MAX_M      (2.0f)
/* 等于 2 倍速过同一个弯 ⇒ 转向跟不上、每次都冲出去（现场"跑成一条大弧"）。 */
#define PB_REV_EPS          (0.10f)
#define PB_REV_CREEP        (0.15f)
/* 20 ms 的单拍毛刺（SBUS 坏帧让 CH5 瞬间越过中位）不该废掉一整趟； 真正的急停远长于 20 ms，安全层的硬停机本来也是**当拍**生效、不等这里。 */
#define PB_ESTOP_TICKS      (4u)
#define PB_REV_HEAD_KP      (0.0f)      /* 我们倒车段只有 0.7 m ⇒ 先 0（纯开环）。要开先确认符号， 反了是正反馈发散 */

/* ---- 状态（遥测用；`pb_state` 上能区分"跑完"和"跑飞"）--------------------------------- */
#define PB_ST_IDLE          (0u)
#define PB_ST_RUN           (1u)
#define PB_ST_DONE          (2u)
#define PB_ST_ABORT         (3u)

void    route_pb_init   (void);
void    route_pb_poll   (void);         /* 主循环每圈一次 */
/* 任一道门没过就提前 return，根本走不到"PUSH 分支设 free-wheel"那几行，而速度环仍在以
 * target=0 闭环 ⇒ 人推不动。调用点：5 ms 中断里、`spd_ctrl_tick()` **之前**。
 */
void    route_pb_sync_coast (void);

/* 起跑失败（没表/没航位/没令牌）后允许重试一次（菜单写 Follow 时调） */
void    route_pb_rearm  (void);

const char * route_pb_reason (void);

void    route_pb_stop   (const char *why);

float   route_pb_elat   (void);         /* 横向误差 m（正 = 车在参考线右侧） */
uint8   route_pb_active (void);         /* 1 = 正在回放 */
uint8   route_pb_state  (void);         /* PB_ST_* */
float   route_pb_ld     (void);         /* 本拍前视距离 m */
float   route_pb_tv     (void);         /* 本拍目标车速 m/s */
float   route_pb_yaw_tgt(void);         /* 本拍目标航向 deg（航向通道才有意义） */
uint16  route_pb_aim    (void);         /* 本拍前视点索引 */
uint16  route_pb_index  (void);         /* 本拍最近点索引 */

#endif
