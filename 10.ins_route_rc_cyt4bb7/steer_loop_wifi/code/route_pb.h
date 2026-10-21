/* 工程 10（卡丁）· 路表回放 —— Pure Pursuit（P2）
 * 工程 8 的 `route_cruise` 是"**里程索引 + 纯航向 PD**"：它把车投影到表的第 `dist/ds` 点上，
 * 只对**航向**做闭环，横向位置**完全开环**。我们在 C 车实测到它的横向误差地板
 * `e = 0.259 + 0.547·v`（m）—— 慢也压不到 0.26 m，**这就是"跟不直"的根**。
 * ---- 这里做什么 -----------------------------------------------------------------------
 * → 前视点到车头连线的夹角 α → 曲率 κ = 2·sin(α)/ld → **转角 δ = atan(L·κ)**
 * → 直接写 `steer_ctrl_set_target_delta(δ)`（走转向内环，不进航向外环）。
 * 而 Pure Pursuit 的几何关系已经把"横向偏差"折进了 α 里 ⇒ 再串一层 P 只会多一个待调增益
 * 和一段相位滞后。L 我们**实测过**（手推 0.648 m / 驱动 0.68 m，见
 * `工程10-轮径反查与转向yaw映射-实测分析.md`）⇒ 直接用几何解，参数少一个。
 * `v_{i-1} ≤ sqrt(v_i² + 2·abrk·ds)` —— 保证进弯前就减到位（只做前向限制会"进弯太快"）。
 * ---- 两种模式（`pb_mode`）--------------------------------------------------------------
 * 1 = **只给转向**（速度不动）：手推着车看方向盘方向对不对 —— **上车第一件事就做这个**。
 * 2 = 转向 + 速度：真正跑。
 * ---- 安全门（照"遥控当安全绳"的语义）---------------------------------------------------
 * · `pb_en = 0` ⇒ 立刻退出，并把目标交还（转角目标对齐实测角，不甩头）
 * · **遥控一旦接管（`RC_Engage == ON`）⇒ 本模块立刻让位**（操作杆一动就夺回控制权）
 * · FAILSAFE / 急停 ⇒ 不跑
 * · 表少于 `PB_MIN_POINTS` 点 / 航位推算没使能 ⇒ 不跑
 */

#ifndef _ROUTE_PB_H
#define _ROUTE_PB_H

#include "zf_common_headfile.h"

/* ---- 参数（ID 43~55）---- */
extern volatile float pb_en;        /* 回放使能 0/1 */
extern volatile float pb_mode;      /* 1 = 只给转向；2 = 转向+速度 */
extern volatile float pb_vmax;      /* 巡航最高车速 m/s */
extern volatile float pb_alat;      /* 允许的横向加速度 m/s^2（决定弯里限速） */
extern volatile float pb_abrk;      /* 刹车减速度 m/s^2（反向传一遍） */
/* 稳态窗口（t=34.9~37.7 s，|δ|=26.76°）三重自洽：
 * κ(IMU Δψ ÷ 里程)      ⇒ R = **1.305 m**
 * R(左右计数 + 几何W)   ⇒ R = **1.298 m**   （完全不用 IMU）
 * W(用 Δψ 反推)         ⇒ W = **0.608 m**   （几何量到 0.605）
 * 轮程比 0.622 vs 纯滚动预测 0.624 ⇒ 手推确实**无滑移**
 * ⇒ `L_eff = R·tan(26.76°) = **0.658**` ⇒ 取 **0.66**（`R_min = 0.66/tan27 = 1.295`，
 * 与实测 1.305 差 0.8%）。
 * （atan(0.63/1.30)=25.86°）。两种描述数学等价，控制里用 0.66 自洽即可；
 * **别把 0.63 填进来**（会让每拍少转 4.3%）。
 * 纯滚动应差 34% ⇒ 内轮空转/被拖 ⇒ 里程虚高 **+22%**、算出 R=1.74 m（+33%）。
 */
#define PB_L_DEFAULT        (0.66f)
extern volatile float pb_l;         /* 有效轴距 m（δ = atan(L·κ)） */
extern volatile float pb_ld_a;      /* 前视 = a + b·|v| */
extern volatile float pb_ld_b;
extern volatile float pb_ld_min;
extern volatile float pb_ld_max;
/* δ = w·steer[best] + (1−w)·δ_pp，  w = pb_ff_gain ∈ [0,1]
 * pb_ff_en = 0 ⇒ 严格退回纯 Pure Pursuit（老行为，一字节不变）。
 * 加得太多会"照着人打的角度跑偏"（人开的那趟本身就有误差），
 * 减到 0 就完全靠 INS 定位。
 */
extern volatile float pb_ff_en;     /* 0/1 = 前馈开关 */
extern volatile float pb_ff_gain;   /* w：0 = 纯 PP，1 = 纯回放人打的角 */

#define PB_MIN_POINTS       (10u)

void    route_pb_init   (void);
void    route_pb_poll   (void);     /* 主循环每圈一次 */
float   route_pb_elat   (void);     /* 最近一次的横向误差 m（正 = 车在参考线右侧） */
uint8   route_pb_active (void);     /* 1 = 正在回放 */

#endif
