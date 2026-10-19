# -*- coding: utf-8 -*-
"""
加电子差速（左右轮目标速度分配）—— 2026-10-17

改动 3 个文件：
  spd_ctrl.h   参数 + 接口声明
  spd_ctrl.c   实现（含实测舵角低通、限幅、倒车关闭）
  route_pb.c   step 8 用差速分配替换"两轮同 rps"
  motor_link.h 参数 ID 56~58

保持各文件原行尾（三个都是 CRLF）。
"""
import os
import sys

BS = chr(92)
D = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "steer_loop_wifi", "code")
D = os.path.abspath(D)


def load(name):
    raw = open(os.path.join(D, name), "rb").read()
    t = raw.decode("utf-8")
    return t


def save(name, t, crlf=False):
    t = t.replace(chr(13) + chr(10), chr(10))
    if crlf:
        t = t.replace(chr(10), chr(13) + chr(10))
    open(os.path.join(D, name), "wb").write(t.encode("utf-8"))


def sub(t, old, new, cnt=1, tag=""):
    n = t.count(old)
    if n != cnt:
        print("[ERR] %s 锚点命中 %d（期望 %d）" % (tag, n, cnt))
        print(repr(old[:80]))
        sys.exit(2)
    return t.replace(old, new, cnt)


# ============================================================ spd_ctrl.h
h = load("spd_ctrl.h")
block = """extern volatile float   spd_ol_duty_r;  /* 右轮开环占空比 %, 带符号(-100~100) */

/*===================================================================================================================
 *  ★★ 电子差速：左右轮目标速度分配（2026-10-17）
 *
 *  为什么**必须有**：这台卡丁是**双电机独立驱动**（左 P09_0 / 右 P10_2，各自一套编码器），
 *  没有机械差速器。转弯时内外轮走过的弧长本来就不同：
 *      满舵 26.8°、R = L/tanδ = 1.30 m、轮距 W = 0.605 m  ⇒  内外差 (W/2)/R = ±23%
 *  两轮硬给同一个目标时，内轮被地面**拖着走**（或打滑），后果不只是跑得别扭：
 *      实测（wave_20261017_213650.csv 驱动段）左右轮程比 **0.88**，纯滚动应为 0.62
 *      ⇒ 里程虚高 **+22%** ⇒ 由里程积分出来的 x/y/yaw 整体偏 ⇒ 回放越跑越偏。
 *  ⇒ 差速在这里不是"舒适性配置"，它是**惯导里程正确性的前提**。
 *
 *  分配律（后轴中心速度 v，曲率 κ = tanδ / L）：
 *      v_l = v·(1 − κ·W/2)        v_r = v·(1 + κ·W/2)
 *  两边都是"同一个 v 乘一个系数" ⇒ **平均速度恒等于 v** ⇒
 *  `ins_odom` 的 `ds = (dl+dr)/2` 不受差速影响（这一点是它能和现有里程/路表共存的关键）。
 *
 *  ---- 三条纪律（都是从 Polaris 的坑里抄来的）-----------------------------------------
 *  ① **用实测舵角，不用目标舵角**。转向机构有滞后/旷量，用目标会让后轮提前差速，
 *     表现是入弯时车尾先被"推"出去。Polaris `kart_control.c` 第 6 步同一结论。
 *  ② **倒车关闭**。符号约定在实车上没验证过，Polaris 也是关的（`left_target 倒车:关差速`）。
 *  ③ **内侧不许停转**。停转的轮子编码器读 0、速度环积分一直堆，出弯来一脚。
 *     上限 0.85 ⇒ 内侧最低 15%。
 *
 *  ⚠ Polaris 的做法**不能照抄**：他们的后轮是共轴机械耦合，两个独立闭环 PID 会互顶发散，
 *    所以退化成"单 PID 控左右平均 + duty 前馈偏置"，且 MAX_RATIO 只有 12%（满舵要 21.6%
 *    ⇒ 补不满），默认是关的。我们双电机独立、每轮一套编码器，正确做法是在**目标速度**上
 *    分配（闭环下达），而不是在 duty 上打偏置（开环）。
 *=================================================================================================================*/
#define SPD_DIFF_W_DEFAULT      (0.605f)    /* 轮距：左右轮**心**间距（实测 60.5 cm）*/
#define SPD_DIFF_MAX_DEFAULT    (0.85f)     /* 单侧偏差上限：内侧最低 15%            */

extern volatile float   spd_diff_en;    /* >0.5 = 开电子差速                     */
extern volatile float   spd_diff_w;     /* 轮距 m                                */
extern volatile float   spd_diff_max;   /* 单侧偏差上限（0.85 ⇒ 内侧 ≥ 15%）     */

/* 把后轴中心速度 v(m/s) 按实测舵角分成左右轮目标速度(m/s)。
   ⚠ vl/vr 一定是"同号且平均等于 v"；v<=0（倒车/静止）时原样返回，不做分配。 */
void spd_diff_alloc (float v_mps, float L_m, float delta_meas_deg, float *vl, float *vr);"""

h = sub(h, "extern volatile float   spd_ol_duty_r;  /* 右轮开环占空比 %, 带符号(-100~100) */",
        block, 1, "spd_ctrl.h 插入点")
save("spd_ctrl.h", h, crlf=True)
print("[OK] spd_ctrl.h")

# ============================================================ spd_ctrl.c
c = load("spd_ctrl.c")
c = sub(c, '#include "motor_drv.h"', '#include "motor_drv.h"\n\n#include <math.h>', 1, "math.h")

impl = """
/*===================================================================================================================
 *  ★★ 电子差速（说明见 spd_ctrl.h）
 *     纯函数式分配：不碰任何状态机，调用方（回放/遥控/跑班）各自决定什么时候用。
 *=================================================================================================================*/
volatile float spd_diff_en  = 1.0f;
volatile float spd_diff_w   = SPD_DIFF_W_DEFAULT;
volatile float spd_diff_max = SPD_DIFF_MAX_DEFAULT;

static float s_diff_dm = 0.0f;          /* 实测舵角一阶低通状态（度）*/

void spd_diff_alloc (float v_mps, float L_m, float delta_meas_deg, float *vl, float *vr)
{
    float dm, k, r;

    if ((NULL == vl) || (NULL == vr)) { return; }

    *vl = v_mps;
    *vr = v_mps;

    if (spd_diff_en < 0.5f)
    {
        s_diff_dm = delta_meas_deg;
        return;
    }

    /* 倒车/静止：不做分配，但**让低通跟上**当前角，免得下次前进第一拍跳变 */
    if (v_mps < 0.02f)
    {
        s_diff_dm = delta_meas_deg;
        return;
    }

    if (L_m < 0.30f) { return; }                    /* 轴距没配好：宁可不差速 */

    /* 一阶低通（α=0.25）：实测舵角带编码器量化噪声，直接进 tan() 会让差速抖 */
    dm = 0.25f * delta_meas_deg + 0.75f * s_diff_dm;
    s_diff_dm = dm;

    k = tanf(dm * 0.0174532925f) / L_m;             /* 1/m，正 = 右转 */
    r = k * spd_diff_w * 0.5f;                      /* 相对偏差 */

    if (r >  spd_diff_max) { r =  spd_diff_max; }
    if (r < -spd_diff_max) { r = -spd_diff_max; }

    /* 右转（k>0）：左轮在外侧 ⇒ 快；右轮在内侧 ⇒ 慢。
       ⚠ 符号来自 `steer_delta > 0 ⇒ 右转`（见 steer_ctrl / 轮径反查文档）。 */
    *vl = v_mps * (1.0f + r);
    *vr = v_mps * (1.0f - r);
}
"""
# 插到文件末尾（spd_ctrl_tick 之后）
c = c.rstrip("\r\n") + "\n" + impl
save("spd_ctrl.c", c, crlf=False)
print("[OK] spd_ctrl.c")

# ============================================================ route_pb.c
p = load("route_pb.c")
old8 = """    if (pb_mode >= 1.5f)
    {
        float vt = s_vlim[j];
        float rps = vt / ((ins_m_per_rev > 1e-4f) ? ins_m_per_rev : 0.8193f);

        /* 到表尾前 1 m 主动收车（不然靠"到表尾"那一拍突然断，会冲出去）
           ⚠ 剩余里程用 (点数 × 点距) 估，**不逐点累加** ——
             这个函数每圈都跑，逐点 sqrt 到 2040 点会吃掉主循环几百微秒，
             把 spd_safety 的 200 ms 心跳门顶穿。点距本来就近等距，误差 < 2%。 */
        {
            float left = (float)(s_n - 1u - j) * ins_route_ds;
            if (left < 1.0f) { rps *= (left / 1.0f); }
        }

        spd_target_l =  rps;
        spd_target_r =  rps;
    }"""
new8 = """    if (pb_mode >= 1.5f)
    {
        float vt  = s_vlim[j];
        float mpr = (ins_m_per_rev > 1e-4f) ? ins_m_per_rev : 0.8193f;
        float vl, vr;

        /* 到表尾前 1 m 主动收车（不然靠"到表尾"那一拍突然断，会冲出去）
           ⚠ 剩余里程用 (点数 × 点距) 估，**不逐点累加** ——
             这个函数每圈都跑，逐点 sqrt 到 2040 点会吃掉主循环几百微秒，
             把 spd_safety 的 200 ms 心跳门顶穿。点距本来就近等距，误差 < 2%。
           ★ 收车必须在**分配之前**先把 v 缩掉，否则两侧比例不变、平均速度没降。 */
        {
            float left = (float)(s_n - 1u - j) * ins_route_ds;
            if (left < 1.0f) { vt *= (left / 1.0f); }
        }

        /* ---- 8b) ★ 电子差速（2026-10-17）----
           转弯时内外轮弧长不同（满舵 ±23%）。不差速 = 内轮被拖着走 ⇒ 编码器里程虚高
           （实测 +22%）⇒ 位姿/航向整体偏。分配律见 spd_ctrl.h：
               v_l = v(1 − κW/2)   v_r = v(1 + κW/2)   κ = tan(δ_meas)/L
           ⚠ 用**实测**舵角：转向有滞后，用目标会让后轮提前差速（车尾先被推出去）。
           ⚠ 平均速度恒等于 vt ⇒ ins_odom 的 (dl+dr)/2 不受影响，路表限速仍然成立。 */
        spd_diff_alloc(vt, pb_l, steer_ctrl_get_meas_delta(), &vl, &vr);

        spd_target_l = vl / mpr;
        spd_target_r = vr / mpr;
    }"""
p = sub(p, old8, new8, 1, "route_pb step8")

# 顺手修过期注释（pb_l 已经定回 0.66）
p = sub(p,
        " *    ⚠ `ins_mpm` 改了必须同批改本值（两者都吃里程尺度）。 */",
        " *    ⚠ `ins_mpm` 改了必须同批改本值（两者都吃里程尺度）。\n"
        " *    ⚠ 2026-10-17 晚**满舵转圈实测**（手推稳态 |δ|=26.76°）三种算法吻合 0.5%：\n"
        " *       R(IMU+里程)=1.305 / R(纯计数+几何W)=1.298 ⇒ L_eff=0.658 ⇒ **定 0.66**。\n"
        " *       几何 0.63 与 0.66 差 4.3% ＝「27° 读数 ≈ 25.9° 真实轮角」，数学等价。\n"
        " *       ⚠ 驱动段**不能**用来标定（内轮被拖，里程虚高 22% ⇒ R 虚高 33%）；\n"
        " *         加了电子差速之后这一点才会好转，届时可重测一次作验证。 */",
        1, "pb_l 注释")
save("route_pb.c", p, crlf=False)
print("[OK] route_pb.c")

# ============================================================ motor_link.h
m = load("motor_link.h")
last = ('    X(55, "pb_ff",      "回放|前馈增益",      0.000f,    2.000f,   1.000f, "-",     0.050f, &pb_ff_gain          )')
newp = (last + " " + BS + "\n"
        '    X(56, "df_en",      "差速|电子差速使能",  0.000f,    1.000f,   1.000f, "-",     1.000f, &spd_diff_en         ) ' + BS + "\n"
        '    X(57, "df_w",       "差速|轮距(轮心)",    0.300f,    1.000f,   SPD_DIFF_W_DEFAULT, "m", 0.005f, &spd_diff_w     ) ' + BS + "\n"
        '    X(58, "df_max",     "差速|单侧偏差上限",  0.100f,    0.950f,   SPD_DIFF_MAX_DEFAULT, "-", 0.010f, &spd_diff_max )')
m = sub(m, last, newp, 1, "motor_link 参数表")
# 参数表引用了 spd_ctrl.h 的宏 —— 确认它有 include
if '#include "spd_ctrl.h"' not in m:
    print("[WARN] motor_link.h 没 include spd_ctrl.h，检查一下")
save("motor_link.h", m, crlf=True)
print("[OK] motor_link.h")
