# -*- coding: utf-8 -*-
"""给转向位置环加积分项（2026-10-18）。

背景：现在 `u = kp*err + u0*sign(err)`，误差进 0.2° 死区后 err 被清 0 ⇒ u 也变 0
⇒ 舵机只能靠摩擦自锁，**停在哪儿取决于摩擦方向** ⇒ 每次使能后的静止角随机（实测 ±0.2°）。
0.2° 舵角误差在 30 m 赛道上 = ±2.4 m 横偏，这就是"看着居中却整体偏"的来源。

改法（Polaris `kart_steer_ctrl` 的内环是 PID，Ki=0.5/Imax=2000，我们照它的思路但保留 u0 前馈）：
    u = kp*err + ki*i_sum + u0*sign(err)     err 出死区：前馈负责起动、积分补残余
    u = ki*i_sum                              err 在死区：只保留积分，不松手（关键）
  · 死区内**冻结**积分（不清零），保留保持力矩
  · 输出被限幅时**撤掉本拍累加**（照 Polaris 的简单抗饱和）
  · 0→1 使能边沿 / 失能 / 保护锁存 ⇒ 清 i_sum
  · `ki = 0` 时 i_sum 恒 0 ⇒ **逐字节等价于改动前**

量纲：`ki` 单位 duty/(度·拍)，5 ms 一拍 ⇒ 1° 恒误差每秒补 `ki*200` duty；
      静差消除时间 `t = kp/(200*ki)`，与误差大小无关（残余 ∝e、积分速率也 ∝e）。
      ki=10 ⇒ 0.15 s；ki=20 ⇒ 0.075 s。安全下限 t ≥ 30~50 ms（5 ms 采样 + 20 ms 编码器滞后）。
"""
import os
import sys

D = r"D:/jisuyueye9car/A_zjc_project/10.ins_route_rc_cyt4bb7/steer_loop_wifi/code"


def rd(p):
    d = open(p, "rb").read()
    crlf = (d.count(b"\r\n") > 0 and d.count(b"\r\n") == d.count(b"\n"))
    return d.decode("utf-8").replace("\r\n", "\n"), crlf


def wr(p, t, crlf):
    if crlf:
        t = t.replace("\n", "\r\n")
    open(p, "wb").write(t.encode("utf-8"))


def sub1(t, old, new, tag):
    n = t.count(old)
    if n != 1:
        print("[ERR] %s 锚点命中 %d 次（应为 1）" % (tag, n))
        sys.exit(2)
    return t.replace(old, new, 1)


# ==================================================================== 1) steer_ctrl.h
p = os.path.join(D, "steer_ctrl.h")
t, crlf = rd(p)

# 1a) 宏
old = """#define STEER_KP_DEFAULT            (300.0f)    /* 比例增益   单位 duty/度   (队友实车验证) */
#define STEER_U0_DEFAULT            (1600.0f)   /* 静摩擦前馈 单位 duty      (队友实车验证) */"""
new = """#define STEER_KP_DEFAULT            (300.0f)    /* 比例增益   单位 duty/度   (队友实车验证) */
#define STEER_U0_DEFAULT            (1600.0f)   /* 静摩擦前馈 单位 duty      (队友实车验证) */

/* ★★ 积分项（2026-10-18 新增）—— 专治"静止角随机"
 *
 *  ---- 为什么必须加 ----------------------------------------------------------------
 *   旧律 `u = kp*err + u0*sign(err)` 有个致命毛病：误差进 0.2° 死区后 `err` 被清 0
 *   ⇒ `u` 也变 0 ⇒ 舵机只能靠**摩擦自锁**。自锁位置取决于此刻摩擦的方向和大小
 *   ⇒ 每次上电/每次使能后的静止角都不一样（实测散布 ±0.2° 以上）。
 *   0.2° 舵角误差 ⇒ κ = tan(0.2°)/0.66 = 0.0053 1/m ⇒ 30 m 赛道横偏 **2.4 m**。
 *   ⇒ "看着居中却整体往某边偏、而且每次都不同"就是它。
 *
 *  ---- 积分怎么解决 ---------------------------------------------------------------
 *   死区内 `err = 0` 但 **i_sum 冻结不清**，输出保持 `ki*i_sum`
 *   ⇒ 舵机被持续压住，不再靠摩擦自锁 ⇒ 静止角由积分决定，**可重复**。
 *   Polaris 的内环就是位置式 PID（Ki=0.5 duty/count、Imax=2000 duty），实测静差 3 count≈0.07°。
 *
 *  ---- 取值（我们自己的量纲，度而不是 count）----------------------------------------
 *   `ki` 单位 duty/(度·拍)，5 ms/拍 ⇒ 1° 恒误差每秒补 `ki*200` duty。
 *   静差消除时间 `t = kp/(200*ki)`，**与误差大小无关**（残余 ∝e、积分速率也 ∝e）：
 *       ki= 5 ⇒ 0.30 s   ki=10 ⇒ 0.15 s   ki=20 ⇒ 0.075 s   ki=40 ⇒ 0.038 s
 *   起点建议 **10~20**；太快会被 5 ms 采样 + 20 ms 编码器滞后拖成抖动（t 不要低于 30~50 ms）。
 *   Polaris 的 0.5 duty/(count·拍) 换算到度约 22 duty/(度·拍)，我们是同一量级。
 *
 *  ---- `imax` 是什么 --------------------------------------------------------------
 *   **积分项（不是累加值）的上限**，单位 duty；累加值上限 = `imax/ki`（照 Polaris 的写法）。
 *   前馈已给 u0=1600、静摩擦实测约 950、保持力矩约 1200~2500 ⇒ 残余最多 ±900
 *   ⇒ 默认 **1000** 就够，别给太大（否则退出时回程滞后）。
 *
 *  ⚠ `ki = 0` 时 i_sum 恒 0 ⇒ **逐字节等价于加积分之前的行为**。
 *    所以要验证"改动没影响"，先烧 ki=0 跑一遍，波形应当完全一样。 */
#define STEER_KI_DEFAULT            (0.0f)      /* 积分增益 duty/(度·拍)；0 = 关（默认） */
#define STEER_IMAX_DEFAULT          (1000.0f)   /* 积分项上限 duty */"""
t = sub1(t, old, new, "steer_ctrl.h 宏")

# 1b) struct 字段
old = """    float   slew_deg_s;         /* 目标变化率上限 度/秒(0 = 关) */
    float   limit_deg;          /* 目标软限位 度 */"""
new = """    float   slew_deg_s;         /* 目标变化率上限 度/秒(0 = 关) */
    float   limit_deg;          /* 目标软限位 度 */
    float   ki;                 /* ★ 积分增益 duty/(度·拍)；0 = 关（见 STEER_KI_DEFAULT 长注释）*/
    float   i_max;              /* ★ 积分**项**上限 duty（累加值上限 = i_max/ki）*/"""
t = sub1(t, old, new, "steer_ctrl.h struct 参数")

old = """    float   err;                /* 本拍误差(度, 已过死区与越限保护) */
    float   u_ff;               /* 本拍静摩擦前馈分量(duty) */"""
new = """    float   err;                /* 本拍误差(度, 已过死区与越限保护) */
    float   u_ff;               /* 本拍静摩擦前馈分量(duty) */
    float   i_sum;              /* ★ 积分累加值(度·拍)；死区内**冻结**不清 —— 它就是保持力矩的记忆 */"""
t = sub1(t, old, new, "steer_ctrl.h struct 状态")

# 1c) setter 声明（放在 set_angle_pid 后面）
old = """void    steer_ctrl_set_angle_pid    (float kp);"""
new = """void    steer_ctrl_set_angle_pid    (float kp);
void    steer_ctrl_set_angle_i      (float ki, float i_max);   /* 0,0 = 关掉积分并清零 */"""
t = sub1(t, old, new, "steer_ctrl.h setter")

wr(p, t, crlf)
print("[1/3] steer_ctrl.h  OK")

# ==================================================================== 2) steer_ctrl.c
p = os.path.join(D, "steer_ctrl.c")
t, crlf = rd(p)
assert not crlf, "steer_ctrl.c 意外变成 CRLF，先核对"

# 2a) init
old = """    steer.head_kp      = STEER_HEAD_KP_DEFAULT;
"""
new = """    steer.head_kp      = STEER_HEAD_KP_DEFAULT;
    steer.ki           = STEER_KI_DEFAULT;
    steer.i_max        = STEER_IMAX_DEFAULT;
"""
t = sub1(t, old, new, "steer_ctrl.c init 参数")

old = """    steer.u_ff          = 0.0f;
    steer.output_duty   = 0;"""
new = """    steer.u_ff          = 0.0f;
    steer.i_sum         = 0.0f;
    steer.output_duty   = 0;"""
t = sub1(t, old, new, "steer_ctrl.c init 状态")

# 2b) 门控清积分（失能/保护/急停）
old = """        steer.err           = 0.0f;
        steer.u_ff          = 0.0f;
        steer.output_duty   = 0;
        steer.was_enabled   = 0;
        return;"""
new = """        steer.err           = 0.0f;
        steer.u_ff          = 0.0f;
        steer.i_sum         = 0.0f;         /* 失能/保护/急停: 清积分记忆，下次使能从 0 学 */
        steer.output_duty   = 0;
        steer.was_enabled   = 0;
        return;"""
t = sub1(t, old, new, "steer_ctrl.c 门控清积分")

# 2c) 控制律：死区注释 + P+I+前馈
old = """    /* 7) 误差死区: 抑制静止抖动 */
    if ((err < steer.deadband_deg) && (err > -steer.deadband_deg))
    {
        err = 0.0f;
    }

    /* 8) 位置环 + 静摩擦前馈 + 输出限幅
       u = kp * err   : 细调
       u0 * sign(err) : 克服静摩擦与减速箱阻力
       err 已过死区与越限保护, 等于 0 时 u0 也不加, 静止不会抖 */
    u = steer.kp * err;

    steer.u_ff = 0.0f;
    if      (err > 0.0f) { steer.u_ff =  steer.u0; }
    else if (err < 0.0f) { steer.u_ff = -steer.u0; }
    u += steer.u_ff;"""
new = """    /* 7) 误差死区: 抑制静止抖动
       ★ 2026-10-18：err 清 0，但**积分不清** —— 死区外的保持力矩由 i_sum 记住并继续输出
          （旧版这里清 0 之后 u 也变 0 ⇒ 靠摩擦自锁 ⇒ 静止角随机，见 STEER_KI_DEFAULT 注释）。 */
    if ((err < steer.deadband_deg) && (err > -steer.deadband_deg))
    {
        err = 0.0f;
    }

    /* 8) 位置环 = P + I + 静摩擦前馈
       u = kp*err      : 比例（细调）
       + ki*i_sum      : 积分（★ 2026-10-18 新增；吃静摩擦/左右不对称的残余，死区内靠它保持）
       + u0*sign(err)  : 静摩擦前馈（起动那一脚，0 拍到位；Polaris 纯 PID 要 215 ms 才动）
       err 已过死区与越限保护, 等于 0 时 u0 不加、积分也**不累加**（冻结），但**照样输出**。 */
    if (0.0f != err)
    {
        steer.i_sum += err;                     /* 死区内冻结（不清），保留保持力矩 */
    }

    if (steer.ki > 1.0e-6f)
    {
        float ilim = steer.i_max / steer.ki;    /* 积分项上限 -> 累加值上限（照 Polaris） */

        if (steer.i_sum >  ilim) { steer.i_sum =  ilim; }
        if (steer.i_sum < -ilim) { steer.i_sum = -ilim; }
    }
    else
    {
        steer.i_sum = 0.0f;                     /* ki=0: 彻底关掉 -> 逐字节等价于旧行为 */
    }

    u = steer.kp * err + steer.ki * steer.i_sum;

    steer.u_ff = 0.0f;
    if      (err > 0.0f) { steer.u_ff =  steer.u0; }
    else if (err < 0.0f) { steer.u_ff = -steer.u0; }
    u += steer.u_ff;"""
t = sub1(t, old, new, "steer_ctrl.c 控制律")

# 2d) 抗饱和（限幅时撤本拍累加）
old = """    if (u >  steer.u_max) { u =  steer.u_max; }
    if (u < -steer.u_max) { u = -steer.u_max; }"""
new = """    /* ★ 抗积分饱和（照 Polaris）：输出被限幅时，把本拍刚累加的那一份撤回去。
       不这么做的话，满舵/大阶跃期间积分会一路积到 imax，退出时表现为"回程滞后"。 */
    if (u >  steer.u_max) { u =  steer.u_max; if (0.0f != err) { steer.i_sum -= err; } }
    if (u < -steer.u_max) { u = -steer.u_max; if (0.0f != err) { steer.i_sum -= err; } }"""
t = sub1(t, old, new, "steer_ctrl.c 抗饱和")

# 2e) 使能 0->1 清积分
old = """        steer.delta  = 0.0f;"""
# 占位（不存在），改用真实锚点
old = """        steer_ctrl_clear_fault();"""
new = """        steer_ctrl_clear_fault();

        /* ★ 清积分记忆：0->1 上升沿是"重新开始"，上一轮的保持力矩不该带过来。
           （标定时靠 `内环关 -> 内环开` 给每段一个干净的起点；想测"稳态保持"就别关。） */
        steer.i_sum = 0.0f;"""
t = sub1(t, old, new, "steer_ctrl.c 使能清积分")

# 2f) setter 实现（紧跟 set_angle_pid）
old = """void steer_ctrl_set_angle_pid (float kp)
{"""
new = """void steer_ctrl_set_angle_i (float ki, float i_max)
{
    if (ki < 0.0f)      { ki = 0.0f; }
    if (i_max < 0.0f)   { i_max = 0.0f; }

    /* 关掉积分时顺手清零 —— 否则下次打开会带着旧账（一次 5 ms 里的大积分 = 一次猛推） */
    if (ki <= 1.0e-6f)  { steer.i_sum = 0.0f; }

    steer.ki    = ki;
    steer.i_max = i_max;
}

void steer_ctrl_set_angle_pid (float kp)
{"""
t = sub1(t, old, new, "steer_ctrl.c setter")

wr(p, t, crlf)
print("[2/3] steer_ctrl.c  OK")

# ==================================================================== 3) motor_link.h
p = os.path.join(D, "motor_link.h")
t, crlf = rd(p)
assert crlf, "motor_link.h 应为 CRLF"

old = """    X(58, "df_max",     "差速|单侧偏差上限",  0.100f,    0.950f,   SPD_DIFF_MAX_DEFAULT, "-", 0.010f, &spd_diff_max ) """
new = """    X(58, "df_max",     "差速|单侧偏差上限",  0.100f,    0.950f,   SPD_DIFF_MAX_DEFAULT, "-", 0.010f, &spd_diff_max ) \\
    X(59, "steer_ki",   "转向|积分 Ki",        0.000f,  200.000f,   STEER_KI_DEFAULT,   "-",     1.000f, &steer.ki      ) \\
    X(60, "steer_imax", "转向|积分上限",       0.000f, 4000.000f,   STEER_IMAX_DEFAULT, "duty", 50.000f, &steer.i_max   ) """
t = sub1(t, old, new, "motor_link.h ID59/60")
wr(p, t, crlf)
print("[3/3] motor_link.h  OK")

print("done")
