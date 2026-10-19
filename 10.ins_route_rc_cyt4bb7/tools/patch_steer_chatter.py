# -*- coding: utf-8 -*-
"""去掉转向内环在死区边界上的输出台阶（2026-10-18 晚）。

实测（wave_20261018_2249，Kp=320/U0=1500/Ki=120/db=0.2）：
    目标恒 0、遥控恒 992、舵机静止不动，而 u 逐拍在 -727/-799/-871/-942/-1014
    与 +580/+609/+667 之间翻向 —— 200 Hz、±800 duty 的抖动，实测角只动 ±1 个计数。

根因三条（都在 steer_ctrl.c 的控制律里）：
    ① 死区把 `err` 清 0，而 `u0*sign(err)` 用的是同一个 err
       ⇒ 误差一跨死区边界，前馈从 ±1500 直接变 0 ⇒ **输出上 1500 duty 的台阶**；
    ② 积分没有分离：一个大阶跃就把 i_sum 顶到 i_max=1300（实测确认），
       到点后 u = u0(1500) - i_max(1300) ≈ 200，**两个大数相减**，1 个计数就决定符号；
    ③ 死区内 u 只剩 ki*i_sum（如 +1083），变成一个"没有误差反馈的恒推力"，
       到极限/贴背隙时就把舵机顶在那儿不动，回程还要等积分退，形成慢环。

改法：err 不清零（P 项永远真实），改用三态滞环决定前馈方向；积分分离 + 上限降到 i_max。
"""
import os
import sys

D = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..",
                 "steer_loop_wifi", "code")
D = os.path.abspath(D)


def load(name):
    p = os.path.join(D, name)
    raw = open(p, "rb").read()
    crlf = raw.count(b"\r\n")
    lf = raw.count(b"\n")
    text = raw.decode("utf-8")
    if crlf:
        if crlf != lf:
            print("[ERR] %s 行尾混用 (CRLF %d / LF %d)" % (name, crlf, lf))
            sys.exit(2)
        text = text.replace("\r\n", "\n")
    return p, text, (crlf != 0)


def save(p, text, crlf):
    if crlf:
        text = text.replace("\n", "\r\n")
    open(p, "wb").write(text.encode("utf-8"))


# ============================================================ 1) steer_ctrl.h
p, s, crlf = load("steer_ctrl.h")

old = """ *  ⚠ `ki = 0` 时 i_sum 恒 0 ⇒ **逐字节等价于加积分之前的行为**。
 *    所以要验证"改动没影响"，先烧 ki=0 跑一遍，波形应当完全一样。 */
#define STEER_KI_DEFAULT            (0.0f)      /* 积分增益 duty/(度·拍)；0 = 关（默认） */
#define STEER_IMAX_DEFAULT          (1000.0f)   /* 积分项上限 duty */"""

new = """ *  ⚠ `ki = 0` 时 i_sum 恒 0 ⇒ 只剩 P + 前馈（等价于 2026-10-18 白天那版）。
 *
 *  ---- 2026-10-18 晚：默认值从 0 改成 12，i_max 1000 → 600 -----------------------------
 *    白天把默认留 0 是为了"先烧 0 验证等价"。实车用滑条调过一轮（Ki=120）之后暴露出
 *    **结构问题**（见下面 STEER_BRK_DEG_DEFAULT 的说明）：Ki 太大 ⇒ 积分一个大阶跃就顶到
 *    i_max ⇒ 到点后 u = u0 - i_max ≈ 200，两个大数相减 ⇒ 逐拍翻向。所以：
 *      · Ki 收到 **12**：静差消除时间 `t = kp/(200·ki)` = 320/2400 ≈ **130 ms**，
 *        而 i_max/ki = 50 度·拍，即 1° 误差要积 50 拍(250 ms)才满 —— 正常工况根本到不了上限；
 *      · i_max 收到 **600**：约 40% 的 u0。判据是 **i_max + kp·deadband < 破冰力**，
 *        这样即使积分积满、方向又错，也不足以把舵机推动（地面实测破冰力 ≥ 950 duty）。 */
#define STEER_KI_DEFAULT            (12.0f)     /* 积分增益 duty/(度·拍)；0 = 关 */
#define STEER_IMAX_DEFAULT          (600.0f)    /* 积分**项**上限 duty（累加值上限 = i_max/ki = 50 度·拍） */

/* ★★ 起步滞环（2026-10-18 晚新增，替代"死区里把 u0 一起关掉"）
 *
 *  ---- 旧结构为什么抖 ----------------------------------------------------------------
 *      旧版：`|err| < deadband` ⇒ err 清 0 ⇒ `u0*sign(err)` 也跟着变 0
 *      ⇒ **输出在死区边界上有一个 u0（本车 1500 duty）的台阶**。
 *      于是 1 个编码器计数（0.02°）的抖动就能让输出从 +667 跳到 -1014（实测原样），
 *      200 Hz 抖、舵机听得到"嗡"、实测角在 ±1~2 个计数间跳。
 *      ⇒ 这不是"增益太大"，是**输出对被控量不连续**，调任何增益都治不好。
 *
 *  ---- 新结构 ----------------------------------------------------------------
 *      |err| > brk      -> 前馈给满 u0（0 拍起动，保留"比 Polaris 快 215 ms"的优点）
 *      |err| < deadband -> 前馈为 0（到位，保持力矩交给积分）
 *      中间就是**滞环带**：1~2 个计数翻不动它 ⇒ 死区边界上不再有台阶，
 *      P 项永远用真实误差（1 个计数 = kp*0.02 = 6.4 duty，无害）⇒ 输出连续。
 *
 *  ⚠ `brk` 必须 > `deadband`（否则滞环带不存在，退化成原来的开关）；
 *    也建议 > **一次"踢"能让舵机走过的角度**（地面实测 0.04~0.1°），否则会变成逐拍踢。 */
#define STEER_BRK_DEG_DEFAULT       (0.20f)     /* 起步门限 度（> deadband） */

/* ★★ 积分分离门限：|err| 超过它就**不积分**（2026-10-18 晚新增）
 *    理由：追赶段（大阶跃）里误差大、持续久，积一分就顶到上限；而到点后那份积分
 *    还在，就变成"顶着目标不松手"的偏置，回程要先等它退（实测回程滞后 0.9 s）。
 *    ⇒ 只在小误差区积分，追赶交给 P + 前馈。这是标准做法（积分分离 PID）。 */
#define STEER_IZONE_DEG_DEFAULT     (4.0f)      /* 积分分离门限 度 */"""

assert s.count(old) == 1
s = s.replace(old, new, 1)

# 死区默认 0.2 → 0.10
old = """#define STEER_DEADBAND_DEG_DEFAULT  (0.2f)      /* 误差死区   单位 度 */"""
new = """/* 误差死区 度。★ 2026-10-18 晚 0.2 → **0.10**：
 *  改动后死区的作用只剩两条：① 冻结积分（别让 ±1 个计数喂积分）② 界定"到位"（撤前馈）。
 *  它**不再**决定静止角（那是滞环 + 积分的活），所以可以收到 0.10°。
 *  ⚠ 残留静差的上界仍 ≈ 死区（0.10° 在 30 m 上是 1.2 m），这一段由**外环航向**吸收 ——
 *    内环的任务是"安静、可重复"，不是"零静差"。 */
#define STEER_DEADBAND_DEG_DEFAULT  (0.10f)     /* 误差死区   单位 度 */"""
assert s.count(old) == 1
s = s.replace(old, new, 1)

# struct 加三个字段
old = """    float   ki;                 /* ★ 积分增益 duty/(度·拍)；0 = 关（见 STEER_KI_DEFAULT 长注释）*/
    float   i_max;              /* ★ 积分**项**上限 duty（累加值上限 = i_max/ki）*/"""
new = """    float   ki;                 /* ★ 积分增益 duty/(度·拍)；0 = 关（见 STEER_KI_DEFAULT 长注释）*/
    float   i_max;              /* ★ 积分**项**上限 duty（累加值上限 = i_max/ki）*/
    float   brk_deg;            /* ★ 起步门限 度（|err| 超过它才给满 u0；见 STEER_BRK_DEG_DEFAULT）*/
    float   izone_deg;          /* ★ 积分分离门限 度（|err| 超过它不积分；见 STEER_IZONE_DEG_DEFAULT）*/"""
assert s.count(old) == 1
s = s.replace(old, new, 1)

old = """    float   u_ff;               /* 本拍静摩擦前馈分量(duty) */"""
new = """    float   u_ff;               /* 本拍静摩擦前馈分量(duty)，= u0 * u_dir，带滞环 */
    int8    u_dir;              /* ★ 前馈方向 -1/0/+1（三态滞环，见 steer_ctrl.c 第 7 步）*/"""
assert s.count(old) == 1
s = s.replace(old, new, 1)

save(p, s, crlf)
print("steer_ctrl.h ok")


# ============================================================ 2) steer_ctrl.c
p, s, crlf = load("steer_ctrl.c")

old = """    steer.ki           = STEER_KI_DEFAULT;
    steer.i_max        = STEER_IMAX_DEFAULT;"""
new = """    steer.ki           = STEER_KI_DEFAULT;
    steer.i_max        = STEER_IMAX_DEFAULT;
    steer.brk_deg      = STEER_BRK_DEG_DEFAULT;
    steer.izone_deg    = STEER_IZONE_DEG_DEFAULT;"""
assert s.count(old) == 1
s = s.replace(old, new, 1)

old = """    steer.i_sum         = 0.0f;
    steer.output_duty   = 0;
    steer.target_yaw    = 0.0f;"""
new = """    steer.i_sum         = 0.0f;
    steer.u_dir         = 0;
    steer.output_duty   = 0;
    steer.target_yaw    = 0.0f;"""
assert s.count(old) == 1
s = s.replace(old, new, 1)

# 局部变量
old = """void steer_ctrl_update (void)
{
    float r;
    float err;
    float u;"""
new = """void steer_ctrl_update (void)
{
    float r;
    float err;
    float u;
    uint8 i_added = 0u;                         /* 本拍是否真的累加了积分（反对饱和要按它撤）*/"""
assert s.count(old) == 1
s = s.replace(old, new, 1)

# 失能分支：清滞环方向
old = """        steer.err           = 0.0f;
        steer.u_ff          = 0.0f;
        steer.i_sum         = 0.0f;         /* 失能/保护/急停: 清积分记忆，下次使能从 0 学 */"""
new = """        steer.err           = 0.0f;
        steer.u_ff          = 0.0f;
        steer.i_sum         = 0.0f;         /* 失能/保护/急停: 清积分记忆，下次使能从 0 学 */
        steer.u_dir         = 0;            /* 前馈方向也清（否则下次使能第一拍会带旧方向）*/"""
assert s.count(old) == 1
s = s.replace(old, new, 1)

# 使能上升沿
old = """        steer.i_sum = 0.0f;
    }

    steer.angle_enable = en ? 1 : 0;"""
new = """        steer.i_sum = 0.0f;
        steer.u_dir = 0;
    }

    steer.angle_enable = en ? 1 : 0;"""
assert s.count(old) == 1
s = s.replace(old, new, 1)

# ===== 控制律主体（第 7~8 步）=====
old = """    /* 7) 误差死区: 抑制静止抖动
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

new = """    /* 7) ★★ 误差死区 + 起步滞环（2026-10-18 晚重写，**这是抖动的根**）
     *
     *  ---- 旧版实测（wave_20261018_2249；Kp=320 / U0=1500 / Ki=120 / db=0.2）------------
     *      目标恒 0°、遥控恒 992、舵机完全静止时，逐拍输出是：
     *          u = -727 / -799 / -871 / -942 / -1014  ↔  +580 / +609 / +667
     *      逐拍翻向、幅度 ±800 duty，而实测角只在 -0.08°（±1~2 个计数）之间跳。
     *
     *  ---- 为什么 -------------------------------------------------------------------
     *      旧版在这一步把 `err` 清 0，而第 8 步的静摩擦前馈 `u0*sign(err)` 用的**就是** err
     *      ⇒ 误差一跨死区边界，前馈从 ±1500 直接掉到 0
     *      ⇒ **输出上有一个 u0（1500 duty）的台阶，正落在稳态度最常待的地方**。
     *      再叠上"积分没有分离"（见第 8 步）：大阶跃把 i_sum 顶到 i_max=1300，
     *      到点后 u = u0(1500) − i_max(1300) ≈ 200 —— **两个大数相减**，
     *      于是 1 个计数（0.02°）的抖动就足以决定符号 ⇒ 抖动时有时无、且常在"极值/中值"
     *      附近出现（那两处刚好是积分被顶满的地方）。
     *
     *  ---- 现在 ---------------------------------------------------------------------
     *      err **不清零**（P 项永远用真实误差，1 个计数 = kp*0.02 = 6.4 duty，无害）
     *      前馈方向改由**三态滞环**决定：
     *          |err| > brk        -> u_dir = ±1   （要动：满前馈，仍然 0 拍起动）
     *          |err| < deadband   -> u_dir = 0    （到位：撤前馈，保持交给积分）
     *          中间（滞环带）      -> u_dir 保持   （1~2 个计数的噪声翻不动它）
     *      ⇒ 输出对 err 连续，台阶消失；静止时不再有 ±1500 的来回跳。 */
    {
        float ae = steer_fabs(err);

        if (ae > steer.brk_deg)
        {
            steer.u_dir = (err > 0.0f) ? 1 : -1;
        }
        else if (ae < steer.deadband_deg)
        {
            steer.u_dir = 0;
        }
        /* 滞环带内：保持上一拍方向 —— 故意什么都不做 */
    }

    /* 8) 位置环 = P + I + 静摩擦前馈
       u = kp*err + ki*i_sum + u0*u_dir
     *
     *  ★ 积分分离：只在 `deadband < |err| < izone` 之间累加。
     *    大误差（追赶段）不积分 ⇒ i_sum 到不了 i_max ⇒ 不会出现
     *    "u0 与 i_max 两个大数相减"那种病态平衡（抖动的第二个条件）。
     *  ★ 死区内**冻结不清**：保持力矩继续输出，这是"每次使能静止角可重复"的来源
     *    （旧旧版死区里 u=0 ⇒ 靠摩擦自锁 ⇒ 静止角随机，30 m 偏 2.4 m，见 2026-10-18 白天）。
     *  ⚠ `ki = 0` 时 i_sum 恒 0 ⇒ 只剩 P + 滞环前馈，等价于白天那版（可用来做 A/B）。 */
    {
        float ae = steer_fabs(err);

        i_added = ((ae > steer.deadband_deg) && (ae < steer.izone_deg)) ? 1u : 0u;
        if (0u != i_added)
        {
            steer.i_sum += err;
        }

        if (steer.ki > 1.0e-6f)
        {
            float ilim = steer.i_max / steer.ki;    /* 积分项上限 -> 累加值上限（照 Polaris） */

            if (steer.i_sum >  ilim) { steer.i_sum =  ilim; }
            if (steer.i_sum < -ilim) { steer.i_sum = -ilim; }
        }
        else
        {
            steer.i_sum = 0.0f;                     /* ki=0: 彻底关掉 */
        }

        steer.u_ff = (0 != steer.u_dir) ? (steer.u0 * (float)steer.u_dir) : 0.0f;
        u = steer.kp * err + steer.ki * steer.i_sum + steer.u_ff;
    }"""
assert s.count(old) == 1
s = s.replace(old, new, 1)

# 反对饱和：只在"本拍真的加过"时才撤
old = """    if (u >  steer.u_max) { u =  steer.u_max; if (0.0f != err) { steer.i_sum -= err; } }
    if (u < -steer.u_max) { u = -steer.u_max; if (0.0f != err) { steer.i_sum -= err; } }"""
new = """    if (u >  steer.u_max) { u =  steer.u_max; if (0u != i_added) { steer.i_sum -= err; } }
    if (u < -steer.u_max) { u = -steer.u_max; if (0u != i_added) { steer.i_sum -= err; } }"""
assert s.count(old) == 1
s = s.replace(old, new, 1)

save(p, s, crlf)
print("steer_ctrl.c ok")


# ============================================================ 3) motor_link.h
p, s, crlf = load("motor_link.h")
BS = chr(92)
old = ('    X(60, "steer_imax", "转向|积分上限",       0.000f, 4000.000f,   STEER_IMAX_DEFAULT, "duty", 50.000f, &steer.i_max   )')
new = (old + ' ' + BS + '\n'
       '    X(61, "steer_brk",  "转向|起步门限",       0.000f,    5.000f,   STEER_BRK_DEG_DEFAULT,   "deg",  0.050f, &steer.brk_deg   ) ' + BS + '\n'
       '    X(62, "steer_izone","转向|积分分离门限",   0.000f,   20.000f,   STEER_IZONE_DEG_DEFAULT, "deg",  0.500f, &steer.izone_deg )')
assert s.count(old) == 1, s.count(old)
s = s.replace(old, new, 1)
save(p, s, crlf)
print("motor_link.h ok")
