# -*- coding: utf-8 -*-
"""把速度环「摩擦前馈 u0」的默认值从台架空载值 0.033 改成户外落地实测 5.7%。

依据（2026-10-18 户外 11 段开环拟合，R²=0.9986）：
    左轮 u0 = 5.82%   右轮 u0 = 5.54%   ⇒ 取 5.7%（0.057）
    旧默认 0.033 是**台架空载**的截距；现场手填的 0.0619 偏大 0.5% duty
    ⇒ 0.5% 在 1.0 rps 段相当于 3.5% 速度（3.0 rps 段只占 1.2%）⇒ 该段 +15.7% 超调的主因。

改四处（必须同源，否则点一次「恢复默认值」行为就变、波形上还看不出）：
    1. 车端  spd_ctrl.h     SPD_U0_DEFAULT        （CRLF 文件）
    2. 上位机 Defaults.cs   只在 BuildKartInsParams() 里覆写 ID7（工程 9 的 0.033 不动）
    3. 上位机 SelfTest.cs   默认值断言（否则自检直接红）
    4. 手机 App MainActivity.java  输入框初值 + parseF 兜底
"""
import os
import sys

ROOT10 = r"D:/jisuyueye9car/A_zjc_project/10.ins_route_rc_cyt4bb7/steer_loop_wifi/code"
HOST = r"D:/jisuyueye9car/KartHost"
APP = r"D:/jisuyueye9car/KartPhone/app/src/com/zjc/kartphone"


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


ok = 0

# ---------------------------------------------------------------- 1) 车端
p = os.path.join(ROOT10, "spd_ctrl.h")
t, crlf = rd(p)
old = """#define SPD_INV_K_DEFAULT       (0.1548f)       /* 1/6.46 */
#define SPD_U0_DEFAULT          (0.033f)"""
new = """#define SPD_INV_K_DEFAULT       (0.1548f)       /* 1/6.46（**台架空载**标定值）
                                                   ⚠ 户外落地实测 K=6.9~7.0 rps/unit ⇒ 1/K = 0.143~0.145（现跑 0.1452），
                                                     与台架差 6% —— 但**没改**：这 6% 正是前馈永远做不准的那部分，
                                                     稳态靠积分兜（实测各段稳态误差 ≤2.3%）。 */
/*  ★ 2026-10-18 户外落地定稿：0.033 → **0.057**
 *  ---- 依据（11 段开环，R² = 0.9986）------------------------------------------------------
 *      左右轮分别拟合：u0 = 5.82% / 5.54% ⇒ 取 **5.7%**。
 *      `u_ff = inv_K·v[rps] + u0`（用 3.0 rps 段验证：0.1452×3 + 0.0619 = 0.4975 vs 实测 0.497）。
 *  ---- 为什么必须改 ----------------------------------------------------------------
 *      0.033 是**空转（无负载）**的摩擦截距；落地后轮胎/减速箱/地面阻力把截距抬了 0.24% duty。
 *      这 0.24% 在 3.0 rps 段只相当于 0.6% 速度，但在 **1.0 rps 段相当于 3.5%** ⇒
 *      低速段前馈长期过量 ⇒ 实测 0→1.0 rps 阶跃超调 **+15.7%**，而 1.5/3.0 段只有 5.8/6.7%。
 *      现场手填的 0.0619 又比实测偏大 0.5% duty，方向对但过头了。
 *  ---- 纪律 ---------------------------------------------------------------------
 *      ⚠ 前馈只能定"截距"，别指望它把稳态做准：同 duty 重复性实测 4~8%（地面阻力/转弯）
 *        ⇒ 稳态永远靠积分。这个值错了只影响**超调**和**起步那一脚**。
 *      ⚠ Flash 里存过参数的话会覆盖本默认值（`app_param_init` 认 magic/ver/count 就采用）⇒
 *        烧完上电可能仍是旧值，要看串口/参数面板确认。 */
#define SPD_U0_DEFAULT          (0.057f)"""
t = sub1(t, old, new, "spd_ctrl.h SPD_U0_DEFAULT")
wr(p, t, crlf)
print("[1/4] spd_ctrl.h  OK")
ok += 1

# ---------------------------------------------------------------- 2) 上位机默认值
p = os.path.join(HOST, "Defaults.cs")
t, crlf = rd(p)
old = """        var duty = list.First(p => p.Id == 2);
        duty.Def = 60f;
        duty.Value = 60f;
"""
new = """        var duty = list.First(p => p.Id == 2);
        duty.Def = 60f;
        duty.Value = 60f;

        // ★ 2026-10-18：u0 默认 0.033（台架空载）→ **0.057**（户外落地实测左 5.82 / 右 5.54）。
        //   同样只覆写**本档案**：工程 9 的 BuildKartRCParams() 保持 0.033（那是台架标定值）。
        //   ⚠ 现场手填过 0.0619 —— 偏大 0.5% duty，在 1.0 rps 段相当于 3.5% 速度，
        //     正是该段 +15.7% 超调的主因（3.0 rps 段只占 1.2%，超调 6.7%）。
        //   与车端 SPD_U0_DEFAULT 必须同源，否则点一次「恢复默认值」就回到 0.033。
        var u0p = list.First(p => p.Id == 7);
        u0p.Def = 0.057f;
        u0p.Value = 0.057f;
"""
t = sub1(t, old, new, "Defaults.cs BuildKartInsParams")
wr(p, t, crlf)
print("[2/4] Defaults.cs  OK")
ok += 1

# ---------------------------------------------------------------- 3) 上位机自检断言
p = os.path.join(HOST, "SelfTest.cs")
t, crlf = rd(p)
old = """        Check(Math.Abs(ps[6].Def - 0.1548f) < 1e-4f && Math.Abs(ps[7].Def - 0.033f) < 1e-4f,
              "inv_K / u0 默认 = 0.1548 / 0.033（与车端 SPD_*_DEFAULT 同源）",
              $"inv_K={ps[6].Def} u0={ps[7].Def}");"""
new = """        // ★ 2026-10-18：u0 改成户外落地实测 5.7%（台架 0.033 在落地会偏小 0.24% duty ⇒ 低速段超调）。
        Check(Math.Abs(ps[6].Def - 0.1548f) < 1e-4f && Math.Abs(ps[7].Def - 0.057f) < 1e-4f,
              "inv_K / u0 默认 = 0.1548（台架）/ 0.057（落地实测 5.7%，与车端 SPD_U0_DEFAULT 同源）",
              $"inv_K={ps[6].Def} u0={ps[7].Def}");"""
t = sub1(t, old, new, "SelfTest.cs u0 断言")
wr(p, t, crlf)
print("[3/4] SelfTest.cs  OK")
ok += 1

# ---------------------------------------------------------------- 4) 手机 App
p = os.path.join(APP, "MainActivity.java")
t, crlf = rd(p)
old = """        etInvK = numEdit("0.1452");
        etU0 = numEdit("0.0619");"""
new = """        etInvK = numEdit("0.1452");
        etU0 = numEdit("0.057");        // ★ 落地实测 5.7%（台架 0.033 偏小、现场手填的 0.0619 偏大）"""
t = sub1(t, old, new, "MainActivity etU0 初值")
old = """        final float u0 = parseF(etU0, 0.0619f, 0f, 0.3f);"""
new = """        final float u0 = parseF(etU0, 0.057f, 0f, 0.3f);"""
t = sub1(t, old, new, "MainActivity parseF u0")
wr(p, t, crlf)
print("[4/4] MainActivity.java  OK")
ok += 1

print("done, %d/4" % ok)
