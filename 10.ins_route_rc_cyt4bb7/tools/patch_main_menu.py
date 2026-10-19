"""把"屏幕菜单 + 12 段跑班"接进 user/main_cm7_0.c 与 user/cm7_0_isr.c。

⚠⚠ 这两个文件是 **GBK**（`code/` 才是 UTF-8），而且**行尾还不一样**：
      main_cm7_0.c = CRLF ，cm7_0_isr.c = LF
   ⇒ 一律按字节改，且**插进去的行用该文件自己的行尾**。

⚠⚠⚠ **踩过的坑（本脚本第一版就是死在这）**：
   Python 的**字节字面量 `b'...'` 里 `\\uXXXX` 不会转义** —— 会原样写进文件，
   中文注释变成字面 `\\u5de5\\u7a0b`。虽然它在 C 注释里不影响编译，但可读性全毁。
   ⇒ 正确做法：正文写成 **str**（真中文），最后 `.encode('gbk')`。

用法：
    python patch_main_menu.py            # 只报告
    python patch_main_menu.py --apply    # 真改
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
USER = os.path.normpath(os.path.join(HERE, "..", "steer_loop_wifi", "user"))

MAIN = os.path.join(USER, "main_cm7_0.c")
ISR = os.path.join(USER, "cm7_0_isr.c")

# ---------------- main_cm7_0.c ----------------
M_INC_ANCHOR = '#include "ins_odom.h"'
M_INC_AFTER = [
    "",
    "/* ---- 工程 10 新增：屏幕菜单 + 12 段跑班 ---- */",
    '#include "flash_map.h"',
    '#include "app_params.h"',
    '#include "bench_log.h"',
    '#include "spd_bench.h"',
    '#include "ui_core.h"',
    '#include "ui_menu.h"',
]

M_INIT_ANCHOR = "    pit_ms_init(PIT_CH2, 5);"
M_INIT_BEFORE = [
    "    /* 参数持久化：从 Work Flash 页 0 读回上次 SAVE 的参数。",
    "       必须在各模块 init 之后、开 5ms 中断之前 —— 跑起来就是你上次存的增益。",
    "       [!] 命令/危险量（target / ol_en / ol_duty / steer_en / ins_zero）**不恢复**，",
    "           上电永远是安全值，见 app_params.c 的 ap_is_cmd()。 */",
    "    app_param_init();",
    "",
    "    /* 12 段跑班：初始化状态（开环一律关掉，不会自己跑）*/",
    "    spd_bench_init();",
    "",
]

M_LOOP_ANCHOR = "        remote_ctrl_apply();"
M_LOOP_AFTER = [
    "",
    "        /* 2d) 工程 10 新增：12 段跑班收尾 + 日志落盘 + 屏幕菜单。",
    "               放在心跳之后：菜单刷屏最差 ~40 ms，不能把心跳饿死（200 ms 门）。",
    "               bench_log_poll() 每圈**最多写一页 Flash**，不长阻塞。 */",
    "        spd_bench_poll();",
    "        bench_log_poll();",
    "        ui_menu_task();",
]

# ---------------- cm7_0_isr.c ----------------
I_INC_ANCHOR = '#include "ins_odom.h"'
I_INC_AFTER = ['#include "spd_bench.h"']

I_TICK_ANCHOR = "    ins_odom_update();"
I_TICK_AFTER = [
    "",
    "    // ---- 工程 10: 12 段跑班（一键跑班）----",
    "    //  必须在 spd_ctrl_tick() 之后：开环占空比是本函数写进 spd_ol_duty_* 的，",
    "    //  而 spd_ctrl_tick 才是真正把它变成 PWM 的地方。",
    "    //  内部每 5ms 复核四道安全门（遥控在线 / STANDBY / CH5 急停 / 保护层），",
    "    //  任一条破了立刻回 0 duty + 退出开环 + 中止记录。没在跑班时它直接 return。",
    "    spd_bench_tick();",
]

JOBS = [
    (MAIN, "M_INC",   M_INC_ANCHOR,  M_INC_AFTER,  "after"),
    (MAIN, "M_INIT",  M_INIT_ANCHOR, M_INIT_BEFORE, "before"),
    (MAIN, "M_LOOP",  M_LOOP_ANCHOR, M_LOOP_AFTER,  "after"),
    (ISR,  "I_INC",   I_INC_ANCHOR,  I_INC_AFTER,  "after"),
    (ISR,  "I_TICK",  I_TICK_ANCHOR, I_TICK_AFTER, "after"),
]


def xml(p, tag, anchor, lines, where, eol, apply):
    raw = open(p, "rb").read()
    a = anchor.encode("gbk")
    body = eol.join(s.encode("gbk") for s in lines)

    if "after" == where:
        old, new = a, a + eol + body
    else:
        old, new = a, body + eol + a

    n = raw.count(old)
    done = raw.count(new) > 0
    print("  %-7s %-16s 命中 %d 次%s" % (tag, os.path.basename(p), n, "（已改过）" if done else ""))

    if done and n == 0:
        return True
    if n != 1:
        print("          -> ❌ 预期恰好 1 次，实际 %d 次，**不动它**" % n)
        return False
    if not apply:
        return True

    open(p + ".tmp_menu", "wb").write(raw.replace(old, new, 1))
    os.replace(p + ".tmp_menu", p)
    return True


def main():
    apply = "--apply" in sys.argv
    ok = True

    # 每个文件按自己的行尾
    eols = {}
    for p in (MAIN, ISR):
        raw = open(p, "rb").read()
        eols[p] = b"\r\n" if raw.count(b"\r\n") > 0 else b"\n"
        print("%-16s 行尾 = %s" % (os.path.basename(p), "CRLF" if eols[p] == b"\r\n" else "LF"))

    for p, tag, anchor, lines, where in JOBS:
        if not xml(p, tag, anchor, lines, where, eols[p], apply):
            ok = False

    if not apply:
        print("\n（只报告。加 --apply 才真写）")
        return 0

    print()
    for p in (MAIN, ISR):
        raw = open(p, "rb").read()
        try:
            raw.decode("gbk")
            enc = "GBK OK"
        except Exception as e:
            enc = "GBK FAIL %s" % e
        print("回读 %-16s %s  CRLF=%d LF=%d  \\u乱码=%d"
              % (os.path.basename(p), enc, raw.count(b"\r\n"), raw.count(b"\n"),
                 raw.count(b"\\u5de5")))

    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
