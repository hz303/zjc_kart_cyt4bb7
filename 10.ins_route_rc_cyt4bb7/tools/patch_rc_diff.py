#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
patch_rc_diff.py —— 给「遥控手动开车」补上电子差速

背景
    差速目前只接在 route_pb.c（回放）里；remote_ctrl.c 的 RC_ENGAGE_ON 分支
    仍然是 spd_target_l = spd_target_r = RC_Target_Rps（两轮同目标）。
    ⇒ 遥控开车时内轮照样被拖，里程照样虚高 +22%，而且没法在台架上手动验证差速。

改动（只动 3 处，语义不变）
    ① include ins_odom.h（ins_m_per_rev）/ route_pb.h（pb_l，与回放共用同一个有效轴距）
    ② RC_ENGAGE_ON 分支：把"两轮同一个数"换成 spd_diff_alloc 分配
    ③ ⚠ 单位：spd_target_* 是 rps，spd_diff_alloc 吃 m/s ⇒ 进来乘 m_per_rev、出去除回去

不动的东西
    · STANDBY / OFF / ESTOP / FAILSAFE 的"不写目标"分支一个字不碰
      （那是 2026-10-13 那次"上位机滑条全废"的教训，见 remote_ctrl.c 的长注释）
    · 只在真·开车（ON）时写目标，授权关系不变

用法
    python tools/patch_rc_diff.py
"""

import os
import sys

D = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "steer_loop_wifi", "code")
P = os.path.join(D, "remote_ctrl.c")

INC_OLD = '''#include "motor_drv.h"
'''
INC_NEW = '''#include "motor_drv.h"
#include "ins_odom.h"           /* ins_m_per_rev：rps <-> m/s */
#include "route_pb.h"           /* pb_l：与回放共用同一个有效轴距 */
'''

BODY_OLD = '''            RC_Target_Rps = rc_throttle_to_target();
            steer.target_delta = RC_Steer_Deg;
            spd_target_l = RC_Target_Rps;
            spd_target_r = RC_Target_Rps;
            break;'''

BODY_NEW = '''            RC_Target_Rps = rc_throttle_to_target();
            steer.target_delta = RC_Steer_Deg;

            /* ★ 电子差速（与回放同一条律，见 `工程10-控制层-航向追随与电子差速.md`）
                  κ = tan(实测δ)/L，v_l = v(1−κW/2)、v_r = v(1+κW/2)
               ⚠⚠ **单位**：`spd_target_*` 是 rps，而 `spd_diff_alloc` 吃 m/s
                  ⇒ 进来乘 `m_per_rev`、出去除回去。漏一边就是 1.8 倍误差（cpr 不是 1）。
               ⚠ 用**实测**舵角（不是目标角）：目标角会让后轮在舵机到位之前就提前差速。
               ⚠ 这句话**不改变授权语义**：仍然只在"遥控真的在开车"时写目标；
                 STANDBY/OFF 的"不写"分支原样保留（见本文件前面的长注释）。 */
            {
                float mpr   = (ins_m_per_rev > 1e-4f) ? ins_m_per_rev : 0.8193f;
                float v_mps = RC_Target_Rps * mpr;
                float vl    = v_mps;
                float vr    = v_mps;

                spd_diff_alloc(v_mps, pb_l, steer_ctrl_get_meas_delta(), &vl, &vr);

                spd_target_l = vl / mpr;
                spd_target_r = vr / mpr;
            }
            break;'''


def main():
    raw = open(P, "rb").read()
    try:
        text = raw.decode("utf-8")
    except UnicodeDecodeError:
        print("[ERR] remote_ctrl.c 不是 UTF-8 —— 本脚本只按 UTF-8 处理，先人工核对")
        return 2

    if "\r\n" in text:
        print("[ERR] 文件是 CRLF —— 本工程该文件一直是 LF，先人工核对再改")
        return 2

    if "spd_diff_alloc(v_mps, pb_l, steer_ctrl_get_meas_delta(), &vl, &vr);" in text:
        print("[SKIP] 已经打过这个补丁（找到 spd_diff_alloc 调用）")
        return 0

    n_body = text.count(BODY_OLD)
    if 1 != n_body:
        print("[ERR] 正文锚点命中 %d 次（期望 1）—— 文件结构变了，先人工核对" % n_body)
        return 3

    if 1 != text.count(INC_OLD):
        print("[ERR] include 锚点不唯一 —— 先人工核对")
        return 3

    text = text.replace(INC_OLD, INC_NEW, 1)
    text = text.replace(BODY_OLD, BODY_NEW, 1)

    tmp = P + ".tmp"
    open(tmp, "wb").write(text.encode("utf-8"))
    os.replace(tmp, P)

    print("[OK] remote_ctrl.c 已补差速")
    print("     + include ins_odom.h / route_pb.h")
    print("     + RC_ENGAGE_ON: 两轮同目标 -> spd_diff_alloc（rps<->m/s 已换算）")
    return 0


if __name__ == "__main__":
    sys.exit(main())
