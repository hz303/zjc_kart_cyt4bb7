# -*- coding: utf-8 -*-
"""
给 main_cm7_0.c 的 1Hz 打印加"编码器链路"诊断字段。

背景（2026-10-16 现场）：
    reset 后正常 -> 油门打满跑一小段 -> 方向盘晃两下 -> 猛向右打死 -> 此后转盘再也控不动。
    -> 根因是电机干扰编码器 SPI 读数 + 旧保护"只进不出"。这两个已在 steer_abs.c / steer_ctrl.c 里治了，
      本脚本只负责**让它看得见**：把"被拒样数 / 自动复位次数 / 未滤波实读值"打出来。

纪律（见技能 patch-gbk-source）：
    ① 写 str 再 .encode('gbk')，**绝不**把中文塞进 b'...' 字节字面量；
    ② 先 encode('gbk') 预检，编不出就整体失败、不落盘；
    ③ 临时文件 + os.replace 原子替换（io.open('w') 一开就把文件截断）；
    ④ 行尾照原文件（本文件是 CRLF）。
"""
import io
import os
import sys

P = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                 "..", "steer_loop_wifi", "user", "main_cm7_0.c")
P = os.path.normpath(P)

OLD_PRINT = '''        printf("        STEER tgt=%d.%02d delta=%d.%02d err=%d.%02d duty=%d raw=%u en=%u flt=0x%02X sc=%u\\r\\n",
               st_t, st_tf,
               st_d, st_df,
               st_e, st_ef,
               (int)steer.output_duty,
               (unsigned)steer_abs_get_raw(),
               (unsigned)steer.angle_enable,
               (unsigned)steer_ctrl_fault(),
               (unsigned)steer_abs_self_check_ok());'''

NEW_PRINT = '''        /* ★ 2026-10-16 加诊断字段（现场症状"打死之后方向盘彻底不动"就靠这一行定位）：
             raw  = 环里**真正用**的值（防抖之后）
             rl   = 本拍 SPI **实读**值（没防抖）—— raw 与 rl 差得远 = 防抖正在拒样，链路在变差
             bad  = 上电以来被拒样数（只增）—— **只要它在涨，就说明电机在干扰编码器**
             ar   = 自动复位次数（只增）—— >0 = 真的发生过"锁存->自动复位"
             flt  = 状态位: 0x01 反馈失效(锁存,会钉输出) / 0x02 顶死告警 / 0x04 **本拍在丢样**(瞬态)
                    -> 现场判读顺序: 先看 flt&0x04 闪不闪(预警) -> 再看 bad 涨不涨 -> 最后才看 0x01 */
        printf("        STEER tgt=%d.%02d delta=%d.%02d err=%d.%02d duty=%d raw=%u rl=%u en=%u "
               "flt=0x%02X sc=%u bad=%u ar=%lu\\r\\n",
               st_t, st_tf,
               st_d, st_df,
               st_e, st_ef,
               (int)steer.output_duty,
               (unsigned)steer_abs_get_raw(),
               (unsigned)steer_abs_get_raw_live(),
               (unsigned)steer.angle_enable,
               (unsigned)steer_ctrl_fault(),
               (unsigned)steer_abs_self_check_ok(),
               (unsigned)steer_abs_bad_cnt(),
               (unsigned long)steer.recover_cnt);'''

OLD_CH = '''    /* bit0 = 绝对编码器自检通过; bit1/bit2 = steer_ctrl 的主动保护锁存
       (0x02 反馈失效 / 0x04 顶死或反转) —— 方向给反时这一位会亮, 是现场第一判据。 */'''

NEW_CH = '''    /* bit0   = 绝对编码器上电自检通过
       bit1   = 0x02 **反馈失效（锁存）** —— 输出已钉 0；会自动复位(限次数), 也能人工 steer_en 0->1
       bit2   = 0x04 顶死/无进展**告警**（不钉输出, 粘滞 2s）
       bit3   = 0x08 **本拍编码器读数被防抖拒掉**（瞬态、不锁存）—— "电机干扰编码器"最早的预警。
                ★ 2026-10-16 加：现场"打死后再也控不动方向盘"的定位就靠这四位里的 bit1/bit3。
                [!] 判读顺序: bit3 闪 -> 丢样计数在涨 -> 才是 bit1。别一看到 bit1 就去改机械。 */'''

def main():
    apply = "--apply" in sys.argv

    raw = open(P, "rb").read()
    crlf = raw.count(b"\r\n")
    s = io.open(P, encoding="gbk").read()

    for old, new, tag in ((OLD_PRINT, NEW_PRINT, "STEER 打印行"),
                          (OLD_CH,    NEW_CH,    "steer_status 注释")):
        if s.count(old) != 1:
            print("!! %s: 命中 %d 次（应为 1）—— 文件已改过或模板过时，未落盘" % (tag, s.count(old)))
            return 2
        s = s.replace(old, new)

    try:
        data = s.encode("gbk")            # ★ 预检：编不出就整体失败，绝不动文件
    except UnicodeEncodeError as e:
        print("!! GBK 编不出（%s）-> 放弃，不落盘" % e)
        return 3

    print("预检 OK: %d 字节 -> %d 字节, 原 CRLF=%d" % (len(raw), len(data), crlf))

    if not apply:
        print("（dry-run，加 --apply 才写）")
        return 0

    bak = P + ".bak_diag"
    if not os.path.exists(bak):
        open(bak, "wb").write(raw)

    tmp = P + ".tmp_diag"
    nl = "\r\n" if crlf > 0 else "\n"
    with io.open(tmp, "w", encoding="gbk", newline=nl) as f:
        f.write(s)
    os.replace(tmp, P)

    chk = open(P, "rb").read()
    print("写后: %d 字节, CRLF=%d" % (len(chk), chk.count(b"\r\n")))
    try:
        chk.decode("gbk")
        print("GBK 可读 OK")
    except Exception as e:
        print("!! GBK 失败 %s" % e)
        return 4
    return 0

if __name__ == "__main__":
    sys.exit(main())
