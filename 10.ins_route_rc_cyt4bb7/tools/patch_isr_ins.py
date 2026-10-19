"""把惯导两拍挂进工程 10 的 5ms 中断（cm7_0_isr.c）。

⚠⚠ 三个坑，都真栽过：
  1. 本文件是 **GBK** 编码（`user/` 下都是），且是 **LF** 行尾
     ⇒ 不能用普通文本编辑工具改（会按 UTF-8 写回去，毁掉满文件的中文注释）。
  2. **`⚠` `⇒` `①` 这类字符 GBK 编不出来**。而且 `io.open(...,'w')` 在 **open 那一刻就把文件截断**，
     等 `write()` 里编码失败时文件**已经空了** —— 第一次跑这个脚本就把 cm7_0_isr.c 清成 0 字节
     （靠预先 copyfile 的 .bak_ins 救回来）。
     ⇒ 所以本脚本：**写之前先 `s.encode('gbk')` 试一次**，并且**写临时文件再 os.replace**（原子）。
  3. 新加的注释一律**只用 ASCII + 普通汉字**，不用任何特殊符号。

用法：python patch_isr_ins.py            # 只报告（会做一次可编码性预检）
      python patch_isr_ins.py --apply    # 真改（先备份 .bak_ins，原子替换）
"""
import io
import os
import shutil
import sys

P = r"D:/jisuyueye9car/A_zjc_project/10.ins_route_rc_cyt4bb7/steer_loop_wifi/user/cm7_0_isr.c"

INC_ANCHOR = '#include "steer_ctrl.h"'
INC_ADD = '#include "ins_imu.h"\n#include "ins_odom.h"\n'

CALL_ANCHOR = "    steer_ctrl_update();\n"
CALL_ADD = (
    "\n"
    "    // ---- 工程 10: 惯导两拍（顺序是硬约束，别调换）----\n"
    "    //  (1) ins_imu_update(): 读 IMU(SPI_2 / P15_0~3) -> Mahony -> 累积 yaw（本拍的新 yaw）\n"
    "    //  (2) ins_odom_update(): 用【本拍】spd_enc 的 delta + (1) 的新 yaw 积分出 (x,y)\n"
    "    //  !! 必须在 spd_ctrl_tick() 之后 —— 那里面的 spd_enc_update() 才是本拍的 delta。\n"
    "    //  !! 也必须在 ins_imu_update() 之后 —— 用的是同一拍的 yaw，不是上一拍的。\n"
    "    //  !! IMU 没接好时 ins_imu_update() 内部直接 return（完全不碰 SPI_2），\n"
    "    //     所以「IMU 掉了」不会把 5ms 中断拖住 —— 这是中断里访问外部器件的唯一硬约束。\n"
    "    ins_imu_update();\n"
    "    ins_odom_update();\n"
)


def line_ending_of(path):
    """★ 行尾必须照着原文件来：读时默认 newline=None 会把 \\r\\n 归一成 \\n，
    写回 newline='' 就等于把整个文件改成 LF（cm7_0_isr.c 本来就是 LF，所以这里没暴露；
    main_cm7_0.c 是 CRLF，那边一跑就把 423 个 CRLF 全丢了）。"""
    raw = open(path, "rb").read(4000)
    return "\r\n" if b"\r\n" in raw else "\n"


def main():
    apply = "--apply" in sys.argv
    s = io.open(P, encoding="gbk").read()
    NL = line_ending_of(P)
    print("行尾 = %r" % NL)

    if '#include "ins_imu.h"' in s or "ins_imu_update();" in s:
        print("已经打过补丁，跳过（幂等）")
        return 0

    if INC_ANCHOR not in s:
        print("FAIL: 找不到 include 锚点")
        return 1
    if CALL_ANCHOR not in s:
        print("FAIL: 找不到 steer_ctrl_update() 锚点")
        return 1

    s = s.replace(INC_ANCHOR, INC_ANCHOR + "\n" + INC_ADD, 1)
    s = s.replace(CALL_ANCHOR, CALL_ANCHOR + CALL_ADD, 1)

    # ---- 写之前的硬检查（第 2 条坑的正面做法）----
    try:
        s.encode("gbk")
    except UnicodeEncodeError as e:
        print("FAIL: 待写内容含 GBK 编不出的字符 %r —— 改掉再跑" % e.object[e.start:e.end])
        return 1

    if not apply:
        print("预检通过（GBK 可编码），只报告。加 --apply 才真写")
        return 0

    if not os.path.exists(P + ".bak_ins"):
        shutil.copyfile(P, P + ".bak_ins")
        print("已备份 ->", P + ".bak_ins")

    tmp = P + ".tmp_ins"
    with io.open(tmp, "w", encoding="gbk", newline=NL) as f:
        f.write(s)
    os.replace(tmp, P)                      # 原子替换
    print("已原子写入", P)

    s2 = io.open(P, encoding="gbk").read()
    ok = ('#include "ins_imu.h"' in s2 and '#include "ins_odom.h"' in s2
          and "ins_imu_update();" in s2 and "ins_odom_update();" in s2)
    print("回读核对: %s" % ("OK" if ok else "FAIL"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
