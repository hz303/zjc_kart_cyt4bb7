#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
patch_ui_init.py —— 给工程 10 的 main_cm7_0.c 补上屏幕/按键初始化

背景（2026-10-17）
  工程 10 的屏幕菜单是从工程 8 搬过来的，但 `ui_init()` 与 `ui_menu_init()`
  这两行**没跟着搬**：`ui_menu_task()` 在主循环里被调，可屏幕从来没被初始化过。
  现象 = **整屏全黑**（背光/复位都不会被拉），很容易被误判成"引脚接错"。

做法
  在 `while (true)` 之前插一段（`ui_init()` + 串口一行自证 + `ui_menu_init()`），
  位置必须与工程 8 一致：motor_link_init() 之后（菜单用 motor_link_now_ms() 做时间分频）。

⚠ 本文件是 **GBK + CRLF**：只能用本脚本改，别用 UTF-8 编辑器直接存。
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
MAIN = os.path.join(HERE, "..", "steer_loop_wifi", "user", "main_cm7_0.c")

ANCHOR = "    while (true)\r\n"

INSERT = """    /*--------------------------- 屏幕 + 按键（菜单） ---------------------------
       屏幕: 逐飞 IPS200 的 **SPI 版**（硬件 SPI_1），6 根脚由 zf_device_ips200.h 定死：
             SCL P12_2 / SDA(MOSI) P12_1 / CS P12_3 / RST P22_4 / DC P22_3 / BLK P11_0
       本工程别的模块**不占**这 6 根：转向绝对编码器走 SPI_4 的 P07_x、
       WiFi 走 SPI_0 的 P02_x、板载 IMU 走 SPI_2 的 P15_x。改屏幕接线只需改那 6 个宏。
       按键: 主板自带 4 键 P20_3/P20_2/P20_0/P20_1（K3/K4 已按工程 8 对调）。

       [!] 必须在 motor_link_init() 之后：菜单用 motor_link_now_ms() 做 10 ms 分频，
           而 key_init(10) 的周期必须与它一致（见 ui_menu.h 头注释）。
       [!] 屏幕 SPI 是毫秒级开销（清一屏约 40 ms）=> 只能在主循环里画，
           绝不能放进 5 ms 中断，否则控制节拍直接被顶穿。
       [!] 这两行**漏了就是全黑屏**（2026-10-17 实际踩到）：ips200_init 不跑，
           屏幕连背光和复位都不会拉，看起来像"接错线"，其实一行代码都没执行。
    */
    ui_init();
    ui_puts_row(0, 0, "Kart10 INS+Route");
    printf("[##] UI init done: IPS200 SPI_1 (SCL P12_2 / SDA P12_1 / CS P12_3 / DC P22_3 / RST P22_4 / BLK P11_0)\\r\\n");
    ui_menu_init();

"""


def main():
    if not os.path.exists(MAIN):
        print("[ERR] 找不到 %s" % MAIN)
        return 2

    raw = open(MAIN, "rb").read()
    try:
        text = raw.decode("gbk")
    except UnicodeDecodeError as e:
        print("[ERR] 不是 GBK：%s" % e)
        return 2

    if "ui_init();" in text:
        print("[SKIP] 已经补过了（文件里已有 ui_init();）")
        return 0

    n = text.count(ANCHOR)
    if n != 1:
        print("[ERR] 锚点 'while (true)' 出现 %d 次（要求 1 次），请人工核对" % n)
        return 2

    # 预检：插入内容必须能 GBK 编码（否则会写出半个字符/替换成 '?'）
    INSERT.encode("gbk")

    text = text.replace(ANCHOR, INSERT + ANCHOR, 1)
    open(MAIN, "wb").write(text.encode("gbk"))
    print("[OK] 已插入 ui_init()/ui_menu_init()：%s" % os.path.relpath(MAIN, HERE))
    print("     行尾保持 CRLF；编码保持 GBK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
