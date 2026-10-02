"""把"角速度积分 + SPI 时序可切"接进工程。

改动：
  1) motor_link.h  通道 16 → 20、参数 9 → 14
  2) main_cm7_0.c  mag_enc_task(now) + 填 4 个新通道

注意：main_cm7_0.c 是 GBK（逐飞原版 + IAR 保存），必须按原编码写回。
      motor_link.h 是 UTF-8。
"""
import os

ROOT = r"D:/jisuyueye9car/A_zjc_project/4.motor_wifi_verify_cyt4bb7/motor_wifi_verify"


def load(p):
    raw = open(p, "rb").read()
    for enc in ("utf-8-sig", "utf-8", "gbk"):
        try:
            return raw.decode(enc), enc
        except UnicodeDecodeError:
            continue
    raise SystemExit("解码失败 " + p)


def save(p, t, enc):
    open(p, "wb").write(t.encode(enc))


# ============================================================ 1) motor_link.h
p = os.path.join(ROOT, "code/motor_link.h")
t, enc = load(p)
print("motor_link.h 编码 =", enc)

old_cnt = "#define MOTOR_CH_COUNT          (16)        /* 遥测通道数（12 电机/链路 + 4 磁编码器）       */"
new_cnt = "#define MOTOR_CH_COUNT          (20)        /* 遥测通道数（12 电机/链路 + 8 磁编码器）       */"
assert t.count(old_cnt) == 1, t.count(old_cnt)
t = t.replace(old_cnt, new_cnt, 1)

NEW_BLOCK = "\n".join([
    '    X(5, "auto_stop",  "保护|失联自停",        0.0f,   1.0f,   1.000f, "-",    1.000f, &motor_auto_stop  )        \\',
    '    X(6, "mag_proto",  "编码器|协议",          0.0f,   3.0f,   3.000f, "-",    1.000f, &mag_proto        )        \\',
    '    X(7, "mag_dir",    "编码器|方向",          0.0f,   1.0f,   1.000f, "-",    1.000f, &mag_dir          )        \\',
    '    X(8, "mag_zero",   "编码器|零点",          0.0f,65535.0f,   0.000f, "raw",  1.000f, &mag_zero         )        \\',
    '    X(9, "sum_clr",    "编码器|积分清零",      0.0f,   1.0f,   0.000f, "-",    1.000f, &mag_sum_clr      )        \\',
    '    X(10,"spi_mode",   "编码器|SPI模式",       0.0f,   3.0f,   2.000f, "-",    1.000f, &mag_spi_mode     )        \\',
    '    X(11,"spi_mhz",    "编码器|SPI速率",       1.0f,  40.0f,   8.000f, "MHz",  1.000f, &mag_spi_mhz      )        \\',
    '    X(12,"k_speed",    "编码器|速度系数",    -10.0f,  10.0f,   1.917476f, "-",  0.001f, &mag_k_speed      )        \\',
    '    X(13,"glitch_th",  "编码器|毛刺阈值",      0.0f,8192.0f, 512.000f, "cnt",  1.000f, &mag_glitch_th    )',
])

lines = t.split("\n")
idx = [i for i, l in enumerate(lines) if 'X(8, "mag_zero"' in l]
assert len(idx) == 1, ("参数块定位失败", idx)
assert lines[idx[0] - 1].rstrip().endswith("\\"), "前一行没有续行符"
lines[idx[0]] = NEW_BLOCK
t = "\n".join(lines)
print("  参数表 → 14 项")

old_ch = "#define CH_MAG_DELTA    (15)    /* 与上次读数的差（看有没有在动）        */"
new_ch = "\n".join([
    old_ch,
    "#define CH_MAG_SPEED    (16)    /* SPEED 寄存器：角速度 rad/s（自带符号） */",
    "#define CH_MAG_SUM      (17)    /* 角速度积分：毫弧度（多圈累计角）       */",
    "#define CH_MAG_DELTAW   (18)    /* 圆上最短差（对照老口径 CH_MAG_DELTA）  */",
    "#define CH_MAG_GLITCH   (19)    /* 单帧毛刺累计次数（SPI 时序余量的指标） */",
])
assert t.count(old_ch) == 1
t = t.replace(old_ch, new_ch, 1)
save(p, t, enc)
print("  motor_link.h ✔ 通道 20 / 参数 14")

# ============================================================ 2) main_cm7_0.c
p = os.path.join(ROOT, "user/main_cm7_0.c")
t, enc = load(p)
print("main_cm7_0.c 编码 =", enc)

old = "            mag_enc_task();"
new = "            mag_enc_task(now);                          /* 传真实时间戳：积分用它算 dt */"
assert t.count(old) == 1, t.count(old)
t = t.replace(old, new, 1)

old_fill = '    motor_link_ch_set(CH_MAG_DELTA,   (float)mag_enc_delta());'
new_fill = "\n".join([
    old_fill,
    "    motor_link_ch_set(CH_MAG_SPEED,   mag_enc_speed_rps());       /* 角速度 rad/s */",
    "    motor_link_ch_set(CH_MAG_SUM,     (float)mag_enc_sum_mrad());  /* 积分 mrad    */",
    "    motor_link_ch_set(CH_MAG_DELTAW,  (float)mag_enc_delta_wrap());/* 圆上最短差   */",
    "    motor_link_ch_set(CH_MAG_GLITCH,  (float)mag_enc_glitch_cnt());/* 毛刺累计次数 */",
])
assert t.count(old_fill) == 1, t.count(old_fill)
t = t.replace(old_fill, new_fill, 1)
save(p, t, enc)
print("  main_cm7_0.c ✔ 调用点 + 4 个新通道")

# ============================================================ 3) 自检
for p, keys in ((os.path.join(ROOT, "code/motor_link.h"),
                 ["MOTOR_CH_COUNT          (20)", 'X(13,"glitch_th"', "CH_MAG_GLITCH   (19)"]),
                (os.path.join(ROOT, "user/main_cm7_0.c"),
                 ["mag_enc_task(now)", "CH_MAG_SPEED", "CH_MAG_GLITCH"])):
    t, _ = load(p)
    print("\n[复核]", os.path.basename(p))
    for k in keys:
        print("   %-34s %s" % (k, "✔" if k in t else "✘ 缺失"))
