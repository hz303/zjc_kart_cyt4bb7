"""给磁编码器加第 4 条协议：命令仍是能读通的 0x8021，但按实测的 14 位解码。

依据（1677 帧实测）：bit15 恒 0、bit14 恒 1（都是固定标志位），角度只在低 14 位；
低 14 位覆盖 87~16366（满量程 99.4%）且有"小步跨过 0/16383"的事件 → 是绕环的真实角度。
用原来的 0x7FFF 会把恒 1 的 bit14 当成角度最高位 → 显示 = 180° + 真角/2。
"""
import os

FW = r"D:/jisuyueye9car/A_zjc_project/4.motor_wifi_verify_cyt4bb7/motor_wifi_verify/code"
KH = r"D:/jisuyueye9car/KartHost"
TL = r"D:/jisuyueye9car/A_zjc_project/4.motor_wifi_verify_cyt4bb7/tools"


def load(p):
    raw = open(p, "rb").read()
    for enc in ("utf-8-sig", "utf-8", "gbk"):
        try:
            txt = raw.decode(enc); break
        except UnicodeDecodeError:
            continue
    else:
        raise SystemExit("解码失败 " + p)
    return txt.replace("\r\n", "\n"), (enc, "\r\n" in txt, raw.startswith(b"\xef\xbb\xbf"))


def save(p, t, meta):
    enc, crlf, bom = meta
    if crlf:
        t = t.replace("\n", "\r\n")
    data = t.encode(enc)
    if bom and not data.startswith(b"\xef\xbb\xbf"):
        data = b"\xef\xbb\xbf" + data
    open(p, "wb").write(data)


def sub(t, old, new, tag):
    assert t.count(old) == 1, f"[{tag}] 命中 {t.count(old)} 次"
    return t.replace(old, new, 1)


# ---------------- 1. mag_enc.c ----------------
p = os.path.join(FW, "mag_enc.c")
t, meta = load(p)
t = sub(t,
        "float   mag_proto = 0.0f;       /* 0=MENC15A/MT6816  1=AS5047  2=RAW 探针 */",
        "float   mag_proto = 3.0f;       /* 0=MENC15A 1=AS5047 2=RAW 3=本机实测14位（默认） */",
        "enc-proto-default")
t = sub(t,
        '    { "RAW 探针",       0x0000, 0xFFFF, 65536 },   /* 只发 0，回什么就显示什么 —— 先确认有回读 */\n',
        '    { "RAW 探针",       0x0000, 0xFFFF, 65536 },   /* 只发 0，回什么就显示什么 —— 先确认有回读 */\n'
        '    /* ★ 本机那块"型号未知"模块实测出来的正确解码：\n'
        '     *   读命令仍然用 0x8021（换命令就没数据了），但 **bit15 恒 0、bit14 恒 1 是固定标志位**，\n'
        '     *   角度只在低 14 位。用 0x7FFF 会把那个恒 1 的 bit14 当成角度最高位，\n'
        '     *   于是 显示角度 = 180° + 真角/2（永远落在 180~360，分辨率也只剩一半）。\n'
        '     *   证据（1677 帧 / 25 s 实测）：bit14 100% 为 1；低 14 位覆盖 87~16366 = 满量程 99.4%；\n'
        '     *   且有 19 次"小步跨过 0/16383 边界"（等价步长 +432 / -375 / +308 …）→ 绕环走的真角度。 */\n'
        '    { "LOCAL-14bit",    0x8021, 0x3FFF, 16384 },\n',
        "enc-table-row")
save(p, t, meta)
print("mag_enc.c      ✓ 协议 3 + 默认值")

# ---------------- 2. mag_enc.h ----------------
p = os.path.join(FW, "mag_enc.h")
t, meta = load(p)
t = sub(t,
        ' *     2 = 只发 0x0000、原样打印回读值（RAW 探针：先确认',
        ' *     **3 = 命令 0x8021 + 低 14 位（本机实测的正确解，默认）**：\n'
        ' *         bit15 恒 0、bit14 恒 1 都是标志位，角度只在低 14 位；\n'
        ' *         用 0（0x7FFF）会把 bit14 当角度高位 → 显示 180°+真角/2\n'
        ' *     2 = 只发 0x0000、原样打印回读值（RAW 探针：先确认',
        "h-doc")
t = sub(t,
        "extern float mag_proto;     /* 协议：0=MENC15A/MT6816  1=AS5047  2=RAW 探针 */",
        "extern float mag_proto;     /* 0=MENC15A 1=AS5047 2=RAW 3=本机实测14位（默认） */",
        "h-extern")
save(p, t, meta)
print("mag_enc.h      ✓ 文档与默认值")

# ---------------- 3. motor_link.h ----------------
p = os.path.join(FW, "motor_link.h")
t, meta = load(p)
t = sub(t,
        'X(6, "mag_proto",  "编码器|协议",          0.0f,   2.0f,   0.000f, "-",    1.000f, &mag_proto        )',
        'X(6, "mag_proto",  "编码器|协议",          0.0f,   3.0f,   3.000f, "-",    1.000f, &mag_proto        )',
        "link-param")
save(p, t, meta)
print("motor_link.h   ✓ mag_proto 量程 0~3，默认 3")

# ---------------- 4. KartHost/Defaults.cs ----------------
p = os.path.join(KH, "Defaults.cs")
t, meta = load(p)
t = sub(t,
        '        P("编码器", "mag_proto",  "编码器协议",        0f,   2f,   0.0f, 1.0f,  "-");   // 0=MENC15A 1=AS5047 2=RAW 探针',
        '        // 3 = 命令 0x8021 + 低 14 位（实测：bit15=0/bit14=1 是标志位）—— 本机的正确解，设成默认\n'
        '        P("编码器", "mag_proto",  "编码器协议",        0f,   3f,   3.0f, 1.0f,  "-");   // 0=MENC15A 1=AS5047 2=RAW 3=实测14位',
        "defaults-param")
save(p, t, meta)
print("Defaults.cs    ✓ 参数与默认值")

# ---------------- 5. SelfTest.cs ----------------
p = os.path.join(KH, "SelfTest.cs")
t, meta = load(p)
t = sub(t,
        '            ("mag_proto",     0f,    2f,   0f,  1.0f),',
        '            ("mag_proto",     0f,    3f,   3f,  1.0f),',
        "selftest-param")
save(p, t, meta)
print("SelfTest.cs    ✓ 断言同步")

# ---------------- 6. mag_sweep.py 记账修正 ----------------
p = os.path.join(TL, "mag_sweep.py")
t, meta = load(p)
t = sub(t,
        '    print("   开始！")\n    sys.stdout.flush()',
        '    print("   开始！")\n    sys.stdout.flush()\n'
        '    # ★ 倒计时这 5 秒没读 socket，帧堆在内核缓冲区里；不清掉的话\n'
        '    #   它们会在下面被一次性读出，把"静止"和"转动"混在一起（踩过）。\n'
        '    #   另外下面一律用**车端 ts** 分桶，因为它才是单调的真实时间轴。\n'
        '    rows.clear()',
        "sweep-clear")
t = sub(t,
        "buckets = collections.defaultdict(list)\nfor r in rows:\n    buckets[int(r[0])].append(r[2])",
        "buckets = collections.defaultdict(list)\nfor r in rows:\n    buckets[r[1] // 1000].append(r[2])",
        "sweep-bucket")
save(p, t, meta)
print("mag_sweep.py   ✓ 记账修正（按车端 ts 分桶）")
print("\n全部改完")
