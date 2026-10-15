"""KartHost 侧同步：单电机档案 16 → 20 通道、参数 9 → 14；模拟器补 4 个通道。

顺带修一个真 bug：Simulator 的 _ch 是在构造时按"当时的档案"定长的（默认卡丁车 32），
切到单电机档案后长度就错了 → 模拟车端会发出 32 通道。改成按当前档案自动重分配。
"""
import os

ROOT = r"D:/jisuyueye9car/KartHost"


def load(p):
    """返回 (统一成 LF 的文本, 编码, 原来的行尾)。匹配一律用 LF，写回时还原行尾。"""
    raw = open(p, "rb").read()
    for enc in ("utf-8-sig", "utf-8"):
        try:
            t = raw.decode(enc)
            break
        except UnicodeDecodeError:
            continue
    else:
        raise SystemExit("解码失败 " + p)
    eol = "\r\n" if "\r\n" in t else "\n"
    return t.replace("\r\n", "\n").replace("\r", "\n"), enc, eol


def save(p, t, enc, eol):
    if eol == "\r\n":
        t = t.replace("\n", "\r\n")
    open(p, "wb").write(t.encode(enc))


def sub(t, old, new, n=1, tag=""):
    assert t.count(old) == n, (tag, "命中 %d 次（期望 %d）" % (t.count(old), n))
    return t.replace(old, new, n)


# ============================================================ Defaults.cs
p = os.path.join(ROOT, "Defaults.cs")
t, enc, eol = load(p)

t = sub(t, '''        "mag_raw",    "mag_angle",  "mag_deg",    "mag_delta",
    };''', '''        "mag_raw",    "mag_angle",  "mag_deg",    "mag_delta",
        "mag_speed",  "mag_sum",    "mag_delta_w","mag_glitch",
    };''', tag="通道名")

t = sub(t, '''        "raw", "raw", "°",    "raw",
    };''', '''        "raw", "raw", "°",    "raw",
        "rad/s", "mrad", "cnt", "cnt",
    };''', tag="通道单位")

t = sub(t, '''        new[] { 13, 14 },        // 角度：raw 计数 vs 换算后的度数
        new[] { 15 },            // 每次读数的变化量（转一下就跳，不转就贴着 0）
    };''', '''        new[] { 13, 14 },        // 角度：raw 计数 vs 换算后的度数
        new[] { 15, 18 },        // 变化量：老口径 vs 圆上最短差（并排看跨零有没有翻号）
        new[] { 16 },            // 角速度 rad/s（自带符号）
        new[] { 17, 13 },        // 积分角 mrad vs 绝对角 —— 两者是否一致就靠这张图
        new[] { 19 },            // 单帧毛刺累计（SPI 时序余量够不够的客观指标）
        new[] { 10, 5 },         // 上电时间 + 在线
    };''', tag="编码器分图")

t = sub(t, '''        P("编码器", "mag_zero",   "编码器零点",        0f, 65535f, 0.0f, 1.0f, "raw");''',
        '''        P("编码器", "mag_zero",   "编码器零点",        0f, 65535f, 0.0f, 1.0f, "raw");
        P("编码器", "sum_clr",    "积分清零",          0f,   1f,    0.0f, 1.0f,  "-");      // 拖到 1 清零积分
        P("编码器", "spi_mode",   "SPI 模式",          0f,   3f,    2.0f, 1.0f,  "-");      // TLE5012B 的 SSC 用 2
        P("编码器", "spi_mhz",    "SPI 速率",          1f,  40f,    8.0f, 1.0f,  "MHz");    // 手册推荐 <=8
        P("编码器", "k_speed",    "速度系数",        -10f,  10f, 1.917476f,0.001f, "-");     // rad/s per LSB
        P("编码器", "glitch_th",  "毛刺阈值",          0f,8192f,  512.0f, 1.0f,  "cnt");    // 单帧毛刺判据''',
        tag="参数表")

save(p, t, enc, eol)
print("Defaults.cs ✔ 20 通道 / 14 参数")

# ============================================================ SelfTest.cs
p = os.path.join(ROOT, "SelfTest.cs")
t, enc, eol = load(p)

t = sub(t, '''            "mag_raw", "mag_angle", "mag_deg", "mag_delta",
        };
        Check(Defaults.ChannelNames.Length == 16, "通道数 = 16", "实际 " + Defaults.ChannelNames.Length);''',
        '''            "mag_raw", "mag_angle", "mag_deg", "mag_delta",
            "mag_speed", "mag_sum", "mag_delta_w", "mag_glitch",
        };
        Check(Defaults.ChannelNames.Length == 20, "通道数 = 20", "实际 " + Defaults.ChannelNames.Length);''',
        tag="通道数断言")

t = sub(t, '''            ("mag_zero",      0f,65535f,   0f,  1.0f),
        };''', '''            ("mag_zero",      0f,65535f,   0f,  1.0f),
            ("sum_clr",       0f,    1f,   0f,  1.0f),
            ("spi_mode",      0f,    3f,   2f,  1.0f),
            ("spi_mhz",       1f,   40f,   8f,  1.0f),
            ("k_speed",     -10f,   10f, 1.917476f, 0.001f),
            ("glitch_th",     0f, 8192f, 512f,  1.0f),
        };''', tag="参数断言")

t = sub(t, '''            Check(Math.Abs(magDelta - 60f) < 5f, "mag_delta ≈ 60（3000 计数/s × 20ms）", "实际 " + magDelta.ToString("F1"));''',
        '''            Check(Math.Abs(magDelta - 60f) < 5f, "mag_delta ≈ 60（3000 计数/s × 20ms）", "实际 " + magDelta.ToString("F1"));

            // ★ 关键断言 6：新加的 4 个通道（角速度 / 积分 / 圆上最短差 / 毛刺）也确实映射到了
            float magDW = store.Latest(18);
            Check(Math.Abs(magDW - 60f) < 5f, "mag_delta_w ≈ 60（模拟器无跨零，应与 mag_delta 一致）",
                  "实际 " + magDW.ToString("F1"));
            float magSpd = store.Latest(16);
            Check(Math.Abs(magSpd - 1.15f) < 0.1f, "mag_speed ≈ 1.15 rad/s（与模拟器 3000 计数/s 自洽）",
                  "实际 " + magSpd.ToString("F2"));
            Check(store.Latest(17) > 0f, "mag_sum 在累加（>0）", "实际 " + store.Latest(17).ToString("F1"));
            Check(store.Latest(19) == 0f, "mag_glitch = 0（模拟器无噪声）", "实际 " + store.Latest(19).ToString("F0"));''',
        tag="新通道断言")

save(p, t, enc, eol)
print("SelfTest.cs ✔ 断言同步")

# ============================================================ Simulator.cs
p = os.path.join(ROOT, "Simulator.cs")
t, enc, eol = load(p)

t = sub(t, "    private readonly float[] _ch = new float[Defaults.ChannelNames.Length];",
        "    private float[] _ch = new float[Defaults.ChannelNames.Length];   // 档案一变就重分配（见 BuildTelemetry）\n"
        "    private double _mSum;                                            // 角速度积分（mrad）",
        tag="_ch 声明")

t = sub(t, """        var ch = _ch;
        Array.Clear(ch, 0, ch.Length);""",
        """        // ★ 档案可能在运行期切换（卡丁车 32 / 单电机 20），长度对不上就重分配 ——
        //   否则模拟车端会按旧档案的长度发帧（以前就是按构造时的 32 发，切档案后对不上）。
        if (_ch.Length != Defaults.ChannelNames.Length) _ch = new float[Defaults.ChannelNames.Length];
        var ch = _ch;
        Array.Clear(ch, 0, ch.Length);""",
        tag="_ch 重分配")

t = sub(t, """        ch[15] = 3000f * 0.02f;                             // mag_delta（20ms 走这么多）
    }""",
        """        ch[15] = 3000f * 0.02f;                             // mag_delta（20ms 走这么多）

        // ---- 2026-10-01 新增：角速度 / 积分 / 圆上最短差 / 毛刺 ----
        ch[16] = 3000f / 16384f * 6.2831853f;               // mag_speed（rad/s，与上面 3000 计数/s 自洽）
        _mSum += ch[16] * 0.02 * 1000.0;                    // mag_sum（mrad，按 20ms 累加）
        ch[17] = (float)_mSum;
        ch[18] = 3000f * 0.02f;                             // mag_delta_w（模拟器不跨零 → 与 mag_delta 相同）
        ch[19] = 0f;                                        // mag_glitch（模拟器无噪声）
    }""",
        tag="模拟器新通道")

save(p, t, enc, eol)
print("Simulator.cs ✔ 4 个新通道 + _ch 重分配")
