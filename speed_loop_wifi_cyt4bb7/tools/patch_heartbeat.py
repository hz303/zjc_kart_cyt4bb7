"""修「上位机没有心跳 → 车端 600ms 就判失联 → 电机自停」这个真 bug。

改四处：
  Heartbeat.cs   新增：心跳节拍的唯一实现（周期常量 + Beat()），便于自检直接测行为
  Simulator.cs   让模拟车端**真的建模失联**（以前 online 写死 1，自检永远看不到这条）
  MainForm.cs    连接期间按 Heartbeat.IntervalMs 发 PING
  SelfTest.cs    TestMotorProfile 里的 sleep 改成"边等边发心跳"，并新增失联/恢复断言
"""
import os
import re

ROOT = r"D:/jisuyueye9car/KartHost"


def load(p):
    """★ 这批 .cs 是 CRLF —— 归一成 LF 再匹配，写回时还原，否则 assert 全挂。"""
    raw = open(os.path.join(ROOT, p), "rb").read()
    for enc in ("utf-8-sig", "utf-8", "gbk"):
        try:
            txt = raw.decode(enc)
            break
        except UnicodeDecodeError:
            continue
    else:
        raise SystemExit("解码失败 " + p)
    return txt.replace("\r\n", "\n"), (enc, "\r\n" in txt)


def save(p, t, meta):
    enc, crlf = meta
    if crlf:
        t = t.replace("\n", "\r\n")
    os.makedirs(os.path.dirname(os.path.join(ROOT, p)), exist_ok=True)
    with open(os.path.join(ROOT, p), "wb") as f:
        f.write(t.encode(enc))


def sub(t, old, new, tag):
    assert t.count(old) == 1, f"[{tag}] 命中 {t.count(old)} 次，应为 1"
    return t.replace(old, new, 1)


# ======================================================================
# 0) 新增 Heartbeat.cs
# ======================================================================
HEARTBEAT = '''namespace KartHost;

/// <summary>
/// 心跳：车端 motor_link 用「距上次下行帧 > lost_ms」判上位机是否还在，
/// 判失联就把电机输出清零。而上位机只在用户操作时发命令 ——
/// **不补心跳的话，现象是"拖到 40% 电机转一下，松手 0.6 秒自己停"，而且完全静默。
/// 这个缺陷不会抛异常、也不会让任何"值对不对"的断言变红，所以单独抽成一个可测的小单元。
/// </summary>
public static class Heartbeat
{
    /// <summary>
    /// 心跳周期 ms。必须**明显小于**车端 lost_ms（参数表默认 600ms，最小可设 100ms）。
    /// 100ms 时对默认值有 6 倍余量；实测每帧只有 15 字节，车端 ACK 也不写日志，代价可忽略。
    /// </summary>
    public const int IntervalMs = 100;

    /// <summary>发一次心跳。链路没开时返回 false（不抛异常）。</summary>
    public static bool Beat(Link link, Func<ushort> nextToken)
    {
        if (link == null || !link.IsOpen) return false;
        return link.Send(Proto.BuildCommand(Cmd.Ping, 0, 0f, nextToken()));
    }
}
'''
open(os.path.join(ROOT, "Heartbeat.cs"), "wb").write(HEARTBEAT.encode("utf-8"))
print("Heartbeat.cs  ✓ 新增")


# ======================================================================
# 1) Simulator.cs —— 建模失联
# ======================================================================
t, meta = load("Simulator.cs")

t = sub(t, "    private double _lastSendT;",
        "    private double _lastSendT;\n"
        "    private double _lastCmdT;                   // 最近一次收到下行帧的时刻（秒）\n"
        "    private bool   _everRx;                     // 上位机开口说过话没有\n", "sim-fields")

t = sub(t, "    public bool IsRunning { get; private set; }",
        "    public bool IsRunning { get; private set; }\n\n"
        "    /// <summary>与车端 motor_link_online() 同义：收到过下行帧 && 距上次 ≤ lost_ms</summary>\n"
        "    public bool Online { get; private set; }", "sim-online-prop")

t = sub(t, """            CommandsGot++;
            Dispatch(cmd, pid, raw, val, token);""",
        """            // ★ 记录"上位机还活着"。真固件用 rx_age < lost_ms 判在线，这里必须一起建模，
            //   否则"上位机不发心跳 → 车端判失联 → 电机自停"在自检里完全看不见。
            _lastCmdT = _t;
            _everRx   = true;

            CommandsGot++;
            Dispatch(cmd, pid, raw, val, token);""", "sim-remember")

t = sub(t, """        // ---- 目标：使能不全 → 一律 0（这就是车端 motor_policy_target 的行为）----
        float cmd = (master && motorEn) ? Prm("duty_cmd") : 0f;""",
        """        // ---- 目标：与车端 motor_policy_target 逐条对齐 ----
        //   ① 使能不全 → 0（Motor 与 Master 都要置位）
        //   ② 上位机从没说过话 → 0
        //   ③ 距上次下行帧超过 lost_ms → 失联 → 0（这条以前漏了，见 Online 的注释）
        float lostMs   = Math.Clamp(Prm("lost_ms"), 100f, 3000f);
        bool  autoStop = Prm("auto_stop") > 0.5f;

        Online = _everRx && (_t - _lastCmdT) * 1000.0 <= lostMs;

        float cmd = (master && motorEn) ? Prm("duty_cmd") : 0f;
        if (!_everRx) cmd = 0f;
        if (autoStop && !Online) cmd = 0f;""", "sim-policy")

t = sub(t, """        ch[5] = 1f;                                         // online
        ch[6] = 20f + (float)(_rnd.NextDouble() * 12.0);    // rx_age ms（有点抖动更真实）""",
        """        ch[5] = Online ? 1f : 0f;                           // online（真建模，不再写死 1）
        ch[6] = (float)((_t - _lastCmdT) * 1000.0);         // rx_age ms（距上次下行帧）""", "sim-ch")
save("Simulator.cs", t, meta)
print("Simulator.cs  ✓ 失联已建模")


# ======================================================================
# 2) MainForm.cs —— 心跳定时器
# ======================================================================
t, meta = load("Ui/MainForm.cs")

t = sub(t, "    private System.Windows.Forms.Timer _uiTimer;",
        "    private System.Windows.Forms.Timer _uiTimer;\n"
        "    private System.Windows.Forms.Timer _hbTimer;    // 心跳：告诉车端\"上位机还活着\"", "mf-field")

t = sub(t, """        _statTimer = new System.Windows.Forms.Timer { Interval = 500 };
        _statTimer.Tick += (_, __) => UpdateStats();""",
        """        _statTimer = new System.Windows.Forms.Timer { Interval = 500 };
        _statTimer.Tick += (_, __) => UpdateStats();

        // ★ 心跳：车端用"距上次下行帧 > lost_ms（默认 600ms）"判失联，失联即清零输出。
        //   而上位机只在用户操作时发命令 → 不补心跳就会"松手 0.6 秒电机自己停"，且完全静默。
        //   周期见 Heartbeat.IntervalMs 的注释（为什么是 100ms）。
        _hbTimer = new System.Windows.Forms.Timer { Interval = Heartbeat.IntervalMs };
        _hbTimer.Tick += (_, __) => Heartbeat.Beat(_link, NextToken);
        _hbTimer.Start();""", "mf-timer")

old_stop = "        try { _uiTimer?.Stop(); _statTimer?.Stop(); _tableTimer?.Stop(); } catch { }"
if old_stop in t:
    t = t.replace(old_stop,
                  "        try { _uiTimer?.Stop(); _statTimer?.Stop(); _tableTimer?.Stop(); _hbTimer?.Stop(); } catch { }", 1)
    print("MainForm.cs  ✓ 心跳已加（含 Dispose 停表）")
else:
    print("MainForm.cs  ✓ 心跳已加（⚠ 未找到 Dispose 停表点，请人工检查）")
save("Ui/MainForm.cs", t, meta)


# ======================================================================
# 3) SelfTest.cs
# ======================================================================
t, meta = load("SelfTest.cs")

# 3.1 if (up) { ... link.Close(); } 之间的 sleep → 带心跳等待
i0 = t.find("        if (up)\n        {\n            Thread.Sleep(600);")
i1 = t.find("            link.Close();", i0)
assert i0 > 0 and i1 > i0, (i0, i1)
seg, n = re.subn(r"Thread\.Sleep\((\d+)\);",
                 lambda m: f"WaitWithHeartbeat(link, {float(m.group(1)) / 1000.0:g});", t[i0:i1])
assert n >= 4, n
t = t[:i0] + seg + t[i1:]
print(f"SelfTest.cs  ✓ TestMotorProfile 内 {n} 处 sleep 改为带心跳等待")

# 3.2 新增断言
t = sub(t, """            link.Close();
        }

        sim.Stop();""",
        """            // ★ 关键断言 6：**心跳一停，车端就该判失联并清零输出**。
            //   以前这里是空的：模拟器把 online 写死成 1，上位机又没有心跳，
            //   于是真机上"拖到 40% 转一下、松手 0.6 秒自己停"照样发生，自检全绿。
            link.Send(Proto.BuildCommand(Cmd.SetEnable, 0,
                                         (ushort)(EnableBits.Master | EnableBits.Motor), 400));
            link.Send(Proto.BuildCommand(Cmd.SetParam, 0, 40f, 401));
            WaitWithHeartbeat(link, 1.2);
            float hbAct = store.Latest(1);
            Check(Math.Abs(hbAct - 40f) < 2.0f, "（前置）有心跳时输出稳定在 +40%",
                  "实际 " + hbAct.ToString("F2"));

            Thread.Sleep(1000);                     // 故意不发心跳，超过 lost_ms=600
            Check(store.Latest(5) < 0.5f, "停心跳 1s 后 online 位变 0",
                  "实际 " + store.Latest(5).ToString("F2"));
            Check(Math.Abs(store.Latest(1)) < 1.0f, "停心跳后 duty_act 自动归 0（失联保护生效）",
                  "实际 " + store.Latest(1).ToString("F2"));
            Check(Math.Abs(store.Latest(0) - 40f) < 1.0f, "（对照）duty_cmd 参数本身没被改",
                  "实际 " + store.Latest(0).ToString("F2"));

            WaitWithHeartbeat(link, 1.6);            // 心跳恢复 —— 走的就是 App 用的那条路
            Check(store.Latest(5) > 0.5f, "心跳恢复后 online 位回到 1",
                  "实际 " + store.Latest(5).ToString("F2"));
            Check(Math.Abs(store.Latest(1) - 40f) < 2.0f, "恢复心跳后输出重新跟上 +40%",
                  "实际 " + store.Latest(1).ToString("F2"));

            // ★ 关键断言 7：心跳参数本身要合规（周期必须明显小于车端 lost_ms）
            Check(Heartbeat.IntervalMs > 0 && Heartbeat.IntervalMs <= 200,
                  "心跳周期 <= 200ms（车端 lost_ms 默认 600、最小 100）",
                  "实际 " + Heartbeat.IntervalMs + "ms");
            Check(!Heartbeat.Beat(new Link(), () => 1), "链路没开时 Beat 返回 false 且不抛异常");

            link.Close();
        }

        sim.Stop();""", "st-assert6")

# 3.3 辅助方法
t = sub(t, "    // ==================================================================\n    // 高 DPI 布局体检",
        """    /// <summary>
    /// 一边等一边按真实上位机那样发心跳 —— **走的就是 App 里 Heartbeat.Beat 那条路**。
    /// ★ 必须这么等：车端/模拟车端都用 lost_ms（默认 600ms）判失联，干 sleep 会让仿真掉线、
    ///   输出清零，断言就变成在测"失联保护"而不是在测目标行为。
    /// </summary>
    private static void WaitWithHeartbeat(Link link, double seconds, int hbMs = 100)
    {
        var sw = System.Diagnostics.Stopwatch.StartNew();
        ushort tok = 4000;
        while (sw.Elapsed.TotalSeconds < seconds)
        {
            Heartbeat.Beat(link, () => tok++);
            Thread.Sleep(hbMs);
        }
    }

    // ==================================================================
    // 高 DPI 布局体检""", "st-helper")
save("SelfTest.cs", t, meta)
print("SelfTest.cs  ✓ 新增断言 + 辅助方法")
print("\n全部改完")
