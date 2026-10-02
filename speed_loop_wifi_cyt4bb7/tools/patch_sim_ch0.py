"""修：模拟器把 ch[0] duty_cmd（= 原始参数值）也一起门控了。

真固件 motor_fill_telemetry 里 CH_DUTY_CMD 取的是 motor_duty_cmd 参数本身，
只门控"实际输出"。所以断使能/失联时 ch[0] 应保持参数值，ch[1] 才归 0。
（上一版模拟器把 ch[0] 写成门控后的 cmd —— 顺手一起纠正。）
"""
import os

ROOT = r"D:/jisuyueye9car/KartHost"
p = os.path.join(ROOT, "Simulator.cs")
raw = open(p, "rb").read()
for enc in ("utf-8-sig", "utf-8", "gbk"):
    try:
        t = raw.decode(enc); break
    except UnicodeDecodeError:
        continue
crlf = "\r\n" in t
t = t.replace("\r\n", "\n")

old = """        float cmd = (master && motorEn) ? Prm("duty_cmd") : 0f;
        if (!_everRx) cmd = 0f;
        if (autoStop && !Online) cmd = 0f;"""
new = """        // rawCmd = 参数本身（ch[0] 报的就是它，真固件 CH_DUTY_CMD 也是取参数值）；
        // 下面几条只决定"实际输出"，不动 ch[0]。
        float rawCmd = Prm("duty_cmd");

        float cmd = (master && motorEn) ? rawCmd : 0f;
        if (!_everRx) cmd = 0f;
        if (autoStop && !Online) cmd = 0f;"""
assert t.count(old) == 1, t.count(old)
t = t.replace(old, new, 1)

old = "        ch[0] = cmd;                                        // duty_cmd"
new = "        ch[0] = rawCmd;                                     // duty_cmd（参数原值，不受门控影响）"
assert t.count(old) == 1, t.count(old)
t = t.replace(old, new, 1)

open(p, "wb").write(t.replace("\n", "\r\n").encode(enc) if crlf else t.encode(enc))
print("Simulator.cs ✓ ch[0] 改为报参数原值")
