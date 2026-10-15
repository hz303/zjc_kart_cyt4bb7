"""给所有实时抓包脚本加"唤醒"步骤。

背景（2026-09-30 实测）：这块 WiFi-SPI 模块跑了约 20 分钟后会进入一种僵死态 ——
**不再发包，PC 收不到任何遥测，但板子自己 `tx` 照涨、`link_err` 照旧是 0**。
实测：从 PC 发一个入站包就能把**上行**唤回来；**下行**（PC→板）则需要复位板子。

所以：绑到 8086 之后先朝广播 + 已知 IP 敲几下 PING，再进入正常流程。
"""
import os

TL = r"D:/jisuyueye9car/A_zjc_project/4.motor_wifi_verify_cyt4bb7/tools"

WAKE = '''s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
# ★ 唤醒：这块模块跑一阵子会进入"只收不发"的僵死态（上行静默、下行也不投递），
#   而板子自己 tx 照涨、link_err 照旧是 0 —— 从 PC 敲一个入站包能把上行唤回来。
for _k in range(5):
    try:
        s.sendto(build_cmd(0x01, 0, 0, 9000 + _k), ("192.168.50.255", 6666))
        s.sendto(build_cmd(0x01, 0, 0, 9100 + _k), ("192.168.50.238", 6666))
    except OSError:
        pass
    time.sleep(0.1)
print("   （已朝模块敲了几帧唤醒包）")

'''

for fn, anchor in (("mag_live.py", "s.settimeout(0.4)"),
                   ("mag_steps.py", "s.settimeout(0.4)"),
                   ("mag_sweep.py", "s.settimeout(0.5)"),
                   ("mag_hold.py", "s.settimeout(0.5)"),
                   ("mag_probe.py", "s.settimeout(0.5)")):
    p = os.path.join(TL, fn)
    if not os.path.exists(p):
        continue
    raw = open(p, "rb").read()
    t = raw.decode("utf-8")
    if "唤醒包" in t:
        print(f"{fn:16s} 已有唤醒，跳过"); continue
    if anchor not in t:
        print(f"{fn:16s} ⚠ 找不到锚点 {anchor}，跳过"); continue
    t = t.replace(anchor, anchor + "\n\n" + WAKE.rstrip("\n"), 1)
    open(p, "wb").write(t.encode("utf-8"))
    print(f"{fn:16s} ✓ 已加唤醒步骤")
