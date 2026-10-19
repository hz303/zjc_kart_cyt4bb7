"""磁编码器专用探针：判断车端 SPI 到底有没有和磁编码器通上。

用法：python mag_watch.py [秒数] [端口]

看什么：
  ch12 mag_raw    原始 16 bit —— **它动不动，是"SPI 通没通"的唯一判据**
  ch13 mag_angle  按当前协议取有效位后的角度
  ch14 mag_deg    角度（°，已含零点/方向）
  ch15 mag_delta  与上次读数之差 —— 转动时跳，停手贴 0

判据（脚本末尾会自动给结论）：
  · ch12 长期恒定为 0 或 0xFFFF  → SPI 没通（接线/供电/磁铁/CS）
  · ch12 在乱跳、但 ch13/14 不成 0~360 规律 → SPI 通了，协议选错（切 mag_proto）
  · ch12 恒定、是某个非 0 定值   → 通了一半（可能 MISO 被拉死，或模块一直在回同一寄存器）
  · ch14 平滑 0~360 循环         → 全部正确
"""
import collections
import socket
import struct
import sys
import time

SECS = float(sys.argv[1]) if len(sys.argv) > 1 else 12.0
PORT = int(sys.argv[2]) if len(sys.argv) > 2 else 8086

MAGIC = b"\xa5\x5a"
TAIL = b"\x00\x00\x80\x7f"

NAMES = ["duty_cmd", "duty_act", "dir", "pwm_raw", "enable", "online",
         "rx_age", "duty_limit", "ramp_ms", "tx_count", "uptime", "init_err",
         "mag_raw", "mag_angle", "mag_deg", "mag_delta"]


def crc16(buf, n):
    crc = 0xFFFF
    for i in range(n):
        crc ^= buf[i]
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc & 0xFFFF


def main():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        s.bind(("0.0.0.0", PORT))
    except OSError as e:
        print(f"[ERR] 绑定 :{PORT} 失败：{e}")
        print("      → 端口被占了。逐飞助手 / KartHost 都监听 8086，先关掉一个。")
        return 1
    s.settimeout(0.5)
    print(f"[..] 监听 UDP :{PORT}  {SECS:g} 秒（盯磁编码器）...")
    print()

    t0 = time.time()
    pkts = ok = bad = 0
    cnt_seen = 0
    src = None
    vals = []                                  # 每个通道的采样序列（均值/极值用）
    rows = []                                  # 逐帧快照（对齐判活用）
    next_print = t0 + 0.5

    while time.time() - t0 < SECS:
        try:
            data, addr = s.recvfrom(65535)
        except socket.timeout:
            continue
        pkts += 1
        src = addr
        if len(data) < 11 or data[:2] != MAGIC or data[-4:] != TAIL:
            bad += 1
            if bad <= 3:
                print(f"  [非本协议帧] {addr} {len(data)}B {data[:32].hex(' ')}")
            continue
        ftype = data[2]
        plen = struct.unpack_from("<H", data, 3)[0]
        body = data[5:5 + plen]
        if ftype != 0x01 or plen < 10:
            continue
        if struct.unpack_from("<H", data, 5 + plen)[0] != crc16(data[2:], 3 + plen):
            bad += 1
            continue
        ok += 1

        seq, ts, mask, cnt = struct.unpack_from("<HIHH", body, 0)
        cnt_seen = cnt
        ch = list(struct.unpack_from("<%df" % cnt, body, 10))
        while len(vals) < len(ch):
            vals.append([])
        for i, v in enumerate(ch):
            vals[i].append(v)
        rows.append((ts, ch[12:16] if len(ch) >= 16 else None))

        now = time.time()
        if now >= next_print:
            next_print = now + 0.5
            if len(ch) >= 16:
                print(f"  ts={ts:7d}ms  raw={ch[12]:9.1f}  angle={ch[13]:9.1f}  "
                      f"deg={ch[14]:7.2f}  delta={ch[15]:8.1f}")
            else:
                print(f"  ts={ts:7d}ms  cnt={cnt}（只有 {cnt} 个通道，没有编码器那 4 个）")

    dur = time.time() - t0
    print()
    print(f"[统计] {dur:.1f}s  包={pkts}  遥测帧={ok}  坏帧={bad}  源={src}")
    if ok == 0:
        print("[结论] 车端没在发遥测 —— 先解决链路，编码器还轮不到。")
        print("       车端 MOTOR_HOST_IP / MOTOR_SOCK_TYPE 是否与本机一致？防火墙入站 UDP 放行了吗？")
        return 2
    print(f"       约 {ok / dur:.1f} 帧/秒，通道数 cnt={cnt_seen}")

    if cnt_seen < 16:
        print()
        print(f"[结论] ⚠ 固件只有 {cnt_seen} 个通道 → **板上跑的是旧版固件**，根本没编进磁编码器代码。"
              f" 重新编译烧录这一版（16 通道）再测。")
        return 3

    def stat(i):
        a = vals[i]
        return min(a), max(a), sum(a) / len(a)

    print()
    print("  ┌ 通道            最小        最大        均值        极差")
    for i in (12, 13, 14, 15):
        lo, hi, av = stat(i)
        print(f"  │ {NAMES[i]:<10} {lo:11.1f} {hi:11.1f} {av:11.1f} {hi - lo:11.1f}")
    print("  └")

    lo12, hi12, av12 = stat(12)
    lo14, hi14, av14 = stat(14)
    lo15, hi15, av15 = stat(15)
    raw_span = hi12 - lo12
    # 去重后的 raw 取值个数：恒定时只有 1~2 个
    uniq_raw = len(set(vals[12]))

    print()
    if uniq_raw <= 2:
        if av12 == 0.0:
            print(f"[结论] ❌ ch12 mag_raw 恒为 0 —— **SPI 没通**。")
        elif abs(av12 - 65535.0) < 1.0:
            print(f"[结论] ❌ ch12 mag_raw 恒为 0xFFFF —— **MISO 一直是高**（最常见：模块没供电 / MISO 没接 / CS 没拉低）。")
        else:
            print(f"[结论] ⚠ ch12 mag_raw 恒为定值 {av12:.0f} —— SPI 有回读但内容不变，"
                  f"查 MISO 是否被拉死、或模块对当前读命令无响应。")
        print("       排查：① 模块 VCC 是否 3.3V 供电 ② SCK/MOSI/MISO/CS 四线是否接反、CS 是否真的是 P15_3")
        print("             ③ 磁铁是否在芯片正上方 0.5~2 mm（多数磁编码器要求轴向）")
        print("             ④ 把上位机「编码器协议」切到 2（RAW 探针）再试一遍")
    elif raw_span > 0 and uniq_raw > 2:
        print(f"[结论] ✅ SPI **通上了**：ch12 在变（{uniq_raw} 个不同取值，极差 {raw_span:.0f}）。")
        if hi14 > 359.0 or lo14 < 0.0:
            print(f"       ⚠ 但 ch14 mag_deg 越界（{lo14:.1f}~{hi14:.1f}）→ 协议可能不对，"
                  f"或 mag_zero 零点设歪了。")
        elif hi14 - lo14 > 300.0:
            print(f"       ✅ ch14 mag_deg 覆盖 {lo14:.1f}~{hi14:.1f}°，看起来是正常的整圈角度。")
        else:
            print(f"       · ch14 mag_deg 只覆盖 {lo14:.1f}~{hi14:.1f}°（没转满一圈，或没转）。")
        if abs(av15) < 1.0:
            print(f"       · ch15 mag_delta ≈ 0（极差 {hi15 - lo15:.0f}）→ 读数稳定，"
                  f"要么磁铁没动，要么读数被卡住。用手慢慢转一下再看。")
        else:
            print(f"       ✅ ch15 mag_delta 在动（均值 {av15:.0f}）→ 角度在实时更新。")
    else:
        print(f"[结论] ⚠ ch12 只有 {uniq_raw} 个不同取值、极差 {raw_span:.0f} —— 疑似半通，转一下再测。")

    s.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
