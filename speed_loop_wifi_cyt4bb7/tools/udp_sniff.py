"""UDP 抓包探针：验证车端 WiFi-SPI 到底有没有把遥测发到本机。

用法：python udp_sniff.py [秒数] [端口]
  - 默认监听 0.0.0.0:8086，跑 10 秒
  - 严格按 kart/motor_link 的帧格式校验：MAGIC A5 5A、CRC16-MODBUS、TAIL 00 00 80 7F
  - 打印每帧的 SEQ / 时间戳 / 通道数 / 前几个通道值，末尾给统计

排障顺序（收不到时按这个查）：
  1) 车端 MOTOR_HOST_IP 是不是热点网卡地址（Windows 移动热点固定 192.168.137.1）
  2) 车端 MOTOR_SOCK_TYPE 与上位机协议一致（UDP vs TCP）
  3) Windows 防火墙：入站 UDP 8086 是否放行（公用网络档案默认拦入站）
"""
import socket
import struct
import sys
import time

PORT = int(sys.argv[2]) if len(sys.argv) > 2 else 8086
SECS = float(sys.argv[1]) if len(sys.argv) > 1 else 10.0

MAGIC = b"\xa5\x5a"
TAIL = b"\x00\x00\x80\x7f"


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
        return 1
    s.settimeout(0.5)
    print(f"[..] 监听 UDP :{PORT}  {SECS} 秒 ...")

    t0 = time.time()
    pkts = 0
    bad = 0
    ok = 0
    bytes_total = 0
    last_seq = None
    gaps = 0
    shown = 0

    while time.time() - t0 < SECS:
        try:
            data, addr = s.recvfrom(65535)
        except socket.timeout:
            continue
        pkts += 1
        bytes_total += len(data)

        # 帧格式：MAGIC(2) TYPE(1) LEN(2) PAYLOAD CRC(2) TAIL(4)
        # ⚠ 格式不认识**不算失败**：先证明"包到了"，把原始 hex 打出来（别人的固件也能用这个探针）
        if len(data) < 11 or data[:2] != MAGIC or data[-4:] != TAIL:
            bad += 1
            if bad <= 5:
                print(f"  [非本协议帧] 来自 {addr}  {len(data)}B  {data[:32].hex(' ')}")
            continue

        ftype = data[2]
        plen = struct.unpack_from("<H", data, 3)[0]
        body = data[5:5 + plen]
        crc_rx = struct.unpack_from("<H", data, 5 + plen)[0]
        crc_calc = crc16(data[2:], 3 + plen)
        if crc_rx != crc_calc:
            bad += 1
            if bad <= 3:
                print(f"  [CRC 错] got 0x{crc_rx:04X} calc 0x{crc_calc:04X}")
            continue

        ok += 1
        if ftype == 0x01 and plen >= 10:      # 遥测
            seq, ts, mask, cnt = struct.unpack_from("<HIHH", body, 0)
            ch = list(struct.unpack_from("<%df" % cnt, body, 10))[:12]
            if last_seq is not None and seq != (last_seq + 1) & 0xFFFF:
                gaps += (seq - last_seq) & 0xFFFF
            last_seq = seq
            if shown < 6:
                shown += 1
                vals = " ".join(f"{v:8.2f}" for v in ch)
                print(f"  [遥测] seq={seq:5d} ts={ts:7d}ms cnt={cnt:2d} | {vals}")
        elif ftype == 0x02 and shown < 12:     # 参数元数据
            pid = body[0]
            lo, hi, df, st = struct.unpack_from("<ffff", body, 1)
            unit = body[17:25].replace(b"\x00", b"").decode("utf-8", "replace")
            name = body[25:41].replace(b"\x00", b"").decode("utf-8", "replace")
            print(f"  [参数表] id={pid} {name} [{lo:g},{hi:g}] def={df:g} step={st:g} {unit}")
        elif ftype == 0x06 and shown < 12:     # 事件
            txt = body[1:].replace(b"\x00", b"").decode("utf-8", "replace")
            print(f"  [事件] {txt}")
        else:
            print(f"  [TYPE 0x{ftype:02X}] 来自 {addr}，{len(data)} 字节")

    dur = time.time() - t0
    print()
    print(f"[统计] {dur:.1f}s  包={pkts}  合规帧={ok}  坏帧={bad}  字节={bytes_total}")
    if ok:
        print(f"       约 {ok / dur:.1f} 帧/秒，约 {bytes_total / dur:.0f} B/s")
    if gaps:
        print(f"       SEQ 缺口 {gaps} 帧（丢包）")
    if pkts == 0:
        print("[诊断] 一个包都没收到：")
        print("       1) 车端 MOTOR_HOST_IP / MOTOR_SOCK_TYPE 是否与这里一致")
        print("       2) PC 是否开着移动热点（热点网卡应为 192.168.137.1）")
        print("       3) Windows 防火墙是否放行入站 UDP 8086（公用网络档案默认拦入站）")
    s.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
