"""盯住某一种协议，长时间采样，看 raw 到底怎么变。

用法：python mag_hold.py [协议 0/1/2] [秒数]

输出：每次读数变化的时刻与值（含间隔），末尾给直方图与判定。
判断依据：
  · 真·磁编码器静止时：主值 + 少量 ±1 LSB 抖动，**跳变间隔无规律**
  · 悬空/无响应：只在一两个"整齐"的值之间跳（0x0000/0x4000/0x5000 这类），
    且**每次切换协议才换值** → 值是 MOSI 的串扰，不是芯片回的
"""
import socket
import struct
import sys
import time
import collections

PROTO = int(sys.argv[1]) if len(sys.argv) > 1 else 0
SECS = float(sys.argv[2]) if len(sys.argv) > 2 else 15.0
PORT = 8086

MAGIC_DN = b"\x5a\xa5"
MAGIC_UP = b"\xa5\x5a"
TAIL = b"\x00\x00\x80\x7f"


def crc16(buf, n):
    crc = 0xFFFF
    for i in range(n):
        crc ^= buf[i]
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc & 0xFFFF


def build_cmd(cmd, pid, val, token):
    pl = struct.pack("<BBfH", cmd, pid, val, token)
    body = bytes([0x01]) + struct.pack("<H", len(pl)) + pl
    return MAGIC_DN + body + struct.pack("<H", crc16(body, len(body)))


s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
s.bind(("0.0.0.0", PORT))
s.settimeout(0.5)

s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
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

peer = None
tok = [0]


def send(cmd, pid=0, val=0.0):
    if peer is None:
        return
    tok[0] = (tok[0] + 1) & 0xFFFF
    s.sendto(build_cmd(cmd, pid, val, tok[0]), peer)


def pump(sec):
    t0 = time.time()
    out = []
    global peer
    while time.time() - t0 < sec:
        try:
            data, addr = s.recvfrom(65535)
        except socket.timeout:
            continue
        if len(data) < 11 or data[:2] != MAGIC_UP or data[-4:] != TAIL:
            continue
        ftype, plen = data[2], struct.unpack_from("<H", data, 3)[0]
        if 5 + plen + 2 > len(data):
            continue
        if struct.unpack_from("<H", data, 5 + plen)[0] != crc16(data[2:], 3 + plen):
            continue
        if peer is None:
            peer = addr
        if ftype == 0x01 and plen >= 10:
            cnt = struct.unpack_from("<H", data, 8)[0]
            if cnt >= 16:
                body = data[5:5 + plen]
                ts = struct.unpack_from("<I", body, 2)[0]
                raw = struct.unpack_from("<f", body, 10 + 12 * 4)[0]
                out.append((ts, int(raw)))
    return out


pump(3.0)
if peer is None:
    print("[ERR] 收不到车端遥测")
    sys.exit(2)
print(f"[OK] 车端 {peer[0]}:{peer[1]}  设协议 {PROTO} 并采样 {SECS:g} 秒")

send(0x03, 6, float(PROTO))          # SET mag_proto
pump(0.6)
send(0x01)                           # PING
samples = pump(SECS)

hist = collections.Counter(v for _, v in samples)
print()
print("--- 读数变化序列（只打变化，含间隔 ms）---")
last = None
for ts, v in samples:
    if v != last:
        gap = "" if last is None else f"  (+{ts - lastts} ms)"
        print(f"  ts={ts:8d}  raw={v:6d}  0x{v:04X}{gap}")
        last = v
        lastts = ts
print()
print(f"--- 直方图（共 {len(samples)} 帧）---")
for v, n in hist.most_common(10):
    print(f"  0x{v:04X} ({v:6d})  ×{n}")
print()
uniq = len(hist)
top = hist.most_common(1)[0]
share = top[1] / len(samples)
print(f"[判定] {uniq} 种取值；主值 0x{top[0]:04X} 占 {share*100:.1f}%")

if share > 0.9 and uniq <= 6:
    print("       → 主值占比极高、取值极少 → 更像**没有从机响应**（悬空/串扰），而不是活的传感器。")
elif share > 0.5 and uniq <= 12:
    print("       → 主值 + 少量 ±1 LSB → 像**活的传感器但磁铁没动**。请转一下磁铁再跑一次。")
else:
    print("       → 取值分散 → 像噪声。检查供电与四根线。")

s.close()
