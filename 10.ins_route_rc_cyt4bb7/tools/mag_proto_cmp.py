"""对比「协议 0（命令 0x8021）」与「协议 1（命令 0x3FFF）」拿到的原始帧，
并同时把 raw 用几种掩码解一遍，直观看出哪个掩码才是对的。

用法：python mag_proto_cmp.py [每种协议秒数]
"""
import collections
import socket
import struct
import sys
import time

SEC = float(sys.argv[1]) if len(sys.argv) > 1 else 2.5
PORT = 8086
MAGIC_DN, MAGIC_UP, TAIL = b"\x5a\xa5", b"\xa5\x5a", b"\x00\x00\x80\x7f"


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
try:
    s.bind(("0.0.0.0", PORT))
except OSError as e:
    print(f"[ERR] 绑定 :{PORT} 失败：{e}  → 先把上位机断开")
    sys.exit(1)
s.settimeout(0.5)
peer, tok = None, 0


def pump(sec):
    global peer, tok
    t0, out = time.time(), []
    while time.time() - t0 < sec:
        try:
            data, addr = s.recvfrom(65535)
        except socket.timeout:
            continue
        if len(data) < 11 or data[:2] != MAGIC_UP or data[-4:] != TAIL:
            continue
        ftype, plen = data[2], struct.unpack_from("<H", data, 3)[0]
        if 5 + plen + 2 > len(data) or ftype != 0x01:
            continue
        if struct.unpack_from("<H", data, 5 + plen)[0] != crc16(data[2:], 3 + plen):
            continue
        peer = addr
        body = data[5:5 + plen]
        if struct.unpack_from("<H", body, 8)[0] >= 16:
            out.append(int(struct.unpack_from("<f", body, 10 + 12 * 4)[0]))
    return out


pump(3.0)
if peer is None:
    print("[ERR] 收不到车端遥测"); sys.exit(2)
print(f"[OK] 车端 {peer[0]}:{peer[1]}\n")

res = {}
for proto, cmdname in ((0, "命令 0x8021"), (1, "命令 0x3FFF")):
    tok += 1
    s.sendto(build_cmd(0x03, 6, float(proto), tok), peer)
    pump(0.8)
    v = pump(SEC)
    res[proto] = v
    if not v:
        print(f"协议 {proto}（{cmdname}）：没收到帧")
        continue
    hi = [x >> 8 for x in v]
    print(f"协议 {proto}（{cmdname}）：{len(v)} 帧，raw {min(v):5d}~{max(v):5d}"
          f" (0x{min(v):04X}~0x{max(v):04X})，高字节 {sorted(set(hi))}"
          f"，取值 {len(set(v))} 种")

print()
print("--- 同一份 raw，用不同掩码解码（看哪个合理）---")
for proto in (0, 1):
    v = res.get(proto) or []
    if not v:
        continue
    print(f"\n协议 {proto}:")
    for mask, full, name in ((0x7FFF, 32768, "0x7FFF/32768 ← 现在的协议 0"),
                             (0x3FFF, 16384, "0x3FFF/16384 ← 我怀疑的正确解"),
                             (0xFFFF, 65536, "0xFFFF/65536")):
        a = [x & mask for x in v]
        lo, hg = min(a), max(a)
        print(f"   {name:34s} 角度 {lo:6d}~{hg:6d} → {lo/full*360:6.1f}°~{hg/full*360:6.1f}°"
              f"  覆盖 {100.0*(hg-lo)/full:5.1f}%")
    # 高两位是否恒定
    b15 = sum((x >> 15) & 1 for x in v)
    b14 = sum((x >> 14) & 1 for x in v)
    print(f"   bit15=1 占 {b15/len(v)*100:5.1f}%   bit14=1 占 {b14/len(v)*100:5.1f}%")

print()
print("=" * 70)
a = res.get(0) or []
b = res.get(1) or []
if a and b:
    same = abs(min(a) - min(b)) < 200 and abs(max(a) - max(b)) < 200
    print(f"协议 0 与 1 的 raw 区间{'接近' if same else '差别很大'}"
          f"（0: 0x{min(a):04X}~0x{max(a):04X} ｜ 1: 0x{min(b):04X}~0x{max(b):04X}）")
    if same:
        print("→ 两种命令拿到的是同类帧 → 直接切协议 1 就能用正确的 14 位解码，**不用重烧**。")
    else:
        print("→ 换命令会改变回读内容 → 协议 1 的命令不适用；需要重烧一条"
              "「命令 0x8021 + 掩码 0x3FFF + 满量程 16384」的协议。")
print("=" * 70)
s.close()
