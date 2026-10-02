"""判别实验：在两种 SPI 读命令之间快速来回切，看回读值跟不跟着命令走。

用法：python mag_alt.py [每段秒数] [轮数]

结论怎么读：
  · 值**跟着命令变、且每次重复都可复现** → MISO 上看到的是 MOSI/SCK 串扰，
    编码器没有驱动 MISO（没供电 / MISO 没接 / 与 MOSI 接反）→ 查硬件
  · 值**与命令无关、始终一样** → 线上有个固定电平（可能是被拉死）
  · 值**随机分散** → 噪声
"""
import socket
import struct
import sys
import time
import collections

SEG = float(sys.argv[1]) if len(sys.argv) > 1 else 1.2
ROUNDS = int(sys.argv[2]) if len(sys.argv) > 2 else 5
PORT = 8086

MAGIC_DN, MAGIC_UP, TAIL = b"\x5a\xa5", b"\xa5\x5a", b"\x00\x00\x80\x7f"
PROTO_NOP = 2        # 命令 0x0000
PROTO_MENC = 0       # 命令 0x8021


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
peer = None
tok = 0


def pump(sec):
    global peer, tok
    t0 = time.time()
    out = []
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
        peer = addr
        if ftype == 0x01 and plen >= 10:
            body = data[5:5 + plen]
            if struct.unpack_from("<H", body, 8)[0] >= 16:
                out.append(int(struct.unpack_from("<f", body, 10 + 12 * 4)[0]))
    return out


pump(3.0)
if peer is None:
    print("[ERR] 收不到车端遥测"); sys.exit(2)
print(f"[OK] 车端 {peer[0]}:{peer[1]}\n")

acc = {PROTO_NOP: collections.Counter(), PROTO_MENC: collections.Counter()}
names = {PROTO_NOP: "命令 0x0000", PROTO_MENC: "命令 0x8021"}
for r in range(ROUNDS):
    for p in (PROTO_NOP, PROTO_MENC):
        tok = (tok + 1) & 0xFFFF
        s.sendto(build_cmd(0x03, 6, float(p), tok), peer)   # SET mag_proto
        pump(0.35)
        vals = pump(SEG - 0.35)
        acc[p].update(vals)
        top = collections.Counter(vals).most_common(3)
        print(f"  轮{r+1} {names[p]}: " + "  ".join(f"0x{v:04X}×{n}" for v, n in top))

print()
print("=" * 66)
for p in (PROTO_NOP, PROTO_MENC):
    c = acc[p]
    tot = sum(c.values())
    top = c.most_common(4)
    print(f"  {names[p]:14s} 共 {tot:4d} 帧，主值 " +
          "  ".join(f"0x{v:04X}({n*100//max(tot,1)}%)" for v, n in top))

main_nop = acc[PROTO_NOP].most_common(1)[0][0] if acc[PROTO_NOP] else None
main_menc = acc[PROTO_MENC].most_common(1)[0][0] if acc[PROTO_MENC] else None
print("=" * 66)
print()
if main_nop == main_menc:
    print(f"[结论] 两种命令下主值相同（都是 0x{main_nop:04X}）→ 回读**与命令无关**。")
    print("       MISO 上是一个固定电平 → 多半被拉死/短接，或编码器输出恒为这个值。")
else:
    print(f"[结论] 回读**跟着命令变**（0x0000 → 0x{main_nop:04X} ；0x8021 → 0x{main_menc:04X}）。")
    print("       → MISO 上看到的几乎肯定是 MOSI/SCK 的串扰，")
    print("         也就是说**编码器没有在驱动 MISO**。优先查：")
    print("         ① 编码器 VCC/GND 有没有真的供上（万用表量芯片脚）")
    print("         ② MI 是不是接到了 P15_0（本工程 MISO）；MO 到 P15_1")
    print("         ③ MI/MO 有没有接反")
    print("       —— 转磁铁也测一下：若转的时候主值完全不抬，就更确定是上面这条。")
s.close()
