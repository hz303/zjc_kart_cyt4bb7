"""磁编码器协议扫描探针（主动说话版）。

用法：python mag_probe.py [每种协议采样秒数]

它做的事：
  1. 绑 8086 收车端遥测，从首帧学会车端地址（源 IP:6666）
  2. 依次把参数「编码器|协议」(id=6) 设为 2/0/1，各采样一段时间
  3. 对每种协议统计 ch12 mag_raw 的不同取值个数 / 极差，并读回参数确认生效
  4. 给出结论：哪种协议能让 raw 随磁铁转动而变化

为什么能这么干：板子不校验来源端口，且此刻没有别的程序占 8086，所以可以直接当上位机用。
"""
import socket
import struct
import sys
import time

SECS_PER_PROTO = float(sys.argv[1]) if len(sys.argv) > 1 else 2.0
PORT = 8086

MAGIC_DN = b"\x5a\xa5"
MAGIC_UP = b"\xa5\x5a"
TAIL = b"\x00\x00\x80\x7f"

P_MAG_PROTO = 6
CMD_PING, CMD_GET, CMD_SET = 0x01, 0x02, 0x03

PROTOS = [(2, "RAW 探针"), (0, "MENC15A/MT6816"), (1, "AS5047/AS5147")]


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


class Car:
    def __init__(self):
        self.s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.s.bind(("0.0.0.0", PORT))
        self.s.settimeout(0.5)

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
        self.peer = None
        self.token = 0
        self.mag = []          # (raw, angle, deg, delta)
        self.ack = []          # (type, id, value)

    def send(self, cmd, pid=0, val=0.0):
        if self.peer is None:
            return
        self.token = (self.token + 1) & 0xFFFF
        self.s.sendto(build_cmd(cmd, pid, val, self.token), self.peer)

    def pump(self, seconds):
        """收包 seconds 秒；更新 peer / mag / ack"""
        t0 = time.time()
        while time.time() - t0 < seconds:
            try:
                data, addr = self.s.recvfrom(65535)
            except socket.timeout:
                continue
            if len(data) < 11 or data[:2] != MAGIC_UP or data[-4:] != TAIL:
                continue
            ftype = data[2]
            plen = struct.unpack_from("<H", data, 3)[0]
            if 5 + plen + 2 > len(data):
                continue
            if struct.unpack_from("<H", data, 5 + plen)[0] != crc16(data[2:], 3 + plen):
                continue
            body = data[5:5 + plen]
            if self.peer is None:
                self.peer = addr
            if ftype == 0x01 and plen >= 10:
                cnt = struct.unpack_from("<H", body, 8)[0]
                if cnt >= 16:
                    ch = struct.unpack_from("<%df" % cnt, body, 10)
                    self.mag.append(ch[12:16])
            elif ftype == 0x03 and plen >= 5:
                self.ack.append((ftype, body[0], struct.unpack_from("<f", body, 1)[0]))
            elif ftype in (0x04, 0x05) and plen >= 3:
                self.ack.append((ftype, body[2] if plen > 2 else 0, 0.0))


def main():
    c = Car()
    print(f"[..] 监听 :{PORT}，等车端第一帧 ...")
    c.pump(4.0)
    if c.peer is None:
        print("[ERR] 没收到车端遥测：车端没在发，或 MOTOR_HOST_IP 不是这台机器。")
        return 2
    print(f"[OK] 车端 = {c.peer[0]}:{c.peer[1]}")

    # 先 ping 一下，并确认参数表/使能状态
    c.pump(0.3)
    c.send(CMD_PING)
    c.pump(0.5)

    results = []
    for val, name in PROTOS:
        c.mag.clear()
        c.ack.clear()
        c.send(CMD_SET, P_MAG_PROTO, float(val))     # 切协议
        c.pump(0.4)
        c.send(CMD_GET, P_MAG_PROTO)                 # 读回确认
        c.pump(SECS_PER_PROTO)

        raws = [m[0] for m in c.mag]
        uniq = sorted(set(raws))
        readback = [v for (t, i, v) in c.ack if t == 0x03 and i == P_MAG_PROTO]
        results.append((name, val, raws, uniq, readback[-1] if readback else None))
        print(f"  [{name:16s}] 设成 {val} → 回读 {readback[-1] if readback else '?'}"
              f"  采样 {len(raws)} 帧  raw 不同取值 {len(uniq)} 个"
              f"  最小 {min(raws):.0f} 最大 {max(raws):.0f}")
        if len(uniq) <= 8:
            print(f"       取值: {[int(u) for u in uniq]}  (hex: {[hex(int(u)) for u in uniq]})")

    print()
    print("=" * 74)
    best = None
    for name, val, raws, uniq, rb in results:
        hs = [hex(int(u)) for u in uniq[:6]]
        print(f"  协议 {val} {name:18s} 取值{len(uniq):4d}种  极差 {max(raws)-min(raws):9.0f}  样例 {hs}")
        if best is None or len(uniq) > len(best[3]):
            best = (name, val, raws, uniq, rb)
    print("=" * 74)
    print()
    print(f"[结论] 变化最多的是「{best[0]}」(mag_proto={best[1]})，{len(best[3])} 种取值。")
    if len(best[3]) >= 4:
        print("       → SPI 有在回读**变化的数据**，说明它和某个协议对上了（或至少模拟前端在动）。")
        print("         请**用手慢慢匀速转磁铁**，再跑一次本脚本：对上的那种协议取值会连续变化。")
    else:
        print("       → 三种协议都只回固定值/零值，说明问题不在协议上，而在物理层：")
        print("         ① 模块 VCC（多为 3.3V，也有 5V 型）② GND 是否共地")
        print("         ③ SCK/MOSI/MISO/CS 四线是否接反（尤其 MISO 与 MOSI）")
        print("         ④ CS 是否接到 P15_3（不是别的脚）")
        print("         ⑤ 磁铁是否在芯片正上方 0.5~2 mm（轴向往复型最常见）")
    c.s.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
