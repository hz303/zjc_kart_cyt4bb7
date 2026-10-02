# 不抢端口也能看车端数据：用 Windows 自带的 pktmon

## 什么时候需要这招

`KartHost` 在 **UDP 模式下会独占 8086**（`Link.ConnectUdp` 刻意不加 `ReuseAddress`，
免得两个程序各收一半包）。这时你直接跑 `udp_sniff.py` / `mag_probe.py` 会得到：

```
[ERR] 绑定 :8086 失败：[WinError 10013] 以一种访问权限不允许的方式做了一个访问套接字的尝试。
```

`pktmon` 是 Windows 10 1809+ 自带的抓包工具，它在**协议栈层**取样，
所以**能看到投递给别的进程的包**，不需要占用那个端口。

## 用法（管理员 PowerShell）

```powershell
pktmon filter remove
pktmon filter add -p 8086
pktmon start --capture -f D:\path\mag.etl
Start-Sleep -Seconds 5
pktmon stop
pktmon format D:\path\mag.etl -o D:\path\out.txt
```

收尾记得 `pktmon filter remove`（否则下次抓包还带着旧筛选器）。

## 输出怎么看

`pktmon format` 的文本长这样（每条事件一行摘要 + 缩进的细节行）：

```
[23]0004.06C4::2026-09-30 16:31:20.155844000 [Microsoft-Windows-PktMon] PktGroupId 6473924464345094, ...
	9C-13-9E-C6-9D-54 > F4-4E-B4-19-92-2B, ethertype IPv4 (0x0800), length 127: 192.168.50.238.6666 > 192.168.50.176.8086: UDP, length 85
```

一行就够判断：**谁发给谁、UDP 长度多少**。
（85 字节 = 本工程遥测帧 `11 + 10 + 16×4`，可以直接反推车端是不是跑的 16 通道固件。）

丢包会记成单独的 `丢弃` 事件，并带 `DropReason`：

```
丢弃： ... 方向 Rx, 类型 IP, 组件 101, ... DropReason INET: transport endpoint was not found
```

## 三个坑（都踩过）

1. **`pktmon format` 的输出是 UTF-16-LE**，直接按 utf-8 读会解码失败。
   ```python
   t = open("out.txt", "rb").read().decode("utf-16")
   ```
2. **默认不带载荷 hex**（只有协议摘要行）。要看 payload 字节，还是得绑端口；
   `pktmon` 用来回答"包到底到没到、谁发的、多大"这类问题最合适。
3. **`DropReason INET: transport endpoint was not found` = 那一刻没有任何进程监听该端口**。
   反过来这也是一个有用的发现手段：用它就能确认"上位机到底退没退出"。

## 实测用它查出来的两件事

- 车端在 `192.168.50.238:6666`，稳定往 `192.168.50.176:8086` 发 **85 字节** UDP →
  **链路没问题、板子跑的就是 16 通道固件**。
- 捕获那一刻所有包都被 `transport endpoint was not found` 丢掉 →
  **KartHost 其实已经退出了**（`netstat -ano -p UDP | grep 8086` 也已经是空的），
  所以随后可以直接绑端口做主动探测。
