"""更新技能 cyt4bb7-wifispi-host-link：
  · 撤回被证伪的「OTHER_TIME_OUT 1000→2」（改成"保持 1000，运行期靠 INT 引脚"）
  · 静默坑 3 → 5（加心跳、模拟器对齐）
  · 新增「车端接不上/读到假数据时怎么查」一节（串口 / pktmon / 换输入看输出）
  · 更新 description
"""
import os

ROOT = r"C:/Users/zjc39/.workbuddy/skills/cyt4bb7-wifispi-host-link"
P = os.path.join(ROOT, "SKILL.md")
t = open(P, "rb").read().decode("utf-8").replace("\r\n", "\n")


def sub(old, new, tag):
    global t
    assert t.count(old) == 1, f"[{tag}] 命中 {t.count(old)} 次"
    t = t.replace(old, new, 1)


# ---------- 1) description ----------
sub("含助查协议帧格式、参数表 X-Macro、上位机档案切换，以及三个静默失效坑"
    "（SET_ENABLE 按数值解释、OTHER_TIME_OUT=1000 阻塞 1 秒、失败分支不能停 poll）与对应的语义级自检写法。",
    "含助查协议帧格式、参数表 X-Macro、上位机档案切换、五个静默失效坑"
    "（SET_ENABLE 按数值解释、OTHER_TIME_OUT 别改小、失败分支不能停 poll、上位机必须发心跳、"
    "模拟器必须与固件策略对齐），以及「车端连上了但数据不对」的判别手法"
    "（串口状态行 / pktmon 不抢端口抓包 / 换输入看输出跟不跟）与语义级自检写法。",
    "desc")

# ---------- 2) §2 必改项 ----------
sub("""### 2. 必改的两个地方

| 位置 | 改动 | 理由 |
|---|---|---|
| 本工程 `libraries/zf_device/zf_device_wifi_spi.c` | `OTHER_TIME_OUT` `1000` → `2` | `wait_idle` 是死等 INT 的忙循环，原值＝阻塞整 1 秒 |
| 发帧前 | `if (0 == gpio_get_level(WIFI_SPI_INT_PIN)) return;` | 模块忙就丢这一帧：遥测可丢，主循环不能拖 |

并保留**每秒一行串口状态**（init_err / online / enable / rx_age / tx / 目标 / 实际），
这是"失败能看见"的最后保险。printf 不要用 `%f`（IAR 默认库不保证浮点格式化，改定点打印）。""",
"""### 2. 别动超时宏；不阻塞靠"先看 INT"

⛔ **`OTHER_TIME_OUT` 保持驱动默认 `1000`，不要改小。**（这条我踩过：曾改成 2 图"不阻塞主循环"，
结果它**初始化也用**——复位模块→读版本→连 WiFi→建 socket 全靠这个超时等模块响应，
2 ms 等于只给刚复位完的模块 2 ms，**初始化直接判失败**。
对照一份实测能连上的参考工程，两份 `zf_device_wifi_spi.c` 归一化编码后逐行 diff 只差这一处。）

✅ 运行期不阻塞的正确做法——**发和收之前都**先看 INT 引脚，模块忙就跳过这一轮：

| 位置 | 判断 |
|---|---|
| 发遥测 | `if (0 == gpio_get_level(WIFI_SPI_INT_PIN)) return;`（丢这一帧：遥测可丢，主循环不能拖） |
| 收命令 | 同样要加（**很容易漏**，漏了就会吃满 `wait_idle` 的阻塞） |

另外 `xxx_link_init()` 之前加 `system_delay_ms(500)` 给模块上电稳定时间
（原厂例程是**按键触发**连 WiFi，天然隔了几秒；上电即连的工程必须自己补）。

并保留**每秒一行串口状态**（init_err / online / enable / rx_age / tx / 目标 / 实际），
这是"失败能看见"的最后保险。printf 不要用 `%f`（IAR 默认库不保证浮点格式化，改定点打印）。
⚠ IAR 双核工程 `Build`(F7) **只编当前选中的项目**，下载会报 `cm_7_1.hex was missing` → 用 **Batch build**。""",
    "sec2")

# ---------- 3) 静默坑 ----------
sub("""## 三个静默失效坑（每次都要检查）

1. **SET_ENABLE 的 VALUE 按数值解释**：上位机 `(ushort)bits`，车端 `(uint16)(val + 0.5f)`。
   写成 `(uint16)raw` 会读到 IEEE754 位模式的低 16 位（6.0f → `00 00 C0 40` → `0x0000`），
   **使能永远传不下来**，且不报错。
2. **OTHER_TIME_OUT**（见上表）。
3. **失败分支里必须继续 poll**，否则失败完全看不见（曾有表现"烧进去了串口毫无数据"）。""",
"""## 五个静默失效坑（每次都要检查）

1. **SET_ENABLE 的 VALUE 按数值解释**：上位机 `(ushort)bits`，车端 `(uint16)(val + 0.5f)`。
   写成 `(uint16)raw` 会读到 IEEE754 位模式的低 16 位（6.0f → `00 00 C0 40` → `0x0000`），
   **使能永远传不下来**，且不报错。
2. **OTHER_TIME_OUT**（见上节：保持 1000，别改小）。
3. **失败分支里必须继续 poll**，否则失败完全看不见（曾有表现"烧进去了串口毫无数据"）。
4. ⭐ **上位机必须发心跳**。车端按 `rx_age > lost_ms`（默认 600ms）判失联并清输出；
   而上位机只在用户操作时才发命令 → **松手 0.6 秒电机自己停**，现象是"拖一下动一下"。
   做法：抽一个 `Heartbeat` 单元（周期常量 + `Beat(link, nextToken)`），用 100ms 定时器调它
   （PING 只 15 字节、车端 ACK 不写日志，代价可忽略）。**别用"把 lost_ms 调大"绕过**——那是把看门狗调钝。
5. ⭐⭐ **模拟车端必须逐条实现固件的门控策略**。上面第 4 条能穿过 68 项自检，
   根因就是 `Simulator.cs` 把 `online` 通道写成常量 `1f`、没实现 `lost_ms` 判据 ——
   **模拟器比固件宽松 = 自己测自己**。给模拟器加任何"物理行为"前，先对着固件的策略函数
   （如 `motor_policy_target()` / `policy` 段）逐条比一遍；自检里等待命令的 `Thread.Sleep`
   要换成"边等边发心跳"的辅助函数，走 App 用的同一条代码路径。""",
    "pits")

# ---------- 4) 新增排障一节 ----------
sub("""## 参考

- `references/protocol-and-pitfalls.md`：帧格式、引脚、参数元数据布局、排障顺序
- `scripts/check_ewp.py`：IAR 工程路径与分组校验""",
"""## 「连上了但数据不对」怎么查（三条都不需要专用工具）

**判据的核心思想：换一个会改变输出的输入，看输出跟不跟着变。** 输出跟着"不该影响它的东西"走，
就说明真正的信号源没接上。

### A. 串口状态行 —— 排障第一现场
每秒一行（`link_err` / `cfg` / `online` / `en` / `rx_age` / `tx` / 目标 / 实际）。
- `tx` 匀速 +50/s 但 `rx_age` 一直涨、`online=0` → **上位机没在说话**（十有八九是坑 4）
- `link_err=1` = WiFi 名/密码错；`=2` = socket 没建起来（IP/端口/协议/防火墙）
- **读它比绑端口更安全**：串口不占 8086，上位机开着也能读。注意逐飞工程 `user/*.c` 是 **GBK**，
  脚本里要按编码解码；波特率默认 115200。

### B. 抓包：`pktmon`（上位机独占端口时用它）
上位机在 UDP 模式会**独占**那个本地端口（刻意不加 `ReuseAddress`），这时直接 bind 会报
`WinError 10013`。用 Windows 自带的 `pktmon` 在协议栈层抓，**不占端口**：
```powershell
pktmon filter remove; pktmon filter add -p <端口>
pktmon start --capture -f out.etl; Start-Sleep 5; pktmon stop
pktmon format out.etl -o out.txt
```
一行摘要就够判断"谁发给谁、UDP 长度多少"（能反推车端固件的通道数）。
坑：输出是 **UTF-16-LE**；默认**不带 payload hex**；
`DropReason INET: transport endpoint was not found` = **那一刻没有进程监听该端口**（也能用来查上位机退没退）。

### C. 外设（SPI 传感器）读到假数据的判别
板子**不校验来源端口**，所以没有上位机时可以自己绑端口、**直接发下行命令当上位机用**
（切协议/改参数/看遥测）。判别脚本四件套：盯通道 / 轮协议 / 长采样+直方图 / **在两个读命令之间来回切**。

最后那个最有信息量：

| 现象 | 含义 |
|---|---|
| 回读**随命令变**且可复现 | 是 MOSI/SCK **串扰** → 传感器根本没驱动 MISO（查供电、查接线，不是查协议） |
| 回读**与命令无关**、恒为某值 | 线被拉死/短接 |
| 回读随机分散 | 噪声 |
| 动一下被测物，回读**连续变化** | 通了，再查数值换算 |

⚠ 别被"偶尔跳一下像真读数"带偏——浮空线上的偶发耦合也会那样。**要看的是统计上稳不稳定、跟不跟着命令。**

## 参考

- `references/protocol-and-pitfalls.md`：帧格式、引脚、参数元数据布局、排障顺序
- `scripts/check_ewp.py`：IAR 工程路径与分组校验""",
    "diag")

open(P, "wb").write(t.encode("utf-8"))
print("SKILL.md ✓ 已更新（撤回错误结论 + 5 个坑 + 排障一节）")

# ---------- 5) 刷新参考文件 ----------
src = r"D:/jisuyueye9car/.workbuddy/memory/车端无线链路速查.md"
dst = os.path.join(ROOT, "references", "protocol-and-pitfalls.md")
open(dst, "wb").write(open(src, "rb").read())
print("references/protocol-and-pitfalls.md ✓ 已同步最新版")
