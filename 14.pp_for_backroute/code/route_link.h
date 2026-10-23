/* C 车惯导 + 路表 —— 串口"灌表"链路（PC → 车）
 * 做什么
 * 把 PC 上做好的**理论路表**（电子地图 → 中线 → 车端字节镜像，见 tools/route_to_carbin.py）
 * 通过**调试串口**灌进车里的某个 Flash 槽，并选为当前表。
 * （`motor_parse_rx()` 写死 LEN==8、`motor_rxbuf[256]`、模块单帧 ~200 B）。
 * 串口是自己的字节流，一层都没有。
 * 走哪条串口
 * `DEBUG_UART_INDEX`（UART_0，P00_1/P00_0）—— 就是 **DAP 自带的那个虚拟串口**（PC 侧 COM33）。
 * ⇒ 传输期间把日志"静音"（`zf_common_debug.c` 的 `__write` 检查本模块的门闩），
 * 收完 COMMIT/ABORT/超时再放开。**传输期间日志会有一段空白，这是有意的。**
 * ⇒ 反方向（PC→车）不受影响：帧有同步头 + CRC，PC 侧扫描同步头即可。
 * 想更快就改那个宏（PC 端也要跟着改）—— 但**先确认 DAP 的虚拟串口吃得住**。
 * 帧格式（全部小端）
 * ┌────────┬────────┬─────────┬──────────────┬──────────┐
 * │ A5 5A  │ type   │ len(2)  │ payload[len] │ crc16(2) │
 * └────────┴────────┴─────────┴──────────────┴──────────┘
 * crc16 = CRC16-MODBUS（0xFFFF / 反射多项式 0xA001），覆盖 type|len|payload（不含同步头）
 * len   = payload 字节数，上限 RL_PAYLOAD_MAX
 * 命令表（PC → 车）
 * 0x01 PING        无
 * 0x02 DIR_REQ     无                          → 0x82 DIR_DATA
 * 0x03 LOAD_BEGIN  slot(1) total_bytes(4)      → 0x85 ACK
 * 0x04 LOAD_DATA   offset(4) len(2) bytes[len] → 0x85 ACK
 * 0x05 LOAD_COMMIT 无                          → 0x85 ACK + （写完）0x86 LOAD_DONE
 * 0x06 LOAD_ABORT  无                          → 0x85 ACK
 * 0x07 SLOT_SEL    slot(1) persist(1) load(1)  → 0x85 ACK
 * 0x08 SLOT_LOAD   slot(1)                     → 0x85 ACK
 * 0x09 SLOT_READ   slot(1) offset(4) n(2)      → 0x87 SLOT_DATA（拉模式，车端无状态）
 * 回复（车 → PC）
 * 0x81 PONG        proto_ver(1) slot_count(1) slot_pages(1) rec_size(1) max_bytes(4) caps(4)
 * 0x82 DIR_DATA    active(1) n(1) {valid(1) slot(1) rsv(2) count(4) lap_mm(4) ds_mm(4) crc_r(4)}×n
 * 0x85 ACK         cmd(1) result(1) detail(2)
 * 0x86 LOAD_DONE   slot(1) count(4) lap_mm(4)
 * 0x87 SLOT_DATA   slot(1) offset(4) n(2) bytes[n]
 * **下发只允许写槽 1..INS_RT_SLOT_COUNT-1**；槽 0 是"手推录制"的家。
 * 所以在协议层就不可能覆盖手推的那张表。
 * 线程/中断约定
 * `route_link_rx_byte()` 只在 **UART_0 接收中断**里调（只投一个字节进环形缓冲）。
 * `route_link_poll()`    只在**主循环**调（会写 Flash、会阻塞发串口）。
 */

#ifndef _ROUTE_LINK_H_
#define _ROUTE_LINK_H_

#include "zf_common_headfile.h"

/* 协议常量（tools/route_send.py 必须与此逐一对应） */
#define RL_SYNC0                (0xA5u)
#define RL_SYNC1                (0x5Au)

#define RL_T_PING               (0x01u)
#define RL_T_DIR_REQ            (0x02u)
#define RL_T_LOAD_BEGIN         (0x03u)
#define RL_T_LOAD_DATA          (0x04u)
#define RL_T_LOAD_COMMIT        (0x05u)
#define RL_T_LOAD_ABORT         (0x06u)
#define RL_T_SLOT_SEL           (0x07u)
#define RL_T_SLOT_LOAD          (0x08u)
#define RL_T_SLOT_READ          (0x09u)     /* slot(1) offset(4) n(2) → 0x87 SLOT_DATA */

#define RL_T_PONG               (0x81u)
#define RL_T_DIR_DATA           (0x82u)
#define RL_T_ACK                (0x85u)
#define RL_T_LOAD_DONE          (0x86u)
#define RL_T_SLOT_DATA          (0x87u)     /* slot(1) offset(4) n(2) bytes[n]（上传用） */

#define RL_PROTO_VER            (2u)

/* 车端只会回一个笼统的 `RL_ERR_STATE`（"状态不允许"），现场完全看不出是"固件旧"。
 * 真实案例：上位机点"上传" → `!! 读槽被拒：状态不允许（cmd=0x09 detail=0）`，
 * 有了 caps，上位机在 OPEN（PING）那一刻就知道车端会什么：
 * · v2 PONG（12 B）⇒ 按 caps 判，缺哪项直接点名；
 * · v1 PONG（8 B）⇒ 老固件，没有能力位 ⇒ 上位机按"1 版已知能力"推断（有 DIR/LOAD/SEL，**没有读槽**）。
 */
#define RL_CAP_DIR              (1u << 0)   /* 0x02/0x82  槽位一览 */
#define RL_CAP_LOAD             (1u << 1)   /* 0x03..0x06 灌表（PC→车） */
#define RL_CAP_SLOT_SEL         (1u << 2)   /* 0x07/0x08  选当前槽 / 载入 RAM */
#define RL_CAP_SLOT_READ        (1u << 3)   /* 0x09/0x87  读槽（车→PC，上传） */
/* 位保留但不上报，PC 侧看到 caps 里没这一位就不该发这个命令。 */
#define RL_CAP_STEP             (1u << 4)   /* 保留位，未实现 */
#define RL_CAP_ALL              (RL_CAP_DIR | RL_CAP_LOAD | RL_CAP_SLOT_SEL | RL_CAP_SLOT_READ)
#define RL_PAYLOAD_MAX          (768u)      /* 单帧 payload 上限（含 LOAD_DATA 的 6 B 头） */
#define RL_CHUNK_MAX            (512u)      /* 一帧最多搬多少"镜像字节" */
#define RL_RX_RING_SZ           (4096u)
#define RL_LOAD_TIMEOUT_MS      (3000u)     /* 传了一半没下文就自动放弃 */

/* ACK 的 result 码（PC 侧的判据，别随便改数值） */
#define RL_OK                   (0u)
#define RL_ERR_SLOT             (1u)        /* 槽号越界 / 试图写槽 0 */
#define RL_ERR_TOO_BIG          (2u)        /* total_bytes 超过单槽上限 */
#define RL_ERR_OFFSET           (3u)        /* LOAD_DATA 的 offset 不连续（丢帧了） */
#define RL_ERR_IMAGE            (4u)        /* 镜像 CRC/magic/布局不过 */
#define RL_ERR_NOSTART          (5u)        /* 没先 LOAD_BEGIN 就发 DATA */
#define RL_ERR_BUSY             (6u)        /* 正在写 Flash，先别来 */
#define RL_ERR_STATE            (7u)        /* 状态不允许（例如没在传输却 COMMIT） */

void    route_link_init      (void);
/* UART_0 接收中断里调（只投一个字节，不做任何解析） */
void    route_link_rx_byte   (uint8 b);
/* （`uart_isr_mask()` 已经把这一批读进库缓冲了；库缓冲只有 16 格 ⇒ 必须立刻搬走） */
void    route_link_uart_irq  (void);
/* 主循环每圈调一次（幂等） */
void    route_link_poll      (void);

uint8   route_link_active    (void);    /* 1 = 正在收（日志此刻被静音） */
uint32  route_link_rx_frames (void);    /* 收到并 CRC 通过的帧数（累计） */
uint32  route_link_rx_bad    (void);    /* CRC 不过/丢弃的字节数 */
uint32  route_link_last_ms   (void);    /* 最近一次收到合法帧的时刻（motor_link_ms 时基） */

#endif
