/* C 车惯导 + 路表 —— 串口灌表链路（PC -> 车）：把 PC 侧理论路表经调试串口灌进 Flash 槽并选为当前表
 * 串口 DEBUG_UART_INDEX（UART_0, P00_1/P00_0, DAP 虚拟串口），波特率 DEBUG_UART_BAUDRATE（115200）
 * 与 1 Hz 日志共用一根线：传输期间静音日志（debug_uart_mute），COMMIT/ABORT/超时后放开 */

#ifndef _ROUTE_LINK_H_
#define _ROUTE_LINK_H_

#include "zf_common_headfile.h"

/* ---- 帧格式（小端）：A5 5A | type | len(2) | payload[len] | crc16(2)，len 上限 RL_PAYLOAD_MAX
 * crc16 = CRC16-MODBUS（初值 0xFFFF / 反射多项式 0xA001），覆盖 type|len|payload，不含同步头；日志全为 ASCII（<0x80），不会撞出同步头 0xA5
 * 命令表（PC -> 车）
 *   0x01 PING 无                               -> 0x81 PONG
 *   0x02 DIR_REQ 无                            -> 0x82 DIR_DATA
 *   0x03 LOAD_BEGIN slot(1) total_bytes(4)     -> 0x85 ACK
 *   0x04 LOAD_DATA offset(4) len(2) bytes[len] -> 0x85 ACK
 *   0x05 LOAD_COMMIT 无                        -> 0x85 ACK，写完再发 0x86 LOAD_DONE
 *   0x06 LOAD_ABORT 无                         -> 0x85 ACK
 *   0x07 SLOT_SEL slot(1) persist(1) load(1)   -> 0x85 ACK
 *   0x08 SLOT_LOAD slot(1)                     -> 0x85 ACK
 *   0x09 SLOT_READ slot(1) offset(4) n(2)      -> 0x87 SLOT_DATA（拉模式，车端无状态）
 * 回复（车 -> PC）
 *   0x81 PONG proto_ver(1) slot_count(1) slot_pages(1) rec_size(1) max_bytes(4) caps(4)
 *   0x82 DIR_DATA active(1) n(1) {valid(1) slot(1) rsv(2) count(4) lap_mm(4) ds_mm(4) crc_r(4)}×n
 *   0x85 ACK cmd(1) result(1) detail(2) | 0x86 LOAD_DONE slot(1) count(4) lap_mm(4)
 *   0x87 SLOT_DATA slot(1) offset(4) n(2) bytes[n]
 */

/* 槽纪律：下发只允许写槽 1..INS_RT_SLOT_COUNT-1，槽 0 留给手推录制。
   route_link_rx_byte() / route_link_uart_irq() 只在 UART_0 接收中断里调，route_link_poll() 只在主循环调 */

/* ---- 协议常量（tools/route_send.py 必须与此逐一对应）---- */
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
#define RL_T_SLOT_READ          (0x09u)     /* slot(1) offset(4) n(2) -> 0x87 SLOT_DATA */

#define RL_T_PONG               (0x81u)
#define RL_T_DIR_DATA           (0x82u)
#define RL_T_ACK                (0x85u)
#define RL_T_LOAD_DONE          (0x86u)
#define RL_T_SLOT_DATA          (0x87u)     /* slot(1) offset(4) n(2) bytes[n]（上传用）*/

#define RL_PROTO_VER            (2u)        /* 2 = PONG 末尾多 caps(4)；1 = 只有前 8 B */

/* 能力位（PONG 的 caps 字段, u32 小端）：v2 PONG（12 B）按位判车端支持哪些命令，v1 PONG（8 B）无 caps；
   加新命令必须同步加一位，否则只能笼统回 RL_ERR_STATE */
#define RL_CAP_DIR              (1u << 0)
#define RL_CAP_LOAD             (1u << 1)
#define RL_CAP_SLOT_SEL         (1u << 2)   /* 0x07/0x08  选当前槽 / 载入 RAM    */
#define RL_CAP_SLOT_READ        (1u << 3)   /* 0x09/0x87  读槽（车->PC，上传）    */
#define RL_CAP_STEP             (1u << 4)   /* 保留位，未实现（转向阶跃）        */
#define RL_CAP_ALL              (RL_CAP_DIR | RL_CAP_LOAD | RL_CAP_SLOT_SEL | RL_CAP_SLOT_READ)
#define RL_PAYLOAD_MAX          (768u)      /* 单帧 payload 上限（含 LOAD_DATA 的 6 B 头）*/
#define RL_CHUNK_MAX            (512u)      /* 一帧最多搬多少镜像字节                      */
#define RL_RX_RING_SZ           (4096u)
#define RL_LOAD_TIMEOUT_MS      (3000u)     /* 传了一半没下文就自动放弃                    */

/* ACK 的 result 码（PC 侧判据，数值固定）*/
#define RL_OK                   (0u)
#define RL_ERR_SLOT             (1u)
#define RL_ERR_TOO_BIG          (2u)
#define RL_ERR_OFFSET           (3u)        /* LOAD_DATA 的 offset 不连续（丢帧） */
#define RL_ERR_IMAGE            (4u)        /* 镜像 CRC/magic/布局不过            */
#define RL_ERR_NOSTART          (5u)        /* 没先 LOAD_BEGIN 就发 DATA         */
#define RL_ERR_BUSY             (6u)
#define RL_ERR_STATE            (7u)        /* 状态不允许（如没在传输却 COMMIT）  */

/* ---- 接口 ---- */
void    route_link_init      (void);
/* UART_0 接收中断里调（只投一个字节，不做任何解析） */
void    route_link_rx_byte   (uint8 b);
/* UART_0 接收中断里调：把库缓冲里的字节全部搬进本模块环形缓冲（库缓冲只有 16 格，必须立刻搬走）*/
void    route_link_uart_irq  (void);
/* 主循环每圈调一次（幂等） */
void    route_link_poll      (void);

/* 诊断 */
uint8   route_link_active    (void);    /* 1 = 正在收（日志此刻被静音）*/
uint32  route_link_rx_frames (void);    /* 收到并 CRC 通过的帧数（累计）*/
uint32  route_link_rx_bad    (void);    /* CRC 不过/丢弃的字节数        */
uint32  route_link_last_ms   (void);    /* 最近一次收到合法帧的时刻（motor_link_ms 时基）*/

#endif
