/* 串口"灌表"链路实现（协议见 route_link.h） */

#include "zf_common_headfile.h"
#include "route_link.h"
#include "motor_link.h"         /* motor_link_now_ms() —— 主循环毫秒时基 */
#include "ins_route.h"

/* 定义在 `libraries/zf_common/zf_common_debug.c`（**两个核都编那个文件**，而本模块只在 CM7_0），
 * 它的 `__write()` 会检查这个变量：传输期间置 1 ⇒ printf 的字节全部丢弃。
 */
extern volatile uint8 debug_uart_mute;

/* 接收：环形缓冲（ISR 写 head，主循环读 tail —— 单生产者单消费者，不用锁） */
static volatile uint16 s_rx_head = 0u;
static volatile uint16 s_rx_tail = 0u;
static volatile uint8  s_rx_ring[RL_RX_RING_SZ];

static uint8   s_frm[RL_PAYLOAD_MAX + 8u];      /* 正在拼的一帧（同步2+type1+len2+payload+crc2） */
static uint16  s_frm_len = 0u;

static uint32  s_frames   = 0u;
static uint32  s_bad      = 0u;
static uint32  s_last_ms  = 0u;

/* 传输状态 */
static uint8   s_in_xfer   = 0u;                /* 1 = 正在收 LOAD_DATA */
static uint8   s_busy_write= 0u;                /* 1 = COMMIT 后正在分步写 Flash */
static uint8   s_slot      = 0u;
static uint32  s_total     = 0u;
static uint32  s_fill      = 0u;
static uint32  s_deadline  = 0u;

/* 记得复核 CM7_0 的 .bss 余量（384 KB）。 */
static uint8   s_buf[INS_RT_SLOT_MAX_BYTES];

static uint16 rl_crc16 (uint16 crc, const uint8 *p, uint32 n)
{
    uint32 i;
    uint8  j;

    /* 本工程里现在有三份实现（ins_route.c 的 rt_crc16、motor_link.c 的 motor_crc16、这里），
     * 以及 PC 侧的 Python。**它们必须算出同一个值** —— 否则现象是"帧看着发出去了，
     * 对面永远不回 ACK"，而三方各自都自认为正确。改一处要四处一起改。
     */
    for (i = 0; i < n; i ++)
    {
        crc ^= (uint16)p[i];
        for (j = 0u; j < 8u; j ++)
        {
            if (0u != (crc & 1u)) { crc = (uint16)((crc >> 1) ^ 0xA001u); }
            else                  { crc = (uint16)(crc >> 1); }
        }
    }
    return crc;
}

static uint32 rl_rd_u32 (const uint8 *p)
{
    return (uint32)p[0] | ((uint32)p[1] << 8) | ((uint32)p[2] << 16) | ((uint32)p[3] << 24);
}

static void rl_wr_u32 (uint8 *p, uint32 v)
{
    p[0] = (uint8)( v        & 0xFFu);
    p[1] = (uint8)((v >>  8) & 0xFFu);
    p[2] = (uint8)((v >> 16) & 0xFFu);
    p[3] = (uint8)((v >> 24) & 0xFFu);
}

static void rl_quiet (uint8 on)
{
    debug_uart_mute = (0u != on) ? 1u : 0u;
}

/* 最大的是 DIR_DATA（142 B payload ⇒ 156 B 帧 ⇒ 115200 下约 13.5 ms）。
 * 13 ms 的主循环停顿可以接受 —— 5 ms 中断（控制 + 惯导）照跑。
 */
static void rl_send (uint8 type, const uint8 *pl, uint16 len)
{
    static uint8 tx[RL_PAYLOAD_MAX + 8u];
    uint16 n = 0u;
    uint16 crc;

    tx[n ++] = RL_SYNC0;
    tx[n ++] = RL_SYNC1;
    tx[n ++] = type;
    tx[n ++] = (uint8)( len       & 0xFFu);
    tx[n ++] = (uint8)((len >> 8) & 0xFFu);
    if ((0u != len) && (NULL != pl)) { memcpy(tx + n, pl, len); }
    n = (uint16)(n + len);

    crc = rl_crc16(0xFFFFu, tx + 2u, (uint32)(3u + len));
    tx[n ++] = (uint8)( crc       & 0xFFu);
    tx[n ++] = (uint8)((crc >> 8) & 0xFFu);

    uart_write_buffer(DEBUG_UART_INDEX, tx, (uint32)n);
}

static void rl_ack (uint8 cmd, uint8 result, uint16 detail)
{
    uint8 pl[4];

    pl[0] = cmd;
    pl[1] = result;
    pl[2] = (uint8)( detail       & 0xFFu);
    pl[3] = (uint8)((detail >> 8) & 0xFFu);

    rl_send(RL_T_ACK, pl, 4u);
}

static void rl_reply_pong (void)
{
    uint8 pl[12];

    pl[0] = (uint8)RL_PROTO_VER;
    pl[1] = (uint8)INS_RT_SLOT_COUNT;
    pl[2] = (uint8)INS_RT_SLOT_PAGES;
    pl[3] = (uint8)INS_RT_REC_SIZE;
    rl_wr_u32(pl + 4u, (uint32)INS_RT_SLOT_MAX_BYTES);
    /* 多出来的 4 B 会被它忽略 ⇒ 向后兼容。新上位机靠它认出"车端固件旧"。 */
    rl_wr_u32(pl + 8u, (uint32)RL_CAP_ALL);

    rl_send(RL_T_PONG, pl, 12u);
}

static void rl_reply_dir (void)
{
    uint8 pl[2u + 20u * INS_RT_SLOT_COUNT];
    uint8 n = 0u;
    uint8 k;
    ins_route_slot_info_t inf;

    pl[n ++] = ins_route_slot_active();
    pl[n ++] = (uint8)INS_RT_SLOT_COUNT;

    for (k = 0u; k < (uint8)INS_RT_SLOT_COUNT; k ++)
    {
        (void)ins_route_slot_info(k, &inf);

        pl[n ++] = inf.valid;
        pl[n ++] = inf.slot;
        pl[n ++] = 0u;
        pl[n ++] = 0u;
        rl_wr_u32(pl + n, inf.count ); n = (uint8)(n + 4u);
        rl_wr_u32(pl + n, inf.lap_mm); n = (uint8)(n + 4u);
        rl_wr_u32(pl + n, inf.ds_mm ); n = (uint8)(n + 4u);
        rl_wr_u32(pl + n, inf.crc_r ); n = (uint8)(n + 4u);
    }

    rl_send(RL_T_DIR_DATA, pl, n);
}

static void rl_load_begin (const uint8 *pl, uint16 len)
{
    uint32 total;

    if (5u != len)                       { rl_ack(RL_T_LOAD_BEGIN, RL_ERR_STATE, len);  return; }
    if (0u != s_busy_write)              { rl_ack(RL_T_LOAD_BEGIN, RL_ERR_BUSY, 0u);    return; }
    if (0u != s_in_xfer)                 { s_in_xfer = 0u; }        /* 上一次没收完 ⇒ 覆盖重来 */

    s_slot = pl[0];
    total  = rl_rd_u32(pl + 1u);

    /* 现在放开 —— 用户要用手推录的表反复覆盖它做实验，禁令只会逼着人绕路。 不写就直接拒（默认仍不会误覆盖）。 */
    if (s_slot >= (uint8)INS_RT_SLOT_COUNT)
    {
        rl_ack(RL_T_LOAD_BEGIN, RL_ERR_SLOT, s_slot);
        return;
    }
    if ((total < 48u) || (total > (uint32)INS_RT_SLOT_MAX_BYTES))
    {
        rl_ack(RL_T_LOAD_BEGIN, RL_ERR_TOO_BIG, (uint16)(total & 0xFFFFu));
        return;
    }

    s_total    = total;
    s_fill     = 0u;
    s_in_xfer  = 1u;
    s_deadline = motor_link_now_ms() + RL_LOAD_TIMEOUT_MS;
    rl_quiet(1u);

    rl_ack(RL_T_LOAD_BEGIN, RL_OK, 0u);
}

static void rl_load_data (const uint8 *pl, uint16 len)
{
    uint32 off;
    uint16 n;

    if (0u == s_in_xfer)                 { rl_ack(RL_T_LOAD_DATA, RL_ERR_NOSTART, 0u); return; }
    if (len < 6u)                        { rl_ack(RL_T_LOAD_DATA, RL_ERR_STATE, len);   return; }

    off = rl_rd_u32(pl);
    n   = (uint16)pl[4] | ((uint16)pl[5] << 8);

    if ((uint32)n + 6u > (uint32)len)    { rl_ack(RL_T_LOAD_DATA, RL_ERR_STATE, len);   return; }
    if (off != s_fill)                   { rl_ack(RL_T_LOAD_DATA, RL_ERR_OFFSET, (uint16)(s_fill & 0xFFFFu)); return; }
    if ((off + (uint32)n) > s_total)     { rl_ack(RL_T_LOAD_DATA, RL_ERR_TOO_BIG, 0u);  return; }

    memcpy(&s_buf[s_fill], pl + 6u, n);
    s_fill    += n;
    s_deadline = motor_link_now_ms() + RL_LOAD_TIMEOUT_MS;

    rl_ack(RL_T_LOAD_DATA, RL_OK, (uint16)(s_fill & 0xFFFFu));
}

static void rl_load_commit (void)
{
    uint8 rc;

    if (0u == s_in_xfer)     { rl_ack(RL_T_LOAD_COMMIT, RL_ERR_NOSTART, 0u); return; }
    if (s_fill < s_total)    { rl_ack(RL_T_LOAD_COMMIT, RL_ERR_OFFSET, (uint16)(s_fill & 0xFFFFu)); return; }

    /* 先校验镜像 + 展开进 RAM，再"武装"分步写盘。 顶穿保护层的心跳门）。所以这里只 ACK "收到并已武装"，写完再发 LOAD_DONE。 */
    rc = ins_route_slot_load_image(s_slot, s_buf, s_fill);

    if (0u != rc)
    {
        s_in_xfer = 0u;
        rl_quiet(0u);
        rl_ack(RL_T_LOAD_COMMIT, (2u == rc) ? RL_ERR_IMAGE : RL_ERR_SLOT, rc);
        return;
    }

    s_in_xfer    = 0u;
    s_busy_write = 1u;
    rl_ack(RL_T_LOAD_COMMIT, RL_OK, 0u);
}

static void rl_abort (uint8 result)
{
    s_in_xfer    = 0u;
    s_fill       = 0u;
    s_busy_write = 0u;
    rl_quiet(0u);
    rl_ack(RL_T_LOAD_ABORT, result, 0u);
}

/* ---- 上传（车 → PC）：按镜像偏移拉一块。**拉模式**，车端不存状态 —— 一台机器的"流控"全在 PC 侧，车端只负责"要哪块给哪块"，出问题也只丢一块。---- */
static void rl_slot_read (const uint8 *pl, uint16 len)
{
    static uint8 out[7u + RL_CHUNK_MAX];
    uint8  slot;
    uint32 off;
    uint16 n, got;

    if (len < 7u) { rl_ack(RL_T_SLOT_READ, RL_ERR_STATE, len); return; }

    slot = pl[0];
    off  = rl_rd_u32(pl + 1u);
    n    = (uint16)pl[5] | ((uint16)pl[6] << 8);

    if (slot >= (uint8)INS_RT_SLOT_COUNT) { rl_ack(RL_T_SLOT_READ, RL_ERR_SLOT, slot); return; }
    if (n > (uint16)RL_CHUNK_MAX)         { rl_ack(RL_T_SLOT_READ, RL_ERR_TOO_BIG, n);   return; }
    if (0u == n)                          { rl_ack(RL_T_SLOT_READ, RL_ERR_STATE, 0u);    return; }

    got = ins_route_slot_read_bytes(slot, off, out + 7u, n);
    if (0u == got) { rl_ack(RL_T_SLOT_READ, RL_ERR_IMAGE, slot); return; }

    out[0] = slot;
    rl_wr_u32(out + 1u, off);
    out[5] = (uint8)( got       & 0xFFu);
    out[6] = (uint8)((got >> 8) & 0xFFu);

    rl_send(RL_T_SLOT_DATA, out, (uint16)(7u + got));
}

static void rl_slot_sel (const uint8 *pl, uint16 len, uint8 force_load)
{
    uint8 slot, persist, load, rc;

    if (len < 3u) { rl_ack(RL_T_SLOT_SEL, RL_ERR_STATE, len); return; }

    slot    = pl[0];
    persist = pl[1];
    load    = (0u != force_load) ? 1u : pl[2];

    if (slot >= (uint8)INS_RT_SLOT_COUNT) { rl_ack(RL_T_SLOT_SEL, RL_ERR_SLOT, slot); return; }

    rc = ins_route_slot_set_active(slot, (0u != persist) ? 1u : 0u);
    if (0u != rc) { rl_ack(RL_T_SLOT_SEL, RL_ERR_SLOT, slot); return; }

    if (0u != load)
    {
        if (0u == ins_route_flash_load_slot(slot))
        {
            rl_ack(RL_T_SLOT_SEL, RL_ERR_IMAGE, slot);
            return;
        }
    }

    rl_ack(RL_T_SLOT_SEL, RL_OK, slot);
}

static void rl_dispatch (uint8 type, const uint8 *pl, uint16 len)
{
    s_frames ++;
    s_last_ms = motor_link_now_ms();

    switch (type)
    {
    case RL_T_PING:        rl_reply_pong();                     break;
    case RL_T_DIR_REQ:     rl_reply_dir();                      break;
    case RL_T_LOAD_BEGIN:  rl_load_begin(pl, len);              break;
    case RL_T_LOAD_DATA:   rl_load_data(pl, len);               break;
    case RL_T_LOAD_COMMIT: rl_load_commit();                    break;
    case RL_T_LOAD_ABORT:  rl_abort(RL_OK);                     break;
    case RL_T_SLOT_SEL:    rl_slot_sel(pl, len, 0u);            break;
    case RL_T_SLOT_LOAD:   rl_slot_sel(pl, len, 1u);            break;
    case RL_T_SLOT_READ:   rl_slot_read(pl, len);               break;
    default:               rl_ack(type, RL_ERR_STATE, 0u);      break;
    }
}

/* 拆帧（逐字节喂）
 * 帧：A5 5A | type | len(2,LE) | payload | crc16(2,LE)，总长 = 7 + len
 * CRC 覆盖 type|len|payload。同步头/长度任一不对就**只丢 1 个字节**再重找
 * （丢整帧会把后面真正的一帧一起丢掉 —— 这条是 motor_link.c 那边踩过的）。
 */
static void rl_drop1 (void)
{
    if (s_frm_len > 1u) { memmove(s_frm, s_frm + 1u, (uint32)(s_frm_len - 1u)); }
    if (s_frm_len > 0u) { s_frm_len --; }
}

static void rl_feed (uint8 b)
{
    uint16 len;
    uint16 total;
    uint16 crc;
    uint16 got;

    if (s_frm_len >= (uint16)sizeof(s_frm)) { s_frm_len = 0u; }
    s_frm[s_frm_len ++] = b;

    /* --- 同步头：滑窗 --- */
    for (;;)
    {
        if (0u == s_frm_len)                     { return; }
        if (s_frm[0] != RL_SYNC0)                { rl_drop1(); continue; }
        if ((s_frm_len >= 2u) && (s_frm[1] != RL_SYNC1)) { rl_drop1(); continue; }
        break;
    }

    if (s_frm_len < 5u) { return; }                                 /* 还读不到 len */

    len = (uint16)s_frm[3] | ((uint16)s_frm[4] << 8);
    if (len > (uint16)RL_PAYLOAD_MAX) { s_bad ++; rl_drop1(); return; }

    total = (uint16)(7u + len);
    if (s_frm_len < total) { return; }

    crc = rl_crc16(0xFFFFu, s_frm + 2u, (uint32)(3u + len));
    got = (uint16)s_frm[5u + len] | ((uint16)s_frm[6u + len] << 8);

    if (crc == got)
    {
        uint8 type = s_frm[2];
        uint8 pl[RL_PAYLOAD_MAX];

        if (len > 0u) { memcpy(pl, s_frm + 5u, len); }
        s_frm_len = 0u;
        rl_dispatch(type, pl, len);
    }
    else
    {
        s_bad ++;
        rl_drop1();
    }
}

/* 对外接口 */
void route_link_init (void)
{
    s_rx_head = s_rx_tail = 0u;
    s_frm_len   = 0u;
    s_frames    = 0u;
    s_bad       = 0u;
    s_in_xfer   = 0u;
    s_busy_write= 0u;
    s_fill      = 0u;
    s_total     = 0u;
    rl_quiet(0u);
}

void route_link_rx_byte (uint8 b)
{
    uint16 nxt = (uint16)((s_rx_head + 1u) & (RL_RX_RING_SZ - 1u));

    if (nxt == s_rx_tail) { s_bad ++; return; }         /* 满：丢新字节 */
    s_rx_ring[s_rx_head] = b;
    s_rx_head = nxt;
}

/* （`uart_data_buffer[UART_0]`，最多 16 格，满了会清 FIFO）；`uart_query_byte()` 才是从
 * 那个缓冲里取。所以 ISR 里必须**立刻全部搬走**，不能只取一个 —— 取慢了库会丢字节。
 * 搬走的字节只进本模块的环形缓冲，解析留给主循环 `route_link_poll()`（解析要写 Flash）。
 */
void route_link_uart_irq (void)
{
    uint8 b;

    while (0u != uart_query_byte(DEBUG_UART_INDEX, &b))
    {
        route_link_rx_byte(b);
    }
}

void route_link_poll (void)
{
    /* --- 1) 把手上的字节全部喂进拆帧器 --- */
    while (s_rx_tail != s_rx_head)
    {
        uint8 b = s_rx_ring[s_rx_tail];

        s_rx_tail = (uint16)((s_rx_tail + 1u) & (RL_RX_RING_SZ - 1u));
        rl_feed(b);
    }

    /* --- 2) 收了一半没下文 ⇒ 超时放弃（顺手把日志放开）--- */
    if (0u != s_in_xfer)
    {
        if ((int32)(motor_link_now_ms() - s_deadline) >= 0)
        {
            s_in_xfer = 0u;
            s_fill    = 0u;
            rl_quiet(0u);
        }
    }

    /* --- 3) COMMIT 后分步写盘：写完了给 PC 一个 LOAD_DONE --- */
    if (0u != s_busy_write)
    {
        if (0u == ins_route_flash_busy())
        {
            uint8 rc = ins_route_flash_last_rc();

            s_busy_write = 0u;
            rl_quiet(0u);

            if (1u == rc)
            {
                uint8 pl[9];
                uint8 k = ins_route_flash_last_slot();

                pl[0] = k;
                rl_wr_u32(pl + 1, (uint32)ins_route_get_count());
                rl_wr_u32(pl + 5, (uint32)(ins_route_get_head()->total_dist * 1000.0f + 0.5f));
                rl_send(RL_T_LOAD_DONE, pl, 9u);
            }
            else
            {
                rl_ack(RL_T_LOAD_COMMIT, RL_ERR_IMAGE, rc);
            }
        }
    }
}

uint8  route_link_active    (void) { return s_in_xfer | s_busy_write; }
uint32 route_link_rx_frames (void) { return s_frames; }
uint32 route_link_rx_bad    (void) { return s_bad;    }
uint32 route_link_last_ms   (void) { return s_last_ms; }
