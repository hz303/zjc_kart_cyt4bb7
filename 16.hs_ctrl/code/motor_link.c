/*********************************************************************************************************************
 * WiFi-SPI 链路实现: 帧收发、下行命令解析、参数表上报、路表 dump
 * 协议与引脚说明见 motor_link.h
 ********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "motor_link.h"
#include "proto.h"
#include <float.h>

/* ---- 一、参数变量: 定义在各模块内(spd_enc.c / spd_ctrl.c / spd_safety.c / motor_drv.c),
 * 本文件只通过 motor_link.h 的参数表取地址 ---- */

/* ---- 二、参数元数据 / 恢复默认值: 表在 param_table.h, 元数据只在 proto.c 展开一次 ---- */

/* 恢复默认值入口, 唯一实现在 proto_param_load_defaults(); ui_menu.c 与 LOAD 命令都走这里 */
void motor_param_load_defaults (void)
{
    proto_param_load_defaults();
}

/* ---- 三、协议常量与内部状态 ---- */
#define MOTOR_MAGIC_UP_0    (0xA5)
#define MOTOR_MAGIC_UP_1    (0x5A)
#define MOTOR_MAGIC_DN_0    (0x5A)
#define MOTOR_MAGIC_DN_1    (0xA5)

#define MOTOR_TYPE_TELEMETRY    (0x01)      /* 车 → PC */
#define MOTOR_TYPE_PARAM_META   (0x02)
#define MOTOR_TYPE_PARAM_VALUE  (0x03)
#define MOTOR_TYPE_ACK          (0x04)
#define MOTOR_TYPE_NAK          (0x05)
#define MOTOR_TYPE_EVENT        (0x06)
#define MOTOR_TYPE_COMMAND      (0x01)      /* PC → 车 */

#define MOTOR_CMD_PING          (0x01)
#define MOTOR_CMD_GET_PARAM     (0x02)
#define MOTOR_CMD_SET_PARAM     (0x03)
#define MOTOR_CMD_SET_ENABLE    (0x04)
#define MOTOR_CMD_SAVE          (0x05)
#define MOTOR_CMD_LOAD          (0x06)
#define MOTOR_CMD_GET_TABLE     (0x07)
#define MOTOR_CMD_ROUTE_DUMP    (0x08)      /* 读路表: 车端分帧把 RAM 里的当前表推上来 */
#define MOTOR_CMD_ESTOP         (0x09)      /* 急停(全局, 不检查权限) */
#define MOTOR_CMD_DRV_REQ       (0x0A)      /* 请求驱动权 */
#define MOTOR_CMD_DRV_REL       (0x0B)      /* 释放驱动权 */

/* 路表 dump 上行帧(与上位机 FrameType 对应)
 *   RouteHead 0x10 = rt_head_t 前 32 B(不含两个 CRC) / RouteData 0x11 = first(4)+n(2)+n x 16 B
 *   RouteEnd  0x12 = count(4) + lap_mm(4) */
#define MOTOR_FT_ROUTE_HEAD     (0x10)
#define MOTOR_FT_ROUTE_DATA     (0x11)
#define MOTOR_FT_ROUTE_END      (0x12)

/* 每帧记录数: 6 B 头 + n x 16 B 不超过 buf[128]-11 = 117 B, 5 条 -> 86 B 载荷 / 97 B 帧 */
#define MOTOR_RD_CHUNK          (5u)
#define MOTOR_RD_GAP_MS         (5u)        /* 两帧最小间隔 ms; 上位机 3 s 无新帧判超时 */

/* 连续多少拍发不出去就放弃(200 x 5 ms ~= 1 s), 防止状态机卡在 HEAD */
#define MOTOR_RD_TRY_MAX        (200u)

/* dump 期间遥测周期压到 100 ms, 否则遥测几乎占满 SPI, dump 抢不到通道 */
#define MOTOR_RD_TEL_MS         (100u)

/* 串口自证: 打印 dump 进度与失败原因, 排完问题可改 0 */
#define MOTOR_RD_DEBUG          (1)

/* 使能位：bit1 电机 / bit2 总使能（与上位机 EnableBits 一致） */
#define MOTOR_EN_MOTOR          (0x0002)
#define MOTOR_EN_MASTER         (0x0004)

static volatile uint32  motor_tick_ms    = 0;        /* 由 5ms 中断累加        */
static uint32           motor_tx_ms      = 0;        /* 上次发遥测时刻         */
static uint32           motor_rx_ms      = 0;        /* 上次尝试收命令时刻     */
static uint32           motor_rx_last_ms = 0;        /* 最近一次收到下行帧     */
static uint8            motor_ever_rx    = 0;
static uint16           motor_tx_seq     = 0;
static uint16           motor_enable     = 0;        /* 上位机下发的使能位图   */
static uint32           motor_tx_count   = 0;
static uint8            motor_init_err   = 3;        /* 3=还没初始化完         */

/* 发送缓冲: 48 通道 => 11 + 10 + 48*4 = 213 B; 18 通道 => 93 B; 按上限 256 开 */
static uint8            motor_txbuf[256];
/* 接收缓冲 */
static uint8            motor_rxbuf[256];
/* 遥测通道值（按上限开：通道数可运行时切，见 s_ch_count） */
static float            motor_ch[MOTOR_CH_COUNT_MAX];

/* 运行时可切的遥测档案与遥测周期(见 motor_link.h 顶部说明) */
volatile float   spd_tel_sel    = 0.0f;                     /* 0=巡线(22) / 1=标定(18) / 2=全量(45) */
volatile float   spd_tel_period = (float)MOTOR_TX_PERIOD_MS;/* 遥测周期 ms          */
static   uint16  s_ch_count     = MOTOR_CH_COUNT_ROUTE;     /* 本拍真正发几个通道(默认巡线档) */
static   uint16  s_tx_period_ms = MOTOR_TX_PERIOD_MS;       /* 本拍真正用的发送周期 */

/* 库函数签名要 char*, 不能直接传字符串字面量(IAR 会报 const 转换) */
static char motor_ssid[]       = MOTOR_WIFI_SSID;
static char motor_pwd[]        = MOTOR_WIFI_PASSWORD;
static char motor_sock_type[]  = MOTOR_SOCK_TYPE;   /* 同上, 库函数签名要 char* */
static char motor_host_ip[]    = MOTOR_HOST_IP;
static char motor_host_port[]  = MOTOR_HOST_PORT;
static char motor_local_port[] = MOTOR_LOCAL_PORT;

/* ---- 四、小工具: CRC16 与小端读写 ---- */
static uint16 motor_crc16 (const uint8 *buf, uint16 len)
{
    uint16 crc = 0xFFFF;
    uint8  i, j;

    for (i = 0; i < len; i ++)
    {
        crc ^= buf[i];
        for (j = 0; j < 8; j ++)
        {
            if (crc & 0x01)
            {
                crc = (uint16)((crc >> 1) ^ 0xA001);
            }
            else
            {
                crc = (uint16)(crc >> 1);
            }
        }
    }
    return crc;
}

static uint16 motor_put_u16 (uint8 *b, uint16 o, uint16 v)
{
    b[o]     = (uint8)(v & 0xFF);
    b[o + 1] = (uint8)((v >> 8) & 0xFF);
    return (uint16)(o + 2);
}

static uint16 motor_put_u32 (uint8 *b, uint16 o, uint32 v)
{
    b[o]     = (uint8)(v & 0xFF);
    b[o + 1] = (uint8)((v >> 8) & 0xFF);
    b[o + 2] = (uint8)((v >> 16) & 0xFF);
    b[o + 3] = (uint8)((v >> 24) & 0xFF);
    return (uint16)(o + 4);
}

/* Cortex-M7 是小端, float 直接拷内存, 无格式化开销 */
static uint16 motor_put_f32 (uint8 *b, uint16 o, float v)
{
    memcpy(b + o, &v, 4);
    return (uint16)(o + 4);
}

static uint16 motor_get_u16 (const uint8 *b, uint16 o)
{
    return (uint16)((uint16)b[o] | ((uint16)b[o + 1] << 8));
}

static uint32 motor_get_u32 (const uint8 *b, uint16 o)
{
    return ((uint32)b[o]) | (((uint32)b[o + 1]) << 8)
         | (((uint32)b[o + 2]) << 16) | (((uint32)b[o + 3]) << 24);
}

static float motor_get_f32 (const uint8 *b, uint16 o)
{
    float v;
    memcpy(&v, b + o, 4);
    return v;
}

/* ---- 五、上行帧发送 ---- */
/* 返回 1 = 这一帧已交给模块; 0 = 模块忙, 本帧未发(调用方据此重试) */
static uint8 motor_send_frame (uint8 type, const uint8 *payload, uint16 plen)
{
    uint8  buf[128];
    uint16 n = 0;

    /* 模块忙(INT 低)直接放弃本帧: 遥测可丢, 主循环不能被 wait_idle 拖住(OTHER_TIME_OUT 保持 1000) */
    if (0 == gpio_get_level(WIFI_SPI_INT_PIN))
    {
        return 0;
    }

    buf[n ++] = MOTOR_MAGIC_UP_0;
    buf[n ++] = MOTOR_MAGIC_UP_1;
    buf[n ++] = type;
    n = motor_put_u16(buf, n, plen);

    if ((plen > 0) && (NULL != payload))
    {
        if ((uint16)(n + plen) > (uint16)sizeof(buf))       /* 保护：绝不越界写栈 */
        {
            return 0u;
        }
        memcpy(buf + n, payload, plen);
        n = (uint16)(n + plen);
    }

    n = motor_put_u16(buf, n, motor_crc16(buf + 2, (uint16)(3 + plen)));

    buf[n ++] = 0x00;
    buf[n ++] = 0x00;
    buf[n ++] = 0x80;
    buf[n ++] = 0x7F;

    /* wifi_spi_send_buffer() 返回剩余未发字节数, 非 0 = 一个字节都没发, 必须判返回值 */
    if (0u != wifi_spi_send_buffer(buf, n))
    {
        return 0u;
    }

    return 1;
}

static void motor_send_ack (uint8 cmd, uint16 token)
{
    uint8 pl[4];
    motor_put_u16(pl, 0, token);
    pl[2] = cmd;
    pl[3] = 0;
    motor_send_frame(MOTOR_TYPE_ACK, pl, 4);
}

/* PING 的 ACK 共 12 B: 前 4 B 为 token(2)+cmd(1)+0(1), 后 8 B 为版本指纹
 *   ver(2) | 参数表 CRC(2) | 通道布局 CRC(2) | 参数行数(2); 上位机按 LEN>=12 才读 */
static void motor_send_ack_ver (uint8 cmd, uint16 token)
{
    uint8 pl[12];

    motor_put_u16(pl, 0, token);
    pl[2] = cmd;
    pl[3] = 0;
    motor_put_u16(pl,  4, proto_ver());
    motor_put_u16(pl,  6, proto_param_table_crc());
    motor_put_u16(pl,  8, proto_ch_layout_crc());
    motor_put_u16(pl, 10, proto_param_count());

    motor_send_frame(MOTOR_TYPE_ACK, pl, 12);
}

static void motor_send_nak (uint8 cmd, uint16 token, uint8 err)
{
    uint8 pl[4];
    motor_put_u16(pl, 0, token);
    pl[2] = cmd;
    pl[3] = err;
    motor_send_frame(MOTOR_TYPE_NAK, pl, 4);
}

static uint8 motor_send_param_value (uint8 id, float v)
{
    uint8 pl[5];
    uint16 n = 0;
    pl[n ++] = id;
    n = motor_put_f32(pl, n, v);
    return motor_send_frame(MOTOR_TYPE_PARAM_VALUE, pl, 5);
}

/* 参数表逐条上报: ID(1)+LO(4)+HI(4)+DEF(4)+STEP(4)+UNIT(8)+NAME(16)+LABEL(32) = 73 */
/* ---- 参数表: 分批发送 ---- */
/* 每 SPD_PARAM_TX_GAP_MS 发一帧, 发不出去下一轮重试同一个 id; 连发会在模块忙时丢帧,
 * 上位机拿到混血表, 两边 ID 含义不同就会写错参数 */
#define SPD_PARAM_TX_GAP_MS     (5u)

static uint8   s_pt_pend = 0;           /* 1 = 有一张参数表等着发 */
static uint8   s_pt_id   = 0;           /* 下一个要发的参数下标   */
static uint32  s_pt_ms   = 0;           /* 上次尝试发送的时刻     */

static void motor_send_param_table (void)
{
    s_pt_id   = 0;
    s_pt_pend = 1;                      /* 只起序列, 真正发送交给 motor_link_task */
}

/* 参数表分批发送：由 motor_link_task() 每圈调用（自带 5ms 限频） */
static void motor_param_table_tick (void)
{
    const proto_param_t *p;
    uint8  pl[73];
    uint16 n;

    if (0 == s_pt_pend)
    {
        return;
    }
    if ((uint32)(motor_tick_ms - s_pt_ms) < SPD_PARAM_TX_GAP_MS)
    {
        return;
    }
    s_pt_ms = motor_tick_ms;

    /* 元数据从协议层取（唯一源）。发到表尾/表长变化时自动收尾，不会越界。 */
    p = proto_param_at(s_pt_id);
    if (NULL == p)
    {
        s_pt_pend = 0;
        return;
    }

    memset(pl, 0, sizeof(pl));
    n = 0;
    pl[n ++] = p->id;
    n = motor_put_f32(pl, n, p->lo);
    n = motor_put_f32(pl, n, p->hi);
    n = motor_put_f32(pl, n, p->def);
    n = motor_put_f32(pl, n, p->step);

    strncpy((char *)(pl + 17), p->unit,  7);
    strncpy((char *)(pl + 25), p->name, 15);
    strncpy((char *)(pl + 41), p->label, 31);

    if (0 == motor_send_frame(MOTOR_TYPE_PARAM_META, pl, 73))
    {
        return;                         /* 模块忙 -> 本项没发出去，下一轮重试同一个 id */
    }

    /* 紧接着报车端当前值: META 只带 lo/hi/def/step; 值没发出去就不推进 id */
    if (0 == motor_send_param_value(p->id, *(p->ptr)))
    {
        return;
    }

    s_pt_id ++;
    if (s_pt_id >= (uint8)proto_param_count())
    {
        s_pt_pend = 0;                  /* 整张表发完 */
    }
}

static void motor_send_event (const char *text)
{
    uint8 pl[64];
    uint8 len;

    pl[0] = 1;                                  /* level = 1(信息) */
    len = (uint8)strlen(text);
    if (len > 62) len = 62;
    memcpy(pl + 1, text, len);
    motor_send_frame(MOTOR_TYPE_EVENT, pl, (uint16)(1 + len));
}

/* ---- 六、遥测: 组帧 + 发送 ---- */
static void motor_send_telemetry (void)
{
    uint16 n = 0;
    uint16 pl;
    uint8  i;

    pl = (uint16)(10 + s_ch_count * 4);         /* SEQ+TS+MASK+CNT+DATA */

    n = 0;
    motor_txbuf[n ++] = MOTOR_MAGIC_UP_0;
    motor_txbuf[n ++] = MOTOR_MAGIC_UP_1;
    motor_txbuf[n ++] = MOTOR_TYPE_TELEMETRY;
    n = motor_put_u16(motor_txbuf, n, pl);
    n = motor_put_u16(motor_txbuf, n, motor_tx_seq ++);
    n = motor_put_u32(motor_txbuf, n, motor_tick_ms);
    /* VALID_MASK: 0xFFFF = 全部有效(哨兵值, 非位图); 16 位表达不了逐通道有效性 */
    n = motor_put_u16(motor_txbuf, n, 0xFFFF);
    n = motor_put_u16(motor_txbuf, n, s_ch_count);

    for (i = 0; i < (uint8)s_ch_count; i ++)
    {
        n = motor_put_f32(motor_txbuf, n, motor_ch[i]);     /* 直接拷内存, 无格式化 */
    }

    n = motor_put_u16(motor_txbuf, n, motor_crc16(motor_txbuf + 2, (uint16)(3 + pl)));

    motor_txbuf[n ++] = 0x00;
    motor_txbuf[n ++] = 0x00;
    motor_txbuf[n ++] = 0x80;
    motor_txbuf[n ++] = 0x7F;

    if (0 == gpio_get_level(WIFI_SPI_INT_PIN))              /* 模块忙 → 本帧不发 */
    {
        return;
    }

    if (0 == wifi_spi_send_buffer(motor_txbuf, n))
    {
        motor_tx_count ++;
    }
}

/* ---- 六b、路表 dump(0x08 -> RouteHead/RouteData/RouteEnd): 一次 poll 一帧, 帧间隔 MOTOR_RD_GAP_MS ----
 * 数据源是 RAM 里的当前表(ins_route_get_points / ins_route_pack_records), 不读 Flash;
 * 发不出去不推进索引, 下一拍重试, 发完结束帧回空闲 */
#define MOTOR_RD_IDLE    (0u)
#define MOTOR_RD_HEAD    (1u)
#define MOTOR_RD_DATA    (2u)
#define MOTOR_RD_END     (3u)

static uint8  s_rd_stage = MOTOR_RD_IDLE;
static uint16 s_rd_next  = 0u;              /* 下一条要发的记录号 */
static uint32 s_rd_ms    = 0u;              /* 上一帧的时刻（motor_tick_ms）*/
static uint16 s_rd_try   = 0u;              /* 当前阶段连续发不出去的次数 */
static uint16 s_rd_frames= 0u;              /* 本次 dump 已成功发出的数据帧数（只用于自证）*/

/* 32 B 表头: 手写字段偏移, 不用 memcpy(rt_head_t 的 crc_r/crc_h 落在 32 B 之后) */
static uint8 motor_rd_send_head (void)
{
    const ins_route_head_t *h = ins_route_get_head();
    uint8  b[32];
    uint16 n = 0u;

    n = motor_put_u32(b, n, 0x4B525443u);                                   /* CTRK */
    n = motor_put_u16(b, n, 2u);                                            /* ver */
    n = motor_put_u16(b, n, (uint16)INS_RT_REC_SIZE);                       /* rec_len */
    n = motor_put_u32(b, n, (uint32)h->count);
    n = motor_put_u32(b, n, (uint32)(ins_route_ds * 1000.0f + 0.5f));        /* ds_mm */
    n = motor_put_u32(b, n, (uint32)(h->total_dist * 1000.0f + 0.5f));      /* lap_mm */
    n = motor_put_u32(b, n, (uint32)(int32)(h->origin_x * 1000.0f));
    n = motor_put_u32(b, n, (uint32)(int32)(h->origin_y * 1000.0f));
    n = motor_put_u16(b, n, (uint16)(int16)(h->origin_yaw * 100.0f));
    n = motor_put_u16(b, n, (uint16)((0u != (ins_route_flags() & INS_ROUTE_ST_FULL)) ? 1u : 0u));

    if (0u == motor_send_frame(MOTOR_FT_ROUTE_HEAD, b, n))
    {
        return 0u;                      /* 没发出去 ⇒ 别推进，下一拍重试 */
    }
    return 1u;
}

/* 返回 1 = 这一帧发出去了（可推进）；0 = 模块忙，下一拍重试 */
static uint8 motor_rd_send_data (void)
{
    uint8  b[6u + (MOTOR_RD_CHUNK * INS_RT_REC_SIZE)];
    uint16 cnt  = ins_route_get_count();
    uint16 left = (cnt > s_rd_next) ? (uint16)(cnt - s_rd_next) : 0u;
    uint16 n = 0u;
    uint16 m;

    if (left > (uint16)MOTOR_RD_CHUNK) { left = (uint16)MOTOR_RD_CHUNK; }
    if (0u == left) { return 1u; }                     /* 没数据了，进结束帧 */

    n = motor_put_u32(b, n, (uint32)s_rd_next);
    n = motor_put_u16(b, n, left);

    m = ins_route_pack_records(s_rd_next, b + n, left);
    if (m != left) { return 1u; }                      /* 保护：别在这里死循环 */

    n = (uint16)(n + (uint16)(m * INS_RT_REC_SIZE));

    if (0u == motor_send_frame(MOTOR_FT_ROUTE_DATA, b, n)) { return 0u; }
    s_rd_next = (uint16)(s_rd_next + left);
    return 1u;
}

static uint8 motor_rd_send_end (void)
{
    uint8  b[8];
    uint16 n = 0u;

    n = motor_put_u32(b, n, (uint32)ins_route_get_count());
    n = motor_put_u32(b, n, (uint32)(ins_route_get_head()->total_dist * 1000.0f + 0.5f));
    if (0u == motor_send_frame(MOTOR_FT_ROUTE_END, b, n))
    {
        return 0u;
    }
    return 1u;
}

/* 主循环每圈调一次（幂等；空闲时立刻返回）—— 由 motor_link_task() 调 */
void motor_route_dump_poll (void)
{
    if (MOTOR_RD_IDLE == s_rd_stage) { return; }
    if ((uint32)(motor_tick_ms - s_rd_ms) < (uint32)MOTOR_RD_GAP_MS) { return; }

    switch (s_rd_stage)
    {
    case MOTOR_RD_HEAD:
        if (0u != motor_rd_send_head())
        {
            s_rd_stage = MOTOR_RD_DATA;
            s_rd_ms    = motor_tick_ms;
            s_rd_try   = 0u;
#if MOTOR_RD_DEBUG
            printf("[RD] head ok: %u pts / %u mm\r\n",
                   (unsigned)ins_route_get_count(),
                   (unsigned)(ins_route_get_head()->total_dist * 1000.0f + 0.5f));
#endif
        }
        else if (++ s_rd_try >= (uint16)MOTOR_RD_TRY_MAX)
        {
            s_rd_stage = MOTOR_RD_IDLE;
#if MOTOR_RD_DEBUG
            printf("[RD] head FAILED %u tries (module busy)\r\n", (unsigned)s_rd_try);
#endif
        }
        break;

    case MOTOR_RD_DATA:
        if (0u != motor_rd_send_data())
        {
            s_rd_ms = motor_tick_ms;
            s_rd_try = 0u;
            s_rd_frames ++;
            if (s_rd_next >= ins_route_get_count()) { s_rd_stage = MOTOR_RD_END; }
        }
        else if (++ s_rd_try >= (uint16)MOTOR_RD_TRY_MAX)
        {
            s_rd_stage = MOTOR_RD_IDLE;
#if MOTOR_RD_DEBUG
            printf("[RD] data stalled at #%u (%u frames sent)\r\n",
                   (unsigned)s_rd_next, (unsigned)s_rd_frames);
#endif
        }
        break;

    case MOTOR_RD_END:
        if (0u != motor_rd_send_end())
        {
            s_rd_stage = MOTOR_RD_IDLE;
            s_rd_ms    = motor_tick_ms;
            s_rd_try   = 0u;
#if MOTOR_RD_DEBUG
            printf("[RD] done: %u frames\r\n", (unsigned)s_rd_frames);
#endif
        }
        else if (++ s_rd_try >= (uint16)MOTOR_RD_TRY_MAX)
        {
            s_rd_stage = MOTOR_RD_IDLE;
#if MOTOR_RD_DEBUG
            printf("[RD] end FAILED (%u frames sent)\r\n", (unsigned)s_rd_frames);
#endif
        }
        break;

    default:
        s_rd_stage = MOTOR_RD_IDLE;
        break;
    }
}

/* ---- 七、下行命令解析 ----
 * 只把二进制命令翻译成协议层命令字交给 proto_exec(): SET / GET / SAVE / LOAD_DEF / ESTOP / DRV_REQ / DRV_REL
 * 权限、量程钳位、交叉钳位、驱动令牌都在 proto.c; 这里只决定回 ACK 还是 NAK */
static uint8 motor_proto (uint8 op, uint8 id, float val, float *out)
{
    proto_msg_t m;

    m.src   = PROTO_SRC_SPI;
    m.op    = op;
    m.id    = id;
    m.val   = val;
    m.token = 0u;

    return proto_exec(&m, out);
}

static void motor_dispatch (uint8 cmd, uint8 pid, uint32 raw, float val, uint16 token)
{
    uint8 i;
    float v;

    (void)raw;

    motor_rx_last_ms = motor_tick_ms;
    motor_ever_rx    = 1;

    /* 续租: 已持有驱动权时任何有效下行帧都算人在; 未持有时不领取, 免锁死屏幕驱动权 */
    if (0u != proto_drv_held_by(PROTO_SRC_SPI))
    {
        proto_drv_renew(PROTO_SRC_SPI);
    }

    switch (cmd)
    {
    case MOTOR_CMD_PING:
        /* 带版本/CRC 的 ACK: 上位机连上即可发现固件与上位机不是同一套 */
        motor_send_ack_ver(cmd, token);
        break;

    case MOTOR_CMD_GET_PARAM:
        v = 0.0f;
        if (PROTO_OK != motor_proto(PROTO_OP_GET, pid, 0.0f, &v))
        {
            motor_send_nak(cmd, token, PROTO_ERR_ID);
            break;
        }
        if (NULL != proto_param_by_id(pid)) { motor_send_param_value(pid, v); }
        motor_send_ack(cmd, token);
        break;

    case MOTOR_CMD_SET_PARAM:
    {
        const proto_param_t *p = proto_param_by_id(pid);
        float db0, brk0, iz0;

        if (NULL == p)
        {
            motor_send_nak(cmd, token, PROTO_ERR_ID);       /* 1：参数编号越界 */
            break;
        }

        db0  = steer.deadband_deg;
        brk0 = steer.brk_deg;
        iz0  = steer.izone_deg;
        v    = 0.0f;

        /* 唯一写入点: 钳位/权限/交叉钳位都在 proto_exec, 非 0 即拒绝(3=权限, 4=令牌在别人手里) */
        i = motor_proto(PROTO_OP_SET, pid, val, &v);
        if (PROTO_OK != i)
        {
            motor_send_nak(cmd, token, i);
            break;
        }

        motor_send_param_value(pid, v);                     /* 回读，上位机据此确认 */

        /* 转向三个门限有大小关系, 逐个拖滑条会拖成非法组合:
             db > brk -> 滞环带为空; db >= izone -> 积分区为空; db = 0 -> |err| 永远 > db
           钳位只在写入处做(steer_ctrl_sanitize_thr), 被连带改掉的项必须回读 */
        if ((steer.deadband_deg != db0) || (steer.brk_deg != brk0) || (steer.izone_deg != iz0))
        {
            uint16 i_db  = proto_param_index_of_ptr(&steer.deadband_deg);
            uint16 i_brk = proto_param_index_of_ptr(&steer.brk_deg);
            uint16 i_iz  = proto_param_index_of_ptr(&steer.izone_deg);

            if (0xFFFFu != i_db)
            {
                const proto_param_t *q = proto_param_at(i_db);
                motor_send_param_value(q->id, *(q->ptr));
            }
            if (0xFFFFu != i_brk)
            {
                const proto_param_t *q = proto_param_at(i_brk);
                motor_send_param_value(q->id, *(q->ptr));
            }
            if (0xFFFFu != i_iz)
            {
                const proto_param_t *q = proto_param_at(i_iz);
                motor_send_param_value(q->id, *(q->ptr));
            }
        }

        motor_send_ack(cmd, token);
        break;
    }

    case MOTOR_CMD_SET_ENABLE:
        if (!(val >= 0.0f && val <= 65535.0f))
        {
            motor_send_nak(cmd, token, PROTO_ERR_VAL);
            break;
        }
        /* VALUE 按数值解释(6.0f -> 6); 不能用 (uint16)raw, raw 是 IEEE754 位模式 */
        /* 使能位也要先领驱动令牌, 拿不到回 busy; 否则遥控持令牌时一条命令就打开两道使能门 */
        if (0u == proto_drv_acquire(PROTO_SRC_SPI))
        {
            motor_send_nak(cmd, token, PROTO_ERR_BUSY);
            break;
        }
        motor_enable = (uint16)(val + 0.5f);
        motor_send_ack(cmd, token);
        break;

    case MOTOR_CMD_GET_TABLE:
        motor_send_param_table();
        motor_send_ack(cmd, token);
        break;

    case MOTOR_CMD_ROUTE_DUMP:
        /* 先 ACK 再分帧发; count==0 也发 head/end, 让上位机拿到 0 点而非超时
           VALUE = 起始索引(0 = 从头), 越界/负值/NaN 一律当 0; head 里报总点数 */
        {
            uint16 from = 0u;
            uint16 cnt  = ins_route_get_count();

            if ((val >= 1.0f) && (val < 65000.0f))
            {
                from = (uint16)(val + 0.5f);
            }
            if (from > cnt) { from = cnt; }

            s_rd_next = from;
        }
        s_rd_ms     = motor_tick_ms;
        s_rd_try    = 0u;
        s_rd_frames = 0u;
        s_rd_stage  = MOTOR_RD_HEAD;
        motor_send_ack(cmd, token);
#if MOTOR_RD_DEBUG
        printf("[RD] cmd 0x08 from %u / %u pts\r\n",
               (unsigned)s_rd_next, (unsigned)ins_route_get_count());
#endif
        break;

    case MOTOR_CMD_SAVE:
        /* 落盘: P_CMD 类参数一律写默认值, 不会把使能/开环状态存进去 */
        v = 0.0f;
        i = motor_proto(PROTO_OP_SAVE, 0u, 0.0f, &v);
        if (PROTO_OK != i)
        {
            motor_send_nak(cmd, token, i);
            break;
        }
        motor_send_ack(cmd, token);
        break;

    case MOTOR_CMD_LOAD:
        i = motor_proto(PROTO_OP_LOAD_DEF, 0u, 0.0f, NULL);
        if (PROTO_OK != i)
        {
            motor_send_nak(cmd, token, i);
            break;
        }
        {
            uint16 k;                                   /* 回读：上位机据此确认 */

            for (k = 0u; k < proto_param_count(); k ++)
            {
                const proto_param_t *q = proto_param_at(k);
                motor_send_param_value(q->id, *(q->ptr));
            }
        }
        motor_send_ack(cmd, token);
        break;

    case MOTOR_CMD_ESTOP:
        /* 急停不检查权限, 立即生效(proto_estop 里还会剥夺驱动令牌) */
        (void)motor_proto(PROTO_OP_ESTOP, 0u, val, NULL);
        motor_send_ack(cmd, token);
        break;

    case MOTOR_CMD_DRV_REQ:
        i = motor_proto(PROTO_OP_DRV_REQ, 0u, 0.0f, NULL);
        if (PROTO_OK != i)
        {
            motor_send_nak(cmd, token, i);
            break;
        }
        motor_send_ack(cmd, token);
        break;

    case MOTOR_CMD_DRV_REL:
        (void)motor_proto(PROTO_OP_DRV_REL, 0u, 0.0f, NULL);
        motor_send_ack(cmd, token);
        break;

    default:
        motor_send_ack(cmd, token);
        break;
    }
}

static void motor_parse_rx (uint8 *buf, uint16 len)
{
    uint16 i = 0;

    /* 下行帧定长 15 字节：5A A5 | 01 | 08 00 | CMD | ID | VALUE(4) | TOKEN(2) | CRC(2) */
    while ((uint16)(i + 15) <= len)
    {
        if (buf[i] != MOTOR_MAGIC_DN_0 || buf[i + 1] != MOTOR_MAGIC_DN_1) { i ++; continue; }
        if (buf[i + 2] != MOTOR_TYPE_COMMAND)                              { i ++; continue; }
        if (motor_get_u16(buf, (uint16)(i + 3)) != 8)                      { i ++; continue; }

        /* CRC 不过只前移 1 字节: 帧头 5A A5 可能是数据里的巧合字节,
         * 跳过整帧会连带丢掉后面真正的帧 */
        if (motor_crc16(buf + i + 2, 11) != motor_get_u16(buf, (uint16)(i + 13)))
        {
            i ++;
            continue;
        }

        motor_dispatch(buf[i + 5],                                          /* CMD   */
                       buf[i + 6],                                          /* ID    */
                       motor_get_u32(buf, (uint16)(i + 7)),                 /* raw   */
                       motor_get_f32(buf, (uint16)(i + 7)),                 /* float */
                       motor_get_u16(buf, (uint16)(i + 11)));               /* TOKEN */

        i = (uint16)(i + 15);
    }
}

/* ---- 八、对外接口 ---- */
uint8 motor_link_init (void)
{
    uint8 ret;

    motor_tick_ms    = 0;
    motor_tx_ms      = 0;
    motor_rx_ms      = 0;
    motor_rx_last_ms = 0;
    motor_ever_rx    = 0;
    motor_tx_seq     = 0;
    motor_enable     = 0;
    motor_tx_count   = 0;

    /* 1) 连 WiFi 热点: 手动建连接, WIFI_SPI_AUTO_CONNECT 必须保持 0(库内函数名有误) */
    ret = wifi_spi_init(motor_ssid, motor_pwd);
    if (0 != ret)
    {
        motor_init_err = 1;
        return 1;                                   /* WiFi 连接失败 */
    }

    /* 2) 建 TCP 连接 */
    ret = wifi_spi_socket_connect(motor_sock_type, motor_host_ip, motor_host_port, motor_local_port);
    if (0 != ret)
    {
        motor_init_err = 2;
        return 2;                                   /* Socket 连接失败 */
    }

    motor_init_err = 0;

    /* 驱动权/急停事件 -> 上行 EVENT 帧, 遥控抢占时上位机靠它得知 */
    proto_set_event_sink(motor_send_event);
    proto_drv_release(PROTO_SRC_SPI);

    /* 3) 上报参数表（上位机据此自动生成滑条） */
    motor_send_param_table();
    motor_send_event("motor link ready");

    return 0;
}

void motor_link_tick_5ms (void)
{
    motor_tick_ms += 5;
}

void motor_link_task (void)
{
    uint16 n;

    if (0 != motor_tick_ms)
    {
        /*----------------------------------------- 收命令（限频）-----------------------------------------*/
        if ((uint32)(motor_tick_ms - motor_rx_ms) >= MOTOR_RX_PERIOD_MS)
        {
            motor_rx_ms = motor_tick_ms;

            /* 先看 INT: 模块忙就跳过本轮, 否则 wifi_spi_read_buffer 的 wait_idle 会卡主循环 1 秒 */
            if (0 != gpio_get_level(WIFI_SPI_INT_PIN))
            {
                n = (uint16)wifi_spi_read_buffer(motor_rxbuf, sizeof(motor_rxbuf));
                if (n > 0)
                {
                    motor_parse_rx(motor_rxbuf, n);
                }
            }
        }

        /*-------------------------------- 发遥测（限频, dump 期间压到 MOTOR_RD_TEL_MS 让出通道）-------------------------------*/
        {
            uint16 tx_period = s_tx_period_ms;
            if ((MOTOR_RD_IDLE != s_rd_stage) && (tx_period < (uint16)MOTOR_RD_TEL_MS))
            {
                tx_period = (uint16)MOTOR_RD_TEL_MS;
            }
            if ((uint32)(motor_tick_ms - motor_tx_ms) >= (uint32)tx_period)
            {
                motor_tx_ms = motor_tick_ms;
                motor_send_telemetry();
            }
        }

        /*----------------------- 参数表分批发送（每圈都调, 自带 5 ms 限频）-----------------------*/
        motor_param_table_tick();

        /*----------------------- 路表 dump（每圈都调, 自带 MOTOR_RD_GAP_MS 限频, 空闲即返回）-----------------------*/
        motor_route_dump_poll();
    }
}

uint8 motor_link_online (void)
{
    if (0 == motor_ever_rx)
    {
        return 0;
    }
    if ((uint32)(motor_tick_ms - motor_rx_last_ms) > (uint32)spd_lost_ms)
    {
        return 0;                                   /* 超时 → 认为失联 */
    }
    return 1;
}

uint16 motor_link_enable_bits (void)
{
    return motor_enable;
}

uint32 motor_link_rx_age_ms (void)
{
    if (0 == motor_ever_rx)
    {
        return 0xFFFFFFFF;
    }
    return (uint32)(motor_tick_ms - motor_rx_last_ms);
}

uint32 motor_link_tx_count (void)
{
    return motor_tx_count;
}

uint32 motor_link_now_ms (void)
{
    return (uint32)motor_tick_ms;
}

uint8 motor_link_init_err (void)
{
    return motor_init_err;
}

uint8 motor_link_ever_rx (void)
{
    return motor_ever_rx;
}

void motor_link_ch_set (uint8 idx, float v)
{
    if (idx < MOTOR_CH_COUNT_MAX)       /* 按上限判, 通道数可运行时切 */
    {
        motor_ch[idx] = v;
    }
}

/* ---- 运行时切换遥测档案(通道数) / 遥测周期 ----
 * 只改帧头 COUNT 与发包循环长度, 不补零不搬运; 切换那一两帧可能是残留值 */
void motor_link_set_ch_count (uint16 n)
{
    if (n < 1u)                    { n = 1u; }
    if (n > MOTOR_CH_COUNT_MAX)    { n = MOTOR_CH_COUNT_MAX; }

    s_ch_count = n;
}

uint16 motor_link_ch_count (void)
{
    return s_ch_count;
}

void motor_link_set_tx_period_ms (uint16 ms)
{
    if (ms < MOTOR_TX_PERIOD_MIN) { ms = MOTOR_TX_PERIOD_MIN; }
    if (ms > 1000u)               { ms = 1000u; }

    s_tx_period_ms = ms;
}

uint16 motor_link_tx_period_ms (void)
{
    return s_tx_period_ms;
}
