/*********************************************************************************************************************
 * 双电机速度环(WiFi) - WiFi-SPI 链路实现
 * 详细协议说明见 motor_link.h
 *
 * 本文件由工作区中已经跑通过的 kart_link.c 裁剪而来：协议、帧格式、解析逻辑逐字节保持一致，
 * 只是把遥测通道的取值改成由 main 写入（motor_link_ch_set），
 * 并去掉了对其它工程变量的依赖，因此可以独立编译。
 ********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "motor_link.h"

/*===================================================================================================================
 *  一、参数变量
 *
 *  本工程把这些变量的**定义**放回各自的模块里(见 spd_enc.c / spd_ctrl.c /
 *  spd_safety.c / motor_drv.c), 本文件只通过 motor_link.h 的参数表取它们的地址。
 *  这样"参数属于谁、默认值是多少"只有一处, 不会再出现两边默认值不一致的隐患。
 *=================================================================================================================*/

/*===================================================================================================================
 *  二、参数表元数据（由 motor_link.h 的 MOTOR_PARAM_TABLE 展开而来）
 *=================================================================================================================*/
typedef struct
{
    uint8       id;
    const char *name;       /* 英文名 */
    const char *label;      /* "组名|中文标签"（上位机按 | 拆分折叠） */
    float       lo;
    float       hi;
    float       def;
    const char *unit;
    float       step;
    volatile float *ptr;   /* 参数被中断和主循环共享, 必须 volatile */
} motor_param_t;

#define MOTOR_DEF_META(id, name, label, lo, hi, def, unit, step, ptr) \
    { (uint8)(id), name, label, lo, hi, def, unit, step, ptr },

static const motor_param_t motor_param_meta[] = { MOTOR_PARAM_TABLE(MOTOR_DEF_META) };
#undef MOTOR_DEF_META

#define MOTOR_PARAM_COUNT   ((uint8)(sizeof(motor_param_meta) / sizeof(motor_param_meta[0])))

/*===================================================================================================================
 *  三、协议常量与内部状态
 *=================================================================================================================*/
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

/* 发送缓冲：遥测帧 = 11 + 10 + 12*4 = 69 字节 */
static uint8            motor_txbuf[256];
/* 接收缓冲 */
static uint8            motor_rxbuf[256];
/* 遥测通道值 */
static float            motor_ch[MOTOR_CH_COUNT];

/* 库函数要 char*（不能直接传字符串字面量，否则 IAR 会报 const 转换） */
static char motor_ssid[]       = MOTOR_WIFI_SSID;
static char motor_pwd[]        = MOTOR_WIFI_PASSWORD;
static char motor_sock_type[]  = MOTOR_SOCK_TYPE;   /* 别直接写字面量：库函数签名要 char*  */
static char motor_host_ip[]    = MOTOR_HOST_IP;
static char motor_host_port[]  = MOTOR_HOST_PORT;
static char motor_local_port[] = MOTOR_LOCAL_PORT;

/*===================================================================================================================
 *  四、小工具：CRC16 与小端读写
 *=================================================================================================================*/
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

/* Cortex-M7 是小端，float 直接拷内存即可 —— 这就是"零格式化开销"的关键 */
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

/*===================================================================================================================
 *  五、上行帧发送
 *=================================================================================================================*/
/* 返回 1 = 这一帧真的交给模块了；0 = 模块忙，本帧没发出去（被跳过）。
   ★ 参数表的分批发送要靠这个返回值做“发不出去就重试”。 */
static uint8 motor_send_frame (uint8 type, const uint8 *payload, uint16 plen)
{
    uint8  buf[128];
    uint16 n = 0;

    /* ★ 模块忙就直接放弃本帧 —— 遥测可以丢，主循环不能被拖住。
     *   注意：驱动里的 OTHER_TIME_OUT 保持默认 1000（初始化要靠它），
     *   运行期就靠这个 INT 判断避免被 wait_idle 卡住。 */
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
        memcpy(buf + n, payload, plen);
        n = (uint16)(n + plen);
    }

    n = motor_put_u16(buf, n, motor_crc16(buf + 2, (uint16)(3 + plen)));

    buf[n ++] = 0x00;
    buf[n ++] = 0x00;
    buf[n ++] = 0x80;
    buf[n ++] = 0x7F;

    wifi_spi_send_buffer(buf, n);

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

/* 参数表逐条上报：ID(1)+LO(4)+HI(4)+DEF(4)+STEP(4)+UNIT(8)+NAME(16)+LABEL(32) = 73 */
/*===================================================================================================================
 *  参数表：分批发送
 *
 *  注意：这里原来是把 MOTOR_PARAM_COUNT 帧一口气连发，而 motor_send_frame()
 *  遇到“模块忙”会静默丢帧（见上面的 INT 判断）。工程 4 只有 13 项、勉强够；
 *  本工程 16 项就会漏掉几帧，上位机于是拿到一张“混血参数表”：
 *  到货的项用新名字，没到货的项还留着上位机本地的旧名字 —— 而两边的 ID 含义完全不同
 *  （上位机本地 2 号是 k_v，车端 2 号是 duty_limit）。用户照着面板拖滑条，
 *  就把值写进了完全不相干的参数里（最狠的是把占空比限幅写成 0.02%，
 *  现象是“上位机一切正常但电机不动”）。
 *
 *  改成：每 SPD_PARAM_TX_GAP_MS 发一帧，发不出去就下一轮重试同一个 id，
 *  直到全部发完。最多 SPD_PARAM_TX_GAP_MS*16 ms（默认 5ms -> 80ms），
 *  而上位机本来就是“收到第一帧后 300ms 统一重建界面”，体感没有差别。
 *=================================================================================================================*/
#define SPD_PARAM_TX_GAP_MS     (5u)

static uint8   s_pt_pend = 0;           /* 1 = 有一张参数表等着发 */
static uint8   s_pt_id   = 0;           /* 下一个要发的参数下标   */
static uint32  s_pt_ms   = 0;           /* 上次尝试发送的时刻     */

static void motor_send_param_table (void)
{
    s_pt_id   = 0;
    s_pt_pend = 1;                      /* 只“武装”序列，真正的发送交给 motor_link_task */
}

/* 参数表分批发送：由 motor_link_task() 每圈调用（自带 5ms 限频） */
static void motor_param_table_tick (void)
{
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

    memset(pl, 0, sizeof(pl));
    n = 0;
    pl[n ++] = motor_param_meta[s_pt_id].id;
    n = motor_put_f32(pl, n, motor_param_meta[s_pt_id].lo);
    n = motor_put_f32(pl, n, motor_param_meta[s_pt_id].hi);
    n = motor_put_f32(pl, n, motor_param_meta[s_pt_id].def);
    n = motor_put_f32(pl, n, motor_param_meta[s_pt_id].step);

    strncpy((char *)(pl + 17), motor_param_meta[s_pt_id].unit,  7);
    strncpy((char *)(pl + 25), motor_param_meta[s_pt_id].name, 15);
    strncpy((char *)(pl + 41), motor_param_meta[s_pt_id].label, 31);

    if (0 == motor_send_frame(MOTOR_TYPE_PARAM_META, pl, 73))
    {
        return;                         /* 模块忙 -> 本项没发出去，下一轮重试同一个 id */
    }

    /* ★★ 紧接着把【车端当前的数值】也报一份。
       没有这一步，上位机面板显示的只是它自己的本地值（ParamPanel.MergeTable 只合并
       名字/量程，**不更新数值**）—— 用户根本分不清"面板上这个 0.005 到底是车上的，
       还是我自己上次留下的"，照着改就会改错参数。META 帧里只有 lo/hi/def/step，
       没有"当前值"，所以必须补这一帧。
       ★ 值没发出去就**不推进 id**，下一轮连同 META 一起重发同一项，
         保证"面板上出现的每一项，数值都是车上的真值"。 */
    if (0 == motor_send_param_value(s_pt_id, *(motor_param_meta[s_pt_id].ptr)))
    {
        return;
    }

    s_pt_id ++;
    if (s_pt_id >= (uint8)MOTOR_PARAM_COUNT)
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

/*===================================================================================================================
 *  六、遥测：组帧 + 发送
 *=================================================================================================================*/
static void motor_send_telemetry (void)
{
    uint16 n = 0;
    uint16 pl;
    uint8  i;

    pl = (uint16)(10 + MOTOR_CH_COUNT * 4);     /* SEQ+TS+MASK+CNT+DATA */

    n = 0;
    motor_txbuf[n ++] = MOTOR_MAGIC_UP_0;
    motor_txbuf[n ++] = MOTOR_MAGIC_UP_1;
    motor_txbuf[n ++] = MOTOR_TYPE_TELEMETRY;
    n = motor_put_u16(motor_txbuf, n, pl);
    n = motor_put_u16(motor_txbuf, n, motor_tx_seq ++);
    n = motor_put_u32(motor_txbuf, n, motor_tick_ms);
    /* VALID_MASK：0xFFFF = **全部有效**（哨兵值，不是按位图）。
     * ⚠ 这个字段只有 16 位，而通道数已经 20 个 —— 按位根本表达不下第 17~20 位，
     *   所以约定"全 1 = 全部有效"。上位机侧 ChannelStore.IsValid() 也按这条解释。
     *   将来真要做逐通道有效性，必须先把字段扩成 u32（改协议）。 */
    n = motor_put_u16(motor_txbuf, n, 0xFFFF);
    n = motor_put_u16(motor_txbuf, n, MOTOR_CH_COUNT);

    for (i = 0; i < MOTOR_CH_COUNT; i ++)
    {
        n = motor_put_f32(motor_txbuf, n, motor_ch[i]);     /* ★ 直接拷内存，无格式化 */
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

/*===================================================================================================================
 *  七、下行命令解析
 *=================================================================================================================*/
static void motor_dispatch (uint8 cmd, uint8 pid, uint32 raw, float val, uint16 token)
{
    uint8 i;
    float v;

    motor_rx_last_ms = motor_tick_ms;
    motor_ever_rx    = 1;

    switch (cmd)
    {
    case MOTOR_CMD_PING:
        motor_send_ack(cmd, token);
        break;

    case MOTOR_CMD_GET_PARAM:
        if (pid < MOTOR_PARAM_COUNT)
        {
            motor_send_param_value(pid, *(motor_param_meta[pid].ptr));
        }
        motor_send_ack(cmd, token);
        break;

    case MOTOR_CMD_SET_PARAM:
        if (pid >= MOTOR_PARAM_COUNT)
        {
            motor_send_nak(cmd, token, 1);              /* 错误码 1：参数编号越界 */
            break;
        }
        v = val;
        if (v < motor_param_meta[pid].lo) v = motor_param_meta[pid].lo;     /* ★ 车端必须钳位 */
        if (v > motor_param_meta[pid].hi) v = motor_param_meta[pid].hi;
        if (motor_param_meta[pid].step > 0.0f)                              /* 按步长吸附 */
        {
            v = (float)((int32)(v / motor_param_meta[pid].step + 0.5f)) * motor_param_meta[pid].step;
            if (v < motor_param_meta[pid].lo) v = motor_param_meta[pid].lo;
            if (v > motor_param_meta[pid].hi) v = motor_param_meta[pid].hi;
        }
        *(motor_param_meta[pid].ptr) = v;
        motor_send_param_value(pid, v);                 /* 回读，上位机据此确认 */
        motor_send_ack(cmd, token);
        break;

    case MOTOR_CMD_SET_ENABLE:
        /* ★ VALUE 字段按"数值"解释（上位机发 6.0f，这里就该得到 6）。
         *   不能写成 (uint16)raw —— raw 是 IEEE754 的位模式，
         *   6.0f 的字节是 00 00 C0 40，截 16 位会变成 0x0000，
         *   后果是"使能怎么都传不下来"，而且不报错。 */
        motor_enable = (uint16)(val + 0.5f);
        motor_send_ack(cmd, token);
        break;

    case MOTOR_CMD_GET_TABLE:
        motor_send_param_table();
        motor_send_ack(cmd, token);
        break;

    case MOTOR_CMD_SAVE:
        /* 本验证工程不做参数持久化（要存需要接 flash 模块），先只回 ACK */
        motor_send_ack(cmd, token);
        break;

    case MOTOR_CMD_LOAD:
        for (i = 0; i < MOTOR_PARAM_COUNT; i ++)        /* 恢复默认值 */
        {
            *(motor_param_meta[i].ptr) = motor_param_meta[i].def;
            motor_send_param_value(i, motor_param_meta[i].def);
        }
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

        /* ★ CRC 不过只前移 1 字节：帧头 5A A5 可能是数据里的巧合字节，
         *   跳过整帧会连带丢掉后面真正的帧。 */
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

/*===================================================================================================================
 *  八、对外接口
 *=================================================================================================================*/
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

    /* 1) 连 WiFi 热点
     * ⚠ 逐飞库里 WIFI_SPI_AUTO_CONNECT 必须保持 0 —— 它内部调用的函数名写错了
     *   （.c 里写的是 wifi_spi_connect_socket，正确名是 wifi_spi_socket_connect），
     *   一旦设成 1 或 2 就会编译不过。所以这里手动建连接。 */
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

            /* ★ 先看 INT 引脚：模块忙就跳过这一轮读取。
             *   否则 wifi_spi_read_buffer 内部的 wait_idle(OTHER_TIME_OUT) 最长会把
             *   主循环卡住 1 秒 —— 少收一轮命令没关系，控制循环不能被拖住。 */
            if (0 != gpio_get_level(WIFI_SPI_INT_PIN))
            {
                n = (uint16)wifi_spi_read_buffer(motor_rxbuf, sizeof(motor_rxbuf));
                if (n > 0)
                {
                    motor_parse_rx(motor_rxbuf, n);
                }
            }
        }

        /*----------------------------------------- 发遥测（限频）-----------------------------------------*/
        if ((uint32)(motor_tick_ms - motor_tx_ms) >= MOTOR_TX_PERIOD_MS)
        {
            motor_tx_ms = motor_tick_ms;
            motor_send_telemetry();
        }

        /*----------------------- 参数表分批发送（每次最多一帧，忙就重试）-----------------------
           必须每圈都调：它自己按 5ms 限频，不能挂在上面那两个限频分支里。 */
        motor_param_table_tick();
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
    if (idx < MOTOR_CH_COUNT)
    {
        motor_ch[idx] = v;
    }
}
