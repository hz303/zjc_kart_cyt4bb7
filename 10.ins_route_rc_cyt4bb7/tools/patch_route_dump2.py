# -*- coding: utf-8 -*-
"""
修 0x08 读路表这条路上三个"静默丢帧"的点（2026-10-17 晚）：

  1) motor_send_frame() 无条件 return 1 —— 而 wifi_spi_send_buffer() 的返回值是
     **剩余未发字节数**（0 才是全发出去）。所以"模块忙/超时"时帧被静默丢掉，
     调用方还以为发出去了。这是 0x08 一帧都到不了的直接原因。
  2) HEAD / END 阶段无视发送结果就推进状态机 ⇒ 表头丢一次，整次 dump 就废
     （上位机 OnRouteData 里 `if (!_rtHaveHead) return;` 把后面所有数据帧都扔掉）。
  3) dump 期间遥测仍按 5 ms / 213 B 抢 SPI 通道 ⇒ dump 帧抢不到。改成 dump 期间
     遥测 100 ms 一帧，把通道让出来。

另外把分块压到 5 条（97 B 帧 < 已验证可发的遥测帧 213 B），并加串口自证打印。
"""
import os
import sys

C = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..",
                 "steer_loop_wifi", "code", "motor_link.c")
C = os.path.abspath(C)

raw = open(C, "rb").read()
text = raw.decode("utf-8")
assert text.count("\r\n") == text.count("\n"), "行尾不统一，先人工核对"
lf = text.replace("\r\n", "\n")


def sub(old, new, cnt=1):
    global lf
    n = lf.count(old)
    if n != cnt:
        print("[ERR] 锚点命中 %d 次（期望 %d）：%r" % (n, cnt, old[:60]))
        sys.exit(2)
    lf = lf.replace(old, new, cnt)


# ---------------------------------------------------------------- 1) 宏
sub("""/* 每帧几条记录：必须让「6 B 头 + n×16 B」不超过 motor_send_frame 的 buf[128]-11 = 117 B
 *   6 条 ⇒ 102 B 载荷 / 113 B 帧。⚠ 改 7 条 = 124 B 帧，顶到缓冲边界。 */
#define MOTOR_RD_CHUNK          (6u)
#define MOTOR_RD_GAP_MS         (10u)       /* 帧间隔：上位机"半路停了"判据是 3 s 无新帧 */""",
    """/* 每帧几条记录：必须让「6 B 头 + n×16 B」不超过 motor_send_frame 的 buf[128]-11 = 117 B
 *   5 条 ⇒ 86 B 载荷 / 97 B 帧。
 *   ⚠ 2026-10-17 从 6 条降到 **5 条**：不是因为 113 B 装不下，而是要压到
 *     **比遥测帧还小**（全量 48 通道 = 213 B，那条路是现场验证过能发的）。
 *     帧越小，模块一次 SPI 事务占用通道的时间越短，抢到空闲窗口的概率越高。 */
#define MOTOR_RD_CHUNK          (5u)
#define MOTOR_RD_GAP_MS         (5u)        /* 两帧最小间隔：上位机"半路停了"判据是 3 s 无新帧 */

/* 连续多少拍发不出去就放弃（每拍 GAP_MS，200 × 5 ms ≈ 1 s）。
 *   ⚠ 没有这个上限，模块要是长期忙，状态机会一直卡在 HEAD，用户点多少次都没反应。 */
#define MOTOR_RD_TRY_MAX        (200u)

/* dump 期间把遥测周期压到这么长（原值 5 ms）。dump 期间通道要让给路表，
   否则 213 B 的遥测帧几乎占满 SPI，dump 永远抢不到。100 ms 够看曲线不"冻"。 */
#define MOTOR_RD_TEL_MS         (100u)

/* 串口自证：现场判断"命令到没到 / 发出去没"的唯一手段（WiFi 侧丢了不会报错）。
   排完问题可以改 0。 */
#define MOTOR_RD_DEBUG          (1)""")

# ---------------------------------------------------------------- 2) motor_send_frame 真实返回
sub("""    buf[n ++] = 0x00;
    buf[n ++] = 0x00;
    buf[n ++] = 0x80;
    buf[n ++] = 0x7F;

    wifi_spi_send_buffer(buf, n);

    return 1;
}""",
    """    buf[n ++] = 0x00;
    buf[n ++] = 0x00;
    buf[n ++] = 0x80;
    buf[n ++] = 0x7F;

    /* ★★ 这里必须看返回值！wifi_spi_send_buffer() 返回的是**剩余未发字节数**：
     *      0      = 全部交给模块了
     *      length = 一个字节都没发（另一上下文占用 / wait_idle 超时）
     *   以前这里写的是 `wifi_spi_send_buffer(buf, n); return 1;` ——
     *   模块忙时帧被静默丢掉，调用方却以为成功了 ⇒ 0x08 的表头丢一次，
     *   整次 dump 就废（上位机把所有数据帧按"无表头"扔掉），而且没有任何报错。 */
    if (0u != wifi_spi_send_buffer(buf, n))
    {
        return 0u;
    }

    return 1;
}""")

# 长度保护：buf[128]
sub("""    if ((plen > 0) && (NULL != payload))
    {
        memcpy(buf + n, payload, plen);
        n = (uint16)(n + plen);
    }""",
    """    if ((plen > 0) && (NULL != payload))
    {
        if ((uint16)(n + plen) > (uint16)sizeof(buf))       /* 保护：绝不越界写栈 */
        {
            return 0u;
        }
        memcpy(buf + n, payload, plen);
        n = (uint16)(n + plen);
    }""")

# ---------------------------------------------------------------- 3) head / end 返回真实结果
sub("""    (void)motor_send_frame(MOTOR_FT_ROUTE_HEAD, b, n);
}""",
    """    if (0u == motor_send_frame(MOTOR_FT_ROUTE_HEAD, b, n))
    {
        return 0u;                      /* 没发出去 ⇒ 别推进，下一拍重试 */
    }
    return 1u;
}""")

sub("""static void motor_rd_send_head (void)
{""",
    """static uint8 motor_rd_send_head (void)
{""")

sub("""    n = motor_put_u32(b, n, (uint32)ins_route_get_count());
    n = motor_put_u32(b, n, (uint32)(ins_route_get_head()->total_dist * 1000.0f + 0.5f));
    (void)motor_send_frame(MOTOR_FT_ROUTE_END, b, n);
}""",
    """    n = motor_put_u32(b, n, (uint32)ins_route_get_count());
    n = motor_put_u32(b, n, (uint32)(ins_route_get_head()->total_dist * 1000.0f + 0.5f));
    if (0u == motor_send_frame(MOTOR_FT_ROUTE_END, b, n))
    {
        return 0u;
    }
    return 1u;
}""")

sub("""static void motor_rd_send_end (void)
{""",
    """static uint8 motor_rd_send_end (void)
{""")

# ---------------------------------------------------------------- 4) 状态机：失败不推进 + 重试上限
sub("""static uint8  s_rd_stage = MOTOR_RD_IDLE;
static uint16 s_rd_next  = 0u;              /* 下一条要发的记录号 */
static uint32 s_rd_ms    = 0u;              /* 上一帧的时刻（motor_tick_ms）*/""",
    """static uint8  s_rd_stage = MOTOR_RD_IDLE;
static uint16 s_rd_next  = 0u;              /* 下一条要发的记录号 */
static uint32 s_rd_ms    = 0u;              /* 上一帧的时刻（motor_tick_ms）*/
static uint16 s_rd_try   = 0u;              /* 当前阶段连续发不出去的次数 */
static uint16 s_rd_frames= 0u;              /* 本次 dump 已成功发出的数据帧数（只用于自证）*/""")

sub("""    case MOTOR_RD_HEAD:
        motor_rd_send_head();
        s_rd_stage = MOTOR_RD_DATA;
        s_rd_ms    = motor_tick_ms;
        break;

    case MOTOR_RD_DATA:
        if (0u != motor_rd_send_data())
        {
            s_rd_ms = motor_tick_ms;
            if (s_rd_next >= ins_route_get_count()) { s_rd_stage = MOTOR_RD_END; }
        }
        break;

    case MOTOR_RD_END:
        motor_rd_send_end();
        s_rd_stage = MOTOR_RD_IDLE;
        s_rd_ms    = motor_tick_ms;
        break;""",
    """    case MOTOR_RD_HEAD:
        if (0u != motor_rd_send_head())
        {
            s_rd_stage = MOTOR_RD_DATA;
            s_rd_ms    = motor_tick_ms;
            s_rd_try   = 0u;
#if MOTOR_RD_DEBUG
            printf("[RD] head ok: %u pts / %u mm\\r\\n",
                   (unsigned)ins_route_get_count(),
                   (unsigned)(ins_route_get_head()->total_dist * 1000.0f + 0.5f));
#endif
        }
        else if (++ s_rd_try >= (uint16)MOTOR_RD_TRY_MAX)
        {
            s_rd_stage = MOTOR_RD_IDLE;
#if MOTOR_RD_DEBUG
            printf("[RD] head FAILED %u tries (module busy)\\r\\n", (unsigned)s_rd_try);
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
            printf("[RD] data stalled at #%u (%u frames sent)\\r\\n",
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
            printf("[RD] done: %u frames\\r\\n", (unsigned)s_rd_frames);
#endif
        }
        else if (++ s_rd_try >= (uint16)MOTOR_RD_TRY_MAX)
        {
            s_rd_stage = MOTOR_RD_IDLE;
#if MOTOR_RD_DEBUG
            printf("[RD] end FAILED (%u frames sent)\\r\\n", (unsigned)s_rd_frames);
#endif
        }
        break;""")

# ---------------------------------------------------------------- 5) 0x08 分支：清计数 + 自证
sub("""        s_rd_next  = 0u;
        s_rd_ms    = motor_tick_ms;
        s_rd_stage = MOTOR_RD_HEAD;
        motor_send_ack(cmd, token);
        break;""",
    """        s_rd_next   = 0u;
        s_rd_ms     = motor_tick_ms;
        s_rd_try    = 0u;
        s_rd_frames = 0u;
        s_rd_stage  = MOTOR_RD_HEAD;
        motor_send_ack(cmd, token);
#if MOTOR_RD_DEBUG
        printf("[RD] cmd 0x08: %u pts\\r\\n", (unsigned)ins_route_get_count());
#endif
        break;""")

# ---------------------------------------------------------------- 6) dump 期间遥测降频
sub("""        /*----------------------------------------- 发遥测（限频）-----------------------------------------*/
        if ((uint32)(motor_tick_ms - motor_tx_ms) >= s_tx_period_ms)
        {
            motor_tx_ms = motor_tick_ms;
            motor_send_telemetry();
        }""",
    """        /*----------------------------------------- 发遥测（限频）-----------------------------------------
           ★ dump 期间把周期压到 MOTOR_RD_TEL_MS：全量 48 通道是 213 B/帧，
             5 ms 一帧几乎占满 WiFi SPI 通道，dump 帧会一直抢不到（现场症状就是
             "点读路表，一帧都回不来"）。让出 95% 通道，几秒内传完就恢复正常。 */
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
        }""")

out = lf.replace("\n", "\r\n")
assert out.count("\r\n") == out.count("\n")
open(C, "wb").write(out.encode("utf-8"))
print("[OK] patched", C)
