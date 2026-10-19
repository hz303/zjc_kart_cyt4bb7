#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
patch_route_dump.py —— 给工程 10 的车端补上「读路表」
    Cmd 0x08  ->  RouteHead 0x10 (32 B 表头) / RouteData 0x11 (first+n+记录) / RouteEnd 0x12

为什么需要
  上位机「读路表」发的是 `Cmd.RouteDump = 0x08`，而车端 `motor_link.c` 的分派只做到 0x07
  ⇒ 车端走 default 回一个 ACK 就完事，**一帧数据都不发**。
  现象：上位机等 3 s 后报「已收 0/-1 点，0 个数据帧」。

  ⚠ 这个洞是在记录格式 12 B → 16 B 之后才暴露的：以前没人真车点过"读路表"，
    自检跑的是上位机内置模拟器（Simulator 实现了 0x08）⇒ "自检全过、真车没数据"。

关键约束（改之前先看）
  `motor_send_frame()` 的发送缓冲是 **uint8 buf[128]**，帧头 11 B + 载荷 ⇒ 载荷 ≤ 117 B。
  一条记录 16 B ⇒ 每帧最多 6 条（6×16+6 = 102 B 载荷，帧 113 B）。
  ⚠ 不要为了少发几帧去放大 buf：帧尺寸是踩过坑的（模块会退回大帧）。

⚠ motor_link.c 是 **UTF-8 + CRLF**：只能按脚本改。本脚本用 chr(13)/chr(10) 处理行尾，
  避免转义字符串在多层引号里被吃掉。
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
C = os.path.join(HERE, "..", "steer_loop_wifi", "code", "motor_link.c")

CRLF = chr(13) + chr(10)
LF = chr(10)

MACRO_OLD = '#define MOTOR_CMD_GET_TABLE     (0x07)'

MACRO_NEW = """#define MOTOR_CMD_GET_TABLE     (0x07)
#define MOTOR_CMD_ROUTE_DUMP    (0x08)      /* ★ 读路表：车端分帧把 RAM 里的当前表推上来 */

/* 路表 dump 的上行帧类型（与上位机 FrameType.RouteHead/Data/End 逐字节对应）
 *   RouteHead 0x10 : 32 B 表头（= rt_head_t 的前 32 B，**不含两个 CRC**）
 *   RouteData 0x11 : first(4) + n(2) + n × INS_RT_REC_SIZE(16) B
 *   RouteEnd  0x12 : count(4) + lap_mm(4) */
#define MOTOR_FT_ROUTE_HEAD     (0x10)
#define MOTOR_FT_ROUTE_DATA     (0x11)
#define MOTOR_FT_ROUTE_END      (0x12)

/* 每帧几条记录：必须让「6 B 头 + n×16 B」不超过 motor_send_frame 的 buf[128]-11 = 117 B
 *   6 条 ⇒ 102 B 载荷 / 113 B 帧。⚠ 改 7 条 = 124 B 帧，顶到缓冲边界。 */
#define MOTOR_RD_CHUNK          (6u)
#define MOTOR_RD_GAP_MS         (10u)       /* 帧间隔：上位机"半路停了"判据是 3 s 无新帧 */"""

FUNC_ANCHOR = """/*===================================================================================================================
 *  七、下行命令解析"""

FUNC_NEW = """/*===================================================================================================================
 *  六b、路表 dump（0x08 → RouteHead/RouteData/RouteEnd）
 *
 *  设计要点：
 *    ① **分步发**：一次 poll 只发一帧、帧间隔 10 ms ⇒ 主循环不被拖住
 *       （spd_safety 的 200 ms 心跳门靠主循环转）。
 *    ② **数据源是 RAM 里的当前表**（ins_route_get_points / ins_route_pack_records），
 *       不读 Flash —— 读 Flash 是阻塞的，而上位机要的本来就是"你现在正在跑的那张"。
 *    ③ **发不出去就重试**：motor_send_frame() 在模块忙时返回 0，这时**不推进索引**，
 *       否则上位机会缺帧（它自己会补，但每次要白等 3 s 超时）。
 *    ④ 发完结束帧自动回空闲 ⇒ 再点一次「读路表」就是重新来一遍。
 *=================================================================================================================*/
#define MOTOR_RD_IDLE    (0u)
#define MOTOR_RD_HEAD    (1u)
#define MOTOR_RD_DATA    (2u)
#define MOTOR_RD_END     (3u)

static uint8  s_rd_stage = MOTOR_RD_IDLE;
static uint16 s_rd_next  = 0u;              /* 下一条要发的记录号 */
static uint32 s_rd_ms    = 0u;              /* 上一帧的时刻（motor_tick_ms）*/

/* 32 B 表头：字段顺序与 rt_head_t 的前 32 B 一致，但**故意手写偏移**而不是 memcpy ——
   rt_head_t 里 crc_r/crc_h 落在 32 B 之后，memcpy 会把它们一起带上来。 */
static void motor_rd_send_head (void)
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

    (void)motor_send_frame(MOTOR_FT_ROUTE_HEAD, b, n);
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

static void motor_rd_send_end (void)
{
    uint8  b[8];
    uint16 n = 0u;

    n = motor_put_u32(b, n, (uint32)ins_route_get_count());
    n = motor_put_u32(b, n, (uint32)(ins_route_get_head()->total_dist * 1000.0f + 0.5f));
    (void)motor_send_frame(MOTOR_FT_ROUTE_END, b, n);
}

/* 主循环每圈调一次（幂等；空闲时立刻返回）—— 由 motor_link_task() 调 */
void motor_route_dump_poll (void)
{
    if (MOTOR_RD_IDLE == s_rd_stage) { return; }
    if ((uint32)(motor_tick_ms - s_rd_ms) < (uint32)MOTOR_RD_GAP_MS) { return; }

    switch (s_rd_stage)
    {
    case MOTOR_RD_HEAD:
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
        break;

    default:
        s_rd_stage = MOTOR_RD_IDLE;
        break;
    }
}

/*===================================================================================================================
 *  七、下行命令解析"""

CASE_OLD = """    case MOTOR_CMD_GET_TABLE:
        motor_send_param_table();
        motor_send_ack(cmd, token);
        break;"""

CASE_NEW = """    case MOTOR_CMD_GET_TABLE:
        motor_send_param_table();
        motor_send_ack(cmd, token);
        break;

    case MOTOR_CMD_ROUTE_DUMP:
        /* ★ 先 ACK 再分帧发：上位机看到 ACK 就知道"命令到了"，
           之后每 10 ms 一帧（见 motor_route_dump_poll）。
           表空（count==0）也照样发 head/end —— 让上位机拿到"0 点"而不是超时。 */
        s_rd_next  = 0u;
        s_rd_ms    = motor_tick_ms;
        s_rd_stage = MOTOR_RD_HEAD;
        motor_send_ack(cmd, token);
        break;"""


def main():
    if not os.path.exists(C):
        print("[ERR] 找不到 %s" % C)
        return 2

    text = open(C, "rb").read().decode("utf-8")

    if "MOTOR_CMD_ROUTE_DUMP" in text:
        print("[SKIP] 已经补过了")
        return 0

    if text.count(CRLF) != text.count(LF):
        print("[ERR] 行尾混用（CRLF %d / LF %d），先人工核对"
              % (text.count(CRLF), text.count(LF)))
        return 2

    text = text.replace(CRLF, LF)          # 统一成 LF 做匹配

    for name, old in (("宏", MACRO_OLD), ("函数块", FUNC_ANCHOR), ("命令分派", CASE_OLD)):
        c = text.count(old)
        if c != 1:
            print("[ERR] 锚点『%s』出现 %d 次（要求 1 次）" % (name, c))
            return 2

    text = text.replace(MACRO_OLD, MACRO_NEW, 1)
    text = text.replace(FUNC_ANCHOR, FUNC_NEW, 1)
    text = text.replace(CASE_OLD, CASE_NEW, 1)

    open(C, "wb").write(text.replace(LF, CRLF).encode("utf-8"))
    print("[OK] motor_link.c：已加 0x08 读路表（每帧 6 条 / 间隔 10 ms / 帧上限 113 B）")
    print("     下一步：把 motor_route_dump_poll() 挂进 motor_link_task()")
    return 0


if __name__ == "__main__":
    sys.exit(main())
