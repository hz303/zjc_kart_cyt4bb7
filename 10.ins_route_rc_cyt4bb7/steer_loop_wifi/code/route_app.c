/*********************************************************************************************************************
 * 工程 10（卡丁）· 路表应用层实现（设计意图见 route_app.h）
 ********************************************************************************************************************/
#include "zf_common_headfile.h"
#include <stdio.h>
#include <string.h>

#include "ins_route.h"
#include "route_link.h"
#include "route_app.h"

/* ---- 命令量（绑定到参数表 ID 38/40/41/42/53）---- */
volatile float rt_rec_cmd  = 0.0f;
volatile float rt_slot_cmd = 0.0f;
volatile float rt_load_cmd = 0.0f;
volatile float rt_save_cmd = 0.0f;
volatile float rt_mark_cmd = 0.0f;

/* ---- 边沿记忆 ---- */
static uint8   s_rec_last   = 0u;
static uint8   s_load_last  = 0u;
static uint8   s_save_last  = 0u;
static uint8   s_mark_last  = 0u;
static uint8   s_slot_last  = 0xFFu;    /* 0xFF = 还没同步过 */

/*===================================================================================================================
 *  一、上电
 *=================================================================================================================*/
void route_app_init (void)
{
    uint32  i;
    uint8   rt_ok;

    ins_route_init();                   /* 清 RAM 缓冲 + 夹点距 */

    /* ① 先读槽索引（决定 active 槽）
       ⚠ 顺序硬约束：内建表 → 载入 active。反了会把用户的表盖掉（见 ins_route.h）。 */
    ins_route_slot_init();

    /* ② 串口灌表的链路：只清接收环形缓冲/状态，UART 本身早就由 debug_init 起来了 */
    route_link_init();

    /* ③ 内建表：**只往空槽里补**（本工程目前是空注册表，见 ins_route_builtin.c）
          ⚠ 还在 5 ms 中断打开之前 ⇒ 这里可以死等写盘完成；限圈数防挂死。 */
#if (1 == INS_ROUTE_BUILTIN_EN)
    for (i = 0u; i < ins_route_builtin_cnt; i ++)
    {
        const ins_route_builtin_t *e = &ins_route_builtin_tbl[i];
        ins_route_slot_info_t      si;

        if (0u != ins_route_slot_info(e->slot, &si))
        {
            printf("[RT] builtin '%s': slot %u 已有表 (%u 点) -> 跳过\r\n",
                   e->name, (unsigned)e->slot, (unsigned)si.count);
            continue;
        }

        if (0u == ins_route_slot_load_image(e->slot, e->img, *e->len))
        {
            uint32 guard;

            for (guard = 0u; (guard < 400u) && (0u != ins_route_flash_busy()); guard ++)
            {
                ins_route_flash_poll();
            }
        }
        (void)ins_route_slot_info(e->slot, &si);
        printf("[RT] builtin '%s' -> 槽 %u: %u 点 / %u mm\r\n",
               e->name, (unsigned)e->slot, (unsigned)si.count, (unsigned)si.lap_mm);
    }
#endif

    /* ④ 载入 active 槽（**阻塞**，所以在 5 ms 中断打开之前） */
    rt_ok = ins_route_flash_load();

    /* [!] 一律不用 %f：IAR 默认库不一定带浮点格式化（主程序里同一条纪律） */
    printf("[RT] 路表: %s  点数=%u  总里程=%u mm  点距=%u mm  槽=%u/%u\r\n",
           (0u != rt_ok) ? "已载入" : "无表/无效",
           (unsigned)ins_route_get_count(),
           (unsigned)(ins_route_get_head()->total_dist * 1000.0f + 0.5f),
           (unsigned)(ins_route_ds * 1000.0f + 0.5f),
           (unsigned)ins_route_slot_active(), (unsigned)INS_RT_SLOT_COUNT);

    /* ⑤ 命令量归零，参数里的 rt_slot 跟上真实 active 槽（界面上不会显示错） */
    rt_rec_cmd  = 0.0f;
    rt_load_cmd = 0.0f;
    rt_save_cmd = 0.0f;
    rt_mark_cmd = 0.0f;
    s_slot_last = ins_route_slot_active();
    rt_slot_cmd = (float)s_slot_last;
}

/*===================================================================================================================
 *  二、主循环
 *=================================================================================================================*/
void route_app_poll (void)
{
    uint8 now;
    uint8 slot;

    /* ---- 1) 录制开关：0→1 开始；1→0 停止并落**槽 0**（手推表永远住槽 0）---- */
    now = (rt_rec_cmd > 0.5f) ? 1u : 0u;
    if (now != s_rec_last)
    {
        s_rec_last = now;

        if (0u != now)
        {
            if (0u == ins_route_is_running())
            {
                ins_route_start();
                printf("[RT] 开始录制（点距 %u mm，最多 %u 点 = %u m）\r\n",
                       (unsigned)(ins_route_ds * 1000.0f + 0.5f),
                       (unsigned)INS_ROUTE_MAX_POINTS,
                       (unsigned)(ins_route_ds * (float)INS_ROUTE_MAX_POINTS));
            }
        }
        else if (0u != ins_route_is_running())
        {
            ins_route_stop();
            printf("[RT] 停止录制：%u 点 / %u mm / 打角 %u 点 / 关键点 %u\r\n",
                   (unsigned)ins_route_get_count(),
                   (unsigned)(ins_route_get_head()->total_dist * 1000.0f + 0.5f),
                   (unsigned)ins_route_steer_count(),
                   (unsigned)ins_route_key_count());

            if (ins_route_get_count() >= (uint16)RT_MIN_SAVE_POINTS)
            {
                ins_route_flash_save_slot(0u);      /* 分步写，不阻塞 */
                printf("[RT] 已在后台写入槽 0\r\n");
            }
            else
            {
                printf("[RT] 点数太少（<%u），不落盘\r\n", (unsigned)RT_MIN_SAVE_POINTS);
            }
        }
    }

    /* ---- 2) 选槽：改了目标槽就切 active（持久化）+ 载入 ----
       ⚠ 这一步**会覆盖 RAM 里没存盘的录制结果**，所以录制中不允许切槽。 */
    slot = (uint8)(rt_slot_cmd + 0.5f);
    if (slot >= (uint8)INS_RT_SLOT_COUNT) { slot = 0u; }

    if (slot != s_slot_last)
    {
        s_slot_last = slot;
        if (0u != ins_route_is_running())
        {
            rt_slot_cmd = (float)ins_route_slot_active();   /* 录制中：拒绝，退回显示 */
        }
        else if (0u == ins_route_flash_busy())
        {
            if (0u == ins_route_slot_set_active(slot, 1u))
            {
                (void)ins_route_flash_load_slot(slot);
                printf("[RT] 切到槽 %u：%u 点 / %u mm\r\n", (unsigned)slot,
                       (unsigned)ins_route_get_count(),
                       (unsigned)(ins_route_get_head()->total_dist * 1000.0f + 0.5f));
            }
        }
    }

    /* ---- 3) 重新载入（0→1）：从 Flash 再读一次 active 槽 ---- */
    now = (rt_load_cmd > 0.5f) ? 1u : 0u;
    if ((0u != now) && (0u == s_load_last) && (0u == ins_route_flash_busy()) &&
        (0u == ins_route_is_running()))
    {
        (void)ins_route_flash_load_slot(ins_route_slot_active());
        printf("[RT] 重载槽 %u：%u 点\r\n", (unsigned)ins_route_slot_active(),
               (unsigned)ins_route_get_count());
    }
    s_load_last = now;

    /* ---- 4) 存盘（0→1）：把 RAM 表写进**当前槽** ---- */
    now = (rt_save_cmd > 0.5f) ? 1u : 0u;
    if ((0u != now) && (0u == s_save_last) && (0u == ins_route_flash_busy()) &&
        (ins_route_get_count() >= (uint16)RT_MIN_SAVE_POINTS))
    {
        ins_route_flash_save_slot(ins_route_slot_active());
        printf("[RT] 写入槽 %u（%u 点）\r\n", (unsigned)ins_route_slot_active(),
               (unsigned)ins_route_get_count());
    }
    s_save_last = now;

    /* ---- 4b) 打关键点（0→1）：给"最后一个已录点"打标（手工方案用）----
       ⚠ 只标点、不做任何 IO ⇒ 遥控器/按键/上位机三种触发方式都能走这一个入口。 */
    now = (rt_mark_cmd > 0.5f) ? 1u : 0u;
    if ((0u != now) && (0u == s_mark_last))
    {
        if (0u != ins_route_mark_key())
        {
            printf("[RT] 关键点 #%u（第 %u 点）\r\n",
                   (unsigned)ins_route_key_count(), (unsigned)ins_route_get_count());
        }
    }
    s_mark_last = now;

    /* ---- 5) 三块模块的轮询 ----
       · `ins_route_poll()` 录制采样（按距离触发，主循环级节拍就够）
       · `route_link_poll()` 串口灌表（顺序同工程 8：它靠 flash_busy 判完成）
       · `ins_route_flash_poll()` 每圈最多写一页，空闲立刻返回 */
    ins_route_poll();
    route_link_poll();
    ins_route_flash_poll();
}

uint8 route_app_slot_next (void)
{
    uint8 s;

    /* 录制中 / 正在写盘时不许切槽（会覆盖没存盘的录制结果） */
    if ((0u != ins_route_is_running()) || (0u != ins_route_flash_busy()))
    {
        return ins_route_slot_active();
    }

    s = (uint8)((ins_route_slot_active() + 1u) % (uint8)INS_RT_SLOT_COUNT);

    if (0u == ins_route_slot_set_active(s, 1u))
    {
        (void)ins_route_flash_load_slot(s);
        rt_slot_cmd = (float)s;
        s_slot_last = s;
        printf("[RT] 切到槽 %u：%u 点 / %u mm\r\n", (unsigned)s,
               (unsigned)ins_route_get_count(),
               (unsigned)(ins_route_get_head()->total_dist * 1000.0f + 0.5f));
    }

    return ins_route_slot_active();
}

/*===================================================================================================================
 *  三、对外状态
 *=================================================================================================================*/
uint16 route_app_count (void)
{
    return ins_route_get_count();
}

uint16 route_app_status (void)
{
    uint16 st = 0u;
    uint8  rc;

    if (0u != ins_route_is_running())                       { st |= RT_ST_REC;      }
    if (0u != (ins_route_flags() & INS_ROUTE_ST_FULL))      { st |= RT_ST_FULL;     }
    if (ins_route_get_count() > 0u)                         { st |= RT_ST_HAVE;     }
    if (0u != ins_route_flash_busy())                       { st |= RT_ST_SAVING;   }
    if (0u != ins_route_flash_loaded())                     { st |= RT_ST_FROMFLSH; }
    if (0u != route_link_active())                          { st |= RT_ST_LINK;     }

    rc = ins_route_flash_last_rc();
    if (2u == rc)                                           { st |= RT_ST_SAVEFAIL; }

    st |= (uint16)(((uint16)ins_route_slot_active() & 0x0Fu) << 8);

    return st;
}

const char *route_app_str (void)
{
    static char s[48];

    snprintf(s, sizeof(s), "S%u %upt %s", (unsigned)ins_route_slot_active(),
             (unsigned)ins_route_get_count(),
             (0u != ins_route_is_running()) ? "REC" :
             ((0u != ins_route_flash_busy()) ? "SAV" : "ok"));
    return s;
}
