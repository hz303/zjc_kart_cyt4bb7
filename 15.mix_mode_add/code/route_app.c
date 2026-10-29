/*********************************************************************************************************************
 * 路表应用层实现（接口与状态位见 route_app.h）
 ********************************************************************************************************************/
#include "zf_common_headfile.h"
#include <stdio.h>
#include <string.h>

#include "ins_route.h"
#include "route_link.h"
#include "route_app.h"
#include "route_pb.h"      /* 回放状态：rt_st 的 PLAY 位 */

/* ---- 命令量（绑定到参数表 ID 38/40/41/42/53）---- */
volatile float rt_rec_cmd  = 0.0f;
volatile float rt_slot_cmd = 0.0f;
volatile float rt_load_cmd = 0.0f;
volatile float rt_save_cmd = 0.0f;
volatile float rt_mark_cmd = 0.0f;
volatile float rt_route_mode = (float)RT_MODE_ZZ;

/* ---- 二值开关（绑定参数表 ID 76/77）----
 * 这两项是设置项，必须持久化，route_app_init() 不清它们 */
volatile float rt_revrec    = 0.0f;
volatile float rt_revplay   = 0.0f;
volatile float pb_drev      = 0.0f;   /* ID85：0=正车，1=倒车 */
volatile float pb_rvk       = 1.0f;   /* ID86：倒车开环航向 Kp（deg/deg，0=纯开环）*/
volatile float pb_ctrlmode  = 1.0f;   /* ID78 默认 1=纯追踪 */
volatile float pb_cauto     = 1.0f;   /* ID87 默认自动：正向 LQR，倒车按表长短择优 */
volatile float ctrl_qe      = 4.00f;   /* ID79 横向误差权重 */
volatile float ctrl_qth     = 1.00f;   /* ID80 航向误差权重 */
volatile float ctrl_r       = 1.00f;   /* ID81 打角权重（1.0 => Ke=2.0, Kth=2.236）*/
volatile float ctrl_tlead   = 0.12f;   /* ID82 前馈预览 s */
/* ID83/ID84 已从参数表删除，两个变量保留以稳定参数地址；任何控制律都不读 */
volatile float ctrl_kusff   = 0.00f;   /* 保留：不足转向补偿 */
volatile float ctrl_esow    = 8.00f;   /* 保留：ESO 带宽 rad/s */

/* ---- 边沿记忆 ---- */
static uint8   s_rec_last   = 0u;
static uint8   s_load_last  = 0u;
static uint8   s_save_last  = 0u;
static uint8   s_mark_last  = 0u;
static uint8   s_slot_last  = 0xFFu;    /* 0xFF = 还没同步过 */

/* ---- 一、上电 ---- */
void route_app_init (void)
{
    uint32  i;
    uint8   rt_ok;

    ins_route_init();                   /* 清 RAM 缓冲 + 夹点距 */

    /* ① 读槽索引决定 active 槽；顺序约束：先补内建表，再载入 */
    ins_route_slot_init();

    /* ② 串口链路：只清接收缓冲与状态，UART 已由 debug_init 起好 */
    route_link_init();

    /* ③ 内建表只补空槽（注册表见 ins_route_builtin.c）
     *    此处仍在 5 ms 中断之前，可死等写盘；限圈数防挂死 */
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

    /* ④ 载入 active 槽（阻塞，须在 5 ms 中断打开之前） */
    rt_ok = ins_route_flash_load();

    /* 不用 %f：IAR 默认库不一定带浮点格式化 */
    printf("[RT] 路表: %s  点数=%u  总里程=%u mm  点距=%u mm  槽=%u/%u\r\n",
           (0u != rt_ok) ? "已载入" : "无表/无效",
           (unsigned)ins_route_get_count(),
           (unsigned)(ins_route_get_head()->total_dist * 1000.0f + 0.5f),
           (unsigned)(ins_route_ds * 1000.0f + 0.5f),
           (unsigned)ins_route_slot_active(), (unsigned)INS_RT_SLOT_COUNT);

    /* ⑤ 命令量归零，rt_slot 跟上真实 active 槽 */
    rt_rec_cmd  = 0.0f;
    rt_load_cmd = 0.0f;
    rt_save_cmd = 0.0f;
    rt_mark_cmd = 0.0f;
    s_slot_last = ins_route_slot_active();
    rt_slot_cmd = (float)s_slot_last;
}

/* ---- 二、主循环 ---- */
void route_app_poll (void)
{
    uint8 now;
    uint8 slot;

    /* ---- 1) 录制开关：0->1 开始；1->0 停止并落槽 0 ---- */
    now = (rt_rec_cmd > 0.5f) ? 1u : 0u;
    if (now != s_rec_last)
    {
        s_rec_last = now;

        if (0u != now)
        {
            /* 录制与回放互斥；另一处检查在 route_pb.c 的 pb_start() */
            if (0u != route_pb_active())
            {
                printf("[RT] !! 回放进行中，不能开始录制（先停回放）\r\n");
                rt_rec_cmd = 0.0f;
                s_rec_last = 0u;
            }
            else if ((0u == ins_route_is_running()) && (0u == ins_route_flash_busy()))
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

    /* ---- 2) 选槽：目标槽变化即切 active（持久化）并载入；录制中不允许切 ---- */
    slot = (uint8)(rt_slot_cmd + 0.5f);
    if (slot >= (uint8)INS_RT_SLOT_COUNT) { slot = 0u; }

    if (slot != s_slot_last)
    {
        s_slot_last = slot;
        if (0u != ins_route_is_running())
        {
            rt_slot_cmd = (float)ins_route_slot_active();   /* 录制中：拒绝，退回原值 */
        }
        else if (0u == ins_route_flash_busy())
        {
            if (0u != ins_route_slot_select(slot, 1u))
            {
                printf("[RT] 切到槽 %u：%u 点 / %u mm\r\n", (unsigned)slot,
                       (unsigned)ins_route_get_count(),
                       (unsigned)(ins_route_get_head()->total_dist * 1000.0f + 0.5f));
            }
            else
            {
                printf("[RT] 槽 %u 无效，保持槽 %u\r\n",
                       (unsigned)slot, (unsigned)ins_route_slot_active());
            }
        }
        s_slot_last = ins_route_slot_active();
        rt_slot_cmd = (float)s_slot_last;
    }

    /* ---- 3) 重载（0->1）：从 Flash 再读一次 active 槽 ---- */
    now = (rt_load_cmd > 0.5f) ? 1u : 0u;
    if ((0u != now) && (0u == s_load_last) && (0u == ins_route_flash_busy()) &&
        (0u == ins_route_is_running()) && (0u == route_pb_active()))
    {
        (void)ins_route_flash_load_slot(ins_route_slot_active());
        printf("[RT] 重载槽 %u：%u 点\r\n", (unsigned)ins_route_slot_active(),
               (unsigned)ins_route_get_count());
    }
    s_load_last = now;

    /* ---- 4) 存盘（0->1）：把 RAM 表写进当前槽 ---- */
    now = (rt_save_cmd > 0.5f) ? 1u : 0u;
    /* 回放期间不存盘：回放会原地倒序 s_wp[]，写盘会把倒序表落进 Flash
     * 倒序标志只存在于 route_pb 的私有静态量，存盘器看不到，重启也不恢复 */
    if ((0u != now) && (0u == s_save_last) && (0u == ins_route_flash_busy()) &&
        (0u == ins_route_is_running()) &&
        (ins_route_get_count() >= (uint16)RT_MIN_SAVE_POINTS))
    {
        ins_route_flash_save_slot(ins_route_slot_active());
        printf("[RT] 写入槽 %u（%u 点）\r\n", (unsigned)ins_route_slot_active(),
               (unsigned)ins_route_get_count());
    }
    s_save_last = now;

    /* ---- 4b) 打关键点（0->1）：给最后一个已录点打标；只标点，不做 IO ---- */
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

    /* ---- 5) 每圈轮询：录制采样 / 串口灌表 / Flash 分步写（每圈最多一页）---- */
    ins_route_poll();
    route_link_poll();
    ins_route_flash_poll();
}

uint8 route_app_slot_next (void)
{
    uint8 s;

    /* 录制中 / 正在写盘时不许切槽（会覆盖没存盘的录制结果） */
    if (route_pb_active() || ins_route_is_running() || ins_route_flash_busy())
    {
        return ins_route_slot_active();
    }

    s = (uint8)((ins_route_slot_active() + 1u) % (uint8)INS_RT_SLOT_COUNT);

    if (0u != ins_route_slot_select(s, 1u))
    {
        rt_slot_cmd = (float)s;
        s_slot_last = s;
        printf("[RT] 切到槽 %u：%u 点 / %u mm\r\n", (unsigned)s,
               (unsigned)ins_route_get_count(),
               (unsigned)(ins_route_get_head()->total_dist * 1000.0f + 0.5f));
    }

    return ins_route_slot_active();
}

/* ---- 三、对外状态 ---- */
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
    if (0u != route_pb_active())                            { st |= RT_ST_PLAY;     }

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
