/* 工程 10（卡丁）· 跑班数据记录实现（见 bench_log.h） 落盘那套状态机整体删掉了，只留 "RAM 攒 → 串口导 CSV"。 */

#include "zf_common_headfile.h"
#include "bench_log.h"

/* 一、记录与缓冲 */
typedef struct
{
    int32 t_ms;
    int16 duty_x10;
    int16 rps_l_x100;
    int16 rps_r_x100;
    int16 seg;
} bl_rec_t;

/* 编译期守卫：12 字节（UART 导出与上位机/脚本按这 12 B 解析，别随手加字段） */
typedef char bl_rec_size_check[(sizeof(bl_rec_t) == 12u) ? 1 : -1];

/* ---- RAM 缓冲：跑班全程只进不出（4096 × 12 B = 48 KB）---- */
static bl_rec_t s_buf[BL_CAPACITY];

static volatile uint16 s_count   = 0u;
static volatile uint32 s_t_ms    = 0u;
static volatile uint8  s_state   = BL_IDLE;
static volatile uint8  s_full    = 0u;
static volatile uint8  s_complete= 0u;

static uint16 s_stored  = 0u;               /* 已冻结、可导出的条数 */
static uint16 s_rd_page = 0u;               /* 导出进度（按"页"分块，只是节流单位） */

/* 二、小工具 */
static int16 bl_i16(float v)
{
    return (int16)((v >= 0.0f) ? (v + 0.5f) : (v - 0.5f));
}

/* 定点打印：v 是 ×10 或 ×100 后的整数，dec = 1 或 2 位小数（带符号） */
static void bl_pf(int16 v, uint8 dec)
{
    uint16 a;

    if (v < 0) { printf("-"); a = (uint16)(-((int16)v)); }
    else       { a = (uint16)v; }

    if (2u == dec) { printf("%u.%02u", (unsigned)(a / 100u), (unsigned)(a % 100u)); }
    else           { printf("%u.%u",   (unsigned)(a / 10u),  (unsigned)(a % 10u));  }
}

/* 三、开始 / 采样 / 结束 */
void bench_log_begin(void)
{
    s_count    = 0u;
    s_t_ms     = 0u;
    s_full     = 0u;
    s_complete = 0u;
    s_rd_page  = 0u;
    s_state    = BL_REC;
}

void bench_log_tick(uint16 t_in_seg_ms, uint8 seg, float duty, float rps_l, float rps_r)
{
    bl_rec_t *r;
    uint8     want;

    if (BL_REC != s_state) { return; }

    if (s_count >= (uint16)BL_CAPACITY) { s_full = 1u; return; }

    if (t_in_seg_ms < BL_BURST_MS)
    {
        want = 1u;                                                  /* 突发段：每拍(5ms)都记 */
    }
    else
    {
        want = ((uint16)((t_in_seg_ms - BL_BURST_MS) % BL_STEADY_MS) < 5u) ? 1u : 0u;
    }

    if (0u == want) { return; }

    r = &s_buf[s_count];
    r->t_ms       = (int32)s_t_ms;
    r->duty_x10   = bl_i16(duty * 10.0f);
    r->rps_l_x100 = bl_i16(rps_l * 100.0f);
    r->rps_r_x100 = bl_i16(rps_r * 100.0f);
    r->seg        = (int16)seg;

    s_count++;
}

void bench_log_tick_clock(void)
{
    s_t_ms += 5u;
}

void bench_log_end(uint8 complete)
{
    if (BL_REC != s_state) { return; }

    s_complete = complete;
    s_stored   = s_count;                       /* 冻结：导出读这一份 */
    s_rd_page  = 0u;
    s_state    = BL_DONE;
}

/* 四、主循环：分页导出 / 清空 */
void bench_log_poll(void)
{
    uint16 per_page = (uint16)BL_EXPORT_PER_PAGE;
    uint16 pages;
    uint16 i;

    if (BL_DUMP != s_state) { return; }

    pages = (uint16)((s_stored + per_page - 1u) / per_page);
    if (s_rd_page >= pages) { s_state = BL_DONE; printf("\r\n# BL end\r\n"); return; }

    {
        uint32 first  = (uint32)s_rd_page * (uint32)per_page;
        uint32 remain = (s_stored > first) ? ((uint32)s_stored - first) : 0u;
        uint16 n      = (uint16)((remain > (uint32)per_page) ? (uint32)per_page : remain);

        for (i = 0u; i < n; i ++)
        {
            const bl_rec_t *p = &s_buf[first + (uint32)i];

            printf("BL,%ld,%d,", (long)p->t_ms, (int)p->seg);
            bl_pf(p->duty_x10,   1u); printf(",");
            bl_pf(p->rps_l_x100, 2u); printf(",");
            bl_pf(p->rps_r_x100, 2u); printf("\r\n");
        }
    }

    s_rd_page++;
}

void bench_log_dump(void)
{
    if (0u == s_stored) { printf("# BL: no data in RAM\r\n"); return; }

    s_rd_page = 0u;
    s_state   = BL_DUMP;
    printf("\r\n# BL v2: t_ms,seg,duty_pct,rps_l,rps_r  (n=%u)\r\n", (unsigned)s_stored);
}

void bench_log_clear(void)
{
    s_count  = 0u;
    s_stored = 0u;
    s_t_ms   = 0u;
    s_full   = 0u;
    s_rd_page= 0u;

    if ((BL_DONE == s_state) || (BL_DUMP == s_state) || (BL_ERR == s_state)) { s_state = BL_IDLE; }
}

uint8  bench_log_state (void) { return s_state; }
uint16 bench_log_count (void) { return s_count; }
uint16 bench_log_stored(void) { return s_stored; }
uint8  bench_log_full  (void) { return s_full; }
