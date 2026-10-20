/*********************************************************************************************************************
 * 工程 11（卡丁）· 参数持久化实现（表驱动，设计见 app_params.h）
 *
 *  一、载荷与缓冲
 *  二、CRC16-MODBUS
 *  三、对外接口
 *
 *  ★★ 2026-10-20：本文件不再展开 `MOTOR_PARAM_TABLE`。参数元数据改由 `proto.c` 提供
 *     （proto_param_count / proto_param_at），"命令不许持久化"改由 `PERM == P_CMD` 表达。
 *     这也是本次"通信统一"的一环：参数表只在 `param_table.h` 里定义一次。
 ********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "app_params.h"
#include "flash_map.h"
#include "proto.h"
#include "steer_ctrl.h"          /* steer_ctrl_sanitize_thr()：加载完必须钳一次门限关系 */

/*===================================================================================================================
 *  一、载荷与缓冲
 *
 *  ⚠ 两个 blob 都放**静态区**而不是栈：`AP_PARAM_MAX = 96` 时一个 blob 就 396 B，
 *    `app_param_save()` 里同时要 b + 读回缓冲 = 792 B。CM7_0 的主循环栈没那么宽裕，
 *    而这两个缓冲本来就不需要可重入（只在初始化和主循环各调用一次）。
 *=================================================================================================================*/
#define AP_MAGIC        (0x315A504Bu)       /* 'KPZ1' (小端字节序显示为 K P Z 1) */

typedef struct
{
    uint32 magic;
    uint16 version;
    uint16 count;
    uint16 crc;                             /* CRC16-MODBUS，覆盖 val[0..count-1] */
    uint16 rsvd;
    float  val[APP_PARAM_MAX];
} ap_blob_t;

#define AP_WORDS        ((uint16)((sizeof(ap_blob_t) + 3u) / 4u))

static ap_blob_t s_blob;                    /* 存/读共用的工作缓冲 */
static ap_blob_t s_rb;                      /* 写后读回校验               */
static uint8     s_loaded = 0u;
static uint8     s_err    = 0u;

/*===================================================================================================================
 *  二、CRC16-MODBUS
 *=================================================================================================================*/
static uint16 ap_crc16 (const uint8 *p, uint16 n)
{
    uint16 crc = 0xFFFFu;
    uint16 i;
    uint8  b;

    for (i = 0u; i < n; i ++)
    {
        crc ^= (uint16)p[i];
        for (b = 0u; b < 8u; b ++)
        {
            crc = (0u != (crc & 1u)) ? (uint16)((crc >> 1) ^ 0xA001u) : (uint16)(crc >> 1);
        }
    }
    return crc;
}

/*===================================================================================================================
 *  三、对外接口
 *=================================================================================================================*/
/* 恢复源码默认值：唯一实现在 proto.c（菜单 9.Reset / 上位机 LOAD 都走那条路）。
   这里保留旧接口，避免还有别的调用点。 */
void app_param_restore_defaults (void)
{
    proto_param_load_defaults();
}

void app_param_init (void)
{
    const proto_param_t *p;
    uint16 i;
    uint16 n;
    uint8  took = 0u;

    s_loaded = 0u;
    s_err    = 0u;

    flash_init();

    flash_buffer_clear();
    flash_read_page_to_buffer(0u, FM_PARM_PAGE, AP_WORDS);
    memcpy(&s_blob, flash_union_buffer, sizeof(s_blob));

    n = proto_param_count();

    /* ---- 1) 头校验 ---- */
    /* ⚠ 四条失败路径都走 `goto done`（不是 `return`）：离开前必须跑一次门限钳位（见末尾）。 */
    if (AP_MAGIC != s_blob.magic)               { s_err = 1u; goto done; }   /* 从没存过 */
    if (s_blob.version != APP_PARAM_VERSION)    { s_err = 2u; goto done; }   /* 版本不符 */
    if (s_blob.count   != n)                    { s_err = 3u; goto done; }   /* 表变了(防错位) */

    /* ---- 2) CRC ---- */
    if (s_blob.crc != ap_crc16((const uint8 *)s_blob.val, (uint16)(n * 4u)))
    {
        s_err = 4u;
        goto done;
    }

    /* ---- 3) 逐项：范围校验 + 跳过命令量 ----
       ★ `P_CMD` 就是"命令/危险量"：保持源码默认值，绝不把上次的"正在使能/正在开环"带回来。 */
    for (i = 0u; i < n; i ++)
    {
        float v = s_blob.val[i];

        p = proto_param_at(i);
        if (NULL == p)              { continue; }
        if (P_CMD == p->perm)       { continue; }

        if ((v >= p->lo) && (v <= p->hi))
        {
            *(p->ptr) = v;
            took ++;
        }
        else
        {
            s_err = 5u;                                             /* 有越界项：单项丢弃，不整块拒 */
        }
    }

    s_loaded = (took > 0u) ? 1u : 0u;

done:
    /* ★★ 2026-10-19：加载完（或加载失败）都必须钳一次三个门限的关系。
       典型场景：Flash 里存的是**旧默认值** `steer_db = 0.20`（那时 ID21 的 def 就是 0.2），
       而 `steer_brk` 也是 0.20 ⇒ 滞环带为空，10-18 晚那套修法当场失效 ——
       而且**不报错**，只表现为"烧了新版还是抖"。
       ⚠ 这里**没有**升 APP_PARAM_VERSION：升版本会把用户调好的 kp/u0 一起清掉，
         而这条钳位只动"本来就构成不成立"的那几个值，代价最小。 */
    steer_ctrl_sanitize_thr();
}

uint8 app_param_save (void)
{
    const proto_param_t *p;
    ap_blob_t *b = &s_blob;
    uint16 i;
    uint16 n;

    n = proto_param_count();

    memset(b, 0, sizeof(*b));
    b->magic   = AP_MAGIC;
    b->version = APP_PARAM_VERSION;
    b->count   = n;
    b->rsvd    = 0u;

    for (i = 0u; i < n; i ++)
    {
        p = proto_param_at(i);

        /* 命令/危险量：写**默认值**，绝不把"正在使能/正在开环"这种状态存进去 */
        if (NULL == p)                              { b->val[i] = 0.0f;          }
        else if (P_CMD == p->perm)                  { b->val[i] = p->def;         }
        else                                        { b->val[i] = *(p->ptr);      }
    }

    b->crc = ap_crc16((const uint8 *)b->val, (uint16)(n * 4u));

    flash_buffer_clear();
    memcpy(flash_union_buffer, b, sizeof(*b));
    flash_erase_page(0u, FM_PARM_PAGE);
    (void)flash_write_page_from_buffer(0u, FM_PARM_PAGE, AP_WORDS);

    /* ---- 读回校验（写完立刻读回比对 —— 这是"真的写进去了"的唯一证据）---- */
    flash_buffer_clear();
    flash_read_page_to_buffer(0u, FM_PARM_PAGE, AP_WORDS);
    memcpy(&s_rb, flash_union_buffer, sizeof(s_rb));

    if (0 != memcmp(&s_rb, b, sizeof(*b)))
    {
        s_err = 6u;
        return 6u;
    }

    s_err    = 0u;
    s_loaded = 1u;
    return 0u;
}

uint8 app_param_loaded (void)   { return s_loaded; }
uint8 app_param_last_err (void) { return s_err; }
