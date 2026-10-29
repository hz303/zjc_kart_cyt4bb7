/*********************************************************************************************************************
 * 参数持久化实现: 载荷与缓冲 / CRC16-MODBUS / 对外接口
 * 参数元数据全部取自 proto.c 的 proto_param_count 与 proto_param_at
 ********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "app_params.h"
#include "flash_map.h"
#include "proto.h"
#include "route_app.h"
#include "steer_ctrl.h"          /* steer_ctrl_sanitize_thr(), 加载完必须钳一次门限关系 */

/* ---- 一、载荷与缓冲, 两个 blob 放静态区(各 396 B), 不要求可重入 ---- */
#define AP_MAGIC        (0x315A504Bu)       /* 'KPZ1', 小端字节序显示为 K P Z 1 */

typedef struct
{
    uint32 magic;
    uint16 version;
    uint16 count;
    uint16 crc;                             /* CRC16-MODBUS, 覆盖 val[0..count-1] */
    uint16 rsvd;
    float  val[APP_PARAM_MAX];
} ap_blob_t;

#define AP_WORDS        ((uint16)((sizeof(ap_blob_t) + 3u) / 4u))

static ap_blob_t s_blob;                    /* 存写与读回共用的工作缓冲 */
static ap_blob_t s_rb;
static uint8     s_loaded = 0u;
static uint8     s_err    = 0u;

/* ---- 二、CRC16-MODBUS ---- */
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

/* ---- 三、对外接口 ---- */
/* 恢复源码默认值, 实现在 proto.c, 保留本接口供既有调用点使用 */
void app_param_restore_defaults (void)
{
    proto_param_load_defaults();
}

void app_param_init (void)
{
    const proto_param_t *p;
    uint16 i;
    uint16 n;
    uint16 stored_n;
    uint8  took = 0u;

    s_loaded = 0u;
    s_err    = 0u;

    flash_init();

    flash_buffer_clear();
    flash_read_page_to_buffer(0u, FM_PARM_PAGE, AP_WORDS);
    memcpy(&s_blob, flash_union_buffer, sizeof(s_blob));

    n = proto_param_count();
    stored_n = s_blob.count;

    /* ---- 1) 头校验, 失败路径一律 goto done 以便末尾钳位 ---- */
    if (AP_MAGIC != s_blob.magic)               { s_err = 1u; goto done; }
    if (s_blob.version != APP_PARAM_VERSION)    { s_err = 2u; goto done; }
    /* ID88 追加在原表末尾, 允许 count 比当前表少 1 且末项为 ID88 */
    if (stored_n != n && !(stored_n + 1u == n &&
        proto_param_at(n - 1u)->id == 88u)) { s_err = 3u; goto done; }

    /* ---- 2) CRC ---- */
    if (s_blob.crc != ap_crc16((const uint8 *)s_blob.val, (uint16)(stored_n * 4u)))
    {
        s_err = 4u;
        goto done;
    }

    /* ---- 3) 逐项范围校验, 越界项丢弃; P_CMD 为命令量, 保持源码默认值 ---- */
    for (i = 0u; i < stored_n; i ++)
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
    if (stored_n != n)
    {
        rt_route_mode = (rt_revrec >= 0.5f) ?
            ((rt_revplay >= 0.5f) ? (float)RT_MODE_FZ : (float)RT_MODE_MIX_S) :
            ((rt_revplay >= 0.5f) ? (float)RT_MODE_ZF : (float)RT_MODE_ZZ);
    }

done:
    /* 加载成功或失败都要钳一次门限关系: Flash 里可能存着旧默认值, db 与 brk 相等会失效,
       此处只动本身不成立的值, 故不升 APP_PARAM_VERSION */
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

        /* 命令量存默认值, 不把正在使能或开环的状态带回来 */
        if (NULL == p)                              { b->val[i] = 0.0f;          }
        else if (P_CMD == p->perm)                  { b->val[i] = p->def;         }
        else                                        { b->val[i] = *(p->ptr);      }
    }

    b->crc = ap_crc16((const uint8 *)b->val, (uint16)(n * 4u));

    flash_buffer_clear();
    memcpy(flash_union_buffer, b, sizeof(*b));
    flash_erase_page(0u, FM_PARM_PAGE);
    (void)flash_write_page_from_buffer(0u, FM_PARM_PAGE, AP_WORDS);

    /* ---- 读回校验, 写完立刻读回逐字节比对 ---- */
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
