/*********************************************************************************************************************
 * 参数持久化实现：按参数表顺序存/读 float 数组, 写后读回校验
 ********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "app_params.h"
#include "flash_map.h"
#include "proto.h"
#include "route_app.h"
#include "steer_ctrl.h"          /* steer_ctrl_sanitize_thr()：加载完必须钳一次门限关系 */

/* ---- 一、载荷与缓冲（两个 blob 放静态区, 只在初始化/主循环调用） ---- */
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

/* flash_buffer_clear() 只清 512 B, 载荷超过 128 word 时须自行 memset */
#define AP_WORDS        ((uint16)((sizeof(ap_blob_t) + 3u) / 4u))

static ap_blob_t s_blob;                    /* 存/读共用的工作缓冲 */
static ap_blob_t s_rb;                      /* 写后读回校验               */
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
/* 恢复源码默认值：实现在 proto.c 的 proto_param_load_defaults() */
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

    /* 失败路径统一 goto done, 离开前要钳一次门限 */
    if (AP_MAGIC != s_blob.magic)               { s_err = 1u; goto done; }
    if (s_blob.version != APP_PARAM_VERSION)    { s_err = 2u; goto done; }
    /* ID88 追加在原表尾部, 旧块多一项也接受 */
    if (stored_n != n && !(stored_n + 1u == n &&
        proto_param_at(n - 1u)->id == 88u)) { s_err = 3u; goto done; }

    if (s_blob.crc != ap_crc16((const uint8 *)s_blob.val, (uint16)(stored_n * 4u)))
    {
        s_err = 4u;
        goto done;
    }

    /* 逐项：范围校验, P_CMD 跳过 */
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
            s_err = 5u;                                             /* 越界项丢弃, 不整块拒 */
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
    /* 加载完（成功或失败）都要钳一次门限关系, 防止旧默认值构成非法组合 */
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

        /* P_CMD 写默认值, 不存当前状态 */
        if (NULL == p)                              { b->val[i] = 0.0f;          }
        else if (P_CMD == p->perm)                  { b->val[i] = p->def;         }
        else                                        { b->val[i] = *(p->ptr);      }
    }

    b->crc = ap_crc16((const uint8 *)b->val, (uint16)(n * 4u));

    flash_buffer_clear();
    memcpy(flash_union_buffer, b, sizeof(*b));
    flash_erase_page(0u, FM_PARM_PAGE);
    (void)flash_write_page_from_buffer(0u, FM_PARM_PAGE, AP_WORDS);

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
