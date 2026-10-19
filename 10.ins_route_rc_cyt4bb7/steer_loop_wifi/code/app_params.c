/*********************************************************************************************************************
 * 工程 10（卡丁）· 参数持久化实现（表驱动，见 app_params.h）
 ********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "app_params.h"
#include "flash_map.h"
#include "motor_link.h"

/*===================================================================================================================
 *  一、参数表的"行数 / 指针 / 量程 / 默认值" 四张平行数组（全部由 X-macro 自动展开）
 *     ⚠ 表一改，这里自动跟着变 —— **不要在别处手工维护参数个数**
 *=================================================================================================================*/
#define AP_ROW_COUNT_1(id, nm, lab, lo, hi, df, un, st, ptr)    + 1
enum { AP_ROW_COUNT = 0 MOTOR_PARAM_TABLE(AP_ROW_COUNT_1) };

static volatile float * const ap_ptr[] =
{
#define AP_ROW_PTR(id, nm, lab, lo, hi, df, un, st, ptr)        (volatile float *)(ptr),
    MOTOR_PARAM_TABLE(AP_ROW_PTR)
};

static const float ap_lo[] =
{
#define AP_ROW_LO(id, nm, lab, lo, hi, df, un, st, ptr)         (lo),
    MOTOR_PARAM_TABLE(AP_ROW_LO)
};

static const float ap_hi[] =
{
#define AP_ROW_HI(id, nm, lab, lo, hi, df, un, st, ptr)         (hi),
    MOTOR_PARAM_TABLE(AP_ROW_HI)
};

static const float ap_df[] =
{
#define AP_ROW_DF(id, nm, lab, lo, hi, df, un, st, ptr)         (df),
    MOTOR_PARAM_TABLE(AP_ROW_DF)
};

static const uint8 ap_id[] =
{
#define AP_ROW_ID(id, nm, lab, lo, hi, df, un, st, ptr)         (uint8)(id),
    MOTOR_PARAM_TABLE(AP_ROW_ID)
};

/*===================================================================================================================
 *  二、**不许持久化的参数**（命令 / 状态 / 危险量）
 *
 *     ⚠⚠ 这是本模块最重要的一条安全规则。参数表里混着三类东西：
 *        ① **配置量**（增益、比例尺、限位）—— 该记；这正是本模块存在的理由；
 *        ② **命令量**（`ins_zero` 是 0→1 边沿命令）—— 记了会在下次上电**自己触发一次归零**；
 *        ③ **危险状态**（`ol_en` / `ol_duty_*` 是开环直给占空比；`target_*` 是目标转速；
 *            `steer_en` 是内环使能）—— 记了会**下次上电直接带着输出启动**。
 *
 *     ⇒ ②③ 一律**不恢复**（载入时跳过、保存时写默认值），上电永远是安全值。
 *=================================================================================================================*/
static uint8 ap_is_cmd(uint8 id)
{
    switch (id)
    {
        case 0:  return 1u;     /* target_l    —— 目标转速 */
        case 1:  return 1u;     /* target_r                   */
        case 23: return 1u;     /* steer_en    —— 转向内环使能 */
        case 29: return 1u;     /* ol_en       —— 开环模式     */
        case 30: return 1u;     /* ol_duty_l   —— 开环占空比   */
        case 31: return 1u;     /* ol_duty_r                  */
        case 37: return 1u;     /* ins_zero    —— 0→1 边沿命令 */
        case 38: return 1u;     /* rt_rec      —— 0→1 开始录制 / 1→0 停止落盘 */
        case 41: return 1u;     /* rt_load     —— 0→1 重载槽 */
        case 42: return 1u;     /* rt_save     —— 0→1 存到槽 */
        case 53: return 1u;     /* rt_mark     —— 0→1 打关键点 */
        default: return 0u;
    }
}

/*===================================================================================================================
 *  三、载荷
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

static uint8  s_loaded = 0u;
static uint8  s_err    = 0u;

/*===================================================================================================================
 *  四、CRC16-MODBUS
 *=================================================================================================================*/
static uint16 ap_crc16(const uint8 *p, uint16 n)
{
    uint16 crc = 0xFFFFu;
    uint16 i;
    uint8  b;

    for (i = 0u; i < n; i++)
    {
        crc ^= (uint16)p[i];
        for (b = 0u; b < 8u; b++)
        {
            crc = (0u != (crc & 1u)) ? (uint16)((crc >> 1) ^ 0xA001u) : (uint16)(crc >> 1);
        }
    }
    return crc;
}

/*===================================================================================================================
 *  五、对外接口
 *=================================================================================================================*/
void app_param_restore_defaults(void)
{
    uint16 i;

    for (i = 0u; i < (uint16)AP_ROW_COUNT; i++)
    {
        *ap_ptr[i] = ap_df[i];
    }
}

void app_param_init(void)
{
    ap_blob_t b;
    uint16    i;
    uint8     took = 0u;

    s_loaded = 0u;
    s_err    = 0u;

    flash_init();

    flash_buffer_clear();
    flash_read_page_to_buffer(0u, FM_PARM_PAGE, AP_WORDS);
    memcpy(&b, flash_union_buffer, sizeof(b));

    /* ---- 1) 头校验 ---- */
    /* ⚠ 四条失败路径都走 `goto done`（不是 `return`）：离开前必须跑一次门限钳位（见末尾）。 */
    if (AP_MAGIC != b.magic)            { s_err = 1u; goto done; }     /* 从没存过 */
    if (b.version != APP_PARAM_VERSION) { s_err = 2u; goto done; }     /* 版本不符 */
    if (b.count   != (uint16)AP_ROW_COUNT) { s_err = 3u; goto done; }  /* 表变了(防错位) */

    /* ---- 2) CRC ---- */
    if (b.crc != ap_crc16((const uint8 *)b.val, (uint16)(b.count * 4u))) { s_err = 4u; goto done; }

    /* ---- 3) 逐项：范围校验 + 跳过命令量 ---- */
    for (i = 0u; i < (uint16)AP_ROW_COUNT; i++)
    {
        float v = b.val[i];

        if (0u != ap_is_cmd(ap_id[i])) { continue; }                /* 命令/危险量：保持默认 */

        if ((v >= ap_lo[i]) && (v <= ap_hi[i]))
        {
            *ap_ptr[i] = v;
            took++;
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
       ⚠ 这里**没有**升 `APP_PARAM_VERSION`：升版本会把用户调好的 kp/u0 一起清掉，
         而这条钳位只动"本来就构成不成立"的那几个值，代价最小。 */
    steer_ctrl_sanitize_thr();
}

uint8 app_param_save(void)
{
    ap_blob_t b;
    uint16    i;
    ap_blob_t rb;

    b.magic   = AP_MAGIC;
    b.version = APP_PARAM_VERSION;
    b.count   = (uint16)AP_ROW_COUNT;
    b.rsvd    = 0u;

    for (i = 0u; i < (uint16)AP_ROW_COUNT; i++)
    {
        /* 命令/危险量：写**默认值**，绝不把"正在使能/正在开环"这种状态存进去 */
        b.val[i] = (0u != ap_is_cmd(ap_id[i])) ? ap_df[i] : (*ap_ptr[i]);
    }

    b.crc = ap_crc16((const uint8 *)b.val, (uint16)(b.count * 4u));

    flash_buffer_clear();
    memcpy(flash_union_buffer, &b, sizeof(b));
    flash_erase_page(0u, FM_PARM_PAGE);
    (void)flash_write_page_from_buffer(0u, FM_PARM_PAGE, AP_WORDS);

    /* ---- 读回校验（写完立刻读回比对 —— 这是"真的写进去了"的唯一证据）---- */
    flash_buffer_clear();
    flash_read_page_to_buffer(0u, FM_PARM_PAGE, AP_WORDS);
    memcpy(&rb, flash_union_buffer, sizeof(rb));

    if (0 != memcmp(&rb, &b, sizeof(b)))
    {
        s_err = 6u;
        return 6u;
    }

    s_err = 0u;
    s_loaded = 1u;
    return 0u;
}

uint8 app_param_loaded(void)   { return s_loaded; }
uint8 app_param_last_err(void) { return s_err; }
