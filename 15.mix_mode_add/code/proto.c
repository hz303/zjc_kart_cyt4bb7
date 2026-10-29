/*********************************************************************************************************************
 * 协议层实现：参数元数据展开、驱动令牌、权限检查、唯一写入口 proto_exec()、指纹与打印
 ********************************************************************************************************************/

#include "zf_common_headfile.h"
#include <string.h>
#include <float.h>
#include "ins_route.h"
#include "route_app.h"

#include "proto.h"
#include "motor_link.h"          /* MOTOR_PARAM_TABLE / CH_* / motor_link_now_ms() */
#include "app_params.h"          /* app_param_save()：SAVE 的落盘实现 */
#include "spd_ctrl.h"
#include "spd_safety.h"
#include "steer_ctrl.h"          /* steer_ctrl_sanitize_thr() / steer_ctrl_set_estop() */
#include "route_pb.h"            /* route_pb_active()：回放中禁改几何类参数 */
#include "head_test.h"

/* ---- 一、参数元数据（param_table.h 唯一定义, 此处展开一次） ---- */
#define PP_ROW(id, nm, lb, lo, hi, df, un, st, pt, pm, gp)                                     \
    { (uint8)(id), (nm), (lb), (lo), (hi), (df), (un), (st), (volatile float *)(pt),           \
      (uint8)(pm), (uint8)(gp) },

static const proto_param_t s_ptab[] = { MOTOR_PARAM_TABLE(PP_ROW) };
#undef PP_ROW

#define PP_COUNT    ((uint16)(sizeof(s_ptab) / sizeof(s_ptab[0])))

/* 编译期护栏：表项数超过 PROTO_PARAM_MAX（载荷数组按其开）就编不过 */
typedef char pp_fits_in_proto_max_[(PP_COUNT <= PROTO_PARAM_MAX) ? 1 : -1];

/* ---- 二、驱动令牌 / 事件 ---- */
static uint8  s_owner      = PROTO_SRC_NONE;
static uint8  s_estop_mask = 0u;        /* 按来源锁存的急停位（bit0 RC / bit1 SPI / bit2 MENU / bit3 INTERNAL）*/
static uint32 s_lease_last = 0u;
static uint16 s_lease_ms   = 0u;
static void (*s_evt)(const char *text) = NULL;

static void proto_event (const char *text)
{
    if (NULL != s_evt) { s_evt(text); }
}

/* 各来源驱动权租期 ms：只决定别的设备何时可接管, 掉线停机由 spd_safety 失联门负责; 遥控由 SBUS 帧续租 */
static uint16 proto_lease_default (uint8 src)
{
    switch (src)
    {
        case PROTO_SRC_RC:      return 600u;
        case PROTO_SRC_SPI:     return 1000u;
        case PROTO_SRC_MENU:    return 800u;
        case PROTO_SRC_INTERNAL:return 1000u;
        default:                return 0u;
    }
}

void proto_init (void)
{
    s_owner      = PROTO_SRC_NONE;
    s_estop_mask = 0u;
    s_lease_last = 0u;
    s_lease_ms   = 0u;
    s_evt        = NULL;
}

void proto_set_event_sink (void (*fn)(const char *text))
{
    s_evt = fn;
}

uint8 proto_drv_owner (void)
{
    return s_owner;
}

uint8 proto_drv_held_by (uint8 src)
{
    return (PROTO_SRC_NONE != src) && (s_owner == src) ? 1u : 0u;
}

uint8 proto_drv_acquire (uint8 src)
{
    if (PROTO_SRC_NONE == src) { return 0u; }

    if (PROTO_SRC_NONE == s_owner)
    {
        s_owner = src;
    }
    else if (s_owner == src)
    {
    }
    else if (PROTO_SRC_RC == src)
    {
        /* 遥控抢占：人看得见车, 优先级最高; 通知上位机 */
        proto_event("RC took over drive");
        s_owner = src;
    }
    else
    {
        return 0u;                  /* 被别的设备占着：上位机/菜单都只能等它到期 */
    }

    s_lease_last = motor_link_now_ms();
    s_lease_ms   = proto_lease_default(src);
    return 1u;
}

void proto_drv_renew (uint8 src)
{
    if ((PROTO_SRC_NONE != src) && (s_owner == src))
    {
        s_lease_last = motor_link_now_ms();
        s_lease_ms   = proto_lease_default(src);
    }
}

void proto_drv_release (uint8 src)
{
    if (s_owner == src)
    {
        s_owner    = PROTO_SRC_NONE;
        s_lease_ms = 0u;
    }
}

void proto_tick (void)
{
    uint32 now;
    uint8  who;

    if (PROTO_SRC_NONE == s_owner) { return; }
    if (0u == s_lease_ms)          { return; }

    now = motor_link_now_ms();
    if ((uint32)(now - s_lease_last) <= (uint32)s_lease_ms) { return; }

    /* 租约到期释放令牌, 不清目标值; 仅在 SPI 链路确认离线时顺手清零目标 */
    who        = s_owner;
    s_owner    = PROTO_SRC_NONE;
    s_lease_ms = 0u;

    if ((PROTO_SRC_SPI == who) && (0u == motor_link_online()))
    {
        spd_target_l = 0.0f;
        spd_target_r = 0.0f;
    }

    proto_event("drive lease expired");
}

void proto_estop (uint8 src, uint8 on)
{
    uint8 bit;
    uint8 was;

    /* 急停按来源锁存一位, 任一位置位即停机; 一个来源写 0 不影响其他来源 */
    if ((src < PROTO_SRC_RC) || (src > PROTO_SRC_INTERNAL)) { src = PROTO_SRC_INTERNAL; }
    bit = (uint8)(1u << (src - 1u));
    was = s_estop_mask;

    if (0u != on)
    {
        if (0u == s_estop_mask)
        {
            /* 急停剥夺令牌 */
            if (PROTO_SRC_NONE != s_owner) { proto_event("ESTOP: lease revoked"); }
            s_owner    = PROTO_SRC_NONE;
            s_lease_ms = 0u;
        }
        s_estop_mask |= bit;
    }
    else
    {
        s_estop_mask &= (uint8)(~bit);
    }

    if ((0u == was) && (0u != s_estop_mask)) { proto_event("ESTOP on"); }

    spd_safety_set_estop((0u != s_estop_mask) ? 1u : 0u);
    steer_ctrl_set_estop((0u != s_estop_mask) ? 1u : 0u);
}

/* ---- 三、权限 ---- */
/* 管理类操作（存 Flash / 恢复默认）：无人驱动或自己是驱动者时允许 */
static uint8 proto_perm_admin (uint8 src)
{
    if ((PROTO_SRC_NONE == s_owner) || (s_owner == src)) { return PROTO_OK; }
    return PROTO_ERR_BUSY;
}

static uint8 proto_perm_set (uint8 src, const proto_param_t *p)
{
    if (NULL == p) { return PROTO_ERR_ID; }

    switch (p->perm)
    {
    case P_RO:
        return PROTO_ERR_PERM;

    case P_RW_LIVE:
        /* 增益/前视/死区：任何时候可写 */
        return PROTO_OK;

    case P_RW_IDLE:
        /* 轴距/cpr/极性/限位：别人在开不许改 */
        if ((PROTO_SRC_NONE != s_owner) && (s_owner != src)) { return PROTO_ERR_BUSY; }
        /* 回放/航向测试运行中, 持有者也不许改 */
        if ((0u != route_pb_active()) || (0u != head_test_active())) { return PROTO_ERR_STATE; }
        return PROTO_OK;

    case P_CMD:
        /* 命令即驱动：必须是令牌持有者, 空闲时自动领取 */
        if (PROTO_SRC_RC == src) { return PROTO_OK; }       /* 遥控走自己的状态机，不进参数表 */
        if (0u == proto_drv_held_by(src))
        {
            if (0u == proto_drv_acquire(src)) { return PROTO_ERR_BUSY; }
        }
        proto_drv_renew(src);
        return PROTO_OK;

    default:
        return PROTO_ERR_PERM;
    }
}

/* ---- 四、唯一写入口 ---- */
uint8 proto_exec (const proto_msg_t *m, float *out)
{
    const proto_param_t *p;
    float v;
    uint8 err = PROTO_OK;

    if (NULL != out) { *out = 0.0f; }
    if (NULL == m)   { return PROTO_ERR_VAL; }

    switch (m->op)
    {
    /*--------------------------------------------------------- 急停（不检查权限）----*/
    case PROTO_OP_ESTOP:
        proto_estop(m->src, (m->val > 0.5f) ? 1u : 0u);
        break;

    case PROTO_OP_DRV_REQ:
        if (0u == proto_drv_acquire(m->src)) { err = PROTO_ERR_BUSY; }
        break;

    case PROTO_OP_DRV_REL:
        proto_drv_release(m->src);
        break;

    /*----------------------------------------------------------- 存盘/恢复默认 ----*/
    case PROTO_OP_SAVE:
        err = proto_perm_admin(m->src);
        if (PROTO_OK == err)
        {
            uint8 e = app_param_save();             /* 阻塞几十 ms：只允许主循环调用 */

            if (NULL != out) { *out = (float)e; }
            if (0u != e)     { err = PROTO_ERR_STATE; }
        }
        break;

    case PROTO_OP_LOAD_DEF:
        err = proto_perm_admin(m->src);
        if (PROTO_OK == err) { proto_param_load_defaults(); }
        break;

    case PROTO_OP_GET:
        p = proto_param_by_id(m->id);
        if (NULL == p) { err = PROTO_ERR_ID; break; }
        if (NULL != out) { *out = *(p->ptr); }
        break;

    /*----------------------------------------------------------------- 写/翻转 ----*/
    case PROTO_OP_SET:
    case PROTO_OP_TOGGLE:
        p = proto_param_by_id(m->id);
        if (NULL == p) { err = PROTO_ERR_ID; break; }

        /* P_CMD 且量程下限 >= 0 时, 写 0 视为停止/释放, 不要求驱动令牌（安全动作）;
           双极性参数（如 steer_tgt）不走此路, 否则负向指令会被当成停止。 */
        if ((P_CMD == p->perm) && (PROTO_OP_SET == m->op) &&
            (m->val < 0.5f) && (p->lo >= 0.0f))
        {
            v = (p->lo > 0.0f) ? p->lo : 0.0f;
            *(p->ptr) = v;
            steer_ctrl_sanitize_thr();
            if (NULL != out) { *out = *(p->ptr); }
            break;
        }

        err = proto_perm_set(m->src, p);
        if (PROTO_OK != err) { break; }
        if ((m->id == 76u || m->id == 77u || m->id == 88u) &&
            (ins_route_is_running() || ins_route_flash_busy() || route_pb_active()))
        { err = PROTO_ERR_STATE; break; }

        v = m->val;
        if (PROTO_OP_TOGGLE == m->op) { v = (*(p->ptr) > 0.5f) ? 0.0f : 1.0f; }
        if (!((v >= -FLT_MAX) && (v <= FLT_MAX))) { err = PROTO_ERR_VAL; break; }
        if (m->id == 88u && (v < 0.0f || v > 4.0f || (v != (float)(uint8)(v + 0.5f))))
        { err = PROTO_ERR_VAL; break; }

        /* 车端只做量程钳位, 不按 step 取整（step 供上位机交互） */
        if (v < p->lo) { v = p->lo; }
        if (v > p->hi) { v = p->hi; }

        *(p->ptr) = v;

        if (m->id == 88u)
        {
            uint8 mode = (uint8)(v + 0.5f);
            rt_route_mode = (float)mode;
            rt_revrec = (mode == RT_MODE_FZ) ? 1.0f : 0.0f;
            rt_revplay = (mode == RT_MODE_ZF || mode == RT_MODE_FZ || mode == RT_MODE_MIX_E) ? 1.0f : 0.0f;
        }
        else if (m->id == 76u || m->id == 77u)
        {
            /* 旧工具改遍历顺序时保留已选的 mix 模式 */
            if (m->id == 77u && rt_route_mode >= (float)RT_MODE_MIX_S && rt_revrec < 0.5f)
            { rt_route_mode = (rt_revplay >= 0.5f) ? (float)RT_MODE_MIX_E : (float)RT_MODE_MIX_S; }
            else if (rt_revrec >= 0.5f)
            { rt_route_mode = (rt_revplay >= 0.5f) ? (float)RT_MODE_FZ : (float)RT_MODE_MIX_S; }
            else
            { rt_route_mode = (rt_revplay >= 0.5f) ? (float)RT_MODE_ZF : (float)RT_MODE_ZZ; }
        }

        /* 交叉钳位（db/brk/izone 大小关系）只在写入处做一次 */
        steer_ctrl_sanitize_thr();

        if (NULL != out) { *out = *(p->ptr); }
        break;

    default:
        err = PROTO_ERR_ID;
        break;
    }

    return err;
}

/* ---- 五、参数元数据接口 ---- */
uint16 proto_param_count (void)
{
    return PP_COUNT;
}

const proto_param_t *proto_param_at (uint16 idx)
{
    if (idx >= PP_COUNT) { return NULL; }
    return &s_ptab[idx];
}

const proto_param_t *proto_param_by_id (uint8 id)
{
    const proto_param_t *p;
    uint16 i;

    /* id 连续时直接下标, 命令路径上避免线性查找 */
    if (id < PP_COUNT)
    {
        p = &s_ptab[id];
        if (p->id == id) { return p; }
    }

    /* 兜底：ID 不连续时线性查找一次 */
    for (i = 0u; i < PP_COUNT; i ++)
    {
        if (s_ptab[i].id == id) { return &s_ptab[i]; }
    }
    return NULL;
}

uint16 proto_param_index_of_ptr (const void *ptr)
{
    uint16 i;

    for (i = 0u; i < PP_COUNT; i ++)
    {
        if ((const void *)s_ptab[i].ptr == ptr) { return i; }
    }
    return 0xFFFFu;
}

void proto_param_load_defaults (void)
{
    uint16 i;

    for (i = 0u; i < PP_COUNT; i ++)
    {
        *(s_ptab[i].ptr) = s_ptab[i].def;
    }

    /* 改完默认值后钳一次门限关系 */
    steer_ctrl_sanitize_thr();
}

/* ---- 六、协议指纹（PING 的 ACK 回给上位机） ---- */
static uint16 proto_crc_step (uint16 crc, const uint8 *p, uint16 n)
{
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

uint16 proto_ver (void)
{
    return (uint16)PROTO_VER;
}

uint16 proto_param_table_crc (void)
{
    uint16 crc = 0xFFFFu;
    uint16 i;

    for (i = 0u; i < PP_COUNT; i ++)
    {
        uint8 buf[16];                      /* {lo, hi, def, id, perm, grp} = 16 B */

        memcpy(buf + 0,  &s_ptab[i].lo,  4);
        memcpy(buf + 4,  &s_ptab[i].hi,  4);
        memcpy(buf + 8,  &s_ptab[i].def, 4);
        buf[12] = s_ptab[i].id;
        buf[13] = s_ptab[i].perm;
        buf[14] = s_ptab[i].grp;
        buf[15] = 0u;

        crc = proto_crc_step(crc, buf, 16u);
    }
    return crc;
}

/* 通道布局指纹：通道数 + 各通道号按协议顺序做 CRC, 通道顺序被改时上位机可立即发现 */
uint16 proto_ch_layout_crc (void)
{
    static const uint16 lay[] =
    {
        (uint16)MOTOR_CH_COUNT,
        (uint16)MOTOR_CH_COUNT_CALIB,
        (uint16)CH_TARGET_L,  (uint16)CH_V_L,      (uint16)CH_U_L,
        (uint16)CH_TARGET_R,  (uint16)CH_V_R,      (uint16)CH_U_R,
        (uint16)CH_ONLINE,    (uint16)CH_SAFETY,   (uint16)CH_ENABLE_BITS,
        (uint16)CH_STEER_TARGET, (uint16)CH_STEER_DELTA, (uint16)CH_STEER_U,
        (uint16)CH_STEER_EN,  (uint16)CH_STEER_STATUS,
        (uint16)CH_RC_ENGAGE, (uint16)CH_INS_YAW,  (uint16)CH_INS_X, (uint16)CH_INS_Y,
        (uint16)CH_RT_CNT,    (uint16)CH_RT_ST,    (uint16)CH_PB_ELAT,
        (uint16)CH_TAR_YAW,   (uint16)CH_HT_SEG,
        (uint16)CT_TARGET_L,  (uint16)CT_STEER_TGT, (uint16)CT_K_Z
    };

    return proto_crc_step(0xFFFFu, (const uint8 *)lay, (uint16)sizeof(lay));
}

/* ---- 七、名字（日志/事件用, 全 ASCII 以免被 EVENT 帧截断） ---- */
const char *proto_src_name (uint8 src)
{
    switch (src)
    {
        case PROTO_SRC_NONE:     return "none";
        case PROTO_SRC_RC:       return "rc";
        case PROTO_SRC_SPI:      return "spi";
        case PROTO_SRC_MENU:     return "menu";
        case PROTO_SRC_INTERNAL: return "internal";
        default:                 return "?";
    }
}

const char *proto_op_name (uint8 op)
{
    switch (op)
    {
        case PROTO_OP_GET:      return "get";
        case PROTO_OP_SET:      return "set";
        case PROTO_OP_TOGGLE:   return "toggle";
        case PROTO_OP_SAVE:     return "save";
        case PROTO_OP_LOAD_DEF: return "load_def";
        case PROTO_OP_ESTOP:    return "estop";
        case PROTO_OP_DRV_REQ:  return "drv_req";
        case PROTO_OP_DRV_REL:  return "drv_rel";
        default:                return "?";
    }
}

const char *proto_err_name (uint8 err)
{
    switch (err)
    {
        case PROTO_OK:        return "ok";
        case PROTO_ERR_ID:    return "id";
        case PROTO_ERR_VAL:   return "val";
        case PROTO_ERR_PERM:  return "perm";
        case PROTO_ERR_BUSY:  return "busy";
        case PROTO_ERR_STATE: return "state";
        default:              return "?";
    }
}
