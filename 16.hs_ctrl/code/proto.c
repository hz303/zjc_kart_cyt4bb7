/*********************************************************************************************************************
 * 协议层实现
 *  一、参数元数据  二、驱动令牌与事件  三、权限  四、唯一写入口  五、元数据接口  六、指纹  七、名字
 ********************************************************************************************************************/

#include "zf_common_headfile.h"
#include <string.h>
#include <float.h>
#include "ins_route.h"
#include "route_app.h"

#include "proto.h"
#include "motor_link.h"
#include "app_params.h"
#include "spd_ctrl.h"
#include "spd_safety.h"
#include "steer_ctrl.h"
#include "route_pb.h"
#include "head_test.h"

/* ---- 一、参数元数据, 表只在 param_table.h 定义一次 ---- */
#define PP_ROW(id, nm, lb, lo, hi, df, un, st, pt, pm, gp)                                     \
    { (uint8)(id), (nm), (lb), (lo), (hi), (df), (un), (st), (volatile float *)(pt),           \
      (uint8)(pm), (uint8)(gp) },

static const proto_param_t s_ptab[] = { MOTOR_PARAM_TABLE(PP_ROW) };
#undef PP_ROW

#define PP_COUNT    ((uint16)(sizeof(s_ptab) / sizeof(s_ptab[0])))

/* 编译期护栏: 表项数超过 PROTO_PARAM_MAX 就编不过, app_params 载荷按它开 */
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

/* 各来源默认租期 ms, 只决定别人何时可接管该来源; 链路真断由 spd_safety 的
   lost_ms 兜底, 遥控由 SBUS 每帧续租故取 600 最短 */
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
        /* 已持有, 直接续租 */
    }
    else if (PROTO_SRC_RC == src)
    {
        /* 遥控抢占, 人已在现场; 必须发 EVENT, 否则上位机看波形不动会误判 */
        proto_event("RC took over drive");
        s_owner = src;
    }
    else
    {
        return 0u;                  /* 被别人占着, 只能等租约到期 */
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

    /* 租约到期只释放令牌, 不清零目标: 上位机拖一次滑条后可能几十秒不再发参数,
       清零会让车在赛道中间停下; 真正的失联由 spd_safety 兜底, 这里只在链路
       确实离线时补一次清零 */
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

    /* 急停按来源锁存, 每来源一位, 任一位置位即停机; 若改成谁调用写谁的值,
       遥控每圈推一次 CH5 会把上位机发的急停静默解除 */
    if ((src < PROTO_SRC_RC) || (src > PROTO_SRC_INTERNAL)) { src = PROTO_SRC_INTERNAL; }
    bit = (uint8)(1u << (src - 1u));
    was = s_estop_mask;

    if (0u != on)
    {
        if (0u == s_estop_mask)
        {
            /* 急停剥夺令牌, 急停期间谁都不能继续持有方向盘 */
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
/* 管理类操作, 保存 Flash 与恢复默认, 仅在无人驱动或自己是驱动者时允许 */
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
        /* 增益/前视/死区, 任何时候可改; 无扰切换在控制拍内做, 改它不产生台阶 */
        return PROTO_OK;

    case P_RW_IDLE:
        /* 轴距/cpr/极性/限位, 别人在开不许改, 会算错几何与尺度 */
        if ((PROTO_SRC_NONE != s_owner) && (s_owner != src)) { return PROTO_ERR_BUSY; }
        /* 回放或航向测试运行中, 持有者也不许改 */
        if ((0u != route_pb_active()) || (0u != head_test_active())) { return PROTO_ERR_STATE; }
        return PROTO_OK;

    case P_CMD:
        /* 命令即驱动, 必须是令牌持有者; 空闲时自动领取, 第一次拖目标就拿到方向盘 */
        if (PROTO_SRC_RC == src) { return PROTO_OK; }       /* 遥控用自己的状态机 */
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
    case PROTO_OP_ESTOP:
        proto_estop(m->src, (m->val > 0.5f) ? 1u : 0u);
        break;

    case PROTO_OP_DRV_REQ:
        if (0u == proto_drv_acquire(m->src)) { err = PROTO_ERR_BUSY; }
        break;

    case PROTO_OP_DRV_REL:
        proto_drv_release(m->src);
        break;

    case PROTO_OP_SAVE:
        err = proto_perm_admin(m->src);
        if (PROTO_OK == err)
        {
            uint8 e = app_param_save();             /* 阻塞几十 ms, 只允许主循环调用 */

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

    case PROTO_OP_SET:
    case PROTO_OP_TOGGLE:
        p = proto_param_by_id(m->id);
        if (NULL == p) { err = PROTO_ERR_ID; break; }

        /* P_CMD 写 0 视为停止或释放, 不要求令牌, 否则遥控持令牌时上位机按停止会回 BUSY
           该特例只适用于非负量程开关量; 双极性参数如 steer_tgt 的负值也小于 0.5,
           会被当成停止, 故以量程下限 p->lo >= 0 作为判据 */
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

        /* 车端只做量程钳位, 不按 step 取整, step 仅给上位机交互 */
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
            /* 旧工具只改行进方向时保留已选的混合模式; 缺的第四种均匀模式用 mix_s */
            if (m->id == 77u && rt_route_mode >= (float)RT_MODE_MIX_S && rt_revrec < 0.5f)
            { rt_route_mode = (rt_revplay >= 0.5f) ? (float)RT_MODE_MIX_E : (float)RT_MODE_MIX_S; }
            else if (rt_revrec >= 0.5f)
            { rt_route_mode = (rt_revplay >= 0.5f) ? (float)RT_MODE_FZ : (float)RT_MODE_MIX_S; }
            else
            { rt_route_mode = (rt_revplay >= 0.5f) ? (float)RT_MODE_ZF : (float)RT_MODE_ZZ; }
        }

        /* 交叉钳位 db/brk/izone 的大小关系只在写入处做一次, 保证上位机显示值与实跑值一致 */
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

    /* id 连续时直接下标, O(1); 命令路径上不走线性循环 */
    if (id < PP_COUNT)
    {
        p = &s_ptab[id];
        if (p->id == id) { return p; }
    }

    /* 兜底: ID 留有空号时线性查一次 */
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

    /* 默认值自洽, 此处再钳一次, 防模块宏改出非法组合 */
    steer_ctrl_sanitize_thr();
}

/* ---- 六、协议指纹, 随 PING 的 ACK 发给上位机 ---- */
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

/* 通道布局指纹: 通道数 + 各通道编号按协议顺序做 CRC, CH_* 顺序一变 CRC 就变 */
uint16 proto_ch_layout_crc (void)
{
    static const uint16 lay[] =
    {
// >>> GEN:TEL_CH —— 由 telemetry/gen_tel.py 从 tel_channels.def 生成，勿手改 <<<
        /* 各档通道数 + 每档通道编号下标, 任何一档顺序变了 CRC 都变 */
        (uint16)MOTOR_CH_COUNT_ROUTE, (uint16)MOTOR_CH_COUNT_CALIB, (uint16)MOTOR_CH_COUNT_FULL, (uint16)MOTOR_CH_COUNT_TRAIN,
        (uint16)RCH_U_L, (uint16)RCH_V_L, (uint16)RCH_TARGET_L, (uint16)RCH_U_R, (uint16)RCH_V_R, (uint16)RCH_TARGET_R,
        (uint16)RCH_STEER_TARGET, (uint16)RCH_STEER_DELTA, (uint16)RCH_STEER_U, (uint16)RCH_K_V, (uint16)RCH_K_Z, (uint16)RCH_INS_YAW,
        (uint16)RCH_TAR_YAW, (uint16)RCH_INS_GZ, (uint16)RCH_PB_ELAT, (uint16)RCH_RC_ENGAGE, (uint16)RCH_STEER_EN, (uint16)RCH_STEER_STATUS,
        (uint16)RCH_RT_CNT, (uint16)RCH_RT_ST, (uint16)RCH_TICK_MS, (uint16)RCH_INS_STATIC,
        (uint16)CT_TARGET_L, (uint16)CT_V_L, (uint16)CT_U_L, (uint16)CT_TARGET_R, (uint16)CT_V_R, (uint16)CT_U_R,
        (uint16)CT_STEER_TGT, (uint16)CT_STEER_DELTA, (uint16)CT_STEER_U, (uint16)CT_ONLINE, (uint16)CT_ENABLE, (uint16)CT_SAFETY,
        (uint16)CT_STEER_EN, (uint16)CT_STEER_ST, (uint16)CT_OL_EN, (uint16)CT_RESV0, (uint16)CT_K_V, (uint16)CT_K_Z,
        (uint16)CH_TARGET_L, (uint16)CH_V_L, (uint16)CH_U_L, (uint16)CH_TARGET_R, (uint16)CH_V_R, (uint16)CH_U_R,
        (uint16)CH_UFF_L, (uint16)CH_UFF_R, (uint16)CH_Z_L, (uint16)CH_Z_R, (uint16)CH_POS_L, (uint16)CH_POS_R,
        (uint16)CH_ONLINE, (uint16)CH_RX_AGE_MS, (uint16)CH_ENABLE_BITS, (uint16)CH_SAFETY, (uint16)CH_STEER_TARGET, (uint16)CH_STEER_DELTA,
        (uint16)CH_STEER_U, (uint16)CH_STEER_EN, (uint16)CH_STEER_STATUS, (uint16)CH_RC_ENGAGE, (uint16)CH_RC_CH1, (uint16)CH_RC_CH2,
        (uint16)CH_RC_FRAMES, (uint16)CH_K_V, (uint16)CH_K_Z, (uint16)CH_INS_YAW, (uint16)CH_INS_GZ, (uint16)CH_INS_X,
        (uint16)CH_INS_Y, (uint16)CH_INS_DIST, (uint16)CH_INS_FLAGS, (uint16)CH_RT_CNT, (uint16)CH_RT_ST, (uint16)CH_PB_ELAT,
        (uint16)CH_TAR_YAW, (uint16)CH_HT_SEG, (uint16)CH_TICK_MS, (uint16)CH_INS_FRZ, (uint16)CH_INS_STATIC, (uint16)CH_PB_BEST,
        (uint16)CH_PB_FI, (uint16)CH_PB_KREF, (uint16)CH_PB_WHY,
        (uint16)TRCH_STEER_TARGET, (uint16)TRCH_STEER_DELTA, (uint16)TRCH_STEER_U, (uint16)TRCH_STEER_STATUS, (uint16)TRCH_TARGET_L, (uint16)TRCH_V_L,
        (uint16)TRCH_U_L, (uint16)TRCH_TARGET_R, (uint16)TRCH_V_R, (uint16)TRCH_U_R, (uint16)TRCH_INS_GZ, (uint16)TRCH_INS_STATIC,
        (uint16)TRCH_ENABLE_BITS, (uint16)TRCH_SAFETY, (uint16)TRCH_RC_ENGAGE, (uint16)TRCH_TICK_MS,
// <<< GEN:TEL_CH <<<
    };

    return proto_crc_step(0xFFFFu, (const uint8 *)lay, (uint16)sizeof(lay));
}

/* ---- 七、名字, 日志与事件用, 全 ASCII 以免 EVENT 帧截断 UTF-8 ---- */
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
