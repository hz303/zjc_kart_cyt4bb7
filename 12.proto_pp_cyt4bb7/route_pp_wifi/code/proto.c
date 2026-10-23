/*********************************************************************************************************************
 * 工程 11（卡丁）· 协议层实现（设计、权限模型、与笔记的偏离见 proto.h）
 *
 *  一、参数元数据（表只在 param_table.h 里定义一次，本文件展开一次）
 *  二、权限 / 驱动令牌
 *  三、唯一写入口 proto_exec()
 *  四、指纹 / 事件 / 打印
 ********************************************************************************************************************/

#include "zf_common_headfile.h"
#include <string.h>
#include <float.h>

#include "proto.h"
#include "motor_link.h"          /* MOTOR_PARAM_TABLE / CH_* / motor_link_now_ms() */
#include "app_params.h"          /* app_param_save()：SAVE 的唯一落盘实现 */
#include "spd_ctrl.h"            /* spd_target_l / spd_target_r */
#include "spd_safety.h"
#include "steer_ctrl.h"          /* steer_ctrl_sanitize_thr() / steer_ctrl_set_estop() */
#include "route_pb.h"            /* route_pb_active()：回放中禁改"几何类"参数 */
#include "head_test.h"           /* head_test_active() */

/*===================================================================================================================
 *  一、参数元数据
 *
 *  ⚠ `param_table.h` 是**唯一**定义处；`motor_link.c` / `app_params.c` / `ui_menu.c`
 *    以前各自展开一遍（三份平行数组），现在一律走本文件的 proto_param_*()。
 *=================================================================================================================*/
#define PP_ROW(id, nm, lb, lo, hi, df, un, st, pt, pm, gp)                                     \
    { (uint8)(id), (nm), (lb), (lo), (hi), (df), (un), (st), (volatile float *)(pt),           \
      (uint8)(pm), (uint8)(gp) },

static const proto_param_t s_ptab[] = { MOTOR_PARAM_TABLE(PP_ROW) };
#undef PP_ROW

#define PP_COUNT    ((uint16)(sizeof(s_ptab) / sizeof(s_ptab[0])))

/* 编译期护栏：表项数超过 PROTO_PARAM_MAX（app_params 的载荷数组按它开）就编不过。
   比"上电跑飞、存参数时越界写栈"早一万倍发现。 */
typedef char pp_fits_in_proto_max_[(PP_COUNT <= PROTO_PARAM_MAX) ? 1 : -1];

/*===================================================================================================================
 *  二、驱动令牌 / 事件
 *=================================================================================================================*/
static uint8  s_owner      = PROTO_SRC_NONE;
static uint8  s_estop_mask = 0u;        /* 按来源锁存的急停位（bit0 RC / bit1 SPI / bit2 MENU / bit3 INTERNAL）*/
static uint32 s_lease_last = 0u;
static uint16 s_lease_ms   = 0u;
static void (*s_evt)(const char *text) = NULL;

static void proto_event (const char *text)
{
    if (NULL != s_evt) { s_evt(text); }
}

/* 各来源的默认租期。为什么不是笔记里的 300 ms：
 *   `spd_safety` 的失联门 (lost_ms=600) 已经把"链路真断"挡住了；这里的租期是
 *   **驱动权**的租期，只用于"别的设备什么时候可以接管"，给宽一点不会降低安全性，
 *   但能避免"上位机心跳偶尔慢一拍 -> 屏幕把方向盘抢走"这种误动作。
 *   遥控给得最短：它由 SBUS 帧续租（每帧都来），600 ms 足够。 */
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
        /* 已经是自己，直接续租 */
    }
    else if (PROTO_SRC_RC == src)
    {
        /* ★ 遥控抢占：人在这儿、看得见车。上位机必须知道（否则看波形不动会以为程序挂了）。
           这是笔记 §7.4 那条规则，也是 remote_ctrl.c 原来最缺的一块。 */
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

    /* ---- 租约到期：释放令牌 ----
       ⚠ **不无条件把目标清零**。上位机拖一次滑条之后可能几十秒不再发参数
         （稳定巡航），如果到期就清零，车会在赛道中间自己停下 —— 这是绝对不能有的行为。
         真正的"人走了"由 spd_safety 的失联门兜底（链路 600 ms 没帧 -> 输出钉 0）。
         这里只在"链路确实已经不在线"时顺手清一次目标，作为双保险。 */
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

    /* ★★ 急停是**按来源锁存**的（每个来源一位），任一位置位就停机。
       为什么不能"谁调用就写谁的 0"：`remote_ctrl_apply()` 每圈都在推它自己的 CH5 状态，
       如果它推 0 就把全局急停清了，那么上位机发的 0x09 会在 **1 ms 后**被遥控悄悄解除 ——
       按钮看起来完全没反应，而且不报错。这个坑必须在协议层堵死。 */
    if ((src < PROTO_SRC_RC) || (src > PROTO_SRC_INTERNAL)) { src = PROTO_SRC_INTERNAL; }
    bit = (uint8)(1u << (src - 1u));
    was = s_estop_mask;

    if (0u != on)
    {
        if (0u == s_estop_mask)
        {
            /* 急停剥夺令牌：谁都不能在急停期间"继续持有方向盘" */
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

/*===================================================================================================================
 *  三、权限
 *=================================================================================================================*/
/* 管理类操作（存 Flash / 恢复默认）：只有"没人驱动"或"自己是驱动者"时允许 */
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
        /* 增益/前视/死区：任何时候都能改（无扰切换已经做了，改它本身不产生台阶） */
        return PROTO_OK;

    case P_RW_IDLE:
        /* 轴距/cpr/极性/限位：别人在开就不许改 —— 会把几何/尺度改错，而且不报错 */
        if ((PROTO_SRC_NONE != s_owner) && (s_owner != src)) { return PROTO_ERR_BUSY; }
        /* 回放/航向测试正在跑的时候，连持有者也不许改（改轴距会把回放算错） */
        if ((0u != route_pb_active()) || (0u != head_test_active())) { return PROTO_ERR_STATE; }
        return PROTO_OK;

    case P_CMD:
        /* 命令 = 驱动：必须是令牌持有者。空闲时**自动领取**（第一次拖目标就拿到方向盘） */
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

/*===================================================================================================================
 *  四、唯一写入口
 *=================================================================================================================*/
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

    /*----------------------------------------------------------------- 驱动权 ----*/
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

    /*--------------------------------------------------------------------- 读 ----*/
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

        /* ★★ 2026-10-20：**"停"永远允许**。
           P_CMD 里写 0 一律视为"停止/释放"，不要求驱动令牌 —— 否则遥控抢了令牌之后，
           屏幕上按 Follow OFF 会回 REFUSE busy，现场就成了"车停不下来"。
           安全动作不该被权限挡住（与急停同一条道理）。

           ⚠⚠ 2026-10-22 修正：这条特例**只能用在非负量程的开关量上**（pb_en / rt_rec / ht_mode…）。
           对**双极性**参数（`steer_tgt` −27~+27）是致命的：负的打角（左转）也 < 0.5
           ⇒ 被当成"停止"、写进 `p->lo` 再因 `p->lo<0` 变成 **0** ⇒ **左转指令永远写不进去**。
           现场症状：App/上位机写负舵角没反应，舵机一直在 0（圆周测试实测 δ=−0.03° 就是它）。
           判据很干脆：量程下限 `p->lo >= 0` 才是"0/1 开关"，才有"写 0 = 停止"的语义。 */
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

        v = m->val;
        if (PROTO_OP_TOGGLE == m->op) { v = (*(p->ptr) > 0.5f) ? 0.0f : 1.0f; }
        if (!((v >= -FLT_MAX) && (v <= FLT_MAX))) { err = PROTO_ERR_VAL; break; }

        /* ⚠ 车端只做**量程钳位**，不按 step 取整：step 是上位机交互用的 */
        if (v < p->lo) { v = p->lo; }
        if (v > p->hi) { v = p->hi; }

        *(p->ptr) = v;

        /* ★ 交叉钳位（db/brk/izone 的大小关系）只有一处实现。放在"写入处"而不是控制拍里：
           否则"上位机显示的值"和"车上实跑的值"会不一致。 */
        steer_ctrl_sanitize_thr();

        if (NULL != out) { *out = *(p->ptr); }
        break;

    default:
        err = PROTO_ERR_ID;
        break;
    }

    return err;
}

/*===================================================================================================================
 *  五、参数元数据接口
 *=================================================================================================================*/
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

    /* 正常情况：id 连续，直接下标（O(1)，命令路径上别做 76 次循环） */
    if (id < PP_COUNT)
    {
        p = &s_ptab[id];
        if (p->id == id) { return p; }
    }

    /* 兜底：万一以后 ID 不连续（留了空号/插队），线性查一次 */
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

    /* 默认值本身是自洽的；这一行是"以后谁改宏改出非法组合"时的最后一道闸 */
    steer_ctrl_sanitize_thr();
}

/*===================================================================================================================
 *  六、协议指纹（PING 的 ACK 里发给上位机，连上就能发现版本/表不匹配）
 *=================================================================================================================*/
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

/* 通道布局指纹：把"通道数 + 每个通道的编号"按协议顺序串起来做 CRC。
 * 固件里没有通道**名字**（名字在上位机），但"第 16 号通道是什么"这个约定在固件里 ——
 * 谁改乱了 CH_* 的顺序，这个值就变，上位机连上时能立刻发现，而不是画出一张错位的图。 */
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

/*===================================================================================================================
 *  七、名字（日志/事件用；全部 ASCII，避免 UTF-8 被 EVENT 帧截断）
 *=================================================================================================================*/
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
