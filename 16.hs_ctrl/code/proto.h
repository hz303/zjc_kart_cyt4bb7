/*********************************************************************************************************************
 * 协议层 / 权限层 / 唯一写入口
 *
 * 三种传输各自解帧（遥控 SBUS 帧 / WiFi-SPI 帧 / 菜单按键），到本层统一成
 * proto_msg_t {src, op, id, val, token}，经 perm_check 后由 cmd_apply 写入，为全工程唯一写入点。
 * 命令同步执行，WiFi 需立即回 ACK/NAK；急停不入权限路径，直接置位。
 * 权限矩阵，行为参数权限，列为驱动令牌归属
 *   权限        无持有者    自己是持有者   别人持有
 *   P_RO          可           可            可     读永远可以
 *   P_RW_LIVE     可           可            可     增益/前视, 任何时候可改
 *   P_RW_IDLE     可           可            否     轴距/cpr/极性/限位, 别人在开不许改
 *   P_CMD         可(自动取)   可            否     目标/使能/回放, 仅持有者可写
 * 驱动权为带租期令牌 drv_lease, 持有者续租, 到期自动释放, 掉线无需猜超时
 *   遥控 ON/STANDBY 抢占（优先级最高）并发一条 EVENT, 回放 route_pb 是 SPI 持有者的子模式
 *   急停不属于权限, 任何来源任何时刻可触发, 立即生效, 不可被本地接管替换
 * PING 的 ACK 在 {token, cmd, err} 之后扩展 8 字节: proto_ver / 参数表 CRC /
 *   通道布局 CRC / 参数行数, 供上位机连上即校验固件是否匹配
 ********************************************************************************************************************/

#ifndef _PROTO_H
#define _PROTO_H

#include "zf_common_headfile.h"

/* ---- 一、版本与规模 ---- */
#define PROTO_VER           (2u)        /* 协议版本, 2 = 带权限/令牌/指纹 */
#define PROTO_PARAM_MAX     (96u)       /* 参数表容量上限, app_params 载荷按它开 */

/* ---- 二、来源, 0=无 1=遥控SBUS(UART_4) 2=WiFi-SPI 3=面板菜单 4=车端内部 ---- */
#define PROTO_SRC_NONE      (0u)
#define PROTO_SRC_RC        (1u)
#define PROTO_SRC_SPI       (2u)
#define PROTO_SRC_MENU      (3u)
#define PROTO_SRC_INTERNAL  (4u)

/* ---- 三、统一操作码, 1~8 依次为 GET/SET/TOGGLE/SAVE/LOAD_DEF/ESTOP/DRV_REQ/DRV_REL
 *  SET 写参数, P_CMD 项即驱动; TOGGLE 为 0/1 翻转; SAVE 写 Flash 阻塞几十 ms;
 *  LOAD_DEF 只写 RAM; ESTOP 立即生效且不检查权限 ---- */
#define PROTO_OP_GET        (1u)
#define PROTO_OP_SET        (2u)
#define PROTO_OP_TOGGLE     (3u)
#define PROTO_OP_SAVE       (4u)
#define PROTO_OP_LOAD_DEF   (5u)
#define PROTO_OP_ESTOP      (6u)
#define PROTO_OP_DRV_REQ    (7u)
#define PROTO_OP_DRV_REL    (8u)

/* ---- 四、结果码, WiFi 侧直接映射成 NAK 的 err 字节, 1=ID 2=VAL 3=PERM 4=BUSY 5=STATE ---- */
#define PROTO_OK            (0u)
#define PROTO_ERR_ID        (1u)
#define PROTO_ERR_VAL       (2u)
#define PROTO_ERR_PERM      (3u)
#define PROTO_ERR_BUSY      (4u)
#define PROTO_ERR_STATE     (5u)

/* ---- 五、参数权限与分组, param_table.h 每行末两个字段 ---- */
#define P_RO            (0u)        /* 只读, 版本与状态 */
#define P_RW_LIVE       (1u)        /* 行驶中可写, 增益/前视/死区 */
#define P_RW_IDLE       (2u)        /* 仅无人驱动时可写, 几何/尺度/限位 */
#define P_CMD           (3u)        /* 命令, 仅令牌持有者可写, 不持久化 */

/* 分组 0~7 依次为 G_SPD 速度环/编码器/电机极性, G_SYS 保护/链路/遥测, G_STEER 转向,
 * G_RC 遥控, G_INS 惯导, G_RT 路表, G_PB 回放, G_TEST 航向测试 */
#define G_SPD           (0u)
#define G_SYS           (1u)
#define G_STEER         (2u)
#define G_RC            (3u)
#define G_INS           (4u)
#define G_RT            (5u)
#define G_PB            (6u)
#define G_TEST          (7u)

/* ---- 六、参数元数据, 与 param_table.h 的 11 个字段一一对应 ---- */
typedef struct
{
    uint8           id;
    const char     *name;
    const char     *label;
    float           lo;
    float           hi;
    float           def;
    const char     *unit;
    float           step;
    volatile float *ptr;
    uint8           perm;       /* P_RO / P_RW_LIVE / P_RW_IDLE / P_CMD */
    uint8           grp;        /* G_*                                  */
} proto_param_t;

/* 一条归一化后的命令, 三种传输解帧后都填这个结构 */
typedef struct
{
    uint8  src;         /* PROTO_SRC_*                        */
    uint8  op;          /* PROTO_OP_*                         */
    uint8  id;          /* 参数 id, GET/SET 用                */
    float  val;         /* 目标值 / ESTOP 开关 / TOGGLE 忽略  */
    uint16 token;       /* 需要 ack 时回带, 仅 WiFi 用        */
} proto_msg_t;

/* ---- 七、接口 ---- */
void    proto_init              (void);

/* 唯一写入口, 返回 PROTO_*; out 可为 NULL, 回读生效值:
 *   GET 当前值, SET 钳位后真正写入的值, SAVE 底层 Flash 错误码, 0 = 成功
 * 只允许在主循环调用, 会写 Flash 与执行器量 */
uint8   proto_exec              (const proto_msg_t *m, float *out);

/* ---- 参数元数据, 全工程只此一份 ---- */
uint16  proto_param_count       (void);
const proto_param_t *proto_param_at   (uint16 idx);
const proto_param_t *proto_param_by_id(uint8 id);
uint16  proto_param_index_of_ptr(const void *ptr);   /* 0xFFFF = 没找到 */
void    proto_param_load_defaults (void);            /* 只写 RAM, 不写 Flash */

/* ---- 协议指纹, PING 的 ACK 里发给上位机 ---- */
uint16  proto_ver               (void);
uint16  proto_param_table_crc   (void);   /* 对每行 {id, lo, hi, def, perm, grp} 做 CRC16 */
uint16  proto_ch_layout_crc     (void);   /* 对遥测通道号/通道数做 CRC16             */

/* ---- 驱动令牌, 带租期 ---- */
uint8   proto_drv_owner         (void);              /* PROTO_SRC_*, NONE = 空闲        */
uint8   proto_drv_held_by       (uint8 src);         /* 1 = 令牌在该来源手里            */
uint8   proto_drv_acquire       (uint8 src);         /* 1 = 拿到, 遥控可抢占            */
void    proto_drv_renew         (uint8 src);         /* 仅持有者调用有效: 续租          */
void    proto_drv_release       (uint8 src);         /* 仅持有者调用有效: 立刻释放       */
/* 主循环每圈一次, 租约到期释放令牌, 链路真断了才顺带清零目标 */
void    proto_tick              (void);

/* ---- 急停, 全局, 不属于权限 ---- */
void    proto_estop             (uint8 src, uint8 on);

/* ---- 事件出口, 车端无屏时通知上位机驱动权被抢 ---- */
void    proto_set_event_sink    (void (*fn)(const char *text));

/* ---- 打印与诊断用字符串 ---- */
const char *proto_src_name      (uint8 src);
const char *proto_op_name       (uint8 op);
const char *proto_err_name      (uint8 err);

#endif
