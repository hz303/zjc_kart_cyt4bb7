/*********************************************************************************************************************
 * 协议层与权限层
 * 三种传输（遥控 SBUS / WiFi-SPI / 面板菜单）各自解帧后统一填成 proto_msg_t，
 * 由 proto_exec() 按参数权限检查后写入；proto_exec() 是唯一写入点。
 ********************************************************************************************************************/

#ifndef _PROTO_H
#define _PROTO_H

#include "zf_common_headfile.h"

/* ---- 一、版本与规模 ---- */
#define PROTO_VER           (2u)        /* 协议版本, 2 = 含权限/令牌/指纹 */
#define PROTO_PARAM_MAX     (96u)       /* 参数表上限, 载荷数组按它开     */

/* ---- 二、命令来源 ---- */
#define PROTO_SRC_NONE      (0u)        /* 无（令牌空闲）                */
#define PROTO_SRC_RC        (1u)        /* 遥控 SBUS（UART_4）           */
#define PROTO_SRC_SPI       (2u)        /* WiFi-SPI 上位机               */
#define PROTO_SRC_MENU      (3u)        /* 面板菜单（4 键 + IPS200）     */
#define PROTO_SRC_INTERNAL  (4u)        /* 车端自己（初始化/安全路径）   */

/* ---- 三、统一操作码 ---- */
#define PROTO_OP_GET        (1u)
#define PROTO_OP_SET        (2u)        /* 写参数（P_CMD 即驱动）        */
#define PROTO_OP_TOGGLE     (3u)
#define PROTO_OP_SAVE       (4u)        /* 参数写 Flash, 阻塞几十 ms     */
#define PROTO_OP_LOAD_DEF   (5u)        /* 恢复源码默认值, 只写 RAM      */
#define PROTO_OP_ESTOP      (6u)        /* 急停, 不检查权限              */
#define PROTO_OP_DRV_REQ    (7u)        /* 请求驱动权（拿令牌）          */
#define PROTO_OP_DRV_REL    (8u)

/* ---- 四、结果码（WiFi 侧映射为 NAK 的 err 字节） ---- */
#define PROTO_OK            (0u)
#define PROTO_ERR_ID        (1u)        /* 参数号/操作码非法       */
#define PROTO_ERR_VAL       (2u)        /* 数值非法（NaN/Inf）     */
#define PROTO_ERR_PERM      (3u)        /* 该类参数不允许这样写    */
#define PROTO_ERR_BUSY      (4u)        /* 驱动权在别人手里        */
#define PROTO_ERR_STATE     (5u)        /* 状态不允许              */

/* ---- 五、参数权限 / 分组 ----
 * P_RO 只读;  P_RW_LIVE 行驶中可写（增益/前视/死区）
 * P_RW_IDLE 仅无人驱动时可写（轴距/cpr/极性/软限位/遥测周期）
 * P_CMD 命令（目标/使能/录制/回放/测试）, 仅令牌持有者可写且不持久化
 */
#define P_RO            (0u)
#define P_RW_LIVE       (1u)
#define P_RW_IDLE       (2u)
#define P_CMD           (3u)

#define G_SPD           (0u)        /* 速度环 / 编码器 / 电机极性 */
#define G_SYS           (1u)        /* 保护 / 链路 / 遥测         */
#define G_STEER         (2u)        /* 转向位置环（内环）         */
#define G_RC            (3u)
#define G_INS           (4u)
#define G_RT            (5u)
#define G_PB            (6u)        /* 回放（Pure Pursuit 外环）  */
#define G_TEST          (7u)

/* ---- 六、参数元数据（与 param_table.h 的 X(...) 11 个字段一一对应） ---- */
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

typedef struct
{
    uint8  src;         /* PROTO_SRC_*        */
    uint8  op;          /* PROTO_OP_*         */
    uint8  id;          /* 参数 id（GET/SET） */
    float  val;         /* 目标值 / ESTOP 开关 / TOGGLE 忽略 */
    uint16 token;       /* 需要 ack 时回带（仅 WiFi 用）      */
} proto_msg_t;

/* ---- 七、接口 ---- */
void    proto_init              (void);

/* 唯一写入口。返回 PROTO_*; out 非 NULL 时回读生效值：
 *   GET -> 当前值;  SET -> 钳位后真正写入的值;  SAVE -> Flash 错误码（0 = 成功）
 * 只允许在主循环调用（会写 Flash、会写执行器量）。 */
uint8   proto_exec              (const proto_msg_t *m, float *out);

/* ---- 参数元数据（全工程只此一份, motor_link/app_params/ui_menu 都从这里取） ---- */
uint16  proto_param_count       (void);
const proto_param_t *proto_param_at   (uint16 idx);
const proto_param_t *proto_param_by_id(uint8 id);
uint16  proto_param_index_of_ptr(const void *ptr);   /* 0xFFFF = 没找到 */
void    proto_param_load_defaults (void);            /* 只写 RAM, 不写 Flash */

/* ---- 协议指纹（PING 的 ACK 里发给上位机） ---- */
uint16  proto_ver               (void);
uint16  proto_param_table_crc   (void);   /* 对每行 {id, lo, hi, def, perm, grp} 做 CRC16 */
uint16  proto_ch_layout_crc     (void);   /* 对遥测通道号/通道数做 CRC16             */

/* ---- 驱动令牌（带租期） ---- */
uint8   proto_drv_owner         (void);              /* PROTO_SRC_*, NONE = 空闲    */
uint8   proto_drv_held_by       (uint8 src);
uint8   proto_drv_acquire       (uint8 src);         /* 1 = 拿到（遥控可抢占）      */
void    proto_drv_renew         (uint8 src);         /* 仅持有者调用有效：续租      */
void    proto_drv_release       (uint8 src);         /* 仅持有者调用有效：立即释放  */
void    proto_tick              (void);

/* ---- 急停（全局, 不属于权限） ---- */
void    proto_estop             (uint8 src, uint8 on);

/* ---- 事件出口：驱动权被抢占时通知上位机 ---- */
void    proto_set_event_sink    (void (*fn)(const char *text));

/* ---- 只给打印/诊断用的字符串 ---- */
const char *proto_src_name      (uint8 src);
const char *proto_op_name       (uint8 op);
const char *proto_err_name      (uint8 err);

#endif
