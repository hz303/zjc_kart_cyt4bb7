/* 工程 11（卡丁）· **协议层 / 权限层 / 唯一写入口**
 * 遥控 SBUS ──┐                        ┌─> spd_target_* / steer.target_delta   （直接写）
 * WiFi-SPI  ──┼─> 各自解析、各自钳位 ──┼─> 参数表 *ptr                          （直接写）
 * 面板菜单  ──┘                        └─> pb_en / rt_rec / ...                （直接写）
 * · 想给"改参数"开权限，就必然把"开车"一起放开 —— 因为两者走同一条通路；
 * · 谁能开车没有建模：遥控在开车时，上位机拖一根滑条就能同时改目标；
 * · 存 Flash 靠 `ap_is_cmd()` 手工黑名单；加一项参数要记得加一行；
 * · 三处写入的钳位/校验各写一遍，行为慢慢分叉。
 * 本层把这件事收成一条：
 * 三种传输 ──> proto_exec(proto_msg_t) ──> perm_check ──> cmd_apply（**唯一写入点**）
 * `proto_msg_t` 是**语义层**的统一命令字，不是字节格式：
 * 字节格式各传输自己保留（SBUS 25 B / WiFi 15 B / 菜单按键），
 * 到了这里一律变成 {src, op, id, val, token}。
 * （WiFi 轮询 / 菜单 / 遥控）**全部跑在主循环里**，没有中断写者；而 WiFi 必须
 * **同步**返回 ACK/NAK（告诉上位机"这一条到底成没成"）。入队只会把 ACK 变成
 * "下一条命令再回上一条"，且要额外的 token 关联表。⇒ 直接同步执行 `proto_exec()`，
 * "唯一写入点"的目的同样达到。**急停仍然不入任何路径，直接置位（见 proto_estop）**。
 * ② **PING 的 ACK 扩展 8 字节**（proto_ver / 参数表 CRC / 通道布局 CRC / 参数行数）。
 * 前 4 字节仍是 {token, cmd, err}，老上位机按原偏移读不受影响；新上位机多读 8 字节
 * 就能在**连上的那一刻**发现"固件/上位机版本不匹配"，而不是等到图线错位。
 * 参数类别       无持有者   自己是持有者   别人持有
 * · 驱动权是一个**带租期的令牌**（drv_lease）：谁知道自己在驱动，谁就续租；
 * 到期自动释放，别的设备立刻可以接管 —— 掉线不用"猜多久算掉"。
 * · 遥控 ON/STANDBY **抢占**（人在这儿、看得见车，优先级最高），并给上位机发一条 EVENT。
 * · 回放（route_pb）是 SPI 持有者的**子模式**：跑的时候自己续 SPI 的租，跑完释放。
 * · 急停**不属于权限**：任何来源、任何时刻都能触发，立即生效、不检查、不可被本地接管替换。
 * ---- 谁能写什么（传输侧只做"解帧"，语义全在这里）----------------------------------------
 * WiFi-SPI : motor_link.c 的 MOTOR_CMD_*  -> PROTO_OP_*
 * 菜单     : ui_menu.c 的 9 项           -> PROTO_OP_*
 * 遥控     : remote_ctrl.c（状态机）      -> PROTO_OP_DRV_REQ/ESTOP + 直接写目标（它就是持有者）
 */

#ifndef _PROTO_H
#define _PROTO_H

#include "zf_common_headfile.h"

/* 一、版本与规模 */
#define PROTO_VER           (2u)
#define PROTO_PARAM_MAX     (96u)       /* 参数表上限（当前 76 项，留余量；app_params 的载荷按它开） */

/* 二、来源（谁能发命令） */
#define PROTO_SRC_NONE      (0u)        /* 无（驱动令牌的"空闲"） */
#define PROTO_SRC_RC        (1u)        /* 遥控 SBUS（UART_4） */
#define PROTO_SRC_SPI       (2u)        /* WiFi-SPI 上位机 */
#define PROTO_SRC_MENU      (3u)        /* 面板菜单（4 键 + IPS200） */
#define PROTO_SRC_INTERNAL  (4u)        /* 车端自己（初始化/安全路径） */

/* 三、统一操作码 */
#define PROTO_OP_GET        (1u)        /* 读一个参数（或命令） */
#define PROTO_OP_SET        (2u)        /* 写一个参数（P_CMD 即"驱动"） */
#define PROTO_OP_TOGGLE     (3u)        /* 0/1 翻转（P_CMD 的开关量） */
#define PROTO_OP_SAVE       (4u)        /* 参数写 Flash（阻塞几十 ms） */
#define PROTO_OP_LOAD_DEF   (5u)        /* 恢复源码默认值（只写 RAM） */
#define PROTO_OP_ESTOP      (6u)        /* 急停：立即生效，不检查权限 */
#define PROTO_OP_DRV_REQ    (7u)        /* 请求驱动权（拿令牌） */
#define PROTO_OP_DRV_REL    (8u)        /* 释放驱动权 */

/* 四、结果码（WiFi 侧直接映射成 NAK 的 err 字节） */
#define PROTO_OK            (0u)
#define PROTO_ERR_ID        (1u)        /* 参数号/操作码非法 */
#define PROTO_ERR_VAL       (2u)        /* 数值非法（NaN/Inf） */
#define PROTO_ERR_PERM      (3u)        /* 该类参数不允许这样写 */
#define PROTO_ERR_BUSY      (4u)        /* 驱动权在别人手里 */
#define PROTO_ERR_STATE     (5u)        /* 状态不允许（如写 Flash 失败） */

/* 五、参数权限 / 分组（`param_table.h` 每一行最后两个字段用的就是这两个宏）
 * 权限四档（与 proto.c 的 perm_check 一一对应）：
 * P_RO       只读（版本/状态这类，谁都不能写）
 * P_RW_LIVE  行驶中随时可写：增益、前视、死区……（无扰切换已经做了，改它不产生台阶）
 * P_RW_IDLE  **只在"没人驱动"时可写**：轴距、cpr、极性、软限位、遥测周期
 * —— 这些量被正在跑的回放/遥控依赖，"边跑边改"会把几何/尺度改错
 * P_CMD      **命令**：目标转速/目标转角/使能/录制/回放/测试……
 * ⇒ 只有**驱动令牌持有者**能写；且**不持久化**（上电不许自己触发一次）
 */
#define P_RO            (0u)
#define P_RW_LIVE       (1u)
#define P_RW_IDLE       (2u)
#define P_CMD           (3u)

#define G_SPD           (0u)        /* 速度环 / 编码器 / 电机极性 */
#define G_SYS           (1u)        /* 保护 / 链路 / 遥测 */
#define G_STEER         (2u)        /* 转向位置环（内环） */
#define G_RC            (3u)
#define G_INS           (4u)
#define G_RT            (5u)
#define G_PB            (6u)        /* 回放（Pure Pursuit 外环） */
#define G_TEST          (7u)        /* 航向测试 */

/* 六、参数元数据（`param_table.h` 的 11 个字段一一对应） */
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
    uint8           grp;
} proto_param_t;

/* 一条归一化后的命令（三种传输解帧后都填这个结构） */
typedef struct
{
    uint8  src;         /* PROTO_SRC_* */
    uint8  op;          /* PROTO_OP_* */
    uint8  id;          /* 参数 id（GET/SET） */
    float  val;         /* 目标值 / ESTOP 开关 / TOGGLE 忽略 */
    uint16 token;       /* 需要 ack 时回带（仅 WiFi 用） */
} proto_msg_t;

/* 七、接口 */
void    proto_init              (void);

/* GET  -> 当前值 SET  -> 钳位（+ 交叉钳位）之后真正写进去的值 SAVE -> 底层 Flash 错误码（0 = 成功） */
uint8   proto_exec              (const proto_msg_t *m, float *out);

/* ---- 参数元数据（**全工程只此一份**；motor_link/app_params/ui_menu 都从这里取）---- */
uint16  proto_param_count       (void);
const proto_param_t *proto_param_at   (uint16 idx);
const proto_param_t *proto_param_by_id(uint8 id);
uint16  proto_param_index_of_ptr(const void *ptr);   /* 0xFFFF = 没找到 */
void    proto_param_load_defaults (void);            /* 只写 RAM，不写 Flash */

/* ---- 协议指纹（PING 的 ACK 里发给上位机）---- */
uint16  proto_ver               (void);
uint16  proto_param_table_crc   (void);   /* 对每行 {id, lo, hi, def, perm, grp} 做 CRC16 */
uint16  proto_ch_layout_crc     (void);   /* 对遥测通道号/通道数做 CRC16 */

/* ---- 驱动令牌（带租期）---- */
uint8   proto_drv_owner         (void);              /* PROTO_SRC_*，NONE = 空闲 */
uint8   proto_drv_held_by       (uint8 src);         /* 1 = 令牌在该来源手里 */
uint8   proto_drv_acquire       (uint8 src);         /* 1 = 拿到（遥控可抢占） */
void    proto_drv_renew         (uint8 src);         /* 只有持有者调用才有效：续租 */
void    proto_drv_release       (uint8 src);         /* 只有持有者调用才有效：立刻释放 */
/* 主循环每圈一次：租约到期 -> 释放令牌（链路真断了才顺手把目标清零） */
void    proto_tick              (void);

/* ---- 急停（全局，不属于权限）---- */
void    proto_estop             (uint8 src, uint8 on);

/* ---- 事件出口（车端没有屏幕时，让上位机知道"驱动权被抢了"）---- */
void    proto_set_event_sink    (void (*fn)(const char *text));

/* ---- 只给打印/诊断用的字符串 ---- */
const char *proto_src_name      (uint8 src);
const char *proto_op_name       (uint8 op);
const char *proto_err_name      (uint8 err);

#endif
