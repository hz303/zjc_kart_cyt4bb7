/*********************************************************************************************************************
 * 参数持久化（整张参数表存进 Flash 页 0）
 *
 * 载荷：magic('KPZ1') | version | count | crc16 | rsvd | float val[count]
 * 校验：magic、version、count 与当前表一致, CRC16-MODBUS 覆盖 val[]; 任一不符整块拒收。
 * P_CMD 项不恢复, 一律回到源码默认值。
 ********************************************************************************************************************/

#ifndef _APP_PARAMS_H
#define _APP_PARAMS_H

#include "zf_common_headfile.h"
#include "proto.h"

/* 载荷上限, 与协议层同源; proto.c 有编译期护栏 */
#define APP_PARAM_MAX       (PROTO_PARAM_MAX)
/* 存储格式版本, 不符则整块作废 */
#define APP_PARAM_VERSION   (2u)

/* 上电读一次：各模块 init 之后、开 5 ms 中断之前调用; 失败保持源码默认值 */
void    app_param_init      (void);

/* 当前运行参数写 Flash（擦+写, 阻塞几十 ms）; 返回 0 = 成功, 非 0 = 失败码 */
uint8   app_param_save      (void);

/* 是否成功从 Flash 载入过有效数据, 0 = 用的是源码默认值 */
uint8   app_param_loaded    (void);

/* 重置为源码默认值, 不写 Flash */
void    app_param_restore_defaults (void);

/* 上一次载入/保存的诊断码, 0 = OK */
uint8   app_param_last_err  (void);

#endif
