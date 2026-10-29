/*********************************************************************************************************************
 * 参数持久化, 把整张参数表存进 Flash 页 0
 *
 * 表驱动, 不写死字段名: 元数据来自 proto.c, 存 = 按 proto_param_at(i) 抄 *ptr,
 * 读 = 逐个写回并按每行 lo/hi 校验, 越界的字段保持默认值。加 ID 不用改本模块。
 * PERM == P_CMD 的项一律不恢复, 存默认值, 即命令不持久化。
 *
 * 载荷格式（页 0）
 *   magic('KPZ1') | version | count | crc16(载荷) | rsvd | float val[count]
 * 校验: magic 与 count 必须与当前表一致, 否则整块拒绝, 防插队或删项造成静默错位; 载荷用 CRC16-MODBUS
 * flash_buffer_clear() 只清 FLASH_PAGE_LENGTH = 512 字节, 而 buffer 是 2048 字节;
 *   载荷 12 B 头 + count*4 B, 上限 96 项时为 396 B 仍在 512 以内, 超过 128 个 uint32 须自行补 memset
 ********************************************************************************************************************/

#ifndef _APP_PARAMS_H
#define _APP_PARAMS_H

#include "zf_common_headfile.h"
#include "proto.h"

/* 载荷上限, 与协议层同源, 越界由 proto.c 的编译期护栏拦住 */
#define APP_PARAM_MAX       (PROTO_PARAM_MAX)
/* 载荷格式版本, 与当前版本不符的块整块作废 */
#define APP_PARAM_VERSION   (2u)

/* 上电读一次, 放在各模块 init 之后、开 5ms 中断之前; 失败则保持源码默认值 */
void    app_param_init      (void);

/* 写 Flash, 擦加写阻塞几十 ms, 只能在主循环调用; 返回 0 = 成功, 非 0 = 失败码 */
uint8   app_param_save      (void);

/* 是否成功载入过有效数据, 0 = 用的是源码默认值 */
uint8   app_param_loaded    (void);

/* 当前运行参数重置为源码默认值, 不写 Flash */
void    app_param_restore_defaults (void);

/* 上一次载入或保存的诊断码, 0 = OK */
uint8   app_param_last_err  (void);

#endif
