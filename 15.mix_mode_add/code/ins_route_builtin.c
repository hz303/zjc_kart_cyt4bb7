/* ---- 内建路表注册表（手写文件，勿被 tools/route_to_carbin.py 覆盖）----
 * 清单项 = 槽号、名字、车端镜像指针、镜像长度；表体在生成物 ins_route_builtin_xxx.c 里。
 * 语义：只在槽为空时写入，不覆盖用户表；写槽会把该槽设为 active，
 *       故清单里最后写下去的那张成为上电默认表。槽位分工见 flash_map.h。
 *       槽 0 手推录制、槽 1 正式赛道、槽 2 调试短表、槽 3/4 实测轨迹。 */
#include "zf_common_headfile.h"
#include "ins_route.h"

/* 生成物里的符号，每张表一对（文件名 = 符号前缀）：
 *   extern const uint32 ins_route_builtin_ring_len;
 *   extern const uint8  ins_route_builtin_ring_img[]; */

/* 空表：cnt = 0 时上电块不迭代；数组留一个占位元素，避免零长度数组的编译错误 */
const ins_route_builtin_t ins_route_builtin_tbl[] =
{
    { (uint8)0xFFu, "none", 0, 0 },     /* 占位，cnt = 0 时不会被用到 */
};

const uint32 ins_route_builtin_cnt = 0u;
