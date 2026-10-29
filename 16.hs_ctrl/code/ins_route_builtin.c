/* ---- 内建路表注册表（手写文件，生成物不覆盖它）----
 *   清单项 {槽号, 名字, img, len}；表体在生成的 ins_route_builtin_xxx.c 里：
 *       tools/route_to_carbin.py <route.json> --outdir <d> --cname ins_route_builtin_xxx
 *   语义：槽是空的才写入，实现见 ins_route_slot_load_image() 与 main_cm7_0.c 上电块。
 *   写槽会把该槽设为 active，故表里最后写下的那张成为上电默认表。
 *   当前为空表：cnt = 0，上电块不迭代；电子制图走 route_link 串口灌表。
 *   加表步骤：生成 .c 拷进本目录、加进 CM7_0 工程、下面加一行、改 cnt。 */
#include "zf_common_headfile.h"
#include "ins_route.h"

/* 生成物里的符号，每张表一对：<前缀>_len（const uint32）与 <前缀>_img[] */

/* 空表：cnt = 0 时上电块不迭代，占位元素避免零长度数组在严格模式下报错 */
const ins_route_builtin_t ins_route_builtin_tbl[] =
{
    { (uint8)0xFFu, "none", 0, 0 },     /* 占位，cnt = 0 故不会被用到 */
};

const uint32 ins_route_builtin_cnt = 0u;
