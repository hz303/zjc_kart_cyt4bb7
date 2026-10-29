/*********************************************************************************************************************
 * C 车惯导 + 路表 - 精简 UI 引擎（竖屏 240x320 + 8x16 字体 + 主板 4 键）
 *
 * 键位：KEY_1 = 确认 / KEY_2 = 返回 / KEY_3 = 上 / KEY_4 = 下，长按 100 ms 连按一步
 * 渲染策略：UI_STATIC 进页面画一次 / UI_REALTIME 每拍比缓存 / UI_ONDEMAND 只在有按键那拍画
 * 增量刷新：cache[] 字符串比较，空闲时零 SPI 传输
 * 格式化一律不用 %f，走整数部分 + 小数部分手写
 ********************************************************************************************************************/

#ifndef _UI_CORE_H
#define _UI_CORE_H

#include "zf_common_headfile.h"

/* ---- 几何：8x16 字体，列宽 8 px、行高 16 px ---- */
#define UI_LINE_H           (16)
#define UI_COL_W            (8)

/* 字段缓存串长度。竖屏 240 px / 8 px = 一行 30 字符（菜单项前面留光标 2 列 ⇒ 28）；
 * 取 40 留余量，过短会静默截断长状态串。 */
#define UI_CACHE_LEN        (40)

/* 屏幕安装方向：1 = 旋转 180°，0 = 默认。两种竖屏模式均为 240x320，改此宏不影响布局。
 * 必须在 ips200_init() 之前确定：显示方向由 init 内的 0x36(MADCTL) 写入面板，
 * init 之后再调 ips200_set_dir() 只改 width/height，面板不变；ui_init() 内顺序已正确。
 * PORTAIT = 0x00 / PORTAIT_180 = 0xC0 / CROSSWISE = 0x70 / CROSSWISE_180 = 0xA0 */
#define UI_ROT180           (1)

/* 屏幕逻辑尺寸；两种竖屏模式均为 240x320，横屏才是 320x240 */
#define UI_SCREEN_W         (240)
#define UI_SCREEN_H         (320)

/* 光标前缀：平时 > ，编辑态 >>（8x16 字体一行最多 30 字符） */
#define UI_CUR_STR          "> "
#define UI_CUR_EDIT_STR     ">>"

/* ---- 字段 / 页面 ---- */
/* policy：UI_STATIC 进页面画一次（标签）/ UI_REALTIME 每拍比缓存 / UI_ONDEMAND 只在有按键那拍画 */
typedef enum
{
    UI_STATIC       = 0,
    UI_REALTIME     = 1,
    UI_ONDEMAND     = 2,
} ui_policy_t;

typedef struct ui_field
{
    uint8       row;                /* 行号，y = row * UI_LINE_H                     */
    uint8       x;                  /* 字符列（非像素），内部经 ui_col() 换算          */
    ui_policy_t policy;
    void      (*format)(char *buf, uint8 size, void *ctx);   /* 生成显示串            */
    void       *ctx;
    char        cache[UI_CACHE_LEN];/* 上次绘制串（引擎维护，勿手写）                 */
} ui_field_t;

/* cursor_row = -1 无光标；cursor_glyph 0 = > ，1 = >>；*_last 与 dirty 由引擎维护 */
typedef struct ui_page
{
    ui_field_t *fields;
    uint8       count;
    int8        cursor_row;
    int8        cursor_row_last;
    uint8       cursor_glyph;
    uint8       cursor_glyph_last;
    uint8       dirty;
} ui_page_t;

/* ---- 基元 ---- */
void    ui_init               (void);
void    ui_clear              (void);

/* 局部清除，坐标为像素，(x2,y2) 含端点。实现为逐行 draw_line，单次最多约 16 次 SPI 传输。 */
void    ui_clear_area         (uint16 x1, uint16 y1, uint16 x2, uint16 y2, const uint16 color);

/* 写串：ui_puts 走像素坐标，ui_puts_row 走行列 */
void    ui_puts               (uint16 x, uint16 y, const char *s);
void    ui_puts_row           (uint8 row, uint8 col, const char *s);
uint16  ui_col                (uint8 col);
/* 从像素 x 起本行最多还能画几个字符。所有写串路径必须先过它：越界触发 IPS200 驱动
 * zf_assert，逐飞的断言失败是 while(true)，整机死循环（见 ui_core.c）。 */
uint8   ui_fit_chars          (uint16 x_px, uint8 n);

/* ---- 页面引擎 ---- */
void    ui_page_mark_dirty    (ui_page_t *pg);
void    ui_field_draw         (ui_field_t *f);                 /* 强制重画一个字段 */
/* key_event: 本拍有没有按键（UI_ONDEMAND 用它决定要不要刷新） */
void    ui_page_render        (ui_page_t *pg, uint8 key_event);

/* ---- 格式化回调（ctx 类型见行尾；均不使用 %f）---- */
void    ui_fmt_f1             (char *buf, uint8 size, void *ctx);   /* ctx = float*，1 位小数  */
void    ui_fmt_f2             (char *buf, uint8 size, void *ctx);   /* ctx = float*，2 位小数  */
void    ui_fmt_f0             (char *buf, uint8 size, void *ctx);   /* ctx = float*，0 位小数  */
void    ui_fmt_i              (char *buf, uint8 size, void *ctx);   /* ctx = int32*            */
void    ui_fmt_s              (char *buf, uint8 size, void *ctx);   /* ctx = const char*       */

/* 手写浮点格式化：支持负数与四舍五入进位，point_num = 0 输出纯整数，写入长度不超过 size */
void    ui_ftoa               (char *buf, uint8 size, float v, uint8 point_num);

#endif
