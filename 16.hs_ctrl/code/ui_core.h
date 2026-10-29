/*********************************************************************************************************************
 * 精简 UI 引擎：字符网格字段 + 光标 + 脏标记
 *      KEY_3 = 上 / KEY_4 = 下 / KEY_1 = 确认 / KEY_2 = 返回，长按 100 ms 走一步（连按在 ui_menu.c）
 *      格式化一律不用 %f：IAR 默认库不一定支持浮点格式化，改走整数部分 + 小数部分手写
 ********************************************************************************************************************/

#ifndef _UI_CORE_H
#define _UI_CORE_H

#include "zf_common_headfile.h"

/* ---- 几何：8x16 字体，列宽 8、行高 16 ---- */
#define UI_LINE_H           (16)                    /* 行高 */
#define UI_COL_W            (8)                     /* 字符宽 */

/* 字段缓存串长度：一行 30 字符，菜单项前留光标 2 列剩 28；取 40 容纳正文与余量，短于此会静默截断 */
#define UI_CACHE_LEN        (40)

/* 屏幕安装方向：1 = 旋转 180°，0 = 默认；只能在 ips200_init() 之前定，
   MADCTL(0x36) 在 init 内按当前方向写入，init 后再设只改 width/height，面板不变 */
#define UI_ROT180           (1)

/* 屏幕逻辑尺寸，竖屏 240x320；实际值以驱动里的 ips200_width_max/height_max 为准 */
#define UI_SCREEN_W         (240)
#define UI_SCREEN_H         (320)

/* 光标前缀：平时 > ，编辑态 >>；8x16 字体下一行最多 30 字符 */
#define UI_CUR_STR          "> "
#define UI_CUR_EDIT_STR     ">>"

/* ---- 字段 / 页面 ---- */
typedef enum
{
    UI_STATIC       = 0,            /* 进页面画一次（标签）       */
    UI_REALTIME     = 1,            /* 每拍比缓存，变了才画（数值）*/
    UI_ONDEMAND     = 2,            /* 只在有按键那一拍画         */
} ui_policy_t;

typedef struct ui_field
{
    uint8       row;                /* 行号，y = row * UI_LINE_H                     */
    uint8       x;                  /* 字符列，不是像素；引擎内部走 ui_col()           */
    ui_policy_t policy;
    void      (*format)(char *buf, uint8 size, void *ctx);   /* 生成显示串            */
    void       *ctx;                /* 传给 format 的上下文（变量指针 / 菜单项指针）  */
    char        cache[UI_CACHE_LEN];/* 上次绘制串（引擎维护，勿手写）                 */
} ui_field_t;

typedef struct ui_page
{
    ui_field_t *fields;
    uint8       count;
    int8        cursor_row;             /* -1 = 无光标                              */
    int8        cursor_row_last;
    uint8       cursor_glyph;           /* 0 = 单 > 号；1 = 双 > 号                   */
    uint8       cursor_glyph_last;
    uint8       dirty;                  /* 1 = 下次调用全清全绘                      */
} ui_page_t;

/* ---- 基元 ---- */
void    ui_init               (void);                          /* 初始化屏幕：竖屏 + 8x16 字体 */
void    ui_clear              (void);

/* 局部清除，坐标为像素，(x2,y2) 含；实现是逐行 draw_line，一次调用最多约 16 次 SPI 小传输 */
void    ui_clear_area         (uint16 x1, uint16 y1, uint16 x2, uint16 y2, const uint16 color);

void    ui_puts               (uint16 x, uint16 y, const char *s);   /* 在像素坐标写串 */
void    ui_puts_row           (uint8 row, uint8 col, const char *s); /* 按行列写串     */
uint16  ui_col                (uint8 col);                     /* 字符列 -> 像素 x */
/* 从像素 x 起这一行最多还能画几个字符；任何往屏幕写串的地方都要先过它，
   越界会触发 IPS200 驱动里的 zf_assert，该断言失败是 while(true)，整机死循环 */
uint8   ui_fit_chars          (uint16 x_px, uint8 n);

/* ---- 页面引擎 ---- */
void    ui_page_mark_dirty    (ui_page_t *pg);                 /* 置全绘标志，进页面时调 */
void    ui_field_draw         (ui_field_t *f);                 /* 强制重画一个字段         */
/* key_event: 本拍有没有按键，UI_ONDEMAND 据此决定要不要刷新 */
void    ui_page_render        (ui_page_t *pg, uint8 key_event);

/* ---- 格式化回调：ctx 类型见下，均不使用 %f ----
   ui_fmt_f1/f2/f0 -> ctx = float*（1/2/0 位小数）  ui_fmt_i -> int32*  ui_fmt_s -> const char* */
void    ui_fmt_f1             (char *buf, uint8 size, void *ctx);
void    ui_fmt_f2             (char *buf, uint8 size, void *ctx);
void    ui_fmt_f0             (char *buf, uint8 size, void *ctx);
void    ui_fmt_i              (char *buf, uint8 size, void *ctx);
void    ui_fmt_s              (char *buf, uint8 size, void *ctx);

/* 手写浮点格式化：把 v 写成 整数.小数 或 整数，缓冲不超过 size，不用 printf 的 %f */
void    ui_ftoa               (char *buf, uint8 size, float v, uint8 point_num);

#endif
