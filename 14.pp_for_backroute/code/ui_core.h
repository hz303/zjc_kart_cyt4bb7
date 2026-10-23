/* C 车惯导 + 路表 - 精简 UI 引擎（从 MM32 `2.imu_encoder_test/code/auto_menu.c` 的机制精简而来）
 * 原工程 `auto_menu.c` 68 KB：双向链表（up/down/enter/back）+ 静态内存池（100 个单元）
 * + 自动分页（每页 8 项）+ 泛型 param_set（4 种参数类型）—— 为 26 个菜单项设计。
 * 本工程只要 8 项 ⇒ 用"静态字段数组 + 光标 + 脏标记"即可，代码量约 1/8。
 * ---- 保留的三件事（观感与操作完全一致）----------------------------------------------
 * ① 键位语义：KEY_3 = 上 / KEY_4 = 下 / KEY_1 = 确认 / KEY_2 = 返回
 * （长按 100 ms 连按一步，见 ui_menu.c）
 * ② 渲染策略：UI_STATIC 进页面画一次 / UI_REALTIME 每拍比缓存变了才画 / UI_ONDEMAND 只在有按键那拍画
 * ③ 增量刷新：靠 `cache[]` 字符串比较 ⇒ **空闲时零 SPI 传输**（屏幕 SPI 传输是毫秒级的，这点很关键）
 * ---- 两处与 MM32 版不同（都是被迫的）------------------------------------------------
 * ① CYT4BB7 的 `zf_device_ips200` **没有 `ips200_clear_area()`**（MM32 版有）⇒ 本文件自己补一个；
 * ② **格式化一律不用 `%f`**：IAR 默认库不一定支持浮点格式化（原工程是因为 tinyprintf 不支持而手写，
 * 本工程是同一个道理）⇒ 全部走"整数部分 + 小数部分"手写。
 */

#ifndef _UI_CORE_H
#define _UI_CORE_H

#include "zf_common_headfile.h"

/* ---- 几何（8x16 字体：列宽 8、行高 16。与 20.zf_clean / 2.imu_encoder_test 一致）---- */
#define UI_LINE_H           (16)
#define UI_COL_W            (8)

/* 每个字段的缓存串长度。
 * 24 会**静默截断**长状态串（例如 "EncCal" + " push 1.00 m, press twice" = 34 字符），
 * 现场表现就是"菜单选项显示不全"。这里给到 40：够放 28 字符正文 + 余量。
 */
#define UI_CACHE_LEN        (40)

/* 卡丁上的这块 IPS200 是**转 180° 装的**（与 C 车相反）⇒ 整屏内容要反转，
 * 否则字是倒的、菜单看着像"乱码"。
 * PORTAIT=0x00 / PORTAIT_180=0xC0 / CROSSWISE=0x70 / CROSSWISE_180=0xA0
 * 它在 init 内部按 `ips200_display_dir` 写死；init 之后再调 `ips200_set_dir()`
 * 只会改 width/height 这两个变量，**面板纹丝不动**（驱动注释的原话是
 * "这个函数只有在初始化屏幕之前调用才生效"）。`ui_init()` 里的顺序已经是
 * set_dir → init，所以改这个宏就够。
 * （一行 30 字符、20 行都不变）。横屏才是 320×240，别混。
 * 换车/换安装方向只改这一行：1 = 旋转 180°（卡丁）；0 = 默认（C 车）。
 */
#define UI_ROT180           (1)

/* 屏幕逻辑尺寸（竖屏 240x320；实际值以驱动里的 ips200_width_max/height_max 为准） */
#define UI_SCREEN_W         (240)
#define UI_SCREEN_H         (320)

/* 光标前缀：平时 "> "，编辑态 ">>"（8x16 字体下一行最多 30 字符） */
#define UI_CUR_STR          "> "
#define UI_CUR_EDIT_STR     ">>"

/* 字段 / 页面 */
typedef enum
{
    UI_STATIC       = 0,            /* 进页面画一次（标签） */
    UI_REALTIME     = 1,            /* 每拍比缓存，变了才画（数值） */
    UI_ONDEMAND     = 2,            /* 只在有按键那一拍画 */
} ui_policy_t;

typedef struct ui_field
{
    uint8       row;                /* 行号，y = row * UI_LINE_H */
    uint8       x;
    ui_policy_t policy;
    void      (*format)(char *buf, uint8 size, void *ctx);   /* 生成显示串 */
    void       *ctx;                /* 传给 format 的上下文（变量指针 / 菜单项指针） */
    char        cache[UI_CACHE_LEN];/* 上次绘制串（引擎维护，勿手写） */
} ui_field_t;

typedef struct ui_page
{
    ui_field_t *fields;
    uint8       count;
    int8        cursor_row;             /* -1 = 无光标 */
    int8        cursor_row_last;        /* 引擎维护 */
    uint8       cursor_glyph;           /* 0 = "> " ; 1 = ">>" */
    uint8       cursor_glyph_last;      /* 引擎维护 */
    uint8       dirty;                  /* 1 = 下次调用全清全绘 */
} ui_page_t;

void    ui_init               (void);                          /* 初始化屏幕（竖屏 + 8x16 字体） */
void    ui_clear              (void);                          /* 清屏（用背景色） */

void    ui_clear_area         (uint16 x1, uint16 y1, uint16 x2, uint16 y2, const uint16 color);

void    ui_puts               (uint16 x, uint16 y, const char *s);   /* 在像素坐标写串 */
void    ui_puts_row           (uint8 row, uint8 col, const char *s); /* 按"行列"写串 */
uint16  ui_col                (uint8 col);                     /* 字符列 -> 像素 x */
/* 任何往屏幕写串的地方都必须先过它 —— 越界会触发 IPS200 驱动里的
 * `zf_assert`，而逐飞那个断言失败是 `while(true)`，**整机死循环**（详见 ui_core.c）。
 */
uint8   ui_fit_chars          (uint16 x_px, uint8 n);

/* 页面引擎 */
void    ui_page_mark_dirty    (ui_page_t *pg);                 /* 置全绘标志（进页面时调） */
void    ui_field_draw         (ui_field_t *f);                 /* 强制重画一个字段 */
/* key_event: 本拍有没有按键（UI_ONDEMAND 用它决定要不要刷新） */
void    ui_page_render        (ui_page_t *pg, uint8 key_event);

/* 格式化回调（ctx 的类型见注释；**都不使用 `%f`**） */
void    ui_fmt_f1             (char *buf, uint8 size, void *ctx);   /* ctx = float*，1 位小数 */
void    ui_fmt_f2             (char *buf, uint8 size, void *ctx);   /* ctx = float*，2 位小数 */
void    ui_fmt_f0             (char *buf, uint8 size, void *ctx);   /* ctx = float*，0 位小数 */
void    ui_fmt_i              (char *buf, uint8 size, void *ctx);   /* ctx = int32* */
void    ui_fmt_s              (char *buf, uint8 size, void *ctx);   /* ctx = const char* */

/* 手写浮点格式化（不使用 printf 的 %f）：把 v 写成"整数.小数"或"整数"，缓冲突 ≤ size */
void    ui_ftoa               (char *buf, uint8 size, float v, uint8 point_num);

#endif
