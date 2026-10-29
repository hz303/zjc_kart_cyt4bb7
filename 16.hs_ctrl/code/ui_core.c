/*********************************************************************************************************************
 * 精简 UI 引擎实现：字符网格绘串、手写浮点格式化、字段增量渲染
 ********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "ui_core.h"

/* ---- 基元 ---- */
uint16 ui_col (uint8 col)
{
    return (uint16)((uint16)col * UI_COL_W);
}

/* 一行最多能画几个字符（从像素 x 起），所有写串前的收口。
 * ips200_show_char() 内有 zf_assert(x < ips200_width_max)，逐飞的 debug_assert_handler()
 * 失败时是 while(true)，越界即整机死循环，主循环里的落盘轮询也一并停掉。
 * 返回值按 最后一个字符起点小于宽度 求，多的截掉；补位也受此上限约束。 */
uint8 ui_fit_chars (uint16 x_px, uint8 n)
{
    uint16 w = ips200_width_max;
    uint16 m;

    if (x_px >= w) { return 0u; }

    /* 最后一个字符的起点仍必须 < w ⇒ 个数上限 = floor((w-1-x)/字符宽) + 1 */
    m = (uint16)(((w - 1u - x_px) / UI_COL_W)) + 1u;

    return ((uint16)n > m) ? (uint8)m : n;
}

void ui_puts (uint16 x, uint16 y, const char *s)
{
    char   tmp[UI_CACHE_LEN + 4u];
    uint8  n;

    if (NULL == s) { return; }

    n = ui_fit_chars(x, (uint8)strlen(s));
    (void)memcpy(tmp, s, n);
    tmp[n] = 0;

    ips200_show_string(x, y, tmp);
}

void ui_puts_row (uint8 row, uint8 col, const char *s)
{
    ui_puts(ui_col(col), (uint16)((uint16)row * UI_LINE_H), s);
}

void ui_clear (void)
{
    ips200_clear();
}

void ui_clear_area (uint16 x1, uint16 y1, uint16 x2, uint16 y2, const uint16 color)
{
    uint16 y;
    uint16 t;

    if (x1 > x2) { t = x1; x1 = x2; x2 = t; }
    if (y1 > y2) { t = y1; y1 = y2; y2 = t; }

    if (x2 >= UI_SCREEN_W) { x2 = UI_SCREEN_W - 1; }
    if (y2 >= UI_SCREEN_H) { y2 = UI_SCREEN_H - 1; }

    for (y = y1; y <= y2; y ++)
    {
        ips200_draw_line(x1, y, x2, y, color);
    }
}

void ui_init (void)
{
    /* set_dir 必须在 init 之前：MADCTL(0x36) 在 init 里按当前方向写进面板，
       init 后再设方向只改 width/height，面板不变。 */
#if UI_ROT180
    ips200_set_dir(IPS200_PORTAIT_180);     /* 屏转 180° 安装 */
#else
    ips200_set_dir(IPS200_PORTAIT);
#endif
    ips200_init(IPS200_TYPE_SPI);
    ips200_set_font(IPS200_8X16_FONT);
    ips200_set_color(RGB565_BLACK, RGB565_WHITE);
    ips200_clear();
}

/* ---- 手写浮点格式化：不用 %f，支持负数与四舍五入进位，point_num = 0 时输出纯整数 ---- */
void ui_ftoa (char *buf, uint8 size, float v, uint8 point_num)
{
    int32  ip;
    int32  ip0;
    int32  fp = 0;
    int32  mul = 1;
    uint8  neg = 0;
    uint8  i = 0;
    uint8  n = 0;
    uint8  k;
    char   tmp[12];

    if ((NULL == buf) || (0 == size)) { return; }

    if (point_num > 4u) { point_num = 4u; }

    if (v < 0.0f) { neg = 1u; v = -v; }

    /* 先按小数位数放大，避免整数部分先截断带来的误差 */
    for (k = 0; k < point_num; k ++) { mul *= 10; }

    ip  = (int32)v;
    ip0 = ip;
    fp  = (int32)((v - (float)ip) * (float)mul + 0.5f);
    if (fp >= mul) { fp -= mul; ip += 1; ip0 += 1; }    /* 四舍五入导致进位 */

    /* 整数部分倒着存 */
    do
    {
        if (n < (uint8)sizeof(tmp)) { tmp[n ++] = (char)('0' + (char)(ip % 10)); }
        ip /= 10;
    } while ((ip > 0) && (n < (uint8)sizeof(tmp)));

    /* 负零不显示负号 */
    if ((0 != neg) && ((ip0 > 0) || (fp > 0))) { buf[i ++] = '-'; }
    while (n > 0)
    {
        if (i < (uint8)(size - 1u)) { buf[i ++] = tmp[-- n]; }
        else                        { n --; }
    }

    if (point_num > 0u)
    {
        if (i < (uint8)(size - 1u)) { buf[i ++] = '.'; }
        for (k = 0; k < point_num; k ++)
        {
            mul /= 10;
            if (i < (uint8)(size - 1u)) { buf[i ++] = (char)('0' + (char)((fp / mul) % 10)); }
        }
    }

    buf[i] = 0;
}

/* ---- 格式化回调 ---- */
void ui_fmt_f1 (char *buf, uint8 size, void *ctx)
{
    float v = (NULL != ctx) ? (*(float *)ctx) : 0.0f;
    ui_ftoa(buf, size, v, 1u);
}

void ui_fmt_f2 (char *buf, uint8 size, void *ctx)
{
    float v = (NULL != ctx) ? (*(float *)ctx) : 0.0f;
    ui_ftoa(buf, size, v, 2u);
}

void ui_fmt_f0 (char *buf, uint8 size, void *ctx)
{
    float v = (NULL != ctx) ? (*(float *)ctx) : 0.0f;
    ui_ftoa(buf, size, v, 0u);
}

void ui_fmt_i (char *buf, uint8 size, void *ctx)
{
    int32 v = (NULL != ctx) ? (*(int32 *)ctx) : 0;

    if (NULL == buf) { return; }

    /* 复用 ui_ftoa，int32 转 float 无精度损失 */
    ui_ftoa(buf, size, (float)v, 0u);
}

void ui_fmt_s (char *buf, uint8 size, void *ctx)
{
    const char *s = (const char *)ctx;
    uint8 i = 0;

    if ((NULL == buf) || (0 == size)) { return; }

    if (NULL == s) { s = ""; }
    while ((s[i] != 0) && (i < (uint8)(size - 1u))) { buf[i] = s[i]; i ++; }
    buf[i] = 0;
}

/* ---- 字段绘制 ---- */
void ui_field_draw (ui_field_t *f)
{
    char   buf[UI_CACHE_LEN];
    char   out[UI_CACHE_LEN + 4];
    uint16 x_px;
    uint8  cap;
    uint8  n;
    uint8  old_n;

    if (NULL == f) { return; }

    buf[0] = 0;
    if (NULL != f->format) { f->format(buf, (uint8)sizeof(buf), f->ctx); }

    x_px = ui_col(f->x);
    cap  = ui_fit_chars(x_px, (uint8)(sizeof(out) - 1u));   /* 这一行最多能画几个字符 */

    /* 空格补齐到上次长度覆盖残留字符，省掉画矩形的 SPI 小传输；补位同样受 cap 限制 */
    {
        uint8 i;
        for (i = 0; (i < (uint8)sizeof(out) - 1u) && (buf[i] != 0); i ++) { out[i] = buf[i]; }
        n = i;
        if (n > cap) { n = cap; }
        out[n] = 0;
    }

    old_n = (uint8)strlen(f->cache);
    while ((n < old_n) && (n < cap))
    {
        out[n ++] = ' ';
    }
    out[n] = 0;

    /* f->x 是字符列，必须过 ui_col() 换成像素；列号当像素用会把值画到标签上且不报错 */
    ips200_show_string(x_px, (uint16)((uint16)f->row * UI_LINE_H), out);

    /* 缓存只存原串，不含补位空格 */
    (void)strcpy(f->cache, buf);
}

void ui_page_mark_dirty (ui_page_t *pg)
{
    if (NULL != pg) { pg->dirty = 1u; }
}

/* ---- 页面渲染：按脏标记、光标变化、字段缓存三级增量 ---- */
void ui_page_render (ui_page_t *pg, uint8 key_event)
{
    uint8 i;

    if (NULL == pg) { return; }

    /* ---- 1. 全量重绘（进页面 / 主动标记脏）---- */
    if (0u != pg->dirty)
    {
        ui_clear();

        for (i = 0; i < pg->count; i ++)
        {
            pg->fields[i].cache[0] = 0;             /* 缓存作废，强制重画 */
            ui_field_draw(&pg->fields[i]);
        }

        pg->cursor_row_last   = -2;                 /* 强制下面重画光标 */
        pg->cursor_glyph_last = 0xFFu;
        pg->dirty = 0u;
    }

    /* ---- 2. 光标（位置或形态变化才动）---- */
    if (pg->cursor_row >= 0)
    {
        if ((pg->cursor_row != pg->cursor_row_last) || (pg->cursor_glyph != pg->cursor_glyph_last))
        {
            if (pg->cursor_row_last >= 0)
            {
                ui_puts_row((uint8)pg->cursor_row_last, 0, "  ");
            }

            ui_puts_row((uint8)pg->cursor_row, 0,
                        (1u == pg->cursor_glyph) ? UI_CUR_EDIT_STR : UI_CUR_STR);

            pg->cursor_row_last   = pg->cursor_row;
            pg->cursor_glyph_last = pg->cursor_glyph;
        }
    }

    /* ---- 3. 字段增量：比缓存，变了才画 ---- */
    for (i = 0; i < pg->count; i ++)
    {
        ui_field_t *f = &pg->fields[i];
        char buf[UI_CACHE_LEN];

        if (UI_STATIC == f->policy) { continue; }
        if ((UI_ONDEMAND == f->policy) && (0u == key_event)) { continue; }

        buf[0] = 0;
        if (NULL != f->format) { f->format(buf, (uint8)sizeof(buf), f->ctx); }

        if (0 != strcmp(buf, f->cache)) { ui_field_draw(f); }
    }
}
