# -*- coding: utf-8 -*-
"""修 main 里被写坏的 printf + RC 每秒诊断行 + CH3 自适应阈值"""
import os
os.chdir(os.path.dirname(os.path.abspath(__file__)))

EOL = "\\r\\n"          # 落到 C 源码里的字面量 \r\n


def c(s):
    """把占位符 @@ 换成 C 的换行转义"""
    return s.replace('@@', EOL)


# ============================ 1) remote_ctrl.h ============================
p = 'route_pp_wifi/code/remote_ctrl.h'
h = open(p, encoding='utf-8').read()

if 'RC_HDG_HYST' not in h:
    anchor = '#define RC_CH_VALID_MIN         (120)'
    assert anchor in h, 'h anchor'
    h = h.replace(anchor,
        anchor + '\n\n'
        '/* \u2605 2026-10-20\uff1aCH3 \u9608\u503c\u4e0d\u518d\u5199\u6b7b 992\uff0c\u6539\u6210**\u8fd0\u884c\u65f6\u81ea\u9002\u5e94**\uff08\u89c1 remote_ctrl.c rc_ch3_track\uff09\u3002\n'
        '   \u73b0\u573a\u4e09\u79cd\u63a5\u6cd5\u90fd\u51fa\u73b0\u8fc7\uff1a\u4f4e=192/\u9ad8=1792\uff08\u6807\u51c6\uff09\uff5c\u4f4e=0/\u9ad8=1792\uff08\u4e2a\u522b\u63a5\u6536\u673a\u628a\u8be5\u6863\u53d1\u6210 0\uff09\uff5c\n'
        '   \u4f4e=992/\u9ad8=1792\uff08\u5f00\u5173\u53ea\u7528\u5230\u4e2d/\u9ad8\u4e24\u6863\uff09\u3002\u5199\u6b7b 992 \u53ea\u80fd\u8986\u76d6\u7b2c\u4e00\u79cd\uff0c\u540e\u4e24\u79cd\u4f1a**\u6c38\u8fdc\u5224\u6210\u8235\u673a\u6a21\u5f0f**\u3002\n'
        '   \u505a\u6cd5\uff1a\u81ea\u5df1\u8bb0 min/max\uff0c\u89c1\u8fc7\u9ad8\u4f4d\uff08>1200\uff09\u4e14\u8df3\u5ea6 >=400 \u624d\u6807\u5b9a\uff0c\u9608\u503c\u53d6\u4e2d\u70b9\u3002 */\n'
        '#define RC_HDG_HYST             (60u)       /* \u9608\u503c\u6ede\u73af\uff08\u8ba1\u6570\uff09\uff0c\u9632\u6b62\u5728\u4e34\u754c\u70b9\u6765\u56de\u8df3\u6a21\u5f0f */', 1)

    anchor2 = 'extern volatile uint8   RC_Hdg_Mode;'
    assert anchor2 in h, 'h anchor2'
    h = h.replace(anchor2,
        anchor2 + '\n'
        'extern volatile uint16  RC_Hdg_Thr;        /* CH3 \u5f53\u524d\u5224\u9ad8/\u4f4e\u7684\u9608\u503c\uff08\u81ea\u9002\u5e94\uff09*/\n'
        'extern volatile uint8   RC_Hdg_Cal;        /* 1 = CH3 \u5df2\u7ecf\u6807\u5b9a\u8fc7\uff08\u89c1\u8fc7\u4e24\u7aef\uff09*/', 1)

    open(p, 'w', encoding='utf-8', newline='\n').write(h)
    print('h ok')
else:
    print('h skip')

# ============================ 2) remote_ctrl.c ============================
p = 'route_pp_wifi/code/remote_ctrl.c'
s = open(p, encoding='utf-8').read()

if 'rc_ch3_track' not in s:
    old = 'volatile uint8   RC_Hdg_Mode    = 0u;'
    assert old in s, 'g mode'
    s = s.replace(old,
        old + '\n'
        'volatile uint16  RC_Hdg_Thr     = 992u;     /* CH3 \u9608\u503c\uff08\u81ea\u9002\u5e94\uff0c\u89c1 rc_ch3_track\uff09*/\n'
        'volatile uint8   RC_Hdg_Cal     = 0u;       /* 1 = \u5df2\u6807\u5b9a\uff08\u89c1\u8fc7\u4e24\u7aef\uff09*/', 1)

    anchor = '/*----------------------------------------------------------\n * \u6309\u6b65\u957f\u91cf\u5316'
    assert anchor in s, 'quant anchor'
    track = c(r'''/*----------------------------------------------------------
 * CH3 二值开关：自适应阈值 + 滞环  ->  RC_Hdg_Mode
 *
 *   ★★ 2026-10-20 第二次修（现场：CH3 拨到"低"仍然只有舵机角模式）
 *
 *   旧写法是 `(raw > 992) ? 舵机 : 航向`，外加"raw<120 判成掉线、直接当舵机"。
 *   这三个假设里任何一个不成立，症状都一模一样：**模式永远出不来**。
 *     ① 低档 = 0    -> 先被 RC_CH_VALID_MIN 判成"没信号"，强制舵机（就是现在的现象）
 *     ② 低档 = 992  -> 阈值正好压在低档上，`>992` 不成立但滞环也进不来，只会单边翻
 *     ③ 量程不是 192~1792（有的接收机 1000~2000）-> 992 这个数根本不在量程里
 *
 *   => 不再相信任何写死的端点：自己记 min/max，
 *        · 见过 >1200（说明开关真能到高位）且跨度 >= 400 => 标定，阈值 = (min+max)/2；
 *        · 通道真的没接（恒 0 / 恒 992，跨度不够）     => **不标定**，保持舵机模式（最保守）。
 *      阈值两侧各留 RC_HDG_HYST 的滞环：开关在临界点抖动时模式不会来回跳
 *      （跳一次就把"松手基准航向"重取一次，车会莫名其妙偏一下）。
 *----------------------------------------------------------*/
static uint16 s_c3_min = 0x0FFFu;
static uint16 s_c3_max = 0u;
static uint8  s_c3_cal = 0u;
static uint8  s_c3_hi  = 0u;        /* 0 = 低档（航向） 1 = 高档（舵机角）*/

static void rc_ch3_track (void)
{
    uint16 thr;
    uint8  mode;

    if (RC_Raw_Ch3 < s_c3_min) { s_c3_min = RC_Raw_Ch3; }
    if (RC_Raw_Ch3 > s_c3_max) { s_c3_max = RC_Raw_Ch3; }

    if ((0u == s_c3_cal) &&
        ((uint32)(s_c3_max - s_c3_min) >= 400u) &&
        (s_c3_max >= 1200u))
    {
        s_c3_cal = 1u;
        printf("[RC] CH3 calibrated: min=%u max=%u" @@,
               (unsigned)s_c3_min, (unsigned)s_c3_max);
    }

    thr = (0u != s_c3_cal) ? (uint16)(((uint32)s_c3_min + (uint32)s_c3_max) / 2u)
                           : (uint16)RC_SW_VAL_MID;

    RC_Hdg_Thr = thr;
    RC_Hdg_Cal = s_c3_cal;

    /* 滞环：从低往高要 > thr+60，从高往低要 < thr-60 */
    if (0u == s_c3_hi)
    {
        if (RC_Raw_Ch3 > (uint16)(thr + RC_HDG_HYST)) { s_c3_hi = 1u; }
    }
    else
    {
        if (((uint32)RC_Raw_Ch3 + RC_HDG_HYST) < (uint32)thr) { s_c3_hi = 0u; }
    }

    /* 没标定 + 通道没信号 = 不知道开关在哪 -> 保持舵机模式（原行为最安全） */
    if ((0u == s_c3_cal) && (0u != (RC_Stick_Bad & 0x04u)))
    {
        mode = 0u;
    }
    else
    {
        mode = (0u != s_c3_hi) ? 0u : 1u;       /* 低 = 航向模式 */
    }

    if (mode != RC_Hdg_Mode)
    {
        RC_Hdg_Mode = mode;
        printf("[RC] CH3 -> %s (raw=%u thr=%u span=%u)" @@,
               (0u != mode) ? "HEADING" : "SERVO",
               (unsigned)RC_Raw_Ch3, (unsigned)thr,
               (unsigned)(s_c3_max - s_c3_min));
    }
}

''')
    s = s.replace(anchor, track + anchor, 1)

    i = s.index('    /* CH3 **\u4e8c\u503c**\u5f00\u5173 -> \u6a21\u5f0f\uff08\u89c1 remote_ctrl.h \u4e09\u00b7\u4e94\uff09')
    j = s.index('    RC_Hdg_Step = RC_HDG_STEP;')
    j = s.index('\n', j) + 1
    new_blk = ('''    /* CH3 **\u4e8c\u503c**\u5f00\u5173 -> \u6a21\u5f0f\uff1a\u4f4e = CH1 \u7ed9\u76ee\u6807\u822a\u5411 / \u9ad8 = CH1 \u7ed9\u8235\u673a\u89d2\u3002\n'
               '       \u2605 \u9608\u503c\u4e0d\u5199\u6b7b\uff0c\u8d70 rc_ch3_track() \u7684\u81ea\u9002\u5e94 + \u6ede\u73af\uff08\u90a3\u6bb5\u6ce8\u91ca\u5199\u4e86\u4e09\u79cd\u73b0\u573a\u63a5\u6cd5\uff09\u3002 */\n'
               '    rc_ch3_track();\n'
               '    RC_Ch3_Pct  = (0u != RC_Hdg_Mode) ? 0u : 100u;\n'
               '    RC_Hdg_Step = RC_HDG_STEP;\n')
    s = s[:i] + new_blk + s[j:]

    open(p, 'w', encoding='utf-8', newline='\n').write(s)
    print('c ok')
else:
    print('c skip')

# ============================ 3) main_cm7_0.c ============================
p = 'route_pp_wifi/user/main_cm7_0.c'
t = open(p, 'rb').read().decode('gbk')

# 3a) 修被写坏的 printf（\r\n 被当成真换行落进去了）
ls = t.split('\n')
fixed = False
for k, l in enumerate(ls):
    if 'IMU yaw=%ld deg' in l and not l.rstrip().endswith('",'):
        ls[k] = '    printf("[##] IMU yaw=%ld deg  rej=%u' + EOL + '",'
        if k + 1 < len(ls) and ls[k + 1].strip() in ('",', '"', ''):
            if ls[k + 1].strip() == '",':
                ls[k + 1] = None
        t = '\n'.join([x for x in ls if x is not None])
        fixed = True
        print('fixed printf at line', k + 1)
        break

if not fixed:
