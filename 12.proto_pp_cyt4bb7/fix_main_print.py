# -*- coding: utf-8 -*-
"""part3: 修 main 里被写坏的 printf + 加 RC3 诊断行（避免三引号，逐行拼）"""
import os
os.chdir(os.path.dirname(os.path.abspath(__file__)))

EOL = "\\r\\n"
p = 'route_pp_wifi/user/main_cm7_0.c'
raw = open(p, 'rb').read().decode('gbk')
crlf = '\r\n' in raw
t = raw.replace('\r\n', '\n')

# ---------- 3a) 修坏掉的 printf ----------
ls = t.split('\n')
done = False
for k, l in enumerate(ls):
    if 'IMU yaw=%ld deg' in l and not l.rstrip().endswith('",'):
        ls[k] = '    printf("[##] IMU yaw=%ld deg  rej=%u' + EOL + '",'
        if k + 1 < len(ls) and ls[k + 1].strip() == '",':
            ls[k + 1] = None
        t = '\n'.join([x for x in ls if x is not None])
        done = True
        print('fixed printf at line', k + 1)
        break
if not done:
    print('printf already ok')

# ---------- 3b) RC3 诊断行 ----------
if 'RC3 c3=' in t:
    print('rc3 already present')
else:
    anchor = ('(unsigned)motor_link_tx_period_ms());\n'
              '    }')
    assert anchor in t, 'anchor'

    add = []
    add.append('')
    add.append('        /* \u2605 CH3 / \u822a\u5411\u6a21\u5f0f\u4e00\u884c\uff082026-10-20 \u52a0\uff09\uff1a\u5224\u65ad"CH3 \u5230\u5e95\u8bfb\u5230\u6ca1\u6709"\u53ea\u770b\u8fd9\u4e00\u884c\u3002')
    add.append('             c3  = CH3 \u539f\u59cb\u503c\uff080 \u4e5f\u53ef\u80fd\u662f**\u6b63\u5e38\u7684\u4f4e\u6863**\uff0c\u522b\u6025\u7740\u5f53\u6389\u7ebf\uff09')
    add.append('             thr = \u5f53\u524d\u5224\u9ad8\u4f4e\u7684\u81ea\u9002\u5e94\u9608\u503c\uff1bcal=1 \u8868\u793a\u5df2\u7ecf\u89c1\u8fc7\u5f00\u5173\u4e24\u7aef')
    add.append('             mode= HDG(CH1 \u63a7\u822a\u5411) / SRV(CH1 \u63a7\u8235\u673a\u89d2)')
    add.append('             off = \u5f53\u524d\u822a\u5411\u504f\u79fb(deg\uff0cCH1 \u7ed9)\uff1bty = \u76ee\u6807\u822a\u5411\uff1byw = \u5b9e\u6d4b\u822a\u5411')
    add.append('             \u5224\u8bfb\uff1a\u62e8 CH3 \u65f6 c3 \u8981\u8df3\u3001mode \u8981\u8ddf\u7740\u7ffb\uff1b')
    add.append('                   c3 \u4e00\u52a8\u4e0d\u52a8     = \u8fd9\u4e2a\u901a\u9053\u538b\u6839\u6ca1\u63a5 / \u6ca1\u6620\u5c04\uff08\u53bb\u67e5\u7ebf\u6216\u901a\u9053\u5206\u914d\uff09')
    add.append('                   \u62e8\u4e86\u4f46 mode \u4e0d\u7ffb = \u91cf\u7a0b\u7279\u6b8a\uff0c\u628a c3 \u4e24\u5934\u8bfb\u5230\u7684\u503c\u53d1\u6211\uff0c\u6539\u6807\u5b9a\u5224\u636e */')
    add.append('        printf("        RC3 c3=%u thr=%u cal=%u mode=%s off=%d ty=%d yw=%d' + EOL + '",')
    add.append('               (unsigned)RC_Raw_Ch3,')
    add.append('               (unsigned)RC_Hdg_Thr,')
    add.append('               (unsigned)RC_Hdg_Cal,')
    add.append('               (0u != RC_Hdg_Mode) ? "HDG" : "SRV",')
    add.append('               (int)RC_Hdg_Off,')
    add.append('               (int)RC_Cmd_Yaw,')
    add.append('               (int)steer_yaw_deg);')

    t = t.replace(anchor,
                  '(unsigned)motor_link_tx_period_ms());\n'
                  + '\n'.join(add) + '\n    }', 1)
    print('rc3 added')

if crlf:
    t = t.replace('\n', '\r\n')
open(p, 'wb').write(t.encode('gbk'))
print('main ok')
