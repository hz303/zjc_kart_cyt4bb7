"""修上一版交付的 kart_link.c 里的使能位解码 bug（与新工程同一个坑）。

问题：以前写成 kart_enable = (uint16)raw;
      raw 是 VALUE 字段的 IEEE754 位模式，6.0f 的字节是 00 00 C0 40，
      截低 16 位得到 0x0000 —— 使能位永远传不下来，且不报错。
修法：与上位机约定"VALUE 按数值解释"，改成 (uint16)(val + 0.5f)。
"""
P = r"D:/jisuyueye9car/sacred_opensource_offroad_code-master/sacred_opensource_offroad_code-master/2024_CYT4BB7/20240920FirstPorject/code/kart_link.c"

OLD = """    case KART_CMD_SET_ENABLE:
        kart_enable = (uint16)raw;
        kart_send_ack(cmd, token);
        break;"""

NEW = """    case KART_CMD_SET_ENABLE:
        /* ★ VALUE 按"数值"解释（上位机发 6.0f → 这里得到 6）。
         *   原写法 (uint16)raw 取的是 IEEE754 位模式的低 16 位，
         *   6.0f 的字节是 00 00 C0 40 → 截出来是 0x0000，使能永远传不下来。 */
        kart_enable = (uint16)(val + 0.5f);
        kart_send_ack(cmd, token);
        break;"""


def load(path):
    raw = open(path, 'rb').read()
    for enc in ('utf-8', 'gbk'):
        try:
            return raw.decode(enc), enc
        except UnicodeDecodeError:
            continue
    raise SystemExit('解码失败')


txt, enc = load(P)
if NEW.split('\n')[1] in txt:
    print('已经改过了，跳过')
elif OLD in txt:
    txt = txt.replace(OLD, NEW, 1)
    open(P, 'wb').write(txt.encode(enc))
    print(f'已修复（编码 {enc}）')
else:
    raise SystemExit('既没找到原文也没有改动痕迹 —— 先人工看一眼')
