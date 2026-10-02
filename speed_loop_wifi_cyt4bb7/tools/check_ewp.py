"""校验 IAR 工程文件(.ewp)里的相对路径能否在新位置解析。

用法：python check_ewp.py <project_config目录>

按项目约定检查三件事：
  1) $PROJ_DIR$ 相对路径展开后是否真的存在（隔绝"移动目录后路径错层"）
  2) 每个 group 的**直接**子 <file> 数量（防止把父组的 Node 数错当成本组）
  3) 工程引用的 ../ 层数分布（防止复制到别的层级后指错）
"""
import os
import sys
import xml.etree.ElementTree as ET


def resolve(proj_dir, raw):
    """把 $PROJ_DIR$ / $WS_DIR$ 相对路径展开成绝对路径。"""
    if not raw:
        return None
    r = raw.replace('$PROJ_DIR$', proj_dir).replace('$WS_DIR$', os.path.dirname(proj_dir))
    r = r.replace('\\', '/')
    parts = []
    for seg in r.split('/'):
        if seg == '..':
            parts.append(seg)
        elif seg == '.':
            continue
        else:
            parts.append(seg)
    return os.path.normpath('/'.join(parts)) if r.startswith('/') else r.replace('../', '@@UP@@')


def main():
    proj_dir = os.path.abspath(sys.argv[1]).replace('\\', '/')
    ewp = os.path.join(proj_dir, 'cyt4bb7_cm_7_0.ewp')

    tree = ET.parse(ewp)
    root = tree.getroot()

    missing = []
    total = 0
    depth_count = {}

    for name_el in root.iter('name'):
        raw = (name_el.text or '').strip()
        if '$PROJ_DIR$' not in raw and '$WS_DIR$' not in raw:
            continue
        total += 1

        # 统计 ../ 层数分布
        ups = raw.count('..\\') + raw.count('../')
        depth_count[ups] = depth_count.get(ups, 0) + 1

        p = raw.replace('$PROJ_DIR$', proj_dir).replace('$WS_DIR$', os.path.dirname(proj_dir))
        p = p.replace('\\', '/')
        base = proj_dir
        tail = p
        if '$PROJ_DIR$' in raw:
            tail = p[len(base):].lstrip('/')
        # 用真实文件系统解析
        cand = os.path.normpath(os.path.join(base, tail)) if tail else None
        if cand is None or not os.path.exists(cand):
            missing.append(raw)

    print('[路径检查] .ewp =', ewp)
    print('  解析基准 $PROJ_DIR$ =', proj_dir)
    print('  含宏路径条目数 =', total)
    print('  ../ 层数分布 =', dict(sorted(depth_count.items())))
    if missing:
        print('  [缺失] 以下条目展开后不存在（%d 条）：' % len(missing))
        for m in missing[:40]:
            print('    -', m)
    else:
        print('  [OK] 全部存在的条目均已命中')

    # group 直接子 file 数量
    print('[分组检查]')
    groups = [g for g in root.iter('group')]
    for g in groups:
        gname_el = g.find('name')
        gname = (gname_el.text or '').strip() if gname_el is not None else '?'
        direct_files = [f for f in g.findall('file')]
        sub_groups = [x for x in g.findall('group')]
        print('  %-14s 直接子文件=%-3d 子组=%d' % (gname, len(direct_files), len(sub_groups)))


if __name__ == '__main__':
    main()
