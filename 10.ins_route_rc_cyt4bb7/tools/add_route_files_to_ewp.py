r"""把工程 10 的「路表（P1/P2）」源文件加进 IAR 工程的 code 组（只 CM7_0）。

⚠ 执行方式：**写成本文件再跑**，别用 shell heredoc（`$PROJ_DIR$\..\..\code\`
   这种以反斜杠结尾的字符串经 heredoc 会把引号吃掉）。

用法：
    python add_route_files_to_ewp.py            # 只报告
    python add_route_files_to_ewp.py --apply    # 真加

幂等：已经在里面的文件不重复加。
"""
import io
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(HERE, ".."))
PC = os.path.join(ROOT, "steer_loop_wifi", "iar", "project_config")
EWP = os.path.normpath(os.path.join(PC, "cyt4bb7_cm_7_0.ewp"))

NEW_FILES = [
    "ins_route.c", "ins_route.h",
    "ins_route_builtin.c",
    "route_link.c", "route_link.h",
    "route_app.c", "route_app.h",
    "route_pb.c", "route_pb.h",
]


def group_span(text):
    m = re.search(r"<group>\s*<name>code</name>", text)
    if not m:
        return None
    start = m.start()
    end = text.find("</group>", start)
    if end < 0:
        return None
    return start, end + len("</group>")


def files_in(block):
    return re.findall(r"<name>([^<]*?code\\[^<]*?)</name>", block)


def main():
    apply = "--apply" in sys.argv
    with io.open(EWP, encoding="utf-8") as f:
        text = f.read()

    span = group_span(text)
    if span is None:
        print("FAIL: 找不到 code 组")
        return 1
    start, end = span
    have = [os.path.basename(p) for p in files_in(text[start:end])]
    print("== IAR 工程 ==")
    print("  code 组现有 %d 个文件" % len(have))

    todo = [n for n in NEW_FILES if n not in have]
    if not todo:
        print("  ✅ 全部已存在（幂等 OK）")
        return 0
    # 只加磁盘上真的存在的文件（route_pb 可能还没写）
    miss = [n for n in todo if not os.path.exists(os.path.join(ROOT, "steer_loop_wifi", "code", n))]
    todo = [n for n in todo if n not in miss]
    if miss:
        print("  ⚠ 磁盘上还没有（跳过）：%s" % " ".join(miss))
    print("  待加入 %d 个: %s" % (len(todo), " ".join(todo)))

    if not apply:
        print("  （只报告。加 --apply 才真写）")
        return 0

    add = "".join(
        "<file><name>$PROJ_DIR$\\..\\..\\code\\%s</name></file>" % n for n in todo
    )
    text = text[:end - len("</group>")] + add + text[end - len("</group>"):]
    with io.open(EWP, "w", encoding="utf-8", newline="") as f:
        f.write(text)

    with io.open(EWP, encoding="utf-8") as f:
        t2 = f.read()
    sp2 = group_span(t2)
    have2 = [os.path.basename(p) for p in files_in(t2[sp2[0]:sp2[1]])]
    missing = [n for n in NEW_FILES if n not in have2 and n not in miss]
    print("  回读：code 组 %d 个文件；核对 %s"
          % (len(have2), "✅ 新文件都在" if not missing else "❌ 还缺 " + " ".join(missing)))
    return 0 if not missing else 1


if __name__ == "__main__":
    sys.exit(main())
