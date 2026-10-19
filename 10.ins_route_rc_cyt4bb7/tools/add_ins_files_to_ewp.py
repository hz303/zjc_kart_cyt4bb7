"""把工程 10 新增的惯导源文件加进 IAR 工程的 code 组。

⚠ 执行方式：**写成本文件再跑**，别用 shell heredoc ——
   路径里有 `$PROJ_DIR$\\..\\..\\code\\` 这种以反斜杠结尾的字符串，
   经 heredoc 传参会把结尾的引号吃掉，直接 SyntaxError（工程 9 踩过）。

用法：
    python add_ins_files_to_ewp.py            # 只报告
    python add_ins_files_to_ewp.py --apply    # 真加

设计：
  · 只动 CM7_0 的工程（业务代码全在主核；CM7_1 是空核，加了也只多编译一遍）
  · **幂等**：已经在里面的文件不重复加
  · 加完立刻回读并打印 code 组的完整文件清单，便于肉眼核对
"""
import io
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
PC = os.path.join(HERE, "..", "steer_loop_wifi", "iar", "project_config")
EWP = os.path.normpath(os.path.join(PC, "cyt4bb7_cm_7_0.ewp"))

NEW_FILES = [
    "ins_imu.c", "ins_imu.h",
    "ins_odom.c", "ins_odom.h",
]


def group_span(text):
    """返回 code 组的 (start, end) 字符区间（含 <group>..</group>）。"""
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
    block = text[start:end]

    have = [os.path.basename(p) for p in files_in(block)]
    print("code 组现有 %d 个文件:" % len(have))
    print("  " + " ".join(sorted(have)))

    todo = [n for n in NEW_FILES if n not in have]
    if not todo:
        print("\n全部已存在，无需改动（幂等 OK）")
    else:
        print("\n待加入 %d 个: %s" % (len(todo), " ".join(todo)))

    if not apply:
        print("\n（只报告。加 --apply 才真写文件）")
        return 0

    if todo:
        add = "".join(
            "<file><name>$PROJ_DIR$\\..\\..\\code\\%s</name></file>" % n for n in todo
        )
        # 插在 </group> 之前，保持原来那种"一行一个 file 元素"的紧凑写法
        text = text[:end - len("</group>")] + add + text[end - len("</group>"):]

        with io.open(EWP, "w", encoding="utf-8", newline="") as f:
            f.write(text)
        print("\n已写入 %s" % EWP)

    # 回读核对
    with io.open(EWP, encoding="utf-8") as f:
        text2 = f.read()
    span2 = group_span(text2)
    have2 = [os.path.basename(p) for p in files_in(text2[span2[0]:span2[1]])]
    print("回读：code 组 %d 个文件" % len(have2))
    missing = [n for n in NEW_FILES if n not in have2]
    print("核对：%s" % ("✅ 新文件都在" if not missing else "❌ 还缺 " + " ".join(missing)))
    return 0 if not missing else 1


if __name__ == "__main__":
    sys.exit(main())
