"""把工程 10 新增的"屏幕菜单 + 跑班"源文件加进 IAR 工程的 code 组，
并把逐飞按键驱动的引脚顺序对齐工程 8（K3/K4 对调）。

⚠ 执行方式：**写成本文件再跑**，别用 shell heredoc ——
   路径里有 `$PROJ_DIR$\\..\\..\\code\\` 这种以反斜杠结尾的字符串，
   经 heredoc 传参会把结尾的引号吃掉，直接 SyntaxError（工程 9 踩过）。

用法：
    python add_menu_files_to_ewp.py            # 只报告
    python add_menu_files_to_ewp.py --apply    # 真加

设计：
  · 只动 CM7_0 的工程（业务代码全在主核；CM7_1 是空核）
  · **幂等**：已经在里面的文件不重复加
  · 按键头文件按**字节**改（那文件是 GBK，用文本编辑器回写会毁中文注释）
"""
import io
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(HERE, ".."))
PC = os.path.join(ROOT, "steer_loop_wifi", "iar", "project_config")
EWP = os.path.normpath(os.path.join(PC, "cyt4bb7_cm_7_0.ewp"))
KEYH = os.path.normpath(os.path.join(ROOT, "libraries", "zf_device", "zf_device_key.h"))

NEW_FILES = [
    "flash_map.h",
    "app_params.c", "app_params.h",
    "bench_log.c",  "bench_log.h",
    "spd_bench.c",  "spd_bench.h",
    "ui_core.c",    "ui_core.h",
    "ui_menu.c",    "ui_menu.h",
]

# 按键引脚顺序：与工程 8 一致（K3=P20_0 / K4=P20_1）
KEY_OLD = b"{P20_3, P20_2, P20_1, P20_0}"
KEY_NEW = b"{P20_3, P20_2, P20_0, P20_1}"


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


def patch_key_header(apply):
    raw = open(KEYH, "rb").read()
    n_old = raw.count(KEY_OLD)
    n_new = raw.count(KEY_NEW)
    print("== 按键头 ==")
    print("  旧串出现 %d 次 / 新串出现 %d 次" % (n_old, n_new))
    if n_old == 0 and n_new == 1:
        print("  ✅ 已是工程 8 的顺序，无需改")
        return True
    if n_old != 1:
        print("  ❌ 认不出该文件（预期旧串恰好 1 次），**不动它**")
        return False
    if not apply:
        print("  （只报告。加 --apply 才真写）")
        return True
    open(KEYH, "wb").write(raw.replace(KEY_OLD, KEY_NEW))
    raw2 = open(KEYH, "rb").read()
    ok = (raw2.count(KEY_NEW) == 1) and (raw2.count(KEY_OLD) == 0)
    print("  已写入；回读核对：%s" % ("✅" if ok else "❌"))
    return ok


def patch_ewp(apply):
    with io.open(EWP, encoding="utf-8") as f:
        text = f.read()

    span = group_span(text)
    if span is None:
        print("FAIL: 找不到 code 组")
        return False
    start, end = span
    have = [os.path.basename(p) for p in files_in(text[start:end])]
    print("== IAR 工程 ==")
    print("  code 组现有 %d 个文件" % len(have))

    todo = [n for n in NEW_FILES if n not in have]
    if not todo:
        print("  ✅ 全部已存在（幂等 OK）")
        return True
    print("  待加入 %d 个: %s" % (len(todo), " ".join(todo)))

    if not apply:
        print("  （只报告。加 --apply 才真写）")
        return True

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
    missing = [n for n in NEW_FILES if n not in have2]
    print("  回读：code 组 %d 个文件；核对 %s"
          % (len(have2), "✅ 新文件都在" if not missing else "❌ 还缺 " + " ".join(missing)))
    return not missing


def main():
    apply = "--apply" in sys.argv
    a = patch_key_header(apply)
    b = patch_ewp(apply)
    if not apply:
        print("\n（只报告。加 --apply 才真写文件）")
    return 0 if (a and b) else 1


if __name__ == "__main__":
    sys.exit(main())
