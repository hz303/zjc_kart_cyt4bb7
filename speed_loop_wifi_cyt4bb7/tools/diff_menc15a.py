"""三方比对 zf_device_menc15a.{c,h}: 原厂 vs 我们 vs 队友(以及 M-Car 做旁证)。

用 difflib 做真正的 LCS diff —— 不要用"逐行按下标比"（插入/删除会造成行号错位，伪装成大量差异）。
"""
import difflib
import os

PRISTINE = r"D:/jisuyueye9car_backup/CYT4BB7_Library-master/Seekfree_CYT4BB_Opensource_Library/libraries/zf_device"
OURS     = r"D:/jisuyueye9car/A_zjc_project/4.motor_wifi_verify_cyt4bb7/libraries/zf_device"
MATE     = r"D:/jisuyueye9car/A_zjc_project/Karting-Project-main/Karting-Project-main/libraries/zf_device"
MCAR     = r"D:/jisuyueye9car/A_zjc_project/M-Car-Project-main/M-Car-Project-main/libraries/zf_device"


def load(p):
    raw = open(p, "rb").read()
    for enc in ("utf-8-sig", "utf-8", "gbk"):
        try:
            t = raw.decode(enc)
            return t.replace("\r\n", "\n").replace("\r", "\n").split("\n"), enc, raw
        except UnicodeDecodeError:
            continue
    raise SystemExit("解码失败 " + p)


def cmp(a_path, b_path, a_name, b_name):
    A, ae, araw = load(a_path)
    B, be, braw = load(b_path)
    sm = difflib.SequenceMatcher(None, A, B)
    ratio = sm.ratio()
    print(f"\n{'='*78}")
    print(f"{a_name}  vs  {b_name}")
    print(f"  编码 {ae} vs {be} | 行数 {len(A)} vs {len(B)} | 字节 {len(araw)} vs {len(braw)}")
    print(f"  相似度(按行) = {ratio:.4f}")
    n_same = sum(bl.size for bl in sm.get_matching_blocks())
    print(f"  相同行 {n_same} / {max(len(A), len(B))}")
    d = list(difflib.unified_diff(A, B, fromfile=a_name, tofile=b_name, lineterm="", n=2))
    body = [x for x in d if x[:1] in "+-" and x[:3] not in ("+++", "---")]
    print(f"  差异行数 = {len(body)}   (+{sum(1 for x in body if x[0]=='+')} / -{sum(1 for x in body if x[0]=='-')})")
    if d:
        print("  ---- diff ----")
        for line in d:
            print("   " + line[:160])
    else:
        print("  >>> 完全一致（字节级也一样）" if araw == braw else "  >>> 文本一致，仅编码/换行不同")
    return ratio


for name in ("zf_device_menc15a.h", "zf_device_menc15a.c"):
    print("\n" + "#" * 78)
    print("#  文件：" + name)
    print("#" * 78)
    cmp(os.path.join(PRISTINE, name), os.path.join(OURS, name), "原厂", "我们")
    cmp(os.path.join(PRISTINE, name), os.path.join(MATE, name), "原厂", "队友")
    cmp(os.path.join(MATE, name), os.path.join(MCAR, name), "队友", "M-Car")
