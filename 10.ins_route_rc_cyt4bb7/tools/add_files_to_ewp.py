"""把本工程的新增源文件加进 IAR 工程的 code 组，并核对结果。

⚠ 执行方式：**必须写成 .py 文件再跑**，不要用 heredoc ——
   脚本里有 `'$PROJ_DIR$\\..\\..\\code\\'` 这种以反斜杠结尾的字符串，
   经 shell heredoc 传参会把结尾的引号吃掉，直接 SyntaxError。
"""
import sys
import xml.etree.ElementTree as ET

EWP = r"D:/jisuyueye9car/A_zjc_project/4.motor_wifi_verify_cyt4bb7/motor_wifi_verify/iar/project_config/cyt4bb7_cm_7_0.ewp"
REL = "$PROJ_DIR$" + "\\..\\..\\code\\"
NEW = ["motor_link.c", "motor_link.h", "motor_drv.c", "motor_drv.h", "mag_enc.c", "mag_enc.h"]


def main():
    tree = ET.parse(EWP)
    root = tree.getroot()

    target = None
    for g in root.iter("group"):
        nm = g.find("name")
        if nm is not None and (nm.text or "").strip() == "code":
            target = g
            break
    if target is None:
        print("FAIL: 找不到 code 组")
        return 1

    before = [(f.find("name").text or "").strip() for f in target.findall("file")]
    print("code 组原直接子文件:", before)

    for n in NEW:
        rel = REL + n
        if rel in before:
            continue
        f = ET.SubElement(target, "file")
        ET.SubElement(f, "name").text = rel

    after = [(f.find("name").text or "").strip() for f in target.findall("file")]
    print("code 组现直接子文件:", after)

    tree.write(EWP, encoding="UTF-8", xml_declaration=True)
    print("ewp 已更新")
    return 0


if __name__ == "__main__":
    sys.exit(main())
