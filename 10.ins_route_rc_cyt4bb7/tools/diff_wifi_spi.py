"""用 difflib 对比两份 zf_device_wifi_spi.c，只看真正的增删行。

⚠ 教训：之前用"逐行按下标比"会因**插入/删除造成的行号错位**刷出几百条假差异。
   必须用 LCS/unified diff，才能看清到底改了什么。
"""
import difflib

FILES = [
    r"D:/jisuyueye9car/A_zjc_project/M-Car-Project-main/M-Car-Project-main/libraries/zf_device/zf_device_wifi_spi.c",
    r"D:/jisuyueye9car/A_zjc_project/4.motor_wifi_verify_cyt4bb7/libraries/zf_device/zf_device_wifi_spi.c",
]


def load(path):
    raw = open(path, "rb").read()
    for enc in ("utf-8-sig", "utf-8", "gbk"):
        try:
            t = raw.decode(enc)
            return t.replace("\r\n", "\n").split("\n"), enc
        except UnicodeDecodeError:
            continue
    return raw.decode("latin-1").split("\n"), "latin-1"


def main():
    (a, ea), (b, eb) = load(FILES[0]), load(FILES[1])
    print(f"[左] M-Car 版  编码={ea} 行数={len(a)}")
    print(f"[右] 本工程版  编码={eb} 行数={len(b)}\n")

    diff = list(difflib.unified_diff(a, b, lineterm="", n=1,
                                     fromfile="M-Car", tofile="mine"))
    # 去掉文件头两行
    body = [l for l in diff[2:]]
    adds = [l for l in body if l.startswith("+") and not l.startswith("+++")]
    dels = [l for l in body if l.startswith("-") and not l.startswith("---")]
    print(f"== 真正新增 {len(adds)} 行 / 删除 {len(dels)} 行 ==\n")
    for l in body:
        print(l[:150])

    # 再确认：把注释与空行去掉后，还有没有实质差异
    def code_only(lines):
        out = []
        for l in lines:
            s = l.strip()
            if not s or s.startswith("//") or s.startswith("*") or s.startswith("/*"):
                continue
            # 去掉行尾注释，避免注释文字差异干扰
            out.append(s.split("//")[0].rstrip())
        return out

    ca, cb = code_only(a), code_only(b)
    sm = difflib.SequenceMatcher(a=ca, b=cb)
    real = [l for l in difflib.unified_diff(ca, cb, lineterm="") if l[:1] in "+-" and l[:3] not in ("+++", "---")]
    print(f"\n== 剥掉注释/空行后的实质差异 {len(real)} 行（相似度 {sm.ratio():.4f}）==")
    for l in real:
        print("   ", l[:150])


if __name__ == "__main__":
    main()
