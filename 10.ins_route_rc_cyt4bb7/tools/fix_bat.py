"""修两个 .bat：改成 CRLF（cmd.exe 不认 LF，会解析成一堆 'ho' 'o.' 之类的乱码），
并把使用说明从 .bat 移进 Python（批处理越短越不容易被解析坏）。

顺带给 mag_live.py 补上「STEP 2b 手捏磁铁」的现场提示。
"""
import os

TL = r"D:/jisuyueye9car/A_zjc_project/4.motor_wifi_verify_cyt4bb7/tools"
PY = r"C:\Users\zjc39\.workbuddy\binaries\python\envs\default\Scripts\python.exe"

BAT_TMPL = """@echo off
chcp 936 >nul
cd /d "%~dp0"
"{py}" {script} %1
echo.
echo [done] raw data saved next to this file (csv)
pause
"""

for name, script in (("mag_live.bat", "mag_live.py"), ("mag_check.bat", "mag_sweep.py sweep 25")):
    body = BAT_TMPL.format(py=PY, script=script)
    # ★ 必须 CRLF：cmd.exe 读 LF-only 的批处理会把行拼错
    open(os.path.join(TL, name), "wb").write(body.replace("\n", "\r\n").encode("ascii"))
    print(f"{name:16s} ✓ 已写成 CRLF")

# ---- mag_live.py：把 STEP 2b 的提示加进现场说明 ----
p = os.path.join(TL, "mag_live.py")
raw = open(p, "rb").read()
t = raw.decode("utf-8")
crlf = "\r\n" in t
t = t.replace("\r\n", "\n")

old = '''print(" ★ 请你把转向**反复从最左推到最右、再推回最左**，推 5~6 个来回。")
print("   每次都尽量推到**同样的两个端点**（这样才看得出值可不可重复）。")
print("   脚本会自己认出你在动；屏幕上出现「√ 第 N 次运动」就说明它在跟。")'''
new = '''print(" ★ 做两件事（脚本最多等 120 秒，够做两个）：")
print()
print("  [1] 把转向**反复从最左推到最右、再推回最左**，推 5~6 个来回。")
print("      每次都尽量推到**同样的两个端点**（这样才看得出值可不可重复）。")
print()
print("  [2] 如果上面一直没出现「√ 第 N 次运动」，就**捏一块小磁铁**，")
print("      在编码器芯片**正上方**慢慢转一整圈。")
print("      ↑ 这一下能把「芯片/读取」和「装配」一刀切开：")
print("         转磁铁有反应 → 芯片和读取都是好的，问题 100% 在装配上")
print("         转磁铁也没反应 → 传感头根本没在工作")
print()
print("  脚本会自己认出你在动；出现「√ 第 N 次运动」就说明它在跟。")'''
assert t.count(old) == 1, t.count(old)
t = t.replace(old, new, 1)
open(p, "wb").write((t.replace("\n", "\r\n") if crlf else t).encode("utf-8"))
print("mag_live.py      ✓ 补上 STEP 2b 提示")
