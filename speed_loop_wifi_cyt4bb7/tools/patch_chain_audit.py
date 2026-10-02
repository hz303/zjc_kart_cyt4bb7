"""链路核对后的两处固件修正。

① VALID_MASK：字段只有 16 位，而通道已经 20 个 → 明确约定 0xFFFF 为"全部有效"哨兵值，
   把注释写清楚，免得以后有人照位去解释第 17~20 位。
② 角速度/积分的符号：跟随 mag_dir，与 mag_deg 保持同一方向约定 —— 否则
   mag_dir=0 时 mag_deg 在减小、而 mag_sum 在增大，"积分 vs 角度"的对照会反号。
"""
import os

ROOT = r"D:/jisuyueye9car/A_zjc_project/4.motor_wifi_verify_cyt4bb7/motor_wifi_verify"


def load(p):
    raw = open(p, "rb").read()
    for enc in ("utf-8-sig", "utf-8", "gbk"):
        try:
            t = raw.decode(enc)
            break
        except UnicodeDecodeError:
            continue
    else:
        raise SystemExit("解码失败 " + p)
    eol = "\r\n" if "\r\n" in t else "\n"
    return t.replace("\r\n", "\n"), enc, eol


def save(p, t, enc, eol):
    if eol == "\r\n":
        t = t.replace("\n", "\r\n")
    open(p, "wb").write(t.encode(enc))


def sub(t, old, new, tag):
    n = t.count(old)
    assert n == 1, (tag, "命中 %d 次" % n)
    print("  ✔", tag)
    return t.replace(old, new, 1)


# ---------------- ① motor_link.c：VALID_MASK 语义 ----------------
p = os.path.join(ROOT, "code/motor_link.c")
t, enc, eol = load(p)
t = sub(t,
        "    n = motor_put_u16(motor_txbuf, n, 0xFFFF);  /* VALID_MASK：16 通道全有效 */",
        "    /* VALID_MASK：0xFFFF = **全部有效**（哨兵值，不是按位图）。\n"
        "     * ⚠ 这个字段只有 16 位，而通道数已经 20 个 —— 按位根本表达不下第 17~20 位，\n"
        "     *   所以约定\"全 1 = 全部有效\"。上位机侧 ChannelStore.IsValid() 也按这条解释。\n"
        "     *   将来真要做逐通道有效性，必须先把字段扩成 u32（改协议）。 */\n"
        "    n = motor_put_u16(motor_txbuf, n, 0xFFFF);",
        "motor_link.c VALID_MASK 注释")
save(p, t, enc, eol)

# ---------------- ② mag_enc.c/h：速度与积分跟随 mag_dir ----------------
p = os.path.join(ROOT, "code/mag_enc.c")
t, enc, eol = load(p)
t = sub(t,
        """        mag_speed_signed_v = s;
        mag_speed_rps_v    = (float)s * mag_k_speed;""",
        """        mag_speed_signed_v = s;
        /* ★ 符号跟随 mag_dir，和 mag_deg 用同一方向约定 ——
         *   否则 mag_dir=0 时 mag_deg 在减小而 mag_sum 在增大，
         *   "速度积分 vs 绝对角度"的对照会反号，融合就错了。 */
        mag_speed_rps_v    = (float)s * mag_k_speed * ((mag_dir < 0.5f) ? -1.0f : 1.0f);""",
        "mag_enc.c 速度符号跟随 dir")
save(p, t, enc, eol)

p = os.path.join(ROOT, "code/mag_enc.h")
t, enc, eol = load(p)
t = sub(t,
        "float   mag_enc_speed_rps   (void);         /* 角速度（rad/s，= 有符号值 × k_speed） */",
        "float   mag_enc_speed_rps   (void);         /* 角速度（rad/s，= 有符号值 × k_speed，符号已跟随 mag_dir） */",
        "mag_enc.h 接口注释")
t = sub(t,
        "int32   mag_enc_sum_mrad    (void);         /* 角速度积分（毫弧度；一圈 = 2π×1000 ≈ 6283） */",
        "int32   mag_enc_sum_mrad    (void);         /* 角速度积分（毫弧度；一圈 = 2π×1000 ≈ 6283）。\n"
        "                                             * ⚠ 改 mag_dir 后建议先把积分清零（历史是按旧符号累的） */",
        "mag_enc.h 积分注释")
save(p, t, enc, eol)
print("固件两处修正完成")
