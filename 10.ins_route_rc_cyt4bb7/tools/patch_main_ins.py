"""把惯导的 6 处接线打进工程 10 的 main_cm7_0.c（P0：能测）。

⚠ 本文件是 **GBK + CRLF**（`user/` 下都是 GBK；这个是 CRLF）。
   所以按 gbk 读、按 gbk 写、newline='' 保住 \\r\\n；
   写之前先 `s.encode('gbk')` 预检 —— `io.open(...,'w')` 一开就截断，
   编码失败会留下一个**空文件**（第一次改 cm7_0_isr.c 就这么把文件清成 0 字节，靠备份救回来）。
   写临时文件再 os.replace ⇒ 原子。

6 处接线：
  1. include ins_imu.h / ins_odom.h
  2. 两个静态状态量（归零边沿记忆 + IMU 重试节拍）
  3. 初始化链：ins_odom_init() + ins_imu_init()（**必须在 pit_ms_init 之前**，标定要 5 秒静止）
  4. 遥测填值：CH_INS_YAW/GZ/X/Y/DIST/FLAGS（39~44）
  5. 主循环：位姿归零边沿 + IMU 掉线 1Hz 重试
  6. 1Hz 打印：一行 INS（判读顺序写在注释里）

用法：python patch_main_ins.py            # 只报告 + GBK 预检
      python patch_main_ins.py --apply    # 真改（先备份 .bak_ins，原子替换）
"""
import io
import os
import shutil
import sys

P = r"D:/jisuyueye9car/A_zjc_project/10.ins_route_rc_cyt4bb7/steer_loop_wifi/user/main_cm7_0.c"

INC_ANCHOR = '#include "remote_ctrl.h"\n'
INC_NEW = INC_ANCHOR + '#include "ins_imu.h"\n#include "ins_odom.h"\n'

G_ANCHOR = "static uint32   g_print_ms = 0;        /* 上次打印状态的时刻 */\n"
G_NEW = G_ANCHOR + (
    "static uint8    g_zero_last    = 0;   /* 惯导|位姿归零 命令的边沿记忆(0->1 只触发一次) */\n"
    "static uint32   g_imu_retry_ms  = 0;   /* IMU 掉线后的重试节拍(1Hz) */\n"
)

INIT_ANCHOR = "    spd_ctrl_init();\n"
INIT_NEW = INIT_ANCHOR + (
    "\n"
    "    /*-------------------------- 惯导: IMU + 航位推算 (工程 10 新增) --------------------------\n"
    "      [!] ins_imu_init() 是**阻塞**的: 上电陀螺零偏标定采 1000 点 x 5ms = **5 秒**,\n"
    "          这 5 秒车必须静止(被碰会判\"极差超限\"并重来, 最多 5 次 -> 最长约 25 秒)。\n"
    "          所以必须放在 pit_ms_init() **之前** —— 开了中断再等, 中断里的 ins_imu_update()\n"
    "          会拿着没标定好的零偏开始积分, yaw 从第一拍就是错的。\n"
    "      [!] 卡丁的 IMU 从来没初始化过(**工程 9 一次都没调过**): 第一次烧录请盯串口横幅,\n"
    "          看它认出的器件 ID。逐飞主板 IMU660RA / IMU660RC / IMU963RA 引脚完全相同\n"
    "          (SPI_2 / P15_0~3), 所以\"认出来了\"这件事本身就要用 ID 证实 —— 963RA 的 0x0F 应为 0x6A。*/\n"
    "    ins_odom_init();\n"
    "    ret = ins_imu_init();\n"
    "    {\n"
    "        uint8 id0f = 0u, id00 = 0u, idv = 0u;\n"
    "        ins_imu_dev_id_get(&id0f, &id00, &idv);\n"
    "        printf(\"[##] INS    imu_init=%u id(0F)=0x%02X id(00)=0x%02X (963RA 期望 0x6A; 0x00/0xFF=没接上)\\r\\n\",\n"
    "               (unsigned)ret, (unsigned)id0f, (unsigned)id00);\n"
    "    }\n"
    "    {\n"
    "        float m  = ins_m_per_rev;\n"
    "        int   mi = (int)m;\n"
    "        int   mf = (int)((m - (float)mi) * 10000.0f);\n"
    "        printf(\"[##] INS    mpm=%d.%04d m/rev (卡丁 cpr=1858 -> 这就是后轮周长; 占位值, 推 2m 必重标)\\r\\n\",\n"
    "               mi, mf);\n"
    "    }\n"
)

CH_ANCHOR = "    motor_link_ch_set(CH_K_Z,          spd_g.k_z);\n"
CH_NEW = CH_ANCHOR + (
    "\n"
    "    /* --- ** 惯导(39~44): 工程 10(2026-10-15) 新增。坐标约定见 motor_link.h 的 CH_INS_YAW 注解。\n"
    "       [!] 必须走**原子快照**: ins_imu_snapshot() / ins_odom_snapshot()。\n"
    "           直接读全局变量会在\"中断正在更新\"时拿到撕裂值(半新半旧), 上位机画出来的轨迹\n"
    "           会出现**锯齿/回跳** —— 而且只在车动的时候出现, 静止测不出来。 */\n"
    "    {\n"
    "        ins_imu_snap_t  imu;\n"
    "        ins_odom_snap_t odom;\n"
    "        uint8           f = 0u;\n"
    "\n"
    "        ins_imu_snapshot(&imu);\n"
    "        ins_odom_snapshot(&odom);\n"
    "\n"
    "        motor_link_ch_set(CH_INS_YAW,  odom.yaw);\n"
    "        motor_link_ch_set(CH_INS_GZ,   imu.gyro_z_dps);\n"
    "        motor_link_ch_set(CH_INS_X,    odom.x);\n"
    "        motor_link_ch_set(CH_INS_Y,    odom.y);\n"
    "        motor_link_ch_set(CH_INS_DIST, odom.dist_abs);\n"
    "\n"
    "        /* 状态位图(8 位, 与工程 8 同一套顺序, 上位机/文档别改) */\n"
    "        if (ins_imu_ready())                 { f |= 0x01u; }   /* IMU 就绪(零偏标定完)  */\n"
    "        if (imu.flags & INS_ST_MAG_CAL_DONE) { f |= 0x02u; }   /* 磁力计已标定          */\n"
    "        if (odom.flags & ODOM_ST_STILL)      { f |= 0x04u; }   /* 判定静止(ZUPT 窗口)   */\n"
    "        if (odom.flags & ODOM_ST_SLIP)       { f |= 0x08u; }   /* 本拍判打滑(只报警)    */\n"
    "        if (imu.flags & INS_ST_IMU_LINK_BAD) { f |= 0x40u; }   /* IMU 通信坏            */\n"
    "        if (imu.flags & INS_ST_WARN)         { f |= 0x80u; }   /* 其它告警              */\n"
    "        motor_link_ch_set(CH_INS_FLAGS, (float)f);\n"
    "    }\n"
)

LOOP_ANCHOR = "        remote_ctrl_apply();\n"
LOOP_NEW = LOOP_ANCHOR + (
    "\n"
    "        /* 2c) 惯导的慢动作: 位姿归零 + IMU 掉线重试。\n"
    "               放主循环的理由与遥控一样 —— 它们是\"慢速决策\", 不是 5ms 控制。\n"
    "               [!] 归零必须【位姿 + 航向】一起归: 只归一个, 后面所有相对量(路表/回放)\n"
    "                  就整体错位了, 而且**看着仍然像一条轨迹**。\n"
    "               [!] IMU 重试只在链路坏时做: 那时中断里的 ins_imu_update() 不碰 SPI_2,\n"
    "                  主循环里重新 init 才没有竞争(ins_imu_try_recover 内部就是这么判的)。 */\n"
    "        {\n"
    "            uint8 nz = (ins_zero_cmd > 0.5f) ? 1u : 0u;\n"
    "\n"
    "            if (nz && (!g_zero_last))\n"
    "            {\n"
    "                ins_odom_reset();\n"
    "                ins_imu_zero_yaw();\n"
    "                printf(\"[##] INS    zero (x/y/yaw = 0)\\r\\n\");\n"
    "            }\n"
    "            g_zero_last = nz;\n"
    "\n"
    "            if (0 == ins_imu_link_ok())\n"
    "            {\n"
    "                uint32 tnow = motor_link_now_ms();\n"
    "                if ((uint32)(tnow - g_imu_retry_ms) >= 1000u)\n"
    "                {\n"
    "                    g_imu_retry_ms = tnow;\n"
    "                    (void)ins_imu_try_recover();\n"
    "                }\n"
    "            }\n"
    "        }\n"
)

PR_ANCHOR = "               (unsigned)motor_link_tx_period_ms());\n    }\n}\n"
PR_NEW = (
    "               (unsigned)motor_link_tx_period_ms());\n"
    "    }\n"
    "\n"
    "    /* 惯导一行: 航向 / 陀螺Z / 位姿 / 里程 / 状态位 / 零偏 / 静止统计。\n"
    "       * 判读顺序(按这个查, 别跳):\n"
    "           横幅 imu_init 非 0        -> IMU 没认出来, 先查 P15 接线与型号, **别去调姿态**\n"
    "           yaw 静止时一直在走        -> 看 bz(在线零偏)那一列; bz 稳而 yaw 仍走 = dt/符号问题\n"
    "           gz 静止时 |值| > 0.2 dps  -> 零偏没收敛, 或 ZUPT 没生效(看 st=)\n"
    "           x/y 推车方向不对          -> 查 INS_YAW_SIGN 与编码器 inv_l/inv_r 极性\n"
    "           dist 与卷尺差 k 倍        -> 就是 ins_mpm 错, 按\"推 2.00 m\"重标(见 ins_odom.h)\n"
    "           fl 的 bit6(0x40) 亮       -> IMU 通信坏; bit0(0x01) 不亮 = 零偏还没标定完 */\n"
    "    {\n"
    "        int   yr, yf, gr, gf, xr, xf, dr, df, sr, sf, br, bf;\n"
    "        float bx, by, bz;\n"
    "\n"
    "        ins_diag_get_bias(&bx, &by, &bz);   /* 在线零偏, **raw LSB 域**(1 LSB = 0.0699 dps) */\n"
    "\n"
    "        yr = pf2(ins_imu_get_yaw(),        &yf);\n"
    "        gr = pf2(ins_imu_get_gyro_z_dps(), &gf);\n"
    "        xr = pf2(ins_odom_get_x(),         &xf);\n"
    "        dr = pf2(ins_odom_get_y(),         &df);\n"
    "        sr = pf2(ins_odom_get_dist(),      &sf);\n"
    "        br = pf2(bz,                       &bf);\n"
    "\n"
    "        printf(\"        INS  yaw=%d.%02d gz=%d.%02d x=%d.%02d y=%d.%02d dist=%d.%02d fl=0x%02X st=%u bz=%d.%02d(LSB) en=%u mpm=%u frz=%lu/%lu\\r\\n\",\n"
    "               yr, yf, gr, gf, xr, xf, dr, df, sr, sf,\n"
    "               (unsigned)ins_odom_flags(),\n"
    "               (unsigned)((ins_imu_flags() & INS_ST_STATIC) ? 1u : 0u),\n"
    "               br, bf,\n"
    "               (unsigned)((ins_odom_enable > 0.5f) ? 1u : 0u),\n"
    "               (unsigned)(int)(ins_m_per_rev * 10000.0f),\n"
    "               (unsigned long)ins_imu_zupt_frz_ticks(),\n"
    "               (unsigned long)(ins_imu_zupt_frz_deg() * 100.0f));\n"
    "    }\n"
    "}\n"
)

EDITS = [
    ("include", INC_ANCHOR, INC_NEW),
    ("static", G_ANCHOR, G_NEW),
    ("init", INIT_ANCHOR, INIT_NEW),
    ("chan", CH_ANCHOR, CH_NEW),
    ("loop", LOOP_ANCHOR, LOOP_NEW),
    ("print", PR_ANCHOR, PR_NEW),
]

MARK = "ins_imu_init();"


def line_ending_of(path):
    """★ 行尾必须"照着原文件来" —— 读的时候用默认 newline=None 会把 \\r\\n 归一成 \\n，
    写回 newline='' 就等于**把整个文件的行尾改成 LF**（第一次跑这个脚本就这么干过：
    main_cm7_0.c 明明 423 行全是 CRLF，写完变成 0 个 CRLF）。看开头就够判断。"""
    raw = open(path, "rb").read(4000)
    return "\r\n" if b"\r\n" in raw else "\n"


def main():
    apply = "--apply" in sys.argv
    NL = line_ending_of(P)
    s = io.open(P, encoding="gbk").read()
    print("行尾 = %s" % repr(NL))

    if MARK in s:
        print("已经打过补丁，跳过（幂等）")
        return 0

    for name, anchor, new in EDITS:
        n = s.count(anchor)
        if n != 1:
            print("FAIL: 锚点 [%s] 命中 %d 次（必须正好 1 次）—— 停手，先核对文件" % (name, n))
            return 1
        s = s.replace(anchor, new, 1)
        print("  已接线: %s" % name)

    try:
        s.encode("gbk")
    except UnicodeEncodeError as e:
        print("FAIL: 待写内容含 GBK 编不出的字符 %r" % e.object[e.start:e.end])
        return 1

    if not apply:
        print("预检通过（GBK 可编码），只报告。加 --apply 才真写")
        return 0

    if not os.path.exists(P + ".bak_ins"):
        shutil.copyfile(P, P + ".bak_ins")
        print("已备份 ->", P + ".bak_ins")

    tmp = P + ".tmp_ins"
    with io.open(tmp, "w", encoding="gbk", newline=NL) as f:
        f.write(s)
    os.replace(tmp, P)
    print("已原子写入", P)

    s2 = io.open(P, encoding="gbk").read()
    need = ["ins_imu_init();", "ins_odom_init();", "CH_INS_YAW", "CH_INS_FLAGS",
            "g_zero_last", "INS  yaw=", "ins_imu_dev_id_get"]
    miss = [k for k in need if k not in s2]
    print("回读核对: %s" % ("OK" if not miss else "FAIL 缺 " + " ".join(miss)))
    return 0 if not miss else 1


if __name__ == "__main__":
    sys.exit(main())
