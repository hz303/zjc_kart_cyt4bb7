/*********************************************************************************************************************
 * 卡丁车参数总表（车端唯一源），只在 proto.c 里展开一次
 *
 * 字段: X(id, name, label, lo, hi, def, unit, step, ptr, PERM, GROUP)
 *   id     参数号, 一旦定下不插队（本地存档按 id 存）
 *   name   英文名, 上位机滑条 key
 *   label  组名|中文标签, 上位机按 | 拆分折叠
 *   lo/hi  量程, 车端只按它钳位, 不按 step 取整
 *   def    源码默认值, 引用模块宏
 *   unit/step 单位与滑条步长, 只给上位机显示与交互
 *   ptr    被控变量地址
 *   PERM   读/写/驱动权限, 见下
 *   GROUP  G_* 分组, 只用于一次保存/恢复一个组, 不参与权限
 *
 * 权限四档（配合 proto.c 的 perm_check）
 *   P_RO       只读, 版本与状态
 *   P_RW_LIVE  行驶中随时可写, 增益/前视/死区
 *   P_RW_IDLE  仅无人驱动时可写, 轴距/cpr/极性/软限位/遥测周期
 *   P_CMD      命令, 目标量/使能/录制/回放/测试; 仅驱动令牌持有者可写, 不持久化
 ********************************************************************************************************************/

#ifndef _PARAM_TABLE_H
#define _PARAM_TABLE_H

#include "zf_common_headfile.h"

/* 下面这些头文件提供参数表要取地址的变量 */
#include "spd_enc.h"
#include "spd_ctrl.h"
#include "spd_safety.h"
#include "motor_drv.h"
#include "steer_ctrl.h"
#include "steer_abs.h"
#include "remote_ctrl.h"
#include "ins_odom.h"
#include "ins_route.h"
#include "route_app.h"
#include "route_pb.h"
#include "head_test.h"

/* ---- 一、权限与分组（宏定义在 proto.h, 此处只 include）---- */
#include "proto.h"

/* ---- 二、表 ----
 *  标定顺序: 后轮 minv -> inv -> cpr; 转向先 u0 再加 kp, 最后调 db/slew
 *  只放变量地址, PING/SAVE/GET_TABLE 等命令字走 proto.h 的 PROTO_OP_* */
#define MOTOR_PARAM_TABLE(X)                                                                                          \
    X( 0, "target_l",   "目标|左轮转速",      -8.000f,    8.000f,    0.000f, "rps",   0.050f, &spd_target_l,        P_CMD,     G_SPD  ) \
    X( 1, "target_r",   "目标|右轮转速",      -8.000f,    8.000f,    0.000f, "rps",   0.050f, &spd_target_r,        P_CMD,     G_SPD  ) \
    X( 2, "duty_limit", "限幅|占空比上限",     0.000f,  100.000f, SPD_DUTY_DEFAULT,  "%",   1.000f, &spd_duty_limit_pct, P_RW_IDLE, G_SYS  ) \
    X( 3, "target_max", "限幅|目标转速上限",   0.000f,   20.000f,    3.600f, "rps",   0.100f, &spd_target_max,      P_RW_IDLE, G_SYS  ) \
    X( 4, "k_v",        "增益|比例 k_v",       0.000f,    2.000f, SPD_K_V_DEFAULT,   "-",   0.005f, &spd_g.k_v,          P_RW_LIVE, G_SPD  ) \
    X( 5, "k_z",        "增益|积分 k_z",       0.000f,   20.000f, SPD_K_Z_DEFAULT,   "-",   0.050f, &spd_g.k_z,          P_RW_LIVE, G_SPD  ) \
    X( 6, "inv_K",      "前馈|1/K",            0.000f,    0.500f, SPD_INV_K_DEFAULT, "-",   0.001f, &spd_g.inv_K,        P_RW_LIVE, G_SPD  ) \
    X( 7, "u0",         "前馈|摩擦截距",       0.000f,    0.300f, SPD_U0_DEFAULT,    "-",   0.001f, &spd_g.u0,           P_RW_LIVE, G_SPD  ) \
    X( 8, "cpr_l",      "编码器|每转计数L",    1.000f,65535.000f, 1858.000f, "cnt",   1.000f, &enc_l.cpr,          P_RW_IDLE, G_SPD  ) \
    X( 9, "cpr_r",      "编码器|每转计数R",    1.000f,65535.000f, 1858.000f, "cnt",   1.000f, &enc_r.cpr,          P_RW_IDLE, G_SPD  ) \
    X(10, "inv_l",      "编码器|测速取反L",    0.000f,    1.000f,    0.000f, "-",     1.000f, &enc_l.invert,       P_RW_IDLE, G_SPD  ) \
    X(11, "inv_r",      "编码器|测速取反R",    0.000f,    1.000f,    1.000f, "-",     1.000f, &enc_r.invert,       P_RW_IDLE, G_SPD  ) \
    X(12, "minv_l",     "电机|极性翻转L",      0.000f,    1.000f,    1.000f, "-",     1.000f, &mot_inv_l,          P_RW_IDLE, G_SPD  ) \
    X(13, "minv_r",     "电机|极性翻转R",      0.000f,    1.000f,    0.000f, "-",     1.000f, &mot_inv_r,          P_RW_IDLE, G_SPD  ) \
    X(14, "lost_ms",    "保护|失联停车延时",  100.000f, 3000.000f,  600.000f, "ms",   10.000f, &spd_lost_ms,        P_RW_IDLE, G_SYS  ) \
    X(15, "auto_stop",  "保护|失联自停",       0.000f,    1.000f,    1.000f, "-",     1.000f, &spd_auto_stop,      P_RW_IDLE, G_SYS  ) \
    X(16, "slew",       "限幅|目标斜坡",       0.000f,  200.000f,   20.000f, "rps/s", 1.000f, &spd_slew_rps_s,     P_RW_LIVE, G_SPD  ) \
    X(17, "enc_win",    "编码器|滤波窗口",     1.000f,   16.000f,    4.000f, "-",     1.000f, &spd_enc_win,        P_RW_IDLE, G_SPD  ) \
    X(18, "steer_tgt",  "转向|目标转角",     -27.000f,   27.000f,    0.000f, "deg",   0.500f, &steer.target_delta,  P_CMD,     G_STEER) \
    X(19, "steer_kp",   "转向|比例 Kp",        0.000f,  500.000f,   STEER_KP_DEFAULT,   "-",    1.000f, &steer.kp,        P_RW_LIVE, G_STEER) \
    X(20, "steer_u0",   "转向|摩擦前馈 U0",    0.000f, 3500.000f,   STEER_U0_DEFAULT, "duty", 10.000f, &steer.u0,        P_RW_LIVE, G_STEER) \
    X(21, "steer_db",   "转向|误差死区",       0.010f,    5.000f, STEER_DEADBAND_DEG_DEFAULT, "deg", 0.050f, &steer.deadband_deg, P_RW_LIVE, G_STEER) \
    X(22, "steer_umax", "转向|输出上限",       0.000f,10000.000f, STEER_U_MAX_DEFAULT,"duty",100.000f, &steer.u_max,        P_RW_LIVE, G_STEER) \
    X(23, "steer_en",   "转向|内环使能",       0.000f,    1.000f,   0.000f,  "-",    1.000f, &steer_en_cmd,        P_CMD,     G_STEER) \
    X(24, "minv_s",     "转向|电机极性",       0.000f,    1.000f,   1.000f,  "-",    1.000f, &mot_inv_s,           P_RW_IDLE, G_STEER) \
    X(25, "steer_slew", "转向|目标斜坡",       0.000f,  200.000f,   0.000f, "deg/s", 1.000f, &steer.slew_deg_s,    P_RW_LIVE, G_STEER) \
    X(26, "steer_lim",  "转向|软限位",         0.000f,   27.000f,  27.000f, "deg",   0.500f, &steer.limit_deg,     P_RW_IDLE, G_STEER) \
    X(27, "rc_arm_en",  "遥控|允许接管",       0.000f,    1.000f,   1.000f, "-",     1.000f, &rc_arm_en,           P_RW_LIVE, G_RC   ) \
    X(28, "rc_thr_max", "遥控|满杆转速",       0.000f,   20.000f,    3.600f, "rps",   0.100f, &rc_thr_max,          P_RW_LIVE, G_RC   ) \
    X(29, "ol_en",      "标定|开环模式",       0.000f,    1.000f,   0.000f, "-",     1.000f, &spd_ol_en,           P_CMD,     G_SYS  ) \
    X(30, "ol_duty_l",  "标定|左轮开环duty", -100.000f,  100.000f,   0.000f, "%",     1.000f, &spd_ol_duty_l,       P_CMD,     G_SYS  ) \
    X(31, "ol_duty_r",  "标定|右轮开环duty", -100.000f,  100.000f,   0.000f, "%",     1.000f, &spd_ol_duty_r,       P_CMD,     G_SYS  ) \
    X(32, "tel_sel",    "标定|遥测档案(0巡1标2全3采)",0.000f, 3.000f,   0.000f, "-",     1.000f, &spd_tel_sel,         P_RW_LIVE, G_SYS  ) \
    X(33, "tel_period", "标定|遥测周期",       5.000f,  200.000f,  20.000f, "ms",    1.000f, &spd_tel_period,      P_RW_LIVE, G_SYS  ) \
    X(34, "ins_mpm",    "惯导|米每编码器转",    0.050f,    5.000f,   0.8193f,"m/rev",0.001f, &ins_m_per_rev,       P_RW_IDLE, G_INS  ) \
    X(35, "ins_odom_en","惯导|航位推算使能",    0.000f,    1.000f,   1.000f, "-",     1.000f, &ins_odom_enable,     P_RW_LIVE, G_INS  ) \
    X(36, "ins_slip",   "惯导|打滑门限",        0.000f,  100.000f,  30.000f, "%",     1.000f, &ins_slip_pct,        P_RW_LIVE, G_INS  ) \
    X(37, "ins_zero",   "惯导|位姿归零(0>1)",   0.000f,    1.000f,   0.000f, "-",     1.000f, &ins_zero_cmd,        P_CMD,     G_INS  ) \
    X(38, "rt_rec",     "路表|录制(0>1开/1>0停)",0.000f,   1.000f,   0.000f, "-",     1.000f, &rt_rec_cmd,          P_RW_LIVE, G_RT   ) /* 录制中允许按, 遥控合闸时也可用 */ \
    X(39, "rt_ds",      "路表|采样点距",       0.050f,    0.150f,   0.100f, "m",     0.010f, &ins_route_ds,        P_RW_LIVE, G_RT   ) \
    X(40, "rt_slot",    "路表|当前槽(0~4)",    0.000f,    4.000f,   0.000f, "-",     1.000f, &rt_slot_cmd,         P_CMD,     G_RT   ) \
    X(41, "rt_load",    "路表|重载槽(0>1)",    0.000f,    1.000f,   0.000f, "-",     1.000f, &rt_load_cmd,         P_CMD,     G_RT   ) \
    X(42, "rt_save",    "路表|存到槽(0>1)",    0.000f,    1.000f,   0.000f, "-",     1.000f, &rt_save_cmd,         P_CMD,     G_RT   ) \
    X(43, "pb_en",      "回放|使能",           0.000f,    1.000f,   0.000f, "-",     1.000f, &pb_en,               P_CMD,     G_PB   ) \
    X(44, "pb_mode",    "回放|1只转向/2带速度",1.000f,    2.000f,   1.000f, "-",     1.000f, &pb_mode,             P_CMD,     G_PB   ) \
    X(45, "pb_vmax",    "回放|巡航车速",       0.100f,    4.000f,   1.500f, "m/s",   0.050f, &pb_vmax,             P_RW_LIVE, G_PB   ) \
    /* 横向加速度上限, m/s2; 回放按它算过弯限速 */ \
    X(46, "pb_alat",    "回放|横向加速度",     0.100f,   10.000f,   3.000f, "m/s2",  0.050f, &pb_alat,             P_RW_LIVE, G_PB   ) \
    X(47, "pb_abrk",    "回放|刹车减速度",     0.100f,   10.000f,   0.800f, "m/s2",  0.050f, &pb_abrk,             P_RW_LIVE, G_PB   ) \
    /* ID 48 有效轴距不开放, 用固件常量 PB_L_DEFAULT */ \
    X(49, "pb_ld_a",    "回放|前视基准",       0.100f,    2.000f,   0.350f, "m",     0.010f, &pb_ld_a,             P_RW_LIVE, G_PB   ) \
    X(50, "pb_ld_b",    "回放|前视随速",       0.000f,    1.000f,   0.200f, "m/(m/s)",0.005f,&pb_ld_b,             P_RW_LIVE, G_PB   ) \
    X(51, "pb_ld_min",  "回放|前视下限",       0.100f,    2.000f,   0.300f, "m",     0.010f, &pb_ld_min,           P_RW_LIVE, G_PB   ) \
    X(52, "pb_ld_max",  "回放|前视上限",       0.300f,    5.000f,   1.500f, "m",     0.010f, &pb_ld_max,           P_RW_LIVE, G_PB   ) \
    X(53, "rt_mark",    "路表|标记关键点(0>1)",0.000f,    1.000f,   0.000f, "-",     1.000f, &rt_mark_cmd,         P_RW_LIVE, G_RT   ) \
    X(54, "pb_ff_en",   "回放|打角前馈使能",   0.000f,    1.000f,   0.000f, "-",     1.000f, &pb_ff_en,            P_RW_LIVE, G_PB   ) \
    X(55, "pb_ff",      "回放|前馈增益",       0.000f,    2.000f,   1.000f, "-",     0.050f, &pb_ff_gain,          P_RW_LIVE, G_PB   ) \
    X(56, "df_en",      "差速|电子差速使能",   0.000f,    1.000f,   1.000f, "-",     1.000f, &spd_diff_en,         P_RW_LIVE, G_SPD  ) \
    /* ID 57 轮距不开放, 用固件常量 SPD_DIFF_W_DEFAULT */ \
    X(58, "df_max",     "差速|单侧偏差上限",   0.100f,    0.950f,   SPD_DIFF_MAX_DEFAULT, "-", 0.010f, &spd_diff_max, P_RW_LIVE, G_SPD  ) \
    X(59, "steer_ki",   "转向|积分 Ki",        0.000f,  200.000f,   STEER_KI_DEFAULT,   "-",     1.000f, &steer.ki,      P_RW_LIVE, G_STEER) \
    X(60, "steer_imax", "转向|积分上限",       0.000f, 4000.000f,   STEER_IMAX_DEFAULT, "duty", 50.000f, &steer.i_max,   P_RW_LIVE, G_STEER) \
    X(61, "steer_brk",  "转向|前馈开门门限",    0.000f,    5.000f,   STEER_BRK_DEG_DEFAULT,   "deg",  0.050f, &steer.brk_deg,   P_RW_LIVE, G_STEER) \
    X(62, "steer_izone","转向|积分清零门限",   0.000f,   20.000f,   STEER_IZONE_DEG_DEFAULT, "deg",  0.500f, &steer.izone_deg, P_RW_LIVE, G_STEER) \
    X(63, "steer_i_db", "转向|积分累加下界",   0.000f,    2.000f,   STEER_I_DB_DEG_DEFAULT,  "deg",  0.010f, &steer.i_db_deg,  P_RW_LIVE, G_STEER) \
    X(64, "pb_vmin",    "回放|最低速度",       0.000f,    1.000f,   0.200f, "m/s",   0.010f, &pb_vmin,             P_RW_LIVE, G_PB   ) \
    X(65, "pb_kwin",    "回放|曲率半窗(点)",   1.000f,   20.000f,   6.000f, "pt",    1.000f, &pb_kwin,             P_RW_LIVE, G_PB   ) \
    X(66, "pb_smode",   "回放|转向通道0航向/1delta",0.000f,1.000f,  0.000f, "-",     1.000f, &pb_smode,            P_RW_LIVE, G_PB   ) \
    X(67, "pb_kh",      "回放|航向增益(0=自动)",0.000f, 10.000f,   0.000f, "-",     0.010f, &pb_kh,               P_RW_LIVE, G_PB   ) \
    X(68, "pb_fin",     "回放|终点判定半径",   0.020f,    1.000f,   0.150f, "m",     0.010f, &pb_fin,              P_RW_LIVE, G_PB   ) \
    X(69, "ht_mode",    "测试|0关1定角2台阶",  0.000f,    2.000f,   0.000f, "-",     1.000f, &ht_mode,             P_CMD,     G_TEST) \
    X(70, "ht_rps",     "测试|后轮 rps",       0.100f,    6.000f,   1.200f, "rps",   0.100f, &ht_rps,              P_RW_LIVE, G_TEST) \
    X(71, "ht_yaw",     "测试|目标航向(相对)",-180.000f, 180.000f,   0.000f, "deg",   1.000f, &ht_yaw,              P_RW_LIVE, G_TEST) \
    X(72, "ht_dwell",   "测试|每段停留",       0.500f,   30.000f,   3.000f, "s",     0.500f, &ht_dwell,            P_RW_LIVE, G_TEST) \
    X(73, "ht_step",    "测试|台阶幅值",       1.000f,   30.000f,   5.000f, "deg",   1.000f, &ht_step,             P_RW_LIVE, G_TEST) \
    X(74, "ht_n",       "测试|段数",           1.000f,   40.000f,   8.000f, "段",    1.000f, &ht_n,                P_RW_LIVE, G_TEST) \
    X(75, "ht_k",       "测试|航向增益",       0.100f,   10.000f,   2.000f, "-",     0.100f, &ht_k,                P_RW_LIVE, G_TEST) \
    X(76, "rt_rrec",    "路表|倒车录表(0/1)",  0.000f,    1.000f,   0.000f, "-",     1.000f, &rt_revrec,           P_RW_LIVE, G_RT  ) /* 1=录制时 v 存 |v|, 保留 REV 标记 */ \
    X(77, "rt_rplay",   "路表|倒序回放(0/1)",  0.000f,    1.000f,   0.000f, "-",     1.000f, &rt_revplay,          P_RW_LIVE, G_RT  ) /* 1=回放时点序倒过来, 车正着开沿表反向走 */ \
    X(78, "pb_ctrl",   "回放|控制律(0~2)",     0.000f,    2.000f, PB_CTRL_DEFAULT, "-",     1.000f, &pb_ctrlmode, P_RW_LIVE, G_PB  ) /* 0=无前馈 1=PP 2=LQR */ \
    X(79, "ctrl_qe",    "控制|LQR横向误差权重", 0.010f,  200.000f,   4.000f, "-",     0.100f, &ctrl_qe,    P_RW_LIVE, G_PB  ) /* Ke=sqrt(qe/r) */ \
    X(80, "ctrl_qth",   "控制|LQR航向误差权重", 0.000f,  200.000f,   1.000f, "-",     0.100f, &ctrl_qth,   P_RW_LIVE, G_PB  ) \
    X(81, "ctrl_r",     "控制|LQR打角权重",     0.010f,   20.000f,   1.000f, "-",     0.010f, &ctrl_r,     P_RW_LIVE, G_PB  ) \
    X(82, "ctrl_tlead", "控制|前馈预览时间",    0.000f,    0.600f,   0.120f, "s",     0.010f, &ctrl_tlead, P_RW_LIVE, G_PB  ) \
 /* ID 83/84 空缺, 编号不复用, 对应变量仍在 route_app.c */ \
  X(85, "pb_drev", "回放|行驶方向(0正1倒)", 0.000f, 1.000f, 0.000f, "-", 1.000f, &pb_drev, P_RW_LIVE, G_PB) \
    X(86, "pb_rvk", "回放|倒车开环航向Kp", 0.000f, 5.000f, 1.000f, "-", 0.100f, &pb_rvk, P_RW_LIVE, G_PB) \
    X(87, "pb_cauto", "回放|自动选控制律(0手动1自动)", 0.000f, 1.000f, 1.000f, "-", 1.000f, &pb_cauto, P_RW_LIVE, G_PB) \
    X(88, "rt_mode", "Route|zz/zf/fz/mix_s/mix_e", 0.000f, 4.000f, 0.000f, "-", 1.000f, &rt_route_mode, P_RW_IDLE, G_RT) \
 /* 高速 LQR: ctrl_hs 为结构掩码, 置 0 即关闭; 其余四项为时间域设计指标 */ \
    X(89, "ctrl_hs",     "控制|高速LQR结构掩码(0关)", 0.000f, 31.000f, 15.000f, "bit", 1.000f, &ctrl_hs,      P_RW_LIVE, G_PB) \
    X(90, "ctrl_hs_wn",  "控制|高速LQR闭环频率",     0.200f,  8.000f, 1.600f, "rad/s",0.050f, &ctrl_hs_wn,   P_RW_LIVE, G_PB) \
    X(91, "ctrl_hs_z",   "控制|高速LQR阻尼比",       0.100f,  3.000f, 0.800f, "-",    0.050f, &ctrl_hs_zeta, P_RW_LIVE, G_PB) \
    X(92, "ctrl_hs_lag", "控制|执行器等效延迟",      0.000f,  0.500f, 0.170f, "s",    0.005f, &ctrl_hs_lag,  P_RW_LIVE, G_PB) \
    X(93, "ctrl_hs_tp",  "控制|HS总预览时间",        0.000f,  0.800f, 0.260f, "s",    0.005f, &ctrl_hs_tp,   P_RW_LIVE, G_PB)

/* 表行数, 供 app_params.c 开数组 */
#define MOTOR_PARAM_ROW_COUNT_1(id, nm, lb, lo, hi, df, un, st, pt, pm, gp)   + 1
enum { MOTOR_PARAM_ROW_COUNT = 0 MOTOR_PARAM_TABLE(MOTOR_PARAM_ROW_COUNT_1) };
#undef MOTOR_PARAM_ROW_COUNT_1

#endif
